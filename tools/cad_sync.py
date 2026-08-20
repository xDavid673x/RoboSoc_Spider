#!/usr/bin/env python3
"""Validate, compare, promote, and generate Fusion-derived Webots geometry."""

from __future__ import annotations

import argparse
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path
import shutil
import struct
import sys
from typing import Any, Iterable


ROOT = Path(__file__).resolve().parents[1]
STAGING = ROOT / "webots/cad/.staging"
COMMITTED_SNAPSHOT = ROOT / "webots/cad/fusion_snapshot.v1.json"
COMMITTED_MANIFEST = ROOT / "webots/cad/spider_geometry.v1.json"
COMMITTED_ASSETS = ROOT / "webots/assets/cad"
SPIDER_PROTO = ROOT / "webots/protos/Spider.proto"
WORLD_DIR = ROOT / "webots/worlds"
OVERRIDES_PATH = ROOT / "webots/cad/attachment_overrides.v1.json"

EXPECTED_LINEAGE = "zyUEBEY-SrODxrte3NrQMw"
EXPECTED_ARCHIVE_SHA256 = "63475fde834256bd3cb79ffbde11c24fb34a39b3abd0d019993e2314a8f25491"
EXPECTED_LEG_MAPPING = {
    "legAssemble:2": "legj",
    "legAssemble:3": "legn",
    "legAssemble:4": "legl",
    "legAssemble:5": "legk",
    "legAssemble:6": "legm",
    "legAssemble:7": "legi",
}
LEG_ORDER = ("legi", "legj", "legk", "legl", "legm", "legn")
JOINT_ORDER = ("coxa", "femur", "tibia")
JOINT_ROLE_NAMES = {"coxa": "Revolute 2", "femur": "Revolute 6", "tibia": "Revolute 5"}
WEBOTS_CANONICAL_AXES = {"coxa": (0.0, 1.0, 0.0), "femur": (0.0, 0.0, 1.0), "tibia": (0.0, 0.0, -1.0)}
RESET_ANGLES_DEG = (0.0, 28.0, 115.0)
JOINT_LIMITS_DEG = {"coxa": (-90.0, 90.0), "femur": (-90.0, 90.0), "tibia": (0.0, 130.0)}
BODY_MASS_KG = 1.2
LINK_MASS_KG = 0.1
JOINT_TORQUE_NM = 2.5
TERRAIN_HALF_THICKNESS_MM = 50.0
GROUND_CENTER_Y_MM = -50.0
SUPPORT_MARGIN_MM = 0.5


class ValidationError(RuntimeError):
    """Raised when a staged or committed CAD snapshot violates its contract."""


def _load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="ascii"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise ValidationError(f"cannot read JSON {path}: {error}") from error
    if not isinstance(value, dict):
        raise ValidationError(f"expected a JSON object in {path}")
    return value


def _write_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="ascii")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _mesh_metadata(path: Path) -> dict[str, Any]:
    payload = path.read_bytes()
    if len(payload) < 84:
        raise ValidationError(f"STL is too short: {path}")
    triangle_count = struct.unpack_from("<I", payload, 80)[0]
    expected_size = 84 + triangle_count * 50
    if len(payload) != expected_size:
        raise ValidationError(f"expected binary STL ({expected_size} bytes), got {len(payload)}: {path}")
    points: list[tuple[float, float, float]] = []
    normalized: list[tuple[tuple[float, float, float], ...]] = []
    offset = 84
    for _ in range(triangle_count):
        values = struct.unpack_from("<12fH", payload, offset)
        vertices = tuple(
            tuple(float(values[3 + vertex * 3 + axis]) for axis in range(3))
            for vertex in range(3)
        )
        points.extend(vertices)
        normalized.append(tuple(sorted(tuple(round(value, 3) for value in point) for point in vertices)))
        offset += 50
    normalized.sort()
    geometry_sha = hashlib.sha256(json.dumps(normalized, separators=(",", ":")).encode("ascii")).hexdigest()
    return {
        "sha256": _sha256(path),
        "geometry_sha256_0_001mm": geometry_sha,
        "triangle_count": triangle_count,
        "bounds_mm": {
            "min": [round(min(point[axis] for point in points), 6) for axis in range(3)],
            "max": [round(max(point[axis] for point in points), 6) for axis in range(3)],
        },
    }


@lru_cache(maxsize=32)
def _mesh_points(path_string: str) -> tuple[tuple[float, float, float], ...]:
    """Read STL vertices once for orientation-independent CAD measurements."""

    path = Path(path_string)
    payload = path.read_bytes()
    _require(len(payload) >= 84, f"STL is too short: {path}")
    triangle_count = struct.unpack_from("<I", payload, 80)[0]
    expected_size = 84 + triangle_count * 50
    _require(len(payload) == expected_size, f"expected binary STL ({expected_size} bytes), got {len(payload)}: {path}")
    points: list[tuple[float, float, float]] = []
    for index in range(triangle_count):
        values = struct.unpack_from("<12fH", payload, 84 + index * 50)
        points.extend(
            tuple(float(values[3 + vertex * 3 + axis]) for axis in range(3))
            for vertex in range(3)
        )
    return tuple(points)


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ValidationError(message)


def _asset_path(asset: dict[str, Any], snapshot_path: Path) -> Path:
    filename = Path(str(asset["path"])).name
    if snapshot_path.parent.name == ".staging":
        candidates = [snapshot_path.parent / "assets" / filename, ROOT / str(asset["path"])]
        for candidate in candidates:
            if candidate.exists() and asset.get("sha256") == _sha256(candidate):
                return candidate
        return candidates[0]
    return ROOT / str(asset["path"])


def _matrix_translation(matrix: Iterable[float]) -> list[float]:
    values = list(matrix)
    _require(len(values) == 16, "matrix must have 16 entries")
    return [float(values[3]), float(values[7]), float(values[11])]


def _matmul(left: list[list[float]], right: list[list[float]]) -> list[list[float]]:
    return [
        [sum(left[row][k] * right[k][column] for k in range(4)) for column in range(4)]
        for row in range(4)
    ]


def _matrix_from_flat(values: Iterable[float]) -> list[list[float]]:
    flat = [float(value) for value in values]
    _require(len(flat) == 16 and all(math.isfinite(value) for value in flat), "invalid transform matrix")
    return [flat[index : index + 4] for index in range(0, 16, 4)]


def _flat_from_matrix(rows: list[list[float]], ndigits: int = 9) -> list[float]:
    return [round(value, ndigits) for row in rows for value in row]


def _translation_matrix(vector: Iterable[float]) -> list[list[float]]:
    x, y, z = [float(value) for value in vector]
    return [[1.0, 0.0, 0.0, x], [0.0, 1.0, 0.0, y], [0.0, 0.0, 1.0, z], [0.0, 0.0, 0.0, 1.0]]


def _rotation_about_axis(axis: Iterable[float], angle_rad: float) -> list[list[float]]:
    x, y, z = _unit(axis)
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    t = 1.0 - c
    return [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0.0],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0.0],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _rotate_about(anchor: Iterable[float], axis: Iterable[float], angle_rad: float) -> list[list[float]]:
    anchor_values = list(anchor)
    return _matmul(_matmul(_translation_matrix(anchor_values), _rotation_about_axis(axis, angle_rad)), _translation_matrix([-v for v in anchor_values]))


def _inverse_rigid(rows: list[list[float]]) -> list[list[float]]:
    rot_t = [[rows[column][row] for column in range(3)] for row in range(3)]
    trans = [rows[row][3] for row in range(3)]
    inv_trans = [-sum(rot_t[row][axis] * trans[axis] for axis in range(3)) for row in range(3)]
    return [
        [rot_t[0][0], rot_t[0][1], rot_t[0][2], inv_trans[0]],
        [rot_t[1][0], rot_t[1][1], rot_t[1][2], inv_trans[1]],
        [rot_t[2][0], rot_t[2][1], rot_t[2][2], inv_trans[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _transform_point(rows: list[list[float]], point: Iterable[float]) -> list[float]:
    x, y, z = [float(value) for value in point]
    return [
        rows[0][0] * x + rows[0][1] * y + rows[0][2] * z + rows[0][3],
        rows[1][0] * x + rows[1][1] * y + rows[1][2] * z + rows[1][3],
        rows[2][0] * x + rows[2][1] * y + rows[2][2] * z + rows[2][3],
    ]


def _transform_vector(rows: list[list[float]], vector: Iterable[float]) -> list[float]:
    x, y, z = [float(value) for value in vector]
    return [
        rows[0][0] * x + rows[0][1] * y + rows[0][2] * z,
        rows[1][0] * x + rows[1][1] * y + rows[1][2] * z,
        rows[2][0] * x + rows[2][1] * y + rows[2][2] * z,
    ]


def _dot(left: Iterable[float], right: Iterable[float]) -> float:
    return sum(float(a) * float(b) for a, b in zip(left, right))


def _sub(left: Iterable[float], right: Iterable[float]) -> list[float]:
    return [float(a) - float(b) for a, b in zip(left, right)]


def _cross(left: Iterable[float], right: Iterable[float]) -> list[float]:
    a = list(left)
    b = list(right)
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def _length(vector: Iterable[float]) -> float:
    return math.sqrt(_dot(vector, vector))


def _unit(vector: Iterable[float]) -> list[float]:
    values = [float(value) for value in vector]
    magnitude = _length(values)
    _require(magnitude > 1e-12, f"zero-length vector cannot be normalized: {values}")
    return [value / magnitude for value in values]


def _round_list(values: Iterable[float], ndigits: int = 9) -> list[float]:
    return [round(float(value), ndigits) for value in values]


def _bounds_union(bounds: Iterable[dict[str, list[float]]]) -> dict[str, list[float]]:
    items = list(bounds)
    _require(bool(items), "cannot union empty bounds")
    return {
        "min": [round(min(item["min"][axis] for item in items), 6) for axis in range(3)],
        "max": [round(max(item["max"][axis] for item in items), 6) for axis in range(3)],
    }


def _bounds_center(bounds: dict[str, list[float]]) -> list[float]:
    return [round((bounds["min"][axis] + bounds["max"][axis]) / 2.0, 6) for axis in range(3)]


def _bounds_size(bounds: dict[str, list[float]]) -> list[float]:
    return [round(bounds["max"][axis] - bounds["min"][axis], 6) for axis in range(3)]


def _relative_bounds(bounds: dict[str, list[float]], origin: Iterable[float]) -> dict[str, list[float]]:
    base = list(origin)
    return {
        "min": [round(bounds["min"][axis] - base[axis], 6) for axis in range(3)],
        "max": [round(bounds["max"][axis] - base[axis], 6) for axis in range(3)],
    }


def _validate_matrix(value: Any, context: str) -> None:
    _require(
        isinstance(value, list)
        and len(value) == 16
        and all(isinstance(item, (int, float)) and math.isfinite(float(item)) for item in value),
        f"invalid {context}",
    )
    rows = _matrix_from_flat(value)
    for row in range(3):
        magnitude = math.sqrt(sum(rows[row][column] ** 2 for column in range(3)))
        _require(math.isclose(magnitude, 1.0, abs_tol=1e-4), f"{context} row {row} is not unit length")
    for a in range(3):
        for b in range(a + 1, 3):
            _require(abs(sum(rows[a][column] * rows[b][column] for column in range(3))) <= 1e-4, f"{context} is not orthogonal")


def validate_snapshot(snapshot_path: Path) -> dict[str, Any]:
    snapshot = _load_json(snapshot_path)
    _require(snapshot.get("schema_version") == 1, "snapshot schema_version must be 1")
    source = snapshot.get("source")
    _require(isinstance(source, dict), "snapshot source must be an object")
    _require(source.get("document") == "Spider", "active Fusion document must be Spider")
    _require(source.get("lineage") == EXPECTED_LINEAGE, "unexpected Fusion lineage")
    _require(source.get("archive_sha256_expected") == EXPECTED_ARCHIVE_SHA256, "unexpected archive fingerprint")
    actual_archive = source.get("archive_sha256_actual")
    _require(actual_archive in (None, EXPECTED_ARCHIVE_SHA256), "local Fusion archive does not match its checked fingerprint")

    coordinate_system = snapshot.get("coordinate_system")
    _require(isinstance(coordinate_system, dict), "coordinate_system must be an object")
    _require(coordinate_system.get("fusion_api_length") == "centimeter", "Fusion API lengths must be centimetres")
    _require(coordinate_system.get("manifest_length") == "millimeter", "manifest lengths must be millimetres")
    _require(coordinate_system.get("fusion_to_webots_xyz") == ["x", "y", "z"], "Fusion assembly root must remain Y-up")
    _require(coordinate_system.get("standalone_component_mesh_axis_map") == ["x", "z", "-y"], "standalone component mesh provenance is missing")

    mapping = snapshot.get("mapping")
    _require(isinstance(mapping, dict), "mapping must be an object")
    _require(mapping.get("body_occurrence") == "Hex base-smaller:1", "unexpected body occurrence")
    _require(mapping.get("legs") == EXPECTED_LEG_MAPPING, "unexpected CAD-to-simulator leg mapping")
    _require(mapping.get("tripod_a") == ["legi", "legk", "legm"], "unexpected tripod A")
    _require(mapping.get("tripod_b") == ["legj", "legl", "legn"], "unexpected tripod B")
    _require(snapshot.get("reset_angles_deg") == [0.0, 28.0, 115.0], "unexpected reset pose")

    assembly = snapshot.get("fusion_assembly")
    _require(isinstance(assembly, dict), "fusion_assembly must be an object")
    occurrences = assembly.get("occurrences")
    parts = assembly.get("visual_parts")
    joints = assembly.get("joints")
    as_built_joints = assembly.get("as_built_joints", [])
    groups = assembly.get("rigid_groups")
    _require(isinstance(occurrences, list) and occurrences, "Fusion snapshot has no occurrences")
    _require(isinstance(parts, list) and parts, "Fusion snapshot has no visible BRep bodies")
    _require(isinstance(joints, list), "Fusion joints must be a list")
    _require(isinstance(as_built_joints, list), "Fusion as-built joints must be a list")
    _require(isinstance(groups, list), "Fusion rigid groups must be a list")
    paths = {item.get("full_path") for item in occurrences if isinstance(item, dict)}
    for required in ("Hex base-smaller:1", *EXPECTED_LEG_MAPPING):
        _require(required in paths, f"required Fusion occurrence is missing: {required}")

    assets = snapshot.get("assets")
    _require(isinstance(assets, list) and assets, "snapshot has no mesh assets")
    asset_ids: set[str] = set()
    asset_paths: set[str] = set()
    for asset in assets:
        _require(isinstance(asset, dict), "asset records must be objects")
        asset_id = str(asset.get("id", ""))
        asset_name = str(asset.get("path", ""))
        _require(asset_id and asset_id not in asset_ids, f"duplicate asset id: {asset_id}")
        _require(asset_name.startswith("webots/assets/cad/") and asset_name.endswith(".stl"), f"invalid asset path: {asset_name}")
        _require(asset_name.isascii(), f"asset path must be ASCII: {asset_name}")
        _require(asset_name not in asset_paths, f"duplicate asset path: {asset_name}")
        path = _asset_path(asset, snapshot_path)
        _require(path.is_file(), f"asset is missing: {path}")
        metadata = _mesh_metadata(path)
        for key in ("sha256", "geometry_sha256_0_001mm", "triangle_count", "bounds_mm"):
            _require(asset.get(key) == metadata[key], f"asset metadata drift for {asset_name}: {key}")
        asset_ids.add(asset_id)
        asset_paths.add(asset_name)
    part_ids: set[tuple[str, str, str]] = set()
    for part in parts:
        _require(part.get("asset") in asset_ids, f"visual part references unknown asset: {part}")
        identity = (str(part.get("occurrence")), str(part.get("component")), str(part.get("body")))
        _require(identity not in part_ids, f"duplicate visual part ownership: {identity}")
        part_ids.add(identity)
        for matrix_name in ("fusion_transform_cm", "webots_transform_mm"):
            _validate_matrix(part.get(matrix_name), f"{matrix_name} for {part.get('occurrence')}")
        bounds = part.get("assembly_bounds_mm")
        _require(isinstance(bounds, dict) and "min" in bounds and "max" in bounds, f"missing assembly bounds for {identity}")
    return snapshot


def _load_overrides() -> dict[str, Any]:
    overrides = _load_json(OVERRIDES_PATH)
    _require(overrides.get("schema_version") == 1, "override schema_version must be 1")
    _require(overrides.get("source_lineage") == EXPECTED_LINEAGE, "override lineage mismatch")
    expected_reverse = {value: key for key, value in EXPECTED_LEG_MAPPING.items()}
    _require(overrides.get("leg_occurrences") == expected_reverse, "override leg occurrence mapping mismatch")
    return overrides


def _joint_records_for_leg(snapshot: dict[str, Any], leg_occurrence: str) -> dict[str, dict[str, Any]]:
    records = [
        joint
        for joint in snapshot["fusion_assembly"]["as_built_joints"]
        if joint.get("assembly_context") == leg_occurrence and joint.get("type") == "revolute"
    ]
    _require(len(records) == 3, f"{leg_occurrence} must have exactly three revolute as-built joints")
    by_name = {str(joint["name"]): joint for joint in records}
    for role, fusion_name in JOINT_ROLE_NAMES.items():
        _require(fusion_name in by_name, f"{leg_occurrence} missing {role} joint {fusion_name}")
    return {role: by_name[fusion_name] for role, fusion_name in JOINT_ROLE_NAMES.items()}


def _override_group_map(overrides: dict[str, Any], leg_name: str) -> dict[str, str]:
    """Return the reviewed exact-occurrence ownership map for one leg."""

    assignments = overrides.get("rigid_group_assignments", {}).get(leg_name)
    _require(isinstance(assignments, dict), f"{leg_name} has no reviewed rigid-group assignments")
    result: dict[str, str] = {}
    for group_name in ("mount", "coxa", "femur", "tibia"):
        values = assignments.get(group_name)
        _require(isinstance(values, list) and values, f"{leg_name} {group_name} override is empty")
        for occurrence in values:
            occurrence_name = str(occurrence)
            _require(occurrence_name not in result, f"{leg_name} occurrence assigned to multiple groups: {occurrence_name}")
            result[occurrence_name] = group_name
    return result


def _leg_parts(
    snapshot: dict[str, Any],
    leg_occurrence: str,
    leg_name: str,
    overrides: dict[str, Any],
) -> dict[str, list[dict[str, Any]]]:
    groups = {name: [] for name in ("mount", "coxa", "femur", "tibia")}
    ownership = _override_group_map(overrides, leg_name)
    seen: set[tuple[str, str, str]] = set()
    for part in snapshot["fusion_assembly"]["visual_parts"]:
        occurrence = str(part["occurrence"])
        if occurrence != leg_occurrence and not occurrence.startswith(leg_occurrence + "+"):
            continue
        key = (occurrence, str(part["component"]), str(part["body"]))
        _require(key not in seen, f"duplicate leg visual body: {key}")
        seen.add(key)
        group = ownership.get(occurrence)
        _require(group is not None, f"unresolved reviewed rigid group for {key}")
        groups[group].append(part)
    _require(set(ownership) == {part["occurrence"] for part in snapshot["fusion_assembly"]["visual_parts"] if str(part["occurrence"]).startswith(leg_occurrence + "+")}, f"{leg_name} ownership override does not cover exactly the visible leg occurrences")
    for group, values in groups.items():
        _require(values, f"{leg_occurrence} group {group} has no visual bodies")
    return groups


def _local_frame(coxa_anchor: list[float], femur_axis: list[float], body_center: list[float]) -> dict[str, list[float]]:
    y_axis = [0.0, 1.0, 0.0]
    x_axis = _unit(_cross(y_axis, femur_axis))
    if _dot(x_axis, _sub(coxa_anchor, body_center)) < 0.0:
        x_axis = [-value for value in x_axis]
    z_axis = _unit(_cross(x_axis, y_axis))
    return {"x_axis": _round_list(x_axis), "y_axis": _round_list(y_axis), "z_axis": _round_list(z_axis)}


def _basis_inverse(frame: dict[str, list[float]], origin: list[float]) -> list[list[float]]:
    x_axis = frame["x_axis"]
    y_axis = frame["y_axis"]
    z_axis = frame["z_axis"]
    return [
        [x_axis[0], x_axis[1], x_axis[2], -_dot(x_axis, origin)],
        [y_axis[0], y_axis[1], y_axis[2], -_dot(y_axis, origin)],
        [z_axis[0], z_axis[1], z_axis[2], -_dot(z_axis, origin)],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _basis_matrix(frame: dict[str, list[float]], origin: list[float]) -> list[list[float]]:
    x_axis = frame["x_axis"]
    y_axis = frame["y_axis"]
    z_axis = frame["z_axis"]
    return [
        [x_axis[0], y_axis[0], z_axis[0], origin[0]],
        [x_axis[1], y_axis[1], z_axis[1], origin[1]],
        [x_axis[2], y_axis[2], z_axis[2], origin[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _axis_alignment(local_axis: list[float], canonical: tuple[float, float, float], context: str) -> int:
    alignment = _dot(local_axis, canonical)
    if math.isclose(alignment, 1.0, abs_tol=1e-6):
        return 1
    if math.isclose(alignment, -1.0, abs_tol=1e-6):
        return -1
    raise ValidationError(f"{context} axis is not collinear with canonical axis: {local_axis} vs {canonical}")


def _line_distance(point_a: list[float], axis_a: list[float], point_b: list[float], axis_b: list[float]) -> float:
    cross_axes = _cross(axis_a, axis_b)
    denominator = _length(cross_axes)
    delta = _sub(point_b, point_a)
    if denominator <= 1e-9:
        return _length(_cross(delta, axis_a))
    return abs(_dot(delta, cross_axes)) / denominator


def _part_manifest(
    part: dict[str, Any],
    body_center: list[float],
    group_frame_body: list[list[float]],
) -> dict[str, Any]:
    root_matrix = _matrix_from_flat(part["webots_transform_mm"])
    body_centered = _matmul(_translation_matrix([-value for value in body_center]), root_matrix)
    group_local = _matmul(_inverse_rigid(group_frame_body), body_centered)
    return {
        "occurrence": part["occurrence"],
        "component": part["component"],
        "body": part["body"],
        "asset": part["asset"],
        "asset_path": next_asset_path_placeholder(part["asset"]),
        "assembly_transform_body_mm": _flat_from_matrix(body_centered),
        "body_centered_transform_mm": _flat_from_matrix(body_centered),
        "group_local_transform_mm": _flat_from_matrix(group_local),
        "assembly_bounds_mm": _relative_bounds(part["assembly_bounds_mm"], body_center),
    }


def next_asset_path_placeholder(asset_id: str) -> str:
    return asset_id


def derive_manifest(
    snapshot: dict[str, Any],
    overrides: dict[str, Any] | None = None,
    snapshot_path: Path | None = None,
) -> dict[str, Any]:
    overrides = overrides or _load_overrides()
    parts = snapshot["fusion_assembly"]["visual_parts"]
    body_parts = [part for part in parts if part.get("occurrence") == snapshot["mapping"]["body_occurrence"]]
    _require(body_parts, "body occurrence has no visual parts")
    body_bounds_root = _bounds_union(part["assembly_bounds_mm"] for part in body_parts)
    body_center = _bounds_center(body_bounds_root)
    asset_by_id = {asset["id"]: asset for asset in snapshot["assets"]}
    asset_files = {
        asset_id: _asset_path(asset, snapshot_path or COMMITTED_SNAPSHOT)
        for asset_id, asset in asset_by_id.items()
    }

    def transformed_part_points(part: dict[str, Any]) -> list[list[float]]:
        matrix = _matrix_from_flat(part["webots_transform_mm"])
        return [_transform_point(matrix, point) for point in _mesh_points(str(asset_files[part["asset"]]))]

    identity_frame = _translation_matrix([0.0, 0.0, 0.0])

    def part_record(part: dict[str, Any], group_frame_body: list[list[float]]) -> dict[str, Any]:
        record = _part_manifest(part, body_center, group_frame_body)
        record["asset_path"] = asset_by_id[part["asset"]]["path"]
        return record

    body_visuals = [part_record(part, identity_frame) for part in body_parts]
    legs: list[dict[str, Any]] = []
    all_owned_parts = {(part["occurrence"], part["component"], part["body"]) for part in body_parts}
    min_support_y = min(part["assembly_bounds_mm"]["min"][1] - body_center[1] for part in parts)

    for leg_name in LEG_ORDER:
        leg_occurrence = {value: key for key, value in EXPECTED_LEG_MAPPING.items()}[leg_name]
        joints = _joint_records_for_leg(snapshot, leg_occurrence)
        anchors_root = {role: _matrix_translation(joints[role]["webots_transform_mm"]) for role in JOINT_ORDER}
        axes_root = {role: _unit(joints[role]["axis_body_webots"]) for role in JOINT_ORDER}
        frame = _local_frame(anchors_root["coxa"], axes_root["femur"], body_center)
        leg_origin_root = anchors_root["coxa"]
        leg_origin_body = _round_list(_sub(leg_origin_root, body_center), 6)
        # All four rigid groups use the same CAD-derived leg orientation at
        # the documented reset pose.  Their origins are the adjacent joint
        # anchors, so every visual transform is an exact rigid-frame local
        # transform and every hinge can apply its reset zero offset.
        leg_to_body = _basis_matrix(frame, leg_origin_body)
        body_to_leg = _inverse_rigid(leg_to_body)
        anchor_bodies = {
            role: _round_list(_sub(anchors_root[role], body_center), 6)
            for role in JOINT_ORDER
        }
        anchors_local = {
            role: _round_list(_transform_point(body_to_leg, anchor_bodies[role]), 6)
            for role in JOINT_ORDER
        }
        _require(max(abs(value) for value in anchors_local["coxa"]) <= 0.01, f"{leg_name} coxa anchor is not at its frame origin")

        joint_manifest = []
        command_signs: dict[str, int] = {}
        for role in JOINT_ORDER:
            anchor_local = anchors_local[role]
            axis_local = _unit(_transform_vector(body_to_leg, axes_root[role]))
            sign = _axis_alignment(axis_local, WEBOTS_CANONICAL_AXES[role], f"{leg_name} {role}")
            command_signs[role] = sign
            reset_angle = RESET_ANGLES_DEG[JOINT_ORDER.index(role)]
            parent_group = {"coxa": "mount", "femur": "coxa", "tibia": "femur"}[role]
            child_group = {"coxa": "coxa", "femur": "femur", "tibia": "tibia"}[role]
            parent_origin = anchors_local["coxa" if role == "coxa" else ("femur" if role == "tibia" else "coxa")]
            parent_anchor = [
                0.0,
                0.0,
                0.0,
            ] if role == "coxa" else _sub(anchor_local, parent_origin)
            cad_transform_parent = _translation_matrix(anchor_local if role == "coxa" else parent_anchor)
            zero_transform = _matmul(
                _rotate_about(parent_anchor, WEBOTS_CANONICAL_AXES[role], math.radians(-reset_angle)),
                cad_transform_parent,
            )
            joint_manifest.append(
                {
                    "role": role,
                    "fusion_name": joints[role]["name"],
                    "entity_token": joints[role]["entity_token"],
                    "parent_group": parent_group,
                    "child_group": child_group,
                    "anchor_root_mm": _round_list(anchors_root[role], 6),
                    "anchor_body_mm": anchor_bodies[role],
                    "anchor_leg_mm": _round_list(anchor_local, 6),
                    "anchor_parent_mm": _round_list(parent_anchor, 6),
                    "fusion_axis_root": _round_list(axes_root[role]),
                    "axis_leg": _round_list(axis_local),
                    "webots_axis": list(WEBOTS_CANONICAL_AXES[role]),
                    "command_sign": sign,
                    "reset_angle_deg": RESET_ANGLES_DEG[JOINT_ORDER.index(role)],
                    "webots_reset_angle_deg": reset_angle,
                    "fusion_reset_angle_deg": round(reset_angle * sign, 9),
                    "limits_deg": list(JOINT_LIMITS_DEG[role]),
                    "cad_transform_parent_mm": _flat_from_matrix(cad_transform_parent),
                    "zero_transform_parent_mm": _flat_from_matrix(zero_transform),
                    "zero_transform_body_mm": _flat_from_matrix(
                        _matmul(leg_to_body, zero_transform)
                    ),
                    "provenance": "Fusion as-built joint",
                }
            )

        leg_groups = _leg_parts(snapshot, leg_occurrence, leg_name, overrides)
        group_origins = {
            "mount": anchor_bodies["coxa"],
            "coxa": anchor_bodies["coxa"],
            "femur": anchor_bodies["femur"],
            "tibia": anchor_bodies["tibia"],
        }
        group_frames = {
            group_name: _basis_matrix(frame, group_origins[group_name])
            for group_name in group_origins
        }
        group_records = {}
        for group_name, group_parts in leg_groups.items():
            group_records[group_name] = {
                "origin_body_mm": _round_list(group_origins[group_name], 6),
                "frame_body_mm": _flat_from_matrix(group_frames[group_name]),
                "visuals": [part_record(part, group_frames[group_name]) for part in group_parts],
                "bounds_body_mm": _relative_bounds(_bounds_union(part["assembly_bounds_mm"] for part in group_parts), body_center),
            }
            for part in group_parts:
                key = (part["occurrence"], part["component"], part["body"])
                _require(key not in all_owned_parts, f"duplicate body ownership: {key}")
                all_owned_parts.add(key)

        coxa_delta = _sub(anchor_bodies["femur"], anchor_bodies["coxa"])
        femur_delta = _sub(anchor_bodies["tibia"], anchor_bodies["femur"])
        tibia_bounds = group_records["tibia"]["bounds_body_mm"]
        tibia_corners = [
            [x, y, z]
            for x in (tibia_bounds["min"][0], tibia_bounds["max"][0])
            for y in (tibia_bounds["min"][1], tibia_bounds["max"][1])
            for z in (tibia_bounds["min"][2], tibia_bounds["max"][2])
        ]
        foot_override = overrides.get("foot_contacts", {}).get(leg_name)
        foot_part = None
        if isinstance(foot_override, dict):
            selector_occurrence = foot_override.get("visual_occurrence")
            foot_part = next(
                (part for part in leg_groups["tibia"] if part.get("occurrence") == selector_occurrence),
                None,
            )
        if foot_part is None:
            foot_part = next(
                (part for part in leg_groups["tibia"] if str(part.get("component")) == "Final_leg_tip"),
                leg_groups["tibia"][0],
            )
        foot_points_root = transformed_part_points(foot_part)
        foot_points_body = [_sub(point, body_center) for point in foot_points_root]
        tibia_length = max(
            _dot(_sub(point, anchor_bodies["tibia"]), frame["x_axis"])
            for point in foot_points_body
        )
        lengths = {
            "coxa": round(abs(_dot(coxa_delta, frame["x_axis"])), 6),
            "femur": round(abs(_dot(femur_delta, frame["x_axis"])), 6),
            "tibia": round(max(1.0, tibia_length), 6),
        }
        # The coxa anchor and femur anchor are not expected to share the same
        # leg-frame Z coordinate in this CAD: the coxa joint is a vertical line.
        # Planarity for IK is the residual after fitting hinge axis directions
        # to the canonical 3R model, i.e. unwanted local-axis components.
        planar_axis_residual = max(
            abs(_unit(_transform_vector(body_to_leg, axes_root["coxa"]))[0]),
            abs(_unit(_transform_vector(body_to_leg, axes_root["coxa"]))[2]),
            abs(_unit(_transform_vector(body_to_leg, axes_root["femur"]))[0]),
            abs(_unit(_transform_vector(body_to_leg, axes_root["femur"]))[1]),
            abs(_unit(_transform_vector(body_to_leg, axes_root["tibia"]))[0]),
            abs(_unit(_transform_vector(body_to_leg, axes_root["tibia"]))[1]),
        )
        planar_residual_deg = round(math.degrees(math.asin(min(1.0, planar_axis_residual))), 6)
        planar_residual_mm = round(abs(_dot(femur_delta, frame["z_axis"])), 6)
        _require(planar_residual_mm <= 0.25, f"{leg_name} planar-fit residual exceeds 0.25 mm: {planar_residual_mm}")
        _require(planar_residual_deg <= 0.25, f"{leg_name} planar-fit angular residual exceeds 0.25 deg: {planar_residual_deg}")
        if isinstance(foot_override, list) and len(foot_override) == 3:
            foot_contact = [float(value) for value in foot_override]
        elif foot_part is not None:
            lowest_y = min(point[1] for point in foot_points_body)
            lowest = [point for point in foot_points_body if math.isclose(point[1], lowest_y, abs_tol=0.001)]
            foot_contact = max(lowest, key=lambda point: _dot(_sub(point, leg_origin_body), frame["x_axis"]))
        else:
            lowest_y = min(corner[1] for corner in tibia_corners)
            lowest = [corner for corner in tibia_corners if math.isclose(corner[1], lowest_y, abs_tol=1e-9)]
            foot_contact = max(lowest, key=lambda corner: _dot(_sub(corner, leg_origin_body), frame["x_axis"]))
        legs.append(
            {
                "name": leg_name,
                "occurrence": leg_occurrence,
                "origin_body_mm": leg_origin_body,
                "frame": frame,
                "leg_to_body_transform_mm": _flat_from_matrix(leg_to_body),
                "command_signs": command_signs,
                "gait_heading_rad": round(math.atan2(frame["x_axis"][2], frame["x_axis"][0]), 9),
                "gait_compensation_rad": round(-math.atan2(frame["x_axis"][2], frame["x_axis"][0]), 9),
                "lengths_mm": lengths,
                "planar_fit_residual_mm": planar_residual_mm,
                "planar_fit_residual_deg": planar_residual_deg,
                "foot_contact_body_mm": _round_list(foot_contact, 6),
                "groups": group_records,
                "joints": joint_manifest,
            }
        )

    expected_visual_count = len(parts)
    _require(len(all_owned_parts) == expected_visual_count, f"visual ownership incomplete: {len(all_owned_parts)} of {expected_visual_count}")
    normal_offset_mm = TERRAIN_HALF_THICKNESS_MM - min_support_y + SUPPORT_MARGIN_MM
    worlds = {}
    for angle in (0, 10, 20, 30):
        radians = math.radians(angle)
        # SlopeTerrain rotates about Webots +Z.  Rotate the body-support
        # offset about the same terrain centre, rather than projecting it onto
        # an unrelated world axis.
        normal_offset_x = -normal_offset_mm * math.sin(radians)
        normal_offset_y = normal_offset_mm * math.cos(radians)
        worlds[f"slope_{angle}" if angle else "flat"] = {
            "terrain_angle_deg": angle,
            "terrain_center_mm": [0.0, GROUND_CENTER_Y_MM, 0.0],
            "initial_translation_m": [
                round(normal_offset_x / 1000.0, 9),
                round((GROUND_CENTER_Y_MM + normal_offset_y) / 1000.0, 9),
                0.0,
            ],
            "initial_rotation": [0.0, 1.0, 0.0, 0.0] if angle == 0 else [0.0, 0.0, 1.0, round(radians, 9)],
        }

    return {
        "schema_version": 1,
        "source": snapshot["source"],
        "coordinate_system": snapshot["coordinate_system"],
        "tolerances": snapshot["tolerances"],
        "mapping": snapshot["mapping"],
        "reset_angles_deg": list(RESET_ANGLES_DEG),
        "joint_limits_deg": {key: list(value) for key, value in JOINT_LIMITS_DEG.items()},
        "dynamics_estimates": {
            "total_robot_mass_kg": 3.0,
            "body_mass_kg": BODY_MASS_KG,
            "each_leg_link_mass_kg": LINK_MASS_KG,
            "foot_friction_static": 0.8,
            "foot_friction_dynamic": 0.6,
            "motor_torque_limit_nm": JOINT_TORQUE_NM,
            "note": "Provisional values; not calibrated measurements.",
        },
        "assets": snapshot["assets"],
        "fusion_assembly": snapshot["fusion_assembly"],
        "body": {
            "occurrence": snapshot["mapping"]["body_occurrence"],
            "center_root_mm": body_center,
            "bounds_body_mm": _relative_bounds(body_bounds_root, body_center),
            "visuals": body_visuals,
            "collision_primitive": {"type": "box", "size_mm": _bounds_size(_relative_bounds(body_bounds_root, body_center))},
        },
        "legs": legs,
        "world": {
            "support_minimum_reset_height_mm": round(min_support_y, 6),
            "terrain_half_thickness_mm": TERRAIN_HALF_THICKNESS_MM,
            "normal_offset_mm": round(normal_offset_mm, 6),
            "poses": worlds,
        },
        "attachment_overrides": overrides,
    }


def validate_manifest(manifest: dict[str, Any]) -> None:
    _require(manifest.get("schema_version") == 1, "manifest schema_version must be 1")
    legs = manifest.get("legs")
    _require(isinstance(legs, list) and [leg.get("name") for leg in legs] == list(LEG_ORDER), "manifest legs are missing or out of order")
    owned: set[tuple[str, str, str]] = set()
    for leg in legs:
        _require(set(leg.get("groups", {})) == {"mount", "coxa", "femur", "tibia"}, f"{leg.get('name')} group set is incomplete")
        _require(len(leg.get("joints", [])) == 3, f"{leg.get('name')} must have three joints")
        for joint in leg["joints"]:
            role = joint["role"]
            _require(role in JOINT_ORDER, f"{leg['name']} has an unknown joint role: {role}")
            _require(joint["limits_deg"] == list(JOINT_LIMITS_DEG[role]), f"{leg['name']} {role} limit drift")
            _require(joint["webots_axis"] == list(WEBOTS_CANONICAL_AXES[role]), f"{leg['name']} {role} axis drift")
            _require(joint["command_sign"] in (-1, 1), f"{leg['name']} {role} invalid command sign")
        for group in leg["groups"].values():
            for visual in group["visuals"]:
                key = (visual["occurrence"], visual["component"], visual["body"])
                _require(key not in owned, f"duplicate manifest visual ownership: {key}")
                owned.add(key)
    for visual in manifest["body"]["visuals"]:
        key = (visual["occurrence"], visual["component"], visual["body"])
        _require(key not in owned, f"duplicate body visual ownership: {key}")
        owned.add(key)
    _require(len(owned) == len(manifest["fusion_assembly"]["visual_parts"]), "manifest does not own every visual part")
    _validate_attachment_frames(manifest)


def _rotation_only(rows: list[list[float]]) -> list[list[float]]:
    result = [row[:] for row in rows]
    for row in range(3):
        result[row][3] = 0.0
    result[3] = [0.0, 0.0, 0.0, 1.0]
    return result


def _rotation_error_deg(expected: list[list[float]], actual: list[list[float]]) -> float:
    expected_rotation = [[expected[row][column] for column in range(3)] for row in range(3)]
    actual_rotation = [[actual[row][column] for column in range(3)] for row in range(3)]
    relative_trace = sum(
        sum(expected_rotation[index][row] * actual_rotation[index][row] for index in range(3))
        for row in range(3)
    )
    cosine = max(-1.0, min(1.0, (relative_trace - 1.0) / 2.0))
    return math.degrees(math.acos(cosine))


def _attachment_error(
    expected: list[list[float]],
    actual: list[list[float]],
) -> tuple[float, float]:
    expected_translation = [expected[row][3] for row in range(3)]
    actual_translation = [actual[row][3] for row in range(3)]
    translation_error = max(
        abs(expected_translation[index] - actual_translation[index])
        for index in range(3)
    )
    return translation_error, _rotation_error_deg(expected, actual)


def _require_attachment(
    expected: list[list[float]],
    actual: list[list[float]],
    context: str,
) -> None:
    translation_error, rotation_error = _attachment_error(expected, actual)
    _require(
        translation_error <= 0.01 and rotation_error <= 0.01,
        f"{context} differs from Fusion by {translation_error:.6f} mm / {rotation_error:.6f} deg",
    )


def _reset_endpoint_transform(joint: dict[str, Any]) -> list[list[float]]:
    anchor = [float(value) for value in joint["anchor_parent_mm"]]
    zero = _matrix_from_flat(joint["zero_transform_parent_mm"])
    return _matmul(
        _translation_matrix(anchor),
        _matmul(
            _rotation_about_axis(joint["webots_axis"], math.radians(joint["reset_angle_deg"])),
            _rotation_only(zero),
        ),
    )


def _validate_attachment_frames(manifest: dict[str, Any]) -> None:
    """Rebuild the generated articulated tree and compare it with Fusion.

    The raw ``fusion_assembly`` records are the source of truth here.  This is
    intentionally separate from the generated manifest fields: changing both a
    visual's local transform and its cached body transform must still fail.
    """

    body = manifest["body"]
    raw_parts = manifest["fusion_assembly"]["visual_parts"]
    raw_part_by_key = {
        (str(part["occurrence"]), str(part["component"]), str(part["body"])): part
        for part in raw_parts
    }
    _require(len(raw_part_by_key) == len(raw_parts), "Fusion visual parts contain duplicate ownership keys")

    # Recompute the body-centred frame from Fusion bounds and verify every body
    # visual against its raw assembly transform.
    raw_body_parts = [part for part in raw_parts if part.get("occurrence") == body["occurrence"]]
    _require(raw_body_parts, "Fusion body occurrence has no visual parts")
    body_center = _bounds_center(_bounds_union(part["assembly_bounds_mm"] for part in raw_body_parts))
    _require(
        max(abs(float(body["center_root_mm"][axis]) - body_center[axis]) for axis in range(3)) <= 0.01,
        "manifest body center differs from Fusion body bounds",
    )
    body_center_matrix = _translation_matrix([-value for value in body_center])

    expected_visuals: dict[tuple[str, str, str], dict[str, Any]] = {}
    for visual in body["visuals"]:
        key = (str(visual["occurrence"]), str(visual["component"]), str(visual["body"]))
        _require(key in raw_part_by_key, f"manifest body visual is absent from Fusion: {key}")
        raw = raw_part_by_key[key]
        fusion_body = _matmul(body_center_matrix, _matrix_from_flat(raw["webots_transform_mm"]))
        _require_attachment(
            fusion_body,
            _matrix_from_flat(visual["body_centered_transform_mm"]),
            f"body visual {visual['occurrence']} transform",
        )
        _require_attachment(
            fusion_body,
            _matrix_from_flat(visual["assembly_transform_body_mm"]),
            f"body visual {visual['occurrence']} assembly transform",
        )
        expected_visuals[key] = visual

    group_parent = {"mount": None, "coxa": "mount", "femur": "coxa", "tibia": "femur"}
    for leg in manifest["legs"]:
        leg_name = str(leg["name"])
        joints = {str(joint["role"]): joint for joint in leg["joints"]}
        _require(set(joints) == set(JOINT_ORDER), f"{leg_name} joint roles are incomplete")
        fusion_joints = _joint_records_for_leg(manifest, str(leg["occurrence"]))
        anchors_root = {
            role: _matrix_translation(fusion_joints[role]["webots_transform_mm"])
            for role in JOINT_ORDER
        }
        axes_root = {
            role: _unit(fusion_joints[role]["axis_body_webots"])
            for role in JOINT_ORDER
        }
        frame = _local_frame(anchors_root["coxa"], axes_root["femur"], body_center)
        leg_origin_body = _sub(anchors_root["coxa"], body_center)
        expected_leg_to_body = _basis_matrix(frame, leg_origin_body)
        leg_to_body = _matrix_from_flat(leg["leg_to_body_transform_mm"])
        _require_attachment(expected_leg_to_body, leg_to_body, f"{leg_name} leg frame")
        _require(
            max(abs(float(leg["origin_body_mm"][axis]) - leg_origin_body[axis]) for axis in range(3)) <= 0.01,
            f"{leg_name} body mount origin differs from Fusion",
        )
        body_to_leg = _inverse_rigid(expected_leg_to_body)
        anchors_body = {role: _sub(anchors_root[role], body_center) for role in JOINT_ORDER}
        anchors_local = {
            role: _transform_point(body_to_leg, anchors_body[role])
            for role in JOINT_ORDER
        }
        _require(max(abs(value) for value in anchors_local["coxa"]) <= 0.01, f"{leg_name} coxa anchor is not at its frame origin")

        for role in JOINT_ORDER:
            joint = joints[role]
            canonical_axis = WEBOTS_CANONICAL_AXES[role]
            axis_local = _unit(_transform_vector(body_to_leg, axes_root[role]))
            sign = _axis_alignment(axis_local, canonical_axis, f"{leg_name} {role}")
            expected_parent_anchor = {
                "coxa": [0.0, 0.0, 0.0],
                "femur": anchors_local["femur"],
                "tibia": _sub(anchors_local["tibia"], anchors_local["femur"]),
            }[role]
            if role == "coxa":
                expected_parent_anchor = [0.0, 0.0, 0.0]
            expected_cad_transform = _translation_matrix(expected_parent_anchor)
            reset_angle = RESET_ANGLES_DEG[JOINT_ORDER.index(role)]
            expected_zero = _matmul(
                _rotate_about(expected_parent_anchor, canonical_axis, math.radians(-reset_angle)),
                expected_cad_transform,
            )
            _require(
                max(abs(float(joint["anchor_root_mm"][axis]) - anchors_root[role][axis]) for axis in range(3)) <= 0.01,
                f"{leg_name} {role} root anchor differs from Fusion",
            )
            _require(
                max(abs(float(joint["anchor_body_mm"][axis]) - anchors_body[role][axis]) for axis in range(3)) <= 0.01,
                f"{leg_name} {role} body anchor differs from Fusion",
            )
            _require(
                max(abs(float(joint["anchor_leg_mm"][axis]) - anchors_local[role][axis]) for axis in range(3)) <= 0.01,
                f"{leg_name} {role} leg anchor differs from Fusion",
            )
            _require(
                max(abs(float(joint["anchor_parent_mm"][axis]) - expected_parent_anchor[axis]) for axis in range(3)) <= 0.01,
                f"{leg_name} {role} parent-local anchor is inconsistent",
            )
            _require(
                max(abs(float(joint["axis_leg"][axis]) - axis_local[axis]) for axis in range(3)) <= 1e-5,
                f"{leg_name} {role} local axis differs from Fusion",
            )
            _require(joint["command_sign"] == sign, f"{leg_name} {role} command sign differs from Fusion")
            _require(joint["reset_angle_deg"] == reset_angle, f"{leg_name} {role} reset angle drift")
            _require(joint["webots_reset_angle_deg"] == reset_angle, f"{leg_name} {role} Webots reset angle drift")
            _require(joint["fusion_reset_angle_deg"] == round(reset_angle * sign, 9), f"{leg_name} {role} Fusion reset angle drift")
            _require_attachment(
                expected_cad_transform,
                _matrix_from_flat(joint["cad_transform_parent_mm"]),
                f"{leg_name} {role} CAD reset anchor",
            )
            _require_attachment(
                expected_zero,
                _matrix_from_flat(joint["zero_transform_parent_mm"]),
                f"{leg_name} {role} zero transform",
            )
            _require_attachment(
                _matmul(expected_leg_to_body, expected_zero),
                _matrix_from_flat(joint["zero_transform_body_mm"]),
                f"{leg_name} {role} body zero transform",
            )
            _require_attachment(
                expected_cad_transform,
                _reset_endpoint_transform(joint),
                f"{leg_name} {role} reset endpoint",
            )
            expected_parent_group = {
                "coxa": "mount",
                "femur": "coxa",
                "tibia": "femur",
            }[role]
            _require(joint["parent_group"] == expected_parent_group, f"{leg_name} {role} parent group drift")
            _require(joint["child_group"] == role, f"{leg_name} {role} child group drift")

        endpoint_by_group = {
            "mount": expected_leg_to_body,
            "coxa": _matmul(expected_leg_to_body, _reset_endpoint_transform(joints["coxa"])),
        }
        endpoint_by_group["femur"] = _matmul(endpoint_by_group["coxa"], _reset_endpoint_transform(joints["femur"]))
        endpoint_by_group["tibia"] = _matmul(endpoint_by_group["femur"], _reset_endpoint_transform(joints["tibia"]))
        expected_group_origins = {
            "mount": anchors_body["coxa"],
            "coxa": anchors_body["coxa"],
            "femur": anchors_body["femur"],
            "tibia": anchors_body["tibia"],
        }
        leg_part_keys: set[tuple[str, str, str]] = set()
        for group_name, group in leg["groups"].items():
            _require(group_name in group_parent, f"{leg_name} has unknown group {group_name}")
            expected_group_frame = _basis_matrix(frame, expected_group_origins[group_name])
            _require_attachment(
                expected_group_frame,
                _matrix_from_flat(group["frame_body_mm"]),
                f"{leg_name} {group_name} frame",
            )
            _require(
                max(abs(float(group["origin_body_mm"][axis]) - expected_group_origins[group_name][axis]) for axis in range(3)) <= 0.01,
                f"{leg_name} {group_name} origin differs from Fusion",
            )
            _require_attachment(
                expected_group_frame,
                endpoint_by_group[group_name],
                f"{leg_name} {group_name} reset endpoint frame",
            )
            if group_name != "mount":
                _require(group_parent[group_name] in leg["groups"], f"{leg_name} {group_name} parent group is missing")
            for visual in group["visuals"]:
                key = (str(visual["occurrence"]), str(visual["component"]), str(visual["body"]))
                _require(key in raw_part_by_key, f"manifest leg visual is absent from Fusion: {key}")
                _require(key not in expected_visuals, f"duplicate manifest visual ownership: {key}")
                raw = raw_part_by_key[key]
                fusion_body = _matmul(body_center_matrix, _matrix_from_flat(raw["webots_transform_mm"]))
                _require_attachment(
                    fusion_body,
                    _matrix_from_flat(visual["body_centered_transform_mm"]),
                    f"{leg_name} {group_name} visual {visual['occurrence']} transform",
                )
                _require_attachment(
                    fusion_body,
                    _matrix_from_flat(visual["assembly_transform_body_mm"]),
                    f"{leg_name} {group_name} assembly transform {visual['occurrence']}",
                )
                expected_local = _matmul(_inverse_rigid(expected_group_frame), fusion_body)
                _require_attachment(
                    expected_local,
                    _matrix_from_flat(visual["group_local_transform_mm"]),
                    f"{leg_name} {group_name} local transform {visual['occurrence']}",
                )
                _require_attachment(
                    fusion_body,
                    _matmul(endpoint_by_group[group_name], _matrix_from_flat(visual["group_local_transform_mm"])),
                    f"{leg_name} {group_name} reset placement {visual['occurrence']}",
                )
                expected_visuals[key] = visual
                leg_part_keys.add(key)
        _require(leg_part_keys, f"{leg_name} has no visual bodies")

    _require(set(expected_visuals) == set(raw_part_by_key), "attachment reconstruction does not cover every Fusion visual exactly once")


def _compare_values(path: str, expected: Any, actual: Any, errors: list[str]) -> None:
    if isinstance(expected, dict) and isinstance(actual, dict):
        if set(expected) != set(actual):
            errors.append(f"{path}: object keys differ")
            return
        for key in sorted(expected):
            _compare_values(f"{path}.{key}", expected[key], actual[key], errors)
        return
    if isinstance(expected, list) and isinstance(actual, list):
        if len(expected) != len(actual):
            errors.append(f"{path}: list lengths differ")
            return
        for index, (left, right) in enumerate(zip(expected, actual)):
            _compare_values(f"{path}[{index}]", left, right, errors)
        return
    if isinstance(expected, (int, float)) and isinstance(actual, (int, float)):
        tolerance = 1e-9
        if "transform" in path or "geometry_" in path or "anchor_" in path or "axis" in path:
            tolerance = 0.01
        if not math.isclose(float(expected), float(actual), rel_tol=0.0, abs_tol=tolerance):
            errors.append(f"{path}: {expected!r} != {actual!r}")
        return
    if expected != actual:
        errors.append(f"{path}: {expected!r} != {actual!r}")


def compare_live(staging_path: Path, committed_path: Path = COMMITTED_SNAPSHOT) -> None:
    staged = validate_snapshot(staging_path)
    committed = validate_snapshot(committed_path)
    errors: list[str] = []
    _compare_values("snapshot", committed, staged, errors)
    if errors:
        preview = "\n".join(errors[:30])
        extra = f"\n... {len(errors) - 30} more differences" if len(errors) > 30 else ""
        raise ValidationError("live Fusion snapshot differs from committed geometry:\n" + preview + extra)
    if committed_path == COMMITTED_SNAPSHOT and COMMITTED_MANIFEST.exists():
        manifest = derive_manifest(staged, snapshot_path=staging_path)
        committed_manifest = _load_json(COMMITTED_MANIFEST)
        errors = []
        _compare_values("manifest", committed_manifest, manifest, errors)
        if errors:
            preview = "\n".join(errors[:30])
            extra = f"\n... {len(errors) - 30} more differences" if len(errors) > 30 else ""
            raise ValidationError("live Fusion manifest differs from committed geometry:\n" + preview + extra)


def _fmt(value: float, ndigits: int = 9) -> str:
    text = f"{float(value):.{ndigits}f}".rstrip("0").rstrip(".")
    return text if text and text != "-0" else "0"


def _vec(values: Iterable[float], scale: float = 1.0) -> str:
    return " ".join(_fmt(float(value) * scale) for value in values)


def _webots_rotation_from_matrix(flat: list[float]) -> tuple[list[float], float]:
    rows = _matrix_from_flat(flat)
    trace = rows[0][0] + rows[1][1] + rows[2][2]
    angle = math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0)))
    skew = [
        rows[2][1] - rows[1][2],
        rows[0][2] - rows[2][0],
        rows[1][0] - rows[0][1],
    ]
    if abs(angle) < 1e-9:
        return [0.0, 1.0, 0.0], 0.0
    if abs(math.pi - angle) < 1e-7 or (trace < -0.999999 and _length(skew) < 1e-7):
        diagonals = [rows[0][0], rows[1][1], rows[2][2]]
        largest = max(range(3), key=lambda index: diagonals[index])
        axis = [0.0, 0.0, 0.0]
        next_index = (largest + 1) % 3
        previous_index = (largest + 2) % 3
        value = math.sqrt(
            max(
                0.0,
                1.0
                + rows[largest][largest]
                - rows[next_index][next_index]
                - rows[previous_index][previous_index],
            )
        ) / 2.0
        _require(value > 1e-12, f"invalid 180-degree rotation matrix: {flat}")
        axis[largest] = value
        if largest == 0:
            axis[1] = (rows[0][1] + rows[1][0]) / (4.0 * value)
            axis[2] = (rows[0][2] + rows[2][0]) / (4.0 * value)
        elif largest == 1:
            axis[0] = (rows[0][1] + rows[1][0]) / (4.0 * value)
            axis[2] = (rows[1][2] + rows[2][1]) / (4.0 * value)
        else:
            axis[0] = (rows[0][2] + rows[2][0]) / (4.0 * value)
            axis[1] = (rows[1][2] + rows[2][1]) / (4.0 * value)
        return _unit(axis), math.pi
    denom = 2.0 * math.sin(angle)
    return _unit([value / denom for value in skew]), angle


def _visual_shape(visual: dict[str, Any], indent: str) -> list[str]:
    return _visual_shape_from_flat(visual["body_centered_transform_mm"], visual["asset_path"], indent)


def _visual_shape_from_flat(flat: list[float], asset_path: str, indent: str) -> list[str]:
    translation = _matrix_translation(flat)
    axis, angle = _webots_rotation_from_matrix(flat)
    return [
        f"{indent}Transform {{",
        f"{indent}  translation {_vec(translation, 0.001)}",
        f"{indent}  rotation {_vec(axis)} {_fmt(angle)}",
        f"{indent}  scale 0.001 0.001 0.001",
        f"{indent}  children [",
        f"{indent}    Shape {{",
        f"{indent}      castShadows FALSE",
        f"{indent}      appearance PBRAppearance {{ baseColor 0.22 0.24 0.28 roughness 0.55 metalness 0.25 }}",
        f"{indent}      geometry Mesh {{ url [ \"../assets/cad/{Path(asset_path).name}\" ] }}",
        f"{indent}    }}",
        f"{indent}  ]",
        f"{indent}}}",
    ]


def _collision_box(bounds: dict[str, list[float]], indent: str, scale: float = 0.001) -> list[str]:
    center = _bounds_center(bounds)
    size = [max(0.005, value) for value in _bounds_size(bounds)]
    return [
        f"{indent}Transform {{",
        f"{indent}  translation {_vec(center, scale)}",
        f"{indent}  children [ Box {{ size {_vec(size, scale)} }} ]",
        f"{indent}}}",
    ]


def _leg_rotation_field(leg: dict[str, Any]) -> str:
    axis, angle = _webots_rotation_from_matrix(leg["leg_to_body_transform_mm"])
    return f"{_vec(axis)} {_fmt(angle)}"


def _leg_local_visual(visual: dict[str, Any], leg: dict[str, Any], origin_leg_mm: list[float]) -> list[float]:
    body_to_leg = _inverse_rigid(_matrix_from_flat(leg["leg_to_body_transform_mm"]))
    local = _matmul(body_to_leg, _matrix_from_flat(visual["body_centered_transform_mm"]))
    local = _matmul(_translation_matrix([-value for value in origin_leg_mm]), local)
    return _flat_from_matrix(local)


def _leg_local_point(body_point_mm: list[float], leg: dict[str, Any], origin_leg_mm: list[float] | None = None) -> list[float]:
    body_to_leg = _inverse_rigid(_matrix_from_flat(leg["leg_to_body_transform_mm"]))
    point = _transform_point(body_to_leg, body_point_mm)
    if origin_leg_mm is not None:
        point = _sub(point, origin_leg_mm)
    return _round_list(point, 6)


def _link_collision_between(start_mm: list[float], end_mm: list[float], thickness_mm: float = 24.0) -> dict[str, list[float]]:
    return {
        "min": [
            min(start_mm[0], end_mm[0]) - thickness_mm / 2.0,
            min(start_mm[1], end_mm[1]) - thickness_mm / 2.0,
            min(start_mm[2], end_mm[2]) - thickness_mm / 2.0,
        ],
        "max": [
            max(start_mm[0], end_mm[0]) + thickness_mm / 2.0,
            max(start_mm[1], end_mm[1]) + thickness_mm / 2.0,
            max(start_mm[2], end_mm[2]) + thickness_mm / 2.0,
        ],
    }


def _group_visual_lines(leg: dict[str, Any], group_name: str, origin_leg_mm: list[float], indent: str) -> list[str]:
    lines: list[str] = []
    for visual in leg["groups"][group_name]["visuals"]:
        # Manifest group-local transforms already include the CAD-derived
        # frame rotation and origin.  Reusing them here avoids applying the
        # leg transform a second time for asymmetric mounts.
        lines.extend(_visual_shape_from_flat(visual["group_local_transform_mm"], visual["asset_path"], indent))
    return lines


def _joint_zero_rotation(joint: dict[str, Any]) -> str:
    """Render the endpoint's reset-zero rotation from its parent transform."""

    axis, angle = _webots_rotation_from_matrix(joint["zero_transform_parent_mm"])
    return f"{_vec(axis)} {_fmt(angle)}"


def _joint_device_lines(leg_name: str, role: str, min_limit: float, max_limit: float, indent: str) -> list[str]:
    return [
        f"{indent}device [",
        f"{indent}  RotationalMotor {{ name \"{leg_name}_{role}_motor\" maxTorque IS jointTorque minPosition {_fmt(min_limit)} maxPosition {_fmt(max_limit)} }}",
        f"{indent}  PositionSensor {{ name \"{leg_name}_{role}_sensor\" }}",
        f"{indent}]",
    ]


def _generate_leg_chain(leg: dict[str, Any], indent: str = "      ") -> list[str]:
    joints = {joint["role"]: joint for joint in leg["joints"]}
    coxa_anchor = joints["coxa"]["anchor_parent_mm"]
    femur_anchor = joints["femur"]["anchor_parent_mm"]
    tibia_anchor = joints["tibia"]["anchor_parent_mm"]
    foot = _leg_local_point(leg["foot_contact_body_mm"], leg)
    # Hinge anchors are parent-local.  The endpoint Solid is translated to
    # the same anchor, while its zero rotation cancels the documented reset
    # command so q=[0,28,115] reproduces the Fusion snapshot.
    femur_rel = list(femur_anchor)
    tibia_rel = list(tibia_anchor)
    foot_rel = _sub(foot, joints["tibia"]["anchor_leg_mm"])
    lines = [
        f"{indent}Transform {{",
        f"{indent}  translation {_vec(leg['origin_body_mm'], 0.001)}",
        f"{indent}  rotation {_leg_rotation_field(leg)}",
        f"{indent}  children [",
        f"{indent}    # {leg['name']} mount is fixed to body; moving links are nested below.",
    ]
    lines.extend(_group_visual_lines(leg, "mount", [0.0, 0.0, 0.0], indent + "    "))
    min_limit, max_limit = [math.radians(value) for value in joints["coxa"]["limits_deg"]]
    lines.extend(
        [
            f"{indent}    HingeJoint {{",
            f"{indent}      jointParameters HingeJointParameters {{ anchor 0 0 0 axis 0 1 0 minStop {_fmt(min_limit)} maxStop {_fmt(max_limit)} }}",
            *_joint_device_lines(leg["name"], "coxa", min_limit, max_limit, indent + "      "),
            f"{indent}      endPoint Solid {{",
            f"{indent}        name \"{leg['name']}_coxa_solid\"",
            f"{indent}        translation {_vec(coxa_anchor, 0.001)}",
            f"{indent}        rotation {_joint_zero_rotation(joints['coxa'])}",
            f"{indent}        children [",
            *_group_visual_lines(leg, "coxa", coxa_anchor, indent + "          "),
        ]
    )
    femur_min, femur_max = [math.radians(value) for value in joints["femur"]["limits_deg"]]
    lines.extend(
        [
            f"{indent}          HingeJoint {{",
            f"{indent}            jointParameters HingeJointParameters {{ anchor {_vec(femur_rel, 0.001)} axis 0 0 1 minStop {_fmt(femur_min)} maxStop {_fmt(femur_max)} }}",
            *_joint_device_lines(leg["name"], "femur", femur_min, femur_max, indent + "            "),
            f"{indent}            endPoint Solid {{",
            f"{indent}              name \"{leg['name']}_femur_solid\"",
            f"{indent}              translation {_vec(femur_rel, 0.001)}",
            f"{indent}              rotation {_joint_zero_rotation(joints['femur'])}",
            f"{indent}              children [",
            *_group_visual_lines(leg, "femur", femur_anchor, indent + "                "),
        ]
    )
    tibia_min, tibia_max = [math.radians(value) for value in joints["tibia"]["limits_deg"]]
    lines.extend(
        [
            f"{indent}                HingeJoint {{",
            f"{indent}                  jointParameters HingeJointParameters {{ anchor {_vec(tibia_rel, 0.001)} axis 0 0 -1 minStop {_fmt(tibia_min)} maxStop {_fmt(tibia_max)} }}",
            *_joint_device_lines(leg["name"], "tibia", tibia_min, tibia_max, indent + "                  "),
            f"{indent}                  endPoint Solid {{",
            f"{indent}                    name \"{leg['name']}_tibia_solid\"",
            f"{indent}                    translation {_vec(tibia_rel, 0.001)}",
            f"{indent}                    rotation {_joint_zero_rotation(joints['tibia'])}",
            f"{indent}                    contactMaterial \"spider_foot\"",
            f"{indent}                    children [",
            *_group_visual_lines(leg, "tibia", tibia_anchor, indent + "                      "),
            f"{indent}                    ]",
            f"{indent}                    boundingObject Group {{",
            f"{indent}                      children [",
            *_collision_box(_link_collision_between([0.0, 0.0, 0.0], foot_rel, 18.0), indent + "                        ", 0.001),
            f"{indent}                      ]",
            f"{indent}                    }}",
            f"{indent}                    physics Physics {{ density -1 mass IS linkMass }}",
            f"{indent}                  }}",
            f"{indent}                }}",
            f"{indent}              ]",
            f"{indent}              boundingObject Group {{",
            f"{indent}                children [",
            *_collision_box(_link_collision_between([0.0, 0.0, 0.0], tibia_rel, 20.0), indent + "                  ", 0.001),
            f"{indent}                ]",
            f"{indent}              }}",
            f"{indent}              physics Physics {{ density -1 mass IS linkMass }}",
            f"{indent}            }}",
            f"{indent}          }}",
            f"{indent}        ]",
            f"{indent}        boundingObject Group {{",
            f"{indent}          children [",
            *_collision_box(_link_collision_between([0.0, 0.0, 0.0], femur_rel, 20.0), indent + "            ", 0.001),
            f"{indent}          ]",
            f"{indent}        }}",
            f"{indent}        physics Physics {{ density -1 mass IS linkMass }}",
            f"{indent}      }}",
            f"{indent}    }}",
            f"{indent}  ]",
            f"{indent}}}",
        ]
    )
    return lines


def generate_spider_proto(manifest: dict[str, Any]) -> str:
    lines = [
        "#VRML_SIM R2025a utf8",
        "",
        "# Generated by tools/cad_sync.py from webots/cad/spider_geometry.v1.json.",
        "PROTO Spider [",
        f"  field SFVec3f translation {_vec(manifest['world']['poses']['flat']['initial_translation_m'])}",
        "  field SFRotation rotation 0 1 0 0",
        "  field SFString controller \"spider_controller\"",
        "  field SFString customData \"\"",
        "  field SFString name \"spider\"",
        "  field SFBool supervisor TRUE",
        "  field SFFloat bodyMass 1.2",
        "  field SFFloat linkMass 0.1",
        "  field SFFloat jointTorque 2.5",
        "] {",
        "  Robot {",
        "    translation IS translation",
        "    rotation IS rotation",
        "    name IS name",
        "    controller IS controller",
        "    customData IS customData",
        "    supervisor IS supervisor",
        "    children [",
    ]
    for visual in manifest["body"]["visuals"]:
        lines.extend(_visual_shape(visual, "      "))
    for leg in manifest["legs"]:
        lines.extend(_generate_leg_chain(leg))
    lines.extend(
        [
            "    ]",
            "    boundingObject Group {",
            "      children [",
        ]
    )
    lines.extend(_collision_box(manifest["body"]["collision_primitive"]["size_mm"] and manifest["body"]["bounds_body_mm"], "        "))
    lines.extend(
        [
            "      ]",
            "    }",
            "    physics Physics { density -1 mass IS bodyMass }",
            "  }",
            "}",
            "",
        ]
    )
    return "\n".join(lines)


def generate_worlds(manifest: dict[str, Any]) -> dict[str, str]:
    worlds = {}
    for key, pose in manifest["world"]["poses"].items():
        angle = pose["terrain_angle_deg"]
        externs = ['EXTERNPROTO "../protos/Spider.proto"']
        terrain = """Solid {
  translation 0 -0.05 0
  contactMaterial "ground"
  children [
    Shape {
      appearance PBRAppearance { baseColor 0.35 0.40 0.35 roughness 0.9 }
      geometry Box { size 4 0.1 4 }
    }
  ]
  boundingObject Box { size 4 0.1 4 }
  physics NULL
}"""
        if angle:
            externs.append('EXTERNPROTO "../protos/SlopeTerrain.proto"')
            terrain = f"SlopeTerrain {{ angle {angle} }}"
        worlds[key] = "\n".join(
            [
                "#VRML_SIM R2025a utf8",
                "",
                *externs,
                "",
                "WorldInfo {",
                "  coordinateSystem \"NUE\"",
                "  basicTimeStep 20",
                "  contactProperties [",
                "    ContactProperties {",
                "      material1 \"spider_foot\"",
                "      material2 \"ground\"",
                "      coulombFriction [ 0.8 ]",
                "      bounce 0",
                "    }",
                "  ]",
                "}",
                "Viewpoint {",
                "  position 0.8 0.48 0.8",
                "  orientation -0.377400 0.697553 0.609089 2.816821",
                f"  description \"Spider validation world {angle} deg\"",
                f"  follow \"spider_{key}\"",
                "  followType \"Tracking Shot\"",
                "  followSmoothness 0.15",
                "}",
                "Background { skyColor [ 0.18 0.22 0.30 ] }",
                "DirectionalLight {",
                "  ambientIntensity 0.6",
                "  direction -0.4 -1 -0.3",
                "  intensity 1.2",
                "  castShadows TRUE",
                "}",
                terrain,
                f"Spider {{ name \"spider_{key}\" translation {_vec(pose['initial_translation_m'])} rotation {_vec(pose['initial_rotation'])} }}",
                "",
            ]
        )
    return worlds


def promote(staging_path: Path) -> None:
    snapshot = validate_snapshot(staging_path)
    overrides = _load_overrides()
    manifest = derive_manifest(snapshot, overrides, staging_path)
    validate_manifest(manifest)
    COMMITTED_ASSETS.mkdir(parents=True, exist_ok=True)
    expected_names = {Path(str(asset["path"])).name for asset in snapshot["assets"]}
    for existing in COMMITTED_ASSETS.glob("*.stl"):
        if existing.name not in expected_names:
            existing.unlink()
    for asset in snapshot["assets"]:
        source = _asset_path(asset, staging_path)
        destination = ROOT / str(asset["path"])
        destination.parent.mkdir(parents=True, exist_ok=True)
        if source.resolve() != destination.resolve():
            shutil.copyfile(source, destination)
    _write_json(COMMITTED_SNAPSHOT, snapshot)
    _write_json(COMMITTED_MANIFEST, manifest)
    SPIDER_PROTO.write_text(generate_spider_proto(manifest), encoding="ascii")
    for key, text in generate_worlds(manifest).items():
        (WORLD_DIR / f"{key}.wbt").write_text(text, encoding="ascii")
    validate_snapshot(COMMITTED_SNAPSHOT)
    validate_manifest(_load_json(COMMITTED_MANIFEST))


def check_generated() -> None:
    snapshot = validate_snapshot(COMMITTED_SNAPSHOT)
    manifest = derive_manifest(snapshot, snapshot_path=COMMITTED_SNAPSHOT)
    committed_manifest = _load_json(COMMITTED_MANIFEST)
    validate_manifest(committed_manifest)
    errors: list[str] = []
    _compare_values("manifest", committed_manifest, manifest, errors)
    expected_proto = generate_spider_proto(committed_manifest)
    if SPIDER_PROTO.read_text(encoding="ascii") != expected_proto:
        errors.append("Spider.proto bytes are not generated from spider_geometry.v1.json")
    for key, text in generate_worlds(committed_manifest).items():
        path = WORLD_DIR / f"{key}.wbt"
        if path.read_text(encoding="ascii") != text:
            errors.append(f"{path.name} bytes are not generated from spider_geometry.v1.json")
    if errors:
        raise ValidationError("\n".join(errors[:30]))


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    check = subparsers.add_parser("check-snapshot", help="validate a Fusion snapshot")
    check.add_argument("--snapshot", type=Path, default=COMMITTED_SNAPSHOT)
    promote_parser = subparsers.add_parser("promote-snapshot", help="promote validated Fusion staging output")
    promote_parser.add_argument("--staging", type=Path, default=STAGING / "fusion_snapshot.v1.json")
    compare = subparsers.add_parser("compare-live", help="compare staged live Fusion output to the committed snapshot")
    compare.add_argument("--staging", type=Path, default=STAGING / "fusion_snapshot.v1.json")
    compare.add_argument("--committed", type=Path, default=COMMITTED_SNAPSHOT)
    subparsers.add_parser("check-generated", help="validate committed manifest and generated Webots bytes")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == "check-snapshot":
            snapshot = validate_snapshot(args.snapshot)
            if args.snapshot == COMMITTED_SNAPSHOT and COMMITTED_MANIFEST.exists():
                validate_manifest(_load_json(COMMITTED_MANIFEST))
            else:
                validate_manifest(derive_manifest(snapshot, snapshot_path=args.snapshot))
        elif args.command == "promote-snapshot":
            promote(args.staging)
        elif args.command == "compare-live":
            compare_live(args.staging, args.committed)
        elif args.command == "check-generated":
            check_generated()
        else:  # pragma: no cover - argparse owns command choices.
            raise AssertionError(args.command)
    except ValidationError as error:
        print(f"CAD sync failed: {error}", file=sys.stderr)
        return 1
    print(f"CAD sync {args.command} passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

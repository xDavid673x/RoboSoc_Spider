"""Read-only Fusion exporter for the Webots spider geometry snapshot.

Run this file from Fusion's Scripts and Add-Ins dialog while the authoritative
``Spider`` design is active.  The design is never modified.  All output is
written below ``webots/cad/.staging`` and must pass ``tools/cad_sync.py``
before it can be promoted into the repository snapshot.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import re
import struct
import traceback

import adsk.core
import adsk.fusion


SCHEMA_VERSION = 1
EXPECTED_DOCUMENT_NAME = "Spider"
EXPECTED_LINEAGE = "zyUEBEY-SrODxrte3NrQMw"
ARCHIVE_SHA256 = "63475fde834256bd3cb79ffbde11c24fb34a39b3abd0d019993e2314a8f25491"
LEG_OCCURRENCES = {
    "legAssemble:2": "legj",
    "legAssemble:3": "legn",
    "legAssemble:4": "legl",
    "legAssemble:5": "legk",
    "legAssemble:6": "legm",
    "legAssemble:7": "legi",
}
BODY_OCCURRENCE = "Hex base-smaller:1"

# Fusion Design API values are centimetres/radians.  The assembled Spider root
# is already Y-up, matching Webots NUE.  The historical X,Z,-Y rotation applies
# only to standalone component STL files; applying it to occurrence transforms
# would rotate the complete assembly onto its side a second time.
FUSION_TO_WEBOTS = (
    (1.0, 0.0, 0.0, 0.0),
    (0.0, 1.0, 0.0, 0.0),
    (0.0, 0.0, 1.0, 0.0),
    (0.0, 0.0, 0.0, 1.0),
)

def _matrix_rows(matrix: adsk.core.Matrix3D) -> list[list[float]]:
    values = [float(value) for value in matrix.asArray()]
    if len(values) != 16:
        raise ValueError("Fusion Matrix3D did not contain 16 values")
    return [values[index : index + 4] for index in range(0, 16, 4)]


def _matmul(left: list[list[float]] | tuple, right: list[list[float]]) -> list[list[float]]:
    return [
        [sum(float(left[row][k]) * right[k][column] for k in range(4)) for column in range(4)]
        for row in range(4)
    ]


def _rounded_matrix(rows: list[list[float]], translation_scale: float = 1.0) -> list[float]:
    result: list[float] = []
    for row_index, row in enumerate(rows):
        for column_index, value in enumerate(row):
            if column_index == 3 and row_index < 3:
                value *= translation_scale
            result.append(round(float(value), 9))
    return result


def _fusion_matrix_cm(matrix: adsk.core.Matrix3D) -> list[float]:
    return _rounded_matrix(_matrix_rows(matrix))


def _webots_matrix_mm(matrix: adsk.core.Matrix3D) -> list[float]:
    # STL vertices remain in their native Fusion component coordinates.  The
    # left multiplication therefore maps those raw XYZ values into Webots.
    return _rounded_matrix(_matmul(FUSION_TO_WEBOTS, _matrix_rows(matrix)), 10.0)


def _webots_vector(vector: adsk.core.Vector3D | None) -> list[float] | None:
    if vector is None:
        return None
    return [round(float(vector.x), 12), round(float(vector.y), 12), round(float(vector.z), 12)]


def _webots_point_mm(point: adsk.core.Point3D) -> list[float]:
    return [
        round(float(point.x) * 10.0, 6),
        round(float(point.y) * 10.0, 6),
        round(float(point.z) * 10.0, 6),
    ]


def _entity_token(entity) -> str | None:
    """Read optional Fusion provenance without aborting on stale API proxies.

    Fusion can retain a deleted/invalid rigid-group proxy in ``allRigidGroups``
    after an assembly edit.  Its ``entityToken`` accessor then raises an
    internal validation error even though the group name and occurrence paths
    are still readable.  Tokens are provenance metadata, not connectivity
    inputs, so preserve the failure explicitly and continue the export.
    """

    try:
        return str(entity.entityToken)
    except Exception as error:  # Fusion API raises several version-specific types.
        return f"<unavailable:{type(error).__name__}>"


def _body_bounds_mm(body) -> dict[str, list[float]]:
    bounds = body.boundingBox
    converted = [_webots_point_mm(bounds.minPoint), _webots_point_mm(bounds.maxPoint)]
    return {
        "min": [min(points[axis] for points in converted) for axis in range(3)],
        "max": [max(points[axis] for points in converted) for axis in range(3)],
    }


def _join_occurrence_path(context, occurrence) -> str:
    if occurrence is None:
        return "<root>"
    path = occurrence.fullPathName
    if context is None:
        return path
    prefix = context.fullPathName
    return path if path == prefix or path.startswith(prefix + "+") else prefix + "+" + path


def _lineage_id(data_file) -> str:
    if data_file is None:
        return ""
    value = str(getattr(data_file, "id", "") or "")
    return value.rsplit(":", 1)[-1]


def _safe_version(data_file) -> int | None:
    value = getattr(data_file, "versionNumber", None)
    try:
        return int(value) if value is not None else None
    except (TypeError, ValueError):
        return None


def _slug(value: str) -> str:
    cleaned = re.sub(r"[^a-zA-Z0-9]+", "_", value).strip("_").lower()
    return cleaned or "body"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _read_binary_stl(path: Path) -> tuple[list[list[list[float]]], list[list[float]]]:
    payload = path.read_bytes()
    if len(payload) < 84:
        raise ValueError(f"STL is too short: {path}")
    triangle_count = struct.unpack_from("<I", payload, 80)[0]
    expected_size = 84 + triangle_count * 50
    if len(payload) != expected_size:
        raise ValueError(f"expected binary STL ({expected_size} bytes), got {len(payload)}: {path}")
    triangles: list[list[list[float]]] = []
    points: list[list[float]] = []
    offset = 84
    for _ in range(triangle_count):
        values = struct.unpack_from("<12fH", payload, offset)
        vertices = [
            [float(values[3 + index * 3 + axis]) for axis in range(3)]
            for index in range(3)
        ]
        triangles.append(vertices)
        points.extend(vertices)
        offset += 50
    return triangles, points


def _mesh_metadata(path: Path) -> dict[str, object]:
    triangles, points = _read_binary_stl(path)
    normalized = []
    for triangle in triangles:
        vertices = sorted(tuple(round(value, 3) for value in vertex) for vertex in triangle)
        normalized.append(vertices)
    normalized.sort()
    digest = hashlib.sha256(
        json.dumps(normalized, separators=(",", ":")).encode("ascii")
    ).hexdigest()
    minimum = [min(point[axis] for point in points) for axis in range(3)]
    maximum = [max(point[axis] for point in points) for axis in range(3)]
    return {
        "sha256": _sha256(path),
        "geometry_sha256_0_001mm": digest,
        "triangle_count": len(triangles),
        "bounds_mm": {
            "min": [round(value, 6) for value in minimum],
            "max": [round(value, 6) for value in maximum],
        },
    }


def _configure_stl_export(options) -> dict[str, object]:
    options.sendToPrintUtility = False
    options.isBinaryFormat = True
    options.unitType = adsk.fusion.DistanceUnits.MillimeterDistanceUnits
    options.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
    return {
        "preset": "high",
        "surface_deviation_fusion_units": round(float(options.surfaceDeviation), 12),
        "maximum_edge_length_fusion_units": round(float(options.maximumEdgeLength), 12),
        "normal_deviation_rad": round(float(options.normalDeviation), 12),
        "aspect_ratio": round(float(options.aspectRatio), 12),
        "note": "Resolved numeric values are recorded because Fusion's preset values are build-dependent; asset hashes remain authoritative.",
    }


def _top_occurrence(path: str) -> str:
    return path.split("+", 1)[0]


def _body_identity(occurrence, body) -> str:
    component = occurrence.component
    document = component.parentDesign.parentDocument
    data_file = getattr(document, "dataFile", None)
    return "|".join(
        (
            _lineage_id(data_file),
            str(component.name),
            str(getattr(body.nativeObject, "name", None) or body.name),
        )
    )


def _occurrence_record(occurrence) -> dict[str, object]:
    component = occurrence.component
    document = component.parentDesign.parentDocument
    data_file = getattr(document, "dataFile", None)
    return {
        "full_path": occurrence.fullPathName,
        "name": occurrence.name,
        "component": component.name,
        "source_lineage": _lineage_id(data_file),
        "source_version": _safe_version(data_file),
        "is_visible": bool(occurrence.isVisible),
        "is_grounded": bool(getattr(occurrence, "isGrounded", False)),
        "is_rigid": bool(getattr(occurrence, "isRigid", False)),
        "is_flexible": bool(getattr(occurrence, "isFlexible", False)),
        "fusion_transform_cm": _fusion_matrix_cm(occurrence.transform2),
        "webots_transform_mm": _webots_matrix_mm(occurrence.transform2),
    }


def _joint_type_name(joint_motion) -> str:
    names = {
        adsk.fusion.JointTypes.RigidJointType: "rigid",
        adsk.fusion.JointTypes.RevoluteJointType: "revolute",
        adsk.fusion.JointTypes.SliderJointType: "slider",
        adsk.fusion.JointTypes.CylindricalJointType: "cylindrical",
        adsk.fusion.JointTypes.PinSlotJointType: "pin_slot",
        adsk.fusion.JointTypes.PlanarJointType: "planar",
        adsk.fusion.JointTypes.BallJointType: "ball",
    }
    return names.get(joint_motion.jointType, f"unknown:{int(joint_motion.jointType)}")


def _joint_record(joint) -> dict[str, object]:
    motion = joint.jointMotion
    occurrence_one = joint.occurrenceOne
    occurrence_two = joint.occurrenceTwo
    record: dict[str, object] = {
        "name": joint.name,
        "entity_token": _entity_token(joint),
        "type": _joint_type_name(motion),
        "occurrence_one": occurrence_one.fullPathName if occurrence_one else "<root>",
        "occurrence_two": occurrence_two.fullPathName if occurrence_two else "<root>",
        "is_flipped": bool(joint.isFlipped),
        "is_valid": bool(joint.isValid),
        "validity_source": "Joint.isValid",
    }
    for side in ("one", "two"):
        matrix = getattr(joint, f"geometry{side.title()}Transform", None)
        if matrix is not None:
            record[f"geometry_{side}_fusion_cm"] = _fusion_matrix_cm(matrix)
            record[f"geometry_{side}_webots_mm"] = _webots_matrix_mm(matrix)
    if record["type"] == "revolute":
        revolute = adsk.fusion.RevoluteJointMotion.cast(motion)
        limits = revolute.rotationLimits
        record.update(
            {
                "axis_body_webots": _webots_vector(revolute.rotationAxisVector),
                "rotation_value_deg": round(math.degrees(float(revolute.rotationValue)), 9),
                "limits": {
                    "minimum_enabled": bool(limits.isMinimumValueEnabled),
                    "minimum_deg": round(math.degrees(float(limits.minimumValue)), 9),
                    "maximum_enabled": bool(limits.isMaximumValueEnabled),
                    "maximum_deg": round(math.degrees(float(limits.maximumValue)), 9),
                    "rest_enabled": bool(limits.isRestValueEnabled),
                    "rest_deg": round(math.degrees(float(limits.restValue)), 9),
                },
            }
        )
    return record


def _as_built_joint_record(joint) -> dict[str, object]:
    motion = joint.jointMotion
    occurrence_one = joint.occurrenceOne
    occurrence_two = joint.occurrenceTwo
    transform = joint.transform
    context = joint.assemblyContext
    root_transform_rows = _matrix_rows(transform)
    if context is not None:
        root_transform_rows = _matmul(_matrix_rows(context.transform2), root_transform_rows)
    fusion_root_transform = _rounded_matrix(root_transform_rows)
    webots_root_transform = _rounded_matrix(
        _matmul(FUSION_TO_WEBOTS, root_transform_rows), 10.0
    )
    record: dict[str, object] = {
        "name": joint.name,
        "entity_token": _entity_token(joint),
        "type": _joint_type_name(motion),
        "assembly_context": context.fullPathName if context else "<root>",
        "occurrence_one": _join_occurrence_path(context, occurrence_one),
        "occurrence_two": _join_occurrence_path(context, occurrence_two),
        "is_valid": bool(joint.isValid),
        "validity_source": "AsBuiltJoint.isValid",
        "fusion_transform_context_cm": _fusion_matrix_cm(transform),
        "fusion_transform_root_cm": fusion_root_transform,
        "webots_transform_mm": webots_root_transform,
        # AsBuiltJoint exposes one solved frame shared by both connected sides,
        # unlike Joint.geometryOneTransform/geometryTwoTransform.  Recording the
        # shared frame twice makes that API provenance explicit and lets the
        # standard-library promotion gate enforce side coincidence uniformly.
        "side_frame_source": "shared_as_built_joint_transform",
        "geometry_one_webots_mm": webots_root_transform,
        "geometry_two_webots_mm": webots_root_transform,
    }
    if record["type"] == "revolute":
        revolute = adsk.fusion.RevoluteJointMotion.cast(motion)
        limits = revolute.rotationLimits
        record.update(
            {
                "axis_body_webots": _webots_vector(revolute.rotationAxisVector),
                "rotation_value_deg": round(math.degrees(float(revolute.rotationValue)), 9),
                "limits": {
                    "minimum_enabled": bool(limits.isMinimumValueEnabled),
                    "minimum_deg": round(math.degrees(float(limits.minimumValue)), 9),
                    "maximum_enabled": bool(limits.isMaximumValueEnabled),
                    "maximum_deg": round(math.degrees(float(limits.maximumValue)), 9),
                    "rest_enabled": bool(limits.isRestValueEnabled),
                    "rest_deg": round(math.degrees(float(limits.restValue)), 9),
                },
            }
        )
    return record


def _joint_origin_record(origin) -> dict[str, object]:
    transform = origin.transform
    return {
        "name": origin.name,
        "entity_token": _entity_token(origin),
        "fusion_transform_cm": _fusion_matrix_cm(transform),
        "webots_transform_mm": _webots_matrix_mm(transform),
        "primary_axis_body_webots": _webots_vector(origin.primaryAxisVector),
        "secondary_axis_body_webots": _webots_vector(origin.secondaryAxisVector),
        "third_axis_body_webots": _webots_vector(origin.thirdAxisVector),
    }


def _rigid_group_record(group) -> dict[str, object]:
    context = getattr(group, "assemblyContext", None)
    return {
        "name": group.name,
        "entity_token": _entity_token(group),
        "assembly_context": context.fullPathName if context else "<root>",
        "occurrences": sorted(occurrence.fullPathName for occurrence in group.occurrences),
    }


def _export_snapshot(app, design: adsk.fusion.Design, repo_root: Path) -> Path:
    document = app.activeDocument
    data_file = getattr(document, "dataFile", None)
    lineage = _lineage_id(data_file)
    if document.name != EXPECTED_DOCUMENT_NAME:
        raise RuntimeError(f"expected active document {EXPECTED_DOCUMENT_NAME!r}, got {document.name!r}")
    if lineage != EXPECTED_LINEAGE:
        raise RuntimeError(f"expected lineage {EXPECTED_LINEAGE}, got {lineage or '<none>'}")

    root = design.rootComponent
    occurrence_by_path = {occ.fullPathName: occ for occ in root.allOccurrences}
    for required in (BODY_OCCURRENCE, *LEG_OCCURRENCES):
        if required not in occurrence_by_path:
            raise RuntimeError(f"required top-level occurrence is missing: {required}")

    staging = repo_root / "webots" / "cad" / ".staging"
    assets_dir = staging / "assets"
    assets_dir.mkdir(parents=True, exist_ok=True)
    for old_asset in assets_dir.glob("*.stl"):
        old_asset.unlink()

    assets: dict[str, dict[str, object]] = {}
    visual_parts: list[dict[str, object]] = []
    export_manager = design.exportManager
    for occurrence in sorted(root.allOccurrences, key=lambda item: item.fullPathName):
        if not occurrence.isVisible:
            continue
        proxy_bodies = {body.name: body for body in occurrence.bRepBodies}
        for native in occurrence.component.bRepBodies:
            if not native.isVisible:
                continue
            identity = _body_identity(occurrence, native)
            asset_id = hashlib.sha256(identity.encode("utf-8")).hexdigest()[:12]
            if asset_id not in assets:
                filename = f"{_slug(occurrence.component.name)}_{_slug(native.name)}_{asset_id}.stl"
                destination = assets_dir / filename
                options = export_manager.createSTLExportOptions(native, str(destination))
                mesh_settings = _configure_stl_export(options)
                if not export_manager.execute(options):
                    raise RuntimeError(f"Fusion failed to export {identity}")
                metadata = _mesh_metadata(destination)
                assets[asset_id] = {
                    "id": asset_id,
                    "identity": identity,
                    "path": f"webots/assets/cad/{filename}",
                    "fusion_mesh_settings": mesh_settings,
                    **metadata,
                }
            visual_parts.append(
                {
                    "occurrence": occurrence.fullPathName,
                    "component": occurrence.component.name,
                    "body": native.name,
                    "body_is_visible": bool(native.isVisible),
                    "asset": asset_id,
                    "fusion_transform_cm": _fusion_matrix_cm(occurrence.transform2),
                    "webots_transform_mm": _webots_matrix_mm(occurrence.transform2),
                    "assembly_bounds_mm": _body_bounds_mm(proxy_bodies.get(native.name, native)),
                }
            )
        mesh_bodies = list(occurrence.component.meshBodies)
        if mesh_bodies:
            identity = "|".join(
                (
                    _lineage_id(getattr(occurrence.component.parentDesign.parentDocument, "dataFile", None)),
                    str(occurrence.component.name),
                    "mesh_component",
                )
            )
            asset_id = hashlib.sha256(identity.encode("utf-8")).hexdigest()[:12]
            if asset_id not in assets:
                filename = f"{_slug(occurrence.component.name)}_mesh_{asset_id}.stl"
                destination = assets_dir / filename
                options = export_manager.createSTLExportOptions(
                    occurrence.component, str(destination)
                )
                mesh_settings = _configure_stl_export(options)
                options.isOneFilePerBody = False
                if not export_manager.execute(options):
                    raise RuntimeError(f"Fusion failed to export {identity}")
                metadata = _mesh_metadata(destination)
                assets[asset_id] = {
                    "id": asset_id,
                    "identity": identity,
                    "path": f"webots/assets/cad/{filename}",
                    "fusion_mesh_settings": mesh_settings,
                    **metadata,
                }
            proxy_meshes = {body.name: body for body in occurrence.meshBodies}
            bounds = []
            for body in mesh_bodies:
                proxy = proxy_meshes.get(body.name, body)
                bounds.append(_body_bounds_mm(proxy))
            visual_parts.append(
                {
                    "occurrence": occurrence.fullPathName,
                    "component": occurrence.component.name,
                    "body": ",".join(sorted(body.name for body in mesh_bodies)),
                    "body_is_visible": any(bool(body.isVisible) for body in mesh_bodies),
                    "geometry_type": "mesh_component",
                    "asset": asset_id,
                    "fusion_transform_cm": _fusion_matrix_cm(occurrence.transform2),
                    "webots_transform_mm": _webots_matrix_mm(occurrence.transform2),
                    "assembly_bounds_mm": {
                        "min": [min(item["min"][axis] for item in bounds) for axis in range(3)],
                        "max": [max(item["max"][axis] for item in bounds) for axis in range(3)],
                    },
                }
            )

    occurrences = [_occurrence_record(occ) for occ in sorted(root.allOccurrences, key=lambda item: item.fullPathName)]
    joints = [_joint_record(joint) for joint in sorted(root.allJoints, key=lambda item: item.name)]
    as_built_joints = [
        _as_built_joint_record(joint)
        for joint in sorted(root.allAsBuiltJoints, key=lambda item: item.name)
    ]
    joint_origins = [
        _joint_origin_record(origin)
        for origin in sorted(root.allJointOrigins, key=lambda item: item.name)
    ]
    rigid_groups = [_rigid_group_record(group) for group in sorted(root.allRigidGroups, key=lambda item: item.name)]

    source_documents = {}
    for occurrence in root.allOccurrences:
        component = occurrence.component
        source_document = component.parentDesign.parentDocument
        source_file = getattr(source_document, "dataFile", None)
        source_lineage = _lineage_id(source_file)
        if source_lineage:
            source_documents[source_lineage] = {
                "name": source_document.name,
                "lineage": source_lineage,
                "version": _safe_version(source_file),
            }

    archive_path = repo_root / "model" / "Full_Spider.f3z"
    archive_actual = _sha256(archive_path) if archive_path.exists() else None
    snapshot = {
        "schema_version": SCHEMA_VERSION,
        "source": {
            "document": document.name,
            "lineage": lineage,
            "version": _safe_version(data_file),
            "fusion_build": str(app.version),
            "archive_path": "model/Full_Spider.f3z",
            "archive_sha256_expected": ARCHIVE_SHA256,
            "archive_sha256_actual": archive_actual,
            "documents": sorted(source_documents.values(), key=lambda item: item["lineage"]),
        },
        "coordinate_system": {
            "fusion_api_length": "centimeter",
            "fusion_snapshot_length": "millimeter",
            "manifest_length": "millimeter",
            "mesh_length": "millimeter",
            "webots_length": "meter",
            "webots_runtime_length": "meter",
            "millimeter_to_webots": 0.001,
            "fusion_to_webots_xyz": ["x", "y", "z"],
            "fusion_to_webots_matrix": [value for row in FUSION_TO_WEBOTS for value in row],
            "standalone_component_mesh_axis_map": ["x", "z", "-y"],
            "assembly_frame_note": "The Fusion root assembly is already Y-up; applying the standalone mesh map again is a double rotation.",
        },
        "mesh_export": {
            "format": "binary_stl",
            "unit": "millimeter",
            "refinement": "high",
            "resolved_settings_location": "assets[*].fusion_mesh_settings",
            "determinism": "byte and 0.001 mm normalized geometry hashes",
        },
        "tolerances": {
            "attachment_mm": 0.01,
            "attachment_deg": 0.01,
            "mesh_quantization_mm": 0.001,
            "planar_fit_mm": 0.25,
            "planar_fit_deg": 0.25,
        },
        "mapping": {
            "body_occurrence": BODY_OCCURRENCE,
            "legs": LEG_OCCURRENCES,
            "tripod_a": ["legi", "legk", "legm"],
            "tripod_b": ["legj", "legl", "legn"],
        },
        "reset_angles_deg": [0.0, 28.0, 115.0],
        "joint_limits_deg": {
            "coxa": [-90.0, 90.0],
            "femur": [-90.0, 90.0],
            "tibia": [0.0, 130.0],
        },
        "assets": sorted(assets.values(), key=lambda item: item["id"]),
        "fusion_assembly": {
            "occurrences": occurrences,
            "visual_parts": visual_parts,
            "joints": joints,
            "as_built_joints": as_built_joints,
            "joint_origins": joint_origins,
            "rigid_groups": rigid_groups,
        },
        "body": {},
        "legs": [],
        "world": {},
    }
    manifest_path = staging / "fusion_snapshot.v1.json"
    manifest_path.write_text(json.dumps(snapshot, indent=2, sort_keys=True) + "\n", encoding="ascii")
    return manifest_path


def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        design = adsk.fusion.Design.cast(app.activeProduct)
        if design is None:
            raise RuntimeError("the active Fusion product is not a Design")
        repo_root = Path(__file__).resolve().parents[2]
        output = _export_snapshot(app, design, repo_root)
        ui.messageBox(f"Spider CAD snapshot exported to:\n{output}")
    except Exception:
        ui.messageBox("Spider CAD export failed:\n\n" + traceback.format_exc())

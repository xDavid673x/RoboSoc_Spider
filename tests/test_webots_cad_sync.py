import copy
import json
import math
from pathlib import Path
import struct

import pytest

from tools import cad_sync


ROOT = Path(__file__).resolve().parents[1]


def _write_triangle_stl(path: Path) -> None:
    header = b"test spider mesh".ljust(80, b"\0")
    triangle = struct.pack(
        "<12fH",
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0,
    )
    path.write_bytes(header + struct.pack("<I", 1) + triangle)


def _snapshot(tmp_path: Path) -> tuple[Path, dict]:
    staging = tmp_path / ".staging"
    assets = staging / "assets"
    assets.mkdir(parents=True)
    mesh = assets / "test_body.stl"
    _write_triangle_stl(mesh)
    metadata = cad_sync._mesh_metadata(mesh)
    occurrence_paths = [
        "Hex base-smaller:1",
        "legAssemble:2",
        "legAssemble:3",
        "legAssemble:4",
        "legAssemble:5",
        "legAssemble:6",
        "legAssemble:7",
    ]
    identity = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    snapshot = {
        "schema_version": 1,
        "source": {
            "document": "Spider",
            "lineage": cad_sync.EXPECTED_LINEAGE,
            "version": 13,
            "fusion_build": "2704.1.53",
            "archive_sha256_expected": cad_sync.EXPECTED_ARCHIVE_SHA256,
            "archive_sha256_actual": cad_sync.EXPECTED_ARCHIVE_SHA256,
            "documents": [],
        },
        "coordinate_system": {
            "fusion_api_length": "centimeter",
            "manifest_length": "millimeter",
            "mesh_length": "millimeter",
            "webots_length": "meter",
            "fusion_to_webots_xyz": ["x", "y", "z"],
            "fusion_to_webots_matrix": identity.copy(),
            "standalone_component_mesh_axis_map": ["x", "z", "-y"],
            "assembly_frame_note": "test",
        },
        "tolerances": {},
        "mapping": {
            "body_occurrence": "Hex base-smaller:1",
            "legs": cad_sync.EXPECTED_LEG_MAPPING.copy(),
            "tripod_a": ["legi", "legk", "legm"],
            "tripod_b": ["legj", "legl", "legn"],
        },
        "reset_angles_deg": [0.0, 28.0, 115.0],
        "joint_limits_deg": {},
        "assets": [
            {
                "id": "mesh",
                "identity": "test",
                "path": "webots/assets/cad/test_body.stl",
                **metadata,
            }
        ],
        "fusion_assembly": {
            "occurrences": [
                {"full_path": value} for value in occurrence_paths
            ],
            "visual_parts": [
                {
                    "occurrence": "Hex base-smaller:1",
                    "component": "body",
                    "body": "mesh",
                    "asset": "mesh",
                    "fusion_transform_cm": identity.copy(),
                    "webots_transform_mm": identity.copy(),
                    "assembly_bounds_mm": {
                        "min": [0.0, 0.0, 0.0],
                        "max": [1.0, 1.0, 1.0],
                    },
                }
            ],
            "as_built_joints": [],
            "joint_origins": [],
            "joints": [],
            "rigid_groups": [],
        },
        "body": {},
        "legs": [],
        "world": {},
    }
    path = staging / "fusion_snapshot.v1.json"
    path.write_text(json.dumps(snapshot), encoding="ascii")
    return path, snapshot


def test_snapshot_validator_accepts_a_complete_checked_export(tmp_path):
    path, snapshot = _snapshot(tmp_path)

    assert cad_sync.validate_snapshot(path) == snapshot


def test_snapshot_validator_rejects_wrong_leg_mapping(tmp_path):
    path, snapshot = _snapshot(tmp_path)
    snapshot["mapping"]["legs"]["legAssemble:7"] = "legj"
    path.write_text(json.dumps(snapshot), encoding="ascii")

    with pytest.raises(cad_sync.ValidationError, match="leg mapping"):
        cad_sync.validate_snapshot(path)


def test_snapshot_validator_rejects_mesh_byte_drift(tmp_path):
    path, _ = _snapshot(tmp_path)
    mesh = path.parent / "assets/test_body.stl"
    mesh.write_bytes(mesh.read_bytes() + b"drift")

    with pytest.raises(cad_sync.ValidationError, match="binary STL"):
        cad_sync.validate_snapshot(path)


def test_live_comparison_uses_attachment_tolerance_for_transforms(tmp_path):
    committed_path, committed = _snapshot(tmp_path / "committed")
    staging_path, staging = _snapshot(tmp_path / "live")
    staging["fusion_assembly"]["visual_parts"][0]["webots_transform_mm"][3] = 0.009
    staging_path.write_text(json.dumps(staging), encoding="ascii")

    cad_sync.compare_live(staging_path, committed_path)

    staging = copy.deepcopy(staging)
    staging["fusion_assembly"]["visual_parts"][0]["webots_transform_mm"][3] = 0.011
    staging_path.write_text(json.dumps(staging), encoding="ascii")
    with pytest.raises(cad_sync.ValidationError, match="differs"):
        cad_sync.compare_live(staging_path, committed_path)


def test_fusion_exporter_declares_read_only_staging_contract():
    exporter = ROOT / "tools/fusion_spider_export/SpiderWebotsExport.py"
    source = exporter.read_text(encoding="ascii")

    compile(source, str(exporter), "exec")
    assert "occurrence.transform2" in source
    assert "root.allJoints" in source
    assert "root.allRigidGroups" in source
    assert '"webots" / "cad" / ".staging"' in source
    assert "MillimeterDistanceUnits" in source
    assert "is_valid" in source
    assert "healthState" not in source
    assert ".deleteMe(" not in source
    assert ".transform2 =" not in source


def test_manifest_schema_surface_matches_committed_manifest():
    schema = json.loads((ROOT / "webots/cad/spider_geometry.schema.json").read_text(encoding="ascii"))
    manifest = json.loads((ROOT / "webots/cad/spider_geometry.v1.json").read_text(encoding="ascii"))

    assert schema["additionalProperties"] is False
    assert set(schema["required"]) == set(manifest)
    assert schema["properties"]["schema_version"]["const"] == manifest["schema_version"]

    for top_level_key in schema["required"]:
        assert top_level_key in schema["properties"]
        assert manifest[top_level_key] not in ({}, [])

    nested_required = {
        "source": {"document", "lineage", "version", "fusion_build", "archive_sha256_expected", "documents"},
        "coordinate_system": {"fusion_api_length", "manifest_length", "mesh_length", "webots_length", "fusion_to_webots_xyz", "fusion_to_webots_matrix"},
        "tolerances": {"attachment_mm", "attachment_deg", "mesh_quantization_mm", "planar_fit_mm", "planar_fit_deg"},
        "mapping": {"body_occurrence", "legs", "tripod_a", "tripod_b"},
        "fusion_assembly": {"occurrences", "visual_parts", "joints", "as_built_joints", "joint_origins", "rigid_groups"},
        "world": {"support_minimum_reset_height_mm", "terrain_half_thickness_mm", "normal_offset_mm", "poses"},
    }
    for key, required in nested_required.items():
        assert required <= set(schema["properties"][key]["required"])
        assert required <= set(manifest[key])


def _committed_manifest() -> dict:
    return json.loads((ROOT / "webots/cad/spider_geometry.v1.json").read_text(encoding="ascii"))


def _assert_rotation_round_trips(flat: list[float]) -> None:
    original = cad_sync._matrix_from_flat(flat)
    axis, angle = cad_sync._webots_rotation_from_matrix(flat)
    reconstructed = cad_sync._rotation_about_axis(axis, angle)

    for row in range(3):
        for column in range(3):
            assert reconstructed[row][column] == pytest.approx(
                original[row][column],
                abs=1e-9,
            )


@pytest.mark.parametrize(
    "axis",
    (
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
        (1.0, 1.0, 0.0),
        (1.0, 1.0, 1.0),
        (1.0, 2.0, 3.0),
        (0.0, 1.0, -1.0),
    ),
)
def test_webots_rotation_serializer_round_trips_pi_rotations(axis):
    flat = cad_sync._flat_from_matrix(cad_sync._rotation_about_axis(axis, math.pi))

    _assert_rotation_round_trips(flat)


def test_webots_rotation_serializer_round_trips_mixed_sign_yz_pi_rotation():
    # 180 degrees around the [0, 1, -1] axis has equal Y/Z diagonals and
    # negative YZ off-diagonals.  The serializer must preserve that sign.
    flat = [
        -1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, -1.0, 0.0,
        0.0, -1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    ]

    _assert_rotation_round_trips(flat)


def test_webots_rotation_serializer_round_trips_committed_transform_rotations():
    manifest = _committed_manifest()
    transforms = []

    for visual in manifest["body"]["visuals"]:
        transforms.extend(
            [
                visual["assembly_transform_body_mm"],
                visual["body_centered_transform_mm"],
                visual["group_local_transform_mm"],
            ]
        )
    for leg in manifest["legs"]:
        transforms.append(leg["leg_to_body_transform_mm"])
        for group in leg["groups"].values():
            transforms.append(group["frame_body_mm"])
            for visual in group["visuals"]:
                transforms.extend(
                    [
                        visual["assembly_transform_body_mm"],
                        visual["body_centered_transform_mm"],
                        visual["group_local_transform_mm"],
                    ]
                )
        for joint in leg["joints"]:
            transforms.extend(
                [
                    joint["zero_transform_parent_mm"],
                    joint["zero_transform_body_mm"],
                ]
            )

    assert transforms
    for flat in transforms:
        _assert_rotation_round_trips(flat)

        original = cad_sync._matrix_from_flat(flat)
        axis, angle = cad_sync._webots_rotation_from_matrix(flat)
        formatted_axis = [float(cad_sync._fmt(value)) for value in axis]
        formatted_angle = float(cad_sync._fmt(angle))
        reconstructed = cad_sync._rotation_about_axis(formatted_axis, formatted_angle)
        for row in range(3):
            for column in range(3):
                assert reconstructed[row][column] == pytest.approx(
                    original[row][column],
                    abs=1e-8,
                )


def test_committed_manifest_reconstructs_all_fusion_visuals_and_joints():
    manifest = _committed_manifest()

    # validate_manifest rebuilds the reset hierarchy from the raw Fusion
    # assembly, so this exercises all 91 visual bodies and 18 joint frames.
    cad_sync.validate_manifest(manifest)


def test_attachment_validator_rejects_mount_drift_over_tolerance():
    manifest = _committed_manifest()
    mount_visual = manifest["legs"][0]["groups"]["mount"]["visuals"][0]
    mount_visual["group_local_transform_mm"][3] += 0.011

    with pytest.raises(cad_sync.ValidationError, match="local transform"):
        cad_sync.validate_manifest(manifest)


def test_attachment_validator_rejects_joint_zero_rotation_drift():
    manifest = _committed_manifest()
    joint = next(joint for joint in manifest["legs"][0]["joints"] if joint["role"] == "femur")
    rotation_delta = cad_sync._rotation_about_axis(joint["webots_axis"], math.radians(0.02))
    existing = cad_sync._matrix_from_flat(joint["zero_transform_parent_mm"])
    joint["zero_transform_parent_mm"] = cad_sync._flat_from_matrix(
        cad_sync._matmul(rotation_delta, existing)
    )

    with pytest.raises(cad_sync.ValidationError, match="zero transform"):
        cad_sync.validate_manifest(manifest)


def test_attachment_validator_rejects_parent_anchor_drift():
    manifest = _committed_manifest()
    joint = next(joint for joint in manifest["legs"][0]["joints"] if joint["role"] == "tibia")
    joint["anchor_parent_mm"][0] += 0.011

    with pytest.raises(cad_sync.ValidationError, match="parent-local anchor"):
        cad_sync.validate_manifest(manifest)

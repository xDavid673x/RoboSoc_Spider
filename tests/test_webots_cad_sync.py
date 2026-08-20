import copy
import json
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
        "legAssemble:1",
        "legAssemble:2",
        "legAssemble:3",
        "legAssemble:4",
        "legAssemble:5",
        "legAssemble:6",
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
    snapshot["mapping"]["legs"]["legAssemble:1"] = "legj"
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
    assert ".deleteMe(" not in source
    assert ".transform2 =" not in source

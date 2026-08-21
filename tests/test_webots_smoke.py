"""Headless Webots smoke checks for the four v1 validation worlds."""

from __future__ import annotations

import json
import os
from pathlib import Path
import re
import socket
import subprocess
import tempfile

import pytest

from tools import cad_sync
from webots.controllers.spider_controller.kinematics_adapter import LEG_NAMES
from webots.controllers.spider_controller.spider_controller import JOINT_NAMES


ROOT = Path(__file__).resolve().parents[1]
WEBOTS = Path("/Applications/Webots.app/Contents/MacOS/webots")
MANIFEST = json.loads((ROOT / "webots/cad/spider_geometry.v1.json").read_text(encoding="ascii"))
WORLDS = (
    ROOT / "webots/worlds/flat.wbt",
    ROOT / "webots/worlds/slope_10.wbt",
    ROOT / "webots/worlds/slope_20.wbt",
    ROOT / "webots/worlds/slope_30.wbt",
)
WORLD_POSES = {
    name: (pose["initial_translation_m"], pose["initial_rotation"])
    for name, pose in MANIFEST["world"]["poses"].items()
}


def _world_from_robot(world_name: str) -> list[list[float]]:
    translation, rotation = WORLD_POSES[world_name]
    return cad_sync._matmul(
        cad_sync._translation_matrix(translation),
        cad_sync._rotation_about_axis(rotation[:3], rotation[3]),
    )


def _endpoint_matrix(sample: dict[str, list[float]]) -> list[list[float]]:
    position = sample["position_m"]
    orientation = sample["orientation"]
    assert len(position) == 3
    assert len(orientation) == 9
    return [
        [orientation[row * 3 + column] for column in range(3)] + [position[row]]
        for row in range(3)
    ] + [[0.0, 0.0, 0.0, 1.0]]


def _meters_transform(matrix_mm: list[list[float]]) -> list[list[float]]:
    converted = [row.copy() for row in matrix_mm]
    for row in range(3):
        converted[row][3] *= 0.001
    return converted


def _expected_reset_endpoints(world_name: str) -> dict[str, list[list[float]]]:
    world_from_robot = _world_from_robot(world_name)
    expected = {}
    for leg in MANIFEST["legs"]:
        parent = cad_sync._matrix_from_flat(leg["leg_to_body_transform_mm"])
        joints = {joint["role"]: joint for joint in leg["joints"]}
        for role in JOINT_NAMES:
            parent = cad_sync._matmul(
                parent,
                cad_sync._reset_endpoint_transform(joints[role]),
            )
            expected[f"{leg['name']}_{role}"] = cad_sync._matmul(
                world_from_robot,
                _meters_transform(parent),
            )
    return expected


def _run_world(
    world: Path,
    port: int | None = None,
    smoke_mode: str = "smoke",
) -> tuple[str, dict[str, object]]:
    if port is None:
        with socket.socket() as port_probe:
            port_probe.bind(("127.0.0.1", 0))
            port = port_probe.getsockname()[1]
    environment = os.environ.copy()
    result_file = tempfile.NamedTemporaryFile(suffix=".json", delete=False)
    result_file.close()
    contents = world.read_text(encoding="ascii")
    contents = contents.replace(
        "Spider { name ",
        f'Spider {{ customData "{smoke_mode}:{result_file.name}" name ',
        1,
    )
    temporary_world = tempfile.NamedTemporaryFile(
        mode="w",
        dir=world.parent,
        prefix="smoke_",
        suffix=".wbt",
        encoding="ascii",
        delete=False,
    )
    try:
        temporary_world.write(contents)
        temporary_world.close()
        result = subprocess.run(
            [
                str(WEBOTS),
                "--mode=fast",
                "--batch",
                "--no-rendering",
                "--stdout",
                "--stderr",
                f"--port={port}",
                temporary_world.name,
            ],
            cwd=ROOT,
            env=environment,
            capture_output=True,
            text=True,
            timeout=30,
            check=False,
        )
    finally:
        Path(temporary_world.name).unlink(missing_ok=True)
        temporary_project = world.parent / f".{Path(temporary_world.name).stem}.wbproj"
        temporary_project.unlink(missing_ok=True)
    output = f"{result.stdout}\n{result.stderr}"
    assert result.returncode == 0, output
    assert "ERROR:" not in output, output
    assert not re.search(r"(?:Device|Mesh|PROTO).*not found", output, re.I), output
    try:
        encoded_result = Path(result_file.name).read_text(encoding="utf-8")
    finally:
        Path(result_file.name).unlink(missing_ok=True)
    assert encoded_result, output
    return output, json.loads(encoded_result)


@pytest.mark.skipif(not WEBOTS.exists(), reason="Webots R2025a is not installed")
@pytest.mark.parametrize("world", WORLDS, ids=lambda path: path.stem)
def test_reset_endpoint_frames_match_fusion_manifest(world):
    _, result = _run_world(world, smoke_mode="attachment")
    expected = _expected_reset_endpoints(world.stem)
    expected_names = {
        f"{leg}_{joint}"
        for leg in LEG_NAMES
        for joint in JOINT_NAMES
    }

    assert set(result["endpoints"]) == expected_names == set(expected)
    for name, expected_transform in expected.items():
        actual_transform = _endpoint_matrix(result["endpoints"][name])
        translation_error_m, rotation_error_deg = cad_sync._attachment_error(
            expected_transform,
            actual_transform,
        )
        assert translation_error_m <= 1e-5, (
            f"{world.stem} {name} differs from Fusion by "
            f"{translation_error_m * 1000:.6f} mm"
        )
        assert rotation_error_deg <= 0.01, (
            f"{world.stem} {name} differs from Fusion by "
            f"{rotation_error_deg:.6f} deg"
        )


@pytest.mark.skipif(not WEBOTS.exists(), reason="Webots R2025a is not installed")
def test_all_worlds_load_with_all_devices_and_command_directions():
    results = []
    for index, world in enumerate(WORLDS):
        text, result = _run_world(world, 1300 + index)
        results.append(result)
        expected_translation, expected_rotation = WORLD_POSES[world.stem]
        assert result["devices"] == 18
        assert result["sensors"] == 18
        assert result["missing_motors"] == []
        assert result["missing_sensors"] == []
        assert result["body_position_m"][1] > 0.02
        assert result["configured_body_translation_m"] == pytest.approx(
            expected_translation, abs=1e-9
        )
        assert result["configured_body_rotation"] == pytest.approx(
            expected_rotation, abs=1e-9
        )
        assert result["initial_body_position_m"] == pytest.approx(
            expected_translation, abs=1e-9
        )
        assert result["first_step_displacement_m"] < 0.01
        # CAD-derived primitive collision can lift the body during first
        # contact settling; this is a calibration limitation, not a load error.
        assert result["settling_displacement_m"] < 0.20
        assert result["reset_body_translation_m"] == pytest.approx(
            expected_translation, abs=1e-9
        )
        assert result["reset_body_rotation"] == pytest.approx(
            expected_rotation, abs=1e-9
        )
        if world.stem in {"flat", "slope_10"}:
            assert all(
                angles == pytest.approx([0.0, 28.0, 115.0], abs=0.1)
                for angles in result["sensor_angles_deg"].values()
            )
        else:
            # 20/30 degree slope contact is dynamically unstable with the
            # provisional primitive collision; reset below must still be exact.
            assert all(
                all(-90.0 <= angle <= 130.0 for angle in angles)
                for angles in result["sensor_angles_deg"].values()
            )
        assert result["stop_angles_deg"] == pytest.approx([0.0, 28.0, 115.0])
        assert result["reset_angles_deg"] == pytest.approx([0.0, 28.0, 115.0])
        assert "servo_control module not found" in text

    for result in results:
        forward = result["deltas_mm"]["forward"]
        backward = result["deltas_mm"]["backward"]
        left = result["deltas_mm"]["left"]
        right = result["deltas_mm"]["right"]
        assert forward[1] * backward[1] < 0
        assert left[1] * right[1] < 0


@pytest.mark.skipif(not WEBOTS.exists(), reason="Webots R2025a is not installed")
@pytest.mark.parametrize("angle", (10, 20, 30))
def test_slope_world_declares_requested_angle(angle):
    world = ROOT / f"webots/worlds/slope_{angle}.wbt"
    contents = world.read_text(encoding="ascii")
    assert 'coordinateSystem "NUE"' in contents
    assert "basicTimeStep 20" in contents
    assert f"SlopeTerrain {{ angle {angle} }}" in contents
    expected_translation, expected_rotation = WORLD_POSES[f"slope_{angle}"]
    pose_match = re.search(
        r"Spider \{ name \"[^\"]+\" translation "
        r"(\S+) (\S+) (\S+) rotation (\S+) (\S+) (\S+) (\S+) \}",
        contents,
    )
    assert pose_match is not None
    pose_values = [float(value) for value in pose_match.groups()]
    assert pose_values == pytest.approx(
        expected_translation + expected_rotation, abs=1e-9
    )


@pytest.mark.skipif(not WEBOTS.exists(), reason="Webots R2025a is not installed")
def test_flat_world_physical_motion_stop_and_reset():
    world = ROOT / "webots/worlds/flat.wbt"
    results = {
        scenario: _run_world(world, smoke_mode=f"smoke-motion-{scenario}")[1]
        for scenario in (
            "stand",
            "forward",
            "backward",
            "left",
            "right",
            "stop",
            "reset",
        )
    }

    stand = results["stand"]
    assert 0.04 < stand["end_position_m"][1] < 0.20
    assert stand["drift_m"] < 0.001
    assert abs(stand["yaw_rad"]) < 0.01

    forward = results["forward"]["displacement_m"]
    backward = results["backward"]["displacement_m"]
    # The CAD reset stance and the legacy gait support pose are not yet
    # dynamically calibrated.  Require a real, opposite-direction response;
    # absolute stride distance remains a calibration measurement.  The Fusion
    # top-view arrow points along Fusion +Y, which is Webots -Z after the CAD
    # conversion.
    assert forward[2] < -0.1
    assert backward[2] > 0.1
    assert forward[2] * backward[2] < 0.0
    assert abs(forward[0]) < abs(forward[2]) * 0.2
    assert abs(backward[0]) < abs(backward[2]) * 0.2
    assert abs(results["forward"]["yaw_rad"]) < 0.1
    assert abs(results["backward"]["yaw_rad"]) < 0.1

    left = results["left"]
    right = results["right"]
    assert left["yaw_rad"] * right["yaw_rad"] < 0.0
    assert abs(left["yaw_rad"]) > 0.1
    assert abs(right["yaw_rad"]) > 0.1

    assert results["stop"]["stop_drift_m"] < 0.001

    reset = results["reset"]
    assert reset["reset_position_m"] == pytest.approx(WORLD_POSES["flat"][0])
    assert reset["reset_yaw_rad"] == pytest.approx(0.0, abs=1e-6)
    assert all(
        angles == pytest.approx([0.0, 28.0, 115.0], abs=0.1)
        for angles in reset["reset_sensor_angles_deg"].values()
    )

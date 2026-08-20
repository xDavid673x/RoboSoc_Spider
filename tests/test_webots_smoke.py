"""Headless Webots smoke checks for the four v1 validation worlds."""

from __future__ import annotations

import json
import os
from pathlib import Path
import re
import subprocess
import tempfile

import pytest


ROOT = Path(__file__).resolve().parents[1]
WEBOTS = Path("/Applications/Webots.app/Contents/MacOS/webots")
WORLDS = (
    ROOT / "webots/worlds/flat.wbt",
    ROOT / "webots/worlds/slope_10.wbt",
    ROOT / "webots/worlds/slope_20.wbt",
    ROOT / "webots/worlds/slope_30.wbt",
)


def _run_world(world: Path, port: int) -> tuple[str, dict[str, object]]:
    environment = os.environ.copy()
    result_file = tempfile.NamedTemporaryFile(suffix=".json", delete=False)
    result_file.close()
    contents = world.read_text(encoding="ascii")
    contents = contents.replace(
        "Spider { name ",
        f'Spider {{ customData "smoke:{result_file.name}" name ',
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
def test_all_worlds_load_with_all_devices_and_command_directions():
    results = []
    for index, world in enumerate(WORLDS):
        text, result = _run_world(world, 1300 + index)
        results.append(result)
        assert result["devices"] == 18
        assert result["sensors"] == 18
        assert result["stop_angles_deg"] == pytest.approx([0.0, 28.0, 115.0])
        assert result["reset_angles_deg"] == pytest.approx([0.0, 28.0, 115.0])
        assert "servo_control module not found" in text

    for result in results:
        forward = result["deltas_mm"]["forward"]
        backward = result["deltas_mm"]["backward"]
        left = result["deltas_mm"]["left"]
        right = result["deltas_mm"]["right"]
        assert forward[1] * backward[1] < 0
        assert left[0] * right[0] < 0


@pytest.mark.skipif(not WEBOTS.exists(), reason="Webots R2025a is not installed")
@pytest.mark.parametrize("angle", (10, 20, 30))
def test_slope_world_declares_requested_angle(angle):
    world = ROOT / f"webots/worlds/slope_{angle}.wbt"
    contents = world.read_text(encoding="ascii")
    assert "basicTimeStep 20" in contents
    assert f"SlopeTerrain {{ angle {angle} " in contents

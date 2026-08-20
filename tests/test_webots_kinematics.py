import math
import os
import subprocess
import sys
from pathlib import Path

import pytest

from webots.controllers.spider_controller.kinematics_adapter import (
    INIT_ANGLES_DEG,
    JOINT_LIMITS_DEG,
    LEG_NAMES,
    TRIPOD_A,
    TRIPOD_B,
    WEBOTS_GAIT_COMPENSATION_RAD,
    Command,
    VirtualSpider,
    body_mount_to_webots,
    clamp_joint_angles,
    deg_to_rad,
    gait_step_for_speed,
    joint_angles_to_webots,
    mm_to_m,
)


def test_length_and_angle_conversions_preserve_boundary_units():
    assert mm_to_m(0) == pytest.approx(0.0)
    assert mm_to_m(250.0) == pytest.approx(0.25)
    assert deg_to_rad(180.0) == pytest.approx(math.pi)
    assert body_mount_to_webots((50.0, -40.0, 0.0)) == pytest.approx(
        (0.05, -0.04, 0.0)
    )


def test_joint_angle_conversion_clamps_each_command_to_mechanical_limits():
    assert clamp_joint_angles((-120.0, 15.0, 200.0)) == [-90.0, 15.0, 130.0]
    assert joint_angles_to_webots((-120.0, 15.0, 200.0)) == pytest.approx(
        [math.radians(-90.0), math.radians(15.0), math.radians(130.0)]
    )


def test_joint_angle_conversion_rejects_wrong_joint_count():
    with pytest.raises(ValueError, match="exactly three"):
        clamp_joint_angles((0.0, 1.0))


def test_command_normalization_applies_defaults_case_and_numeric_clamps():
    command = Command.from_mapping(
        {
            "mode": "WALK",
            "vx": 2,
            "vy": "-2",
            "turn": 0.25,
            "speed": 4,
            "height": -4,
        }
    )

    assert command == Command(
        mode="walk",
        vx=1.0,
        vy=-1.0,
        turn=0.25,
        speed=1.0,
        height=-1.0,
    )


def test_command_normalization_uses_stand_for_missing_or_unknown_mode():
    assert Command.from_mapping(None) == Command(mode="stand")
    assert Command.from_mapping({"mode": "teleport"}).mode == "stand"
    assert Command.stop() == Command(mode="stop")


def test_tripod_constants_partition_all_six_leg_names():
    assert set(TRIPOD_A).isdisjoint(TRIPOD_B)
    assert TRIPOD_A | TRIPOD_B == set(LEG_NAMES)
    assert TRIPOD_A == {"legi", "legk", "legm"}
    assert TRIPOD_B == {"legj", "legl", "legn"}


def test_webots_mount_compensation_and_gait_rate_match_the_body_frame():
    spider = VirtualSpider()

    assert spider.gait.anti_beta_dict == pytest.approx(
        {
            "legi": 0.0,
            "legj": math.pi / 4.0,
            "legk": 3.0 * math.pi / 4.0,
            "legl": math.pi,
            "legm": -3.0 * math.pi / 4.0,
            "legn": -math.pi / 4.0,
        }
    )
    assert WEBOTS_GAIT_COMPENSATION_RAD == spider.gait.anti_beta_dict
    assert gait_step_for_speed(0.0) == 48
    assert gait_step_for_speed(0.7) == 28
    assert gait_step_for_speed(1.0) == 20


def test_virtual_spider_creates_six_named_legs_with_three_joints_each():
    spider = VirtualSpider()

    assert tuple(spider.legs) == LEG_NAMES
    assert set(spider.joint_angles_deg()) == set(LEG_NAMES)
    assert all(len(angles) == 3 for angles in spider.joint_angles_deg().values())
    assert set(spider.joint_angles_rad()) == set(LEG_NAMES)
    assert set(spider.tip_positions_mm()) == set(LEG_NAMES)


def test_virtual_spider_reset_restores_init_angles_and_gait_state():
    spider = VirtualSpider()
    spider.command({"mode": "walk", "vx": 1.0, "speed": 0.5})
    spider.command({"mode": "turn", "turn": 1.0, "speed": 0.5})

    spider.reset()

    assert spider.last_mode == "init"
    assert spider.last_command == Command(mode="init")
    assert spider.gait.walk_t == 0
    assert spider.gait.walk_tripod_idx == 0
    assert spider.gait.turn_t == 0
    assert spider.gait.turn_tripod_idx == 0
    for angles in spider.joint_angles_deg().values():
        assert angles == pytest.approx(INIT_ANGLES_DEG)


def test_virtual_spider_positive_and_negative_turns_mirror_tangential_motion():
    positive = VirtualSpider()
    negative = VirtualSpider()

    positive.command({"mode": "turn", "turn": 1.0, "speed": 0.5})
    negative.command({"mode": "turn", "turn": -1.0, "speed": 0.5})

    positive_tip = positive.tip_positions_mm()["legi"]
    negative_tip = negative.tip_positions_mm()["legi"]
    assert positive_tip[0] == pytest.approx(negative_tip[0])
    assert positive_tip[1] == pytest.approx(-negative_tip[1])
    assert positive_tip[1] > 0.0
    assert positive_tip[2] == pytest.approx(negative_tip[2])


def test_adapter_imports_without_servo2040_hardware_module():
    adapter_path = (
        Path(__file__).resolve().parents[1]
        / "webots"
        / "controllers"
        / "spider_controller"
    )
    env = os.environ.copy()
    env["PYTHONPATH"] = str(adapter_path)
    probe = (
        "import sys; sys.modules['servo_control'] = None; "
        "import kinematics_adapter; "
        "assert kinematics_adapter.VirtualSpider"
    )

    result = subprocess.run(
        [sys.executable, "-c", probe],
        cwd=adapter_path,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr

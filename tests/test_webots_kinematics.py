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
    MAX_WALK_SPEED,
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
    load_cad_manifest,
    mm_to_m,
    turn_gait_compensation_rad,
)
from servo2040_receiver.legs_IK import SERVO_OFFSETS


def _ik_delta_to_body(leg_manifest, delta_ik):
    # SpiderLeg uses X/Y as its horizontal plane and Z as vertical;
    # Webots' leg frame uses X/Z horizontally and Y vertically.
    delta_leg = [delta_ik[0], delta_ik[2], -delta_ik[1]]
    transform = leg_manifest["leg_to_body_transform_mm"]
    return [
        sum(
            transform[row * 4 + column] * delta_leg[column]
            for column in range(3)
        )
        for row in range(3)
    ]


def test_length_and_angle_conversions_preserve_boundary_units():
    assert mm_to_m(0) == pytest.approx(0.0)
    assert mm_to_m(250.0) == pytest.approx(0.25)
    assert deg_to_rad(180.0) == pytest.approx(math.pi)
    assert body_mount_to_webots((50.0, -40.0, 0.0)) == pytest.approx(
        (0.05, -0.04, 0.0)
    )


@pytest.mark.parametrize(
    "pose_deg",
    (
        (0.0, 28.0, 115.0),
        (20.0, -10.0, 80.0),
        (-35.0, 45.0, 120.0),
    ),
)
def test_reachable_inverse_and_forward_kinematics_round_trip(pose_deg):
    leg = VirtualSpider().legs["legi"]
    leg.set_angles(list(pose_deg))
    target_mm = list(leg.forwardKinematics()[3])

    inverse_angles_deg = leg.calculate_inverse_angles(target_mm)
    leg.set_angles(inverse_angles_deg)

    assert leg.forwardKinematics()[3] == pytest.approx(target_mm, abs=1e-9)


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


def test_walk_and_turn_gaits_alternate_tripods_each_phase():
    walk_spider = VirtualSpider()
    walk_phase_steps = gait_step_for_speed(MAX_WALK_SPEED) + 1
    assert walk_spider.gait.tripods[walk_spider.gait.walk_tripod_idx] == TRIPOD_A

    for _ in range(walk_phase_steps):
        walk_spider.command({"mode": "walk", "vx": 1.0, "speed": 1.0})
    assert walk_spider.gait.tripods[walk_spider.gait.walk_tripod_idx] == TRIPOD_B

    for _ in range(walk_phase_steps):
        walk_spider.command({"mode": "walk", "vx": 1.0, "speed": 1.0})
    assert walk_spider.gait.tripods[walk_spider.gait.walk_tripod_idx] == TRIPOD_A

    turn_spider = VirtualSpider()
    turn_phase_steps = gait_step_for_speed(1.0) + 1
    for _ in range(turn_phase_steps):
        turn_spider.command({"mode": "turn", "turn": 1.0, "speed": 1.0})
    assert turn_spider.gait.tripods[turn_spider.gait.turn_tripod_idx] == TRIPOD_B

    for _ in range(turn_phase_steps):
        turn_spider.command({"mode": "turn", "turn": 1.0, "speed": 1.0})
    assert turn_spider.gait.tripods[turn_spider.gait.turn_tripod_idx] == TRIPOD_A


@pytest.mark.parametrize(
    ("command", "cycle_steps"),
    (
        (
            {"mode": "walk", "vx": 1.0, "speed": 1.0},
            2 * (gait_step_for_speed(MAX_WALK_SPEED) + 1),
        ),
        (
            {"mode": "turn", "turn": 1.0, "speed": 1.0},
            2 * (gait_step_for_speed(1.0) + 1),
        ),
        (
            {"mode": "turn", "turn": -1.0, "speed": 1.0},
            2 * (gait_step_for_speed(1.0) + 1),
        ),
    ),
)
def test_full_gait_cycles_keep_internal_joint_state_within_limits(
    command, cycle_steps
):
    spider = VirtualSpider()

    for _ in range(cycle_steps):
        spider.command(command)
        reported_angles = spider.joint_angles_deg()
        for name, leg in spider.legs.items():
            raw_command_angles = [
                angle - SERVO_OFFSETS[index]
                for index, angle in enumerate(leg.get_angles())
            ]
            assert raw_command_angles == pytest.approx(reported_angles[name])
            for angle, (lower, upper) in zip(
                raw_command_angles, JOINT_LIMITS_DEG
            ):
                assert lower <= angle <= upper


def test_webots_mount_compensation_and_gait_rate_match_the_body_frame():
    spider = VirtualSpider()

    assert spider.gait.anti_beta_dict == pytest.approx(
        {
            "legi": 0.0,
            "legj": -math.pi / 3.0,
            "legk": -2.0 * math.pi / 3.0,
            "legl": math.pi,
            "legm": 2.0 * math.pi / 3.0,
            "legn": math.pi / 3.0,
        }
    )
    assert WEBOTS_GAIT_COMPENSATION_RAD == spider.gait.anti_beta_dict
    assert gait_step_for_speed(0.0) == 48
    assert gait_step_for_speed(0.7) == 28
    assert gait_step_for_speed(1.0) == 20


def test_all_six_leg_strides_align_with_webots_negative_z():
    spider = VirtualSpider()
    manifest = load_cad_manifest()
    assert manifest is not None
    legs_by_name = {leg["name"]: leg for leg in manifest["legs"]}
    legs = list(spider.legs.values())
    targets_start = spider.gait.calculate_gait_targets(
        legs,
        TRIPOD_A,
        [-50.0, -125.0],
        [50.0, -125.0],
        -125.0,
        30.0,
        20,
        0,
        0.0,
        130.0,
    )
    targets_mid = spider.gait.calculate_gait_targets(
        legs,
        TRIPOD_A,
        [-50.0, -125.0],
        [50.0, -125.0],
        -125.0,
        30.0,
        20,
        10,
        0.0,
        130.0,
    )
    targets_end = spider.gait.calculate_gait_targets(
        legs,
        TRIPOD_A,
        [-50.0, -125.0],
        [50.0, -125.0],
        -125.0,
        30.0,
        20,
        20,
        0.0,
        130.0,
    )

    for name in LEG_NAMES:
        delta_ik = [
            targets_end[name][index] - targets_start[name][index]
            for index in range(3)
        ]
        delta_body = _ik_delta_to_body(legs_by_name[name], delta_ik)
        expected_z = -100.0 if name in TRIPOD_A else 100.0
        assert delta_body == pytest.approx([0.0, 0.0, expected_z], abs=1e-6)
        if name in TRIPOD_A:
            assert targets_mid[name][2] > targets_start[name][2]
        else:
            assert targets_mid[name][2] == pytest.approx(targets_start[name][2])


@pytest.mark.parametrize("turn", (-1.0, 1.0))
def test_all_six_turn_strides_are_tangent_to_the_body(turn):
    spider = VirtualSpider()
    manifest = load_cad_manifest()
    assert manifest is not None
    legs_by_name = {leg["name"]: leg for leg in manifest["legs"]}
    legs = list(spider.legs.values())
    spider.gait.anti_beta_dict = {
        name: turn_gait_compensation_rad(turn) for name in LEG_NAMES
    }
    path_angle_deg = -(10.0 + 30.0 * abs(turn)) * turn
    targets_start = spider.gait.calculate_gait_targets(
        legs,
        TRIPOD_A,
        [-50.0, -125.0],
        [50.0, -125.0],
        -125.0,
        30.0,
        20,
        0,
        path_angle_deg,
        130.0,
    )
    targets_end = spider.gait.calculate_gait_targets(
        legs,
        TRIPOD_A,
        [-50.0, -125.0],
        [50.0, -125.0],
        -125.0,
        30.0,
        20,
        20,
        path_angle_deg,
        130.0,
    )

    for name in LEG_NAMES:
        delta_ik = [
            targets_end[name][index] - targets_start[name][index]
            for index in range(3)
        ]
        delta_body = _ik_delta_to_body(legs_by_name[name], delta_ik)
        radial = legs_by_name[name]["frame"]["x_axis"]
        direction = 1.0 if turn < 0.0 else -1.0
        if name in TRIPOD_B:
            direction *= -1.0
        expected = [
            direction * 100.0 * radial[2],
            0.0,
            direction * -100.0 * radial[0],
        ]
        assert delta_body == pytest.approx(expected, abs=1e-6)


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


def test_virtual_gait_never_calls_importable_hardware_batch_hooks():
    adapter_path = (
        Path(__file__).resolve().parents[1]
        / "webots"
        / "controllers"
        / "spider_controller"
    )
    env = os.environ.copy()
    env["PYTHONPATH"] = str(adapter_path)
    probe = """
import sys
import types

calls = []
servo_control = types.ModuleType("servo_control")
servo_control.begin_batch = lambda: calls.append("begin")
servo_control.end_batch = lambda: calls.append("end")
sys.modules["servo_control"] = servo_control

import kinematics_adapter

spider = kinematics_adapter.VirtualSpider()
spider.command({"mode": "walk", "vx": 1.0, "speed": 1.0})
assert spider.gait.use_hardware_batch is False
assert calls == []

from tripot_gait import Tripot_gait

class FakeLeg:
    name = "legi"

    def calculate_inverse_angles(self, target):
        return [0.0, 0.0, 0.0]

    def set_angles(self, angles):
        return angles

    def forwardKinematics(self):
        return [[0.0, 0.0, 0.0]] * 4

hardware_gait = Tripot_gait()
hardware_gait._apply_targets([FakeLeg()], {"legi": [0.0, 0.0, 0.0]})
assert hardware_gait.use_hardware_batch is True
assert calls == ["begin", "end"]
"""

    result = subprocess.run(
        [sys.executable, "-c", probe],
        cwd=adapter_path,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr

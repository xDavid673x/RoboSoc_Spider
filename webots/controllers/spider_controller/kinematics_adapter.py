"""Webots-facing adapter for the repository's millimetre/degree gait math.

The existing gait and IK modules are tuned around millimetres and degrees. This
module keeps that convention intact and converts only at the Webots boundary,
where Webots expects metres and radians.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping


CONTROLLER_DIR = Path(__file__).resolve().parent
REPO_ROOT = CONTROLLER_DIR.parents[2]
SERVO_DIR = REPO_ROOT / "servo2040_receiver"
if str(SERVO_DIR) not in sys.path:
    sys.path.insert(0, str(SERVO_DIR))

from legs_IK import SERVO_OFFSETS, SpiderLeg  # noqa: E402
from tripot_gait import Tripot_gait  # noqa: E402


LEG_NAMES = ("legi", "legj", "legk", "legl", "legm", "legn")
TRIPOD_A = frozenset(("legi", "legk", "legm"))
TRIPOD_B = frozenset(("legj", "legl", "legn"))
TRIPODS = (TRIPOD_A, TRIPOD_B)

LEG_MOUNT_MM: dict[str, tuple[float, float, float]] = {
    "legi": (50.0, 0.0, 0.0),
    "legj": (50.0, 40.0, 0.0),
    "legk": (-50.0, 40.0, 0.0),
    "legl": (-50.0, 0.0, 0.0),
    "legm": (-50.0, -40.0, 0.0),
    "legn": (50.0, -40.0, 0.0),
}

# ``Tripot_gait``'s simulation branch assumes its matplotlib body frame.  The
# Webots PROTO applies its own reflected Y-up mount rotations, so these inverse
# mount angles keep every stance-foot stride parallel in the Webots body frame.
WEBOTS_GAIT_COMPENSATION_RAD: dict[str, float] = {
    "legi": 0.0,
    "legj": math.pi / 4.0,
    "legk": 3.0 * math.pi / 4.0,
    "legl": math.pi,
    "legm": -3.0 * math.pi / 4.0,
    "legn": -math.pi / 4.0,
}

JOINT_LIMITS_DEG: tuple[tuple[float, float], ...] = (
    (-90.0, 90.0),
    (-90.0, 90.0),
    (0.0, 130.0),
)

DEFAULT_LEG_LENGTHS_MM = (43.8, 88.0, 166.0)
INIT_ANGLES_DEG = (0.0, 28.0, 115.0)
MM_TO_M = 0.001
MAX_WALK_SPEED = 0.7
GAIT_STEP_MIN = 20
GAIT_STEP_MAX = 48


def mm_to_m(value_mm: float) -> float:
    """Convert one millimetre value to metres."""

    return float(value_mm) * MM_TO_M


def deg_to_rad(value_deg: float) -> float:
    """Convert one degree value to radians."""

    return math.radians(float(value_deg))


def clamp(value: float, lower: float, upper: float) -> float:
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        numeric = 0.0
    return max(lower, min(upper, numeric))


def clamp_joint_angles(angles_deg: Iterable[float]) -> list[float]:
    """Clamp command-space angles to the confirmed mechanical limits."""

    values = list(angles_deg)
    if len(values) != len(JOINT_LIMITS_DEG):
        raise ValueError("a spider leg must have exactly three joint angles")
    return [
        clamp(value, lower, upper)
        for value, (lower, upper) in zip(values, JOINT_LIMITS_DEG)
    ]


def gait_step_for_speed(speed: float) -> int:
    normalized = clamp(speed, 0.0, 1.0)
    step = round(GAIT_STEP_MAX - (GAIT_STEP_MAX - GAIT_STEP_MIN) * normalized)
    return max(GAIT_STEP_MIN, min(GAIT_STEP_MAX, int(step)))


def joint_angles_to_webots(angles_deg: Iterable[float]) -> list[float]:
    """Convert command-space degrees to Webots radians after limiting."""

    return [deg_to_rad(value) for value in clamp_joint_angles(angles_deg)]


def body_mount_to_webots(mount_mm: Iterable[float]) -> tuple[float, float, float]:
    """Convert a body mount translation from millimetres to metres."""

    values = tuple(float(value) for value in mount_mm)
    if len(values) != 3:
        raise ValueError("a mount translation must have three coordinates")
    return tuple(mm_to_m(value) for value in values)


@dataclass(frozen=True)
class Command:
    """Normalized command shape shared with the existing web controller."""

    mode: str = "stand"
    vx: float = 0.0
    vy: float = 0.0
    turn: float = 0.0
    speed: float = 0.0
    height: float = 0.0

    @classmethod
    def from_mapping(cls, payload: Mapping[str, Any] | None) -> "Command":
        payload = payload or {}
        mode = str(payload.get("mode", "stand")).lower()
        if mode not in {"init", "stand", "stop", "walk", "turn", "stance"}:
            mode = "stand"
        return cls(
            mode=mode,
            vx=clamp(payload.get("vx", 0.0), -1.0, 1.0),
            vy=clamp(payload.get("vy", 0.0), -1.0, 1.0),
            turn=clamp(payload.get("turn", 0.0), -1.0, 1.0),
            speed=clamp(payload.get("speed", 0.0), 0.0, 1.0),
            height=clamp(payload.get("height", 0.0), -1.0, 1.0),
        )

    @classmethod
    def stop(cls) -> "Command":
        return cls(mode="stop")


class VirtualSpider:
    """Pure-python gait state used by the Webots controller and unit tests."""

    def __init__(self, leg_lengths_mm: Iterable[float] = DEFAULT_LEG_LENGTHS_MM):
        lengths = tuple(float(value) for value in leg_lengths_mm)
        if len(lengths) != 3 or any(value <= 0 for value in lengths):
            raise ValueError("leg_lengths_mm must contain three positive values")
        self.legs = {
            name: SpiderLeg(name, *lengths)
            for name in LEG_NAMES
        }
        self.gait = Tripot_gait()
        self.gait.anti_beta_dict = WEBOTS_GAIT_COMPENSATION_RAD.copy()
        self.last_mode = "init"
        self.last_command = Command(mode="init")
        self.reset()

    def reset(self) -> None:
        """Restore gait phases, pose, and virtual joint state."""

        self.gait.anti_beta_dict = WEBOTS_GAIT_COMPENSATION_RAD.copy()
        self.gait.reset_walk_phase(0)
        self.gait.reset_turn_phase(0)
        for leg in self.legs.values():
            leg.set_angles(list(INIT_ANGLES_DEG))
            leg.forwardKinematics()
        self.last_mode = "init"
        self.last_command = Command(mode="init")

    @staticmethod
    def _command_angles_deg(leg: SpiderLeg) -> list[float]:
        return [
            angle - SERVO_OFFSETS[index]
            for index, angle in enumerate(leg.get_angles())
        ]

    def _enforce_joint_limits(self) -> None:
        """Keep virtual leg state and Webots motor targets on the same pose."""

        for leg in self.legs.values():
            limited_angles = clamp_joint_angles(self._command_angles_deg(leg))
            leg.set_angles(limited_angles)
            leg.forwardKinematics()

    def command(self, payload: Mapping[str, Any] | Command | None) -> None:
        command = payload if isinstance(payload, Command) else Command.from_mapping(payload)
        if command.mode == "init":
            self.reset()
            self.last_command = command
            return

        if command.mode in {"stand", "stop", "stance"}:
            self.gait.reset_walk_phase(0)
            self.gait.reset_turn_phase(0)
            for leg in self.legs.values():
                leg.set_angles(list(INIT_ANGLES_DEG))
                leg.forwardKinematics()
            self.last_mode = "stand"
            self.last_command = command
            return

        if command.mode == "turn":
            turn_angle = (10.0 + 30.0 * abs(command.turn)) * command.turn
            compensation = (
                math.pi - math.radians(turn_angle)
                if command.turn > 0.0
                else -math.radians(turn_angle)
            )
            self.gait.anti_beta_dict = {
                name: compensation for name in LEG_NAMES
            }
            if self.last_mode != "turn":
                self.gait.reset_turn_phase()
            self.gait.turn_step(
                list(self.legs.values()),
                turn_ratio=command.turn,
                max_angle=abs(turn_angle),
                T=80.0 + 70.0 * command.speed,
                body_height=-125.0 + 45.0 * command.height,
                A=20.0 + 20.0 * command.speed,
                step=gait_step_for_speed(command.speed),
                xpos=130.0 + 20.0 * command.height,
            )
            self._enforce_joint_limits()
            self.last_mode = "turn"
            self.last_command = command
            return

        if command.mode == "walk":
            self.gait.anti_beta_dict = WEBOTS_GAIT_COMPENSATION_RAD.copy()
            magnitude = math.hypot(command.vx, command.vy)
            if magnitude >= 0.03:
                walk_speed = min(command.speed, MAX_WALK_SPEED)
                if self.last_mode != "walk":
                    self.gait.reset_walk_phase()
                self.gait.walk_step(
                    list(self.legs.values()),
                    angle=math.degrees(math.atan2(command.vy, command.vx)),
                    T=max(40.0, (80.0 + 130.0 * walk_speed) * min(1.0, magnitude)),
                    body_height=-125.0 + 45.0 * command.height,
                    A=15.0 + 20.0 * walk_speed,
                    step=gait_step_for_speed(walk_speed),
                    xpos=130.0 + 20.0 * command.height,
                )
                self._enforce_joint_limits()
            self.last_mode = "walk"
            self.last_command = command
            return

        raise ValueError(f"unsupported command mode: {command.mode}")

    def joint_angles_deg(self) -> dict[str, list[float]]:
        return {
            name: clamp_joint_angles(self._command_angles_deg(leg))
            for name, leg in self.legs.items()
        }

    def joint_angles_rad(self) -> dict[str, list[float]]:
        return {
            name: joint_angles_to_webots(angles)
            for name, angles in self.joint_angles_deg().items()
        }

    def tip_positions_mm(self) -> dict[str, list[float]]:
        return {
            name: list(leg.forwardKinematics()[3])
            for name, leg in self.legs.items()
        }

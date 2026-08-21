"""Webots-facing adapter for the repository's millimetre/degree gait math.

The existing gait and IK modules are tuned around millimetres and degrees. This
module keeps that convention intact and converts only at the Webots boundary,
where Webots expects metres and radians.
"""

from __future__ import annotations

import math
import json
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

JOINT_LIMITS_DEG: tuple[tuple[float, float], ...] = (
    (-90.0, 90.0),
    (-90.0, 90.0),
    (0.0, 130.0),
)

DEFAULT_LEG_LENGTHS_MM = (43.8, 88.0, 166.0)
INIT_ANGLES_DEG = (0.0, 28.0, 115.0)
CAD_MANIFEST = REPO_ROOT / "webots" / "cad" / "spider_geometry.v1.json"
MM_TO_M = 0.001
MAX_WALK_SPEED = 0.7
GAIT_STEP_MIN = 20
GAIT_STEP_MAX = 48


def _fallback_leg_lengths() -> dict[str, tuple[float, float, float]]:
    return {name: DEFAULT_LEG_LENGTHS_MM for name in LEG_NAMES}


def _fallback_gait_compensation() -> dict[str, float]:
    return {
        "legi": 0.0,
        "legj": -math.pi / 4.0,
        "legk": -3.0 * math.pi / 4.0,
        "legl": -math.pi,
        "legm": 3.0 * math.pi / 4.0,
        "legn": math.pi / 4.0,
    }


def load_cad_manifest(path: Path = CAD_MANIFEST) -> dict[str, Any] | None:
    """Load the committed CAD manifest when available.

    Webots launches controllers from inside the controller directory, so the
    path is resolved from this file instead of the process working directory.
    Unit tests can still import the adapter before a CAD snapshot exists.
    """

    try:
        manifest = json.loads(path.read_text(encoding="ascii"))
    except FileNotFoundError:
        return None
    if manifest.get("schema_version") != 1:
        raise ValueError(f"unsupported spider CAD manifest schema: {path}")
    return manifest


def _cad_leg_lengths() -> dict[str, tuple[float, float, float]]:
    manifest = load_cad_manifest()
    if manifest is None:
        return _fallback_leg_lengths()
    lengths: dict[str, tuple[float, float, float]] = {}
    for leg in manifest["legs"]:
        values = leg["lengths_mm"]
        lengths[leg["name"]] = (
            float(values["coxa"]),
            float(values["femur"]),
            float(values["tibia"]),
        )
    return lengths


def _cad_gait_compensation() -> dict[str, float]:
    manifest = load_cad_manifest()
    if manifest is None:
        return _fallback_gait_compensation()
    return {
        leg["name"]: float(leg["gait_compensation_rad"])
        for leg in manifest["legs"]
    }


CAD_LEG_LENGTHS_MM = _cad_leg_lengths()
WEBOTS_GAIT_COMPENSATION_RAD = _cad_gait_compensation()


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


def turn_gait_compensation_rad(turn: float) -> float:
    """Align the legacy gait sweep with an in-place body turn.

    ``turn_step`` rotates its path by the negative public turn angle.  Adding
    that signed angle here cancels the path rotation for a left turn; a
    further half-turn reverses every foot sweep for a right turn.  With the
    radial CAD leg frames, those local sweeps become body-frame tangents.
    """

    normalized = clamp(turn, -1.0, 1.0)
    turn_angle = math.radians((10.0 + 30.0 * abs(normalized)) * normalized)
    return turn_angle + (math.pi if normalized > 0.0 else 0.0)


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

    def __init__(
        self,
        leg_lengths_mm: Iterable[float] | Mapping[str, Iterable[float]] | None = None,
    ):
        if leg_lengths_mm is None:
            per_leg_lengths = CAD_LEG_LENGTHS_MM
        elif isinstance(leg_lengths_mm, Mapping):
            per_leg_lengths = {
                name: tuple(float(value) for value in leg_lengths_mm[name])
                for name in LEG_NAMES
            }
        else:
            lengths = tuple(float(value) for value in leg_lengths_mm)
            per_leg_lengths = {name: lengths for name in LEG_NAMES}
        for name, lengths in per_leg_lengths.items():
            if len(lengths) != 3 or any(value <= 0 for value in lengths):
                raise ValueError(f"{name} leg_lengths_mm must contain three positive values")
        self.legs = {
            name: SpiderLeg(name, *per_leg_lengths[name], pin_list=None)
            for name in LEG_NAMES
        }
        self.gait = Tripot_gait(use_hardware_batch=False)
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
            compensation = turn_gait_compensation_rad(command.turn)
            self.gait.anti_beta_dict = {
                name: compensation for name in LEG_NAMES
            }
            if self.last_mode != "turn":
                self.gait.reset_turn_phase()
            self.gait.turn_step(
                list(self.legs.values()),
                turn_ratio=-command.turn,
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

"""Standalone Webots controller for the six-legged Spider model.

The kinematics and gait implementation remains pure Python in
``kinematics_adapter``.  This module only owns the Webots device boundary and
the keyboard-to-command reducer, so it can be imported and tested without
Webots being installed.
"""

from __future__ import annotations

from collections.abc import Iterable
import json
import math
from pathlib import Path
from typing import Any

try:
    from .kinematics_adapter import LEG_NAMES, Command, VirtualSpider
except ImportError:  # Webots executes controller files as standalone scripts.
    from kinematics_adapter import LEG_NAMES, Command, VirtualSpider

try:  # Webots is available only when this file runs as a controller.
    from controller import Keyboard, Supervisor
except ImportError:  # pragma: no cover - exercised by normal Python imports.
    Keyboard = None
    Supervisor = None


TIME_STEP_MS = 20
JOINT_NAMES = ("coxa", "femur", "tibia")
BODY_TRANSLATION = (0.0, 0.133, 0.0)
BODY_ROTATION = (0.0, 1.0, 0.0, 0.0)


def _key_name(key: Any) -> str:
    """Normalize Webots integer key codes and test-friendly string keys."""

    if isinstance(key, str):
        return key.lower()
    if isinstance(key, int) and key >= 0:
        try:
            return chr(key).lower()
        except ValueError:
            return ""
    return ""


def keyboard_command(keys: Iterable[Any]) -> Command:
    """Reduce currently pressed keys to one normalized gait command.

    ``W``/``S`` drive forward/backward and ``A``/``D`` turn left/right.
    Turning has precedence over walking when both are held.  Space holds the
    standing pose and ``R`` requests a full reset.
    """

    pressed = {_key_name(key) for key in keys}
    if "r" in pressed:
        return Command(mode="init")
    if " " in pressed or "space" in pressed:
        return Command(mode="stand")

    turn = float("d" in pressed) - float("a" in pressed)
    if turn:
        return Command(mode="turn", turn=turn, speed=1.0)

    vx = float("w" in pressed) - float("s" in pressed)
    if vx:
        return Command(mode="walk", vx=vx, speed=1.0)
    return Command.stop()


class SpiderController:
    """Webots Supervisor wrapper around :class:`VirtualSpider`."""

    def __init__(self, supervisor: Any | None = None):
        if supervisor is None:
            if Supervisor is None:
                raise RuntimeError("Webots controller API is not available")
            supervisor = Supervisor()
        self.supervisor = supervisor
        self.timestep = TIME_STEP_MS
        self.spider = VirtualSpider()
        self.body = self._get_self_node()
        self.initial_body_translation = self._read_body_field(
            "translation", "getSFVec3f", BODY_TRANSLATION
        )
        self.initial_body_rotation = self._read_body_field(
            "rotation", "getSFRotation", BODY_ROTATION
        )
        self.motors: dict[str, Any] = {}
        self.sensors: dict[str, Any] = {}
        self.joints: dict[str, Any] = {}
        self._lookup_devices()
        self._enable_sensors()
        self.reset()

    def _get_self_node(self) -> Any | None:
        getter = getattr(self.supervisor, "getSelf", None)
        return getter() if getter else None

    def _read_body_field(
        self,
        field_name: str,
        getter_name: str,
        fallback: tuple[float, ...],
    ) -> tuple[float, ...]:
        if self.body is not None:
            get_field = getattr(self.body, "getField", None)
            field = get_field(field_name) if get_field else None
            getter = getattr(field, getter_name, None)
            if getter:
                try:
                    values = tuple(float(value) for value in getter())
                except (TypeError, ValueError):
                    values = ()
                if len(values) == len(fallback) and all(
                    math.isfinite(value) for value in values
                ):
                    return values
        return tuple(fallback)

    def _body_field_pose(self) -> tuple[list[float], list[float]]:
        translation = self._read_body_field(
            "translation", "getSFVec3f", self.initial_body_translation
        )
        rotation = self._read_body_field(
            "rotation", "getSFRotation", self.initial_body_rotation
        )
        return list(translation), list(rotation)

    def _lookup_devices(self) -> None:
        get_device = getattr(self.supervisor, "getDevice", None)
        if get_device is None:
            return
        get_from_device = getattr(self.supervisor, "getFromDevice", None)
        for leg in LEG_NAMES:
            for joint in JOINT_NAMES:
                key = f"{leg}_{joint}"
                motor = get_device(f"{key}_motor")
                self.motors[key] = motor
                self.sensors[key] = get_device(f"{key}_sensor")
                # Webots' Python Device exposes its Supervisor lookup tag here.
                device_tag = getattr(motor, "_tag", None)
                if get_from_device is not None and device_tag is not None:
                    motor_node = get_from_device(device_tag)
                    if motor_node is not None:
                        joint_node = motor_node.getParentNode()
                        if joint_node is not None:
                            self.joints[key] = joint_node

    def _enable_sensors(self) -> None:
        for sensor in self.sensors.values():
            enable = getattr(sensor, "enable", None)
            if enable:
                enable(self.timestep)

    def _reset_body(self) -> None:
        if self.body is None:
            return
        get_field = getattr(self.body, "getField", None)
        if get_field:
            translation = get_field("translation")
            if translation is not None and hasattr(translation, "setSFVec3f"):
                translation.setSFVec3f(list(self.initial_body_translation))
            rotation = get_field("rotation")
            if rotation is not None and hasattr(rotation, "setSFRotation"):
                rotation.setSFRotation(list(self.initial_body_rotation))
        reset_physics = getattr(self.body, "resetPhysics", None)
        if reset_physics:
            reset_physics()

    def _reset_joint_positions(self) -> None:
        angles = self.spider.joint_angles_rad()
        for leg in LEG_NAMES:
            for joint_index, joint in enumerate(JOINT_NAMES):
                joint_node = self.joints.get(f"{leg}_{joint}")
                set_position = getattr(joint_node, "setJointPosition", None)
                if set_position:
                    set_position(angles[leg][joint_index])

    def reset(self) -> None:
        """Reset physics, body pose, gait phases, and all joint commands."""

        self.spider.reset()
        self._reset_body()
        self._reset_joint_positions()
        self._apply_joint_angles()
        simulation_reset = getattr(self.supervisor, "simulationResetPhysics", None)
        if simulation_reset:
            simulation_reset()

    def _apply_joint_angles(self) -> None:
        angles = self.spider.joint_angles_rad()
        for leg in LEG_NAMES:
            for joint_index, joint in enumerate(JOINT_NAMES):
                motor = self.motors.get(f"{leg}_{joint}")
                if motor is not None and hasattr(motor, "setPosition"):
                    motor.setPosition(angles[leg][joint_index])

    def apply(self, command: Command | dict[str, Any] | None) -> None:
        """Advance gait state and send the resulting angles to Webots motors."""

        normalized = command if isinstance(command, Command) else Command.from_mapping(command)
        if normalized.mode == "init":
            self.reset()
            return
        self.spider.command(normalized)
        self._apply_joint_angles()

    def run(self) -> None:
        """Run until Webots requests termination."""

        if Keyboard is None:
            raise RuntimeError("Webots controller API is not available")
        keyboard = Keyboard()
        keyboard.enable(self.timestep)
        while self.supervisor.step(self.timestep) != -1:
            pressed = []
            key = keyboard.getKey()
            while key >= 0:
                pressed.append(key)
                key = keyboard.getKey()
            self.apply(keyboard_command(pressed))

    def _step_simulation(
        self,
        count: int,
        command: Command | dict[str, Any] | None = None,
    ) -> None:
        for _ in range(count):
            if command is not None:
                self.apply(command)
            if self.supervisor.step(self.timestep) == -1:
                raise RuntimeError("Webots terminated during the smoke sequence")

    def _body_state(self) -> tuple[list[float], float]:
        if self.body is None:
            raise RuntimeError("Supervisor self node is unavailable")
        position = list(self.body.getPosition())
        orientation = list(self.body.getOrientation())
        yaw = math.atan2(orientation[2], orientation[0])
        return position, yaw

    def _sensor_angles_deg(self) -> dict[str, list[float]]:
        return {
            leg: [
                math.degrees(self.sensors[f"{leg}_{joint}"].getValue())
                for joint in JOINT_NAMES
            ]
            for leg in LEG_NAMES
        }

    @staticmethod
    def _angle_delta(final: float, initial: float) -> float:
        return math.atan2(math.sin(final - initial), math.cos(final - initial))

    @staticmethod
    def _write_smoke_result(result: dict[str, Any], result_path: str | None) -> None:
        encoded_result = json.dumps(result, sort_keys=True)
        if result_path:
            Path(result_path).write_text(encoded_result, encoding="utf-8")
        print("SPIDER_SMOKE_RESULT " + encoded_result, flush=True)

    def run_smoke(self, result_path: str | None = None) -> None:
        """Run deterministic command checks and quit a headless Webots run.

        The smoke path deliberately reports adapter reference deltas rather
        than claiming calibrated dynamic performance.  It verifies that the
        real Webots world resolved every device and that the four keyboard
        directions reach the existing gait boundary with opposite signs.
        """

        initial_body_position, initial_body_yaw = self._body_state()
        self._step_simulation(1, Command.stop())
        first_step_position, first_step_yaw = self._body_state()
        self._step_simulation(79, Command.stop())
        body_position, body_yaw = self._body_state()
        missing_motors = sorted(
            key for key, motor in self.motors.items() if motor is None
        )
        missing_sensors = sorted(
            key for key, sensor in self.sensors.items() if sensor is None
        )

        scenarios = {
            "forward": {"mode": "walk", "vx": 1.0, "speed": 1.0},
            "backward": {"mode": "walk", "vx": -1.0, "speed": 1.0},
            "left": {"mode": "turn", "turn": -1.0, "speed": 1.0},
            "right": {"mode": "turn", "turn": 1.0, "speed": 1.0},
        }
        deltas: dict[str, list[float]] = {}
        for name, payload in scenarios.items():
            self.spider.reset()
            before = self.spider.tip_positions_mm()["legi"]
            self.apply(payload)
            after = self.spider.tip_positions_mm()["legi"]
            deltas[name] = [after[index] - before[index] for index in range(3)]

        self.apply({"mode": "stop"})
        stop_angles = self.spider.joint_angles_deg()["legi"]
        self.apply({"mode": "init"})
        reset_angles = self.spider.joint_angles_deg()["legi"]
        reset_translation, reset_rotation = self._body_field_pose()
        result = {
            "devices": len(self.motors) - len(missing_motors),
            "sensors": len(self.sensors) - len(missing_sensors),
            "missing_motors": missing_motors,
            "missing_sensors": missing_sensors,
            "sensor_angles_deg": (
                {} if missing_sensors else self._sensor_angles_deg()
            ),
            "configured_body_translation_m": list(self.initial_body_translation),
            "configured_body_rotation": list(self.initial_body_rotation),
            "initial_body_position_m": initial_body_position,
            "initial_body_yaw_rad": initial_body_yaw,
            "first_step_position_m": first_step_position,
            "first_step_yaw_rad": first_step_yaw,
            "first_step_displacement_m": math.dist(
                initial_body_position, first_step_position
            ),
            "body_position_m": body_position,
            "body_yaw_rad": body_yaw,
            "settling_displacement_m": math.dist(
                initial_body_position, body_position
            ),
            "deltas_mm": deltas,
            "stop_angles_deg": stop_angles,
            "reset_angles_deg": reset_angles,
            "reset_body_translation_m": reset_translation,
            "reset_body_rotation": reset_rotation,
        }
        self._write_smoke_result(result, result_path)
        simulation_quit = getattr(self.supervisor, "simulationQuit", None)
        if simulation_quit:
            simulation_quit(0)

    def run_motion_smoke(
        self,
        scenario: str,
        result_path: str | None = None,
    ) -> None:
        """Measure one isolated physical behavior in a fresh Webots process."""

        scenarios = {
            "forward": {"mode": "walk", "vx": 1.0, "speed": 1.0},
            "backward": {"mode": "walk", "vx": -1.0, "speed": 1.0},
            "left": {"mode": "turn", "turn": -1.0, "speed": 1.0},
            "right": {"mode": "turn", "turn": 1.0, "speed": 1.0},
        }
        self.reset()
        self._step_simulation(80, Command.stop())

        result: dict[str, Any] = {"scenario": scenario}
        if scenario in scenarios:
            start_position, start_yaw = self._body_state()
            motion_steps = 80 if scenario in {"left", "right"} else 120
            self._step_simulation(motion_steps, scenarios[scenario])
            end_position, end_yaw = self._body_state()
            result.update(
                {
                    "start_position_m": start_position,
                    "end_position_m": end_position,
                    "displacement_m": [
                        end_position[index] - start_position[index]
                        for index in range(3)
                    ],
                    "yaw_rad": self._angle_delta(end_yaw, start_yaw),
                }
            )
        elif scenario == "stop":
            self._step_simulation(160, scenarios["forward"])
            self._step_simulation(30, Command.stop())
            stop_start, _ = self._body_state()
            self._step_simulation(40, Command.stop())
            stop_end, _ = self._body_state()
            result.update(
                {
                    "stop_start_m": stop_start,
                    "stop_end_m": stop_end,
                    "stop_drift_m": math.dist(stop_start, stop_end),
                }
            )
        elif scenario == "reset":
            self._step_simulation(120, scenarios["forward"])
            before_reset, _ = self._body_state()
            self.reset()
            self._step_simulation(1)
            reset_position, reset_yaw = self._body_state()
            result.update(
                {
                    "before_reset_m": before_reset,
                    "reset_position_m": reset_position,
                    "reset_yaw_rad": reset_yaw,
                    "reset_angles_deg": self.spider.joint_angles_deg()["legi"],
                    "reset_sensor_angles_deg": self._sensor_angles_deg(),
                }
            )
        elif scenario == "stand":
            start_position, start_yaw = self._body_state()
            start_angles = self._sensor_angles_deg()
            self._step_simulation(120, Command.stop())
            end_position, end_yaw = self._body_state()
            result.update(
                {
                    "start_position_m": start_position,
                    "end_position_m": end_position,
                    "drift_m": math.dist(start_position, end_position),
                    "yaw_rad": self._angle_delta(end_yaw, start_yaw),
                    "start_angles_deg": start_angles,
                    "end_angles_deg": self._sensor_angles_deg(),
                }
            )
        else:
            raise ValueError(f"unknown motion smoke scenario: {scenario}")

        self._write_smoke_result(result, result_path)
        simulation_quit = getattr(self.supervisor, "simulationQuit", None)
        if simulation_quit:
            simulation_quit(0)


def main() -> None:
    controller = SpiderController()
    get_custom_data = getattr(controller.supervisor, "getCustomData", None)
    custom_data = get_custom_data() if get_custom_data else ""
    if custom_data.startswith("smoke-motion-"):
        mode, result_path = custom_data.split(":", 1)
        controller.run_motion_smoke(mode.removeprefix("smoke-motion-"), result_path)
    elif custom_data.startswith("smoke:"):
        controller.run_smoke(custom_data.removeprefix("smoke:"))
    else:
        controller.run()


if __name__ == "__main__":
    main()

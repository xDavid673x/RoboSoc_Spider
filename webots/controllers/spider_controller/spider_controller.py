"""Standalone Webots controller for the six-legged Spider model.

The kinematics and gait implementation remains pure Python in
``kinematics_adapter``.  This module only owns the Webots device boundary and
the keyboard-to-command reducer, so it can be imported and tested without
Webots being installed.
"""

from __future__ import annotations

from collections.abc import Iterable
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
BODY_TRANSLATION = (0.0, 0.0, 0.16)
BODY_ROTATION = (0.0, 0.0, 1.0, 0.0)


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
        self.motors: dict[str, Any] = {}
        self.sensors: dict[str, Any] = {}
        self._lookup_devices()
        self._enable_sensors()
        self.reset()

    def _get_self_node(self) -> Any | None:
        getter = getattr(self.supervisor, "getSelf", None)
        return getter() if getter else None

    def _lookup_devices(self) -> None:
        get_device = getattr(self.supervisor, "getDevice", None)
        if get_device is None:
            return
        for leg in LEG_NAMES:
            for joint in JOINT_NAMES:
                key = f"{leg}_{joint}"
                self.motors[key] = get_device(f"{key}_motor")
                self.sensors[key] = get_device(f"{key}_sensor")

    def _enable_sensors(self) -> None:
        for sensor in self.sensors.values():
            enable = getattr(sensor, "enable", None)
            if enable:
                enable(self.timestep)

    def _reset_body(self) -> None:
        if self.body is None:
            return
        reset_physics = getattr(self.body, "resetPhysics", None)
        if reset_physics:
            reset_physics()
        get_field = getattr(self.body, "getField", None)
        if not get_field:
            return
        translation = get_field("translation")
        if translation is not None and hasattr(translation, "setSFVec3f"):
            translation.setSFVec3f(list(BODY_TRANSLATION))
        rotation = get_field("rotation")
        if rotation is not None and hasattr(rotation, "setSFRotation"):
            rotation.setSFRotation(list(BODY_ROTATION))

    def reset(self) -> None:
        """Reset physics, body pose, gait phases, and all joint commands."""

        simulation_reset = getattr(self.supervisor, "simulationResetPhysics", None)
        if simulation_reset:
            simulation_reset()
        self._reset_body()
        self.spider.reset()
        self._apply_joint_angles()

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
            key = keyboard.getKey()
            pressed = [] if key < 0 else [key]
            self.apply(keyboard_command(pressed))


def main() -> None:
    SpiderController().run()


if __name__ == "__main__":
    main()

import math

from webots.controllers.spider_controller.spider_controller import (
    JOINT_NAMES,
    LEG_NAMES,
    TIME_STEP_MS,
    SpiderController,
    keyboard_command,
)


class FakeField:
    def __init__(self):
        self.value = None

    def setSFVec3f(self, value):
        self.value = value

    def setSFRotation(self, value):
        self.value = value


class FakeBody:
    def __init__(self):
        self.reset_count = 0
        self.fields = {"translation": FakeField(), "rotation": FakeField()}

    def resetPhysics(self):
        self.reset_count += 1

    def getField(self, name):
        return self.fields[name]


class FakeDevice:
    def __init__(self):
        self.enabled = []
        self.positions = []

    def enable(self, timestep):
        self.enabled.append(timestep)

    def setPosition(self, position):
        self.positions.append(position)


class FakeSupervisor:
    def __init__(self):
        self.body = FakeBody()
        self.devices = {}
        self.physics_reset_count = 0

    def getSelf(self):
        return self.body

    def getDevice(self, name):
        return self.devices.setdefault(name, FakeDevice())

    def simulationResetPhysics(self):
        self.physics_reset_count += 1


def test_keyboard_reducer_prioritizes_reset_then_stop_then_turn_then_walk():
    assert keyboard_command(["w", "r"]).mode == "init"
    assert keyboard_command(["w", " "]).mode == "stand"
    command = keyboard_command(["w", "a"])
    assert command.mode == "turn"
    assert command.turn == -1.0
    assert keyboard_command(["w"]).vx == 1.0
    assert keyboard_command(["s"]).vx == -1.0


def test_controller_looks_up_all_devices_and_enables_sensors():
    supervisor = FakeSupervisor()
    controller = SpiderController(supervisor)

    assert controller.timestep == TIME_STEP_MS
    assert len(controller.motors) == 18
    assert len(controller.sensors) == 18
    assert all(
        sensor.enabled == [TIME_STEP_MS] for sensor in controller.sensors.values()
    )
    assert all(
        f"{leg}_{joint}" in controller.motors
        for leg in LEG_NAMES
        for joint in JOINT_NAMES
    )


def test_reset_restores_body_physics_and_joint_pose():
    supervisor = FakeSupervisor()
    controller = SpiderController(supervisor)
    supervisor.body.fields["translation"].value = None
    controller.apply({"mode": "walk", "vx": 1.0, "speed": 1.0})
    controller.reset()

    assert supervisor.physics_reset_count == 2
    assert supervisor.body.reset_count == 2
    assert supervisor.body.fields["translation"].value == [0.0, 0.133, 0.0]
    assert supervisor.body.fields["rotation"].value == [0.0, 1.0, 0.0, 0.0]
    assert controller.spider.last_mode == "init"
    assert all(
        motor.positions[-1] == math.radians(angle)
        for motor, angle in zip(
            [supervisor.devices[f"legi_{joint}_motor"] for joint in JOINT_NAMES],
            (0.0, 28.0, 115.0),
        )
    )

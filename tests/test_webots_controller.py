import math

from webots.controllers.spider_controller.spider_controller import (
    JOINT_NAMES,
    LEG_NAMES,
    TIME_STEP_MS,
    SpiderController,
    keyboard_command,
)


class FakeField:
    def __init__(self, value=None):
        self.value = value

    def getSFVec3f(self):
        return self.value

    def getSFRotation(self):
        return self.value

    def setSFVec3f(self, value):
        self.value = value

    def setSFRotation(self, value):
        self.value = value


class FakeBody:
    def __init__(self, translation=None, rotation=None):
        self.reset_count = 0
        self.fields = {
            "translation": FakeField(translation),
            "rotation": FakeField(rotation),
        }

    def resetPhysics(self):
        self.reset_count += 1

    def getField(self, name):
        return self.fields[name]


class FakeDevice:
    def __init__(self, tag):
        self._tag = tag
        self.enabled = []
        self.positions = []

    def enable(self, timestep):
        self.enabled.append(timestep)

    def setPosition(self, position):
        self.positions.append(position)


class FakeJoint:
    def __init__(self):
        self.positions = []

    def setJointPosition(self, position):
        self.positions.append(position)


class FakeDeviceNode:
    def __init__(self, joint):
        self.joint = joint

    def getParentNode(self):
        return self.joint


class FakeSupervisor:
    def __init__(self, body_translation=None, body_rotation=None):
        self.body = FakeBody(body_translation, body_rotation)
        self.devices = {}
        self.joints = {}
        self.physics_reset_count = 0

    def getSelf(self):
        return self.body

    def getDevice(self, name):
        return self.devices.setdefault(name, FakeDevice(name))

    def getFromDevice(self, tag):
        return FakeDeviceNode(self.joints.setdefault(tag, FakeJoint()))

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
    assert len(controller.joints) == 18
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
        joint.positions[-1] == math.radians(angle)
        for joint, angle in zip(
            [supervisor.joints[f"legi_{joint}_motor"] for joint in JOINT_NAMES],
            (0.0, 28.0, 115.0),
        )
    )
    assert all(
        motor.positions[-1] == math.radians(angle)
        for motor, angle in zip(
            [supervisor.devices[f"legi_{joint}_motor"] for joint in JOINT_NAMES],
            (0.0, 28.0, 115.0),
        )
    )


def test_reset_restores_the_body_pose_configured_by_the_world():
    translation = [-0.062589686, 0.121963750, 0.0]
    rotation = [0.0, 0.0, 1.0, 0.349065850]
    supervisor = FakeSupervisor(translation, rotation)
    controller = SpiderController(supervisor)

    supervisor.body.fields["translation"].value = [1.0, 2.0, 3.0]
    supervisor.body.fields["rotation"].value = [0.0, 1.0, 0.0, 1.0]
    controller.reset()

    assert controller.initial_body_translation == tuple(translation)
    assert controller.initial_body_rotation == tuple(rotation)
    assert supervisor.body.fields["translation"].value == translation
    assert supervisor.body.fields["rotation"].value == rotation

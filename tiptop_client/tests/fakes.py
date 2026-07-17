from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class FakeHeader:
    stamp: object | None = None


@dataclass
class FakeJointState:
    position: list[float] = field(default_factory=list)
    name: list[str] = field(default_factory=list)
    velocity: list[float] = field(default_factory=list)
    effort: list[float] = field(default_factory=list)
    header: FakeHeader = field(default_factory=FakeHeader)


class FakePublisher:
    def __init__(self) -> None:
        self.messages: list[FakeJointState] = []
        self.unregistered = False

    def publish(self, message: FakeJointState) -> None:
        self.messages.append(
            FakeJointState(
                position=list(message.position),
                name=list(message.name),
                velocity=list(message.velocity),
                effort=list(message.effort),
            )
        )

    def unregister(self) -> None:
        self.unregistered = True


class FakeSubscriber:
    def unregister(self) -> None:
        return None


class FakeRos:
    class core:
        @staticmethod
        def is_initialized() -> bool:
            return True

    class Time:
        @staticmethod
        def now() -> float:
            return 123.0

    def __init__(self) -> None:
        self.publisher = FakePublisher()

    def Publisher(self, *_args, **_kwargs) -> FakePublisher:
        return self.publisher

    def Subscriber(self, *_args, **_kwargs) -> FakeSubscriber:
        return FakeSubscriber()


class FakeBackend:
    dof = 6

    def __init__(self) -> None:
        self.closed = False
        self.stop_called = False

    def health(self):
        return {"robot_state_received": True, "trajectory_running": False, "dof": 6}

    def get_joint_positions(self):
        return [0.0] * self.dof

    def open_gripper(self, **_kwargs):
        return {"success": True}

    def close_gripper(self, **_kwargs):
        return {"success": True}

    def execute_joint_impedance_path(self, *_args):
        return {"success": True}

    def stop(self):
        self.stop_called = True
        return {"success": True}

    def close(self):
        self.closed = True

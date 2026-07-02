from __future__ import annotations

from dataclasses import dataclass

from message.dds import dds_type_for_topic
from runtime.adapters.dds.nav import DDSNavInModule, DDSNavOutModule
from runtime.blueprints.stacks.navigation import navigation
from runtime.msgs.geometry import Pose, PoseStamped, Twist, Vector3
from runtime.msgs.nav import Path
from runtime.profiles.binding_policy import navigation_input_uses_dds, navigation_output_uses_dds
from runtime.runtime_interface import TOPICS


@dataclass
class DDS_Time:
    sec: int = 0
    nanosec: int = 0


@dataclass
class DDS_Header:
    stamp: DDS_Time
    frame_id: str


@dataclass
class DDS_Point:
    x: float
    y: float
    z: float


@dataclass
class DDS_Quaternion:
    x: float
    y: float
    z: float
    w: float


@dataclass
class DDS_Pose:
    position: DDS_Point
    orientation: DDS_Quaternion


@dataclass
class DDS_PoseStamped:
    header: DDS_Header
    pose: DDS_Pose


@dataclass
class DDS_Vector3:
    x: float
    y: float
    z: float


@dataclass
class DDS_Twist:
    linear: DDS_Vector3
    angular: DDS_Vector3


@dataclass
class DDS_TwistStamped:
    header: DDS_Header
    twist: DDS_Twist


@dataclass
class DDS_Path:
    header: DDS_Header
    poses: list[DDS_PoseStamped]


@dataclass
class DDS_String:
    data: str


class _FakePublisher:
    def __init__(self, topic: str) -> None:
        self.topic = topic
        self.messages = []
        self.closed = False

    def publish(self, msg) -> None:
        self.messages.append(msg)

    def close(self) -> None:
        self.closed = True


class _FakeSubscriber:
    def __init__(self, topic: str) -> None:
        self.topic = topic
        self.closed = False

    def close(self) -> None:
        self.closed = True


class _FakeDDSTransport:
    def __init__(self) -> None:
        self.publishers: dict[str, _FakePublisher] = {}
        self.callbacks = {}
        self.closed = False

    def create_publisher(self, config):
        publisher = _FakePublisher(config.name)
        self.publishers[config.name] = publisher
        return publisher

    def create_subscriber(self, config, callback):
        self.callbacks[config.name] = callback
        return _FakeSubscriber(config.name)

    def emit(self, topic: str, msg) -> None:
        self.callbacks[topic](msg)

    def close(self) -> None:
        self.closed = True


def _install_fake_dds_types(monkeypatch) -> None:
    import message.dds_types as dds_mod

    for cls in (
        DDS_Time,
        DDS_Header,
        DDS_Point,
        DDS_Quaternion,
        DDS_Pose,
        DDS_PoseStamped,
        DDS_Vector3,
        DDS_Twist,
        DDS_TwistStamped,
        DDS_Path,
        DDS_String,
    ):
        monkeypatch.setattr(dds_mod, cls.__name__, cls, raising=False)


def test_dds_topic_registry_covers_navigation_boundary() -> None:
    assert dds_type_for_topic(TOPICS.global_path).__name__ == "Path"
    assert dds_type_for_topic(TOPICS.local_path).__name__ == "Path"
    assert dds_type_for_topic(TOPICS.nav_way_point).__name__ == "PoseStamped"
    assert dds_type_for_topic(TOPICS.goal_pose).__name__ == "PoseStamped"
    assert dds_type_for_topic(TOPICS.cancel).__name__ == "Text"
    assert dds_type_for_topic(TOPICS.semantic_instruction).__name__ == "Text"
    assert dds_type_for_topic(TOPICS.cmd_vel).__name__ == "TwistStamped"


def test_dds_nav_out_publishes_typed_messages(monkeypatch) -> None:
    _install_fake_dds_types(monkeypatch)
    transport = _FakeDDSTransport()
    nav_out = DDSNavOutModule(transport=transport, default_frame_id="map")
    nav_out.setup()

    nav_out.global_path._deliver(
        Path(
            poses=[PoseStamped(Pose(1.0, 2.0, 0.3), frame_id="map")],
            frame_id="map",
        )
    )
    nav_out.local_path._deliver(
        Path(
            poses=[PoseStamped(Pose(3.0, 4.0, 0.2), frame_id="map")],
            frame_id="map",
        )
    )
    nav_out.waypoint._deliver(PoseStamped(Pose(5.0, 6.0, 0.1), frame_id="map"))
    nav_out.cmd_vel._deliver(Twist(Vector3(0.4, 0.0, 0.0), Vector3(0.0, 0.0, 0.2)))

    global_path = transport.publishers[TOPICS.global_path].messages[-1]
    local_path = transport.publishers[TOPICS.local_path].messages[-1]
    waypoint = transport.publishers[TOPICS.nav_way_point].messages[-1]
    cmd_vel = transport.publishers[TOPICS.cmd_vel].messages[-1]

    assert isinstance(global_path, DDS_Path)
    assert global_path.header.frame_id == "map"
    assert global_path.poses[0].pose.position.x == 1.0
    assert isinstance(local_path, DDS_Path)
    assert local_path.poses[0].pose.position.y == 4.0
    assert isinstance(waypoint, DDS_PoseStamped)
    assert waypoint.pose.position.x == 5.0
    assert isinstance(cmd_vel, DDS_TwistStamped)
    assert cmd_vel.header.frame_id == "body"
    assert cmd_vel.twist.linear.x == 0.4
    assert nav_out.health()["publish_counts"] == {
        TOPICS.global_path: 1,
        TOPICS.local_path: 1,
        TOPICS.nav_way_point: 1,
        TOPICS.cmd_vel: 1,
    }


def test_dds_nav_in_publishes_module_messages(monkeypatch) -> None:
    _install_fake_dds_types(monkeypatch)
    transport = _FakeDDSTransport()
    nav_in = DDSNavInModule(transport=transport, default_frame_id="map")
    goals = []
    cancels = []
    instructions = []
    nav_in.goal_pose.subscribe(goals.append)
    nav_in.cancel.subscribe(cancels.append)
    nav_in.instruction.subscribe(instructions.append)
    nav_in.setup()

    transport.emit(
        TOPICS.goal_pose,
        DDS_PoseStamped(
            header=DDS_Header(DDS_Time(12, 500_000_000), "map"),
            pose=DDS_Pose(
                position=DDS_Point(1.0, 2.0, 0.3),
                orientation=DDS_Quaternion(0.0, 0.0, 0.0, 1.0),
            ),
        ),
    )
    transport.emit(TOPICS.cancel, DDS_String("operator_cancel"))
    transport.emit(TOPICS.semantic_instruction, DDS_String("inspect pump"))

    assert goals[-1].x == 1.0
    assert goals[-1].y == 2.0
    assert goals[-1].ts == 12.5
    assert cancels == ["operator_cancel"]
    assert instructions == ["inspect pump"]
    assert nav_in.health()["message_counts"] == {
        TOPICS.goal_pose: 1,
        TOPICS.cancel: 1,
        TOPICS.semantic_instruction: 1,
    }


def test_dds_nav_adapters_disable_when_python_dds_is_unavailable(monkeypatch) -> None:
    nav_in = DDSNavInModule()
    nav_out = DDSNavOutModule()
    monkeypatch.setattr(
        nav_in,
        "_create_default_transport",
        lambda: (_ for _ in ()).throw(ImportError("missing cyclonedds")),
    )
    monkeypatch.setattr(
        nav_out,
        "_create_default_transport",
        lambda: (_ for _ in ()).throw(ImportError("missing cyclonedds")),
    )

    nav_in.setup()
    nav_out.setup()

    assert nav_in.health()["transport"] == "disabled"
    assert nav_in.health()["subscribed_topics"] == []
    assert "missing cyclonedds" in nav_in.health()["disabled_reason"]
    assert nav_out.health()["transport"] == "disabled"
    assert "missing cyclonedds" in nav_out.health()["disabled_reason"]


def test_navigation_stack_selects_dds_nav_adapters() -> None:
    bp = navigation(
        enable_nav_in=True,
        enable_nav_out=True,
        nav_in_adapter="dds_nav_input",
        nav_out_adapter="dds_nav_output",
        _endpoint_transport="dds",
        enable_native=False,
        planning_frame_id="map",
    )
    entries = {entry.name: entry for entry in bp._entries}
    wires = {
        f"{wire.out_module}.{wire.out_port}->{wire.in_module}.{wire.in_port}"
        for wire in bp._wires
    }

    assert entries["nav.in"].module_cls is DDSNavInModule
    assert entries["nav.out"].module_cls is DDSNavOutModule
    assert entries["nav.in"].config == {"default_frame_id": "map"}
    assert entries["nav.out"].config == {"default_frame_id": "map"}
    assert navigation_input_uses_dds({"nav_in_adapter": "dds_nav_input"})
    assert navigation_output_uses_dds({"nav_out_adapter": "dds_nav_output"})
    assert "nav.in.goal_pose->nav.mission.goal_pose" in wires
    assert "nav.in.cancel->nav.mission.cancel" in wires
    assert "nav.in.instruction->nav.mission.instruction" in wires
    assert "nav.mission.global_path->nav.out.global_path" in wires
    assert "nav.local_planner.local_path->nav.out.local_path" in wires
    assert "nav.mission.waypoint->nav.out.waypoint" in wires

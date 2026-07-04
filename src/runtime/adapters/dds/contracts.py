"""Typed DDS endpoint contract for Thunder field runtime."""

from __future__ import annotations

from dataclasses import dataclass

from message.dds import TopicSpec, dds_topic_name, topic_spec
from runtime.runtime_interface import (
    THUNDER_FIELD_RUNTIME_CONTRACT,
    TOPICS,
    runtime_topic_allowed_frame_ids,
)
from runtime.tf import TF_STATIC_TOPIC, TF_TOPIC

THUNDER_FIELD_DDS_CONTRACT_NAME = "thunder_field_dds_v1"
DDS_PAYLOAD_FORMAT = "dds.idl.v1"


@dataclass(frozen=True)
class DDSEndpointBinding:
    """A canonical LingTu topic bound to one typed DDS topic."""

    topic: str
    channel: str
    direction: str
    schema: str
    type_name: str
    import_path: str
    idl_type: str
    cpp_type: str
    payload_format: str = DDS_PAYLOAD_FORMAT
    frame_ids: tuple[str, ...] = ()
    required: bool = True
    note: str = ""

    @property
    def typed(self) -> bool:
        return True


@dataclass(frozen=True)
class DDSEndpointContract:
    """Named typed DDS endpoint contract for a runtime endpoint."""

    name: str
    runtime_contract: str
    transport: str
    bindings: tuple[DDSEndpointBinding, ...]

    @property
    def topics(self) -> tuple[str, ...]:
        return tuple(binding.topic for binding in self.bindings)

    @property
    def required_topics(self) -> tuple[str, ...]:
        return tuple(binding.topic for binding in self.bindings if binding.required)

    def binding_for_topic(self, topic: str) -> DDSEndpointBinding:
        for binding in self.bindings:
            if binding.topic == topic:
                return binding
        raise KeyError(f"DDS contract '{self.name}' has no binding for topic '{topic}'")


def _topic_spec(topic: str) -> TopicSpec:
    spec = topic_spec(topic)
    if spec is None:
        raise KeyError(f"topic {topic!r} has no typed DDS topic spec")
    return spec


def _binding(
    topic: str,
    *,
    direction: str,
    schema: str,
    required: bool = True,
    note: str = "",
) -> DDSEndpointBinding:
    spec = _topic_spec(topic)
    return DDSEndpointBinding(
        topic=topic,
        channel=dds_topic_name(topic, typed=True),
        direction=direction,
        schema=schema,
        type_name=spec.type_name,
        import_path=spec.import_path,
        idl_type=spec.idl_type,
        cpp_type=spec.cpp_type,
        frame_ids=tuple(
            runtime_topic_allowed_frame_ids(THUNDER_FIELD_RUNTIME_CONTRACT).get(topic, ())
        ),
        required=required,
        note=note,
    )


THUNDER_FIELD_DDS_CONTRACT = DDSEndpointContract(
    name=THUNDER_FIELD_DDS_CONTRACT_NAME,
    runtime_contract=THUNDER_FIELD_RUNTIME_CONTRACT,
    transport="dds",
    bindings=(
        _binding(
            TF_TOPIC,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.TFMessage",
            required=False,
            note="Dynamic frame transforms for the native runtime FrameTree.",
        ),
        _binding(
            TF_STATIC_TOPIC,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.TFMessage",
            required=False,
            note="Static frame transforms for sensor mounts and calibration.",
        ),
        _binding(
            TOPICS.lidar_scan,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.LivoxFrame",
            note="Native raw Livox MID-360 frame entering the SLAM/localization boundary.",
        ),
        _binding(
            TOPICS.imu,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.Imu",
            note="Time-aligned IMU stream for the field localization boundary.",
        ),
        _binding(
            TOPICS.odometry,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.Odometry",
        ),
        _binding(
            TOPICS.registered_cloud,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.PointCloud2",
        ),
        _binding(
            TOPICS.map_cloud,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.PointCloud2",
        ),
        _binding(
            TOPICS.saved_map_cloud,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.PointCloud2",
            required=False,
        ),
        _binding(
            TOPICS.localization_health,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.Text",
            required=False,
        ),
        _binding(
            TOPICS.localization_quality,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.Float32",
            required=False,
        ),
        _binding(
            TOPICS.goal_pose,
            direction="endpoint_to_lingtu",
            schema="geometry_msgs/msg/PoseStamped",
            required=False,
            note="Operator or supervisor navigation goal entering the LingTu planner.",
        ),
        _binding(
            TOPICS.cancel,
            direction="endpoint_to_lingtu",
            schema="std_msgs/msg/String",
            required=False,
            note="Mission cancel command entering the LingTu navigation FSM.",
        ),
        _binding(
            TOPICS.semantic_instruction,
            direction="endpoint_to_lingtu",
            schema="std_msgs/msg/String",
            required=False,
            note="Natural-language navigation instruction entering semantic planning.",
        ),
        _binding(
            TOPICS.traversability,
            direction="endpoint_to_lingtu",
            schema="lingtu.dds.OccupancyGrid",
            required=False,
            note="Live terrain traversability risk grid for the native C++ local planner.",
        ),
        _binding(
            TOPICS.global_path,
            direction="lingtu_to_endpoint",
            schema="nav_msgs/msg/Path",
        ),
        _binding(
            TOPICS.local_path,
            direction="lingtu_to_endpoint",
            schema="nav_msgs/msg/Path",
        ),
        _binding(
            TOPICS.nav_way_point,
            direction="lingtu_to_endpoint",
            schema="geometry_msgs/msg/PoseStamped",
            required=False,
            note="Active navigation waypoint for endpoint visualization and supervision.",
        ),
        _binding(
            TOPICS.cmd_vel,
            direction="lingtu_to_endpoint",
            schema="geometry_msgs/msg/TwistStamped",
            note="Muxed body-frame command after safety arbitration.",
        ),
    ),
)

_CONTRACTS: dict[str, DDSEndpointContract] = {
    THUNDER_FIELD_DDS_CONTRACT.name: THUNDER_FIELD_DDS_CONTRACT,
}


def endpoint_contract(name: str) -> DDSEndpointContract:
    """Return a named DDS endpoint contract."""

    try:
        return _CONTRACTS[name]
    except KeyError as exc:
        choices = ", ".join(endpoint_contract_names())
        raise KeyError(f"unknown DDS endpoint contract '{name}' (choices: {choices})") from exc


def endpoint_contract_names() -> tuple[str, ...]:
    """Return known DDS endpoint contract names."""

    return tuple(sorted(_CONTRACTS))


def binding_for_topic(contract_name: str, topic: str) -> DDSEndpointBinding:
    """Return the binding for a canonical topic in a contract."""

    return endpoint_contract(contract_name).binding_for_topic(topic)

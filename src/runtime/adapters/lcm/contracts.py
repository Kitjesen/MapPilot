"""LCM endpoint contracts for Thunder field runtime.

The low-level LCM package stays inside ``runtime.transport``. This module defines
the product endpoint topic contract: canonical LingTu topics, LCM channel names,
direction, schemas, and frame expectations.
"""

from __future__ import annotations

import re
from dataclasses import dataclass

from runtime.runtime_interface import (
    THUNDER_FIELD_RUNTIME_CONTRACT,
    TOPICS,
    runtime_topic_allowed_frame_ids,
)

THUNDER_FIELD_LCM_CONTRACT_NAME = "thunder_field_lcm_v1"
LCM_PAYLOAD_FORMAT = "lingtu.transport.json.v1"


@dataclass(frozen=True)
class LCMEndpointBinding:
    """A canonical LingTu topic bound to one LCM channel."""

    topic: str
    channel: str
    direction: str
    schema: str
    payload_format: str = LCM_PAYLOAD_FORMAT
    frame_ids: tuple[str, ...] = ()
    required: bool = True
    note: str = ""


@dataclass(frozen=True)
class LCMEndpointContract:
    """Named LCM endpoint contract for a runtime endpoint."""

    name: str
    runtime_contract: str
    transport: str
    bindings: tuple[LCMEndpointBinding, ...]

    @property
    def topics(self) -> tuple[str, ...]:
        return tuple(binding.topic for binding in self.bindings)

    @property
    def required_topics(self) -> tuple[str, ...]:
        return tuple(binding.topic for binding in self.bindings if binding.required)

    def binding_for_topic(self, topic: str) -> LCMEndpointBinding:
        for binding in self.bindings:
            if binding.topic == topic:
                return binding
        raise KeyError(f"LCM contract '{self.name}' has no binding for topic '{topic}'")


def lcm_channel_for_topic(topic: str) -> str:
    """Return the stable LCM channel name for a canonical topic."""

    token = re.sub(r"[^A-Za-z0-9]+", "_", topic).strip("_").upper()
    return f"LINGTU_{token}"


def _binding(
    topic: str,
    *,
    direction: str,
    schema: str,
    required: bool = True,
    note: str = "",
) -> LCMEndpointBinding:
    return LCMEndpointBinding(
        topic=topic,
        channel=lcm_channel_for_topic(topic),
        direction=direction,
        schema=schema,
        frame_ids=tuple(
            runtime_topic_allowed_frame_ids(THUNDER_FIELD_RUNTIME_CONTRACT).get(topic, ())
        ),
        required=required,
        note=note,
    )


THUNDER_FIELD_LCM_CONTRACT = LCMEndpointContract(
    name=THUNDER_FIELD_LCM_CONTRACT_NAME,
    runtime_contract=THUNDER_FIELD_RUNTIME_CONTRACT,
    transport="lcm",
    bindings=(
        _binding(
            TOPICS.lidar_scan,
            direction="endpoint_to_lingtu",
            schema="lingtu.sensor.point_cloud2.v1",
            note="Normalized MID-360 scan entering the SLAM/localization boundary.",
        ),
        _binding(
            TOPICS.imu,
            direction="endpoint_to_lingtu",
            schema="lingtu.sensor.imu.v1",
            note="Time-aligned IMU stream for the field localization boundary.",
        ),
        _binding(
            TOPICS.odometry,
            direction="endpoint_to_lingtu",
            schema="lingtu.nav.odometry.v1",
        ),
        _binding(
            TOPICS.registered_cloud,
            direction="endpoint_to_lingtu",
            schema="lingtu.sensor.point_cloud2.v1",
        ),
        _binding(
            TOPICS.map_cloud,
            direction="endpoint_to_lingtu",
            schema="lingtu.sensor.point_cloud2.v1",
        ),
        _binding(
            TOPICS.localization_health,
            direction="endpoint_to_lingtu",
            schema="lingtu.status.localization_health.v1",
            required=False,
        ),
        _binding(
            TOPICS.localization_quality,
            direction="endpoint_to_lingtu",
            schema="lingtu.status.localization_quality.v1",
            required=False,
        ),
        _binding(
            TOPICS.goal_pose,
            direction="endpoint_to_lingtu",
            schema="lingtu.geometry.pose_stamped.v1",
            required=False,
            note="Operator or supervisor navigation goal entering the LingTu planner.",
        ),
        _binding(
            TOPICS.cancel,
            direction="endpoint_to_lingtu",
            schema="lingtu.control.cancel.v1",
            required=False,
            note="Mission cancel command entering the LingTu navigation FSM.",
        ),
        _binding(
            TOPICS.semantic_instruction,
            direction="endpoint_to_lingtu",
            schema="lingtu.control.instruction.v1",
            required=False,
            note="Natural-language navigation instruction entering semantic planning.",
        ),
        _binding(
            TOPICS.global_path,
            direction="lingtu_to_endpoint",
            schema="lingtu.nav.path.v1",
        ),
        _binding(
            TOPICS.local_path,
            direction="lingtu_to_endpoint",
            schema="lingtu.nav.path.v1",
        ),
        _binding(
            TOPICS.nav_way_point,
            direction="lingtu_to_endpoint",
            schema="lingtu.geometry.pose_stamped.v1",
            required=False,
            note="Active navigation waypoint for endpoint visualization and supervision.",
        ),
        _binding(
            TOPICS.cmd_vel,
            direction="lingtu_to_endpoint",
            schema="lingtu.geometry.twist.v1",
            note="Muxed body-frame command after safety arbitration.",
        ),
    ),
)

_CONTRACTS: dict[str, LCMEndpointContract] = {
    THUNDER_FIELD_LCM_CONTRACT.name: THUNDER_FIELD_LCM_CONTRACT,
}


def endpoint_contract(name: str) -> LCMEndpointContract:
    """Return a named LCM endpoint contract."""

    try:
        return _CONTRACTS[name]
    except KeyError as exc:
        choices = ", ".join(endpoint_contract_names())
        raise KeyError(f"unknown LCM endpoint contract '{name}' (choices: {choices})") from exc


def endpoint_contract_names() -> tuple[str, ...]:
    """Return known LCM endpoint contract names."""

    return tuple(sorted(_CONTRACTS))


def binding_for_topic(contract_name: str, topic: str) -> LCMEndpointBinding:
    """Return the binding for a canonical topic in a contract."""

    return endpoint_contract(contract_name).binding_for_topic(topic)

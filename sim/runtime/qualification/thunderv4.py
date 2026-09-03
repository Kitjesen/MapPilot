"""Thunder V4 sensor requirements used by production qualification."""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from typing import Any

from sim.runtime.sensors.evidence import SensorEvidenceError, build_sensor_stream_summary
from sim.runtime.sensors.runtime import SensorRuntime

THUNDERV4_NAVIGATION_STREAM_IDS = (
    "thunder_01.front_depth",
    "thunder_01.front_rgb",
    "thunder_01.imu",
    "thunder_01.mid360",
    "thunder_01.truth_odom",
)

_UNREAL_NAVIGATION_CONTRACT = {
    "thunder_01.front_depth": (
        "depth",
        "visual",
        "unreal_camera",
        "camera_shm",
        "lingtu.dds.Image",
        None,
    ),
    "thunder_01.front_rgb": (
        "rgb",
        "visual",
        "unreal_camera",
        "camera_shm",
        "lingtu.dds.Image",
        None,
    ),
    "thunder_01.imu": (
        "imu",
        "physics",
        "mujoco_sensor",
        "typed_dds",
        "lingtu.dds.Imu",
        None,
    ),
    "thunder_01.mid360": (
        "mid360",
        "physics",
        "mujoco_livox_model",
        "typed_dds",
        "lingtu.dds.LivoxFrame",
        "thunder_01/lidar1_link_site",
    ),
    "thunder_01.truth_odom": (
        "truth_odom",
        "physics",
        "mujoco_truth",
        "typed_dds",
        "lingtu.dds.Odometry",
        None,
    ),
}

_HEADLESS_NAVIGATION_CONTRACT = {
    **_UNREAL_NAVIGATION_CONTRACT,
    "thunder_01.front_depth": (
        "depth",
        "sensor_runtime",
        "mujoco_camera",
        "camera_shm",
        "lingtu.dds.Image",
        None,
    ),
    "thunder_01.front_rgb": (
        "rgb",
        "sensor_runtime",
        "mujoco_camera",
        "camera_shm",
        "lingtu.dds.Image",
        None,
    ),
}

_NAVIGATION_CONTRACTS = (
    _UNREAL_NAVIGATION_CONTRACT,
    _HEADLESS_NAVIGATION_CONTRACT,
)


def build_thunderv4_navigation_stream_summary(
    plan: SensorRuntime,
    observations: Iterable[Mapping[str, Any]],
    *,
    model_generation: int,
    reset_generation: int,
    shm_allocations: Mapping[str, str],
) -> dict[str, Any]:
    """Build the exact five-stream Thunder V4 qualification view."""

    if not isinstance(plan, SensorRuntime):
        raise SensorEvidenceError("plan must be a SensorRuntime")
    actual = frozenset(stream.sensor_id for stream in plan.streams)
    expected = frozenset(THUNDERV4_NAVIGATION_STREAM_IDS)
    if actual != expected:
        missing = sorted(expected - actual)
        extra = sorted(actual - expected)
        raise SensorEvidenceError(
            "Thunder V4 navigation requires the exact five-stream set; "
            f"missing={missing}, extra={extra}"
        )
    actual_contracts = {
        stream.sensor_id: (
            stream.stream_kind,
            stream.route.owner,
            stream.route.source,
            stream.route.transport,
            stream.message_type,
            stream.raycast_frame_stable_id,
        )
        for stream in plan.streams
    }
    if not any(actual_contracts == expected for expected in _NAVIGATION_CONTRACTS):
        raise SensorEvidenceError("Thunder V4 navigation route contract mismatch")
    return build_sensor_stream_summary(
        plan,
        observations,
        model_generation=model_generation,
        reset_generation=reset_generation,
        required_stream_ids=THUNDERV4_NAVIGATION_STREAM_IDS,
        shm_allocations=shm_allocations,
    )


__all__ = [
    "THUNDERV4_NAVIGATION_STREAM_IDS",
    "build_thunderv4_navigation_stream_summary",
]

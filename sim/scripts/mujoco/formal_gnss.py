"""Optional GNSS measurements inside the Product-owned physics feeder.

This module owns neither processes nor DDS. It reads the already-stepped
MuJoCo data on the owner thread and uses the canonical GNSS measurement model.
"""

from __future__ import annotations

import math
from typing import Any, Mapping

from sim.runtime.sensors.contracts import DeadlinePolicy, ScheduledSensorSample, SensorRoute, SensorStreamPlan
from sim.runtime.sensors.gnss import GnssExtractor, GnssSample
from sim.runtime.sensors.runtime import SensorRuntime


def planned_gnss(
    plan: Mapping[str, Any],
    *,
    instance_id: str,
    body_names: set[str],
    max_hz: float | None,
) -> SensorStreamPlan | None:
    """Validate the opt-in declaration before creating an engine or connection."""
    declarations = plan.get("streams", {}).get("gnss", [])
    if declarations == []:
        return None
    if max_hz is None:
        raise ValueError("GNSS requires the Product-owned IMU sensor endpoint")
    runtime = SensorRuntime.from_plan({**plan, "streams": {"gnss": declarations}})
    if len(runtime.streams) != 1:
        raise ValueError("formal GNSS requires exactly one stream")
    stream = runtime.streams[0]
    if (
        stream.route != SensorRoute("physics", "mujoco_gnss", "typed_dds")
        or stream.message_type != "lingtu.dds.GnssFix"
    ):
        raise ValueError("GNSS requires the native physics/mujoco_gnss/typed_dds route")
    if stream.gnss is None or stream.instance_id != instance_id:
        raise ValueError("GNSS must belong to the physics robot")
    prefix = f"{instance_id}/"
    body = stream.gnss.body_frame_id
    if not body.startswith(prefix) or body[len(prefix) :] not in body_names:
        raise ValueError("GNSS source must be a declared physics robot body")
    if not stream.frame_id.startswith(prefix):
        raise ValueError("GNSS frame must belong to the physics robot")
    if float(stream.rate_hz) > max_hz:
        raise ValueError("GNSS rate cannot exceed the physics sampling rate")
    return stream


class FormalGnss:
    """Sample live body poses, never relabel a current pose as an older deadline."""

    def __init__(self, stream: SensorStreamPlan, product_session_id: str, model_generation: int, reset_generation: int):
        if stream.gnss is None:
            raise ValueError("GNSS configuration is required")
        self.stream = stream
        self.extractor = GnssExtractor(stream.gnss)
        self._session = product_session_id
        self._model_generation = model_generation
        self._reset_generation = reset_generation
        self._body = stream.gnss.body_frame_id.removeprefix(f"{stream.instance_id}/")
        self._start_ns: int | None = None
        self._last_ns = -1
        self._next_slot = 0
        self._sample_count = 0
        self._dropped = 0
        self._last: GnssSample | None = None

    def set_noise_enabled(self, enabled: bool) -> None:
        """Owner-thread control; retain the configured deviations and seed."""
        self.extractor.set_noise_enabled(enabled)

    def set_fix_available(self, available: bool) -> None:
        """Explicit scenario outage, not an inferred satellite reception model."""
        self.extractor.set_fix_available(available)

    def sample(self, engine: Any, sim_time_s: float) -> GnssSample | None:
        if not math.isfinite(sim_time_s) or sim_time_s < 0:
            raise ValueError("GNSS simulation time must be finite and non-negative")
        now = round(sim_time_s * 1_000_000_000)
        if now < self._last_ns:
            raise ValueError("GNSS simulation time moved backwards within this Product run")
        self._last_ns = now
        if self._start_ns is None:
            self._start_ns = now
        elapsed = now - self._start_ns
        if elapsed < self.stream.deadline_ns(self._next_slot):
            return None
        slot = self.stream.latest_sequence_at(elapsed)
        position, quaternion = engine.get_body_pose(self._body)
        request = ScheduledSensorSample(
            self._session,
            self.stream,
            self._model_generation,
            self._reset_generation,
            self._sample_count,
            now,
            DeadlinePolicy.DROP_INTERMEDIATE,
        )
        sample = self.extractor(
            request,
            {
                "session_id": self._session,
                "model_generation": self._model_generation,
                "reset_generation": self._reset_generation,
                "sim_time_ns": now,
                "bodies": [
                    {
                        "stable_id": self.extractor.config.body_frame_id,
                    "position_m": list(position),
                    "quaternion_wxyz": list(quaternion),
                    }
                ],
            },
        )
        self._dropped += slot - self._next_slot
        self._next_slot = slot + 1
        self._sample_count += 1
        self._last = sample
        return sample

    def evidence(self) -> dict[str, Any]:
        """Sampling evidence is not a DDS subscriber acknowledgement."""
        return {
            "sensor_id": self.stream.sensor_id,
            "frame_id": self.stream.frame_id,
            "expected_hz": float(self.stream.rate_hz),
            "sampled_count": self._sample_count,
            "missed_deadlines": self._dropped,
            "clock": "simulation",
            "delivery": "queued_on_product_imu_endpoint_not_subscriber_ack",
            "latest": None
            if self._last is None
            else {
                "sim_time_ns": self._last.stamp.sim_time_ns,
                **self._last.fidelity(),
            },
        }

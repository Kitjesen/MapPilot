"""Simulation acoustic blocks from current MuJoCo body poses, without stepping."""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace

import mujoco
import numpy as np

from .acoustics import AcousticClip, AcousticFrame, AcousticSource, MicrophoneConfig, MicrophoneSensor
from .samples import SensorSampleStamp


def _text(value: object, name: str) -> None:
    if not isinstance(value, str) or not value or value != value.strip():
        raise ValueError(f"{name} must be a non-empty trimmed string")


def _body_id(model: mujoco.MjModel, name: str) -> int:
    _text(name, "body name")
    body = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name))
    if body <= 0:
        raise ValueError(f"body {name!r} must name an existing non-world physical body")
    return body


@dataclass(frozen=True, slots=True)
class AcousticSceneSource:
    """Finite or explicitly looped Pa clip at a body-local position (metres)."""

    source_id: str
    body_name: str
    pressure_pa: np.ndarray
    sample_rate_hz: int
    start_sim_time_ns: int
    reference_distance_m: float
    local_offset_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    loop: bool = False
    _clip: AcousticClip = field(init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        if type(self.loop) is not bool:
            raise ValueError("loop must be a boolean")
        _text(self.source_id, "source_id")
        _text(self.body_name, "body_name")
        clip = AcousticClip(self.pressure_pa)
        object.__setattr__(self, "_clip", clip)
        object.__setattr__(self, "pressure_pa", clip.pressure_pa)
        offset = np.asarray(self.local_offset_m, dtype=float)
        if offset.shape != (3,) or not np.isfinite(offset).all():
            raise ValueError("local_offset_m must contain three finite metre coordinates")
        object.__setattr__(self, "local_offset_m", tuple(offset.tolist()))
        for name, minimum in (("sample_rate_hz", 1), ("start_sim_time_ns", 0)):
            value = getattr(self, name)
            if type(value) is not int or value < minimum:
                raise ValueError(f"{name} must be an integer >= {minimum}")
        radius = self.reference_distance_m
        if isinstance(radius, bool) or not isinstance(radius, (int, float)) or not math.isfinite(radius) or radius <= 0:
            raise ValueError("reference_distance_m must be finite and positive")


class AcousticScene:
    """Sample one instantaneous body-geometry observation for an acoustic block.

    Channel offsets in MicrophoneConfig are receiver-body-local here, unlike
    the world-aligned offsets accepted directly by MicrophoneSensor.

    Occlusion follows mj_ray's nearest visible-geom hit, over all geom groups
    including static bodies. Only the receiver body itself is excluded; its
    ancestors/descendants are NOT excluded. A nearest hit on the exact declared
    source body is arrival at that emitting object, not an occluder. Source
    descendants are not exempt. No traversal beyond a source-body hit, acoustic
    transmission, reflections or diffraction are modelled.
    """

    def __init__(
        self,
        config: MicrophoneConfig,
        *,
        receiver_body_name: str,
        sources: tuple[AcousticSceneSource, ...],
    ) -> None:
        self.sensor = MicrophoneSensor(config)
        _text(receiver_body_name, "receiver_body_name")
        if not isinstance(sources, (tuple, list)) or not all(
            isinstance(source, AcousticSceneSource) for source in sources
        ):
            raise ValueError("sources must contain AcousticSceneSource declarations")
        if len({source.source_id for source in sources}) != len(sources):
            raise ValueError("source_id must be unique in an acoustic scene")
        self.receiver_body_name = receiver_body_name
        self.sources = tuple(sources)

    def sample(self, model: mujoco.MjModel, data: mujoco.MjData, stamp: SensorSampleStamp) -> AcousticFrame:
        """Refresh kinematics and sample without changing qpos/qvel or time."""
        if not isinstance(stamp, SensorSampleStamp):
            raise ValueError("stamp must be a SensorSampleStamp")
        if not math.isfinite(data.time) or data.time < 0 or stamp.sim_time_ns != round(data.time * 1e9):
            raise ValueError("stamp simulation time must equal round(data.time * 1e9)")
        receiver = _body_id(model, self.receiver_body_name)
        source_bodies = tuple(_body_id(model, declaration.body_name) for declaration in self.sources)
        mujoco.mj_kinematics(model, data)
        receiver_rotation = data.xmat[receiver].reshape(3, 3)
        channels = np.asarray(self.sensor.config.channel_offsets_m) @ receiver_rotation.T + data.xpos[receiver]
        sources = []
        blocked_paths = []
        hits = []
        for declaration, body in zip(self.sources, source_bodies):
            position = data.xpos[body] + data.xmat[body].reshape(3, 3) @ declaration.local_offset_m
            sources.append(
                AcousticSource(
                    stamp=stamp,
                    source_id=declaration.source_id,
                    pressure_pa=declaration._clip,
                    reference_distance_m=declaration.reference_distance_m,
                    sample_rate_hz=declaration.sample_rate_hz,
                    start_sim_time_ns=declaration.start_sim_time_ns,
                    position_world_m=tuple(position.tolist()),
                    loop=declaration.loop,
                )
            )
            for channel, origin in enumerate(channels):
                delta = position - origin
                target_distance = float(np.linalg.norm(delta))
                if target_distance == 0:
                    continue
                geom_id = np.array([-1], dtype=np.int32)
                distance = mujoco.mj_ray(model, data, origin, delta / target_distance, None, 1, receiver, geom_id)
                if 0 <= distance < target_distance and int(model.geom_bodyid[geom_id[0]]) != body:
                    blocked_paths.append((declaration.source_id, channel))
                    hits.append(
                        {
                            "source_id": declaration.source_id,
                            "channel": channel,
                            "geom_id": int(geom_id[0]),
                            "geom_name": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(geom_id[0])),
                            "distance_m": float(distance),
                            "target_distance_m": target_distance,
                        }
                    )
        frame = self.sensor.sample(
            stamp,
            tuple(sources),
            channel_positions_world_m=tuple(map(tuple, channels.tolist())),
            blocked_paths=tuple(blocked_paths),
        )
        return replace(
            frame,
            metadata={
                **frame.metadata,
                "scene": {
                    "pose_source": "mujoco_mj_kinematics",
                    "receiver_body_name": self.receiver_body_name,
                    "receiver_body_id": receiver,
                    "sim_time_ns": stamp.sim_time_ns,
                    "receiver_exclusion": "exact_body_only_not_subtree",
                    "source_geom_policy": "nearest_hit_on_exact_source_body_is_arrival_not_occlusion",
                    "ray_geometry": "mujoco_visible_geoms_all_groups_static_and_dynamic",
                    "source_mounts": [
                        {
                            "source_id": declaration.source_id,
                            "body_name": declaration.body_name,
                            "body_id": body,
                            "local_offset_m": list(declaration.local_offset_m),
                        }
                        for declaration, body in zip(self.sources, source_bodies)
                    ],
                    "occlusion_hits": hits,
                },
            },
        )

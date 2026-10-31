"""Simulation-only pressure sampling of finite or explicit cyclic sources along direct paths.

This is not a room acoustic simulator: no reflections, diffraction, air
absorption, transducer frequency response or playback are modelled. Geometry is
held fixed within each block. Waveforms specify pressure at a reference radius;
propagation time is measured from the source position, not that radius.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field, replace
from numbers import Real
from typing import cast

import numpy as np

from .measurement_control import MeasurementNoise
from .samples import SensorSampleStamp


def _integer(value: object, name: str, minimum: int = 1) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ValueError(f"{name} must be an integer >= {minimum}")


def _positive(value: object, name: str) -> None:
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be positive and finite")


def _array(value: object, name: str) -> np.ndarray:
    raw = np.asarray(value)
    if raw.dtype.kind not in "iuf" or not np.isfinite(raw).all():
        raise ValueError(f"{name} must contain finite numeric values")
    return raw.astype(np.float64, copy=True)


def _positions(value: object, name: str, channels: int | None = None) -> np.ndarray:
    array = _array(value, name)
    if array.ndim != 2 or array.shape[1] != 3 or len(array) == 0:
        raise ValueError(f"{name} must have shape [channels, 3] with at least one channel")
    if channels is not None and len(array) != channels:
        raise ValueError(f"{name} must have {channels} channels")
    return array


@dataclass(frozen=True, slots=True)
class AcousticClip:
    """Validated mono pressure snapshot backed by immutable bytes, safe to share."""

    pressure_pa: np.ndarray

    def __post_init__(self) -> None:
        pressure = _array(self.pressure_pa, "pressure_pa")
        if pressure.ndim != 1 or not len(pressure):
            raise ValueError("pressure_pa must be a non-empty mono waveform")
        object.__setattr__(self, "pressure_pa", np.frombuffer(pressure.tobytes(), dtype=np.float64))


@dataclass(frozen=True, slots=True)
class AcousticSource:
    """Finite mono pressure waveform (Pa) on the source's simulation-time grid.

    ``stamp`` is the source-position observation, not the waveform start. It
    must match the receiving block's session, generations and first-sample time.
    Supply a new observation for each block, retaining waveform start time.
    Support is [start, start + len(pressure_pa) / sample_rate_hz): linear
    interpolation reaches zero one sample after the last supplied value.
    Times before the explicit start and after that support are known silence.
    Explicit loop=True instead repeats the clip after start, interpolating the
    last sample to the first. Delay is applied before wrapping; no pre-start sound.
    """

    stamp: SensorSampleStamp
    source_id: str
    pressure_pa: np.ndarray | AcousticClip
    reference_distance_m: float
    sample_rate_hz: int
    start_sim_time_ns: int
    position_world_m: tuple[float, float, float]
    loop: bool = False

    def __post_init__(self) -> None:
        if not isinstance(self.stamp, SensorSampleStamp):
            raise ValueError("source stamp must be a SensorSampleStamp")
        if type(self.loop) is not bool:
            raise ValueError("loop must be a boolean")
        if not isinstance(self.source_id, str) or not self.source_id or self.source_id.strip() != self.source_id:
            raise ValueError("source_id must be a non-empty trimmed string")
        _positive(self.reference_distance_m, "reference_distance_m")
        _integer(self.sample_rate_hz, "sample_rate_hz")
        _integer(self.start_sim_time_ns, "start_sim_time_ns", minimum=0)
        if isinstance(self.pressure_pa, AcousticClip):
            pressure = self.pressure_pa.pressure_pa
        else:
            pressure = _array(self.pressure_pa, "pressure_pa")
            if pressure.ndim != 1 or not len(pressure):
                raise ValueError("pressure_pa must be a non-empty mono waveform")
            pressure.setflags(write=False)
        object.__setattr__(self, "pressure_pa", pressure)
        position = _array(self.position_world_m, "position_world_m")
        if position.shape != (3,):
            raise ValueError("position_world_m must contain three coordinates")
        object.__setattr__(self, "position_world_m", tuple(position.tolist()))


@dataclass(frozen=True, slots=True)
class MicrophoneConfig:
    """Integer-Hz sample grid anchored at sim time zero; Pa RMS Gaussian noise.

    Noise uses fixed 1024-sample tiles keyed by the shared noise generator and
    absolute sample index, so block size and scheduling sequence do not change
    a sample's noise. Calibration is optional but must be explicit for volts.
    """

    sample_rate_hz: int
    block_samples: int
    channel_offsets_m: tuple[tuple[float, float, float], ...]
    speed_of_sound_mps: float = 343.0
    sensitivity_volts_per_pa: float | None = None
    calibration_metadata: dict | None = None
    noise: MeasurementNoise = field(default_factory=lambda: MeasurementNoise(unit="Pa"))

    def __post_init__(self) -> None:
        _integer(self.sample_rate_hz, "sample_rate_hz")
        if self.sample_rate_hz > 1_000_000_000:
            raise ValueError("sample_rate_hz cannot exceed the nanosecond clock resolution")
        _integer(self.block_samples, "block_samples")
        _positive(self.speed_of_sound_mps, "speed_of_sound_mps")
        offsets = _positions(self.channel_offsets_m, "channel_offsets_m")
        object.__setattr__(self, "channel_offsets_m", tuple(tuple(row) for row in offsets.tolist()))
        if not isinstance(self.noise, MeasurementNoise) or self.noise.unit != "Pa":
            raise ValueError("microphone noise must be MeasurementNoise with unit Pa")
        if self.sensitivity_volts_per_pa is not None:
            _positive(self.sensitivity_volts_per_pa, "sensitivity_volts_per_pa")
            if not isinstance(self.calibration_metadata, dict) or not self.calibration_metadata:
                raise ValueError("sensitivity_volts_per_pa requires explicit calibration_metadata object")
        elif self.calibration_metadata is not None:
            raise ValueError("calibration_metadata requires sensitivity_volts_per_pa")
        try:
            calibration = json.loads(json.dumps(self.calibration_metadata, allow_nan=False))
        except (TypeError, ValueError) as exc:
            raise ValueError("calibration_metadata must contain finite JSON data") from exc
        object.__setattr__(self, "calibration_metadata", calibration)


@dataclass(frozen=True, slots=True)
class AcousticFrame:
    """Pressure Pa and per-sample validity, both shaped [channel, sample].

    ``stamp`` identifies the first sample. Source support outside the finite
    waveform is known silence, not missing data.
    """

    stamp: SensorSampleStamp
    pressure_pa: np.ndarray
    valid_mask: np.ndarray
    sample_rate_hz: int
    metadata: dict
    voltage_v: np.ndarray | None = None


class MicrophoneSensor:
    def __init__(self, config: MicrophoneConfig):
        if not isinstance(config, MicrophoneConfig):
            raise ValueError("config must be a MicrophoneConfig")
        self.config = config

    def sample(
        self,
        stamp: SensorSampleStamp,
        sources: tuple[AcousticSource, ...],
        *,
        position_world_m: tuple[float, float, float] = (0.0, 0.0, 0.0),
        channel_positions_world_m: tuple[tuple[float, float, float], ...] | None = None,
        blocked_paths: tuple[tuple[str, int], ...] = (),
    ) -> AcousticFrame:
        """Sample fixed block geometry; offsets are world-aligned, not body-rotated.

        Rotating mounts must supply already transformed channel world positions.
        The caller supplies receiver geometry at ``stamp``. Source observations
        are checked against that time and generation. Blocked direct paths
        invalidate the entire affected channel (NaN pressure, false validity),
        even when other sources contribute: their sum is not the full signal.
        """
        config = self.config
        if not isinstance(stamp, SensorSampleStamp):
            raise ValueError("stamp must be a SensorSampleStamp")
        origin = _array(position_world_m, "position_world_m")
        if origin.shape != (3,):
            raise ValueError("position_world_m must contain three coordinates")
        positions = (
            np.asarray(config.channel_offsets_m) + origin
            if channel_positions_world_m is None
            else _positions(channel_positions_world_m, "channel_positions_world_m", len(config.channel_offsets_m))
        )
        if not isinstance(sources, (tuple, list)) or not all(isinstance(source, AcousticSource) for source in sources):
            raise ValueError("sources must contain AcousticSource objects with explicit metadata")
        source_ids = {source.source_id for source in sources}
        if len(source_ids) != len(sources):
            raise ValueError("source_id must be unique within a block")
        identity_fields = ("session_id", "model_generation", "reset_generation", "sim_time_ns")
        for source in sources:
            if any(getattr(source.stamp, name) != getattr(stamp, name) for name in identity_fields):
                raise ValueError("source geometry stamp must match receiver session, generations and source time")
        for source_id, channel in blocked_paths:
            if source_id not in source_ids:
                raise ValueError("blocked path source_id must name a supplied source")
            _integer(channel, "blocked path channel", minimum=0)
            if channel >= len(positions):
                raise ValueError("blocked path channel is outside the microphone array")
        pressure = np.zeros((len(positions), config.block_samples), dtype=np.float64)
        valid = np.ones(pressure.shape, dtype=bool)
        first_sample_index = (stamp.sim_time_ns * config.sample_rate_hz + 999_999_999) // 1_000_000_000
        if first_sample_index * 1_000_000_000 // config.sample_rate_hz != stamp.sim_time_ns:
            raise ValueError("stamp.sim_time_ns must be on the sample grid (floor(index * 1e9 / rate))")
        sample_indices = first_sample_index + np.arange(config.block_samples, dtype=np.int64)
        for source in sources:
            distances = np.linalg.norm(positions - source.position_world_m, axis=1)
            for channel, distance in enumerate(distances):
                if (source.source_id, channel) in blocked_paths:
                    valid[channel] = False
                    continue
                indices = (
                    sample_indices * (source.sample_rate_hz / config.sample_rate_hz)
                    - source.start_sim_time_ns * source.sample_rate_hz / 1e9
                    - distance * source.sample_rate_hz / config.speed_of_sound_mps
                )
                # Gather only this block's neighbours; the final neighbour is
                # the implicit zero sample, with no fade before explicit start.
                # __post_init__ normalizes either constructor input to ndarray.
                waveform = cast(np.ndarray, source.pressure_pa)
                inside = indices >= 0 if source.loop else (indices >= 0) & (indices < len(waveform))
                active_indices = indices[inside]
                if source.loop:
                    active_indices = np.remainder(active_indices, len(waveform))
                lower = np.floor(active_indices).astype(np.intp)
                left = waveform[lower]
                if source.loop:
                    right = waveform[(lower + 1) % len(waveform)]
                else:
                    right = np.zeros_like(left)
                    has_right = lower + 1 < len(waveform)
                    right[has_right] = waveform[lower[has_right] + 1]
                interpolated = np.zeros_like(indices)
                interpolated[inside] = left + (right - left) * (active_indices - lower)
                pressure[channel] += (
                    interpolated * source.reference_distance_m / max(distance, source.reference_distance_m)
                )
        if config.noise.enabled and config.noise.stddev:
            tile_samples = 1024
            end_index = first_sample_index + config.block_samples
            for tile_start in range(first_sample_index // tile_samples * tile_samples, end_index, tile_samples):
                tile_stamp = replace(
                    stamp,
                    sequence=tile_start // tile_samples,
                    sim_time_ns=tile_start * 1_000_000_000 // config.sample_rate_hz,
                )
                tile = (
                    config.noise.generator(tile_stamp)
                    .normal(
                        0.0,
                        config.noise.stddev,
                        size=(tile_samples, len(positions)),
                    )
                    .T
                )
                start, end = max(first_sample_index, tile_start), min(end_index, tile_start + tile_samples)
                pressure[:, start - first_sample_index : end - first_sample_index] += tile[
                    :, start - tile_start : end - tile_start
                ]
        pressure[~valid] = np.nan
        voltage = None if config.sensitivity_volts_per_pa is None else pressure * config.sensitivity_volts_per_pa
        for values in (pressure, voltage):
            if values is not None and (
                not np.isfinite(values[valid]).all() or (np.abs(values[valid]) > np.finfo(np.float32).max).any()
            ):
                raise ValueError("measurement exceeds finite float32 range")
        return AcousticFrame(
            stamp=stamp,
            pressure_pa=pressure.astype(np.float32),
            valid_mask=valid,
            sample_rate_hz=config.sample_rate_hz,
            metadata={
                "simulation": True,
                "pressure_unit": "Pa",
                "propagation_model": "direct_path_only",
                "provenance": {
                    "model_id": "direct_path_pressure",
                    "model_version": "1",
                    "qualification": "EXPERIMENTAL",
                },
                "near_field_policy": "gain_capped_at_reference_radius",
                "first_sample_index": first_sample_index,
                "sample_grid_origin_sim_time_ns": 0,
                "blocked_paths": [{"source_id": source_id, "channel": channel} for source_id, channel in blocked_paths],
                "sensitivity_volts_per_pa": config.sensitivity_volts_per_pa,
                "calibration": json.loads(json.dumps(config.calibration_metadata, allow_nan=False)),
                "noise": config.noise.to_dict(),
                "noise_time_basis": "absolute_sample_grid_1024_sample_tiles",
                "speed_of_sound_mps": config.speed_of_sound_mps,
                "channel_positions_world_m": positions.tolist(),
                "geometry_policy": "fixed_during_block",
                "interpolation": "linear_per_source_support"
                if any(s.loop for s in sources)
                else "linear_zero_extended_samples",
                "sources": [
                    {
                        "source_id": source.source_id,
                        "reference_distance_m": source.reference_distance_m,
                        "sample_rate_hz": source.sample_rate_hz,
                        "start_sim_time_ns": source.start_sim_time_ns,
                        "waveform_samples": len(cast(np.ndarray, source.pressure_pa)),
                        "loop": source.loop,
                        "position_world_m": list(source.position_world_m),
                        "source_sim_time_ns": source.stamp.sim_time_ns,
                    }
                    for source in sources
                ],
            },
            voltage_v=None if voltage is None else voltage.astype(np.float32),
        )

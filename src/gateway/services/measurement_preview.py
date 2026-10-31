"""Bounded display projections; full-resolution measurements stay in raw NPZ."""

from __future__ import annotations

from itertools import pairwise
from typing import Any

import numpy as np

from gateway.services.measurements import Measurement
from runtime.msgs.measurement import AcousticMeasurement, ThermalMeasurement


def _acoustic_preview(sample: AcousticMeasurement) -> dict[str, Any]:
    pressure = sample.pressure_pa
    valid = sample.valid_mask & np.isfinite(pressure)
    channels, count = pressure.shape
    edges = np.linspace(0, count, min(count, 256) + 1, dtype=int)
    low: list[list[float | None]] = [[] for _ in range(channels)]
    high: list[list[float | None]] = [[] for _ in range(channels)]
    fractions: list[list[float]] = [[] for _ in range(channels)]
    for start, end in pairwise(edges):
        for channel in range(channels):
            mask = valid[channel, start:end]
            values = pressure[channel, start:end][mask]
            low[channel].append(float(values.min()) if len(values) else None)
            high[channel].append(float(values.max()) if len(values) else None)
            fractions[channel].append(float(mask.mean()))
    return {
        "kind": "acoustic",
        "display_only": True,
        "sampling": "min_max_envelope",
        "sample_rate_hz": sample.sample_rate_hz,
        "source_sample_count": count,
        "bin_start_samples": edges[:-1].tolist(),
        "bin_end_samples": edges[1:].tolist(),
        "pressure_min_pa": low,
        "pressure_max_pa": high,
        "valid_fraction": fractions,
    }


def measurement_preview(sample: Measurement) -> dict[str, Any]:
    """Retain measured units and explicit invalid pixels, never infer a result."""
    if isinstance(sample, AcousticMeasurement):
        return _acoustic_preview(sample)
    height, width = sample.valid_mask.shape
    rows = np.linspace(0, height - 1, min(height, 144), dtype=int)
    columns = np.linspace(0, width - 1, min(width, 192), dtype=int)
    selected = np.ix_(rows, columns)
    valid = sample.valid_mask[selected].copy()
    image: dict[str, Any] = {
        "source_width": width,
        "source_height": height,
        "width": len(columns),
        "height": len(rows),
        "row_indices": rows.tolist(),
        "column_indices": columns.tolist(),
        "sampling": "nearest_source_pixel",
        "display_only": True,
    }
    if isinstance(sample, ThermalMeasurement):
        values = sample.temperature_k[selected]
        valid &= np.isfinite(values)
        image.update(kind="thermal", temperature_k=_nullable_pixels(values, valid))
    else:
        classes, instances = sample.class_ids[selected], sample.instance_ids[selected]
        valid &= np.isfinite(classes) & np.isfinite(instances)
        image.update(
            kind="semantic",
            ground_truth=True,
            class_ids=_nullable_pixels(classes, valid),
            instance_ids=_nullable_pixels(instances, valid),
        )
    image["valid_mask"] = valid.tolist()
    return image


def _nullable_pixels(values: np.ndarray, valid: np.ndarray) -> list:
    nullable = values.astype(object)
    nullable[~valid] = None
    return list(nullable.tolist())

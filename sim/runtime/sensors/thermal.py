"""Simulation-only per-pixel broadband greybody thermal measurements.

For opaque, diffuse grey surfaces the normalized broadband radiance is
R = tau * (epsilon * Ts**4 + (1-epsilon) * Tref**4)
    + (1-tau) * Tatm**4.
The common Stefan-Boltzmann / pi factor cancels when recovering brightness
temperature R**0.25. This is NOT a calibrated band-limited LWIR response.
The emission/reflection/path decomposition follows FLIR's measurement formula:
https://support.flir.com/docdownload/assets/web/27eh/en-us/T505006.xml.html
The T**4 substitution, uniform backgrounds and constant path transmittance
are explicit simulation assumptions, not a detector calibration.

Geometry must be nearest-visible physical geometry, never UI/render truth.
The caller owns raycasting and passes its exact source stamp. Distances only
validate hits; no inverse-square falloff is applied to resolved radiance.
"""

from __future__ import annotations

import json
import math
from collections.abc import Mapping
from dataclasses import asdict, dataclass, field
from types import MappingProxyType
from typing import Any

import numpy as np

from .measurement_control import MeasurementNoise
from .samples import SensorSampleStamp


def _finite(value: Any, name: str) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be finite")


def _temperature(value: Any, name: str) -> None:
    _finite(value, name)
    if not 0 < value <= float(np.finfo(np.float32).max):
        raise ValueError(f"{name} must be positive and float32 representable")


@dataclass(frozen=True, slots=True)
class ThermalSurface:
    """Declared simulation surface properties, not inferred heat dynamics."""

    temperature_k: float
    emissivity: float

    def __post_init__(self) -> None:
        _temperature(self.temperature_k, "temperature_k")
        _finite(self.emissivity, "emissivity")
        if not 0 <= self.emissivity <= 1:
            raise ValueError("emissivity must be in [0, 1]")


@dataclass(frozen=True, slots=True)
class ThermalConfig:
    """Explicit spatial calibration plus declared simulation radiometry.

    Camera JSON requires model, width, height and calibration_id. Pinhole uses
    fx/fy/cx/cy in pixels; other models carry their own nonempty parameters.
    Temperature bounds apply before and after output-domain Kelvin noise;
    out-of-range values are invalidated, never saturated into valid pixels.
    """

    camera_calibration: Mapping[str, Any]
    background_temperature_k: float
    transmittance: float = 1.0
    atmospheric_temperature_k: float = 293.15
    temperature_mode: str = "apparent"
    min_temperature_k: float = 1.0
    max_temperature_k: float = 2000.0
    noise: MeasurementNoise = field(default_factory=lambda: MeasurementNoise(unit="K"))
    _calibration_json: str = field(init=False, repr=False)

    def __post_init__(self) -> None:
        if not isinstance(self.noise, MeasurementNoise) or self.noise.unit != "K":
            raise ValueError("thermal noise must be MeasurementNoise with unit K")
        for name in ("background_temperature_k", "atmospheric_temperature_k", "min_temperature_k", "max_temperature_k"):
            _temperature(getattr(self, name), name)
        if self.min_temperature_k >= self.max_temperature_k:
            raise ValueError("min_temperature_k must be below max_temperature_k")
        _finite(self.transmittance, "transmittance")
        if not 0 <= self.transmittance <= 1:
            raise ValueError("transmittance must be in [0, 1]")
        if self.temperature_mode not in {"apparent", "surface_corrected"}:
            raise ValueError("temperature_mode must be apparent or surface_corrected")
        if not isinstance(self.camera_calibration, Mapping):
            raise ValueError("camera_calibration must be an explicit JSON mapping")
        camera = dict(self.camera_calibration)
        for name in ("model", "calibration_id"):
            if not isinstance(camera.get(name), str) or not camera[name].strip():
                raise ValueError(f"camera_calibration requires {name}")
        for name in ("width", "height"):
            if type(camera.get(name)) is not int or camera[name] <= 0:
                raise ValueError(f"camera_calibration requires positive integer {name}")
        if camera["model"] == "pinhole":
            for name in ("fx", "fy", "cx", "cy"):
                _finite(camera.get(name), f"camera_calibration.{name}")
            if camera["fx"] <= 0 or camera["fy"] <= 0:
                raise ValueError("camera_calibration focal lengths must be positive")
        elif not isinstance(camera.get("parameters"), Mapping) or not camera["parameters"]:
            raise ValueError("other camera models require explicit parameters")
        try:
            encoded = json.dumps(camera, allow_nan=False)
        except (ValueError, TypeError) as exc:
            raise ValueError("camera_calibration must contain finite JSON values") from exc
        object.__setattr__(self, "_calibration_json", encoded)
        object.__setattr__(self, "camera_calibration", MappingProxyType(json.loads(encoded)))


@dataclass(frozen=True, slots=True)
class ThermalFrame:
    stamp: SensorSampleStamp
    temperature_k: np.ndarray
    valid_mask: np.ndarray
    metadata: dict[str, Any]


class ThermalSensor:
    """Array model; physical ray acquisition is owned by the runtime."""

    def __init__(self, config: ThermalConfig, surfaces: Mapping[int, ThermalSurface]):
        if not isinstance(config, ThermalConfig):
            raise ValueError("config must be a ThermalConfig")
        if not isinstance(surfaces, Mapping) or any(
            type(key) is not int or key < 0 or not isinstance(value, ThermalSurface) for key, value in surfaces.items()
        ):
            raise ValueError("surfaces must map non-negative physical geom IDs to ThermalSurface")
        self.config = config
        self.surfaces = MappingProxyType(dict(surfaces))

    def measure(
        self,
        stamp: SensorSampleStamp,
        *,
        source_stamp: SensorSampleStamp,
        geom_ids: np.ndarray,
        distances_m: np.ndarray,
        valid_rays: np.ndarray,
    ) -> ThermalFrame:
        """Measure one HxW nearest-hit raster from exactly ``source_stamp``.

        Negative/unknown IDs, nonpositive/nonfinite distances and invalid rays
        remain NaN. No search through an unknown foreground surface is made.
        Surface correction cannot recover epsilon=0 or tau=0 observations.
        Inputs are not mutated; output arrays are independently owned/read-only.
        """

        if not isinstance(stamp, SensorSampleStamp) or source_stamp != stamp:
            raise ValueError("source_stamp must match the requested stamp exactly")
        geom_ids, distances_m, valid_rays = map(np.asarray, (geom_ids, distances_m, valid_rays))
        camera = json.loads(self.config._calibration_json)
        shape = (camera["height"], camera["width"])
        if any(array.shape != shape for array in (geom_ids, distances_m, valid_rays)):
            raise ValueError("raster shape must match camera calibration height/width")
        if not np.issubdtype(geom_ids.dtype, np.integer) or valid_rays.dtype != np.bool_:
            raise ValueError("geom_ids must be integer and valid_rays must be boolean")
        if distances_m.dtype.kind not in "fiu":
            raise ValueError("distances_m must be real numeric distances")
        result = np.full(geom_ids.shape, np.nan, dtype=np.float64)
        valid = np.zeros(geom_ids.shape, dtype=bool)
        config = self.config
        for geom_id, surface in self.surfaces.items():
            selected = (geom_ids == geom_id) & valid_rays & np.isfinite(distances_m) & (distances_m > 0)
            scale = max(surface.temperature_k, config.background_temperature_k, config.atmospheric_temperature_k)
            background = (config.background_temperature_k / scale) ** 4
            atmosphere = (config.atmospheric_temperature_k / scale) ** 4
            radiance = surface.emissivity * (surface.temperature_k / scale) ** 4 + (1 - surface.emissivity) * background
            received = config.transmittance * radiance + (1 - config.transmittance) * atmosphere
            if config.temperature_mode == "surface_corrected":
                if surface.emissivity == 0 or config.transmittance == 0:
                    continue
                # The simulated forward/inverse greybody maps cancel exactly.
                # Avoid cancellation error at inclusive measurement bounds.
                temperature = surface.temperature_k
            else:
                temperature = received**0.25 * scale if received > 0 else math.nan
            if not config.min_temperature_k <= temperature <= config.max_temperature_k:
                continue
            result[selected] = temperature
            valid[selected] = True
        if config.noise.enabled and config.noise.stddev > 0:
            with np.errstate(over="ignore", invalid="ignore"):
                result[valid] += config.noise.generator(stamp).normal(0, config.noise.stddev, size=shape)[valid]
        valid &= np.isfinite(result) & (result >= config.min_temperature_k) & (result <= config.max_temperature_k)
        result[~valid] = np.nan
        result = result.astype(np.float32)
        result.setflags(write=False)
        valid.setflags(write=False)
        kind = "apparent_brightness" if config.temperature_mode == "apparent" else "emissivity_corrected_surface"
        return ThermalFrame(
            stamp,
            result,
            valid,
            {
                "env": "sim",
                "qualification": "EXPERIMENTAL",
                "temperature_kind": kind,
                "temperature_unit": "K",
                "truth_model": "declared_surface_broadband_greybody",
                "provenance": {
                    "model_id": "declared_surface_broadband_greybody",
                    "model_version": "1",
                    "qualification": "EXPERIMENTAL",
                },
                "calibrated_lwir": False,
                "camera_calibration": camera,
                "source_stamp": asdict(source_stamp),
                "geometry_source": "nearest_visible_physical_geom_id_raster",
                "surface_properties": [
                    {"geom_id": geom_id, **asdict(surface)}
                    for geom_id, surface in sorted(self.surfaces.items())
                ],
                "background_temperature_k": config.background_temperature_k,
                "atmospheric_temperature_k": config.atmospheric_temperature_k,
                "transmittance": config.transmittance,
                "valid_temperature_bounds_k": [config.min_temperature_k, config.max_temperature_k],
                "noise": asdict(config.noise),
                "noise_domain": "reported_temperature_k_after_radiometric_inversion",
                "normalized_radiance_formula": "tau*(epsilon*Ts^4+(1-epsilon)*Tref^4)+(1-tau)*Tatm^4",
                "invalid_pixel_semantics": "NaN_temperature_and_false_mask_no_background_fill",
                "calibration_assumptions": [
                    "Broadband Stefan-Boltzmann T^4 greybody; no spectral detector response or LWIR calibration",
                    "Opaque diffuse surfaces; spatially uniform reflected and atmospheric backgrounds",
                    "Declared constant transmittance; distances validate hits only, no inverse-square radiance falloff",
                    "No heat transport, subpixel mixing, optics PSF, detector drift or thermal evolution model",
                    "Surface correction uses declared emissivity/background/path, not independently measured truth",
                ],
            },
        )

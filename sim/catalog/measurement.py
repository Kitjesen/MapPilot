"""Build-time measurement declarations; standard library only, no engine access."""

from __future__ import annotations

import json
import math
import re
from collections.abc import Mapping
from fractions import Fraction
from typing import Any

_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_./:-]*$")
_FLOAT32_MAX = 3.4028234663852886e38


def _object(value: Any, required: set[str], optional: set[str], name: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be an object")
    if missing := required - value.keys():
        raise ValueError(f"{name} missing fields: {sorted(missing)}")
    if unknown := value.keys() - required - optional:
        raise ValueError(f"{name} unknown fields: {sorted(map(str, unknown))}")
    return dict(value)


def _name(value: Any, name: str) -> None:
    if not isinstance(value, str) or _NAME.fullmatch(value) is None:
        raise ValueError(f"{name} must be a nonempty trimmed physical name")


def _number(value: Any, name: str, minimum: float = -math.inf, maximum: float = math.inf,
            *, exclusive: bool = False) -> None:
    if (isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)
            or value > maximum or value < minimum or (exclusive and value == minimum)):
        raise ValueError(f"{name} must be finite numeric data in the declared range")


def _integer(value: Any, name: str, minimum: int = 1, maximum: int | None = None) -> None:
    if type(value) is not int or value < minimum or (maximum is not None and value > maximum):
        raise ValueError(f"{name} must be an integer in the declared range")


def _vector(value: Any, size: int, name: str) -> None:
    if not isinstance(value, list) or len(value) != size:
        raise ValueError(f"{name} must contain {size} finite numbers")
    for item in value:
        _number(item, name)


def validate_scene(value: Any, *, geom_names: set[str], body_names: set[str]) -> dict[str, Any]:
    """Validate WorldPackage-owned properties against its directly resolved MJCF."""
    scene = _object(value, set(), {"thermal_surfaces", "semantic_labels", "acoustic_sources"}, "measurements")
    instance_classes: dict[int, int] = {}
    for group in ("thermal_surfaces", "semantic_labels"):
        declarations = scene.get(group, {})
        if not isinstance(declarations, Mapping):
            raise ValueError(f"measurements.{group} must be an object")
        for geom, values in declarations.items():
            context = f"measurements.{group}.{geom}"
            _name(geom, context)
            if geom not in geom_names:
                raise ValueError(f"{context} is not a world MJCF geom")
            if group == "thermal_surfaces":
                surface = _object(values, {"temperature_k", "emissivity"}, set(), context)
                _number(surface["temperature_k"], f"{context}.temperature_k", 0, _FLOAT32_MAX, exclusive=True)
                _number(surface["emissivity"], f"{context}.emissivity", 0, 1)
            else:
                label = _object(values, {"class_id", "instance_id"}, set(), context)
                for key in label:
                    _integer(label[key], f"{context}.{key}", maximum=2**31 - 1)
                instance, class_id = label["instance_id"], label["class_id"]
                if instance_classes.get(instance, class_id) != class_id:
                    raise ValueError("measurements.semantic_labels must assign one class per instance")
                instance_classes[instance] = class_id
    sources = scene.get("acoustic_sources", [])
    if not isinstance(sources, list):
        raise ValueError("measurements.acoustic_sources must be a list")
    seen: set[str] = set()
    for value in sources:
        context = "measurements.acoustic_sources"
        source = _object(value, {"source_id", "body_name", "pressure_pa", "sample_rate_hz",
                                 "start_sim_time_ns", "reference_distance_m"}, {"local_offset_m", "loop"}, context)
        if "loop" in source and type(source["loop"]) is not bool:
            raise ValueError(f"{context}.loop must be a boolean")
        for key in ("source_id", "body_name"):
            _name(source[key], f"{context}.{key}")
        if source["body_name"] not in body_names:
            raise ValueError(f"{context}.body_name must name a world MJCF body")
        if source["source_id"] in seen:
            raise ValueError(f"{context}.source_id must be unique")
        seen.add(source["source_id"])
        pressure = source["pressure_pa"]
        if not isinstance(pressure, list) or not pressure:
            raise ValueError(f"{context}.pressure_pa must be a finite nonempty mono waveform list")
        for sample in pressure:
            _number(sample, f"{context}.pressure_pa")
        _integer(source["sample_rate_hz"], f"{context}.sample_rate_hz")
        _integer(source["start_sim_time_ns"], f"{context}.start_sim_time_ns", 0)
        _number(source["reference_distance_m"], f"{context}.reference_distance_m", 0, exclusive=True)
        if "local_offset_m" in source:
            _vector(source["local_offset_m"], 3, f"{context}.local_offset_m")
    return scene


def _camera(value: Any) -> None:
    name = "measurement.camera"
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be an object")
    model = value.get("model")
    fields = {
        "pinhole": ({"fx_px", "fy_px", "cx_px", "cy_px"}, set()),
        "equidistant_fisheye": ({"focal_px", "cx_px", "cy_px", "fov_deg", "valid_radius_px"}, set()),
        "equirectangular": (set(), {"longitude_min_deg", "longitude_max_deg", "latitude_min_deg", "latitude_max_deg"}),
    }
    if not isinstance(model, str) or model not in fields:
        raise ValueError(f"{name}.model must name a supported calibrated projection")
    required, optional = fields[model]
    camera = _object(value, required | {"model", "width", "height"}, optional, name)
    for key in ("width", "height"):
        _integer(camera[key], f"{name}.{key}")
    for key in required | optional:
        if key in camera:
            _number(camera[key], f"{name}.{key}")
    for key in {"fx_px", "fy_px", "focal_px", "valid_radius_px"} & camera.keys():
        _number(camera[key], f"{name}.{key}", 0, exclusive=True)
    if model == "equidistant_fisheye" and not 0 < camera["fov_deg"] < 360:
        raise ValueError(f"{name}.fov_deg must be in (0, 360)")
    if model == "equirectangular":
        lon_min, lon_max = camera.get("longitude_min_deg", -180), camera.get("longitude_max_deg", 180)
        lat_min, lat_max = camera.get("latitude_min_deg", -90), camera.get("latitude_max_deg", 90)
        if not 0 < lon_max - lon_min <= 360 or not -90 <= lat_min < lat_max <= 90:
            raise ValueError(f"{name} panorama bounds must be ordered with valid spans")


def camera_projection_preset(camera: Mapping[str, Any], model: str) -> dict[str, Any]:
    """Explicit software projection presets at the installed camera resolution."""
    if model not in {"pinhole", "equidistant_fisheye", "equirectangular"}:
        raise ValueError("camera_preset must name a supported calibrated projection")
    if model == camera["model"]:
        return dict(camera)
    width, height = camera["width"], camera["height"]
    result = dict(model=model, width=width, height=height)
    if model == "pinhole":
        result.update(fx_px=width / 2, fy_px=width / 2, cx_px=width / 2, cy_px=height / 2)
    elif model == "equidistant_fisheye":
        radius = min(width, height) / 2
        result.update(focal_px=radius / (math.pi / 2), cx_px=width / 2, cy_px=height / 2,
                      fov_deg=180.0, valid_radius_px=radius)
    else:
        result.update(longitude_min_deg=-180.0, longitude_max_deg=180.0,
                      latitude_min_deg=-90.0, latitude_max_deg=90.0)
    _camera(result)
    return result


def validate_configuration(value: Any, kind: str, rate_hz: int | float) -> dict[str, Any]:
    """Validate only the current measurement_configuration factory's input fields.

    Cross-field constraints stay here; JSON schemas describe strict names and
    scalar ranges. Catalog supplies mount_pose after this input validation.
    """
    required = {"body_frame_id", "microphone" if kind == "acoustic" else "camera"}
    optional = {"noise"}
    if kind != "acoustic":
        optional |= {"gimbal", "max_distance_m"}
    if kind == "thermal":
        required.add("thermal")
    config = _object(value, required, optional, "measurement")
    _name(config["body_frame_id"], "measurement.body_frame_id")
    unit = {"thermal": "K", "semantic": "label", "acoustic": "Pa"}[kind]
    noise = _object(config.get("noise", {"unit": unit}), {"unit"}, {"enabled", "stddev", "seed"}, "measurement.noise")
    if noise["unit"] != unit:
        raise ValueError(f"measurement.noise.unit must be {unit}")
    if type(noise.get("enabled", False)) is not bool:
        raise ValueError("measurement.noise.enabled must be boolean")
    _number(noise.get("stddev", 0), "measurement.noise.stddev", 0)
    _integer(noise.get("seed", 0), "measurement.noise.seed", 0, 2**64 - 1)
    if kind == "semantic" and noise.get("enabled", False):
        raise ValueError("measurement.noise must be disabled for semantic labels")
    if kind == "acoustic":
        name = "measurement.microphone"
        microphone = _object(config["microphone"], {"sample_rate_hz", "block_samples", "channel_offsets_m"},
                             {"speed_of_sound_mps", "sensitivity_volts_per_pa", "calibration_metadata"}, name)
        _integer(microphone["sample_rate_hz"], f"{name}.sample_rate_hz", maximum=10**9)
        _integer(microphone["block_samples"], f"{name}.block_samples")
        if Fraction(str(rate_hz)) != Fraction(microphone["sample_rate_hz"], microphone["block_samples"]):
            raise ValueError(f"{name} cadence must equal sample_rate_hz / block_samples")
        offsets = microphone["channel_offsets_m"]
        if not isinstance(offsets, list) or not offsets:
            raise ValueError(f"{name}.channel_offsets_m must contain channels")
        for offset in offsets:
            _vector(offset, 3, f"{name}.channel_offsets_m")
        _number(microphone.get("speed_of_sound_mps", 343.0), f"{name}.speed_of_sound_mps", 0, exclusive=True)
        sensitivity, calibration = microphone.get("sensitivity_volts_per_pa"), microphone.get("calibration_metadata")
        if sensitivity is not None:
            _number(sensitivity, f"{name}.sensitivity_volts_per_pa", 0, exclusive=True)
            if not isinstance(calibration, dict) or not calibration:
                raise ValueError(f"{name}.calibration_metadata requires an explicit nonempty object")
            try:
                json.dumps(calibration, allow_nan=False)
            except (ValueError, TypeError) as exc:
                raise ValueError(f"{name}.calibration_metadata must contain finite JSON data") from exc
        elif calibration is not None:
            raise ValueError(f"{name}.calibration_metadata requires sensitivity_volts_per_pa")
        return config
    _camera(config["camera"])
    _number(config.get("max_distance_m", 100), "measurement.max_distance_m", 0.001, exclusive=True)
    gimbal = _object(config.get("gimbal", {}), set(),
                     {"pan_deg", "tilt_deg", "pan_limits_deg", "tilt_limits_deg"}, "measurement.gimbal")
    for axis, default in (("pan", [-180, 180]), ("tilt", [-90, 90])):
        angle, limits = gimbal.get(f"{axis}_deg", 0), gimbal.get(f"{axis}_limits_deg", default)
        _number(angle, f"measurement.gimbal.{axis}_deg")
        _vector(limits, 2, f"measurement.gimbal.{axis}_limits_deg")
        if not limits[0] <= angle <= limits[1]:
            raise ValueError(f"measurement.gimbal.{axis}_deg must be within ordered limits")
    if kind == "thermal":
        name = "measurement.thermal"
        thermal = _object(config["thermal"], {"calibration_id", "background_temperature_k"},
                          {"transmittance", "atmospheric_temperature_k", "temperature_mode",
                           "min_temperature_k", "max_temperature_k"}, name)
        if not isinstance(thermal["calibration_id"], str) or not thermal["calibration_id"].strip():
            raise ValueError(f"{name}.calibration_id must be a nonempty string")
        for key in ("background_temperature_k", "atmospheric_temperature_k", "min_temperature_k", "max_temperature_k"):
            if key in thermal:
                _number(thermal[key], f"{name}.{key}", 0, _FLOAT32_MAX, exclusive=True)
        if thermal.get("min_temperature_k", 1) >= thermal.get("max_temperature_k", 2000):
            raise ValueError(f"{name} temperature bounds must be ordered")
        _number(thermal.get("transmittance", 1), f"{name}.transmittance", 0, 1)
        mode = thermal.get("temperature_mode", "apparent")
        if not isinstance(mode, str) or mode not in {"apparent", "surface_corrected"}:
            raise ValueError(f"{name}.temperature_mode is unsupported")
    return config

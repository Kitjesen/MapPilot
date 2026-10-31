from __future__ import annotations

import json
from dataclasses import replace

import numpy as np
import pytest
from sim.runtime.sensors.measurement_control import MeasurementNoise
from sim.runtime.sensors.samples import SensorSampleStamp
from sim.runtime.sensors.thermal import ThermalConfig, ThermalSensor, ThermalSurface


def _stamp() -> SensorSampleStamp:
    return SensorSampleStamp("thermal-test", "robot", "thermal", "camera", 2, 1, 4, 123000)


def _calibration():
    return {
        "model": "pinhole",
        "width": 2,
        "height": 2,
        "fx": 80.0,
        "fy": 80.0,
        "cx": 0.5,
        "cy": 0.5,
        "calibration_id": "sim-declared-v1",
    }


@pytest.mark.parametrize("temperature,valid", [(280.0, True), (400.0, True), (279.99, False), (400.01, False)])
def test_corrected_range_endpoints_do_not_gain_inversion_roundoff(temperature, valid):
    config = ThermalConfig(
        _calibration(),
        280,
        transmittance=0.1,
        temperature_mode="surface_corrected",
        min_temperature_k=280,
        max_temperature_k=400,
    )
    sensor = ThermalSensor(config, {1: ThermalSurface(temperature, 0.5)})
    result = sensor.measure(
        _stamp(),
        source_stamp=_stamp(),
        geom_ids=np.ones((2, 2), dtype=int),
        distances_m=np.ones((2, 2)),
        valid_rays=np.ones((2, 2), dtype=bool),
    )
    assert result.valid_mask.all() == valid
    assert result.metadata["provenance"] == {
        "model_id": "declared_surface_broadband_greybody", "model_version": "1", "qualification": "EXPERIMENTAL"}
    if valid:
        np.testing.assert_array_equal(result.temperature_k, np.float32(temperature))
    else:
        assert np.isnan(result.temperature_k).all()


def test_full_image_brightness_uses_each_visible_surfaces_emissivity_and_background():
    sensor = ThermalSensor(
        ThermalConfig(camera_calibration=_calibration(), background_temperature_k=280),
        {1: ThermalSurface(400, 1.0), 2: ThermalSurface(400, 0.25)},
    )
    stamp = _stamp()
    frame = sensor.measure(
        stamp,
        source_stamp=stamp,
        geom_ids=np.array([[1, 2], [2, 1]]),
        distances_m=np.ones((2, 2)),
        valid_rays=np.ones((2, 2), dtype=bool),
    )
    expected = (0.25 * 400.0**4 + 0.75 * 280.0**4) ** 0.25
    np.testing.assert_allclose(frame.temperature_k, [[400, expected], [expected, 400]], rtol=1e-6)
    assert frame.temperature_k.dtype == np.float32
    assert frame.valid_mask.dtype == np.bool_
    assert frame.valid_mask.all()
    assert frame.stamp == stamp


def test_unknown_nearest_geometry_misses_and_invalid_ranges_stay_invalid_without_mutation():
    surfaces = {1: ThermalSurface(400, 1)}
    sensor = ThermalSensor(ThermalConfig(_calibration(), 280), surfaces)
    surfaces[99] = ThermalSurface(600, 1)
    geom_ids = np.array([[99, -1], [1, 1]])
    distances = np.array([[1.0, np.inf], [np.nan, 2.0]])
    rays = np.array([[True, True], [True, False]])
    before = [a.copy() for a in (geom_ids, distances, rays)]
    frame = sensor.measure(_stamp(), source_stamp=_stamp(), geom_ids=geom_ids, distances_m=distances, valid_rays=rays)
    assert not frame.valid_mask.any()
    assert np.isnan(frame.temperature_k).all()
    for original, copied in zip((geom_ids, distances, rays), before):
        np.testing.assert_array_equal(original, copied)
    with pytest.raises(TypeError):
        sensor.surfaces[2] = ThermalSurface(500, 1)
    assert not frame.temperature_k.flags.writeable
    assert not frame.valid_mask.flags.writeable


def test_path_emission_and_emissivity_correction_are_explicit_distinct_temperature_modes():
    stamp = _stamp()
    surfaces = {1: ThermalSurface(400, 0.25), 2: ThermalSurface(400, 0)}
    inputs = dict(
        source_stamp=stamp,
        geom_ids=np.array([[1, 2], [1, 2]]),
        distances_m=np.ones((2, 2)),
        valid_rays=np.ones((2, 2), dtype=bool),
    )
    apparent = ThermalSensor(
        ThermalConfig(_calibration(), 280, transmittance=0.5, atmospheric_temperature_k=300), surfaces
    ).measure(stamp, **inputs)
    expected = (0.5 * (0.25 * 400.0**4 + 0.75 * 280.0**4) + 0.5 * 300.0**4) ** 0.25
    assert apparent.temperature_k[0, 0] == pytest.approx(expected, rel=1e-6)
    corrected = ThermalSensor(
        ThermalConfig(
            _calibration(), 280, transmittance=0.5, atmospheric_temperature_k=300, temperature_mode="surface_corrected"
        ),
        surfaces,
    ).measure(stamp, **inputs)
    np.testing.assert_allclose(corrected.temperature_k[:, 0], 400, rtol=1e-6)
    assert not corrected.valid_mask[:, 1].any()
    assert np.isnan(corrected.temperature_k[:, 1]).all()
    assert corrected.metadata["temperature_kind"] == "emissivity_corrected_surface"


def test_calibration_source_identity_and_numeric_bounds_are_enforced_and_disclosed():
    calibration = _calibration()
    sensor = ThermalSensor(
        ThermalConfig(calibration, 280, min_temperature_k=250, max_temperature_k=450),
        {1: ThermalSurface(500, 1), 2: ThermalSurface(300, 1)},
    )
    calibration["fx"] = 999
    stamp = _stamp()
    inputs = dict(
        source_stamp=stamp,
        geom_ids=np.array([[1, 2], [2, -1]]),
        distances_m=np.array([[1.0, 0.0], [1.0, 1.0]]),
        valid_rays=np.ones((2, 2), dtype=bool),
    )
    frame = sensor.measure(stamp, **inputs)
    np.testing.assert_array_equal(frame.valid_mask, [[False, False], [True, False]])
    assert np.isnan(frame.temperature_k[~frame.valid_mask]).all()
    assert frame.metadata["camera_calibration"]["fx"] == 80
    assert frame.metadata["env"] == "sim"
    assert frame.metadata["calibrated_lwir"] is False
    assert frame.metadata["truth_model"] == "declared_surface_broadband_greybody"
    assert frame.metadata["source_stamp"]["sim_time_ns"] == stamp.sim_time_ns
    assert "T^4" in " ".join(frame.metadata["calibration_assumptions"])
    json.dumps(frame.metadata, allow_nan=False)
    for field in ("model_generation", "reset_generation", "sim_time_ns", "sequence"):
        bad = dict(inputs, source_stamp=replace(stamp, **{field: getattr(stamp, field) + 1}))
        with pytest.raises(ValueError, match="source_stamp"):
            sensor.measure(stamp, **bad)
    with pytest.raises(ValueError, match="shape"):
        sensor.measure(stamp, **dict(inputs, distances_m=np.ones((1, 2))))
    with pytest.raises(ValueError, match="shape"):
        ThermalSensor(ThermalConfig(dict(_calibration(), width=3), 280), {}).measure(stamp, **inputs)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"background_temperature_k": -1},
        {"transmittance": 1.1},
        {"transmittance": float("nan")},
        {"atmospheric_temperature_k": 0},
        {"temperature_mode": "lwir"},
        {"min_temperature_k": 500, "max_temperature_k": 400},
        {"max_temperature_k": float("inf")},
        {"camera_calibration": {}},
        {"camera_calibration": dict(_calibration(), fx=0)},
    ],
)
def test_bad_configuration_is_rejected(kwargs):
    with pytest.raises(ValueError):
        ThermalConfig(**dict({"camera_calibration": _calibration(), "background_temperature_k": 280}, **kwargs))


@pytest.mark.parametrize("temperature,emissivity", [(0, 1), (float("nan"), 1), (300, -0.1), (300, 1.1)])
def test_invalid_declared_surface_is_rejected(temperature, emissivity):
    with pytest.raises(ValueError):
        ThermalSurface(temperature, emissivity)


def test_common_kelvin_noise_is_replayable_switchable_and_cannot_revive_invalid_pixels():
    stamp = _stamp()
    inputs = dict(
        source_stamp=stamp,
        geom_ids=np.array([[1, 1], [1, -1]]),
        distances_m=np.ones((2, 2)),
        valid_rays=np.ones((2, 2), dtype=bool),
    )
    surfaces = {1: ThermalSurface(300, 1)}
    base = ThermalConfig(_calibration(), 280)
    clean = ThermalSensor(base, surfaces).measure(stamp, **inputs)
    disabled = ThermalSensor(
        replace(base, noise=MeasurementNoise("K", enabled=False, stddev=50, seed=3)), surfaces
    ).measure(stamp, **inputs)
    np.testing.assert_array_equal(disabled.temperature_k, clean.temperature_k)
    noise = MeasurementNoise("K", enabled=True, stddev=2, seed=17)
    sensor = ThermalSensor(replace(base, noise=noise), surfaces)
    noisy = sensor.measure(stamp, **inputs)
    replay = sensor.measure(stamp, **inputs)
    np.testing.assert_array_equal(noisy.temperature_k, replay.temperature_k)
    assert np.any(noisy.temperature_k[clean.valid_mask] != clean.temperature_k[clean.valid_mask])
    assert np.isnan(noisy.temperature_k[1, 1]) and not noisy.valid_mask[1, 1]
    expected = 300 + noise.generator(stamp).normal(0, 2, size=(2, 2))
    np.testing.assert_allclose(noisy.temperature_k[clean.valid_mask], expected[clean.valid_mask], rtol=1e-6)
    bounded = ThermalSensor(
        replace(base, noise=MeasurementNoise("K", True, 1e6, 17), min_temperature_k=299, max_temperature_k=301),
        surfaces,
    ).measure(stamp, **inputs)
    assert np.isnan(bounded.temperature_k[~bounded.valid_mask]).all()
    assert not bounded.valid_mask.any()
    assert noisy.metadata["noise"]["unit"] == "K"
    assert noisy.metadata["noise"]["enabled"] is True
    json.dumps(noisy.metadata, allow_nan=False)
    with pytest.raises(ValueError, match="K"):
        replace(base, noise=MeasurementNoise("m"))


def test_zero_transmittance_is_path_brightness_not_a_recoverable_surface_temperature():
    inputs = dict(
        source_stamp=_stamp(),
        geom_ids=np.ones((2, 2), dtype=int),
        distances_m=np.array([[1.0, 10.0], [100.0, 1000.0]]),
        valid_rays=np.ones((2, 2), dtype=bool),
    )
    base = ThermalConfig(_calibration(), 280, atmospheric_temperature_k=310, transmittance=0)
    apparent = ThermalSensor(base, {1: ThermalSurface(700, 0.8)}).measure(_stamp(), **inputs)
    np.testing.assert_allclose(apparent.temperature_k, 310)
    corrected = ThermalSensor(
        replace(base, temperature_mode="surface_corrected"), {1: ThermalSurface(700, 0.8)}
    ).measure(_stamp(), **inputs)
    assert not corrected.valid_mask.any()
    assert np.isnan(corrected.temperature_k).all()


def test_other_calibrated_projection_metadata_is_preserved_without_inventing_pinhole_intrinsics():
    camera = {
        "model": "fisheye",
        "width": 2,
        "height": 2,
        "calibration_id": "declared-fisheye-v1",
        "parameters": {"projection": "equidistant", "focal_length_px": 60},
    }
    config = ThermalConfig(camera, 280)
    camera["parameters"]["focal_length_px"] = 900
    sensor = ThermalSensor(config, {1: ThermalSurface(300, 1)})
    inputs = dict(
        source_stamp=_stamp(),
        geom_ids=np.ones((2, 2), dtype=int),
        distances_m=np.ones((2, 2)),
        valid_rays=np.ones((2, 2), dtype=bool),
    )
    frame = sensor.measure(_stamp(), **inputs)
    assert frame.metadata["camera_calibration"]["parameters"]["focal_length_px"] == 60
    assert "fx" not in frame.metadata["camera_calibration"]
    for bad in (
        {"geom_ids": np.ones((2, 2))},
        {"valid_rays": np.ones((2, 2), dtype=int)},
        {"distances_m": np.full((2, 2), "bad")},
    ):
        with pytest.raises(ValueError):
            sensor.measure(_stamp(), **dict(inputs, **bad))

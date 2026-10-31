"""Local Gateway measurement contracts; no servers, drivers, or simulator."""

from __future__ import annotations

import json
from dataclasses import replace
from io import BytesIO
from types import SimpleNamespace

import numpy as np
import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from gateway.routes.measurements import register_measurement_routes
from gateway.services.measurements import LatestMeasurements
from runtime.contracts.sensor_control import SensorControlResult
from runtime.msgs.measurement import (
    AcousticMeasurement,
    MeasurementIdentity,
    SemanticMeasurement,
    ThermalMeasurement,
)

BASE = "/api/v1/measurements"


def test_disabled_ack_hides_previous_sample_from_live_download(live):
    cache, client, _ = live
    original = sample()
    cache.observe(original)
    identity = original.identity
    cache.apply_control(
        SensorControlResult(
            "disable",
            identity.product_session_id,
            identity.session_id,
            identity.sensor_id,
            identity.model_generation,
            identity.reset_generation,
            identity.sim_time_ns,
            True,
            "applied",
            {"enabled": False, "rate_hz": 10},
        )
    )
    state = client.get(BASE + "/front").json()
    assert state["status"] == "disabled" and state["fresh"] is False
    assert state["controls"]["enabled"] is False
    assert client.get(BASE + "/front/raw").status_code == 409


@pytest.fixture
def live(monkeypatch):
    clock = SimpleNamespace(epoch=100.0, monotonic=10.0)
    monkeypatch.setattr("gateway.services.measurements.time.time", lambda: clock.epoch)
    monkeypatch.setattr("gateway.services.measurements.time.monotonic", lambda: clock.monotonic)
    cache = LatestMeasurements(("front", "rear", "mic", "labels"), stale_timeout_s=2.0)
    app = FastAPI()
    register_measurement_routes(app, SimpleNamespace(_measurements=cache))
    with TestClient(app) as client:
        yield cache, client, clock


def sample(modality="thermal", sensor_id="front", sequence=1):
    identity = MeasurementIdentity("product", "session", "robot", sensor_id, "optical", 2, 3, sequence, 999)
    mask = np.array([[True, False]], dtype=bool)
    metadata = {
        "units": {"temperature_k": "K", "valid_mask": "1"},
        "frame_metadata": {"camera_calibration": {"qualification": "test-only"}},
        "unknown_statistic": float("nan"),
    }
    values = np.array([[300.0, np.nan]], dtype=np.float32)
    if modality == "thermal":
        return ThermalMeasurement(identity, values, mask, metadata, 100.0)
    if modality == "acoustic":
        metadata.update(
            units={"pressure_pa": "Pa", "voltage_v": "V", "valid_mask": "1"},
            frame_metadata={"calibration": {"id": "mic-test"}},
        )
        return AcousticMeasurement(identity, values, mask, 8000, values * 0.01, metadata, 100.0)
    metadata.update(
        units={"class_ids": "label", "instance_ids": "label", "valid_mask": "1"},
        calibration_metadata={"projection": {"model": "pinhole"}},
        ground_truth=True,
    )
    return SemanticMeasurement(
        identity, np.array([[7, 0]], dtype=np.uint32), np.array([[42, 0]], dtype=np.uint32), mask, metadata, 100.0
    )


def test_declared_missing_unknown_and_read_only(live):
    cache, client, _ = live
    response = client.get(BASE)
    assert response.status_code == 200
    assert response.headers["cache-control"] == "no-store"
    assert len(response.json()["sensors"]) == 4
    for entry in response.json()["sensors"]:
        assert entry["status"] == "no_sample"
        assert entry["validity"] is entry["source_identity"] is entry["units"] is None
        assert entry["fresh"] is False
    assert client.get(f"{BASE}/front/raw").status_code == 503
    for suffix in ("", "/raw"):
        response = client.get(f"{BASE}/unknown{suffix}")
        assert response.status_code == 404
        assert response.json()["error"] == "unknown_sensor"
    assert client.post(f"{BASE}/front").status_code == 405
    assert cache._latest == {}


@pytest.mark.parametrize("gateway", [None, SimpleNamespace()])
def test_standalone_routes_report_explicit_unavailable(gateway):
    app = FastAPI()
    register_measurement_routes(app, gateway)
    with TestClient(app) as client:
        for suffix in ("", "/front", "/front/raw"):
            response = client.get(BASE + suffix)
            assert response.status_code == 503
            assert response.json()["error"] == "measurement_unavailable"
            assert response.json()["status"] == "disabled"


def test_app_without_gateway_and_empty_configuration_are_explicit(monkeypatch):
    from gateway.app_factory import build_gateway_app

    # Video registration currently dereferences gw unconditionally; that
    # separate owner's route is outside this measurement registration test.
    monkeypatch.setattr("gateway.routes.operations.register_video_routes", lambda app, gw: None)
    with TestClient(build_gateway_app(None)) as client:
        assert client.get(BASE).status_code == 503
    assert LatestMeasurements(()).snapshot()["status"] == "disabled"
    assert LatestMeasurements(()).snapshot()["sensors"] == []


@pytest.mark.parametrize(
    "modality,sensor_id,names",
    [
        ("thermal", "front", ("temperature_k",)),
        ("acoustic", "mic", ("pressure_pa", "voltage_v")),
        ("semantic", "labels", ("class_ids", "instance_ids")),
    ],
)
def test_live_status_and_lossless_npz(live, modality, sensor_id, names):
    cache, client, _ = live
    observation = sample(modality, sensor_id)
    cache.observe(observation)
    response = client.get(f"{BASE}/{sensor_id}")
    status = response.json()
    assert status["status"] == "live"
    assert status["fresh"] and not status["stale"]
    assert status["modality"] == modality
    assert status["source_identity"]["product_session_id"] == "product"
    assert status["source_identity"]["sim_time_ns"] == 999
    assert status["calibration"] is not None
    assert status["units"] == observation.metadata["units"]
    assert status["validity"]["status"] == "partial"
    assert status["validity"]["valid_elements"] == 1
    assert status["captured_at_s"] is None and not status["clock_mapped"]
    assert status["metadata"]["unknown_statistic"] is None
    json.dumps(status, allow_nan=False)
    response = client.get(status["raw_url"])
    assert response.status_code == 200
    assert response.headers["content-type"] == "application/x-npz"
    with np.load(BytesIO(response.content), allow_pickle=False) as raw:
        assert set(raw.files) == {*names, "valid_mask", "metadata_json"}
        for name in (*names, "valid_mask"):
            original = getattr(observation, name)
            assert raw[name].dtype == original.dtype
            np.testing.assert_array_equal(raw[name], original)
        assert json.loads(str(raw["metadata_json"]))["source_identity"] == status["source_identity"]


def test_cache_is_bounded_and_replaces_only_the_matching_sensor(live):
    cache, client, _ = live
    cache.observe(sample(sensor_id="front"))
    cache.observe(sample(sensor_id="rear"))
    latest = sample(sequence=9)
    cache.observe(latest)
    latest.temperature_k[:] = 999  # A publisher cannot mutate the cached snapshot.
    for index in range(100):
        cache.observe(sample(sensor_id=f"undeclared-{index}"))
    assert set(cache._latest) == {"front", "rear"}
    front = client.get(f"{BASE}/front").json()
    rear = client.get(f"{BASE}/rear").json()
    assert front["source_identity"]["sequence"] == 9
    assert rear["source_identity"]["sequence"] == 1
    assert front["arrays"]["temperature_k"]["max"] == 300.0


def test_stale_receipt_is_explicit_and_raw_is_not_served_as_live(live):
    cache, client, clock = live
    cache.observe(sample())
    clock.monotonic += 2.1
    status = client.get(f"{BASE}/front").json()
    assert status["status"] == "stale" and status["stale"] and not status["fresh"]
    assert status["receipt_age_s"] == pytest.approx(2.1)
    response = client.get(f"{BASE}/front/raw")
    assert response.status_code == 409 and response.json()["error"] == "stale"
    clock.epoch += 10.0
    cache.observe(sample())
    assert client.get(f"{BASE}/front").json()["status"] == "stale"


def test_all_invalid_never_becomes_zero_temperature(live):
    cache, client, _ = live
    invalid = replace(
        sample(), temperature_k=np.array([[np.nan, np.nan]], dtype=np.float32), valid_mask=np.array([[False, False]])
    )
    cache.observe(invalid)
    status = client.get(f"{BASE}/front").json()
    assert status["validity"]["status"] == "invalid"
    assert status["arrays"]["temperature_k"]["min"] is None
    with np.load(BytesIO(client.get(status["raw_url"]).content), allow_pickle=False) as raw:
        assert np.isnan(raw["temperature_k"]).all()


def test_thermal_preview_is_measured_kelvin_with_invalid_pixels_not_zero(live):
    cache, client, _ = live
    cache.observe(sample())
    response = client.get(f"{BASE}/front/preview")
    assert response.status_code == 200
    body = response.json()
    assert body["measurement"]["source_identity"]["sequence"] == 1
    assert body["display"]["temperature_k"] == [[300.0, None]]
    assert body["display"]["valid_mask"] == [[True, False]]
    assert body["display"]["column_indices"] == [0, 1]
    assert body["display"]["row_indices"] == [0]
    assert body["display"]["kind"] == "thermal"


def test_acoustic_preview_envelopes_keep_spikes_pa_and_sample_time(live):
    cache, client, _ = live
    pressure = np.zeros((2, 512), dtype=np.float32)
    pressure[0, 1] = 1.25
    pressure[1, 0] = -2.0
    mask = np.ones_like(pressure, dtype=bool)
    mask[0, 4:6] = False
    mask[1, 1] = False
    cache.observe(replace(sample("acoustic", "mic"), pressure_pa=pressure, valid_mask=mask, voltage_v=None))
    response = client.get(f"{BASE}/mic/preview")
    assert response.status_code == 200
    display = response.json()["display"]
    assert display["kind"] == "acoustic"
    assert display["sample_rate_hz"] == 8000
    assert display["source_sample_count"] == 512
    assert display["bin_start_samples"] == list(range(0, 512, 2))
    assert display["bin_end_samples"] == list(range(2, 514, 2))
    assert display["pressure_min_pa"][0][0] == 0.0
    assert display["pressure_max_pa"][0][0] == 1.25
    assert display["pressure_min_pa"][0][2] is None
    assert display["pressure_max_pa"][0][2] is None
    assert display["valid_fraction"][0][2] == 0.0
    assert display["pressure_min_pa"][1][0] == -2.0
    assert display["valid_fraction"][1][0] == 0.5


def test_semantic_preview_is_ground_truth_and_preserves_label_ids(live):
    cache, client, _ = live
    cache.observe(sample("semantic", "labels"))
    response = client.get(f"{BASE}/labels/preview")
    assert response.status_code == 200
    display = response.json()["display"]
    assert display["kind"] == "semantic" and display["ground_truth"] is True
    assert display["class_ids"] == [[7, None]]
    assert display["instance_ids"] == [[42, None]]
    assert display["valid_mask"] == [[True, False]]


@pytest.mark.parametrize("modality,sensor_id", [("thermal", "front"), ("semantic", "labels")])
def test_image_preview_is_bounded_nearest_pixels_not_resized_measurement(live, modality, sensor_id):
    cache, client, _ = live
    values = np.arange(600 * 800, dtype=np.uint32).reshape(600, 800)
    source = sample(modality, sensor_id)
    changes = {"valid_mask": np.ones(values.shape, dtype=bool)}
    if modality == "thermal":
        changes["temperature_k"] = values.astype(np.float32) + 300
        field = "temperature_k"
    else:
        changes.update(class_ids=values, instance_ids=values + 500_000)
        field = "instance_ids"
    source = replace(source, **changes)
    cache.observe(source)
    display = client.get(f"{BASE}/{sensor_id}/preview").json()["display"]
    assert (display["source_width"], display["source_height"]) == (800, 600)
    assert (display["width"], display["height"]) == (192, 144)
    assert display["sampling"] == "nearest_source_pixel" and display["display_only"]
    rows, columns = display["row_indices"], display["column_indices"]
    np.testing.assert_array_equal(display[field], getattr(source, field)[np.ix_(rows, columns)])
    with np.load(BytesIO(client.get(f"{BASE}/{sensor_id}/raw").content), allow_pickle=False) as raw:
        np.testing.assert_array_equal(raw[field], getattr(source, field))


def test_preview_refuses_missing_disabled_and_stale_measurements(live):
    cache, client, clock = live
    assert client.get(f"{BASE}/front/preview").status_code == 503
    assert client.get(f"{BASE}/unknown/preview").status_code == 404
    original = sample()
    cache.observe(original)
    clock.monotonic += 3
    response = client.get(f"{BASE}/front/preview")
    assert response.status_code == 409 and response.json()["error"] == "stale"
    assert "display" not in response.json()
    clock.monotonic -= 3
    identity = original.identity
    cache.apply_control(
        SensorControlResult(
            "stop-preview",
            identity.product_session_id,
            identity.session_id,
            identity.sensor_id,
            identity.model_generation,
            identity.reset_generation,
            identity.sim_time_ns,
            True,
            "applied",
            {"enabled": False},
        )
    )
    response = client.get(f"{BASE}/front/preview")
    assert response.status_code == 409 and response.json()["error"] == "disabled"
    assert "display" not in response.json()


def test_unmasked_nonfinite_values_are_not_reported_as_valid(live):
    cache, client, _ = live
    cache.observe(replace(sample(), valid_mask=np.array([[True, True]])))
    assert client.get(f"{BASE}/front").json()["validity"]["valid_elements"] == 1


def test_future_receipt_is_explicit_not_normal(live):
    cache, client, _ = live
    cache.observe(replace(sample(), received_at_s=200.0))
    assert client.get(f"{BASE}/front").json()["status"] == "invalid_clock"
    assert client.get(f"{BASE}/front/raw").status_code == 409


@pytest.mark.parametrize("timeout", [0, -1, float("nan"), float("inf"), True])
def test_staleness_configuration_is_finite_positive(timeout):
    with pytest.raises(ValueError):
        LatestMeasurements(("front",), timeout)


def test_gateway_ports_setup_and_app_registration(monkeypatch):
    import gateway.gateway_module as module

    monkeypatch.setattr(module, "refresh_session_projection", lambda gw: None)
    monkeypatch.setattr(module, "reconcile_native_explore", lambda gw: None)
    gateway = module.GatewayModule(measurement_sensor_ids=("front", "mic", "labels"))
    gateway.setup()
    try:
        for name, expected in (
            ("thermal", ThermalMeasurement),
            ("acoustic", AcousticMeasurement),
            ("semantic", SemanticMeasurement),
        ):
            port = getattr(gateway, name)
            assert port.msg_type is expected
            port._deliver(sample(name, {"thermal": "front", "acoustic": "mic", "semantic": "labels"}[name]))
        assert set(gateway._measurements._latest) == {"front", "mic", "labels"}
        with TestClient(gateway._app) as client:
            assert client.get(BASE).status_code == 200
            assert len(client.get(BASE).json()["sensors"]) == 3
    finally:
        gateway.stop()

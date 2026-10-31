"""Camera scheduler evidence follows selected formal streams, not process presence."""

from typing import Any

import pytest
from sim.scripts.mujoco import product_acceptance

from lingtu.run_plan import RunPlan
from tests.sim.test_formal_feeder import _run_plan
from tests.sim.test_formal_measurement_feeder import _with_measurements
from tests.sim.test_mujoco_product_acceptance import _feeder_status_payload


def _selected_plan(rgb_hz: float | None = 15.0, depth_hz: float | None = 10.0) -> RunPlan:
    payload = _run_plan(camera=True, enable_camera=True).as_dict()
    streams = payload["launch"]["simulation"]["sensor_plan"]["streams"]
    for kind, rate in (("rgb", rgb_hz), ("depth", depth_hz)):
        if rate is None:
            streams[kind] = []
        else:
            streams[kind][0].update(
                rate_hz=rate, owner="sensor_runtime", source="mujoco_camera", transport="camera_shm"
            )
    return RunPlan.from_dict(payload)


def _status(plan: RunPlan, rates: dict[str, float]) -> dict[str, Any]:
    status: dict[str, Any] = _feeder_status_payload(plan)
    streams = status["streams"]
    streams.pop("camera_rgbd", None)
    for kind, rate in rates.items():
        streams[f"camera_{kind}"] = {
            "expected_hz": rate,
            "scheduled_count": int(rate * status["window_s"]),
            "published_count": int(rate * status["window_s"]),
            "dropped_count": 0,
            "actual_hz": rate,
            "max_schedule_lateness_ms": 1.0,
        }
    return status


def test_independent_camera_counts_pass_real_performance_validation() -> None:
    plan = _selected_plan()
    status = _status(plan, {"rgb": 15.0, "depth": 10.0})

    evidence = product_acceptance._sensor_runtime_evidence(plan, status)

    assert evidence["ok"] is True
    assert evidence["streams"]["camera_rgb"]["expected_hz"] == 15.0
    assert evidence["streams"]["camera_depth"]["expected_hz"] == 10.0
    assert status["streams"]["camera_rgb"]["published_count"] == 150
    assert status["streams"]["camera_depth"]["published_count"] == 100


@pytest.mark.parametrize("case", ("host_disabled", "no_rgbd", "measurement_only"))
def test_camera_endpoint_without_enabled_rgbd_needs_no_frames(case: str) -> None:
    payload = (_with_measurements() if case == "measurement_only" else _selected_plan()).as_dict()
    if case == "host_disabled":
        payload["host"]["config"]["enable_camera"] = False
    else:
        streams = payload["launch"]["simulation"]["sensor_plan"]["streams"]
        streams["rgb"] = []
        streams["depth"] = []
    plan = RunPlan.from_dict(payload)

    evidence = product_acceptance._sensor_runtime_evidence(plan, _status(plan, {}))

    assert evidence["ok"] is True
    assert not any(name.startswith("camera_") for name in evidence["streams"])


@pytest.mark.parametrize("kind", ("rgb", "depth"))
def test_selected_single_camera_does_not_require_disabled_peer(kind: str) -> None:
    plan = _selected_plan(rgb_hz=15.0 if kind == "rgb" else None, depth_hz=10.0 if kind == "depth" else None)
    rate = 15.0 if kind == "rgb" else 10.0

    evidence = product_acceptance._sensor_runtime_evidence(plan, _status(plan, {kind: rate}))

    assert {name for name in evidence["streams"] if name.startswith("camera_")} == {f"camera_{kind}"}


@pytest.mark.parametrize("kind", ("rgb", "depth"))
@pytest.mark.parametrize("fault", ("missing", "wrong_rate"))
def test_selected_camera_peer_requires_matching_reported_rate(kind: str, fault: str) -> None:
    plan = _selected_plan()
    status = _status(plan, {"rgb": 15.0, "depth": 10.0})
    if fault == "missing":
        status["streams"].pop(f"camera_{kind}")
    else:
        status["streams"][f"camera_{kind}"]["expected_hz"] = 12.0

    with pytest.raises(RuntimeError, match="stream set" if fault == "missing" else f"camera_{kind} expected_hz"):
        product_acceptance._sensor_runtime_evidence(plan, status)


@pytest.mark.parametrize(
    ("field", "value", "message"),
    (
        ("actual_hz", 14.24, "more than 5%"),
        ("dropped_count", 2, "exceeds 1%"),
        ("window_s", 2.99, "insufficient observation"),
    ),
)
def test_selected_camera_keeps_performance_gates(field: str, value: float, message: str) -> None:
    plan = _selected_plan()
    status = _status(plan, {"rgb": 15.0, "depth": 10.0})
    if field == "window_s":
        status[field] = value
    elif field == "dropped_count":
        status["streams"]["camera_depth"].update(published_count=98, dropped_count=2)
    else:
        status["streams"]["camera_rgb"][field] = value

    with pytest.raises(RuntimeError, match=message):
        product_acceptance._sensor_runtime_evidence(plan, status)


def test_selected_camera_accepts_exact_drop_rate_and_window_limits() -> None:
    plan = _selected_plan()
    status = _status(plan, {"rgb": 15.0, "depth": 10.0})
    status["window_s"] = 3.0
    status["streams"]["camera_rgb"]["actual_hz"] = 14.25
    status["streams"]["camera_depth"].update(published_count=99, dropped_count=1)

    assert product_acceptance._sensor_runtime_evidence(plan, status)["ok"] is True


@pytest.mark.parametrize(
    ("owner", "source", "transport"),
    (
        ("visual", "unreal_camera", "camera_shm"),
        ("sensor_runtime", "unreal_camera", "camera_shm"),
        ("visual", "mujoco_camera", "camera_shm"),
        ("sensor_runtime", "mujoco_camera", "typed_dds"),
    ),
)
def test_unsupported_declared_camera_routes_are_rejected(owner: str, source: str, transport: str) -> None:
    payload = _selected_plan().as_dict()
    payload["launch"]["simulation"]["sensor_plan"]["streams"]["rgb"][0].update(
        owner=owner, source=source, transport=transport
    )
    plan = RunPlan.from_dict(payload)

    with pytest.raises(RuntimeError, match="unsupported formal camera route for rgb"):
        product_acceptance._sensor_runtime_evidence(plan, _status(plan, {"rgb": 15.0, "depth": 10.0}))


def test_sparse_legacy_camera_fixture_keeps_combined_rgbd_evidence() -> None:
    plan = _run_plan(camera=True, enable_camera=True)

    evidence = product_acceptance._sensor_runtime_evidence(plan, _feeder_status_payload(plan))

    assert "camera_rgbd" in evidence["streams"]
    assert "camera_rgb" not in evidence["streams"]
    assert "camera_depth" not in evidence["streams"]


def test_sparse_legacy_camera_still_requires_equal_rates() -> None:
    plan = _run_plan(camera=True, enable_camera=True, depth_camera_hz=10.0)

    with pytest.raises(RuntimeError, match="RGB and depth rates differ"):
        product_acceptance._sensor_runtime_evidence(plan, _feeder_status_payload(plan))

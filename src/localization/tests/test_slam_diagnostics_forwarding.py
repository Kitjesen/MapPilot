from __future__ import annotations

from localization.adapters.status import CppSlamStatusAdapterModule


def _payload() -> dict:
    return {
        "runtime_instance_id": "slam-diag-test",
        "source_epoch": 1,
        "state": "TRACKING",
        "reason": "tracking",
        "alive": True,
        "has_odom": False,
        "stamp_s": 10.0,
        "confidence": 0.8,
        "localization_quality": 0.75,
        "observation_sequence": 1,
        "scan_end_s": 10.0,
        "fastlio_degeneracy": {
            "detected": True,
            "degenerate_dof_count": 2,
            "condition_number": 42000.0,
            "min_eigenvalue": 0.25,
            "max_eigenvalue": 10500.0,
            "effective_ratio": 2.0 / 3.0,
            "pos_cov_trace": 0.004,
            "iter_num": 4,
            "converged": True,
        },
        "fastlio_lidar_update": {
            "attempted": True,
            "accepted": False,
            "attempt_sequence": 9,
            "rejection_reason": "candidate_translation_limit_exceeded",
            "previous_rejection_reason": "no_valid_measurement",
            "consecutive_rejections": 2,
            "downsampled_points": 1832,
            "effective_points": 417,
            "candidate": {
                "translation_m": 0.72,
                "rotation_rad": 0.03,
                "velocity_mps": 0.91,
                "velocity_delta_mps": 0.12,
            },
            "thresholds": {
                "max_translation_m": 0.5,
                "max_rotation_rad": 0.35,
                "max_velocity_mps": 3.0,
                "max_velocity_delta_mps": 1.0,
            },
            "information_ldlt": {
                "evaluated": False,
                "decomposition_success": False,
                "positive": False,
            },
            "candidate_covariance": {
                "evaluated": False,
                "finite": False,
                "positive_diagonal": False,
            },
            "posterior_covariance": {
                "evaluated": False,
                "finite": False,
                "positive_diagonal": False,
            },
        },
    }


def test_cpp_slam_adapter_forwards_fastlio_degeneracy_metrics() -> None:
    adapter = CppSlamStatusAdapterModule()
    statuses: list[dict] = []
    adapter.localization_status.subscribe(statuses.append)

    adapter._publish_status_snapshot(_payload())

    status = statuses[-1]
    assert "degeneracy" not in status
    assert status["degeneracy_detected"] is True
    assert status["degenerate_dof_count"] == 2
    assert status["condition_number"] == 42000.0
    assert status["min_eigenvalue"] == 0.25
    assert status["max_eigenvalue"] == 10500.0
    assert status["effective_ratio"] == 2.0 / 3.0
    assert status["pos_cov_trace"] == 0.004
    assert status["ieskf_iter_num"] == 4
    assert status["ieskf_converged"] is True
    assert status["fastlio_degeneracy"]["detected"] is True


def test_cpp_slam_adapter_does_not_invent_metrics_when_block_is_missing() -> None:
    adapter = CppSlamStatusAdapterModule()
    statuses: list[dict] = []
    adapter.localization_status.subscribe(statuses.append)
    payload = _payload()
    payload.pop("fastlio_degeneracy")

    adapter._publish_status_snapshot(payload)

    status = statuses[-1]
    assert status["fastlio_degeneracy"] is None
    assert "degeneracy_detected" not in status
    assert "condition_number" not in status


def test_cpp_slam_adapter_forwards_fastlio_lidar_update_diagnostics_unchanged() -> None:
    adapter = CppSlamStatusAdapterModule()
    statuses: list[dict] = []
    adapter.localization_status.subscribe(statuses.append)
    payload = _payload()

    adapter._publish_status_snapshot(payload)

    assert statuses[-1]["reason"] == "tracking"
    assert statuses[-1]["fastlio_lidar_update"] == payload["fastlio_lidar_update"]


def test_fastlio_degeneracy_reaches_gateway_sse_and_rest_without_semantic_policy() -> None:
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import LocalizationStatusResponse
    from gateway.services.runtime_status import build_localization_status

    adapter = CppSlamStatusAdapterModule()
    gateway = GatewayModule()
    events = gateway._sse_subscribe()
    adapter.localization_status.subscribe(gateway._on_localization_status)

    adapter._publish_status_snapshot(_payload())

    event = events.get_nowait()
    assert event["type"] == "slam_diag"
    assert event["data"]["degeneracy_detected"] is True
    assert event["data"]["min_eigenvalue"] == 0.25
    assert event["data"]["max_eigenvalue"] == 10500.0
    assert "degeneracy" not in event["data"]

    payload = build_localization_status(gateway)
    model = LocalizationStatusResponse.model_validate(payload)
    assert model.degeneracy_detected is True
    assert model.min_eigenvalue == 0.25
    assert model.max_eigenvalue == 10500.0
    assert model.degeneracy is None

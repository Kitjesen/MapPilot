from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SMOKE_SCRIPT = ROOT / "tools" / "bench" / "camera_lidar_optimizer_abi_smoke.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "camera_lidar_optimizer_abi_smoke",
        SMOKE_SCRIPT,
    )
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _payload_with_checks(**overrides: bool) -> dict:
    checks = {
        "abi_version": True,
        "pose3_size": True,
        "ct_gicp_size": True,
        "ct_gicp_source_size": True,
        "ct_gicp_target_size": True,
        "linearization_size": True,
        "optimizer_config_size": True,
        "optimizer_result_size": True,
        "return_code": True,
        "used_correspondences": True,
        "cost": True,
        "gradient": True,
        "rhs": True,
        "hessian": True,
        "optimizer_return_code": True,
        "optimizer_used_correspondences": True,
        "optimizer_cost_reduction": True,
        "optimizer_final_cost_finite": True,
        "optimizer_pose0_translation": True,
        "dynamic_optimizer_return_code": True,
        "dynamic_optimizer_used_correspondences": True,
        "dynamic_optimizer_cost_reduction": True,
        "dynamic_optimizer_final_cost_finite": True,
        "dynamic_optimizer_pose0_translation": True,
    }
    checks.update(overrides)
    return {
        "schema": "lingtu.camera_lidar_optimizer_abi_smoke.v1",
        "checks": checks,
    }


def test_evaluate_result_accepts_all_required_checks() -> None:
    module = _load_module()

    result = module.evaluate_result(_payload_with_checks())

    assert result == {
        "schema": "lingtu.camera_lidar_optimizer_abi_smoke.v1",
        "ok": True,
        "failed_checks": [],
    }


def test_evaluate_result_reports_failed_abi_and_optimizer_checks() -> None:
    module = _load_module()

    result = module.evaluate_result(
        _payload_with_checks(
            abi_version=False,
            optimizer_cost_reduction=False,
        )
    )

    assert result["ok"] is False
    assert result["failed_checks"] == ["abi_version", "optimizer_cost_reduction"]


def test_evaluate_result_rejects_missing_required_check() -> None:
    module = _load_module()
    payload = _payload_with_checks()
    del payload["checks"]["dynamic_optimizer_pose0_translation"]

    result = module.evaluate_result(payload)

    assert result["ok"] is False
    assert result["failed_checks"] == ["dynamic_optimizer_pose0_translation:missing"]


def test_smoke_script_tracks_current_abi_version() -> None:
    module = _load_module()

    assert module.EXPECTED_ABI_VERSION == 2

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "tools" / "bench" / "pct_native_rust_parity.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("pct_native_rust_parity", SCRIPT)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _actual(path_offset: float = 0.0) -> dict:
    return {
        "schema": "lingtu.pct.preview.actual.v2",
        "ok": True,
        "path_count": 5,
        "path_distance_m": 4.0 + path_offset,
        "goal_error_m": 0.1,
        "first": [0.0, 0.0, 0.0],
        "last": [4.0 + path_offset, 0.0, 0.0],
        "input": {
            "start": [0.0, 0.0, 0.0],
            "goal": [4.0, 0.0, 0.0],
            "obstacle_thr": 49.9,
            "tomogram_sha256": "abc",
        },
        "path": {
            "samples": {
                "by_arclength_fraction": [
                    {"fraction": 0.0, "point": [0.0, 0.0, 0.0]},
                    {"fraction": 0.5, "point": [2.0 + path_offset, 0.0, 0.0]},
                    {"fraction": 1.0, "point": [4.0 + path_offset, 0.0, 0.0]},
                ]
            }
        },
        "optimizer_accessors": {
            "available": True,
            "native_wrapper_compatible": True,
            "finite": True,
            "row_count": 5,
            "state_dim": 6,
            "result_matrix_shape": [5, 6],
            "layers_shape": [5],
            "heights_shape": [5],
            "ceilings_shape": [5],
            "opt_init_value_shape": [6, 5],
            "opt_init_layer_shape": [5],
            "heading_rate_shape": [5],
        },
    }


def test_compare_actuals_passes_for_same_input_close_paths() -> None:
    module = _load_module()

    result = module.compare_actuals(
        _actual(),
        _actual(path_offset=0.05),
        path_count_delta_max=1,
        path_distance_abs_tol=0.1,
        goal_error_abs_tol=0.1,
        sample_abs_tol=0.1,
    )

    assert result["verdict"] == "pass"
    assert result["failed_checks"] == []


def test_compare_actuals_fails_for_path_and_input_mismatch() -> None:
    module = _load_module()
    rust = _actual(path_offset=3.0)
    rust["input"]["tomogram_sha256"] = "different"
    rust["optimizer_accessors"]["finite"] = False
    rust["optimizer_accessors"]["result_matrix_shape"] = [4, 6]

    result = module.compare_actuals(
        _actual(),
        rust,
        path_count_delta_max=1,
        path_distance_abs_tol=0.1,
        goal_error_abs_tol=0.1,
        sample_abs_tol=0.1,
    )

    assert result["verdict"] == "fail"
    assert "input:tomogram_sha256" in result["failed_checks"]
    assert "path_distance_m" in result["failed_checks"]
    assert "last" in result["failed_checks"]
    assert "optimizer_accessors:rust_process_finite" in result["failed_checks"]
    assert "optimizer_accessors:result_matrix_shape" in result["failed_checks"]

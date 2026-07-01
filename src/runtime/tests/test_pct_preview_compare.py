from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
COMPARE_SCRIPT = ROOT / "tools" / "bench" / "pct_preview_compare.py"
GOLDEN = (
    ROOT
    / "src"
    / "nav"
    / "tests"
    / "planning_backends"
    / "fixtures"
    / "pct_preview"
    / "building2_9_smoke.json"
)


def _load_module():
    spec = importlib.util.spec_from_file_location("pct_preview_compare", COMPARE_SCRIPT)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _golden() -> dict:
    return json.loads(GOLDEN.read_text(encoding="utf-8"))


def _passing_actual() -> dict:
    return {
        "schema": "lingtu.pct.preview.actual.v2",
        "ok": True,
        "planner": "pct",
        "path_count": 55,
        "path_distance_m": 17.82,
        "start": [2.0, 3.0, 0.0],
        "goal": [18.0, 11.0, 0.0],
        "first": [2.0, 3.0, 0.0],
        "last": [17.9, 11.05, 0.0],
        "goal_error_m": 0.12,
        "input": {
            "tomogram": "src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/rsc/tomogram/building2_9.pickle",
            "tomogram_sha256": "cd75fdf87120f7a9da4a8208687473199702e6871d9e5f225f6ba6784f5637ee",
            "tomogram_size_bytes": 638628,
            "tomogram_data_shape": [5, 7, 97, 94],
            "tomogram_data_dtype": "float16",
            "obstacle_thr": 49.9,
            "start": [2.0, 3.0, 0.0],
            "goal": [18.0, 11.0, 0.0],
        },
        "path_samples": [
            {"index": 0, "point": [2.0, 3.0, 0.0]},
            {"index": 27, "point": [10.0, 7.0, 0.0]},
            {"index": 54, "point": [17.9, 11.05, 0.0]},
        ],
        "path": {
            "shape": [55, 3],
            "finite": True,
            "count": 55,
            "distance_m": 17.82,
            "start_error_m": 0.0,
            "goal_error_m": 0.12,
            "bounds_xyz": {
                "min": [2.0, 3.0, 0.0],
                "max": [17.9, 11.05, 0.2],
            },
            "samples": {
                "by_index": [
                    {"index": 0, "point": [2.0, 3.0, 0.0]},
                    {"index": 27, "point": [10.0, 7.0, 0.0]},
                    {"index": 54, "point": [17.9, 11.05, 0.0]},
                ],
                "by_arclength_fraction": [
                    {"fraction": 0.0, "point": [2.0, 3.0, 0.0]},
                    {"fraction": 0.5, "point": [10.0, 7.0, 0.0]},
                    {"fraction": 1.0, "point": [17.9, 11.05, 0.0]},
                ],
            },
        },
        "diagnostics": {
            "last_path_mode": "optimized_trajectory",
            "last_optimizer_enabled": True,
            "last_optimizer_attempted": True,
            "last_optimizer_accepted": True,
        },
        "runtime": {"arch": "x86_64", "python": "py310", "lib_dir": "/native"},
    }


def test_building2_9_golden_fixture_has_expected_schema() -> None:
    golden = _golden()

    assert golden["schema"] == "lingtu.pct.preview.golden.v1"
    assert golden["case"] == "building2_9_smoke"
    assert golden["expected"]["path_count"]["nominal"] == 55
    assert golden["expected"]["goal_error_m"]["max"] == 0.35
    assert golden["actual_schema"] == "lingtu.pct.preview.actual.v2"
    assert golden["input"]["tomogram_data_shape"] == [5, 7, 97, 94]
    assert golden["expected"]["path"]["finite"] is True
    assert "path" in golden["required_fields"]
    assert "path_samples" in golden["required_fields"]
    assert "last_path_mode" in golden["required_diagnostics"]


def test_compare_passes_known_good_pct_preview_metrics() -> None:
    module = _load_module()

    result = module.compare(_golden(), _passing_actual())

    assert result["verdict"] == "pass"
    assert result["failed_checks"] == []
    assert result["skipped_checks"] == []
    assert {check["name"] for check in result["checks"]} >= {
        "ok",
        "planner",
        "path_count",
        "path_distance_m",
        "goal_error_m",
        "required_field:runtime",
        "required_field:path_samples",
        "required_diagnostics:last_path_mode",
        "actual_schema",
        "input:tomogram_sha256",
        "input:tomogram_size_bytes",
        "input:tomogram_data_shape",
        "input:tomogram_data_dtype",
        "input:obstacle_thr",
        "path.finite",
        "path.min_columns",
        "path.count_matches_path_count",
    }


def test_compare_fails_schema_and_input_identity_mismatch() -> None:
    module = _load_module()
    actual = _passing_actual()
    actual["schema"] = "lingtu.pct.preview.actual.v1"
    actual["input"]["tomogram_sha256"] = "bad"
    actual["input"]["tomogram_data_shape"] = [1, 2, 3]
    actual["input"]["obstacle_thr"] = 40.0
    actual["path"]["finite"] = False
    actual["path"]["shape"] = [55, 2]

    result = module.compare(_golden(), actual)

    assert result["verdict"] == "fail"
    failed = {check["name"] for check in result["failed_checks"]}
    assert {
        "actual_schema",
        "input:tomogram_sha256",
        "input:tomogram_data_shape",
        "input:obstacle_thr",
        "path.finite",
        "path.min_columns",
    } <= failed


def test_compare_fails_goal_error_and_missing_required_field() -> None:
    module = _load_module()
    actual = _passing_actual()
    actual["goal_error_m"] = 2.0
    actual.pop("runtime")

    result = module.compare(_golden(), actual)

    assert result["verdict"] == "fail"
    failed = {check["name"] for check in result["failed_checks"]}
    assert "goal_error_m" in failed
    assert "required_field:runtime" in failed


def test_compare_checks_optional_path_samples_and_diagnostics() -> None:
    module = _load_module()
    golden = _golden()
    golden["path_samples"] = [
        {"index": 0, "point": [2.0, 3.0, 0.0]},
        {"index": 54, "point": [17.9, 11.05, 0.0]},
    ]
    golden["expected_diagnostics"] = {
        "last_path_mode": {"one_of": ["optimized_trajectory", "native_astar_raw_path"]},
        "last_optimizer_attempted": True,
    }
    golden["path"] = {
        "samples": {
            "by_arclength_fraction": [
                {"fraction": 0.0, "point": [2.0, 3.0, 0.0]},
                {"fraction": 0.5, "point": [10.0, 7.0, 0.0]},
            ]
        }
    }

    result = module.compare(golden, _passing_actual())

    assert result["verdict"] == "pass"
    assert {check["name"] for check in result["checks"]} >= {
        "path_sample:0",
        "path_sample:54",
        "path_fraction_sample:0",
        "path_fraction_sample:0.5",
        "diagnostics:last_path_mode",
        "diagnostics:last_optimizer_attempted",
    }


def test_compare_checks_expected_optimizer_accessors() -> None:
    module = _load_module()
    golden = _golden()
    golden["required_fields"] = [*golden["required_fields"], "optimizer_accessors"]
    golden["expected_optimizer_accessors"] = {
        "available": True,
        "native_wrapper_compatible": True,
        "finite": True,
        "row_count": 55,
        "state_dim": 6,
        "result_matrix_shape": [55, 6],
        "layers_shape": [55],
        "heights_shape": [55],
        "opt_init_value_shape": [6, 55],
        "opt_init_layer_shape": [55],
    }
    actual = _passing_actual()
    actual["optimizer_accessors"] = {
        "available": True,
        "native_wrapper_compatible": True,
        "finite": True,
        "row_count": 55,
        "state_dim": 6,
        "result_matrix_shape": [55, 6],
        "layers_shape": [55],
        "heights_shape": [55],
        "opt_init_value_shape": [6, 55],
        "opt_init_layer_shape": [55],
    }

    result = module.compare(golden, actual)

    assert result["verdict"] == "pass"
    assert {
        "optimizer_accessors:native_wrapper_compatible",
        "optimizer_accessors:result_matrix_shape",
        "optimizer_accessors:opt_init_value_shape",
    } <= {check["name"] for check in result["checks"]}


def test_compare_fails_expected_optimizer_accessor_mismatch() -> None:
    module = _load_module()
    golden = _golden()
    golden["required_fields"] = [*golden["required_fields"], "optimizer_accessors"]
    golden["expected_optimizer_accessors"] = {
        "native_wrapper_compatible": True,
        "result_matrix_shape": [55, 6],
    }
    actual = _passing_actual()
    actual["optimizer_accessors"] = {
        "native_wrapper_compatible": False,
        "result_matrix_shape": [55, 4],
    }

    result = module.compare(golden, actual)

    assert result["verdict"] == "fail"
    assert {
        "optimizer_accessors:native_wrapper_compatible",
        "optimizer_accessors:result_matrix_shape",
    } <= {check["name"] for check in result["failed_checks"]}


def test_compare_fails_path_sample_outside_tolerance() -> None:
    module = _load_module()
    golden = _golden()
    golden["path_samples"] = [{"index": 54, "point": [99.0, 11.05, 0.0]}]

    result = module.compare(golden, _passing_actual())

    assert result["verdict"] == "fail"
    assert "path_sample:54" in {check["name"] for check in result["failed_checks"]}


def test_compare_reads_metrics_from_nested_v2_path_when_top_level_is_absent() -> None:
    module = _load_module()
    actual = _passing_actual()
    actual.pop("path_count")
    actual.pop("path_distance_m")
    actual.pop("goal_error_m")

    result = module.compare(_golden(), actual)

    assert result["verdict"] == "pass"


def test_compare_skips_optional_samples_when_actual_lacks_them() -> None:
    module = _load_module()
    golden = _golden()
    golden["path"] = {
        "samples": {
            "by_arclength_fraction": [
                {"fraction": 0.5, "point": [10.0, 7.0, 0.0]},
            ]
        }
    }
    actual = _passing_actual()
    actual.pop("path_samples")
    actual.pop("path")

    result = module.compare(golden, actual)

    assert result["verdict"] == "fail"
    assert "required_field:path_samples" in {
        check["name"] for check in result["failed_checks"]
    }
    assert result["skipped_checks"] == [
        {
            "name": "path.samples.by_arclength_fraction",
            "reason": "actual_has_no_arclength_samples",
        }
    ]


def test_compare_cli_enforce_returns_nonzero_on_failed_verdict(tmp_path: Path) -> None:
    module = _load_module()
    actual_path = tmp_path / "actual.json"
    actual = _passing_actual()
    actual["path_count"] = 5
    actual_path.write_text(json.dumps(actual), encoding="utf-8")

    code = module.main(
        [
            "--golden",
            str(GOLDEN),
            "--actual-json",
            str(actual_path),
            "--json",
            "--enforce",
        ]
    )

    assert code == 1

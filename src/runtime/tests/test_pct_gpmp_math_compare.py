from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "tools" / "bench" / "pct_gpmp_math_compare.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("pct_gpmp_math_compare", SCRIPT)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _result() -> dict:
    return {
        "schema": "lingtu.pct_gpmp_math.result.v1",
        "producer": "test",
        "cases": [
            {
                "case": "wnoj_positive_heading_rate",
                "mode": "wnoj",
                "state_order": ["x", "vx", "ax", "y", "vy", "ay"],
                "input": {
                    "qc": 0.1,
                    "delta": 1.0,
                    "tau": 0.35,
                    "x1": [0.2, 1.1, -0.4, -0.3, 0.7, 1.8],
                    "path_point": {
                        "x": 2.0,
                        "y": 3.0,
                        "heading": 0.7853981633974483,
                    },
                },
                "output": {
                    "q_delta": [[1.0, 0.0], [0.0, 1.0]],
                    "prior_residual": [0.1, 0.2],
                    "heading_rate_residual": 0.5,
                    "trajectory": [[0.0, 1.0], [1.0, 2.0]],
                },
            },
            {
                "case": "wnoa_nominal",
                "mode": "wnoa",
                "state_order": ["x", "vx", "y", "vy"],
                "input": {"qc": 0.1, "delta": 1.0, "tau": 0.35},
                "output": {
                    "q_delta": [[1.0]],
                    "prior_residual": [0.1],
                },
            },
        ],
    }


def test_compare_passes_nested_numeric_result_with_relative_tolerance() -> None:
    module = _load_module()
    rust = _result()
    baseline = _result()
    rust["cases"][0]["output"]["prior_residual"][0] += 5e-10

    result = module.compare(rust, baseline, abs_tol=1e-12, rel_tol=1e-8)

    assert result["verdict"] == "pass"
    assert result["failed_checks"] == []
    assert result["matched_cases"] == ["wnoa_nominal", "wnoj_positive_heading_rate"]
    assert result["missing_in_rust"] == []
    assert result["missing_in_baseline"] == []
    assert result["tolerances"] == {"abs_tol": 1e-12, "rel_tol": 1e-8}
    assert result["failed_checks"] == [
        check for check in result["checks"] if not check["passed"]
    ]


def test_compare_fails_numeric_tolerance_and_reports_error() -> None:
    module = _load_module()
    rust = _result()
    baseline = _result()
    rust["cases"][0]["output"]["heading_rate_residual"] = 0.9

    result = module.compare(rust, baseline, abs_tol=1e-9, rel_tol=1e-9)

    assert result["verdict"] == "fail"
    failed = {check["name"]: check for check in result["failed_checks"]}
    check = failed["wnoj_positive_heading_rate:output.heading_rate_residual"]
    assert check["max_error"] == 0.4
    assert check["abs_tol"] == 1e-9
    assert check["rel_tol"] == 1e-9


def test_compare_fails_shape_and_non_finite_values() -> None:
    module = _load_module()
    rust = _result()
    baseline = _result()
    rust["cases"][0]["output"]["q_delta"] = [[1.0, 0.0, 0.0]]
    rust["cases"][1]["output"]["prior_residual"] = [float("nan")]

    result = module.compare(rust, baseline)

    failed = {check["name"]: check for check in result["failed_checks"]}
    assert failed["wnoj_positive_heading_rate:output.q_delta"]["reason"] == (
        "shape_mismatch"
    )
    assert failed["wnoa_nominal:output.prior_residual"]["reason"] == (
        "non_numeric_or_length_mismatch"
    )


def test_compare_fails_missing_cases_duplicates_and_metadata_mismatch() -> None:
    module = _load_module()
    rust = _result()
    baseline = _result()
    rust["cases"] = [rust["cases"][0], rust["cases"][0]]
    baseline["cases"][0]["mode"] = "wrong"
    baseline["cases"][0]["state_order"] = ["wrong"]

    result = module.compare(rust, baseline)

    failed = {check["name"] for check in result["failed_checks"]}
    assert "rust_duplicate_case_names" in failed
    assert "case_names" in failed
    assert "wnoj_positive_heading_rate:mode" in failed
    assert "wnoj_positive_heading_rate:state_order" in failed
    assert result["missing_in_rust"] == ["wnoa_nominal"]
    assert result["missing_in_baseline"] == []


def test_compare_fails_missing_output_object_and_output_field() -> None:
    module = _load_module()
    rust = _result()
    baseline = _result()
    rust["cases"][0].pop("output")
    rust["cases"][1]["output"].pop("prior_residual")

    result = module.compare(rust, baseline)

    failed = {check["name"]: check for check in result["failed_checks"]}
    assert failed["wnoj_positive_heading_rate:output"]["reason"] == (
        "missing_output_object"
    )
    assert failed["wnoa_nominal:output.prior_residual"]["reason"] == (
        "missing_in_actual"
    )


def test_compare_cli_json_out_and_enforce(tmp_path: Path) -> None:
    module = _load_module()
    rust_path = tmp_path / "rust.json"
    baseline_path = tmp_path / "baseline.json"
    out_path = tmp_path / "compare.json"
    rust = _result()
    baseline = _result()
    rust["cases"][0]["output"]["prior_residual"] = [9.0, 9.0]
    rust_path.write_text(json.dumps(rust), encoding="utf-8")
    baseline_path.write_text(json.dumps(baseline), encoding="utf-8")

    code = module.main(
        [
            "--rust-result",
            str(rust_path),
            "--baseline",
            str(baseline_path),
            "--json-out",
            str(out_path),
            "--json",
            "--enforce",
        ]
    )

    assert code == 1
    saved = json.loads(out_path.read_text(encoding="utf-8"))
    assert saved["schema"] == "lingtu.pct_gpmp_math.compare.v1"
    assert saved["verdict"] == "fail"

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "tools" / "bench" / "pct_gpmp_math_acceptance.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("pct_gpmp_math_acceptance", SCRIPT)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _math_payload(producer: str) -> dict:
    return {
        "schema": "lingtu.pct_gpmp_math.result.v1",
        "producer": producer,
        "cases": [
            {"case": "wnoj_positive_heading_rate"},
            {"case": "wnoj_negative_heading_rate"},
            {"case": "wnoj_heading_rate_inside_limit"},
            {"case": "wnoa_nominal"},
        ],
    }


def _passing_payload() -> dict:
    return {
        "rust_fixture": {"payload": _math_payload("rust")},
        "cpp_baseline": {"payload": _math_payload("cpp")},
        "comparison": {"payload": {"verdict": "pass"}},
        "migration_status": {
            "ok": True,
            "claims": {"pct_gpmp_math_kernel_parity": True},
            "pct_gpmp_math_readiness": {"status": "pass"},
        },
    }


def test_evaluate_acceptance_requires_required_cases_compare_and_status_claim() -> None:
    module = _load_module()
    payload = _passing_payload()

    result = module.evaluate_acceptance(payload)

    assert result["verdict"] == "pass"
    assert result["failed_checks"] == []
    assert result["required_cases"] == list(module.REQUIRED_CASES)

    payload["rust_fixture"]["payload"]["cases"].pop()
    payload["comparison"]["payload"]["verdict"] = "fail"
    payload["migration_status"]["claims"]["pct_gpmp_math_kernel_parity"] = False

    result = module.evaluate_acceptance(payload)

    assert result["verdict"] == "fail"
    assert "rust_missing_case:wnoa_nominal" in result["failed_checks"]
    assert "comparison_failed" in result["failed_checks"]
    assert (
        "migration_status_claim_false:pct_gpmp_math_kernel_parity"
        in result["failed_checks"]
    )


def test_parser_defaults_to_strict_math_tolerances() -> None:
    module = _load_module()
    args = module.build_parser().parse_args([])

    assert args.abs_tol == 1e-9
    assert args.rel_tol == 1e-9
    assert args.config == "Release"


def test_artifact_path_accepts_repo_relative_and_absolute_paths() -> None:
    module = _load_module()
    relative = "tools/bench/pct_gpmp_math_compare.py"
    absolute = SCRIPT.resolve()

    assert module._artifact_path(relative) == ROOT / relative
    assert module._artifact_path(str(absolute)) == absolute

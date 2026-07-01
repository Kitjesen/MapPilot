#!/usr/bin/env python3
"""Compare GPMP pure-math Rust output with a C++ baseline JSON."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


SCHEMA = "lingtu.pct_gpmp_math.compare.v1"
RESULT_SCHEMA = "lingtu.pct_gpmp_math.result.v1"
DEFAULT_ABS_TOL = 1e-9
DEFAULT_REL_TOL = 1e-9
_MISSING = object()


def _load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8-sig") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return data


def _num(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _flatten_numbers(value: Any) -> list[float] | None:
    if isinstance(value, list):
        out: list[float] = []
        for item in value:
            nested = _flatten_numbers(item)
            if nested is None:
                return None
            out.extend(nested)
        return out
    number = _num(value)
    return [number] if number is not None else None


def _shape(value: Any) -> list[int] | None:
    if not isinstance(value, list):
        return []
    length = len(value)
    if length == 0:
        return [0]
    nested = _shape(value[0])
    if nested is None:
        return None
    for item in value[1:]:
        if _shape(item) != nested:
            return None
    return [length, *nested]


def _check_equal(name: str, expected: Any, actual: Any) -> dict[str, Any]:
    return {
        "name": name,
        "passed": expected == actual,
        "expected": expected,
        "actual": actual,
    }


def _check_numeric(
    name: str,
    expected: Any,
    actual: Any,
    abs_tol: float,
    rel_tol: float,
) -> dict[str, Any]:
    expected_shape = _shape(expected)
    actual_shape = _shape(actual)
    if expected_shape != actual_shape:
        return {
            "name": name,
            "passed": False,
            "expected_shape": expected_shape,
            "actual_shape": actual_shape,
            "reason": "shape_mismatch",
        }
    expected_values = _flatten_numbers(expected)
    actual_values = _flatten_numbers(actual)
    if expected_values is None or actual_values is None or len(expected_values) != len(actual_values):
        return {
            "name": name,
            "passed": False,
            "expected": expected,
            "actual": actual,
            "reason": "non_numeric_or_length_mismatch",
        }
    max_error = 0.0
    max_allowed = 0.0
    for lhs, rhs in zip(expected_values, actual_values):
        error = abs(lhs - rhs)
        allowed = max(abs_tol, rel_tol * max(1.0, abs(lhs), abs(rhs)))
        max_error = max(max_error, error)
        max_allowed = max(max_allowed, allowed)
    return {
        "name": name,
        "passed": max_error <= max_allowed,
        "expected_shape": expected_shape,
        "actual_shape": actual_shape,
        "abs_tol": abs_tol,
        "rel_tol": rel_tol,
        "max_error": max_error,
        "max_allowed_error": max_allowed,
    }


def _check_nested_numeric(
    name: str,
    expected: Any,
    actual: Any,
    abs_tol: float,
    rel_tol: float,
) -> list[dict[str, Any]]:
    checks: list[dict[str, Any]] = []

    def walk(path: str, expected_value: Any, actual_value: Any) -> None:
        if expected_value is _MISSING or actual_value is _MISSING:
            checks.append(
                {
                    "name": path,
                    "passed": False,
                    "reason": (
                        "missing_in_actual"
                        if actual_value is _MISSING
                        else "missing_in_baseline"
                    ),
                }
            )
            return
        if isinstance(expected_value, dict) or isinstance(actual_value, dict):
            if not isinstance(expected_value, dict) or not isinstance(actual_value, dict):
                checks.append(
                    {
                        "name": path,
                        "passed": False,
                        "expected_type": type(expected_value).__name__,
                        "actual_type": type(actual_value).__name__,
                        "reason": "type_mismatch",
                    }
                )
                return
            for key in sorted(set(expected_value) | set(actual_value)):
                walk(
                    f"{path}.{key}",
                    expected_value.get(key, _MISSING),
                    actual_value.get(key, _MISSING),
                )
            return
        checks.append(_check_numeric(path, expected_value, actual_value, abs_tol, rel_tol))

    walk(name, expected, actual)
    return checks


def _cases_by_name(payload: dict[str, Any]) -> tuple[dict[str, dict[str, Any]], list[str]]:
    cases = payload.get("cases")
    if not isinstance(cases, list):
        return {}, ["<cases_not_list>"]
    out: dict[str, dict[str, Any]] = {}
    duplicates: list[str] = []
    for case in cases:
        if isinstance(case, dict) and isinstance(case.get("case"), str):
            case_name = case["case"]
            if case_name in out:
                duplicates.append(case_name)
                continue
            out[case_name] = case
    return out, duplicates


def compare(
    rust: dict[str, Any],
    baseline: dict[str, Any],
    *,
    abs_tol: float = DEFAULT_ABS_TOL,
    rel_tol: float = DEFAULT_REL_TOL,
) -> dict[str, Any]:
    checks: list[dict[str, Any]] = []
    checks.append(_check_equal("rust_schema", RESULT_SCHEMA, rust.get("schema")))
    checks.append(_check_equal("baseline_schema", RESULT_SCHEMA, baseline.get("schema")))

    rust_cases, rust_duplicates = _cases_by_name(rust)
    baseline_cases, baseline_duplicates = _cases_by_name(baseline)
    checks.append(_check_equal("rust_duplicate_case_names", [], sorted(rust_duplicates)))
    checks.append(
        _check_equal("baseline_duplicate_case_names", [], sorted(baseline_duplicates))
    )
    checks.append(_check_equal("case_names", sorted(baseline_cases), sorted(rust_cases)))

    missing_in_rust = sorted(set(baseline_cases) - set(rust_cases))
    missing_in_baseline = sorted(set(rust_cases) - set(baseline_cases))
    matched_cases = sorted(set(rust_cases) & set(baseline_cases))

    for case_name in matched_cases:
        rust_case = rust_cases[case_name]
        baseline_case = baseline_cases[case_name]
        checks.append(
            _check_equal(
                f"{case_name}:mode",
                baseline_case.get("mode"),
                rust_case.get("mode"),
            )
        )
        checks.append(
            _check_equal(
                f"{case_name}:state_order",
                baseline_case.get("state_order"),
                rust_case.get("state_order"),
            )
        )
        checks.extend(
            _check_nested_numeric(
                f"{case_name}:input",
                baseline_case.get("input", {}),
                rust_case.get("input", {}),
                abs_tol,
                rel_tol,
            )
        )
        baseline_output = baseline_case.get("output")
        rust_output = rust_case.get("output")
        if not isinstance(baseline_output, dict) or not isinstance(rust_output, dict):
            checks.append(
                {
                    "name": f"{case_name}:output",
                    "passed": False,
                    "reason": "missing_output_object",
                }
            )
            continue
        checks.extend(
            _check_nested_numeric(
                f"{case_name}:output",
                baseline_output,
                rust_output,
                abs_tol,
                rel_tol,
            )
        )

    failed = [check for check in checks if not check.get("passed")]
    return {
        "schema": SCHEMA,
        "verdict": "pass" if not failed else "fail",
        "checks": checks,
        "failed_checks": failed,
        "skipped_checks": [],
        "tolerances": {"abs_tol": abs_tol, "rel_tol": rel_tol},
        "matched_cases": matched_cases,
        "missing_in_rust": missing_in_rust,
        "missing_in_baseline": missing_in_baseline,
        "rust_producer": rust.get("producer"),
        "baseline_producer": baseline.get("producer"),
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rust-result", required=True, type=Path)
    parser.add_argument("--baseline", required=True, type=Path)
    parser.add_argument("--abs-tol", type=float, default=DEFAULT_ABS_TOL)
    parser.add_argument("--rel-tol", type=float, default=DEFAULT_REL_TOL)
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--json", action="store_true", help="emit JSON only")
    parser.add_argument("--enforce", action="store_true", help="exit nonzero on failed verdict")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    result = compare(
        _load_json(args.rust_result),
        _load_json(args.baseline),
        abs_tol=args.abs_tol,
        rel_tol=args.rel_tol,
    )
    text = json.dumps(result, indent=2, sort_keys=True)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(text + "\n", encoding="utf-8")
    if args.json:
        print(text)
    else:
        print(f"PCT/GPMP math comparison: {result['verdict']}")
        for check in result["checks"]:
            marker = "ok" if check.get("passed") else "fail"
            print(f"  {marker}: {check['name']}")
    return 1 if args.enforce and result["verdict"] != "pass" else 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Compare PCT preview JSON against a machine-readable golden fixture."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any


SCHEMA = "lingtu.pct.preview.compare.v1"
GOLDEN_SCHEMA = "lingtu.pct.preview.golden.v1"


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


def _vec(value: Any) -> list[float] | None:
    if not isinstance(value, list):
        return None
    out: list[float] = []
    for item in value:
        number = _num(item)
        if number is None:
            return None
        out.append(number)
    return out


def _check_bool(name: str, expected: bool, actual: Any) -> dict[str, Any]:
    passed = bool(actual) is bool(expected)
    return {
        "name": name,
        "passed": passed,
        "expected": bool(expected),
        "actual": bool(actual),
    }


def _check_equal(name: str, expected: Any, actual: Any) -> dict[str, Any]:
    return {
        "name": name,
        "passed": actual == expected,
        "expected": expected,
        "actual": actual,
    }


def _check_range(name: str, expected: dict[str, Any], actual_value: Any) -> dict[str, Any]:
    actual = _num(actual_value)
    if actual is None:
        return {
            "name": name,
            "passed": False,
            "expected": expected,
            "actual": actual_value,
            "reason": "actual_not_finite_number",
        }

    lower = _num(expected.get("min"))
    upper = _num(expected.get("max"))
    target = _num(expected.get("target", expected.get("nominal")))
    abs_tol = _num(expected.get("abs_tol"))
    passed = True
    if lower is not None and actual < lower:
        passed = False
    if upper is not None and actual > upper:
        passed = False
    if target is not None and abs_tol is not None and abs(actual - target) > abs_tol:
        passed = False
    return {
        "name": name,
        "passed": passed,
        "expected": expected,
        "actual": actual,
    }


def _check_vector(
    name: str,
    expected: list[float],
    actual_value: Any,
    *,
    abs_tol: float,
) -> dict[str, Any]:
    actual = _vec(actual_value)
    if actual is None or len(actual) != len(expected):
        return {
            "name": name,
            "passed": False,
            "expected": expected,
            "actual": actual_value,
            "reason": "shape_or_number_mismatch",
        }
    max_error = max((abs(a - b) for a, b in zip(actual, expected)), default=0.0)
    return {
        "name": name,
        "passed": max_error <= abs_tol,
        "expected": expected,
        "actual": actual,
        "abs_tol": abs_tol,
        "max_error": max_error,
    }


def _check_sample_point(
    expected: dict[str, Any],
    actual_samples: dict[int, list[float]],
    *,
    abs_tol: float,
) -> dict[str, Any]:
    index = expected.get("index")
    if not isinstance(index, int):
        return {
            "name": "path_sample:<invalid>",
            "passed": False,
            "expected": expected,
            "actual": None,
            "reason": "sample_index_missing",
        }
    actual = actual_samples.get(index)
    return _check_vector(
        f"path_sample:{index}",
        expected.get("point", []),
        actual,
        abs_tol=float(expected.get("abs_tol", abs_tol)),
    )


def _sample_map(samples: Any) -> dict[int, list[float]]:
    if not isinstance(samples, list):
        return {}
    out: dict[int, list[float]] = {}
    for sample in samples:
        if not isinstance(sample, dict):
            continue
        index = sample.get("index")
        point = _vec(sample.get("point"))
        if isinstance(index, int) and point is not None:
            out[index] = point
    return out


def _fraction_sample_map(samples: Any) -> dict[float, list[float]]:
    if not isinstance(samples, list):
        return {}
    out: dict[float, list[float]] = {}
    for sample in samples:
        if not isinstance(sample, dict):
            continue
        fraction = _num(sample.get("fraction"))
        point = _vec(sample.get("point"))
        if fraction is not None and point is not None:
            out[round(fraction, 6)] = point
    return out


def _actual_input(actual: dict[str, Any]) -> dict[str, Any]:
    value = actual.get("input")
    return value if isinstance(value, dict) else {}


def _actual_point(actual: dict[str, Any], field: str) -> Any:
    if field in actual:
        return actual.get(field)
    return _actual_input(actual).get(field)


def _actual_input_identity(actual: dict[str, Any], field: str) -> Any:
    input_spec = _actual_input(actual)
    if field in input_spec:
        return input_spec.get(field)

    tomogram_file = input_spec.get("tomogram_file")
    if not isinstance(tomogram_file, dict):
        return None
    aliases = {
        "tomogram_sha256": "sha256",
        "tomogram_size_bytes": "size_bytes",
        "tomogram_exists": "exists",
        "tomogram_data_shape": "data_shape",
        "tomogram_data_dtype": "data_dtype",
        "tomogram_schema": "schema",
        "tomogram_schema_version": "schema_version",
        "tomogram_version": "version",
    }
    nested_field = aliases.get(field, field)
    return tomogram_file.get(nested_field)


def _actual_metric(actual: dict[str, Any], field: str) -> Any:
    if field in actual:
        return actual[field]
    path = actual.get("path") if isinstance(actual.get("path"), dict) else {}
    mapping = {
        "path_count": "count",
        "path_distance_m": "distance_m",
        "goal_error_m": "goal_error_m",
        "start_error_m": "start_error_m",
    }
    nested = mapping.get(field)
    if nested:
        return path.get(nested)
    return None


def _actual_path(actual: dict[str, Any]) -> dict[str, Any]:
    value = actual.get("path")
    return value if isinstance(value, dict) else {}


def _actual_optimizer_accessors(actual: dict[str, Any]) -> dict[str, Any]:
    value = actual.get("optimizer_accessors")
    return value if isinstance(value, dict) else {}


def _check_min_columns(name: str, minimum: int, shape_value: Any) -> dict[str, Any]:
    shape = _vec(shape_value)
    actual_columns = int(shape[1]) if shape is not None and len(shape) >= 2 else None
    return {
        "name": name,
        "passed": actual_columns is not None and actual_columns >= minimum,
        "expected": f">={minimum}",
        "actual": actual_columns,
    }


def _actual_samples_by_index(actual: dict[str, Any]) -> Any:
    if "path_samples" in actual:
        return actual.get("path_samples")
    path = actual.get("path") if isinstance(actual.get("path"), dict) else {}
    samples = path.get("samples") if isinstance(path.get("samples"), dict) else {}
    return samples.get("by_index")


def _actual_samples_by_fraction(actual: dict[str, Any]) -> Any:
    path = actual.get("path") if isinstance(actual.get("path"), dict) else {}
    samples = path.get("samples") if isinstance(path.get("samples"), dict) else {}
    return samples.get("by_arclength_fraction")


def _check_expected_value(name: str, expected: Any, actual: Any) -> dict[str, Any]:
    if isinstance(expected, dict):
        if "one_of" in expected:
            allowed = expected.get("one_of")
            return {
                "name": name,
                "passed": isinstance(allowed, list) and actual in allowed,
                "expected": expected,
                "actual": actual,
            }
        if any(key in expected for key in ("min", "max", "target", "nominal", "abs_tol")):
            return _check_range(name, expected, actual)
    return _check_equal(name, expected, actual)


def compare(golden: dict[str, Any], actual: dict[str, Any]) -> dict[str, Any]:
    if golden.get("schema") != GOLDEN_SCHEMA:
        return {
            "schema": SCHEMA,
            "case": golden.get("case", ""),
            "verdict": "fail",
            "checks": [
                {
                    "name": "golden_schema",
                    "passed": False,
                    "expected": GOLDEN_SCHEMA,
                    "actual": golden.get("schema"),
                }
            ],
        }

    checks: list[dict[str, Any]] = []
    skipped: list[dict[str, Any]] = []
    expected = golden.get("expected") or {}
    input_spec = golden.get("input") or {}
    tolerances = golden.get("tolerances") or {}
    point_tol = float(tolerances.get("point_abs_tol", 1e-6))
    sample_tol = float(tolerances.get("sample_point_abs_tol", point_tol))

    expected_actual_schema = golden.get("actual_schema") or expected.get("actual_schema")
    if expected_actual_schema:
        checks.append(_check_equal("actual_schema", expected_actual_schema, actual.get("schema")))

    if "ok" in expected:
        checks.append(_check_bool("ok", bool(expected["ok"]), actual.get("ok")))
    if "planner" in expected:
        checks.append(_check_equal("planner", expected["planner"], actual.get("planner")))
    if "start" in input_spec:
        checks.append(
            _check_vector("start", input_spec["start"], _actual_point(actual, "start"), abs_tol=point_tol)
        )
    if "goal" in input_spec:
        checks.append(
            _check_vector("goal", input_spec["goal"], _actual_point(actual, "goal"), abs_tol=point_tol)
        )
    for field in (
        "obstacle_thr",
        "tomogram_sha256",
        "tomogram_size_bytes",
        "tomogram_exists",
        "tomogram_data_shape",
        "tomogram_data_dtype",
        "tomogram_schema",
        "tomogram_schema_version",
        "tomogram_version",
    ):
        if field in input_spec:
            checks.append(
                _check_expected_value(
                    f"input:{field}",
                    input_spec[field],
                    _actual_input_identity(actual, field),
                )
            )
    tomogram_file = input_spec.get("tomogram_file")
    if isinstance(tomogram_file, dict):
        for field in ("sha256", "size_bytes", "exists"):
            if field in tomogram_file:
                checks.append(
                    _check_equal(
                        f"input:tomogram_file.{field}",
                        tomogram_file[field],
                        _actual_input_identity(actual, field),
                    )
                )

    for field in ("path_count", "path_distance_m", "goal_error_m"):
        if isinstance(expected.get(field), dict):
            checks.append(_check_range(field, expected[field], _actual_metric(actual, field)))
        elif field in expected:
            checks.append(_check_equal(field, expected[field], _actual_metric(actual, field)))

    path_expected = expected.get("path")
    if isinstance(path_expected, dict):
        actual_path = _actual_path(actual)
        if "finite" in path_expected:
            checks.append(
                _check_bool("path.finite", bool(path_expected["finite"]), actual_path.get("finite"))
            )
        if "min_columns" in path_expected:
            checks.append(
                _check_min_columns(
                    "path.min_columns",
                    int(path_expected["min_columns"]),
                    actual_path.get("shape"),
                )
            )
        if path_expected.get("count_matches_path_count"):
            checks.append(
                _check_equal(
                    "path.count_matches_path_count",
                    _actual_metric(actual, "path_count"),
                    actual_path.get("count"),
                )
            )

    expected_samples = golden.get("path_samples") or expected.get("path_samples")
    if isinstance(expected_samples, list):
        raw_samples = _actual_samples_by_index(actual)
        if raw_samples is None:
            skipped.append(
                {
                    "name": "path_samples",
                    "reason": "actual_has_no_index_samples",
                }
            )
        else:
            actual_samples = _sample_map(raw_samples)
            for sample in expected_samples:
                if isinstance(sample, dict):
                    checks.append(_check_sample_point(sample, actual_samples, abs_tol=sample_tol))

    nested_expected_samples = (
        golden.get("path", {})
        .get("samples", {})
        .get("by_arclength_fraction")
        if isinstance(golden.get("path"), dict)
        else None
    )
    if isinstance(nested_expected_samples, list):
        raw_samples = _actual_samples_by_fraction(actual)
        if raw_samples is None:
            skipped.append(
                {
                    "name": "path.samples.by_arclength_fraction",
                    "reason": "actual_has_no_arclength_samples",
                }
            )
        else:
            actual_samples = _fraction_sample_map(raw_samples)
            for sample in nested_expected_samples:
                if not isinstance(sample, dict):
                    continue
                fraction = _num(sample.get("fraction"))
                if fraction is None:
                    continue
                actual_point = actual_samples.get(round(fraction, 6))
                checks.append(
                    _check_vector(
                        f"path_fraction_sample:{fraction:g}",
                        sample.get("point", []),
                        actual_point,
                        abs_tol=float(sample.get("abs_tol", sample_tol)),
                    )
                )

    expected_diagnostics = golden.get("expected_diagnostics") or expected.get("diagnostics")
    if isinstance(expected_diagnostics, dict):
        actual_diagnostics = actual.get("diagnostics") or {}
        for field, field_expected in expected_diagnostics.items():
            if not isinstance(actual_diagnostics, dict) or field not in actual_diagnostics:
                skipped.append(
                    {
                        "name": f"diagnostics:{field}",
                        "reason": "actual_has_no_diagnostic_field",
                    }
                )
                continue
            checks.append(
                _check_expected_value(
                    f"diagnostics:{field}",
                    field_expected,
                    actual_diagnostics.get(field),
                )
            )

    expected_optimizer_accessors = (
        golden.get("expected_optimizer_accessors") or expected.get("optimizer_accessors")
    )
    if isinstance(expected_optimizer_accessors, dict):
        actual_accessors = _actual_optimizer_accessors(actual)
        for field, field_expected in expected_optimizer_accessors.items():
            checks.append(
                _check_expected_value(
                    f"optimizer_accessors:{field}",
                    field_expected,
                    actual_accessors.get(field),
                )
            )

    for field in golden.get("required_diagnostics", []):
        diagnostics = actual.get("diagnostics") or {}
        checks.append(
            {
                "name": f"required_diagnostics:{field}",
                "passed": isinstance(diagnostics, dict) and field in diagnostics,
                "expected": "present",
                "actual": "present" if isinstance(diagnostics, dict) and field in diagnostics else "missing",
            }
        )

    for field in golden.get("required_fields", []):
        checks.append(
            {
                "name": f"required_field:{field}",
                "passed": field in actual,
                "expected": "present",
                "actual": "present" if field in actual else "missing",
            }
        )

    failed = [check for check in checks if not check.get("passed")]
    return {
        "schema": SCHEMA,
        "case": golden.get("case", ""),
        "verdict": "pass" if not failed else "fail",
        "checks": checks,
        "skipped_checks": skipped,
        "failed_checks": failed,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--golden", required=True, type=Path)
    parser.add_argument("--actual-json", required=True, type=Path)
    parser.add_argument("--json", action="store_true", help="emit JSON only")
    parser.add_argument("--enforce", action="store_true", help="exit nonzero on failed verdict")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    result = compare(_load_json(args.golden), _load_json(args.actual_json))
    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        print(f"PCT preview comparison: {result['verdict']}")
        for check in result["checks"]:
            marker = "ok" if check.get("passed") else "fail"
            print(f"  {marker}: {check['name']}")
    return 1 if args.enforce and result["verdict"] != "pass" else 0


if __name__ == "__main__":
    raise SystemExit(main())

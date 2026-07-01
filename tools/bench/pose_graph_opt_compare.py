from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import subprocess
import sys
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
KERNEL = ROOT / "src" / "kernels" / "slam" / "pose_graph_opt"
MANIFEST = KERNEL / "Cargo.toml"
DEFAULT_TOLERANCES: dict[str, float | None] = {
    "final_cost_delta_abs_max": 1e-6,
    "final_residual_rms_delta_abs_max": 1e-6,
    "final_residual_max_delta_abs_max": 1e-6,
    "pose_translation_rmse_max": 1e-4,
    "pose_rotation_rmse_rad_max": 1e-4,
    "rust_to_baseline_time_ratio_max": None,
}


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run Rust pose_graph_opt benchmarks and optionally compare with a C++/GTSAM baseline JSON."
    )
    subparsers = parser.add_subparsers(dest="command")
    compare_suite = subparsers.add_parser(
        "compare-suite",
        help="Compare Rust and optional C++/GTSAM suite JSON outputs.",
    )
    compare_suite.add_argument("--rust-result", type=Path, required=True)
    compare_suite.add_argument("--baseline", type=Path, required=True)
    compare_suite.add_argument("--fixture-dir", type=Path, required=True)
    compare_suite.add_argument("--json-out", type=Path)
    add_tolerance_arguments(compare_suite)
    compare_suite.add_argument(
        "--enforce",
        action="store_true",
        help="Exit nonzero when suite comparison summary fails.",
    )

    parser.add_argument(
        "--poses",
        nargs="+",
        type=int,
        default=[16, 64, 256],
        help="Pose counts to benchmark.",
    )
    parser.add_argument(
        "--baseline",
        type=Path,
        help="Optional JSON file produced by the legacy C++/GTSAM benchmark.",
    )
    parser.add_argument(
        "--fixture",
        type=Path,
        help="Optional fixture JSON; baseline_tolerances override default thresholds.",
    )
    parser.add_argument(
        "--rust-result",
        type=Path,
        help="Optional JSON file produced by the Rust fixture runner; skips cargo benchmark.",
    )
    parser.add_argument(
        "--json-out",
        type=Path,
        help="Optional path for the combined comparison JSON.",
    )
    add_tolerance_arguments(parser)
    parser.add_argument(
        "--enforce",
        action="store_true",
        help="Exit nonzero when baseline comparison summary fails.",
    )
    args = parser.parse_args()
    if args.command == "compare-suite":
        return run_compare_suite_command(args)

    fixture_data = json.loads(args.fixture.read_text(encoding="utf-8")) if args.fixture else None

    rust_result = (
        json.loads(args.rust_result.read_text(encoding="utf-8"))
        if args.rust_result
        else run_rust_benchmark(args.poses)
    )
    output: dict[str, Any] = {"rust": rust_result}
    if args.baseline:
        baseline = json.loads(args.baseline.read_text(encoding="utf-8"))
        tolerances = comparison_tolerances(
            fixture=fixture_data,
            overrides={
                "final_cost_delta_abs_max": args.final_cost_delta_abs_max,
                "final_residual_rms_delta_abs_max": args.final_residual_rms_delta_abs_max,
                "final_residual_max_delta_abs_max": args.final_residual_max_delta_abs_max,
                "pose_translation_rmse_max": args.pose_translation_rmse_max,
                "pose_rotation_rmse_rad_max": args.pose_rotation_rmse_rad_max,
                "rust_to_baseline_time_ratio_max": args.rust_to_baseline_time_ratio_max,
            },
        )
        comparison = compare_cases(rust_result, baseline)
        output["cpp_baseline"] = baseline
        output["comparison"] = comparison
        output["summary"] = summarize_comparison(
            rust_result,
            baseline,
            comparison,
            tolerances,
            expected_fixture_hash=fixture_hash(fixture_data) if fixture_data else None,
        )

    text = json.dumps(output, indent=2, sort_keys=True)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(text + "\n", encoding="utf-8")
    print(text)
    if args.enforce and output.get("summary", {}).get("verdict") != "pass":
        return 1
    return 0


def add_tolerance_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--final-cost-delta-abs-max",
        type=float,
        help="Override final cost delta tolerance.",
    )
    parser.add_argument(
        "--final-residual-rms-delta-abs-max",
        type=float,
        help="Override final residual RMS delta tolerance.",
    )
    parser.add_argument(
        "--final-residual-max-delta-abs-max",
        type=float,
        help="Override final residual max delta tolerance.",
    )
    parser.add_argument(
        "--pose-translation-rmse-max",
        type=float,
        help="Override optimized pose translation RMSE tolerance.",
    )
    parser.add_argument(
        "--pose-rotation-rmse-rad-max",
        type=float,
        help="Override optimized pose rotation RMSE tolerance.",
    )
    parser.add_argument(
        "--rust-to-baseline-time-ratio-max",
        type=float,
        help="Optional performance ratio ceiling for Rust elapsed time / baseline elapsed time.",
    )


def tolerance_overrides_from_args(args: argparse.Namespace) -> dict[str, float | None]:
    return {
        "final_cost_delta_abs_max": args.final_cost_delta_abs_max,
        "final_residual_rms_delta_abs_max": args.final_residual_rms_delta_abs_max,
        "final_residual_max_delta_abs_max": args.final_residual_max_delta_abs_max,
        "pose_translation_rmse_max": args.pose_translation_rmse_max,
        "pose_rotation_rmse_rad_max": args.pose_rotation_rmse_rad_max,
        "rust_to_baseline_time_ratio_max": args.rust_to_baseline_time_ratio_max,
    }


def run_compare_suite_command(args: argparse.Namespace) -> int:
    rust_result = json.loads(args.rust_result.read_text(encoding="utf-8"))
    baseline = json.loads(args.baseline.read_text(encoding="utf-8"))
    expected_hashes, tolerances_by_case, fixture_records = load_suite_fixture_expectations(
        args.fixture_dir,
        overrides=tolerance_overrides_from_args(args),
    )
    comparison = compare_cases(rust_result, baseline)
    output = {
        "rust": rust_result,
        "cpp_baseline": baseline,
        "comparison": comparison,
        "summary": summarize_comparison(
            rust_result,
            baseline,
            comparison,
            comparison_tolerances(overrides=tolerance_overrides_from_args(args)),
            expected_fixture_hashes_by_case=expected_hashes,
            tolerances_by_case=tolerances_by_case,
        ),
        "fixtures": fixture_records,
    }
    text = json.dumps(output, indent=2, sort_keys=True)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(text + "\n", encoding="utf-8")
    print(text)
    if args.enforce and output["summary"]["verdict"] != "pass":
        return 1
    return 0


def run_rust_benchmark(poses: list[int]) -> dict[str, Any]:
    env = os.environ.copy()
    command = [
        "cargo",
        "run",
        "--release",
        "--manifest-path",
        str(MANIFEST),
        "--example",
        "benchmark",
        "--",
        *[str(value) for value in poses],
    ]
    completed = subprocess.run(
        command,
        cwd=KERNEL,
        env=env,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    return json.loads(completed.stdout)


def compare_cases(rust_result: dict[str, Any], baseline: dict[str, Any]) -> list[dict[str, Any]]:
    rust_cases = {case_key(case): case for case in rust_result.get("cases", [])}
    baseline_cases = {case_key(case): case for case in baseline.get("cases", [])}
    comparison = []
    for key in sorted(set(rust_cases) & set(baseline_cases)):
        rust = rust_cases[key]
        base = baseline_cases[key]
        comparison.append(
            {
                "case": key[0],
                "poses": key[1],
                "rust_fixture_hash": case_fixture_hash(rust),
                "baseline_fixture_hash": case_fixture_hash(base),
                "rust_elapsed_ms": rust["elapsed_ms"],
                "baseline_elapsed_ms": base["elapsed_ms"],
                "rust_to_baseline_time_ratio": safe_ratio(
                    rust["elapsed_ms"], base["elapsed_ms"]
                ),
                "rust_final_cost": rust["final_cost"],
                "baseline_final_cost": base.get("final_cost"),
                "rust_factors": rust.get("factors"),
                "baseline_factors": base.get("factors"),
                "final_cost_delta": (
                    rust["final_cost"] - base["final_cost"]
                    if "final_cost" in base
                    else None
                ),
                "rust_status": rust.get("status"),
                "baseline_status": base.get("status"),
                "rust_converged": rust.get("converged"),
                "baseline_converged": base.get("converged"),
                "rust_written": rust.get("written"),
                "baseline_written": base.get("written"),
                "rust_final_residual_rms": rust.get("final_residual_rms"),
                "baseline_final_residual_rms": base.get("final_residual_rms"),
                "final_residual_rms_delta": optional_delta(
                    rust.get("final_residual_rms"), base.get("final_residual_rms")
                ),
                "rust_final_residual_max": rust.get("final_residual_max"),
                "baseline_final_residual_max": base.get("final_residual_max"),
                "final_residual_max_delta": optional_delta(
                    rust.get("final_residual_max"), base.get("final_residual_max")
                ),
                "rust_iterations": rust.get("iterations"),
                "baseline_iterations": base.get("iterations"),
                "rust_accepted_steps": rust.get("accepted_steps"),
                "baseline_accepted_steps": base.get("accepted_steps"),
                "rust_optimized_poses": rust.get("optimized_poses"),
                "baseline_optimized_poses": base.get("optimized_poses"),
                "pose_translation_rmse": pose_translation_rmse(
                    rust.get("optimized_poses"), base.get("optimized_poses")
                ),
                "pose_rotation_rmse_rad": pose_rotation_rmse_rad(
                    rust.get("optimized_poses"), base.get("optimized_poses")
                ),
            }
        )
    return comparison


def comparison_tolerances(
    *,
    fixture: dict[str, Any] | None = None,
    overrides: dict[str, float | None] | None = None,
) -> dict[str, float | None]:
    tolerances = dict(DEFAULT_TOLERANCES)
    if fixture:
        for key, value in fixture.get("baseline_tolerances", {}).items():
            if key in tolerances:
                tolerances[key] = float(value)
    for key, value in (overrides or {}).items():
        if value is not None:
            tolerances[key] = value
    return tolerances


def summarize_comparison(
    rust_result: dict[str, Any],
    baseline: dict[str, Any],
    comparison: list[dict[str, Any]],
    tolerances: dict[str, float | None],
    *,
    expected_fixture_hash: str | None = None,
    expected_fixture_hashes_by_case: dict[tuple[str, int], str] | None = None,
    tolerances_by_case: dict[tuple[str, int], dict[str, float | None]] | None = None,
) -> dict[str, Any]:
    rust_keys = {case_key(case) for case in rust_result.get("cases", [])}
    baseline_keys = {case_key(case) for case in baseline.get("cases", [])}
    matched_keys = sorted(rust_keys & baseline_keys)
    missing_in_rust = sorted(baseline_keys - rust_keys)
    missing_in_baseline = sorted(rust_keys - baseline_keys)

    checks: list[dict[str, Any]] = []
    for item in comparison:
        key = (item["case"], item["poses"])
        checks.extend(case_checks(item, (tolerances_by_case or {}).get(key, tolerances)))

    failures = [
        check
        for check in checks
        if check["status"] == "fail"
    ]
    skipped = [
        check
        for check in checks
        if check["status"] == "skipped"
    ]
    if missing_in_rust:
        failures.append(
            {
                "case": None,
                "poses": None,
                "metric": "missing_in_rust",
                "status": "fail",
                "actual": len(missing_in_rust),
                "limit": 0,
            }
        )
    if missing_in_baseline:
        failures.append(
            {
                "case": None,
                "poses": None,
                "metric": "missing_in_baseline",
                "status": "fail",
                "actual": len(missing_in_baseline),
                "limit": 0,
            }
        )
    identity_checks = fixture_identity_checks(
        rust_result,
        baseline,
        expected_fixture_hash=expected_fixture_hash,
    )
    identity_checks.extend(
        suite_fixture_identity_checks(
            comparison,
            expected_fixture_hashes_by_case or {},
        )
    )
    failures.extend(check for check in identity_checks if check["status"] == "fail")

    summary = {
        "verdict": "pass" if not failures and matched_keys else "fail",
        "fixture_hashes": {
            "expected": expected_fixture_hash,
            "rust": result_fixture_hash(rust_result),
            "baseline": result_fixture_hash(baseline),
        },
        "matched_cases": [case_key_json(key) for key in matched_keys],
        "missing_in_rust": [case_key_json(key) for key in missing_in_rust],
        "missing_in_baseline": [case_key_json(key) for key in missing_in_baseline],
        "tolerances": tolerances,
        "checks": checks,
        "identity_checks": identity_checks,
        "failed_checks": failures,
        "skipped_checks": skipped,
    }
    if tolerances_by_case:
        summary["tolerances_by_case"] = [
            {
                **case_key_json(key),
                "tolerances": tolerances_by_case[key],
            }
            for key in sorted(tolerances_by_case)
        ]
    if expected_fixture_hashes_by_case:
        summary["fixture_hashes_by_case"] = [
            {
                **case_key_json(key),
                "expected": expected_fixture_hashes_by_case[key],
            }
            for key in sorted(expected_fixture_hashes_by_case)
        ]
    return summary


def case_checks(
    item: dict[str, Any],
    tolerances: dict[str, float | None],
) -> list[dict[str, Any]]:
    return [
        expected_check(item, metric="rust_status", expected=0),
        expected_check(item, metric="baseline_status", expected=0),
        expected_check(item, metric="rust_converged", expected=True),
        expected_check(item, metric="baseline_converged", expected=True),
        expected_check(item, metric="rust_written", expected=item["poses"]),
        expected_check(item, metric="baseline_written", expected=item["poses"]),
        paired_expected_check(
            item,
            metric="factors_match",
            actual=item.get("rust_factors"),
            expected=item.get("baseline_factors"),
        ),
        paired_expected_check(
            item,
            metric="case_fixture_hash_match",
            actual=item.get("rust_fixture_hash"),
            expected=item.get("baseline_fixture_hash"),
        ),
        required_expected_check(
            item,
            metric="rust_optimized_pose_count",
            actual=optimized_pose_count(item.get("rust_optimized_poses")),
            expected=item["poses"],
        ),
        required_expected_check(
            item,
            metric="baseline_optimized_pose_count",
            actual=optimized_pose_count(item.get("baseline_optimized_poses")),
            expected=item["poses"],
        ),
        abs_max_check(
            item,
            metric="final_cost_delta",
            tolerance_name="final_cost_delta_abs_max",
            tolerances=tolerances,
        ),
        abs_max_check(
            item,
            metric="final_residual_rms_delta",
            tolerance_name="final_residual_rms_delta_abs_max",
            tolerances=tolerances,
        ),
        abs_max_check(
            item,
            metric="final_residual_max_delta",
            tolerance_name="final_residual_max_delta_abs_max",
            tolerances=tolerances,
        ),
        max_check(
            item,
            metric="pose_translation_rmse",
            tolerance_name="pose_translation_rmse_max",
            tolerances=tolerances,
        ),
        max_check(
            item,
            metric="pose_rotation_rmse_rad",
            tolerance_name="pose_rotation_rmse_rad_max",
            tolerances=tolerances,
        ),
        max_check(
            item,
            metric="rust_to_baseline_time_ratio",
            tolerance_name="rust_to_baseline_time_ratio_max",
            tolerances=tolerances,
        ),
    ]


def fixture_identity_checks(
    rust_result: dict[str, Any],
    baseline: dict[str, Any],
    *,
    expected_fixture_hash: str | None,
) -> list[dict[str, Any]]:
    rust_hash = result_fixture_hash(rust_result)
    baseline_hash = result_fixture_hash(baseline)
    checks = [
        paired_expected_check(
            {},
            metric="rust_fixture_hash",
            actual=rust_hash,
            expected=expected_fixture_hash,
        ),
        paired_expected_check(
            {},
            metric="baseline_fixture_hash",
            actual=baseline_hash,
            expected=expected_fixture_hash,
        ),
    ]
    if rust_hash is not None and baseline_hash is not None:
        checks.append(
            paired_expected_check(
                {},
                metric="rust_baseline_fixture_hash_match",
                actual=rust_hash,
                expected=baseline_hash,
            )
        )
    return checks


def suite_fixture_identity_checks(
    comparison: list[dict[str, Any]],
    expected_fixture_hashes_by_case: dict[tuple[str, int], str],
) -> list[dict[str, Any]]:
    checks: list[dict[str, Any]] = []
    for item in comparison:
        key = (item["case"], item["poses"])
        expected = expected_fixture_hashes_by_case.get(key)
        checks.append(
            paired_expected_check(
                item,
                metric="rust_case_fixture_hash",
                actual=item.get("rust_fixture_hash"),
                expected=expected,
            )
        )
        checks.append(
            paired_expected_check(
                item,
                metric="baseline_case_fixture_hash",
                actual=item.get("baseline_fixture_hash"),
                expected=expected,
            )
        )
    return checks


def load_suite_fixture_expectations(
    fixture_dir: Path,
    *,
    overrides: dict[str, float | None] | None = None,
) -> tuple[
    dict[tuple[str, int], str],
    dict[tuple[str, int], dict[str, float | None]],
    list[dict[str, Any]],
]:
    fixture_paths = sorted(path for path in fixture_dir.glob("*.json") if path.is_file())
    if not fixture_paths:
        raise ValueError(f"no fixture JSON files found in {fixture_dir}")
    expected_hashes: dict[tuple[str, int], str] = {}
    tolerances_by_case: dict[tuple[str, int], dict[str, float | None]] = {}
    fixture_records: list[dict[str, Any]] = []
    for fixture_path in fixture_paths:
        fixture_data = json.loads(fixture_path.read_text(encoding="utf-8"))
        key = fixture_case_key(fixture_data)
        expected_hash = fixture_hash(fixture_data)
        if key in expected_hashes:
            raise ValueError(f"duplicate fixture case in suite: {key}")
        expected_hashes[key] = expected_hash
        tolerances_by_case[key] = comparison_tolerances(
            fixture=fixture_data,
            overrides=overrides,
        )
        fixture_records.append(
            {
                "path": fixture_path.as_posix(),
                "case": key[0],
                "poses": key[1],
                "fixture_hash": expected_hash,
            }
        )
    return expected_hashes, tolerances_by_case, fixture_records


def fixture_case_key(fixture_data: dict[str, Any]) -> tuple[str, int]:
    return (str(fixture_data.get("case", "default")), len(fixture_data.get("poses", [])))


def expected_check(item: dict[str, Any], *, metric: str, expected: Any) -> dict[str, Any]:
    return paired_expected_check(
        item,
        metric=metric,
        actual=item.get(metric),
        expected=expected,
    )


def paired_expected_check(
    item: dict[str, Any],
    *,
    metric: str,
    actual: Any,
    expected: Any,
) -> dict[str, Any]:
    if actual is None or expected is None:
        status = "skipped"
    else:
        status = "pass" if actual == expected else "fail"
    return {
        "case": item.get("case"),
        "poses": item.get("poses"),
        "metric": metric,
        "status": status,
        "actual": actual,
        "expected": expected,
    }


def required_expected_check(
    item: dict[str, Any],
    *,
    metric: str,
    actual: Any,
    expected: Any,
) -> dict[str, Any]:
    return {
        "case": item.get("case"),
        "poses": item.get("poses"),
        "metric": metric,
        "status": "pass" if actual == expected else "fail",
        "actual": actual,
        "expected": expected,
    }


def abs_max_check(
    item: dict[str, Any],
    *,
    metric: str,
    tolerance_name: str,
    tolerances: dict[str, float | None],
) -> dict[str, Any]:
    actual = item.get(metric)
    limit = tolerances.get(tolerance_name)
    measured = abs(actual) if actual is not None else None
    return check_result(item, metric, measured, limit)


def max_check(
    item: dict[str, Any],
    *,
    metric: str,
    tolerance_name: str,
    tolerances: dict[str, float | None],
) -> dict[str, Any]:
    return check_result(item, metric, item.get(metric), tolerances.get(tolerance_name))


def check_result(
    item: dict[str, Any],
    metric: str,
    actual: float | None,
    limit: float | None,
) -> dict[str, Any]:
    if limit is None:
        status = "skipped"
    elif actual is None:
        status = "skipped"
    else:
        status = "pass" if actual <= limit else "fail"
    return {
        "case": item["case"],
        "poses": item["poses"],
        "metric": metric,
        "status": status,
        "actual": actual,
        "limit": limit,
    }


def case_key(case: dict[str, Any]) -> tuple[str, int]:
    return (case.get("case", "default"), case["poses"])


def case_key_json(key: tuple[str, int]) -> dict[str, Any]:
    return {"case": key[0], "poses": key[1]}


def case_fixture_hash(case: dict[str, Any]) -> str | None:
    value = case.get("fixture_hash")
    if value is None:
        value = case.get("fixture_identity", {}).get("hash")
    return str(value) if value is not None else None


def optimized_pose_count(value: Any) -> int | None:
    return len(value) if isinstance(value, list) else None


def fixture_hash(fixture_data: dict[str, Any]) -> str:
    payload = json.dumps(
        fixture_data,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def result_fixture_hash(result: dict[str, Any]) -> str | None:
    value = result.get("fixture_hash")
    if value is None:
        value = result.get("metadata", {}).get("fixture_hash")
    return str(value) if value is not None else None


def optional_delta(left: float | None, right: float | None) -> float | None:
    if left is None or right is None:
        return None
    return left - right


def safe_ratio(numerator: float, denominator: float) -> float | None:
    if denominator == 0:
        return None
    return numerator / denominator


def pose_translation_rmse(
    left_poses: list[dict[str, Any]] | None,
    right_poses: list[dict[str, Any]] | None,
) -> float | None:
    if not comparable_pose_lists(left_poses, right_poses):
        return None
    total = 0.0
    assert left_poses is not None
    assert right_poses is not None
    for left, right in zip(left_poses, right_poses):
        total += sum(
            (float(left["t_xyz"][idx]) - float(right["t_xyz"][idx])) ** 2
            for idx in range(3)
        )
    return math.sqrt(total / len(left_poses))


def pose_rotation_rmse_rad(
    left_poses: list[dict[str, Any]] | None,
    right_poses: list[dict[str, Any]] | None,
) -> float | None:
    if not comparable_pose_lists(left_poses, right_poses):
        return None
    total = 0.0
    assert left_poses is not None
    assert right_poses is not None
    for left, right in zip(left_poses, right_poses):
        left_q = [float(value) for value in left["q_wxyz"]]
        right_q = [float(value) for value in right["q_wxyz"]]
        dot = abs(sum(left_q[idx] * right_q[idx] for idx in range(4)))
        dot = min(1.0, max(-1.0, dot))
        angle = 2.0 * math.acos(dot)
        total += angle * angle
    return math.sqrt(total / len(left_poses))


def comparable_pose_lists(
    left_poses: list[dict[str, Any]] | None,
    right_poses: list[dict[str, Any]] | None,
) -> bool:
    if not left_poses or not right_poses:
        return False
    if len(left_poses) != len(right_poses):
        return False
    return all(
        len(pose.get("t_xyz", [])) == 3 and len(pose.get("q_wxyz", [])) == 4
        for pose in [*left_poses, *right_poses]
    )


if __name__ == "__main__":
    sys.exit(main())

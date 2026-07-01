#!/usr/bin/env python3
"""Smoke-test the camera-LiDAR Rust optimizer C ABI with ctypes."""

from __future__ import annotations

import argparse
import ctypes
import json
import platform
import statistics
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "src/kernels/calibration/camera_lidar_optimizer/Cargo.toml"
SCHEMA = "lingtu.camera_lidar_optimizer_abi_smoke.v1"
EXPECTED_ABI_VERSION = 2
REQUIRED_CHECKS = (
    "abi_version",
    "pose3_size",
    "ct_gicp_size",
    "ct_gicp_source_size",
    "ct_gicp_target_size",
    "linearization_size",
    "optimizer_config_size",
    "optimizer_result_size",
    "return_code",
    "used_correspondences",
    "cost",
    "gradient",
    "rhs",
    "hessian",
    "optimizer_return_code",
    "optimizer_used_correspondences",
    "optimizer_cost_reduction",
    "optimizer_final_cost_finite",
    "optimizer_pose0_translation",
    "dynamic_optimizer_return_code",
    "dynamic_optimizer_used_correspondences",
    "dynamic_optimizer_cost_reduction",
    "dynamic_optimizer_final_cost_finite",
    "dynamic_optimizer_pose0_translation",
)


class Pose3(ctypes.Structure):
    _fields_ = [
        ("t_xyz", ctypes.c_double * 3),
        ("q_wxyz", ctypes.c_double * 4),
    ]


class CtGicpCorrespondence(ctypes.Structure):
    _fields_ = [
        ("time", ctypes.c_double),
        ("source_xyz", ctypes.c_double * 3),
        ("target_xyz", ctypes.c_double * 3),
        ("source_cov_row_major", ctypes.c_double * 9),
        ("target_cov_row_major", ctypes.c_double * 9),
    ]


class CtGicpSourcePoint(ctypes.Structure):
    _fields_ = [
        ("time", ctypes.c_double),
        ("xyz", ctypes.c_double * 3),
        ("cov_row_major", ctypes.c_double * 9),
    ]


class CtGicpTargetPoint(ctypes.Structure):
    _fields_ = [
        ("xyz", ctypes.c_double * 3),
        ("cov_row_major", ctypes.c_double * 9),
    ]


class TwoPoseLinearization(ctypes.Structure):
    _fields_ = [
        ("cost", ctypes.c_double),
        ("used_correspondences", ctypes.c_uint64),
        ("hessian_12x12_row_major", ctypes.c_double * 144),
        ("gradient_12", ctypes.c_double * 12),
        ("rhs_12", ctypes.c_double * 12),
    ]


class TwoPoseOptimizerConfig(ctypes.Structure):
    _fields_ = [
        ("max_iterations", ctypes.c_uint32),
        ("derivative_eps", ctypes.c_double),
        ("initial_lambda", ctypes.c_double),
        ("lambda_factor", ctypes.c_double),
        ("step_tolerance", ctypes.c_double),
        ("relative_cost_tolerance", ctypes.c_double),
        ("prior_precision", ctypes.c_double),
        ("between_precision", ctypes.c_double),
    ]


class TwoPoseOptimizationResult(ctypes.Structure):
    _fields_ = [
        ("pose0", Pose3),
        ("pose1", Pose3),
        ("initial_cost", ctypes.c_double),
        ("final_cost", ctypes.c_double),
        ("iterations", ctypes.c_uint32),
        ("accepted_steps", ctypes.c_uint32),
        ("last_lambda", ctypes.c_double),
        ("used_correspondences", ctypes.c_uint64),
    ]


def _double_array(values: list[float]):
    return (ctypes.c_double * len(values))(*values)


def _library_path(manifest: Path, release: bool) -> Path:
    target_dir = manifest.parent / "target" / ("release" if release else "debug")
    system = platform.system().lower()
    if system == "windows":
        filename = "lingtu_camera_lidar_optimizer.dll"
    elif system == "darwin":
        filename = "liblingtu_camera_lidar_optimizer.dylib"
    else:
        filename = "liblingtu_camera_lidar_optimizer.so"
    return target_dir / filename


def _build(manifest: Path, release: bool) -> None:
    command = ["cargo", "build", "--manifest-path", str(manifest)]
    if release:
        command.append("--release")
    subprocess.run(command, check=True, cwd=ROOT)


def _configure(lib: ctypes.CDLL) -> None:
    lib.lingtu_camera_lidar_optimizer_abi_version.restype = ctypes.c_uint32
    lib.lingtu_camera_lidar_optimizer_sizeof_pose3.restype = ctypes.c_size_t
    lib.lingtu_camera_lidar_optimizer_sizeof_ct_gicp_correspondence.restype = ctypes.c_size_t
    lib.lingtu_camera_lidar_optimizer_sizeof_ct_gicp_source_point.restype = ctypes.c_size_t
    lib.lingtu_camera_lidar_optimizer_sizeof_ct_gicp_target_point.restype = ctypes.c_size_t
    lib.lingtu_camera_lidar_optimizer_sizeof_two_pose_linearization.restype = ctypes.c_size_t
    lib.lingtu_camera_lidar_optimizer_sizeof_two_pose_optimizer_config.restype = ctypes.c_size_t
    lib.lingtu_camera_lidar_optimizer_sizeof_two_pose_optimization_result.restype = ctypes.c_size_t
    lib.lingtu_camera_lidar_optimizer_linearize_ct_gicp.argtypes = [
        ctypes.POINTER(Pose3),
        ctypes.POINTER(Pose3),
        ctypes.POINTER(CtGicpCorrespondence),
        ctypes.c_size_t,
        ctypes.c_double,
        ctypes.POINTER(TwoPoseLinearization),
    ]
    lib.lingtu_camera_lidar_optimizer_linearize_ct_gicp.restype = ctypes.c_int
    lib.lingtu_camera_lidar_optimizer_optimize_ct_gicp_two_pose.argtypes = [
        ctypes.POINTER(Pose3),
        ctypes.POINTER(Pose3),
        ctypes.POINTER(Pose3),
        ctypes.POINTER(Pose3),
        ctypes.POINTER(CtGicpCorrespondence),
        ctypes.c_size_t,
        ctypes.POINTER(TwoPoseOptimizerConfig),
        ctypes.POINTER(TwoPoseOptimizationResult),
    ]
    lib.lingtu_camera_lidar_optimizer_optimize_ct_gicp_two_pose.restype = ctypes.c_int
    lib.lingtu_camera_lidar_optimizer_optimize_ct_gicp_dynamic_two_pose.argtypes = [
        ctypes.POINTER(Pose3),
        ctypes.POINTER(Pose3),
        ctypes.POINTER(Pose3),
        ctypes.POINTER(Pose3),
        ctypes.POINTER(CtGicpSourcePoint),
        ctypes.c_size_t,
        ctypes.POINTER(CtGicpTargetPoint),
        ctypes.c_size_t,
        ctypes.c_double,
        ctypes.c_uint32,
        ctypes.POINTER(TwoPoseOptimizerConfig),
        ctypes.POINTER(TwoPoseOptimizationResult),
    ]
    lib.lingtu_camera_lidar_optimizer_optimize_ct_gicp_dynamic_two_pose.restype = ctypes.c_int


def _diag_covariance(diagonal: list[float]):
    return _double_array(
        [
            diagonal[0],
            0.0,
            0.0,
            0.0,
            diagonal[1],
            0.0,
            0.0,
            0.0,
            diagonal[2],
        ]
    )


def _duration_stats(samples_ns: list[int]) -> dict:
    samples_ms = [sample / 1_000_000.0 for sample in samples_ns]
    return {
        "repeats": len(samples_ms),
        "min_ms": min(samples_ms),
        "median_ms": statistics.median(samples_ms),
        "mean_ms": statistics.fmean(samples_ms),
        "max_ms": max(samples_ms),
    }


def _benchmark_call(
    *,
    repeats: int,
    warmups: int,
    call,
) -> dict:
    for _ in range(max(0, warmups)):
        status = call()
        if status != 0:
            raise RuntimeError(f"benchmark warmup call failed with {status}")

    samples_ns: list[int] = []
    for _ in range(max(0, repeats)):
        started = time.perf_counter_ns()
        status = call()
        elapsed = time.perf_counter_ns() - started
        if status != 0:
            raise RuntimeError(f"benchmark call failed with {status}")
        samples_ns.append(elapsed)
    return _duration_stats(samples_ns)


def evaluate_result(payload: dict) -> dict:
    checks = payload.get("checks") if isinstance(payload.get("checks"), dict) else {}
    failed_checks = sorted(name for name, passed in checks.items() if not passed)
    missing_checks = [name for name in REQUIRED_CHECKS if name not in checks]
    failed_checks.extend(f"{name}:missing" for name in missing_checks)
    failed_checks = sorted(set(failed_checks))
    return {
        "schema": payload.get("schema"),
        "ok": payload.get("schema") == SCHEMA and not failed_checks,
        "failed_checks": failed_checks,
    }


def run_smoke(
    manifest: Path,
    release: bool,
    skip_build: bool,
    *,
    benchmark_repeats: int = 0,
    benchmark_warmups: int = 5,
) -> dict:
    manifest = manifest.resolve()
    if not skip_build:
        _build(manifest, release)

    library = _library_path(manifest, release)
    if not library.is_file():
        raise FileNotFoundError(f"Rust library not found: {library}")

    lib = ctypes.CDLL(str(library))
    _configure(lib)

    pose0 = Pose3(_double_array([0.0, 0.0, 0.0]), _double_array([1.0, 0.0, 0.0, 0.0]))
    pose1 = Pose3(_double_array([0.0, 0.0, 0.0]), _double_array([1.0, 0.0, 0.0, 0.0]))
    correspondence = CtGicpCorrespondence(
        ctypes.c_double(0.0),
        _double_array([1.0, 0.0, 0.0]),
        _double_array([0.0, 0.0, 0.0]),
        _diag_covariance([0.0, 0.0, 0.0]),
        _diag_covariance([0.25, 1.0, 1.0]),
    )
    output = TwoPoseLinearization()
    code = lib.lingtu_camera_lidar_optimizer_linearize_ct_gicp(
        ctypes.byref(pose0),
        ctypes.byref(pose1),
        ctypes.byref(correspondence),
        1,
        1e-6,
        ctypes.byref(output),
    )

    optimize_correspondence = CtGicpCorrespondence(
        ctypes.c_double(0.0),
        _double_array([0.0, 0.0, 0.0]),
        _double_array([1.0, 2.0, 3.0]),
        _diag_covariance([0.0, 0.0, 0.0]),
        _diag_covariance([1.0, 1.0, 1.0]),
    )
    optimizer_config = TwoPoseOptimizerConfig(
        ctypes.c_uint32(20),
        ctypes.c_double(1e-6),
        ctypes.c_double(1e-3),
        ctypes.c_double(10.0),
        ctypes.c_double(1e-9),
        ctypes.c_double(1e-10),
        ctypes.c_double(0.0),
        ctypes.c_double(0.0),
    )
    optimizer_output = TwoPoseOptimizationResult()
    optimizer_code = lib.lingtu_camera_lidar_optimizer_optimize_ct_gicp_two_pose(
        ctypes.byref(pose0),
        ctypes.byref(pose1),
        ctypes.byref(pose0),
        ctypes.byref(pose0),
        ctypes.byref(optimize_correspondence),
        1,
        ctypes.byref(optimizer_config),
        ctypes.byref(optimizer_output),
    )
    pose0_translation = [optimizer_output.pose0.t_xyz[i] for i in range(3)]

    dynamic_sources = (CtGicpSourcePoint * 1)(
        CtGicpSourcePoint(
            ctypes.c_double(0.0),
            _double_array([0.0, 0.0, 0.0]),
            _diag_covariance([0.0, 0.0, 0.0]),
        )
    )
    dynamic_targets = (CtGicpTargetPoint * 1)(
        CtGicpTargetPoint(
            _double_array([1.0, 2.0, 3.0]),
            _diag_covariance([1.0, 1.0, 1.0]),
        )
    )
    dynamic_optimizer_output = TwoPoseOptimizationResult()
    dynamic_optimizer_code = lib.lingtu_camera_lidar_optimizer_optimize_ct_gicp_dynamic_two_pose(
        ctypes.byref(pose0),
        ctypes.byref(pose1),
        ctypes.byref(pose0),
        ctypes.byref(pose0),
        dynamic_sources,
        len(dynamic_sources),
        dynamic_targets,
        len(dynamic_targets),
        5.0,
        3,
        ctypes.byref(optimizer_config),
        ctypes.byref(dynamic_optimizer_output),
    )
    dynamic_pose0_translation = [
        dynamic_optimizer_output.pose0.t_xyz[i] for i in range(3)
    ]

    checks = {
        "abi_version": lib.lingtu_camera_lidar_optimizer_abi_version() == EXPECTED_ABI_VERSION,
        "pose3_size": lib.lingtu_camera_lidar_optimizer_sizeof_pose3() == ctypes.sizeof(Pose3),
        "ct_gicp_size": (
            lib.lingtu_camera_lidar_optimizer_sizeof_ct_gicp_correspondence()
            == ctypes.sizeof(CtGicpCorrespondence)
        ),
        "ct_gicp_source_size": (
            lib.lingtu_camera_lidar_optimizer_sizeof_ct_gicp_source_point()
            == ctypes.sizeof(CtGicpSourcePoint)
        ),
        "ct_gicp_target_size": (
            lib.lingtu_camera_lidar_optimizer_sizeof_ct_gicp_target_point()
            == ctypes.sizeof(CtGicpTargetPoint)
        ),
        "linearization_size": (
            lib.lingtu_camera_lidar_optimizer_sizeof_two_pose_linearization()
            == ctypes.sizeof(TwoPoseLinearization)
        ),
        "optimizer_config_size": (
            lib.lingtu_camera_lidar_optimizer_sizeof_two_pose_optimizer_config()
            == ctypes.sizeof(TwoPoseOptimizerConfig)
        ),
        "optimizer_result_size": (
            lib.lingtu_camera_lidar_optimizer_sizeof_two_pose_optimization_result()
            == ctypes.sizeof(TwoPoseOptimizationResult)
        ),
        "return_code": code == 0,
        "used_correspondences": output.used_correspondences == 1,
        "cost": abs(output.cost - 2.0) <= 1e-10,
        "gradient": abs(output.gradient_12[3] - 4.0) <= 1e-8,
        "rhs": abs(output.rhs_12[3] + 4.0) <= 1e-8,
        "hessian": abs(output.hessian_12x12_row_major[3 * 12 + 3] - 4.0) <= 1e-8,
        "optimizer_return_code": optimizer_code == 0,
        "optimizer_used_correspondences": optimizer_output.used_correspondences == 1,
        "optimizer_cost_reduction": optimizer_output.final_cost < optimizer_output.initial_cost,
        "optimizer_final_cost_finite": (
            optimizer_output.final_cost == optimizer_output.final_cost
            and abs(optimizer_output.final_cost) < float("inf")
        ),
        "optimizer_pose0_translation": all(
            abs(actual - expected) <= 1e-5
            for actual, expected in zip(pose0_translation, [1.0, 2.0, 3.0])
        ),
        "dynamic_optimizer_return_code": dynamic_optimizer_code == 0,
        "dynamic_optimizer_used_correspondences": (
            dynamic_optimizer_output.used_correspondences == 1
        ),
        "dynamic_optimizer_cost_reduction": (
            dynamic_optimizer_output.final_cost < dynamic_optimizer_output.initial_cost
        ),
        "dynamic_optimizer_final_cost_finite": (
            dynamic_optimizer_output.final_cost == dynamic_optimizer_output.final_cost
            and abs(dynamic_optimizer_output.final_cost) < float("inf")
        ),
        "dynamic_optimizer_pose0_translation": all(
            abs(actual - expected) <= 1e-5
            for actual, expected in zip(dynamic_pose0_translation, [1.0, 2.0, 3.0])
        ),
    }
    payload = {
        "schema": SCHEMA,
        "ok": all(checks.values()),
        "library": str(library),
        "checks": checks,
        "result": {
            "abi_version": lib.lingtu_camera_lidar_optimizer_abi_version(),
            "return_code": code,
            "cost": output.cost,
            "used_correspondences": output.used_correspondences,
            "gradient_tx0": output.gradient_12[3],
            "rhs_tx0": output.rhs_12[3],
            "hessian_tx0_tx0": output.hessian_12x12_row_major[3 * 12 + 3],
            "optimizer_return_code": optimizer_code,
            "optimizer_initial_cost": optimizer_output.initial_cost,
            "optimizer_final_cost": optimizer_output.final_cost,
            "optimizer_pose0_translation": pose0_translation,
            "dynamic_optimizer_return_code": dynamic_optimizer_code,
            "dynamic_optimizer_initial_cost": dynamic_optimizer_output.initial_cost,
            "dynamic_optimizer_final_cost": dynamic_optimizer_output.final_cost,
            "dynamic_optimizer_pose0_translation": dynamic_pose0_translation,
        },
    }
    if benchmark_repeats > 0:
        linearization_benchmark_output = TwoPoseLinearization()
        optimizer_benchmark_output = TwoPoseOptimizationResult()
        dynamic_benchmark_output = TwoPoseOptimizationResult()
        payload["benchmark"] = {
            "unit": "milliseconds",
            "warmups": max(0, int(benchmark_warmups)),
            "linearize_ct_gicp": _benchmark_call(
                repeats=int(benchmark_repeats),
                warmups=int(benchmark_warmups),
                call=lambda: lib.lingtu_camera_lidar_optimizer_linearize_ct_gicp(
                    ctypes.byref(pose0),
                    ctypes.byref(pose1),
                    ctypes.byref(correspondence),
                    1,
                    1e-6,
                    ctypes.byref(linearization_benchmark_output),
                ),
            ),
            "optimize_ct_gicp_two_pose": _benchmark_call(
                repeats=int(benchmark_repeats),
                warmups=int(benchmark_warmups),
                call=lambda: lib.lingtu_camera_lidar_optimizer_optimize_ct_gicp_two_pose(
                    ctypes.byref(pose0),
                    ctypes.byref(pose1),
                    ctypes.byref(pose0),
                    ctypes.byref(pose0),
                    ctypes.byref(optimize_correspondence),
                    1,
                    ctypes.byref(optimizer_config),
                    ctypes.byref(optimizer_benchmark_output),
                ),
            ),
            "optimize_ct_gicp_dynamic_two_pose": _benchmark_call(
                repeats=int(benchmark_repeats),
                warmups=int(benchmark_warmups),
                call=lambda: lib.lingtu_camera_lidar_optimizer_optimize_ct_gicp_dynamic_two_pose(
                    ctypes.byref(pose0),
                    ctypes.byref(pose1),
                    ctypes.byref(pose0),
                    ctypes.byref(pose0),
                    dynamic_sources,
                    len(dynamic_sources),
                    dynamic_targets,
                    len(dynamic_targets),
                    5.0,
                    3,
                    ctypes.byref(optimizer_config),
                    ctypes.byref(dynamic_benchmark_output),
                ),
            ),
        }
    payload["summary"] = evaluate_result(payload)
    payload["ok"] = payload["summary"]["ok"]
    return payload


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest-path", type=Path, default=MANIFEST)
    parser.add_argument("--release", action="store_true")
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument(
        "--benchmark-repeats",
        type=int,
        default=0,
        help="repeat ABI calls and include timing stats; disabled by default",
    )
    parser.add_argument("--benchmark-warmups", type=int, default=5)
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--json-out", type=Path, default=None)
    args = parser.parse_args(argv)

    result = run_smoke(
        args.manifest_path,
        args.release,
        args.skip_build,
        benchmark_repeats=args.benchmark_repeats,
        benchmark_warmups=args.benchmark_warmups,
    )
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(result, indent=2, sort_keys=True), encoding="utf-8")
    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        status = "OK" if result["ok"] else "FAILED"
        print(f"Camera-LiDAR optimizer ABI smoke: {status}")
        print(f"  library: {result['library']}")
        print(f"  abi_version: {result['result']['abi_version']}")
        print(f"  return_code: {result['result']['return_code']}")
        print(f"  cost: {result['result']['cost']}")
        print(f"  gradient_tx0: {result['result']['gradient_tx0']}")
        print(f"  optimizer_return_code: {result['result']['optimizer_return_code']}")
        print(f"  optimizer_final_cost: {result['result']['optimizer_final_cost']}")
        print(
            "  dynamic_optimizer_return_code: "
            f"{result['result']['dynamic_optimizer_return_code']}"
        )
        print(
            "  dynamic_optimizer_final_cost: "
            f"{result['result']['dynamic_optimizer_final_cost']}"
        )
        if result.get("benchmark"):
            print("  benchmark:")
            for name, stats in result["benchmark"].items():
                if not isinstance(stats, dict):
                    continue
                print(
                    "    "
                    f"{name}: median={stats['median_ms']:.6f}ms "
                    f"min={stats['min_ms']:.6f}ms max={stats['max_ms']:.6f}ms"
                )
        if result["summary"]["failed_checks"]:
            print("  failed_checks:")
            for failed_check in result["summary"]["failed_checks"]:
                print(f"    - {failed_check}")
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

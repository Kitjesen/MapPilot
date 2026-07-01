from __future__ import annotations

import argparse
import ctypes
import hashlib
import json
import math
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
KERNEL = ROOT / "src" / "kernels" / "slam" / "pose_graph_opt"
MANIFEST = KERNEL / "Cargo.toml"
SCHEMA = "lingtu.pose_graph_opt.fixture.v1"
OK = 0


class Pose3C(ctypes.Structure):
    _fields_ = [
        ("t_xyz", ctypes.c_double * 3),
        ("q_wxyz", ctypes.c_double * 4),
    ]


class Prior3C(ctypes.Structure):
    _fields_ = [
        ("index", ctypes.c_uint32),
        ("reserved0", ctypes.c_uint32),
        ("pose", Pose3C),
        ("information_upper", ctypes.c_double * 21),
    ]


class Between3C(ctypes.Structure):
    _fields_ = [
        ("from_index", ctypes.c_uint32),
        ("to_index", ctypes.c_uint32),
        ("pose_from_to", Pose3C),
        ("information_upper", ctypes.c_double * 21),
    ]


class ConfigC(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("version", ctypes.c_uint32),
        ("max_iterations", ctypes.c_uint32),
        ("method", ctypes.c_uint32),
        ("fixed_pose_index", ctypes.c_int32),
        ("auto_anchor", ctypes.c_uint32),
        ("initial_lambda", ctypes.c_double),
        ("tolerance", ctypes.c_double),
        ("numeric_epsilon", ctypes.c_double),
    ]


class ReportC(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("version", ctypes.c_uint32),
        ("status", ctypes.c_int32),
        ("iterations", ctypes.c_uint32),
        ("accepted_steps", ctypes.c_uint32),
        ("rejected_steps", ctypes.c_uint32),
        ("converged", ctypes.c_uint32),
        ("reserved0", ctypes.c_uint32),
        ("initial_cost", ctypes.c_double),
        ("final_cost", ctypes.c_double),
    ]


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate and run neutral pose_graph_opt fixtures."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    generate = subparsers.add_parser("generate", help="Generate one fixture JSON.")
    generate.add_argument("--case", choices=["pgo_loop", "hba_full_info"], required=True)
    generate.add_argument("--poses", type=int, default=16)
    generate.add_argument("--output", type=Path, required=True)

    generate_suite = subparsers.add_parser(
        "generate-suite", help="Generate pgo_loop and hba_full_info fixtures."
    )
    generate_suite.add_argument("--poses", nargs="+", type=int, default=[16, 64, 256])
    generate_suite.add_argument("--output-dir", type=Path, required=True)

    run = subparsers.add_parser("run", help="Run a fixture through the Rust C ABI.")
    run.add_argument("--fixture", type=Path, required=True)
    run.add_argument("--json-out", type=Path)
    run.add_argument("--release", action="store_true", default=True)

    run_suite = subparsers.add_parser(
        "run-suite", help="Run all fixture JSON files in a directory through the Rust C ABI."
    )
    run_suite.add_argument("--fixture-dir", type=Path, required=True)
    run_suite.add_argument("--json-out", type=Path, required=True)
    run_suite.add_argument("--release", action="store_true", default=True)

    args = parser.parse_args()
    if args.command == "generate":
        fixture = generate_fixture(args.case, args.poses)
        write_json(args.output, fixture)
        return 0
    if args.command == "generate-suite":
        args.output_dir.mkdir(parents=True, exist_ok=True)
        for pose_count in args.poses:
            for case in ("pgo_loop", "hba_full_info"):
                write_json(
                    args.output_dir / f"{case}_{pose_count}.json",
                    generate_fixture(case, pose_count),
                )
        return 0
    if args.command == "run":
        fixture = json.loads(args.fixture.read_text(encoding="utf-8"))
        result = run_fixture(fixture, release=args.release)
        if args.json_out:
            write_json(args.json_out, result)
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0
    if args.command == "run-suite":
        result = run_fixture_suite(args.fixture_dir, release=args.release)
        write_json(args.json_out, result)
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0
    raise AssertionError(args.command)


def generate_fixture(case: str, pose_count: int) -> dict[str, Any]:
    if pose_count < 2:
        raise ValueError("pose_count must be at least 2")
    if case == "pgo_loop":
        return pgo_loop_fixture(pose_count)
    if case == "hba_full_info":
        return hba_full_info_fixture(pose_count)
    raise ValueError(f"unsupported fixture case: {case}")


def pgo_loop_fixture(pose_count: int) -> dict[str, Any]:
    odom = rpy_pose(0.85, 0.08, 0.03, 0.012, -0.006, 0.045)
    truth = truth_chain(pose_count, odom)
    poses = drifted_poses(
        truth,
        lambda scale: rpy_pose(
            0.018 * scale,
            -0.010 * scale,
            0.003 * scale,
            0.0008 * scale,
            -0.0005 * scale,
            0.0025 * scale,
        ),
    )
    priors = [
        {
            "index": 0,
            "pose": identity_pose(),
            "information_upper": diagonal_information_upper([100.0] * 6),
        }
    ]
    betweens = [
        {
            "from_index": idx - 1,
            "to_index": idx,
            "pose_from_to": between_pose(truth[idx - 1], truth[idx]),
            "information_upper": diagonal_information_upper([10.0] * 6),
        }
        for idx in range(1, pose_count)
    ]
    betweens.append(
        {
            "from_index": 0,
            "to_index": pose_count - 1,
            "pose_from_to": between_pose(truth[0], truth[pose_count - 1]),
            "information_upper": diagonal_information_upper([100.0] * 6),
        }
    )
    return fixture(case="pgo_loop", poses=poses, priors=priors, betweens=betweens)


def hba_full_info_fixture(pose_count: int) -> dict[str, Any]:
    odom = rpy_pose(0.65, -0.04, 0.06, -0.006, 0.010, 0.030)
    truth = truth_chain(pose_count, odom)
    poses = drifted_poses(
        truth,
        lambda scale: rpy_pose(
            -0.012 * scale,
            0.007 * scale,
            -0.002 * scale,
            -0.0006 * scale,
            0.0004 * scale,
            0.0018 * scale,
        ),
    )
    betweens = [
        {
            "from_index": idx - 1,
            "to_index": idx,
            "pose_from_to": between_pose(truth[idx - 1], truth[idx]),
            "information_upper": full_information_upper(),
        }
        for idx in range(1, pose_count)
    ]
    if pose_count > 4:
        for idx in range(0, pose_count - 4):
            betweens.append(
                {
                    "from_index": idx,
                    "to_index": idx + 4,
                    "pose_from_to": between_pose(truth[idx], truth[idx + 4]),
                    "information_upper": full_information_upper(),
                }
            )
    return fixture(case="hba_full_info", poses=poses, priors=[], betweens=betweens)


def fixture(
    *,
    case: str,
    poses: list[dict[str, list[float]]],
    priors: list[dict[str, Any]],
    betweens: list[dict[str, Any]],
) -> dict[str, Any]:
    return {
        "schema": SCHEMA,
        "schema_version": 1,
        "case": case,
        "coverage": f"{case}_prior_between" if priors else f"{case}_between_only",
        "pose_format": "t_xyz_q_wxyz",
        "tangent_order": ["rx", "ry", "rz", "tx", "ty", "tz"],
        "information_upper_order": "row_major_upper_6x6",
        "information_packing": "upper_triangle_row_major_6x6",
        "config": {
            "max_iterations": 80,
            "method": 1,
            "fixed_pose_index": 0,
            "auto_anchor": True,
            "initial_lambda": 1e-3,
            "tolerance": 1e-9,
            "numeric_epsilon": 1e-6,
        },
        "poses": poses,
        "priors": priors,
        "betweens": betweens,
        "expected": {
            "status": "ok",
            "pose_count": len(poses),
            "factor_count": len(priors) + len(betweens),
            "initial_cost_min": 0.0,
            "final_cost_max": 1e-6,
            "iterations_max": 80,
            "accepted_steps_min": 1,
        },
        "baseline_tolerances": {
            "final_cost_delta_abs_max": 1e-6,
            "final_residual_rms_delta_abs_max": 1e-6,
            "final_residual_max_delta_abs_max": 1e-6,
        },
        "metadata": {
            "generator": "tools/bench/pose_graph_opt_fixture.py",
            "source": "synthetic_caller_shaped",
        },
    }


def run_fixture(fixture_data: dict[str, Any], *, release: bool = True) -> dict[str, Any]:
    validate_fixture(fixture_data)
    lib = load_library(release=release)
    config = config_from_fixture(fixture_data)
    poses = pose_array(fixture_data["poses"])
    priors = prior_array(fixture_data["priors"])
    betweens = between_array(fixture_data["betweens"])
    report = ReportC()

    handle = lib.lt_pose_graph_opt_create(ctypes.byref(config))
    if not handle:
        raise RuntimeError("lt_pose_graph_opt_create returned null")
    start = time.perf_counter()
    try:
        status = lib.lt_pose_graph_opt_process_se3(
            handle,
            poses,
            len(poses),
            priors if len(priors) else None,
            len(priors),
            betweens if len(betweens) else None,
            len(betweens),
            ctypes.byref(report),
        )
        elapsed_ms = (time.perf_counter() - start) * 1000.0
        if status != OK:
            raise RuntimeError(f"lt_pose_graph_opt_process_se3 failed with {status}")
        written = ctypes.c_uint64()
        copy_status = lib.lt_pose_graph_opt_copy_result_poses(
            handle, poses, len(poses), ctypes.byref(written)
        )
        if copy_status != OK:
            raise RuntimeError(f"lt_pose_graph_opt_copy_result_poses failed with {copy_status}")
    finally:
        lib.lt_pose_graph_opt_destroy(handle)

    optimized_poses = [pose_to_json(poses[idx]) for idx in range(written.value)]
    final_residual_rms, final_residual_max = factor_residual_stats(
        fixture_data,
        optimized_poses,
    )
    fixture_hash_value = fixture_hash(fixture_data)
    fixture_identity_value = fixture_identity(fixture_data)
    return {
        "benchmark": "lingtu_pose_graph_opt_fixture",
        "fixture_hash": fixture_hash_value,
        "fixture_identity": fixture_identity_value,
        "unit": "milliseconds",
        "cases": [
            {
                "case": fixture_data["case"],
                "poses": len(fixture_data["poses"]),
                "factors": len(fixture_data["priors"]) + len(fixture_data["betweens"]),
                "fixture_hash": fixture_hash_value,
                "fixture_identity": fixture_identity_value,
                "status": int(report.status),
                "converged": bool(report.converged),
                "written": int(written.value),
                "initial_cost": report.initial_cost,
                "final_cost": report.final_cost,
                "final_residual_rms": final_residual_rms,
                "final_residual_max": final_residual_max,
                "iterations": int(report.iterations),
                "accepted_steps": int(report.accepted_steps),
                "rejected_steps": int(report.rejected_steps),
                "elapsed_ms": elapsed_ms,
                "optimized_poses": optimized_poses,
            }
        ],
    }


def run_fixture_suite(fixture_dir: Path, *, release: bool = True) -> dict[str, Any]:
    fixture_paths = sorted(path for path in fixture_dir.glob("*.json") if path.is_file())
    if not fixture_paths:
        raise ValueError(f"no fixture JSON files found in {fixture_dir}")
    cases: list[dict[str, Any]] = []
    fixtures: list[dict[str, Any]] = []
    for fixture_path in fixture_paths:
        fixture_data = json.loads(fixture_path.read_text(encoding="utf-8"))
        result = run_fixture(fixture_data, release=release)
        case = dict(result["cases"][0])
        case["fixture_path"] = fixture_path.as_posix()
        cases.append(case)
        fixtures.append(
            {
                "path": fixture_path.as_posix(),
                "fixture_hash": result["fixture_hash"],
                "fixture_identity": result["fixture_identity"],
            }
        )
    return {
        "benchmark": "lingtu_pose_graph_opt_fixture_suite",
        "unit": "milliseconds",
        "fixtures": fixtures,
        "cases": cases,
        "metadata": {
            "generator": "tools/bench/pose_graph_opt_fixture.py",
            "fixture_count": len(fixtures),
        },
    }


def fixture_hash(fixture_data: dict[str, Any]) -> str:
    payload = json.dumps(
        fixture_data,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def fixture_identity(fixture_data: dict[str, Any]) -> dict[str, Any]:
    return {
        "schema": fixture_data.get("schema"),
        "schema_version": fixture_data.get("schema_version"),
        "case": fixture_data.get("case"),
        "poses": len(fixture_data.get("poses", [])),
        "factors": len(fixture_data.get("priors", [])) + len(fixture_data.get("betweens", [])),
        "tangent_order": fixture_data.get("tangent_order"),
        "information_packing": fixture_data.get("information_packing"),
        "config": fixture_data.get("config", {}),
        "hash": fixture_hash(fixture_data),
    }


def factor_residual_stats(
    fixture_data: dict[str, Any],
    optimized_poses: list[dict[str, list[float]]],
) -> tuple[float, float]:
    residual_values: list[float] = []
    for prior in fixture_data.get("priors", []):
        error_pose = compose_pose(
            inverse_pose(prior["pose"]),
            optimized_poses[int(prior["index"])],
        )
        residual_values.extend(se3_log(error_pose))
    for between in fixture_data.get("betweens", []):
        predicted = between_pose(
            optimized_poses[int(endpoint_value(between, "from"))],
            optimized_poses[int(endpoint_value(between, "to"))],
        )
        error_pose = compose_pose(inverse_pose(between["pose_from_to"]), predicted)
        residual_values.extend(se3_log(error_pose))
    if not residual_values:
        return 0.0, 0.0
    squared_sum = sum(value * value for value in residual_values)
    max_abs = max(abs(value) for value in residual_values)
    return math.sqrt(squared_sum / len(residual_values)), max_abs


def se3_log(pose: dict[str, list[float]]) -> list[float]:
    omega = so3_log(pose["q_wxyz"])
    theta = vector_norm(omega)
    omega_hat = skew(omega)
    omega_hat2 = matmul3(omega_hat, omega_hat)
    if theta < 1e-10:
        v_inv = matadd3(identity3(), matscale3(omega_hat, -0.5), matscale3(omega_hat2, 1.0 / 12.0))
    else:
        coefficient = (
            1.0 / (theta * theta)
            - (1.0 + math.cos(theta)) / (2.0 * theta * math.sin(theta))
        )
        v_inv = matadd3(identity3(), matscale3(omega_hat, -0.5), matscale3(omega_hat2, coefficient))
    rho = matvec3(v_inv, pose["t_xyz"])
    return omega + rho


def so3_log(q_wxyz: list[float]) -> list[float]:
    q = normalize_quat(q_wxyz)
    w = min(1.0, max(-1.0, q[0]))
    xyz = q[1:4]
    sin_half = vector_norm(xyz)
    if sin_half < 1e-12:
        return [2.0 * value for value in xyz]
    theta = 2.0 * math.atan2(sin_half, w)
    scale = theta / sin_half
    return [scale * value for value in xyz]


def vector_norm(values: list[float]) -> float:
    return math.sqrt(sum(value * value for value in values))


def identity3() -> list[list[float]]:
    return [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]


def skew(values: list[float]) -> list[list[float]]:
    x, y, z = values
    return [
        [0.0, -z, y],
        [z, 0.0, -x],
        [-y, x, 0.0],
    ]


def matadd3(*matrices: list[list[float]]) -> list[list[float]]:
    return [
        [sum(matrix[row][col] for matrix in matrices) for col in range(3)]
        for row in range(3)
    ]


def matscale3(matrix: list[list[float]], scale: float) -> list[list[float]]:
    return [[scale * matrix[row][col] for col in range(3)] for row in range(3)]


def matmul3(left: list[list[float]], right: list[list[float]]) -> list[list[float]]:
    return [
        [
            sum(left[row][idx] * right[idx][col] for idx in range(3))
            for col in range(3)
        ]
        for row in range(3)
    ]


def matvec3(matrix: list[list[float]], vector: list[float]) -> list[float]:
    return [
        sum(matrix[row][col] * vector[col] for col in range(3))
        for row in range(3)
    ]


def validate_fixture(fixture_data: dict[str, Any]) -> None:
    if fixture_data.get("schema") != SCHEMA:
        raise ValueError(f"unsupported fixture schema: {fixture_data.get('schema')}")
    poses = fixture_data.get("poses")
    if not isinstance(poses, list) or not poses:
        raise ValueError("fixture must contain at least one pose")
    for pose in poses:
        validate_pose(pose)
    for prior in fixture_data.get("priors", []):
        if not isinstance(prior.get("index"), int):
            raise ValueError("prior index must be an integer")
        validate_pose(prior["pose"])
        validate_information(prior["information_upper"])
    for between in fixture_data.get("betweens", []):
        if not isinstance(endpoint_value(between, "from"), int) or not isinstance(
            endpoint_value(between, "to"), int
        ):
            raise ValueError("between endpoints must be integers")
        validate_pose(between["pose_from_to"])
        validate_information(between["information_upper"])


def validate_pose(pose: dict[str, Any]) -> None:
    if len(pose.get("t_xyz", [])) != 3:
        raise ValueError("pose.t_xyz must have 3 values")
    if len(pose.get("q_wxyz", [])) != 4:
        raise ValueError("pose.q_wxyz must have 4 values")


def validate_information(upper: list[float]) -> None:
    if len(upper) != 21:
        raise ValueError("information_upper must have 21 values")


def load_library(*, release: bool) -> ctypes.CDLL:
    if shutil.which("cargo") is None:
        raise RuntimeError("cargo is required to build pose_graph_opt")
    profile = "release" if release else "debug"
    command = ["cargo", "build", "--manifest-path", str(MANIFEST)]
    if release:
        command.insert(2, "--release")
    subprocess.run(command, cwd=KERNEL, check=True)
    candidates = library_candidates(KERNEL / "target", profile)
    for candidate in candidates:
        if candidate.exists():
            if sys.platform == "win32":
                dll_dir = os.add_dll_directory(str(candidate.parent))
            else:
                dll_dir = None
            lib = ctypes.CDLL(str(candidate))
            lib._lingtu_dll_dir = dll_dir
            configure_signatures(lib)
            return lib
    raise FileNotFoundError(f"pose_graph_opt library not found in {candidates}")


def library_candidates(target_dir: Path, profile: str) -> list[Path]:
    lib_dir = target_dir / profile
    if sys.platform == "win32":
        names = ["lingtu_pose_graph_opt.dll", "liblingtu_pose_graph_opt.dll"]
    elif sys.platform == "darwin":
        names = ["liblingtu_pose_graph_opt.dylib"]
    else:
        names = ["liblingtu_pose_graph_opt.so"]
    return [lib_dir / name for name in names]


def configure_signatures(lib: ctypes.CDLL) -> None:
    lib.lt_pose_graph_opt_create.argtypes = [ctypes.POINTER(ConfigC)]
    lib.lt_pose_graph_opt_create.restype = ctypes.c_void_p
    lib.lt_pose_graph_opt_destroy.argtypes = [ctypes.c_void_p]
    lib.lt_pose_graph_opt_destroy.restype = None
    lib.lt_pose_graph_opt_process_se3.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(Pose3C),
        ctypes.c_uint64,
        ctypes.POINTER(Prior3C),
        ctypes.c_uint64,
        ctypes.POINTER(Between3C),
        ctypes.c_uint64,
        ctypes.POINTER(ReportC),
    ]
    lib.lt_pose_graph_opt_process_se3.restype = ctypes.c_int32
    lib.lt_pose_graph_opt_copy_result_poses.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(Pose3C),
        ctypes.c_uint64,
        ctypes.POINTER(ctypes.c_uint64),
    ]
    lib.lt_pose_graph_opt_copy_result_poses.restype = ctypes.c_int32


def config_from_fixture(fixture_data: dict[str, Any]) -> ConfigC:
    config = fixture_data.get("config", {})
    fixed_pose_index = config.get("fixed_pose_index", 0)
    if fixed_pose_index is None:
        fixed_pose_index = -1
    return ConfigC(
        ctypes.sizeof(ConfigC),
        1,
        int(config.get("max_iterations", 80)),
        int(config.get("method", 1)),
        int(fixed_pose_index),
        1 if config.get("auto_anchor", True) else 0,
        float(config.get("initial_lambda", 1e-3)),
        float(config.get("tolerance", 1e-9)),
        float(config.get("numeric_epsilon", 1e-6)),
    )


def pose_array(poses: list[dict[str, Any]]) -> ctypes.Array[Pose3C]:
    return (Pose3C * len(poses))(*(pose_from_json(pose) for pose in poses))


def prior_array(priors: list[dict[str, Any]]) -> ctypes.Array[Prior3C]:
    return (Prior3C * len(priors))(
        *(
            Prior3C(
                int(prior["index"]),
                0,
                pose_from_json(prior["pose"]),
                upper_from_json(prior["information_upper"]),
            )
            for prior in priors
        )
    )


def between_array(betweens: list[dict[str, Any]]) -> ctypes.Array[Between3C]:
    return (Between3C * len(betweens))(
        *(
            Between3C(
                int(endpoint_value(between, "from")),
                int(endpoint_value(between, "to")),
                pose_from_json(between["pose_from_to"]),
                upper_from_json(between["information_upper"]),
            )
            for between in betweens
        )
    )


def endpoint_value(between: dict[str, Any], endpoint: str) -> Any:
    abi_key = f"{endpoint}_index"
    if abi_key in between:
        return between[abi_key]
    return between[endpoint]


def pose_from_json(pose: dict[str, Any]) -> Pose3C:
    return Pose3C(
        (ctypes.c_double * 3)(*pose["t_xyz"]),
        (ctypes.c_double * 4)(*pose["q_wxyz"]),
    )


def pose_to_json(pose: Pose3C) -> dict[str, list[float]]:
    return {
        "t_xyz": [pose.t_xyz[idx] for idx in range(3)],
        "q_wxyz": [pose.q_wxyz[idx] for idx in range(4)],
    }


def upper_from_json(values: list[float]) -> ctypes.Array[ctypes.c_double]:
    return (ctypes.c_double * 21)(*[float(value) for value in values])


def truth_chain(pose_count: int, odom: dict[str, list[float]]) -> list[dict[str, list[float]]]:
    poses = [identity_pose()]
    for _ in range(1, pose_count):
        poses.append(compose_pose(poses[-1], odom))
    return poses


def drifted_poses(
    truth: list[dict[str, list[float]]],
    drift_fn,
) -> list[dict[str, list[float]]]:
    poses = [identity_pose()]
    for idx, pose in enumerate(truth[1:], start=1):
        poses.append(compose_pose(pose, drift_fn(float(idx))))
    return poses


def identity_pose() -> dict[str, list[float]]:
    return {"t_xyz": [0.0, 0.0, 0.0], "q_wxyz": [1.0, 0.0, 0.0, 0.0]}


def rpy_pose(
    x: float,
    y: float,
    z: float,
    roll: float,
    pitch: float,
    yaw: float,
) -> dict[str, list[float]]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return {
        "t_xyz": [x, y, z],
        "q_wxyz": normalize_quat(
            [
                cr * cp * cy + sr * sp * sy,
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
            ]
        ),
    }


def compose_pose(
    left: dict[str, list[float]], right: dict[str, list[float]]
) -> dict[str, list[float]]:
    left_q = left["q_wxyz"]
    right_q = right["q_wxyz"]
    rotated = rotate_vector(left_q, right["t_xyz"])
    return {
        "t_xyz": [
            left["t_xyz"][idx] + rotated[idx]
            for idx in range(3)
        ],
        "q_wxyz": normalize_quat(quat_mul(left_q, right_q)),
    }


def inverse_pose(pose: dict[str, list[float]]) -> dict[str, list[float]]:
    q_inv = quat_conjugate(pose["q_wxyz"])
    t_inv = rotate_vector(q_inv, [-value for value in pose["t_xyz"]])
    return {"t_xyz": t_inv, "q_wxyz": q_inv}


def between_pose(
    left: dict[str, list[float]], right: dict[str, list[float]]
) -> dict[str, list[float]]:
    return compose_pose(inverse_pose(left), right)


def quat_mul(left: list[float], right: list[float]) -> list[float]:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return [
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    ]


def quat_conjugate(q_wxyz: list[float]) -> list[float]:
    return [q_wxyz[0], -q_wxyz[1], -q_wxyz[2], -q_wxyz[3]]


def normalize_quat(q_wxyz: list[float]) -> list[float]:
    norm = math.sqrt(sum(value * value for value in q_wxyz))
    if norm <= 0.0:
        raise ValueError("zero quaternion")
    q = [value / norm for value in q_wxyz]
    if q[0] < 0.0:
        q = [-value for value in q]
    return q


def rotate_vector(q_wxyz: list[float], vector: list[float]) -> list[float]:
    vector_quat = [0.0, vector[0], vector[1], vector[2]]
    rotated = quat_mul(quat_mul(q_wxyz, vector_quat), quat_conjugate(q_wxyz))
    return rotated[1:4]


def diagonal_information_upper(diagonal: list[float]) -> list[float]:
    values = [[0.0 for _ in range(6)] for _ in range(6)]
    for idx, value in enumerate(diagonal):
        values[idx][idx] = value
    return pack_upper(values)


def full_information_upper() -> list[float]:
    values = [[0.0 for _ in range(6)] for _ in range(6)]
    for idx in range(6):
        values[idx][idx] = 20.0
    for row, col, value in (
        (0, 3, 0.8),
        (1, 4, -0.6),
        (2, 5, 0.4),
        (3, 4, 0.3),
    ):
        values[row][col] = value
        values[col][row] = value
    return pack_upper(values)


def pack_upper(matrix: list[list[float]]) -> list[float]:
    return [matrix[row][col] for row in range(6) for col in range(row, 6)]


def write_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


if __name__ == "__main__":
    sys.exit(main())

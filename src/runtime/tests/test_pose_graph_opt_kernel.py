import ctypes
import math
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
KERNEL = ROOT / "src" / "kernels" / "slam" / "pose_graph_opt"
MANIFEST = KERNEL / "Cargo.toml"

OK = 0
NON_FINITE_INPUT = -4


class Pose3(ctypes.Structure):
    _fields_ = [
        ("t_xyz", ctypes.c_double * 3),
        ("q_wxyz", ctypes.c_double * 4),
    ]


class Prior3(ctypes.Structure):
    _fields_ = [
        ("index", ctypes.c_uint32),
        ("reserved0", ctypes.c_uint32),
        ("pose", Pose3),
        ("information_upper", ctypes.c_double * 21),
    ]


class Between3(ctypes.Structure):
    _fields_ = [
        ("from_index", ctypes.c_uint32),
        ("to_index", ctypes.c_uint32),
        ("pose_from_to", Pose3),
        ("information_upper", ctypes.c_double * 21),
    ]


class Config(ctypes.Structure):
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


class Report(ctypes.Structure):
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


def identity_pose(x=0.0, y=0.0, z=0.0):
    return Pose3((ctypes.c_double * 3)(x, y, z), (ctypes.c_double * 4)(1.0, 0.0, 0.0, 0.0))


def rpy_pose(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    q_wxyz = (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )
    return Pose3((ctypes.c_double * 3)(x, y, z), (ctypes.c_double * 4)(*q_wxyz))


def quaternion_dot(a, b):
    return sum(a.q_wxyz[idx] * b.q_wxyz[idx] for idx in range(4))


def diagonal_information(values):
    upper = (ctypes.c_double * 21)()
    out = 0
    for row in range(6):
        for col in range(row, 6):
            upper[out] = values[row] if row == col else 0.0
            out += 1
    return upper


def full_information():
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

    upper = (ctypes.c_double * 21)()
    out = 0
    for row in range(6):
        for col in range(row, 6):
            upper[out] = values[row][col]
            out += 1
    return upper


def default_config():
    return Config(
        ctypes.sizeof(Config),
        1,
        30,
        1,
        0,
        1,
        1e-3,
        1e-9,
        1e-6,
    )


def _library_candidates(target_dir):
    lib_dir = target_dir / "debug"
    if sys.platform == "win32":
        names = ["lingtu_pose_graph_opt.dll", "liblingtu_pose_graph_opt.dll"]
    elif sys.platform == "darwin":
        names = ["liblingtu_pose_graph_opt.dylib"]
    else:
        names = ["liblingtu_pose_graph_opt.so"]
    return [lib_dir / name for name in names]


def _staticlib_candidates(target_dir):
    lib_dir = target_dir / "release"
    if sys.platform == "win32":
        names = ["lingtu_pose_graph_opt.lib", "liblingtu_pose_graph_opt.a"]
    elif sys.platform == "darwin":
        names = ["liblingtu_pose_graph_opt.a"]
    else:
        names = ["liblingtu_pose_graph_opt.a"]
    return [lib_dir / name for name in names]


@pytest.fixture(scope="session")
def pose_graph_opt_lib(tmp_path_factory):
    if shutil.which("cargo") is None:
        pytest.skip("cargo is required for pose_graph_opt ABI tests")
    target_dir = tmp_path_factory.mktemp("pose_graph_opt_target")
    env = os.environ.copy()
    env["CARGO_TARGET_DIR"] = str(target_dir)
    subprocess.run(
        ["cargo", "build", "--manifest-path", str(MANIFEST)],
        cwd=KERNEL,
        env=env,
        check=True,
    )

    for candidate in _library_candidates(target_dir):
        if candidate.exists():
            if sys.platform == "win32":
                dll_dir = os.add_dll_directory(str(candidate.parent))
            else:
                dll_dir = None
            lib = ctypes.CDLL(str(candidate))
            lib._lingtu_dll_dir = dll_dir
            _configure_signatures(lib)
            return lib
    pytest.fail(f"pose_graph_opt dynamic library not found under {target_dir}")


def _configure_signatures(lib):
    lib.lt_pose_graph_opt_abi_version.restype = ctypes.c_uint32
    lib.lt_pose_graph_opt_abi_sizeof_pose3.restype = ctypes.c_uint64
    lib.lt_pose_graph_opt_abi_sizeof_prior3.restype = ctypes.c_uint64
    lib.lt_pose_graph_opt_abi_sizeof_between3.restype = ctypes.c_uint64
    lib.lt_pose_graph_opt_abi_sizeof_config.restype = ctypes.c_uint64
    lib.lt_pose_graph_opt_abi_sizeof_report.restype = ctypes.c_uint64

    lib.lt_pose_graph_opt_create.argtypes = [ctypes.POINTER(Config)]
    lib.lt_pose_graph_opt_create.restype = ctypes.c_void_p
    lib.lt_pose_graph_opt_destroy.argtypes = [ctypes.c_void_p]
    lib.lt_pose_graph_opt_destroy.restype = None
    lib.lt_pose_graph_opt_process_se3.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(Pose3),
        ctypes.c_uint64,
        ctypes.POINTER(Prior3),
        ctypes.c_uint64,
        ctypes.POINTER(Between3),
        ctypes.c_uint64,
        ctypes.POINTER(Report),
    ]
    lib.lt_pose_graph_opt_process_se3.restype = ctypes.c_int32
    lib.lt_pose_graph_opt_copy_result_poses.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(Pose3),
        ctypes.c_uint64,
        ctypes.POINTER(ctypes.c_uint64),
    ]
    lib.lt_pose_graph_opt_copy_result_poses.restype = ctypes.c_int32


def test_pose_graph_opt_abi_layout(pose_graph_opt_lib):
    lib = pose_graph_opt_lib

    assert lib.lt_pose_graph_opt_abi_version() == 2
    assert lib.lt_pose_graph_opt_abi_sizeof_pose3() == ctypes.sizeof(Pose3)
    assert lib.lt_pose_graph_opt_abi_sizeof_prior3() == ctypes.sizeof(Prior3)
    assert lib.lt_pose_graph_opt_abi_sizeof_between3() == ctypes.sizeof(Between3)
    assert lib.lt_pose_graph_opt_abi_sizeof_config() == ctypes.sizeof(Config)
    assert lib.lt_pose_graph_opt_abi_sizeof_report() == ctypes.sizeof(Report)


def test_pose_graph_opt_staticlib_contract(tmp_path):
    if shutil.which("cargo") is None:
        pytest.skip("cargo is required for pose_graph_opt staticlib tests")
    del tmp_path
    target_dir = KERNEL / "target"
    env = os.environ.copy()
    env.pop("CARGO_TARGET_DIR", None)
    subprocess.run(
        ["cargo", "build", "--release", "--manifest-path", str(MANIFEST)],
        cwd=KERNEL,
        env=env,
        check=True,
    )
    assert any(candidate.exists() for candidate in _staticlib_candidates(target_dir))

    if sys.platform != "win32":
        return

    native_libs = subprocess.run(
        [
            "cargo",
            "rustc",
            "--release",
            "--manifest-path",
            str(MANIFEST),
            "--",
            "--print",
            "native-static-libs",
        ],
        cwd=KERNEL,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=True,
    ).stdout.lower()
    for required in ["kernel32", "ntdll", "userenv", "ws2_32", "dbghelp"]:
        assert required in native_libs

    cmake_text = (
        (ROOT / "src" / "slam" / "pgo" / "CMakeLists.txt").read_text()
        + "\n"
        + (ROOT / "src" / "slam" / "hba" / "CMakeLists.txt").read_text()
    ).lower()
    for required in ["kernel32", "ntdll", "userenv", "ws2_32", "dbghelp"]:
        assert required in cmake_text
    if "windows.lib" in native_libs or "windows" in native_libs:
        assert "windows" in cmake_text


def test_pose_graph_opt_windows_artifact_candidates_include_msvc_names(monkeypatch, tmp_path):
    monkeypatch.setattr(sys, "platform", "win32")
    target_dir = tmp_path / "target"

    assert target_dir / "debug" / "lingtu_pose_graph_opt.dll" in _library_candidates(target_dir)
    assert target_dir / "release" / "lingtu_pose_graph_opt.lib" in _staticlib_candidates(target_dir)


def test_pose_graph_opt_abi_solves_two_pose_graph(pose_graph_opt_lib):
    lib = pose_graph_opt_lib
    poses = (Pose3 * 2)(identity_pose(), identity_pose(1.3, 0.2, 0.0))
    priors = (Prior3 * 1)(
        Prior3(0, 0, identity_pose(), diagonal_information([100.0] * 6))
    )
    betweens = (Between3 * 1)(
        Between3(0, 1, identity_pose(1.0, 0.0, 0.0), diagonal_information([10.0] * 6))
    )
    report = Report()
    config = default_config()

    handle = lib.lt_pose_graph_opt_create(ctypes.byref(config))
    assert handle
    try:
        status = lib.lt_pose_graph_opt_process_se3(
            handle,
            poses,
            len(poses),
            priors,
            len(priors),
            betweens,
            len(betweens),
            ctypes.byref(report),
        )
        assert status == OK
        assert report.final_cost < report.initial_cost

        written = ctypes.c_uint64()
        status = lib.lt_pose_graph_opt_copy_result_poses(
            handle, poses, len(poses), ctypes.byref(written)
        )
        assert status == OK
        assert written.value == 2
        assert abs(poses[1].t_xyz[0] - 1.0) < 1e-5
        assert abs(poses[1].t_xyz[1]) < 1e-5
    finally:
        lib.lt_pose_graph_opt_destroy(handle)


def test_pose_graph_opt_abi_solves_rotated_se3_between_graph(pose_graph_opt_lib):
    lib = pose_graph_opt_lib
    target = rpy_pose(1.0, 0.25, 0.08, roll=0.08, pitch=-0.04, yaw=0.35)
    poses = (Pose3 * 2)(
        identity_pose(),
        rpy_pose(1.35, -0.1, 0.18, roll=0.02, pitch=0.03, yaw=0.08),
    )
    priors = (Prior3 * 1)(
        Prior3(0, 0, identity_pose(), diagonal_information([100.0] * 6))
    )
    betweens = (Between3 * 1)(
        Between3(0, 1, target, diagonal_information([20.0] * 6))
    )
    report = Report()
    config = default_config()
    config.max_iterations = 60

    handle = lib.lt_pose_graph_opt_create(ctypes.byref(config))
    assert handle
    try:
        status = lib.lt_pose_graph_opt_process_se3(
            handle,
            poses,
            len(poses),
            priors,
            len(priors),
            betweens,
            len(betweens),
            ctypes.byref(report),
        )
        assert status == OK
        assert report.final_cost < report.initial_cost

        written = ctypes.c_uint64()
        status = lib.lt_pose_graph_opt_copy_result_poses(
            handle, poses, len(poses), ctypes.byref(written)
        )
        assert status == OK
        assert written.value == 2
        for idx in range(3):
            assert abs(poses[1].t_xyz[idx] - target.t_xyz[idx]) < 1e-5
        assert abs(quaternion_dot(poses[1], poses[1]) - 1.0) < 1e-9
        assert abs(quaternion_dot(poses[1], target)) > 1.0 - 1e-8
    finally:
        lib.lt_pose_graph_opt_destroy(handle)


def test_pose_graph_opt_abi_solves_hba_full_information_graph(pose_graph_opt_lib):
    lib = pose_graph_opt_lib
    poses = (Pose3 * 4)(
        identity_pose(),
        identity_pose(0.8, 0.1, 0.0),
        identity_pose(1.7, -0.1, 0.1),
        identity_pose(2.5, 0.2, -0.1),
    )
    betweens = (Between3 * 4)(
        Between3(0, 1, identity_pose(0.7, 0.0, 0.0), full_information()),
        Between3(1, 2, identity_pose(0.7, 0.0, 0.0), full_information()),
        Between3(2, 3, identity_pose(0.7, 0.0, 0.0), full_information()),
        Between3(0, 3, identity_pose(2.1, 0.0, 0.0), full_information()),
    )
    report = Report()
    config = default_config()
    config.max_iterations = 60

    handle = lib.lt_pose_graph_opt_create(ctypes.byref(config))
    assert handle
    try:
        status = lib.lt_pose_graph_opt_process_se3(
            handle,
            poses,
            len(poses),
            None,
            0,
            betweens,
            len(betweens),
            ctypes.byref(report),
        )
        assert status == OK
        assert report.final_cost < report.initial_cost * 1e-6

        written = ctypes.c_uint64()
        status = lib.lt_pose_graph_opt_copy_result_poses(
            handle, poses, len(poses), ctypes.byref(written)
        )
        assert status == OK
        assert written.value == 4
        assert abs(poses[3].t_xyz[0] - 2.1) < 1e-5
        assert abs(poses[3].t_xyz[1]) < 1e-5
        assert abs(poses[3].t_xyz[2]) < 1e-5
    finally:
        lib.lt_pose_graph_opt_destroy(handle)


def test_pose_graph_opt_abi_rejects_invalid_quaternion(pose_graph_opt_lib):
    lib = pose_graph_opt_lib
    poses = (Pose3 * 1)(
        Pose3(
            (ctypes.c_double * 3)(0.0, 0.0, 0.0),
            (ctypes.c_double * 4)(0.0, 0.0, 0.0, 0.0),
        )
    )
    report = Report()
    config = default_config()

    handle = lib.lt_pose_graph_opt_create(ctypes.byref(config))
    assert handle
    try:
        status = lib.lt_pose_graph_opt_process_se3(
            handle,
            poses,
            len(poses),
            None,
            0,
            None,
            0,
            ctypes.byref(report),
        )
        assert status == NON_FINITE_INPUT
    finally:
        lib.lt_pose_graph_opt_destroy(handle)

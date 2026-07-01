"""PCT runtime loader.

This is the PCT runtime entrypoint used by LingTu Python code. The default
runtime is the Rust GPMP process/FFI path. The old native/GTSAM path remains
only for explicit parity checks and must be enabled by env vars.
"""

from __future__ import annotations

import ctypes
import heapq
import hashlib
import json
import math
import os
import pickle
import platform
import subprocess
import sys
import tempfile
import time
import types
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable


PCT_EXTENSION_MODULES = ("a_star", "ele_planner", "traj_opt")
PCT_SHARED_LIBS = (
    "libmetis-gtsam.so",
    "libgtsam.so",
    "libcommon_smoothing.so",
    "liba_star_search.so",
    "libmap_manager.so",
    "libgpmp_optimizer.so",
    "libele_planner_lib.so",
)
PCT_NATIVE_BINARY_FORMAT = "linux_elf"
PCT_RUST_PROCESS_BINARY_FORMAT = "rust_process_exe"
PCT_RUST_FFI_BINARY_FORMAT = "rust_cdylib"
PCT_KNOWN_GOOD_PYTHON_TAG = "py310"
PCT_PLANNER_DIR_NAMES = ("pct_planner", "PCT_planner")
PCT_RUNTIME_DIR_NAMES = ("pct_runtime",)
PCT_RUNNABLE_DIR_NAMES = PCT_RUNTIME_DIR_NAMES
PCT_PLANNER_RUNTIME_ENV = "LINGTU_PCT_PLANNER_RUNTIME"
PCT_LEGACY_NATIVE_ALLOW_ENV = "LINGTU_PCT_ALLOW_LEGACY_GTSAM_NATIVE"
PCT_DEFAULT_PLANNER_RUNTIME = "rust_process"
PCT_SUPPORTED_PLANNER_RUNTIMES = ("native", "rust_process")
PCT_RUST_OPTIMIZER_BIN_ENV = "LINGTU_GPMP_OPTIMIZER_BIN"
PCT_RUST_OPTIMIZER_LIB_ENV = "LINGTU_GPMP_OPTIMIZER_LIB"
PCT_RUST_OPTIMIZER_CALL_ENV = "LINGTU_GPMP_OPTIMIZER_CALL"
PCT_RUST_OPTIMIZER_ABI_VERSION = 1
PCT_GPMP_OPTIMIZER_REQUEST_SCHEMA = "lingtu.pct_gpmp.optimize.request.v1"
PCT_GPMP_OPTIMIZER_RESPONSE_SCHEMA = "lingtu.pct_gpmp.optimize.response.v1"
PCT_GPMP_REQUESTED_LINEAR_SOLVERS = {"auto", "dense", "sparse", "block_tridiagonal"}
PCT_GPMP_REPORTED_LINEAR_SOLVERS = {"dense", "sparse_cholesky", "block_tridiagonal"}
PCT_GPMP_NONLINEAR_OPTIMIZERS = {"levenberg_marquardt", "gauss_newton"}
PCT_RUST_OPTIMIZER_CALL_MODES = {"auto", "ffi", "process"}


@dataclass(frozen=True)
class PctRuntimePaths:
    repo_root: Path
    runnable_root: Path
    planner_root: Path
    scripts_dir: Path
    lib_root: Path
    lib_dir: Path
    canonical_arch: str
    python_tag: str


@dataclass(frozen=True)
class PctPlannerRuntime:
    """Loaded PCT planning runtime with a stable replacement boundary."""

    name: str
    planner: Any
    runtime_paths: PctRuntimePaths | None
    diagnostics: dict[str, Any]


def _repo_root_from_here() -> Path:
    return Path(__file__).resolve().parents[8]


def _canonical_arch(machine: str | None = None) -> str:
    raw = (machine or platform.machine()).lower()
    return {
        "amd64": "x86_64",
        "x64": "x86_64",
        "x86_64": "x86_64",
        "arm64": "aarch64",
        "aarch64": "aarch64",
    }.get(raw, raw)


def _python_tag() -> str:
    return f"py{sys.version_info.major}{sys.version_info.minor}"


def _requested_pct_planner_runtime() -> str:
    return os.environ.get(PCT_PLANNER_RUNTIME_ENV, PCT_DEFAULT_PLANNER_RUNTIME)


def _truthy_env(value: str | None) -> bool:
    return str(value or "").strip().lower() in {"1", "true", "yes", "on"}


def _legacy_native_runtime_allowed() -> bool:
    return _truthy_env(os.environ.get(PCT_LEGACY_NATIVE_ALLOW_ENV))


def _auto_pct_planner_runtime(repo_root: str | os.PathLike[str] | Path | None = None) -> str:
    root = Path(repo_root).resolve() if repo_root is not None else _repo_root_from_here()
    if any(candidate.is_file() for candidate in _candidate_rust_gpmp_binaries(root)):
        return "rust_process"
    return "rust_process"


def resolve_pct_planner_runtime(
    requested: str | None = None,
    *,
    repo_root: str | os.PathLike[str] | Path | None = None,
) -> str:
    """Return the concrete PCT planner name selected for this process.

    The default is the Rust process runtime. ``auto`` is kept as an explicit
    compatibility selector and always resolves to Rust; it never falls back to
    legacy Linux/GTSAM native modules. Native mode is reserved for parity
    baselines and requires both ``LINGTU_PCT_PLANNER_RUNTIME=native`` and
    ``LINGTU_PCT_ALLOW_LEGACY_GTSAM_NATIVE=1``.
    """
    value = (requested if requested is not None else _requested_pct_planner_runtime())
    runtime = str(value or PCT_DEFAULT_PLANNER_RUNTIME).strip().lower()
    if runtime == "auto":
        runtime = _auto_pct_planner_runtime(repo_root)
    if runtime not in PCT_SUPPORTED_PLANNER_RUNTIMES:
        supported = ", ".join(PCT_SUPPORTED_PLANNER_RUNTIMES)
        raise ValueError(
            f"Unsupported PCT planner runtime {value!r}; supported runtimes: {supported}. "
            "Use the default Rust process runtime or explicitly select native "
            "only for legacy GTSAM comparison runs."
        )
    if runtime == "native" and not _legacy_native_runtime_allowed():
        raise ValueError(
            "PCT native runtime is a legacy GTSAM comparison path. Set "
            f"{PCT_LEGACY_NATIVE_ALLOW_ENV}=1 together with "
            f"{PCT_PLANNER_RUNTIME_ENV}=native only for parity baselines."
        )
    return runtime


def inspect_pct_planner_runtime(
    requested: str | None = None,
    *,
    repo_root: str | os.PathLike[str] | Path | None = None,
) -> dict[str, Any]:
    value = requested if requested is not None else _requested_pct_planner_runtime()
    try:
        resolved = resolve_pct_planner_runtime(value, repo_root=repo_root)
        error = ""
    except ValueError as exc:
        resolved = ""
        error = str(exc)
    return {
        "runtime_env": PCT_PLANNER_RUNTIME_ENV,
        "requested": str(value or PCT_DEFAULT_PLANNER_RUNTIME),
        "resolved": resolved,
        "supported": error == "",
        "supported_runtimes": list(PCT_SUPPORTED_PLANNER_RUNTIMES),
        "default_runtime": PCT_DEFAULT_PLANNER_RUNTIME,
        "legacy_native_allow_env": PCT_LEGACY_NATIVE_ALLOW_ENV,
        "legacy_native_allowed": _legacy_native_runtime_allowed(),
        "error": error,
    }


def _extension_glob(module: str, py_tag: str) -> str:
    if py_tag.startswith("py"):
        abi = py_tag[2:]
    else:
        abi = py_tag
    return f"{module}.cpython-{abi}-*.so"


def _expected_extension_name(module: str, py_tag: str, canonical_arch: str) -> str:
    abi = py_tag[2:] if py_tag.startswith("py") else py_tag
    arch = "x86_64" if canonical_arch == "x86_64" else canonical_arch
    return f"{module}.cpython-{abi}-{arch}-linux-gnu.so"


def _has_extension_modules(path: Path, py_tag: str) -> bool:
    if not path.is_dir():
        return False
    for module in PCT_EXTENSION_MODULES:
        if not any(path.glob(_extension_glob(module, py_tag))):
            return False
    return True


def _missing_shared_libs(path: Path) -> list[str]:
    if not path.is_dir():
        return list(PCT_SHARED_LIBS)
    return [name for name in PCT_SHARED_LIBS if not (path / name).exists()]


def _has_runtime_shared_libs(path: Path) -> bool:
    return not _missing_shared_libs(path)


def _platform_system() -> str:
    return platform.system().lower()


def _host_platform_supported(system: str | None = None) -> bool:
    return (system or _platform_system()) == "linux"


def _host_platform_blocker(system: str | None = None) -> str:
    resolved = system or _platform_system()
    if _host_platform_supported(resolved):
        return ""
    return (
        "PCT native artifacts are Linux ELF extension modules; run this gate on "
        f"Linux or build a matching native runtime for host platform {resolved!r}."
    )


def _recommended_build_command(canonical_arch: str) -> str:
    if canonical_arch == "x86_64":
        return (
            "LINGTU_PCT_BUILD_LEGACY_GTSAM_NATIVE=1 "
            "bash src/nav/services/plan/global_planner/algorithm/pct/runtime/build_legacy_native_x86_64.sh"
        )
    if canonical_arch == "aarch64":
        return (
            "build on the S100P/aarch64 target or deploy matching "
            "pct_planner/planner/lib CPython 3.10 artifacts"
        )
    return "build matching PCT native artifacts for this host architecture and Python ABI"


def _rust_gpmp_binary_name() -> str:
    return "gpmp_optimize.exe" if os.name == "nt" else "gpmp_optimize"


def _rust_gpmp_library_names() -> tuple[str, ...]:
    if os.name == "nt":
        return ("lingtu_gpmp_trajectory_optimizer.dll",)
    if sys.platform == "darwin":
        return ("liblingtu_gpmp_trajectory_optimizer.dylib",)
    return ("liblingtu_gpmp_trajectory_optimizer.so",)


def _rust_gpmp_crate_root(repo_root: Path) -> Path:
    return repo_root / "src" / "kernels" / "planning" / "gpmp_trajectory_optimizer"


def _first_existing_dir(candidates: Iterable[Path]) -> Path:
    candidates_list = list(candidates)
    for candidate in candidates_list:
        if candidate.is_dir():
            return candidate
    return candidates_list[0]


def _pct_runtime_root(repo_root: Path) -> Path:
    return (
        repo_root
        / "src"
        / "nav"
        / "services"
        / "plan"
        / "global_planner"
        / "algorithm"
        / "pct"
        / "runtime"
    )


def _runnable_root(repo_root: Path) -> Path:
    return _pct_runtime_root(repo_root)


def _candidate_rust_gpmp_artifact_dirs(repo_root: Path) -> list[Path]:
    runnable_root = _runnable_root(repo_root)
    canonical_arch = _canonical_arch()
    return [
        runnable_root / "rust" / canonical_arch,
        runnable_root / "rust",
    ]


def _candidate_rust_gpmp_binaries(repo_root: Path) -> list[Path]:
    env_bin = os.environ.get(PCT_RUST_OPTIMIZER_BIN_ENV)
    candidates: list[Path] = []
    if env_bin:
        candidates.append(Path(env_bin).expanduser())
    crate_root = _rust_gpmp_crate_root(repo_root)
    name = _rust_gpmp_binary_name()
    for artifact_dir in _candidate_rust_gpmp_artifact_dirs(repo_root):
        candidates.append(artifact_dir / name)
    candidates.extend(
        [
            crate_root / "target" / "release" / name,
            crate_root / "target" / "debug" / name,
        ]
    )
    seen: set[Path] = set()
    unique: list[Path] = []
    for candidate in candidates:
        resolved = candidate.resolve() if candidate.exists() else candidate
        if resolved in seen:
            continue
        seen.add(resolved)
        unique.append(candidate)
    return unique


def _candidate_rust_gpmp_libraries(repo_root: Path) -> list[Path]:
    env_lib = os.environ.get(PCT_RUST_OPTIMIZER_LIB_ENV)
    candidates: list[Path] = []
    if env_lib:
        candidates.append(Path(env_lib).expanduser())
    crate_root = _rust_gpmp_crate_root(repo_root)
    for name in _rust_gpmp_library_names():
        for artifact_dir in _candidate_rust_gpmp_artifact_dirs(repo_root):
            candidates.append(artifact_dir / name)
        candidates.extend(
            [
                crate_root / "target" / "release" / name,
                crate_root / "target" / "debug" / name,
            ]
        )
    seen: set[Path] = set()
    unique: list[Path] = []
    for candidate in candidates:
        resolved = candidate.resolve() if candidate.exists() else candidate
        if resolved in seen:
            continue
        seen.add(resolved)
        unique.append(candidate)
    return unique


def resolve_rust_gpmp_optimizer_binary(
    repo_root: str | os.PathLike[str] | None = None,
) -> Path:
    root = Path(repo_root).resolve() if repo_root is not None else _repo_root_from_here()
    for candidate in _candidate_rust_gpmp_binaries(root):
        if candidate.is_file():
            return candidate.resolve()
    searched = ", ".join(str(path) for path in _candidate_rust_gpmp_binaries(root))
    raise FileNotFoundError(
        f"No Rust GPMP optimizer binary found. Set {PCT_RUST_OPTIMIZER_BIN_ENV} "
        f"or build with: cargo build --release --manifest-path "
        f"{_rust_gpmp_crate_root(root) / 'Cargo.toml'} --bin gpmp_optimize. "
        f"Searched: {searched}"
    )


def resolve_rust_gpmp_optimizer_library(
    repo_root: str | os.PathLike[str] | None = None,
) -> Path:
    root = Path(repo_root).resolve() if repo_root is not None else _repo_root_from_here()
    for candidate in _candidate_rust_gpmp_libraries(root):
        if candidate.is_file():
            return candidate.resolve()
    searched = ", ".join(str(path) for path in _candidate_rust_gpmp_libraries(root))
    raise FileNotFoundError(
        f"No Rust GPMP optimizer dynamic library found. Set {PCT_RUST_OPTIMIZER_LIB_ENV} "
        f"or build with: cargo build --release --manifest-path "
        f"{_rust_gpmp_crate_root(root) / 'Cargo.toml'}. "
        f"Searched: {searched}"
    )


def inspect_rust_gpmp_optimizer_binary(
    repo_root: str | os.PathLike[str] | None = None,
) -> dict[str, Any]:
    root = Path(repo_root).resolve() if repo_root is not None else _repo_root_from_here()
    candidates = _candidate_rust_gpmp_binaries(root)
    try:
        binary = resolve_rust_gpmp_optimizer_binary(root)
        return {
            "env": PCT_RUST_OPTIMIZER_BIN_ENV,
            "ok": True,
            "binary": str(binary),
            "crate_root": str(_rust_gpmp_crate_root(root)),
            "searched": [str(path) for path in candidates],
            "error": "",
        }
    except FileNotFoundError as exc:
        return {
            "env": PCT_RUST_OPTIMIZER_BIN_ENV,
            "ok": False,
            "binary": "",
            "crate_root": str(_rust_gpmp_crate_root(root)),
            "searched": [str(path) for path in candidates],
            "error": str(exc),
        }


def inspect_rust_gpmp_optimizer_library(
    repo_root: str | os.PathLike[str] | None = None,
) -> dict[str, Any]:
    root = Path(repo_root).resolve() if repo_root is not None else _repo_root_from_here()
    candidates = _candidate_rust_gpmp_libraries(root)
    try:
        library = resolve_rust_gpmp_optimizer_library(root)
        return {
            "env": PCT_RUST_OPTIMIZER_LIB_ENV,
            "ok": True,
            "library": str(library),
            "crate_root": str(_rust_gpmp_crate_root(root)),
            "searched": [str(path) for path in candidates],
            "error": "",
        }
    except FileNotFoundError as exc:
        return {
            "env": PCT_RUST_OPTIMIZER_LIB_ENV,
            "ok": False,
            "library": "",
            "crate_root": str(_rust_gpmp_crate_root(root)),
            "searched": [str(path) for path in candidates],
            "error": str(exc),
        }


def resolve_rust_gpmp_optimizer_call_mode(value: str | None = None) -> str:
    requested = (value or os.environ.get(PCT_RUST_OPTIMIZER_CALL_ENV) or "auto").strip().lower()
    if requested not in PCT_RUST_OPTIMIZER_CALL_MODES:
        raise ValueError(
            f"unsupported {PCT_RUST_OPTIMIZER_CALL_ENV}: {requested!r}; "
            f"expected one of {sorted(PCT_RUST_OPTIMIZER_CALL_MODES)}"
        )
    return requested



__all__ = [name for name in globals() if not name.startswith('__')]


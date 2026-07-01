from __future__ import annotations

from .common import *  # noqa: F401,F403

def _first_existing_dir(candidates: Iterable[Path]) -> Path:
    candidates_list = list(candidates)
    for candidate in candidates_list:
        if candidate.is_dir():
            return candidate
    return candidates_list[0]


def _external_pct_root(repo_root: Path) -> Path:
    return (
        repo_root
        / "src"
        / "nav"
        / "services"
        / "plan"
        / "global_planner"
        / "algorithm"
        / "pct"
        / "vendor"
    )


def _planner_root(repo_root: Path) -> Path:
    return _first_existing_dir(
        _external_pct_root(repo_root) / name / "planner"
        for name in PCT_PLANNER_DIR_NAMES
    )


def _pct_root(repo_root: Path) -> Path:
    return _planner_root(repo_root).parent


def _tomogram_search_dir(repo_root: Path, cfg: Any) -> Path:
    wrapper_cfg = getattr(cfg, "wrapper", None)
    tomo_dir = str(getattr(wrapper_cfg, "tomo_dir", "/rsc/tomogram/") or "/rsc/tomogram/")
    root = _pct_root(repo_root)
    if tomo_dir.startswith(("/", "\\")):
        return Path(str(root) + tomo_dir)
    return root / tomo_dir


def _resolve_pct_tomogram_input(
    repo_root: Path,
    tomogram_path: str | os.PathLike[str],
    cfg: Any,
) -> Path:
    requested = Path(tomogram_path).expanduser()
    candidate_bases: list[Path] = [requested]
    if not requested.is_absolute():
        candidate_bases.append(Path.cwd() / requested)
        candidate_bases.append(_tomogram_search_dir(repo_root, cfg) / requested)

    candidates: list[Path] = []
    for base in candidate_bases:
        if base.suffix:
            candidates.append(base)
        else:
            candidates.extend((base.with_suffix(".pickle"), base.with_suffix(".pcd"), base))

    seen: set[Path] = set()
    for candidate in candidates:
        key = candidate.resolve() if candidate.exists() else candidate
        if key in seen:
            continue
        seen.add(key)
        if candidate.exists():
            return candidate.resolve()
    return requested


def _runnable_root(repo_root: Path) -> Path:
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


def _available_extension_modules(path: Path) -> list[str]:
    if not path.is_dir():
        return []
    names: list[str] = []
    for module in PCT_EXTENSION_MODULES:
        names.extend(item.name for item in path.glob(f"{module}.cpython-*-*.so"))
    return sorted(names)


def _candidate_diagnostic(
    path: Path,
    py_tag: str,
    canonical_arch: str,
) -> dict[str, Any]:
    missing = [
        _expected_extension_name(module, py_tag, canonical_arch)
        for module in PCT_EXTENSION_MODULES
        if not any(path.glob(_extension_glob(module, py_tag)))
    ]
    return {
        "path": str(path),
        "exists": path.is_dir(),
        "available_extension_modules": _available_extension_modules(path),
        "missing": missing,
        "shared_missing": _missing_shared_libs(path),
        "has_current_abi_extensions": path.is_dir() and not missing,
    }


def _candidate_runtime_dirs(
    runnable_root: Path,
    canonical_arch: str,
    py_tag: str,
) -> list[Path]:
    # Runtime binaries are copied into algorithm/pct/runtime/native, not loaded from the upstream planner/lib tree.
    env_dir = os.environ.get("LINGTU_PCT_LIB_DIR")
    candidates: list[Path] = []
    if env_dir:
        candidates.append(Path(env_dir).expanduser())
    candidates.extend(
        [
            runnable_root / "native" / f"{canonical_arch}_{py_tag}",
            runnable_root / "native" / canonical_arch,
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


def resolve_pct_runtime_paths(
    repo_root: str | os.PathLike[str] | None = None,
    *,
    machine: str | None = None,
) -> PctRuntimePaths:
    """Return the original PCT planner paths for this architecture.

    Raises:
        FileNotFoundError: when no directory contains the required extension
            modules for the current Python ABI.
    """
    root = Path(repo_root).resolve() if repo_root is not None else _repo_root_from_here()
    runnable_root = _runnable_root(root)
    planner_root = _planner_root(root)
    scripts_dir = planner_root / "scripts"
    lib_root = planner_root / "lib"
    canonical_arch = _canonical_arch(machine)
    py_tag = _python_tag()

    candidates = _candidate_runtime_dirs(runnable_root, canonical_arch, py_tag)
    shared_diagnostics: list[str] = []
    for candidate in candidates:
        if _has_extension_modules(candidate, py_tag):
            missing_shared = _missing_shared_libs(candidate)
            if missing_shared:
                shared_diagnostics.append(f"{candidate}: missing {', '.join(missing_shared)}")
                continue
            return PctRuntimePaths(
                repo_root=root,
                runnable_root=runnable_root,
                planner_root=planner_root,
                scripts_dir=scripts_dir,
                lib_root=lib_root,
                lib_dir=candidate,
                canonical_arch=canonical_arch,
                python_tag=py_tag,
            )

    searched = ", ".join(str(path) for path in candidates)
    shared_suffix = ""
    if shared_diagnostics:
        shared_suffix = f" Shared library gaps: {'; '.join(shared_diagnostics)}"
    raise FileNotFoundError(
        f"No runnable PCT native modules for arch={canonical_arch} python={py_tag}. "
        f"Searched: {searched}.{shared_suffix}"
    )


def inspect_pct_runtime(
    repo_root: str | os.PathLike[str] | None = None,
    *,
    machine: str | None = None,
) -> dict[str, Any]:
    """Return the same native-runtime selection that ``prepare_pct_runtime`` uses."""
    root = Path(repo_root).resolve() if repo_root is not None else _repo_root_from_here()
    runnable_root = _runnable_root(root)
    planner_root = _planner_root(root)
    lib_root = planner_root / "lib"
    canonical_arch = _canonical_arch(machine)
    py_tag = _python_tag()
    candidates = _candidate_runtime_dirs(runnable_root, canonical_arch, py_tag)
    platform_system = _platform_system()
    runtime_report = inspect_pct_planner_runtime(repo_root=root)
    if runtime_report["resolved"] == "rust_process":
        rust_report = inspect_rust_gpmp_optimizer_binary(root)
        rust_library_report = inspect_rust_gpmp_optimizer_library(root)
        call_mode = resolve_rust_gpmp_optimizer_call_mode()
        if call_mode == "ffi":
            rust_ok = bool(rust_library_report["ok"])
            binary_format = PCT_RUST_FFI_BINARY_FORMAT
            lib_dir = (
                str(Path(rust_library_report["library"]).parent)
                if rust_library_report["library"]
                else ""
            )
            searched = rust_library_report["searched"]
            required = list(_rust_gpmp_library_names())
            missing = [] if rust_ok else list(_rust_gpmp_library_names())
            error = rust_library_report["error"]
        elif call_mode == "process":
            rust_ok = bool(rust_report["ok"])
            binary_format = PCT_RUST_PROCESS_BINARY_FORMAT
            lib_dir = str(Path(rust_report["binary"]).parent) if rust_report["binary"] else ""
            searched = rust_report["searched"]
            required = [_rust_gpmp_binary_name()]
            missing = [] if rust_ok else [_rust_gpmp_binary_name()]
            error = rust_report["error"]
        else:
            rust_ok = bool(rust_library_report["ok"] or rust_report["ok"])
            binary_format = (
                PCT_RUST_FFI_BINARY_FORMAT
                if rust_library_report["ok"]
                else PCT_RUST_PROCESS_BINARY_FORMAT
            )
            lib_dir = (
                str(Path(rust_library_report["library"]).parent)
                if rust_library_report["library"]
                else str(Path(rust_report["binary"]).parent)
                if rust_report["binary"]
                else ""
            )
            searched = rust_library_report["searched"] + rust_report["searched"]
            required = [*_rust_gpmp_library_names(), _rust_gpmp_binary_name()]
            missing = [] if rust_ok else required
            error = "" if rust_ok else (rust_library_report["error"] or rust_report["error"])
        return {
            "machine": (machine or platform.machine()).lower(),
            "canonical_arch": canonical_arch,
            "python_tag": py_tag,
            "known_good_python_tag": PCT_KNOWN_GOOD_PYTHON_TAG,
            "python_abi_matches_known_good": True,
            "platform_system": platform_system,
            "os_name": os.name,
            "native_binary_format": binary_format,
            "planner_runtime": runtime_report,
            "rust_optimizer_call_mode": call_mode,
            "host_platform_supported": True,
            "host_platform_blocker": "",
            "lib_dir": lib_dir,
            "searched": searched,
            "candidate_diagnostics": [],
            "required": required,
            "missing": missing,
            "shared_missing": [],
            "optional_missing": [],
            "recommended_build_command": (
                "cargo build --release --manifest-path "
                f"{_rust_gpmp_crate_root(root) / 'Cargo.toml'}"
            ),
            "rust_gpmp_optimizer": rust_report,
            "rust_gpmp_optimizer_library": rust_library_report,
            "ok": bool(runtime_report["supported"] and rust_ok),
            "error": runtime_report["error"] or error,
        }

    try:
        paths = resolve_pct_runtime_paths(root, machine=machine)
        chosen = paths.lib_dir
        load_error = ""
    except FileNotFoundError as exc:
        chosen = next(
            (path for path in candidates if path.exists() and any(path.glob("*.so*"))),
            runnable_root / "native" / canonical_arch,
        )
        load_error = str(exc)

    required = [
        _expected_extension_name(module, py_tag, canonical_arch)
        for module in PCT_EXTENSION_MODULES
    ]
    missing = [
        expected
        for module, expected in zip(PCT_EXTENSION_MODULES, required)
        if not any(chosen.glob(_extension_glob(module, py_tag)))
    ]
    shared_missing = _missing_shared_libs(chosen)
    return {
        "machine": (machine or platform.machine()).lower(),
        "canonical_arch": canonical_arch,
        "python_tag": py_tag,
        "known_good_python_tag": PCT_KNOWN_GOOD_PYTHON_TAG,
        "python_abi_matches_known_good": py_tag == PCT_KNOWN_GOOD_PYTHON_TAG,
        "platform_system": platform_system,
        "os_name": os.name,
        "native_binary_format": PCT_NATIVE_BINARY_FORMAT,
        "planner_runtime": runtime_report,
        "host_platform_supported": _host_platform_supported(platform_system),
        "host_platform_blocker": _host_platform_blocker(platform_system),
        "lib_dir": str(chosen),
        "searched": [str(path) for path in candidates],
        "candidate_diagnostics": [
            _candidate_diagnostic(path, py_tag, canonical_arch)
            for path in candidates
        ],
        "required": required,
        "missing": missing,
        "shared_missing": shared_missing,
        "optional_missing": [],
        "recommended_build_command": _recommended_build_command(canonical_arch),
        "ok": runtime_report["supported"] and not missing and not shared_missing,
        "error": runtime_report["error"] or load_error,
    }


def _prepend_sys_path(paths: Iterable[Path]) -> None:
    for path in reversed([str(p) for p in paths]):
        if path not in sys.path:
            sys.path.insert(0, path)


def _set_linux_library_env(lib_dir: Path) -> None:
    old = os.environ.get("LD_LIBRARY_PATH", "")
    parts = [str(lib_dir)]
    if old:
        parts.append(old)
    os.environ["LD_LIBRARY_PATH"] = os.pathsep.join(parts)


def _preload_shared_libs(lib_dir: Path) -> list[str]:
    loaded: list[str] = []
    if os.name == "nt":
        return loaded
    mode = getattr(ctypes, "RTLD_GLOBAL", 0)
    for name in PCT_SHARED_LIBS:
        path = lib_dir / name
        if not path.exists():
            continue
        try:
            ctypes.CDLL(str(path), mode=mode)
        except OSError as exc:
            raise RuntimeError(f"PCT shared library failed to load: {path}: {exc}") from exc
        loaded.append(str(path))
    return loaded


def _install_lib_namespace(paths: PctRuntimePaths) -> None:
    """Expose arch-specific native modules as the ``lib`` package."""
    module = sys.modules.get("lib")
    if module is None or not hasattr(module, "__path__"):
        module = types.ModuleType("lib")
        module.__path__ = []  # type: ignore[attr-defined]
        sys.modules["lib"] = module

    namespace_path = list(getattr(module, "__path__", []))
    for path in (str(paths.lib_dir), str(paths.lib_root)):
        if path not in namespace_path:
            namespace_path.insert(0, path)
    module.__path__ = namespace_path  # type: ignore[attr-defined]


def prepare_pct_runtime(
    repo_root: str | os.PathLike[str] | None = None,
    *,
    machine: str | None = None,
    preload_shared: bool = True,
) -> PctRuntimePaths:
    """Prepare imports and shared libraries for original PCT wrapper."""
    paths = resolve_pct_runtime_paths(repo_root, machine=machine)
    _prepend_sys_path([paths.scripts_dir, paths.planner_root, paths.lib_dir, paths.lib_root])
    _set_linux_library_env(paths.lib_dir)
    if preload_shared:
        _preload_shared_libs(paths.lib_dir)
    _install_lib_namespace(paths)
    return paths


def _pct_cache_dirs(source: Path) -> list[Path]:
    dirs: list[Path] = []
    configured = os.environ.get("LINGTU_PCT_CACHE_DIR") or os.environ.get("LINGTU_CACHE_DIR")
    if configured:
        dirs.append(Path(configured).expanduser() / "pct_runnable")
    dirs.append(source.parent / ".pct_runnable_cache")
    source_key = hashlib.sha256(str(source).encode("utf-8", errors="surrogateescape")).hexdigest()[:16]
    dirs.append(Path(tempfile.gettempdir()) / "lingtu_pct_runnable_cache" / source_key)

    unique: list[Path] = []
    seen: set[Path] = set()
    for candidate in dirs:
        resolved = candidate.resolve() if candidate.exists() else candidate
        if resolved in seen:
            continue
        seen.add(resolved)
        unique.append(candidate)
    return unique


def prepare_tomogram_for_pct(
    tomogram_path: str | os.PathLike[str],
) -> Path:
    """Return a PCT-compatible tomogram path without mutating the source file.

    Some legacy LingTu builders stored tomogram grid axes as x,y for planner
    preview code. The original PCT wrapper expects row=y, col=x and derives
    ``map_dim`` directly from the last two dimensions. Only legacy
    ``builder_xy`` pickles need a cached transposed copy.
    """
    source = Path(tomogram_path).resolve()
    if source.suffix != ".pickle" or not source.exists():
        return source

    try:
        with source.open("rb") as handle:
            raw = pickle.load(handle)  # noqa: S301  # trusted local tomogram
    except Exception:
        return source

    data = raw.get("data") if isinstance(raw, dict) else None
    grid_info = raw.get("grid_info") if isinstance(raw, dict) else None
    axis_order = str((grid_info or {}).get("axis_order") or "")
    if (
        not isinstance(raw, dict)
        or data is None
        or getattr(data, "ndim", None) != 4
        or raw.get("pct_axes_transposed") is True
        or "slice_h0" not in raw
        or "slice_dh" not in raw
    ):
        return source
    if axis_order in {"row_y_col_x", "yx"}:
        return source

    import numpy as np

    stat = source.stat()
    cache_name = f"{source.stem}.pct_axes_{stat.st_size}_{stat.st_mtime_ns}.pickle"
    normalized: dict[str, Any] | None = None
    last_error: OSError | None = None
    for cache_dir in _pct_cache_dirs(source):
        tmp_cache: Path | None = None
        try:
            cache_dir.mkdir(parents=True, exist_ok=True)
            cache = cache_dir / cache_name
            if cache.exists():
                return cache
            if normalized is None:
                normalized = dict(raw)
                normalized["data"] = np.ascontiguousarray(np.transpose(np.asarray(data), (0, 1, 3, 2)))
                normalized["pct_axes_transposed"] = True
                normalized["pct_axes_source"] = str(source)
            tmp_cache = cache.with_name(f".{cache.name}.{os.getpid()}.tmp")
            with tmp_cache.open("wb") as handle:
                pickle.dump(normalized, handle, protocol=pickle.HIGHEST_PROTOCOL)
            os.replace(tmp_cache, cache)
            return cache
        except OSError as exc:
            last_error = exc
            if tmp_cache is not None:
                try:
                    tmp_cache.unlink(missing_ok=True)
                except OSError:
                    pass
            continue
    raise RuntimeError(f"Unable to write PCT tomogram cache for {source}") from last_error



from __future__ import annotations

from .common import *  # noqa: F401,F403
from .ffi import RustGpmpOptimizerLibrary
from .native_runtime import (
    _planner_root,
    _resolve_pct_tomogram_input,
    _runnable_root,
    prepare_pct_runtime,
    prepare_tomogram_for_pct,
)
from .tomogram_planner import RustProcessTomogramPlanner

def load_tomogram_planner(
    tomogram_path: str | os.PathLike[str],
    *,
    repo_root: str | os.PathLike[str] | None = None,
    obstacle_thr: float = 49.9,
) -> tuple[Any, PctRuntimePaths]:
    """Load the original ``TomogramPlanner`` against a tomogram file."""
    runtime = load_pct_planner_runtime(
        tomogram_path,
        repo_root=repo_root,
        obstacle_thr=obstacle_thr,
    )
    if runtime.runtime_paths is None:
        raise RuntimeError(f"PCT planner runtime {runtime.name!r} did not provide runtime paths")
    return runtime.planner, runtime.runtime_paths


def load_pct_planner_runtime(
    tomogram_path: str | os.PathLike[str],
    *,
    repo_root: str | os.PathLike[str] | None = None,
    obstacle_thr: float | None = None,
    runtime: str | None = None,
    planner_config: Any | None = None,
    load_tomogram_kwargs: dict[str, Any] | None = None,
) -> PctPlannerRuntime:
    """Load the selected PCT planning runtime.

    This is the single seam for replacing the current Linux-native PCT wrapper
    with a Rust or process-backed implementation.  ``planner_config`` and
    ``load_tomogram_kwargs`` let ROS entrypoints keep their existing runtime
    parameters while still using this runtime selector.
    """
    root = Path(repo_root).resolve() if repo_root is not None else _repo_root_from_here()
    runtime_name = resolve_pct_planner_runtime(runtime, repo_root=root)

    class _MinimalCfg:
        class planner:
            use_quintic = True
            optimize_trajectory = True
            max_heading_rate = 10
            obstacle_thr = 49.9

        class wrapper:
            tomo_dir = "/"
            pcd_dir = None

    cfg = planner_config if planner_config is not None else _MinimalCfg
    planner_cfg = getattr(cfg, "planner", None)
    resolved_obstacle_thr = float(
        obstacle_thr
        if obstacle_thr is not None
        else getattr(planner_cfg, "obstacle_thr", 49.9)
    )
    # Class-body lookup rules do not close over function locals reliably.
    _MinimalCfg.planner.obstacle_thr = resolved_obstacle_thr
    if planner_cfg is not None:
        setattr(planner_cfg, "obstacle_thr", resolved_obstacle_thr)
    tomogram_kwargs = dict(load_tomogram_kwargs or {})
    resolved_tomogram = _resolve_pct_tomogram_input(root, tomogram_path, cfg)

    if runtime_name == "rust_process":
        call_mode = resolve_rust_gpmp_optimizer_call_mode()
        binary: Path | None = None
        library_path: Path | None = None
        optimizer_library: RustGpmpOptimizerLibrary | None = None
        ffi_error = ""

        if call_mode in {"auto", "ffi"}:
            try:
                library_path = resolve_rust_gpmp_optimizer_library(root)
                optimizer_library = RustGpmpOptimizerLibrary(library_path)
            except Exception as exc:  # noqa: BLE001 - auto mode falls back to process.
                ffi_error = f"{type(exc).__name__}: {exc}"
                if call_mode == "ffi":
                    raise RuntimeError(
                        f"Rust GPMP optimizer FFI requested but unavailable: {ffi_error}"
                    ) from exc

        resolved_call_mode = "ffi" if optimizer_library is not None else "process"
        if optimizer_library is None:
            binary = resolve_rust_gpmp_optimizer_binary(root)

        runnable_root = _runnable_root(root)
        planner_root = _planner_root(root)
        paths = PctRuntimePaths(
            repo_root=root,
            runnable_root=runnable_root,
            planner_root=planner_root,
            scripts_dir=planner_root / "scripts",
            lib_root=_rust_gpmp_crate_root(root),
            lib_dir=(library_path.parent if library_path is not None else binary.parent),
            canonical_arch=_canonical_arch(),
            python_tag=_python_tag(),
        )
        planner = RustProcessTomogramPlanner(
            cfg,
            binary,
            optimizer_library=optimizer_library,
            optimizer_call_mode=resolved_call_mode,
        )
        prepared_tomogram = prepare_tomogram_for_pct(resolved_tomogram)
        planner.loadTomogram(str(prepared_tomogram), **tomogram_kwargs)
        return PctPlannerRuntime(
            name=runtime_name,
            planner=planner,
            runtime_paths=paths,
            diagnostics={
                "runtime": runtime_name,
                "optimizer_call_mode_requested": call_mode,
                "optimizer_call_mode": resolved_call_mode,
                "optimizer_bin": str(binary) if binary is not None else "",
                "optimizer_library": str(library_path) if library_path is not None else "",
                "optimizer_ffi_error": ffi_error,
                "tomogram": str(Path(tomogram_path)),
                "resolved_tomogram": str(resolved_tomogram),
                "prepared_tomogram": str(prepared_tomogram),
            },
        )

    if runtime_name == "native":
        paths = prepare_pct_runtime(root)
        from planner_wrapper import TomogramPlanner  # type: ignore[import-not-found]

        planner = TomogramPlanner(cfg)
        prepared_tomogram = prepare_tomogram_for_pct(resolved_tomogram)
        planner.loadTomogram(str(prepared_tomogram), **tomogram_kwargs)
        return PctPlannerRuntime(
            name=runtime_name,
            planner=planner,
            runtime_paths=paths,
            diagnostics={
                "runtime": runtime_name,
                "runtime_lib_dir": str(paths.lib_dir),
                "tomogram": str(Path(tomogram_path)),
                "resolved_tomogram": str(resolved_tomogram),
                "prepared_tomogram": str(prepared_tomogram),
            },
        )

    raise AssertionError(f"unhandled PCT planner runtime: {runtime_name}")

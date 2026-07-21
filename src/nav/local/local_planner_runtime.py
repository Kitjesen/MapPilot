"""Backend runtime setup for nav.local_planner."""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Any, Callable

from nav.local.models import LocalPlannerGridConfig
from nav.local.native import (
    create_cmu_py_backend,
    create_nanobind_backend,
)
from nav.local.parameters import (
    read_local_planner_frame_params,
    read_local_planner_grid_config,
    read_local_planner_python_params,
)
from runtime.backend_status import BackendStatus

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class LocalPlannerRuntime:
    backend: str
    status: BackendStatus
    core: Any | None = None
    nav_kernel: Any | None = None
    path_data: dict[str, Any] | None = None
    grid_config: LocalPlannerGridConfig = field(default_factory=LocalPlannerGridConfig)
    effective_params: dict[str, Any] = field(default_factory=dict)


def setup_local_planner_backend(
    backend: str,
    *,
    status: BackendStatus,
    nav_kernel_importer: Callable[[tuple[str, ...]], Any | None] | None = None,
    path_loader: Callable[[str], dict[str, Any] | None] | None = None,
) -> LocalPlannerRuntime:
    """Create the requested local-planner backend runtime."""

    grid_config = read_local_planner_grid_config()
    _log_grid_config(grid_config)

    if backend == "nanobind":
        return _setup_nanobind(
            status,
            grid_config,
            nav_kernel_importer=nav_kernel_importer,
        )
    if backend == "cmu_py":
        return _setup_cmu_py(
            "cmu_py",
            status,
            grid_config,
            nav_kernel_importer=nav_kernel_importer,
            path_loader=path_loader,
        )

    if backend == "simple":
        logger.info("LocalPlanner: simple backend (straight-line)")
        return LocalPlannerRuntime(
            backend="simple",
            status=status,
            grid_config=grid_config,
            effective_params=read_local_planner_frame_params(),
        )

    raise ValueError(f"Unknown local_planner backend {backend!r}")


def _setup_nanobind(
    status: BackendStatus,
    grid_config: LocalPlannerGridConfig,
    *,
    nav_kernel_importer: Callable[[tuple[str, ...]], Any | None] | None = None,
) -> LocalPlannerRuntime:
    backend = create_nanobind_backend(
        **({"nav_kernel_importer": nav_kernel_importer} if nav_kernel_importer is not None else {})
    )
    if backend.core is None:
        reason = backend.unavailable_reason or "compatible LingTu native navigation kernel missing"
        raise RuntimeError(
            "LocalPlanner [nanobind]: "
            f"{reason}. Rebuild the native kernel or explicitly choose "
            f"backend='cmu_py' / 'simple'. To build: {backend.build_hint}"
        )

    if backend.missing_params:
        logger.warning(
            "LocalPlanner [nanobind]: native kernel is missing "
            "LocalPlannerParams fields: %s",
            ", ".join(sorted(set(backend.missing_params))),
        )

    params = backend.effective_params
    logger.info(
        "LocalPlanner [nanobind]: effective CMU params "
        "(adjacent_range=%.2f, point_per_path_thre=%s, dir_weight=%.3f, "
        "slope_weight=%.2f, near_field_stop_dis=%.2f)",
        float(params.get("adjacent_range", 3.5)),
        params.get("point_per_path_thre", 2),
        float(params.get("dir_weight", 0.02)),
        float(params.get("slope_weight", 0.0)),
        float(params.get("near_field_stop_dis", 0.5)),
    )
    logger.info(
        "LocalPlanner [nanobind]: C++ LocalPlanner loaded "
        "(343 paths x 36 dirs)"
    )
    return LocalPlannerRuntime(
        backend="nanobind",
        status=status,
        core=backend.core,
        grid_config=grid_config,
        effective_params=backend.effective_params,
    )


def _setup_cmu_py(
    backend_name: str,
    status: BackendStatus,
    grid_config: LocalPlannerGridConfig,
    *,
    nav_kernel_importer: Callable[[tuple[str, ...]], Any | None] | None = None,
    path_loader: Callable[[str], dict[str, Any] | None] | None = None,
) -> LocalPlannerRuntime:
    backend = create_cmu_py_backend(
        **({"nav_kernel_importer": nav_kernel_importer} if nav_kernel_importer is not None else {}),
        **({"path_loader": path_loader} if path_loader is not None else {}),
    )
    if backend.nav_kernel is not None:
        logger.info("LocalPlanner [cmu_py]: native scoring acceleration loaded")
    else:
        logger.info("LocalPlanner [cmu_py]: using numpy-only scorer")
    return LocalPlannerRuntime(
        backend=backend_name,
        status=status,
        nav_kernel=backend.nav_kernel,
        path_data=backend.path_data,
        grid_config=grid_config,
        effective_params={
            **read_local_planner_frame_params(),
            **read_local_planner_python_params(),
        },
    )


def _log_grid_config(grid_config: LocalPlannerGridConfig) -> None:
    if grid_config.loaded_from_config:
        logger.info(
            "LocalPlanner: loaded grid params from config "
            "(voxel=%.3f, x_off=%.2f, y_off=%.2f, radius=%.2f)",
            grid_config.voxel_size,
            grid_config.x_offset,
            grid_config.y_offset,
            grid_config.search_radius,
        )
        return
    logger.info(
        "LocalPlanner: using default grid params "
        "(voxel=0.02, x_off=3.2, y_off=5.25, radius=0.45) - "
        "add local_planner_grid to robot_config.yaml to override"
    )

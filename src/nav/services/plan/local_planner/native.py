"""Native local planner backend creation."""

from __future__ import annotations

import os
from typing import Any, Callable

from nav.kernel import nav_kernel_build_hint, try_import_nav_kernel
from nav.services.plan.local_planner.models import (
    CmuPyLocalPlannerBackend,
    NanobindLocalPlannerBackend,
)
from nav.services.plan.local_planner.parameters import build_local_planner_params
from nav.services.plan.local_planner.path_tables import (
    load_cmu_py_paths,
    local_planner_paths_dir,
)


def create_nanobind_backend(
    *,
    nav_kernel_importer: Callable[[tuple[str, ...]], Any | None] = try_import_nav_kernel,
    build_hint_provider: Callable[[], str] = nav_kernel_build_hint,
    config_getter: Callable[[], Any] | None = None,
    paths_dir: str | None = None,
) -> NanobindLocalPlannerBackend:
    """Create the nanobind nav_kernel LocalPlanner backend, or report unavailability."""
    hint = build_hint_provider()
    nav_kernel = nav_kernel_importer(("LocalPlannerParams", "nav.local_planner"))
    if nav_kernel is None:
        return NanobindLocalPlannerBackend(
            core=None,
            unavailable_reason="compatible LingTu native navigation kernel missing",
            build_hint=hint,
        )

    resolved_paths_dir = os.path.normpath(paths_dir or local_planner_paths_dir())
    try:
        params, summary, missing = build_local_planner_params(
            nav_kernel,
            config_getter=config_getter,
        )
        core = nav_kernel.LocalPlanner(params)
        if not core.load_paths(resolved_paths_dir):
            raise RuntimeError(
                f"LocalPlanner [nanobind]: failed to load paths from {resolved_paths_dir}. "
                f"Cannot start without path lookup table."
            )
        return NanobindLocalPlannerBackend(
            core=core,
            effective_params=summary,
            missing_params=missing,
            paths_dir=resolved_paths_dir,
            build_hint=hint,
        )
    except RuntimeError:
        raise
    except Exception as e:
        raise RuntimeError(
            f"LocalPlanner [nanobind]: LingTu native navigation kernel init failed: {e}. "
            f"Rebuild the native kernel or explicitly choose backend='cmu_py' / 'simple'."
        ) from e


def create_cmu_py_backend(
    *,
    nav_kernel_importer: Callable[[tuple[str, ...]], Any | None] = try_import_nav_kernel,
    path_loader: Callable[[str], dict | None] = load_cmu_py_paths,
    paths_dir: str | None = None,
) -> CmuPyLocalPlannerBackend:
    """Create the cmu_py backend data bundle."""
    nav_kernel = nav_kernel_importer(())
    resolved_paths_dir = os.path.normpath(paths_dir or local_planner_paths_dir())
    path_data = path_loader(resolved_paths_dir)
    if path_data is None:
        raise RuntimeError(
            f"LocalPlanner [cmu_py]: failed to load paths from {resolved_paths_dir}. "
            f"Cannot start without path lookup table - use backend='simple' explicitly "
            f"for passthrough testing only."
        )
    return CmuPyLocalPlannerBackend(
        nav_kernel=nav_kernel,
        path_data=path_data,
        paths_dir=resolved_paths_dir,
    )

"""Navigation service stack: goals, patrol routes, and optional schedules."""

from __future__ import annotations

import logging

from runtime.blueprint import Blueprint

logger = logging.getLogger(__name__)


def services(
    *,
    enable_goals: bool = True,
    enable_patrol_routes: bool = True,
    enable_scheduler: bool = False,
    enable_navigation: bool = True,
    **config,
) -> Blueprint:
    """Add navigation support services that are not planning algorithms."""

    bp = Blueprint()

    if not enable_navigation:
        from runtime.adapters.navigation_io import (
            add_navigation_output_adapter,
        )

        add_navigation_output_adapter(bp, **config)

    if enable_goals:
        try:
            from nav.services.goals import GoalService

            kwargs = {}
            if config.get("planning_frame_id") is not None:
                kwargs["planning_frame_id"] = config.get("planning_frame_id")
            bp.add(
                GoalService,
                alias="nav.goals",
                **kwargs,
            )
        except ImportError as exc:
            logger.warning("GoalService not available: %s", exc)

    if enable_patrol_routes:
        try:
            from nav.services.patrol import PatrolManagerModule

            kwargs = {}
            if config.get("patrol_routes_dir") is not None:
                kwargs["routes_dir"] = config.get("patrol_routes_dir")
            bp.add(
                PatrolManagerModule,
                alias="PatrolManagerModule",
                **kwargs,
            )
        except ImportError as exc:
            logger.warning("PatrolManagerModule not available: %s", exc)

    if enable_scheduler:
        try:
            from nav.services.scheduler import TaskSchedulerModule

            kwargs = {}
            if config.get("schedule_file") is not None:
                kwargs["schedule_file"] = config.get("schedule_file")
            bp.add(
                TaskSchedulerModule,
                alias="TaskSchedulerModule",
                **kwargs,
            )
        except ImportError as exc:
            logger.warning("TaskSchedulerModule not available: %s", exc)

    return bp


__all__ = ["services"]

"""Navigation service stack: command ingress only.

Inspection route storage/execution is native C++ and enters through typed
inspection commands. The old Python patrol/scheduler modules are compatibility
only and are not mounted by product stacks by default.
"""

from __future__ import annotations

import logging

from runtime.blueprint import Blueprint

logger = logging.getLogger(__name__)


def services(
    *,
    enable_goals: bool = True,
    enable_building: bool = False,
    enable_patrol_routes: bool = False,
    enable_scheduler: bool = False,
    enable_navigation: bool = True,
    **config,
) -> Blueprint:
    """Add navigation support services that are not planning algorithms."""

    bp = Blueprint()
    native_commands = bool(config.get("native_navigation_endpoint"))
    operator_assisted = (
        native_commands
        and str(config.get("product_mode") or "").strip().lower() == "teleop_avoid"
    )
    building_added = False

    if native_commands:
        try:
            from nav.commands.module import Commands
            from nav.inspection.service import Inspection

            bp.add(Commands, alias="nav.commands")
            bp.add(Inspection, alias="nav.inspection")
        except ImportError as exc:
            logger.warning("Native navigation command services not available: %s", exc)

    if operator_assisted:
        try:
            from nav.commands.operator_motion import OperatorMotion

            bp.add(
                OperatorMotion,
                alias="operator.motion",
                command_module="nav.commands",
            )
        except ImportError as exc:
            logger.warning("Operator motion ingress not available: %s", exc)

    if enable_building:
        try:
            from nav.building.service import BuildingService

            bp.add(
                BuildingService,
                alias="nav.building",
                maps_module="maps.service",
                mission_module=str(config.get("building_mission_module") or "nav.building.mission"),
            )
            building_added = True
        except ImportError as exc:
            logger.warning("Building navigation service not available: %s", exc)

    if enable_goals:
        try:
            from nav.services.goals import GoalService

            kwargs = {}
            if config.get("planning_frame_id") is not None:
                kwargs["planning_frame_id"] = config.get("planning_frame_id")
            if native_commands:
                kwargs["command_module"] = "nav.commands"
            if building_added:
                kwargs["building_module"] = "nav.building"
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

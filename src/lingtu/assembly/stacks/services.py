"""Navigation service stack: command ingress only.

Inspection route storage/execution is native C++ and enters through typed
inspection commands.
"""

from __future__ import annotations

from runtime.blueprint import Blueprint
from runtime.runtime_interface import TOPICS


def services(
    *,
    enable_goals: bool = True,
    **config,
) -> Blueprint:
    """Add navigation support services that are not planning algorithms."""

    bp = Blueprint()
    native_commands = bool(config.get("native_navigation_endpoint"))
    if native_commands:
        from lingtu.assembly.host_bus import HostBus
        from nav.commands.module import Commands

        required_topics = {
            str(topic)
            for topic in config.get("_product_required_topics", ())
        }
        required_capabilities = {
            str(capability)
            for capability in config.get("_product_required_capabilities", ())
        }
        inspection_task_topics = {
            TOPICS.inspection_task_request,
            TOPICS.inspection_task_ack,
            TOPICS.inspection_task_event,
        }
        has_inspection_task_contract = (
            inspection_task_topics <= required_topics
            and "inspection_evidence_capture_and_result_ack" in required_capabilities
        )
        bp.add(
            HostBus,
            alias="host.bus",
            require_map_scene=(
                TOPICS.maps_state in required_topics
                or TOPICS.maps_scene in required_topics
            ),
            require_inspection_task_events=(
                TOPICS.inspection_task_event in required_topics
            ),
            require_exploration_run_events=(
                TOPICS.exploration_run_event in required_topics
            ),
        )
        bp.add(
            Commands,
            alias="nav.commands",
            require_inspection_task_commands=has_inspection_task_contract,
        )
        if has_inspection_task_contract:
            from nav.inspection.service import Inspection

            bp.add(Inspection, alias="nav.inspection")

    if enable_goals and native_commands:
        from nav.services.goals import GoalService

        kwargs = {"command_module": "nav.commands"}
        if config.get("planning_frame_id") is not None:
            kwargs["planning_frame_id"] = config.get("planning_frame_id")
        bp.add(
            GoalService,
            alias="nav.goals",
            **kwargs,
        )

    return bp


__all__ = ["services"]

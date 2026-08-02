from __future__ import annotations

import json

from lingtu.assembly.stacks.navigation import navigation
from lingtu.assembly.wires.navigation import navigation_support_specs
from nav.services.goals import GoalService
from nav.skills import NavSkills
from runtime.msgs.nav import (
    NavigationCommandKind,
    NavigationCommandReceipt,
    NavigationGoalState,
    NavigationGoalStatus,
    NavigationLifecycle,
    NavigationState,
)


def _wire(skills: NavSkills, goals: GoalService) -> None:
    skills.goal_command.subscribe(goals.goal_command._deliver)
    goals.goal_status.subscribe(skills.goal_status._deliver)


def test_nav_skills_is_l6_navigation_adapter() -> None:
    skills = NavSkills()
    skills.setup()

    assert skills._layer == 6
    assert skills.mission_status._callback is not None
    assert skills.goal_status._callback is not None
    assert not hasattr(skills, "stop_signal")


def test_nav_skills_routes_goal_through_goal_service_ack() -> None:
    skills = NavSkills()
    goals = GoalService()
    skills.setup()
    goals.setup()
    _wire(skills, goals)
    received = []
    goals.goal_pose.subscribe(received.append)

    result = json.loads(skills.navigate_to(2.0, -1.5, yaw=0.4, z=0.2))

    assert result["accepted"] is True
    assert result["state"] == "accepted"
    assert result["request_id"].startswith("nav-")
    assert len(received) == 1
    assert received[0].frame_id == "map"
    assert received[0].x == 2.0
    assert received[0].y == -1.5
    assert received[0].z == 0.2


def test_nav_skills_preserves_request_id_through_native_command_capability() -> None:
    class Commands:
        def __init__(self) -> None:
            self.task_id = ""
            self.request_id = ""

        def send_goal(
            self,
            x,
            y,
            z,
            yaw,
            *,
            task_id,
            request_id=None,
        ) -> NavigationCommandReceipt:
            del x, y, z, yaw
            self.task_id = str(task_id or "")
            self.request_id = str(request_id or "")
            return NavigationCommandReceipt(
                accepted=True,
                kind=int(NavigationCommandKind.GOAL),
                task_id=self.task_id,
                request_id=self.request_id,
                endpoint_timestamp_s=123.0,
                reason="accepted",
            )

    commands = Commands()
    skills = NavSkills()
    goals = GoalService(command_module="nav.commands")
    goals.on_system_modules({"nav.commands": commands})
    skills.setup()
    goals.setup()
    _wire(skills, goals)

    result = json.loads(skills.navigate_to(1.0, 2.0))

    assert result["accepted"] is True
    assert commands.task_id == result["task_id"]
    assert result["task_id"] != result["request_id"]
    assert commands.request_id == result["request_id"]
    assert result["sink"] == "native_dds"


def test_nav_skills_cancel_and_stop_use_cancel_command() -> None:
    skills = NavSkills()
    goals = GoalService()
    skills.setup()
    goals.setup()
    _wire(skills, goals)
    reasons: list[str] = []
    goals.cancel.subscribe(reasons.append)

    cancelled = json.loads(skills.cancel_mission("operator_cancel"))
    stopped = json.loads(skills.stop_navigation())

    assert cancelled["accepted"] is True
    assert stopped["accepted"] is True
    assert reasons == ["operator_cancel", "navigation_stop"]


def test_nav_skills_patrol_accepts_structured_waypoints() -> None:
    skills = NavSkills()
    goals = GoalService()
    skills.setup()
    goals.setup()
    _wire(skills, goals)
    routes: list[list[dict]] = []
    goals.patrol_goals.subscribe(routes.append)

    result = json.loads(
        skills.start_patrol(
            [{"x": 1.0, "y": 2.0}, {"x": 3.0, "y": 4.0, "z": 0.2}],
            loop=True,
        )
    )

    assert result["accepted"] is True
    assert result["waypoint_count"] == 2
    assert routes[0][0] == {
        "x": 1.0,
        "y": 2.0,
        "z": 0.0,
        "frame_id": "map",
        "loop": True,
    }


def test_nav_skills_rejects_non_serializable_patrol_values() -> None:
    skills = NavSkills()
    skills.setup()

    result = json.loads(skills.start_patrol([{"x": object(), "y": 2.0}]))

    assert result["accepted"] is False
    assert result["state"] == "rejected"
    assert result["reason"] == "command_not_serializable"


def test_nav_skills_ignores_ack_for_other_command_sources() -> None:
    skills = NavSkills()
    skills.setup()

    skills.goal_status._deliver({"request_id": "gateway-1", "success": True})

    assert skills._acks == {}


def test_nav_skills_rejects_truthy_non_boolean_goal_ack() -> None:
    skills = NavSkills()
    skills.setup()

    def acknowledge(payload: str) -> None:
        request = json.loads(payload)
        skills.goal_status._deliver(
            {
                "request_id": request["request_id"],
                "accepted": "false",
            }
        )

    skills.goal_command.subscribe(acknowledge)

    result = json.loads(skills.navigate_to(1.0, 2.0))

    assert result["accepted"] is False
    assert result["state"] == "rejected"


def test_nav_skills_returns_canonical_mission_status() -> None:
    skills = NavSkills()
    skills.setup()
    skills.mission_status._deliver(
        {
            "state": "TRACKING",
            "mission_mode": "NAVIGATE",
            "planning_frame_id": "map",
            "position": {"x": 1.0, "y": 2.0, "z": 0.3, "yaw": 0.4},
            "wp_index": 2,
            "wp_total": 5,
            "ts": 42.0,
        }
    )

    result = json.loads(skills.get_navigation_status())

    assert result["state"] == "TRACKING"
    assert result["planning_frame_id"] == "map"
    assert result["position"]["z"] == 0.3
    assert result["wp_index"] == 2
    assert result["wp_total"] == 5


def test_nav_skills_prefers_native_navigation_state() -> None:
    skills = NavSkills()
    skills.setup()
    skills.mission_status._deliver({"state": "EXECUTING", "wp_index": 3})
    skills.navigation_state._deliver(
        NavigationState(
            ts=42.0,
            frame_id="map",
            boot_id="navd-boot",
            sequence=7,
            lifecycle_state=int(NavigationLifecycle.RECOVERING),
            active_task_id="navigation-task-gateway-42",
            active_request_id="gateway-42",
            goal_epoch=3,
            progress=0.4,
            authority="autonomy",
        )
    )

    result = json.loads(skills.get_navigation_status())
    progress = json.loads(skills.get_navigation_progress())

    assert result["state"] == "RECOVERING"
    assert result["source"] == "native_navigation_state"
    assert result["active_request_id"] == "gateway-42"
    assert progress["state"] == "RECOVERING"
    assert progress["progress_pct"] == 40.0


def test_nav_skills_returns_only_owned_native_goal_status() -> None:
    skills = NavSkills()
    skills.setup()

    def acknowledge(payload: str) -> None:
        request = json.loads(payload)
        skills.goal_status._deliver(
            {
                "request_id": request["request_id"],
                "accepted": True,
            }
        )

    skills.goal_command.subscribe(acknowledge)
    submitted = json.loads(skills.navigate_to(1.0, 2.0))
    request_id = submitted["request_id"]

    skills.navigation_goal_status._deliver(
        NavigationGoalStatus(
            ts=43.0,
            frame_id="map",
            boot_id="navd-boot",
            sequence=8,
            task_id="navigation-task-inspection-other",
            request_id="inspection-other",
            state=int(NavigationGoalState.REACHED),
            goal_epoch=4,
        )
    )
    skills.navigation_goal_status._deliver(
        NavigationGoalStatus(
            ts=44.0,
            frame_id="map",
            boot_id="navd-boot",
            sequence=9,
            task_id="navigation-task-inspection-current",
            request_id=request_id,
            state=int(NavigationGoalState.REACHED),
            goal_epoch=4,
            reason="goal_reached",
        )
    )

    result = json.loads(skills.get_navigation_result(request_id))
    foreign = json.loads(skills.get_navigation_result("inspection-other"))

    assert result["found"] is True
    assert result["status"]["state_name"] == "REACHED"
    assert result["status"]["terminal"] is True
    assert foreign["found"] is False
    assert foreign["reason"] == "request_not_owned_by_nav_skills"


def test_nav_skills_does_not_own_rejected_request() -> None:
    skills = NavSkills()
    skills.setup()

    def reject(payload: str) -> None:
        request = json.loads(payload)
        skills.goal_status._deliver(
            {
                "request_id": request["request_id"],
                "accepted": False,
                "reason": "busy",
            }
        )

    skills.goal_command.subscribe(reject)
    submitted = json.loads(skills.navigate_to(1.0, 2.0))
    result = json.loads(skills.get_navigation_result(submitted["request_id"]))

    assert submitted["accepted"] is False
    assert result["found"] is False
    assert result["reason"] == "request_not_owned_by_nav_skills"


def test_nav_skills_wiring_uses_goal_service_only() -> None:
    wires = {(wire.out_module, wire.out_port, wire.in_module, wire.in_port) for wire in navigation_support_specs()}

    assert ("nav.skills", "goal_command", "nav.goals", "goal_command") in wires
    assert ("nav.goals", "goal_status", "nav.skills", "goal_status") in wires
    assert not any(
        out_module == "nav.skills" and in_module == "nav.mission"
        for out_module, _out_port, in_module, _in_port in wires
    )


def test_navigation_stack_has_one_default_patrol_owner() -> None:
    blueprint = navigation(enable_native=False)
    aliases = [entry.name for entry in blueprint._entries]

    assert "nav.skills" in aliases
    assert "nav.patrol" not in aliases


def test_mcp_discovers_nav_skills_as_navigation_tool_owner() -> None:
    from gateway.mcp_server import MCPServerModule

    skills = NavSkills()
    mcp = MCPServerModule(port=0)
    mcp.on_system_modules({"nav.skills": skills, "MCPServerModule": mcp})

    assert mcp._tool_registry["navigate_to"].__self__ is skills
    descriptor = next(tool for tool in mcp._tool_list if tool["name"] == "start_patrol")
    assert descriptor["inputSchema"]["properties"]["waypoints"]["type"] == "array"
    assert mcp._tool_registry["get_navigation_result"].__self__ is skills


def test_local_profile_and_field_product_share_nav_skills_with_different_sinks() -> None:
    from lingtu.assembly.products import resolve_product_host_config
    from lingtu.assembly.profile_builder import (
        blueprint_for_resolved_product,
        blueprint_for_resolved_profile,
    )
    from runtime.profiles.resolver import resolve_profile_config

    profiles = {}
    for name in ("sim_nav", "nav"):
        config = (
            resolve_product_host_config(name, "real")
            if name == "nav"
            else resolve_profile_config(name)
        )
        blueprint = (
            blueprint_for_resolved_product(name, config)
            if name == "nav"
            else blueprint_for_resolved_profile(name, config)
        )
        entries = {entry.name: entry.config for entry in blueprint._entries}
        wires = {(wire.out_module, wire.out_port, wire.in_module, wire.in_port) for wire in blueprint._wires}
        profiles[name] = (entries, wires)

    for entries, wires in profiles.values():
        assert "nav.skills" in entries
        assert ("nav.skills", "goal_command", "nav.goals", "goal_command") in wires
        assert ("nav.goals", "goal_status", "nav.skills", "goal_status") in wires
    assert "command_module" not in profiles["sim_nav"][0]["nav.goals"]
    assert profiles["nav"][0]["nav.goals"]["command_module"] == "nav.commands"
    assert "nav.commands" not in profiles["sim_nav"][0]
    assert "nav.inspection" not in profiles["sim_nav"][0]
    assert "nav.commands" in profiles["nav"][0]
    assert "nav.inspection" not in profiles["nav"][0]


def test_only_compiled_inspection_product_mounts_inspection_service() -> None:
    from lingtu.assembly.products import resolve_product_host_runtime
    from lingtu.assembly.profile_builder import blueprint_from_run_plan, compile_run_plan

    products = {}
    for name in ("nav", "inspection"):
        resolved = resolve_product_host_runtime(name, "real")
        products[name] = compile_run_plan(
            resolved.product,
            resolved.env,
            resolved.config,
        )

    assert "nav.inspection" not in products["nav"].modules
    assert "nav.inspection" in products["inspection"].modules
    nav_entries = {
        entry.name: entry.config
        for entry in blueprint_from_run_plan(products["nav"])._entries
    }
    inspection_entries = {
        entry.name: entry.config
        for entry in blueprint_from_run_plan(products["inspection"])._entries
    }
    assert nav_entries["nav.commands"]["require_inspection_task_commands"] is False
    assert (
        inspection_entries["nav.commands"]["require_inspection_task_commands"]
        is True
    )

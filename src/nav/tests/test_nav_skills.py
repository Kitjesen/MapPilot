from __future__ import annotations

import json

from nav.api.skills import NavigationSkillsMixin
from nav.services.goals import GoalService
from nav.skills import NavigationSkillsModule, NavSkills
from runtime.blueprints.stacks.navigation import navigation
from runtime.blueprints.wires.navigation import navigation_support_specs


def _wire(skills: NavSkills, goals: GoalService) -> None:
    skills.goal_command.subscribe(goals.goal_command._deliver)
    goals.goal_status.subscribe(skills.goal_status._deliver)


def test_nav_skills_is_l6_and_keeps_compatibility_alias() -> None:
    skills = NavSkills()
    skills.setup()

    assert NavigationSkillsModule is NavSkills
    assert NavigationSkillsMixin is NavSkills
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


def test_nav_skills_preserves_request_id_through_native_dds_client(monkeypatch) -> None:
    from runtime.adapters.native import navigation

    class Client:
        def __init__(self) -> None:
            self.request_id = ""

        def send_goal(self, x, y, z, yaw, *, request_id=None) -> None:
            del x, y, z, yaw
            self.request_id = str(request_id or "")

    client = Client()
    monkeypatch.setattr(
        navigation,
        "get_native_navigation_client",
        lambda *, required=False: client,
    )
    skills = NavSkills()
    goals = GoalService(native_endpoint=True)
    skills.setup()
    goals.setup()
    _wire(skills, goals)

    result = json.loads(skills.navigate_to(1.0, 2.0))

    assert result["accepted"] is True
    assert client.request_id == result["request_id"]
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


def test_sim_and_field_profiles_share_nav_skills_with_different_sinks() -> None:
    from runtime.blueprints.profile_builder import blueprint_for_resolved_profile
    from runtime.profiles.resolver import resolve_profile_config

    profiles = {}
    for name in ("sim_nav", "nav"):
        config = resolve_profile_config(name)
        blueprint = blueprint_for_resolved_profile(name, config)
        entries = {entry.name: entry.config for entry in blueprint._entries}
        wires = {(wire.out_module, wire.out_port, wire.in_module, wire.in_port) for wire in blueprint._wires}
        profiles[name] = (entries, wires)

    for entries, wires in profiles.values():
        assert "nav.skills" in entries
        assert ("nav.skills", "goal_command", "nav.goals", "goal_command") in wires
        assert ("nav.goals", "goal_status", "nav.skills", "goal_status") in wires
    assert profiles["sim_nav"][0]["nav.goals"]["native_endpoint"] is False
    assert profiles["nav"][0]["nav.goals"]["native_endpoint"] is True

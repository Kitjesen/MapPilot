"""Gateway MCP server tests."""

from __future__ import annotations

import json
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from gateway.mcp_server import MCPServerModule, _text
from runtime.module import Module, skill
from runtime.msgs.geometry import Pose, Vector3
from runtime.msgs.nav import Odometry
from runtime.msgs.semantic import SceneGraph
from runtime.stream import In, Out

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_mcp(**kw) -> MCPServerModule:
    mod = MCPServerModule(**kw)
    mod.setup()
    return mod


def _odom(x: float = 1.0, y: float = 2.0, z: float = 0.0) -> Odometry:
    od = Odometry()
    od.pose = Pose(position=Vector3(x, y, z))
    return od


def _scene_graph() -> SceneGraph:
    from runtime.msgs.semantic import Detection3D, Region

    return SceneGraph(
        objects=[
            Detection3D(id="1", label="chair", position=Vector3(1, 2, 0)),
            Detection3D(id="2", label="table", position=Vector3(3, 4, 0)),
        ],
        regions=[Region(name="office", object_ids=["1", "2"], center=Vector3(2, 3, 0))],
    )


def _navigation_status() -> dict:
    return {
        "schema_version": 3,
        "task": {"state": "IDLE", "task_id": "", "reason": ""},
        "goal_admission": {"state": "ACCEPTING", "reason": ""},
        "control": {"authority": "NONE", "resume_required": False, "reason": ""},
        "motion": {
            "permission": "CLEAR",
            "observation": "QUIET",
            "stop_confirmation": "NOT_REQUESTED",
            "reason": "",
        },
        "ts": 123.0,
    }


def _fake_nav_skills():
    """Return a minimal Module with @skill navigate_to and get_navigation_status."""
    from runtime.module import Module, skill

    class _FakeNav(Module, layer=5):
        def __init__(self):
            super().__init__()
            self.goals = []

        @skill
        def navigate_to(self, x: float, y: float, yaw: float = 0.0) -> str:
            """Navigate to map coordinates (x, y)."""
            self.goals.append((x, y, yaw))
            return json.dumps({"status": "navigating", "goal": [x, y], "yaw": yaw})

        @skill
        def get_navigation_status(self) -> str:
            """Return current navigation state."""
            return json.dumps({"state": "IDLE", "goal": None})

        @skill
        def stop_navigation(self) -> str:
            """Stop all robot motion."""
            return json.dumps({"status": "stopped"})

    return _FakeNav()


class _MetadataSkills(Module):
    @skill(
        description="Read one retained task without changing runtime state.",
        parameters={"task_id": "Task identity returned by the start call."},
        output_schema={
            "type": "object",
            "properties": {"found": {"type": "boolean"}},
            "required": ["found"],
        },
        read_only=True,
    )
    def inspect_task(self, task_id: str) -> dict:
        return {"found": bool(task_id)}


class _InstructionOwner(Module):
    @skill
    def send_instruction(self, text: str) -> str:
        return f"external:{text}"


# ===========================================================================
# 1. Port declarations
# ===========================================================================


class TestMCPPorts(unittest.TestCase):
    def setUp(self):
        self.mod = MCPServerModule()

    def test_layer(self):
        self.assertEqual(MCPServerModule._layer, 6)

    def test_in_ports(self):
        for name in ("odometry", "scene_graph"):
            with self.subTest(port=name):
                self.assertIsInstance(getattr(self.mod, name), In)

    def test_out_ports(self):
        for name in ("instruction", "mode_cmd"):
            with self.subTest(port=name):
                self.assertIsInstance(getattr(self.mod, name), Out)

    def test_default_port_number(self):
        self.assertEqual(self.mod._port, 8090)

    def test_custom_port_number(self):
        m = MCPServerModule(port=9999)
        self.assertEqual(m._port, 9999)


# ===========================================================================
# 2. Telemetry callbacks
# ===========================================================================


class TestMCPTelemetry(unittest.TestCase):
    def setUp(self):
        self.mod = _make_mcp()

    def test_odom_cached(self):
        self.mod._on_odom(_odom(3.5, 7.2))
        self.assertIsNotNone(self.mod._odom)
        self.assertAlmostEqual(self.mod._odom["x"], 3.5)
        self.assertAlmostEqual(self.mod._odom["y"], 7.2)

    def test_odom_missing_before_callback(self):
        self.assertIsNone(self.mod._odom)

    def test_scene_graph_cached_as_json(self):
        self.mod._on_sg(_scene_graph())
        parsed = json.loads(self.mod._sg_json)
        self.assertIn("objects", parsed)

    def test_hierarchical_scene_graph_cache_keeps_context(self):
        graph = _scene_graph()
        graph.graph_level = "hierarchical"
        graph.graph_version = "3.0"
        graph.map_id = "building"
        graph.content_epoch = 7
        graph.floors = [{"floor_id": 0, "room_ids": [1]}]
        graph.rooms = [{"room_id": 1, "name": "office", "object_ids": ["1", "2"]}]
        graph.hierarchy_edges = [{
            "parent_type": "floor", "parent_id": 0,
            "child_type": "room", "child_id": 1,
        }]

        self.mod._on_sg(graph)
        parsed = json.loads(self.mod.get_scene_graph())

        self.assertEqual(parsed["graph_level"], "hierarchical")
        self.assertEqual(parsed["map_id"], "building")
        self.assertEqual(parsed["content_epoch"], 7)
        self.assertEqual(parsed["rooms"][0]["name"], "office")
        self.assertEqual(parsed["hierarchy_edges"][0]["child_id"], 1)

# ===========================================================================
# 3. Dynamic @skill discovery via on_system_modules
# ===========================================================================


class TestMCPSkillDiscovery(unittest.TestCase):
    def test_builtin_skills_registered_with_self(self):
        """MCPServerModule's own @skills appear after on_system_modules."""
        m = _make_mcp()
        m.on_system_modules({"MCPServerModule": m})
        self.assertGreaterEqual(len(m._tool_list), 12)
        names = {t["name"] for t in m._tool_list}
        for expected in (
            "emergency_stop",
            "get_health",
            "get_config",
            "get_robot_position",
            "get_scene_graph",
            "detect_objects",
            "query_memory",
            "list_tagged_locations",
            "tag_location",
            "navigate_to_object",
            "send_instruction",
            "set_mode",
        ):
            with self.subTest(tool=expected):
                self.assertIn(expected, names)

    def test_external_module_skills_registered(self):
        m = _make_mcp()
        nav = _fake_nav_skills()
        m.on_system_modules({"nav.skills": nav})
        self.assertIn("navigate_to", m._tool_registry)
        self.assertIn("get_navigation_status", m._tool_registry)

    def test_tool_list_contains_descriptor_fields(self):
        m = _make_mcp()
        m.on_system_modules({"MCPServerModule": m})
        for tool in m._tool_list:
            with self.subTest(tool=tool["name"]):
                self.assertIn("name", tool)
                self.assertIn("description", tool)
                self.assertIn("inputSchema", tool)

    def test_empty_modules_gives_empty_registry(self):
        m = _make_mcp()
        m.on_system_modules({})
        self.assertEqual(m._tool_registry, {})
        self.assertEqual(m._tool_list, [])

    def test_navigation_skills_are_discovered(self):
        m = _make_mcp()
        nav = _fake_nav_skills()

        m.on_system_modules({"MCPServerModule": m, "nav.skills": nav})
        fn = m._tool_registry.get("navigate_to")
        self.assertIsNotNone(fn)
        # The bound method's __self__ should be the nav module, not the MCP module
        self.assertIs(fn.__self__, nav)

    def test_tool_list_no_duplicate_names(self):
        m = _make_mcp()
        nav = _fake_nav_skills()
        m.on_system_modules({"MCPServerModule": m, "nav.skills": nav})
        names = [t["name"] for t in m._tool_list]
        self.assertEqual(len(names), len(set(names)), "Duplicate tool names in _tool_list")

    def test_shared_discovery_preserves_metadata(self):
        m = _make_mcp()
        m.on_system_modules({"metadata": _MetadataSkills()})

        tool = next(item for item in m._tool_list if item["name"] == "inspect_task")
        self.assertEqual(
            tool["inputSchema"]["properties"]["task_id"]["description"],
            "Task identity returned by the start call.",
        )
        self.assertEqual(tool["outputSchema"]["type"], "object")
        self.assertTrue(tool["annotations"]["readOnlyHint"])

    def test_same_module_instance_alias_is_discovered_once(self):
        m = _make_mcp()
        skills = _MetadataSkills()
        m.on_system_modules({"metadata.primary": skills, "metadata.alias": skills})

        self.assertEqual(
            [item["name"] for item in m._tool_list].count("inspect_task"),
            1,
        )

    def test_distinct_skill_owners_are_not_silently_overwritten(self):
        m = _make_mcp()
        with self.assertRaisesRegex(ValueError, "duplicate skill 'inspect_task'"):
            m.on_system_modules({"metadata.a": _MetadataSkills(), "metadata.b": _MetadataSkills()})

    def test_gateway_fallback_yields_to_real_skill_owner(self):
        m = _make_mcp()
        owner = _InstructionOwner()
        m.on_system_modules({"MCPServerModule": m, "semantic.instructions": owner})

        self.assertIs(m._tool_registry["send_instruction"].__self__, owner)


class TestMCPStructuredToolResults(unittest.TestCase):
    def test_dict_result_includes_text_and_structured_content(self):
        response = _text(7, {"found": True, "label": "水瓶"})

        result = response["result"]
        self.assertEqual(result["structuredContent"], {"found": True, "label": "水瓶"})
        self.assertEqual(json.loads(result["content"][0]["text"]), result["structuredContent"])

    def test_declared_object_output_parses_json_string(self):
        response = _text(8, '{"found": true}', output_schema={"type": "object"})

        self.assertEqual(response["result"]["structuredContent"], {"found": True})

    def test_ordinary_json_text_is_not_claimed_as_structured(self):
        response = _text(9, '{"found": true}')

        self.assertNotIn("structuredContent", response["result"])

    def test_declared_object_output_rejects_non_object_json(self):
        with self.assertRaisesRegex(TypeError, "non-object JSON"):
            _text(10, '[1, 2]', output_schema={"type": "object"})


# ===========================================================================
# 4. Calling tools through _tool_registry (same path as HTTP handler)
# ===========================================================================


class TestMCPToolExecution(unittest.TestCase):
    def setUp(self):
        self.mod = _make_mcp()
        self.nav = _fake_nav_skills()
        self.mod.on_system_modules(
            {
                "MCPServerModule": self.mod,
                "nav.skills": self.nav,
            }
        )
        self.mod._on_odom(_odom(1.0, 2.0))
        self.mod._on_sg(_scene_graph())

    def _call(self, name: str, **kwargs):
        fn = self.mod._tool_registry[name]
        return json.loads(fn(**kwargs))

    def test_get_config_reports_robot_driver_limits(self):
        from runtime.config import RobotConfig, SpeedConfig

        cfg = RobotConfig(speed=SpeedConfig(max_linear=0.7, max_angular=0.9))
        with patch("runtime.config.get_config", return_value=cfg):
            result = self._call("get_config")
        self.assertEqual(result["speed"], {"max_linear": 0.7, "max_angular": 0.9})

    # -- stop --
    def test_stop_halts_motion(self):
        with patch("gateway.mcp_server.native_estop", return_value=True):
            result = self._call("emergency_stop")
        self.assertEqual(result["status"], "emergency_stopped")

    def test_emergency_stop_uses_native_latched_boundary_when_available(self):
        with patch("gateway.mcp_server.native_estop", return_value=True) as send:
            result = json.loads(self.mod.emergency_stop())

        self.assertEqual(result["status"], "emergency_stopped")
        self.assertEqual(result["control_boundary"], "native_estop")
        send.assert_called_once_with(self.mod, "mcp_emergency_stop")

    # -- navigate_to (from Navigation) --
    def test_navigate_to_dispatches_to_nav(self):
        result = self._call("navigate_to", x=5.0, y=3.0)
        self.assertEqual(result["status"], "navigating")
        self.assertEqual(self.nav.goals, [(5.0, 3.0, 0.0)])

    def test_navigate_to_with_yaw(self):
        result = self._call("navigate_to", x=1.0, y=2.0, yaw=1.57)
        self.assertAlmostEqual(result["yaw"], 1.57)
        self.assertEqual(self.nav.goals[-1], (1.0, 2.0, 1.57))

    # -- navigate_to_object --
    def test_navigate_to_object_publishes_instruction(self):
        sent = []
        self.mod.instruction._add_callback(sent.append)
        result = self._call("navigate_to_object", instruction="the red chair")
        self.assertEqual(result["status"], "submitted")
        self.assertFalse(result["execution_confirmed"])
        self.assertIn("the red chair", sent)

    # -- send_instruction --
    def test_send_instruction_publishes(self):
        sent = []
        self.mod.instruction._add_callback(sent.append)
        result = self._call("send_instruction", text="go to the lab")
        self.assertEqual(result["status"], "submitted")
        self.assertFalse(result["execution_confirmed"])
        self.assertEqual(sent, ["go to the lab"])

    # -- get_robot_position --
    def test_get_robot_position_returns_coords(self):
        result = self._call("get_robot_position")
        self.assertAlmostEqual(result["x"], 1.0)
        self.assertAlmostEqual(result["y"], 2.0)

    def test_get_robot_position_no_odom(self):
        m = _make_mcp()
        m.on_system_modules({"MCPServerModule": m})
        result = json.loads(m._tool_registry["get_robot_position"]())
        self.assertIn("error", result)

    # -- get_scene_graph --
    def test_get_scene_graph_returns_valid_json(self):
        result = self._call("get_scene_graph")
        self.assertIn("objects", result)
        self.assertEqual(len(result["objects"]), 2)

    # -- detect_objects --
    def test_detect_objects_filters_by_label(self):
        result = self._call("detect_objects", query="chair")
        self.assertEqual(result["count"], 1)
        self.assertEqual(result["matches"][0]["label"], "chair")

    def test_detect_objects_no_match(self):
        result = self._call("detect_objects", query="robot_dog")
        self.assertEqual(result["count"], 0)

    # -- set_mode --
    def test_set_mode_valid(self):
        result = self._call("set_mode", mode="autonomous")
        self.assertEqual(result["mode"], "autonomous")

    def test_set_mode_invalid(self):
        result = self._call("set_mode", mode="turbo")
        self.assertIn("error", result)

    def test_set_mode_estop_also_stops(self):
        with patch("gateway.mcp_server.native_estop", return_value=True):
            result = self._call("set_mode", mode="estop")
        self.assertEqual(result["stage"], "native_command_ack")

    # -- get_health --
    def test_get_health_no_system_handle(self):
        result = self._call("get_health")
        self.assertIn("error", result)

    def test_get_health_with_mock_handle(self):
        import types

        handle = types.SimpleNamespace(
            health=lambda: {"modules": 3, "ok": True},
            modules={"MCPServerModule": self.mod},
        )
        self.mod._system_handle = handle
        result = self._call("get_health")
        self.assertTrue(result.get("ok") or "modules" in result)

    def test_get_health_prefers_runtime_status_provider(self):
        import types

        provider = types.SimpleNamespace(
            health=lambda: {"ok": True, "phase": "ready", "source": "provider"},
            modules={"provider.module": types.SimpleNamespace(running=True)},
        )
        handle = types.SimpleNamespace(
            health=lambda: {"ok": False, "source": "legacy"},
            modules={},
        )
        self.mod.set_system_handle(handle)
        self.mod.set_runtime_status_provider(provider)

        result = self._call("get_health")

        self.assertTrue(result["ok"])
        self.assertEqual(result["source"], "provider")

    def test_list_modules_uses_runtime_status_provider_without_system_handle(self):
        import types

        module = types.SimpleNamespace(
            layer=6,
            running=True,
            ports_in={"odometry": object()},
            ports_out={"goal_pose": object()},
        )
        provider = types.SimpleNamespace(
            health=lambda: {"ok": True},
            modules={"provider.module": module},
        )
        self.mod.set_runtime_status_provider(provider)

        result = self._call("list_modules")

        self.assertIn("provider.module", result["modules"])
        self.assertEqual(result["modules"]["provider.module"]["layer"], 6)
        self.assertTrue(result["modules"]["provider.module"]["running"])
        self.assertEqual(result["modules"]["provider.module"]["ports_in"], ["odometry"])
        self.assertEqual(result["modules"]["provider.module"]["ports_out"], ["goal_pose"])

    # -- get_navigation_status (from Navigation) --
    def test_get_navigation_status_from_nav(self):
        result = self._call("get_navigation_status")
        self.assertIn("state", result)

    # -- unknown tool not in registry --
    def test_unknown_tool_absent_from_registry(self):
        self.assertIsNone(self.mod._tool_registry.get("nonexistent_xyz"))


# ===========================================================================
# 5. health()
# ===========================================================================


class TestMCPHealth(unittest.TestCase):
    def test_health_contains_mcp_section(self):
        m = _make_mcp()
        m.on_system_modules({"MCPServerModule": m})
        h = m.health()
        self.assertIn("mcp", h)
        self.assertIsInstance(h["mcp"]["tools"], int)
        self.assertGreaterEqual(h["mcp"]["tools"], 12)

    def test_health_tools_count_matches_tool_list(self):
        m = _make_mcp()
        m.on_system_modules({"MCPServerModule": m})
        self.assertEqual(m.health()["mcp"]["tools"], len(m._tool_list))


class TestMCPCapabilities(unittest.TestCase):
    def test_capabilities_use_gateway_navigation_status(self):
        status = _navigation_status()
        m = _make_mcp()
        m.on_system_modules({"MCPServerModule": m, "GatewayModule": SimpleNamespace()})

        with patch("gateway.mcp_server.build_navigation_status", return_value=status) as build:
            payload = m._capabilities_payload()

        build.assert_called_once_with(m._all_modules["GatewayModule"])
        self.assertIs(payload["navigation_status"], status)
        self.assertNotIn("navigation_state", payload)
        self.assertFalse(payload["motion_tools_blocked"])

    def test_capabilities_fail_closed_without_gateway(self):
        m = _make_mcp()
        m.on_system_modules({"MCPServerModule": m})

        payload = m._capabilities_payload()

        self.assertIsNone(payload["navigation_status"])
        self.assertTrue(payload["motion_tools_blocked"])

    def test_capabilities_fail_closed_for_unknown_or_missing_axis_state(self):
        fields = (
            ("task", "state"),
            ("goal_admission", "state"),
            ("control", "authority"),
            ("motion", "permission"),
            ("motion", "observation"),
            ("motion", "stop_confirmation"),
        )
        m = _make_mcp()
        m.on_system_modules({"MCPServerModule": m, "GatewayModule": SimpleNamespace()})

        for axis, field in fields:
            for value in ("UNKNOWN", None):
                with self.subTest(axis=axis, field=field, value=value):
                    status = _navigation_status()
                    if value is None:
                        status[axis].pop(field)
                    else:
                        status[axis][field] = value
                    with patch("gateway.mcp_server.build_navigation_status", return_value=status):
                        payload = m._capabilities_payload()
                    self.assertTrue(payload["motion_tools_blocked"])


if __name__ == "__main__":
    unittest.main(verbosity=2)

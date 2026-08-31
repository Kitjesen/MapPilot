"""Gateway MCP server tests."""

from __future__ import annotations

import json
import os
import sys
import unittest
from unittest.mock import patch

# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
_here = os.path.dirname(os.path.abspath(__file__))
_repo = os.path.abspath(os.path.join(_here, "..", "..", "..", ".."))
_src = os.path.join(_repo, "src")
for _p in [_repo, _src]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

from gateway.mcp_server import MCPServerModule
from runtime.msgs.geometry import Pose, Vector3
from runtime.msgs.nav import NavigationState, Odometry
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


# ===========================================================================
# 1. Port declarations
# ===========================================================================


class TestMCPPorts(unittest.TestCase):
    def setUp(self):
        self.mod = MCPServerModule()

    def test_layer(self):
        self.assertEqual(MCPServerModule._layer, 6)

    def test_in_ports(self):
        for name in ("odometry", "scene_graph", "navigation_state", "navigation_goal_status"):
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

    def test_navigation_state_cached(self):
        self.mod._on_navigation_state(
            NavigationState(
                boot_id="navd-test",
                sequence=1,
                active_task_id="task-1",
                active_request_id="request-1",
            )
        )
        self.assertEqual(self.mod._navigation_state["active_task_id"], "task-1")


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


if __name__ == "__main__":
    unittest.main(verbosity=2)

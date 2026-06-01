"""MCP JSON-RPC 2.0 client for the LingTu MCP server (port 8090).

Wraps each ``@skill`` tool as a typed Python method.

Usage::

    from lingtu_sdk import LingTuClient
    from lingtu_sdk.mcp import LingTuMCP

    robot = LingTuClient("192.168.66.190")
    mcp = LingTuMCP(robot)
    mcp.call("navigate_to", {"x": 10.0, "y": 5.0})
"""

from __future__ import annotations

import json
import logging
from typing import Any

import httpx

from lingtu_sdk import LingTuClient
from lingtu_sdk.config import LingTuConfig

log = logging.getLogger(__name__)

_MCP_PORT = 8090


class LingTuMCP:
    """MCP JSON-RPC 2.0 client for AI agent control of the LingTu robot.

    Connects to the MCP server at ``http://{host}:8090/mcp``.

    Args:
        client: A connected ``LingTuClient`` (host is extracted from it).
        host: Explicit host override (takes precedence if both are given).
        port: MCP server port (default 8090).
        timeout: Request timeout in seconds.
    """

    def __init__(
        self,
        client: LingTuClient | None = None,
        *,
        host: str | None = None,
        port: int = _MCP_PORT,
        timeout: float = 10.0,
    ) -> None:
        if host:
            self._host = host
        elif client is not None:
            # Extract host from client.base_url (format: "http://host:port")
            self._host = client.base_url.split("://")[1].rsplit(":", 1)[0]
        else:
            self._host = "127.0.0.1"
        self._base = f"http://{self._host}:{port}"
        self._timeout = timeout
        self._req_id = 0

    # ------------------------------------------------------------------
    # Core JSON-RPC 2.0
    # ------------------------------------------------------------------

    def call(self, tool: str, params: dict[str, Any] | None = None) -> Any:
        """Call an MCP tool by name.

        Args:
            tool: Tool name matching a ``@skill`` method on the robot.
            params: Keyword arguments for the tool.

        Returns:
            The parsed result payload (``result.content[0].text`` decoded from JSON
            where possible, or the raw dict).
        """
        self._req_id += 1
        payload = {
            "jsonrpc": "2.0",
            "id": self._req_id,
            "method": "tools/call",
            "params": {"name": tool, "arguments": params or {}},
        }
        resp = httpx.post(
            f"{self._base}/mcp",
            json=payload,
            timeout=self._timeout,
        )
        resp.raise_for_status()
        body: dict[str, Any] = resp.json()

        if "error" in body:
            err = body["error"]
            raise MCPError(err.get("code", -1), err.get("message", "Unknown error"))

        result = body.get("result", {})
        content = result.get("content") or []
        for item in content:
            text = item.get("text", "")
            if text:
                try:
                    return json.loads(text)
                except (json.JSONDecodeError, TypeError):
                    return text
        return result

    def list_tools(self) -> list[dict[str, Any]]:
        """Return the full MCP tool list from the robot.

        Each entry has ``name``, ``description``, and ``inputSchema``.
        """
        self._req_id += 1
        payload = {
            "jsonrpc": "2.0",
            "id": self._req_id,
            "method": "tools/list",
        }
        resp = httpx.post(f"{self._base}/mcp", json=payload, timeout=self._timeout)
        resp.raise_for_status()
        body: dict[str, Any] = resp.json()
        result = body.get("result", {})
        return result.get("tools", [])

    def health(self) -> dict[str, Any]:
        """MCP server health — tool count, port, handle status."""
        resp = httpx.get(f"{self._base}/health", timeout=self._timeout)
        resp.raise_for_status()
        return resp.json()

    # ------------------------------------------------------------------
    # Convenience — Navigation
    # ------------------------------------------------------------------

    def navigate_to(self, x: float, y: float, yaw: float = 0.0) -> Any:
        """Navigate to absolute map coordinates.

        Calls the ``navigate_to`` tool (from NavigationModule).
        """
        return self.call("navigate_to", {"x": x, "y": y, "yaw": yaw})

    def navigate_to_object(self, instruction: str) -> Any:
        """Navigate to a described object or place using semantic understanding.

        Calls the ``navigate_to_object`` tool (from MCPServerModule).
        """
        return self.call("navigate_to_object", {"instruction": instruction})

    def send_instruction(self, text: str) -> Any:
        """Send a natural-language instruction to the semantic planner.

        Calls the ``send_instruction`` tool (from SemanticPlannerModule).
        """
        return self.call("send_instruction", {"text": text})

    def stop(self) -> Any:
        """Emergency stop — immediately halt all motion.

        Calls the ``emergency_stop`` tool.
        """
        return self.call("emergency_stop")

    def stop_navigation(self) -> Any:
        """Stop the current navigation mission.

        Calls the ``stop_navigation`` tool (from NavigationModule).
        """
        return self.call("stop_navigation")

    def cancel_mission(self, reason: str = "user_cancel") -> Any:
        """Cancel the current navigation mission gracefully.

        Calls the ``cancel_mission`` tool.
        """
        return self.call("cancel_mission", {"reason": reason})

    def get_navigation_status(self) -> Any:
        """Get current navigation state, position, and mission progress.

        Calls the ``get_navigation_status`` tool.
        """
        return self.call("get_navigation_status")

    def set_mode(self, mode: str) -> Any:
        """Set robot operating mode: ``manual``, ``autonomous``, or ``estop``.

        Calls the ``set_mode`` tool.
        """
        return self.call("set_mode", {"mode": mode})

    # ------------------------------------------------------------------
    # Convenience — Perception
    # ------------------------------------------------------------------

    def find_object(self, target: str) -> Any:
        """Trigger visual find mode for a target object.

        Calls the ``find_object`` tool (from VisualServoModule).
        """
        return self.call("find_object", {"target": target})

    def detect_objects(self, query: str) -> Any:
        """Search the current scene for objects matching *query*.

        Calls the ``detect_objects`` tool.
        """
        return self.call("detect_objects", {"query": query})

    def get_scene_graph(self) -> Any:
        """Return the current scene graph.

        Calls the ``get_scene_graph`` tool.
        """
        return self.call("get_scene_graph")

    def get_robot_position(self) -> Any:
        """Return the robot's current position in the map frame.

        Calls the ``get_robot_position`` tool.
        """
        return self.call("get_robot_position")

    def follow_person(self, description: str) -> Any:
        """Trigger person following mode.

        Calls the ``follow_person`` tool (from VisualServoModule).
        """
        return self.call("follow_person", {"description": description})

    def stop_servo(self) -> Any:
        """Stop all visual tracking.

        Calls the ``stop_servo`` tool.
        """
        return self.call("stop_servo")

    def get_servo_status(self) -> Any:
        """Return current visual servo state.

        Calls the ``get_servo_status`` tool.
        """
        return self.call("get_servo_status")

    # ------------------------------------------------------------------
    # Convenience — Memory & Locations
    # ------------------------------------------------------------------

    def query_memory(self, text: str) -> Any:
        """Search episodic, spatial, and vector memory for past observations.

        Calls the ``query_memory`` tool.
        """
        return self.call("query_memory", {"query": text})

    def tag_location(self, name: str) -> Any:
        """Save the robot's current position under *name*.

        Calls the ``tag_location`` tool.
        """
        return self.call("tag_location", {"name": name})

    def list_tagged_locations(self) -> Any:
        """List all named tagged locations.

        Calls the ``list_tagged_locations`` tool.
        """
        return self.call("list_tagged_locations")

    # ------------------------------------------------------------------
    # Convenience — Maps
    # ------------------------------------------------------------------

    def list_maps(self) -> Any:
        """List saved maps and the active one.

        Calls the ``list_maps`` tool (from MapManagerModule).
        """
        return self.call("list_maps")

    def save_map(self, name: str, slam_profile: str | None = None) -> Any:
        """Save the current SLAM map.

        Calls the ``save_map`` tool (from MapManagerModule).
        """
        params: dict[str, Any] = {"name": name}
        if slam_profile:
            params["slam_profile"] = slam_profile
        return self.call("save_map", params)

    def use_map(self, name: str) -> Any:
        """Activate a saved map for navigation.

        Calls the ``use_map`` tool (from MapManagerModule).
        """
        return self.call("use_map", {"name": name})

    def build_tomogram(self, name: str) -> Any:
        """Build tomogram from map PCD.

        Calls the ``build_tomogram`` tool (from MapManagerModule).
        """
        return self.call("build_tomogram", {"name": name})

    # ------------------------------------------------------------------
    # Convenience — Exploration
    # ------------------------------------------------------------------

    def begin_exploration(self) -> Any:
        """Start autonomous frontier exploration.

        Calls the ``begin_exploration`` tool (from FrontierExplorerModule).
        """
        return self.call("begin_exploration")

    def end_exploration(self) -> Any:
        """Stop autonomous frontier exploration.

        Calls the ``end_exploration`` tool.
        """
        return self.call("end_exploration")

    def get_frontiers(self) -> Any:
        """Return the most recently computed frontier clusters.

        Calls the ``get_frontiers`` tool.
        """
        return self.call("get_frontiers")

    # ------------------------------------------------------------------
    # Convenience — System
    # ------------------------------------------------------------------

    def get_health(self) -> Any:
        """Return full system health: modules, connections, message counts.

        Calls the ``get_health`` tool.
        """
        return self.call("get_health")

    def list_modules(self) -> Any:
        """List all deployed modules with layer, port counts, and running state.

        Calls the ``list_modules`` tool.
        """
        return self.call("list_modules")

    def get_config(self) -> Any:
        """Return robot configuration: speed limits, geometry, safety thresholds.

        Calls the ``get_config`` tool.
        """
        return self.call("get_config")

    def decompose_task(self, instruction: str) -> Any:
        """Decompose a complex instruction into ordered sub-goals.

        Calls the ``decompose_task`` tool (from SemanticPlannerModule).
        """
        return self.call("decompose_task", {"instruction": instruction})

    def get_safety_status(self) -> Any:
        """Get current safety state.

        Calls the ``get_safety_status`` tool (from SafetyRingModule).
        """
        return self.call("get_safety_status")

    def get_planner_status(self) -> Any:
        """Return current semantic planner state and counters.

        Calls the ``get_planner_status`` tool (from SemanticPlannerModule).
        """
        return self.call("get_planner_status")


class MCPError(Exception):
    """Raised when an MCP tool call returns a JSON-RPC error."""

    def __init__(self, code: int, message: str) -> None:
        self.code = code
        self.message = message
        super().__init__(f"[MCP {code}] {message}")

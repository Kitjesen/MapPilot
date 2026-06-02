"""MCP JSON-RPC 2.0 client for the LingTu MCP server (port 8090).

Wraps every ``@skill`` tool as a typed Python convenience method.

Usage::

    from lingtu_sdk import LingTuClient
    from lingtu_sdk.mcp import LingTuMCP

    robot = LingTuClient("192.168.66.190")
    mcp = LingTuMCP(robot)

    # Typed convenience methods
    mcp.navigate_to(10.0, 5.0)
    print(mcp.get_scene_graph())

    # Raw call (any tool name)
    mcp.call("any_tool_name", {"param": "value"})
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

    Usage::

        mcp = LingTuMCP(LingTuClient("192.168.66.190"))
        mcp.navigate_to(10.0, 5.0)
        mcp.follow_person("person in red shirt")
        mcp.get_navigation_status()
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
            The parsed result payload (``result.content[0].text`` decoded
            from JSON where possible, or the raw dict).

        Example::

            result = mcp.call("navigate_to", {"x": 10.0, "y": 5.0})
            result = mcp.call("get_health")
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

        Example::

            for t in mcp.list_tools():
                print(t["name"], t["description"])
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
        """MCP server health -- tool count, port, handle status.

        Example::

            h = mcp.health()
            print(h.get("tool_count", 0))
        """
        resp = httpx.get(f"{self._base}/health", timeout=self._timeout)
        resp.raise_for_status()
        return resp.json()

    # ------------------------------------------------------------------
    # Convenience -- Navigation
    # ------------------------------------------------------------------

    def navigate_to(self, x: float, y: float, yaw: float = 0.0) -> Any:
        """Navigate to absolute map coordinates.

        Example::

            mcp.navigate_to(10.0, 5.0)
        """
        return self.call("navigate_to", {"x": x, "y": y, "yaw": yaw})

    def navigate_to_object(self, instruction: str) -> Any:
        """Navigate to a described object or place using semantic understanding.

        Example::

            mcp.navigate_to_object("go to the charger station")
        """
        return self.call("navigate_to_object", {"instruction": instruction})

    def send_instruction(self, text: str) -> Any:
        """Send a natural-language instruction to the semantic planner.

        Example::

            mcp.send_instruction("前往会议室")
        """
        return self.call("send_instruction", {"text": text})

    def stop(self) -> Any:
        """Emergency stop -- immediately halt all motion.

        Example::

            mcp.stop()
        """
        return self.call("emergency_stop")

    def stop_navigation(self) -> Any:
        """Stop the current navigation mission.

        Example::

            mcp.stop_navigation()
        """
        return self.call("stop_navigation")

    def cancel_mission(self, reason: str = "user_cancel") -> Any:
        """Cancel the current navigation mission gracefully.

        Args:
            reason: Cancel reason string (default ``"user_cancel"``).
        """
        return self.call("cancel_mission", {"reason": reason})

    def get_navigation_status(self) -> Any:
        """Get current navigation state, position, and mission progress.

        Example::

            ns = mcp.get_navigation_status()
        """
        return self.call("get_navigation_status")

    def set_mode(self, mode: str) -> Any:
        """Set robot operating mode: ``manual``, ``autonomous``, or ``estop``.

        Example::

            mcp.set_mode("autonomous")
        """
        return self.call("set_mode", {"mode": mode})

    def start_patrol(self, waypoints: list[dict[str, Any]]) -> Any:
        """Start patrol mode -- navigate waypoints sequentially.

        Args:
            waypoints: List of waypoint dicts, each with ``x`` and ``y``,
                and optionally ``yaw`` / ``z`` / ``tolerance``.

        Example::

            mcp.start_patrol([
                {"x": 0.0, "y": 0.0},
                {"x": 5.0, "y": 3.0, "yaw": 1.57},
                {"x": 10.0, "y": 5.0, "tolerance": 0.5},
            ])
        """
        return self.call("start_patrol", {"waypoints_json": json.dumps(waypoints)})

    # ------------------------------------------------------------------
    # Convenience -- Perception
    # ------------------------------------------------------------------

    def find_object(self, target: str) -> Any:
        """Trigger visual find mode for a target object.

        Example::

            mcp.find_object("fire extinguisher")
        """
        return self.call("find_object", {"target": target})

    def detect_objects(self, query: str) -> Any:
        """Search the current scene for objects matching *query*.

        Example::

            mcp.detect_objects("person")
        """
        return self.call("detect_objects", {"query": query})

    def get_scene_graph(self) -> Any:
        """Return the current scene graph.

        Example::

            sg = mcp.get_scene_graph()
        """
        return self.call("get_scene_graph")

    def get_robot_position(self) -> Any:
        """Return the robot's current position in the map frame.

        Example::

            pos = mcp.get_robot_position()
        """
        return self.call("get_robot_position")

    def follow_person(self, description: str) -> Any:
        """Trigger person following mode.

        Example::

            mcp.follow_person("person in blue jacket")
        """
        return self.call("follow_person", {"description": description})

    def stop_servo(self) -> Any:
        """Stop all visual tracking.

        Example::

            mcp.stop_servo()
        """
        return self.call("stop_servo")

    def get_servo_status(self) -> Any:
        """Return current visual servo state.

        Example::

            st = mcp.get_servo_status()
        """
        return self.call("get_servo_status")

    def tune_bbox_gains(self, duration: float = 10.0) -> Any:
        """Run Ziegler-Nichols relay-based PD gain auto-tuning for BBoxNavigator.

        Args:
            duration: Tuning duration in seconds (default 10).

        Example::

            mcp.tune_bbox_gains(duration=15.0)
        """
        return self.call("tune_bbox_gains", {"duration": duration})

    # ------------------------------------------------------------------
    # Convenience -- Memory & Locations
    # ------------------------------------------------------------------

    def query_memory(self, text: str) -> Any:
        """Search episodic, spatial, and vector memory for past observations.

        Example::

            mcp.query_memory("where did I see the backpack")
        """
        return self.call("query_memory", {"query": text})

    def tag_location(self, name: str) -> Any:
        """Save the robot's current position under *name*.

        Example::

            mcp.tag_location("实验室入口")
        """
        return self.call("tag_location", {"name": name})

    def list_tagged_locations(self) -> Any:
        """List all named tagged locations.

        Example::

            locs = mcp.list_tagged_locations()
        """
        return self.call("list_tagged_locations")

    def list_tags(self) -> Any:
        """Alias for :meth:`list_tagged_locations`.

        Calls ``list_tags`` MCP tool (from TaggedLocationsModule).
        """
        return self.call("list_tags")

    def go_to_tag(self, name: str) -> Any:
        """Navigate to a tagged location by name.

        Args:
            name: Name of the tagged location.

        Example::

            mcp.go_to_tag("充电站")
        """
        return self.call("go_to_tag", {"name": name})

    def get_recent_observations(self, count: int = 10) -> Any:
        """Return the most recent episodic observations.

        Args:
            count: Number of observations to return (default 10).

        Example::

            obs = mcp.get_recent_observations(5)
        """
        return self.call("get_recent_observations", {"count": count})

    def query_location(self, text: str) -> Any:
        """Fuzzy search for a location by natural language description.

        Example::

            mcp.query_location("where did I park")
        """
        return self.call("query_location", {"text": text})

    def get_memory_stats(self) -> Any:
        """Return vector memory statistics.

        Example::

            stats = mcp.get_memory_stats()
        """
        return self.call("get_memory_stats")

    # ------------------------------------------------------------------
    # Convenience -- Mission Logger
    # ------------------------------------------------------------------

    def list_missions(self, count: int = 10) -> Any:
        """List the *count* most recent navigation missions.

        Args:
            count: Number of missions (default 10).

        Example::

            mcp.list_missions(5)
        """
        return self.call("list_missions", {"count": count})

    def get_mission_stats(self) -> Any:
        """Return aggregate statistics for all recorded navigation missions.

        Example::

            stats = mcp.get_mission_stats()
        """
        return self.call("get_mission_stats")

    # ------------------------------------------------------------------
    # Convenience -- Semantic Mapper (Rooms & Objects)
    # ------------------------------------------------------------------

    def get_room_summary(self) -> Any:
        """Return a text summary of all known rooms and their objects.

        Example::

            summary = mcp.get_room_summary()
        """
        return self.call("get_room_summary")

    def query_room_for_object(self, label: str) -> Any:
        """Return which room types are most likely to contain the given object.

        Args:
            label: Object label (e.g. ``"backpack"``).

        Example::

            mcp.query_room_for_object("backpack")
        """
        return self.call("query_room_for_object", {"label": label})

    def get_exploration_target(self, instruction: str) -> Any:
        """Return the best exploration target for a given instruction.

        Example::

            mcp.get_exploration_target("find the exit")
        """
        return self.call("get_exploration_target", {"instruction": instruction})

    def get_semantic_status(self) -> Any:
        """Return KG + TSG statistics.

        Example::

            st = mcp.get_semantic_status()
        """
        return self.call("get_semantic_status")

    # ------------------------------------------------------------------
    # Convenience -- Temporal Memory
    # ------------------------------------------------------------------

    def query_temporal(self, question: str) -> Any:
        """Answer a natural language question about temporal scene memory.

        Example::

            mcp.query_temporal("what objects were near the door at 2pm?")
        """
        return self.call("query_temporal", {"question": question})

    def get_entity_history(self, label: str) -> Any:
        """Return all sightings of a specific object label.

        Args:
            label: Object label.

        Example::

            mcp.get_entity_history("backpack")
        """
        return self.call("get_entity_history", {"label": label})

    # ------------------------------------------------------------------
    # Convenience -- Maps
    # ------------------------------------------------------------------

    def list_maps(self) -> Any:
        """List saved maps and the active one.

        Example::

            maps = mcp.list_maps()
        """
        return self.call("list_maps")

    def save_map(self, name: str, slam_profile: str | None = None) -> Any:
        """Save the current SLAM map as *name* and build all artifacts.

        Args:
            name: Map name.
            slam_profile: Optional SLAM profile override.
        """
        params: dict[str, Any] = {"name": name}
        if slam_profile:
            params["slam_profile"] = slam_profile
        return self.call("save_map", params)

    def use_map(self, name: str) -> Any:
        """Activate a saved map for navigation.

        Example::

            mcp.use_map("factory_01")
        """
        return self.call("use_map", {"name": name})

    def build_tomogram(self, name: str) -> Any:
        """Build tomogram from map PCD.

        Example::

            mcp.build_tomogram("factory_01")
        """
        return self.call("build_tomogram", {"name": name})

    # ------------------------------------------------------------------
    # Convenience -- Exploration
    # ------------------------------------------------------------------

    def begin_exploration(self) -> Any:
        """Start autonomous frontier exploration.

        Example::

            mcp.begin_exploration()
        """
        return self.call("begin_exploration")

    def end_exploration(self) -> Any:
        """Stop autonomous frontier exploration.

        Example::

            mcp.end_exploration()
        """
        return self.call("end_exploration")

    def get_frontiers(self) -> Any:
        """Return the most recently computed frontier clusters.

        Example::

            frontiers = mcp.get_frontiers()
        """
        return self.call("get_frontiers")

    def clear_frontier_blocks(self) -> Any:
        """Clear navigation-failed frontier goals from block list.

        Example::

            mcp.clear_frontier_blocks()
        """
        return self.call("clear_frontier_blocks")

    # ------------------------------------------------------------------
    # Convenience -- TARE Exploration
    # ------------------------------------------------------------------

    def start_tare_exploration(self) -> Any:
        """Start TARE exploration (CMU hierarchical exploration).

        Example::

            mcp.start_tare_exploration()
        """
        return self.call("start_tare_exploration")

    def stop_tare_exploration(self) -> Any:
        """Stop TARE exploration.

        Example::

            mcp.stop_tare_exploration()
        """
        return self.call("stop_tare_exploration")

    def get_tare_status(self) -> Any:
        """Return TARE exploration state.

        Example::

            st = mcp.get_tare_status()
        """
        return self.call("get_tare_status")

    # ------------------------------------------------------------------
    # Convenience -- Exploration Supervisor
    # ------------------------------------------------------------------

    def get_exploration_supervisor(self) -> Any:
        """Return the latest exploration supervisor state.

        Example::

            st = mcp.get_exploration_supervisor()
        """
        return self.call("get_exploration_supervisor")

    def clear_exploration_fallback(self) -> Any:
        """Reset the fallback_requested flag.

        Example::

            mcp.clear_exploration_fallback()
        """
        return self.call("clear_exploration_fallback")

    # ------------------------------------------------------------------
    # Convenience -- Teleop
    # ------------------------------------------------------------------

    def get_teleop_status(self) -> Any:
        """Return current teleop status.

        Example::

            st = mcp.get_teleop_status()
        """
        return self.call("get_teleop_status")

    def force_release(self) -> Any:
        """Force-release teleop control and resume autonomy.

        Example::

            mcp.force_release()
        """
        return self.call("force_release")

    # ------------------------------------------------------------------
    # Convenience -- Rerun Visualisation
    # ------------------------------------------------------------------

    def start_rerun(self) -> Any:
        """Start Rerun web viewer (returns URL).

        Example::

            url = mcp.start_rerun()
        """
        return self.call("start_rerun")

    def stop_rerun(self) -> Any:
        """Stop Rerun visualization and release resources.

        Example::

            mcp.stop_rerun()
        """
        return self.call("stop_rerun")

    def rerun_status(self) -> Any:
        """Return Rerun status.

        Example::

            st = mcp.rerun_status()
        """
        return self.call("rerun_status")

    # ------------------------------------------------------------------
    # Convenience -- Voxel Grid
    # ------------------------------------------------------------------

    def get_voxel_stats(self) -> Any:
        """Return stats about the current voxel map.

        Example::

            st = mcp.get_voxel_stats()
        """
        return self.call("get_voxel_stats")

    def clear_voxels(self) -> Any:
        """Reset the entire voxel map.

        Example::

            mcp.clear_voxels()
        """
        return self.call("clear_voxels")

    def query_voxel(self, x: float, y: float, z: float) -> Any:
        """Check whether the voxel at (x, y, z) is occupied.

        Example::

            occ = mcp.query_voxel(1.0, 2.0, 0.5)
        """
        return self.call("query_voxel", {"x": x, "y": y, "z": z})

    # ------------------------------------------------------------------
    # Convenience -- Traversable Frontiers
    # ------------------------------------------------------------------

    def get_traversable_frontiers(self) -> Any:
        """Return ranked frontier candidates with traversability evidence.

        Example::

            f = mcp.get_traversable_frontiers()
        """
        return self.call("get_traversable_frontiers")

    def refresh_candidates(self) -> Any:
        """Compute and publish traversable frontier candidates without motion.

        Example::

            mcp.refresh_candidates()
        """
        return self.call("refresh_candidates")

    # ------------------------------------------------------------------
    # Convenience -- System
    # ------------------------------------------------------------------

    def get_health(self) -> Any:
        """Return full system health: modules, connections, message counts.

        Example::

            h = mcp.get_health()
        """
        return self.call("get_health")

    def list_modules(self) -> Any:
        """List all deployed modules with layer, port counts, and running state.

        Example::

            mods = mcp.list_modules()
        """
        return self.call("list_modules")

    def get_config(self) -> Any:
        """Return robot configuration: speed limits, geometry, safety thresholds.

        Example::

            cfg = mcp.get_config()
        """
        return self.call("get_config")

    def decompose_task(self, instruction: str) -> Any:
        """Decompose a complex instruction into ordered sub-goals.

        Example::

            mcp.decompose_task("inspect room A then room B")
        """
        return self.call("decompose_task", {"instruction": instruction})

    def get_safety_status(self) -> Any:
        """Get current safety state: level, cross-track error, etc.

        Example::

            st = mcp.get_safety_status()
        """
        return self.call("get_safety_status")

    def get_planner_status(self) -> Any:
        """Return current semantic planner state and counters.

        Example::

            st = mcp.get_planner_status()
        """
        return self.call("get_planner_status")

    def switch_backend(self, category: str, backend: str, config_json: str = "{}") -> Any:
        """Switch a low-risk runtime backend.

        Args:
            category: Backend category (e.g. ``"detector"``, ``"planner"``).
            backend: Backend name (e.g. ``"yoloe"``, ``"astar"``).
            config_json: JSON string of backend configuration.

        Example::

            mcp.switch_backend("planner", "pct")
        """
        return self.call("switch_backend", {
            "category": category,
            "backend": backend,
            "config_json": config_json,
        })

    # ------------------------------------------------------------------
    # Convenience -- NTRIP / GNSS
    # ------------------------------------------------------------------

    def get_ntrip_status(self) -> Any:
        """Return NTRIP caster connection state.

        Example::

            st = mcp.get_ntrip_status()
        """
        return self.call("get_ntrip_status")

    def get_gnss_fusion_status(self) -> Any:
        """Return GNSS-SLAM fusion health.

        Example::

            st = mcp.get_gnss_fusion_status()
        """
        return self.call("get_gnss_fusion_status")

    def relock_gnss_alignment(self) -> Any:
        """Force re-locking of the GNSS<->SLAM alignment offset.

        Example::

            mcp.relock_gnss_alignment()
        """
        return self.call("relock_gnss_alignment")

    def set_gnss_fusion(self, enabled: bool) -> Any:
        """Enable or disable GNSS-SLAM fusion.

        Args:
            enabled: True to enable, False to disable.

        Example::

            mcp.set_gnss_fusion(True)
        """
        return self.call("set_gnss_fusion", {"enabled": enabled})


class MCPError(Exception):
    """Raised when an MCP tool call returns a JSON-RPC error."""

    def __init__(self, code: int, message: str) -> None:
        self.code = code
        self.message = message
        super().__init__(f"[MCP {code}] {message}")

"""Synchronous HTTP client for the LingTu robot Gateway.

Provides typed convenience methods for every major Gateway REST endpoint.
Uses only the Python stdlib -- no third-party dependencies.

Usage::

    from lingtu_sdk import LingTuClient

    robot = LingTuClient("192.168.66.190", 5050)
    robot.go(10.0, 5.0, yaw=1.57)
    print(robot.position())
"""

from __future__ import annotations

import json
import time
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from typing import Any

# ruff: noqa: S310  -- stdlib HTTP client; URL opening is intentional

# ---------------------------------------------------------------------------
# Typed response models
# ---------------------------------------------------------------------------


@dataclass
class Position:
    """Robot position in the map frame."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0


@dataclass
class HealthStatus:
    """System health overview."""

    modules_ok: int = 0
    modules_fail: int = 0
    modules_total: int = 0
    slam_rate: float = 0.0
    mode: str = ""
    session: str = ""
    raw: dict[str, Any] = field(default_factory=dict)


@dataclass
class NavigationStatus:
    """Navigation mission state."""

    state: str = "idle"
    distance_to_goal: float = 0.0
    time_elapsed: float = 0.0
    goal: Position = field(default_factory=Position)
    raw: dict[str, Any] = field(default_factory=dict)


@dataclass
class MapInfo:
    """Summary of a single saved map."""

    name: str = ""
    has_pcd: bool = False
    is_active: bool = False
    size_bytes: int = 0
    raw: dict[str, Any] = field(default_factory=dict)


@dataclass
class MapList:
    """Response from the maps endpoint."""

    maps: list[MapInfo] = field(default_factory=list)
    active_map: str | None = None
    raw: dict[str, Any] = field(default_factory=dict)


@dataclass
class SessionInfo:
    """Current robot session state."""

    mode: str = "idle"
    active_map: str | None = None
    slam_profile: str | None = None
    raw: dict[str, Any] = field(default_factory=dict)


@dataclass
class CommandResult:
    """Result of a command (go, stop, set_mode, etc.)."""

    ok: bool = False
    message: str = ""
    raw: dict[str, Any] = field(default_factory=dict)


@dataclass
class RobotState:
    """Full robot state snapshot."""

    mode: str = ""
    odometry: Position = field(default_factory=Position)
    mission: NavigationStatus = field(default_factory=NavigationStatus)
    safety_level: str = ""
    raw: dict[str, Any] = field(default_factory=dict)


# ---------------------------------------------------------------------------
# Client
# ---------------------------------------------------------------------------


class LingTuClient:
    """Synchronous client for LingTu robot control and state inspection.

    Uses only the Python stdlib -- no third-party dependencies.

    Usage::

        robot = LingTuClient("192.168.66.190", 5050)

        # Navigate to coordinates
        robot.go(10.0, 5.0, yaw=1.57)

        # Navigate by semantic label
        robot.go_to("会议室")

        # Get full state
        s = robot.state()

        # Context manager (auto-close)
        with LingTuClient() as r:
            print(r.health())
    """

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 5050,
        api_key: str | None = None,
    ) -> None:
        """Connect to a LingTu robot Gateway.

        Args:
            host: Robot IP or hostname.
            port: Gateway HTTP port (default 5050).
            api_key: Optional shared secret sent as ``X-API-Key`` header.

        Example::

            robot = LingTuClient("192.168.66.190")
        """
        self.base_url = f"http://{host}:{port}"
        self._api_key = api_key

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------

    def __enter__(self) -> LingTuClient:
        return self

    def __exit__(self, *args: object) -> None:
        self.close()

    # ------------------------------------------------------------------
    # Navigation
    # ------------------------------------------------------------------

    def go(self, x: float, y: float, yaw: float = 0.0) -> CommandResult:
        """Navigate to map coordinates.

        Args:
            x: X coordinate in metres (map frame).
            y: Y coordinate in metres.
            yaw: Heading in radians (default 0).

        Returns:
            :class:`CommandResult` with status and goal info.

        Example::

            robot.go(10.0, 5.0)              # go to (10, 5)
            robot.go(5.0, 3.0, yaw=1.57)     # go to (5, 3), facing east
        """
        return self._command("/api/v1/goal", {"x": x, "y": y, "yaw": yaw})

    def go_to(self, target: str) -> CommandResult:
        """Navigate by semantic label or natural-language instruction.

        Args:
            target: Location name, object description, or instruction text.

        Returns:
            :class:`CommandResult`.

        Example::

            robot.go_to("会议室")           # go to meeting room
            robot.go_to("find the charger") # semantic instruction
        """
        return self._command("/api/v1/instruction", {"text": target})

    def stop(self) -> CommandResult:
        """Emergency stop -- immediately halt all motion.

        Example::

            robot.stop()
        """
        return self._command("/api/v1/stop")

    def cancel(self, reason: str = "client_cancel") -> CommandResult:
        """Gracefully cancel the current navigation mission.

        Args:
            reason: Cancel reason string (default ``"client_cancel"``).

        Example::

            robot.cancel()
            robot.cancel("obstacle detected")
        """
        return self._command("/api/v1/navigation/cancel", {"reason": reason})

    def navigate_click(self, x: float, y: float, yaw: float = 0.0) -> CommandResult:
        """Navigate to a map-viewer click point.

        Args:
            x: X coordinate in metres.
            y: Y coordinate in metres.
            yaw: Optional heading in radians.

        Example::

            robot.navigate_click(12.0, 8.0)
        """
        return self._command("/api/v1/navigate/click", {"x": x, "y": y, "yaw": yaw})

    def batch_go(self, waypoints: list[tuple[float, float, float]]) -> list[CommandResult]:
        """Navigate through a sequence of waypoints (blocking).

        Each waypoint is visited in order.  The method returns only after
        the final waypoint has been reached (or any one fails).

        Args:
            waypoints: List of ``(x, y, yaw)`` tuples.

        Returns:
            List of :class:`CommandResult`, one per waypoint.

        Example::

            robot.batch_go([
                (10.0, 5.0, 0.0),
                (15.0, 8.0, 1.57),
                (20.0, 10.0, 3.14),
            ])
        """
        results: list[CommandResult] = []
        for i, (x, y, yaw) in enumerate(waypoints):
            result = self.go(x, y, yaw)
            results.append(result)
            if not result.ok:
                break
            if i < len(waypoints) - 1:
                self.wait_until_arrived()
        return results

    def wait_until_arrived(
        self,
        timeout: float = 120.0,
        poll_interval: float = 0.5,
        distance_threshold: float = 0.3,
    ) -> NavigationStatus:
        """Block until the current navigation mission completes.

        Polls ``navigation_status()`` every *poll_interval* seconds.
        Raises :class:`TimeoutError` if the mission does not complete
        within *timeout* seconds.

        Args:
            timeout: Maximum wait time in seconds (default 120).
            poll_interval: Seconds between status checks (default 0.5).
            distance_threshold: Metres from goal considered "arrived"
                (default 0.3).

        Returns:
            Final :class:`NavigationStatus`.

        Raises:
            TimeoutError: If the mission does not finish in time.

        Example::

            robot.go(10.0, 5.0)
            status = robot.wait_until_arrived()
            print(f"Arrived! Distance: {status.distance_to_goal:.2f}m")
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            status = self.navigation_status()
            state = status.state
            if state in ("idle", "completed", "failed", "cancelled"):
                return status
            if status.distance_to_goal is not None and status.distance_to_goal <= distance_threshold:
                return status
            time.sleep(poll_interval)

        raise TimeoutError(
            f"Navigation did not complete within {timeout}s "
            f"(state={status.state}, dist={status.distance_to_goal:.2f}m)"
        )

    # ------------------------------------------------------------------
    # State
    # ------------------------------------------------------------------

    def state(self) -> RobotState:
        """Get full robot state: odometry, mission, safety, mode, etc.

        Example::

            s = robot.state()
            print(f"Mode: {s.mode}, Pos: ({s.odometry.x:.2f}, {s.odometry.y:.2f})")
        """
        raw = self._get("/api/v1/state")
        return self._parse_state(raw)

    def health(self) -> HealthStatus:
        """Get system health overview (modules, sensors, SLAM rate, etc.).

        Example::

            h = robot.health()
            print(f"Modules: {h.modules_ok}/{h.modules_total} ok")
        """
        raw = self._get("/api/v1/health")
        return HealthStatus(
            modules_ok=raw.get("modules_ok", 0),
            modules_fail=raw.get("modules_fail", 0),
            modules_total=raw.get("modules_ok", 0) + raw.get("modules_fail", 0),
            slam_rate=raw.get("slam_rate", 0.0),
            mode=raw.get("mode", ""),
            session=raw.get("session", ""),
            raw=raw,
        )

    def position(self) -> Position:
        """Get current position.

        This is a convenience wrapper around ``state()``.

        Example::

            p = robot.position()
            print(f"x={p.x:.2f}, y={p.y:.2f}, yaw={p.yaw:.2f}")
        """
        return self.state().odometry

    def session(self) -> SessionInfo:
        """Get current session state (mode, SLAM profile, active map, etc.).

        Example::

            s = robot.session()
            print(f"Mode: {s.mode}, Map: {s.active_map}")
        """
        raw = self._get("/api/v1/session")
        return SessionInfo(
            mode=raw.get("mode", "idle"),
            active_map=raw.get("active_map"),
            slam_profile=raw.get("slam_profile"),
            raw=raw,
        )

    def navigation_status(self) -> NavigationStatus:
        """Get current navigation mission status.

        Includes mission state, distance to goal, elapsed time, and target.

        Example::

            ns = robot.navigation_status()
            print(f"State: {ns.state}, Dist: {ns.distance_to_goal:.2f}m")
        """
        raw = self._get("/api/v1/navigation/status")
        goal_raw = raw.get("goal", {})
        return NavigationStatus(
            state=raw.get("state", "idle"),
            distance_to_goal=raw.get("distance_to_goal", 0.0),
            time_elapsed=raw.get("time_elapsed", 0.0),
            goal=Position(
                x=goal_raw.get("x", 0.0),
                y=goal_raw.get("y", 0.0),
                z=goal_raw.get("z", 0.0),
                yaw=goal_raw.get("yaw", 0.0),
            ),
            raw=raw,
        )

    def localization_status(self) -> dict[str, Any]:
        """Get localization health: alignment, residual, fix quality.

        Returns:
            Raw response dict.

        Example::

            loc = robot.localization_status()
            print(f"Fix: {loc.get('fix_quality', '?')}")
        """
        return self._get("/api/v1/localization/status")

    def path(self) -> dict[str, Any]:
        """Get the latest planned global path as a list of poses.

        Example::

            p = robot.path()
            for pt in p.get("path", []):
                print(pt.get("x"), pt.get("y"))
        """
        return self._get("/api/v1/path")

    # ------------------------------------------------------------------
    # Velocity control
    # ------------------------------------------------------------------

    def drive(self, vx: float = 0.0, vy: float = 0.0, wz: float = 0.0) -> CommandResult:
        """Send direct velocity command to the robot.

        Args:
            vx: Linear velocity forward (m/s).
            vy: Linear velocity sideways (m/s).
            wz: Angular velocity (rad/s).

        Example::

            robot.drive(vx=0.5, wz=0.3)  # drive forward + turn
        """
        return self._command("/api/v1/cmd_vel", {"vx": vx, "vy": vy, "wz": wz})

    # ------------------------------------------------------------------
    # Maps
    # ------------------------------------------------------------------

    def maps(self) -> MapList:
        """List saved maps on the robot.

        Example::

            ml = robot.maps()
            for m in ml.maps:
                print(f"  {m.name}  active={m.is_active}")
        """
        raw = self._get("/api/v1/slam/maps")
        return self._parse_map_list(raw)

    def save_map(self, name: str | None = None) -> CommandResult:
        """Save the current SLAM map.

        Args:
            name: Map name.  If omitted the server auto-generates one.

        Example::

            robot.save_map("factory_north_wing")
        """
        body: dict[str, Any] = {}
        if name:
            body["name"] = name
        return self._command("/api/v1/map/save", body)

    def use_map(self, name: str) -> CommandResult:
        """Activate a saved map for navigation.

        Args:
            name: Name of an existing saved map.

        Example::

            robot.use_map("factory_north_wing")
        """
        return self._command("/api/v1/map/activate", {"name": name})

    def rename_map(self, old_name: str, new_name: str) -> CommandResult:
        """Rename a saved map.

        Example::

            robot.rename_map("temp", "final_map")
        """
        return self._command("/api/v1/map/rename", {"old_name": old_name, "new_name": new_name})

    def restore_map(self, name: str) -> CommandResult:
        """Restore a map.pcd from DUFOMap pre-filter backup.

        Args:
            name: The saved map name to restore.

        Example::

            robot.restore_map("factory_north_wing")
        """
        return self._command("/api/v1/map/restore_predufo", {"name": name})

    def reset_map_cloud(self) -> CommandResult:
        """Clear the accumulated map cloud (visualisation only).

        SLAM ikd-tree is untouched.  Useful for clearing the viewer.

        Example::

            robot.reset_map_cloud()
        """
        return self._command("/api/v1/map_cloud/reset")

    def map_points(self) -> dict[str, Any]:
        """Get current live map point cloud as JSON.

        Example::

            pts = robot.map_points()
            print(f"Point count: {len(pts.get('points', []))}")
        """
        return self._get("/api/v1/map/points")

    def saved_map_points(self, name: str) -> dict[str, Any]:
        """Get a saved map point cloud as JSON.

        Args:
            name: Saved map name.

        Example::

            pts = robot.saved_map_points("factory_north_wing")
        """
        return self._get(f"/api/v1/maps/{name}/points")

    # ------------------------------------------------------------------
    # Mode
    # ------------------------------------------------------------------

    def set_mode(self, mode: str) -> CommandResult:
        """Set robot operating mode.

        Args:
            mode: One of ``"manual"``, ``"autonomous"``, or ``"estop"``.

        Example::

            robot.set_mode("autonomous")
        """
        return self._command("/api/v1/mode", {"mode": mode})

    # ------------------------------------------------------------------
    # Perception
    # ------------------------------------------------------------------

    def scene(self) -> dict[str, Any]:
        """Get the current scene graph (detected objects, relations, regions).

        Example::

            sg = robot.scene()
            for obj in sg.get("objects", []):
                print(obj.get("label"), obj.get("position"))
        """
        return self._get("/api/v1/scene_graph")

    def locations(self) -> dict[str, Any]:
        """List tagged navigation locations.

        Example::

            locs = robot.locations()
            for loc in locs.get("locations", []):
                print(f"  {loc.get('name')}  ({loc.get('x'):.1f}, {loc.get('y'):.1f})")
        """
        return self._get("/api/v1/locations")

    def tag_location(
        self,
        name: str,
        x: float | None = None,
        y: float | None = None,
        yaw: float | None = None,
        use_current_pose: bool = False,
    ) -> CommandResult:
        """Create or update a tagged navigation location.

        Args:
            name: Location label.
            x: X coordinate (required unless ``use_current_pose`` is true).
            y: Y coordinate (required unless ``use_current_pose`` is true).
            yaw: Optional heading.
            use_current_pose: If true, use the robot's current odometry pose.

        Example::

            robot.tag_location("充电站", use_current_pose=True)
            robot.tag_location("入口", x=10.0, y=5.0)
        """
        body: dict[str, Any] = {"name": name, "use_current_pose": use_current_pose}
        if x is not None:
            body["x"] = x
        if y is not None:
            body["y"] = y
        if yaw is not None:
            body["yaw"] = yaw
        return self._command("/api/v1/locations", body)

    def delete_location(self, name: str) -> CommandResult:
        """Delete a tagged navigation location.

        Args:
            name: Location label to remove.

        Example::

            robot.delete_location("充电站")
        """
        return self._command(f"/api/v1/locations/{name}")

    # ------------------------------------------------------------------
    # Camera
    # ------------------------------------------------------------------

    def camera_snapshot(self) -> bytes:
        """Fetch a JPEG camera snapshot from the robot.

        Returns:
            Raw JPEG bytes.

        Example::

            jpg = robot.camera_snapshot()
            with open("snapshot.jpg", "wb") as f:
                f.write(jpg)
        """
        url = self.base_url + "/api/v1/camera/snapshot"
        req = urllib.request.Request(url)
        if self._api_key:
            req.add_header("X-API-Key", self._api_key)
        with urllib.request.urlopen(req) as r:
            return r.read()

    # ------------------------------------------------------------------
    # SLAM
    # ------------------------------------------------------------------

    def slam_status(self) -> dict[str, Any]:
        """Get SLAM service status.

        Example::

            st = robot.slam_status()
            print(f"SLAM: {st.get('state', '?')}")
        """
        return self._get("/api/v1/slam/status")

    def slam_switch(self, profile: str) -> CommandResult:
        """Hot-switch the SLAM profile.

        Args:
            profile: One of ``"fastlio2"``, ``"pointlio"``, ``"localizer"``.

        Example::

            robot.slam_switch("localizer")
        """
        return self._command("/api/v1/slam/switch", {"profile": profile})

    def slam_relocalize(self, x: float | None = None, y: float | None = None, yaw: float | None = None) -> CommandResult:
        """Relocalize against a saved map with an optional pose guess.

        Args:
            x: Optional X guess.
            y: Optional Y guess.
            yaw: Optional heading guess.

        Example::

            robot.slam_relocalize()
            robot.slam_relocalize(x=10.0, y=5.0, yaw=0.0)
        """
        body: dict[str, Any] = {}
        if x is not None:
            body["x"] = x
        if y is not None:
            body["y"] = y
        if yaw is not None:
            body["yaw"] = yaw
        return self._command("/api/v1/slam/relocalize", body)

    def slam_auto_relocalize(self) -> CommandResult:
        """Global relocalize via 3D-BBS (no pose guess required).

        Example::

            robot.slam_auto_relocalize()
        """
        return self._command("/api/v1/slam/auto_relocalize")

    # ------------------------------------------------------------------
    # Exploration
    # ------------------------------------------------------------------

    def explore_start(self) -> CommandResult:
        """Start autonomous frontier exploration.

        Example::

            robot.explore_start()
        """
        return self._command("/api/v1/explore/start")

    def explore_stop(self) -> CommandResult:
        """Stop autonomous frontier exploration.

        Example::

            robot.explore_stop()
        """
        return self._command("/api/v1/explore/stop")

    def explore_status(self) -> dict[str, Any]:
        """Get exploration status.

        Example::

            st = robot.explore_status()
            print(f"Exploring: {st.get('active', False)}")
        """
        return self._get("/api/v1/explore/status")

    # ------------------------------------------------------------------
    # Rosbag recording
    # ------------------------------------------------------------------

    def bag_start(self, name: str | None = None) -> CommandResult:
        """Start rosbag recording.

        Args:
            name: Optional bag name.

        Example::

            robot.bag_start("test_run_01")
        """
        body: dict[str, Any] = {}
        if name:
            body["name"] = name
        return self._command("/api/v1/bag/start", body)

    def bag_stop(self) -> CommandResult:
        """Stop rosbag recording.

        Example::

            robot.bag_stop()
        """
        return self._command("/api/v1/bag/stop")

    def bag_status(self) -> dict[str, Any]:
        """Get rosbag recording status.

        Example::

            st = robot.bag_status()
            print(f"Recording: {st.get('recording', False)}")
        """
        return self._get("/api/v1/bag/status")

    # ------------------------------------------------------------------
    # Temporal memory
    # ------------------------------------------------------------------

    def memory_temporal(self) -> dict[str, Any]:
        """Query temporal entity observations.

        Example::

            mem = robot.memory_temporal()
            for obs in mem.get("observations", []):
                print(obs.get("label"), obs.get("timestamp"))
        """
        return self._get("/api/v1/memory/temporal")

    def memory_temporal_semantic(self, query: str) -> dict[str, Any]:
        """Semantic similarity search over temporal observations.

        Args:
            query: Natural language query.

        Example::

            mem = robot.memory_temporal_semantic("where was the backpack seen?")
        """
        return self._post("/api/v1/memory/temporal/semantic", {"query": query})

    # ------------------------------------------------------------------
    # Lease (control ownership)
    # ------------------------------------------------------------------

    def acquire_lease(self, client_id: str, ttl: float = 30.0) -> CommandResult:
        """Acquire the control lease.

        Only one client may hold the lease at a time.

        Args:
            client_id: Unique client identifier.
            ttl: Lease TTL in seconds (default 30).

        Example::

            robot.acquire_lease("my_app", ttl=60.0)
        """
        return self._command("/api/v1/lease", {
            "action": "acquire",
            "client_id": client_id,
            "ttl": ttl,
        })

    def release_lease(self, client_id: str) -> CommandResult:
        """Release the control lease.

        Args:
            client_id: Unique client identifier.

        Example::

            robot.release_lease("my_app")
        """
        return self._command("/api/v1/lease", {
            "action": "release",
            "client_id": client_id,
        })

    def renew_lease(self, client_id: str, ttl: float = 30.0) -> CommandResult:
        """Renew the control lease before it expires.

        Args:
            client_id: Unique client identifier.
            ttl: TTL in seconds (default 30).

        Example::

            robot.renew_lease("my_app")
        """
        return self._command("/api/v1/lease", {
            "action": "renew",
            "client_id": client_id,
            "ttl": ttl,
        })

    # ------------------------------------------------------------------
    # Session lifecycle
    # ------------------------------------------------------------------

    def start_session(
        self,
        mode: str,
        map_name: str | None = None,
        slam_profile: str | None = None,
    ) -> CommandResult:
        """Enter a robot session mode.

        Args:
            mode: ``"mapping"``, ``"navigating"``, or ``"exploring"``.
            map_name: Map to use (required for navigating).
            slam_profile: SLAM backend (``"fastlio2"``, ``"localizer"``, etc.).

        Example::

            robot.start_session("navigating", map_name="factory_01")
            robot.start_session("mapping", slam_profile="fastlio2")
        """
        body: dict[str, Any] = {"mode": mode}
        if map_name:
            body["map_name"] = map_name
        if slam_profile:
            body["slam_profile"] = slam_profile
        return self._command("/api/v1/session/start", body)

    def end_session(self) -> CommandResult:
        """End the current robot session.

        Example::

            robot.end_session()
        """
        return self._command("/api/v1/session/end")

    # ------------------------------------------------------------------
    # Driver swap
    # ------------------------------------------------------------------

    def swap_driver(
        self,
        driver: str,
        config: dict[str, Any] | None = None,
    ) -> CommandResult:
        """Hot-swap the robot driver at runtime.

        Args:
            driver: Driver name (e.g. ``"thunder"``, ``"stub"``, ``"sim_mujoco"``).
            config: Optional driver-specific configuration.

        Example::

            robot.swap_driver("sim_mujoco")
            robot.swap_driver("thunder", {"host": "192.168.66.190"})
        """
        body: dict[str, Any] = {"driver": driver}
        if config:
            body["config"] = config
        return self._command("/api/v1/driver/swap", body)

    def swap(self, driver: str, config: dict[str, Any] | None = None) -> CommandResult:
        """Alias for :meth:`swap_driver`.

        Shorter name for interactive use.

        Example::

            robot.swap("stub")
        """
        return self.swap_driver(driver, config)

    # ------------------------------------------------------------------
    # Runtime backend
    # ------------------------------------------------------------------

    def switch_backend(self, category: str, backend: str, config: dict[str, Any] | None = None) -> CommandResult:
        """Switch a runtime backend (non-motion).

        Args:
            category: Backend category (e.g. ``"detector"``, ``"planner"``).
            backend: Backend name (e.g. ``"yoloe"``, ``"astar"``).
            config: Optional backend config dict.

        Example::

            robot.switch_backend("planner", "pct")
        """
        body: dict[str, Any] = {"category": category, "backend": backend}
        if config:
            body["config"] = config
        return self._command("/api/v1/runtime/backend", body)

    # ------------------------------------------------------------------
    # App / system info
    # ------------------------------------------------------------------

    def capabilities(self) -> dict[str, Any]:
        """Get the API capability manifest from the robot.

        Example::

            caps = robot.capabilities()
            print(f"API version: {caps.get('version', '?')}")
        """
        return self._get("/api/v1/app/capabilities")

    def bootstrap(self) -> dict[str, Any]:
        """Get the bootstrap snapshot (full system state for web clients).

        Example::

            b = robot.bootstrap()
        """
        return self._get("/api/v1/app/bootstrap")

    def devices(self) -> dict[str, Any]:
        """Get the hardware device registry status.

        Example::

            d = robot.devices()
        """
        return self._get("/api/v1/devices")

    def readiness(self) -> dict[str, Any]:
        """Get client readiness snapshot.

        Example::

            r = robot.readiness()
        """
        return self._get("/api/v1/readiness")

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------

    def field_check(self) -> CommandResult:
        """Run a read-only product field readiness check.

        Example::

            result = robot.field_check()
        """
        return self._command("/api/v1/diagnostics/field-check")

    def runtime_contract(self) -> dict[str, Any]:
        """Get the canonical runtime interface contract.

        Example::

            rc = robot.runtime_contract()
        """
        return self._get("/api/v1/diagnostics/runtime-contract")

    # ------------------------------------------------------------------
    # Auth
    # ------------------------------------------------------------------

    def auth_login(self, api_key: str) -> dict[str, Any]:
        """Login with an API key.

        Args:
            api_key: The API key to authenticate with.

        Example::

            robot.auth_login("my-secret-key")
        """
        return self._post("/api/v1/auth/login", {"api_key": api_key})

    def auth_check(self) -> dict[str, Any]:
        """Check if authentication is required.

        Example::

            ac = robot.auth_check()
        """
        return self._get("/api/v1/auth/check")

    # ------------------------------------------------------------------
    # Internal HTTP helpers
    # ------------------------------------------------------------------

    def _command(self, path: str, data: dict[str, Any] | None = None) -> CommandResult:
        raw = self._post(path, data)
        ok = raw.get("ok", False) or raw.get("success", False)
        if not ok and "error" not in raw and "message" not in raw:
            ok = True  # no error signal = success
        return CommandResult(
            ok=ok,
            message=raw.get("message", raw.get("reason", raw.get("error", ""))),
            raw=raw,
        )

    def _get(self, path: str) -> dict[str, Any]:
        url = self.base_url + path
        req = urllib.request.Request(url)
        if self._api_key:
            req.add_header("X-API-Key", self._api_key)
        try:
            with urllib.request.urlopen(req) as r:
                return json.loads(r.read())
        except urllib.error.HTTPError as exc:
            body = exc.read()
            try:
                return json.loads(body)
            except json.JSONDecodeError:
                return {"error": f"HTTP {exc.code}", "detail": body.decode(errors="replace")}

    def _post(self, path: str, data: dict[str, Any] | None = None) -> dict[str, Any]:
        url = self.base_url + path
        body = json.dumps(data or {}).encode("utf-8")
        req = urllib.request.Request(
            url,
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        if self._api_key:
            req.add_header("X-API-Key", self._api_key)
        try:
            with urllib.request.urlopen(req) as r:
                return json.loads(r.read())
        except urllib.error.HTTPError as exc:
            err_body = exc.read()
            try:
                return json.loads(err_body)
            except json.JSONDecodeError:
                return {"error": f"HTTP {exc.code}", "detail": err_body.decode(errors="replace")}

    @staticmethod
    def _parse_state(raw: dict[str, Any]) -> RobotState:
        o = raw.get("odometry", {})
        m = raw.get("navigation", raw.get("mission", {}))
        return RobotState(
            mode=raw.get("mode", ""),
            odometry=Position(
                x=o.get("x", 0.0),
                y=o.get("y", 0.0),
                z=o.get("z", 0.0),
                yaw=o.get("yaw", 0.0),
            ),
            mission=NavigationStatus(
                state=m.get("state", raw.get("mission_state", "idle")),
                distance_to_goal=m.get("distance_to_goal", 0.0),
                time_elapsed=m.get("time_elapsed", 0.0),
                raw=m,
            ),
            safety_level=raw.get("safety_level", raw.get("safety", {}).get("level", "")),
            raw=raw,
        )

    @staticmethod
    def _parse_map_list(raw: dict[str, Any]) -> MapList:
        active = raw.get("active_map")
        maps_raw = raw.get("maps", [])
        maps: list[MapInfo] = []
        for m in maps_raw:
            if isinstance(m, str):
                maps.append(MapInfo(name=m, raw={"name": m}))
            elif isinstance(m, dict):
                maps.append(MapInfo(
                    name=m.get("name", ""),
                    has_pcd=m.get("has_pcd", False),
                    is_active=m.get("is_active", False) or m.get("name") == active,
                    size_bytes=m.get("size_bytes", 0),
                    raw=m,
                ))
        return MapList(maps=maps, active_map=active, raw=raw)

    def close(self) -> None:
        """Close any held resources (no-op for the stdlib client).

        Example::

            robot.close()
        """
        pass

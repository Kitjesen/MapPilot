"""Synchronous HTTP client for the LingTu robot Gateway."""

from __future__ import annotations

import json
import urllib.request
import urllib.error
from typing import Any


class LingTuClient:
    """Synchronous client for LingTu robot control and state inspection.

    Uses only the Python stdlib — no third-party dependencies.

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

    def go(self, x: float, y: float, yaw: float = 0.0) -> dict[str, Any]:
        """Navigate to map coordinates.

        Args:
            x: X coordinate in metres (map frame).
            y: Y coordinate in metres.
            yaw: Heading in radians (default 0).

        Returns:
            Server response with status and goal.
        """
        return self._post("/api/v1/goal", {"x": x, "y": y, "yaw": yaw})

    def go_to(self, target: str) -> dict[str, Any]:
        """Navigate by semantic label or natural-language instruction.

        Args:
            target: Location name, object description, or instruction text.

        Returns:
            Server response.
        """
        return self._post("/api/v1/instruction", {"text": target})

    def stop(self) -> dict[str, Any]:
        """Emergency stop — immediately halt all motion."""
        return self._post("/api/v1/stop")

    def cancel(self, reason: str = "client_cancel") -> dict[str, Any]:
        """Gracefully cancel the current navigation mission.

        Args:
            reason: Cancel reason string (default "client_cancel").
        """
        return self._post("/api/v1/navigation/cancel", {"reason": reason})

    # ------------------------------------------------------------------
    # State
    # ------------------------------------------------------------------

    def state(self) -> dict[str, Any]:
        """Get full robot state: odometry, mission, safety, mode, etc."""
        return self._get("/api/v1/state")

    def health(self) -> dict[str, Any]:
        """Get system health overview (modules, sensors, SLAM rate, etc.)."""
        return self._get("/api/v1/health")

    def position(self) -> dict[str, float]:
        """Get current position as {x, y, z, yaw}.

        This is a convenience wrapper around ``state()``.
        """
        s = self.state()
        o = s.get("odometry", {})
        return {
            "x": o.get("x", 0.0),
            "y": o.get("y", 0.0),
            "z": o.get("z", 0.0),
            "yaw": o.get("yaw", 0.0),
        }

    def session(self) -> dict[str, Any]:
        """Get current session state (mode, SLAM profile, active map, etc.)."""
        return self._get("/api/v1/session")

    # ------------------------------------------------------------------
    # Velocity control
    # ------------------------------------------------------------------

    def drive(self, vx: float = 0.0, vy: float = 0.0, wz: float = 0.0) -> dict[str, Any]:
        """Send direct velocity command to the robot.

        Args:
            vx: Linear velocity forward (m/s).
            vy: Linear velocity sideways (m/s).
            wz: Angular velocity (rad/s).
        """
        return self._post("/api/v1/cmd_vel", {"vx": vx, "vy": vy, "wz": wz})

    # ------------------------------------------------------------------
    # Maps
    # ------------------------------------------------------------------

    def maps(self) -> dict[str, Any]:
        """List saved maps on the robot."""
        return self._get("/api/v1/slam/maps")

    def save_map(self, name: str | None = None) -> dict[str, Any]:
        """Save the current SLAM map.

        Args:
            name: Map name. If omitted the server auto-generates one.
        """
        body: dict[str, Any] = {}
        if name:
            body["name"] = name
        return self._post("/api/v1/map/save", body)

    def use_map(self, name: str) -> dict[str, Any]:
        """Activate a saved map for navigation.

        Args:
            name: Name of an existing saved map.
        """
        return self._post("/api/v1/map/activate", {"name": name})

    def rename_map(self, old_name: str, new_name: str) -> dict[str, Any]:
        """Rename a saved map."""
        return self._post("/api/v1/map/rename", {"old_name": old_name, "new_name": new_name})

    # ------------------------------------------------------------------
    # Mode
    # ------------------------------------------------------------------

    def set_mode(self, mode: str) -> dict[str, Any]:
        """Set robot operating mode.

        Args:
            mode: One of ``"manual"``, ``"autonomous"``, or ``"estop"``.
        """
        return self._post("/api/v1/mode", {"mode": mode})

    # ------------------------------------------------------------------
    # Perception
    # ------------------------------------------------------------------

    def scene(self) -> dict[str, Any]:
        """Get the current scene graph (detected objects, relations, regions)."""
        return self._get("/api/v1/scene_graph")

    def locations(self) -> dict[str, Any]:
        """List tagged navigation locations."""
        return self._get("/api/v1/locations")

    def tag_location(
        self,
        name: str,
        x: float | None = None,
        y: float | None = None,
        yaw: float | None = None,
        use_current_pose: bool = False,
    ) -> dict[str, Any]:
        """Create or update a tagged navigation location.

        Args:
            name: Location label.
            x: X coordinate (required unless ``use_current_pose`` is true).
            y: Y coordinate (required unless ``use_current_pose`` is true).
            yaw: Optional heading.
            use_current_pose: If true, use the robot's current odometry pose.
        """
        body: dict[str, Any] = {"name": name, "use_current_pose": use_current_pose}
        if x is not None:
            body["x"] = x
        if y is not None:
            body["y"] = y
        if yaw is not None:
            body["yaw"] = yaw
        return self._post("/api/v1/locations", body)

    # ------------------------------------------------------------------
    # Lease (control ownership)
    # ------------------------------------------------------------------

    def acquire_lease(self, client_id: str, ttl: float = 30.0) -> dict[str, Any]:
        """Acquire the control lease.

        Only one client may hold the lease at a time.
        """
        return self._post("/api/v1/lease", {
            "action": "acquire",
            "client_id": client_id,
            "ttl": ttl,
        })

    def release_lease(self, client_id: str) -> dict[str, Any]:
        """Release the control lease."""
        return self._post("/api/v1/lease", {
            "action": "release",
            "client_id": client_id,
        })

    def renew_lease(self, client_id: str, ttl: float = 30.0) -> dict[str, Any]:
        """Renew the control lease before it expires."""
        return self._post("/api/v1/lease", {
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
    ) -> dict[str, Any]:
        """Enter a robot session mode.

        Args:
            mode: ``"mapping"``, ``"navigating"``, or ``"exploring"``.
            map_name: Map to use (required for navigating).
            slam_profile: SLAM backend (``"fastlio2"``, ``"localizer"``, etc.).
        """
        body: dict[str, Any] = {"mode": mode}
        if map_name:
            body["map_name"] = map_name
        if slam_profile:
            body["slam_profile"] = slam_profile
        return self._post("/api/v1/session/start", body)

    def end_session(self) -> dict[str, Any]:
        """End the current robot session."""
        return self._post("/api/v1/session/end")

    # ------------------------------------------------------------------
    # Driver swap (Phase 2)
    # ------------------------------------------------------------------

    def swap_driver(
        self,
        driver: str,
        config: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Hot-swap the robot driver at runtime.

        Args:
            driver: Driver name (e.g. ``"thunder"``, ``"stub"``).
            config: Optional driver-specific configuration.
        """
        body: dict[str, Any] = {"driver": driver}
        if config:
            body["config"] = config
        return self._post("/api/v1/driver/swap", body)

    # ------------------------------------------------------------------
    # Internal HTTP helpers
    # ------------------------------------------------------------------

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

    def close(self) -> None:
        """Close any held resources (no-op for the stdlib client)."""
        pass

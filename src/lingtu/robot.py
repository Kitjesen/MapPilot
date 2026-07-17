"""Local Robot facade over the real LingTu blueprint."""

from __future__ import annotations

import json
import logging

logger = logging.getLogger(__name__)


class Robot:
    """Complete robot interface backed by a runtime profile.

    ``mode`` selects a profile from ``cli/profiles_data.py``:

    - ``nav``: saved-map navigation
    - ``sim``: MuJoCo simulation
    - ``explore``: frontier exploration
    - ``map``: mapping / SLAM
    - ``dev``: semantic pipeline on the stub driver
    """

    _MODE_TO_PROFILE = {
        "nav": "nav",
        "sim": "sim",
        "explore": "explore",
        "map": "map",
        "dev": "dev",
    }

    def __init__(self, mode: str = "nav", *, profile: str | None = None, **overrides):
        self._mode = mode
        self._profile = profile or self._MODE_TO_PROFILE.get(mode, mode)
        self._overrides = overrides
        self._system = None
        self._started = False

    def start(self) -> Robot:
        """Build and start the selected runtime profile."""

        if self._started:
            return self
        from .runtime import build_system

        self._system = build_system(self._profile, overrides=self._overrides)
        self._system.start()
        self._started = True
        logger.info("Robot started (profile=%s, %d modules)", self._profile, len(self._system.modules))
        return self

    def shutdown(self) -> None:
        """Stop the current runtime profile."""

        if self._system is not None:
            try:
                self._system.stop()
            except Exception as e:
                logger.debug("Robot shutdown error: %s", e)
        self._system = None
        self._started = False

    @property
    def system(self):
        """The underlying SystemHandle, or ``None`` until ``start()``."""

        return self._system

    def _mod(self, name: str):
        if self._system is None:
            return None
        try:
            return self._system.get_module(name)
        except KeyError:
            return None

    def _nav(self):
        """Return the canonical command adapter, then the compatibility target."""

        return self._mod("nav.skills") or self._mod("nav.mission")

    def go(self, instruction: str) -> str:
        """Send a natural-language instruction to the semantic planner."""

        sem = self._mod("SemanticPlannerModule")
        if sem is None:
            return "SemanticPlannerModule not available (profile has no semantic stack)"
        if hasattr(sem, "send_instruction"):
            return str(sem.send_instruction(instruction))
        sem.instruction._deliver(instruction)
        return f"Instruction sent: {instruction}"

    def go_to(self, x: float, y: float, yaw: float = 0.0, *, z: float | None = None) -> str:
        """Navigate to map coordinates.

        ``z`` is optional for old 2D calls. If omitted, Navigation uses
        the current odometry height and then falls back to ``0.0``.
        """

        nav = self._nav()
        if nav is None:
            return "Navigation not available"
        if z is None:
            return nav.navigate_to(x, y, yaw)
        return nav.navigate_to(x, y, yaw, z=z)

    def go_to_3d(self, x: float, y: float, z: float, yaw: float = 0.0) -> str:
        """Navigate to an explicit 3D map coordinate."""

        return self.go_to(x, y, yaw=yaw, z=z)

    def stop_motion(self) -> str:
        """Immediately stop all robot motion."""

        safety = self._mod("nav.safety")
        if safety is not None and hasattr(safety, "emergency_stop"):
            return safety.emergency_stop()
        nav = self._nav()
        if nav is None:
            return "Navigation not available"
        return nav.stop_navigation()

    def cancel(self) -> str:
        """Cancel the current navigation mission."""

        nav = self._nav()
        if nav is None:
            return "Navigation not available"
        return nav.cancel_mission()

    def status(self) -> str:
        """Current mission/navigation state."""

        nav = self._nav()
        if nav is None:
            return "NOT_STARTED"
        get_status = getattr(nav, "get_navigation_status", None)
        if callable(get_status):
            try:
                payload = json.loads(get_status())
                return str(payload.get("state", "UNKNOWN"))
            except Exception as e:
                logger.debug("Robot status parse failed: %s", e)
        health = getattr(nav, "health", None)
        if callable(health):
            return str(health().get("state", "UNKNOWN"))
        return "UNKNOWN"

    def follow(self, description: str) -> str:
        """Follow a described person, e.g. ``"person in red"``."""

        vs = self._mod("VisualServoModule")
        if vs is None:
            return "VisualServoModule not in this profile; try nav/sim/explore/dev"
        return vs.follow_person(description)

    def stop_follow(self) -> str:
        """Stop person following / visual servoing."""

        vs = self._mod("VisualServoModule")
        if vs is None:
            return "VisualServoModule not available"
        return vs.stop_servo()

    def approach(self, label: str) -> str:
        """Visually find and approach an object by label."""

        vs = self._mod("VisualServoModule")
        if vs is None:
            return "VisualServoModule not available"
        return vs.find_object(label)

    def save_map(self, name: str) -> bool:
        """Save the current SLAM map."""

        return self._map_skill_ok("save_map", name)

    def use_map(self, name: str) -> bool:
        """Activate a saved map by name."""

        return self._map_skill_ok("use_map", name)

    def list_maps(self) -> list:
        """List maps known to the active maps module."""

        mm = self._mod("maps.service")
        if mm is None:
            return []
        try:
            return json.loads(mm.list_maps())
        except Exception as e:
            logger.debug("list_maps failed: %s", e)
            return []

    def _map_skill_ok(self, skill: str, name: str) -> bool:
        mm = self._mod("maps.service")
        if mm is None:
            logger.error("MapsModule not available")
            return False
        try:
            result = getattr(mm, skill)(name)
            if isinstance(result, str):
                payload = json.loads(result)
                if "ok" in payload:
                    return bool(payload["ok"])
                if "success" in payload:
                    return bool(payload["success"])
                return "error" not in payload
            return bool(result)
        except Exception as e:
            logger.error("%s failed: %s", skill, e)
            return False

    def detect(self) -> list[dict]:
        """Current scene-graph detections."""

        sem = self._mod("SemanticPlannerModule")
        get_objects = getattr(sem, "get_scene_objects", None) if sem is not None else None
        if not callable(get_objects):
            return []
        try:
            payload = json.loads(get_objects())
            return payload if isinstance(payload, list) else []
        except Exception as e:
            logger.debug("scene object query failed: %s", e)
            return []

    def find(self, label: str) -> dict | None:
        """Best scene-graph match for a label, or ``None``."""

        target = label.lower()
        best, best_score = None, 0.0
        for d in self.detect():
            dl = d["label"].lower()
            if (target in dl or dl in target) and d["confidence"] > best_score:
                best, best_score = d, d["confidence"]
        return best

    def get_pose(self):
        """Robot position ``(x, y, z)`` in the map frame, or ``None``."""

        nav = self._nav()
        get_status = getattr(nav, "get_navigation_status", None) if nav is not None else None
        if not callable(get_status):
            return None
        try:
            payload = json.loads(get_status())
            pos = payload.get("position") or {}
            return (float(pos["x"]), float(pos["y"]), float(pos.get("z", 0.0)))
        except Exception as e:
            logger.debug("Robot pose query failed: %s", e)
            return None

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *args):
        self.shutdown()

    def __repr__(self):
        return "Robot(profile=%s, started=%s)" % (self._profile, self._started)

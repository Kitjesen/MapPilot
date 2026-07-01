"""Robot — all-in-one SDK facade over the real LingTu blueprint.

One import, full capability. Internally resolves the same product profiles that
``lingtu.py`` uses, then builds the selected product or compatibility Blueprint
through the shared runtime profile builder.

Usage::

    from lingtu import Robot

    robot = Robot("sim").start()        # sim/nav/explore/map/dev profiles
    robot.go("体育馆")                   # semantic navigation
    robot.go_to(5.0, 3.0)               # coordinate navigation
    robot.follow("person in red")       # follow a described person
    robot.stop_follow()
    robot.save_map("building_a")
    robot.shutdown()

    # context-manager form
    with Robot("sim") as robot:
        robot.follow("person in red")
"""

from __future__ import annotations

import logging

logger = logging.getLogger(__name__)


class Robot:
    """Complete robot interface backed by a real blueprint system.

    ``mode`` / ``profile`` selects the blueprint profile (see
    ``cli/profiles_data.py``):

      ``nav``     — saved-map navigation (default; real robot, semantic stack)
      ``sim``     — MuJoCo simulation, full semantic stack (best for local demos)
      ``explore`` — frontier exploration
      ``map``     — mapping / SLAM
      ``dev``     — semantic pipeline on the stub driver

    Person following ("person in red") needs the semantic stack (VisualServo);
    ``nav`` / ``sim`` / ``explore`` / ``dev`` all include it. ``map`` does not.

    Override any resolved profile flag via keyword args, e.g.
    ``Robot("nav", llm="openai", encoder="clip")`` to enable VLM/CLIP person
    selection on a real robot (the defaults — kimi + mobileclip — fall back to
    following the most-salient person).
    """

    _MODE_TO_PROFILE = {
        "nav": "nav",
        "sim": "sim",
        "explore": "explore",
        "map": "map",
        "dev": "dev",
    }

    def __init__(self, mode: str = "nav", *, profile: str | None = None, **overrides):
        """
        Args:
            mode: shorthand profile selector (nav/sim/explore/map/dev).
            profile: explicit CLI profile name; overrides the ``mode`` mapping.
            **overrides: resolved profile config overrides
                         (e.g. ``llm="openai"``, ``encoder="clip"``,
                         ``gateway_port=5051``).
        """
        self._mode = mode
        self._profile = profile or self._MODE_TO_PROFILE.get(mode, mode)
        self._overrides = overrides
        self._system = None
        self._started = False

    # ── Lifecycle ──────────────────────────────────────────────────────────
    def start(self) -> "Robot":
        """Build and start the full blueprint system for the chosen profile."""
        if self._started:
            return self
        from core.blueprints.profile_builder import build_system_from_resolved_profile
        from core.runtime.resolver import resolve_profile_config

        cfg = resolve_profile_config(self._profile, overrides=self._overrides)
        self._system = build_system_from_resolved_profile(self._profile, cfg)
        self._system.start()
        self._started = True
        logger.info("Robot started (profile=%s, %d modules)",
                    self._profile, len(self._system.modules))
        return self

    def shutdown(self) -> None:
        """Stop the whole system."""
        if self._system is not None:
            try:
                self._system.stop()
            except Exception as e:
                logger.debug("Robot shutdown error: %s", e)
        self._system = None
        self._started = False

    @property
    def system(self):
        """The underlying SystemHandle (``None`` until ``start()``)."""
        return self._system

    def _mod(self, name: str):
        if self._system is None:
            return None
        try:
            return self._system.get_module(name)
        except KeyError:
            return None

    # ── Navigation ─────────────────────────────────────────────────────────
    def go(self, instruction: str) -> str:
        """Natural-language navigation via the semantic planner.

        Also routes person-following intent ("跟着穿红衣服的人" / "follow ...").
        """
        sem = self._mod("SemanticPlannerModule")
        if sem is None:
            return "SemanticPlannerModule not available (profile has no semantic stack)"
        if hasattr(sem, "send_instruction"):
            return str(sem.send_instruction(instruction))
        sem.instruction._deliver(instruction)
        return f"Instruction sent: {instruction}"

    def go_to(self, x: float, y: float, yaw: float = 0.0) -> str:
        """Navigate to map coordinates."""
        nav = self._mod("NavigationModule")
        if nav is None:
            return "NavigationModule not available"
        return nav.navigate_to(x, y, yaw)

    def stop_motion(self) -> str:
        """Immediately stop all robot motion."""
        nav = self._mod("NavigationModule")
        if nav is None:
            return "NavigationModule not available"
        return nav.stop_navigation()

    def cancel(self) -> str:
        """Cancel the current navigation mission."""
        nav = self._mod("NavigationModule")
        if nav is None:
            return "NavigationModule not available"
        return nav.cancel_mission()

    def status(self) -> str:
        """Current mission/navigation state."""
        nav = self._mod("NavigationModule")
        if nav is None:
            return "NOT_STARTED"
        state = getattr(nav, "_state", None)
        return str(getattr(state, "value", state) or "UNKNOWN")

    # ── Person following / visual servo ──────────────────────────────────────
    def follow(self, description: str) -> str:
        """Follow a described person, e.g. ``"person in red"``.

        Locks onto the matching person via CLIP (image-capable encoder) or VLM
        (look at the crops), then tracks them through the planning stack. With
        the default kimi + mobileclip stack neither is available, so it falls
        back to following the most-salient person — pass ``llm="openai"`` (or
        ``encoder="clip"``) at construction for description-based selection.
        """
        vs = self._mod("VisualServoModule")
        if vs is None:
            return ("VisualServoModule not in this profile — person following "
                    "needs the semantic stack (try nav/sim/explore/dev)")
        return vs.follow_person(description)

    def stop_follow(self) -> str:
        """Stop person following / visual servoing."""
        vs = self._mod("VisualServoModule")
        if vs is None:
            return "VisualServoModule not available"
        return vs.stop_servo()

    def approach(self, label: str) -> str:
        """Visually find and approach an object by label (visual servo)."""
        vs = self._mod("VisualServoModule")
        if vs is None:
            return "VisualServoModule not available"
        return vs.find_object(label)

    # ── Mapping ──────────────────────────────────────────────────────────────
    def save_map(self, name: str) -> bool:
        """Save the current SLAM map (PGO + DUFOMap + tomogram + occupancy)."""
        return self._map_skill_ok("save_map", name)

    def use_map(self, name: str) -> bool:
        """Activate a saved map by name."""
        return self._map_skill_ok("use_map", name)

    def list_maps(self) -> list:
        mm = self._mod("MapManagerModule")
        if mm is None:
            return []
        try:
            import json
            return json.loads(mm.list_maps())
        except Exception as e:
            logger.debug("list_maps failed: %s", e)
            return []

    def _map_skill_ok(self, skill: str, name: str) -> bool:
        mm = self._mod("MapManagerModule")
        if mm is None:
            logger.error("MapManagerModule not available")
            return False
        try:
            result = getattr(mm, skill)(name)
            if isinstance(result, str):
                import json
                return bool(json.loads(result).get("ok", True))
            return bool(result)
        except Exception as e:
            logger.error("%s failed: %s", skill, e)
            return False

    # ── Perception ───────────────────────────────────────────────────────────
    def detect(self) -> list[dict]:
        """Current scene-graph detections: ``[{label, confidence, position}]``."""
        sem = self._mod("SemanticPlannerModule")
        sg = getattr(sem, "_current_scene_graph", None) if sem is not None else None
        if sg is None or not getattr(sg, "objects", None):
            return []
        out = []
        for o in sg.objects:
            pos = getattr(o, "position", None)
            out.append({
                "label": o.label or "",
                "confidence": float(getattr(o, "confidence", 0.0)),
                "position": [float(getattr(pos, "x", 0.0)),
                             float(getattr(pos, "y", 0.0)),
                             float(getattr(pos, "z", 0.0))] if pos else [0.0, 0.0, 0.0],
            })
        return out

    def find(self, label: str) -> dict | None:
        """Best scene-graph match for a label, or ``None``."""
        target = label.lower()
        best, best_score = None, 0.0
        for d in self.detect():
            dl = d["label"].lower()
            if (target in dl or dl in target) and d["confidence"] > best_score:
                best, best_score = d, d["confidence"]
        return best

    # ── Pose ─────────────────────────────────────────────────────────────────
    def get_pose(self):
        """Robot position ``(x, y, z)`` in the map frame, or ``None``."""
        nav = self._mod("NavigationModule")
        pos = getattr(nav, "_robot_pos", None) if nav is not None else None
        if pos is None:
            return None
        return (float(pos[0]), float(pos[1]), float(pos[2]))

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *args):
        self.shutdown()

    def __repr__(self):
        return "Robot(profile=%s, started=%s)" % (self._profile, self._started)

"""MCP/skill surface for Navigation."""

from __future__ import annotations

import json
import math

from nav.mission.model.status import build_navigation_status_payload
from nav.services.goals import build_goal_pose
from runtime.module import skill
from runtime.msgs.nav import Odometry


class NavigationSkillsMixin:
    @skill
    def send_instruction(self, text: str) -> str:
        self._on_instruction(text)
        return json.dumps({"status": "sent", "instruction": text})

    def submit_odometry(self, odom: Odometry) -> None:
        self._on_odom(odom)

    @skill
    def navigate_to(
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
        z: float | None = None,
    ) -> str:
        try:
            goal_x = float(x)
            goal_y = float(y)
            goal_yaw = float(yaw)
            goal_z = self._resolve_goal_z(z)
        except (TypeError, ValueError) as exc:
            return json.dumps({
                "status": "rejected",
                "reason": "invalid_coordinates",
                "error": str(exc),
                "frame_id": self._planning_frame_id,
            })
        if not all(math.isfinite(v) for v in (goal_x, goal_y, goal_yaw, goal_z)):
            return json.dumps({
                "status": "rejected",
                "reason": "invalid_coordinates",
                "error": "x, y, z, and yaw must be finite",
                "frame_id": self._planning_frame_id,
            })
        self._on_goal(build_goal_pose(
            x=goal_x,
            y=goal_y,
            z=goal_z,
            yaw=goal_yaw,
            planning_frame_id=self._planning_frame_id,
        ))
        return json.dumps({
            "status": "navigating",
            "goal": [goal_x, goal_y, goal_z],
            "yaw": goal_yaw,
            "frame_id": self._planning_frame_id,
        })

    def _resolve_goal_z(self, z: float | None) -> float:
        if z is not None:
            try:
                z_value = float(z)
                if math.isfinite(z_value):
                    return z_value
            except (TypeError, ValueError):
                pass
            raise ValueError("z must be finite")
        try:
            current_z = float(self._robot_pos[2])
            if math.isfinite(current_z):
                return current_z
        except (AttributeError, IndexError, TypeError, ValueError):
            pass
        return 0.0

    @skill
    def stop_navigation(self) -> str:
        self._on_stop(2)
        return json.dumps({"status": "stopped"})

    @skill
    def cancel_mission(self, reason: str = "user_cancel") -> str:
        self._on_cancel(reason)
        return json.dumps({"status": "cancelled", "reason": reason})

    @skill
    def get_navigation_status(self) -> str:
        return json.dumps(build_navigation_status_payload(self))

    @skill
    def start_patrol(self, waypoints_json: str) -> str:
        try:
            goals = json.loads(waypoints_json)
            if not isinstance(goals, list) or not goals:
                return json.dumps({
                    "status": "rejected",
                    "reason": "waypoints must be a non-empty JSON list",
                })
            self._on_patrol_goals(goals)
            return json.dumps({"status": "submitted", "goals": len(goals)})
        except Exception as exc:
            return json.dumps({"error": str(exc)})

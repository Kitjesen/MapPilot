"""GeofenceManagerModule -- hive Module version of GeofenceManager.

Manages geofence polygons (add/remove/enable/disable/clear) and checks
robot intrusion via ray-casting point-in-polygon.

Ports:
    In:  odometry (Odometry)           -- robot pose
         geofence_command (str)        -- JSON command string
    Out: geofence_alert (dict)         -- intrusion warnings or command responses
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
from typing import Any

from runtime import In, Module, Out
from runtime.msgs.nav import Odometry
from runtime.registry import register
from runtime.yaml_helpers import load_yaml, save_yaml


@register("safety", "geofence", description="Geofence boundary monitor")
class GeofenceManagerModule(Module, layer=6):
    """Geofence management module (hive Module).

    Pure geometry + file-system logic.  Boundary publishing to
    /nav/navigation_boundary requires the ROS2 node version.
    Call ``check_intrusion()`` periodically to detect violations.
    """

    odometry: In[Odometry]
    geofence_command: In[str]
    geofence_alert: Out[dict]
    stop_cmd: Out[int]

    def __init__(self, **config: Any) -> None:
        super().__init__(**config)
        default_file = config.get(
            "geofence_file", os.path.expanduser("~/.lingtu/geofences.yaml")
        )
        self._geofence_file = Path(default_file)
        self._geofence_file.parent.mkdir(parents=True, exist_ok=True)
        self._fences: dict[str, dict[str, Any]] = self._load_fences()
        self._robot_x: float = 0.0
        self._robot_y: float = 0.0

    def setup(self) -> None:
        self.odometry.subscribe(self._on_odom)
        self.geofence_command.subscribe(self._on_command)

    # -- callbacks --------------------------------------------------------------

    def _on_odom(self, msg: Odometry) -> None:
        self._robot_x = msg.x
        self._robot_y = msg.y
        self.check_intrusion()

    def _on_command(self, raw: str) -> None:
        """Parse JSON command and dispatch to handler."""
        try:
            cmd = json.loads(raw) if isinstance(raw, str) else raw
        except (json.JSONDecodeError, TypeError):
            cmd = {}

        action = cmd.get("action", "")
        resp: dict[str, Any] = {"action": action, "success": False}

        try:
            if action == "add":
                resp = self._add_fence(cmd)
            elif action == "remove":
                resp = self._remove_fence(cmd.get("name", ""))
            elif action == "list":
                resp = self._list_fences()
            elif action == "clear":
                resp = self._clear_fences()
            elif action == "enable":
                resp = self._set_enabled(cmd.get("name", ""), True)
            elif action == "disable":
                resp = self._set_enabled(cmd.get("name", ""), False)
            else:
                resp["message"] = f"unknown action: {action}"
        except Exception as exc:
            resp["message"] = str(exc)

        self.geofence_alert.publish(resp)

    # -- fence CRUD -------------------------------------------------------------

    def _add_fence(self, cmd: dict[str, Any]) -> dict[str, Any]:
        name = cmd.get("name", "")
        polygon = cmd.get("polygon", [])
        if not name:
            return {"action": "add", "success": False, "message": "missing fence name"}
        try:
            polygon = self._normalize_polygon(polygon)
        except ValueError as exc:
            return {"action": "add", "success": False, "message": str(exc)}
        self._fences[name] = {"polygon": polygon, "enabled": True}
        self._save_fences()
        return {"action": "add", "success": True, "message": f"fence added: {name}"}

    def _remove_fence(self, name: str) -> dict[str, Any]:
        if not name:
            return {"action": "remove", "success": False, "message": "missing fence name"}
        if name not in self._fences:
            return {"action": "remove", "success": False, "message": f"fence not found: {name}"}
        del self._fences[name]
        self._save_fences()
        return {"action": "remove", "success": True, "message": f"fence removed: {name}"}

    def _list_fences(self) -> dict[str, Any]:
        fences = [
            {
                "name": name,
                "vertex_count": len(data.get("polygon", [])),
                "enabled": data.get("enabled", True),
            }
            for name, data in self._fences.items()
        ]
        return {"action": "list", "success": True, "fences": fences}

    def _clear_fences(self) -> dict[str, Any]:
        count = len(self._fences)
        self._fences.clear()
        self._save_fences()
        return {"action": "clear", "success": True, "message": f"cleared {count} fences"}

    def _set_enabled(self, name: str, enabled: bool) -> dict[str, Any]:
        action_name = "enable" if enabled else "disable"
        if not name:
            return {"action": action_name, "success": False, "message": "missing fence name"}
        if name not in self._fences:
            return {"action": action_name, "success": False, "message": f"fence not found: {name}"}
        self._fences[name]["enabled"] = enabled
        self._save_fences()
        state = "enabled" if enabled else "disabled"
        return {"action": action_name, "success": True, "message": f"fence {state}: {name}"}

    # -- intrusion detection (call periodically) --------------------------------

    def check_intrusion(self) -> list[dict[str, Any]]:
        """Check if robot is inside any enabled geofence.

        Returns list of intrusion alerts (empty if safe).
        Each alert is also published to geofence_alert.
        """
        alerts = self._intrusions()
        for alert in alerts:
            self.geofence_alert.publish(alert)
            self.stop_cmd.publish(2)
        return alerts

    def _intrusions(self) -> list[dict[str, Any]]:
        alerts: list[dict[str, Any]] = []
        for name, data in self._fences.items():
            if not data.get("enabled", True):
                continue
            if self._point_in_polygon(self._robot_x, self._robot_y, data.get("polygon", [])):
                alert = {
                    "action": "warning",
                    "type": "intrusion",
                    "fence": name,
                    "robot_x": self._robot_x,
                    "robot_y": self._robot_y,
                    "message": f"robot inside restricted zone: {name}",
                }
                alerts.append(alert)
        return alerts

    @staticmethod
    def _normalize_polygon(polygon: Any) -> list[list[float]]:
        if not isinstance(polygon, list) or len(polygon) < 3:
            raise ValueError("polygon needs >= 3 vertices")
        points: list[list[float]] = []
        for point in polygon:
            if not isinstance(point, (list, tuple)) or len(point) < 2:
                raise ValueError("polygon vertices must be [x, y]")
            try:
                x = float(point[0])
                y = float(point[1])
            except (TypeError, ValueError) as exc:
                raise ValueError("polygon vertices must be finite numbers") from exc
            if not math.isfinite(x) or not math.isfinite(y):
                raise ValueError("polygon vertices must be finite numbers")
            points.append([x, y])
        return points

    @staticmethod
    def _point_in_polygon(px: float, py: float, polygon: list) -> bool:
        """Ray-casting point-in-polygon test."""
        try:
            points = GeofenceManagerModule._normalize_polygon(polygon)
        except ValueError:
            return False
        n = len(points)
        inside = False
        j = n - 1
        for i in range(n):
            xi, yi = points[i]
            xj, yj = points[j]
            if ((yi > py) != (yj > py)) and (
                px < (xj - xi) * (py - yi) / (yj - yi) + xi
            ):
                inside = not inside
            j = i
        return inside

    # -- health ----------------------------------------------------------------

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["zone_count"] = len(self._fences)
        enabled = sum(1 for f in self._fences.values() if f.get("enabled", True))
        info["enabled_count"] = enabled
        info["violations"] = len(self._intrusions())
        info["robot_position"] = {"x": self._robot_x, "y": self._robot_y}
        return info

    # -- persistence ------------------------------------------------------------

    def _load_fences(self) -> dict[str, dict[str, Any]]:
        data = load_yaml(self._geofence_file)
        return data if isinstance(data, dict) else {}

    def _save_fences(self) -> None:
        save_yaml(self._geofence_file, self._fences)

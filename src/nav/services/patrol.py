"""PatrolManagerModule -- hive Module version of PatrolManager.

Manages patrol route CRUD (save/load/list/delete) and start/stop patrol.
Extracted pure file-system and route logic; ROS2 topic publishing is
replaced by typed Out ports.

Ports:
    In:  patrol_command (str)     -- JSON command string
         odometry (Odometry)      -- robot pose for future waypoint tracking
    Out: patrol_goals (list)      -- waypoint list sent to mission_arc
         patrol_status (str)      -- JSON status/response string
"""

from __future__ import annotations

import json
import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from runtime import In, Module, Out
from runtime.msgs.nav import Odometry
from runtime.yaml_helpers import load_yaml, save_yaml


class PatrolManagerModule(Module, layer=6):
    """Patrol route management module (hive Module).

    Pure file-system route CRUD + patrol start/stop logic.
    No ROS2 dependency.
    """

    patrol_command: In[str]
    odometry: In[Odometry]
    patrol_goals: Out[list]
    patrol_status: Out[str]

    def __init__(self, **config: Any) -> None:
        super().__init__(**config)
        default_routes_dir = config.get(
            "routes_dir", os.path.expanduser("~/.lingtu/patrol_routes")
        )
        self._routes_dir = Path(default_routes_dir)
        self._routes_dir.mkdir(parents=True, exist_ok=True)
        self._active_route: str | None = None
        self._current_pose: dict[str, float] | None = None

    def setup(self) -> None:
        self.patrol_command.subscribe(self._on_command)
        self.odometry.subscribe(self._on_odom)

    # -- callbacks --------------------------------------------------------------

    def _on_odom(self, msg: Odometry) -> None:
        self._current_pose = {"x": msg.x, "y": msg.y, "z": msg.z}

    def _on_command(self, raw: str) -> None:
        """Parse JSON command and dispatch to handler."""
        try:
            cmd = json.loads(raw) if isinstance(raw, str) else raw
        except (json.JSONDecodeError, TypeError):
            cmd = {}

        action = cmd.get("action", "")
        resp: dict[str, Any] = {"action": action, "success": False}

        try:
            if action == "save":
                resp = self._save_route(cmd)
            elif action == "load":
                resp = self._load_route(cmd.get("name", ""))
            elif action == "list":
                resp = self._list_routes()
            elif action == "delete":
                resp = self._delete_route(cmd.get("name", ""))
            elif action == "start":
                resp = self._start_patrol(cmd.get("name", ""))
            elif action == "stop":
                resp = self._stop_patrol()
            else:
                resp["message"] = f"unknown action: {action}"
        except Exception as exc:
            resp["message"] = str(exc)

        self.patrol_status.publish(json.dumps(resp, ensure_ascii=False))

    # -- route CRUD -------------------------------------------------------------

    def _save_route(self, cmd: dict[str, Any]) -> dict[str, Any]:
        name = cmd.get("name", "")
        if not name:
            return {"action": "save", "success": False, "message": "missing route name"}
        name, path = self._route_path(name)
        waypoints = cmd.get("waypoints", [])
        if not waypoints:
            return {"action": "save", "success": False, "message": "waypoints list empty"}

        route_data = {
            "name": name,
            "waypoints": waypoints,
            "loop": cmd.get("loop", False),
            "created": datetime.now(timezone.utc).isoformat(),
        }
        save_yaml(path, route_data)
        return {
            "action": "save",
            "success": True,
            "message": f"route saved: {name} ({len(waypoints)} waypoints)",
        }

    def _load_route(self, name: str) -> dict[str, Any]:
        if not name:
            return {"action": "load", "success": False, "message": "missing route name"}
        name, path = self._route_path(name)
        if not path.exists():
            return {"action": "load", "success": False, "message": f"route not found: {name}"}
        data = load_yaml(path)
        return {"action": "load", "success": True, "route": data}

    def _list_routes(self) -> dict[str, Any]:
        routes: list[dict[str, Any]] = []
        for f in sorted(self._routes_dir.glob("*.yaml")):
            data = load_yaml(f)
            if data:
                routes.append({
                    "name": data.get("name", f.stem),
                    "waypoint_count": len(data.get("waypoints", [])),
                    "loop": data.get("loop", False),
                    "created": data.get("created", ""),
                })
        return {"action": "list", "success": True, "routes": routes}

    def _delete_route(self, name: str) -> dict[str, Any]:
        if not name:
            return {"action": "delete", "success": False, "message": "missing route name"}
        name, path = self._route_path(name)
        if not path.exists():
            return {"action": "delete", "success": False, "message": f"route not found: {name}"}
        try:
            path.unlink()
            return {"action": "delete", "success": True, "message": f"route deleted: {name}"}
        except OSError as exc:
            return {"action": "delete", "success": False, "message": str(exc)}

    # -- patrol control ---------------------------------------------------------

    def _start_patrol(self, name: str) -> dict[str, Any]:
        if not name:
            return {"action": "start", "success": False, "message": "missing route name"}
        name, path = self._route_path(name)
        if not path.exists():
            return {"action": "start", "success": False, "message": f"route not found: {name}"}
        data = load_yaml(path)
        if not data or not data.get("waypoints"):
            return {"action": "start", "success": False, "message": f"invalid route data: {name}"}

        self.patrol_goals.publish(
            self._waypoints_for_publish(data["waypoints"], loop=bool(data.get("loop")))
        )
        self._active_route = name
        return {
            "action": "start",
            "success": True,
            "message": f"patrol started: {name}",
            "route_name": name,
        }

    def _stop_patrol(self) -> dict[str, Any]:
        route_name = self._active_route or "(none)"
        self._active_route = None
        return {"action": "stop", "success": True, "message": f"patrol stopped: {route_name}"}

    def _route_path(self, name: Any) -> tuple[str, Path]:
        route_name = str(name or "").strip()
        if not route_name:
            raise ValueError("missing route name")
        if (
            route_name in {".", ".."}
            or any(sep in route_name for sep in ("/", "\\", ":"))
            or "\x00" in route_name
        ):
            raise ValueError("invalid route name")
        root = self._routes_dir.resolve()
        path = (root / f"{route_name}.yaml").resolve()
        try:
            path.relative_to(root)
        except ValueError as exc:
            raise ValueError("invalid route name") from exc
        return route_name, path

    @staticmethod
    def _waypoints_for_publish(waypoints: list, *, loop: bool) -> list:
        if not loop:
            return waypoints
        result = []
        for item in waypoints:
            if isinstance(item, dict):
                wp = dict(item)
            elif isinstance(item, (list, tuple)) and len(item) >= 2:
                wp = {"x": item[0], "y": item[1]}
                if len(item) > 2:
                    wp["z"] = item[2]
            else:
                wp = item
            if isinstance(wp, dict):
                wp["loop"] = True
            result.append(wp)
        return result


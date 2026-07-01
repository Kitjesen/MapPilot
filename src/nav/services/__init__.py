"""Navigation support services.

Primary services:
  - maps: saved-map lifecycle and POIs.
  - plan: global/local planning service entrypoints.
  - goals: map-frame goal command entry.
  - patrol: saved route CRUD and patrol start.
  - scheduler: optional scheduled patrol trigger.
  - geofence: restricted-zone monitor.
"""

from __future__ import annotations

from importlib import import_module
from typing import Any

__all__ = ["GoalService", "MapService", "create_planner_service"]

_EXPORTS = {
    "GoalService": "nav.services.goals",
    "MapService": "nav.services.maps",
    "create_planner_service": "nav.services.plan.factory",
}


def __getattr__(name: str) -> Any:
    module_name = _EXPORTS.get(name)
    if module_name is None:
        raise AttributeError(name)
    module = import_module(module_name)
    return getattr(module, name)

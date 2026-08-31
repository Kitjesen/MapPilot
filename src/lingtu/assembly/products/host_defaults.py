"""Blueprint defaults for Host processes inside Field Products.

Runtime Graph YAML declares the Products. Product assembly owns these Python
Host inputs.
"""

from __future__ import annotations

from typing import Any

from runtime.graph.loader import load_runtime_graph

_RUNTIME_GRAPH_PRODUCTS = load_runtime_graph().products
FIELD_PRODUCT_NAMES = tuple(_RUNTIME_GRAPH_PRODUCTS)
_TELEOP_LIMITS = {
    "path_follower_max_speed": 0.50,
    "teleop_max_speed_mps": 0.50,
    "teleop_max_yaw_rate_rad_s": 1.0,
}

FIELD_PRODUCT_HOST_DEFAULTS: dict[str, dict[str, Any]] = {
    "explore": {
        "enable_semantic": False,
        "exploration_backend": "none",
    },
    "inspection": {
        "llm": "qwen",
        "encoder": "none",
        "enable_semantic": True,
        "enable_teleop": False,
        "enable_goals": True,
        "enable_inspection_evidence": True,
        "inspection_evidence_max_rgb_odom_skew_s": 0.2,
    },
    "map": {
        "enable_navigation": False,
        "enable_semantic": False,
        "enable_teleop": True,
        "enable_goals": False,
    },
    "nav": {
        "waypoint_threshold": 0.20,
        "final_waypoint_threshold": 0.35,
        "path_follower_goal_tolerance": 0.20,
        "path_follower_lookahead": 0.35,
        "path_follower_max_speed": 0.50,
        "path_follower_max_accel": 1.0,
        "path_follower_min_speed": 0.08,
        "enable_semantic": False,
    },
    "teleop": {
        **_TELEOP_LIMITS,
        "enable_navigation": False,
        "enable_semantic": False,
        "enable_teleop": True,
        "enable_goals": False,
        "run_startup_checks": False,
    },
    "teleop_avoid": {
        **_TELEOP_LIMITS,
        "enable_navigation": False,
        "enable_semantic": False,
        "enable_teleop": True,
        "enable_goals": False,
    },
    "tracking": {
        "encoder": "none",
        "enable_semantic": True,
        "enable_semantic_planning": False,
        "enable_teleop": False,
        "enable_goals": True,
    },
}

FIELD_PRODUCT_VARIANT_HOST_DEFAULTS: dict[str, dict[str, dict[str, Any]]] = {
    "explore": {
        "live": FIELD_PRODUCT_HOST_DEFAULTS["explore"],
        "map": {
            "map_artifact_gate_required": True,
            "enable_semantic": False,
            "exploration_backend": "tare",
        },
    }
}

if tuple(FIELD_PRODUCT_HOST_DEFAULTS) != FIELD_PRODUCT_NAMES:
    missing = sorted(set(FIELD_PRODUCT_NAMES) - set(FIELD_PRODUCT_HOST_DEFAULTS))
    extra = sorted(set(FIELD_PRODUCT_HOST_DEFAULTS) - set(FIELD_PRODUCT_NAMES))
    raise RuntimeError(
        "Field Product Host defaults must match Runtime Graph exactly "
        f"(missing={missing}, extra={extra})"
    )


__all__ = [
    "FIELD_PRODUCT_HOST_DEFAULTS",
    "FIELD_PRODUCT_NAMES",
    "FIELD_PRODUCT_VARIANT_HOST_DEFAULTS",
]

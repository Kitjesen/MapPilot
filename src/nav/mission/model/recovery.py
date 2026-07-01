"""Mission recovery policy.

Navigation executes the motion, this file only chooses the tiny recovery
sequence from the latest terrain class.
"""

from __future__ import annotations

from typing import Any


RECOVERY_STRATEGIES: dict[str, dict[str, Any]] = {
    "cliff": {
        "strategy": "rotate_only",
        "backup_speed": 0.0,
        "backup_duration": 0.0,
        "rotate_speed": 0.3,
        "rotate_duration": 2.5,
        "forward_speed": 0.0,
        "forward_duration": 0.0,
    },
    "unsafe_forward": {
        "strategy": "rotate_only",
        "backup_speed": 0.0,
        "backup_duration": 0.0,
        "rotate_speed": 0.3,
        "rotate_duration": 2.5,
        "forward_speed": 0.0,
        "forward_duration": 0.0,
    },
    "narrow": {
        "strategy": "short_backup_rotate",
        "backup_speed": -0.15,
        "backup_duration": 0.8,
        "rotate_speed": 0.5,
        "rotate_duration": 0.8,
        "forward_speed": 0.0,
        "forward_duration": 0.0,
    },
    "corridor": {
        "strategy": "short_backup_rotate",
        "backup_speed": -0.15,
        "backup_duration": 0.8,
        "rotate_speed": 0.5,
        "rotate_duration": 0.8,
        "forward_speed": 0.0,
        "forward_duration": 0.0,
    },
    "stuck_in_soft": {
        "strategy": "long_backup_nudge",
        "backup_speed": -0.15,
        "backup_duration": 2.0,
        "rotate_speed": 0.0,
        "rotate_duration": 0.0,
        "forward_speed": 0.15,
        "forward_duration": 0.5,
    },
    "grip_loss": {
        "strategy": "long_backup_nudge",
        "backup_speed": -0.15,
        "backup_duration": 2.0,
        "rotate_speed": 0.0,
        "rotate_duration": 0.0,
        "forward_speed": 0.15,
        "forward_duration": 0.5,
    },
}


DEFAULT_RECOVERY_STRATEGY: dict[str, Any] = {
    "strategy": "default_backup_rotate",
    "backup_speed": -0.2,
    "backup_duration": 1.5,
    "rotate_speed": 0.5,
    "rotate_duration": 1.5,
    "forward_speed": 0.0,
    "forward_duration": 0.0,
}


def recovery_strategy_for(traversability_class: str) -> dict[str, Any]:
    return dict(RECOVERY_STRATEGIES.get(traversability_class, DEFAULT_RECOVERY_STRATEGY))

"""Simulation drivers -- no hardware required."""

import time
from typing import Any


def build_sim_robot_state() -> dict[str, Any]:
    """Return a standard robot-state dict shared by all sim drivers.

    ``MujocoDriverModule`` and ``SimEndpointDriverModule`` publish the
    same robot-state fields. This helper keeps them in sync.
    """
    return {
        "standing": True,
        "enabled": True,
        "emergency": False,
        "connected": True,
        "battery_voltage": 0.0,
        "battery_soc": 0.0,
        "current_gait": "trot",
        "timestamp": time.time(),
    }


from .stub import StubConnection  # noqa: E402

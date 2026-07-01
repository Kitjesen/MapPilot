"""Safety-stop, safety-state, and velocity-arbitration wires."""

from __future__ import annotations

from .context import WiringContext
from .types import WireSpec


def required_safety_stop_specs(
    names: set[str] | frozenset[str],
    *,
    driver_module: str,
) -> tuple[WireSpec, ...]:
    specs = [WireSpec("nav.safety", "stop_cmd", "nav.mission", "stop_signal")]
    if driver_module in names:
        specs.append(WireSpec("nav.safety", "stop_cmd", driver_module, "stop_signal"))
    if "GeofenceManagerModule" in names:
        specs.append(WireSpec("GeofenceManagerModule", "stop_cmd", "nav.mission", "stop_signal"))
        if driver_module in names:
            specs.append(WireSpec("GeofenceManagerModule", "stop_cmd", driver_module, "stop_signal"))
    return tuple(specs)


def safety_status_specs() -> tuple[WireSpec, ...]:
    return (
        WireSpec("nav.safety", "safety_state", "GatewayModule", "safety_state"),
        WireSpec("nav.safety", "safety_state", "MCPServerModule", "safety_state"),
        WireSpec("nav.safety", "execution_eval", "GatewayModule", "execution_eval"),
        WireSpec("nav.safety", "dialogue_state", "GatewayModule", "dialogue_state"),
    )


def cmd_vel_mux_specs(ctx: WiringContext) -> tuple[WireSpec, ...]:
    return (
        WireSpec("TeleopModule", "cmd_vel", "nav.velocity_mux", "teleop_cmd_vel"),
        WireSpec("nav.mission", "recovery_cmd_vel", "nav.velocity_mux", "recovery_cmd_vel"),
        WireSpec("nav.path_follower", "cmd_vel", "nav.velocity_mux", "path_follower_cmd_vel"),
        WireSpec("nav.velocity_mux", "driver_cmd_vel", ctx.driver_module, "cmd_vel"),
        WireSpec("nav.velocity_mux", "driver_cmd_vel", "nav.safety", "cmd_vel"),
    )

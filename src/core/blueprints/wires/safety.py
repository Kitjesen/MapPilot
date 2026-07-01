"""Safety-stop, safety-state, and velocity-arbitration wires."""

from __future__ import annotations

from .context import WiringContext
from .types import WireSpec


def required_safety_stop_specs(
    names: set[str] | frozenset[str],
    *,
    driver_module: str,
) -> tuple[WireSpec, ...]:
    specs = [WireSpec("SafetyRingModule", "stop_cmd", "NavigationModule", "stop_signal")]
    if driver_module in names:
        specs.append(WireSpec("SafetyRingModule", "stop_cmd", driver_module, "stop_signal"))
    if "GeofenceManagerModule" in names:
        specs.append(WireSpec("GeofenceManagerModule", "stop_cmd", "NavigationModule", "stop_signal"))
        if driver_module in names:
            specs.append(WireSpec("GeofenceManagerModule", "stop_cmd", driver_module, "stop_signal"))
    return tuple(specs)


def safety_status_specs() -> tuple[WireSpec, ...]:
    return (
        WireSpec("SafetyRingModule", "safety_state", "GatewayModule", "safety_state"),
        WireSpec("SafetyRingModule", "safety_state", "MCPServerModule", "safety_state"),
        WireSpec("SafetyRingModule", "execution_eval", "GatewayModule", "execution_eval"),
        WireSpec("SafetyRingModule", "dialogue_state", "GatewayModule", "dialogue_state"),
    )


def cmd_vel_mux_specs(ctx: WiringContext) -> tuple[WireSpec, ...]:
    return (
        WireSpec("TeleopModule", "cmd_vel", "CmdVelMux", "teleop_cmd_vel"),
        WireSpec("NavigationModule", "recovery_cmd_vel", "CmdVelMux", "recovery_cmd_vel"),
        WireSpec("PathFollowerModule", "cmd_vel", "CmdVelMux", "path_follower_cmd_vel"),
        WireSpec("CmdVelMux", "driver_cmd_vel", ctx.driver_module, "cmd_vel"),
        WireSpec("CmdVelMux", "driver_cmd_vel", "SafetyRingModule", "cmd_vel"),
        WireSpec("CmdVelMux", "driver_cmd_vel", "EndpointPathBridgeModule", "cmd_vel"),
    )

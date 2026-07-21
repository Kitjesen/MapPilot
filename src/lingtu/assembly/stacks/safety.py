"""Safety stack: SafetyRing + Geofence."""

from __future__ import annotations

from runtime.blueprint import Blueprint
from runtime.plugin_resolution import optional_stack_module, stack_module


def safety(
    *,
    enable_cmd_vel_mux: bool = True,
    cmd_vel_mux_source_timeout: float | None = None,
    enable_collision_monitor: bool = False,
    collision_monitor_timeout_s: float | None = None,
    collision_monitor_horizon_s: float | None = None,
    collision_monitor_step_s: float | None = None,
    collision_monitor_stop_cost: float | None = None,
    collision_monitor_slow_cost: float | None = None,
    collision_monitor_slowdown_scale: float | None = None,
) -> Blueprint:
    """Safety reflex + geofence boundary enforcement + cmd_vel arbitration."""
    bp = Blueprint()

    SafetyRing = stack_module(
        "safety",
        "ring",
        seed_group="safety",
        fallback="nav.services.safety.safety_ring.SafetyRing",
    )
    bp.add(SafetyRing, alias="nav.safety")

    if enable_cmd_vel_mux:
        VelocityMux = stack_module(
            "safety",
            "cmd_vel_mux",
            seed_group="safety",
            fallback="nav.services.safety.velocity_mux.VelocityMux",
        )
        mux_kwargs = {}
        if cmd_vel_mux_source_timeout is not None:
            mux_kwargs["source_timeout"] = float(cmd_vel_mux_source_timeout)
        mux_kwargs["enable_collision_monitor"] = bool(enable_collision_monitor)
        for key, value in {
            "collision_monitor_timeout_s": collision_monitor_timeout_s,
            "collision_monitor_horizon_s": collision_monitor_horizon_s,
            "collision_monitor_step_s": collision_monitor_step_s,
            "collision_monitor_stop_cost": collision_monitor_stop_cost,
            "collision_monitor_slow_cost": collision_monitor_slow_cost,
            "collision_monitor_slowdown_scale": collision_monitor_slowdown_scale,
        }.items():
            if value is not None:
                mux_kwargs[key] = float(value)
        bp.add(VelocityMux, alias="nav.velocity_mux", **mux_kwargs)

    GeofenceManagerModule = optional_stack_module(
        "safety",
        "geofence",
        seed_group="safety",
        fallback="nav.services.geofence.GeofenceManagerModule",
    )
    if GeofenceManagerModule is not None:
        bp.add(GeofenceManagerModule, alias="GeofenceManagerModule")

    return bp

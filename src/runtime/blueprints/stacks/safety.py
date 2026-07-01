"""Safety stack: SafetyRing + Geofence."""

from __future__ import annotations

from runtime.blueprint import Blueprint
from runtime.blueprints.stacks._registry import optional_stack_module, stack_module


def safety(*, cmd_vel_mux_source_timeout: float | None = None) -> Blueprint:
    """Safety reflex + geofence boundary enforcement + cmd_vel arbitration."""
    bp = Blueprint()

    SafetyRing = stack_module(
        "safety",
        "ring",
        seed_group="safety",
        fallback="nav.services.safety.safety_ring.SafetyRing",
    )
    bp.add(SafetyRing, alias="nav.safety")

    VelocityMux = stack_module(
        "safety",
        "cmd_vel_mux",
        seed_group="safety",
        fallback="nav.services.safety.velocity_mux.VelocityMux",
    )
    mux_kwargs = {}
    if cmd_vel_mux_source_timeout is not None:
        mux_kwargs["source_timeout"] = float(cmd_vel_mux_source_timeout)
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

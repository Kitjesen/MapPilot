"""Localization-status normalization and forwarding."""

from __future__ import annotations

import time
from typing import Any

from gateway.services.runtime_status import backend_capability_defaults


def handle_localization_status(gw: Any, state: dict[str, Any]) -> None:
    """Normalize localization diagnostics and forward them to SSE."""
    if not isinstance(state, dict):
        return
    data = dict(state)
    profile = gw._slam_profile_from_status(data) or gw._get_slam_profile()
    if profile and profile != "unknown" and not data.get("backend"):
        data["backend"] = profile
    capability_defaults = backend_capability_defaults(profile)
    if profile in {"genz", "super_lio", "super_lio_relocation"}:
        data.setdefault("health_source", "odom_map_cloud")
        data["relocalization_supported"] = False
        data["saved_map_relocalization_supported"] = False
        data.setdefault("relocalization_state", "unsupported")
        data["map_save_supported"] = profile == "super_lio"
        if profile == "super_lio":
            data.setdefault("map_save_source", "live_map_cloud_snapshot")
        elif profile == "super_lio_relocation":
            data.setdefault("map_save_source", "active_map")
        if "map_state" not in data:
            data["map_state"] = (
                ("live_map_cloud" if profile in {"genz", "super_lio"} else "relocation_map_cloud")
                if data.get("map_cloud_fresh")
                else "map_cloud_stale"
            )
        if "localizer_health" not in data:
            state_name = str(data.get("state", "") or "").upper()
            if state_name in {"LOST", "UNINIT", "UNINITIALIZED"}:
                data["localizer_health"] = "LIO_LOST"
            elif state_name in {"DIVERGED"}:
                data["localizer_health"] = "LIO_DIVERGED"
            elif state_name in {"DEGRADED", "FALLBACK_GNSS_ONLY"}:
                data["localizer_health"] = "LIO_DEGRADED"
            else:
                data["localizer_health"] = "LIO_TRACKING"
    elif profile == "localizer":
        data.setdefault("relocalization_supported", True)
        data.setdefault("saved_map_relocalization_supported", True)
    elif profile in {"fastlio2", "slam"}:
        data.setdefault("relocalization_supported", False)
        data.setdefault("saved_map_relocalization_supported", False)
    data.setdefault(
        "saved_map_relocalization_supported",
        data.get(
            "relocalization_supported",
            capability_defaults["saved_map_relocalization_supported"],
        ),
    )
    data.setdefault(
        "restart_recovery_supported",
        capability_defaults["restart_recovery_supported"],
    )
    data.setdefault("recovery_method", capability_defaults["recovery_method"])
    data["_gateway_received_ts"] = time.time()
    data["_gateway_received_mono"] = time.monotonic()

    with gw._state_lock:
        previous = dict(getattr(gw, "_localization_status", None) or {})
    previous_runtime = str(previous.get("runtime_instance_id") or "").strip()
    current_runtime = str(data.get("runtime_instance_id") or "").strip()
    runtime_changed = bool(previous_runtime and current_runtime and previous_runtime != current_runtime)
    try:
        previous_sequence = int(previous.get("observation_sequence"))
        current_sequence = int(data.get("observation_sequence"))
        sequence_rolled_back = current_sequence < previous_sequence
    except (TypeError, ValueError):
        sequence_rolled_back = False
    try:
        previous_jump_sequence = int(previous.get("map_frame_jump_sequence"))
        current_jump_sequence = int(data.get("map_frame_jump_sequence"))
        jump_sequence_advanced = current_jump_sequence > previous_jump_sequence
    except (TypeError, ValueError):
        jump_sequence_advanced = False
    map_frame_jumped = jump_sequence_advanced or (
        bool(data.get("map_frame_jump")) and not bool(previous.get("map_frame_jump"))
    )
    reset_reason = None
    if runtime_changed:
        reset_reason = "slam_runtime_changed"
    elif sequence_rolled_back:
        reset_reason = "slam_observation_sequence_rollback"
    elif map_frame_jumped:
        reset_reason = "slam_map_frame_jump"
    if reset_reason is not None:
        clear_cache = getattr(gw, "clear_map_cloud_cache", None)
        if callable(clear_cache):
            clear_cache(reason=reset_reason)
        data["viewer_epoch_reset_reason"] = reset_reason

    with gw._state_lock:
        gw._session_runtime.set_localization_status(data)
    gw._blackbox.record("slam_diag", data)
    gw.push_event({"type": "slam_diag", "data": data})

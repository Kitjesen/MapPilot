"""Expose the native planner's static eligibility layer for the current map."""

from __future__ import annotations

import contextlib
import math
from typing import Any

from gateway.services.native_control import status_is_fresh
from gateway.services.native_status import read_navigation_status, read_planning_map
from gateway.services.runtime_status import safe_session


def planning_map_view(snapshot, native, state, session, *, now_s=None) -> dict[str, Any]:
    def unavailable(reason):
        return {"available": False, "reason": reason, "cells": [], "rows": 0, "cols": 0}

    if not status_is_fresh(native, now_s=now_s) or not status_is_fresh(
        {"stamp_s": state.get("ts")}, now_s=now_s
    ):
        return unavailable("navigation_status_stale")
    if not snapshot:
        return unavailable("planning_map_pending")
    session_id = session.get("product_session_id")
    if not session_id or snapshot.get("product_session_id") != session_id or (
        native.get("native_product", {}).get("product_session_id") != session_id
    ):
        return unavailable("planning_map_session_mismatch")
    if not session.get("active_map") or snapshot.get("map_id") != session["active_map"] or (
        snapshot.get("map_id") != state.get("map_id")
        or snapshot.get("map_content_epoch") != state.get("map_content_epoch")
    ):
        return unavailable("planning_map_identity_mismatch")
    if snapshot.get("available") is not True:
        return unavailable(str(snapshot.get("reason") or "planning_map_pending"))
    rows, cols = snapshot.get("rows"), snapshot.get("cols")
    cells, origin = snapshot.get("cells"), snapshot.get("origin")
    resolution = snapshot.get("resolution")
    def valid_number(value):
        return isinstance(value, (int, float)) and math.isfinite(value)
    if (snapshot.get("schema_version") != 1 or snapshot.get("frame_id") != "map"
        or type(rows) is not int or type(cols) is not int or rows <= 0 or cols <= 0
        or not isinstance(cells, list) or len(cells) != rows * cols
        or any(type(v) is not int or v not in (0, 1, 2) for v in cells)
        or not isinstance(origin, list) or len(origin) != 3 or not all(map(valid_number, origin))
        or not valid_number(resolution) or resolution <= 0
        or not valid_number(snapshot.get("reference_z"))):
        return unavailable("planning_map_invalid")
    # A saved map does not expire by creation time. Live session/map identity
    # above determines whether this layer still belongs to the current planner.
    return snapshot


def build_planning_map_view(gw) -> dict[str, Any]:
    with getattr(gw, "_state_lock", contextlib.nullcontext()):
        state = dict(getattr(gw, "_navigation_state", None) or {})
    return planning_map_view(
        read_planning_map(), read_navigation_status(), state, safe_session(gw)
    )

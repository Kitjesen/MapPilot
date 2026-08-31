"""Read-only Product session route."""

from __future__ import annotations

import logging
from typing import Any

from gateway.schemas import SessionResponse
from gateway.services.mapd_transport import mapd_query
from gateway.services.native_control import endpoint_only_enabled
from lingtu.products import product_name

logger = logging.getLogger(__name__)


def _current_runtime_product(gw: Any) -> str | None:
    plan = getattr(gw, "_compiled_run_plan", None)
    raw_product = (
        getattr(plan, "product", None)
        if plan is not None
        else getattr(gw, "_compiled_product", None)
    )
    product = str(raw_product or "").strip()
    if not product:
        return None
    try:
        return product_name(product)
    except ValueError:
        return None


def _externally_owned_product(gw: Any) -> str | None:
    plan = getattr(gw, "_compiled_run_plan", None)
    if plan is not None:
        if str(getattr(plan, "process_control", "") or "").strip() != "systemd":
            return None
        return _current_runtime_product(gw) or ""
    try:
        return (_current_runtime_product(gw) or "") if endpoint_only_enabled(gw) else None
    except ValueError:
        return _current_runtime_product(gw) or ""


def _maps_active_name(gw: Any) -> str:
    try:
        response = mapd_query(gw, {"action": "get_active_map"})
    except Exception as exc:
        logger.warning("mapd active-map query failed: %s", exc)
        return ""
    if response.get("success") is True:
        return str(response.get("active") or "").strip()
    return ""


def _external_product_map_guard(
    gw: Any,
    *,
    current_product: str,
    requested_map: str,
) -> tuple[str, dict[str, Any] | None]:
    """Validate a field map against native mapd's active map."""

    active_map = _maps_active_name(gw)
    resolved_map = active_map if requested_map == "active" else requested_map
    if resolved_map and active_map == resolved_map:
        return resolved_map, None
    return resolved_map, {
        "reason_code": "product_map_switch_required",
        "current_product": current_product,
        "current_map": active_map or None,
        "requested_map": resolved_map or requested_map or None,
    }


def register_session_routes(app, gw) -> None:
    """Register the read-only Product session view."""

    @app.get(
        "/api/v1/session",
        summary="Current Product session state and capabilities",
        response_model=SessionResponse,
    )
    async def session_get():
        return gw._session_snapshot()

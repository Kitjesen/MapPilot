"""Session lifecycle routes for GatewayModule."""

from __future__ import annotations

import hmac
import logging
import os
import time
from collections.abc import Mapping
from typing import Any

from fastapi import Request
from fastapi.responses import JSONResponse

from gateway.schemas import (
    SessionResponse,
    SessionStartRequest,
    SessionTransitionResponse,
)
from gateway.services.control_commands import ControlCommandService
from gateway.services.exploration import (
    ExplorationRunError,
    product_control_owns_explore,
    start_exploration_run,
)
from gateway.services.map_service import (
    ensure_maps_service,
    map_runtime_lock,
    map_service_command,
    map_service_query,
)
from gateway.services.native_control import (
    endpoint_only_enabled,
)
from gateway.services.native_control import (
    read_status as read_native_control_status,
)
from gateway.services.native_control import (
    status_is_fresh as native_control_status_is_fresh,
)
from gateway.services.runtime_switch_plan import build_operator_command
from maps.services.storage import safe_map_name
from runtime.contracts.product_runtime import (
    PRODUCT_CONTROL_SESSION_HEADER,
    PRODUCT_SESSION_ID_ENV,
)
from runtime.profiles.product_lifecycle import product_name
from runtime.runtime_policy import (
    default_slam_profile_for_mode,
    is_supported_slam_profile,
    normalize_slam_profile,
)

logger = logging.getLogger(__name__)


def _transition_payload(
    success: bool,
    *,
    session: dict[str, Any] | None = None,
    message: str | None = None,
    detail: dict[str, Any] | None = None,
) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "schema_version": 1,
        "ok": bool(success),
        "success": bool(success),
        "ts": time.time(),
    }
    if session is not None:
        payload["session"] = session
    if message is not None:
        payload["message"] = message
    if detail is not None:
        payload["detail"] = detail
    return payload


def _transition_response(
    success: bool,
    *,
    status_code: int,
    session: dict[str, Any] | None = None,
    message: str | None = None,
    detail: dict[str, Any] | None = None,
) -> JSONResponse:
    return JSONResponse(
        _transition_payload(
            success,
            session=session,
            message=message,
            detail=detail,
        ),
        status_code=status_code,
    )


def _body_mapping(body: Any) -> dict[str, Any]:
    """Normalise request body to a plain dict.

    Why raw dicts are accepted:
      Pydantic models define fixed fields, but ROS2 frontends / WebSocket
      messages send JSON with backend-variant keys (e.g. "map_name" vs.
      "map", "slam_profile" vs. "slam_backend"). Accepting raw dicts avoids
      per-endpoint model proliferation and keeps route handlers flexible.

    Trade-off:
      Pydantic type coercion (float, int, str trim) is bypassed at the
      boundary. Call sites MUST manually coerce numeric fields with
      float() / int() — see slam_relocalize (x, y, yaw) and bag_start
      (duration) for examples.
    """
    if hasattr(body, "model_dump"):
        return body.model_dump(exclude_none=True)
    assert isinstance(body, dict), f"expected dict or Pydantic model, got {type(body).__name__}"
    return body


def _clear_gateway_map_cloud(gw: Any, reason: str) -> None:
    clear_cache = getattr(gw, "clear_map_cloud_cache", None)
    if callable(clear_cache):
        clear_cache(reason=reason)
        return
    raise RuntimeError("gateway map cloud cache service is unavailable")


def _normalize_slam_profile(profile: str) -> str:
    return normalize_slam_profile(profile)


def _prospective_lifecycle_catalog() -> dict[str, Mapping[str, Any]]:
    """Return current Product declarations for requests without a RunPlan."""

    from runtime.graph import load_runtime_graph

    return {
        name: spec for name, spec in load_runtime_graph().products.items() if spec.get("operator_switchable") is True
    }


_PROSPECTIVE_PRODUCT_LIFECYCLES = _prospective_lifecycle_catalog()
_DEFAULT_PRODUCT_IDENTITY_BY_MODE = {
    "idle": (None, "idle"),
    **{
        str(lifecycle.get("session_mode") or ""): (
            product,
            str(lifecycle.get("product_session") or ""),
        )
        for product, lifecycle in _PROSPECTIVE_PRODUCT_LIFECYCLES.items()
        if lifecycle.get("default_for_session_mode") is True
    },
}

_REQUIRED_PRODUCTS_BY_SESSION_MODE = {
    mode: tuple(
        product
        for product, lifecycle in _PROSPECTIVE_PRODUCT_LIFECYCLES.items()
        if lifecycle.get("session_mode") == mode
    )
    for mode in sorted(
        {
            str(lifecycle.get("session_mode") or "")
            for lifecycle in _PROSPECTIVE_PRODUCT_LIFECYCLES.values()
            if lifecycle.get("session_mode") != "none"
        }
    )
}


def _default_product_identity_for_mode(mode: str) -> tuple[str | None, str]:
    return _DEFAULT_PRODUCT_IDENTITY_BY_MODE.get(
        str(mode or "").strip().lower(),
        (None, str(mode or "unknown").strip().lower() or "unknown"),
    )


def _normalize_product_identity(
    gw: Any,
    payload: dict[str, Any],
    mode: str,
) -> tuple[str | None, str]:
    raw_product = str(payload.get("product") or payload.get("profile") or "").strip()
    product = product_name(raw_product) if raw_product else None
    explicit_session = str(payload.get("product_session") or "").strip().lower()
    default_product, mode_default_session = _default_product_identity_for_mode(mode)

    lifecycle = _lifecycle_for_product(gw, product)
    if lifecycle is not None:
        default_session = str(lifecycle.get("product_session") or "").strip()
    else:
        default_session = mode_default_session
        if product is None:
            product = default_product

    return product, explicit_session or default_session


def _runtime_lifecycle(gw: Any) -> Mapping[str, Any] | None:
    plan = getattr(gw, "_compiled_run_plan", None)
    if plan is None:
        return None
    lifecycle = getattr(plan, "lifecycle", None)
    if not isinstance(lifecycle, Mapping):
        raise RuntimeError("compiled RunPlan is missing lifecycle")
    product = str(getattr(plan, "product", "") or "").strip()
    if str(lifecycle.get("product") or "").strip() != product:
        raise RuntimeError("compiled RunPlan lifecycle Product mismatch")
    return lifecycle


def _lifecycle_for_product(
    gw: Any,
    product: str | None,
) -> Mapping[str, Any] | None:
    runtime_lifecycle = _runtime_lifecycle(gw)
    if runtime_lifecycle is not None:
        runtime_product = str(runtime_lifecycle.get("product") or "").strip()
        if product is None or product == runtime_product:
            return runtime_lifecycle
    return _PROSPECTIVE_PRODUCT_LIFECYCLES.get(product or "")


def _gateway_maps_service(gw: Any) -> Any | None:
    try:
        return ensure_maps_service(gw)
    except RuntimeError as exc:
        logger.warning("maps.service lookup failed: %s", exc)
        return None


def _maps_get_bundle(manager: Any, map_name: str, capability: str) -> dict[str, Any]:
    return map_service_query(
        manager,
        {
            "action": "get_map_bundle",
            "name": map_name,
            "capability": capability,
        },
    )


def _current_runtime_product(gw: Any) -> str | None:
    plan = getattr(gw, "_compiled_run_plan", None)
    raw_product = (getattr(plan, "product", None) if plan is not None else getattr(gw, "_session_product", None)) or ""
    product = str(raw_product).strip()
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
        uses_native_endpoint = endpoint_only_enabled(gw)
    except ValueError:
        uses_native_endpoint = True
    if not uses_native_endpoint:
        return None
    return _current_runtime_product(gw) or ""


def _session_end_control_blocker(
    gw: Any,
    request: Request | None,
) -> dict[str, Any] | None:
    """Require the boot-scoped ProductControl credential for field sessions."""

    current_product = _externally_owned_product(gw)
    if current_product is None or request is None:
        return None
    expected = str(os.environ.get(PRODUCT_SESSION_ID_ENV) or "").strip()
    provided = str(request.headers.get(PRODUCT_CONTROL_SESSION_HEADER) or "").strip()
    env = str(getattr(getattr(gw, "_compiled_run_plan", None), "env", "") or "real")
    detail = {
        "reason_code": "operator_product_control_required",
        "current_product": current_product or None,
        "operator_command": f"python -m lingtu.control stop-session --env {env}",
    }
    if not expected:
        detail["control_session"] = "unavailable"
        return detail
    if not provided or not hmac.compare_digest(provided, expected):
        detail["control_session"] = "unauthorized"
        return detail
    return None


def _switch_guidance(
    gw: Any,
    *,
    target_product: str | None,
    current_product: str | None = None,
    map_name: str | None = None,
) -> dict[str, Any]:
    """Return the read-only preview link and exact ProductControl command."""

    detail: dict[str, Any] = {
        "switch_plan": "/api/v1/runtime/switch-plan",
    }
    if target_product is None:
        return detail
    env = str(getattr(getattr(gw, "_compiled_run_plan", None), "env", "") or "real")
    request: dict[str, Any] = {
        "target_product": target_product,
        "relocalize": True,
    }
    if current_product:
        request["current_product"] = current_product
    if map_name:
        request["map_name"] = map_name
    detail["operator_command"] = build_operator_command(request, env=env)
    return detail


def _external_product_mode_guard(
    gw: Any,
    mode: str,
    *,
    requested_product: str | None,
) -> dict[str, Any] | None:
    """Reject low-level sessions that would cross an externally-owned product graph."""

    current_product = _externally_owned_product(gw)
    if current_product is None:
        return None
    if requested_product is not None and requested_product != current_product:
        return {
            "reason_code": "product_mismatch",
            "current_product": current_product,
            "requested_product": requested_product,
            **_switch_guidance(
                gw,
                target_product=requested_product,
                current_product=current_product,
            ),
        }

    lifecycle = _lifecycle_for_product(gw, current_product)
    runtime_session_mode = str((lifecycle or {}).get("session_mode") or "").strip()
    required_products = _REQUIRED_PRODUCTS_BY_SESSION_MODE.get(mode, ())
    if runtime_session_mode != mode:
        target_product = required_products[0] if len(required_products) == 1 else requested_product
        return {
            "reason_code": "product_mode_switch_required",
            "current_product": current_product or None,
            "required_products": list(required_products),
            **_switch_guidance(
                gw,
                target_product=target_product,
                current_product=current_product or None,
            ),
        }

    expected_control_mode = str((lifecycle or {}).get("native_control_mode") or "").strip()
    status = read_native_control_status()
    actual_control_mode = str(status.get("control_mode") or "").strip().lower() if isinstance(status, dict) else ""
    if expected_control_mode and (
        not native_control_status_is_fresh(status) or actual_control_mode != expected_control_mode
    ):
        return {
            "reason_code": "native_control_mode_not_ready",
            "current_product": current_product,
            "expected_control_mode": expected_control_mode,
            "actual_control_mode": actual_control_mode or None,
            "control_status_fresh": native_control_status_is_fresh(status),
            "operator_command": (
                "python -m lingtu.control reapply "
                f"--env {str(getattr(getattr(gw, '_compiled_run_plan', None), 'env', '') or 'real')}"
            ),
        }
    return None


def _external_product_map_guard(
    gw: Any,
    *,
    current_product: str,
    requested_map: str,
) -> tuple[str, dict[str, Any] | None]:
    """Validate a field map from read-only active-map evidence."""

    manager = _gateway_maps_service(gw)
    active_map = _maps_active_name(manager) if manager is not None else ""
    resolved_map = active_map if requested_map == "active" else requested_map
    if resolved_map and active_map == resolved_map:
        return resolved_map, None
    return resolved_map, {
        "reason_code": "product_map_switch_required",
        "current_product": current_product,
        "current_map": active_map or None,
        "requested_map": resolved_map or requested_map or None,
        **_switch_guidance(
            gw,
            target_product=current_product,
            current_product=current_product,
            map_name=resolved_map or requested_map or None,
        ),
    }


def _external_same_session_active(
    gw: Any,
    *,
    mode: str,
    product: str | None,
    product_session: str,
    map_name: str,
) -> bool:
    """Return whether an externally-owned runtime is already in this session."""

    if not _externally_owned_product(gw):
        return False
    if gw._session_mode != mode:
        return False
    if (gw._session_product or None) != (product or None):
        return False
    if str(gw._session_product_session or "").strip().lower() != product_session:
        return False
    if mode == "navigating":
        return str(gw._session_map or "").strip() == str(map_name or "").strip()
    return True


def _maps_set_active(manager: Any, map_name: str) -> dict[str, Any]:
    return map_service_command(manager, {"action": "set_active", "name": map_name})


def _maps_active_name(manager: Any) -> str:
    response = map_service_query(manager, {"action": "get_active"})
    if isinstance(response, dict) and response.get("success") is True:
        return str(response.get("active") or "").strip()
    return ""


def _bundle_artifact_path(bundle: dict[str, Any]) -> str:
    artifact = bundle.get("artifact") if isinstance(bundle.get("artifact"), dict) else {}
    raw_uri = str(artifact.get("uri") or artifact.get("path") or "")
    if not raw_uri:
        return ""
    if os.path.isabs(raw_uri):
        return raw_uri
    base = str(bundle.get("map_dir") or "")
    return str(os.path.join(base, raw_uri) if base else raw_uri)


def _activate_session_map_via_maps_service(
    gw: Any,
    map_name: str,
) -> tuple[bool, str, dict[str, Any], str]:
    manager = _gateway_maps_service(gw)
    if manager is None:
        return (
            False,
            "maps.service is unavailable",
            {"reason_code": "maps_service_unavailable"},
            "",
        )
    previous_active = _maps_active_name(manager)
    bundle = _maps_get_bundle(manager, map_name, "navigation_safety_3d")
    if bundle.get("success") is not True:
        return False, str(bundle.get("message") or "navigation_safety_3d bundle unavailable"), bundle, ""
    active = _maps_set_active(manager, map_name)
    if active.get("success") is not True:
        return False, str(active.get("message") or "failed to activate map"), active, ""
    map_path = (
        str(active.get("octomap") or "")
        or _bundle_artifact_path(bundle)
        or str(active.get("occupancy") or "")
        or str(active.get("pcd") or "")
    )
    return (
        True,
        "",
        {
            "bundle": bundle,
            "activation": active,
            "previous_active": previous_active,
        },
        map_path,
    )


def _rollback_session_map_activation(
    gw: Any,
    activation_detail: dict[str, Any],
    reload_planner_map: Any,
) -> dict[str, Any]:
    previous_active = str(activation_detail.get("previous_active") or "").strip()
    activation = activation_detail.get("activation")
    activated_map = str(activation.get("active") if isinstance(activation, Mapping) else "").strip()
    if not previous_active or previous_active == activated_map:
        rollback = {
            "success": previous_active == activated_map,
            "skipped": True,
            "reason": "no_distinct_previous_active_map",
        }
        activation_detail["rollback"] = rollback
        return rollback

    manager = _gateway_maps_service(gw)
    if manager is None:
        rollback = {
            "success": False,
            "reason": "maps_service_unavailable",
        }
        activation_detail["rollback"] = rollback
        return rollback

    rollback = _maps_set_active(manager, previous_active)
    if rollback.get("success") is True and callable(reload_planner_map):
        previous_bundle = _maps_get_bundle(
            manager,
            previous_active,
            "navigation_safety_3d",
        )
        previous_path = _bundle_artifact_path(previous_bundle)
        if previous_bundle.get("success") is True and previous_path:
            try:
                rollback["planner_reload"] = reload_planner_map(previous_path)
            except Exception as exc:
                rollback["planner_reload"] = {
                    "ok": False,
                    "reason": "planner_rollback_failed",
                    "message": str(exc),
                }
    activation_detail["rollback"] = rollback
    return rollback


def register_session_routes(app, gw) -> None:
    command_service = ControlCommandService(gw)

    @app.get(
        "/api/v1/session",
        summary="Current session state + capabilities",
        response_model=SessionResponse,
    )
    async def session_get():
        inferred_mode, inferred_map = gw._session_detect_current_mode()
        if inferred_mode != gw._session_mode:
            product, product_session = _default_product_identity_for_mode(inferred_mode)
            gw._session_mode = inferred_mode
            gw._session_product = product
            gw._session_product_session = product_session
            gw._session_map = inferred_map
            gw._session_since = time.time()
        return gw._session_snapshot()

    @app.post(
        "/api/v1/session/start",
        summary="Start a local session or verify the active field Product session",
        response_model=SessionTransitionResponse,
        responses={
            400: {"model": SessionTransitionResponse},
            409: {"model": SessionTransitionResponse},
            500: {"model": SessionTransitionResponse},
            503: {"model": SessionTransitionResponse},
        },
    )
    async def session_start(body: SessionStartRequest):
        payload = _body_mapping(body)
        mode = (payload.get("mode") or "").strip().lower()
        start_task = bool(payload.get("start_task", True))
        map_name = payload.get("map_name") or payload.get("map") or ""
        slam_profile = (payload.get("slam_profile") or payload.get("slam_backend") or "").strip().lower()
        slam_profile = _normalize_slam_profile(slam_profile)
        if mode not in ("mapping", "navigating", "exploring"):
            return _transition_response(
                False,
                status_code=400,
                message=(f"Unknown mode: {mode!r}. Use 'mapping' | 'navigating' | 'exploring'."),
            )
        raw_requested_product = str(payload.get("product") or payload.get("profile") or "").strip()
        requested_product = product_name(raw_requested_product) if raw_requested_product else None
        product_mode_blocker = _external_product_mode_guard(
            gw,
            mode,
            requested_product=requested_product,
        )
        if product_mode_blocker is not None:
            return _transition_response(
                False,
                status_code=409,
                message=(
                    "Low-level session start cannot switch the externally-owned "
                    "Product. Preview the change with /api/v1/runtime/switch-plan, "
                    "then run the ProductControl command returned in detail."
                ),
                detail=product_mode_blocker,
            )
        external_product = _externally_owned_product(gw)
        if external_product and requested_product is None:
            payload["product"] = external_product
        product, product_session = _normalize_product_identity(
            gw,
            payload,
            mode,
        )
        if slam_profile and not is_supported_slam_profile(slam_profile):
            return _transition_response(
                False,
                status_code=400,
                message=(
                    f"Unknown slam_profile: {slam_profile!r}. "
                    "Use 'none' | 'native_dds' | 'fastlio2' | 'genz' | 'localizer'."
                ),
            )
        if map_name:
            err = safe_map_name(map_name)
            if err is not None:
                return _transition_response(False, status_code=400, message=err)
        lifecycle = _lifecycle_for_product(gw, product)
        if external_product and mode == "navigating" and (lifecycle is None or lifecycle.get("requires_map") is True):
            map_name, product_map_blocker = _external_product_map_guard(
                gw,
                current_product=external_product,
                requested_map=str(map_name or ""),
            )
            if product_map_blocker is not None:
                return _transition_response(
                    False,
                    status_code=409,
                    message=(
                        "The active Product is bound to a different map. Preview "
                        "the change with /api/v1/runtime/switch-plan, then run the "
                        "ProductControl command returned in detail."
                    ),
                    detail=product_map_blocker,
                )
        if _external_same_session_active(
            gw,
            mode=mode,
            product=product,
            product_session=product_session,
            map_name=str(map_name or ""),
        ):
            if mode == "exploring" and start_task and product_control_owns_explore(gw):
                try:
                    run = start_exploration_run(
                        gw,
                        request_id=str(payload.get("request_id") or "") or None,
                    )
                except ExplorationRunError as exc:
                    return _transition_response(
                        False,
                        status_code=exc.status_code,
                        session=gw._session_snapshot(),
                        message=str(exc),
                        detail={"error": exc.code, **exc.detail},
                    )
                return _transition_payload(
                    True,
                    session=gw._session_snapshot(),
                    detail={"exploration_run": run},
                )
            return _transition_payload(True, session=gw._session_snapshot())
        if mode == "exploring" and start_task and not gw._explorer_available():
            return _transition_response(
                False,
                status_code=503,
                message=("Exploration backend not running - switch to the 'explore' Product."),
            )
        if gw._session_mode != "idle":
            return _transition_response(
                False,
                status_code=409,
                message=(f"Already in {gw._session_mode}. Call /session/end first."),
            )
        if gw._session_pending:
            return _transition_response(
                False,
                status_code=409,
                message="Another transition in progress",
            )
        if mode == "exploring" and start_task:
            readiness = gw._exploration_start_readiness()
            if not readiness.get("can_start", False):
                blockers = readiness.get("blockers") or ["navigation_not_ready"]
                return _transition_response(
                    False,
                    status_code=409,
                    message=(
                        "Exploration cannot start until navigation readiness "
                        f"blockers clear: {', '.join(map(str, blockers))}"
                    ),
                    detail=readiness,
                )

        with map_runtime_lock():
            if gw._session_mode != "idle":
                return _transition_response(
                    False,
                    status_code=409,
                    message=f"Already in {gw._session_mode}. Call /session/end first.",
                )
            if gw._session_pending:
                return _transition_response(
                    False,
                    status_code=409,
                    message="Another transition in progress",
                )
            gw._session_pending = True
        gw._session_error = ""
        try:
            if mode == "navigating":
                requires_map = lifecycle.get("requires_map") is True if lifecycle is not None else True
                if requires_map and not map_name:
                    return _transition_response(
                        False,
                        status_code=400,
                        message="map_name is required for navigating",
                    )

                map_err = safe_map_name(map_name) if map_name else None
                if map_err is not None:
                    return _transition_response(False, status_code=400, message=map_err)
                if map_name == "active":
                    manager = _gateway_maps_service(gw)
                    map_name = _maps_active_name(manager) if manager is not None else ""
                    if not map_name:
                        return _transition_response(
                            False,
                            status_code=409,
                            message="no active map is selected",
                        )
                with map_runtime_lock():
                    if map_name:
                        if external_product:
                            map_name, product_map_blocker = _external_product_map_guard(
                                gw,
                                current_product=external_product,
                                requested_map=map_name,
                            )
                            if product_map_blocker is not None:
                                return _transition_response(
                                    False,
                                    status_code=409,
                                    message=(
                                        "The active Product is bound to a different map. "
                                        "Preview the change with /api/v1/runtime/switch-plan, "
                                        "then run the ProductControl command returned in detail."
                                    ),
                                    detail=product_map_blocker,
                                )
                        else:
                            (
                                map_ready,
                                activation_message,
                                activation_detail,
                                map_path,
                            ) = _activate_session_map_via_maps_service(gw, map_name)
                            if map_ready is not True:
                                status_code = 400
                                missing_capability = (
                                    isinstance(activation_detail, dict)
                                    and activation_detail.get("reason_code") == "missing_capability"
                                )
                                if missing_capability:
                                    status_code = 409
                                elif "artifact gate failed" in activation_message:
                                    status_code = 409
                                return _transition_response(
                                    False,
                                    status_code=status_code,
                                    message=activation_message or "map activation failed",
                                    detail={"map_activation": activation_detail},
                                )
                            reload_planner_map = command_service.reload_navigation_map
                            try:
                                planner_reload = reload_planner_map(map_path)
                            except Exception as exc:
                                logger.warning(
                                    "planner map reload after session map activation failed: %s",
                                    exc,
                                )
                                _rollback_session_map_activation(
                                    gw,
                                    activation_detail,
                                    reload_planner_map,
                                )
                                return _transition_response(
                                    False,
                                    status_code=409,
                                    message=f"planner map reload failed: {exc}",
                                    detail={"map_activation": activation_detail},
                                )
                            if not isinstance(planner_reload, Mapping) or planner_reload.get("ok") is not True:
                                _rollback_session_map_activation(
                                    gw,
                                    activation_detail,
                                    reload_planner_map,
                                )
                                return _transition_response(
                                    False,
                                    status_code=409,
                                    message="planner map reload failed; navigation session was not started",
                                    detail={
                                        "map_activation": activation_detail,
                                        "planner_reload": planner_reload,
                                    },
                                )
                            _clear_gateway_map_cloud(gw, "session_map_activation")

            backend = normalize_slam_profile(slam_profile or str(gw._get_slam_profile() or "").strip().lower())
            if not is_supported_slam_profile(backend):
                backend = default_slam_profile_for_mode(mode)
            if mode in {"mapping", "exploring"}:
                _clear_gateway_map_cloud(gw, "session_transition")

            if mode == "exploring" and start_task:
                try:
                    if product_control_owns_explore(gw):
                        exploration_run = start_exploration_run(
                            gw,
                            request_id=str(payload.get("request_id") or "") or None,
                        )
                    else:
                        gw._begin_exploration()
                        exploration_run = None
                    gw._exploring = True
                except ExplorationRunError as e:
                    gw._session_error = f"Explorer start failed: {e}"
                    return _transition_response(
                        False,
                        status_code=e.status_code,
                        message=gw._session_error,
                        detail={"error": e.code, **e.detail},
                    )
                except Exception as e:
                    gw._session_error = f"Explorer start failed: {e}"
                    return _transition_response(
                        False,
                        status_code=500,
                        message=gw._session_error,
                    )

            gw._session_mode = mode
            gw._session_product = product
            gw._session_product_session = product_session
            gw._session_map = map_name if mode in {"navigating", "exploring"} else None
            gw._session_slam_profile = backend
            gw._cached_slam_profile = backend
            gw._slam_profile_ts = time.time()
            gw._session_since = time.time()
            gw.push_event({"type": "session", "data": gw._session_snapshot()})
            transition_detail = (
                {"exploration_run": exploration_run}
                if mode == "exploring" and start_task and exploration_run is not None
                else None
            )
            return _transition_payload(
                True,
                session=gw._session_snapshot(),
                detail=transition_detail,
            )
        except Exception as e:
            gw._session_error = str(e)
            return _transition_response(
                False,
                status_code=500,
                message=str(e),
            )
        finally:
            gw._session_pending = False

    @app.post(
        "/api/v1/session/end",
        summary="End a local session or accept ProductControl-authorized field shutdown",
        response_model=SessionTransitionResponse,
        responses={
            409: {"model": SessionTransitionResponse},
            500: {"model": SessionTransitionResponse},
        },
    )
    async def session_end(request: Request = None):
        control_blocker = _session_end_control_blocker(gw, request)
        if control_blocker is not None:
            return _transition_response(
                False,
                status_code=409,
                message=("Field session shutdown is owned by ProductControl; use the operator command in detail."),
                detail=control_blocker,
            )
        if gw._session_mode == "idle":
            return _transition_payload(True, session=gw._session_snapshot())
        if gw._session_pending:
            return _transition_response(
                False,
                status_code=409,
                message="Transition in progress",
            )
        gw._session_pending = True
        try:
            if gw._exploring and gw._explorer_available():
                try:
                    gw._end_exploration()
                except Exception as e:
                    logger.warning("session/end: end_exploration failed: %s", e)
                gw._exploring = False
                gw.push_event({"type": "exploring", "active": False})
            gw._session_mode = "idle"
            gw._session_product = _current_runtime_product(gw)
            gw._session_product_session = "idle"
            gw._session_map = None
            gw._session_slam_profile = "stopped"
            gw._cached_slam_profile = "stopped"
            gw._slam_profile_ts = time.time()
            gw._session_since = time.time()
            gw._session_error = ""
            gw.push_event({"type": "session", "data": gw._session_snapshot()})
            return _transition_payload(True, session=gw._session_snapshot())
        except Exception as e:
            gw._session_error = str(e)
            return _transition_response(
                False,
                status_code=500,
                message=str(e),
            )
        finally:
            gw._session_pending = False

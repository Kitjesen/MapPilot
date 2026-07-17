"""Session lifecycle routes for GatewayModule."""

from __future__ import annotations

import logging
import os
import time
from collections.abc import Mapping
from typing import Any

from fastapi.responses import JSONResponse

from gateway.schemas import (
    SessionResponse,
    SessionStartRequest,
    SessionTransitionResponse,
)
from gateway.services.map_service import ensure_maps_service, map_service_command, map_service_query
from gateway.services.native_control import (
    endpoint_only_enabled,
)
from gateway.services.native_control import (
    read_status as read_native_control_status,
)
from gateway.services.native_control import (
    status_is_fresh as native_control_status_is_fresh,
)
from maps.services.storage import safe_map_name
from runtime.profiles.product_mode_contracts import PRODUCT_MODE_CONTRACTS
from runtime.profiles.resolver import canonical_profile_name
from runtime.runtime_policy import (
    default_slam_profile_for_mode,
    is_supported_slam_profile,
    normalize_slam_profile,
    session_transition_plan,
    slam_switch_plan,
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


_DEFAULT_PRODUCT_IDENTITY_BY_MODE = {
    "idle": (None, "idle"),
    "mapping": ("map", "mapping"),
    "navigating": ("nav", "navigation"),
    "exploring": ("tare_explore", "exploration"),
}

_REQUIRED_PRODUCT_PROFILES_BY_SESSION_MODE = {
    "mapping": ("map",),
    "navigating": ("teleop_avoid", "tracking", "nav", "inspection"),
    "exploring": ("tare_explore",),
}

_EXPECTED_NATIVE_CONTROL_MODE_BY_PROFILE = {
    "map": "teleop",
    "teleop_avoid": "teleop_avoid",
    "tracking": "autonomy",
    "nav": "autonomy",
    "inspection": "autonomy",
    "tare_explore": "autonomy",
}


def _default_product_identity_for_mode(mode: str) -> tuple[str | None, str]:
    return _DEFAULT_PRODUCT_IDENTITY_BY_MODE.get(
        str(mode or "").strip().lower(),
        (None, str(mode or "unknown").strip().lower() or "unknown"),
    )


def _normalize_product_identity(
    payload: dict[str, Any],
    mode: str,
) -> tuple[str | None, str]:
    raw_profile = payload.get("product_profile") or payload.get("profile") or ""
    product_profile = canonical_profile_name(str(raw_profile).strip()) if str(raw_profile or "").strip() else None
    explicit_session = str(payload.get("product_session") or "").strip().lower()
    default_profile, mode_default_session = _default_product_identity_for_mode(mode)

    if product_profile in PRODUCT_MODE_CONTRACTS:
        contract_session = (
            str(getattr(PRODUCT_MODE_CONTRACTS[product_profile], "product_session", "") or "").strip().lower()
        )
        default_session = contract_session or mode_default_session
    else:
        default_session = mode_default_session
        if product_profile is None:
            product_profile = default_profile

    return product_profile, explicit_session or default_session


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


def _current_runtime_product_profile(gw: Any) -> str | None:
    raw_profile = os.environ.get("LINGTU_PROFILE") or getattr(gw, "_runtime_product_profile", None) or ""
    profile = canonical_profile_name(str(raw_profile).strip())
    return profile if profile in PRODUCT_MODE_CONTRACTS else None


def _externally_owned_product_profile(gw: Any) -> str | None:
    if bool(getattr(gw, "_manage_session_services", True)):
        return None
    try:
        uses_native_endpoint = endpoint_only_enabled()
    except ValueError:
        uses_native_endpoint = True
    if not uses_native_endpoint:
        return None
    return _current_runtime_product_profile(gw) or ""


def _external_product_mode_guard(
    gw: Any,
    mode: str,
    *,
    requested_profile: str | None,
) -> dict[str, Any] | None:
    """Reject low-level sessions that would cross an externally-owned product graph."""

    current_profile = _externally_owned_product_profile(gw)
    if current_profile is None:
        return None
    if requested_profile is not None and requested_profile != current_profile:
        return {
            "reason_code": "product_profile_mismatch",
            "current_profile": current_profile,
            "requested_profile": requested_profile,
            "runtime_switch": "/api/v1/runtime/switch",
        }

    required_profiles = _REQUIRED_PRODUCT_PROFILES_BY_SESSION_MODE.get(mode, ())
    if current_profile not in required_profiles:
        return {
            "reason_code": "product_mode_switch_required",
            "current_profile": current_profile or None,
            "required_profiles": list(required_profiles),
            "runtime_switch": "/api/v1/runtime/switch",
        }

    expected_control_mode = _EXPECTED_NATIVE_CONTROL_MODE_BY_PROFILE.get(current_profile)
    status = read_native_control_status()
    actual_control_mode = str(status.get("control_mode") or "").strip().lower() if isinstance(status, dict) else ""
    if expected_control_mode and (
        not native_control_status_is_fresh(status) or actual_control_mode != expected_control_mode
    ):
        return {
            "reason_code": "native_control_mode_not_ready",
            "current_profile": current_profile,
            "expected_control_mode": expected_control_mode,
            "actual_control_mode": actual_control_mode or None,
            "control_status_fresh": native_control_status_is_fresh(status),
            "runtime_switch": "/api/v1/runtime/switch",
        }
    return None


def _external_same_session_active(
    gw: Any,
    *,
    mode: str,
    product_profile: str | None,
    product_session: str,
    map_name: str,
) -> bool:
    """Return whether an externally-owned runtime is already in this session."""

    if not _externally_owned_product_profile(gw):
        return False
    if gw._session_mode != mode:
        return False
    if (gw._session_product_profile or None) != (product_profile or None):
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
    @app.get(
        "/api/v1/session",
        summary="Current session state + capabilities",
        response_model=SessionResponse,
    )
    async def session_get():
        inferred_mode, inferred_map = gw._session_detect_current_mode()
        if inferred_mode != gw._session_mode:
            product_profile, product_session = _default_product_identity_for_mode(inferred_mode)
            gw._session_mode = inferred_mode
            gw._session_product_profile = product_profile
            gw._session_product_session = product_session
            gw._session_map = inferred_map
            gw._session_since = time.time()
        return gw._session_snapshot()

    @app.post(
        "/api/v1/session/start",
        summary="Enter a low-level mapping, navigating, or exploring session",
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
        map_name = payload.get("map_name") or payload.get("map") or ""
        slam_profile = (payload.get("slam_profile") or payload.get("slam_backend") or "").strip().lower()
        slam_profile = _normalize_slam_profile(slam_profile)
        if mode not in ("mapping", "navigating", "exploring"):
            return _transition_response(
                False,
                status_code=400,
                message=(f"Unknown mode: {mode!r}. Use 'mapping' | 'navigating' | 'exploring'."),
            )
        raw_requested_profile = str(payload.get("product_profile") or payload.get("profile") or "").strip()
        requested_profile = canonical_profile_name(raw_requested_profile) if raw_requested_profile else None
        product_mode_blocker = _external_product_mode_guard(
            gw,
            mode,
            requested_profile=requested_profile,
        )
        if product_mode_blocker is not None:
            return _transition_response(
                False,
                status_code=409,
                message=(
                    "Low-level session start cannot switch the externally-owned "
                    "product runtime. Use /api/v1/runtime/switch."
                ),
                detail=product_mode_blocker,
            )
        external_profile = _externally_owned_product_profile(gw)
        if external_profile and requested_profile is None:
            payload["product_profile"] = external_profile
        product_profile, product_session = _normalize_product_identity(
            payload,
            mode,
        )
        if _external_same_session_active(
            gw,
            mode=mode,
            product_profile=product_profile,
            product_session=product_session,
            map_name=str(map_name or ""),
        ):
            return _transition_payload(True, session=gw._session_snapshot())
        if slam_profile and not is_supported_slam_profile(slam_profile):
            return _transition_response(
                False,
                status_code=400,
                message=(
                    f"Unknown slam_profile: {slam_profile!r}. "
                    "Use 'none' | 'fastlio2' | 'genz' | 'localizer' | "
                    "'super_lio' | 'super_lio_relocation'."
                ),
            )
        if map_name:
            err = safe_map_name(map_name)
            if err is not None:
                return _transition_response(False, status_code=400, message=err)
        if slam_profile == "super_lio_relocation" and mode != "navigating":
            return _transition_response(
                False,
                status_code=400,
                message="super_lio_relocation requires navigating with map_name",
            )
        if mode == "exploring" and not gw._explorer_available():
            return _transition_response(
                False,
                status_code=503,
                message=("Exploration backend not running - start lingtu with 'explore' or 'tare_explore' profile."),
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
        if mode == "exploring":
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

        if mode == "navigating":
            if not map_name:
                return _transition_response(
                    False,
                    status_code=400,
                    message="map_name is required for navigating",
                )

            map_err = safe_map_name(map_name)
            if map_err is not None:
                return _transition_response(
                    False,
                    status_code=400,
                    message=map_err,
                )
            if map_name == "active":
                manager = _gateway_maps_service(gw)
                map_name = _maps_active_name(manager) if manager is not None else ""
                if not map_name:
                    return _transition_response(
                        False,
                        status_code=409,
                        message="no active map is selected",
                    )
            (
                map_ready,
                activation_message,
                activation_detail,
                map_path,
            ) = _activate_session_map_via_maps_service(gw, map_name)
            if map_ready is not True:
                status_code = 400
                if isinstance(activation_detail, dict) and activation_detail.get("reason_code") == "missing_capability":
                    status_code = 409
                elif "artifact gate failed" in activation_message:
                    status_code = 409
                return _transition_response(
                    False,
                    status_code=status_code,
                    message=activation_message or "map activation failed",
                    detail={"map_activation": activation_detail},
                )
            nav = (getattr(gw, "_all_modules", {}) or {}).get("nav.mission")
            reload_planner_map = getattr(nav, "reload_planner_map", None)
            if callable(reload_planner_map):
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

        gw._session_pending = True
        gw._session_error = ""
        try:
            manages_services = bool(getattr(gw, "_manage_session_services", True))
            backend = slam_profile or (
                default_slam_profile_for_mode(mode)
                if manages_services
                else str(gw._get_slam_profile() or "").strip().lower()
            )
            if not backend:
                backend = default_slam_profile_for_mode(mode)

            if manages_services:
                from runtime.service_manager import get_service_manager

                svc = get_service_manager()
                plan = session_transition_plan(mode, backend)
                svc.stop(*plan.stop)
                if plan.ensure:
                    svc.ensure(*plan.ensure)
                ok = svc.wait_ready(*plan.wait_ready, timeout=10.0) if plan.wait_ready else True
                if plan.clear_live_map:
                    _clear_gateway_map_cloud(gw, "session_transition")
                if not ok:
                    gw._session_error = "Services not ready after 10s"
                    return _transition_response(
                        False,
                        status_code=500,
                        message=gw._session_error,
                    )
            elif mode in {"mapping", "exploring"}:
                _clear_gateway_map_cloud(gw, "session_transition")

            if (
                manages_services
                and mode == "navigating"
                and map_name
                and backend not in {"super_lio", "super_lio_relocation"}
            ):
                gw._spawn_auto_relocalize(map_name)

            if mode == "exploring":
                try:
                    gw._begin_exploration()
                    gw._exploring = True
                except Exception as e:
                    gw._session_error = f"Explorer start failed: {e}"
                    return _transition_response(
                        False,
                        status_code=500,
                        message=gw._session_error,
                    )

            gw._session_mode = mode
            gw._session_product_profile = product_profile
            gw._session_product_session = product_session
            gw._session_map = map_name if mode == "navigating" else None
            gw._session_slam_profile = backend
            gw._cached_slam_profile = backend
            gw._slam_profile_ts = time.time()
            gw._session_since = time.time()
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

    @app.post(
        "/api/v1/session/end",
        summary="Exit current mode and return to idle",
        response_model=SessionTransitionResponse,
        responses={
            409: {"model": SessionTransitionResponse},
            500: {"model": SessionTransitionResponse},
        },
    )
    async def session_end():
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
            if bool(getattr(gw, "_manage_session_services", True)):
                from runtime.service_manager import get_service_manager

                svc = get_service_manager()
                svc.stop(*slam_switch_plan("stop").stop)
            gw._session_mode = "idle"
            gw._session_product_profile = _current_runtime_product_profile(gw)
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

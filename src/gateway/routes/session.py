"""Session lifecycle routes for GatewayModule."""

from __future__ import annotations

import logging
import os
import time
from typing import Any

from fastapi.responses import JSONResponse

from runtime.runtime_interface import TOPICS, topic_default_frame_id
from runtime.runtime_policy import (
    default_slam_profile_for_mode,
    is_supported_slam_profile,
    normalize_slam_profile,
    session_transition_plan,
    slam_switch_plan,
)
from gateway.schemas import (
    SessionResponse,
    SessionStartRequest,
    SessionTransitionResponse,
)
from gateway.services.map_paths import nav_map_root
from gateway.services.map_safety import safe_map_name
from runtime.same_source_map_artifacts import (
    validate_saved_map_artifact_dir,
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


def _normalize_slam_profile(profile: str) -> str:
    return normalize_slam_profile(profile)


def register_session_routes(app, gw) -> None:
    @app.get(
        "/api/v1/session",
        summary="Current session state + capabilities",
        response_model=SessionResponse,
    )
    async def session_get():
        inferred_mode, inferred_map = gw._session_detect_current_mode()
        if inferred_mode != gw._session_mode:
            gw._session_mode = inferred_mode
            gw._session_map = inferred_map
            gw._session_since = time.time()
        return gw._session_snapshot()

    @app.post(
        "/api/v1/session/start",
        summary="Enter mapping or navigating mode",
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
        slam_profile = (
            payload.get("slam_profile") or payload.get("slam_backend") or ""
        ).strip().lower()
        slam_profile = _normalize_slam_profile(slam_profile)
        if mode not in ("mapping", "navigating", "exploring"):
            return _transition_response(
                False,
                status_code=400,
                message=(
                    f"Unknown mode: {mode!r}. "
                    "Use 'mapping' | 'navigating' | 'exploring'."
                ),
            )
        if slam_profile and not is_supported_slam_profile(slam_profile):
            return _transition_response(
                False,
                status_code=400,
                message=(
                    f"Unknown slam_profile: {slam_profile!r}. "
                    "Use 'none' | 'fastlio2' | 'localizer' | "
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
                message=(
                    "Exploration backend not running - start lingtu with "
                    "'explore' or 'tare_explore' profile."
                ),
            )
        if gw._session_mode != "idle":
            return _transition_response(
                False,
                status_code=409,
                message=(
                    f"Already in {gw._session_mode}. "
                    "Call /session/end first."
                ),
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

            map_root = nav_map_root()
            base = (map_root / map_name).resolve()
            try:
                base.relative_to(map_root)
            except ValueError:
                return _transition_response(
                    False,
                    status_code=400,
                    message="map_name escapes NAV_MAP_DIR",
                )
            if not (base / "map.pcd").is_file():
                return _transition_response(
                    False,
                    status_code=400,
                    message=f"Map '{map_name}' has no map.pcd",
                )
            if not (base / "tomogram.pickle").is_file():
                return _transition_response(
                    False,
                    status_code=400,
                    message=(
                        f"Map '{map_name}' has no tomogram - build it "
                        f"first (REPL: map build {map_name})"
                    ),
                )
            artifact_gate = validate_saved_map_artifact_dir(
                base,
                require_tomogram=True,
                expected_frame_id=topic_default_frame_id(TOPICS.saved_map_cloud),
            )
            artifact_gate["required"] = True
            if artifact_gate.get("ok") is not True:
                blockers = [
                    str(item)
                    for item in (artifact_gate.get("blockers") or [])
                    if str(item)
                ]
                detail = "; ".join(blockers) if blockers else "unknown blocker"
                return _transition_response(
                    False,
                    status_code=409,
                    message=f"saved map artifact gate failed: {detail}",
                    detail={"artifact_gate": artifact_gate},
                )
            active = map_root / "active"
            try:
                if active.is_symlink() or active.exists():
                    active.unlink()
                os.symlink(map_name, str(active))
            except OSError as e:
                return _transition_response(
                    False,
                    status_code=500,
                    message=f"Failed to activate map: {e}",
                )

        gw._session_pending = True
        gw._session_error = ""
        try:
            from runtime.service_manager import get_service_manager

            svc = get_service_manager()
            backend = slam_profile or default_slam_profile_for_mode(mode)
            plan = session_transition_plan(mode, backend)
            svc.stop(*plan.stop)
            if plan.ensure:
                svc.ensure(*plan.ensure)
            ok = (
                svc.wait_ready(*plan.wait_ready, timeout=10.0)
                if plan.wait_ready
                else True
            )
            if plan.clear_live_map:
                with gw._map_cloud_lock:
                    gw._map_points = None
                    gw._voxel_hits.clear()
            if not ok:
                gw._session_error = "Services not ready after 10s"
                return _transition_response(
                    False,
                    status_code=500,
                    message=gw._session_error,
                )

            if (
                mode == "navigating"
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
            from runtime.service_manager import get_service_manager

            svc = get_service_manager()
            svc.stop(*slam_switch_plan("stop").stop)
            gw._session_mode = "idle"
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

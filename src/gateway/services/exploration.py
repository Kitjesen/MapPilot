"""Exploration runtime helpers for GatewayModule."""

from __future__ import annotations

import inspect
import json
import logging
import math
import os
from collections.abc import Callable, Mapping
from typing import Any

from gateway.schemas import GATEWAY_MAP_FRAME_ID
from gateway.services.command_boundary import navigation_commands
from gateway.services.native_exploration import read_fresh_status

logger = logging.getLogger(__name__)

_EXPLORE_VARIANTS = frozenset({"live", "map"})


def product_control_owns_explore(gw: Any) -> bool:
    """Return whether this Host belongs to a systemd-managed Explore Product."""

    plan = getattr(gw, "_compiled_run_plan", None)
    return bool(
        plan is not None
        and str(getattr(plan, "product", "") or "").strip() == "explore"
        and str(getattr(plan, "process_control", "") or "").strip() == "systemd"
    )


def external_explore_binding(gw: Any) -> dict[str, Any]:
    """Validate one committed Product session against fresh native evidence.

    This is a read-only gate. ProductControl remains the sole owner of Product
    selection and process lifecycle; Gateway only proves that the already
    selected Explore Product, its durable commit, this Host process and mapd's
    accepted native snapshot all describe the same session.
    """

    plan = getattr(gw, "_compiled_run_plan", None)
    result: dict[str, Any] = {
        "managed": product_control_owns_explore(gw),
        "valid": False,
        "blockers": [],
        "product_session_id": None,
        "variant": None,
        "map_id": None,
        "map_name": None,
        "native_status": None,
        "committed_at": None,
    }
    blockers: list[str] = result["blockers"]
    if not result["managed"]:
        blockers.append("product_control_explore_not_active")
        return result

    lifecycle = getattr(plan, "lifecycle", None)
    variant = str(getattr(plan, "product_variant", "") or "").strip()
    result["variant"] = variant or None
    if (
        variant not in _EXPLORE_VARIANTS
        or not isinstance(lifecycle, Mapping)
        or str(lifecycle.get("product") or "").strip() != "explore"
        or str(lifecycle.get("product_variant") or "").strip() != variant
        or str(lifecycle.get("session_mode") or "").strip() != "exploring"
        or bool(lifecycle.get("requires_map")) != (variant == "map")
    ):
        blockers.append("explore_run_plan_invalid")

    product_session_id = str(
        getattr(gw, "_compiled_product_session_id", "") or ""
    ).strip()
    result["product_session_id"] = product_session_id or None
    if not product_session_id:
        blockers.append("product_session_missing")

    current = _committed_current_run()
    if current is None:
        blockers.append("product_activation_not_committed")
    else:
        result["committed_at"] = current.get("committed_at")
        committed_map_name = str(current.get("map_name") or "").strip()
        result["map_name"] = committed_map_name or None
        current_checks = (
            current.get("schema_version") == "lingtu.current.v1",
            str(current.get("product") or "").strip() == "explore",
            str(current.get("product_variant") or "").strip() == variant,
            str(current.get("env") or "").strip()
            == str(getattr(plan, "env", "") or "").strip(),
        )
        if not all(current_checks):
            blockers.append("product_activation_identity_mismatch")
        if str(current.get("product_session_id") or "").strip() != product_session_id:
            blockers.append("product_session_mismatch")

    status = _native_status()
    if not isinstance(status, Mapping):
        blockers.append("native_exploration_status_unavailable")
        return _finish_binding(result)
    result["native_status"] = dict(status)
    if not str(status.get("boot_id") or "").strip():
        blockers.append("native_exploration_boot_identity_missing")
    if not _native_runtime_ready(status, variant=variant):
        blockers.append("native_exploration_not_ready")

    map_status = status.get("map")
    map_identity = current.get("map_identity") if isinstance(current, Mapping) else None
    if not _map_identity_matches(
        map_status,
        route=variant,
        map_identity=map_identity,
    ):
        blockers.append("exploration_map_identity_mismatch")
    elif isinstance(map_status, Mapping):
        result["map_id"] = str(map_status.get("map_id") or "").strip() or None
        if result["map_name"] is None:
            result["map_name"] = result["map_id"]

    if str(status.get("product_session_id") or "").strip() != product_session_id:
        blockers.append("native_exploration_product_session_mismatch")

    return _finish_binding(result)


def _finish_binding(result: dict[str, Any]) -> dict[str, Any]:
    blockers = result.get("blockers")
    if isinstance(blockers, list):
        result["blockers"] = list(dict.fromkeys(str(item) for item in blockers))
        result["valid"] = not result["blockers"]
    return result


def _committed_current_run() -> Mapping[str, Any] | None:
    try:
        from lingtu.product_lock import resolve_current_run_path

        path = resolve_current_run_path(environment=os.environ)
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError, json.JSONDecodeError):
        return None
    return payload if isinstance(payload, Mapping) else None


def _native_runtime_ready(status: Mapping[str, Any], *, variant: str) -> bool:
    if not (
        status.get("schema_version") == "lingtu.explore.status.v2"
        and status.get("endpoint") == "lingtu_explore_dds"
        and bool(str(status.get("boot_id") or "").strip())
        and str(status.get("route") or "").strip() == variant
        and status.get("ready") is True
    ):
        return False
    input_status = status.get("input")
    counters = status.get("counters")
    if not isinstance(input_status, Mapping) or not isinstance(counters, Mapping):
        return False
    try:
        odometry_age_s = float(input_status.get("odometry_age_s"))
        snapshot_age_s = float(input_status.get("snapshot_age_s"))
        odometry_messages = int(counters.get("odometry_messages"))
        snapshot_messages = int(counters.get("snapshot_messages"))
        odometry_max_age_s = float(
            os.environ.get("LINGTU_EXPLORE_ODOMETRY_MAX_AGE_S", "1.0") or "1.0"
        )
        snapshot_max_age_s = float(
            os.environ.get("LINGTU_EXPLORE_SNAPSHOT_MAX_AGE_S", "3.0") or "3.0"
        )
    except (TypeError, ValueError):
        return False
    return bool(
        all(
            math.isfinite(value)
            for value in (
                odometry_age_s,
                snapshot_age_s,
                odometry_max_age_s,
                snapshot_max_age_s,
            )
        )
        and odometry_max_age_s > 0.0
        and snapshot_max_age_s > 0.0
        and -0.05 <= odometry_age_s <= odometry_max_age_s
        and -0.05 <= snapshot_age_s <= snapshot_max_age_s
        and odometry_messages > 0
        and snapshot_messages > 0
    )


def _native_start_idle(status: Any) -> bool:
    return bool(
        isinstance(status, Mapping)
        and "pending_goal" in status
        and "pending_segment" in status
        and status.get("active") is False
        and status.get("paused") is False
        and str(status.get("state") or "").strip().lower() == "idle"
        and status.get("pending_goal") is None
        and status.get("pending_segment") is None
    )


def _map_identity_matches(
    status: Any,
    *,
    route: str,
    map_identity: Any,
) -> bool:
    if not isinstance(status, Mapping):
        return False
    try:
        reset_epoch = int(status.get("reset_epoch"))
        generation = int(status.get("generation"))
        map_content_epoch = status.get("map_content_epoch")
    except (TypeError, ValueError):
        return False
    if isinstance(map_content_epoch, bool) or not isinstance(map_content_epoch, int):
        return False
    if not (
        str(status.get("frame_id") or "").strip() == "map"
        and reset_epoch > 0
        and generation > 0
    ):
        return False
    if route == "live":
        return bool(
            map_identity is None
            and status.get("live") is True
            and not str(status.get("map_id") or "").strip()
            and map_content_epoch == 0
            and not any(
                str(os.environ.get(name) or "").strip()
                for name in (
                    "LINGTU_MAP_ID",
                    "LINGTU_MAP_CONTENT_EPOCH",
                )
            )
        )
    if route != "map" or not isinstance(map_identity, Mapping):
        return False
    map_id = str(map_identity.get("map_id") or "").strip()
    content_epoch = map_identity.get("content_epoch")
    frame_id = str(map_identity.get("frame_id") or "").strip()
    if (
        not map_id
        or frame_id != "map"
        or isinstance(content_epoch, bool)
        or not isinstance(content_epoch, int)
        or content_epoch <= 0
    ):
        return False
    return bool(
        status.get("live") is False
        and str(status.get("map_id") or "").strip() == map_id
        and map_content_epoch == content_epoch
        and str(os.environ.get("LINGTU_MAP_ID") or "").strip() == map_id
        and str(os.environ.get("LINGTU_MAP_CONTENT_EPOCH") or "").strip()
        == str(content_epoch)
        and str(os.environ.get("LINGTU_MAP_FRAME") or "").strip() == frame_id
    )


def _native_commands(gw: Any) -> Any | None:
    commands = navigation_commands(gw)
    if commands is None:
        return None
    if not callable(getattr(commands, "start_exploration", None)):
        return None
    if not callable(getattr(commands, "stop_exploration", None)):
        return None
    return commands


def _native_status() -> dict[str, Any] | None:
    return read_fresh_status()


def reconcile_exploration_runtime(
    gw: Any,
    binding: Mapping[str, Any],
) -> dict[str, Any]:
    """Reconcile a durable run with the exact native endpoint process."""

    from gateway.services.explore_runs import ensure_explore_runs

    if binding.get("valid") is not True:
        raise ExplorationRunError(
            "exploration_product_binding_invalid",
            "Explore runtime reconciliation requires a valid Product binding.",
            status_code=409,
            detail={"blockers": list(binding.get("blockers") or [])},
        )
    product_session_id = str(binding.get("product_session_id") or "").strip()
    status = binding.get("native_status")
    boot_id = (
        str(status.get("boot_id") or "").strip()
        if isinstance(status, Mapping)
        else ""
    )
    if not product_session_id or not boot_id:
        raise ExplorationRunError(
            "native_exploration_boot_identity_missing",
            "The native Explore endpoint has no trustworthy process identity.",
            status_code=409,
        )
    runs = ensure_explore_runs(gw)
    _require_durable_explore_runs(runs)
    changed = runs.reconcile_runtime(
        product_session_id=product_session_id,
        observed_boot_id=boot_id,
    )
    return {
        "boot_id": boot_id,
        "changed": changed,
        "latest": runs.latest_for_session(product_session_id),
        "health": runs.health(),
    }


class DirectedExplorationError(RuntimeError):
    """A directed native-exploration request cannot safely be issued."""

    def __init__(
        self,
        code: str,
        message: str,
        *,
        status_code: int,
        detail: dict[str, Any] | None = None,
    ) -> None:
        super().__init__(message)
        self.code = code
        self.status_code = status_code
        self.detail = detail or {}


class ExplorationRunError(RuntimeError):
    """A finite Explore run command cannot be admitted safely."""

    def __init__(
        self,
        code: str,
        message: str,
        *,
        status_code: int,
        detail: dict[str, Any] | None = None,
    ) -> None:
        super().__init__(message)
        self.code = code
        self.status_code = status_code
        self.detail = detail or {}


def _require_durable_explore_runs(runs: Any) -> None:
    health = runs.health()
    journal = health.get("journal") if isinstance(health, Mapping) else None
    if not isinstance(journal, Mapping):
        raise ExplorationRunError(
            "exploration_run_journal_unavailable",
            "The Explore run journal has no trustworthy health status.",
            status_code=503,
        )
    status = str(journal.get("status") or "").strip()
    path = str(journal.get("path") or "").strip()
    if status != "ready" or not path:
        raise ExplorationRunError(
            "exploration_run_journal_unavailable",
            str(journal.get("error") or "A durable Explore run journal is required."),
            status_code=503,
            detail={"journal": dict(journal)},
        )


def _run_has_interrupted_continuity(run: Mapping[str, Any] | None) -> bool:
    """Return true when a retained run lost endpoint continuity without parking."""

    if not isinstance(run, Mapping) or run.get("terminal") is True:
        return False
    continuity = run.get("continuity")
    return (
        isinstance(continuity, Mapping)
        and str(continuity.get("status") or "").strip().lower() == "interrupted"
        and run.get("state_available") is False
    )


def exploration_map_save_readiness(gw: Any) -> dict[str, Any]:
    """Prove that live Explore mapping is parked before snapshot admission."""

    from gateway.services.explore_runs import ensure_explore_runs

    runs = ensure_explore_runs(gw)
    _require_durable_explore_runs(runs)
    binding = external_explore_binding(gw)
    if not binding.get("valid"):
        return {
            "can_save": False,
            "reason": "exploration_product_binding_invalid",
            "message": "The committed Explore Product identity is not trustworthy.",
            "blockers": list(binding.get("blockers") or []),
        }
    if binding.get("variant") != "live":
        return {
            "can_save": False,
            "reason": "map_save_requires_live_exploration",
            "message": "Saved-map coverage exploration does not create a new mapping result.",
        }

    product_session_id = str(binding.get("product_session_id") or "").strip()
    latest_run = runs.latest_for_session(product_session_id)
    if latest_run is not None and (
        latest_run.get("terminal") is not True
        or (latest_run.get("motion_stop") or {}).get("confirmed") is not True
    ):
        return {
            "can_save": False,
            "reason": "exploration_run_not_safely_finished",
            "message": (
                "Finish the active Explore run and wait for confirmed parking "
                "before saving its live map."
            ),
            "exploration_run_id": latest_run.get("exploration_run_id"),
            "exploration_state": latest_run.get("state"),
        }

    if not _native_start_idle(binding.get("native_status")):
        return {
            "can_save": False,
            "reason": "native_exploration_motion_not_idle",
            "message": (
                "The native Explore endpoint still owns or is clearing a motion request."
            ),
        }
    return {
        "can_save": True,
        "reason": "exploration_safely_parked",
        "message": "Exploration is idle and any retained run has confirmed parking.",
    }


def _directed_context_from_status(status: Any) -> tuple[dict[str, Any], str, str]:
    """Validate authoritative native status for a directed intent.

    The HTTP process cannot select a native exploration session or transform a
    target into another frame. Both values are taken from the fresh endpoint
    status snapshot so an old browser command cannot affect a restarted TARE
    session.
    """

    if not isinstance(status, Mapping):
        raise DirectedExplorationError(
            "native_tare_status_unavailable",
            "A fresh native TARE status is required for directed exploration.",
            status_code=503,
            detail={"source": "native_tare_status"},
        )
    if status.get("active") is not True:
        raise DirectedExplorationError(
            "native_tare_not_active",
            "Directed exploration requires an active native TARE session.",
            status_code=409,
            detail={"source": "native_tare_status", "active": bool(status.get("active"))},
        )

    product_session_id = str(status.get("product_session_id") or "").strip()
    if not product_session_id:
        raise DirectedExplorationError(
            "native_tare_product_session_unavailable",
            "The active native TARE status has no session identifier.",
            status_code=409,
            detail={"source": "native_tare_status"},
        )

    map_status = status.get("map")
    if not isinstance(map_status, Mapping):
        raise DirectedExplorationError(
            "native_tare_map_context_unavailable",
            "The active native TARE status has no map context.",
            status_code=409,
            detail={"source": "native_tare_status", "product_session_id": product_session_id},
        )
    frame_id = str(map_status.get("frame_id") or "").strip()
    if frame_id != GATEWAY_MAP_FRAME_ID:
        raise DirectedExplorationError(
            "native_tare_map_frame_mismatch",
            "Directed exploration requires the native TARE map frame.",
            status_code=409,
            detail={
                "source": "native_tare_status",
                "expected_frame_id": GATEWAY_MAP_FRAME_ID,
                "actual_frame_id": frame_id or None,
                "product_session_id": product_session_id,
            },
        )
    return dict(status), product_session_id, frame_id


def _directed_command(gw: Any, method: str) -> Any:
    """Return only the native command endpoint method; never a local fallback."""

    commands = navigation_commands(gw)
    operation = getattr(commands, method, None) if commands is not None else None
    if not callable(operation):
        raise DirectedExplorationError(
            "directed_exploration_command_unavailable",
            "The native directed-exploration command boundary is unavailable.",
            status_code=503,
            detail={"source": "nav.commands", "method": method},
        )
    return operation


def _invoke_directed_command(operation: Any, args: tuple[Any, ...], kwargs: dict[str, Any]) -> None:
    """Call a command method once, accepting keyword-only test doubles too."""

    try:
        signature = inspect.signature(operation)
        use_keywords = False
        try:
            signature.bind(*args)
        except TypeError:
            signature.bind(**kwargs)
            use_keywords = True
        result = operation(**kwargs) if use_keywords else operation(*args)
    except DirectedExplorationError:
        raise
    except Exception as exc:
        message = str(exc) or type(exc).__name__
        code = (
            "directed_exploration_command_rejected"
            if "rejected" in message.lower()
            else "directed_exploration_command_unavailable"
        )
        raise DirectedExplorationError(
            code,
            message,
            status_code=409 if code.endswith("rejected") else 503,
            detail={"source": "nav.commands"},
        ) from exc

    if result is True:
        return
    if isinstance(result, Mapping) and result.get("accepted", result.get("success")) is True:
        return
    if result is False or (isinstance(result, Mapping) and result.get("accepted", result.get("success")) is False):
        raise DirectedExplorationError(
            "directed_exploration_command_rejected",
            "The native endpoint rejected the directed-exploration command.",
            status_code=409,
            detail={"source": "nav.commands"},
        )
    raise DirectedExplorationError(
        "directed_exploration_command_unavailable",
        "The native directed-exploration command did not acknowledge the request.",
        status_code=503,
        detail={"source": "nav.commands"},
    )


def _raise_directed_binding_error(binding: Mapping[str, Any]) -> None:
    blockers = [str(item) for item in binding.get("blockers") or []]
    if any(
        blocker
        in {
            "product_activation_not_committed",
            "native_exploration_status_unavailable",
        }
        for blocker in blockers
    ):
        raise DirectedExplorationError(
            "product_activation_state_unavailable",
            "The committed Explore Product state is unavailable.",
            status_code=503,
            detail={"source": "product_control", "blockers": blockers},
        )
    code = blockers[0] if blockers else "product_activation_identity_mismatch"
    raise DirectedExplorationError(
        code,
        "The active Explore Product no longer matches this Gateway session.",
        status_code=409,
        detail={"source": "product_control", "blockers": blockers},
    )


def _invoke_directed_for_verified_product_session(
    gw: Any,
    *,
    method: str,
    args_for_product_session: Callable[
        [str], tuple[tuple[Any, ...], dict[str, Any]]
    ],
) -> tuple[dict[str, Any], str, str]:
    """Issue one command against the session verified in the same lock."""

    def invoke(status: Any) -> tuple[dict[str, Any], str, str]:
        verified_status, product_session_id, frame_id = _directed_context_from_status(status)
        operation = _directed_command(gw, method)
        args, kwargs = args_for_product_session(product_session_id)
        _invoke_directed_command(operation, args, kwargs)
        return verified_status, product_session_id, frame_id

    if not product_control_owns_explore(gw):
        return invoke(_native_status())

    try:
        from lingtu.product_lock import ProductControlBusy, ProductControlLock

        with ProductControlLock(environment=os.environ, timeout_s=0.0):
            binding = external_explore_binding(gw)
            if not binding.get("valid"):
                _raise_directed_binding_error(binding)
            return invoke(binding.get("native_status"))
    except ProductControlBusy as exc:
        raise DirectedExplorationError(
            "product_activation_pending",
            "Explore Product activation is changing; retry after it completes.",
            status_code=409,
            detail={"source": "product_control"},
        ) from exc
    except OSError as exc:
        raise DirectedExplorationError(
            "product_activation_state_unavailable",
            "The committed Explore Product state is unavailable.",
            status_code=503,
            detail={"source": "product_control"},
        ) from exc


def directed_exploration_target(
    gw: Any,
    *,
    x: float,
    y: float,
    ttl_s: float,
    reason: str,
    request_id: str | None,
) -> dict[str, Any]:
    """Set a native TARE direction intent without publishing a navigation goal."""

    x_value = float(x)
    y_value = float(y)
    ttl_value = float(ttl_s)
    reason_value = str(reason)

    def args_for_product_session(
        product_session_id: str,
    ) -> tuple[tuple[Any, ...], dict[str, Any]]:
        args = (
            x_value,
            y_value,
            ttl_value,
            product_session_id,
            reason_value,
            request_id,
        )
        return args, {
            "x": x_value,
            "y": y_value,
            "ttl_s": ttl_value,
            "product_session_id": product_session_id,
            "reason": reason_value,
            "request_id": request_id,
        }

    status, product_session_id, frame_id = _invoke_directed_for_verified_product_session(
        gw,
        method="set_directed_exploration_target",
        args_for_product_session=args_for_product_session,
    )
    return {
        "intent": {
            "active": True,
            "x": x_value,
            "y": y_value,
            "ttl_s": ttl_value,
            "product_session_id": product_session_id,
            "frame_id": frame_id,
            "reason": reason_value,
            "request_id": request_id,
        },
        "native": {
            "active": True,
            "state": status.get("state"),
            "stamp_s": status.get("stamp_s"),
        },
    }


def clear_directed_exploration_target(
    gw: Any,
    *,
    reason: str,
    request_id: str | None,
) -> dict[str, Any]:
    """Clear only the native TARE direction intent for its current session."""

    reason_value = str(reason)

    def args_for_product_session(
        product_session_id: str,
    ) -> tuple[tuple[Any, ...], dict[str, Any]]:
        args = (product_session_id, reason_value, request_id)
        return args, {
            "product_session_id": product_session_id,
            "reason": reason_value,
            "request_id": request_id,
        }

    status, product_session_id, frame_id = _invoke_directed_for_verified_product_session(
        gw,
        method="clear_directed_exploration_target",
        args_for_product_session=args_for_product_session,
    )
    return {
        "intent": {
            "active": False,
            "x": None,
            "y": None,
            "ttl_s": None,
            "product_session_id": product_session_id,
            "frame_id": frame_id,
            "reason": reason_value,
            "request_id": request_id,
        },
        "native": {
            "active": True,
            "state": status.get("state"),
            "stamp_s": status.get("stamp_s"),
        },
    }


def explorer_backend(gw: Any) -> str:
    if _native_status() is not None:
        return "tare"
    return "none"


def explorer_available(gw: Any) -> bool:
    return _native_commands(gw) is not None and _native_status() is not None


def explorer_stop_available(gw: Any) -> bool:
    return _native_commands(gw) is not None


def explorer_unavailable_detail() -> dict[str, Any]:
    return {
        "reason": "explorer_backend_not_running",
        "required_product": "explore",
        "supported_products": ["explore"],
        "action": "switch LingTu to the explore Product before starting exploration",
    }


def coerce_explorer_result(result: Any) -> Any:
    if isinstance(result, str):
        try:
            return json.loads(result)
        except Exception as exc:
            logger.debug("_coerce_explorer_result JSON parse failed: %s", exc)
            return result
    return result


def _require_explorer_ack(result: Any, *, action: str) -> Any:
    """Validate all explorer backends against one action-specific ACK rule."""

    expected_statuses = {
        "start": {"started", "already_running"},
        "stop": {"stopped"},
    }
    expected = expected_statuses[action]

    if isinstance(result, str):
        stripped = result.strip()
        if stripped in expected:
            return stripped
        normalized = coerce_explorer_result(stripped)
        if not isinstance(normalized, Mapping):
            raise RuntimeError(
                f"exploration {action} returned an invalid acknowledgement"
            )
    else:
        normalized = result

    if normalized is True:
        return normalized
    if not isinstance(normalized, Mapping):
        raise RuntimeError(f"exploration {action} returned an invalid acknowledgement")

    if normalized.get("accepted") is False or normalized.get("success") is False:
        raise RuntimeError(f"exploration {action} rejected: negative acknowledgement")

    explicit_positive = (
        normalized.get("accepted") is True
        or normalized.get("success") is True
    )
    status_positive = normalized.get("status") in expected
    if explicit_positive or status_positive:
        return normalized

    raise RuntimeError(f"exploration {action} returned an invalid acknowledgement")


def _explore_run_map_binding(binding: Mapping[str, Any]) -> dict[str, Any] | None:
    route = str(binding.get("variant") or "").strip()
    if route != "map":
        return None
    native_status = binding.get("native_status")
    map_status = native_status.get("map") if isinstance(native_status, Mapping) else None
    if not isinstance(map_status, Mapping):
        raise ExplorationRunError(
            "exploration_map_identity_unavailable",
            "The active Explore Product has no verified saved-map identity.",
            status_code=409,
        )
    try:
        map_content_epoch = int(map_status.get("map_content_epoch"))
    except (TypeError, ValueError) as exc:
        raise ExplorationRunError(
            "exploration_map_identity_unavailable",
            "The active Explore Product has an invalid map version.",
            status_code=409,
        ) from exc
    map_id = str(map_status.get("map_id") or "").strip()
    if not map_id or map_content_epoch <= 0:
        raise ExplorationRunError(
            "exploration_map_identity_unavailable",
            "The active Explore Product map binding is incomplete.",
            status_code=409,
        )
    return {
        "map_id": map_id,
        "map_content_epoch": map_content_epoch,
    }


def _typed_exploration_receipt(
    result: Any,
    *,
    action: str,
    exploration_run_id: str,
    request_id: str,
) -> dict[str, Any]:
    normalized = coerce_explorer_result(result)
    if not isinstance(normalized, Mapping):
        raise ExplorationRunError(
            "exploration_receipt_invalid",
            f"Native exploration {action} returned no typed receipt.",
            status_code=503,
        )
    receipt = dict(normalized)
    if str(receipt.get("exploration_run_id") or "") != exploration_run_id:
        raise ExplorationRunError(
            "exploration_receipt_identity_mismatch",
            f"Native exploration {action} returned the wrong exploration_run_id.",
            status_code=503,
            detail={"native": receipt},
        )
    if str(receipt.get("request_id") or "") != request_id:
        raise ExplorationRunError(
            "exploration_receipt_identity_mismatch",
            f"Native exploration {action} returned the wrong request_id.",
            status_code=503,
            detail={"native": receipt},
        )
    if not isinstance(receipt.get("accepted"), bool):
        raise ExplorationRunError(
            "exploration_receipt_invalid",
            f"Native exploration {action} receipt has no admission result.",
            status_code=503,
            detail={"native": receipt},
        )
    return receipt


def _dispatch_native_exploration_start(
    gw: Any,
    *,
    binding: Mapping[str, Any],
    exploration_run_id: str,
    request_id: str,
) -> dict[str, Any]:
    commands = _native_commands(gw)
    if commands is None:
        raise ExplorationRunError(
            "exploration_command_unavailable",
            "The native Explore command boundary is unavailable.",
            status_code=503,
        )
    native_status = binding.get("native_status")
    if not _native_start_idle(native_status):
        active_run_id = (
            str(native_status.get("exploration_run_id") or "").strip()
            if isinstance(native_status, Mapping)
            else ""
        )
        if active_run_id != exploration_run_id:
            raise ExplorationRunError(
                "native_exploration_not_idle",
                "Another native Explore run is active or stopping.",
                status_code=409,
                detail={"active_exploration_run_id": active_run_id or None},
            )
    try:
        result = commands.start_exploration(
            exploration_run_id=exploration_run_id,
            product_session_id=str(binding["product_session_id"]),
            reason="gateway_start",
            request_id=request_id,
        )
    except Exception as exc:
        raise ExplorationRunError(
            "exploration_command_unavailable",
            str(exc) or "Native Explore start failed before a typed ACK was received.",
            status_code=503,
        ) from exc
    return _typed_exploration_receipt(
        result,
        action="start",
        exploration_run_id=exploration_run_id,
        request_id=request_id,
    )


def start_exploration_run(gw: Any, request_id: str | None = None) -> dict[str, Any]:
    """Durably reserve and admit one Product-bound native Explore execution."""

    from gateway.services.explore_runs import (
        ExploreRunConflict,
        ExploreRunJournalUnavailable,
        ensure_explore_runs,
        new_request_id,
    )
    from lingtu.product_lock import ProductControlBusy, ProductControlLock

    request = str(request_id or "").strip() or new_request_id()
    runs = ensure_explore_runs(gw)
    _require_durable_explore_runs(runs)
    try:
        with ProductControlLock(environment=os.environ, timeout_s=0.0):
            binding = external_explore_binding(gw)
            if not binding["valid"]:
                raise ExplorationRunError(
                    "exploration_product_binding_invalid",
                    "Explore start does not match the committed Product session.",
                    status_code=409,
                    detail={"blockers": list(binding["blockers"])},
                )
            continuity = reconcile_exploration_runtime(gw, binding)
            latest = continuity.get("latest")
            if _run_has_interrupted_continuity(latest):
                raise ExplorationRunError(
                    "exploration_run_interrupted",
                    (
                        "The previous Explore endpoint restarted before the run "
                        "produced confirmed parking evidence. Stop or cold-start "
                        "the Explore Product before admitting another run."
                    ),
                    status_code=409,
                    detail={"exploration_run": dict(latest)},
                )
            route = str(binding["variant"])
            map_identity = _explore_run_map_binding(binding)
            existing = runs.query_start_request(request)
            if existing is None:
                readiness = gw._exploration_start_readiness()
                if not readiness.get("can_start", False):
                    raise ExplorationRunError(
                        "exploration_not_ready",
                        "Exploration readiness blockers have not cleared.",
                        status_code=409,
                        detail=dict(readiness),
                    )
            reservation = runs.reserve_start(
                request,
                product_session_id=str(binding["product_session_id"]),
                route=route,
                map=map_identity,
            )
            run = reservation["run"]
            replay = bool(reservation["replay"])
            if replay and run.get("admission") in {"accepted", "rejected"}:
                return {
                    **run,
                    "ok": run.get("admission") == "accepted",
                    "accepted": run.get("admission") == "accepted",
                    "request_id": request,
                    "replay": True,
                }
            try:
                receipt = _dispatch_native_exploration_start(
                    gw,
                    binding=binding,
                    exploration_run_id=str(run["exploration_run_id"]),
                    request_id=request,
                )
            except ExplorationRunError as exc:
                exc.detail.setdefault("request_id", request)
                exc.detail.setdefault("exploration_run_id", run["exploration_run_id"])
                exc.detail.setdefault("run", run)
                raise
            try:
                projected = runs.record_admission(
                    str(run["exploration_run_id"]),
                    accepted=bool(receipt["accepted"]),
                    reason=str(receipt.get("reason") or "native_exploration_ack"),
                )
            except ExploreRunJournalUnavailable as exc:
                compensation: dict[str, Any] = {
                    "attempted": False,
                    "accepted": False,
                    "reason": "motion_not_admitted",
                }
                if receipt["accepted"] is True:
                    compensation["attempted"] = True
                    stop_request_id = new_request_id()
                    commands = _native_commands(gw)
                    stop_operation = (
                        getattr(commands, "stop_exploration", None)
                        if commands is not None
                        else None
                    )
                    try:
                        if not callable(stop_operation):
                            raise RuntimeError("native Explore stop is unavailable")
                        stop_receipt = _typed_exploration_receipt(
                            stop_operation(
                                exploration_run_id=str(run["exploration_run_id"]),
                                product_session_id=str(binding["product_session_id"]),
                                reason="exploration_journal_write_failed",
                                request_id=stop_request_id,
                            ),
                            action="finish",
                            exploration_run_id=str(run["exploration_run_id"]),
                            request_id=stop_request_id,
                        )
                        compensation = {
                            "attempted": True,
                            "accepted": stop_receipt.get("accepted") is True,
                            "reason": str(stop_receipt.get("reason") or ""),
                            "request_id": stop_request_id,
                        }
                    except Exception as stop_exc:
                        compensation = {
                            "attempted": True,
                            "accepted": False,
                            "reason": str(stop_exc) or type(stop_exc).__name__,
                            "request_id": stop_request_id,
                        }
                raise ExplorationRunError(
                    "exploration_run_journal_unavailable",
                    str(exc),
                    status_code=503,
                    detail={
                        "request_id": request,
                        "exploration_run_id": run["exploration_run_id"],
                        "compensating_stop": compensation,
                    },
                ) from exc
            return {
                **projected,
                "ok": bool(receipt["accepted"]),
                "accepted": bool(receipt["accepted"]),
                "request_id": request,
                "replay": replay,
                "native": receipt,
            }
    except ProductControlBusy as exc:
        raise ExplorationRunError(
            "product_activation_pending",
            "ProductControl is changing the active Product.",
            status_code=409,
        ) from exc
    except ExploreRunConflict as exc:
        raise ExplorationRunError(
            "exploration_request_conflict", str(exc), status_code=409
        ) from exc
    except ExploreRunJournalUnavailable as exc:
        raise ExplorationRunError(
            "exploration_run_journal_unavailable", str(exc), status_code=503
        ) from exc
    except OSError as exc:
        raise ExplorationRunError(
            "product_activation_state_unavailable", str(exc), status_code=503
        ) from exc


def command_exploration_run(
    gw: Any,
    exploration_run_id: str,
    *,
    action: str,
    request_id: str | None = None,
    reason: str | None = None,
) -> dict[str, Any]:
    """Admit one pause, resume, or finish command for the current run."""

    from gateway.services.explore_runs import ensure_explore_runs, new_request_id
    from lingtu.product_lock import ProductControlBusy, ProductControlLock

    operations = {
        "pause": ("pause_exploration", "operator_pause"),
        "resume": ("resume_exploration", "operator_resume"),
        "finish": ("stop_exploration", "operator_finish"),
    }
    if action not in operations:
        raise ValueError(f"unsupported exploration run action {action!r}")
    command_request_id = str(request_id or "").strip() or new_request_id()
    runs = ensure_explore_runs(gw)
    _require_durable_explore_runs(runs)
    try:
        with ProductControlLock(environment=os.environ, timeout_s=0.0):
            try:
                run = runs.query(exploration_run_id)
            except ValueError as exc:
                raise ExplorationRunError(
                    "exploration_run_id_invalid",
                    str(exc),
                    status_code=400,
                ) from exc
            if not run.get("found"):
                raise ExplorationRunError(
                    "exploration_run_not_found",
                    "The requested Explore run is not retained on this robot.",
                    status_code=404,
                )
            if _run_has_interrupted_continuity(run):
                raise ExplorationRunError(
                    "exploration_run_interrupted",
                    (
                        "The Explore endpoint restarted and no terminal parking "
                        "evidence exists for this run. Use ProductControl to stop "
                        "or cold-start the Explore Product."
                    ),
                    status_code=409,
                    detail={"exploration_run": run},
                )
            if run.get("terminal"):
                if action == "finish":
                    return {
                        **run,
                        "ok": True,
                        "accepted": True,
                        "request_id": command_request_id,
                        "replay": True,
                    }
                raise ExplorationRunError(
                    "exploration_run_terminal",
                    "A terminal Explore run cannot be changed.",
                    status_code=409,
                    detail={"state": run.get("state")},
                )
            binding = external_explore_binding(gw)
            if not binding["valid"]:
                raise ExplorationRunError(
                    "exploration_product_binding_invalid",
                    "The Explore run no longer matches the committed Product session.",
                    status_code=409,
                    detail={"blockers": list(binding["blockers"])},
                )
            expected_product_session_id = str(run["identity"]["product_session_id"])
            if str(binding["product_session_id"]) != expected_product_session_id:
                raise ExplorationRunError(
                    "exploration_product_session_mismatch",
                    "The Explore run belongs to a different Product session.",
                    status_code=409,
                )
            commands = _native_commands(gw)
            method_name, default_reason = operations[action]
            operation = getattr(commands, method_name, None) if commands is not None else None
            if not callable(operation):
                raise ExplorationRunError(
                    "exploration_command_unavailable",
                    f"Native Explore {action} is unavailable.",
                    status_code=503,
                )
            try:
                result = operation(
                    exploration_run_id=exploration_run_id,
                    product_session_id=expected_product_session_id,
                    reason=str(reason or default_reason),
                    request_id=command_request_id,
                )
            except Exception as exc:
                raise ExplorationRunError(
                    "exploration_command_unavailable",
                    str(exc) or f"Native Explore {action} failed before ACK.",
                    status_code=503,
                ) from exc
            receipt = _typed_exploration_receipt(
                result,
                action=action,
                exploration_run_id=exploration_run_id,
                request_id=command_request_id,
            )
            if receipt["accepted"] is not True:
                raise ExplorationRunError(
                    "exploration_command_rejected",
                    str(receipt.get("reason") or f"Native Explore {action} was rejected."),
                    status_code=409,
                    detail={"native": receipt},
                )
            return {
                **runs.query(exploration_run_id),
                "ok": True,
                "accepted": True,
                "request_id": command_request_id,
                "replay": bool(receipt.get("duplicate", False)),
                "native": receipt,
            }
    except ProductControlBusy as exc:
        raise ExplorationRunError(
            "product_activation_pending",
            "ProductControl is changing the active Product.",
            status_code=409,
        ) from exc


def begin_exploration(
    gw: Any,
    *,
    exploration_run_id: str = "",
    request_id: str = "",
) -> Any:
    commands = _native_commands(gw)
    if commands is not None and _native_status() is not None:
        if product_control_owns_explore(gw):
            try:
                from lingtu.product_lock import ProductControlBusy, ProductControlLock

                with ProductControlLock(environment=os.environ, timeout_s=0.0):
                    binding = external_explore_binding(gw)
                    if not binding["valid"]:
                        reasons = ", ".join(binding["blockers"])
                        raise RuntimeError(f"exploration start rejected: {reasons}")
                    if not _native_start_idle(binding["native_status"]):
                        raise RuntimeError(
                            "exploration start rejected: native_exploration_not_idle"
                        )
                    if exploration_run_id or request_id:
                        if not exploration_run_id or not request_id:
                            raise RuntimeError(
                                "exploration start rejected: incomplete run identity"
                            )
                        return _dispatch_native_exploration_start(
                            gw,
                            binding=binding,
                            exploration_run_id=exploration_run_id,
                            request_id=request_id,
                        )
                    _require_explorer_ack(
                        commands.start_exploration(
                            product_session_id=binding["product_session_id"],
                            reason="gateway_start",
                        ),
                        action="start",
                    )
            except ProductControlBusy as exc:
                raise RuntimeError(
                    "exploration start rejected: product_activation_pending"
                ) from exc
            except OSError as exc:
                raise RuntimeError(
                    "exploration start rejected: product_activation_state_unavailable"
                ) from exc
        else:
            _require_explorer_ack(
                commands.start_exploration(reason="gateway_start"),
                action="start",
            )
        return {"accepted": True, "backend": "tare", "runtime": "native_dds"}
    raise RuntimeError("explorer_not_running")


def end_exploration(gw: Any) -> Any:
    commands = _native_commands(gw)
    if commands is not None:
        _require_explorer_ack(
            commands.stop_exploration(reason="gateway_stop"),
            action="stop",
        )
        return {"accepted": True, "backend": "tare", "runtime": "native_dds"}
    raise RuntimeError("explorer_not_running")


def tare_status_payload(gw: Any) -> dict[str, Any]:
    status = _native_status()
    if status is None:
        return {}
    return {
        "runtime": "native_dds",
        "status": status,
        "stats": {
            "planner": status.get("planner") or {},
            "counters": status.get("counters") or {},
        },
    }


def exploration_status_payload(gw: Any) -> dict[str, Any]:
    backend = gw._explorer_backend()
    readiness = gw._exploration_start_readiness()
    if backend == "none":
        return {
            "available": False,
            "backend": "none",
            "exploring": False,
            "frontier_count": 0,
            **readiness,
            **gw._explorer_unavailable_detail(),
        }
    tare = gw._tare_status_payload()
    tare_status = tare.get("status") if isinstance(tare, dict) else {}
    tare_started = False
    frontier_count = 0
    if isinstance(tare_status, dict):
        tare_started = bool(tare_status.get("active", tare_status.get("started", False)))
        planner = tare_status.get("planner") or {}
        if isinstance(planner, dict):
            try:
                frontier_count = int(planner.get("frontier_clusters", 0) or 0)
            except (TypeError, ValueError):
                frontier_count = 0
    return {
        "available": gw._explorer_available(),
        "backend": "tare",
        "exploring": tare_started,
        "frontier_count": frontier_count,
        "tare": tare,
        **readiness,
    }


def exploration_start_readiness(gw: Any) -> dict[str, Any]:
    blockers: list[str] = []
    advisories: list[str] = []
    navigation: dict[str, Any] = {}
    run_projection: dict[str, Any] = {}
    exploration_ignored_nav_blockers = {"navigation_session_inactive"}

    if (
        product_control_owns_explore(gw)
        and str(gw._session_mode or "idle").lower() == "idle"
    ):
        from gateway.services.session_view import reconcile_native_explore

        reconcile_native_explore(gw)

    native_status = _native_status()
    native_active = bool(native_status and native_status.get("active") is True)
    if not gw._explorer_available():
        if native_status is not None and _native_commands(gw) is None:
            blockers.append("exploration_command_boundary_unavailable")
        else:
            blockers.append("explorer_backend_not_running")
    if gw._exploring or native_active:
        blockers.append("exploration_already_active")
    session_mode = str(gw._session_mode or "idle").lower()
    deferred_exploring_task = session_mode == "exploring" and not gw._exploring and not native_active
    if session_mode != "idle" and not deferred_exploring_task:
        blockers.append("session_not_idle")
    if product_control_owns_explore(gw):
        binding = external_explore_binding(gw)
        blockers.extend(binding["blockers"])
        if binding["valid"] and not _native_start_idle(binding["native_status"]):
            blockers.append("native_exploration_not_idle")
        if session_mode != "exploring":
            blockers.append("exploration_session_not_staged")
        try:
            from gateway.services.explore_runs import ensure_explore_runs

            runs = ensure_explore_runs(gw)
            continuity = (
                reconcile_exploration_runtime(gw, binding)
                if binding.get("valid")
                else None
            )
            health = continuity["health"] if continuity is not None else runs.health()
            latest = (
                continuity["latest"]
                if continuity is not None
                else (
                    runs.latest_for_session(str(binding.get("product_session_id") or ""))
                    if binding.get("product_session_id")
                    else None
                )
            )
            run_projection = {
                "health": health,
                "latest": latest,
                "runtime": continuity,
            }
            journal = health.get("journal") if isinstance(health, Mapping) else None
            if not (
                isinstance(journal, Mapping)
                and journal.get("status") == "ready"
                and str(journal.get("path") or "").strip()
            ):
                blockers.append("exploration_run_journal_unavailable")
            if isinstance(latest, Mapping) and latest.get("terminal") is not True:
                blockers.append("exploration_run_unresolved")
        except ExplorationRunError as exc:
            run_projection = {"error": exc.code, "detail": exc.detail}
            blockers.append(exc.code)
        except Exception as exc:
            run_projection = {"error": str(exc) or type(exc).__name__}
            blockers.append("exploration_run_journal_unavailable")

    try:
        from gateway.services.runtime_status import build_navigation_status

        navigation = build_navigation_status(gw)
        readiness = navigation.get("readiness") or {}
        nav_blockers = readiness.get("blockers") or []
        nav_advisories = readiness.get("advisories") or []
        if isinstance(nav_blockers, list):
            blockers.extend(str(code) for code in nav_blockers if str(code) not in exploration_ignored_nav_blockers)
        if isinstance(nav_advisories, list):
            advisories.extend(str(code) for code in nav_advisories)
        if not blockers and not bool(readiness.get("can_execute_autonomy", False)) and not nav_blockers:
            blockers.append("navigation_not_ready")
    except Exception as exc:
        logger.debug(
            "_exploration_start_readiness: build_navigation_status failed: %s",
            exc,
        )
        blockers.append("navigation_status_unavailable")

    blockers = list(dict.fromkeys(blockers))
    advisories = list(dict.fromkeys(advisories))
    return {
        "can_start": bool(gw._explorer_available()) and not blockers,
        "blockers": blockers,
        "advisories": advisories,
        "navigation": {
            "state": navigation.get("state"),
            "can_accept_goal": navigation.get("can_accept_goal"),
            "readiness": navigation.get("readiness") or {},
        },
        "run_projection": run_projection,
    }

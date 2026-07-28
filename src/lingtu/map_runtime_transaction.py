"""Transactional coordination between saved maps and navigation runtime state."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from threading import RLock
from typing import Any

MapCommand = Callable[[dict[str, Any]], dict[str, Any]]
PlannerReload = Callable[[str], Mapping[str, Any]]


class MapRuntimeTransaction:
    """Keep active-map selection and the current planner view consistent.

    Persistent map ownership remains in ``maps.service``.  This coordinator is
    deliberately small: it only sequences existing capabilities and describes
    whether a change is committed now, rejected, or rolled back.
    """

    _MOTION_SESSION_MODES = frozenset({"navigating", "exploring"})
    _shared_lock = RLock()

    def __init__(self, map_command: MapCommand, planner_reload: PlannerReload) -> None:
        self._map_command = map_command
        self._planner_reload = planner_reload

    @classmethod
    def shared_lock(cls):
        """Return the process-wide lock shared by all map runtime mutations."""
        return cls._shared_lock

    def activate(
        self,
        target_map: str,
        *,
        session_mode: str = "",
        session_map: str = "",
        session_pending: bool = False,
    ) -> dict[str, Any]:
        """Select a map and reconcile the planner, rolling back on failure."""
        with self._shared_lock:
            target = str(target_map or "").strip()
            if session_pending:
                return self._result(
                    success=False,
                    target=target,
                    previous=str(session_map or "").strip(),
                    state="rejected",
                    runtime_consistent=True,
                    reason_code="session_transition_in_progress",
                    message="Map activation is blocked while a session transition is in progress.",
                )
            return self._activate_locked(
                target,
                session_mode=session_mode,
                session_map=session_map,
            )

    def _activate_locked(
        self,
        target_map: str,
        *,
        session_mode: str,
        session_map: str,
    ) -> dict[str, Any]:
        target = str(target_map or "").strip()
        mode = str(session_mode or "").strip().lower()
        bound_map = str(session_map or "").strip()
        if mode in self._MOTION_SESSION_MODES and bound_map != target:
            return self._result(
                success=False,
                target=target,
                previous=bound_map,
                state="rejected",
                runtime_consistent=True,
                reason_code="active_session_map_conflict",
                message=(
                    "Cross-map activation is blocked during an active navigation "
                    "session; use the full runtime switch so localization and "
                    "planning change maps together."
                ),
            )
        try:
            previous_response = self._map_command({"action": "get_active"})
        except Exception as exc:
            return self._result(
                success=False,
                target=target,
                previous="",
                state="rejected",
                runtime_consistent=False,
                reason_code="active_map_query_failed",
                message=str(exc),
            )
        if not isinstance(previous_response, Mapping) or previous_response.get("success") is not True:
            response = previous_response if isinstance(previous_response, Mapping) else {}
            reason_code = str(response.get("reason_code") or "active_map_query_failed")
            if reason_code != "map_not_found":
                return self._result(
                    success=False,
                    target=target,
                    previous="",
                    state="rejected",
                    runtime_consistent=False,
                    reason_code=reason_code,
                    message=str(response.get("message") or "failed to read active map"),
                )
            previous = ""
        else:
            previous = str(previous_response.get("active") or "").strip()
        if mode in self._MOTION_SESSION_MODES:
            if previous != target:
                return self._result(
                    success=False,
                    target=target,
                    previous=previous,
                    state="rejected",
                    runtime_consistent=False,
                    reason_code="session_active_map_drift",
                    message="Session map cache does not match the authoritative active map.",
                    active=previous or None,
                )
            return self._result(
                success=True,
                target=target,
                previous=previous,
                state="committed",
                runtime_consistent=True,
                active=target,
                unchanged=True,
                message="Map is already bound to the active navigation session.",
            )

        try:
            activation = dict(self._map_command({"action": "set_active", "name": target}))
        except Exception as exc:
            return self._result(
                success=False,
                target=target,
                previous=previous,
                state="rejected",
                runtime_consistent=True,
                reason_code="map_activation_failed",
                message=str(exc),
            )
        if activation.get("success") is not True:
            activation["transaction"] = self._transaction_state(
                target=target,
                previous=previous,
                state="rejected",
                runtime_consistent=True,
            )
            if not activation.get("reason_code"):
                message = str(activation.get("message") or "").lower()
                activation["reason_code"] = "map_not_found" if "not found" in message else "map_activation_failed"
            return activation

        map_path = self._runtime_artifact_path(activation)
        try:
            planner_reload = self._planner_reload(map_path)
        except Exception as exc:
            planner_reload = {
                "ok": False,
                "reason": "planner_reload_failed",
                "message": str(exc),
                "map_path": map_path,
            }
        if not isinstance(planner_reload, Mapping):
            planner_reload = {
                "ok": False,
                "reason": "planner_reload_invalid_response",
                "map_path": map_path,
            }
        planner_result = dict(planner_reload)
        activation["planner_reload"] = planner_result

        if planner_result.get("ok") is True and planner_result.get("delegated") is True:
            planner_result["ok"] = False
            planner_result["confirmed"] = False
            planner_result["reason"] = "runtime_binding_unconfirmed"

        if planner_result.get("ok") is True:
            planner_result["confirmed"] = True
            activation["transaction"] = self._transaction_state(
                target=target,
                previous=previous,
                state="committed",
                runtime_consistent=True,
            )
            return activation

        rollback = self._rollback(previous, target)
        activation["success"] = False
        activation["reason_code"] = str(planner_result.get("reason") or "planner_reload_failed")
        activation["rollback"] = rollback
        rollback_ok = rollback.get("success") is True
        activation["message"] = (
            "Planner map reload failed; active map was rolled back."
            if rollback_ok
            else "Planner map reload failed and active-map rollback failed."
        )
        activation["transaction"] = self._transaction_state(
            target=target,
            previous=previous,
            state="rolled_back" if rollback_ok else "rollback_failed",
            runtime_consistent=rollback_ok,
        )
        return activation

    def _rollback(self, previous: str, target: str) -> dict[str, Any]:
        if previous == target:
            return {"success": True, "skipped": True, "previous_active": previous}
        command: dict[str, Any]
        if previous:
            command = {"action": "set_active", "name": previous}
        else:
            command = {"action": "clear_active"}
        try:
            rollback = dict(self._map_command(command))
        except Exception as exc:
            return {
                "success": False,
                "previous_active": previous or None,
                "message": str(exc),
            }
        rollback["previous_active"] = previous or None
        if rollback.get("success") is not True:
            return rollback
        if not previous:
            rollback["success"] = False
            rollback["planner_reload"] = {
                "ok": False,
                "reason": "planner_clear_unconfirmed",
                "map_path": "",
            }
            return rollback
        rollback_path = self._runtime_artifact_path(rollback)
        if not rollback_path:
            rollback["success"] = False
            rollback["planner_reload"] = {
                "ok": False,
                "reason": "planner_rollback_path_missing",
                "map_path": "",
            }
            return rollback
        try:
            planner_rollback = self._planner_reload(rollback_path)
        except Exception as exc:
            planner_rollback = {
                "ok": False,
                "reason": "planner_rollback_failed",
                "message": str(exc),
                "map_path": rollback_path,
            }
        if not isinstance(planner_rollback, Mapping):
            planner_rollback = {
                "ok": False,
                "reason": "planner_rollback_invalid_response",
                "map_path": rollback_path,
            }
        rollback["planner_reload"] = dict(planner_rollback)
        rollback["success"] = planner_rollback.get("ok") is True and planner_rollback.get("delegated") is not True
        return rollback

    @staticmethod
    def _runtime_artifact_path(response: Mapping[str, Any]) -> str:
        return str(response.get("octomap") or response.get("occupancy") or response.get("pcd") or "")

    @classmethod
    def _result(
        cls,
        *,
        success: bool,
        target: str,
        previous: str,
        state: str,
        runtime_consistent: bool,
        restart_required: bool = False,
        **fields: Any,
    ) -> dict[str, Any]:
        return {
            "success": success,
            **fields,
            "transaction": cls._transaction_state(
                target=target,
                previous=previous,
                state=state,
                runtime_consistent=runtime_consistent,
                restart_required=restart_required,
            ),
        }

    @staticmethod
    def _transaction_state(
        *,
        target: str,
        previous: str,
        state: str,
        runtime_consistent: bool,
        restart_required: bool = False,
    ) -> dict[str, Any]:
        return {
            "operation": "activate_map",
            "state": state,
            "previous_active": previous or None,
            "target_map": target,
            "runtime_consistent": runtime_consistent,
            "restart_required": restart_required,
        }

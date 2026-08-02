"""Native navigation command boundary exposed as one runtime capability."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any, Callable

from nav.adapters.native.abi import (
    NativeCommandSession,
    get_native_command_session,
)
from nav.adapters.native.commands import get_native_navigation_client
from nav.adapters.native.exploration_commands import (
    get_native_exploration_command_client,
)
from nav.adapters.native.inspection_commands import (
    get_native_inspection_task_client,
    normalize_route_revision,
)
from nav.adapters.native.operator_motion import get_native_operator_motion_client
from runtime import Module, rpc
from runtime.msgs import (
    NavigationCommandKind,
    NavigationCommandReceipt,
    OperatorMotionReceipt,
)
from runtime.registry import register


@register("nav_commands", "native", description="Typed native navigation command boundary")
class Commands(Module, layer=3):
    """Own the process-wide C++ DDS command client.

    Gateway and domain services discover this capability by its stable
    ``nav.commands`` runtime ID. Native client details stay behind this
    boundary, and every operation waits for the endpoint business ACK.
    """

    runtime_id = "nav.commands"

    def __init__(self, **config: Any) -> None:
        super().__init__(**config)
        self._require_inspection_task_commands = bool(
            config.get("require_inspection_task_commands", False)
        )
        self._session: NativeCommandSession | None = None
        self._navigation_abi_ready = False
        self._operator_motion_abi_ready = False
        self._inspection_task_abi_ready = False
        self._failure = ""

    def setup(self) -> None:
        """Validate both map-product command ABIs on the shared session."""

        self._failure = ""
        self._session = None
        self._navigation_abi_ready = False
        self._operator_motion_abi_ready = False
        self._inspection_task_abi_ready = False
        try:
            session = get_native_command_session(required=True)
        except Exception as exc:
            self._failure = f"native_command_session_unavailable:{exc}"
            raise RuntimeError(self._failure) from exc
        if session is None:
            self._failure = "native_command_session_unavailable"
            raise RuntimeError(self._failure)
        self._session = session
        try:
            session.require_open()
        except Exception as exc:
            self._failure = f"native_command_handle_unavailable:{exc}"
            raise RuntimeError(self._failure) from exc
        try:
            session.ensure_navigation_abi()
            self._navigation_abi_ready = True
        except Exception as exc:
            self._failure = f"native_navigation_abi_unavailable:{exc}"
            raise RuntimeError(self._failure) from exc
        try:
            session.ensure_operator_motion_abi()
            self._operator_motion_abi_ready = True
        except Exception as exc:
            self._failure = f"native_operator_motion_abi_unavailable:{exc}"
            raise RuntimeError(self._failure) from exc
        if self._require_inspection_task_commands:
            try:
                session.ensure_inspection_task_abi()
                self._inspection_task_abi_ready = True
            except Exception as exc:
                self._failure = f"native_inspection_task_abi_unavailable:{exc}"
                raise RuntimeError(self._failure) from exc

    def startup_readiness(self) -> str | None:
        if self._failure:
            return self._failure
        session = self._session
        if session is None:
            return "native_command_session_unavailable"
        try:
            session.require_open()
        except Exception as exc:
            return f"native_command_handle_unavailable:{exc}"
        if not self._navigation_abi_ready:
            return "native_navigation_abi_unverified"
        if not self._operator_motion_abi_ready:
            return "native_operator_motion_abi_unverified"
        if self._require_inspection_task_commands and not self._inspection_task_abi_ready:
            return "native_inspection_task_abi_unverified"
        if not self._running:
            return "not_running"
        return None

    def health(self) -> dict[str, object]:
        """Expose local command-session readiness without issuing a command."""

        readiness = self.startup_readiness()
        handle_open = False
        if self._session is not None:
            try:
                self._session.require_open()
                handle_open = True
            except Exception:
                pass
        return {
            "ok": readiness is None,
            "running": bool(self._running),
            "readiness": readiness or "ready",
            "failure": self._failure,
            "native": {
                "session_acquired": self._session is not None,
                "handle_open": handle_open,
                "navigation_abi_ready": self._navigation_abi_ready,
                "operator_motion_abi_ready": self._operator_motion_abi_ready,
            },
        }

    @rpc
    def send_goal(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        *,
        task_id: str,
        request_id: str | None = None,
    ) -> NavigationCommandReceipt:
        return self._navigation_receipt(
            lambda client: client.start_task(
                float(x),
                float(y),
                float(z),
                float(yaw),
                task_id=task_id,
                request_id=request_id,
            ),
            action="send_goal",
            expected_kind=NavigationCommandKind.GOAL,
            task_id=task_id,
            request_id=request_id,
        )

    @rpc
    def cancel_task(
        self,
        task_id: str,
        reason: str = "cancel",
        request_id: str | None = None,
    ) -> NavigationCommandReceipt:
        return self._navigation_receipt(
            lambda client: client.cancel_task(
                str(task_id or ""),
                str(reason or "cancel"),
                request_id=request_id,
            ),
            action="cancel_task",
            expected_kind=NavigationCommandKind.TASK_CANCEL,
            task_id=task_id,
            request_id=request_id,
        )

    @rpc
    def pause_task(
        self,
        task_id: str,
        reason: str = "operator_pause",
        request_id: str | None = None,
    ) -> NavigationCommandReceipt:
        """Request a stop-confirmed pause for one product task."""

        return self._navigation_receipt(
            lambda client: client.pause_task(
                str(task_id or ""),
                str(reason or "operator_pause"),
                request_id=request_id,
            ),
            action="pause_task",
            expected_kind=NavigationCommandKind.PAUSE_TASK,
            task_id=task_id,
            request_id=request_id,
        )

    @rpc
    def resume_task(
        self,
        task_id: str,
        reason: str = "operator_resume",
        request_id: str | None = None,
    ) -> NavigationCommandReceipt:
        """Request continuation of the same paused product task."""

        return self._navigation_receipt(
            lambda client: client.resume_task(
                str(task_id or ""),
                str(reason or "operator_resume"),
                request_id=request_id,
            ),
            action="resume_task",
            expected_kind=NavigationCommandKind.RESUME_TASK,
            task_id=task_id,
            request_id=request_id,
        )

    @rpc
    def claim(
        self,
        source_id: str,
        source_epoch: int,
        sequence: int,
        *,
        lease_ttl_ms: int,
        request_id: str | None = None,
    ) -> OperatorMotionReceipt:
        return self._operator_motion_receipt(
            lambda client: client.claim(
                str(source_id),
                int(source_epoch),
                int(sequence),
                lease_ttl_ms=int(lease_ttl_ms),
                request_id=request_id,
            ),
            action="claim",
        )

    @rpc
    def sample(
        self,
        source_id: str,
        source_epoch: int,
        sequence: int,
        vx: float,
        vy: float,
        wz: float,
        *,
        deadman: bool = True,
        freshness_budget_ms: int = 350,
        request_id: str | None = None,
    ) -> bool:
        if not isinstance(deadman, bool):
            raise TypeError("operator motion deadman must be a boolean")
        return self._operator_motion_sample(
            lambda client: client.sample(
                str(source_id),
                int(source_epoch),
                int(sequence),
                float(vx),
                float(vy),
                float(wz),
                deadman=deadman,
                freshness_budget_ms=int(freshness_budget_ms),
                request_id=request_id,
            ),
            action="sample",
        )

    @rpc
    def hold(
        self,
        source_id: str,
        source_epoch: int,
        sequence: int,
        *,
        reason: str = "operator_hold",
        request_id: str | None = None,
    ) -> OperatorMotionReceipt:
        return self._operator_motion_receipt(
            lambda client: client.hold(
                str(source_id),
                int(source_epoch),
                int(sequence),
                reason=str(reason or "operator_hold"),
                request_id=request_id,
            ),
            action="hold",
        )

    @rpc
    def release(
        self,
        source_id: str,
        source_epoch: int,
        sequence: int,
        *,
        reason: str = "operator_release",
        request_id: str | None = None,
    ) -> OperatorMotionReceipt:
        return self._operator_motion_receipt(
            lambda client: client.release(
                str(source_id),
                int(source_epoch),
                int(sequence),
                reason=str(reason or "operator_release"),
                request_id=request_id,
            ),
            action="release",
        )

    @rpc
    def stop_motion(self, reason: str = "stop", request_id: str | None = None) -> bool:
        return self._navigation(lambda client: client.stop(str(reason or "stop"), request_id=request_id))

    @rpc
    def estop(self, reason: str = "estop", request_id: str | None = None) -> bool:
        return self._navigation(lambda client: client.estop(str(reason or "estop"), request_id=request_id))

    @rpc
    def clear_estop(
        self,
        reason: str = "clear_estop",
        request_id: str | None = None,
    ) -> bool:
        return self._navigation(
            lambda client: client.clear_estop(
                str(reason or "clear_estop"),
                request_id=request_id,
            )
        )

    @rpc
    def resume_autonomy(
        self,
        reason: str = "resume_autonomy",
        request_id: str | None = None,
    ) -> bool:
        return self._navigation(
            lambda client: client.resume_autonomy(
                str(reason or "resume_autonomy"),
                request_id=request_id,
            )
        )

    @rpc
    def start_exploration(
        self,
        exploration_run_id: str,
        session_id: str,
        reason: str = "operator_start",
        request_id: str | None = None,
    ) -> dict[str, object]:
        return self._exploration_receipt(
            lambda client: client.start(
                exploration_run_id=str(exploration_run_id or ""),
                session_id=str(session_id or ""),
                reason=str(reason or "operator_start"),
                request_id=request_id,
            ),
            action="start",
            exploration_run_id=exploration_run_id,
            request_id=request_id,
        )

    @rpc
    def pause_exploration(
        self,
        exploration_run_id: str,
        session_id: str,
        reason: str = "operator_pause",
        request_id: str | None = None,
    ) -> dict[str, object]:
        return self._exploration_receipt(
            lambda client: client.pause(
                exploration_run_id=str(exploration_run_id or ""),
                session_id=str(session_id or ""),
                reason=str(reason or "operator_pause"),
                request_id=request_id,
            ),
            action="pause",
            exploration_run_id=exploration_run_id,
            request_id=request_id,
        )

    @rpc
    def resume_exploration(
        self,
        exploration_run_id: str,
        session_id: str,
        reason: str = "operator_resume",
        request_id: str | None = None,
    ) -> dict[str, object]:
        return self._exploration_receipt(
            lambda client: client.resume(
                exploration_run_id=str(exploration_run_id or ""),
                session_id=str(session_id or ""),
                reason=str(reason or "operator_resume"),
                request_id=request_id,
            ),
            action="resume",
            exploration_run_id=exploration_run_id,
            request_id=request_id,
        )

    @rpc
    def stop_exploration(
        self,
        exploration_run_id: str,
        session_id: str,
        reason: str = "operator_stop",
        request_id: str | None = None,
    ) -> dict[str, object]:
        return self._exploration_receipt(
            lambda client: client.stop(
                exploration_run_id=str(exploration_run_id or ""),
                session_id=str(session_id or ""),
                reason=str(reason or "operator_stop"),
                request_id=request_id,
            ),
            action="stop",
            exploration_run_id=exploration_run_id,
            request_id=request_id,
        )

    @rpc
    def set_directed_exploration_target(
        self,
        x: float,
        y: float,
        ttl_s: float,
        exploration_run_id: str,
        session_id: str,
        reason: str = "operator_directed_explore",
        request_id: str | None = None,
    ) -> dict[str, object]:
        return self._exploration_receipt(
            lambda client: client.set_directed_target(
                float(x),
                float(y),
                float(ttl_s),
                exploration_run_id=str(exploration_run_id or ""),
                session_id=str(session_id or ""),
                reason=str(reason or "operator_directed_explore"),
                request_id=request_id,
            ),
            action="set_directed_target",
            exploration_run_id=exploration_run_id,
            request_id=request_id,
        )

    @rpc
    def clear_directed_exploration_target(
        self,
        exploration_run_id: str,
        session_id: str,
        reason: str = "operator_clear_directed_explore",
        request_id: str | None = None,
    ) -> dict[str, object]:
        return self._exploration_receipt(
            lambda client: client.clear_directed_target(
                exploration_run_id=str(exploration_run_id or ""),
                session_id=str(session_id or ""),
                reason=str(reason or "operator_clear_directed_explore"),
                request_id=request_id,
            ),
            action="clear_directed_target",
            exploration_run_id=exploration_run_id,
            request_id=request_id,
        )

    @rpc
    def start_inspection_task(
        self,
        task_id: str,
        route_id: str,
        revision: int = 0,
        request_id: str | None = None,
    ) -> bool:
        clean_task_id = str(task_id or "").strip()
        clean_route_id = str(route_id or "").strip()
        if not clean_task_id:
            raise RuntimeError("inspection task_id is required")
        if not clean_route_id:
            raise RuntimeError("inspection route_id is required")
        route_revision = normalize_route_revision(revision)
        return self._inspection_task(
            lambda client: client.start(
                clean_task_id,
                clean_route_id,
                revision=route_revision,
                request_id=request_id,
            )
        )

    @rpc
    def pause_inspection_task(
        self,
        task_id: str,
        reason: str = "operator_pause",
        request_id: str | None = None,
    ) -> bool:
        return self._inspection_task_command(
            "pause",
            task_id,
            reason,
            request_id,
        )

    @rpc
    def resume_inspection_task(
        self,
        task_id: str,
        reason: str = "operator_resume",
        request_id: str | None = None,
    ) -> bool:
        return self._inspection_task_command(
            "resume",
            task_id,
            reason,
            request_id,
        )

    @rpc
    def cancel_inspection_task(
        self,
        task_id: str,
        reason: str = "operator_cancel",
        request_id: str | None = None,
    ) -> bool:
        return self._inspection_task_command(
            "cancel",
            task_id,
            reason,
            request_id,
        )

    @staticmethod
    def _navigation(operation: Callable[[Any], None]) -> bool:
        client = get_native_navigation_client(required=True)
        if client is None:
            raise RuntimeError("native navigation command boundary is unavailable")
        operation(client)
        return True

    @staticmethod
    def _navigation_receipt(
        operation: Callable[[Any], Any],
        *,
        action: str,
        expected_kind: NavigationCommandKind,
        task_id: str,
        request_id: str | None,
    ) -> NavigationCommandReceipt:
        normalized_task_id = str(task_id or "").strip()
        if not normalized_task_id:
            raise ValueError(f"native navigation {action} task_id is required")
        client = get_native_navigation_client(required=True)
        if client is None:
            raise RuntimeError("native navigation command boundary is unavailable")
        receipt = operation(client)
        if not isinstance(receipt, NavigationCommandReceipt):
            raise RuntimeError(f"native navigation {action} returned an invalid receipt")
        if receipt.kind != int(expected_kind):
            raise RuntimeError(f"native navigation {action} returned the wrong command kind")
        if receipt.task_id != normalized_task_id:
            raise RuntimeError(f"native navigation {action} returned the wrong task_id")
        normalized_request_id = str(request_id or "").strip()
        request_matches = receipt.request_id == normalized_request_id
        if action not in {"pause_task", "resume_task"}:
            request_matches = request_matches or receipt.request_id.startswith(
                f"{normalized_request_id}-clock-retry-"
            )
        if normalized_request_id and not request_matches:
            raise RuntimeError(f"native navigation {action} returned the wrong request_id")
        return receipt

    @staticmethod
    def _operator_motion_receipt(
        operation: Callable[[Any], Any],
        *,
        action: str,
    ) -> OperatorMotionReceipt:
        client = get_native_operator_motion_client(required=True)
        if client is None:
            raise RuntimeError("native operator motion command boundary is unavailable")
        receipt = operation(client)
        if not isinstance(receipt, OperatorMotionReceipt):
            raise RuntimeError(f"native operator motion {action} returned an invalid receipt")
        return receipt

    @staticmethod
    def _operator_motion_sample(
        operation: Callable[[Any], Any],
        *,
        action: str,
    ) -> bool:
        client = get_native_operator_motion_client(required=True)
        if client is None:
            raise RuntimeError("native operator motion command boundary is unavailable")
        submitted = operation(client)
        if submitted is not True:
            raise RuntimeError(f"native operator motion {action} returned an invalid submission result")
        return True

    @staticmethod
    def _exploration_receipt(
        operation: Callable[[Any], Any],
        *,
        action: str,
        exploration_run_id: str,
        request_id: str | None,
    ) -> dict[str, object]:
        client = get_native_exploration_command_client(required=True)
        if client is None:
            raise RuntimeError("native exploration command boundary is unavailable")
        raw_receipt = operation(client)
        if not isinstance(raw_receipt, Mapping):
            raise RuntimeError(f"native exploration {action} returned an invalid receipt")

        accepted = raw_receipt.get("accepted")
        duplicate = raw_receipt.get("duplicate")
        receipt_request_id = raw_receipt.get("request_id")
        receipt_run_id = raw_receipt.get("exploration_run_id")
        reason = raw_receipt.get("reason")
        if (
            not isinstance(accepted, bool)
            or not isinstance(duplicate, bool)
            or not isinstance(receipt_request_id, str)
            or not receipt_request_id
            or not isinstance(receipt_run_id, str)
            or not isinstance(reason, str)
        ):
            raise RuntimeError(f"native exploration {action} returned an invalid receipt")

        expected_run_id = str(exploration_run_id or "")
        if receipt_run_id != expected_run_id:
            raise RuntimeError(
                f"native exploration {action} returned the wrong exploration_run_id"
            )
        expected_request_id = str(request_id or "")
        if expected_request_id and receipt_request_id != expected_request_id:
            raise RuntimeError(f"native exploration {action} returned the wrong request_id")
        return {
            "accepted": accepted,
            "request_id": receipt_request_id,
            "exploration_run_id": receipt_run_id,
            "reason": reason,
            "duplicate": duplicate,
        }

    @staticmethod
    def _inspection_task(operation: Callable[[Any], None]) -> bool:
        client = get_native_inspection_task_client(required=True)
        if client is None:
            raise RuntimeError("native inspection task command boundary is unavailable")
        operation(client)
        return True

    @classmethod
    def _inspection_task_command(
        cls,
        operation_name: str,
        task_id: str,
        reason: str,
        request_id: str | None,
    ) -> bool:
        clean_task_id = str(task_id or "").strip()
        if not clean_task_id:
            raise RuntimeError("inspection task_id is required")
        command = str(operation_name or "").strip()
        if command not in {"pause", "resume", "cancel"}:
            raise RuntimeError("inspection task command is invalid")
        default_reason = f"operator_{command}"
        return cls._inspection_task(
            lambda client: getattr(client, command)(
                clean_task_id,
                str(reason or default_reason),
                request_id=request_id,
            )
        )


__all__ = ["Commands"]

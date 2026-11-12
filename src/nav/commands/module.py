"""Native navigation command boundary exposed as one runtime capability."""

from __future__ import annotations

from collections.abc import Mapping
from functools import cached_property
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
    def preview_plan(
        self,
        x: float,
        y: float,
        z: float,
        *,
        acceptance_radius_m: float | None = None,
    ) -> dict[str, object]:
        """Run one read-only plan through the native navigation endpoint."""

        return self._navigation_client.preview_plan(
            x, y, z,
            acceptance_radius_m=acceptance_radius_m,
        )

    @rpc
    def query_semantic_views(
        self,
        boot_id: str,
        map_id: str,
        map_content_epoch: int,
        camera_range_m: float,
        camera_horizontal_fov_rad: float,
        frame_id: str = "map",
        frame_epoch: int = 0,
        reference_z: float = 0.0,
        views: list[dict[str, object]] | None = None,
        timeout_ms: int = 3000,
    ) -> dict[str, object]:
        """Query native 3D observation candidates without authorizing motion."""

        return self._navigation_client.query_semantic_views(
            boot_id=boot_id,
            map_id=map_id,
            map_content_epoch=map_content_epoch,
            frame_id=frame_id,
            frame_epoch=frame_epoch,
            reference_z=reference_z,
            camera_range_m=camera_range_m,
            camera_horizontal_fov_rad=camera_horizontal_fov_rad,
            views=views,
            timeout_ms=timeout_ms,
        )

    @rpc
    def send_goal(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float | None,
        *,
        task_id: str,
        request_id: str | None = None,
        max_speed_mps: float | None = None,
        acceptance_radius_m: float | None = None,
    ) -> NavigationCommandReceipt:
        return self._navigation_client.start_task(
            x, y, z, yaw, task_id=task_id, request_id=request_id,
            max_speed_mps=max_speed_mps, acceptance_radius_m=acceptance_radius_m,
        )

    @rpc
    def cancel_task(
        self, task_id: str, reason: str = "cancel", request_id: str | None = None,
    ) -> NavigationCommandReceipt:
        return self._navigation_client.cancel_task(task_id, reason, request_id=request_id)

    @rpc
    def set_task_speed_limit(
        self, task_id: str, max_speed_mps: float, request_id: str | None = None,
    ) -> NavigationCommandReceipt:
        return self._navigation_client.set_task_speed_limit(task_id, max_speed_mps, request_id=request_id)

    @rpc
    def select_planner(self, name: str, request_id: str | None = None) -> dict[str, object]:
        return self._navigation_control_receipt(
            lambda client: client.select_planner(name, request_id=request_id),
            action="select_planner", expected_kind=NavigationCommandKind.SELECT_PLANNER,
            request_id=request_id,
        )

    @rpc
    def pause_task(
        self, task_id: str, reason: str = "operator_pause", request_id: str | None = None,
    ) -> NavigationCommandReceipt:
        return self._navigation_client.pause_task(task_id, reason, request_id=request_id)

    @rpc
    def resume_task(
        self, task_id: str, reason: str = "operator_resume", request_id: str | None = None,
    ) -> NavigationCommandReceipt:
        return self._navigation_client.resume_task(task_id, reason, request_id=request_id)

    @rpc
    def operator_motion_status(self, max_age_ms: int = 250) -> dict[str, object] | None:
        """Read the native authority snapshot, including its freshness."""
        client = get_native_operator_motion_client(required=True)
        if client is None:
            raise RuntimeError("native operator motion status boundary is unavailable")
        return client.status(max_age_ms)

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
            )
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
        manual_mode: bool = False,
        freshness_budget_ms: int = 350,
        request_id: str | None = None,
    ) -> bool:
        if not isinstance(deadman, bool):
            raise TypeError("operator motion deadman must be a boolean")
        if not isinstance(manual_mode, bool):
            raise TypeError("operator motion manual_mode must be a boolean")
        return self._operator_motion_sample(
            lambda client: client.sample(
                str(source_id),
                int(source_epoch),
                int(sequence),
                float(vx),
                float(vy),
                float(wz),
                deadman=deadman,
                manual_mode=manual_mode,
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
            )
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
            )
        )

    @rpc
    def stop_motion(self, reason: str = "stop", request_id: str | None = None) -> bool:
        self._navigation_client.stop(reason, request_id=request_id)
        return True

    @rpc
    def estop(self, reason: str = "estop", request_id: str | None = None) -> bool:
        self._navigation_client.estop(reason, request_id=request_id)
        return True

    @rpc
    def clear_estop(
        self,
        reason: str = "clear_estop",
        request_id: str | None = None,
    ) -> bool:
        self._navigation_client.clear_estop(reason, request_id=request_id)
        return True

    @rpc
    def resume_autonomy(
        self,
        reason: str = "resume_autonomy",
        request_id: str | None = None,
    ) -> bool:
        self._navigation_client.resume_autonomy(reason, request_id=request_id)
        return True

    @rpc
    def resume_autonomy_with_receipt(
        self,
        reason: str = "resume_autonomy",
        request_id: str | None = None,
    ) -> dict[str, object]:
        return self._navigation_control_receipt(
            lambda client: client.resume_autonomy_with_receipt(
                str(reason or "resume_autonomy"),
                request_id=request_id,
            ),
            action="resume_autonomy",
            expected_kind=NavigationCommandKind.RESUME_AUTONOMY,
            request_id=request_id,
        )

    @rpc
    def stand(self, reason: str = "stand", request_id: str | None = None) -> dict[str, object]:
        return self._navigation_control_receipt(
            lambda client: client.stand(str(reason or "stand"), request_id=request_id),
            action="stand", expected_kind=NavigationCommandKind.STAND, request_id=request_id,
        )

    @rpc
    def sit(self, reason: str = "sit", request_id: str | None = None) -> dict[str, object]:
        return self._navigation_control_receipt(
            lambda client: client.sit(str(reason or "sit"), request_id=request_id),
            action="sit", expected_kind=NavigationCommandKind.SIT, request_id=request_id,
        )

    @rpc
    def enable_motors(
        self, reason: str = "enable_motors", request_id: str | None = None,
    ) -> dict[str, object]:
        return self._navigation_control_receipt(
            lambda client: client.enable_motors(reason, request_id=request_id),
            action="enable_motors", expected_kind=NavigationCommandKind.ENABLE_MOTORS,
            request_id=request_id,
        )

    @rpc
    def disable_motors(
        self, reason: str = "disable_motors", request_id: str | None = None,
    ) -> dict[str, object]:
        return self._navigation_control_receipt(
            lambda client: client.disable_motors(reason, request_id=request_id),
            action="disable_motors", expected_kind=NavigationCommandKind.DISABLE_MOTORS,
            request_id=request_id,
        )

    @rpc
    def switch_profile(
        self, name: str, reason: str = "switch_profile", request_id: str | None = None,
    ) -> dict[str, object]:
        return self._navigation_control_receipt(
            lambda client: client.switch_profile(name, reason, request_id=request_id),
            action="switch_profile", expected_kind=NavigationCommandKind.SWITCH_PROFILE,
            request_id=request_id,
        )

    @rpc
    def external_enter(self, reason="external_enter", request_id=None):
        return self._navigation_control_receipt(
            lambda client: client.external_enter(reason, request_id=request_id),
            action="external_enter", expected_kind=NavigationCommandKind.ENTER_EXTERNAL,
            request_id=request_id,
        )

    @rpc
    def external_enable(self, session_id, reason="external_enable", request_id=None):
        return self._navigation_control_receipt(
            lambda client: client.external_enable(session_id, reason, request_id=request_id),
            action="external_enable", expected_kind=NavigationCommandKind.ENABLE_EXTERNAL,
            request_id=request_id,
        )

    @rpc
    def external_disable(self, session_id, reason="external_disable", request_id=None):
        return self._navigation_control_receipt(
            lambda client: client.external_disable(session_id, reason, request_id=request_id),
            action="external_disable", expected_kind=NavigationCommandKind.DISABLE_MOTORS,
            request_id=request_id,
        )

    @rpc
    def external_exit(self, session_id, reason="external_exit", request_id=None):
        return self._navigation_control_receipt(
            lambda client: client.external_exit(session_id, reason, request_id=request_id),
            action="external_exit", expected_kind=NavigationCommandKind.EXIT_EXTERNAL,
            request_id=request_id,
        )

    @rpc
    def external_frame(self, session_id, sequence, joints, input_window):
        client = get_native_navigation_client(required=True)
        return client.external_frame(session_id, sequence, joints, input_window)

    @rpc
    def external_heartbeat(self, session_id):
        return get_native_navigation_client(required=True).external_heartbeat(session_id)

    @rpc
    def external_status(self):
        return get_native_navigation_client(required=True).external_status()

    @rpc
    def start_exploration(
        self,
        exploration_run_id: str,
        product_session_id: str,
        reason: str = "operator_start",
        request_id: str | None = None,
    ) -> dict[str, object]:
        return self._exploration_receipt(
            lambda client: client.start(
                exploration_run_id=str(exploration_run_id or ""),
                product_session_id=str(product_session_id or ""),
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
        product_session_id: str,
        reason: str = "operator_pause",
        request_id: str | None = None,
    ) -> dict[str, object]:
        return self._exploration_receipt(
            lambda client: client.pause(
                exploration_run_id=str(exploration_run_id or ""),
                product_session_id=str(product_session_id or ""),
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
        product_session_id: str,
        reason: str = "operator_resume",
        request_id: str | None = None,
    ) -> dict[str, object]:
        return self._exploration_receipt(
            lambda client: client.resume(
                exploration_run_id=str(exploration_run_id or ""),
                product_session_id=str(product_session_id or ""),
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
        product_session_id: str,
        reason: str = "operator_stop",
        request_id: str | None = None,
    ) -> dict[str, object]:
        return self._exploration_receipt(
            lambda client: client.stop(
                exploration_run_id=str(exploration_run_id or ""),
                product_session_id=str(product_session_id or ""),
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
        product_session_id: str,
        reason: str = "operator_directed_explore",
        request_id: str | None = None,
    ) -> dict[str, object]:
        return self._exploration_receipt(
            lambda client: client.set_directed_target(
                float(x),
                float(y),
                float(ttl_s),
                exploration_run_id=str(exploration_run_id or ""),
                product_session_id=str(product_session_id or ""),
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
        product_session_id: str,
        reason: str = "operator_clear_directed_explore",
        request_id: str | None = None,
    ) -> dict[str, object]:
        return self._exploration_receipt(
            lambda client: client.clear_directed_target(
                exploration_run_id=str(exploration_run_id or ""),
                product_session_id=str(product_session_id or ""),
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

    @cached_property
    def _navigation_client(self):
        client = get_native_navigation_client(required=True)
        if client is None:
            raise RuntimeError("native navigation command boundary is unavailable")
        return client

    def _navigation_control_receipt(
        self, operation: Callable[[Any], Any],
        *,
        action: str,
        expected_kind: NavigationCommandKind,
        request_id: str | None,
    ) -> dict[str, object]:
        raw_receipt = operation(self._navigation_client)
        if not isinstance(raw_receipt, Mapping):
            raise RuntimeError(f"native navigation {action} returned an invalid receipt")

        accepted = raw_receipt.get("accepted")
        kind = raw_receipt.get("kind")
        task_id = raw_receipt.get("task_id")
        receipt_request_id = raw_receipt.get("request_id")
        endpoint_timestamp_s = raw_receipt.get("endpoint_timestamp_s")
        reason = raw_receipt.get("reason")
        if (
            not isinstance(accepted, bool)
            or kind != int(expected_kind)
            or task_id not in (None, "")
            or not isinstance(receipt_request_id, str)
            or not receipt_request_id
            or not isinstance(endpoint_timestamp_s, (int, float))
            or isinstance(endpoint_timestamp_s, bool)
            or not isinstance(reason, str)
        ):
            raise RuntimeError(f"native navigation {action} returned an invalid receipt")

        expected_request_id = str(request_id or "").strip()
        request_matches = receipt_request_id == expected_request_id
        request_matches = request_matches or receipt_request_id.startswith(
            f"{expected_request_id}-clock-retry-"
        )
        if expected_request_id and not request_matches:
            raise RuntimeError(f"native navigation {action} returned the wrong request_id")
        return {
            "accepted": accepted,
            "kind": int(kind),
            "task_id": task_id,
            "request_id": receipt_request_id,
            "endpoint_timestamp_s": float(endpoint_timestamp_s),
            "reason": reason,
        }

    @staticmethod
    def _operator_motion_receipt(
        operation: Callable[[Any], Any],
    ) -> OperatorMotionReceipt:
        client = get_native_operator_motion_client(required=True)
        if client is None:
            raise RuntimeError("native operator motion command boundary is unavailable")
        return operation(client)

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

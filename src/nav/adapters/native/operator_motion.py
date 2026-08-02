"""Typed operator-motion interface over the process-wide native DDS session."""

from __future__ import annotations

import ctypes
import os
from typing import Any

from nav.adapters.native.abi import (
    NATIVE_OPERATOR_MOTION_RECEIPT_ABI_VERSION,
    NativeCommandClientError,
    NativeCommandSession,
    _NativeOperatorMotionReceiptV1,
    get_native_command_session,
)
from runtime.msgs import OperatorMotionAction, OperatorMotionReceipt

OperatorMotionClientError = NativeCommandClientError


def _require_sequence(sequence: int) -> int:
    value = int(sequence)
    if value <= 0:
        raise ValueError("operator motion sequence must be positive")
    return value


def _require_deadman(deadman: bool) -> bool:
    if not isinstance(deadman, bool):
        raise TypeError("operator motion deadman must be a boolean")
    return deadman


def _decode_fixed_text(value: bytes | bytearray | memoryview) -> str:
    return bytes(value).split(b"\0", 1)[0].decode("utf-8", errors="replace")


def _receipt_from_native(
    native: _NativeOperatorMotionReceiptV1,
    *,
    expected_action: OperatorMotionAction,
    request_id: str,
    source_id: str,
    source_epoch: int,
    source_sequence: int,
) -> OperatorMotionReceipt:
    expected_size = ctypes.sizeof(_NativeOperatorMotionReceiptV1)
    if int(native.abi_version) != NATIVE_OPERATOR_MOTION_RECEIPT_ABI_VERSION:
        raise NativeCommandClientError(
            "native operator motion receipt ABI version mismatch: "
            f"expected {NATIVE_OPERATOR_MOTION_RECEIPT_ABI_VERSION}, "
            f"got {int(native.abi_version)}"
        )
    if int(native.struct_size) != expected_size:
        raise NativeCommandClientError(
            "native operator motion receipt struct size mismatch: "
            f"expected {expected_size}, got {int(native.struct_size)}"
        )
    if int(native.accepted) not in (0, 1):
        raise NativeCommandClientError(f"native operator motion receipt has invalid accepted={int(native.accepted)}")

    actual_request_id = _decode_fixed_text(native.request_id)
    actual_source_id = _decode_fixed_text(native.source_id)
    if request_id and actual_request_id != request_id:
        raise NativeCommandClientError("native operator motion receipt request_id mismatch")
    if actual_source_id != source_id:
        raise NativeCommandClientError("native operator motion receipt source_id mismatch")
    if int(native.source_epoch) != source_epoch:
        raise NativeCommandClientError("native operator motion receipt source_epoch mismatch")
    if int(native.source_sequence) != source_sequence:
        raise NativeCommandClientError("native operator motion receipt source_sequence mismatch")
    if int(native.action) != int(expected_action):
        raise NativeCommandClientError("native operator motion receipt action mismatch")

    try:
        return OperatorMotionReceipt(
            accepted=bool(native.accepted),
            action=int(native.action),
            request_id=actual_request_id,
            source_id=actual_source_id,
            source_epoch=int(native.source_epoch),
            source_sequence=int(native.source_sequence),
            accepted_sequence=int(native.accepted_sequence),
            final_output_sequence=int(native.final_output_sequence),
            endpoint_timestamp_s=float(native.endpoint_timestamp_s),
            reason=_decode_fixed_text(native.reason),
        )
    except ValueError as exc:
        raise NativeCommandClientError(f"invalid native operator motion receipt: {exc}") from exc


class NativeOperatorMotionClient:
    """Narrow client for claim/sample/hold/release operator motion commands."""

    def __init__(
        self,
        library_path: str | os.PathLike[str],
        *,
        domain_id: int = 0,
        timeout_ms: int = 1000,
        teleop_timeout_ms: int | None = None,
        operator_motion_timeout_ms: int | None = None,
        library: Any | None = None,
    ) -> None:
        self._session = NativeCommandSession(
            library_path,
            domain_id=domain_id,
            timeout_ms=timeout_ms,
            teleop_timeout_ms=teleop_timeout_ms,
            operator_motion_timeout_ms=operator_motion_timeout_ms,
            library=library,
        )
        self._owns_session = True
        try:
            self._session.ensure_operator_motion_abi()
        except Exception:
            self._session.close()
            raise

    @classmethod
    def _from_session(cls, session: NativeCommandSession) -> NativeOperatorMotionClient:
        instance = cls.__new__(cls)
        instance._session = session
        instance._owns_session = False
        session.ensure_operator_motion_abi()
        return instance

    def _call_receipt(
        self,
        function_name: str,
        expected_action: OperatorMotionAction,
        *,
        request_id: str,
        source_id: str,
        source_epoch: int,
        source_sequence: int,
        arguments: tuple[object, ...],
    ) -> OperatorMotionReceipt:
        native = _NativeOperatorMotionReceiptV1()
        native.abi_version = NATIVE_OPERATOR_MOTION_RECEIPT_ABI_VERSION
        native.struct_size = ctypes.sizeof(_NativeOperatorMotionReceiptV1)
        self._session.call(function_name, *arguments, ctypes.byref(native))
        return _receipt_from_native(
            native,
            expected_action=expected_action,
            request_id=request_id,
            source_id=source_id,
            source_epoch=source_epoch,
            source_sequence=source_sequence,
        )

    def claim(
        self,
        source_id: str,
        source_epoch: int,
        sequence: int,
        *,
        lease_ttl_ms: int,
        request_id: str | None = None,
    ) -> OperatorMotionReceipt:
        """Claim motion authority for one operator source and wait for ACK."""

        clean_request_id = str(request_id or "")
        clean_source_id = str(source_id)
        clean_source_epoch = int(source_epoch)
        clean_sequence = _require_sequence(sequence)
        return self._call_receipt(
            "lingtu_nav_client_operator_motion_claim_with_receipt_v1",
            OperatorMotionAction.CLAIM,
            request_id=clean_request_id,
            source_id=clean_source_id,
            source_epoch=clean_source_epoch,
            source_sequence=clean_sequence,
            arguments=(
                clean_request_id.encode("utf-8"),
                clean_source_id.encode("utf-8"),
                clean_source_epoch,
                clean_sequence,
                int(lease_ttl_ms),
                self._session.operator_motion_timeout_ms,
            ),
        )

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
        """Write one replaceable joystick sample to DDS without an endpoint ACK."""

        checked_deadman = _require_deadman(deadman)
        self._session.call(
            "lingtu_nav_client_operator_motion_sample",
            str(request_id or "").encode("utf-8"),
            str(source_id).encode("utf-8"),
            int(source_epoch),
            _require_sequence(sequence),
            1 if checked_deadman else 0,
            float(vx),
            float(vy),
            float(wz),
            int(freshness_budget_ms),
            self._session.operator_motion_timeout_ms,
        )
        return True

    def hold(
        self,
        source_id: str,
        source_epoch: int,
        sequence: int,
        *,
        reason: str = "operator_hold",
        request_id: str | None = None,
    ) -> OperatorMotionReceipt:
        """Ask the native endpoint to enter a zero-output barrier."""

        clean_request_id = str(request_id or "")
        clean_source_id = str(source_id)
        clean_source_epoch = int(source_epoch)
        clean_sequence = _require_sequence(sequence)
        return self._call_receipt(
            "lingtu_nav_client_operator_motion_hold_with_receipt_v1",
            OperatorMotionAction.HOLD,
            request_id=clean_request_id,
            source_id=clean_source_id,
            source_epoch=clean_source_epoch,
            source_sequence=clean_sequence,
            arguments=(
                clean_request_id.encode("utf-8"),
                clean_source_id.encode("utf-8"),
                clean_source_epoch,
                clean_sequence,
                str(reason or "operator_hold").encode("utf-8"),
                self._session.operator_motion_timeout_ms,
            ),
        )

    def release(
        self,
        source_id: str,
        source_epoch: int,
        sequence: int,
        *,
        reason: str = "operator_release",
        request_id: str | None = None,
    ) -> OperatorMotionReceipt:
        """Release this operator source after the native zero barrier."""

        clean_request_id = str(request_id or "")
        clean_source_id = str(source_id)
        clean_source_epoch = int(source_epoch)
        clean_sequence = _require_sequence(sequence)
        return self._call_receipt(
            "lingtu_nav_client_operator_motion_release_with_receipt_v1",
            OperatorMotionAction.RELEASE,
            request_id=clean_request_id,
            source_id=clean_source_id,
            source_epoch=clean_source_epoch,
            source_sequence=clean_sequence,
            arguments=(
                clean_request_id.encode("utf-8"),
                clean_source_id.encode("utf-8"),
                clean_source_epoch,
                clean_sequence,
                str(reason or "operator_release").encode("utf-8"),
                self._session.operator_motion_timeout_ms,
            ),
        )

    def close(self) -> None:
        """Close a directly owned session; factory views leave it shared."""

        if self._owns_session:
            self._session.close()


def get_native_operator_motion_client(
    *,
    required: bool = False,
) -> NativeOperatorMotionClient | None:
    """Return the operator-motion view of the process-wide native command session."""

    session = get_native_command_session(required=required)
    if session is None:
        return None
    return NativeOperatorMotionClient._from_session(session)

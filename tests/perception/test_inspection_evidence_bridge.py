"""Reject pre-simplification native layouts before they can write ctypes memory."""

import ctypes
from types import SimpleNamespace

import pytest

from perception.inspection.native_bridge import (
    ABI_VERSION,
    InspectionEvidenceBridgeError,
    NativeInspectionEvidenceBridge,
    _Result,
)


def library(version: int | None) -> tuple[SimpleNamespace, list[str]]:
    calls: list[str] = []

    def create(_domain: int) -> int:
        calls.append("create")
        return 1

    lib = SimpleNamespace(
        lingtu_inspection_evidence_bridge_create=create,
        lingtu_inspection_evidence_bridge_destroy=lambda _handle: calls.append("destroy"),
        lingtu_inspection_evidence_bridge_take_request=lambda *_: 0,
        lingtu_inspection_evidence_bridge_write_result=lambda *_: 0,
        lingtu_inspection_evidence_bridge_last_error=lambda *_: b"",
    )
    if version is not None:
        lib.lingtu_inspection_evidence_bridge_abi_version = lambda: version
    return lib, calls


@pytest.mark.parametrize("version", [None, 1, ABI_VERSION + 1])
def test_old_or_mismatched_library_is_rejected_before_native_calls(version: int | None) -> None:
    lib, calls = library(version)
    with pytest.raises(InspectionEvidenceBridgeError, match="ABI"):
        NativeInspectionEvidenceBridge(library=lib)
    assert calls == []


def test_matching_library_can_receive_and_close() -> None:
    lib, calls = library(ABI_VERSION)
    with NativeInspectionEvidenceBridge(library=lib) as bridge:
        assert bridge.take_request() is None
    assert calls == ["create", "destroy"]


@pytest.mark.parametrize("persisted", [False, True])
def test_artifact_handle_reuses_operation_id_only_after_persistence(persisted: bool) -> None:
    lib, _ = library(ABI_VERSION)
    observed: list[tuple[bytes, bytes]] = []

    def write(_handle: int, output: object) -> int:
        result = ctypes.cast(output, ctypes.POINTER(_Result)).contents
        observed.append((result.request_id, result.evidence_id))
        return 0

    lib.lingtu_inspection_evidence_bridge_write_result = write
    with NativeInspectionEvidenceBridge(library=lib) as bridge:
        bridge.write_result(request_id="action-1", persisted=persisted)
    assert observed == [(b"action-1", b"action-1" if persisted else b"")]

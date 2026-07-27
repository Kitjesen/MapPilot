from __future__ import annotations

import pytest

from nav.adapters.native import abi as navigation_abi
from nav.adapters.native.commands import get_native_navigation_client
from nav.adapters.native.inspection_commands import (
    InspectionCommandClientError,
    NativeInspectionCommandClient,
    get_native_inspection_command_client,
)


class _Function:
    def __init__(self, callback):
        self._callback = callback
        self.argtypes = None
        self.restype = None

    def __call__(self, *args):
        return self._callback(*args)


class _InspectionLibrary:
    """Inspection-only ABI fake; navigation command symbols are deliberately absent."""

    def __init__(self) -> None:
        self.calls = []
        self.error = b""
        self.lingtu_nav_client_abi_version = _Function(lambda: navigation_abi.NATIVE_COMMAND_ABI_VERSION)
        self.lingtu_nav_client_capabilities = _Function(lambda: 0x03)
        self.lingtu_nav_client_create = _Function(self._create)
        self.lingtu_nav_client_destroy = _Function(self._destroy)
        self.lingtu_nav_client_start_inspection = _Function(self._start)
        self.lingtu_nav_client_pause_inspection = _Function(self._pause)
        self.lingtu_nav_client_resume_inspection = _Function(self._resume)
        self.lingtu_nav_client_cancel_inspection = _Function(self._cancel)
        self.lingtu_nav_client_last_error = _Function(lambda _handle: self.error)

    def _create(self, domain_id):
        self.calls.append(("create", domain_id))
        return 73

    def _destroy(self, handle):
        self.calls.append(("destroy", handle))

    def _start(self, handle, request_id, route_id, revision, timeout_ms):
        self.calls.append(("start", handle, request_id, route_id, revision, timeout_ms))
        return 0

    def _pause(self, handle, request_id, reason, timeout_ms):
        self.calls.append(("pause", handle, request_id, reason, timeout_ms))
        return 0

    def _resume(self, handle, request_id, reason, timeout_ms):
        self.calls.append(("resume", handle, request_id, reason, timeout_ms))
        return 0

    def _cancel(self, handle, request_id, reason, timeout_ms):
        self.calls.append(("cancel", handle, request_id, reason, timeout_ms))
        return 0


class _CombinedLibrary(_InspectionLibrary):
    def __init__(self) -> None:
        super().__init__()
        self.lingtu_nav_client_capabilities = _Function(
            lambda: 0x03 | navigation_abi.NATIVE_COMMAND_CAP_NAVIGATION_COMMAND_RECEIPT
        )
        for name in (
            "lingtu_nav_client_send_goal",
            "lingtu_nav_client_send_goal_with_id",
            "lingtu_nav_client_cancel",
            "lingtu_nav_client_cancel_with_id",
            "lingtu_nav_client_start_task_with_receipt_v1",
            "lingtu_nav_client_cancel_task_with_receipt_v1",
            "lingtu_nav_client_send_teleop",
            "lingtu_nav_client_send_teleop_with_id",
            "lingtu_nav_client_stop",
            "lingtu_nav_client_stop_with_id",
            "lingtu_nav_client_estop",
            "lingtu_nav_client_estop_with_id",
            "lingtu_nav_client_clear_estop",
            "lingtu_nav_client_clear_estop_with_id",
            "lingtu_nav_client_resume_autonomy",
            "lingtu_nav_client_resume_autonomy_with_id",
        ):
            setattr(self, name, _Function(lambda *_args: 0))


def test_inspection_commands_have_an_independent_python_interface(tmp_path):
    library = _InspectionLibrary()
    client = NativeInspectionCommandClient(
        tmp_path / "liblingtu_nav_client.so",
        domain_id=9,
        timeout_ms=250,
        library=library,
    )

    client.start("route-a", revision=7, request_id="inspection-start")
    client.pause("operator_hold", request_id="inspection-pause")
    client.resume("operator_resume", request_id="inspection-resume")
    client.cancel("operator_cancel", request_id="inspection-cancel")
    client.close()

    assert library.calls == [
        ("create", 9),
        ("start", 73, b"inspection-start", b"route-a", 7, 250),
        ("pause", 73, b"inspection-pause", b"operator_hold", 250),
        ("resume", 73, b"inspection-resume", b"operator_resume", 250),
        ("cancel", 73, b"inspection-cancel", b"operator_cancel", 250),
        ("destroy", 73),
    ]


def test_inspection_command_surfaces_native_rejection(tmp_path):
    library = _InspectionLibrary()
    library.error = b"inspection_route_not_found"
    library.lingtu_nav_client_start_inspection = _Function(lambda *_args: -1)
    client = NativeInspectionCommandClient(
        tmp_path / "liblingtu_nav_client.so",
        library=library,
    )

    try:
        client.start("missing-route")
    except InspectionCommandClientError as exc:
        assert str(exc) == "inspection_route_not_found"
    else:
        raise AssertionError("inspection rejection was not raised")
    finally:
        client.close()


@pytest.mark.parametrize("revision", [0, (1 << 64) - 1])
def test_inspection_start_accepts_uint64_revision_bounds(tmp_path, revision):
    library = _InspectionLibrary()
    client = NativeInspectionCommandClient(
        tmp_path / "liblingtu_nav_client.so",
        timeout_ms=250,
        library=library,
    )

    try:
        client.start("route-a", revision=revision, request_id="inspection-start")
    finally:
        client.close()

    assert library.calls[1] == (
        "start",
        73,
        b"inspection-start",
        b"route-a",
        revision,
        250,
    )


@pytest.mark.parametrize("revision", [-1, 1 << 64])
def test_inspection_start_rejects_revision_outside_uint64(tmp_path, revision):
    library = _InspectionLibrary()
    client = NativeInspectionCommandClient(
        tmp_path / "liblingtu_nav_client.so",
        library=library,
    )

    try:
        with pytest.raises(ValueError, match="route revision must be between 0 and UINT64_MAX"):
            client.start("route-a", revision=revision)
    finally:
        client.close()

    assert [call for call in library.calls if call[0] == "start"] == []


def test_inspection_client_requires_capability_and_releases_handle(tmp_path):
    library = _InspectionLibrary()
    library.lingtu_nav_client_capabilities = _Function(lambda: 0x01)

    try:
        NativeInspectionCommandClient(
            tmp_path / "liblingtu_nav_client.so",
            domain_id=9,
            library=library,
        )
    except InspectionCommandClientError as exc:
        assert "inspection commands capability" in str(exc)
    else:
        raise AssertionError("missing inspection capability was accepted")

    assert library.calls == [("create", 9), ("destroy", 73)]


def test_navigation_and_inspection_views_share_one_process_handle(tmp_path, monkeypatch):
    library = _CombinedLibrary()
    library_path = tmp_path / "liblingtu_nav_client.so"
    library_path.touch()
    monkeypatch.setenv("LINGTU_NAV_CLIENT_LIB", str(library_path))
    monkeypatch.setattr(navigation_abi.ctypes, "CDLL", lambda _path: library)
    navigation_abi._close_sessions()

    try:
        navigation = get_native_navigation_client(required=True)
        inspection = get_native_inspection_command_client(required=True)

        assert navigation is not None
        assert inspection is not None
        assert navigation._session is inspection._session
        assert library.calls == [("create", 0)]
    finally:
        navigation_abi._close_sessions()

    assert library.calls == [("create", 0), ("destroy", 73)]

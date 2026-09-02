from __future__ import annotations

import pytest

from nav.adapters.native import abi as navigation_abi
from nav.adapters.native import inspection_commands
from nav.adapters.native.commands import get_native_navigation_client
from nav.adapters.native.inspection_commands import (
    InspectionTaskClientError,
    NativeInspectionTaskClient,
    get_native_inspection_task_client,
)


class _Function:
    def __init__(self, callback):
        self._callback = callback
        self.argtypes = None
        self.restype = None

    def __call__(self, *args):
        return self._callback(*args)


class _CoreInspectionLibrary:
    def __init__(self) -> None:
        self.calls = []
        self.error = b""
        self.lingtu_nav_client_abi_version = _Function(
            lambda: navigation_abi.NATIVE_COMMAND_ABI_VERSION
        )
        self.lingtu_nav_client_capabilities = _Function(
            lambda: navigation_abi.NATIVE_COMMAND_CAP_INSPECTION
        )
        self.lingtu_nav_client_create = _Function(self._create)
        self.lingtu_nav_client_destroy = _Function(self._destroy)
        self.lingtu_nav_client_last_error = _Function(lambda _handle: self.error)

    def _create(self, domain_id):
        self.calls.append(("create", domain_id))
        return 73

    def _destroy(self, handle):
        self.calls.append(("destroy", handle))


class _InspectionTaskLibrary(_CoreInspectionLibrary):
    def __init__(self) -> None:
        super().__init__()
        self.lingtu_nav_client_start_inspection_task = _Function(self._task_start)
        self.lingtu_nav_client_pause_inspection_task = _Function(self._task_pause)
        self.lingtu_nav_client_resume_inspection_task = _Function(self._task_resume)
        self.lingtu_nav_client_cancel_inspection_task = _Function(self._task_cancel)

    def _task_start(self, handle, task_id, request_id, route_id, revision, timeout_ms):
        self.calls.append(
            ("task_start", handle, task_id, request_id, route_id, revision, timeout_ms)
        )
        return 0

    def _task_pause(self, handle, task_id, request_id, reason, timeout_ms):
        self.calls.append(("task_pause", handle, task_id, request_id, reason, timeout_ms))
        return 0

    def _task_resume(self, handle, task_id, request_id, reason, timeout_ms):
        self.calls.append(("task_resume", handle, task_id, request_id, reason, timeout_ms))
        return 0

    def _task_cancel(self, handle, task_id, request_id, reason, timeout_ms):
        self.calls.append(("task_cancel", handle, task_id, request_id, reason, timeout_ms))
        return 0


class _CombinedLibrary(_InspectionTaskLibrary):
    def __init__(self) -> None:
        super().__init__()
        self.lingtu_nav_client_capabilities = _Function(
            lambda: (
                navigation_abi.NATIVE_COMMAND_CAP_NAVIGATION
                | navigation_abi.NATIVE_COMMAND_CAP_INSPECTION
                | navigation_abi.NATIVE_COMMAND_CAP_NAVIGATION_COMMAND_RECEIPT
            )
        )
        for name in (
            "lingtu_nav_client_stop",
            "lingtu_nav_client_stop_with_id",
            "lingtu_nav_client_estop",
            "lingtu_nav_client_estop_with_id",
            "lingtu_nav_client_clear_estop",
            "lingtu_nav_client_clear_estop_with_id",
            "lingtu_nav_client_resume_autonomy",
            "lingtu_nav_client_resume_autonomy_with_id",
            "lingtu_nav_client_start_task_with_receipt_v1",
            "lingtu_nav_client_cancel_task_with_receipt_v1",
            "lingtu_nav_client_pause_task_with_receipt_v1",
            "lingtu_nav_client_resume_task_with_receipt_v1",
        ):
            setattr(self, name, _Function(lambda *_args: 0))


def test_task_addressed_inspection_client_uses_only_task_commands(tmp_path) -> None:
    library = _InspectionTaskLibrary()
    client = NativeInspectionTaskClient(
        tmp_path / "liblingtu_nav_client.so",
        domain_id=9,
        timeout_ms=250,
        library=library,
    )

    client.start(
        " inspection-task-42 ",
        " route-a ",
        revision=7,
        request_id="inspection-start-42",
    )
    client.pause("inspection-task-42", "operator_hold", request_id="inspection-pause-42")
    client.resume(
        "inspection-task-42",
        "operator_resume",
        request_id="inspection-resume-42",
    )
    client.cancel(
        "inspection-task-42",
        "operator_cancel",
        request_id="inspection-cancel-42",
    )
    client.close()

    assert library.calls == [
        ("create", 9),
        (
            "task_start",
            73,
            b"inspection-task-42",
            b"inspection-start-42",
            b"route-a",
            7,
            250,
        ),
        (
            "task_pause",
            73,
            b"inspection-task-42",
            b"inspection-pause-42",
            b"operator_hold",
            250,
        ),
        (
            "task_resume",
            73,
            b"inspection-task-42",
            b"inspection-resume-42",
            b"operator_resume",
            250,
        ),
        (
            "task_cancel",
            73,
            b"inspection-task-42",
            b"inspection-cancel-42",
            b"operator_cancel",
            250,
        ),
        ("destroy", 73),
    ]


def test_taskless_python_inspection_client_is_retired() -> None:
    assert not hasattr(inspection_commands, "NativeInspectionCommandClient")
    assert not hasattr(inspection_commands, "get_native_inspection_command_client")
    assert not hasattr(inspection_commands, "InspectionCommandClientError")


def test_task_client_rejects_missing_identity_and_task_abi(tmp_path) -> None:
    library = _InspectionTaskLibrary()
    client = NativeInspectionTaskClient(tmp_path / "liblingtu_nav_client.so", library=library)
    try:
        with pytest.raises(ValueError, match="inspection task id is required"):
            client.start("", "route-a")
        with pytest.raises(ValueError, match="inspection route id is required"):
            client.start("task-42", "  ")
    finally:
        client.close()
    assert [call for call in library.calls if call[0].startswith("task_")] == []

    with pytest.raises(InspectionTaskClientError, match="inspection task command ABI"):
        NativeInspectionTaskClient(
            tmp_path / "liblingtu_nav_client.so",
            library=_CoreInspectionLibrary(),
        )


def test_task_inspection_command_surfaces_native_rejection(tmp_path) -> None:
    library = _InspectionTaskLibrary()
    library.error = b"inspection_route_not_found"
    library.lingtu_nav_client_start_inspection_task = _Function(lambda *_args: -1)
    client = NativeInspectionTaskClient(
        tmp_path / "liblingtu_nav_client.so",
        library=library,
    )

    try:
        with pytest.raises(InspectionTaskClientError, match="inspection_route_not_found"):
            client.start("task-42", "missing-route")
    finally:
        client.close()


@pytest.mark.parametrize("revision", [0, (1 << 64) - 1])
def test_inspection_task_start_accepts_uint64_revision_bounds(tmp_path, revision) -> None:
    library = _InspectionTaskLibrary()
    client = NativeInspectionTaskClient(
        tmp_path / "liblingtu_nav_client.so",
        timeout_ms=250,
        library=library,
    )

    try:
        client.start(
            "task-42",
            "route-a",
            revision=revision,
            request_id="inspection-start",
        )
    finally:
        client.close()

    assert library.calls[1] == (
        "task_start",
        73,
        b"task-42",
        b"inspection-start",
        b"route-a",
        revision,
        250,
    )


@pytest.mark.parametrize("revision", [-1, 1 << 64])
def test_inspection_task_start_rejects_revision_outside_uint64(tmp_path, revision) -> None:
    library = _InspectionTaskLibrary()
    client = NativeInspectionTaskClient(
        tmp_path / "liblingtu_nav_client.so",
        library=library,
    )

    try:
        with pytest.raises(ValueError, match="route revision must be between 0 and UINT64_MAX"):
            client.start("task-42", "route-a", revision=revision)
    finally:
        client.close()

    assert [call for call in library.calls if call[0] == "task_start"] == []


def test_inspection_task_client_requires_capability_and_releases_handle(tmp_path) -> None:
    library = _InspectionTaskLibrary()
    library.lingtu_nav_client_capabilities = _Function(lambda: 0)

    with pytest.raises(InspectionTaskClientError, match="inspection task commands capability"):
        NativeInspectionTaskClient(
            tmp_path / "liblingtu_nav_client.so",
            domain_id=9,
            library=library,
        )

    assert library.calls == [("create", 9), ("destroy", 73)]


def test_navigation_and_inspection_task_views_share_one_process_handle(tmp_path, monkeypatch) -> None:
    library = _CombinedLibrary()
    library_path = tmp_path / "liblingtu_nav_client.so"
    library_path.touch()
    monkeypatch.setenv("LINGTU_NAV_CLIENT_LIB", str(library_path))
    monkeypatch.setattr(navigation_abi.ctypes, "CDLL", lambda _path: library)
    navigation_abi._close_sessions()

    try:
        navigation = get_native_navigation_client(required=True)
        inspection = get_native_inspection_task_client(required=True)

        assert navigation is not None
        assert inspection is not None
        assert navigation._session is inspection._session
        assert library.calls == [("create", 0)]
    finally:
        navigation_abi._close_sessions()

    assert library.calls == [("create", 0), ("destroy", 73)]

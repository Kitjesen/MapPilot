from __future__ import annotations

import threading

from runtime.adapters.native.navigation import (
    NativeNavigationClient,
    NavigationClientError,
)


class _Function:
    def __init__(self, callback):
        self._callback = callback
        self.argtypes = None
        self.restype = None

    def __call__(self, *args):
        return self._callback(*args)


class _Library:
    def __init__(self) -> None:
        self.calls = []
        self.error = b""
        self.lingtu_nav_client_abi_version = _Function(lambda: 1)
        self.lingtu_nav_client_capabilities = _Function(lambda: 0x03)
        self.lingtu_nav_client_create = _Function(self._create)
        self.lingtu_nav_client_destroy = _Function(self._destroy)
        self.lingtu_nav_client_send_goal = _Function(self._goal)
        self.lingtu_nav_client_send_goal_with_id = _Function(self._goal_with_id)
        self.lingtu_nav_client_cancel = _Function(self._cancel)
        self.lingtu_nav_client_cancel_with_id = _Function(self._cancel_with_id)
        self.lingtu_nav_client_send_teleop = _Function(self._teleop)
        self.lingtu_nav_client_send_teleop_with_id = _Function(self._teleop_with_id)
        self.lingtu_nav_client_stop = _Function(self._stop)
        self.lingtu_nav_client_stop_with_id = _Function(self._stop_with_id)
        self.lingtu_nav_client_estop = _Function(self._estop)
        self.lingtu_nav_client_estop_with_id = _Function(self._estop_with_id)
        self.lingtu_nav_client_clear_estop = _Function(self._clear_estop)
        self.lingtu_nav_client_clear_estop_with_id = _Function(self._clear_estop_with_id)
        self.lingtu_nav_client_resume_autonomy = _Function(self._resume_autonomy)
        self.lingtu_nav_client_resume_autonomy_with_id = _Function(self._resume_autonomy_with_id)
        self.lingtu_nav_client_last_error = _Function(lambda _handle: self.error)

    def _create(self, domain_id):
        self.calls.append(("create", domain_id))
        return 41

    def _destroy(self, handle):
        self.calls.append(("destroy", handle))

    def _goal(self, handle, x, y, z, yaw, timeout_ms):
        self.calls.append(("goal", handle, x, y, z, yaw, timeout_ms))
        return 0

    def _goal_with_id(self, handle, request_id, x, y, z, yaw, timeout_ms):
        self.calls.append(("goal", handle, request_id, x, y, z, yaw, timeout_ms))
        return 0

    def _cancel(self, handle, reason, timeout_ms):
        self.calls.append(("cancel", handle, reason, timeout_ms))
        return 0

    def _cancel_with_id(self, handle, request_id, reason, timeout_ms):
        self.calls.append(("cancel", handle, request_id, reason, timeout_ms))
        return 0

    def _teleop(self, handle, vx, vy, wz, timeout_ms):
        self.calls.append(("teleop", handle, vx, vy, wz, timeout_ms))
        return 0

    def _teleop_with_id(self, handle, request_id, vx, vy, wz, timeout_ms):
        self.calls.append(("teleop", handle, request_id, vx, vy, wz, timeout_ms))
        return 0

    def _stop(self, handle, reason, timeout_ms):
        self.calls.append(("stop", handle, reason, timeout_ms))
        return 0

    def _stop_with_id(self, handle, request_id, reason, timeout_ms):
        self.calls.append(("stop", handle, request_id, reason, timeout_ms))
        return 0

    def _estop(self, handle, reason, timeout_ms):
        self.calls.append(("estop", handle, reason, timeout_ms))
        return 0

    def _estop_with_id(self, handle, request_id, reason, timeout_ms):
        self.calls.append(("estop", handle, request_id, reason, timeout_ms))
        return 0

    def _clear_estop(self, handle, reason, timeout_ms):
        self.calls.append(("clear_estop", handle, reason, timeout_ms))
        return 0

    def _clear_estop_with_id(self, handle, request_id, reason, timeout_ms):
        self.calls.append(("clear_estop", handle, request_id, reason, timeout_ms))
        return 0

    def _resume_autonomy(self, handle, reason, timeout_ms):
        self.calls.append(("resume_autonomy", handle, reason, timeout_ms))
        return 0

    def _resume_autonomy_with_id(self, handle, request_id, reason, timeout_ms):
        self.calls.append(("resume_autonomy", handle, request_id, reason, timeout_ms))
        return 0


def test_native_client_reuses_one_cpp_handle_for_commands(tmp_path):
    library = _Library()
    client = NativeNavigationClient(
        tmp_path / "liblingtu_nav_client.so",
        domain_id=7,
        timeout_ms=250,
        library=library,
    )

    client.send_goal(1.0, 2.0, 0.3, 0.4, request_id="goal-001")
    client.cancel("operator_cancel", request_id="cancel-001")
    client.send_teleop(0.2, -0.1, 0.5, request_id="teleop-001")
    client.stop("operator_stop", request_id="stop-001")
    client.estop("operator_estop", request_id="estop-001")
    client.clear_estop("operator_reset", request_id="clear-estop-001")
    client.resume_autonomy("operator_resume", request_id="resume-001")
    client.close()

    assert library.calls == [
        ("create", 7),
        ("goal", 41, b"goal-001", 1.0, 2.0, 0.3, 0.4, 250),
        ("cancel", 41, b"cancel-001", b"operator_cancel", 250),
        ("teleop", 41, b"teleop-001", 0.2, -0.1, 0.5, 250),
        ("stop", 41, b"stop-001", b"operator_stop", 250),
        ("estop", 41, b"estop-001", b"operator_estop", 250),
        ("clear_estop", 41, b"clear-estop-001", b"operator_reset", 250),
        ("resume_autonomy", 41, b"resume-001", b"operator_resume", 250),
        ("destroy", 41),
    ]


def test_native_client_surfaces_cpp_delivery_error(tmp_path):
    library = _Library()
    library.error = b"no matched DDS reader for goal_pose"
    library.lingtu_nav_client_send_goal_with_id = _Function(lambda *_args: -1)
    client = NativeNavigationClient(
        tmp_path / "liblingtu_nav_client.so",
        library=library,
    )

    try:
        client.send_goal(1.0, 2.0, 0.0, 0.0)
    except NavigationClientError as exc:
        assert str(exc) == "no matched DDS reader for goal_pose"
    else:
        raise AssertionError("native client error was not raised")
    finally:
        client.close()


def test_native_navigation_client_rejects_incompatible_abi(tmp_path):
    library = _Library()
    library.lingtu_nav_client_abi_version = _Function(lambda: 99)

    try:
        NativeNavigationClient(
            tmp_path / "liblingtu_nav_client.so",
            library=library,
        )
    except NavigationClientError as exc:
        assert "ABI version" in str(exc)
    else:
        raise AssertionError("incompatible native client ABI was accepted")


def test_native_navigation_client_requires_navigation_capability(tmp_path):
    library = _Library()
    library.lingtu_nav_client_capabilities = _Function(lambda: 0x02)

    try:
        NativeNavigationClient(
            tmp_path / "liblingtu_nav_client.so",
            library=library,
        )
    except NavigationClientError as exc:
        assert "navigation commands capability" in str(exc)
    else:
        raise AssertionError("missing navigation capability was accepted")

    assert library.calls == [("create", 0), ("destroy", 41)]


def test_python_session_does_not_serialize_estop_behind_goal(tmp_path):
    library = _Library()
    goal_started = threading.Event()
    release_goal = threading.Event()
    estop_seen = threading.Event()

    def blocked_goal(*_args):
        goal_started.set()
        release_goal.wait(timeout=1.0)
        return 0

    def immediate_estop(*_args):
        estop_seen.set()
        return 0

    library.lingtu_nav_client_send_goal_with_id = _Function(blocked_goal)
    library.lingtu_nav_client_estop_with_id = _Function(immediate_estop)
    client = NativeNavigationClient(
        tmp_path / "liblingtu_nav_client.so",
        library=library,
    )
    goal_thread = threading.Thread(
        target=lambda: client.send_goal(1.0, 2.0, 0.0, 0.0),
    )
    estop_thread = threading.Thread(target=lambda: client.estop("operator_estop"))

    try:
        goal_thread.start()
        assert goal_started.wait(timeout=0.5)
        estop_thread.start()
        assert estop_seen.wait(timeout=0.25)
    finally:
        release_goal.set()
        goal_thread.join(timeout=1.0)
        estop_thread.join(timeout=1.0)
        client.close()

    assert not goal_thread.is_alive()
    assert not estop_thread.is_alive()

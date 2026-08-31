from __future__ import annotations

import threading

import pytest

from sim.runtime.windows_timing import WindowsTimingError, deadline_waiter


class _TimerApi:
    def __init__(
        self,
        *,
        handle: int = 17,
        arm_result: bool = True,
        wait_result: int = 0,
        close_result: bool = True,
    ) -> None:
        self.calls: list[tuple[str, int]] = []
        self.handle = handle
        self.arm_result = arm_result
        self.wait_result = wait_result
        self.close_result = close_result

    def create(self) -> int:
        self.calls.append(("create", 0))
        return self.handle

    def arm(self, handle: int, due_100ns: int) -> bool:
        assert handle == 17
        self.calls.append(("arm", due_100ns))
        return self.arm_result

    def wait(self, handle: int) -> int:
        self.calls.append(("wait", handle))
        return self.wait_result

    def close(self, handle: int) -> bool:
        self.calls.append(("close", handle))
        return self.close_result


def test_windows_deadline_waiter_reuses_and_closes_one_timer() -> None:
    stop_event = threading.Event()
    api = _TimerApi()

    with deadline_waiter(stop_event, platform="win32", api=api) as wait:
        assert wait(0.005) is False
        assert wait(0.010) is False

    assert api.calls == [
        ("create", 0),
        ("arm", -50_000),
        ("wait", 17),
        ("arm", -100_000),
        ("wait", 17),
        ("close", 17),
    ]


def test_windows_deadline_waiter_closes_after_body_failure() -> None:
    api = _TimerApi()

    with pytest.raises(LookupError, match="body failed"):
        with deadline_waiter(threading.Event(), platform="win32", api=api):
            raise LookupError("body failed")

    assert api.calls[-1] == ("close", 17)


def test_windows_deadline_waiter_preserves_body_failure_when_close_also_fails() -> None:
    api = _TimerApi(close_result=False)

    with pytest.raises(LookupError, match="body failed") as raised:
        with deadline_waiter(threading.Event(), platform="win32", api=api):
            raise LookupError("body failed")

    assert isinstance(raised.value.__context__, WindowsTimingError)
    assert "CloseHandle failed" in str(raised.value.__context__)
    assert api.calls[-1] == ("close", 17)


def test_non_windows_deadline_waiter_uses_the_stop_event() -> None:
    stop_event = threading.Event()
    stop_event.set()
    api = _TimerApi()

    with deadline_waiter(stop_event, platform="linux", api=api) as wait:
        assert wait(0.005) is True

    assert api.calls == []


@pytest.mark.parametrize(
    ("api", "operation"),
    (
        (_TimerApi(handle=0), "CreateWaitableTimerExW"),
        (_TimerApi(arm_result=False), "SetWaitableTimer"),
        (_TimerApi(wait_result=0xFFFFFFFF), "WaitForSingleObject"),
        (_TimerApi(close_result=False), "CloseHandle"),
    ),
)
def test_windows_deadline_waiter_fails_closed_on_native_error(
    api: _TimerApi,
    operation: str,
) -> None:
    with pytest.raises(
        WindowsTimingError,
        match=rf"{operation} failed \(GetLastError=\d+\)",
    ):
        with deadline_waiter(threading.Event(), platform="win32", api=api) as wait:
            wait(0.005)

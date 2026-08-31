"""High-resolution Windows waits for short simulation deadlines."""

from __future__ import annotations

import ctypes
import math
import sys
from collections.abc import Callable, Iterator
from contextlib import contextmanager
from typing import Protocol


class WindowsTimingError(RuntimeError):
    """Raised when a Windows waitable-timer operation fails."""


class _StopEvent(Protocol):
    def is_set(self) -> bool: ...

    def wait(self, timeout: float | None = None) -> bool: ...


class _WaitableTimerApi(Protocol):
    def create(self) -> int: ...

    def arm(self, handle: int, due_100ns: int) -> bool: ...

    def wait(self, handle: int) -> int: ...

    def close(self, handle: int) -> bool: ...


class _Kernel32TimerApi:
    def __init__(self) -> None:
        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        kernel32.CreateWaitableTimerExW.argtypes = [
            ctypes.c_void_p,
            ctypes.c_wchar_p,
            ctypes.c_ulong,
            ctypes.c_ulong,
        ]
        kernel32.CreateWaitableTimerExW.restype = ctypes.c_void_p
        kernel32.SetWaitableTimer.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_longlong),
            ctypes.c_long,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_int,
        ]
        kernel32.SetWaitableTimer.restype = ctypes.c_int
        kernel32.WaitForSingleObject.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        kernel32.WaitForSingleObject.restype = ctypes.c_ulong
        kernel32.CloseHandle.argtypes = [ctypes.c_void_p]
        kernel32.CloseHandle.restype = ctypes.c_int
        self._kernel32 = kernel32

    def create(self) -> int:
        handle = self._kernel32.CreateWaitableTimerExW(
            None,
            None,
            0x00000002,  # CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
            0x001F0003,  # TIMER_ALL_ACCESS
        )
        return int(handle or 0)

    def arm(self, handle: int, due_100ns: int) -> bool:
        due = ctypes.c_longlong(due_100ns)
        return bool(self._kernel32.SetWaitableTimer(handle, ctypes.byref(due), 0, None, None, False))

    def wait(self, handle: int) -> int:
        return int(self._kernel32.WaitForSingleObject(handle, 0xFFFFFFFF))

    def close(self, handle: int) -> bool:
        return bool(self._kernel32.CloseHandle(handle))


def _native_error(operation: str) -> WindowsTimingError:
    get_last_error = getattr(ctypes, "get_last_error", lambda: 0)
    return WindowsTimingError(f"{operation} failed (GetLastError={get_last_error()})")


@contextmanager
def deadline_waiter(
    stop_event: _StopEvent,
    *,
    platform: str | None = None,
    api: _WaitableTimerApi | None = None,
) -> Iterator[Callable[[float], bool]]:
    """Yield a stop-aware wait function and release its native handle."""

    if (platform or sys.platform) != "win32":
        yield lambda timeout_s: stop_event.wait(timeout_s)
        return
    timer = api or _Kernel32TimerApi()
    handle = timer.create()
    if not handle:
        raise _native_error("CreateWaitableTimerExW")

    def wait(timeout_s: float) -> bool:
        if stop_event.is_set():
            return True
        if not math.isfinite(timeout_s) or timeout_s < 0.0:
            raise ValueError("deadline wait timeout must be finite and non-negative")
        if timeout_s == 0.0:
            return stop_event.is_set()
        due_100ns = -max(1, math.ceil(timeout_s * 10_000_000.0))
        if not timer.arm(handle, due_100ns):
            raise _native_error("SetWaitableTimer")
        if timer.wait(handle) != 0:
            raise _native_error("WaitForSingleObject")
        return stop_event.is_set()

    body_error: BaseException | None = None
    try:
        yield wait
    except BaseException as error:
        body_error = error
        raise
    finally:
        if not timer.close(handle):
            close_error = _native_error("CloseHandle")
            if body_error is None:
                raise close_error
            close_error.__context__ = body_error.__context__
            body_error.__context__ = close_error

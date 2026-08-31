"""Own one subprocess tree for a simulation runtime component."""

from __future__ import annotations

import os
import signal
import subprocess
import time
from dataclasses import dataclass
from typing import Any, Literal

from sim.runtime.windows_cpu_isolation import (
    WindowsCpuIsolationError,
    validate_windows_affinity_mask,
)


class ProcessOwnershipError(RuntimeError):
    """Raised when a child cannot be placed under tree ownership."""


@dataclass(frozen=True, slots=True)
class ProcessShutdownSnapshot:
    """Immutable facts read from one owned direct child after tree closure."""

    pid: int
    exit_code: int | None
    direct_child_running_after_close: bool
    process_owner_closed: bool
    termination_mode: Literal["natural", "owned_terminate"]


if os.name == "nt":
    import ctypes
    from ctypes import wintypes

    _CREATE_SUSPENDED = 0x00000004
    _NORMAL_PRIORITY_CLASS = 0x00000020
    _JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE = 0x00002000
    _JOB_OBJECT_LIMIT_AFFINITY = 0x00000010
    _JOB_OBJECT_LIMIT_PRIORITY_CLASS = 0x00000020
    _JOB_OBJECT_EXTENDED_LIMIT_INFORMATION_CLASS = 9
    _PROCESS_TERMINATE = 0x0001
    _PROCESS_SET_QUOTA = 0x0100
    _PROCESS_QUERY_LIMITED_INFORMATION = 0x1000
    _THREAD_SUSPEND_RESUME = 0x0002
    _TH32CS_SNAPTHREAD = 0x00000004
    _ERROR_NO_MORE_FILES = 18
    _INVALID_HANDLE_VALUE = ctypes.c_void_p(-1).value
    _PRIORITY_CLASS_FLAGS = (
        0x00000020  # NORMAL_PRIORITY_CLASS
        | 0x00000040  # IDLE_PRIORITY_CLASS
        | 0x00000080  # HIGH_PRIORITY_CLASS
        | 0x00000100  # REALTIME_PRIORITY_CLASS
        | 0x00004000  # BELOW_NORMAL_PRIORITY_CLASS
        | 0x00008000  # ABOVE_NORMAL_PRIORITY_CLASS
    )

    class _IoCounters(ctypes.Structure):
        _fields_ = [
            ("ReadOperationCount", ctypes.c_uint64),
            ("WriteOperationCount", ctypes.c_uint64),
            ("OtherOperationCount", ctypes.c_uint64),
            ("ReadTransferCount", ctypes.c_uint64),
            ("WriteTransferCount", ctypes.c_uint64),
            ("OtherTransferCount", ctypes.c_uint64),
        ]

    class _BasicLimitInformation(ctypes.Structure):
        _fields_ = [
            ("PerProcessUserTimeLimit", ctypes.c_int64),
            ("PerJobUserTimeLimit", ctypes.c_int64),
            ("LimitFlags", wintypes.DWORD),
            ("MinimumWorkingSetSize", ctypes.c_size_t),
            ("MaximumWorkingSetSize", ctypes.c_size_t),
            ("ActiveProcessLimit", wintypes.DWORD),
            ("Affinity", ctypes.c_size_t),
            ("PriorityClass", wintypes.DWORD),
            ("SchedulingClass", wintypes.DWORD),
        ]

    class _ExtendedLimitInformation(ctypes.Structure):
        _fields_ = [
            ("BasicLimitInformation", _BasicLimitInformation),
            ("IoInfo", _IoCounters),
            ("ProcessMemoryLimit", ctypes.c_size_t),
            ("JobMemoryLimit", ctypes.c_size_t),
            ("PeakProcessMemoryUsed", ctypes.c_size_t),
            ("PeakJobMemoryUsed", ctypes.c_size_t),
        ]

    class _ThreadEntry32(ctypes.Structure):
        _fields_ = [
            ("dwSize", wintypes.DWORD),
            ("cntUsage", wintypes.DWORD),
            ("th32ThreadID", wintypes.DWORD),
            ("th32OwnerProcessID", wintypes.DWORD),
            ("tpBasePri", wintypes.LONG),
            ("tpDeltaPri", wintypes.LONG),
            ("dwFlags", wintypes.DWORD),
        ]

    _KERNEL32 = ctypes.WinDLL("kernel32", use_last_error=True)
    _KERNEL32.CreateJobObjectW.argtypes = [ctypes.c_void_p, wintypes.LPCWSTR]
    _KERNEL32.CreateJobObjectW.restype = wintypes.HANDLE
    _KERNEL32.SetInformationJobObject.argtypes = [
        wintypes.HANDLE,
        ctypes.c_int,
        ctypes.c_void_p,
        wintypes.DWORD,
    ]
    _KERNEL32.SetInformationJobObject.restype = wintypes.BOOL
    _KERNEL32.QueryInformationJobObject.argtypes = [
        wintypes.HANDLE,
        ctypes.c_int,
        ctypes.c_void_p,
        wintypes.DWORD,
        ctypes.POINTER(wintypes.DWORD),
    ]
    _KERNEL32.QueryInformationJobObject.restype = wintypes.BOOL
    _KERNEL32.OpenProcess.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
    _KERNEL32.OpenProcess.restype = wintypes.HANDLE
    _KERNEL32.AssignProcessToJobObject.argtypes = [wintypes.HANDLE, wintypes.HANDLE]
    _KERNEL32.AssignProcessToJobObject.restype = wintypes.BOOL
    _KERNEL32.IsProcessInJob.argtypes = [
        wintypes.HANDLE,
        wintypes.HANDLE,
        ctypes.POINTER(wintypes.BOOL),
    ]
    _KERNEL32.IsProcessInJob.restype = wintypes.BOOL
    _KERNEL32.GetProcessAffinityMask.argtypes = [
        wintypes.HANDLE,
        ctypes.POINTER(ctypes.c_size_t),
        ctypes.POINTER(ctypes.c_size_t),
    ]
    _KERNEL32.GetProcessAffinityMask.restype = wintypes.BOOL
    _KERNEL32.GetPriorityClass.argtypes = [wintypes.HANDLE]
    _KERNEL32.GetPriorityClass.restype = wintypes.DWORD
    _KERNEL32.CreateToolhelp32Snapshot.argtypes = [wintypes.DWORD, wintypes.DWORD]
    _KERNEL32.CreateToolhelp32Snapshot.restype = wintypes.HANDLE
    _KERNEL32.Thread32First.argtypes = [
        wintypes.HANDLE,
        ctypes.POINTER(_ThreadEntry32),
    ]
    _KERNEL32.Thread32First.restype = wintypes.BOOL
    _KERNEL32.Thread32Next.argtypes = [
        wintypes.HANDLE,
        ctypes.POINTER(_ThreadEntry32),
    ]
    _KERNEL32.Thread32Next.restype = wintypes.BOOL
    _KERNEL32.OpenThread.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
    _KERNEL32.OpenThread.restype = wintypes.HANDLE
    _KERNEL32.ResumeThread.argtypes = [wintypes.HANDLE]
    _KERNEL32.ResumeThread.restype = wintypes.DWORD
    _KERNEL32.TerminateJobObject.argtypes = [wintypes.HANDLE, wintypes.UINT]
    _KERNEL32.TerminateJobObject.restype = wintypes.BOOL
    _KERNEL32.CloseHandle.argtypes = [wintypes.HANDLE]
    _KERNEL32.CloseHandle.restype = wintypes.BOOL

    class _WindowsJob:
        def __init__(self, affinity_mask: int | None = None) -> None:
            handle = _KERNEL32.CreateJobObjectW(None, None)
            if not handle:
                raise ctypes.WinError(ctypes.get_last_error())
            self._handle: int | None = handle
            self._affinity_mask = affinity_mask
            limits = _ExtendedLimitInformation()
            limits.BasicLimitInformation.LimitFlags = _JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE
            if affinity_mask is not None:
                limits.BasicLimitInformation.LimitFlags |= (
                    _JOB_OBJECT_LIMIT_AFFINITY | _JOB_OBJECT_LIMIT_PRIORITY_CLASS
                )
                limits.BasicLimitInformation.Affinity = affinity_mask
                limits.BasicLimitInformation.PriorityClass = _NORMAL_PRIORITY_CLASS
            try:
                if not _KERNEL32.SetInformationJobObject(
                    handle,
                    _JOB_OBJECT_EXTENDED_LIMIT_INFORMATION_CLASS,
                    ctypes.byref(limits),
                    ctypes.sizeof(limits),
                ):
                    error = ctypes.WinError(ctypes.get_last_error())
                    if affinity_mask is not None:
                        raise ProcessOwnershipError(
                            "current launcher cannot configure the Windows Job "
                            "affinity and Normal-priority limits"
                        ) from error
                    raise error
                if affinity_mask is not None:
                    self._verify_limits()
            except BaseException:
                self.close()
                raise

        def assign(self, pid: int) -> None:
            if self._handle is None:
                raise ProcessOwnershipError("Windows Job Object is closed")
            access = _PROCESS_TERMINATE | _PROCESS_SET_QUOTA
            if self._affinity_mask is not None:
                access |= _PROCESS_QUERY_LIMITED_INFORMATION
            process = _KERNEL32.OpenProcess(
                access,
                False,
                pid,
            )
            if not process:
                raise ctypes.WinError(ctypes.get_last_error())
            try:
                if not _KERNEL32.AssignProcessToJobObject(self._handle, process):
                    raise ctypes.WinError(ctypes.get_last_error())
                if self._affinity_mask is not None:
                    self._verify_process(process)
            finally:
                _KERNEL32.CloseHandle(process)

        def _verify_limits(self) -> None:
            handle = self._handle
            if handle is None:
                raise ProcessOwnershipError("Windows Job Object is closed")
            limits = _ExtendedLimitInformation()
            returned = wintypes.DWORD()
            if not _KERNEL32.QueryInformationJobObject(
                handle,
                _JOB_OBJECT_EXTENDED_LIMIT_INFORMATION_CLASS,
                ctypes.byref(limits),
                ctypes.sizeof(limits),
                ctypes.byref(returned),
            ):
                raise ctypes.WinError(ctypes.get_last_error())
            basic = limits.BasicLimitInformation
            required = _JOB_OBJECT_LIMIT_AFFINITY | _JOB_OBJECT_LIMIT_PRIORITY_CLASS
            if basic.LimitFlags & required != required:
                raise ProcessOwnershipError(
                    "Windows Job did not retain affinity and priority limits"
                )
            if int(basic.Affinity) != self._affinity_mask:
                raise ProcessOwnershipError("Windows Job affinity verification failed")
            if int(basic.PriorityClass) != _NORMAL_PRIORITY_CLASS:
                raise ProcessOwnershipError("Windows Job priority verification failed")

        def _verify_process(self, process: int) -> None:
            handle = self._handle
            if handle is None:
                raise ProcessOwnershipError("Windows Job Object is closed")
            in_job = wintypes.BOOL()
            if not _KERNEL32.IsProcessInJob(process, handle, ctypes.byref(in_job)):
                raise ctypes.WinError(ctypes.get_last_error())
            if not in_job.value:
                raise ProcessOwnershipError("child process is not in the configured Job")
            process_mask = ctypes.c_size_t()
            system_mask = ctypes.c_size_t()
            if not _KERNEL32.GetProcessAffinityMask(
                process,
                ctypes.byref(process_mask),
                ctypes.byref(system_mask),
            ):
                raise ctypes.WinError(ctypes.get_last_error())
            if int(process_mask.value) != self._affinity_mask:
                raise ProcessOwnershipError("child process affinity verification failed")
            priority = _KERNEL32.GetPriorityClass(process)
            if not priority:
                raise ctypes.WinError(ctypes.get_last_error())
            if int(priority) != _NORMAL_PRIORITY_CLASS:
                raise ProcessOwnershipError("child process priority verification failed")

        def terminate(self) -> None:
            if self._handle is not None and not _KERNEL32.TerminateJobObject(self._handle, 1):
                raise ctypes.WinError(ctypes.get_last_error())

        def close(self) -> None:
            handle = self._handle
            self._handle = None
            if handle is not None:
                _KERNEL32.CloseHandle(handle)


    def _resume_suspended_process(pid: int) -> None:
        """Resume the only primary thread of a newly suspended process."""

        snapshot = _KERNEL32.CreateToolhelp32Snapshot(_TH32CS_SNAPTHREAD, 0)
        if snapshot == _INVALID_HANDLE_VALUE:
            raise ctypes.WinError(ctypes.get_last_error())
        thread_ids: list[int] = []
        try:
            entry = _ThreadEntry32()
            entry.dwSize = ctypes.sizeof(entry)
            if not _KERNEL32.Thread32First(snapshot, ctypes.byref(entry)):
                raise ctypes.WinError(ctypes.get_last_error())
            while True:
                if int(entry.th32OwnerProcessID) == pid:
                    thread_ids.append(int(entry.th32ThreadID))
                entry.dwSize = ctypes.sizeof(entry)
                ctypes.set_last_error(0)
                if not _KERNEL32.Thread32Next(snapshot, ctypes.byref(entry)):
                    error = ctypes.get_last_error()
                    if error not in (0, _ERROR_NO_MORE_FILES):
                        raise ctypes.WinError(error)
                    break
        finally:
            _KERNEL32.CloseHandle(snapshot)
        if len(thread_ids) != 1:
            raise ProcessOwnershipError(
                f"suspended child must expose exactly one primary thread; found {len(thread_ids)}"
            )
        thread = _KERNEL32.OpenThread(_THREAD_SUSPEND_RESUME, False, thread_ids[0])
        if not thread:
            raise ctypes.WinError(ctypes.get_last_error())
        try:
            previous_suspend_count = _KERNEL32.ResumeThread(thread)
            if previous_suspend_count == 0xFFFFFFFF:
                raise ctypes.WinError(ctypes.get_last_error())
            if previous_suspend_count != 1:
                raise ProcessOwnershipError(
                    "child primary thread did not have the expected suspend count"
                )
        finally:
            _KERNEL32.CloseHandle(thread)


class ProcessTreeOwner:
    """Bind one direct child and every descendant to one cleanup boundary."""

    def __init__(self, *, affinity_mask: int | None = None) -> None:
        if affinity_mask is not None:
            affinity_mask = validate_windows_affinity_mask(affinity_mask)
            if os.name != "nt":
                raise WindowsCpuIsolationError(
                    "process affinity is only available on Windows"
                )
        self._affinity_mask = affinity_mask
        if os.name == "nt":
            self._job = (
                _WindowsJob(affinity_mask=affinity_mask)
                if affinity_mask is not None
                else _WindowsJob()
            )
        else:
            self._job = None
        self._process_group: int | None = None
        self._suspended_launch_pending = False
        self._closed = False

    def popen_options(self, *, creationflags: int = 0) -> dict[str, Any]:
        """Return platform launch options required before attaching a child."""

        if os.name == "nt":
            if self._affinity_mask is not None:
                if self._closed:
                    raise ProcessOwnershipError("process tree owner is closed")
                conflicting_priority = creationflags & (
                    _PRIORITY_CLASS_FLAGS & ~_NORMAL_PRIORITY_CLASS
                )
                if conflicting_priority:
                    raise ProcessOwnershipError(
                        "affinity launch requested a non-Normal priority class"
                    )
                creationflags |= _CREATE_SUSPENDED | _NORMAL_PRIORITY_CLASS
                self._suspended_launch_pending = True
            return {"creationflags": creationflags}
        if os.name == "posix":
            return {"start_new_session": True}
        return {}

    def attach(self, process: subprocess.Popen[Any]) -> None:
        """Attach a newly launched direct child or fail closed."""

        if self._closed:
            raise ProcessOwnershipError("process tree owner is closed")
        if not isinstance(process.pid, int) or process.pid <= 0:
            raise ProcessOwnershipError("owned child has an invalid pid")
        try:
            if self._job is not None:
                if self._affinity_mask is not None and not self._suspended_launch_pending:
                    raise ProcessOwnershipError(
                        "affinity child was not launched through suspended Job admission"
                    )
                self._job.assign(process.pid)
                if self._affinity_mask is not None:
                    _resume_suspended_process(process.pid)
                    self._suspended_launch_pending = False
            elif os.name == "posix":
                # Windows typeshed omits POSIX APIs even though this guarded
                # branch is the implementation used on POSIX hosts.
                if os.getpgid(process.pid) != process.pid:  # type: ignore[attr-defined]
                    raise ProcessOwnershipError("POSIX child was not launched in its own process group")
                self._process_group = process.pid
        except BaseException as exc:
            cleanup_error: BaseException | None = None
            try:
                self._terminate_direct(process)
            except BaseException as cleanup_exc:
                cleanup_error = cleanup_exc
            self.close()
            cleanup_detail = "" if cleanup_error is None else f"; direct cleanup also failed: {cleanup_error}"
            raise ProcessOwnershipError(f"cannot own child process tree for pid {process.pid}{cleanup_detail}") from exc

    def terminate(self, process: subprocess.Popen[Any], *, timeout_s: float) -> None:
        """Terminate the complete tree, escalate, and release ownership."""

        if timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        if process.poll() is not None:
            self.close_after_exit()
            return
        try:
            if self._job is not None:
                self._job.terminate()
                self._wait_direct(process, timeout_s)
            elif self._process_group is not None and os.name == "posix":
                self._terminate_posix_group(process, timeout_s)
            else:
                self._terminate_direct(process, timeout_s)
        finally:
            self.close()

    def close_after_exit(self) -> None:
        """Release an exited direct child and remove lingering descendants."""

        try:
            if self._process_group is not None and os.name == "posix":
                self._signal_group(signal.SIGTERM)
                self._wait_group_empty(0.5)
                self._signal_group(signal.SIGKILL)  # type: ignore[attr-defined]
                if not self._wait_group_empty(1.0):
                    raise ProcessOwnershipError(
                        "POSIX process group remained alive after SIGKILL"
                    )
        finally:
            self.close()

    def close(self) -> None:
        """Close the ownership boundary; a Windows close kills the whole Job."""

        if self._closed:
            return
        self._closed = True
        self._suspended_launch_pending = False
        if self._job is not None:
            self._job.close()
        self._process_group = None

    def _terminate_posix_group(self, process: subprocess.Popen[Any], timeout_s: float) -> None:
        self._signal_group(signal.SIGTERM)
        try:
            process.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            pass
        self._wait_group_empty(min(timeout_s, 0.5))
        self._signal_group(signal.SIGKILL)  # type: ignore[attr-defined]
        self._wait_direct(process, timeout_s)
        if not self._wait_group_empty(timeout_s):
            raise ProcessOwnershipError(
                "POSIX process group remained alive after SIGKILL"
            )

    def _signal_group(self, value: signal.Signals) -> None:
        group = self._process_group
        if group is None:
            return
        try:
            os.killpg(group, value)  # type: ignore[attr-defined]
        except ProcessLookupError:
            return

    def _wait_group_empty(self, timeout_s: float) -> bool:
        group = self._process_group
        if group is None:
            return True
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                os.killpg(group, 0)  # type: ignore[attr-defined]
            except ProcessLookupError:
                return True
            time.sleep(0.01)
        return False

    @staticmethod
    def _wait_direct(process: subprocess.Popen[Any], timeout_s: float) -> None:
        try:
            process.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=timeout_s)

    @staticmethod
    def _terminate_direct(process: subprocess.Popen[Any], timeout_s: float = 1.0) -> None:
        if process.poll() is None:
            process.terminate()
        ProcessTreeOwner._wait_direct(process, timeout_s)


__all__ = [
    "ProcessOwnershipError",
    "ProcessShutdownSnapshot",
    "ProcessTreeOwner",
]

# ruff: noqa: S101, S603

from __future__ import annotations

import ctypes
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import pytest

from sim.runtime.process_owner import ProcessTreeOwner
from sim.runtime.windows_cpu_isolation import discover_windows_cpu_topology


def _process_has_exited(pid: int) -> bool:
    if os.name == "nt":
        from ctypes import wintypes

        synchronize = 0x00100000
        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        kernel32.OpenProcess.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
        kernel32.OpenProcess.restype = wintypes.HANDLE
        kernel32.WaitForSingleObject.argtypes = [wintypes.HANDLE, wintypes.DWORD]
        kernel32.WaitForSingleObject.restype = wintypes.DWORD
        kernel32.CloseHandle.argtypes = [wintypes.HANDLE]
        handle = kernel32.OpenProcess(synchronize, False, pid)
        if not handle:
            return True
        try:
            return kernel32.WaitForSingleObject(handle, 0) == 0
        finally:
            kernel32.CloseHandle(handle)
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return True
    return False


def test_process_tree_owner_sets_only_the_host_specific_popen_option() -> None:
    owner = ProcessTreeOwner()
    try:
        options = owner.popen_options(creationflags=123)
        if os.name == "nt":
            assert options == {"creationflags": 123}
        elif os.name == "posix":
            assert options == {"start_new_session": True}
        else:
            assert options == {}
    finally:
        owner.close()


def test_process_tree_owner_terminates_a_descendant(tmp_path: Path) -> None:
    child_pid_path = tmp_path / "child.pid"
    parent_code = """
import pathlib
import subprocess
import sys
import time

input()
child = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])
pathlib.Path(sys.argv[1]).write_text(str(child.pid), encoding="utf-8")
while True:
    time.sleep(1)
"""
    owner = ProcessTreeOwner()
    process = subprocess.Popen(
        [sys.executable, "-c", parent_code, str(child_pid_path)],
        stdin=subprocess.PIPE,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        text=True,
        **owner.popen_options(),
    )
    try:
        owner.attach(process)
        assert process.stdin is not None
        process.stdin.write("go\n")
        process.stdin.flush()
        deadline = time.monotonic() + 5.0
        while not child_pid_path.is_file() and time.monotonic() < deadline:
            time.sleep(0.01)
        assert child_pid_path.is_file()
        child_pid = int(child_pid_path.read_text(encoding="utf-8"))

        owner.terminate(process, timeout_s=3.0)

        deadline = time.monotonic() + 3.0
        while not _process_has_exited(child_pid) and time.monotonic() < deadline:
            time.sleep(0.01)
        assert _process_has_exited(child_pid)
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=3.0)
        owner.close()


def _current_process_affinity_mask() -> int:
    from ctypes import wintypes

    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel32.GetCurrentProcess.argtypes = []
    kernel32.GetCurrentProcess.restype = wintypes.HANDLE
    kernel32.GetProcessAffinityMask.argtypes = [
        wintypes.HANDLE,
        ctypes.POINTER(ctypes.c_size_t),
        ctypes.POINTER(ctypes.c_size_t),
    ]
    kernel32.GetProcessAffinityMask.restype = wintypes.BOOL
    process_mask = ctypes.c_size_t()
    system_mask = ctypes.c_size_t()
    if not kernel32.GetProcessAffinityMask(
        kernel32.GetCurrentProcess(),
        ctypes.byref(process_mask),
        ctypes.byref(system_mask),
    ):
        raise ctypes.WinError(ctypes.get_last_error())
    return int(process_mask.value)


@pytest.mark.skipif(os.name != "nt", reason="requires real Windows Job APIs")
def test_affinity_child_is_suspended_until_verified_and_descendant_inherits_job(
    tmp_path: Path,
) -> None:
    marker = tmp_path / "child-started.txt"
    allowed_mask = _current_process_affinity_mask()
    topology = discover_windows_cpu_topology()
    affinity_mask = next(
        core.affinity_mask
        for core in topology.cores
        if core.processor_group == 0
        and core.affinity_mask & allowed_mask == core.affinity_mask
    )
    helper_code = r"""
import ctypes
import json
import pathlib
import subprocess
import sys
from ctypes import wintypes

NORMAL_PRIORITY_CLASS = 0x20
BELOW_NORMAL_PRIORITY_CLASS = 0x4000
kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
kernel32.GetCurrentProcess.argtypes = []
kernel32.GetCurrentProcess.restype = wintypes.HANDLE
kernel32.IsProcessInJob.argtypes = [wintypes.HANDLE, wintypes.HANDLE, ctypes.POINTER(wintypes.BOOL)]
kernel32.IsProcessInJob.restype = wintypes.BOOL
kernel32.GetProcessAffinityMask.argtypes = [wintypes.HANDLE, ctypes.POINTER(ctypes.c_size_t), ctypes.POINTER(ctypes.c_size_t)]
kernel32.GetProcessAffinityMask.restype = wintypes.BOOL
kernel32.GetPriorityClass.argtypes = [wintypes.HANDLE]
kernel32.GetPriorityClass.restype = wintypes.DWORD
kernel32.SetPriorityClass.argtypes = [wintypes.HANDLE, wintypes.DWORD]
kernel32.SetPriorityClass.restype = wintypes.BOOL

def probe():
    process = kernel32.GetCurrentProcess()
    in_job = wintypes.BOOL()
    process_mask = ctypes.c_size_t()
    system_mask = ctypes.c_size_t()
    if not kernel32.IsProcessInJob(process, None, ctypes.byref(in_job)):
        raise ctypes.WinError(ctypes.get_last_error())
    if not kernel32.GetProcessAffinityMask(process, ctypes.byref(process_mask), ctypes.byref(system_mask)):
        raise ctypes.WinError(ctypes.get_last_error())
    priority = kernel32.GetPriorityClass(process)
    if not priority:
        raise ctypes.WinError(ctypes.get_last_error())
    priority_change_succeeded = bool(kernel32.SetPriorityClass(process, BELOW_NORMAL_PRIORITY_CLASS))
    priority_after_change = kernel32.GetPriorityClass(process)
    if not priority_after_change:
        raise ctypes.WinError(ctypes.get_last_error())
    return {
        "in_job": bool(in_job.value),
        "affinity_mask": int(process_mask.value),
        "priority": int(priority),
        "priority_change_succeeded": priority_change_succeeded,
        "priority_after_change": int(priority_after_change),
    }

if sys.argv[1] == "child":
    print(json.dumps(probe()), flush=True)
else:
    pathlib.Path(sys.argv[1]).write_text("started", encoding="utf-8")
    child = subprocess.run(
        [sys.executable, "-c", sys.argv[2], "child"],
        check=True,
        capture_output=True,
        text=True,
        timeout=10,
    )
    print(json.dumps({"parent": probe(), "child": json.loads(child.stdout)}), flush=True)
"""
    owner = ProcessTreeOwner(affinity_mask=affinity_mask)
    process = subprocess.Popen(
        [sys.executable, "-c", helper_code, str(marker), helper_code],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        **owner.popen_options(),
    )
    try:
        time.sleep(0.15)
        assert not marker.exists(), "affinity child executed before Job verification"

        owner.attach(process)
        stdout, stderr = process.communicate(timeout=15.0)
        assert process.returncode == 0, stderr
        report = json.loads(stdout)
        for role in ("parent", "child"):
            assert isinstance(report[role].pop("priority_change_succeeded"), bool)
            assert report[role] == {
                "in_job": True,
                "affinity_mask": affinity_mask,
                "priority": subprocess.NORMAL_PRIORITY_CLASS,
                "priority_after_change": subprocess.NORMAL_PRIORITY_CLASS,
            }
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=3.0)
        owner.close_after_exit()

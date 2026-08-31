"""Windows CPU topology and affinity partition contracts."""

# ruff: noqa: S101

from __future__ import annotations

import os
import subprocess

import pytest

import sim.runtime.process_owner as process_owner_module
import sim.runtime.windows_cpu_isolation as cpu_isolation_module
from sim.runtime.process_owner import ProcessTreeOwner
from sim.runtime.windows_cpu_isolation import (
    WindowsCpuIsolationConfig,
    WindowsCpuIsolationError,
    WindowsCpuIsolationPlan,
    WindowsCpuTopology,
    WindowsPhysicalCore,
    bind_current_thread_affinity,
    discover_windows_cpu_topology,
    resolve_windows_cpu_isolation,
)


def _topology() -> WindowsCpuTopology:
    return WindowsCpuTopology(
        cores=(
            WindowsPhysicalCore(0, 0, 0b00000011, 2),
            WindowsPhysicalCore(1, 0, 0b00001100, 2),
            WindowsPhysicalCore(2, 0, 0b00110000, 1),
            WindowsPhysicalCore(3, 0, 0b11000000, 1),
        )
    )


def test_resolver_assigns_two_complete_performance_cores_and_leaves_rest_for_unreal() -> None:
    plan = resolve_windows_cpu_isolation(
        WindowsCpuIsolationConfig(),
        topology=_topology(),
    )

    assert plan.processor_group == 0
    assert plan.mujoco_affinity_mask == 0b00000011
    assert plan.owner_thread_affinity_mask == 0b00001100
    assert plan.unreal_affinity_mask == 0b11110000
    assert plan.mujoco_core_id == 0
    assert plan.owner_core_id == 1
    assert plan.unreal_core_ids == (2, 3)


def test_default_resolver_excludes_cores_outside_launcher_process_affinity(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        cpu_isolation_module,
        "discover_windows_cpu_topology",
        _topology,
    )
    monkeypatch.setattr(
        cpu_isolation_module,
        "discover_windows_process_affinity_mask",
        lambda: 0b11111101,
        raising=False,
    )

    plan = resolve_windows_cpu_isolation(WindowsCpuIsolationConfig())

    assert plan.mujoco_core_id == 1
    assert plan.owner_core_id == 2
    assert plan.unreal_core_ids == (3,)
    assert plan.mujoco_affinity_mask == 0b00001100
    assert plan.owner_thread_affinity_mask == 0b00110000
    assert plan.unreal_affinity_mask == 0b11000000


def test_topology_rejects_overlapping_core_masks() -> None:
    with pytest.raises(WindowsCpuIsolationError, match="overlap"):
        WindowsCpuTopology(
            cores=(
                WindowsPhysicalCore(0, 0, 0b0011, 0),
                WindowsPhysicalCore(1, 0, 0b0010, 0),
                WindowsPhysicalCore(2, 0, 0b1100, 0),
            )
        )


def test_physical_core_rejects_empty_affinity_mask() -> None:
    with pytest.raises(WindowsCpuIsolationError, match="non-empty"):
        WindowsPhysicalCore(0, 0, 0, 0)


def test_resolver_rejects_empty_unreal_partition() -> None:
    topology = WindowsCpuTopology(
        cores=(
            WindowsPhysicalCore(0, 0, 0b0011, 0),
            WindowsPhysicalCore(1, 0, 0b1100, 0),
        )
    )

    with pytest.raises(WindowsCpuIsolationError, match="Unreal partition is empty"):
        resolve_windows_cpu_isolation(WindowsCpuIsolationConfig(), topology=topology)


def test_config_rejects_overlapping_role_ranks() -> None:
    with pytest.raises(WindowsCpuIsolationError, match="must be distinct"):
        WindowsCpuIsolationConfig(
            mujoco_performance_rank=0,
            owner_performance_rank=0,
        )


def test_resolved_plan_rejects_overlapping_role_masks() -> None:
    with pytest.raises(WindowsCpuIsolationError, match="masks overlap"):
        WindowsCpuIsolationPlan(
            processor_group=0,
            mujoco_core_id=0,
            owner_core_id=1,
            unreal_core_ids=(2,),
            mujoco_affinity_mask=0b0011,
            owner_thread_affinity_mask=0b0110,
            unreal_affinity_mask=0b1000,
        )


def test_resolver_rejects_multi_group_topology_in_affinity_v1() -> None:
    topology = WindowsCpuTopology(
        cores=(
            WindowsPhysicalCore(0, 0, 0b0011, 0),
            WindowsPhysicalCore(1, 0, 0b1100, 0),
            WindowsPhysicalCore(2, 1, 0b0011, 0),
        )
    )

    with pytest.raises(WindowsCpuIsolationError, match="one Windows processor group"):
        resolve_windows_cpu_isolation(WindowsCpuIsolationConfig(), topology=topology)


def test_thread_affinity_scope_restores_the_previous_mask() -> None:
    calls: list[tuple[str, int]] = []

    class Api:
        def bind_current_thread(self, mask: int) -> int:
            calls.append(("bind", mask))
            return 0b11111111

        def restore_current_thread(self, mask: int) -> None:
            calls.append(("restore", mask))

    with bind_current_thread_affinity(0b0011, api=Api()):
        calls.append(("body", 0))

    assert calls == [
        ("bind", 0b0011),
        ("body", 0),
        ("restore", 0b11111111),
    ]


def test_thread_affinity_scope_restores_after_body_failure() -> None:
    calls: list[tuple[str, int]] = []

    class Api:
        def bind_current_thread(self, mask: int) -> int:
            calls.append(("bind", mask))
            return 0b11111111

        def restore_current_thread(self, mask: int) -> None:
            calls.append(("restore", mask))

    with pytest.raises(RuntimeError, match="body failed"):
        with bind_current_thread_affinity(0b0011, api=Api()):
            raise RuntimeError("body failed")

    assert calls == [("bind", 0b0011), ("restore", 0b11111111)]


def test_thread_affinity_scope_rolls_back_when_enter_validation_fails() -> None:
    calls: list[tuple[str, int]] = []

    class Api:
        def bind_current_thread(self, mask: int) -> int:
            calls.append(("bind", mask))
            return 0

        def restore_current_thread(self, mask: int) -> None:
            calls.append(("restore", mask))

    with pytest.raises(WindowsCpuIsolationError, match="previous thread affinity"):
        with bind_current_thread_affinity(0b0011, api=Api()):
            raise AssertionError("scope body must not run")

    assert calls == [("bind", 0b0011), ("restore", 0)]


@pytest.mark.skipif(os.name != "nt", reason="Windows Job Object contract")
def test_process_tree_owner_configures_affinity_before_attaching_child(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[tuple[str, int | None]] = []

    class Job:
        def __init__(self, affinity_mask: int | None = None) -> None:
            calls.append(("job", affinity_mask))

        def assign(self, pid: int) -> None:
            calls.append(("assign", pid))

        def terminate(self) -> None:
            calls.append(("terminate", None))

        def close(self) -> None:
            calls.append(("close", None))

    class Process:
        pid = 321

        @staticmethod
        def poll() -> None:
            return None

    monkeypatch.setattr(process_owner_module, "_WindowsJob", Job)
    monkeypatch.setattr(
        process_owner_module,
        "_resume_suspended_process",
        lambda pid: calls.append(("resume", pid)),
        raising=False,
    )

    owner = ProcessTreeOwner(affinity_mask=0b0011)
    options = owner.popen_options(creationflags=0x08000000)
    owner.attach(Process())  # type: ignore[arg-type]

    assert options["creationflags"] & 0x00000004
    assert options["creationflags"] & subprocess.NORMAL_PRIORITY_CLASS
    assert calls[:3] == [
        ("job", 0b0011),
        ("assign", 321),
        ("resume", 321),
    ]


@pytest.mark.skipif(os.name != "nt", reason="Windows suspended launch contract")
def test_process_tree_owner_terminates_suspended_child_when_resume_fails(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[tuple[str, int | None]] = []

    class Job:
        def __init__(self, affinity_mask: int | None = None) -> None:
            calls.append(("job", affinity_mask))

        def assign(self, pid: int) -> None:
            calls.append(("assign", pid))

        def terminate(self) -> None:
            calls.append(("job_terminate", None))

        def close(self) -> None:
            calls.append(("close", None))

    class Process:
        pid = 322
        terminated = False

        def poll(self) -> int | None:
            return 1 if self.terminated else None

        def terminate(self) -> None:
            calls.append(("process_terminate", self.pid))
            self.terminated = True

        def wait(self, *, timeout: float) -> int:
            del timeout
            calls.append(("process_wait", self.pid))
            return 1

        def kill(self) -> None:
            calls.append(("process_kill", self.pid))
            self.terminated = True

    def fail_resume(pid: int) -> None:
        calls.append(("resume", pid))
        raise KeyboardInterrupt("resume interrupted")

    monkeypatch.setattr(process_owner_module, "_WindowsJob", Job)
    monkeypatch.setattr(
        process_owner_module,
        "_resume_suspended_process",
        fail_resume,
        raising=False,
    )

    owner = ProcessTreeOwner(affinity_mask=0b0011)
    owner.popen_options()
    with pytest.raises(process_owner_module.ProcessOwnershipError, match="cannot own"):
        owner.attach(Process())  # type: ignore[arg-type]

    assert calls == [
        ("job", 0b0011),
        ("assign", 322),
        ("resume", 322),
        ("process_terminate", 322),
        ("process_wait", 322),
        ("close", None),
    ]


@pytest.mark.skipif(os.name != "nt", reason="requires Win32 topology API")
def test_actual_windows_topology_resolves_complete_non_overlapping_partitions() -> None:
    topology = discover_windows_cpu_topology()
    plan = resolve_windows_cpu_isolation(WindowsCpuIsolationConfig(), topology=topology)

    assert len(topology.cores) >= 3
    assert plan.mujoco_affinity_mask & plan.owner_thread_affinity_mask == 0
    assert plan.mujoco_affinity_mask & plan.unreal_affinity_mask == 0
    assert plan.owner_thread_affinity_mask & plan.unreal_affinity_mask == 0
    assert (
        plan.mujoco_affinity_mask
        | plan.owner_thread_affinity_mask
        | plan.unreal_affinity_mask
    ) == topology.affinity_mask(0)

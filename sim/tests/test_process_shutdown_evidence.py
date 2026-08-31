"""Immutable shutdown facts for playable-owned runtime processes."""

# ruff: noqa: S101

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest

import sim.runtime.coordinator.mujoco_process as mujoco_process_module
import sim.runtime.coordinator.unreal_process as unreal_process_module
from sim.runtime.coordinator.mujoco_process import MujocoProcess
from sim.runtime.coordinator.unreal_process import UnrealProcess


class _Process:
    def __init__(self, *, pid: int, exit_code: int | None) -> None:
        self.pid = pid
        self.exit_code = exit_code
        self.stdin = None
        self.stdout = None

    def poll(self) -> int | None:
        return self.exit_code

    def terminate(self) -> None:
        self.exit_code = 1

    def kill(self) -> None:
        self.exit_code = 1

    def wait(self, timeout: float) -> int:
        del timeout
        if self.exit_code is None:
            self.exit_code = 1
        return self.exit_code


class _Owner:
    def __init__(
        self,
        *,
        terminated_exit_code: int = 1,
        close_error: BaseException | None = None,
    ) -> None:
        self.terminated_exit_code = terminated_exit_code
        self.close_error = close_error
        self.closed = False

    def terminate(self, process: _Process, *, timeout_s: float) -> None:
        assert timeout_s > 0
        if process.poll() is None:
            process.exit_code = self.terminated_exit_code

    def close_after_exit(self) -> None:
        if self.close_error is not None:
            raise self.close_error
        self.closed = True


def _unreal() -> UnrealProcess:
    return UnrealProcess(
        Path("D:/UnrealEditor.exe"),
        Path("D:/RobotSimUE.uproject"),
        "/Game/RobotSim/Maps/FactoryPark_HF",
    )


def test_unreal_locks_natural_zero_exit_only_after_owner_close() -> None:
    process = _Process(pid=4104, exit_code=0)
    owner = _Owner()
    unreal = _unreal()
    unreal._process = process  # type: ignore[assignment]
    unreal._process_owner = owner  # type: ignore[assignment]

    unreal.terminate()

    assert owner.closed is True
    assert unreal.last_shutdown is not None
    assert unreal.last_shutdown.pid == 4104
    assert unreal.last_shutdown.exit_code == 0
    assert unreal.last_shutdown.direct_child_running_after_close is False
    assert unreal.last_shutdown.process_owner_closed is True
    assert unreal.last_shutdown.termination_mode == "natural"


def test_unreal_records_owned_termination_as_nonqualifying_fact() -> None:
    process = _Process(pid=4105, exit_code=None)
    unreal = _unreal()
    unreal._process = process  # type: ignore[assignment]
    unreal._process_owner = _Owner(terminated_exit_code=1)  # type: ignore[assignment]

    unreal.terminate()

    assert unreal.last_shutdown is not None
    assert unreal.last_shutdown.exit_code == 1
    assert unreal.last_shutdown.termination_mode == "owned_terminate"


def test_unreal_preserves_shutdown_snapshot_when_owner_close_raises() -> None:
    process = _Process(pid=4106, exit_code=0)
    error = RuntimeError("fake descendant cleanup failed")
    unreal = _unreal()
    unreal._process = process  # type: ignore[assignment]
    unreal._process_owner = _Owner(close_error=error)  # type: ignore[assignment]

    with pytest.raises(RuntimeError, match="fake descendant cleanup failed"):
        unreal.terminate()

    assert unreal.last_shutdown is not None
    assert unreal.last_shutdown.exit_code == 0
    assert unreal.last_shutdown.process_owner_closed is False


def test_unreal_terminate_preserves_primary_baseexception_and_notes_cleanup() -> None:
    process = _Process(pid=4108, exit_code=None)
    interruption = KeyboardInterrupt("terminate interrupted")

    class FailingOwner(_Owner):
        def terminate(self, process: _Process, *, timeout_s: float) -> None:
            del process, timeout_s
            raise interruption

        def close_after_exit(self) -> None:
            raise RuntimeError("owner close after interrupt failed")

    unreal = _unreal()
    unreal._process = process  # type: ignore[assignment]
    unreal._process_owner = FailingOwner()  # type: ignore[assignment]

    with pytest.raises(KeyboardInterrupt) as error:
        unreal.terminate()

    assert error.value is interruption
    notes = "\n".join(getattr(error.value, "__notes__", ()))
    assert "owner close after interrupt failed" in notes
    assert unreal._process is None
    assert unreal._process_owner is None
    assert unreal.last_shutdown is not None
    assert unreal.last_shutdown.process_owner_closed is False


def test_mujoco_locks_natural_zero_exit_and_owner_close() -> None:
    process = _Process(pid=5104, exit_code=0)
    owner = _Owner()
    mujoco = MujocoProcess(Path("D:/mujoco-host.exe"))
    mujoco._process = process  # type: ignore[assignment]
    mujoco._process_owner = owner  # type: ignore[assignment]

    mujoco._close(termination_mode="natural")

    assert owner.closed is True
    assert mujoco.last_shutdown is not None
    assert mujoco.last_shutdown.pid == 5104
    assert mujoco.last_shutdown.exit_code == 0
    assert mujoco.last_shutdown.direct_child_running_after_close is False
    assert mujoco.last_shutdown.process_owner_closed is True
    assert mujoco.last_shutdown.termination_mode == "natural"


def test_mujoco_stop_keyboard_interrupt_terminates_before_reraising(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    process = _Process(pid=5105, exit_code=None)
    owner = _Owner()
    mujoco = MujocoProcess(Path("D:/mujoco-host.exe"))
    mujoco._process = process  # type: ignore[assignment]
    mujoco._process_owner = owner  # type: ignore[assignment]
    interruption = KeyboardInterrupt("stop interrupted")

    def fail_request(_command: str) -> dict[str, Any]:
        raise interruption

    monkeypatch.setattr(mujoco, "_request", fail_request)

    with pytest.raises(KeyboardInterrupt) as error:
        mujoco.stop()

    assert error.value is interruption
    assert owner.closed is True
    assert mujoco._process is None
    assert mujoco._process_owner is None
    assert mujoco.last_shutdown is not None
    assert mujoco.last_shutdown.direct_child_running_after_close is False


def test_mujoco_owner_preflight_failure_closes_stderr(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    executable = tmp_path / "mujoco-host.exe"
    executable.write_text("", encoding="utf-8")
    log_dir = tmp_path / "logs"
    log_dir.mkdir()
    mujoco = MujocoProcess(executable, affinity_mask=0b0011)
    monkeypatch.setattr(mujoco, "_command", lambda _plan: [str(executable)])
    monkeypatch.setattr(
        mujoco_process_module,
        "ProcessTreeOwner",
        lambda **_kwargs: (_ for _ in ()).throw(RuntimeError("Job preflight failed")),
    )

    with pytest.raises(RuntimeError, match="Job preflight failed"):
        mujoco.prepare(
            SimpleNamespace(repo_root=tmp_path),  # type: ignore[arg-type]
            SimpleNamespace(log_dir=log_dir),  # type: ignore[arg-type]
        )

    assert mujoco._stderr is None
    assert mujoco._process is None
    assert mujoco._process_owner is None


def test_unreal_owner_preflight_failure_closes_stderr(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    unreal = _unreal()
    monkeypatch.setattr(
        unreal_process_module,
        "ProcessTreeOwner",
        lambda **_kwargs: (_ for _ in ()).throw(RuntimeError("Job preflight failed")),
    )

    with pytest.raises(RuntimeError, match="Job preflight failed"):
        unreal._launch(
            command=["D:/UnrealEditor.exe"],
            cwd=tmp_path,
            allocation=SimpleNamespace(
                log_dir=tmp_path,
                child_environment=lambda: {},
            ),
            stderr_name="unreal.stderr.log",
        )

    assert unreal._stderr is None
    assert unreal._process is None
    assert unreal._process_owner is None


def test_unreal_launch_preserves_original_error_when_cleanup_fails(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    process = _Process(pid=4107, exit_code=None)

    class FailingOwner:
        def popen_options(self, *, creationflags: int = 0) -> dict[str, int]:
            return {"creationflags": creationflags}

        def attach(self, _process: _Process) -> None:
            raise ValueError("attach failed")

        def terminate(self, _process: _Process, *, timeout_s: float) -> None:
            del timeout_s
            raise RuntimeError("owner terminate failed")

        def close(self) -> None:
            raise RuntimeError("owner close failed")

    monkeypatch.setattr(unreal_process_module, "ProcessTreeOwner", FailingOwner)
    unreal = UnrealProcess(
        Path("D:/UnrealEditor.exe"),
        Path("D:/RobotSimUE.uproject"),
        "/Game/RobotSim/Maps/FactoryPark_HF",
        popen_factory=lambda *_args, **_kwargs: process,  # type: ignore[arg-type]
    )

    with pytest.raises(ValueError, match="attach failed") as error:
        unreal._launch(
            command=["D:/UnrealEditor.exe"],
            cwd=tmp_path,
            allocation=SimpleNamespace(
                log_dir=tmp_path,
                child_environment=lambda: {},
            ),
            stderr_name="unreal.stderr.log",
        )

    notes = "\n".join(getattr(error.value, "__notes__", ()))
    assert "owner terminate failed" in notes
    assert "owner close failed" in notes
    assert unreal._stderr is None
    assert unreal._process is None
    assert unreal._process_owner is None

from __future__ import annotations

import json
import subprocess
import threading
from pathlib import Path

import pytest

from lingtu.control import ProductControl
from lingtu.operator_drive import main as drive_main
from lingtu.product_lock import ProductControlBusy, ProductControlLock
from lingtu.run_plan import CURRENT_RUN_SCHEMA


def _write_current(
    tmp_path: Path,
    *,
    product: str,
    domain_id: str | None = "0",
    env: str = "real",
    robot: str = "unitree/go2",
) -> None:
    control = ProductControl(
        robot=robot,
        env=env,
        env_config={"backend": "mujoco"} if env == "sim" else None,
        process_env={},
    )
    plan = control._resolve(product)
    if env == "real":
        native_environment = plan.native_process_environment
        if domain_id is None:
            native_environment.pop("LINGTU_DDS_DOMAIN_ID", None)
        else:
            native_environment["LINGTU_DDS_DOMAIN_ID"] = domain_id
        plan = plan.with_native_process_environment(native_environment)
    product_session_id = "1" * 32
    plan_path = plan.write(tmp_path / f"plan-{product_session_id}.json")
    (tmp_path / "current.json").write_text(
        json.dumps(
            {
                "schema_version": CURRENT_RUN_SCHEMA,
                "product": plan.product,
                "product_variant": plan.product_variant,
                "env": plan.env,
                "run_plan_path": str(plan_path),
                "product_session_id": product_session_id,
            }
        ),
        encoding="utf-8",
    )


def _ready_path(command: list[str]) -> Path:
    option = command.index("--ready-file")
    return Path(command[option + 1])


class _FakeProcess:
    def __init__(
        self,
        command: list[str],
        *,
        ready: bool = True,
        final_returncode: int = 0,
        stdout: str = "native details",
        stderr: str = "",
        finish: threading.Event | None = None,
        timeout_once: bool = False,
    ) -> None:
        self.command = command
        self.returncode: int | None = None
        self._final_returncode = final_returncode
        self._stdout = stdout
        self._stderr = stderr
        self._finish = finish
        self._timeout_once = timeout_once
        self._timed_out = False
        if ready:
            _ready_path(command).write_text("ready\n", encoding="utf-8")

    def poll(self) -> int | None:
        return self.returncode

    def communicate(self, timeout: float | None = None) -> tuple[str, str]:
        if self._timeout_once and not self._timed_out:
            self._timed_out = True
            raise subprocess.TimeoutExpired(
                self.command,
                timeout if timeout is not None else 0.0,
            )
        if self._finish is not None:
            assert self._finish.wait(timeout=2.0)
        self.returncode = self._final_returncode
        return self._stdout, self._stderr

    def terminate(self) -> None:
        self.returncode = -15

    def kill(self) -> None:
        self.returncode = -9


def _install_native(
    monkeypatch,
    *,
    final_returncode: int = 0,
    stderr: str = "",
    timeout_once: bool = False,
) -> list[list[str]]:
    calls: list[list[str]] = []

    def popen(command: list[str], **_kwargs) -> _FakeProcess:
        calls.append(command)
        return _FakeProcess(
            command,
            final_returncode=final_returncode,
            stderr=stderr,
            timeout_once=timeout_once,
        )

    monkeypatch.setattr("lingtu.operator_drive.subprocess.Popen", popen)
    return calls


def _arguments(tmp_path: Path, direction: str = "forward") -> list[str]:
    return [
        direction,
        "--robot",
        "unitree/go2",
        "--state-dir",
        str(tmp_path),
    ]


def test_drive_forward_hides_native_command_and_uses_bounded_default(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    _write_current(tmp_path, product="teleop")
    binary = tmp_path / "lingtu_nav_control"
    binary.write_text("test binary", encoding="utf-8")
    monkeypatch.setenv("LINGTU_NAV_CONTROL_BIN", str(binary))
    calls = _install_native(monkeypatch)

    assert drive_main(_arguments(tmp_path)) == 0

    assert capsys.readouterr().out.strip() == (
        "completed: forward at 0.20 m/s for 2.0 s "
        "(nominal open-loop distance 0.40 m)"
    )
    command = calls[0]
    assert command[:14] == [
        str(binary),
        "operator-motion",
        "0.2",
        "0",
        "0",
        "--duration-s",
        "2",
        "--rate-hz",
        "50",
        "--source-id",
        "lingtu-drive",
        "--domain-id",
        "0",
        "--ready-file",
    ]
    assert _ready_path(command).parent == tmp_path.resolve()


def test_drive_accepts_a_larger_explicit_bounded_range(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    _write_current(tmp_path, product="teleop")
    binary = tmp_path / "lingtu_nav_control"
    binary.write_text("test binary", encoding="utf-8")
    monkeypatch.setenv("LINGTU_NAV_CONTROL_BIN", str(binary))
    _install_native(monkeypatch)

    assert drive_main([*_arguments(tmp_path), "--speed", "0.3", "--seconds", "3"]) == 0
    assert capsys.readouterr().out.strip() == (
        "completed: forward at 0.30 m/s for 3.0 s "
        "(nominal open-loop distance 0.90 m)"
    )


def test_drive_accepts_the_active_simulation_environment(
    tmp_path: Path,
    capsys,
) -> None:
    _write_current(
        tmp_path,
        product="teleop_avoid",
        env="sim",
        robot="doso/thunder_v4",
    )

    assert drive_main(
        [
            "forward",
            "--robot",
            "doso/thunder_v4",
            "--env",
            "sim",
            "--state-dir",
            str(tmp_path),
            "--dry-run",
            "--json",
        ]
    ) == 0

    assert json.loads(capsys.readouterr().out)["env"] == "sim"


def test_drive_accepts_teleop_avoid_and_uses_its_exact_dds_domain(
    tmp_path: Path,
    monkeypatch,
) -> None:
    _write_current(tmp_path, product="teleop_avoid", domain_id="37")
    binary = tmp_path / "lingtu_nav_control"
    binary.write_text("test binary", encoding="utf-8")
    monkeypatch.setenv("LINGTU_NAV_CONTROL_BIN", str(binary))
    calls = _install_native(monkeypatch)

    assert drive_main(_arguments(tmp_path, "left")) == 0
    domain_option = calls[0].index("--domain-id")
    assert calls[0][domain_option + 1] == "37"


def test_drive_rejects_missing_dds_domain_before_native_motion(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    _write_current(tmp_path, product="teleop", domain_id=None)
    native_called = False

    def popen(command: list[str], **_kwargs) -> _FakeProcess:
        nonlocal native_called
        native_called = True
        return _FakeProcess(command)

    monkeypatch.setattr("lingtu.operator_drive.subprocess.Popen", popen)

    assert drive_main(_arguments(tmp_path)) == 2
    assert native_called is False
    assert "active RunPlan is missing LINGTU_DDS_DOMAIN_ID" in capsys.readouterr().err


def test_drive_propagates_native_cleanup_failure_without_success_report(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    _write_current(tmp_path, product="teleop")
    binary = tmp_path / "lingtu_nav_control"
    binary.write_text("test binary", encoding="utf-8")
    monkeypatch.setenv("LINGTU_NAV_CONTROL_BIN", str(binary))
    _install_native(
        monkeypatch,
        final_returncode=1,
        stderr="operator-motion cleanup release failed",
    )

    assert drive_main(_arguments(tmp_path)) == 2
    captured = capsys.readouterr()
    assert "operator-motion cleanup release failed" in captured.err
    assert "completed" not in captured.out


def test_drive_reports_native_timeout_without_success_report(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    _write_current(tmp_path, product="teleop")
    binary = tmp_path / "lingtu_nav_control"
    binary.write_text("test binary", encoding="utf-8")
    monkeypatch.setenv("LINGTU_NAV_CONTROL_BIN", str(binary))
    _install_native(monkeypatch, timeout_once=True)

    assert drive_main(_arguments(tmp_path)) == 2
    captured = capsys.readouterr()
    assert "operator-motion timed out" in captured.err
    assert "completed" not in captured.out


def test_drive_holds_product_lock_only_until_native_admission(
    tmp_path: Path,
    monkeypatch,
) -> None:
    _write_current(tmp_path, product="teleop")
    binary = tmp_path / "lingtu_nav_control"
    binary.write_text("test binary", encoding="utf-8")
    monkeypatch.setenv("LINGTU_NAV_CONTROL_BIN", str(binary))
    native_started = threading.Event()
    native_finished = threading.Event()
    ready_paths: list[Path] = []
    exit_codes: list[int] = []

    def popen(command: list[str], **_kwargs) -> _FakeProcess:
        ready_paths.append(_ready_path(command))
        native_started.set()
        return _FakeProcess(command, ready=False, finish=native_finished)

    monkeypatch.setattr("lingtu.operator_drive.subprocess.Popen", popen)

    thread = threading.Thread(
        target=lambda: exit_codes.append(drive_main(_arguments(tmp_path)))
    )
    thread.start()
    assert native_started.wait(timeout=2.0)
    with pytest.raises(ProductControlBusy):
        with ProductControlLock(tmp_path, timeout_s=0.0):
            pass

    ready_paths[0].write_text("ready\n", encoding="utf-8")
    with ProductControlLock(tmp_path, timeout_s=1.0):
        pass
    assert native_finished.is_set() is False

    native_finished.set()
    thread.join(timeout=2.0)
    assert thread.is_alive() is False
    assert exit_codes == [0]


def test_drive_rejects_missing_active_product(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    native_called = False

    def popen(command: list[str], **_kwargs) -> _FakeProcess:
        nonlocal native_called
        native_called = True
        return _FakeProcess(command)

    monkeypatch.setattr("lingtu.operator_drive.subprocess.Popen", popen)

    assert drive_main(_arguments(tmp_path)) == 2
    assert native_called is False
    assert "current run record not found" in capsys.readouterr().err


def test_drive_rejects_a_non_teleop_product_before_native_motion(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    _write_current(tmp_path, product="map")
    native_called = False

    def popen(command: list[str], **_kwargs) -> _FakeProcess:
        nonlocal native_called
        native_called = True
        return _FakeProcess(command)

    monkeypatch.setattr("lingtu.operator_drive.subprocess.Popen", popen)

    assert drive_main(_arguments(tmp_path)) == 2
    assert native_called is False
    assert "requires the active Product to be teleop or teleop_avoid" in capsys.readouterr().err

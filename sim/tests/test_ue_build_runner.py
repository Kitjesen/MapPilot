"""Integration contracts for the owned Build.bat and direct UBT launchers."""

# ruff: noqa: S101, S603, S607

from __future__ import annotations

import ctypes
import json
import os
import subprocess
import sys
import threading
import time
from _thread import interrupt_main
from pathlib import Path
from typing import TypedDict

import pytest

from sim.toolchains.ue_build import (
    UnrealBuildLockTimeoutError,
    UnrealBuildTimeoutError,
    project_lock_path,
    run_unreal_build,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
RUNNER = REPO_ROOT / "sim" / "toolchains" / "ue_build.py"


def _process_has_exited(pid: int) -> bool:
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
        return bool(kernel32.WaitForSingleObject(handle, 0) == 0)
    finally:
        kernel32.CloseHandle(handle)


def _wait_for_pid(path: Path, *, timeout_s: float = 8.0) -> int:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if path.is_file():
            try:
                value = path.read_text(encoding="ascii").strip()
                if value:
                    return int(value)
            except (OSError, ValueError):
                pass
        time.sleep(0.02)
    raise AssertionError(f"timed out waiting for fake build child pid: {path}")


def _wait_for_exit(pid: int, *, timeout_s: float = 5.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if _process_has_exited(pid):
            return True
        time.sleep(0.02)
    return _process_has_exited(pid)


def _terminate_pid(pid: int) -> None:
    if not _process_has_exited(pid):
        os.kill(pid, 15)


def _write_fake_build(tmp_path: Path, pid_path: Path) -> Path:
    child_script = tmp_path / "fake_ubt_child.py"
    child_script.write_text(
        """
import pathlib
import sys
import time

pathlib.Path(sys.argv[1]).write_text(str(__import__("os").getpid()), encoding="ascii")
time.sleep(60)
""".lstrip(),
        encoding="utf-8",
    )
    build_bat = tmp_path / "Build.bat"
    build_bat.write_text(
        f'@echo off\r\n"{sys.executable}" "{child_script}" "{pid_path}"\r\n',
        encoding="ascii",
    )
    return build_bat


def _write_backgrounding_fake_build(tmp_path: Path, pid_path: Path) -> Path:
    child_script = tmp_path / "fake_background_ubt_child.py"
    child_script.write_text(
        """
import os
import pathlib
import sys
import time

pathlib.Path(sys.argv[1]).write_text(str(os.getpid()), encoding="ascii")
time.sleep(60)
""".lstrip(),
        encoding="utf-8",
    )
    build_bat = tmp_path / "Build.bat"
    build_bat.write_text(
        (
            "@echo off\r\n"
            f'start "" /b "{sys.executable}" "{child_script}" "{pid_path}"\r\n'
            ":wait_for_pid\r\n"
            f'if not exist "{pid_path}" (\r\n'
            "  ping -n 2 127.0.0.1 >nul\r\n"
            "  goto wait_for_pid\r\n"
            ")\r\n"
            "exit /b 0\r\n"
        ),
        encoding="ascii",
    )
    return build_bat


def _write_fake_ubt_dll(tmp_path: Path) -> Path:
    ubt_dll = tmp_path / "UnrealBuildTool.dll"
    ubt_dll.write_text(
        """
import json
import pathlib
import sys

capture = next(
    argument.split("=", maxsplit=1)[1]
    for argument in sys.argv[1:]
    if argument.startswith("-Capture=")
)
pathlib.Path(capture).write_text(json.dumps(sys.argv[1:]), encoding="utf-8")
raise SystemExit(9)
""".lstrip(),
        encoding="utf-8",
    )
    return ubt_dll


class _DirectEnvironment(TypedDict):
    engine_root: Path
    ubt_user_settings_dir: Path


def _fake_direct_environment(tmp_path: Path) -> _DirectEnvironment:
    engine_root = tmp_path / "Fake UE Root"
    build_root = engine_root / "Engine" / "Build"
    build_root.mkdir(parents=True, exist_ok=True)
    build_version = build_root / "Build.version"
    if not build_version.exists():
        build_version.write_text("{}\n", encoding="utf-8")
    settings_root = tmp_path / "Fake UBT Settings"
    settings_root.mkdir(exist_ok=True)
    return {
        "engine_root": engine_root.resolve(),
        "ubt_user_settings_dir": settings_root.resolve(),
    }


def test_direct_ubt_runner_passes_auditable_argv_and_returns_exit_code(
    tmp_path: Path,
) -> None:
    spaced_root = tmp_path / "Direct UBT With Spaces"
    spaced_root.mkdir()
    capture_path = spaced_root / "arguments.json"
    ubt_dll = _write_fake_ubt_dll(spaced_root)
    uproject = spaced_root / "Robot Sim UE.uproject"
    uproject.write_text("{}\n", encoding="utf-8")
    log_path = spaced_root / "explicit ubt.log"

    returncode = run_unreal_build(
        ubt_dll=ubt_dll.resolve(),
        **_fake_direct_environment(tmp_path),
        dotnet=Path(sys.executable).resolve(),
        log_path=log_path.resolve(),
        uproject=uproject,
        timeout_s=5.0,
        extra_args=(f"-Capture={capture_path.resolve()}",),
    )

    arguments = json.loads(capture_path.read_text(encoding="utf-8"))
    assert returncode == 9
    assert arguments[:7] == [
        "RobotSimUEEditor",
        "Win64",
        "Development",
        f"-Project={uproject.resolve()}",
        "-WaitMutex",
        "-NoHotReloadFromIDE",
        f"-Log={log_path.resolve()}",
    ]
    assert len(arguments) == 8
    assert not any(argument.startswith("-Session=") for argument in arguments)
    assert not any(
        argument.startswith("-TelemetryProvider=") for argument in arguments
    )
    assert arguments[7] == f"-Capture={capture_path.resolve()}"


def test_direct_ubt_runner_pins_patched_engine_environment(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv(
        "LINGTU_UNREAL_CONTENT_ROOT_DIRECTORY", "C:\\ambient-root"
    )
    monkeypatch.setenv(
        "LINGTU_UNREAL_USER_SETTING_DIRECTORY", "C:\\ambient-settings"
    )
    monkeypatch.setenv("LINGTU_UNREAL_UNREVIEWED", "must-not-leak")
    monkeypatch.setenv("LINGTU_UBT_UNREVIEWED", "legacy-must-not-leak")
    capture_path = tmp_path / "environment.json"
    ubt_dll = tmp_path / "UnrealBuildTool.dll"
    ubt_dll.write_text(
        """
import json
import os
import pathlib
import sys

capture = next(
    argument.split("=", maxsplit=1)[1]
    for argument in sys.argv[1:]
    if argument.startswith("-Capture=")
)
payload = {
    "root": os.environ.get("LINGTU_UNREAL_CONTENT_ROOT_DIRECTORY"),
    "settings": os.environ.get("LINGTU_UNREAL_USER_SETTING_DIRECTORY"),
    "ambient": os.environ.get("LINGTU_UNREAL_UNREVIEWED"),
    "legacy_ambient": os.environ.get("LINGTU_UBT_UNREVIEWED"),
}
pathlib.Path(capture).write_text(json.dumps(payload), encoding="utf-8")
""".lstrip(),
        encoding="utf-8",
    )
    engine_root = tmp_path / "UE Root"
    build_root = engine_root / "Engine" / "Build"
    build_root.mkdir(parents=True)
    (build_root / "Build.version").write_text("{}\n", encoding="utf-8")
    settings_root = tmp_path / "UBT Settings"
    settings_root.mkdir()
    uproject = tmp_path / "RobotSimUE.uproject"
    uproject.write_text("{}\n", encoding="utf-8")

    returncode = run_unreal_build(
        ubt_dll=ubt_dll.resolve(),
        dotnet=Path(sys.executable).resolve(),
        log_path=(tmp_path / "ubt.log").resolve(),
        engine_root=engine_root.resolve(),
        ubt_user_settings_dir=settings_root.resolve(),
        uproject=uproject,
        timeout_s=5.0,
        extra_args=(f"-Capture={capture_path.resolve()}",),
    )

    payload = json.loads(capture_path.read_text(encoding="utf-8"))
    assert returncode == 0
    assert payload == {
        "root": str(engine_root.resolve()),
        "settings": str(settings_root.resolve()),
        "ambient": None,
        "legacy_ambient": None,
    }


@pytest.mark.parametrize(
    ("argument", "message"),
    [
        ("-NoMutex", "-WaitMutex is mandatory"),
        ("/nomutex", "-WaitMutex is mandatory"),
        ("--NOMUTEX=true", "-WaitMutex is mandatory"),
        ("-Project=C:\\foreign.uproject", "cannot override"),
        ("/LOG=C:\\foreign.log", "cannot override"),
        ("-RootDirectory=C:\\foreign-engine", "cannot override"),
        ("-Session={00000000-0000-0000-0000-000000000000}", "cannot override"),
        ("-WaitMutex=false", "cannot override"),
        ("--NoHotReloadFromIDE=false", "cannot override"),
    ],
)
def test_direct_ubt_runner_rejects_arguments_that_override_pinned_evidence(
    tmp_path: Path,
    argument: str,
    message: str,
) -> None:
    marker = tmp_path / "ubt-started.txt"
    ubt_dll = tmp_path / "UnrealBuildTool.dll"
    ubt_dll.write_text(
        (
            "import pathlib\n"
            f"pathlib.Path({str(marker)!r}).write_text('started', encoding='ascii')\n"
        ),
        encoding="utf-8",
    )
    uproject = tmp_path / "RobotSimUE.uproject"
    uproject.write_text("{}\n", encoding="utf-8")

    with pytest.raises(ValueError, match=message):
        run_unreal_build(
            ubt_dll=ubt_dll.resolve(),
            **_fake_direct_environment(tmp_path),
            dotnet=Path(sys.executable).resolve(),
            log_path=(tmp_path / "ubt.log").resolve(),
            uproject=uproject,
            timeout_s=5.0,
            extra_args=(argument,),
        )

    assert not marker.exists()


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("target", "-Project=C:\\foreign.uproject"),
        ("platform", "/Log=C:\\foreign.log"),
        ("configuration", "-WaitMutex=false"),
        ("target", "RobotSimUEEditor=Injected"),
        ("platform", "Win 64"),
        ("configuration", ""),
    ],
)
def test_runner_rejects_option_shaped_build_identity_fields(
    tmp_path: Path,
    field: str,
    value: str,
) -> None:
    marker = tmp_path / "ubt-started.txt"
    ubt_dll = tmp_path / "UnrealBuildTool.dll"
    ubt_dll.write_text(
        (
            "import pathlib\n"
            f"pathlib.Path({str(marker)!r}).write_text('started', encoding='ascii')\n"
        ),
        encoding="utf-8",
    )
    uproject = tmp_path / "RobotSimUE.uproject"
    uproject.write_text("{}\n", encoding="utf-8")
    target = value if field == "target" else "RobotSimUEEditor"
    platform = value if field == "platform" else "Win64"
    configuration = value if field == "configuration" else "Development"

    with pytest.raises(ValueError, match=r"must be an Unreal identity, not an option"):
        run_unreal_build(
            ubt_dll=ubt_dll.resolve(),
            **_fake_direct_environment(tmp_path),
            dotnet=Path(sys.executable).resolve(),
            log_path=(tmp_path / "ubt.log").resolve(),
            uproject=uproject,
            timeout_s=5.0,
            target=target,
            platform=platform,
            configuration=configuration,
        )

    assert not marker.exists()


@pytest.mark.skipif(os.name != "nt", reason="validates direct UBT Job Object ownership")
def test_direct_ubt_timeout_closes_the_owned_dotnet_process(tmp_path: Path) -> None:
    pid_path = tmp_path / "direct-dotnet.pid"
    ubt_dll = tmp_path / "UnrealBuildTool.dll"
    ubt_dll.write_text(
        """
import os
import pathlib
import sys
import time

pid_path = next(
    argument.split("=", maxsplit=1)[1]
    for argument in sys.argv[1:]
    if argument.startswith("-PidPath=")
)
pathlib.Path(pid_path).write_text(str(os.getpid()), encoding="ascii")
time.sleep(60)
""".lstrip(),
        encoding="utf-8",
    )
    uproject = tmp_path / "RobotSimUE.uproject"
    uproject.write_text("{}\n", encoding="utf-8")
    child_pid = -1

    try:
        with pytest.raises(UnrealBuildTimeoutError, match=r"timed out after 2 seconds"):
            run_unreal_build(
                ubt_dll=ubt_dll.resolve(),
                **_fake_direct_environment(tmp_path),
                dotnet=Path(sys.executable).resolve(),
                log_path=(tmp_path / "direct-ubt.log").resolve(),
                uproject=uproject,
                timeout_s=2.0,
                extra_args=(f"-PidPath={pid_path.resolve()}",),
            )
        child_pid = _wait_for_pid(pid_path, timeout_s=1.0)
        assert _wait_for_exit(child_pid), "direct UBT timeout left dotnet alive"
    finally:
        if child_pid > 0:
            _terminate_pid(child_pid)


def test_direct_ubt_runner_never_interprets_arguments_through_a_shell(
    tmp_path: Path,
) -> None:
    capture_path = tmp_path / "arguments.json"
    injected_marker = tmp_path / "shell-injection.txt"
    ubt_dll = _write_fake_ubt_dll(tmp_path)
    uproject = tmp_path / "RobotSimUE.uproject"
    uproject.write_text("{}\n", encoding="utf-8")
    shell_text = f"& echo injected > {injected_marker}"

    returncode = run_unreal_build(
        ubt_dll=ubt_dll.resolve(),
        **_fake_direct_environment(tmp_path),
        dotnet=Path(sys.executable).resolve(),
        log_path=(tmp_path / "ubt.log").resolve(),
        uproject=uproject,
        timeout_s=5.0,
        extra_args=(f"-Capture={capture_path.resolve()}", shell_text),
    )

    arguments = json.loads(capture_path.read_text(encoding="utf-8"))
    assert returncode == 9
    assert arguments[-1] == shell_text
    assert not injected_marker.exists()


@pytest.mark.parametrize(
    "field",
    [
        "ubt_dll",
        "dotnet",
        "log_path",
        "engine_root",
        "ubt_user_settings_dir",
    ],
)
def test_direct_ubt_runner_requires_explicit_absolute_tool_and_log_paths(
    tmp_path: Path,
    field: str,
) -> None:
    ubt_dll = _write_fake_ubt_dll(tmp_path)
    uproject = tmp_path / "RobotSimUE.uproject"
    uproject.write_text("{}\n", encoding="utf-8")
    request: dict[str, object] = {
        "ubt_dll": ubt_dll.resolve(),
        "dotnet": Path(sys.executable).resolve(),
        "log_path": (tmp_path / "ubt.log").resolve(),
        "uproject": uproject,
        "timeout_s": 5.0,
    }
    request.update(_fake_direct_environment(tmp_path))
    request[field] = Path(request[field]).name  # type: ignore[arg-type]

    with pytest.raises(ValueError, match=r"explicit normalized absolute path"):
        run_unreal_build(**request)  # type: ignore[arg-type]


def test_direct_ubt_runner_requires_existing_regular_tools_and_log_location(
    tmp_path: Path,
) -> None:
    uproject = tmp_path / "RobotSimUE.uproject"
    uproject.write_text("{}\n", encoding="utf-8")
    ubt_dll = _write_fake_ubt_dll(tmp_path)
    regular_dotnet = Path(sys.executable).resolve()
    regular_log = (tmp_path / "ubt.log").resolve()
    direct_environment = _fake_direct_environment(tmp_path)

    with pytest.raises(ValueError, match=r"UBT DLL is missing"):
        run_unreal_build(
            ubt_dll=(tmp_path / "missing.dll").resolve(),
            **direct_environment,
            dotnet=regular_dotnet,
            log_path=regular_log,
            uproject=uproject,
        )

    ubt_directory = tmp_path / "directory.dll"
    ubt_directory.mkdir()
    with pytest.raises(ValueError, match=r"UBT DLL is not a regular file"):
        run_unreal_build(
            ubt_dll=ubt_directory.resolve(),
            **direct_environment,
            dotnet=regular_dotnet,
            log_path=regular_log,
            uproject=uproject,
        )

    dotnet_directory = tmp_path / "dotnet-directory"
    dotnet_directory.mkdir()
    with pytest.raises(ValueError, match=r"dotnet is not a regular file"):
        run_unreal_build(
            ubt_dll=ubt_dll.resolve(),
            **direct_environment,
            dotnet=dotnet_directory.resolve(),
            log_path=regular_log,
            uproject=uproject,
        )

    missing_log_parent = tmp_path / "missing-logs" / "ubt.log"
    with pytest.raises(ValueError, match=r"UBT log parent is missing"):
        run_unreal_build(
            ubt_dll=ubt_dll.resolve(),
            **direct_environment,
            dotnet=regular_dotnet,
            log_path=missing_log_parent.absolute(),
            uproject=uproject,
        )

    log_directory = tmp_path / "directory.log"
    log_directory.mkdir()
    with pytest.raises(ValueError, match=r"UBT log path is not a regular file"):
        run_unreal_build(
            ubt_dll=ubt_dll.resolve(),
            **direct_environment,
            dotnet=regular_dotnet,
            log_path=log_directory.resolve(),
            uproject=uproject,
        )


@pytest.mark.parametrize(
    "omitted",
    ["dotnet", "log_path", "engine_root", "ubt_user_settings_dir"],
)
def test_direct_ubt_runner_requires_all_explicit_runtime_inputs(
    tmp_path: Path,
    omitted: str,
) -> None:
    ubt_dll = _write_fake_ubt_dll(tmp_path)
    uproject = tmp_path / "RobotSimUE.uproject"
    uproject.write_text("{}\n", encoding="utf-8")
    request: dict[str, object] = {
        "ubt_dll": ubt_dll.resolve(),
        "dotnet": Path(sys.executable).resolve(),
        "log_path": (tmp_path / "ubt.log").resolve(),
        "uproject": uproject,
    }
    request.update(_fake_direct_environment(tmp_path))
    request[omitted] = None

    with pytest.raises(ValueError, match=r"requires explicit dotnet and log_path"):
        run_unreal_build(**request)  # type: ignore[arg-type]


def test_direct_ubt_runner_rejects_linked_tool_file(tmp_path: Path) -> None:
    real_root = tmp_path / "real"
    real_root.mkdir()
    real_ubt = _write_fake_ubt_dll(real_root)
    linked_ubt = tmp_path / "UnrealBuildTool.dll"
    try:
        linked_ubt.symlink_to(real_ubt)
    except OSError as exc:
        pytest.skip(f"file symlink creation is unavailable: {exc}")
    uproject = tmp_path / "RobotSimUE.uproject"
    uproject.write_text("{}\n", encoding="utf-8")

    with pytest.raises(ValueError, match=r"link or reparse point"):
        run_unreal_build(
            ubt_dll=linked_ubt.absolute(),
            **_fake_direct_environment(tmp_path),
            dotnet=Path(sys.executable).resolve(),
            log_path=(tmp_path / "ubt.log").resolve(),
            uproject=uproject,
            timeout_s=5.0,
        )


def test_direct_ubt_runner_rejects_linked_log_parent(tmp_path: Path) -> None:
    real_log_root = tmp_path / "real-logs"
    real_log_root.mkdir()
    linked_log_root = tmp_path / "linked-logs"
    try:
        linked_log_root.symlink_to(real_log_root, target_is_directory=True)
    except OSError as exc:
        pytest.skip(f"directory symlink creation is unavailable: {exc}")
    ubt_dll = _write_fake_ubt_dll(tmp_path)
    uproject = tmp_path / "RobotSimUE.uproject"
    uproject.write_text("{}\n", encoding="utf-8")

    with pytest.raises(ValueError, match=r"link or reparse point"):
        run_unreal_build(
            ubt_dll=ubt_dll.resolve(),
            **_fake_direct_environment(tmp_path),
            dotnet=Path(sys.executable).resolve(),
            log_path=(linked_log_root.absolute() / "ubt.log"),
            uproject=uproject,
            timeout_s=5.0,
        )


def test_direct_ubt_cli_uses_explicit_tool_and_log_paths(tmp_path: Path) -> None:
    capture_path = tmp_path / "cli-arguments.json"
    ubt_dll = _write_fake_ubt_dll(tmp_path)
    uproject = tmp_path / "RobotSimUE.uproject"
    uproject.write_text("{}\n", encoding="utf-8")
    log_path = tmp_path / "cli-ubt.log"
    direct_environment = _fake_direct_environment(tmp_path)

    result = subprocess.run(
        [
            sys.executable,
            str(RUNNER),
            "--ubt-dll",
            str(ubt_dll.resolve()),
            "--dotnet",
            str(Path(sys.executable).resolve()),
            "--log",
            str(log_path.resolve()),
            "--engine-root",
            str(direct_environment["engine_root"]),
            "--ubt-user-settings-dir",
            str(direct_environment["ubt_user_settings_dir"]),
            "--uproject",
            str(uproject.resolve()),
            "--timeout-seconds",
            "5",
            "--",
            f"-Capture={capture_path.resolve()}",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 9
    assert json.loads(capture_path.read_text(encoding="utf-8"))[-1] == (
        f"-Capture={capture_path.resolve()}"
    )


def test_cli_rejects_build_bat_and_ubt_dll_together(tmp_path: Path) -> None:
    build_bat = tmp_path / "Build.bat"
    build_bat.write_text("@exit /b 0\r\n", encoding="ascii")
    ubt_dll = _write_fake_ubt_dll(tmp_path)
    uproject = tmp_path / "RobotSimUE.uproject"
    uproject.write_text("{}\n", encoding="utf-8")

    result = subprocess.run(
        [
            sys.executable,
            str(RUNNER),
            "--build-bat",
            str(build_bat),
            "--ubt-dll",
            str(ubt_dll.resolve()),
            "--uproject",
            str(uproject.resolve()),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 2
    assert "not allowed with argument" in result.stderr


@pytest.mark.skipif(os.name != "nt", reason="validates one lock across build modes")
def test_build_bat_and_direct_ubt_share_the_canonical_project_lock(
    tmp_path: Path,
) -> None:
    first_root = tmp_path / "batch"
    first_root.mkdir()
    first_pid_path = first_root / "child.pid"
    first_build = _write_fake_build(first_root, first_pid_path)
    uproject = tmp_path / "RobotSimUE.uproject"
    uproject.write_text("{}\n", encoding="utf-8")
    direct_marker = tmp_path / "direct-started.txt"
    ubt_dll = tmp_path / "UnrealBuildTool.dll"
    ubt_dll.write_text(
        (
            "import pathlib\n"
            f"pathlib.Path({str(direct_marker)!r}).write_text('started', encoding='ascii')\n"
        ),
        encoding="utf-8",
    )
    first_runner = subprocess.Popen(
        [
            sys.executable,
            str(RUNNER),
            "--build-bat",
            str(first_build),
            "--uproject",
            str(uproject),
            "--timeout-seconds",
            "20",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    first_child_pid = -1
    try:
        first_child_pid = _wait_for_pid(first_pid_path)
        with pytest.raises(UnrealBuildLockTimeoutError, match=r"lock|waiting"):
            run_unreal_build(
                ubt_dll=ubt_dll.resolve(),
                **_fake_direct_environment(tmp_path),
                dotnet=Path(sys.executable).resolve(),
                log_path=(tmp_path / "direct.log").resolve(),
                uproject=uproject,
                timeout_s=0.4,
            )
        assert not direct_marker.exists()

        first_runner.terminate()
        first_runner.wait(timeout=3.0)
        assert _wait_for_exit(first_child_pid)

        assert (
            run_unreal_build(
                ubt_dll=ubt_dll.resolve(),
                **_fake_direct_environment(tmp_path),
                dotnet=Path(sys.executable).resolve(),
                log_path=(tmp_path / "direct.log").resolve(),
                uproject=uproject,
                timeout_s=5.0,
            )
            == 0
        )
        assert direct_marker.is_file()
    finally:
        if first_runner.poll() is None:
            first_runner.kill()
            first_runner.wait(timeout=3.0)
        if first_child_pid > 0:
            _terminate_pid(first_child_pid)


def test_runner_preserves_wait_mutex_and_returns_build_bat_exit_code(
    tmp_path: Path,
) -> None:
    spaced_root = tmp_path / "UE Root With Spaces"
    spaced_root.mkdir()
    arguments_path = spaced_root / "arguments.txt"
    build_bat = spaced_root / "Build.bat"
    build_bat.write_text(
        f"@echo off\r\n@echo %* > \"{arguments_path}\"\r\n@exit /b 7\r\n",
        encoding="ascii",
    )
    uproject = spaced_root / "Robot Sim UE.uproject"
    uproject.write_text("{}\n", encoding="utf-8")

    returncode = run_unreal_build(
        build_bat=build_bat,
        uproject=uproject,
        timeout_s=5.0,
    )

    arguments = arguments_path.read_text(encoding="ascii")
    assert returncode == 7
    assert '"RobotSimUEEditor" "Win64" "Development"' in arguments
    assert str(uproject.resolve()) in arguments
    assert "-WaitMutex" in arguments
    assert "-NoHotReloadFromIDE" in arguments
    assert "-NoMutex" not in arguments


@pytest.mark.parametrize("argument", ["-NoMutex", "/nomutex", "--NOMUTEX=true"])
def test_runner_rejects_every_no_mutex_spelling_before_starting_build(
    tmp_path: Path,
    argument: str,
) -> None:
    marker = tmp_path / "build-started.txt"
    build_bat = tmp_path / "Build.bat"
    build_bat.write_text(
        f'@echo started>"{marker}"\r\n',
        encoding="ascii",
    )
    uproject = tmp_path / "RobotSimUE.uproject"
    uproject.write_text("{}\n", encoding="utf-8")

    with pytest.raises(ValueError, match=r"forbidden; -WaitMutex is mandatory"):
        run_unreal_build(
            build_bat=build_bat,
            uproject=uproject,
            timeout_s=5.0,
            extra_args=(argument,),
        )

    assert not marker.exists()


@pytest.mark.parametrize(
    "argument",
    ["-WaitMutex=false", "--NoHotReloadFromIDE=false"],
)
def test_build_bat_runner_rejects_overrides_of_mandatory_arguments(
    tmp_path: Path,
    argument: str,
) -> None:
    marker = tmp_path / "build-started.txt"
    build_bat = tmp_path / "Build.bat"
    build_bat.write_text(f'@echo started>"{marker}"\r\n', encoding="ascii")
    uproject = tmp_path / "RobotSimUE.uproject"
    uproject.write_text("{}\n", encoding="utf-8")

    with pytest.raises(ValueError, match=r"cannot override"):
        run_unreal_build(
            build_bat=build_bat,
            uproject=uproject,
            timeout_s=5.0,
            extra_args=(argument,),
        )

    assert not marker.exists()


def test_project_lock_is_repository_local_and_stable_for_the_canonical_path(
    tmp_path: Path,
) -> None:
    uproject = tmp_path / "RobotSimUE.uproject"
    uproject.write_text("{}\n", encoding="utf-8")

    lock_path = project_lock_path(uproject)

    assert lock_path.parent == REPO_ROOT / "build" / "locks" / "ue-build"
    assert lock_path.suffix == ".lock"
    assert lock_path == project_lock_path(uproject.resolve())


def test_runner_cli_rejects_no_mutex_without_starting_build(tmp_path: Path) -> None:
    marker = tmp_path / "build-started.txt"
    build_bat = tmp_path / "Build.bat"
    build_bat.write_text(f'@echo started>"{marker}"\r\n', encoding="ascii")
    uproject = tmp_path / "RobotSimUE.uproject"
    uproject.write_text("{}\n", encoding="utf-8")

    result = subprocess.run(
        [
            sys.executable,
            str(RUNNER),
            "--build-bat",
            str(build_bat),
            "--uproject",
            str(uproject),
            "--",
            "-NoMutex",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 2
    assert "-WaitMutex is mandatory" in result.stderr
    assert not marker.exists()


@pytest.mark.skipif(os.name != "nt", reason="validates Windows Job Object inheritance")
def test_runner_timeout_closes_the_complete_build_tree(tmp_path: Path) -> None:
    pid_path = tmp_path / "timed-out-child.pid"
    build_bat = _write_fake_build(tmp_path, pid_path)
    uproject = tmp_path / "RobotSimUE.uproject"
    uproject.write_text("{}\n", encoding="utf-8")
    child_pid = -1

    try:
        with pytest.raises(UnrealBuildTimeoutError, match=r"timed out after 2 seconds"):
            run_unreal_build(
                build_bat=build_bat,
                uproject=uproject,
                timeout_s=2.0,
            )
        child_pid = _wait_for_pid(pid_path, timeout_s=1.0)
        assert _wait_for_exit(child_pid), "build timeout left the fake UBT child alive"
    finally:
        if child_pid > 0:
            _terminate_pid(child_pid)


@pytest.mark.skipif(os.name != "nt", reason="validates Windows Job Object inheritance")
def test_runner_keyboard_interrupt_closes_the_complete_build_tree(
    tmp_path: Path,
) -> None:
    pid_path = tmp_path / "interrupted-child.pid"
    build_bat = _write_fake_build(tmp_path, pid_path)
    uproject = tmp_path / "RobotSimUE.uproject"
    uproject.write_text("{}\n", encoding="utf-8")
    interrupt = threading.Timer(1.0, interrupt_main)
    child_pid = -1

    try:
        interrupt.start()
        with pytest.raises(KeyboardInterrupt):
            run_unreal_build(
                build_bat=build_bat,
                uproject=uproject,
                timeout_s=3.0,
            )
        child_pid = _wait_for_pid(pid_path, timeout_s=1.0)
        assert _wait_for_exit(child_pid), "Ctrl+C left the fake UBT child alive"
    finally:
        interrupt.cancel()
        interrupt.join(timeout=3.0)
        if child_pid > 0:
            _terminate_pid(child_pid)


@pytest.mark.skipif(os.name != "nt", reason="validates Windows Job Object inheritance")
def test_runner_normal_exit_closes_background_descendants(tmp_path: Path) -> None:
    pid_path = tmp_path / "background-child.pid"
    build_bat = _write_backgrounding_fake_build(tmp_path, pid_path)
    uproject = tmp_path / "RobotSimUE.uproject"
    uproject.write_text("{}\n", encoding="utf-8")
    child_pid = -1

    try:
        assert (
            run_unreal_build(
                build_bat=build_bat,
                uproject=uproject,
                timeout_s=8.0,
            )
            == 0
        )
        child_pid = _wait_for_pid(pid_path, timeout_s=1.0)
        assert _wait_for_exit(child_pid), (
            "successful Build.bat exit left its background descendant alive"
        )
    finally:
        if child_pid > 0:
            _terminate_pid(child_pid)


@pytest.mark.skipif(os.name != "nt", reason="validates Windows cross-process ownership")
def test_canonical_uproject_lock_serializes_independent_runner_processes(
    tmp_path: Path,
) -> None:
    first_root = tmp_path / "first"
    first_root.mkdir()
    first_pid_path = first_root / "child.pid"
    first_build = _write_fake_build(first_root, first_pid_path)
    uproject = tmp_path / "RobotSimUE.uproject"
    uproject.write_text("{}\n", encoding="utf-8")
    second_marker = tmp_path / "second-build-started.txt"
    second_build = tmp_path / "SecondBuild.bat"
    second_build.write_text(
        f'@echo started>"{second_marker}"\r\n@exit /b 0\r\n',
        encoding="ascii",
    )

    first_runner = subprocess.Popen(
        [
            sys.executable,
            str(RUNNER),
            "--build-bat",
            str(first_build),
            "--uproject",
            str(uproject),
            "--timeout-seconds",
            "20",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    first_child_pid = -1
    try:
        first_child_pid = _wait_for_pid(first_pid_path)
        with pytest.raises(UnrealBuildLockTimeoutError, match=r"same|lock|waiting"):
            run_unreal_build(
                build_bat=second_build,
                uproject=uproject,
                timeout_s=0.4,
            )
        assert not second_marker.exists(), (
            "a second process entered Build.bat while the canonical project was locked"
        )

        first_runner.terminate()
        first_runner.wait(timeout=3.0)
        assert _wait_for_exit(first_child_pid)

        assert (
            run_unreal_build(
                build_bat=second_build,
                uproject=uproject,
                timeout_s=5.0,
            )
            == 0
        )
        assert second_marker.is_file()
    finally:
        if first_runner.poll() is None:
            first_runner.kill()
            first_runner.wait(timeout=3.0)
        if first_child_pid > 0:
            _terminate_pid(first_child_pid)


@pytest.mark.skipif(os.name != "nt", reason="validates Windows Job Object inheritance")
def test_owned_runner_prevents_the_orphan_left_by_an_interrupted_direct_build(
    tmp_path: Path,
) -> None:
    direct_pid_path = tmp_path / "direct-child.pid"
    direct_root = tmp_path / "direct"
    direct_root.mkdir()
    direct_build = _write_fake_build(direct_root, direct_pid_path)

    direct = subprocess.Popen(
        ["cmd.exe", "/d", "/s", "/c", str(direct_build)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    direct_child_pid = -1
    try:
        direct_child_pid = _wait_for_pid(direct_pid_path)
        direct.terminate()
        direct.wait(timeout=3.0)
        assert not _process_has_exited(direct_child_pid), (
            "the control launch unexpectedly owned its descendant; the regression "
            "test no longer reproduces the old PowerShell '& Build.bat' leak"
        )
    finally:
        if direct.poll() is None:
            direct.kill()
            direct.wait(timeout=3.0)
        if direct_child_pid > 0:
            _terminate_pid(direct_child_pid)

    owned_root = tmp_path / "owned"
    owned_root.mkdir()
    owned_pid_path = owned_root / "owned-child.pid"
    owned_build = _write_fake_build(owned_root, owned_pid_path)
    uproject = owned_root / "RobotSimUE.uproject"
    uproject.write_text("{}\n", encoding="utf-8")

    runner = subprocess.Popen(
        [
            sys.executable,
            str(RUNNER),
            "--build-bat",
            str(owned_build),
            "--uproject",
            str(uproject),
            "--timeout-seconds",
            "20",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    owned_child_pid = -1
    try:
        owned_child_pid = _wait_for_pid(owned_pid_path)
        runner.terminate()
        runner.wait(timeout=3.0)
        assert _wait_for_exit(owned_child_pid), (
            "terminating the build runner left its Build.bat descendant alive"
        )
    finally:
        if runner.poll() is None:
            runner.kill()
            runner.wait(timeout=3.0)
        if owned_child_pid > 0:
            _terminate_pid(owned_child_pid)

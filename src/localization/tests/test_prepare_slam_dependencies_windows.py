from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

import pytest

pytestmark = pytest.mark.skipif(os.name != "nt", reason="Windows-only dependency preparation")

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "build" / "prepare_slam_dependencies_windows.ps1"
POWERSHELL = shutil.which("pwsh")


def _run_script(*arguments: str, env: dict[str, str] | None = None) -> subprocess.CompletedProcess[str]:
    assert POWERSHELL is not None
    return subprocess.run(  # noqa: S603 - executes the repository-owned helper.
        [POWERSHELL, "-NoProfile", "-File", str(SCRIPT), *arguments],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
        env=env,
    )


def test_dependency_preflight_reports_pinned_plan_without_writes(tmp_path: Path) -> None:
    vcpkg_root = tmp_path / "third_party" / "toolchains" / "vcpkg"
    install_root = tmp_path / "third_party" / "install" / "slam-windows"
    binary_cache = tmp_path / "third_party" / "cache" / "vcpkg"

    completed = _run_script(
        "-VcpkgRoot",
        str(vcpkg_root),
        "-InstallRoot",
        str(install_root),
        "-BinaryCache",
        str(binary_cache),
        "-PreflightOnly",
    )

    assert completed.returncode == 0, completed.stdout + completed.stderr
    assert "9e593bb18ea69cc5095e012465dcd675a822ed0d" in completed.stdout
    assert "x64-windows" in completed.stdout
    assert not vcpkg_root.exists()
    assert not install_root.exists()
    assert not binary_cache.exists()


def test_dependency_preflight_rejects_ambiguous_binary_cache_path(tmp_path: Path) -> None:
    completed = _run_script(
        "-VcpkgRoot",
        str(tmp_path / "vcpkg"),
        "-InstallRoot",
        str(tmp_path / "installed"),
        "-BinaryCache",
        str(tmp_path / "cache,other"),
        "-PreflightOnly",
    )

    assert completed.returncode != 0
    assert "comma or semicolon" in (completed.stdout + completed.stderr)
    assert not (tmp_path / "vcpkg").exists()


def test_dependency_preflight_rejects_overlapping_managed_roots_without_writes(tmp_path: Path) -> None:
    vcpkg_root = tmp_path / "managed" / "vcpkg"
    completed = _run_script(
        "-VcpkgRoot",
        str(vcpkg_root),
        "-InstallRoot",
        str(vcpkg_root / "installed"),
        "-BinaryCache",
        str(tmp_path / "cache"),
        "-PreflightOnly",
    )

    assert completed.returncode != 0
    assert "must not overlap" in (completed.stdout + completed.stderr)
    assert not (tmp_path / "managed").exists()


def test_dependency_preflight_detects_overlap_through_directory_junction(tmp_path: Path) -> None:
    actual_root = tmp_path / "actual"
    actual_root.mkdir()
    junction = tmp_path / "junction"
    subprocess.run(  # noqa: S603 - creates a disposable local junction fixture.
        [os.environ["ComSpec"], "/d", "/c", "mklink", "/J", str(junction), str(actual_root)],
        check=True,
        capture_output=True,
        text=True,
    )

    completed = _run_script(
        "-VcpkgRoot",
        str(junction / "vcpkg"),
        "-InstallRoot",
        str(actual_root / "vcpkg" / "installed"),
        "-BinaryCache",
        str(tmp_path / "cache"),
        "-PreflightOnly",
    )

    assert completed.returncode != 0
    assert "must not overlap" in (completed.stdout + completed.stderr)
    assert not (actual_root / "vcpkg").exists()


def test_dependency_prepare_checks_out_and_installs_exact_target_and_host(
    tmp_path: Path,
) -> None:
    baseline = "9e593bb18ea69cc5095e012465dcd675a822ed0d"
    vcpkg_root = tmp_path / "third_party" / "toolchains" / "vcpkg"
    install_root = tmp_path / "third_party" / "install" / "slam-windows"
    binary_cache = tmp_path / "third_party" / "cache" / "vcpkg"
    (vcpkg_root / ".git").mkdir(parents=True)
    command_log = tmp_path / "commands.txt"
    fake_bin = tmp_path / "fake-bin"
    fake_bin.mkdir()
    (fake_bin / "git.cmd").write_text(
        '@echo git %*>> "%LINGTU_COMMAND_LOG%"\n'
        f'@echo %* | findstr /C:"rev-parse HEAD" >nul && echo {baseline}\n'
        '@echo %* | findstr /C:"remote get-url origin" >nul && echo https://github.com/microsoft/vcpkg.git\n'
        '@echo %* | findstr /C:"symbolic-ref" >nul && exit /b 1\n'
        "@exit /b 0\n",
        encoding="utf-8",
    )
    (vcpkg_root / "bootstrap-vcpkg.bat").write_text(
        '@echo bootstrap-vcpkg.bat %*>> "%LINGTU_COMMAND_LOG%"\n@exit /b 0\n',
        encoding="utf-8",
    )
    shim_source = tmp_path / "vcpkg-shim.cs"
    shim_source.write_text(
        "using System; using System.IO; class Shim { static int Main(string[] a) { "
        'File.AppendAllText(Environment.GetEnvironmentVariable("LINGTU_COMMAND_LOG"), '
        '"vcpkg.exe " + String.Join(" ", a) + " binary=" + '
        'Environment.GetEnvironmentVariable("VCPKG_BINARY_SOURCES") + " target=" + '
        'Environment.GetEnvironmentVariable("VCPKG_DEFAULT_TRIPLET") + " host=" + '
        'Environment.GetEnvironmentVariable("VCPKG_DEFAULT_HOST_TRIPLET") + "\\n"); '
        "return 0; } }",
        encoding="utf-8",
    )
    csc = Path("C:/Windows/Microsoft.NET/Framework64/v4.0.30319/csc.exe")
    subprocess.run(  # noqa: S603 - compiles a local deterministic test shim.
        [str(csc), "/nologo", f"/out:{vcpkg_root / 'vcpkg.exe'}", str(shim_source)],
        check=True,
        capture_output=True,
        text=True,
    )
    environment = os.environ.copy()
    environment["PATH"] = f"{fake_bin};{environment['PATH']}"
    environment["LINGTU_COMMAND_LOG"] = str(command_log)

    completed = _run_script(
        "-VcpkgRoot",
        str(vcpkg_root),
        "-InstallRoot",
        str(install_root),
        "-BinaryCache",
        str(binary_cache),
        env=environment,
    )

    assert completed.returncode == 0, completed.stdout + completed.stderr
    commands = command_log.read_text(encoding="utf-8")
    assert "fetch" not in commands
    assert "checkout" not in commands
    assert "bootstrap-vcpkg.bat -disableMetrics" in commands
    assert "--triplet=x64-windows --host-triplet=x64-windows" in commands
    assert f"files,{binary_cache},readwrite" in commands
    assert "target=x64-windows host=x64-windows" in commands


def test_dependency_prepare_rejects_dirty_existing_checkout_before_network_or_bootstrap(
    tmp_path: Path,
) -> None:
    baseline = "9e593bb18ea69cc5095e012465dcd675a822ed0d"
    vcpkg_root = tmp_path / "vcpkg"
    (vcpkg_root / ".git").mkdir(parents=True)
    (vcpkg_root / "bootstrap-vcpkg.bat").write_text("@echo bootstrap>> %LINGTU_COMMAND_LOG%\n", encoding="utf-8")
    command_log = tmp_path / "commands.txt"
    fake_bin = tmp_path / "fake-bin"
    fake_bin.mkdir()
    (fake_bin / "git.cmd").write_text(
        '@echo git %*>> "%LINGTU_COMMAND_LOG%"\n'
        f'@echo %* | findstr /C:"rev-parse HEAD" >nul && echo {baseline}\n'
        '@echo %* | findstr /C:"remote get-url origin" >nul && echo https://github.com/microsoft/vcpkg.git\n'
        '@echo %* | findstr /C:"symbolic-ref" >nul && exit /b 1\n'
        '@echo %* | findstr /C:"status --porcelain" >nul && echo ?? dirty.txt\n'
        "@exit /b 0\n",
        encoding="utf-8",
    )
    environment = os.environ.copy()
    environment["PATH"] = f"{fake_bin};{environment['PATH']}"
    environment["LINGTU_COMMAND_LOG"] = str(command_log)

    completed = _run_script(
        "-VcpkgRoot",
        str(vcpkg_root),
        "-InstallRoot",
        str(tmp_path / "installed"),
        "-BinaryCache",
        str(tmp_path / "cache"),
        env=environment,
    )

    assert completed.returncode != 0
    commands = command_log.read_text(encoding="utf-8")
    assert "must be clean" in (completed.stdout + completed.stderr)
    assert " fetch " not in commands
    assert " checkout " not in commands
    assert "bootstrap" not in commands
    assert "vcpkg.exe" not in commands


def test_dependency_prepare_clone_failure_does_not_block_retry_with_stale_quarantine(
    tmp_path: Path,
) -> None:
    vcpkg_root = tmp_path / "vcpkg"
    stale_quarantine = Path(f"{vcpkg_root}.incoming")
    stale_quarantine.mkdir(parents=True)
    command_log = tmp_path / "commands.txt"
    fake_bin = tmp_path / "fake-bin"
    fake_bin.mkdir()
    (fake_bin / "git.cmd").write_text(
        '@echo git %*>> "%LINGTU_COMMAND_LOG%"\n'
        '@echo %* | findstr /B /C:"clone " >nul && exit /b 9\n'
        "@exit /b 0\n",
        encoding="utf-8",
    )
    environment = os.environ.copy()
    environment["PATH"] = f"{fake_bin};{environment['PATH']}"
    environment["LINGTU_COMMAND_LOG"] = str(command_log)

    for _attempt in range(2):
        completed = _run_script(
            "-VcpkgRoot",
            str(vcpkg_root),
            "-InstallRoot",
            str(tmp_path / "installed"),
            "-BinaryCache",
            str(tmp_path / "cache"),
            env=environment,
        )
        assert completed.returncode != 0
        assert "vcpkg clone failed" in (completed.stdout + completed.stderr)

    clone_lines = [
        line for line in command_log.read_text(encoding="utf-8").splitlines() if "git clone " in line
    ]
    assert len(clone_lines) == 2
    assert clone_lines[0] != clone_lines[1]
    assert all(f"{vcpkg_root}.incoming." in line for line in clone_lines)


@pytest.mark.parametrize("failure_stage", ["clone", "fetch", "checkout"])
def test_dependency_prepare_cleans_only_its_partial_quarantine(
    tmp_path: Path, failure_stage: str,
) -> None:
    vcpkg_root = tmp_path / "vcpkg"
    command_log = tmp_path / "commands.txt"
    fake_bin = tmp_path / "fake-bin"
    fake_bin.mkdir()
    (fake_bin / "git.cmd").write_text(
        '@echo git %*>> "%LINGTU_COMMAND_LOG%"\n'
        "@set last=\n"
        "@for %%A in (%*) do @set last=%%~A\n"
        '@echo %* | findstr /B /C:"clone " >nul || goto after_clone\n'
        '@mkdir "%last%\\.git"\n'
        '@if "%LINGTU_FAIL_STAGE%"=="clone" exit /b 9\n'
        ":after_clone\n"
        '@echo %* | findstr /C:" fetch " >nul && if "%LINGTU_FAIL_STAGE%"=="fetch" exit /b 9\n'
        '@echo %* | findstr /C:" checkout " >nul && if "%LINGTU_FAIL_STAGE%"=="checkout" exit /b 9\n'
        "@exit /b 0\n",
        encoding="utf-8",
    )
    environment = os.environ.copy()
    environment["PATH"] = f"{fake_bin};{environment['PATH']}"
    environment["LINGTU_COMMAND_LOG"] = str(command_log)
    environment["LINGTU_FAIL_STAGE"] = failure_stage

    completed = _run_script(
        "-VcpkgRoot",
        str(vcpkg_root),
        "-InstallRoot",
        str(tmp_path / "installed"),
        "-BinaryCache",
        str(tmp_path / "cache"),
        env=environment,
    )

    assert completed.returncode != 0
    assert failure_stage in (completed.stdout + completed.stderr)
    assert not list(tmp_path.glob("vcpkg.incoming.*"))
    assert not vcpkg_root.exists()

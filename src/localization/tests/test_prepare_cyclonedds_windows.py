from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

import pytest

pytestmark = pytest.mark.skipif(os.name != "nt", reason="Windows-only CycloneDDS preparation")
ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "build" / "prepare_cyclonedds_windows.ps1"
POWERSHELL = shutil.which("pwsh")
RUN_REAL_INTEGRATION = os.environ.get("LINGTU_RUN_CYCLONEDDS_WINDOWS_INTEGRATION") == "1"


def _run(*arguments: str, env: dict[str, str] | None = None) -> subprocess.CompletedProcess[str]:
    assert POWERSHELL is not None
    return subprocess.run(  # noqa: S603 - runs the repository-owned preparation entry point.
        [POWERSHELL, "-NoProfile", "-File", str(SCRIPT), *arguments],
        cwd=ROOT,
        capture_output=True,
        check=False,
        text=True,
        env=env,
    )


def test_preflight_reports_pinned_windows_build_without_writing(tmp_path: Path) -> None:
    source = tmp_path / "source"
    build = tmp_path / "build"
    install = tmp_path / "install"
    sdk = tmp_path / "sdk"

    completed = _run(
        "-SourceRoot", str(source), "-BuildRoot", str(build), "-InstallRoot", str(install),
        "-SdkRoot", str(sdk), "-PreflightOnly",
    )

    assert completed.returncode == 0, completed.stdout + completed.stderr
    assert "e54e991f75a3e67f8e628da3171122e36ea5b872" in completed.stdout
    assert "56508d35826c362782fc8a388cad351a3d491f51" in completed.stdout
    assert "Visual Studio 17 2022, v143, x64, Release, /MD" in completed.stdout
    assert "shared core, security disabled, idlc enabled" in completed.stdout
    assert not any(path.exists() for path in (source, build, install, sdk))


def test_configure_disables_system_runtime_install_and_rejects_cache_drift() -> None:
    script = SCRIPT.read_text(encoding="utf-8")

    assert '"-DCMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_SKIP=TRUE"' in script
    assert 'Assert-CMakeCacheValue "CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_SKIP" "TRUE"' in script


def test_dirty_existing_checkout_is_rejected_before_fetch_or_checkout(tmp_path: Path) -> None:
    source = tmp_path / "source"
    (source / ".git").mkdir(parents=True)
    build = tmp_path / "build"
    install = tmp_path / "install"
    sdk = tmp_path / "sdk"
    fake_bin = tmp_path / "fake-bin"
    fake_bin.mkdir()
    command_log = tmp_path / "git-commands.txt"
    (fake_bin / "git.cmd").write_text(
        '@echo %*>> "%LINGTU_GIT_LOG%"\n'
        '@echo %* | findstr /C:"config --local --get core.autocrlf" >nul && (echo false& exit /b 0)\n'
        '@echo %* | findstr /C:"config --local --get core.eol" >nul && (echo lf& exit /b 0)\n'
        '@echo %* | findstr /C:"config --local --get core.safecrlf" >nul && (echo true& exit /b 0)\n'
        '@echo %* | findstr /C:"remote get-url origin" >nul && '
        "(echo https://github.com/eclipse-cyclonedds/cyclonedds.git& exit /b 0)\n"
        '@echo %* | findstr /C:"status --porcelain --untracked-files=all" >nul && (echo ?? dirty.txt& exit /b 0)\n'
        "@exit /b 0\n",
        encoding="utf-8",
    )
    environment = os.environ.copy()
    environment["PATH"] = f"{fake_bin};{environment['PATH']}"
    environment["LINGTU_GIT_LOG"] = str(command_log)

    completed = _run(
        "-SourceRoot",
        str(source),
        "-BuildRoot",
        str(build),
        "-InstallRoot",
        str(install),
        "-SdkRoot",
        str(sdk),
        env=environment,
    )

    assert completed.returncode != 0
    assert "clean before any mutation" in completed.stdout + completed.stderr
    commands = command_log.read_text(encoding="utf-8")
    assert "remote get-url origin" in commands
    assert "status --porcelain --untracked-files=all" in commands
    assert " fetch " not in f" {commands} "
    assert " checkout " not in f" {commands} "
    assert not build.exists()
    assert not install.exists()
    assert not sdk.exists()


def test_existing_checkout_with_global_or_local_autocrlf_is_rejected_read_only(tmp_path: Path) -> None:
    source = tmp_path / "source"
    (source / ".git").mkdir(parents=True)
    build = tmp_path / "build"
    install = tmp_path / "install"
    sdk = tmp_path / "sdk"
    fake_bin = tmp_path / "fake-bin"
    fake_bin.mkdir()
    command_log = tmp_path / "git-commands.txt"
    (fake_bin / "git.cmd").write_text(
        '@echo %*>> "%LINGTU_GIT_LOG%"\n'
        '@echo %* | findstr /C:"config --local --get core.autocrlf" >nul && (echo true& exit /b 0)\n'
        "@exit /b 0\n",
        encoding="utf-8",
    )
    environment = os.environ.copy()
    environment["PATH"] = f"{fake_bin};{environment['PATH']}"
    environment["LINGTU_GIT_LOG"] = str(command_log)

    completed = _run(
        "-SourceRoot",
        str(source),
        "-BuildRoot",
        str(build),
        "-InstallRoot",
        str(install),
        "-SdkRoot",
        str(sdk),
        env=environment,
    )

    assert completed.returncode != 0
    assert "checkout EOL policy" in completed.stdout + completed.stderr
    commands = command_log.read_text(encoding="utf-8")
    assert "config --local --get core.autocrlf" in commands
    assert "remote get-url" not in commands
    assert " fetch " not in f" {commands} "
    assert " checkout " not in f" {commands} "
    assert not build.exists()
    assert not install.exists()
    assert not sdk.exists()


def test_preflight_rejects_ancestor_descendant_path_overlap_without_writes(tmp_path: Path) -> None:
    source = tmp_path / "source"
    build = source / "nested-build"
    install = tmp_path / "install"
    sdk = tmp_path / "sdk"

    completed = _run(
        "-SourceRoot",
        str(source),
        "-BuildRoot",
        str(build),
        "-InstallRoot",
        str(install),
        "-SdkRoot",
        str(sdk),
        "-PreflightOnly",
    )

    assert completed.returncode != 0
    assert "distinct and non-overlapping" in completed.stdout + completed.stderr
    assert not any(path.exists() for path in (source, build, install, sdk))


def test_preflight_resolves_junction_aliases_before_overlap_check(tmp_path: Path) -> None:
    assert POWERSHELL is not None
    physical = tmp_path / "physical-source"
    physical.mkdir()
    junction = tmp_path / "source-junction"
    environment = os.environ.copy()
    environment["LINGTU_TEST_JUNCTION"] = str(junction)
    environment["LINGTU_TEST_JUNCTION_TARGET"] = str(physical)
    created = subprocess.run(  # noqa: S603 - creates a test-only junction inside pytest tmp_path.
        [
            POWERSHELL,
            "-NoProfile",
            "-Command",
            "New-Item -ItemType Junction -Path $env:LINGTU_TEST_JUNCTION "
            "-Target $env:LINGTU_TEST_JUNCTION_TARGET | Out-Null",
        ],
        cwd=ROOT,
        capture_output=True,
        check=False,
        text=True,
        env=environment,
    )
    assert created.returncode == 0, created.stdout + created.stderr
    install = tmp_path / "install"
    sdk = tmp_path / "sdk"
    try:
        completed = _run(
            "-SourceRoot",
            str(junction),
            "-BuildRoot",
            str(physical),
            "-InstallRoot",
            str(install),
            "-SdkRoot",
            str(sdk),
            "-PreflightOnly",
        )
    finally:
        junction.rmdir()

    assert completed.returncode != 0
    assert "distinct and non-overlapping" in completed.stdout + completed.stderr
    assert not install.exists()
    assert not sdk.exists()


@pytest.mark.skipif(
    not RUN_REAL_INTEGRATION,
    reason="set LINGTU_RUN_CYCLONEDDS_WINDOWS_INTEGRATION=1 for the real VS2022 SDK build",
)
def test_real_windows_sdk_build_verify_and_publish_contract() -> None:
    integration_root_value = os.environ.get("LINGTU_CYCLONEDDS_WINDOWS_INTEGRATION_ROOT", "")
    integration_root = Path(integration_root_value)
    assert integration_root_value and integration_root.is_absolute()
    source = integration_root / "source"
    build = integration_root / "build"
    install = integration_root / "install"
    sdk = integration_root / "sdk"

    completed = _run(
        "-SourceRoot",
        str(source),
        "-BuildRoot",
        str(build),
        "-InstallRoot",
        str(install),
        "-SdkRoot",
        str(sdk),
    )

    assert completed.returncode == 0, completed.stdout + completed.stderr
    assert (sdk / "evidence/sdk-receipt.json").is_file()
    assert not Path(f"{sdk}.incoming").exists()
    assert POWERSHELL is not None
    verified = subprocess.run(  # noqa: S603 - runs the repository-owned published SDK verifier.
        [POWERSHELL, "-NoProfile", "-File", str(ROOT / "scripts/build/verify_cyclonedds_windows_sdk.ps1"),
         "-SdkRoot", str(sdk)],
        cwd=ROOT,
        capture_output=True,
        check=False,
        text=True,
    )
    assert verified.returncode == 0, verified.stdout + verified.stderr

from __future__ import annotations

import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
MODULE = ROOT / "scripts" / "deploy" / "windows" / "windows_vc_redist.psm1"
SCRIPT = ROOT / "scripts" / "deploy" / "windows" / "ensure_windows_vc_redist.ps1"
POWERSHELL = shutil.which("pwsh")
WINDOWS_POWERSHELL = shutil.which("powershell")


def _pwsh() -> str:
    if POWERSHELL is None:
        pytest.skip("pwsh is unavailable")
    return POWERSHELL


def _invoke_bootstrap(
    tmp_path: Path,
    registrations: list[dict[str, object]],
    *,
    minimum_version: str = "14.44.35207.0",
    check_only: bool = False,
    installer_exit_code: int = 0,
    quiet: bool = False,
    log_path: Path | None = None,
    use_default_installer_invoker: bool = False,
) -> tuple[subprocess.CompletedProcess[str], dict[str, object] | None, Path]:
    powershell = _pwsh()
    registrations_path = tmp_path / "registrations.json"
    registrations_path.write_text(json.dumps(registrations), encoding="utf-8")
    invocation_path = tmp_path / "installer-invocation.json"
    installer_path = tmp_path / "vc_redist.x64.exe"
    installer_path.write_bytes(b"test fixture")
    harness_path = tmp_path / "invoke-bootstrap.ps1"
    harness_path.write_text(
        r"""
$ErrorActionPreference = "Stop"
Import-Module -Force $env:LINGTU_TEST_MODULE
$registrations = @(Get-Content -LiteralPath $env:LINGTU_TEST_REGISTRATIONS -Raw | ConvertFrom-Json)
$registrationQueue = [Collections.Queue]::new()
foreach ($registration in $registrations) { $registrationQueue.Enqueue($registration) }
$registrationReader = {
    if ($registrationQueue.Count -eq 0) { throw "No test registration remains." }
    return $registrationQueue.Dequeue()
}.GetNewClosure()
$installerInvoker = {
    param([string]$Path, [string[]]$Arguments)
    [IO.File]::WriteAllText(
        $env:LINGTU_TEST_INVOCATION,
        ([ordered]@{ path = $Path; arguments = @($Arguments) } | ConvertTo-Json -Compress)
    )
    return [int]$env:LINGTU_TEST_INSTALLER_EXIT_CODE
}.GetNewClosure()
$parameters = @{
    MinimumVersion = [version]$env:LINGTU_TEST_MINIMUM_VERSION
    InstallerPath = [IO.FileInfo]$env:LINGTU_TEST_INSTALLER_PATH
    RegistrationReader = $registrationReader
}
if ($env:LINGTU_TEST_DEFAULT_INSTALLER -ne "1") { $parameters.InstallerInvoker = $installerInvoker }
if ($env:LINGTU_TEST_CHECK_ONLY -eq "1") { $parameters.CheckOnly = $true }
if ($env:LINGTU_TEST_QUIET -eq "1") { $parameters.Quiet = $true }
if ($env:LINGTU_TEST_LOG_PATH) { $parameters.LogPath = $env:LINGTU_TEST_LOG_PATH }
Invoke-LingTuVcRedistBootstrap @parameters | ConvertTo-Json -Compress
""".strip()
        + "\n",
        encoding="utf-8",
    )
    environment = os.environ.copy()
    environment.update(
        {
            "LINGTU_TEST_MODULE": str(MODULE),
            "LINGTU_TEST_REGISTRATIONS": str(registrations_path),
            "LINGTU_TEST_INVOCATION": str(invocation_path),
            "LINGTU_TEST_INSTALLER_PATH": str(installer_path),
            "LINGTU_TEST_INSTALLER_EXIT_CODE": str(installer_exit_code),
            "LINGTU_TEST_MINIMUM_VERSION": minimum_version,
            "LINGTU_TEST_CHECK_ONLY": "1" if check_only else "0",
            "LINGTU_TEST_QUIET": "1" if quiet else "0",
            "LINGTU_TEST_LOG_PATH": str(log_path) if log_path is not None else "",
            "LINGTU_TEST_DEFAULT_INSTALLER": (
                "1" if use_default_installer_invoker else "0"
            ),
        }
    )
    completed = subprocess.run(  # noqa: S603 - runs a repository-owned test harness.
        [powershell, "-NoProfile", "-File", str(harness_path)],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )
    output_lines = [line for line in completed.stdout.splitlines() if line.strip()]
    result = json.loads(output_lines[-1]) if completed.returncode == 0 and output_lines else None
    return completed, result, invocation_path


def test_satisfied_runtime_does_not_start_installer(tmp_path: Path) -> None:
    completed, result, invocation_path = _invoke_bootstrap(
        tmp_path,
        [{"Installed": True, "Version": "v14.44.35207.0"}],
    )

    assert completed.returncode == 0, completed.stdout + completed.stderr
    assert result == {
        "Status": "Satisfied",
        "InstalledVersion": "14.44.35207.0",
        "RequiredVersion": "14.44.35207.0",
        "InstallerExitCode": None,
    }
    assert not invocation_path.exists()


def test_check_only_reports_install_required_without_starting_installer(
    tmp_path: Path,
) -> None:
    completed, result, invocation_path = _invoke_bootstrap(
        tmp_path,
        [{"Installed": True, "Version": "14.40.33810.0"}],
        check_only=True,
    )

    assert completed.returncode == 0, completed.stdout + completed.stderr
    assert result == {
        "Status": "InstallRequired",
        "InstalledVersion": "14.40.33810.0",
        "RequiredVersion": "14.44.35207.0",
        "InstallerExitCode": None,
    }
    assert not invocation_path.exists()


def test_outdated_runtime_is_installed_and_rechecked(tmp_path: Path) -> None:
    completed, result, invocation_path = _invoke_bootstrap(
        tmp_path,
        [
            {"Installed": True, "Version": "14.40.33810.0"},
            {"Installed": True, "Version": "14.44.35211.0"},
        ],
    )

    assert completed.returncode == 0, completed.stdout + completed.stderr
    assert result == {
        "Status": "Installed",
        "InstalledVersion": "14.44.35211.0",
        "RequiredVersion": "14.44.35207.0",
        "InstallerExitCode": 0,
    }
    invocation = json.loads(invocation_path.read_text(encoding="utf-8"))
    assert Path(invocation["path"]).name == "vc_redist.x64.exe"
    assert invocation["arguments"] == ["/install", "/passive", "/norestart"]


def test_failed_installer_exit_code_is_rejected(tmp_path: Path) -> None:
    completed, result, invocation_path = _invoke_bootstrap(
        tmp_path,
        [{"Installed": False, "Version": None}],
        installer_exit_code=1603,
    )

    assert completed.returncode != 0
    assert result is None
    assert "installer failed with exit code 1603" in completed.stdout + completed.stderr
    assert invocation_path.exists()


def test_success_exit_is_rejected_when_runtime_is_still_too_old(tmp_path: Path) -> None:
    completed, result, invocation_path = _invoke_bootstrap(
        tmp_path,
        [
            {"Installed": True, "Version": "14.40.33810.0"},
            {"Installed": True, "Version": "14.40.33810.0"},
        ],
    )

    assert completed.returncode != 0
    assert result is None
    assert "remains below 14.44.35207.0" in completed.stdout + completed.stderr
    assert invocation_path.exists()


def test_restart_required_exit_is_a_verified_success(tmp_path: Path) -> None:
    completed, result, _ = _invoke_bootstrap(
        tmp_path,
        [
            {"Installed": False, "Version": None},
            {"Installed": True, "Version": "14.44.35211.0"},
        ],
        installer_exit_code=3010,
    )

    assert completed.returncode == 0, completed.stdout + completed.stderr
    assert result == {
        "Status": "RestartRequired",
        "InstalledVersion": "14.44.35211.0",
        "RequiredVersion": "14.44.35207.0",
        "InstallerExitCode": 3010,
    }


def test_quiet_install_forwards_an_absolute_log_path(tmp_path: Path) -> None:
    log_path = tmp_path / "logs" / "vc redist.log"
    completed, _, invocation_path = _invoke_bootstrap(
        tmp_path,
        [
            {"Installed": False, "Version": None},
            {"Installed": True, "Version": "14.44.35211.0"},
        ],
        quiet=True,
        log_path=log_path,
    )

    assert completed.returncode == 0, completed.stdout + completed.stderr
    invocation = json.loads(invocation_path.read_text(encoding="utf-8"))
    assert invocation["arguments"] == [
        "/install",
        "/quiet",
        "/norestart",
        "/log",
        f'"{log_path}"',
    ]


@pytest.mark.skipif(os.name != "nt", reason="validates a Windows Authenticode file")
def test_default_installer_rejects_an_unsigned_executable(tmp_path: Path) -> None:
    completed, result, invocation_path = _invoke_bootstrap(
        tmp_path,
        [{"Installed": False, "Version": None}],
        use_default_installer_invoker=True,
    )

    assert completed.returncode != 0
    assert result is None
    assert "valid Microsoft signature" in completed.stdout + completed.stderr
    assert not invocation_path.exists()


@pytest.mark.skipif(os.name != "nt", reason="validates a signed Windows executable")
def test_installer_identity_rejects_other_microsoft_executables() -> None:
    powershell = _pwsh()
    environment = os.environ.copy()
    environment["LINGTU_TEST_MODULE"] = str(MODULE)
    environment["LINGTU_TEST_SIGNED_EXE"] = str(
        Path(os.environ["SystemRoot"]) / "System32" / "where.exe"
    )
    command = r"""
$ErrorActionPreference = "Stop"
$module = Import-Module -Force -PassThru $env:LINGTU_TEST_MODULE
& $module {
    param([string]$Path)
    Assert-LingTuVcRedistInstaller -InstallerPath ([IO.FileInfo]$Path)
} $env:LINGTU_TEST_SIGNED_EXE
"""
    completed = subprocess.run(  # noqa: S603 - inspects a trusted Windows binary.
        [powershell, "-NoProfile", "-Command", command],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )

    assert completed.returncode != 0
    assert "is not the Microsoft Visual C++ Redistributable x64 installer" in (
        completed.stdout + completed.stderr
    )


def test_cli_help_describes_the_safe_check_and_install_inputs() -> None:
    powershell = _pwsh()
    environment = os.environ.copy()
    environment["LINGTU_TEST_SCRIPT"] = str(SCRIPT)
    completed = subprocess.run(  # noqa: S603 - runs the repository-owned CLI.
        [
            powershell,
            "-NoProfile",
            "-Command",
            "Get-Help -Full -Name $env:LINGTU_TEST_SCRIPT",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )

    assert completed.returncode == 0, completed.stdout + completed.stderr
    output = completed.stdout + completed.stderr
    assert "MinimumVersion" in output
    assert "InstallerPath" in output
    assert "CheckOnly" in output


@pytest.mark.skipif(os.name != "nt", reason="reads the Windows x64 runtime registry")
def test_cli_check_only_uses_a_distinct_install_required_exit_code() -> None:
    powershell = _pwsh()
    completed = subprocess.run(  # noqa: S603 - runs the repository-owned CLI.
        [
            powershell,
            "-NoProfile",
            "-File",
            str(SCRIPT),
            "-MinimumVersion",
            "99.0.0.0",
            "-CheckOnly",
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 2, completed.stdout + completed.stderr
    result = json.loads(completed.stdout)
    assert result["Status"] == "InstallRequired"
    assert result["RequiredVersion"] == "99.0.0.0"
    assert result["InstallerExitCode"] is None
    assert result["InstalledVersion"]


@pytest.mark.skipif(
    os.name != "nt" or WINDOWS_POWERSHELL is None,
    reason="Windows PowerShell 5.1 is unavailable",
)
def test_module_loads_in_the_fresh_windows_powershell_runtime() -> None:
    assert WINDOWS_POWERSHELL is not None
    environment = os.environ.copy()
    environment["LINGTU_TEST_MODULE"] = str(MODULE)
    completed = subprocess.run(  # noqa: S603 - imports the repository-owned module.
        [
            WINDOWS_POWERSHELL,
            "-NoProfile",
            "-Command",
            (
                "Import-Module -Force $env:LINGTU_TEST_MODULE; "
                "(Get-Command Invoke-LingTuVcRedistBootstrap).Name"
            ),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )

    assert completed.returncode == 0, completed.stdout + completed.stderr
    assert completed.stdout.strip() == "Invoke-LingTuVcRedistBootstrap"

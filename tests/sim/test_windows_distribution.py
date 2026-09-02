"""Public-behaviour tests for the fail-closed Windows distribution module."""

# ruff: noqa: S101, S603

from __future__ import annotations

import json
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any

import pytest

from sim.distribution.windows import DistributionError, WindowsDistribution
from sim.runtime.process_owner import ProcessTreeOwner

ROOT = Path(__file__).resolve().parents[2]


def _write(path: Path, content: str | bytes) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    if isinstance(content, bytes):
        path.write_bytes(content)
    else:
        path.write_text(content, encoding="utf-8")
    return path


def _trusted_fixture(tmp_path: Path) -> tuple[Path, dict[str, str], Path]:
    repo = tmp_path / "repo"
    project = repo / "sim/runtime/visual/RobotSimUE"
    unreal = tmp_path / "UE_5.8"
    msvc = tmp_path / "VisualStudio"
    windows_sdk = tmp_path / "WindowsKits10"
    auto_sdk = tmp_path / "UnrealAutoSDK"
    invocation_marker = tmp_path / "uat-was-invoked.txt"

    for relative in (
        "sim/distribution/windows/__init__.py",
        "sim/distribution/windows/__main__.py",
        "sim/distribution/windows/core.py",
        "scripts/sim/package_windows.ps1",
    ):
        _write(repo / relative, f"fixture:{relative}\n")

    _write(
        project / "RobotSimUE.uproject",
        json.dumps(
            {
                "FileVersion": 3,
                "EngineAssociation": "",
                "Modules": [{"Name": "RobotSimUE", "Type": "Runtime"}],
                "Plugins": [{"Name": "LingTuSim", "Enabled": True}],
            }
        ),
    )
    _write(project / "Source/RobotSimUE.Target.cs", "TargetType.Game\n")
    _write(project / "Config/DefaultEngine.ini", "[/Script/Engine.RendererSettings]\n")
    _write(project / "Content/RobotSim/Maps/TestMap.umap", b"fixture-map")
    _write(
        project / "Plugins/LingTuSim/LingTuSim.uplugin",
        json.dumps(
            {
                "FileVersion": 3,
                "SupportedTargetPlatforms": ["Win64"],
                "Modules": [
                    {
                        "Name": "LingTuSimRuntime",
                        "Type": "Runtime",
                        "LoadingPhase": "Default",
                    }
                ],
            }
        ),
    )

    _write(
        unreal / "Engine/Build/Build.version",
        json.dumps(
            {
                "MajorVersion": 5,
                "MinorVersion": 8,
                "PatchVersion": 1,
                "Changelist": 56057345,
                "BranchName": "++UE5+Release-5.8",
            }
        ),
    )
    _write(unreal / "Engine/Build/InstalledBuild.txt", "fixture-installed-build\n")
    _write(unreal / "Engine/Config/BaseEngine.ini", "[Core.System]\n")
    _write(
        unreal / "Engine/Intermediate/ScriptModules/AutomationUtils.Automation.json",
        '{"Version": 7}\n',
    )
    _write(unreal / "Engine/Intermediate/Build/BuildRules/UE5Rules.dll", b"fixture-rules")
    _write(unreal / "Engine/Intermediate/Build/BuildRules/UE5ProgramRules.dll", b"fixture-program-rules")
    _write(
        unreal / "Engine/Intermediate/Build/BuildRules/UE5ProgramRulesManifest.json",
        json.dumps(
            {
                "SourceFiles": [
                    r"D:\build\++UE5\Sync\Engine\Source\Programs\UnrealPak\UnrealPak.Build.cs",
                    r"D:\build\++UE5\Sync\Engine\Source\Programs\UnrealPak\UnrealPak.Target.cs",
                ]
            }
        ),
    )
    _write(
        unreal / "Engine/Source/Programs/AutomationTool/AutomationTool.csproj",
        "<Project />\n",
    )
    _write(
        unreal / "Engine/Source/Programs/Shared/EpicGames.Build/Unreal.cs",
        (
            "using System;\n"
            "namespace EpicGames.Build;\n"
            "internal static class Unreal\n"
            "{\n"
            "\tprivate static DirectoryReference FindRootDirectory()\n"
            "\t{\n"
            "\t\treturn new DirectoryReference(\"fixture\");\n"
            "\t}\n"
            "\tprivate static FileReference FindUnrealBuildToolDll()\n"
            "\t{\n"
            "\t\treturn new FileReference(\"fixture\");\n"
            "\t}\n"
            "\tprivate static DirectoryReference GetUserSettingDirectory()\n"
            "\t{\n"
            "\t\tif (OperatingSystem.IsMacOS() || OperatingSystem.IsLinux())\n"
            "\t\t{\n"
            "\t\t\treturn GetApplicationSettingDirectory();\n"
            "\t\t}\n"
            "\t\treturn DirectoryReference.Combine(EngineDirectory, \"Saved\");\n"
            "\t}\n"
            "}\n"
        ),
    )
    _write(
        unreal / "Engine/Source/Programs/Shared/MetaData.cs",
        '[assembly: System.Reflection.AssemblyVersion("5.8.0.0")]\n',
    )
    _write(
        unreal / "Engine/Intermediate/Build/Win64/x64/UnrealGame/Shipping/fixture.lib",
        b"fixture-precompiled-engine",
    )
    _write(
        unreal / "Engine/Build/BatchFiles/RunUAT.bat",
        (
            "@echo off\r\n"
            "if not defined LINGTU_DISTRIBUTION_ARCHIVE_ROOT exit /b 9\r\n"
            'mkdir "%LINGTU_DISTRIBUTION_ARCHIVE_ROOT%\\Windows\\RobotSimUE\\Binaries\\Win64"\r\n'
            f'echo invoked>"{invocation_marker}"\r\n'
            'echo MZ-packaged>"%LINGTU_DISTRIBUTION_ARCHIVE_ROOT%\\Windows'
            '\\RobotSimUE.exe"\r\n'
            'echo MZ-shipping>"%LINGTU_DISTRIBUTION_ARCHIVE_ROOT%\\Windows\\RobotSimUE'
            '\\Binaries\\Win64\\RobotSimUE-Win64-Shipping.exe"\r\n'
            "exit /b 0\r\n"
        ),
    )
    _write(unreal / "Engine/Binaries/Win64/UnrealEditor-Cmd.exe", b"fixture-editor")
    _write(
        unreal / "Engine/Binaries/DotNET/AutomationTool/AutomationTool.dll",
        b"fixture-automation-tool",
    )
    _write(
        unreal / "Engine/Binaries/DotNET/AutomationTool/EpicGames.Build.dll",
        b"fixture-epicgames-build",
    )
    _write(
        unreal / "Engine/Binaries/DotNET/UnrealBuildTool/UnrealBuildTool.dll",
        b"fixture-unreal-build-tool",
    )
    _write(
        unreal / "Engine/Binaries/DotNET/UnrealBuildTool/EpicGames.Build.dll",
        b"fixture-epicgames-build",
    )
    for dependency in (
        "EpicGames.Core.dll",
        "EpicGames.IoHash.dll",
        "EpicGames.MsBuild.dll",
        "Microsoft.Extensions.FileSystemGlobbing.dll",
        "Microsoft.Extensions.Logging.Abstractions.dll",
    ):
        _write(
            unreal / f"Engine/Binaries/DotNET/UnrealBuildTool/{dependency}",
            f"fixture:{dependency}".encode("ascii"),
        )
    _write(
        unreal / "Engine/Binaries/ThirdParty/DotNet/10.0/win-x64/dotnet.exe",
        b"fixture-dotnet",
    )
    _write(
        msvc / "VC/Auxiliary/Build/Microsoft.VCToolsVersion.default.txt",
        "14.44.35207\n",
    )
    _write(
        msvc / "VC/Tools/MSVC/14.44.35207/bin/Hostx64/x64/cl.exe",
        b"fixture-msvc",
    )
    _write(windows_sdk / "bin/10.0.26100.0/x64/rc.exe", b"fixture-windows-sdk")
    _write(windows_sdk / "Lib/10.0.26100.0/um/x64/kernel32.lib", b"fixture-lib")
    _write(
        auto_sdk / "HostWin64/Win64/Windows Kits/NETFXSDK/4.6.2/Include/um/mscoree.h",
        b"fixture-netfx-header",
    )

    policy = {
        "schema": "lingtu.sim.windows-distribution-policy.v1",
        "product": {
            "id": "robotsimue",
            "project": "sim/runtime/visual/RobotSimUE/RobotSimUE.uproject",
            "target": "RobotSimUE",
            "platform": "Win64",
            "configuration": "Shipping",
            "maps": ["/Game/RobotSim/Maps/TestMap"],
        },
        "unreal": {
            "version": "5.8.1",
            "changelist": 56057345,
            "branch": "++UE5+Release-5.8",
            "evidence": [
                {"path": "Engine/Build/Build.version"},
                {"path": "Engine/Build/BatchFiles/RunUAT.bat"},
                {"path": "Engine/Binaries/Win64/UnrealEditor-Cmd.exe"},
                {"path": "Engine/Binaries/DotNET/AutomationTool/AutomationTool.dll"},
                {"path": "Engine/Binaries/DotNET/UnrealBuildTool/UnrealBuildTool.dll"},
            ],
        },
        "host_toolchain": {
            "msvc": {
                "version": "14.44.35207",
                "version_file": "VC/Auxiliary/Build/Microsoft.VCToolsVersion.default.txt",
                "compiler": "VC/Tools/MSVC/14.44.35207/bin/Hostx64/x64/cl.exe",
            },
            "windows_sdk": {
                "version": "10.0.26100.0",
                "resource_compiler": "bin/10.0.26100.0/x64/rc.exe",
                "library_evidence": "Lib/10.0.26100.0/um/x64/kernel32.lib",
            },
            "unreal_auto_sdk": {
                "netfx_header": ("HostWin64/Win64/Windows Kits/NETFXSDK/4.6.2/Include/um/mscoree.h"),
            },
        },
    }
    policy_path = repo / "sim/distribution/windows/policy.v1.json"
    _write(policy_path, json.dumps(policy, indent=2, sort_keys=True) + "\n")

    environment = {
        "LINGTU_UNREAL_CANDIDATE_ROOT": str(unreal),
        "LINGTU_MSVC_CANDIDATE_ROOT": str(msvc),
        "LINGTU_WINDOWS_SDK_CANDIDATE_ROOT": str(windows_sdk),
        "UE_SDKS_ROOT": str(auto_sdk),
    }
    return repo, environment, invocation_marker


@pytest.fixture(autouse=True)
def _stub_epicgames_build_patch(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Keep unit package tests independent of the real bundled .NET SDK."""

    def build_patch(*, unreal_root: Path, runtime_root: Path) -> Path:
        assert unreal_root.is_dir()
        output = runtime_root / "EngineCompatibilityPatch/out/EpicGames.Build.dll"
        _write(output, b"MZfixture-patched-epicgames-build")
        return output

    monkeypatch.setattr(
        WindowsDistribution,
        "_build_epicgames_build_patch",
        staticmethod(build_patch),
        raising=False,
    )


def _replace_pinned_uat(repo: Path, environment: dict[str, str], content: str) -> Path:
    run_uat = Path(environment["LINGTU_UNREAL_CANDIDATE_ROOT"]) / "Engine/Build/BatchFiles/RunUAT.bat"
    run_uat.write_text(content, encoding="utf-8")
    return run_uat


def _windows_process_exited(pid: int, *, timeout_ms: int) -> bool:
    import ctypes
    from ctypes import wintypes

    synchronize = 0x00100000
    wait_object_0 = 0
    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel32.OpenProcess.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
    kernel32.OpenProcess.restype = wintypes.HANDLE
    kernel32.WaitForSingleObject.argtypes = [wintypes.HANDLE, wintypes.DWORD]
    kernel32.WaitForSingleObject.restype = wintypes.DWORD
    kernel32.CloseHandle.argtypes = [wintypes.HANDLE]
    kernel32.CloseHandle.restype = wintypes.BOOL
    process = kernel32.OpenProcess(synchronize, False, pid)
    if not process:
        return True
    try:
        return bool(kernel32.WaitForSingleObject(process, timeout_ms) == wait_object_0)
    finally:
        kernel32.CloseHandle(process)


def _wait_for_path(path: Path, *, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if path.exists():
            return True
        time.sleep(0.01)
    return False


def _wait_for_pid(path: Path, *, timeout_s: float) -> int | None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            value = path.read_text(encoding="ascii").strip()
        except (FileNotFoundError, OSError):
            value = ""
        if value.isdecimal() and int(value) > 0:
            return int(value)
        time.sleep(0.01)
    return None


@pytest.mark.skipif(sys.platform != "win32", reason="exercises Windows batch process-tree ownership")
def test_package_timeout_terminates_uat_descendants(tmp_path: Path) -> None:
    repo, environment, _invocation_marker = _trusted_fixture(tmp_path)
    child_started = tmp_path / "uat-child-started.txt"
    child_script = _write(
        tmp_path / "long_lived_uat_child.py",
        (
            "from __future__ import annotations\n"
            "import os\n"
            "import sys\n"
            "import time\n"
            "from pathlib import Path\n"
            "Path(sys.argv[1]).write_text(str(os.getpid()), encoding='ascii')\n"
            "time.sleep(30)\n"
        ),
    )
    uat_parent_script = _write(
        tmp_path / "fake_uat_parent.py",
        (
            "from __future__ import annotations\n"
            "import subprocess\n"
            "import sys\n"
            "import time\n"
            "subprocess.Popen([sys.executable, sys.argv[1], sys.argv[2]])\n"
            "time.sleep(30)\n"
        ),
    )
    _replace_pinned_uat(
        repo,
        environment,
        (
            "@echo off\r\n"
            f'"{sys.executable}" -c "import time; time.sleep(0.2)"\r\n'
            f'"{sys.executable}" "{uat_parent_script}" "{child_script}" "{child_started}"\r\n'
            "exit /b 0\r\n"
        ),
    )

    with pytest.raises(DistributionError, match="timed out"):
        WindowsDistribution(
            repo_root=repo,
            environment=environment,
            package_timeout_s=5.0,
        ).execute("package")

    child_pid = _wait_for_pid(child_started, timeout_s=1.0)
    assert child_pid is not None
    assert _windows_process_exited(child_pid, timeout_ms=1000)


@pytest.mark.skipif(sys.platform != "win32", reason="exercises Windows ownership launch gate")
def test_package_does_not_invoke_uat_before_worker_tree_is_owned(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    repo, environment, invocation_marker = _trusted_fixture(tmp_path)

    class DelayedAttachOwner(ProcessTreeOwner):
        def attach(self, process: subprocess.Popen[Any]) -> None:
            time.sleep(1.0)
            assert not invocation_marker.exists()
            super().attach(process)

    monkeypatch.setattr(
        "sim.distribution.windows.core.ProcessTreeOwner",
        DelayedAttachOwner,
    )

    packaged = WindowsDistribution(
        repo_root=repo,
        environment=environment,
        package_timeout_s=15.0,
    ).execute("package")

    assert invocation_marker.is_file()
    assert packaged.manifest["state"] == "packaged"


@pytest.mark.skipif(sys.platform != "win32", reason="exercises Windows batch process-tree ownership")
def test_package_success_terminates_background_uat_descendants(tmp_path: Path) -> None:
    repo, environment, _invocation_marker = _trusted_fixture(tmp_path)
    child_started = tmp_path / "uat-background-child.txt"
    child_script = _write(
        tmp_path / "long_lived_uat_child.py",
        (
            "from __future__ import annotations\n"
            "import os\n"
            "import sys\n"
            "import time\n"
            "from pathlib import Path\n"
            "Path(sys.argv[1]).write_text(str(os.getpid()), encoding='ascii')\n"
            "time.sleep(30)\n"
        ),
    )
    uat_parent_script = _write(
        tmp_path / "successful_fake_uat.py",
        (
            "from __future__ import annotations\n"
            "import os\n"
            "import subprocess\n"
            "import sys\n"
            "import time\n"
            "from pathlib import Path\n"
            "subprocess.Popen([sys.executable, sys.argv[1], sys.argv[2]])\n"
            "deadline = time.monotonic() + 10.0\n"
            "marker = Path(sys.argv[2])\n"
            "while not marker.exists() or not marker.read_text(encoding='ascii').strip():\n"
            "    if time.monotonic() >= deadline:\n"
            "        raise SystemExit(8)\n"
            "    time.sleep(0.01)\n"
            "executable = Path(os.environ['LINGTU_DISTRIBUTION_ARCHIVE_ROOT']) / 'Windows/RobotSimUE.exe'\n"
            "executable.parent.mkdir(parents=True, exist_ok=True)\n"
            "executable.write_bytes(b'MZfixture')\n"
            "shipping = executable.parent / 'RobotSimUE/Binaries/Win64/RobotSimUE-Win64-Shipping.exe'\n"
            "shipping.parent.mkdir(parents=True, exist_ok=True)\n"
            "shipping.write_bytes(b'MZfixture-shipping')\n"
        ),
    )
    _replace_pinned_uat(
        repo,
        environment,
        (
            "@echo off\r\n"
            f'"{sys.executable}" "{uat_parent_script}" "{child_script}" "{child_started}"\r\n'
            "exit /b %errorlevel%\r\n"
        ),
    )

    packaged = WindowsDistribution(
        repo_root=repo,
        environment=environment,
        package_timeout_s=15.0,
    ).execute("package")

    child_pid = _wait_for_pid(child_started, timeout_s=1.0)
    assert child_pid is not None
    assert _windows_process_exited(child_pid, timeout_ms=1000)
    assert packaged.manifest["state"] == "packaged"


@pytest.mark.skipif(sys.platform != "win32", reason="exercises Windows batch invocation")
def test_package_owned_worker_handles_pinned_uat_path_with_spaces(tmp_path: Path) -> None:
    repo, environment, invocation_marker = _trusted_fixture(tmp_path)
    original_unreal = Path(environment["LINGTU_UNREAL_CANDIDATE_ROOT"])
    spaced_unreal = tmp_path / "Epic Games" / original_unreal.name
    spaced_unreal.parent.mkdir(parents=True)
    original_unreal.replace(spaced_unreal)
    environment["LINGTU_UNREAL_CANDIDATE_ROOT"] = str(spaced_unreal)

    packaged = WindowsDistribution(
        repo_root=repo,
        environment=environment,
        package_timeout_s=15.0,
    ).execute("package")

    assert invocation_marker.is_file()
    assert packaged.manifest["state"] == "packaged"


@pytest.mark.skipif(sys.platform != "win32", reason="exercises Windows batch process-tree ownership")
@pytest.mark.parametrize("failure", [RuntimeError, KeyboardInterrupt])
def test_package_wait_failure_always_terminates_uat_descendants(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    failure: type[BaseException],
) -> None:
    repo, environment, _invocation_marker = _trusted_fixture(tmp_path)
    child_started = tmp_path / "uat-child-started.txt"
    child_script = _write(
        tmp_path / "long_lived_uat_child.py",
        (
            "from __future__ import annotations\n"
            "import os\n"
            "import sys\n"
            "import time\n"
            "from pathlib import Path\n"
            "Path(sys.argv[1]).write_text(str(os.getpid()), encoding='ascii')\n"
            "time.sleep(30)\n"
        ),
    )
    uat_parent_script = _write(
        tmp_path / "fake_uat_parent.py",
        (
            "from __future__ import annotations\n"
            "import subprocess\n"
            "import sys\n"
            "import time\n"
            "subprocess.Popen([sys.executable, sys.argv[1], sys.argv[2]])\n"
            "time.sleep(30)\n"
        ),
    )
    _replace_pinned_uat(
        repo,
        environment,
        (
            "@echo off\r\n"
            f'"{sys.executable}" -c "import time; time.sleep(0.2)"\r\n'
            f'"{sys.executable}" "{uat_parent_script}" "{child_script}" "{child_started}"\r\n'
            "exit /b 0\r\n"
        ),
    )
    observed_child_pid: list[int] = []

    class InterruptingPopen(subprocess.Popen[bytes]):
        _inject_failure = True

        def wait(self, timeout: float | None = None) -> int:
            if self._inject_failure:
                child_pid = _wait_for_pid(child_started, timeout_s=10.0)
                assert child_pid is not None
                observed_child_pid.append(child_pid)
                self._inject_failure = False
                raise failure("injected package wait failure")
            return super().wait(timeout=timeout)

    monkeypatch.setattr("sim.distribution.windows.core.subprocess.Popen", InterruptingPopen)

    with pytest.raises(failure, match="injected package wait failure"):
        WindowsDistribution(
            repo_root=repo,
            environment=environment,
            package_timeout_s=30.0,
        ).execute("package")

    assert len(observed_child_pid) == 1
    assert _windows_process_exited(observed_child_pid[0], timeout_ms=1000)


@pytest.mark.parametrize(
    "invalid_timeout",
    [0.0, -1.0, float("nan"), float("inf"), float("-inf"), True],
)
def test_package_timeout_requires_a_finite_positive_number(
    tmp_path: Path,
    invalid_timeout: float,
) -> None:
    repo, environment, invocation_marker = _trusted_fixture(tmp_path)

    with pytest.raises(ValueError, match="finite positive number"):
        WindowsDistribution(
            repo_root=repo,
            environment=environment,
            package_timeout_s=invalid_timeout,
        )

    assert not invocation_marker.exists()
    assert not (repo / "build/distribution/windows").exists()


@pytest.mark.skipif(sys.platform != "win32", reason="exercises Windows package-process concurrency")
def test_package_rejects_a_concurrent_run_before_starting_second_uat(
    tmp_path: Path,
) -> None:
    repo, environment, invocation_marker = _trusted_fixture(tmp_path)
    release_uat = tmp_path / "release-first-uat.txt"
    _replace_pinned_uat(
        repo,
        environment,
        (
            "@echo off\r\n"
            f'echo invoked>>"{invocation_marker}"\r\n'
            ":wait_for_release\r\n"
            f'if exist "{release_uat}" goto released\r\n'
            '"%SystemRoot%\\System32\\ping.exe" 127.0.0.1 -n 2 >nul\r\n'
            "goto wait_for_release\r\n"
            ":released\r\n"
            "if not defined LINGTU_DISTRIBUTION_ARCHIVE_ROOT exit /b 9\r\n"
            'mkdir "%LINGTU_DISTRIBUTION_ARCHIVE_ROOT%\\Windows\\RobotSimUE\\Binaries\\Win64"\r\n'
            'echo MZ-packaged>"%LINGTU_DISTRIBUTION_ARCHIVE_ROOT%\\Windows'
            '\\RobotSimUE.exe"\r\n'
            'echo MZ-shipping>"%LINGTU_DISTRIBUTION_ARCHIVE_ROOT%\\Windows\\RobotSimUE'
            '\\Binaries\\Win64\\RobotSimUE-Win64-Shipping.exe"\r\n'
            "exit /b 0\r\n"
        ),
    )
    first_result: list[object] = []
    first_errors: list[BaseException] = []

    def package_first() -> None:
        try:
            first_result.append(
                WindowsDistribution(
                    repo_root=repo,
                    environment=environment,
                    package_timeout_s=30.0,
                ).execute("package")
            )
        except BaseException as exc:  # pragma: no cover - asserted below
            first_errors.append(exc)

    first_thread = threading.Thread(target=package_first, daemon=True)
    first_thread.start()
    try:
        assert _wait_for_path(invocation_marker, timeout_s=15.0)
        with pytest.raises(DistributionError, match="already in progress"):
            WindowsDistribution(
                repo_root=repo,
                environment=environment,
                package_timeout_s=1.0,
            ).execute("package")
    finally:
        release_uat.write_text("release", encoding="ascii")
        first_thread.join(timeout=20.0)

    assert not first_thread.is_alive()
    assert first_errors == []
    assert len(first_result) == 1
    assert invocation_marker.read_text(encoding="utf-8").splitlines() == [
        "invoked",
        "invoked",
    ]


def test_dry_run_is_deterministic_and_never_invokes_uat(tmp_path: Path) -> None:
    repo, environment, invocation_marker = _trusted_fixture(tmp_path)
    _write(
        repo
        / "sim/runtime/visual/RobotSimUE/Build/Windows/FileOpenOrder/CookerOpenOrder.log",
        "generated\n",
    )
    _write(
        repo / "sim/runtime/visual/RobotSimUE/Build/Windows/Resources/application.ico",
        b"authored-icon",
    )
    distribution = WindowsDistribution(repo_root=repo, environment=environment)

    first = distribution.execute("dry-run")
    assert first.manifest_path is not None
    first_bytes = first.manifest_path.read_bytes()
    second = distribution.execute("dry-run")
    assert second.manifest_path is not None

    assert second.manifest_path.read_bytes() == first_bytes
    assert first.manifest["state"] == "planned"
    assert first.manifest["claims"] == {
        "cook_completed": False,
        "stage_completed": False,
        "package_completed": False,
        "shipping_build_produced": False,
        "packaged_smoke_passed": False,
    }
    assert first.manifest["artifacts"] == []
    assert first.manifest["toolchain"]["unreal"]["version"] == "5.8.1"
    phases = first.manifest["pipeline"]["phases"]
    assert [phase["id"] for phase in phases] == [
        "build_cook",
        "stage_package_archive",
    ]
    build_cook = phases[0]["command"]
    stage_package = phases[1]["command"]
    assert phases[0]["content_root"] == "pinned_install"
    assert phases[1]["content_root"] == "writable_projection"
    assert "-MapsToCook=/Game/RobotSim/Maps/TestMap" in build_cook
    assert build_cook.count("-ddc=InstalledNoZenLocalFallback") == 1
    assert build_cook.count("-NoZenAutoLaunch=127.0.0.1") == 1
    assert build_cook.count(
        "-AdditionalCookerOptions=-SkipZenStore "
        "-DisablePlugins=ModelContextProtocol,AllToolsets,Terminal"
    ) == 1
    assert "-build" in build_cook
    assert "-cook" in build_cook
    assert "-stage" not in build_cook
    assert "-skipbuild" in stage_package
    assert "-skipcook" in stage_package
    assert "-stagingdirectory=${STAGE_ROOT}" in stage_package
    assert "-archive" in stage_package
    assert "-archivedirectory=${ARCHIVE_ROOT}/Windows" in stage_package
    assert not any(argument.startswith("-map=") for argument in build_cook)
    for required_flag in (
        "-installed",
        "-distribution",
        "-nocompileuat",
    ):
        assert required_flag in build_cook
        assert required_flag in stage_package
    assert not invocation_marker.exists()
    assert not (repo / "build/distribution/windows/releases").exists()


def test_preflight_accepts_changed_unreal_entrypoint_at_the_pinned_path(
    tmp_path: Path,
) -> None:
    repo, environment, invocation_marker = _trusted_fixture(tmp_path)
    run_uat = Path(environment["LINGTU_UNREAL_CANDIDATE_ROOT"]) / "Engine/Build/BatchFiles/RunUAT.bat"
    run_uat.write_text("@echo off\r\necho tampered\r\n", encoding="utf-8")

    result = WindowsDistribution(repo_root=repo, environment=environment).execute("dry-run")

    assert not invocation_marker.exists()
    assert result.manifest["state"] == "planned"


def test_preflight_verifies_trusted_inputs_without_writing_distribution_artifacts(
    tmp_path: Path,
) -> None:
    repo, environment, invocation_marker = _trusted_fixture(tmp_path)

    result = WindowsDistribution(repo_root=repo, environment=environment).execute("preflight")

    assert result.manifest_path is None
    assert result.manifest["schema"] == "lingtu.sim.windows-distribution-preflight.v1"
    assert result.manifest["state"] == "passed"
    assert result.manifest["checks"] == [
        "trusted_policy",
        "project_contract",
        "unreal_pin",
        "host_toolchain_pin",
    ]
    assert result.manifest["shipping_build_produced"] is False
    assert not invocation_marker.exists()
    assert not (repo / "build/distribution/windows").exists()


def test_package_claims_shipping_only_after_uat_produces_executable(
    tmp_path: Path,
) -> None:
    repo, environment, invocation_marker = _trusted_fixture(tmp_path)
    distribution = WindowsDistribution(repo_root=repo, environment=environment)
    distribution.execute("dry-run")

    packaged = distribution.execute("package")

    assert invocation_marker.is_file()
    assert packaged.manifest_path is not None
    assert packaged.manifest_path.parent.name == "robotsimue-win64-shipping"
    assert packaged.manifest["state"] == "packaged"
    assert packaged.manifest["claims"] == {
        "cook_completed": True,
        "stage_completed": True,
        "package_completed": True,
        "shipping_build_produced": True,
        "packaged_smoke_passed": False,
    }
    artifact_by_path = {
        item["path"]: item for item in packaged.manifest["artifacts"]
    }
    launcher_artifact = artifact_by_path["package/Windows/RobotSimUE.exe"]
    launcher = packaged.manifest_path.parent / launcher_artifact["path"]
    shipping_artifact = artifact_by_path[
        "package/Windows/RobotSimUE/Binaries/Win64/RobotSimUE-Win64-Shipping.exe"
    ]
    runtime_artifact = artifact_by_path[
        "package/Windows/RobotSimUE/Binaries/Win64/RobotSimUE-Win64-Release.exe"
    ]
    shipping = packaged.manifest_path.parent / shipping_artifact["path"]
    runtime = packaged.manifest_path.parent / runtime_artifact["path"]
    assert launcher_artifact["bytes"] == launcher.stat().st_size
    assert shipping_artifact["bytes"] == shipping.stat().st_size
    assert runtime_artifact["bytes"] == runtime.stat().st_size
    assert runtime.read_bytes() == shipping.read_bytes()


def test_package_isolates_automationtool_writable_user_and_temp_roots(
    tmp_path: Path,
) -> None:
    repo, environment, _invocation_marker = _trusted_fixture(tmp_path)
    observation_path = tmp_path / "uat-environment.json"
    capture_script = _write(
        tmp_path / "capture_uat_environment.py",
        (
            "from __future__ import annotations\n"
            "import json\n"
            "import os\n"
            "import sys\n"
            "from pathlib import Path\n"
            "names = (\n"
            "    'LOCALAPPDATA', 'APPDATA', 'TEMP', 'TMP',\n"
            "    'DOTNET_CLI_HOME', 'NUGET_PACKAGES',\n"
            "    'uebp_EngineSavedFolder', 'uebp_LogFolder', 'uebp_FinalLogFolder',\n"
            "    'uebp_LOCAL_ROOT',\n"
            "    'UE-LocalDataCachePath',\n"
            "    'LINGTU_UNREAL_ROOT_DIRECTORY', 'UBA_ROOT',\n"
            "    'LINGTU_UNREAL_CONTENT_ROOT_DIRECTORY',\n"
            "    'LINGTU_UNREAL_USER_SETTING_DIRECTORY',\n"
            ")\n"
            "path = Path(sys.argv[1])\n"
            "observations = json.loads(path.read_text(encoding='utf-8')) if path.exists() else []\n"
            "observations.append({**{name: os.environ[name] for name in names}, '__arguments__': sys.argv[2:]})\n"
            "path.write_text(json.dumps(observations), encoding='utf-8')\n"
            "executable = Path(os.environ['LINGTU_DISTRIBUTION_ARCHIVE_ROOT']) / 'Windows/RobotSimUE.exe'\n"
            "executable.parent.mkdir(parents=True, exist_ok=True)\n"
            "executable.write_bytes(b'MZfixture')\n"
            "shipping = executable.parent / 'RobotSimUE/Binaries/Win64/RobotSimUE-Win64-Shipping.exe'\n"
            "shipping.parent.mkdir(parents=True, exist_ok=True)\n"
            "shipping.write_bytes(b'MZfixture-shipping')\n"
        ),
    )
    _replace_pinned_uat(
        repo,
        environment,
        (
            "@echo off\r\n"
            f'"{sys.executable}" "{capture_script}" "{observation_path}" %*\r\n'
            "exit /b %errorlevel%\r\n"
        ),
    )

    WindowsDistribution(repo_root=repo, environment=environment).execute("package")

    observations = json.loads(observation_path.read_text(encoding="utf-8"))
    assert len(observations) == 2
    build_observed, observed = observations
    for name in ("LOCALAPPDATA", "APPDATA", "TEMP", "TMP"):
        root = Path(observed[name])
        assert root.is_dir()
        root.relative_to(repo / "build/ue")
    for name in ("DOTNET_CLI_HOME", "NUGET_PACKAGES"):
        root = Path(observed[name])
        assert root.is_dir()
        root.relative_to(repo / "build/ue")
    for name in (
        "uebp_EngineSavedFolder",
        "uebp_LogFolder",
        "uebp_FinalLogFolder",
        "UBA_ROOT",
    ):
        root = Path(observed[name])
        assert root.is_dir()
        root.relative_to(repo / "build/ue")
    assert Path(observed["UE-LocalDataCachePath"]) == repo / "build/ue/DerivedDataCache"

    assert observed["LOCALAPPDATA"] != observed["APPDATA"]
    assert observed["TEMP"] == observed["TMP"]
    assert observed["uebp_LogFolder"] == observed["uebp_FinalLogFolder"]

    writable_unreal_root = Path(observed["LINGTU_UNREAL_ROOT_DIRECTORY"])
    writable_unreal_root.relative_to(repo / "build/ue")
    writable_engine = writable_unreal_root / "Engine"
    assert (writable_engine / "Build/Build.version").is_file()
    assert (writable_engine / "Build/InstalledBuild.txt").is_file()
    assert (writable_engine / "Config/BaseEngine.ini").read_text(encoding="utf-8") == "[Core.System]\n"
    user_settings = Path(observed["LINGTU_UNREAL_USER_SETTING_DIRECTORY"])
    assert user_settings.is_dir()
    user_settings.relative_to(repo / "build/ue")
    script_modules = writable_engine / "Intermediate/ScriptModules"
    assert script_modules.is_symlink()
    assert script_modules.resolve() == (
        Path(environment["LINGTU_UNREAL_CANDIDATE_ROOT"])
        / "Engine/Intermediate/ScriptModules"
    ).resolve()
    build_rules = writable_engine / "Intermediate/Build/BuildRules"
    assert build_rules.is_dir()
    assert build_rules.is_symlink()
    assert (build_rules / "UE5Rules.dll").read_bytes() == b"fixture-rules"
    assert (build_rules / "UE5ProgramRules.dll").read_bytes() == b"fixture-program-rules"
    writable_win64 = writable_engine / "Intermediate/Build/Win64"
    assert writable_win64.is_symlink()
    assert (writable_engine / "Source").is_symlink()
    patched_automation = (
        writable_engine / "Binaries/DotNET/AutomationTool/EpicGames.Build.dll"
    )
    patched_ubt = (
        writable_engine / "Binaries/DotNET/UnrealBuildTool/EpicGames.Build.dll"
    )
    for patched in (patched_automation, patched_ubt):
        assert patched.read_bytes() == b"MZfixture-patched-epicgames-build"
        assert not patched.is_symlink()
    direct = next(
        argument
        for argument in observed["__arguments__"]
        if argument.startswith("-XmlConfigCache=")
    )
    cache_path = Path(direct.removeprefix("-XmlConfigCache="))
    assert cache_path.parent.is_dir()
    cache_path.relative_to(repo / "build/ue")
    ubt_arguments = next(
        argument
        for argument in observed["__arguments__"]
        if argument.startswith("-ubtargs=")
    )
    assert "-UsePrecompiled" in ubt_arguments
    assert "-NoHotReloadFromIDE" in ubt_arguments
    assert "-SkipRulesCompile" not in ubt_arguments
    assert "-NoUBA" not in ubt_arguments
    assert "-XmlConfigCache=" not in ubt_arguments
    assert "-rootdirectory=" not in ubt_arguments.casefold()
    assert Path(observed["LINGTU_UNREAL_CONTENT_ROOT_DIRECTORY"]) == writable_unreal_root
    assert Path(build_observed["LINGTU_UNREAL_CONTENT_ROOT_DIRECTORY"]) == Path(
        environment["LINGTU_UNREAL_CANDIDATE_ROOT"]
    )
    assert "-build" in build_observed["__arguments__"]
    assert "-cook" in build_observed["__arguments__"]
    assert "-skipbuild" in observed["__arguments__"]
    assert "-skipcook" in observed["__arguments__"]
    assert Path(observed["uebp_LOCAL_ROOT"]) == writable_unreal_root
    assert "-nocompileuat" in observed["__arguments__"]
    assert "-nocompileeditor" not in observed["__arguments__"]
    assert not any(
        argument.casefold().startswith("-rootdirectory=")
        for argument in observed["__arguments__"]
    )


def test_epicgames_build_patch_inserts_two_fixed_isolation_overrides() -> None:
    source = (
        "private static DirectoryReference FindRootDirectory()\n"
        "{\n"
        "\treturn new DirectoryReference(\"fixture\");\n"
        "}\n"
        "private static FileReference FindUnrealBuildToolDll()\n"
        "{\n"
        "\treturn new FileReference(\"fixture\");\n"
        "}\n"
        "private static DirectoryReference GetUserSettingDirectory()\n"
        "{\n"
        "\tif (OperatingSystem.IsMacOS() || OperatingSystem.IsLinux())\n"
        "\t{\n"
        "\t\treturn GetApplicationSettingDirectory();\n"
        "\t}\n"
        "}\n"
    )

    patched = WindowsDistribution._patch_epicgames_build_source(source)

    for environment_name in (
        "LINGTU_UNREAL_CONTENT_ROOT_DIRECTORY",
        "LINGTU_UNREAL_USER_SETTING_DIRECTORY",
    ):
        assert patched.count(environment_name) == 1
    assert "return new DirectoryReference(overrideDirectory);" in patched
    assert "if (OperatingSystem.IsMacOS() || OperatingSystem.IsLinux())" in patched


def test_epicgames_build_patch_fails_closed_on_source_shape_drift() -> None:
    unexpected = (
        "private static DirectoryReference FindRootDirectories()\n"
        "{\n"
        "\treturn new DirectoryReference(\"fixture\");\n"
        "}\n"
        "private static FileReference FindUnrealBuildToolDll()\n"
        "{\n"
        "\treturn new FileReference(\"fixture\");\n"
        "}\n"
        "private static DirectoryReference GetUserSettingsDirectory()\n"
        "{\n"
        "\treturn GetApplicationSettingDirectory();\n"
        "}\n"
    )

    with pytest.raises(DistributionError, match="source anchor"):
        WindowsDistribution._patch_epicgames_build_source(unexpected)



def test_cli_has_no_arbitrary_unreal_executable_interface(tmp_path: Path) -> None:
    marker = tmp_path / "injected.exe"
    marker.write_bytes(b"not an Unreal executable")

    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "sim.distribution.windows",
            "dry-run",
            "--unreal-executable",
            str(marker),
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )

    assert completed.returncode == 2
    assert "unrecognized arguments: --unreal-executable" in completed.stderr


def test_preflight_detects_pinned_unreal_from_epic_launcher_inventory(
    tmp_path: Path,
) -> None:
    repo, environment, _ = _trusted_fixture(tmp_path)
    unreal_root = environment.pop("LINGTU_UNREAL_CANDIDATE_ROOT")
    program_data = tmp_path / "ProgramData"
    _write(
        program_data / "Epic/UnrealEngineLauncher/LauncherInstalled.dat",
        json.dumps(
            {
                "InstallationList": [
                    {
                        "AppName": "UE_5.8",
                        "InstallLocation": unreal_root,
                    }
                ]
            }
        ),
    )
    environment["ProgramData"] = str(program_data)

    result = WindowsDistribution(repo_root=repo, environment=environment).execute("preflight")

    assert result.manifest["state"] == "passed"
    assert result.unreal_root == Path(unreal_root).resolve()


def test_package_fails_closed_when_uat_exits_zero_without_shipping_executable(
    tmp_path: Path,
) -> None:
    repo, environment, invocation_marker = _trusted_fixture(tmp_path)
    _replace_pinned_uat(
        repo,
        environment,
        f'@echo off\r\necho invoked>"{invocation_marker}"\r\nexit /b 0\r\n',
    )

    with pytest.raises(DistributionError, match="without the required Shipping"):
        WindowsDistribution(repo_root=repo, environment=environment).execute("package")

    assert invocation_marker.is_file()
    assert not (repo / "build/distribution/windows/releases").exists()


def test_policy_rejects_unknown_executable_or_uat_argument_fields(
    tmp_path: Path,
) -> None:
    repo, environment, _ = _trusted_fixture(tmp_path)
    policy_path = repo / "sim/distribution/windows/policy.v1.json"
    policy = json.loads(policy_path.read_text(encoding="utf-8"))
    policy["unreal"]["executable"] = str(tmp_path / "injected.exe")
    policy["product"]["uat_arguments"] = ["-run=arbitrary"]
    policy_path.write_text(json.dumps(policy), encoding="utf-8")

    with pytest.raises(DistributionError, match="unexpected fields"):
        WindowsDistribution(repo_root=repo, environment=environment).execute("preflight")

    assert not (repo / "build/distribution/windows").exists()


def test_operator_wrapper_and_simstudio_do_not_expose_executable_injection() -> None:
    wrapper = (ROOT / "scripts/sim/package_windows.ps1").read_text(encoding="utf-8")
    lowered = wrapper.lower()

    for forbidden_parameter in (
        "$unrealroot",
        "$unrealexecutable",
        "$runuatpath",
        "$uatarguments",
        "$outputdirectory",
    ):
        assert forbidden_parameter not in lowered
    assert "-m sim.distribution.windows $operation" in wrapper

    for route in (ROOT / "tools/simstudio").rglob("*.py"):
        assert "sim.distribution.windows" not in route.read_text(encoding="utf-8")


def test_non_shipping_policy_can_never_produce_a_distribution_plan(
    tmp_path: Path,
) -> None:
    repo, environment, _ = _trusted_fixture(tmp_path)
    policy_path = repo / "sim/distribution/windows/policy.v1.json"
    policy = json.loads(policy_path.read_text(encoding="utf-8"))
    policy["product"]["configuration"] = "Development"
    policy_path.write_text(json.dumps(policy), encoding="utf-8")

    with pytest.raises(DistributionError, match="configuration must be Shipping"):
        WindowsDistribution(repo_root=repo, environment=environment).execute("dry-run")

    assert not (repo / "build/distribution/windows").exists()


def test_preflight_requires_the_complete_unreal_trust_set(tmp_path: Path) -> None:
    repo, environment, _ = _trusted_fixture(tmp_path)
    policy_path = repo / "sim/distribution/windows/policy.v1.json"
    policy = json.loads(policy_path.read_text(encoding="utf-8"))
    policy["unreal"]["evidence"] = [
        item for item in policy["unreal"]["evidence"] if not item["path"].endswith("UnrealBuildTool.dll")
    ]
    policy_path.write_text(json.dumps(policy), encoding="utf-8")

    with pytest.raises(DistributionError, match="complete pinned evidence set"):
        WindowsDistribution(repo_root=repo, environment=environment).execute("preflight")


def test_preflight_rejects_editor_only_lingtu_shipping_module(tmp_path: Path) -> None:
    repo, environment, _ = _trusted_fixture(tmp_path)
    descriptor_path = repo / "sim/runtime/visual/RobotSimUE/Plugins/LingTuSim/LingTuSim.uplugin"
    descriptor = json.loads(descriptor_path.read_text(encoding="utf-8"))
    descriptor["Modules"][0]["Type"] = "Editor"
    descriptor_path.write_text(json.dumps(descriptor), encoding="utf-8")

    with pytest.raises(DistributionError, match="must be Runtime for Shipping"):
        WindowsDistribution(repo_root=repo, environment=environment).execute("preflight")


def test_package_rejects_named_executable_without_windows_pe_magic(
    tmp_path: Path,
) -> None:
    repo, environment, _ = _trusted_fixture(tmp_path)
    _replace_pinned_uat(
        repo,
        environment,
        (
            "@echo off\r\n"
            "if not defined LINGTU_DISTRIBUTION_ARCHIVE_ROOT exit /b 9\r\n"
            'mkdir "%LINGTU_DISTRIBUTION_ARCHIVE_ROOT%\\Windows"\r\n'
            'echo not-a-pe>"%LINGTU_DISTRIBUTION_ARCHIVE_ROOT%\\Windows'
            '\\RobotSimUE.exe"\r\n'
            "exit /b 0\r\n"
        ),
    )

    with pytest.raises(DistributionError, match="not a Windows PE executable"):
        WindowsDistribution(repo_root=repo, environment=environment).execute("package")

    assert not (repo / "build/distribution/windows/releases").exists()

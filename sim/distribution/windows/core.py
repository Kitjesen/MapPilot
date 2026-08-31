"""Trusted Windows distribution planning and execution.

The public interface intentionally accepts an operation, a repository root,
process environment roots, and a bounded package timeout only. Executable paths
and arbitrary UAT arguments are not caller-controlled: they are derived from a
repository-owned policy and checked before use.
"""

from __future__ import annotations

import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from types import TracebackType
from typing import IO, Any, Mapping

from sim.runtime.process_owner import ProcessTreeOwner

_POLICY_RELATIVE_PATH = Path("sim/distribution/windows/policy.v1.json")
_RELEASE_NAME = "robotsimue-win64-shipping"
_REQUIRED_UNREAL_EVIDENCE = {
    "Engine/Build/Build.version",
    "Engine/Build/BatchFiles/RunUAT.bat",
    "Engine/Binaries/Win64/UnrealEditor-Cmd.exe",
    "Engine/Binaries/DotNET/AutomationTool/AutomationTool.dll",
    "Engine/Binaries/DotNET/UnrealBuildTool/UnrealBuildTool.dll",
}
_DEFAULT_PACKAGE_TIMEOUT_S = 6 * 60 * 60.0
_PROCESS_CLEANUP_TIMEOUT_S = 5.0
_SHIPPING_EXECUTABLE_RELATIVE_PATH = Path(
    "Windows/RobotSimUE/Binaries/Win64/RobotSimUE-Win64-Shipping.exe"
)
_PRODUCT_RUNTIME_RELATIVE_PATH = Path(
    "Windows/RobotSimUE/Binaries/Win64/RobotSimUE-Win64-Release.exe"
)
_DEVELOPMENT_ONLY_UNREAL_PLUGINS = (
    "ModelContextProtocol",
    "AllToolsets",
    "Terminal",
)
_EPICGAMES_BUILD_SOURCE = Path(
    "Engine/Source/Programs/Shared/EpicGames.Build"
)
_EPICGAMES_BUILD_METADATA = Path("Engine/Source/Programs/Shared/MetaData.cs")
_EPICGAMES_BUILD_BINARY_DIRS = (
    Path("Engine/Binaries/DotNET/AutomationTool"),
    Path("Engine/Binaries/DotNET/UnrealBuildTool"),
)
_EPICGAMES_BUILD_REFERENCE_NAMES = (
    "EpicGames.Core.dll",
    "EpicGames.IoHash.dll",
    "EpicGames.MsBuild.dll",
    "Microsoft.Extensions.FileSystemGlobbing.dll",
    "Microsoft.Extensions.Logging.Abstractions.dll",
)
_UNREAL_BUNDLED_DOTNET = Path(
    "Engine/Binaries/ThirdParty/DotNet/10.0/win-x64/dotnet.exe"
)
_EPICGAMES_BUILD_PATCH_PROJECT = """\
<Project Sdk="Microsoft.NET.Sdk">
  <PropertyGroup>
    <TargetFramework>net10.0</TargetFramework>
    <AssemblyName>EpicGames.Build</AssemblyName>
    <RootNamespace>UnrealBuildBase</RootNamespace>
    <OutputType>Library</OutputType>
    <AllowUnsafeBlocks>true</AllowUnsafeBlocks>
    <Nullable>enable</Nullable>
    <GenerateAssemblyInfo>false</GenerateAssemblyInfo>
    <GenerateTargetFrameworkAttribute>false</GenerateTargetFrameworkAttribute>
    <EnableDefaultCompileItems>false</EnableDefaultCompileItems>
    <RestoreIgnoreFailedSources>true</RestoreIgnoreFailedSources>
  </PropertyGroup>
  <ItemGroup>
    <Compile Include="Source\\EpicGames.Build\\**\\*.cs" />
    <Compile Include="Source\\MetaData.cs" Link="Properties\\MetaData.cs" />
  </ItemGroup>
  <ItemGroup>
    <Reference Include="EpicGames.Core" HintPath="$(UbtBinaryDir)\\EpicGames.Core.dll" />
    <Reference Include="EpicGames.IoHash" HintPath="$(UbtBinaryDir)\\EpicGames.IoHash.dll" />
    <Reference Include="EpicGames.MsBuild" HintPath="$(UbtBinaryDir)\\EpicGames.MsBuild.dll" />
    <Reference Include="Microsoft.Extensions.FileSystemGlobbing" HintPath="$(UbtBinaryDir)\\Microsoft.Extensions.FileSystemGlobbing.dll" />
    <Reference Include="Microsoft.Extensions.Logging.Abstractions" HintPath="$(UbtBinaryDir)\\Microsoft.Extensions.Logging.Abstractions.dll" />
    <Reference Include="Microsoft.Build" HintPath="$(MSBuildBinPath)\\Microsoft.Build.dll" />
    <Reference Include="Microsoft.Build.Framework" HintPath="$(MSBuildBinPath)\\Microsoft.Build.Framework.dll" />
    <Reference Include="Microsoft.Build.Utilities.Core" HintPath="$(MSBuildBinPath)\\Microsoft.Build.Utilities.Core.dll" />
  </ItemGroup>
</Project>
"""
def _csharp_method_anchor(return_type: str, method_name: str) -> re.Pattern[str]:
    return re.compile(
        rf"(?m)^(?P<indent>[ \t]*)private static {re.escape(return_type)} "
        rf"{re.escape(method_name)}\(\)\n(?P=indent)\{{\n"
    )


_EPICGAMES_BUILD_ROOT_PATTERN = _csharp_method_anchor(
    "DirectoryReference", "FindRootDirectory"
)
_EPICGAMES_BUILD_USER_SETTINGS_PATTERN = _csharp_method_anchor(
    "DirectoryReference", "GetUserSettingDirectory"
)
_OWNED_PROCESS_WORKER_SOURCE = """\
import subprocess
import sys

if sys.stdin.buffer.read(1) != b"\\n":
    raise SystemExit(125)
child = subprocess.Popen(
    sys.argv[1:],
    stdin=subprocess.DEVNULL,
    shell=False,
)
raise SystemExit(child.wait())
"""
_OWNED_UAT_WORKER_SOURCE = """\
import json
import os
import subprocess
import sys

if sys.stdin.buffer.read(1) != b"\\n":
    raise SystemExit(125)
phases = json.loads(sys.argv[1])
for phase in phases:
    os.environ["LINGTU_UNREAL_CONTENT_ROOT_DIRECTORY"] = phase["content_root"]
    print(f"LINGTU_UAT_PHASE {phase['id']}", flush=True)
    child = subprocess.Popen(
        phase["command"],
        stdin=subprocess.DEVNULL,
        shell=False,
    )
    returncode = child.wait()
    if returncode != 0:
        raise SystemExit(returncode)
"""


class DistributionError(RuntimeError):
    """Raised when a distribution operation cannot satisfy its trust contract."""


class _DistributionLease:
    """Hold one non-blocking OS lock while packaging."""

    def __init__(self, path: Path) -> None:
        self._path = path
        self._stream: IO[bytes] | None = None

    def __enter__(self) -> _DistributionLease:
        self._path.parent.mkdir(parents=True, exist_ok=True)
        stream = self._path.open("a+b")
        stream.seek(0, os.SEEK_END)
        if stream.tell() == 0:
            stream.write(b"\0")
            stream.flush()
        stream.seek(0)
        try:
            self._lock(stream)
        except OSError as exc:
            stream.close()
            raise DistributionError("Windows distribution package is already in progress") from exc
        self._stream = stream
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        del exc_type, exc, traceback
        stream = self._stream
        self._stream = None
        if stream is None:
            return
        try:
            self._unlock(stream)
        finally:
            stream.close()

    @staticmethod
    def _lock(stream: IO[bytes]) -> None:
        if os.name == "nt":
            import msvcrt

            msvcrt.locking(stream.fileno(), msvcrt.LK_NBLCK, 1)
            return
        if os.name == "posix":
            fcntl_module: Any = __import__("fcntl")
            fcntl_module.flock(
                stream.fileno(),
                fcntl_module.LOCK_EX | fcntl_module.LOCK_NB,
            )
            return
        raise OSError(f"unsupported package-lock platform: {os.name}")

    @staticmethod
    def _unlock(stream: IO[bytes]) -> None:
        stream.seek(0)
        if os.name == "nt":
            import msvcrt

            msvcrt.locking(stream.fileno(), msvcrt.LK_UNLCK, 1)
        elif os.name == "posix":
            fcntl_module: Any = __import__("fcntl")
            fcntl_module.flock(stream.fileno(), fcntl_module.LOCK_UN)


@dataclass(frozen=True)
class DistributionResult:
    """Observable result of one Windows distribution operation."""

    manifest_path: Path | None
    manifest: dict[str, Any]
    unreal_root: Path


class WindowsDistribution:
    """Create deterministic RobotSimUE distribution plans behind one interface."""

    def __init__(
        self,
        *,
        repo_root: Path,
        environment: Mapping[str, str] | None = None,
        package_timeout_s: float = _DEFAULT_PACKAGE_TIMEOUT_S,
    ) -> None:
        self._repo_root = Path(repo_root).resolve()
        self._environment = dict(os.environ if environment is None else environment)
        if (
            isinstance(package_timeout_s, bool)
            or not isinstance(package_timeout_s, (int, float))
            or not math.isfinite(package_timeout_s)
            or package_timeout_s <= 0
        ):
            raise ValueError("package_timeout_s must be a finite positive number")
        self._package_timeout_s = float(package_timeout_s)

    def execute(self, operation: str) -> DistributionResult:
        """Execute ``preflight``, ``dry-run``, or ``package``.

        Preflight and dry-run never invoke Unreal AutomationTool. Preflight is
        read-only; dry-run additionally writes a deterministic plan manifest.
        """

        if operation not in {"preflight", "dry-run", "package"}:
            raise DistributionError(f"unsupported distribution operation: {operation}")

        policy_path = self._repo_root / _POLICY_RELATIVE_PATH
        policy = self._read_policy(policy_path)
        project_path = self._resolve_repo_file(policy["product"]["project"])
        self._validate_project(project_path, policy["product"])

        unreal_root, unreal_info = self._detect_unreal(policy["unreal"])
        toolchain_info = self._validate_host_toolchain(policy["host_toolchain"])
        if operation == "preflight":
            report = {
                "schema": "lingtu.sim.windows-distribution-preflight.v1",
                "state": "passed",
                "checks": [
                    "trusted_policy",
                    "project_contract",
                    "unreal_pin",
                    "host_toolchain_pin",
                ],
                "shipping_build_produced": False,
                "toolchain": {
                    "unreal": unreal_info,
                    "host": toolchain_info,
                },
            }
            return DistributionResult(
                manifest_path=None,
                manifest=report,
                unreal_root=unreal_root,
            )

        phase_templates = self._phase_command_templates(policy["product"])
        manifest = {
            "schema": "lingtu.sim.windows-distribution-manifest.v1",
            "state": "planned",
            "release": _RELEASE_NAME,
            "product": {
                "id": policy["product"]["id"],
                "target": policy["product"]["target"],
                "platform": policy["product"]["platform"],
                "configuration": policy["product"]["configuration"],
                "maps": list(policy["product"]["maps"]),
            },
            "claims": {
                "cook_completed": False,
                "stage_completed": False,
                "package_completed": False,
                "shipping_build_produced": False,
                "packaged_smoke_passed": False,
            },
            "toolchain": {
                "unreal": unreal_info,
                "host": toolchain_info,
            },
            "pipeline": {
                "driver": "RunUAT BuildCookRun (two phase)",
                "phases": phase_templates,
                "shell": False,
            },
            "artifacts": [],
        }
        if operation == "package":
            return self._package(
                manifest=manifest,
                policy=policy,
                project_path=project_path,
                unreal_root=unreal_root,
            )
        manifest_dir = self._repo_root / "build/distribution/windows/plans" / _RELEASE_NAME
        manifest_path = manifest_dir / "distribution.manifest.json"
        manifest_dir.mkdir(parents=True, exist_ok=True)
        manifest_path.write_bytes(self._canonical_json(manifest))
        return DistributionResult(
            manifest_path=manifest_path,
            manifest=manifest,
            unreal_root=unreal_root,
        )

    def _package(
        self,
        *,
        manifest: dict[str, Any],
        policy: Mapping[str, Any],
        project_path: Path,
        unreal_root: Path,
    ) -> DistributionResult:
        """Run the pinned UAT entrypoint and atomically publish its archive."""

        distribution_root = self._repo_root / "build/distribution/windows"
        lease_path = distribution_root / "package.lock"
        with _DistributionLease(lease_path):
            return self._package_exclusive(
                manifest=manifest,
                policy=policy,
                project_path=project_path,
                unreal_root=unreal_root,
            )

    def _package_exclusive(
        self,
        *,
        manifest: dict[str, Any],
        policy: Mapping[str, Any],
        project_path: Path,
        unreal_root: Path,
    ) -> DistributionResult:
        """Package while the distribution identity lease is held."""

        distribution_root = self._repo_root / "build/distribution/windows"
        release_dir = distribution_root / "releases" / _RELEASE_NAME
        if release_dir.exists():
            raise DistributionError(f"immutable Windows distribution already exists: {release_dir}")
        work_parent = distribution_root / "work"
        work_parent.mkdir(parents=True, exist_ok=True)
        work_dir = Path(tempfile.mkdtemp(prefix="robotsimue-", dir=work_parent))
        archive_root = work_dir / "package"
        stage_root = work_dir / "stage"
        uat_log = work_dir / "RunUAT.log"
        # UE's installed precompiled manifests still flow through Win32 APIs
        # with the legacy MAX_PATH boundary. Keep the Engine projection below
        # 50 characters on the normal repository path, as UBT itself advises.
        runtime_parent = self._repo_root / "build/ue"
        runtime_parent.mkdir(parents=True, exist_ok=True)
        runtime_root = Path(tempfile.mkdtemp(prefix="r-", dir=runtime_parent))
        shared_derived_data_cache = runtime_parent / "DerivedDataCache"
        writable_environment = {
            "LOCALAPPDATA": runtime_root / "LocalAppData",
            "APPDATA": runtime_root / "RoamingAppData",
            "TEMP": runtime_root / "Temp",
            "TMP": runtime_root / "Temp",
            "DOTNET_CLI_HOME": runtime_root / "DotNetHome",
            "NUGET_PACKAGES": runtime_root / "NuGetPackages",
            "uebp_EngineSavedFolder": runtime_root / "EngineSaved",
            "uebp_LogFolder": runtime_root / "AutomationToolLogs",
            "uebp_FinalLogFolder": runtime_root / "AutomationToolLogs",
            # DDC is a content-addressed cache, not run state.  Keep it under
            # the repository-owned build root so retries reuse an expensive
            # completed Cook while every other mutable UE root stays isolated.
            "UE-LocalDataCachePath": shared_derived_data_cache,
            "UBA_ROOT": runtime_root / "UnrealBuildAccelerator",
            "LINGTU_UNREAL_USER_SETTING_DIRECTORY": runtime_root / "UnrealUserSettings",
        }
        for path in set(writable_environment.values()):
            path.mkdir(parents=True, exist_ok=True)
        unreal_policy = self._mapping(policy, "unreal")
        uat_item = next(
            (
                item
                for item in unreal_policy.get("evidence", [])
                if isinstance(item, dict) and item.get("path") == "Engine/Build/BatchFiles/RunUAT.bat"
            ),
            None,
        )
        if uat_item is None:
            raise DistributionError("Unreal policy does not pin RunUAT.bat")
        patched_epicgames_build = self._build_epicgames_build_patch(
            unreal_root=unreal_root,
            runtime_root=runtime_root,
        )
        writable_unreal_root = self._prepare_writable_unreal_root(
            unreal_root=unreal_root,
            runtime_root=runtime_root,
            patched_epicgames_build=patched_epicgames_build,
        )
        xml_config_cache = runtime_root / "UnrealBuildTool" / "XmlConfigCache.bin"
        xml_config_cache.parent.mkdir(parents=True, exist_ok=True)
        run_uat = writable_unreal_root / uat_item["path"]

        content_roots = {
            "build_cook": unreal_root,
            "stage_package_archive": writable_unreal_root,
        }
        phase_specs = []
        for phase in manifest["pipeline"]["phases"]:
            phase_id = phase["id"]
            if phase_id not in content_roots:
                raise DistributionError(f"unsupported Windows distribution phase: {phase_id}")
            phase_specs.append(
                {
                    "id": phase_id,
                    "content_root": str(content_roots[phase_id]),
                    "command": self._materialize_command(
                        phase["command"],
                        run_uat=run_uat,
                        project_path=project_path,
                        archive_root=archive_root,
                        stage_root=stage_root,
                        xml_config_cache=xml_config_cache,
                    ),
                }
            )
        child_environment = dict(os.environ)
        child_environment.update(self._environment)
        child_environment.update(
            {name: str(path) for name, path in writable_environment.items()}
        )
        child_environment["UE_SDKS_ROOT"] = str(self._trusted_root("UE_SDKS_ROOT", self._auto_sdk_candidates()))
        child_environment["LINGTU_DISTRIBUTION_ARCHIVE_ROOT"] = str(archive_root)
        child_environment["LINGTU_UNREAL_ROOT_DIRECTORY"] = str(writable_unreal_root)
        child_environment["uebp_LOCAL_ROOT"] = str(writable_unreal_root)
        # The owned worker keeps build/cook rooted at the pinned installation,
        # then switches only stage/package to the writable Engine projection.
        # The second phase must create StagedBuild_*.ini under Engine/Intermediate,
        # while compiling through two textual Engine roots breaks installed PCHs.
        child_environment["LINGTU_UNREAL_CONTENT_ROOT_DIRECTORY"] = str(
            unreal_root
        )
        worker_command = [
            sys.executable,
            "-I",
            "-c",
            _OWNED_UAT_WORKER_SOURCE,
            json.dumps(phase_specs, separators=(",", ":"), sort_keys=True),
        ]
        process_owner = ProcessTreeOwner()
        process: subprocess.Popen[bytes] | None = None
        gate_stream: IO[bytes] | None = None
        try:
            try:
                with uat_log.open("wb") as log_stream:
                    # Every argv element is derived from the strict repository policy;
                    # callers cannot provide an executable or additional UAT argument.
                    # The isolated worker cannot launch UAT until ownership is attached.
                    process = subprocess.Popen(  # noqa: S603
                        worker_command,
                        cwd=self._repo_root,
                        stdin=subprocess.PIPE,
                        stdout=log_stream,
                        stderr=subprocess.STDOUT,
                        shell=False,
                        env=child_environment,
                        **process_owner.popen_options(),
                    )
                    gate_stream = process.stdin
                    if gate_stream is None:
                        raise DistributionError("owned UAT worker gate was not created")
                    process_owner.attach(process)
                    gate_stream.write(b"\n")
                    gate_stream.flush()
                    gate_stream.close()
                    gate_stream = None
                    try:
                        returncode = process.wait(timeout=self._package_timeout_s)
                    except subprocess.TimeoutExpired as exc:
                        raise DistributionError(
                            "Unreal BuildCookRun timed out after "
                            f"{self._package_timeout_s:g}s; log={uat_log}"
                        ) from exc
            except OSError as exc:
                raise DistributionError(
                    f"failed to start pinned Unreal AutomationTool; work={work_dir}: {exc}"
                ) from exc
        finally:
            try:
                if gate_stream is not None:
                    gate_stream.close()
            finally:
                if process is None:
                    process_owner.close()
                elif process.poll() is None:
                    process_owner.terminate(
                        process,
                        timeout_s=_PROCESS_CLEANUP_TIMEOUT_S,
                    )
                else:
                    process_owner.close_after_exit()
        if returncode != 0:
            raise DistributionError(f"Unreal BuildCookRun failed with exit code {returncode}; log={uat_log}")

        expected_executable = archive_root / "Windows/RobotSimUE.exe"
        if not expected_executable.is_file():
            raise DistributionError(
                "Unreal BuildCookRun exited successfully without the required "
                f"Shipping executable: {expected_executable}"
            )
        with expected_executable.open("rb") as executable_stream:
            if executable_stream.read(2) != b"MZ":
                raise DistributionError(
                    f"RobotSimUE.exe is not a Windows PE executable; candidate remains isolated at {work_dir}"
                )
        shipping_executable = archive_root / _SHIPPING_EXECUTABLE_RELATIVE_PATH
        if not shipping_executable.is_file():
            raise DistributionError(
                "Unreal BuildCookRun exited successfully without the compiled "
                f"Shipping runtime: {shipping_executable}"
            )
        with shipping_executable.open("rb") as executable_stream:
            if executable_stream.read(2) != b"MZ":
                raise DistributionError(
                    "RobotSimUE Shipping runtime is not a Windows PE executable; "
                    f"candidate remains isolated at {work_dir}"
                )
        product_runtime = archive_root / _PRODUCT_RUNTIME_RELATIVE_PATH
        if product_runtime.exists():
            raise DistributionError(
                "Unreal package unexpectedly contains the reserved product runtime "
                f"path: {product_runtime}"
            )
        try:
            shutil.copy2(shipping_executable, product_runtime)
        except OSError as exc:
            raise DistributionError(
                f"could not materialize the RobotSimUE product runtime: {exc}"
            ) from exc
        if product_runtime.stat().st_size != shipping_executable.stat().st_size:
            raise DistributionError(
                "RobotSimUE product runtime does not match the compiled Shipping "
                f"binary; candidate remains isolated at {work_dir}"
            )
        artifacts = self._artifact_records(archive_root, work_dir)
        packaged_manifest = {
            **manifest,
            "state": "packaged",
            "claims": {
                "cook_completed": True,
                "stage_completed": True,
                "package_completed": True,
                "shipping_build_produced": True,
                "packaged_smoke_passed": False,
            },
            "artifacts": artifacts,
        }
        work_manifest = work_dir / "distribution.manifest.json"
        work_manifest.write_bytes(self._canonical_json(packaged_manifest))
        release_dir.parent.mkdir(parents=True, exist_ok=True)
        try:
            work_dir.replace(release_dir)
        except OSError as exc:
            raise DistributionError(f"could not atomically publish Windows distribution: {exc}") from exc
        public_release_dir = self._public_release_path(release_dir, artifacts)
        return DistributionResult(
            manifest_path=public_release_dir / "distribution.manifest.json",
            manifest=packaged_manifest,
            unreal_root=unreal_root,
        )

    @staticmethod
    def _public_release_path(
        release_dir: Path,
        artifacts: list[dict[str, Any]],
    ) -> Path:
        """Return a readable Windows path when a release crosses ``MAX_PATH``.

        AutomationTool writes into a deliberately short candidate directory.
        A nested artifact can hit
        the legacy 260-character boundary even though the atomic rename itself
        succeeds.  Returning the extended-length form keeps callers able to
        verify the exact files named by the manifest.
        """

        if os.name != "nt":
            return release_dir
        paths = [release_dir / "distribution.manifest.json"]
        paths.extend(release_dir / str(item["path"]) for item in artifacts)
        if all(len(str(path)) < 260 for path in paths):
            return release_dir
        raw = str(release_dir)
        if raw.startswith("\\\\?\\"):
            return release_dir
        if raw.startswith("\\\\"):
            return Path("\\\\?\\UNC\\" + raw.lstrip("\\"))
        return Path("\\\\?\\" + raw)

    def _materialize_command(
        self,
        template: list[str],
        *,
        run_uat: Path,
        project_path: Path,
        archive_root: Path,
        stage_root: Path,
        xml_config_cache: Path,
    ) -> list[str]:
        replacements = {
            "${UNREAL_ROOT}/Engine/Build/BatchFiles/RunUAT.bat": str(run_uat),
            "${REPO_ROOT}/sim/runtime/visual/RobotSimUE/RobotSimUE.uproject": str(project_path),
            "${ARCHIVE_ROOT}": str(archive_root),
            "${STAGE_ROOT}": str(stage_root),
            "${XML_CONFIG_CACHE}": str(xml_config_cache),
        }
        materialized = []
        for argument in template:
            value = argument
            for placeholder, replacement in replacements.items():
                value = value.replace(placeholder, replacement)
            materialized.append(value)
        if any("${" in argument for argument in materialized):
            raise DistributionError("internal UAT command contains an unresolved placeholder")
        return materialized

    @staticmethod
    def _prepare_writable_unreal_root(
        *,
        unreal_root: Path,
        runtime_root: Path,
        patched_epicgames_build: Path,
    ) -> Path:
        """Create an installed-semantic Engine view inside the owned runtime tree.

        UE's Launcher marker must remain present so AutomationTool consumes Epic's
        precompiled engine rules and binaries.  The only binary overlay is a
        workspace-built ``EpicGames.Build.dll`` that adds one fixed user-settings
        environment override. Everything else remains linked to the pinned install.
        """

        source_engine = unreal_root / "Engine"
        source_build = source_engine / "Build"
        if not (source_build / "Build.version").is_file():
            raise DistributionError(f"Unreal Build.version is missing: {source_build}")
        if not (source_build / "InstalledBuild.txt").is_file():
            raise DistributionError(
                f"pinned Unreal installation has no InstalledBuild marker: {source_build}"
            )
        if not patched_epicgames_build.is_file():
            raise DistributionError(
                f"EpicGames.Build compatibility patch is missing: {patched_epicgames_build}"
            )

        writable_unreal_root = runtime_root / "r"
        writable_engine = writable_unreal_root / "Engine"
        writable_build = writable_engine / "Build"
        try:
            writable_engine.mkdir(parents=True, exist_ok=False)
            shutil.copytree(source_build, writable_build)
            for item in source_engine.iterdir():
                if item.name.casefold() in {"build", "binaries", "intermediate"}:
                    continue
                destination = writable_engine / item.name
                if item.is_dir():
                    destination.symlink_to(item, target_is_directory=True)
                elif item.is_file():
                    WindowsDistribution._hardlink_or_copy(str(item), str(destination))

            WindowsDistribution._prepare_unreal_intermediate_view(
                source_engine=source_engine,
                writable_engine=writable_engine,
            )
            WindowsDistribution._prepare_unreal_binaries_view(
                source_engine=source_engine,
                writable_engine=writable_engine,
                patched_epicgames_build=patched_epicgames_build,
            )
        except OSError as exc:
            raise DistributionError(
                f"failed to create isolated Unreal Engine view at {writable_unreal_root}: {exc}"
            ) from exc

        if not (writable_build / "InstalledBuild.txt").is_file():
            raise DistributionError("isolated Unreal Engine view lost InstalledBuild.txt")
        return writable_unreal_root

    @staticmethod
    def _prepare_unreal_intermediate_view(
        *, source_engine: Path, writable_engine: Path
    ) -> None:
        source_intermediate = source_engine / "Intermediate"
        if not source_intermediate.is_dir():
            raise DistributionError(
                f"Unreal Intermediate directory is missing: {source_intermediate}"
            )
        writable_intermediate = writable_engine / "Intermediate"
        writable_intermediate.mkdir(parents=True, exist_ok=False)
        for item in source_intermediate.iterdir():
            if item.name.casefold() == "build":
                continue
            destination = writable_intermediate / item.name
            if item.is_dir():
                destination.symlink_to(item, target_is_directory=True)
            elif item.is_file():
                WindowsDistribution._hardlink_or_copy(str(item), str(destination))

        source_build = source_intermediate / "Build"
        if not source_build.is_dir():
            raise DistributionError(
                f"Unreal precompiled Intermediate/Build is missing: {source_build}"
            )
        writable_build = writable_intermediate / "Build"
        writable_build.mkdir(parents=True, exist_ok=False)
        for item in source_build.iterdir():
            destination = writable_build / item.name
            if item.is_dir():
                destination.symlink_to(item, target_is_directory=True)
            elif item.is_file():
                WindowsDistribution._hardlink_or_copy(str(item), str(destination))

    @staticmethod
    def _prepare_unreal_binaries_view(
        *,
        source_engine: Path,
        writable_engine: Path,
        patched_epicgames_build: Path,
    ) -> None:
        source_binaries = source_engine / "Binaries"
        source_dotnet = source_binaries / "DotNET"
        if not source_dotnet.is_dir():
            raise DistributionError(f"Unreal DotNET binaries are missing: {source_dotnet}")
        writable_binaries = writable_engine / "Binaries"
        writable_binaries.mkdir(parents=True, exist_ok=False)
        for item in source_binaries.iterdir():
            if item.name.casefold() == "dotnet":
                continue
            destination = writable_binaries / item.name
            if item.is_dir():
                destination.symlink_to(item, target_is_directory=True)
            elif item.is_file():
                WindowsDistribution._hardlink_or_copy(str(item), str(destination))

        writable_dotnet = writable_binaries / "DotNET"
        writable_dotnet.mkdir(parents=True, exist_ok=False)
        patched_names = {path.name.casefold() for path in _EPICGAMES_BUILD_BINARY_DIRS}
        for item in source_dotnet.iterdir():
            if item.name.casefold() in patched_names:
                continue
            destination = writable_dotnet / item.name
            if item.is_dir():
                destination.symlink_to(item, target_is_directory=True)
            elif item.is_file():
                WindowsDistribution._hardlink_or_copy(str(item), str(destination))

        for relative_directory in _EPICGAMES_BUILD_BINARY_DIRS:
            source_directory = source_engine.parent / relative_directory
            if not source_directory.is_dir():
                raise DistributionError(
                    f"Unreal managed tool directory is missing: {source_directory}"
                )
            destination_directory = writable_engine.parent / relative_directory
            destination_directory.mkdir(parents=True, exist_ok=False)
            for item in source_directory.iterdir():
                destination = destination_directory / item.name
                if item.name.casefold() == "epicgames.build.dll":
                    shutil.copy2(
                        WindowsDistribution._extended_windows_path(
                            str(patched_epicgames_build)
                        ),
                        WindowsDistribution._extended_windows_path(str(destination)),
                    )
                elif item.is_dir():
                    destination.symlink_to(item, target_is_directory=True)
                elif item.is_file():
                    WindowsDistribution._hardlink_or_copy(str(item), str(destination))

    @staticmethod
    def _patch_epicgames_build_source(source: str) -> str:
        """Add the two fixed early overrides required by the isolated UE view."""

        environment_names = (
            "LINGTU_UNREAL_CONTENT_ROOT_DIRECTORY",
            "LINGTU_UNREAL_USER_SETTING_DIRECTORY",
        )
        if any(name in source for name in environment_names):
            raise DistributionError(
                "EpicGames.Build source already contains a LingTu compatibility override"
            )

        def insert(
            value: str,
            *,
            pattern: re.Pattern[str],
            method_name: str,
            body: tuple[str, ...],
        ) -> str:
            matches = list(pattern.finditer(value))
            if len(matches) != 1:
                raise DistributionError(
                    f"EpicGames.Build {method_name} source anchor did not match exactly once"
                )
            match = matches[0]
            indent = match.group("indent") + "\t"
            override = "".join(f"{indent}{line}\n" for line in body) + "\n"
            return value[: match.end()] + override + value[match.end() :]

        patched = insert(
            source,
            pattern=_EPICGAMES_BUILD_ROOT_PATTERN,
            method_name="FindRootDirectory",
            body=(
                "string? overrideDirectory = Environment.GetEnvironmentVariable("
                '"LINGTU_UNREAL_CONTENT_ROOT_DIRECTORY");',
                "if (!String.IsNullOrWhiteSpace(overrideDirectory))",
                "{",
                "\treturn DirectoryReference.FindCorrectCase(new DirectoryReference("
                "Path.GetFullPath(overrideDirectory)));",
                "}",
            ),
        )
        return insert(
            patched,
            pattern=_EPICGAMES_BUILD_USER_SETTINGS_PATTERN,
            method_name="GetUserSettingDirectory",
            body=(
                "string? overrideDirectory = Environment.GetEnvironmentVariable("
                '"LINGTU_UNREAL_USER_SETTING_DIRECTORY");',
                "if (!String.IsNullOrWhiteSpace(overrideDirectory))",
                "{",
                "\treturn new DirectoryReference(overrideDirectory);",
                "}",
            ),
        )

    @staticmethod
    def _build_epicgames_build_patch(
        *, unreal_root: Path, runtime_root: Path
    ) -> Path:
        """Rebuild EpicGames.Build from the pinned UE source in an owned directory."""

        source_root = unreal_root / _EPICGAMES_BUILD_SOURCE
        metadata_source = unreal_root / _EPICGAMES_BUILD_METADATA
        dotnet = unreal_root / _UNREAL_BUNDLED_DOTNET
        ubt_binary_dir = unreal_root / "Engine/Binaries/DotNET/UnrealBuildTool"
        if not source_root.is_dir():
            raise DistributionError(
                f"EpicGames.Build source directory is missing: {source_root}"
            )
        if not metadata_source.is_file():
            raise DistributionError(
                f"Unreal shared assembly metadata is missing: {metadata_source}"
            )
        if not dotnet.is_file():
            raise DistributionError(f"Unreal bundled .NET host is missing: {dotnet}")
        for dependency in _EPICGAMES_BUILD_REFERENCE_NAMES:
            if not (ubt_binary_dir / dependency).is_file():
                raise DistributionError(
                    f"EpicGames.Build reference is missing: {ubt_binary_dir / dependency}"
                )

        patch_root = runtime_root / "EngineCompatibilityPatch"
        copied_source_root = patch_root / "Source/EpicGames.Build"
        copied_source_root.mkdir(parents=True, exist_ok=False)
        source_files = sorted(
            (
                path
                for path in source_root.rglob("*.cs")
                if not {"bin", "obj"}.intersection(
                    part.casefold() for part in path.relative_to(source_root).parts
                )
            ),
            key=lambda path: path.as_posix(),
        )
        if not source_files:
            raise DistributionError(f"EpicGames.Build contains no C# source: {source_root}")
        for source_file in source_files:
            destination = copied_source_root / source_file.relative_to(source_root)
            destination.parent.mkdir(parents=True, exist_ok=True)
            # Launcher source files carry a read-only Windows attribute. The
            # isolated copy is intentionally writable because Unreal.cs is the
            # one source file transformed below.
            shutil.copyfile(source_file, destination)

        copied_unreal_source = copied_source_root / "Unreal.cs"
        if not copied_unreal_source.is_file():
            raise DistributionError(
                f"EpicGames.Build Unreal.cs is missing: {source_root / 'Unreal.cs'}"
            )
        try:
            original = copied_unreal_source.read_text(encoding="utf-8")
        except OSError as exc:
            raise DistributionError(
                f"could not read EpicGames.Build Unreal.cs: {copied_unreal_source}"
            ) from exc
        copied_unreal_source.write_text(
            WindowsDistribution._patch_epicgames_build_source(original),
            encoding="utf-8",
            newline="\n",
        )
        shutil.copyfile(metadata_source, patch_root / "Source/MetaData.cs")
        project_path = patch_root / "EpicGames.Build.LingTuPatch.csproj"
        project_path.write_text(
            _EPICGAMES_BUILD_PATCH_PROJECT,
            encoding="utf-8",
            newline="\n",
        )
        output_dir = patch_root / "out"
        log_path = patch_root / "build.log"
        environment = dict(os.environ)
        environment.update(
            {
                "DOTNET_CLI_HOME": str(patch_root / "DotNetHome"),
                "NUGET_PACKAGES": str(patch_root / "NuGetPackages"),
                "DOTNET_SKIP_FIRST_TIME_EXPERIENCE": "1",
                "DOTNET_CLI_TELEMETRY_OPTOUT": "1",
                "DOTNET_CLI_WORKLOAD_UPDATE_NOTIFY_DISABLE": "1",
                "MSBuildEnableWorkloadResolver": "false",
            }
        )
        Path(environment["DOTNET_CLI_HOME"]).mkdir(parents=True, exist_ok=True)
        Path(environment["NUGET_PACKAGES"]).mkdir(parents=True, exist_ok=True)
        command = [
            str(dotnet),
            "build",
            str(project_path),
            "-c",
            "Release",
            "--output",
            str(output_dir),
            "--ignore-failed-sources",
            "-v",
            "minimal",
            f"-p:UbtBinaryDir={ubt_binary_dir}",
        ]
        worker_command = [
            sys.executable,
            "-I",
            "-c",
            _OWNED_PROCESS_WORKER_SOURCE,
            *command,
        ]
        process_owner = ProcessTreeOwner()
        process: subprocess.Popen[bytes] | None = None
        gate_stream: IO[bytes] | None = None
        try:
            with log_path.open("wb") as log_stream:
                process = subprocess.Popen(  # noqa: S603
                    worker_command,
                    cwd=patch_root,
                    stdin=subprocess.PIPE,
                    stdout=log_stream,
                    stderr=subprocess.STDOUT,
                    shell=False,
                    env=environment,
                    **process_owner.popen_options(),
                )
                gate_stream = process.stdin
                if gate_stream is None:
                    raise DistributionError(
                        "EpicGames.Build patch worker gate was not created"
                    )
                process_owner.attach(process)
                gate_stream.write(b"\n")
                gate_stream.flush()
                gate_stream.close()
                gate_stream = None
                try:
                    returncode = process.wait(timeout=10 * 60.0)
                except subprocess.TimeoutExpired as exc:
                    raise DistributionError(
                        f"EpicGames.Build compatibility patch timed out; log={log_path}"
                    ) from exc
        except OSError as exc:
            raise DistributionError(
                f"could not build EpicGames.Build compatibility patch; log={log_path}: {exc}"
            ) from exc
        finally:
            try:
                if gate_stream is not None:
                    gate_stream.close()
            finally:
                if process is None:
                    process_owner.close()
                elif process.poll() is None:
                    process_owner.terminate(
                        process,
                        timeout_s=_PROCESS_CLEANUP_TIMEOUT_S,
                    )
                else:
                    process_owner.close_after_exit()

        if returncode != 0:
            raise DistributionError(
                "EpicGames.Build compatibility patch failed with exit code "
                f"{returncode}; log={log_path}"
            )
        output = output_dir / "EpicGames.Build.dll"
        if not output.is_file() or output.stat().st_size == 0:
            raise DistributionError(
                f"EpicGames.Build compatibility patch produced no assembly; log={log_path}"
            )
        with output.open("rb") as stream:
            if stream.read(2) != b"MZ":
                raise DistributionError(
                    f"EpicGames.Build compatibility output is not a PE assembly: {output}"
                )
        return output

    @staticmethod
    def _hardlink_or_copy(source: str, destination: str) -> str:
        source_path = WindowsDistribution._extended_windows_path(source)
        destination_path = WindowsDistribution._extended_windows_path(destination)
        try:
            os.link(source_path, destination_path)
            return destination
        except OSError:
            shutil.copy2(source_path, destination_path)
            return destination

    @staticmethod
    def _extended_windows_path(path: str) -> str:
        if os.name != "nt":
            return path
        absolute = os.path.abspath(path)
        if absolute.startswith("\\\\?\\"):
            return absolute
        if absolute.startswith("\\\\"):
            return "\\\\?\\UNC\\" + absolute[2:]
        return "\\\\?\\" + absolute

    def _artifact_records(self, archive_root: Path, work_dir: Path) -> list[dict[str, Any]]:
        records = []
        for path in sorted(archive_root.rglob("*"), key=lambda item: item.as_posix()):
            if not path.is_file():
                continue
            resolved = path.resolve()
            try:
                resolved.relative_to(archive_root)
            except ValueError as exc:
                raise DistributionError(f"packaged artifact escapes archive: {path}") from exc
            records.append(
                {
                    "path": resolved.relative_to(work_dir).as_posix(),
                    "bytes": resolved.stat().st_size,
                }
            )
        if not records:
            raise DistributionError("Unreal package archive contains no artifacts")
        return records

    @staticmethod
    def _canonical_json(value: object) -> bytes:
        return (json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "\n").encode("utf-8")

    def _read_policy(self, policy_path: Path) -> dict[str, Any]:
        if not policy_path.is_file():
            raise DistributionError(f"trusted distribution policy is missing: {policy_path}")
        try:
            policy = json.loads(policy_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise DistributionError(f"trusted distribution policy is invalid: {exc}") from exc
        if not isinstance(policy, dict):
            raise DistributionError("trusted distribution policy must be a JSON object")
        if policy.get("schema") != "lingtu.sim.windows-distribution-policy.v1":
            raise DistributionError("unsupported Windows distribution policy schema")
        self._assert_exact_keys(
            policy,
            {"schema", "product", "unreal", "host_toolchain"},
            "distribution policy",
        )
        for section in ("product", "unreal", "host_toolchain"):
            if not isinstance(policy.get(section), dict):
                raise DistributionError(f"distribution policy is missing object: {section}")
        product = policy["product"]
        unreal = policy["unreal"]
        host = policy["host_toolchain"]
        self._assert_exact_keys(
            product,
            {"id", "project", "target", "platform", "configuration", "maps"},
            "distribution product policy",
        )
        self._validate_product_policy(product)
        self._assert_exact_keys(
            unreal,
            {"version", "changelist", "branch", "evidence"},
            "Unreal policy",
        )
        self._assert_exact_keys(
            host,
            {"msvc", "windows_sdk", "unreal_auto_sdk"},
            "host toolchain policy",
        )
        self._assert_exact_keys(
            self._mapping(host, "msvc"),
            {
                "version",
                "version_file",
                "compiler",
            },
            "MSVC policy",
        )
        self._assert_exact_keys(
            self._mapping(host, "windows_sdk"),
            {
                "version",
                "resource_compiler",
                "library_evidence",
            },
            "Windows SDK policy",
        )
        self._assert_exact_keys(
            self._mapping(host, "unreal_auto_sdk"),
            {"netfx_header"},
            "Unreal AutoSDK policy",
        )
        evidence = unreal.get("evidence")
        if not isinstance(evidence, list):
            raise DistributionError("Unreal evidence must be an array")
        for index, item in enumerate(evidence):
            if not isinstance(item, dict):
                raise DistributionError(f"Unreal evidence[{index}] must be an object")
            self._assert_exact_keys(item, {"path"}, f"Unreal evidence[{index}]")
        evidence_paths = [item["path"] for item in evidence]
        if len(evidence_paths) != len(_REQUIRED_UNREAL_EVIDENCE) or set(evidence_paths) != _REQUIRED_UNREAL_EVIDENCE:
            raise DistributionError("Unreal policy must contain the complete pinned evidence set")
        return policy

    @staticmethod
    def _validate_product_policy(product: Mapping[str, Any]) -> None:
        fixed_values = {
            "id": "robotsimue",
            "project": "sim/runtime/visual/RobotSimUE/RobotSimUE.uproject",
            "target": "RobotSimUE",
            "platform": "Win64",
            "configuration": "Shipping",
        }
        for field, expected in fixed_values.items():
            if product.get(field) != expected:
                raise DistributionError(f"Windows distribution {field} must be {expected}")

    @staticmethod
    def _assert_exact_keys(value: Mapping[str, Any], expected: set[str], context: str) -> None:
        actual = set(value)
        unexpected = sorted(actual - expected)
        if unexpected:
            raise DistributionError(f"{context} has unexpected fields: {', '.join(unexpected)}")
        missing = sorted(expected - actual)
        if missing:
            raise DistributionError(f"{context} is missing required fields: {', '.join(missing)}")

    def _resolve_repo_file(self, relative: object) -> Path:
        if not isinstance(relative, str) or not relative.strip():
            raise DistributionError("repository file path must be a non-empty string")
        candidate = Path(relative)
        if candidate.is_absolute() or ".." in candidate.parts:
            raise DistributionError(f"repository file path is not trusted: {relative}")
        resolved = (self._repo_root / candidate).resolve()
        try:
            resolved.relative_to(self._repo_root)
        except ValueError as exc:
            raise DistributionError(f"repository file path escapes repository: {relative}") from exc
        if not resolved.is_file():
            raise DistributionError(f"required repository file is missing: {relative}")
        return resolved

    def _trusted_root(self, variable: str, fallback_candidates: list[Path] | None = None) -> Path:
        raw = self._environment.get(variable, "").strip()
        if raw:
            root = Path(raw).resolve()
            if not root.is_dir():
                raise DistributionError(f"candidate root does not exist for {variable}: {root}")
            return root
        candidates = self._existing_unique_roots(fallback_candidates or [])
        if candidates:
            return candidates[0]
        raise DistributionError(f"no trusted candidate root was detected for {variable}")

    def _detect_unreal(self, policy: Mapping[str, Any]) -> tuple[Path, dict[str, Any]]:
        candidates = self._unreal_candidates(policy)
        if not candidates:
            raise DistributionError(
                "no Unreal installation candidate was detected from the trusted "
                "environment, Epic Launcher inventory, or standard install roots"
            )
        failures = []
        for root in candidates:
            try:
                return root, self._validate_unreal(root, policy)
            except DistributionError as exc:
                failures.append(f"{root}: {exc}")
        raise DistributionError("no detected Unreal installation matched the pinned policy: " + "; ".join(failures))

    def _unreal_candidates(self, policy: Mapping[str, Any]) -> list[Path]:
        explicit = self._environment.get("LINGTU_UNREAL_CANDIDATE_ROOT", "").strip()
        if explicit:
            return [Path(explicit).resolve()]

        raw_candidates: list[Path] = []
        for variable in ("UE_ROOT", "UNREAL_ENGINE_ROOT"):
            value = self._environment.get(variable, "").strip()
            if value:
                raw_candidates.append(Path(value))
        program_data = self._environment.get("ProgramData", "").strip()
        if program_data:
            inventory = Path(program_data) / "Epic/UnrealEngineLauncher/LauncherInstalled.dat"
            if inventory.is_file():
                try:
                    payload = json.loads(inventory.read_text(encoding="utf-8-sig"))
                    for entry in payload.get("InstallationList", []):
                        if isinstance(entry, dict) and isinstance(entry.get("InstallLocation"), str):
                            raw_candidates.append(Path(entry["InstallLocation"]))
                except (OSError, json.JSONDecodeError):
                    pass

        version = str(policy.get("version", ""))
        major_minor = ".".join(version.split(".")[:2])
        for drive in self._windows_drive_roots():
            raw_candidates.extend(
                (
                    drive / f"Program Files/Epic Games/UE_{major_minor}",
                    drive / f"Program Files (x86)/Epic Games/UE_{major_minor}",
                )
            )
        return self._existing_unique_roots(raw_candidates)

    @staticmethod
    def _windows_drive_roots() -> list[Path]:
        if os.name != "nt":
            return [Path("/")]
        try:
            import ctypes

            mask = int(ctypes.windll.kernel32.GetLogicalDrives())
        except (AttributeError, OSError, ValueError):
            return [Path("C:/"), Path("D:/")]
        return [Path(f"{chr(ord('A') + index)}:/") for index in range(26) if mask & (1 << index)]

    @staticmethod
    def _existing_unique_roots(candidates: list[Path]) -> list[Path]:
        roots = []
        seen = set()
        for candidate in candidates:
            try:
                resolved = candidate.resolve()
            except OSError:
                continue
            key = os.path.normcase(str(resolved))
            if key in seen or not resolved.is_dir():
                continue
            seen.add(key)
            roots.append(resolved)
        return roots

    def _validate_unreal(self, root: Path, policy: Mapping[str, Any]) -> dict[str, Any]:
        evidence = policy.get("evidence")
        if not isinstance(evidence, list) or not evidence:
            raise DistributionError("Unreal policy has no pinned evidence")
        records = []
        for item in evidence:
            records.append(self._validate_evidence(root, item, "Unreal"))

        build_version_path = root / "Engine/Build/Build.version"
        try:
            build_version = json.loads(build_version_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise DistributionError(f"Unreal Build.version is invalid: {exc}") from exc
        detected_version = ".".join(
            str(build_version.get(key)) for key in ("MajorVersion", "MinorVersion", "PatchVersion")
        )
        expected = str(policy.get("version", ""))
        if detected_version != expected:
            raise DistributionError(f"Unreal version mismatch: expected {expected}, detected {detected_version}")
        if build_version.get("Changelist") != policy.get("changelist"):
            raise DistributionError("Unreal changelist does not match the pinned policy")
        if build_version.get("BranchName") != policy.get("branch"):
            raise DistributionError("Unreal branch does not match the pinned policy")
        return {
            "version": expected,
            "changelist": policy["changelist"],
            "branch": policy["branch"],
            "evidence": records,
        }

    def _validate_host_toolchain(self, policy: Mapping[str, Any]) -> dict[str, Any]:
        msvc_policy = self._mapping(policy, "msvc")
        msvc_root = self._trusted_root("LINGTU_MSVC_CANDIDATE_ROOT", self._msvc_candidates())
        version_path = self._path_under(msvc_root, msvc_policy.get("version_file"), "MSVC")
        version_record = self._evidence_record(
            version_path,
            msvc_root,
            "MSVC version file",
        )
        detected_version = version_path.read_text(encoding="utf-8").strip()
        if detected_version != msvc_policy.get("version"):
            raise DistributionError("MSVC toolset version does not match the pinned policy")
        compiler = self._path_under(msvc_root, msvc_policy.get("compiler"), "MSVC")
        compiler_record = self._evidence_record(
            compiler,
            msvc_root,
            "MSVC compiler",
        )

        sdk_policy = self._mapping(policy, "windows_sdk")
        sdk_root = self._trusted_root("LINGTU_WINDOWS_SDK_CANDIDATE_ROOT", self._windows_sdk_candidates())
        resource_compiler = self._path_under(sdk_root, sdk_policy.get("resource_compiler"), "Windows SDK")
        rc_record = self._evidence_record(
            resource_compiler,
            sdk_root,
            "Windows SDK resource compiler",
        )
        library = self._path_under(sdk_root, sdk_policy.get("library_evidence"), "Windows SDK")
        lib_record = self._evidence_record(
            library,
            sdk_root,
            "Windows SDK library",
        )

        auto_policy = self._mapping(policy, "unreal_auto_sdk")
        auto_root = self._trusted_root("UE_SDKS_ROOT", self._auto_sdk_candidates())
        netfx = self._path_under(auto_root, auto_policy.get("netfx_header"), "Unreal AutoSDK")
        netfx_record = self._evidence_record(
            netfx,
            auto_root,
            "Unreal AutoSDK .NET Framework header",
        )
        return {
            "msvc": {
                "version": detected_version,
                "evidence": [version_record, compiler_record],
            },
            "windows_sdk": {
                "version": sdk_policy.get("version"),
                "evidence": [rc_record, lib_record],
            },
            "unreal_auto_sdk": {"evidence": [netfx_record]},
        }

    def _msvc_candidates(self) -> list[Path]:
        candidates = []
        vs_install = self._environment.get("VSINSTALLDIR", "").strip()
        if vs_install:
            candidates.append(Path(vs_install))
        for drive in self._windows_drive_roots():
            for program_files in ("Program Files", "Program Files (x86)"):
                base = drive / program_files / "Microsoft Visual Studio/2022"
                if base.is_dir():
                    candidates.extend(sorted(path for path in base.iterdir() if path.is_dir()))
        return candidates

    def _windows_sdk_candidates(self) -> list[Path]:
        candidates = []
        sdk_env = self._environment.get("WindowsSdkDir", "").strip()
        if sdk_env:
            candidates.append(Path(sdk_env))
        for drive in self._windows_drive_roots():
            candidates.extend(
                (
                    drive / "Windows Kits/10",
                    drive / "Program Files (x86)/Windows Kits/10",
                    drive / "Program Files/Windows Kits/10",
                )
            )
        return candidates

    def _auto_sdk_candidates(self) -> list[Path]:
        return [drive / "Development/UnrealAutoSDK" for drive in self._windows_drive_roots()]

    @staticmethod
    def _mapping(parent: Mapping[str, Any], key: str) -> Mapping[str, Any]:
        value = parent.get(key)
        if not isinstance(value, dict):
            raise DistributionError(f"distribution policy is missing object: {key}")
        return value

    def _validate_evidence(self, root: Path, item: object, context: str) -> dict[str, Any]:
        if not isinstance(item, dict):
            raise DistributionError(f"{context} evidence must be an object")
        path = self._path_under(root, item.get("path"), context)
        return self._evidence_record(path, root, context)

    def _evidence_record(
        self,
        path: Path,
        root: Path,
        context: str,
    ) -> dict[str, Any]:
        return {
            "path": path.relative_to(root).as_posix(),
            "bytes": path.stat().st_size,
        }

    def _path_under(self, root: Path, relative: object, context: str) -> Path:
        if not isinstance(relative, str) or not relative.strip():
            raise DistributionError(f"{context} evidence path must be non-empty")
        candidate = Path(relative)
        if candidate.is_absolute() or ".." in candidate.parts:
            raise DistributionError(f"{context} evidence path is not trusted: {relative}")
        path = (root / candidate).resolve()
        try:
            path.relative_to(root)
        except ValueError as exc:
            raise DistributionError(f"{context} evidence escapes its root") from exc
        if not path.is_file():
            raise DistributionError(f"{context} evidence file is missing: {relative}")
        return path

    def _validate_project(self, project_path: Path, product: Mapping[str, Any]) -> None:
        try:
            project = json.loads(project_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise DistributionError(f"RobotSimUE project is invalid: {exc}") from exc
        if project.get("EngineAssociation") not in (None, ""):
            raise DistributionError("RobotSimUE must not select an unpinned EngineAssociation")
        target = product.get("target")
        modules = project.get("Modules", [])
        if not isinstance(target, str) or not any(
            isinstance(module, dict) and module.get("Name") == target for module in modules
        ):
            raise DistributionError("RobotSimUE target is not declared by the project")
        plugins = project.get("Plugins", [])
        if not any(
            isinstance(plugin, dict) and plugin.get("Name") == "LingTuSim" and plugin.get("Enabled") is True
            for plugin in plugins
        ):
            raise DistributionError("RobotSimUE must enable the LingTuSim runtime plugin")
        self._validate_runtime_plugin(project_path.parent)
        target_file = project_path.parent / f"Source/{target}.Target.cs"
        if not target_file.is_file():
            raise DistributionError(f"RobotSimUE game target is missing: {target_file}")
        maps = product.get("maps")
        if not isinstance(maps, list) or not maps:
            raise DistributionError("Windows distribution must cook at least one explicit map")
        for asset_path in maps:
            if not isinstance(asset_path, str) or not asset_path.startswith("/Game/"):
                raise DistributionError(f"invalid Unreal map asset path: {asset_path}")
            map_file = project_path.parent / "Content" / f"{asset_path[6:]}.umap"
            if not map_file.is_file():
                raise DistributionError(f"required Unreal map is missing: {asset_path}")

    @staticmethod
    def _validate_runtime_plugin(project_root: Path) -> None:
        descriptor_path = project_root / "Plugins/LingTuSim/LingTuSim.uplugin"
        if not descriptor_path.is_file():
            raise DistributionError("LingTuSim plugin descriptor is missing")
        try:
            descriptor = json.loads(descriptor_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise DistributionError(f"LingTuSim plugin descriptor is invalid: {exc}") from exc
        supported = descriptor.get("SupportedTargetPlatforms")
        if not isinstance(supported, list) or "Win64" not in supported:
            raise DistributionError("LingTuSim plugin must support Win64")
        modules = descriptor.get("Modules")
        if not isinstance(modules, list) or not modules:
            raise DistributionError("LingTuSim plugin declares no Shipping modules")
        for module in modules:
            if not isinstance(module, dict) or module.get("Type") != "Runtime":
                name = module.get("Name", "<unknown>") if isinstance(module, dict) else "<invalid>"
                raise DistributionError(f"LingTuSim module {name} must be Runtime for Shipping")

    @staticmethod
    def _phase_command_templates(product: Mapping[str, Any]) -> list[dict[str, Any]]:
        maps = "+".join(str(value) for value in product["maps"])
        common = [
            "${UNREAL_ROOT}/Engine/Build/BatchFiles/RunUAT.bat",
            "BuildCookRun",
            "-project=${REPO_ROOT}/sim/runtime/visual/RobotSimUE/RobotSimUE.uproject",
            "-XmlConfigCache=${XML_CONFIG_CACHE}",
            "-ubtargs=-UsePrecompiled -NoHotReloadFromIDE",
            f"-target={product['target']}",
            f"-platform={product['platform']}",
            f"-clientconfig={product['configuration']}",
            "-installed",
            "-distribution",
            "-nocompileuat",
            "-NoCompile",
        ]
        common_tail = [
            "-unattended",
            "-utf8output",
            "-noP4",
        ]
        build_cook = [
            *common,
            f"-MapsToCook={maps}",
            # The installed default graph starts a per-user Zen service even
            # when UE-LocalDataCachePath is isolated.  Cook must remain inside
            # the distribution worker's writable filesystem roots.
            "-ddc=InstalledNoZenLocalFallback",
            # UE still constructs its default Zen service settings for Cook
            # diagnostics even when the selected DDC graph contains no local
            # Zen node.  Force connect-only mode so it never attempts to
            # install or update files below the Windows user profile.
            "-NoZenAutoLaunch=127.0.0.1",
            # UE 5.8 defaults ProjectPackagingSettings.bUseZenStore to true.
            # NoZenAutoLaunch only changes service discovery; the cook commandlet
            # still needs its documented switch to select loose package output.
            "-AdditionalCookerOptions=-SkipZenStore "
            f"-DisablePlugins={','.join(_DEVELOPMENT_ONLY_UNREAL_PLUGINS)}",
            "-build",
            "-cook",
            *common_tail,
        ]
        stage_package_archive = [
            *common,
            "-skipbuild",
            "-skipcook",
            "-stage",
            "-stagingdirectory=${STAGE_ROOT}",
            "-pak",
            "-iostore",
            "-package",
            "-archive",
            "-archivedirectory=${ARCHIVE_ROOT}/Windows",
            "-prereqs",
            *common_tail,
        ]
        return [
            {
                "id": "build_cook",
                "content_root": "pinned_install",
                "command": build_cook,
            },
            {
                "id": "stage_package_archive",
                "content_root": "writable_projection",
                "command": stage_package_archive,
            },
        ]

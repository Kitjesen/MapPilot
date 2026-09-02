from __future__ import annotations

import hashlib
import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest

pytestmark = pytest.mark.skipif(os.name != "nt", reason="Windows-only build script")

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "build" / "build_slam_core_windows.ps1"
POWERSHELL = shutil.which("pwsh")


def _run_script(*arguments: str, env: dict[str, str] | None = None) -> subprocess.CompletedProcess[str]:
    assert POWERSHELL is not None
    return subprocess.run(  # noqa: S603 - executes the repository-owned build helper.
        [POWERSHELL, "-NoProfile", "-File", str(SCRIPT), *arguments],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
        env=env,
    )


def _run_staged_loader_helper(executable: Path, expected_usage: str) -> subprocess.CompletedProcess[str]:
    assert POWERSHELL is not None
    command = r"""
$tokens = $null
$errors = $null
$ast = [System.Management.Automation.Language.Parser]::ParseFile($env:LINGTU_TEST_SCRIPT, [ref]$tokens, [ref]$errors)
$function = $ast.Find({
    param($node)
    $node -is [System.Management.Automation.Language.FunctionDefinitionAst] -and
        $node.Name -eq 'Assert-StagedExecutableLoads'
}, $true)
if (-not $function) { throw 'Assert-StagedExecutableLoads was not found' }
Invoke-Expression $function.Extent.Text
Assert-StagedExecutableLoads $env:LINGTU_TEST_EXECUTABLE $env:LINGTU_TEST_USAGE
"""
    environment = os.environ.copy()
    environment["PATH"] = rf"C:\lingtu-forbidden-path;{environment.get('PATH', '')}"
    environment["LINGTU_TEST_SCRIPT"] = str(SCRIPT)
    environment["LINGTU_TEST_EXECUTABLE"] = str(executable)
    environment["LINGTU_TEST_USAGE"] = expected_usage
    return subprocess.run(  # noqa: S603 - executes the repository-owned helper function.
        [POWERSHELL, "-NoProfile", "-Command", command],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )


def _write_file(path: Path, text: str = "# test fixture\n") -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def test_staged_loader_smoke_uses_only_system_path_and_accepts_help_exit_two(tmp_path: Path) -> None:
    executable = tmp_path / "help-ok.cmd"
    executable.write_text(
        '@echo %PATH% | findstr /I /C:"lingtu-forbidden-path" >nul && exit /b 9\n'
        "@echo usage: staged-test\n"
        "@exit /b 2\n",
        encoding="utf-8",
    )

    completed = _run_staged_loader_helper(executable, "usage: staged-test")

    assert completed.returncode == 0, completed.stdout + completed.stderr


def test_staged_loader_smoke_reports_windows_loader_failure(tmp_path: Path) -> None:
    executable = tmp_path / "loader-failure.cmd"
    executable.write_text("@exit /b -1073741515\n", encoding="utf-8")

    completed = _run_staged_loader_helper(executable, "usage: unreachable")

    assert completed.returncode != 0
    assert "Windows loader failed" in completed.stdout + completed.stderr


def _make_dependency_prefixes(tmp_path: Path) -> tuple[Path, Path]:
    assert POWERSHELL is not None
    dependencies = tmp_path / "deps" / "x64-windows"
    cyclone = tmp_path / "cyclone" / "x64-windows"
    _write_file(dependencies / "share" / "eigen3" / "Eigen3Config.cmake")
    _write_file(dependencies / "share" / "pcl" / "PCLConfig.cmake")
    _write_file(dependencies / "share" / "yaml-cpp" / "yaml-cpp-config.cmake")
    _write_file(cyclone / "lib" / "cmake" / "CycloneDDS" / "CycloneDDSConfig.cmake")
    _write_file(
        cyclone / "lib" / "cmake" / "CycloneDDS" / "CycloneDDSConfigVersion.cmake",
        'set(PACKAGE_VERSION "11.0.1")\n',
    )
    (cyclone / "bin").mkdir(parents=True)
    _write_file(cyclone / "licenses" / "LICENSE", "fixture license\n")
    _write_file(cyclone / "licenses" / "NOTICE.md", "fixture notice\n")
    _write_file(cyclone / "lib" / "ddsc.lib")
    (cyclone / "evidence").mkdir(parents=True)
    shutil.copy2(
        ROOT / "scripts" / "build" / "locks" / "cyclonedds-windows-x64.json",
        cyclone / "evidence" / "source-lock.json",
    )
    _write_file(cyclone / "evidence" / "source.json", "{}\n")
    _write_file(cyclone / "evidence" / "toolchain.json", "{}\n")
    _write_file(cyclone / "evidence" / "build.json", "{}\n")
    license_path = cyclone / "licenses" / "LICENSE"
    license_sha1 = hashlib.sha1(license_path.read_bytes(), usedforsecurity=False).hexdigest()
    verification_code = hashlib.sha1(license_sha1.encode("ascii"), usedforsecurity=False).hexdigest()
    _write_file(
        cyclone / "evidence" / "sbom.spdx.json",
        json.dumps(
            {
                "spdxVersion": "SPDX-2.3",
                "packages": [
                    {
                        "SPDXID": "SPDXRef-Package-CycloneDDS",
                        "name": "Eclipse Cyclone DDS",
                        "versionInfo": "11.0.1",
                        "licenseConcluded": "EPL-2.0 OR BSD-3-Clause",
                        "packageVerificationCode": {"packageVerificationCodeValue": verification_code},
                    }
                ],
                "files": [
                    {
                        "SPDXID": "SPDXRef-File-1",
                        "fileName": "./licenses/LICENSE",
                        "checksums": [
                            {"algorithm": "SHA1", "checksumValue": license_sha1},
                            {
                                "algorithm": "SHA256",
                                "checksumValue": hashlib.sha256(license_path.read_bytes()).hexdigest(),
                            },
                        ],
                    }
                ],
                "relationships": [
                    {
                        "spdxElementId": "SPDXRef-DOCUMENT",
                        "relationshipType": "DESCRIBES",
                        "relatedSpdxElement": "SPDXRef-Package-CycloneDDS",
                    },
                    {
                        "spdxElementId": "SPDXRef-Package-CycloneDDS",
                        "relationshipType": "CONTAINS",
                        "relatedSpdxElement": "SPDXRef-File-1",
                    },
                ],
            }
        ),
    )
    shutil.copy2(POWERSHELL, cyclone / "bin" / "idlc.exe")
    shutil.copy2(Path("C:/Windows/System32/kernel32.dll"), cyclone / "bin" / "ddsc.dll")
    manifest_path = cyclone / "evidence" / "files.sha256"
    manifest_lines = []
    for path in sorted(path for path in cyclone.rglob("*") if path.is_file()):
        relative = path.relative_to(cyclone).as_posix()
        if relative in {"evidence/files.sha256", "evidence/sdk-receipt.json"}:
            continue
        manifest_lines.append(f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {relative}\n")
    manifest_path.write_text("".join(manifest_lines), encoding="utf-8")
    receipt = {
        "schema_version": 1,
        "sdk": {"name": "CycloneDDS", "version": "11.0.1"},
        "source": {
            "repository": "https://github.com/eclipse-cyclonedds/cyclonedds.git",
            "tag": "11.0.1",
            "commit": "e54e991f75a3e67f8e628da3171122e36ea5b872",
            "tree": "56508d35826c362782fc8a388cad351a3d491f51",
        },
        "toolchain": {
            "generator": "Visual Studio 17 2022",
            "toolset": "v143",
            "architecture": "x64",
            "configuration": "Release",
            "msvc_runtime": "/MD",
        },
        "paths": {
            "license": "licenses/LICENSE",
            "notice": "licenses/NOTICE.md",
            "cmake_config": "lib/cmake/CycloneDDS/CycloneDDSConfig.cmake",
            "idlc": "bin/idlc.exe",
            "ddsc_dll": "bin/ddsc.dll",
            "ddsc_import_library": "lib/ddsc.lib",
        },
        "verification": {
            "result": "passed",
            "pe_x64": True,
            "dll_closure": "passed",
            "idl_smoke": "passed",
            "consumer_compile_link": "passed",
            "sanitized_dll_search": True,
        },
    }
    _write_file(cyclone / "evidence" / "sdk-receipt.json", json.dumps(receipt))
    return dependencies, cyclone


def _make_vcpkg_arguments(tmp_path: Path, dependencies: Path) -> list[str]:
    vcpkg_root = tmp_path / "vcpkg"
    install_root = dependencies.parent
    binary_cache = tmp_path / "vcpkg-cache"
    _write_file(vcpkg_root / ".git" / "HEAD", "9e593bb18ea69cc5095e012465dcd675a822ed0d\n")
    _write_file(vcpkg_root / "scripts" / "buildsystems" / "vcpkg.cmake")
    git_shim = vcpkg_root / "git-shim.cmd"
    git_shim.write_text(
        '@echo %* | findstr /C:"rev-parse HEAD" >nul && echo 9e593bb18ea69cc5095e012465dcd675a822ed0d\n'
        '@echo %* | findstr /C:"symbolic-ref" >nul && exit /b 1\n'
        "@exit /b 0\n",
        encoding="utf-8",
    )
    binary_cache.mkdir(exist_ok=True)
    return [
        "-VcpkgRoot",
        str(vcpkg_root),
        "-VcpkgInstallRoot",
        str(install_root),
        "-VcpkgBinaryCache",
        str(binary_cache),
        "-GitExecutable",
        str(git_shim),
    ]


def test_windows_slam_build_preflight_fails_closed_without_dependency_prefix(
    tmp_path: Path,
) -> None:
    completed = _run_script(
        "-CycloneDDSPrefix",
        str(tmp_path / "cyclone"),
        "-BuildDir",
        str(tmp_path / "build"),
        "-PreflightOnly",
    )

    assert completed.returncode != 0
    assert "DependencyPrefix" in (completed.stdout + completed.stderr)
    assert not (tmp_path / "build").exists()


def test_windows_slam_build_preflight_is_read_only_for_valid_x64_prefixes(
    tmp_path: Path,
) -> None:
    dependencies, cyclone = _make_dependency_prefixes(tmp_path)
    build_dir = tmp_path / "unused-build"

    completed = _run_script(
        "-DependencyPrefix",
        str(dependencies),
        "-CycloneDDSPrefix",
        str(cyclone),
        "-BuildDir",
        str(build_dir),
        *_make_vcpkg_arguments(tmp_path, dependencies),
        "-PreflightOnly",
    )

    assert completed.returncode == 0, completed.stdout + completed.stderr
    assert "Preflight passed" in completed.stdout
    assert not build_dir.exists()


def test_windows_slam_build_preflight_requires_pinned_vcpkg_inputs(
    tmp_path: Path,
) -> None:
    dependencies, cyclone = _make_dependency_prefixes(tmp_path)

    completed = _run_script(
        "-DependencyPrefix",
        str(dependencies),
        "-CycloneDDSPrefix",
        str(cyclone),
        "-BuildDir",
        str(tmp_path / "build"),
        "-PreflightOnly",
    )

    assert completed.returncode != 0
    assert "VcpkgRoot" in (completed.stdout + completed.stderr)


def test_windows_slam_build_preflight_requires_cmake_327(tmp_path: Path) -> None:
    dependencies, cyclone = _make_dependency_prefixes(tmp_path)
    fake_bin = tmp_path / "fake-bin"
    fake_bin.mkdir()
    (fake_bin / "cmake.cmd").write_text(
        "@echo cmake version 3.26.4\n@exit /b 0\n",
        encoding="utf-8",
    )
    environment = os.environ.copy()
    environment["PATH"] = f"{fake_bin};{environment['PATH']}"

    completed = _run_script(
        "-DependencyPrefix",
        str(dependencies),
        "-CycloneDDSPrefix",
        str(cyclone),
        "-BuildDir",
        str(tmp_path / "build"),
        *_make_vcpkg_arguments(tmp_path, dependencies),
        "-PreflightOnly",
        env=environment,
    )

    assert completed.returncode != 0
    assert "CMake 3.27 or newer" in (completed.stdout + completed.stderr)


def test_windows_slam_build_rejects_ambiguous_binary_cache_path(tmp_path: Path) -> None:
    dependencies, cyclone = _make_dependency_prefixes(tmp_path)
    vcpkg_arguments = _make_vcpkg_arguments(tmp_path, dependencies)
    ambiguous_cache = tmp_path / "cache;other"
    ambiguous_cache.mkdir()
    vcpkg_arguments[5] = str(ambiguous_cache)

    completed = _run_script(
        "-DependencyPrefix",
        str(dependencies),
        "-CycloneDDSPrefix",
        str(cyclone),
        "-BuildDir",
        str(tmp_path / "build"),
        *vcpkg_arguments,
        "-PreflightOnly",
    )

    assert completed.returncode != 0
    assert "comma or semicolon" in (completed.stdout + completed.stderr)


def test_windows_slam_build_rejects_fabricated_sdk_before_configure(
    tmp_path: Path,
) -> None:
    dependencies, cyclone = _make_dependency_prefixes(tmp_path)
    fake_bin = tmp_path / "fake-bin"
    fake_bin.mkdir()
    command_log = tmp_path / "commands.txt"
    (fake_bin / "cmake.cmd").write_text(
        '@if not "%1"=="--version" goto run\n'
        "@echo cmake version 3.31.6\n@exit /b 0\n:run\n"
        '@echo cmake %* binary=%VCPKG_BINARY_SOURCES%>> "%LINGTU_COMMAND_LOG%"\n@exit /b 0\n',
        encoding="utf-8",
    )
    (fake_bin / "ctest.cmd").write_text(
        '@echo ctest %* path=%PATH%>> "%LINGTU_COMMAND_LOG%"\n@exit /b 0\n',
        encoding="utf-8",
    )
    environment = os.environ.copy()
    environment["PATH"] = f"{fake_bin};{environment['PATH']}"
    environment["LINGTU_COMMAND_LOG"] = str(command_log)
    build_dir = tmp_path / "isolated-slam-build"

    completed = _run_script(
        "-DependencyPrefix",
        str(dependencies),
        "-CycloneDDSPrefix",
        str(cyclone),
        "-BuildDir",
        str(build_dir),
        *_make_vcpkg_arguments(tmp_path, dependencies),
        env=environment,
    )

    assert completed.returncode != 0
    output = completed.stdout + completed.stderr
    assert "authoritative CycloneDDS SDK verification failed" in output
    assert not command_log.exists() or " -S " not in command_log.read_text(encoding="utf-8")


def test_windows_slam_build_declares_exact_product_flags_and_cache_identity() -> None:
    script = SCRIPT.read_text(encoding="utf-8")

    for expected in (
        '"-G", "Visual Studio 17 2022"',
        '"-A", "x64"',
        '"-DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreadedDLL"',
        '"-DVCPKG_TARGET_TRIPLET=x64-windows"',
        '"-DVCPKG_HOST_TRIPLET=x64-windows"',
        '"-DLINGTU_SLAM_FASTLIO2_BACKEND=ON"',
        '"-DLINGTU_SLAM_BUILD_DDS_RUNTIME=ON"',
        '"-DLINGTU_SLAM_BUILD_TESTS=ON"',
        '"-DLINGTU_ENABLE_SMALL_GICP=OFF"',
    ):
        assert expected in script
    assert '& $PowerShellExecutable -NoProfile -File $CycloneDDSVerifier -SdkRoot $CycloneDDSPrefix' in script
    assert script.count("Assert-AuthoritativeCycloneDDSSdk") == 2


def test_windows_slam_build_rejects_cyclonedds_version_drift(tmp_path: Path) -> None:
    dependencies, cyclone = _make_dependency_prefixes(tmp_path)
    _write_file(
        cyclone / "lib" / "cmake" / "CycloneDDS" / "CycloneDDSConfigVersion.cmake",
        'set(PACKAGE_VERSION "0.8.2")\n',
    )

    completed = _run_script(
        "-DependencyPrefix",
        str(dependencies),
        "-CycloneDDSPrefix",
        str(cyclone),
        "-BuildDir",
        str(tmp_path / "build"),
        *_make_vcpkg_arguments(tmp_path, dependencies),
        "-PreflightOnly",
    )

    assert completed.returncode != 0
    assert "version drift" in (completed.stdout + completed.stderr)


def test_windows_slam_build_rejects_linux_or_cyclonedds_drifted_cache(
    tmp_path: Path,
) -> None:
    dependencies, cyclone = _make_dependency_prefixes(tmp_path)
    build_dir = tmp_path / "reused-build"
    _write_file(
        build_dir / "CMakeCache.txt",
        "\n".join(
            (
                "CMAKE_SYSTEM_NAME:INTERNAL=Linux",
                "CMAKE_GENERATOR:INTERNAL=Ninja",
                "CMAKE_GENERATOR_PLATFORM:INTERNAL=",
                "CycloneDDS_DIR:PATH=C:/stale/cyclonedds/lib/cmake/CycloneDDS",
            )
        ),
    )

    completed = _run_script(
        "-DependencyPrefix",
        str(dependencies),
        "-CycloneDDSPrefix",
        str(cyclone),
        "-BuildDir",
        str(build_dir),
        *_make_vcpkg_arguments(tmp_path, dependencies),
        "-PreflightOnly",
    )

    assert completed.returncode != 0
    assert "non-Windows/VS2022-x64" in (completed.stdout + completed.stderr)


def test_windows_slam_build_rejects_cyclonedds_path_drift_in_windows_cache(
    tmp_path: Path,
) -> None:
    dependencies, cyclone = _make_dependency_prefixes(tmp_path)
    build_dir = tmp_path / "reused-build"
    _write_file(
        build_dir / "CMakeCache.txt",
        "\n".join(
            (
                "CMAKE_SYSTEM_NAME:INTERNAL=Windows",
                "CMAKE_GENERATOR:INTERNAL=Visual Studio 17 2022",
                "CMAKE_GENERATOR_PLATFORM:INTERNAL=x64",
                "CycloneDDS_DIR:PATH=C:/stale/cyclonedds/lib/cmake/CycloneDDS",
            )
        ),
    )

    completed = _run_script(
        "-DependencyPrefix",
        str(dependencies),
        "-CycloneDDSPrefix",
        str(cyclone),
        "-BuildDir",
        str(build_dir),
        *_make_vcpkg_arguments(tmp_path, dependencies),
        "-PreflightOnly",
    )

    assert completed.returncode != 0
    assert "path drift" in (completed.stdout + completed.stderr)


def test_windows_slam_build_accepts_native_vs_cache_without_system_name(
    tmp_path: Path,
) -> None:
    dependencies, cyclone = _make_dependency_prefixes(tmp_path)
    build_dir = tmp_path / "native-vs-build"
    cyclone_config_dir = cyclone / "lib" / "cmake" / "CycloneDDS"
    cache_text = "\n".join(
        (
            f"CMAKE_HOME_DIRECTORY:INTERNAL={ROOT / 'src' / 'localization' / 'slam' / 'cpp'}",
            "CMAKE_GENERATOR:INTERNAL=Visual Studio 17 2022",
            "CMAKE_GENERATOR_PLATFORM:INTERNAL=x64",
            f"CycloneDDS_DIR:PATH={cyclone_config_dir}",
            f"CMAKE_TOOLCHAIN_FILE:FILEPATH={tmp_path / 'vcpkg' / 'scripts' / 'buildsystems' / 'vcpkg.cmake'}",
            f"VCPKG_INSTALLED_DIR:PATH={dependencies.parent}",
            "VCPKG_TARGET_TRIPLET:STRING=x64-windows",
            "VCPKG_HOST_TRIPLET:STRING=x64-windows",
            "CMAKE_MSVC_RUNTIME_LIBRARY:STRING=MultiThreadedDLL",
            f"CYCLONEDDS_IDLC_EXECUTABLE:FILEPATH={cyclone / 'bin' / 'idlc.exe'}",
            f"Eigen3_DIR:PATH={dependencies / 'share' / 'eigen3'}",
            f"PCL_DIR:PATH={dependencies / 'share' / 'pcl'}",
            f"yaml-cpp_DIR:PATH={dependencies / 'share' / 'yaml-cpp'}",
            "LINGTU_VCPKG_MANIFEST_SHA256:STRING="
            + __import__("hashlib").sha256(
                (ROOT / "scripts/build/vcpkg/slam-windows/vcpkg.json").read_bytes()
            ).hexdigest(),
            "LINGTU_SLAM_FASTLIO2_BACKEND:BOOL=ON",
            "LINGTU_SLAM_BUILD_DDS_RUNTIME:BOOL=ON",
            "LINGTU_SLAM_BUILD_TESTS:BOOL=ON",
            "LINGTU_ENABLE_SMALL_GICP:BOOL=OFF",
        )
    )
    _write_file(build_dir / "CMakeCache.txt", cache_text)

    completed = _run_script(
        "-DependencyPrefix",
        str(dependencies),
        "-CycloneDDSPrefix",
        str(cyclone),
        "-BuildDir",
        str(build_dir),
        *_make_vcpkg_arguments(tmp_path, dependencies),
        "-PreflightOnly",
    )

    assert completed.returncode == 0, completed.stdout + completed.stderr

    for original, replacement in (
        (
            f"CMAKE_TOOLCHAIN_FILE:FILEPATH={tmp_path / 'vcpkg' / 'scripts' / 'buildsystems' / 'vcpkg.cmake'}",
            "CMAKE_TOOLCHAIN_FILE:FILEPATH=C:/stale/vcpkg.cmake",
        ),
        (f"VCPKG_INSTALLED_DIR:PATH={dependencies.parent}", "VCPKG_INSTALLED_DIR:PATH=C:/stale/installed"),
        ("VCPKG_TARGET_TRIPLET:STRING=x64-windows", "VCPKG_TARGET_TRIPLET:STRING=x86-windows"),
        ("VCPKG_HOST_TRIPLET:STRING=x64-windows", "VCPKG_HOST_TRIPLET:STRING=x86-windows"),
        ("CMAKE_MSVC_RUNTIME_LIBRARY:STRING=MultiThreadedDLL", "CMAKE_MSVC_RUNTIME_LIBRARY:STRING=MultiThreaded"),
        (
            f"CYCLONEDDS_IDLC_EXECUTABLE:FILEPATH={cyclone / 'bin' / 'idlc.exe'}",
            "CYCLONEDDS_IDLC_EXECUTABLE:FILEPATH=C:/stale/idlc.exe",
        ),
        (f"Eigen3_DIR:PATH={dependencies / 'share' / 'eigen3'}", "Eigen3_DIR:PATH=C:/stale/eigen3"),
        (f"PCL_DIR:PATH={dependencies / 'share' / 'pcl'}", "PCL_DIR:PATH=C:/stale/pcl"),
        (f"yaml-cpp_DIR:PATH={dependencies / 'share' / 'yaml-cpp'}", "yaml-cpp_DIR:PATH=C:/stale/yaml-cpp"),
        (
            f"CMAKE_HOME_DIRECTORY:INTERNAL={ROOT / 'src' / 'localization' / 'slam' / 'cpp'}",
            "CMAKE_HOME_DIRECTORY:INTERNAL=C:/stale/source",
        ),
        ("LINGTU_SLAM_FASTLIO2_BACKEND:BOOL=ON", "LINGTU_SLAM_FASTLIO2_BACKEND:BOOL=OFF"),
        ("LINGTU_SLAM_BUILD_DDS_RUNTIME:BOOL=ON", "LINGTU_SLAM_BUILD_DDS_RUNTIME:BOOL=OFF"),
        ("LINGTU_SLAM_BUILD_TESTS:BOOL=ON", "LINGTU_SLAM_BUILD_TESTS:BOOL=OFF"),
        ("LINGTU_ENABLE_SMALL_GICP:BOOL=OFF", "LINGTU_ENABLE_SMALL_GICP:BOOL=ON"),
        (
            "LINGTU_VCPKG_MANIFEST_SHA256:STRING="
            + __import__("hashlib").sha256(
                (ROOT / "scripts/build/vcpkg/slam-windows/vcpkg.json").read_bytes()
            ).hexdigest(),
            "LINGTU_VCPKG_MANIFEST_SHA256:STRING=stale",
        ),
    ):
        _write_file(build_dir / "CMakeCache.txt", cache_text.replace(original, replacement))
        drifted = _run_script(
            "-DependencyPrefix",
            str(dependencies),
            "-CycloneDDSPrefix",
            str(cyclone),
            "-BuildDir",
            str(build_dir),
            *_make_vcpkg_arguments(tmp_path, dependencies),
            "-PreflightOnly",
        )
        assert drifted.returncode != 0
        assert "drift" in (drifted.stdout + drifted.stderr)

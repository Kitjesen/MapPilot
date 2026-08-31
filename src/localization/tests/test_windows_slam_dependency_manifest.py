from __future__ import annotations

import hashlib
import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest

pytestmark = pytest.mark.skipif(os.name != "nt", reason="Windows-only runtime staging")

ROOT = Path(__file__).resolve().parents[3]
MANIFEST = ROOT / "scripts" / "build" / "vcpkg" / "slam-windows" / "vcpkg.json"
STAGE_SCRIPT = ROOT / "scripts" / "build" / "cmake" / "stage_windows_runtime.cmake"
CMAKE = shutil.which("cmake")


def _compile_native_fixture(tmp_path: Path) -> tuple[Path, Path, Path]:
    vswhere = Path(os.environ["ProgramFiles(x86)"]) / "Microsoft Visual Studio/Installer/vswhere.exe"
    vs_install = Path(
        subprocess.run(  # noqa: S603 - queries the installed Visual Studio instance.
            [
                str(vswhere),
                "-latest",
                "-products",
                "*",
                "-requires",
                "Microsoft.VisualStudio.Component.VC.Tools.x86.x64",
                "-property",
                "installationPath",
            ],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
    )
    dumpbin = next((vs_install / "VC/Tools/MSVC").glob("*/bin/Hostx64/x64/dumpbin.exe"))
    fixture_dir = tmp_path / "native-fixture"
    fixture_dir.mkdir()
    (fixture_dir / "fixture.c").write_text(
        "__declspec(dllexport) int fixture_value(void) { return 7; }\n",
        encoding="utf-8",
    )
    (fixture_dir / "app.c").write_text(
        "__declspec(dllimport) int fixture_value(void); int main(void) { return fixture_value() == 7 ? 0 : 1; }\n",
        encoding="utf-8",
    )
    vsdevcmd = vs_install / "Common7" / "Tools" / "VsDevCmd.bat"
    build_batch = fixture_dir / "build-fixture.cmd"
    build_batch.write_text(
        f'@call "{vsdevcmd}" -arch=x64 -host_arch=x64 >nul\n'
        f'@cd /d "{fixture_dir}"\n'
        "@cl /nologo /LD /MD fixture.c /Fe:fixture.dll /link /IMPLIB:fixture.lib\n"
        "@if errorlevel 1 exit /b 1\n"
        "@cl /nologo /MD app.c fixture.lib /Fe:app.exe\n",
        encoding="utf-8",
    )
    subprocess.run(  # noqa: S603 - compiles a hermetic local PE/DLL fixture with VS2022.
        [os.environ["ComSpec"], "/d", "/c", str(build_batch)],
        check=True,
        capture_output=True,
        text=True,
    )
    return fixture_dir / "app.exe", fixture_dir / "fixture.dll", dumpbin


def test_windows_slam_manifest_pins_baseline_and_minimal_direct_dependencies() -> None:
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))

    assert manifest["builtin-baseline"] == "9e593bb18ea69cc5095e012465dcd675a822ed0d"
    assert manifest["dependencies"] == [
        "eigen3",
        {"name": "pcl", "default-features": False},
        "yaml-cpp",
    ]


def test_windows_runtime_stage_has_no_recursive_delete_surface() -> None:
    script = STAGE_SCRIPT.read_text(encoding="utf-8")

    assert "REMOVE_RECURSE" not in script
    assert "STAGE_ROOT must be the exact 'stage' child of ALLOWED_STAGE_PARENT" in script
    assert 'file(RENAME "${_temporary_stage}" "${STAGE_ROOT}")' in script


def test_windows_runtime_stage_revalidates_system_policy_before_reuse() -> None:
    script = STAGE_SCRIPT.read_text(encoding="utf-8")

    reuse_guard = script.index('if(EXISTS "${STAGE_ROOT}")')
    assert reuse_guard > script.index("file(GET_RUNTIME_DEPENDENCIES")
    assert reuse_guard > script.index('set(_vc_runtime_receipt "name\\tsha256\\tarch\\n")')
    assert reuse_guard > script.index('_assert_x64_pe("${_apiset_schema}"')
    receipt = script[script.rindex("string(SHA256 _stage_input_receipt", 0, reuse_guard) : reuse_guard]
    for policy_identity in (
        "_dependency_tool_sha256",
        "_windows_system_directory",
        "_apiset_schema_sha256",
        "_vc_runtime_receipt",
    ):
        assert policy_identity in receipt


def test_windows_runtime_stage_rejects_syswow64_dependencies() -> None:
    script = STAGE_SCRIPT.read_text(encoding="utf-8")

    syswow64_guard = 'if(_runtime_lower MATCHES "^[a-z]:/windows/syswow64/")'
    system32_guard = 'if(_runtime_lower MATCHES "^[a-z]:/windows/system32/")'
    assert syswow64_guard in script
    assert system32_guard in script

    syswow64_block = script[
        script.index(syswow64_guard) : script.index(system32_guard)
    ]
    assert 'message(FATAL_ERROR "Resolved x86 SysWOW64 dependency is forbidden:' in syswow64_block
    assert "continue()" not in syswow64_block

    system32_block = script[
        script.index(system32_guard) : script.index(
            'string(FIND "${_runtime_lower}" "${_dependency_prefix_lower}"',
            script.index(system32_guard),
        )
    ]
    assert '_assert_x64_pe("${_runtime_real}" "Resolved Windows system dependency")' in system32_block
    assert "continue()" in system32_block


@pytest.mark.parametrize(
    ("runtime_owner", "app_local_runtime"),
    [
        pytest.param("pcl", False, id="pcl-prefix-runtime"),
        pytest.param("yaml-cpp", False, id="yaml-prefix-runtime"),
        pytest.param("pcl", True, id="pcl-app-local-runtime"),
    ],
)
def test_windows_runtime_stage_uses_only_resolved_real_install_evidence(
    tmp_path: Path, runtime_owner: str, app_local_runtime: bool,
) -> None:
    assert CMAKE is not None
    dependency_prefix = tmp_path / "installed" / "x64-windows"
    cyclone_prefix = tmp_path / "cyclone"
    fixture_executable, fixture_dll, dumpbin = _compile_native_fixture(tmp_path)
    for port in ("eigen3", "pcl", "yaml-cpp"):
        copyright_file = dependency_prefix / "share" / port / "copyright"
        copyright_file.parent.mkdir(parents=True, exist_ok=True)
        copyright_file.write_text(f"real {port} fixture\n", encoding="utf-8")
    cyclone_license = cyclone_prefix / "licenses" / "LICENSE"
    cyclone_license.parent.mkdir(parents=True)
    cyclone_license.write_text("real CycloneDDS fixture\n", encoding="utf-8")
    (cyclone_prefix / "licenses" / "NOTICE.md").write_text("fixture notice\n", encoding="utf-8")
    (cyclone_prefix / "bin").mkdir()
    shutil.copy2(fixture_executable, cyclone_prefix / "bin" / "idlc.exe")
    shutil.copy2(fixture_dll, cyclone_prefix / "bin" / "ddsc.dll")
    (cyclone_prefix / "lib" / "cmake" / "CycloneDDS").mkdir(parents=True)
    (cyclone_prefix / "lib" / "cmake" / "CycloneDDS" / "CycloneDDSConfig.cmake").write_text(
        "# fixture\n", encoding="utf-8"
    )
    (cyclone_prefix / "lib" / "ddsc.lib").write_text("fixture\n", encoding="utf-8")
    evidence_dir = cyclone_prefix / "evidence"
    evidence_dir.mkdir()
    for name in ("source.json", "toolchain.json", "build.json"):
        (evidence_dir / name).write_text("{}\n", encoding="utf-8")
    source_lock = evidence_dir / "source-lock.json"
    shutil.copy2(
        ROOT / "scripts" / "build" / "locks" / "cyclonedds-windows-x64.json",
        source_lock,
    )
    sbom_path = evidence_dir / "sbom.spdx.json"
    license_sha1 = hashlib.sha1(cyclone_license.read_bytes(), usedforsecurity=False).hexdigest()
    sbom_path.write_text(
        json.dumps(
            {
                "spdxVersion": "SPDX-2.3",
                "packages": [{"name": "Eclipse Cyclone DDS"}],
                "files": [{"SPDXID": "SPDXRef-File-1", "fileName": "./licenses/LICENSE"}],
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
                "fixtureSha1": license_sha1,
            }
        ),
        encoding="utf-8",
    )
    manifest_path = evidence_dir / "files.sha256"
    manifest_lines = []
    for path in sorted(path for path in cyclone_prefix.rglob("*") if path.is_file()):
        relative = path.relative_to(cyclone_prefix).as_posix()
        if relative in {"evidence/files.sha256", "evidence/sdk-receipt.json"}:
            continue
        manifest_lines.append(f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {relative}\n")
    manifest_path.write_text("".join(manifest_lines), encoding="utf-8")
    cyclone_receipt = evidence_dir / "sdk-receipt.json"
    cyclone_receipt.write_text(
        json.dumps(
            {
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
        ),
        encoding="utf-8",
    )
    (dependency_prefix / "bin").mkdir(parents=True)
    shutil.copy2(fixture_dll, dependency_prefix / "bin" / "fixture.dll")
    info_dir = dependency_prefix.parent / "vcpkg" / "info"
    info_dir.mkdir(parents=True)
    (info_dir / "eigen3_3.4.0_x64-windows.list").write_text(
        "x64-windows/include/eigen3/Eigen/Core\n", encoding="utf-8"
    )
    (info_dir / "pcl_1.0_x64-windows.list").write_text(
        "x64-windows/bin/fixture.dll\n"
        if runtime_owner == "pcl"
        else "x64-windows/lib/pcl.lib\n",
        encoding="utf-8",
    )
    (info_dir / "yaml-cpp_0.8.0_x64-windows.list").write_text(
        "x64-windows/bin/fixture.dll\n"
        if runtime_owner == "yaml-cpp"
        else "x64-windows/lib/yaml-cpp.lib\n",
        encoding="utf-8",
    )
    build_dir = tmp_path / "build"
    build_dir.mkdir()
    (build_dir / "CMakeCache.txt").write_text("# fixture\n", encoding="utf-8")
    stage_root = build_dir / "stage"
    system_executable = build_dir / "idlc.exe"
    shutil.copy2(fixture_executable, system_executable)
    if app_local_runtime:
        shutil.copy2(fixture_dll, build_dir / "fixture.dll")

    completed = subprocess.run(  # noqa: S603 - runs CMake's repository-owned stage script.
        [
            CMAKE,
            f"-DSLAMD_EXECUTABLE={system_executable}",
            f"-DSLAMCTL_EXECUTABLE={system_executable}",
            f"-DDEPENDENCY_PREFIX={dependency_prefix}",
            f"-DCYCLONEDDS_PREFIX={cyclone_prefix}",
            f"-DCYCLONEDDS_SDK_RECEIPT={cyclone_receipt}",
            f"-DCYCLONEDDS_CANONICAL_LOCK={ROOT / 'scripts/build/locks/cyclonedds-windows-x64.json'}",
            "-DCYCLONEDDS_VERSION=11.0.1",
            f"-DDEPENDENCY_TOOL={dumpbin}",
            f"-DWINDOWS_SYSTEM_DIRECTORY={Path('C:/Windows/System32')}",
            f"-DALLOWED_STAGE_PARENT={build_dir}",
            f"-DSTAGE_ROOT={stage_root}",
            "-P",
            str(STAGE_SCRIPT),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stdout + completed.stderr
    assert (stage_root / "bin" / "slamd.exe").is_file()
    assert (stage_root / "bin" / "slamctl.exe").is_file()
    assert (stage_root / "bin" / "fixture.dll").is_file()
    assert not (stage_root / "bin" / "msvcp140.dll").exists()
    assert not (stage_root / "bin" / "vcruntime140.dll").exists()
    assert not (stage_root / "bin" / "concrt140.dll").exists()
    prerequisites = json.loads((stage_root / "evidence" / "prerequisites.json").read_text(encoding="utf-8"))
    assert prerequisites["prerequisites"][0]["name"] == ("Microsoft Visual C++ Redistributable")
    stage_reuse_key = (stage_root / "evidence" / "stage-receipt.sha256").read_text(
        encoding="utf-8"
    ).strip()
    stage_receipt = json.loads(
        (stage_root / "evidence" / "stage-receipt.json").read_text(encoding="utf-8")
    )
    assert set(stage_receipt) == {
        "schema_version",
        "reuse_key_sha256",
        "payload_sha256",
        "architecture",
        "policy",
    }
    assert stage_receipt["schema_version"] == 1
    assert stage_receipt["reuse_key_sha256"] == stage_reuse_key
    assert stage_receipt["architecture"] == "x64"
    assert stage_receipt["policy"] == "windows-system-and-vc-runtime-v1"
    assert len(stage_receipt["payload_sha256"]) == 64
    int(stage_receipt["payload_sha256"], 16)
    assert (stage_root / "evidence" / "runtime-dependencies.tsv").is_file()
    sbom = json.loads((stage_root / "evidence" / "sbom.spdx.json").read_text(encoding="utf-8"))
    assert sbom["spdxVersion"] == "SPDX-2.3"
    assert sbom["creationInfo"]["creators"] == ["Tool: LingTu stage_windows_runtime.cmake"]
    assert sbom["files"]
    assert {package["name"] for package in sbom["packages"]} >= {
        "eigen3",
        "pcl",
        "yaml-cpp",
        "CycloneDDS",
    }
    packages = {package["name"]: package for package in sbom["packages"]}
    assert packages[runtime_owner]["filesAnalyzed"] is True
    assert packages[runtime_owner]["packageVerificationCode"]
    non_runtime_packages = {"eigen3", "pcl", "yaml-cpp"} - {runtime_owner}
    for non_runtime_package in non_runtime_packages:
        assert packages[non_runtime_package]["filesAnalyzed"] is False
        assert "packageVerificationCode" not in packages[non_runtime_package]
    assert all(package["copyrightText"] == "NOASSERTION" for package in sbom["packages"])
    assert all(file_record["licenseInfoInFiles"] == ["NOASSERTION"] for file_record in sbom["files"])
    relationships = {
        (relationship["spdxElementId"], relationship["relationshipType"], relationship["relatedSpdxElement"])
        for relationship in sbom["relationships"]
    }
    file_ids = {file_record["fileName"]: file_record["SPDXID"] for file_record in sbom["files"]}
    package_ids = {package["SPDXID"] for package in sbom["packages"]}
    assert all(("SPDXRef-DOCUMENT", "DESCRIBES", package_id) in relationships for package_id in package_ids)
    assert all(
        any(source == package_id and relation == "CONTAINS" for source, relation, _target in relationships)
        for package_id in package_ids
    )
    lingtu_id = packages["LingTu"]["SPDXID"]
    runtime_owner_id = packages[runtime_owner]["SPDXID"]
    assert (runtime_owner_id, "CONTAINS", file_ids["./bin/fixture.dll"]) in relationships
    assert (
        runtime_owner_id,
        "CONTAINS",
        file_ids[f"./licenses/vcpkg/{runtime_owner}.txt"],
    ) in relationships
    assert (stage_root / "licenses" / "vcpkg" / f"{runtime_owner}.txt").is_file()
    assert (lingtu_id, "DEPENDS_ON", packages["eigen3"]["SPDXID"]) in relationships
    assert (lingtu_id, "DYNAMIC_LINK", packages["CycloneDDS"]["SPDXID"]) in relationships
    assert sum(
        relationship["spdxElementId"] == lingtu_id
        and relationship["relationshipType"] == "DYNAMIC_LINK"
        and relationship["relatedSpdxElement"] == packages["CycloneDDS"]["SPDXID"]
        for relationship in sbom["relationships"]
    ) == 1
    for package_name in ("pcl", "yaml-cpp"):
        expected_link = "DYNAMIC_LINK" if package_name == runtime_owner else "STATIC_LINK"
        assert (lingtu_id, expected_link, packages[package_name]["SPDXID"]) in relationships
    assert (stage_root / "licenses" / "cyclonedds" / "LICENSE").is_file()
    assert (stage_root / "licenses" / "cyclonedds" / "NOTICE.md").is_file()
    evidence = (stage_root / "evidence" / "runtime-dependencies.tsv").read_text(encoding="utf-8")
    assert "source_path" in evidence
    assert str(cyclone_license).replace("\\", "/") in evidence.replace("\\", "/")
    expected_version = "1.0" if runtime_owner == "pcl" else "0.8.0"
    assert f"\t{runtime_owner}\t{expected_version}\tn/a\n" in evidence
    expected_runtime_source = dependency_prefix / "bin" / "fixture.dll"
    assert (
        f"bin/fixture.dll\t{hashlib.sha256(expected_runtime_source.read_bytes()).hexdigest()}\t"
        f"{expected_runtime_source.as_posix()}\t{runtime_owner}\t{expected_version}\tx64\n"
    ) in evidence.replace("\\", "/")

    repeated = subprocess.run(  # noqa: S603 - verifies safe idempotent staging.
        completed.args,
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert repeated.returncode == 0, repeated.stdout + repeated.stderr
    assert "already matches verified receipt" in repeated.stdout

    if app_local_runtime:
        app_local_dll = build_dir / "fixture.dll"
        pristine_app_local = app_local_dll.read_bytes()
        app_local_dll.write_bytes(pristine_app_local + b"app-local-drift")
        mismatched_copy = subprocess.run(  # noqa: S603 - verifies authoritative vcpkg binding.
            completed.args,
            cwd=ROOT,
            check=False,
            capture_output=True,
            text=True,
        )
        assert mismatched_copy.returncode != 0
        assert "does not match its authoritative vcpkg original" in (
            mismatched_copy.stdout + mismatched_copy.stderr
        )
        app_local_dll.write_bytes(pristine_app_local)

        second_owner_list = info_dir / "yaml-cpp_0.8.0_x64-windows.list"
        pristine_second_owner = second_owner_list.read_text(encoding="utf-8")
        second_owner_list.write_text(
            pristine_second_owner + "x64-windows/bin/fixture.dll\n",
            encoding="utf-8",
        )
        ambiguous_owner = subprocess.run(  # noqa: S603 - verifies unique vcpkg ownership.
            completed.args,
            cwd=ROOT,
            check=False,
            capture_output=True,
            text=True,
        )
        assert ambiguous_owner.returncode != 0
        ambiguous_output = ambiguous_owner.stdout + ambiguous_owner.stderr
        assert "must map to at most one installed vcpkg package" in ambiguous_output
        assert "found 2" in ambiguous_output
        second_owner_list.write_text(pristine_second_owner, encoding="utf-8")

    staged_notice = stage_root / "licenses" / "cyclonedds" / "NOTICE.md"
    staged_notice.unlink()
    missing = subprocess.run(  # noqa: S603 - verifies exact stage file coverage.
        completed.args,
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert missing.returncode != 0
    assert "is missing" in (missing.stdout + missing.stderr)
    shutil.copy2(cyclone_prefix / "licenses" / "NOTICE.md", staged_notice)

    injected_file = stage_root / "injected.txt"
    injected_file.write_text("not covered\n", encoding="utf-8")
    injected = subprocess.run(  # noqa: S603 - verifies exact stage file coverage.
        completed.args,
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert injected.returncode != 0
    assert "not covered" in (injected.stdout + injected.stderr)
    injected_file.unlink()

    mutated_lock = tmp_path / "mutated-cyclonedds-lock.json"
    mutated_lock.write_text(
        (ROOT / "scripts/build/locks/cyclonedds-windows-x64.json")
        .read_text(encoding="utf-8")
        .replace('"tag": "11.0.1"', '"tag": "tampered"'),
        encoding="utf-8",
    )
    mutated_lock_args = [
        f"-DCYCLONEDDS_CANONICAL_LOCK={mutated_lock}"
        if argument.startswith("-DCYCLONEDDS_CANONICAL_LOCK=")
        else argument
        for argument in completed.args
    ]
    bad_lock = subprocess.run(  # noqa: S603 - verifies repository-lock binding.
        mutated_lock_args,
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert bad_lock.returncode != 0
    assert "CycloneDDS SDK receipt identity or verification result is invalid" in (
        bad_lock.stdout + bad_lock.stderr
    )

    staged_slamd = stage_root / "bin" / "slamd.exe"
    staged_slamd.write_bytes(staged_slamd.read_bytes() + b"tamper")
    tampered = subprocess.run(  # noqa: S603 - verifies receipt-protected idempotence.
        completed.args,
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert tampered.returncode != 0
    assert "hash mismatch" in (tampered.stdout + tampered.stderr)
    shutil.copy2(system_executable, staged_slamd)

    staged_slamd.write_bytes(staged_slamd.read_bytes() + b"synchronized tamper")
    staged_slamd_sha = hashlib.sha256(staged_slamd.read_bytes()).hexdigest()
    runtime_tsv = stage_root / "evidence" / "runtime-dependencies.tsv"
    runtime_lines = runtime_tsv.read_text(encoding="utf-8").splitlines()
    runtime_lines = [
        "\t".join([fields[0], staged_slamd_sha, *fields[2:]])
        if (fields := line.split("\t"))[0] == "bin/slamd.exe"
        else line
        for line in runtime_lines
    ]
    runtime_tsv.write_text("\n".join(runtime_lines) + "\n", encoding="utf-8")
    staged_sbom_path = stage_root / "evidence" / "sbom.spdx.json"
    staged_sbom = json.loads(staged_sbom_path.read_text(encoding="utf-8"))
    for file_record in staged_sbom["files"]:
        if file_record["fileName"] == "./bin/slamd.exe":
            file_record["checksums"][0]["checksumValue"] = staged_slamd_sha
    staged_files_by_id = {
        file_record["SPDXID"]: stage_root / file_record["fileName"].removeprefix("./")
        for file_record in staged_sbom["files"]
    }
    contained_files: dict[str, list[Path]] = {}
    for relationship in staged_sbom["relationships"]:
        if relationship["relationshipType"] == "CONTAINS":
            contained_files.setdefault(relationship["spdxElementId"], []).append(
                staged_files_by_id[relationship["relatedSpdxElement"]]
            )
    for package in staged_sbom["packages"]:
        if package["filesAnalyzed"]:
            member_sha1s = sorted(
                hashlib.sha1(path.read_bytes(), usedforsecurity=False).hexdigest()
                for path in contained_files[package["SPDXID"]]
            )
            package["packageVerificationCode"]["packageVerificationCodeValue"] = hashlib.sha1(
                "".join(member_sha1s).encode("ascii"), usedforsecurity=False
            ).hexdigest()
    staged_sbom_path.write_text(json.dumps(staged_sbom, separators=(",", ":")) + "\n", encoding="utf-8")
    stage_manifest = stage_root / "evidence" / "stage-files.sha256"
    stage_manifest.write_text(
        "".join(
            f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.relative_to(stage_root).as_posix()}\n"
            for path in sorted(path for path in stage_root.rglob("*") if path.is_file())
            if path != stage_manifest
        ),
        encoding="utf-8",
    )
    synchronized_tamper = subprocess.run(  # noqa: S603 - reruns the repository-owned stage command.
        completed.args,
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert synchronized_tamper.returncode != 0
    assert "current authoritative input" in (
        synchronized_tamper.stdout + synchronized_tamper.stderr
    )

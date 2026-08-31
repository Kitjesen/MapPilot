from __future__ import annotations

import hashlib
import json
import re
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
STAGE_SCRIPT = ROOT / "scripts" / "build" / "cmake" / "stage_cyclonedds_windows_sdk.cmake"
VERIFY_SCRIPT = ROOT / "scripts" / "build" / "verify_cyclonedds_windows_sdk.ps1"
LOCK = ROOT / "scripts" / "build" / "locks" / "cyclonedds-windows-x64.json"
CMAKE = shutil.which("cmake")
POWERSHELL = shutil.which("pwsh")
COMMIT = "e54e991f75a3e67f8e628da3171122e36ea5b872"
TREE = "56508d35826c362782fc8a388cad351a3d491f51"


def _fake_pe() -> bytes:
    pe = bytearray(128)
    pe[0:2] = b"MZ"
    pe[0x3C:0x40] = (64).to_bytes(4, "little")
    pe[64:68] = b"PE\0\0"
    pe[68:70] = (0x8664).to_bytes(2, "little")
    return bytes(pe)


def _write_fake_tree(tmp_path: Path) -> tuple[Path, Path, Path, Path]:
    install = tmp_path / "install"
    source = tmp_path / "source"
    receipts = tmp_path / "receipts"
    final = tmp_path / "sdk"
    for relative in (
        "include/dds/dds.h",
        "lib/ddsc.lib",
        "bin/ddsc.dll",
        "bin/idlc.exe",
        "lib/cmake/CycloneDDS/CycloneDDSConfig.cmake",
    ):
        path = install / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(_fake_pe() if relative in {"bin/ddsc.dll", "bin/idlc.exe"} else f"fake {relative}".encode())
    source.mkdir()
    (source / "LICENSE").write_text("fake license\n", encoding="utf-8")
    (source / "NOTICE.md").write_text("fake notice\n", encoding="utf-8")
    receipts.mkdir()
    (receipts / "source.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "repository": "https://github.com/eclipse-cyclonedds/cyclonedds.git",
                "tag": "11.0.1",
                "tag_commit": COMMIT,
                "commit": COMMIT,
                "tree": TREE,
                "detached": True,
                "clean": True,
                "checkout_eol_policy": "core.autocrlf=false;core.eol=lf;core.safecrlf=true",
            }
        ),
        encoding="utf-8",
    )
    (receipts / "toolchain.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "generator": "Visual Studio 17 2022",
                "toolset": "v143",
                "architecture": "x64",
                "configuration": "Release",
                "msvc_runtime": "/MD",
            }
        ),
        encoding="utf-8",
    )
    (receipts / "build.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "build_shared_libs": True,
                "enable_security": False,
                "build_idlc": True,
                "install_system_runtime_libs_skip": True,
                "source_commit": COMMIT,
                "source_tree": TREE,
            }
        ),
        encoding="utf-8",
    )
    return install, source, receipts, final


def _stage(install: Path, source: Path, receipts: Path, final: Path) -> subprocess.CompletedProcess[str]:
    assert CMAKE is not None
    return subprocess.run(  # noqa: S603 - runs the repository-owned CMake staging script.
        [
            CMAKE,
            f"-DINSTALL_ROOT={install}",
            f"-DSOURCE_ROOT={source}",
            f"-DRECEIPT_ROOT={receipts}",
            f"-DLOCK_PATH={LOCK}",
            f"-DFINAL_SDK_ROOT={final}",
            "-DEXPECTED_VERSION=11.0.1",
            "-P",
            str(STAGE_SCRIPT),
        ],
        cwd=ROOT,
        capture_output=True,
        check=False,
        text=True,
    )


def _refresh_manifest_entry(quarantine: Path, relative: str) -> None:
    manifest = quarantine / "evidence/files.sha256"
    lines = manifest.read_text(encoding="utf-8").splitlines()
    digest = hashlib.sha256((quarantine / relative).read_bytes()).hexdigest()
    manifest.write_text(
        "\n".join(f"{digest}  {relative}" if line.endswith(f"  {relative}") else line for line in lines) + "\n",
        encoding="utf-8",
    )


def _verify(quarantine: Path, *, create_receipt: bool = True) -> subprocess.CompletedProcess[str]:
    assert POWERSHELL is not None
    command = [POWERSHELL, "-NoProfile", "-File", str(VERIFY_SCRIPT), "-SdkRoot", str(quarantine)]
    if create_receipt:
        command.append("-CreateReceipt")
    return subprocess.run(  # noqa: S603 - runs the repository-owned verifier.
        command,
        cwd=ROOT,
        capture_output=True,
        check=False,
        text=True,
    )


def _assert_spdx_23_is_internally_consistent(quarantine: Path) -> None:
    spdx = json.loads((quarantine / "evidence/sbom.spdx.json").read_text(encoding="utf-8"))
    assert spdx["spdxVersion"] == "SPDX-2.3"
    assert spdx["SPDXID"] == "SPDXRef-DOCUMENT"
    assert spdx["dataLicense"] == "CC0-1.0"
    assert re.fullmatch(
        rf"https://inovxio\.example/spdx/cyclonedds/11\.0\.1/{COMMIT}/"
        r"[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}",
        spdx["documentNamespace"],
    )
    package = spdx["packages"][0]
    sha1s: list[str] = []
    file_ids: set[str] = set()
    for record in spdx["files"]:
        relative = record["fileName"].removeprefix("./")
        payload = quarantine / relative
        checksums = {item["algorithm"]: item["checksumValue"] for item in record["checksums"]}
        assert checksums["SHA1"] == hashlib.sha1(payload.read_bytes()).hexdigest()  # noqa: S324 - SPDX 2.3 requires SHA-1.
        assert checksums["SHA256"] == hashlib.sha256(payload.read_bytes()).hexdigest()
        assert record["licenseInfoInFiles"] == ["NOASSERTION"]
        sha1s.append(checksums["SHA1"])
        file_ids.add(record["SPDXID"])
    verification = hashlib.sha1("".join(sorted(sha1s)).encode()).hexdigest()  # noqa: S324 - SPDX algorithm.
    assert package["packageVerificationCode"]["packageVerificationCodeValue"] == verification
    relationships = spdx["relationships"]
    assert {
        "spdxElementId": "SPDXRef-DOCUMENT",
        "relationshipType": "DESCRIBES",
        "relatedSpdxElement": "SPDXRef-Package-CycloneDDS",
    } in relationships
    contained = {
        item["relatedSpdxElement"]
        for item in relationships
        if item["spdxElementId"] == "SPDXRef-Package-CycloneDDS" and item["relationshipType"] == "CONTAINS"
    }
    assert contained == file_ids


@pytest.mark.skipif(CMAKE is None, reason="cmake unavailable")
def test_stage_keeps_valid_spdx_sdk_in_quarantine_and_hashes_the_sbom(tmp_path: Path) -> None:
    install, source, receipts, final = _write_fake_tree(tmp_path)

    completed = _stage(install, source, receipts, final)

    quarantine = Path(f"{final}.incoming")
    assert completed.returncode == 0, completed.stdout + completed.stderr
    assert not final.exists()
    assert (quarantine / "bin/ddsc.dll").is_file()
    _assert_spdx_23_is_internally_consistent(quarantine)
    manifest = (quarantine / "evidence/files.sha256").read_text(encoding="utf-8")
    sbom_sha = hashlib.sha256((quarantine / "evidence/sbom.spdx.json").read_bytes()).hexdigest()
    assert f"{sbom_sha}  evidence/sbom.spdx.json" in manifest


@pytest.mark.skipif(CMAKE is None, reason="cmake unavailable")
def test_each_generated_spdx_document_has_a_unique_bound_namespace(tmp_path: Path) -> None:
    namespaces: list[str] = []
    for name in ("first", "second"):
        install, source, receipts, final = _write_fake_tree(tmp_path / name)
        completed = _stage(install, source, receipts, final)
        assert completed.returncode == 0, completed.stdout + completed.stderr
        quarantine = Path(f"{final}.incoming")
        document = json.loads((quarantine / "evidence/sbom.spdx.json").read_text(encoding="utf-8"))
        namespaces.append(document["documentNamespace"])
    assert namespaces[0] != namespaces[1]
    assert all(f"/11.0.1/{COMMIT}/" in namespace for namespace in namespaces)


@pytest.mark.skipif(CMAKE is None, reason="cmake unavailable")
@pytest.mark.parametrize(
    "runtime_name",
    ["vcruntime140.dll", "concret140.dll", "mfc140u.dll", "api-ms-win-crt-runtime-l1-1-0.dll"],
)
def test_stage_rejects_microsoft_runtime_families(tmp_path: Path, runtime_name: str) -> None:
    install, source, receipts, final = _write_fake_tree(tmp_path)
    (install / "bin" / runtime_name).write_bytes(b"must be installed by Microsoft")

    completed = _stage(install, source, receipts, final)

    assert completed.returncode != 0
    assert "Microsoft runtime DLL" in completed.stdout + completed.stderr
    assert not final.exists()
    assert not Path(f"{final}.incoming").exists()


@pytest.mark.skipif(CMAKE is None, reason="cmake unavailable")
def test_standalone_stage_rejects_overlapping_input_and_output_roots(tmp_path: Path) -> None:
    install, source, receipts, _ = _write_fake_tree(tmp_path)
    final = install / "nested-sdk"

    completed = _stage(install, source, receipts, final)

    assert completed.returncode != 0
    assert "paths overlap" in completed.stdout + completed.stderr
    assert not final.exists()
    assert not Path(f"{final}.incoming").exists()


@pytest.mark.skipif(CMAKE is None or POWERSHELL is None, reason="cmake or pwsh unavailable")
def test_invalid_spdx_fails_in_quarantine_without_publishing_final_sdk(tmp_path: Path) -> None:
    install, source, receipts, final = _write_fake_tree(tmp_path)
    staged = _stage(install, source, receipts, final)
    assert staged.returncode == 0, staged.stdout + staged.stderr
    quarantine = Path(f"{final}.incoming")
    sbom = quarantine / "evidence/sbom.spdx.json"
    document = json.loads(sbom.read_text(encoding="utf-8"))
    document["relationships"] = []
    sbom.write_text(json.dumps(document), encoding="utf-8")
    _refresh_manifest_entry(quarantine, "evidence/sbom.spdx.json")

    completed = _verify(quarantine)

    assert completed.returncode != 0
    assert "DESCRIBES package relationship" in completed.stdout + completed.stderr
    assert not final.exists()


@pytest.mark.skipif(CMAKE is None or POWERSHELL is None, reason="cmake or pwsh unavailable")
def test_realistic_spdx_iso_timestamp_remains_a_string_in_pwsh(tmp_path: Path) -> None:
    install, source, receipts, final = _write_fake_tree(tmp_path)
    staged = _stage(install, source, receipts, final)
    assert staged.returncode == 0, staged.stdout + staged.stderr
    quarantine = Path(f"{final}.incoming")
    document = json.loads((quarantine / "evidence/sbom.spdx.json").read_text(encoding="utf-8"))
    assert re.fullmatch(r"\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z", document["creationInfo"]["created"])

    completed = _verify(quarantine)

    assert completed.returncode != 0
    output = completed.stdout + completed.stderr
    assert "No PE imports could be parsed" in output
    assert "document metadata" not in output
    assert "legal file SHA-256 mismatch" not in output
    assert not final.exists()


@pytest.mark.skipif(CMAKE is None or POWERSHELL is None, reason="cmake or pwsh unavailable")
def test_missing_dll_and_payload_tamper_are_rejected_before_publish(tmp_path: Path) -> None:
    install, source, receipts, final = _write_fake_tree(tmp_path)
    staged = _stage(install, source, receipts, final)
    assert staged.returncode == 0, staged.stdout + staged.stderr
    quarantine = Path(f"{final}.incoming")
    (quarantine / "bin/ddsc.dll").unlink()

    missing = _verify(quarantine)

    assert missing.returncode != 0
    assert "missing required file" in missing.stdout + missing.stderr
    assert not final.exists()

    shutil.rmtree(quarantine)
    staged = _stage(install, source, receipts, final)
    assert staged.returncode == 0, staged.stdout + staged.stderr
    quarantine = Path(f"{final}.incoming")
    payload = quarantine / "include/dds/dds.h"
    payload.write_bytes(payload.read_bytes() + b"tamper")

    tampered = _verify(quarantine)

    assert tampered.returncode != 0
    assert "hash mismatch" in tampered.stdout + tampered.stderr
    assert not final.exists()


@pytest.mark.skipif(CMAKE is None or POWERSHELL is None, reason="cmake or pwsh unavailable")
def test_tampered_sdk_receipt_is_rejected(tmp_path: Path) -> None:
    install, source, receipts, final = _write_fake_tree(tmp_path)
    staged = _stage(install, source, receipts, final)
    assert staged.returncode == 0, staged.stdout + staged.stderr
    quarantine = Path(f"{final}.incoming")
    receipt = {
        "schema_version": 1,
        "sdk": {"name": "CycloneDDS", "version": "11.0.1"},
        "source": {
            "repository": "https://github.com/eclipse-cyclonedds/cyclonedds.git",
            "tag": "11.0.1",
            "commit": COMMIT,
            "tree": TREE,
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
            "result": "failed",
            "pe_x64": True,
            "dll_closure": "passed",
            "idl_smoke": "passed",
            "consumer_compile_link": "passed",
            "sanitized_dll_search": True,
        },
    }
    (quarantine / "evidence/sdk-receipt.json").write_text(json.dumps(receipt), encoding="utf-8")

    completed = _verify(quarantine, create_receipt=False)

    assert completed.returncode != 0
    assert "verification receipt" in completed.stdout + completed.stderr
    assert not final.exists()

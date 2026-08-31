"""Contract tests for the read-only Windows toolchain lock artifacts."""

import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
TOOLCHAINS = ROOT / "sim" / "toolchains"
SCHEMA_PATH = TOOLCHAINS / "windows-toolchain-lock.v1.schema.json"
EXAMPLE_PATH = TOOLCHAINS / "windows.lock.example.json"
SCRIPT_PATH = TOOLCHAINS / "detect_windows_toolchain.ps1"
COMPONENTS = ("unreal_engine", "visual_studio_msvc", "windows_sdk", "mujoco_sdk", "blender")
SHA256 = re.compile(r"^[0-9a-fA-F]{64}$")


def read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def test_schema_and_example_exist_and_are_json():
    schema = read_json(SCHEMA_PATH)
    example = read_json(EXAMPLE_PATH)
    assert schema["$schema"].endswith("draft/2020-12/schema")
    assert schema["$id"].endswith("windows-toolchain-lock.v1.schema.json")
    assert example["schema"] == "lingtu.sim.windows-toolchain-lock"
    assert example["schema_version"] == 1


def test_schema_declares_all_components_and_strict_hash_contract():
    schema = read_json(SCHEMA_PATH)
    assert set(schema["properties"]["components"]["required"]) == set(COMPONENTS)
    assert schema["$defs"]["sha256"]["properties"]["algorithm"]["const"] == "sha256"
    assert schema["$defs"]["sha256"]["properties"]["value"]["pattern"] == "^[A-Fa-f0-9]{64}$"
    assert schema["$defs"]["component_state"]["enum"] == ["not_found", "detected", "validated", "locked"]


def test_example_has_no_fabricated_installation_facts_and_is_unlocked():
    example = read_json(EXAMPLE_PATH)
    assert example["summary"] == {"state": "unlocked", "locked_count": 0, "component_count": 5}
    for name in COMPONENTS:
        component = example["components"][name]
        assert component["status"]["state"] == "not_found"
        assert component["version"]["exact"] is None
        assert component["version"]["source"] is None
        assert component["source"]["location"] is None
        assert component["architecture"] == "unknown"
        assert component["sha256"]["value"] is None
        assert component["installation"]["root_path"] is None
        assert component["installation"]["executable_path"] is None
        assert component["lock"] is None


def test_example_shape_is_complete_without_external_jsonschema_dependency():
    example = read_json(EXAMPLE_PATH)
    assert set(example) == {"schema", "schema_version", "generated_at_utc", "host", "policy", "summary", "components"}
    for name in COMPONENTS:
        component = example["components"][name]
        assert set(component) == {
            "id",
            "status",
            "version",
            "source",
            "architecture",
            "sha256",
            "installation",
            "validation",
            "lock",
        }
        assert set(component["status"]) == {"state", "reasons"}
        assert set(component["version"]) == {"exact", "source"}
        assert set(component["source"]) == {"type", "location", "evidence"}
        assert set(component["sha256"]) == {"value", "path", "algorithm", "evidence"}
        assert set(component["installation"]) == {"root_path", "executable_path", "evidence"}
        assert set(component["validation"]) == {"checks", "validated_at_utc"}


def test_locked_requirements_are_explicit_and_hashes_are_strict():
    schema_text = SCHEMA_PATH.read_text(encoding="utf-8")
    for token in ("locked", "exact_version", "executable_path", "sha256", "validated_at_utc"):
        assert token in schema_text
    assert SHA256.fullmatch("a" * 64)
    for invalid in ("", "a" * 63, "a" * 65, "g" * 64, "a" * 32):
        assert not SHA256.fullmatch(invalid)


def test_detector_is_read_only_and_covers_required_sources():
    script = SCRIPT_PATH.read_text(encoding="utf-8")
    forbidden_commands = (
        "Invoke-WebRequest", "Invoke-RestMethod", "Start-BitsTransfer", "winget", "choco", "scoop",
        "Install-Package", "Install-Module", "Remove-Item", "Set-ItemProperty", "reg add", "git clone",
    )
    lowered = script.lower()
    for command in forbidden_commands:
        assert command.lower() not in lowered, command
    assert "ConvertTo-Json" in script
    assert "Get-FileHash" in script
    for token in (
        "LauncherInstalled.dat",
        "vswhere.exe",
        "Windows Kits",
        "NETFXSDK",
        "MUJOCO_HOME",
        "blender.exe",
        "LockFile",
    ):
        assert token in script


def test_detector_default_repo_root_is_the_repository_not_its_parent():
    script = SCRIPT_PATH.read_text(encoding="utf-8")

    assert "[string]$RepoRoot = (Split-Path -Parent (Split-Path -Parent $PSScriptRoot))" in script


def test_detector_reuses_the_fail_closed_netfxsdk_probe() -> None:
    script = SCRIPT_PATH.read_text(encoding="utf-8")

    assert "windows_netfxsdk.ps1" in script
    assert "-AllowMissing" in script
    assert "function Find-NetFxSdkRoot" not in script

from __future__ import annotations

import json
import re
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "runtime" / "visual" / "RobotSimUE"
PLUGIN = PROJECT / "Plugins" / "LingTuSim"
TRIPO_PLUGIN = PROJECT / "Plugins" / "Tripo3DUEBridge"


def _implementation_files() -> list[Path]:
    source_roots = (PROJECT / "Source", PLUGIN / "Source")
    return sorted(
        path
        for source_root in source_roots
        for path in source_root.rglob("*")
        if _is_production_implementation_file(path)
    )


def _is_production_implementation_file(path: Path) -> bool:
    if not path.is_file():
        return False
    if any(part.lower() in {"test", "tests"} for part in path.relative_to(PROJECT).parts):
        return False
    if _is_test_file(path):
        return False
    if path.suffix.lower() in {".h", ".cpp"}:
        return True
    return path.name.endswith((".Build.cs", ".Target.cs"))


def _is_test_file(path: Path) -> bool:
    stem = path.stem.lower()
    return (
        stem.startswith("test_")
        or stem.endswith("_test")
        or stem.endswith("test")
        or stem.endswith("tests")
    )


def test_implementation_files_cover_production_without_unreal_tests() -> None:
    files = set(_implementation_files())

    assert PROJECT / "Source" / "RobotSimUE" / "Private" / "RobotSimUE.cpp" in files
    assert (
        PLUGIN
        / "Source"
        / "LingTuSimRuntime"
        / "Private"
        / "LingTuSimBundleLoader.cpp"
    ) in files
    assert (
        PLUGIN
        / "Source"
        / "LingTuSimVisual"
        / "Private"
        / "LingTuSimVisualWorldSubsystem.cpp"
    ) in files
    assert (
        PLUGIN
        / "Source"
        / "LingTuSimVisual"
        / "Private"
        / "Tests"
        / "LingTuSimVisualRuntimeTest.cpp"
    ) not in files


def test_has_one_project_and_runtime_plugin() -> None:
    assert list(PROJECT.rglob("*.uproject")) == [PROJECT / "RobotSimUE.uproject"]
    project = json.loads((PROJECT / "RobotSimUE.uproject").read_text(encoding="utf-8"))
    assert [module["Name"] for module in project["Modules"]] == ["RobotSimUE"]
    assert project["Plugins"] == [
        {"Name": "LingTuSim", "Enabled": True},
        {"Name": "PythonScriptPlugin", "Enabled": True},
        {"Name": "EditorScriptingUtilities", "Enabled": True},
        {"Name": "ModelContextProtocol", "Enabled": True},
        {"Name": "AllToolsets", "Enabled": True},
        {"Name": "Terminal", "Enabled": True},
        {
            "Name": "Tripo3DUEBridge",
            "Enabled": True,
            "SupportedTargetPlatforms": ["Win64"],
        },
    ]
    assert project["EngineAssociation"] == ""

    development_launcher = (PROJECT / "Scripts" / "ue_dev.ps1").read_text(
        encoding="utf-8"
    )
    for plugin in ("ModelContextProtocol", "AllToolsets", "Terminal"):
        assert plugin in development_launcher
    assert '"-EnablePlugins=$($DevelopmentPlugins -join \',\')"' in development_launcher

    plugin = json.loads((PLUGIN / "LingTuSim.uplugin").read_text(encoding="utf-8"))
    expected = {
        "LingTuSimRuntime",
        "LingTuSimSession",
        "LingTuSimVisual",
        "LingTuSimSensors",
        "LingTuSimUI",
    }
    assert {module["Name"] for module in plugin["Modules"]} == expected
    assert all(module["Type"] == "Runtime" for module in plugin["Modules"])
    assert plugin["SupportedTargetPlatforms"] == ["Win64"]
    assert not list((PLUGIN / "Source" / "LingTuSimMujoco").rglob("*"))
    assert not list((PLUGIN / "Source" / "LingTuSimDiagnostics").rglob("*"))


def test_tripo_bridge_is_a_version_bound_editor_only_candidate_asset_tool() -> None:
    descriptor = json.loads(
        (TRIPO_PLUGIN / "Tripo3DUEBridge.uplugin").read_text(encoding="utf-8")
    )
    assert descriptor["VersionName"] == "1.0.4"
    assert descriptor["EngineVersion"] == "5.8.0"
    assert descriptor["SupportedTargetPlatforms"] == ["Win64"]
    assert descriptor["Modules"] == [
        {
            "Name": "Tripo3DUEBridge",
            "Type": "Editor",
            "LoadingPhase": "Default",
        }
    ]

    modules = json.loads(
        (
            TRIPO_PLUGIN
            / "Binaries"
            / "Win64"
            / "UnrealEditor.modules"
        ).read_text(encoding="utf-8")
    )
    assert modules["BuildId"].isdigit()
    assert modules["Modules"] == {
        "Tripo3DUEBridge": "UnrealEditor-Tripo3DUEBridge.dll"
    }

    protocol = (
        TRIPO_PLUGIN
        / "Source"
        / "Tripo3DUEBridge"
        / "Public"
        / "TripoProtocol.h"
    ).read_text(encoding="utf-8")
    assert 'SERVER_HOST = TEXT("127.0.0.1")' in protocol
    assert 'ASSET_IMPORT_ROOT = TEXT("/Game/TripoModels")' in protocol
    assert "0.0.0.0" not in protocol

    provenance = json.loads(
        (TRIPO_PLUGIN / "LINGTU_PROVENANCE.json").read_text(encoding="utf-8")
    )
    assert provenance["source"]["archive_sha256"] == (
        "6f1fb698eecae1d702707cedf737d2e2f064f666c62f60e673faeca7a1a60e05"
    )
    assert provenance["source"]["license_file_present"] is False
    assert provenance["qualification"] == {
        "distribution": "unqualified",
        "allowed_usage": "local_editor_candidate_asset_import",
        "runtime_dependency": False,
        "world_package_authority": False,
        "collision_authority": False,
    }


def test_has_targets_and_module_registration() -> None:
    source = PROJECT / "Source"
    for name in ("RobotSimUE.Target.cs", "RobotSimUEEditor.Target.cs"):
        assert (source / name).is_file()
    module_source = source / "RobotSimUE"
    assert (module_source / "RobotSimUE.Build.cs").is_file()
    assert "IMPLEMENT_PRIMARY_GAME_MODULE" in (module_source / "Private" / "RobotSimUE.cpp").read_text(encoding="utf-8")

    modules = json.loads((PLUGIN / "LingTuSim.uplugin").read_text(encoding="utf-8"))["Modules"]
    for module in modules:
        root = PLUGIN / "Source" / module["Name"]
        assert (root / f"{module['Name']}.Build.cs").is_file()
        cpp = list((root / "Private").glob("*.cpp"))
        assert cpp and any(
            re.search(
                rf"IMPLEMENT_MODULE\([^,]+,\s*{module['Name']}\s*\)",
                path.read_text(encoding="utf-8"),
            )
            for path in cpp
        )


def test_json_and_build_cs_static_references_are_closed() -> None:
    project = json.loads((PROJECT / "RobotSimUE.uproject").read_text(encoding="utf-8"))
    plugin = json.loads((PLUGIN / "LingTuSim.uplugin").read_text(encoding="utf-8"))
    assert project["Modules"] == [
        {"Name": "RobotSimUE", "Type": "Runtime", "LoadingPhase": "Default"}
    ]
    enabled_plugin_dependencies = {
        dependency["Name"]
        for dependency in plugin.get("Plugins", [])
        if dependency.get("Enabled") is True
    }
    assert "PlatformCrypto" in enabled_plugin_dependencies

    module_names = {module["Name"] for module in plugin["Modules"]}
    allowed_engine_modules = {
        "Core",
        "CoreUObject",
        "Engine",
        "InputCore",
        "Json",
        "JsonUtilities",
        "Networking",
        "PlatformCryptoContext",
        "RenderCore",
        "RHI",
        "Slate",
        "SlateCore",
        "Sockets",
    }
    for module_name in module_names:
        build_cs = PLUGIN / "Source" / module_name / f"{module_name}.Build.cs"
        assert build_cs.is_file()
        dependencies = set(re.findall(r'"([A-Za-z0-9_]+)"', build_cs.read_text(encoding="utf-8")))
        assert dependencies <= allowed_engine_modules | module_names
        for dependency in dependencies & module_names:
            assert (PLUGIN / "Source" / dependency / f"{dependency}.Build.cs").is_file()


def test_sensors_public_api_does_not_export_visual_or_rendering_dependencies() -> None:
    build_rules = (
        PLUGIN
        / "Source"
        / "LingTuSimSensors"
        / "LingTuSimSensors.Build.cs"
    ).read_text(encoding="utf-8")
    public_block = re.search(
        r"PublicDependencyModuleNames\.AddRange\(new\[\]\s*\{(?P<body>.*?)\}\);",
        build_rules,
        re.DOTALL,
    )
    assert public_block is not None
    public_dependencies = set(re.findall(r'"([A-Za-z0-9_]+)"', public_block.group("body")))
    assert {"LingTuSimVisual", "Json", "RenderCore", "RHI"}.isdisjoint(public_dependencies)


def test_runtime_is_plan_boundary_not_source_model_parser() -> None:
    implementation = "\n".join(
        path.read_text(encoding="utf-8") for path in _implementation_files()
    )
    for token in (
        "RobotConfig",
        "MJCF",
        "mjs_loadXML",
        "mj_loadXML",
        "FXmlFile",
        "yaml",
        "YAML",
        "mjData",
    ):
        assert token not in implementation
    assert not re.search(r"\b21\b", implementation)
    assert not re.search(r"four\s+legs", implementation, re.IGNORECASE)
    assert "FSessionBundleView" in implementation
    assert "FSnapshotEnvelope" in implementation


def test_compiled_artifact_boundary_and_runtime_contract() -> None:
    runtime_types = (
        PLUGIN / "Source" / "LingTuSimRuntime" / "Public" / "LingTuSimRuntimeTypes.h"
    ).read_text(encoding="utf-8")
    for artifact in (
        "physics.plan.json",
        "visual.plan.json",
        "sensor.plan.json",
        "control.plan.json",
        "scenario.plan.json",
        "transport.intent.json",
    ):
        assert artifact in runtime_types
    for field in (
        "SessionId",
        "ModelGeneration",
        "ResetGeneration",
        "SimTimeNs",
        "StableId",
        "InstanceId",
        "FrameId",
    ):
        assert field in runtime_types

    sensor_sources = "\n".join(
        path.read_text(encoding="utf-8")
        for path in (PLUGIN / "Source" / "LingTuSimSensors").rglob("*.h")
    )
    assert "PixelStreaming" not in sensor_sources


def test_readme_records_validated_windows_build_without_shipping_claim() -> None:
    readme = (PROJECT / "README.md").read_text(encoding="utf-8")
    normalized = re.sub(r"\s+", " ", readme)
    assert "Unreal Engine 5.8.1" in readme
    assert "MuJoCo 3.10.0" in readme
    assert "RobotSimUEEditor Win64 Development" in readme
    assert "not yet a claim that cooking, packaging, live sensors" in normalized


def test_preview_renderer_enables_lumen_and_virtual_shadows() -> None:
    config = (PROJECT / "Config" / "DefaultEngine.ini").read_text(encoding="utf-8")

    assert "r.DynamicGlobalIlluminationMethod=1" in config
    assert "r.ReflectionMethod=1" in config
    assert "r.Shadow.Virtual.Enable=1" in config
    assert "r.GenerateMeshDistanceFields=True" in config
    assert "[/Script/WindowsTargetPlatform.WindowsTargetSettings]" in config
    assert "DefaultGraphicsRHI=DefaultGraphicsRHI_DX12" in config
    assert "-D3D12TargetedShaderFormats=PCD3D_SM5" in config
    assert "+D3D12TargetedShaderFormats=PCD3D_SM6" in config


def test_thunderv4_preview_entry_is_parseable_and_builds_canonical_physics_host_when_needed() -> None:
    script = PROJECT / "Scripts" / "run_thunderv4_preview.ps1"
    result = subprocess.run(
        [
            "powershell",
            "-NoProfile",
            "-Command",
            f"$null = [scriptblock]::Create((Get-Content -LiteralPath '{script}' -Raw)); 'ok'",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    assert result.stdout.strip() == "ok"

    source = script.read_text(encoding="utf-8")
    assert "[string]$UnrealAutoSdkRoot" in source
    assert "[string]$MujocoHost" in source
    assert "[string]$MuJoCoRoot" in source
    assert "[switch]$Unattended" in source
    assert "[switch]$RebuildMaterials" in source
    assert "[string]$DerivedDataCacheRoot" in source
    assert "sim\\runtime\\physics" in source
    assert "build\\mujoco-runtime-physics-win" in source
    assert "-DMUJOCO_ROOT=$MuJoCoRoot" in source
    assert "$env:UE_SDKS_ROOT" in source
    assert "thunderv4_controlled_headless\\session.yaml" in source
    assert "[string]$SnapshotPath" in source
    assert "[string]$RobotPackagePath" in source
    assert "[string]$RobotMjcfPath" in source
    assert "[string]$VisualMeshRoot" in source
    assert "[string]$FbxOutputDir" in source
    assert "[int]$MaxTrianglesPerAsset = 60000" in source
    assert "[int]$MaxTotalTriangles = 1100000" in source
    assert "'--max-triangles-per-asset', $MaxTrianglesPerAsset" in source
    assert "'--max-total-triangles', $MaxTotalTriangles" in source
    assert "@('--source-mesh-root', $visualMeshRoot)" in source
    assert "Explicit SimulationSnapshot is missing" in source
    assert "--mujoco-host $headlessExe" in source
    assert "--snapshot $snapshot" in source
    assert "--keyframe v4_nominal_stand" not in source
    assert "Explicit MuJoCo host is missing" in source
    assert """('-ExecCmds="py {0}"' -f $previewScript)""" in source
    assert "-ExecutePythonScript" not in source


def test_thunderv4_preview_uses_one_writable_lane_local_ddc() -> None:
    source = (PROJECT / "Scripts" / "run_thunderv4_preview.ps1").read_text(
        encoding="utf-8"
    )
    config = (PROJECT / "Config" / "DefaultEngine.ini").read_text(encoding="utf-8")

    assert "Join-Path $buildRoot 'unreal-ddc\\thunderv4-preview'" in source
    assert "DerivedDataCacheRoot must remain inside the repository build root" in source
    assert "New-Item -ItemType Directory -Path $derivedDataCache -Force" in source
    assert "LingTu DDC write probe" in source
    assert "Flush($true)" in source
    assert "${env:UE-LocalDataCachePath} = $derivedDataCache" in source
    assert "${env:UE-SharedDataCachePath} = 'None'" in source
    assert "'-DDC=LingTuPreview'" in source
    assert """('-LocalDataCachePath="{0}"' -f $derivedDataCache)""" in source
    assert "-DDC-ForceMemoryCache" not in source
    assert "Using data cache path" in source
    assert "Writable" in source
    assert "unexpected writable DDC stores" in source
    assert "[DerivedDataCacheGraphs]" in config
    assert (
        "LingTuPreview=(ProjectPak, InstalledProjectPak, "
        "EnginePak=InstalledEnginePak, Local=InstalledLocal)"
    ) in config


def test_thunderv4_preview_quotes_every_argument_that_can_contain_spaces() -> None:
    source = (PROJECT / "Scripts" / "run_thunderv4_preview.ps1").read_text(
        encoding="utf-8"
    )

    assert """('"{0}"' -f $project)""" in source
    assert """('-ExecCmds="py {0}"' -f $previewScript)""" in source
    assert """('-LocalDataCachePath="{0}"' -f $derivedDataCache)""" in source
    assert """('-abslog="{0}"' -f $unrealLog)""" in source


def test_thunderv4_preview_entry_uses_success_and_error_sentinels() -> None:
    powershell_source = (PROJECT / "Scripts" / "run_thunderv4_preview.ps1").read_text(
        encoding="utf-8"
    )
    python_source = (PROJECT / "Scripts" / "build_thunderv4_preview.py").read_text(
        encoding="utf-8"
    )

    for name in (
        "LINGTU_THUNDER_PREVIEW_SUCCESS",
        "LINGTU_THUNDER_PREVIEW_ERROR",
        "LINGTU_THUNDER_PREVIEW_UNATTENDED",
        "LINGTU_THUNDER_REBUILD_MATERIALS",
    ):
        assert name in powershell_source
        assert name in python_source

    assert "lingtu.sim.unreal-offline-preview-success.v1" in python_source
    assert "traceback.format_exc()" in python_source
    assert "unreal.SystemLibrary.quit_editor()" in python_source
    assert "unreal.register_slate_post_tick_callback" in python_source
    assert "unreal.unregister_slate_post_tick_callback" in python_source
    assert "AutomationScheduler" not in python_source
    assert "Unreal preview Python failed" in powershell_source
    assert "Timed out waiting for the Unreal offline preview success sentinel" in powershell_source


def test_readme_describes_thunderv4_preview_as_offline_not_live_runtime() -> None:
    readme = (PROJECT / "README.md").read_text(encoding="utf-8")
    normalized = re.sub(r"\s+", " ", readme)

    assert "ThunderV4 offline preview entry" in readme
    assert "not the live runtime" in readme
    assert "does not stream simulator state into UE" in normalized
    assert "-MujocoHost" in readme
    assert "-MuJoCoRoot" in readme
    assert "-UnrealAutoSdkRoot" in readme
    assert "-Unattended" in readme

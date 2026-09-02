# ruff: noqa: S101

"""Rendering contracts for the FactoryPark_HF Unreal authoring script."""

from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / "sim" / "runtime" / "visual" / "RobotSimUE" / "Scripts" / "build_factory_park_hf.py"
RUNNER_PATH = SCRIPT_PATH.with_name("run_factory_park_hf.ps1")


def _load_builder_module():
    spec = importlib.util.spec_from_file_location("lingtu_factory_park_hf_rendering_builder", SCRIPT_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_master_material_usage_supports_hism_and_nanite_without_fallback() -> None:
    builder = _load_builder_module()
    instanced_usage = object()
    nanite_usage = object()
    builder.unreal = SimpleNamespace(
        MaterialUsage=SimpleNamespace(
            MATUSAGE_INSTANCED_STATIC_MESHES=instanced_usage,
            MATUSAGE_NANITE=nanite_usage,
        )
    )

    class MaterialLibrary:
        def __init__(self) -> None:
            self.enabled: set[object] = set()

        def set_base_material_usage(self, material, usage, enabled):
            assert material == "master"
            assert enabled is True
            self.enabled.add(usage)

        def has_material_usage(self, material, usage):
            assert material == "master"
            return usage in self.enabled

    evidence = builder._configure_master_material_usages("master", MaterialLibrary())

    assert evidence == {
        "InstancedStaticMeshes": True,
        "Nanite": True,
    }


def test_lumen_world_applies_manual_ev100_and_white_balance() -> None:
    builder = _load_builder_module()
    manual_exposure = object()
    white_balance = object()

    class SystemLibrary:
        commands: list[str] = []

        @classmethod
        def execute_console_command(cls, _context, command):
            cls.commands.append(command)

    builder.unreal = SimpleNamespace(
        AutoExposureMethod=SimpleNamespace(AEM_MANUAL=manual_exposure),
        TemperatureMethod=SimpleNamespace(TEMP_WHITE_BALANCE=white_balance),
        SystemLibrary=SystemLibrary,
    )

    class Settings:
        def __init__(self) -> None:
            self.values: dict[str, object] = {}

        def set_editor_property(self, name, value):
            self.values[name] = value

        def get_editor_property(self, name):
            return self.values[name]

    class PostProcessVolume:
        def __init__(self) -> None:
            self.settings = Settings()
            self.assigned_settings = None

        def get_editor_property(self, name):
            assert name == "settings"
            return self.settings

        def set_editor_property(self, name, value):
            assert name == "settings"
            self.assigned_settings = value

    post_process = PostProcessVolume()
    realism = {
        "lighting": {
            "unreal": {
                "global_illumination": "Lumen",
                "reflections": "Lumen",
                "shadow_method": "VirtualShadowMaps",
            },
            "exposure": {
                "mode": "manual",
                "ev100": 14.0,
                "white_balance_k": 5600,
            },
        }
    }

    evidence = builder._configure_lumen_world(post_process, realism)

    assert post_process.assigned_settings is post_process.settings
    assert post_process.settings.values == {
        "override_auto_exposure_method": True,
        "auto_exposure_method": manual_exposure,
        "override_auto_exposure_apply_physical_camera_exposure": True,
        "auto_exposure_apply_physical_camera_exposure": True,
        "override_camera_iso": True,
        "camera_iso": 100.0,
        "override_camera_shutter_speed": True,
        "camera_shutter_speed": 256.0,
        "override_depth_of_field_fstop": True,
        "depth_of_field_fstop": 8.0,
        "override_auto_exposure_bias": True,
        "auto_exposure_bias": 0.0,
        "override_temperature_type": True,
        "temperature_type": white_balance,
        "override_white_temp": True,
        "white_temp": 5600.0,
        "override_white_tint": True,
        "white_tint": 0.0,
    }
    assert evidence["exposure"] == {
        "mode": "manual_physical_camera",
        "ev100": 14.0,
        "camera_iso": 100.0,
        "camera_shutter_speed": 256.0,
        "camera_fstop": 8.0,
        "exposure_compensation": 0.0,
        "white_balance_k": 5600.0,
        "white_tint": 0.0,
        "verified": True,
    }


def test_sun_uses_neutral_color_temperature_and_physical_source_angle() -> None:
    builder = _load_builder_module()
    builder.unreal = SimpleNamespace(Color=lambda red, green, blue, alpha: (red, green, blue, alpha))

    class LightComponent:
        def __init__(self) -> None:
            self.values: dict[str, object] = {}

        def set_editor_property(self, name, value):
            self.values[name] = value

        def get_editor_property(self, name):
            return self.values[name]

    component = LightComponent()
    evidence = builder._configure_sun_component(
        component,
        {
            "illuminance_lux": 78000,
            "color_temperature_k": 5600,
            "angular_diameter_deg": 0.535,
        },
    )

    assert component.values == {
        "intensity": 78000.0,
        "light_color": (255, 255, 255, 255),
        "atmosphere_sun_light": True,
        "use_temperature": True,
        "temperature": 5600.0,
        "light_source_angle": 0.535,
    }
    assert evidence == {
        "illuminance_lux": 78000.0,
        "base_light_color_rgba": [255, 255, 255, 255],
        "color_temperature_k": 5600.0,
        "angular_diameter_deg": 0.535,
        "verified": True,
    }


def test_required_property_readback_compares_unreal_struct_values_semantically() -> None:
    builder = _load_builder_module()

    class Color:
        def __init__(self, red: int, green: int, blue: int, alpha: int) -> None:
            self.r = red
            self.g = green
            self.b = blue
            self.a = alpha

    class LightComponent:
        def __init__(self, *, corrupt_readback: bool = False) -> None:
            self.value: Color | None = None
            self.corrupt_readback = corrupt_readback

        def set_editor_property(self, name: str, value: Color) -> None:
            assert name == "light_color"
            self.value = value

        def get_editor_property(self, name: str) -> Color:
            assert name == "light_color"
            assert self.value is not None
            return Color(
                self.value.r,
                self.value.g,
                self.value.b - int(self.corrupt_readback),
                self.value.a,
            )

    expected = Color(255, 255, 255, 255)
    builder._set_required_editor_property(
        LightComponent(),
        "light_color",
        expected,
    )

    with pytest.raises(RuntimeError, match="property readback failed"):
        builder._set_required_editor_property(
            LightComponent(corrupt_readback=True),
            "light_color",
            expected,
        )


def test_rendering_refresh_updates_material_sun_and_exposure(monkeypatch) -> None:
    builder = _load_builder_module()

    class Actor:
        def __init__(self, label: str) -> None:
            self._label = label
            self.light_component = object()

        def get_actor_label(self) -> str:
            return self._label

    sun = Actor(builder.SUN_LABEL)
    post_process = Actor(builder.POST_PROCESS_LABEL)

    class ActorSubsystem:
        @staticmethod
        def get_all_level_actors() -> list[Actor]:
            return [sun, post_process]

    master_material = object()
    realism = {
        "lighting": {
            "sun": {
                "illuminance_lux": 78000.0,
                "color_temperature_k": 5600.0,
                "angular_diameter_deg": 0.535,
            },
            "exposure": {"mode": "manual", "ev100": 14.0},
        }
    }
    monkeypatch.setattr(
        builder,
        "_configure_master_material_usages",
        lambda material: (
            {"InstancedStaticMeshes": True, "Nanite": True}
            if material is master_material
            else pytest.fail("wrong master material")
        ),
    )
    monkeypatch.setattr(
        builder,
        "_configure_sun_component",
        lambda component, recipe: (
            {"verified": True, "illuminance_lux": recipe["illuminance_lux"]}
            if component is sun.light_component
            else pytest.fail("wrong sun component")
        ),
    )
    monkeypatch.setattr(
        builder,
        "_configure_lumen_world",
        lambda actor, _realism: (
            {"exposure": {"verified": True, "mode": "manual_physical_camera", "ev100": 14.0}}
            if actor is post_process
            else pytest.fail("wrong post process")
        ),
    )

    evidence = builder._refresh_existing_rendering_contract(
        ActorSubsystem(),
        realism,
        master_material,
    )

    assert evidence == {
        "material_usages": {"InstancedStaticMeshes": True, "Nanite": True},
        "sun": {"verified": True, "illuminance_lux": 78000.0},
        "lumen": {
            "exposure": {
                "verified": True,
                "mode": "manual_physical_camera",
                "ev100": 14.0,
            }
        },
        "verified": True,
    }


def test_launcher_rejects_missing_rendering_fix_evidence() -> None:
    runner = RUNNER_PATH.read_text(encoding="utf-8")
    source = SCRIPT_PATH.read_text(encoding="utf-8")

    assert "[switch]$RefreshRenderingOnly" in runner
    assert "LINGTU_FACTORY_PARK_HF_REFRESH_RENDERING_ONLY" in runner
    assert "rendering_contract_refresh_v1" in runner
    assert '"rendering_contract_refreshed": rendering_refresh is not None' in source

    for contract in (
        "master_graph.material_usages.InstancedStaticMeshes",
        "master_graph.material_usages.Nanite",
        "$exposureEvidence = $successJson.lighting.lumen.exposure",
        "$exposureEvidence.mode",
        "$exposureEvidence.ev100",
        "$exposureEvidence.white_balance_k",
        "$sunEvidence = $successJson.lighting.sun",
        "$sunEvidence.base_light_color_rgba",
        "$sunEvidence.angular_diameter_deg",
    ):
        assert contract in runner

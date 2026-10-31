"""Deterministic baked material textures for FactoryPark_HF."""

from __future__ import annotations

import importlib.util
import math
import struct
from pathlib import Path
from types import SimpleNamespace

from sim.tools.worlds.factory_park_hf.material_textures import (
    author_surface_textures,
    generate_surface_texture_set,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
BUILDER_PATH = (
    REPO_ROOT
    / "sim"
    / "runtime"
    / "visual"
    / "RobotSimUE"
    / "Scripts"
    / "build_factory_park_hf.py"
)


def _load_builder_module():
    spec = importlib.util.spec_from_file_location("factory_park_hf_material_builder", BUILDER_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _pixel(rgb: bytes, size: int, x: int, y: int) -> bytes:
    offset = 3 * (y * size + x)
    return rgb[offset : offset + 3]


def test_asphalt_and_concrete_textures_close_every_tile_seam() -> None:
    textures = generate_surface_texture_set(size=32)

    assert set(textures) == {"asphalt_worn", "concrete_weathered"}
    for texture_set in textures.values():
        for rgb in (texture_set.detail_rgb, texture_set.normal_rgb):
            for index in range(32):
                assert _pixel(rgb, 32, 0, index) == _pixel(rgb, 32, 31, index)
                assert _pixel(rgb, 32, index, 0) == _pixel(rgb, 32, index, 31)


def test_authored_textures_are_deterministic_rgb_png_files(tmp_path) -> None:
    first = author_surface_textures(tmp_path / "first", size=32)
    second = author_surface_textures(tmp_path / "second", size=32)

    assert first.keys() == second.keys()
    for profile in first:
        for channel in ("detail", "normal"):
            first_path = first[profile][channel]
            second_path = second[profile][channel]
            payload = first_path.read_bytes()
            assert payload == second_path.read_bytes()
            assert payload.startswith(b"\x89PNG\r\n\x1a\n")
            width, height, bit_depth, color_type = struct.unpack(">IIBB", payload[16:26])
            assert (width, height, bit_depth, color_type) == (32, 32, 8, 2)


def test_ue_texture_import_is_named_synchronous_replaceable_and_saved(tmp_path) -> None:
    builder = _load_builder_module()

    class AssetImportTask:
        def __init__(self) -> None:
            self.values: dict[str, object] = {}

        def set_editor_property(self, name, value) -> None:
            self.values[name] = value

    builder.unreal = SimpleNamespace(AssetImportTask=AssetImportTask)
    source = tmp_path / "asphalt_worn_normal.png"
    task = builder._configure_texture_import_task(source, "T_FP_AsphaltWorn_Normal")

    assert task.values == {
        "filename": str(source),
        "destination_path": builder.MATERIAL_TEXTURE_ASSET_ROOT,
        "destination_name": "T_FP_AsphaltWorn_Normal",
        "automated": True,
        "replace_existing": True,
        "replace_existing_settings": True,
        "save": True,
        "async_": False,
    }


def test_only_selected_profiles_bind_world_scaled_detail_textures() -> None:
    builder = _load_builder_module()

    class Texture:
        def __init__(self, path: str) -> None:
            self.path = path

        def get_path_name(self) -> str:
            return self.path

    class Library:
        def __init__(self) -> None:
            self.scalars: dict[str, float] = {}
            self.vectors: dict[str, object] = {}
            self.textures: dict[str, Texture] = {}

        def set_material_instance_scalar_parameter_value(self, _instance, name, value):
            self.scalars[str(name)] = value
            return True

        def get_material_instance_scalar_parameter_value(self, _instance, name):
            return self.scalars[str(name)]

        def set_material_instance_vector_parameter_value(self, _instance, name, value):
            self.vectors[str(name)] = value
            return True

        def get_material_instance_vector_parameter_value(self, _instance, name):
            return self.vectors[str(name)]

        def set_material_instance_texture_parameter_value(self, _instance, name, value):
            self.textures[str(name)] = value
            return True

        def get_material_instance_texture_parameter_value(self, _instance, name):
            return self.textures[str(name)]

    builder.unreal = SimpleNamespace(
        Name=str,
        LinearColor=lambda r, g, b, a: SimpleNamespace(r=r, g=g, b=b, a=a),
    )
    textures = {
        "asphalt_worn": {
            "detail": Texture("/Game/T_Asphalt_Detail"),
            "normal": Texture("/Game/T_Asphalt_Normal"),
            "tile_size_cm": 200.0,
        }
    }
    selected_library = Library()
    selected = builder._apply_surface_detail_parameters(
        "instance",
        {"profile": "asphalt_worn"},
        textures,
        selected_library,
    )

    assert selected["enabled"] is True
    assert selected["mapping"] == "world_aligned_triplanar_cm"
    assert selected["tile_size_cm"] == 200.0
    assert selected_library.scalars == {"DetailEnabled": 1.0}
    assert set(selected_library.textures) == {"DetailRoughnessTexture", "DetailNormalTexture"}
    tile_size = selected_library.vectors["DetailTileSizeCm"]
    assert (tile_size.r, tile_size.g, tile_size.b, tile_size.a) == (200.0, 200.0, 200.0, 1.0)

    flat_library = Library()
    flat = builder._apply_surface_detail_parameters(
        "instance",
        {"profile": "warehouse_galvanized"},
        textures,
        flat_library,
    )
    assert flat == {
        "enabled": False,
        "profile": "warehouse_galvanized",
        "flat_material_preserved": True,
    }
    assert flat_library.scalars == {"DetailEnabled": 0.0}
    assert flat_library.vectors == {}
    assert flat_library.textures == {}


def test_imported_texture_settings_preserve_linear_detail_and_normal_integrity() -> None:
    builder = _load_builder_module()
    wrap = object()
    normal_compression = object()
    default_compression = object()
    from_group_mips = object()
    world = object()
    world_normal = object()
    builder.unreal = SimpleNamespace(
        TextureAddress=SimpleNamespace(TA_WRAP=wrap),
        TextureCompressionSettings=SimpleNamespace(
            TC_DEFAULT=default_compression,
            TC_NORMALMAP=normal_compression,
        ),
        TextureMipGenSettings=SimpleNamespace(TMGS_FROM_TEXTURE_GROUP=from_group_mips),
        TextureGroup=SimpleNamespace(
            TEXTUREGROUP_WORLD=world,
            TEXTUREGROUP_WORLD_NORMAL_MAP=world_normal,
        ),
        EditorAssetLibrary=SimpleNamespace(
            save_loaded_asset=lambda _texture, only_if_is_dirty: not only_if_is_dirty
        ),
    )

    class Texture:
        def __init__(self, path: str) -> None:
            self.path = path
            self.values: dict[str, object] = {}

        def set_editor_property(self, name, value) -> None:
            self.values[name] = value

        def get_editor_property(self, name):
            return self.values[name]

        def get_path_name(self) -> str:
            return self.path

    detail = Texture("/Game/T_Detail")
    normal = Texture("/Game/T_Normal")
    detail_evidence = builder._configure_imported_detail_texture(detail, channel="detail")
    normal_evidence = builder._configure_imported_detail_texture(normal, channel="normal")

    assert detail.values == {
        "srgb": False,
        "compression_settings": default_compression,
        "address_x": wrap,
        "address_y": wrap,
        "mip_gen_settings": from_group_mips,
        "lod_group": world,
    }
    assert normal.values == {
        "srgb": False,
        "compression_settings": normal_compression,
        "address_x": wrap,
        "address_y": wrap,
        "mip_gen_settings": from_group_mips,
        "lod_group": world_normal,
    }
    assert detail_evidence["verified"] is True
    assert normal_evidence["normal_compression"] is True


def test_generated_normals_are_varied_forward_facing_unit_vectors() -> None:
    textures = generate_surface_texture_set(size=32)

    for texture_set in textures.values():
        normals = {
            tuple(texture_set.normal_rgb[offset : offset + 3])
            for offset in range(0, len(texture_set.normal_rgb), 3)
        }
        assert len(normals) > 32
        for red, green, blue in normals:
            vector = (
                red / 127.5 - 1.0,
                green / 127.5 - 1.0,
                blue / 127.5 - 1.0,
            )
            assert 0.985 <= math.sqrt(sum(channel * channel for channel in vector)) <= 1.015
            assert vector[2] > 0.0


def test_master_material_consumes_world_aligned_detail_and_normal() -> None:
    builder = _load_builder_module()

    class Expression:
        def __init__(self, kind: object) -> None:
            self.kind = kind
            self.values: dict[str, object] = {}
            self.material_function = None

        def set_editor_property(self, name, value) -> None:
            self.values[name] = value

        def set_material_function(self, material_function) -> bool:
            self.material_function = material_function
            return True

    class MaterialLibrary:
        connections: list[tuple[Expression, str, Expression, str]] = []
        outputs: list[tuple[Expression, str, object]] = []
        usages: set[object] = set()

        @staticmethod
        def delete_all_material_expressions(_material) -> None:
            return None

        @staticmethod
        def create_material_expression(_material, kind, _x, _y):
            return Expression(kind)

        @staticmethod
        def get_material_expression_input_names(_expression):
            return ["A", "B", "Alpha", "TextureObject", "TextureSize"]

        @staticmethod
        def get_material_expression_output_names(_expression):
            return ["", "XYZ Texture"]

        @classmethod
        def connect_material_expressions(cls, source, output, target, input_name):
            cls.connections.append((source, output, target, input_name))
            return True

        @classmethod
        def connect_material_property(cls, expression, output, property_):
            cls.outputs.append((expression, output, property_))
            return True

        @classmethod
        def set_base_material_usage(cls, _material, usage, enabled):
            assert enabled is True
            cls.usages.add(usage)

        @classmethod
        def has_material_usage(cls, _material, usage):
            return usage in cls.usages

        @staticmethod
        def layout_material_expressions(_material) -> None:
            return None

        @staticmethod
        def recompile_material(_material):
            return []

    class EditorAssetLibrary:
        @staticmethod
        def set_metadata_tag(_asset, _key, _value) -> None:
            return None

        @staticmethod
        def save_loaded_asset(_asset, only_if_is_dirty) -> None:
            assert only_if_is_dirty is False

    classes = {
        name: object()
        for name in (
            "MaterialExpressionVectorParameter",
            "MaterialExpressionScalarParameter",
            "MaterialExpressionTextureObjectParameter",
            "MaterialExpressionMaterialFunctionCall",
            "MaterialExpressionComponentMask",
            "MaterialExpressionLinearInterpolate",
            "MaterialExpressionConstant3Vector",
            "MaterialExpressionSaturate",
        )
    }
    normal_property = object()
    builder.unreal = SimpleNamespace(
        **classes,
        Material=object,
        MaterialFactoryNew=lambda: object(),
        MaterialEditingLibrary=MaterialLibrary,
        MaterialSamplerType=SimpleNamespace(SAMPLERTYPE_COLOR=object(), SAMPLERTYPE_NORMAL=object()),
        MaterialUsage=SimpleNamespace(
            MATUSAGE_INSTANCED_STATIC_MESHES=object(),
            MATUSAGE_NANITE=object(),
        ),
        MaterialProperty=SimpleNamespace(
            MP_BASE_COLOR=object(),
            MP_METALLIC=object(),
            MP_ROUGHNESS=object(),
            MP_SPECULAR=object(),
            MP_NORMAL=normal_property,
        ),
        Name=str,
        LinearColor=lambda r, g, b, a: SimpleNamespace(r=r, g=g, b=b, a=a),
        load_asset=lambda path: SimpleNamespace(path=path),
        log=lambda _message: None,
        EditorAssetLibrary=EditorAssetLibrary,
    )
    builder._create_or_load_asset = lambda *_args: SimpleNamespace(get_path_name=lambda: "/Game/M")

    _master, evidence = builder._build_master_pbr_material()

    assert evidence["surface_detail_mapping"] == "world_aligned_triplanar_cm"
    assert "normal" in evidence["material_outputs"]
    assert any(property_ is normal_property for _, _, property_ in MaterialLibrary.outputs)
    assert "detail_normal_object" in evidence["expression_connections"]
    assert "detail_normal_enabled" in evidence["expression_connections"]


def test_material_runner_accepts_textured_graph_without_requiring_legacy_four_outputs():
    runner = BUILDER_PATH.with_name("run_factory_park_hf.ps1").read_text(encoding="utf-8")
    assert "'surface_detail_mapping' -in $masterGraph.PSObject.Properties.Name" in runner
    assert "'normal' -notin @($masterGraph.material_outputs)" in runner
    assert "surface_detail_textures.imported_texture_count -ne 4" in runner


def test_generated_pngs_are_imported_into_the_consumed_texture_library(tmp_path) -> None:
    builder = _load_builder_module()
    builder.MATERIAL_TEXTURE_SOURCE_ROOT = tmp_path
    wrap = object()
    normal_compression = object()
    default_compression = object()
    from_group_mips = object()
    world = object()
    world_normal = object()

    class Texture2D:
        def __init__(self, path: str) -> None:
            self.path = path
            self.values: dict[str, object] = {}

        def set_editor_property(self, name, value) -> None:
            self.values[name] = value

        def get_editor_property(self, name):
            return self.values[name]

        def get_path_name(self) -> str:
            return self.path

    class AssetImportTask:
        def __init__(self) -> None:
            self.values: dict[str, object] = {}
            self.objects: list[Texture2D] = []

        def set_editor_property(self, name, value) -> None:
            self.values[name] = value

        def get_objects(self):
            return self.objects

    class AssetTools:
        imported_tasks: list[AssetImportTask] = []

        @classmethod
        def import_asset_tasks(cls, tasks) -> None:
            cls.imported_tasks = list(tasks)
            for task in cls.imported_tasks:
                name = task.values["destination_name"]
                root = builder.MATERIAL_TEXTURE_ASSET_ROOT
                task.objects = [Texture2D(f"{root}/{name}.{name}")]

    builder.unreal = SimpleNamespace(
        AssetImportTask=AssetImportTask,
        AssetToolsHelpers=SimpleNamespace(get_asset_tools=lambda: AssetTools),
        Texture2D=Texture2D,
        TextureAddress=SimpleNamespace(TA_WRAP=wrap),
        TextureCompressionSettings=SimpleNamespace(
            TC_DEFAULT=default_compression,
            TC_NORMALMAP=normal_compression,
        ),
        TextureMipGenSettings=SimpleNamespace(TMGS_FROM_TEXTURE_GROUP=from_group_mips),
        TextureGroup=SimpleNamespace(
            TEXTUREGROUP_WORLD=world,
            TEXTUREGROUP_WORLD_NORMAL_MAP=world_normal,
        ),
        EditorAssetLibrary=SimpleNamespace(
            save_loaded_asset=lambda _texture, only_if_is_dirty: not only_if_is_dirty
        ),
    )

    textures, evidence = builder._build_surface_detail_texture_library()

    assert set(textures) == {"asphalt_worn", "concrete_weathered"}
    assert all(set(texture_set) == {"detail", "normal", "tile_size_cm"} for texture_set in textures.values())
    assert len(AssetTools.imported_tasks) == 4
    assert evidence["generated_file_count"] == 4
    assert evidence["imported_texture_count"] == 4
    assert evidence["all_import_settings_verified"] is True
    assert evidence["rendered_validation_performed"] is False

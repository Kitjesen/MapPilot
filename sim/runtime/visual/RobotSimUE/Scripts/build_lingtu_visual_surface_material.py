"""Create or validate the generic LingTu visual surface material.

Run with UnrealEditor-Cmd/UnrealEditor:
  -ExecutePythonScript=Scripts/build_lingtu_visual_surface_material.py
"""

from __future__ import annotations

import unreal


MATERIAL_DIR = "/Game/RobotSim/Materials"
MATERIAL_NAME = "M_LingTuVisualSurface"
MATERIAL_PATH = f"{MATERIAL_DIR}/{MATERIAL_NAME}"


def _asset_tools() -> unreal.AssetTools:
    return unreal.AssetToolsHelpers.get_asset_tools()


def _connect_parameter(
    material: unreal.Material,
    expression_class: type,
    parameter_name: str,
    default_value: object,
    x: int,
    y: int,
    property_id: unreal.MaterialProperty,
) -> None:
    expression = unreal.MaterialEditingLibrary.create_material_expression(
        material,
        expression_class,
        x,
        y,
    )
    expression.set_editor_property("parameter_name", parameter_name)
    if isinstance(default_value, unreal.LinearColor):
        expression.set_editor_property("default_value", default_value)
    else:
        expression.set_editor_property("default_value", float(default_value))
    unreal.MaterialEditingLibrary.connect_material_property(
        expression,
        "",
        property_id,
    )


def build_material() -> unreal.Material:
    unreal.EditorAssetLibrary.make_directory(MATERIAL_DIR)
    material = unreal.load_asset(MATERIAL_PATH)
    if material is None:
        material = _asset_tools().create_asset(
            MATERIAL_NAME,
            MATERIAL_DIR,
            unreal.Material,
            unreal.MaterialFactoryNew(),
        )
    if material is None or not isinstance(material, unreal.Material):
        raise RuntimeError(f"could not create {MATERIAL_PATH}")

    editing = unreal.MaterialEditingLibrary
    editing.delete_all_material_expressions(material)
    _connect_parameter(
        material,
        unreal.MaterialExpressionVectorParameter,
        "BaseColor",
        unreal.LinearColor(1.0, 1.0, 1.0, 1.0),
        -420,
        -120,
        unreal.MaterialProperty.MP_BASE_COLOR,
    )
    _connect_parameter(
        material,
        unreal.MaterialExpressionScalarParameter,
        "Metallic",
        0.0,
        -420,
        60,
        unreal.MaterialProperty.MP_METALLIC,
    )
    _connect_parameter(
        material,
        unreal.MaterialExpressionScalarParameter,
        "Roughness",
        0.55,
        -420,
        180,
        unreal.MaterialProperty.MP_ROUGHNESS,
    )
    editing.recompile_material(material)
    unreal.EditorAssetLibrary.set_metadata_tag(
        material,
        "LingTu.VisualSurface",
        "BaseColor,Metallic,Roughness",
    )
    unreal.EditorAssetLibrary.save_loaded_asset(material, only_if_is_dirty=False)
    return material


def validate_material(material: unreal.Material) -> None:
    if unreal.load_asset(MATERIAL_PATH) is None:
        raise RuntimeError(f"{MATERIAL_PATH} did not reload after save")
    metadata = unreal.EditorAssetLibrary.get_metadata_tag(
        material,
        "LingTu.VisualSurface",
    )
    if metadata != "BaseColor,Metallic,Roughness":
        raise RuntimeError(f"{MATERIAL_PATH} metadata validation failed")


def main() -> None:
    material = build_material()
    validate_material(material)
    unreal.log(f"LingTu visual surface material ready: {MATERIAL_PATH}")


if __name__ == "__main__":
    main()

"""Import the offline robot showroom FBX library into Unreal Editor.

Standalone commandlet usage from the RobotSimUE project directory::

    UnrealEditor-Cmd.exe RobotSimUE.uproject -unattended -nop4 \
        -run=pythonscript -script=Scripts/import_robot_showroom.py

The script imports only static assets.  It does not create a level, launch PIE,
or modify packaging configuration.
"""

from __future__ import annotations

import json
import math
import re
from pathlib import Path
from typing import Any

import unreal

REPOSITORY_ROOT = Path(__file__).resolve().parents[5]
SOURCE_DIR = REPOSITORY_ROOT / "build/robot-showroom"
MANIFEST_PATH = SOURCE_DIR / "manifest.json"
RESULT_PATH = SOURCE_DIR / "import-result.json"

SHOWROOM_ROOT = "/Game/RobotSim/FrontEnd/Showroom"
ROBOT_DESTINATION = f"{SHOWROOM_ROOT}/Robots"
MATERIAL_DESTINATION = f"{SHOWROOM_ROOT}/Materials"
HEIGHT_ABSOLUTE_TOLERANCE_CM = 0.5
HEIGHT_RELATIVE_TOLERANCE = 0.005

ROBOT_ID_PATTERN = re.compile(r"^[a-z0-9_]+$")
ROLE_PBR = {
    "silver": {"metallic": 0.50, "roughness": 0.42},
    "graphite": {"metallic": 0.32, "roughness": 0.46},
    "rubber": {"metallic": 0.0, "roughness": 0.68},
    "sensor": {"metallic": 0.34, "roughness": 0.30},
    "customrgba": {"metallic": 0.05, "roughness": 0.46},
}


def _finite_number(value: object, context: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RuntimeError(f"{context} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise RuntimeError(f"{context} must be finite")
    return result


def _normalize_emissive(value: object, context: str) -> dict[str, Any]:
    """Validate the optional manifest emissive contract for one material slot."""
    if value is None:
        return {"rgb": [0.0, 0.0, 0.0], "strength": 0.0}
    if not isinstance(value, dict):
        raise RuntimeError(f"{context} must be an object")
    rgb = value.get("rgb")
    if not isinstance(rgb, list) or len(rgb) != 3:
        raise RuntimeError(f"{context}.rgb must contain three numbers")
    normalized_rgb = [
        _finite_number(component, f"{context}.rgb") for component in rgb
    ]
    if any(component < 0.0 or component > 1.0 for component in normalized_rgb):
        raise RuntimeError(f"{context}.rgb is outside 0..1")
    strength = _finite_number(value.get("strength"), f"{context}.strength")
    if strength < 0.0:
        raise RuntimeError(f"{context}.strength must be non-negative")
    return {"rgb": normalized_rgb, "strength": strength}


def _load_manifest() -> list[dict[str, Any]]:
    if not MANIFEST_PATH.is_file():
        raise RuntimeError(f"showroom manifest does not exist: {MANIFEST_PATH}")
    payload = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    if not isinstance(payload, list) or not payload:
        raise RuntimeError("showroom manifest must be a non-empty top-level array")

    robots: list[dict[str, Any]] = []
    seen_ids: set[str] = set()
    seen_files: set[str] = set()
    for index, raw_robot in enumerate(payload):
        if not isinstance(raw_robot, dict):
            raise RuntimeError(f"manifest item {index} must be an object")
        robot = dict(raw_robot)
        robot_id = robot.get("id")
        if (
            not isinstance(robot_id, str)
            or ROBOT_ID_PATTERN.fullmatch(robot_id) is None
            or robot_id in seen_ids
        ):
            raise RuntimeError(f"manifest item {index} has an invalid or duplicate id")
        filename = robot.get("asset_filename")
        expected_filename = f"SM_{robot_id}.fbx"
        if filename != expected_filename or filename in seen_files:
            raise RuntimeError(
                f"{robot_id}: expected unique asset filename {expected_filename!r}"
            )
        source = SOURCE_DIR / filename
        if not source.is_file() or source.stat().st_size <= 0:
            raise RuntimeError(f"{robot_id}: FBX source is missing or empty: {source}")

        height_m = _finite_number(robot.get("real_height_m"), f"{robot_id}.real_height_m")
        if height_m <= 0.0:
            raise RuntimeError(f"{robot_id}: real_height_m must be positive")
        raw_slots = robot.get("material_slots")
        if not isinstance(raw_slots, list) or not raw_slots:
            raise RuntimeError(f"{robot_id}: material_slots must be a non-empty array")
        slots: list[dict[str, Any]] = []
        for expected_slot, raw_slot in enumerate(raw_slots):
            if not isinstance(raw_slot, dict) or raw_slot.get("slot") != expected_slot:
                raise RuntimeError(f"{robot_id}: material slot indices must be contiguous")
            role = raw_slot.get("role")
            if role not in ROLE_PBR:
                raise RuntimeError(f"{robot_id}: unsupported material role {role!r}")
            rgba = raw_slot.get("rgba")
            if not isinstance(rgba, list) or len(rgba) != 4:
                raise RuntimeError(f"{robot_id}: slot {expected_slot} must provide RGBA")
            normalized_rgba = [
                _finite_number(value, f"{robot_id}.material_slots[{expected_slot}].rgba")
                for value in rgba
            ]
            if any(value < 0.0 or value > 1.0 for value in normalized_rgba):
                raise RuntimeError(f"{robot_id}: slot {expected_slot} RGBA is outside 0..1")
            slot = dict(raw_slot)
            slot["rgba"] = normalized_rgba
            slot["emissive"] = _normalize_emissive(
                raw_slot.get("emissive"),
                f"{robot_id}.material_slots[{expected_slot}].emissive",
            )
            slot.update(ROLE_PBR[role])
            slots.append(slot)

        robot["source_path"] = source
        robot["asset_name"] = f"SM_{robot_id}"
        robot["asset_path"] = f"{ROBOT_DESTINATION}/SM_{robot_id}"
        robot["real_height_m"] = height_m
        robot["material_slots"] = slots
        robots.append(robot)
        seen_ids.add(robot_id)
        seen_files.add(filename)
    return robots


def _asset_tools() -> Any:
    return unreal.AssetToolsHelpers.get_asset_tools()


def _create_or_load_material(name: str) -> Any:
    asset_path = f"{MATERIAL_DESTINATION}/{name}"
    material = unreal.load_asset(asset_path) if unreal.EditorAssetLibrary.does_asset_exist(asset_path) else None
    if material is None:
        material = _asset_tools().create_asset(
            name,
            MATERIAL_DESTINATION,
            unreal.Material,
            unreal.MaterialFactoryNew(),
        )
    if material is None or not isinstance(material, unreal.Material):
        raise RuntimeError(f"could not create material {asset_path}")
    return material


def _expression(material: Any, expression_class: Any, x: int, y: int) -> Any:
    expression = unreal.MaterialEditingLibrary.create_material_expression(
        material, expression_class, x, y
    )
    if expression is None:
        raise RuntimeError(
            f"could not create {expression_class} for {material.get_path_name()}"
        )
    return expression


def _connect_expression(source: Any, output: str, target: Any, input_name: str) -> None:
    result = unreal.MaterialEditingLibrary.connect_material_expressions(
        source, output, target, input_name
    )
    if result is False:
        raise RuntimeError(f"could not connect material expression input {input_name}")


def _connect_property(expression: Any, property_id: Any) -> None:
    result = unreal.MaterialEditingLibrary.connect_material_property(
        expression, "", property_id
    )
    if result is False:
        raise RuntimeError(f"could not connect material property {property_id}")


def _constant(material: Any, value: float, x: int, y: int) -> Any:
    expression = _expression(material, unreal.MaterialExpressionConstant, x, y)
    expression.set_editor_property("r", value)
    return expression


def _color(material: Any, rgba: list[float], x: int, y: int) -> Any:
    expression = _expression(material, unreal.MaterialExpressionVectorParameter, x, y)
    expression.set_editor_property("parameter_name", unreal.Name("BaseColorRGBA"))
    expression.set_editor_property("default_value", unreal.LinearColor(*rgba))
    return expression


def _vector_parameter(
    material: Any,
    name: str,
    rgba: list[float],
    x: int,
    y: int,
) -> Any:
    expression = _expression(material, unreal.MaterialExpressionVectorParameter, x, y)
    expression.set_editor_property("parameter_name", unreal.Name(name))
    expression.set_editor_property("default_value", unreal.LinearColor(*rgba))
    return expression


def _scalar_parameter(material: Any, name: str, value: float, x: int, y: int) -> Any:
    expression = _expression(material, unreal.MaterialExpressionScalarParameter, x, y)
    expression.set_editor_property("parameter_name", unreal.Name(name))
    expression.set_editor_property("default_value", value)
    return expression


def _finish_material(material: Any, metadata: dict[str, str]) -> None:
    unreal.MaterialEditingLibrary.layout_material_expressions(material)
    compiler_errors = list(unreal.MaterialEditingLibrary.recompile_material(material))
    if compiler_errors:
        raise RuntimeError(
            f"material compile failed for {material.get_path_name()}: {compiler_errors}"
        )
    for key, value in metadata.items():
        unreal.EditorAssetLibrary.set_metadata_tag(material, key, value)
    if not unreal.EditorAssetLibrary.save_loaded_asset(
        material, only_if_is_dirty=False
    ):
        raise RuntimeError(f"could not save material {material.get_path_name()}")


def _build_robot_material(robot_id: str, slot: dict[str, Any]) -> Any:
    name = f"M_{robot_id}_{slot['slot']}"
    material = _create_or_load_material(name)
    editing = unreal.MaterialEditingLibrary
    editing.delete_all_material_expressions(material)
    base_color = _color(material, slot["rgba"], -420, -100)
    metallic = _constant(material, float(slot["metallic"]), -420, 40)
    roughness = _constant(material, float(slot["roughness"]), -420, 160)
    _connect_property(base_color, unreal.MaterialProperty.MP_BASE_COLOR)
    _connect_property(metallic, unreal.MaterialProperty.MP_METALLIC)
    _connect_property(roughness, unreal.MaterialProperty.MP_ROUGHNESS)
    emissive = slot["emissive"]
    if emissive["strength"] > 0.0:
        emissive_color = _vector_parameter(
            material,
            "EmissiveRGB",
            [*emissive["rgb"], 1.0],
            -420,
            300,
        )
        emissive_strength = _scalar_parameter(
            material,
            "EmissiveStrength",
            float(emissive["strength"]),
            -420,
            420,
        )
        emissive_output = _expression(
            material,
            unreal.MaterialExpressionMultiply,
            -160,
            340,
        )
        _connect_expression(emissive_color, "", emissive_output, "A")
        _connect_expression(emissive_strength, "", emissive_output, "B")
        _connect_property(
            emissive_output,
            unreal.MaterialProperty.MP_EMISSIVE_COLOR,
        )
    _finish_material(
        material,
        {
            "LingTu.ShowroomRobot": robot_id,
            "LingTu.ShowroomSlot": str(slot["slot"]),
            "LingTu.ShowroomRole": str(slot["role"]),
            "LingTu.ShowroomRGBA": ",".join(str(value) for value in slot["rgba"]),
            "LingTu.ShowroomEmissiveRGB": ",".join(
                str(value) for value in emissive["rgb"]
            ),
            "LingTu.ShowroomEmissiveStrength": str(emissive["strength"]),
        },
    )
    return material


def _build_surface_material(name: str = "M_Surface", *, two_sided: bool = False) -> Any:
    material = _create_or_load_material(name)
    material.set_editor_property("two_sided", two_sided)
    editing = unreal.MaterialEditingLibrary
    editing.delete_all_material_expressions(material)

    tint = _expression(material, unreal.MaterialExpressionVectorParameter, -760, -180)
    tint.set_editor_property("parameter_name", unreal.Name("Tint"))
    tint.set_editor_property(
        "default_value", unreal.LinearColor(0.82, 0.84, 0.86, 1.0)
    )
    roughness = _expression(material, unreal.MaterialExpressionScalarParameter, -760, 180)
    roughness.set_editor_property("parameter_name", unreal.Name("Roughness"))
    roughness.set_editor_property("default_value", 0.72)
    world_position = _expression(material, unreal.MaterialExpressionWorldPosition, -1040, 20)
    frequency = _constant(material, 0.0025, -1040, 130)
    scaled_position = _expression(material, unreal.MaterialExpressionMultiply, -820, 20)
    noise = _expression(material, unreal.MaterialExpressionNoise, -600, 20)
    noise.set_editor_property("levels", 1)
    noise.set_editor_property("quality", 1)
    noise.set_editor_property("turbulence", False)
    noise.set_editor_property("output_min", 0.0)
    noise.set_editor_property("output_max", 1.0)
    color_amplitude = _constant(material, 0.06, -560, -150)
    color_bias = _constant(material, 0.97, -560, -70)
    color_detail = _expression(material, unreal.MaterialExpressionMultiply, -360, -130)
    color_multiplier = _expression(material, unreal.MaterialExpressionAdd, -160, -110)
    modulated_tint = _expression(material, unreal.MaterialExpressionMultiply, 40, -150)
    roughness_amplitude = _constant(material, 0.04, -360, 170)
    roughness_detail = _expression(material, unreal.MaterialExpressionMultiply, -140, 150)
    modulated_roughness = _expression(material, unreal.MaterialExpressionAdd, 80, 160)
    clamped_roughness = _expression(material, unreal.MaterialExpressionSaturate, 300, 160)

    _connect_expression(world_position, "", scaled_position, "A")
    _connect_expression(frequency, "", scaled_position, "B")
    # UE 5.8 labels this pin from WorldPositionOriginType, not the C++ field name.
    position_pin = unreal.MaterialEditingLibrary.get_material_expression_input_names(noise)[0]
    _connect_expression(scaled_position, "", noise, position_pin)
    _connect_expression(noise, "", color_detail, "A")
    _connect_expression(color_amplitude, "", color_detail, "B")
    _connect_expression(color_detail, "", color_multiplier, "A")
    _connect_expression(color_bias, "", color_multiplier, "B")
    _connect_expression(tint, "", modulated_tint, "A")
    _connect_expression(color_multiplier, "", modulated_tint, "B")
    _connect_expression(noise, "", roughness_detail, "A")
    _connect_expression(roughness_amplitude, "", roughness_detail, "B")
    _connect_expression(roughness, "", modulated_roughness, "A")
    _connect_expression(roughness_detail, "", modulated_roughness, "B")
    _connect_expression(modulated_roughness, "", clamped_roughness, "")
    _connect_property(modulated_tint, unreal.MaterialProperty.MP_BASE_COLOR)
    _connect_property(clamped_roughness, unreal.MaterialProperty.MP_ROUGHNESS)
    _finish_material(
        material,
        {
            "LingTu.ShowroomMaterial": "Surface",
            "LingTu.ShowroomDetail": "low-frequency-world-position-noise;color+roughness",
        },
    )
    return material


def _build_sky_material() -> Any:
    material = _create_or_load_material("M_Sky")
    material.set_editor_property("two_sided", True)
    material.set_editor_property("shading_model", unreal.MaterialShadingModel.MSM_UNLIT)
    material.set_editor_property("is_sky", True)
    editing = unreal.MaterialEditingLibrary
    editing.delete_all_material_expressions(material)

    lower = _expression(material, unreal.MaterialExpressionVectorParameter, -500, -180)
    lower.set_editor_property("parameter_name", "HorizonColor")
    lower.set_editor_property("default_value", unreal.LinearColor(0.72, 0.84, 0.96, 1.0))
    upper = _expression(material, unreal.MaterialExpressionVectorParameter, -500, -60)
    upper.set_editor_property("parameter_name", "ZenithColor")
    upper.set_editor_property("default_value", unreal.LinearColor(0.98, 0.99, 1.0, 1.0))
    world_position = _expression(material, unreal.MaterialExpressionWorldPosition, -900, 100)
    world_z = _expression(material, unreal.MaterialExpressionComponentMask, -700, 100)
    world_z.set_editor_property("r", False)
    world_z.set_editor_property("g", False)
    world_z.set_editor_property("b", True)
    frequency = _constant(material, 0.001, -700, 220)
    scaled_z = _expression(material, unreal.MaterialExpressionMultiply, -500, 100)
    midpoint = _constant(material, 0.5, -500, 220)
    biased_z = _expression(material, unreal.MaterialExpressionAdd, -300, 100)
    gradient = _expression(material, unreal.MaterialExpressionSaturate, -100, 100)
    sky_color = _expression(material, unreal.MaterialExpressionLinearInterpolate, 120, -40)

    _connect_expression(world_position, "", world_z, "")
    _connect_expression(world_z, "", scaled_z, "A")
    _connect_expression(frequency, "", scaled_z, "B")
    _connect_expression(scaled_z, "", biased_z, "A")
    _connect_expression(midpoint, "", biased_z, "B")
    _connect_expression(biased_z, "", gradient, "")
    _connect_expression(lower, "", sky_color, "A")
    _connect_expression(upper, "", sky_color, "B")
    _connect_expression(gradient, "", sky_color, "Alpha")
    _connect_property(sky_color, unreal.MaterialProperty.MP_EMISSIVE_COLOR)
    _finish_material(
        material,
        {
            "LingTu.ShowroomMaterial": "Sky",
            "LingTu.ShowroomDetail": "two-sided-unlit-world-z-gradient",
        },
    )
    return material


def _import_task(robot: dict[str, Any]) -> Any:
    task = unreal.AssetImportTask()
    # Explicit factory keeps material sections; UE 5.8's Interchange override
    # collapses the disabled material-import path to one section for these FBXs.
    task.set_editor_property("factory", unreal.FbxFactory())
    task.set_editor_property("filename", str(robot["source_path"]))
    task.set_editor_property("destination_path", ROBOT_DESTINATION)
    task.set_editor_property("destination_name", robot["asset_name"])
    task.set_editor_property("automated", True)
    task.set_editor_property("replace_existing", True)
    task.set_editor_property("replace_existing_settings", True)
    task.set_editor_property("save", False)
    task.set_editor_property("async_", False)

    options = unreal.FbxImportUI()
    options.set_editor_property("import_mesh", True)
    options.set_editor_property("import_as_skeletal", False)
    options.set_editor_property("import_materials", False)
    options.set_editor_property("import_textures", False)
    options.set_editor_property("create_physics_asset", False)
    options.set_editor_property("mesh_type_to_import", unreal.FBXImportType.FBXIT_STATIC_MESH)
    static_data = options.get_editor_property("static_mesh_import_data")
    static_data.set_editor_property("combine_meshes", True)
    static_data.set_editor_property("auto_generate_collision", False)
    static_data.set_editor_property("convert_scene", True)
    static_data.set_editor_property("convert_scene_unit", True)
    static_data.set_editor_property("import_uniform_scale", 1.0)
    static_data.set_editor_property("generate_lightmap_u_vs", False)
    static_data.set_editor_property("build_reversed_index_buffer", robot.get("build_reversed_index_buffer", True))
    static_data.set_editor_property("normal_generation_method", unreal.FBXNormalGenerationMethod.BUILT_IN)
    static_data.set_editor_property(
        "normal_import_method",
        unreal.FBXNormalImportMethod.FBXNIM_IMPORT_NORMALS,
    )
    task.set_editor_property("options", options)
    return task


def _package_asset_paths(directory: str) -> set[str]:
    paths: set[str] = set()
    for object_path in unreal.EditorAssetLibrary.list_assets(
        directory, recursive=False, include_folder=False
    ):
        package_path = str(object_path).split(".", 1)[0]
        asset = unreal.load_asset(package_path)
        if isinstance(asset, unreal.StaticMesh):
            paths.add(package_path)
    return paths


def _disable_and_validate_collision(mesh: Any) -> dict[str, Any]:
    subsystem = unreal.get_editor_subsystem(unreal.StaticMeshEditorSubsystem)
    # Commandlets do not create editor subsystems. The import already disables
    # collision generation; validate the empty BodySetup in either host.
    remove_result = bool(subsystem.remove_collisions(mesh)) if subsystem else False
    body_setup = mesh.get_editor_property("body_setup")
    if body_setup is None:
        raise RuntimeError(f"{mesh.get_path_name()}: imported StaticMesh has no BodySetup")
    body_setup.set_editor_property(
        "collision_trace_flag",
        unreal.CollisionTraceFlag.CTF_USE_SIMPLE_AS_COMPLEX,
    )
    aggregate = body_setup.get_editor_property("agg_geom")
    element_properties = (
        "sphere_elems",
        "box_elems",
        "sphyl_elems",
        "convex_elems",
        "tapered_capsule_elems",
        "level_set_elems",
        "skinned_level_set_elems",
        "ml_level_set_elems",
        "skinned_triangle_mesh_elems",
    )
    counts = {
        property_name: len(aggregate.get_editor_property(property_name))
        for property_name in element_properties
    }
    if sum(counts.values()) != 0:
        raise RuntimeError(
            f"{mesh.get_path_name()}: collision geometry remains after removal: {counts}"
        )
    return {
        "remove_api_returned": remove_result,
        "simple_shape_count": 0,
        "trace_flag": "CTF_USE_SIMPLE_AS_COMPLEX",
    }


def _validate_and_finish_mesh(robot: dict[str, Any]) -> dict[str, Any]:
    mesh = unreal.load_asset(robot["asset_path"])
    if mesh is None or not isinstance(mesh, unreal.StaticMesh):
        raise RuntimeError(
            f"{robot['id']}: expected StaticMesh at {robot['asset_path']}"
        )
    static_materials = list(mesh.get_editor_property("static_materials"))
    expected_slots = robot["material_slots"]
    if len(static_materials) != len(expected_slots):
        raise RuntimeError(
            f"{robot['id']}: imported slot count {len(static_materials)} "
            f"does not match manifest {len(expected_slots)}"
        )

    material_records: list[dict[str, Any]] = []
    for slot in expected_slots:
        material = _build_robot_material(robot["id"], slot)
        slot_index = int(slot["slot"])
        static_materials[slot_index].set_editor_property("material_interface", material)
        material_records.append(
            {
                "slot": slot_index,
                "asset": str(material.get_path_name()),
                "role": slot["role"],
                "rgba": slot["rgba"],
                "metallic": slot["metallic"],
                "roughness": slot["roughness"],
                "emissive": slot["emissive"],
            }
        )

    # Commit all assignments once; each set_material() otherwise schedules
    # another expensive build of the same large CAD mesh.
    mesh.set_editor_property("static_materials", static_materials)
    collision = _disable_and_validate_collision(mesh)
    unreal.EditorAssetLibrary.set_metadata_tag(mesh, "LingTu.Showroom", "Robot")
    unreal.EditorAssetLibrary.set_metadata_tag(mesh, "LingTu.ShowroomSource", robot["asset_filename"])
    unreal.EditorAssetLibrary.set_metadata_tag(mesh, "LingTu.Collision", "Disabled")
    if not unreal.EditorAssetLibrary.save_loaded_asset(mesh, only_if_is_dirty=False):
        raise RuntimeError(f"{robot['id']}: could not save imported StaticMesh")

    reloaded = unreal.load_asset(robot["asset_path"])
    if reloaded is None or not isinstance(reloaded, unreal.StaticMesh):
        raise RuntimeError(f"{robot['id']}: StaticMesh did not reload after save")
    saved_slots = list(reloaded.get_editor_property("static_materials"))
    if len(saved_slots) != len(material_records):
        raise RuntimeError(f"{robot['id']}: saved material slot count changed")
    for index, (saved_slot, material_record) in enumerate(
        zip(saved_slots, material_records)
    ):
        assigned = saved_slot.get_editor_property("material_interface")
        if assigned is None or str(assigned.get_path_name()) != material_record["asset"]:
            raise RuntimeError(f"{robot['id']}: material slot {index} was not saved")

    bounds = reloaded.get_bounding_box()
    actual_height_cm = float(bounds.max.z - bounds.min.z)
    expected_height_cm = float(robot["real_height_m"]) * 100.0
    tolerance_cm = max(
        HEIGHT_ABSOLUTE_TOLERANCE_CM,
        expected_height_cm * HEIGHT_RELATIVE_TOLERANCE,
    )
    if not math.isclose(
        actual_height_cm,
        expected_height_cm,
        rel_tol=0.0,
        abs_tol=tolerance_cm,
    ):
        raise RuntimeError(
            f"{robot['id']}: imported height {actual_height_cm:.4f} cm does not match "
            f"manifest {expected_height_cm:.4f} cm (tolerance {tolerance_cm:.4f} cm)"
        )
    return {
        "id": robot["id"],
        "source_fbx": robot["asset_filename"],
        "static_mesh": str(reloaded.get_path_name()),
        "type": "StaticMesh",
        "combined_mesh": True,
        "collision": collision,
        "normal_import_method": "FBXNIM_IMPORT_NORMALS",
        "material_count": len(material_records),
        "materials": material_records,
        "expected_height_cm": expected_height_cm,
        "actual_height_cm": actual_height_cm,
        "height_error_cm": actual_height_cm - expected_height_cm,
        "height_tolerance_cm": tolerance_cm,
    }


def _write_result(report: dict[str, Any]) -> None:
    RESULT_PATH.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def main() -> None:
    report: dict[str, Any] = {
        "schema": "lingtu.sim.robot-showroom-import-result.v1",
        "status": "failed",
        "source_manifest": str(MANIFEST_PATH),
        "robot_destination": ROBOT_DESTINATION,
        "material_destination": MATERIAL_DESTINATION,
        "settings": {
            "combine_meshes": True,
            "auto_generate_collision": False,
            "normal_import_method": "FBXNIM_IMPORT_NORMALS",
            "convert_scene": True,
            "convert_scene_unit": True,
            "import_uniform_scale": 1.0,
            "source_scene_unit": "meter",
            "unreal_unit": "centimeter",
        },
    }
    try:
        robots = _load_manifest()
        unreal.EditorAssetLibrary.make_directory(ROBOT_DESTINATION)
        unreal.EditorAssetLibrary.make_directory(MATERIAL_DESTINATION)
        surface = _build_surface_material()
        display_shell = _build_surface_material("M_DisplayShell", two_sided=True)
        sky = _build_sky_material()

        imported = []
        for robot in robots:
            task = _import_task(robot)
            _asset_tools().import_asset_tasks([task])
            if not task.get_editor_property("imported_object_paths"):
                raise RuntimeError(f"{robot['id']}: FBX import produced no objects")
            imported.append(_validate_and_finish_mesh(robot))
            report["robots"] = imported
            _write_result(report)
        expected_paths = {robot["asset_path"] for robot in robots}
        actual_paths = _package_asset_paths(ROBOT_DESTINATION)
        if actual_paths != expected_paths:
            raise RuntimeError(
                "strict showroom StaticMesh set mismatch: "
                f"missing={sorted(expected_paths - actual_paths)}, "
                f"unexpected={sorted(actual_paths - expected_paths)}"
            )
        report.update(
            {
                "status": "success",
                "expected_import_count": len(robots),
                "actual_import_count": len(imported),
                "surface_material": str(surface.get_path_name()),
                "display_shell_material": str(display_shell.get_path_name()),
                "sky_material": str(sky.get_path_name()),
                "robots": imported,
            }
        )
        _write_result(report)
        unreal.log(
            f"LINGTU_ROBOT_SHOWROOM_IMPORT_OK count={len(imported)} report={RESULT_PATH}"
        )
    except Exception as error:
        report["error_type"] = type(error).__name__
        report["error"] = str(error)
        _write_result(report)
        unreal.log_error(
            f"LINGTU_ROBOT_SHOWROOM_IMPORT_FAILED report={RESULT_PATH}: {error}"
        )
        raise


if __name__ == "__main__":
    main()

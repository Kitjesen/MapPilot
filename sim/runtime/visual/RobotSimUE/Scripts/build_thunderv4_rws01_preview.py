"""Import and capture the accepted RWS-01 payload on the ThunderV4 preview robot."""

from __future__ import annotations

import json
import os
import traceback
from pathlib import Path

try:
    import unreal
except ModuleNotFoundError:
    unreal = None


REPO_ROOT = Path(__file__).resolve().parents[5]
SOURCE_ASSET = Path(
    os.environ.get(
        "LINGTU_RWS01_ASSET",
        REPO_ROOT
        / "build"
        / "simstudio"
        / "artifacts"
        / "tripo"
        / "rws-01-v002"
        / "conditioned-v4"
        / "rws-01-v002-runtime.glb",
    )
).resolve()
SCREENSHOT_PATH = Path(
    os.environ.get(
        "LINGTU_RWS01_SCREENSHOT",
        REPO_ROOT / "build" / "unreal-preview" / "thunderv4-rws01.png",
    )
).resolve()
SUCCESS_SENTINEL = Path(
    os.environ.get(
        "LINGTU_RWS01_PREVIEW_SUCCESS",
        REPO_ROOT / "build" / "unreal-preview" / "thunderv4-rws01.success.json",
    )
).resolve()
ERROR_SENTINEL = Path(
    os.environ.get(
        "LINGTU_RWS01_PREVIEW_ERROR",
        REPO_ROOT / "build" / "unreal-preview" / "thunderv4-rws01.error.txt",
    )
).resolve()
UNATTENDED = os.environ.get("LINGTU_RWS01_PREVIEW_UNATTENDED") == "1"

SOURCE_MAP = "/Game/RobotSim/Maps/ThunderV4_RuntimePreview"
MESH_DESTINATION = "/Game/RobotSim/Payloads/FictionalRWS01/Runtime"
BASE_BODY_STABLE_ID = "thunder_01/base_link"

EXPECTED_RUNTIME_MESHES = (
    "SM_RWS01_EOSensor",
    "SM_RWS01_Launcher",
    "SM_RWS01_MountBase",
    "SM_RWS01_RecoilHousing",
    "SM_RWS01_YawFrame",
)
GLB_STATIC_MESH_PATH_FRAGMENT = "/rws-01-v002-runtime/StaticMeshes/"
PAYLOAD_VISUAL_POLICY = {
    "collision_enabled": "NoCollision",
    "simulate_physics": False,
    "generate_overlap_events": False,
    "authority": "MuJoCo",
}

_SCREENSHOT_TASK = None
_SCREENSHOT_TICK_HANDLE = None
_SCREENSHOT_TICK_CALLBACK = None


def payload_mount_transform_cm() -> dict[str, list[float]]:
    """Return the visual-only payload transform relative to Thunder's base frame."""

    return {
        "location_cm": [0.0, 0.0, 14.0],
        "rotation_deg": [0.0, 0.0, 0.0],
        "scale": [1.0, 1.0, 1.0],
    }


def acceptance_camera_post_process_profile() -> dict[str, float]:
    """Return deterministic exposure controls for the dark PBR payload."""

    return {
        "auto_exposure_min_brightness": 0.58,
        "auto_exposure_max_brightness": 0.58,
        "auto_exposure_bias": -2.0,
        "bloom_intensity": 0.05,
    }


def preview_map_strategy() -> dict[str, object]:
    """Describe the single-world preview strategy used to avoid editor World leaks."""

    return {"load": SOURCE_MAP, "persist": False}


def select_imported_static_mesh_paths(imported_object_paths: list[str]) -> dict[str, str]:
    """Select the five meshes produced by this GLB import, without stale fallback."""

    selected: dict[str, str] = {}
    for raw_path in imported_object_paths:
        object_path = str(raw_path).replace("\\", "/")
        if GLB_STATIC_MESH_PATH_FRAGMENT not in object_path:
            continue
        object_name = object_path.rsplit(".", 1)[-1]
        if object_name not in EXPECTED_RUNTIME_MESHES:
            continue
        if object_name in selected:
            raise RuntimeError(f"current GLB import returned duplicate mesh {object_name}")
        selected[object_name] = object_path

    missing = [name for name in EXPECTED_RUNTIME_MESHES if name not in selected]
    if missing:
        raise RuntimeError(
            "current GLB import did not return all accepted RWS-01 meshes: "
            f"missing={missing}, imported={list(imported_object_paths)}"
        )
    return {name: selected[name] for name in EXPECTED_RUNTIME_MESHES}


def _write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _quit_editor_if_unattended() -> None:
    if UNATTENDED:
        unreal.log("LINGTU_RWS01_PREVIEW_UNATTENDED_EXIT=1")
        unreal.SystemLibrary.quit_editor()


def _import_payload_meshes() -> dict[str, object]:
    if not SOURCE_ASSET.is_file():
        raise RuntimeError(f"accepted RWS-01 GLB is missing: {SOURCE_ASSET}")
    if SOURCE_ASSET.suffix.lower() != ".glb":
        raise RuntimeError(f"RWS-01 preview accepts only the conditioned GLB: {SOURCE_ASSET}")

    task = unreal.AssetImportTask()
    task.set_editor_property("filename", str(SOURCE_ASSET))
    task.set_editor_property("destination_path", MESH_DESTINATION)
    task.set_editor_property("automated", True)
    task.set_editor_property("replace_existing", True)
    task.set_editor_property("replace_existing_settings", True)
    task.set_editor_property("save", True)
    task.set_editor_property("async_", False)
    unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])

    imported_paths = select_imported_static_mesh_paths(
        [str(path) for path in task.imported_object_paths]
    )
    meshes: dict[str, object] = {}
    for name in EXPECTED_RUNTIME_MESHES:
        asset_path = imported_paths[name]
        mesh = unreal.load_asset(asset_path)
        if mesh is None or not isinstance(mesh, unreal.StaticMesh):
            raise RuntimeError(
                f"RWS-01 GLB path is not a StaticMesh: {asset_path}"
            )
        static_materials = list(mesh.get_editor_property("static_materials"))
        if not static_materials or any(
            slot.get_editor_property("material_interface") is None
            for slot in static_materials
        ):
            raise RuntimeError(f"RWS-01 GLB mesh has an unbound material: {asset_path}")
        meshes[name] = mesh
    return meshes


def _actor_tags(actor: object) -> set[str]:
    return {str(tag) for tag in actor.get_editor_property("tags")}


def _prepare_preview_map(level_subsystem: object) -> None:
    if not unreal.EditorAssetLibrary.does_asset_exist(SOURCE_MAP):
        raise RuntimeError(f"ThunderV4 source preview map is missing: {SOURCE_MAP}")
    if not level_subsystem.load_level(SOURCE_MAP):
        raise RuntimeError(f"could not load ThunderV4 preview map {SOURCE_MAP}")


def _find_base_actor(actor_subsystem: object) -> object:
    matches = [
        actor
        for actor in actor_subsystem.get_all_level_actors()
        if BASE_BODY_STABLE_ID in _actor_tags(actor)
    ]
    if len(matches) != 1:
        raise RuntimeError(
            f"expected exactly one {BASE_BODY_STABLE_ID} actor in {SOURCE_MAP}, found {len(matches)}"
        )
    return matches[0]


def _remove_previous_payload_preview(actor_subsystem: object) -> None:
    for actor in tuple(actor_subsystem.get_all_level_actors()):
        if "LingTuPayloadPreview" in _actor_tags(actor):
            if not actor_subsystem.destroy_actor(actor):
                raise RuntimeError(f"could not remove previous payload actor {actor.get_actor_label()}")


def _unreal_mount_transform() -> object:
    mount = payload_mount_transform_cm()
    return unreal.Transform(
        location=unreal.Vector(*mount["location_cm"]),
        rotation=unreal.Rotator(*mount["rotation_deg"]),
        scale=unreal.Vector(*mount["scale"]),
    )


def _spawn_payload_actor(
    actor_subsystem: object,
    base_actor: object,
    name: str,
    mesh: object,
) -> object:
    actor = actor_subsystem.spawn_actor_from_class(
        unreal.StaticMeshActor,
        unreal.Vector(0.0, 0.0, 0.0),
        unreal.Rotator(0.0, 0.0, 0.0),
        False,
    )
    if actor is None:
        raise RuntimeError(f"failed to spawn RWS-01 mesh actor {name}")
    actor.set_actor_label(f"RWS01_{name}")
    actor.set_editor_property(
        "tags",
        [
            unreal.Name("LingTuPayloadPreview"),
            unreal.Name("VisualOnly"),
            unreal.Name("PayloadPackage:fictional_rws_01@1.0.0"),
            unreal.Name(f"PayloadMesh:{name}"),
        ],
    )
    component = actor.get_editor_property("static_mesh_component")
    component.set_mobility(unreal.ComponentMobility.MOVABLE)
    component.set_static_mesh(mesh)
    component.set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)
    component.set_simulate_physics(False)
    component.set_editor_property("generate_overlap_events", False)
    component.set_editor_property("cast_shadow", True)
    actor.attach_to_actor(
        base_actor,
        "",
        unreal.AttachmentRule.KEEP_RELATIVE,
        unreal.AttachmentRule.KEEP_RELATIVE,
        unreal.AttachmentRule.KEEP_RELATIVE,
        False,
    )
    actor.set_actor_relative_transform(_unreal_mount_transform(), False, True)
    if actor.get_attach_parent_actor() != base_actor:
        raise RuntimeError(f"RWS-01 mesh {name} did not attach to {BASE_BODY_STABLE_ID}")
    return actor


def _spawn_camera(actor_subsystem: object) -> object:
    location = unreal.Vector(245.0, -315.0, 160.0)
    target = unreal.Vector(0.0, 0.0, 66.0)
    rotation = unreal.MathLibrary.find_look_at_rotation(location, target)
    camera = actor_subsystem.spawn_actor_from_class(
        unreal.CameraActor,
        location,
        rotation,
        False,
    )
    if camera is None:
        raise RuntimeError("failed to spawn RWS-01 acceptance camera")
    camera.set_actor_label("Thunder_RWS01_AcceptanceCamera")
    camera.set_editor_property(
        "tags",
        [unreal.Name("LingTuPayloadPreview"), unreal.Name("LingTuPayloadPreviewCamera")],
    )
    camera.camera_component.set_editor_property("field_of_view", 43.0)
    camera.camera_component.set_editor_property("aspect_ratio", 16.0 / 9.0)
    camera.camera_component.set_editor_property("post_process_blend_weight", 1.0)
    settings = camera.camera_component.get_editor_property("post_process_settings")
    profile = acceptance_camera_post_process_profile()
    settings.set_editor_property("override_auto_exposure_min_brightness", True)
    settings.set_editor_property("override_auto_exposure_max_brightness", True)
    settings.set_editor_property("override_auto_exposure_bias", True)
    settings.set_editor_property("override_bloom_intensity", True)
    for property_name, value in profile.items():
        settings.set_editor_property(property_name, value)
    camera.camera_component.set_editor_property("post_process_settings", settings)
    unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).set_level_viewport_camera_info(
        location,
        rotation,
    )
    return camera


def _write_success(imported_assets: list[str]) -> None:
    SUCCESS_SENTINEL.parent.mkdir(parents=True, exist_ok=True)
    SUCCESS_SENTINEL.write_text(
        json.dumps(
            {
                "schema": "lingtu.sim.payload-offline-preview-success.v1",
                "mode": "offline_preview",
                "payload": "fictional_rws_01@1.0.0",
                "source_asset": str(SOURCE_ASSET),
                "map": SOURCE_MAP,
                "map_persisted": False,
                "screenshot": str(SCREENSHOT_PATH),
                "mount": payload_mount_transform_cm(),
                "visual_policy": PAYLOAD_VISUAL_POLICY,
                "meshes": imported_assets,
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )


def _request_screenshot(camera: object, imported_assets: list[str]) -> None:
    global _SCREENSHOT_TASK, _SCREENSHOT_TICK_CALLBACK, _SCREENSHOT_TICK_HANDLE

    SCREENSHOT_PATH.parent.mkdir(parents=True, exist_ok=True)
    unreal.AutomationLibrary.finish_loading_before_screenshot()
    task = unreal.AutomationLibrary.take_high_res_screenshot(
        1920,
        1080,
        str(SCREENSHOT_PATH),
        camera=camera,
        mask_enabled=False,
        capture_hdr=False,
        delay=3.0,
    )
    if not task.is_valid_task():
        raise RuntimeError("Unreal rejected the RWS-01 high-resolution screenshot task")

    def wait_for_screenshot(_delta_seconds: float) -> None:
        global _SCREENSHOT_TASK, _SCREENSHOT_TICK_CALLBACK, _SCREENSHOT_TICK_HANDLE
        if not task.is_task_done():
            return
        if _SCREENSHOT_TICK_HANDLE is not None:
            unreal.unregister_slate_post_tick_callback(_SCREENSHOT_TICK_HANDLE)
            _SCREENSHOT_TICK_HANDLE = None
        _SCREENSHOT_TASK = None
        _SCREENSHOT_TICK_CALLBACK = None
        if SCREENSHOT_PATH.is_file():
            _write_success(imported_assets)
            unreal.log(f"LINGTU_RWS01_PREVIEW_READY={SCREENSHOT_PATH}")
        else:
            error = f"Unreal screenshot task completed without creating {SCREENSHOT_PATH}"
            _write_text(ERROR_SENTINEL, error + "\n")
            unreal.log_error(f"LINGTU_RWS01_PREVIEW_ERROR={ERROR_SENTINEL}")
        _quit_editor_if_unattended()

    _SCREENSHOT_TASK = task
    _SCREENSHOT_TICK_CALLBACK = wait_for_screenshot
    _SCREENSHOT_TICK_HANDLE = unreal.register_slate_post_tick_callback(wait_for_screenshot)


def build_preview() -> None:
    """Materialize the visual-only payload and schedule its acceptance screenshot."""

    level_subsystem = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    actor_subsystem = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    _prepare_preview_map(level_subsystem)
    meshes = _import_payload_meshes()
    _remove_previous_payload_preview(actor_subsystem)
    base_actor = _find_base_actor(actor_subsystem)
    for name in EXPECTED_RUNTIME_MESHES:
        _spawn_payload_actor(actor_subsystem, base_actor, name, meshes[name])
    camera = _spawn_camera(actor_subsystem)
    _request_screenshot(camera, list(EXPECTED_RUNTIME_MESHES))
    unreal.log(
        "LingTu RWS-01 offline preview staged: "
        f"payload={len(meshes)} meshes, parent={BASE_BODY_STABLE_ID}, map={SOURCE_MAP}, persisted=false"
    )


def main() -> None:
    """Run the Unreal Editor preview with fail-closed evidence output."""

    try:
        build_preview()
    except Exception as error:
        _write_text(
            ERROR_SENTINEL,
            f"{type(error).__name__}: {error}\n{traceback.format_exc()}",
        )
        unreal.log_error(f"LINGTU_RWS01_PREVIEW_ERROR={ERROR_SENTINEL}")
        _quit_editor_if_unattended()
        raise


if unreal is None:
    if __name__ == "__main__":
        raise RuntimeError("build_thunderv4_rws01_preview.py must run inside Unreal Editor Python")
else:
    main()

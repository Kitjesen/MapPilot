"""Build and verify a small XGRIDS LCC2 map in the RobotSimUE editor.

Run through UnrealEditor -ExecCmds="py <absolute path to this script>".
The XGRIDS SDK and official sample are locally mounted by stage_xgrids_poc.ps1.
"""

import json
import pathlib
import time
import traceback

import unreal

MAP_PATH = "/Game/RobotSim/Maps/GSPOC_XGrids_LCC2"
SCENE_PATH = "3DGSData/lcc2/LCC2.lcc2"
PROJECT_ROOT = pathlib.Path(unreal.Paths.project_dir())
OUTPUT = PROJECT_ROOT / "Saved" / "GSPOC"
SCREENSHOT = OUTPUT / "xgrids_lcc2.png"
EVIDENCE = OUTPUT / "evidence.json"
ERROR = OUTPUT / "error.txt"
PROXY_FBX = OUTPUT / "GSPOC_LCC2_CollisionProxy.fbx"
PROXY_ASSET = "/Game/RobotSim/GSPOC/Meshes/GSPOC_LCC2_CollisionProxy"
OUTPUT.mkdir(parents=True, exist_ok=True)
for stale in (SCREENSHOT, EVIDENCE, ERROR):
    stale.unlink(missing_ok=True)

_component = None
_camera = None
_markers = []
_proxy = None
_proxy_bounds = None
_screenshot_task = None
_screenshot_started = False
_screenshot_done = False
_loaded_at = None
_started = time.monotonic()
_tick_handle = None
_tick_callback = None
_last_trace_at = 0.0


def _stop(message: str, success: bool) -> None:
    global _tick_handle, _tick_callback, _screenshot_task
    if _tick_handle is not None:
        unreal.unregister_slate_post_tick_callback(_tick_handle)
        _tick_handle = None
    _tick_callback = None
    _screenshot_task = None
    if success:
        unreal.log(message)
    else:
        ERROR.write_text(message + "\n", encoding="utf-8")
        unreal.log_error(message)
    unreal.SystemLibrary.quit_editor()


def _trace_collision() -> tuple[bool, str]:
    world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
    # The marker meshes are visual depth references, not collision evidence.
    for x in (-1600.0, -800.0, 0.0, 800.0, 1600.0):
        for y in (-1600.0, -800.0, 0.0, 800.0, 1600.0):
            result = unreal.SystemLibrary.line_trace_single(
                world,
                unreal.Vector(x, y, 5000.0),
                unreal.Vector(x, y, -5000.0),
                unreal.TraceTypeQuery.TRACE_TYPE_QUERY1,
                True,
                _markers,
                unreal.DrawDebugTrace.NONE,
                True,
                unreal.LinearColor(1.0, 0.0, 0.0, 1.0),
                unreal.LinearColor(0.0, 1.0, 0.0, 1.0),
                0.0,
            )
            if result is None or result is False:
                continue
            if isinstance(result, tuple):
                if result and bool(result[0]):
                    return True, str(result)
                continue
            return True, str(result)
    return False, "No line trace hit in 25 sampled columns"


def _tick(_delta_seconds: float) -> None:
    global _screenshot_task, _screenshot_started, _screenshot_done, _loaded_at, _last_trace_at
    elapsed = time.monotonic() - _started
    try:
        loaded = bool(_component.check_if_loaded())
        splats = bool(_component.have_valid_splat_data())
        collision = bool(_component.have_valid_collision_data())
        if loaded and splats and _loaded_at is None:
            _loaded_at = time.monotonic()
            unreal.log("LINGTU_GSPOC_SPLATS_READY")
        if _loaded_at is not None and not _screenshot_started and time.monotonic() - _loaded_at >= 8.0:
            _screenshot_started = True
            unreal.AutomationLibrary.finish_loading_before_screenshot()
            _screenshot_task = unreal.AutomationLibrary.take_high_res_screenshot(
                1280, 720, str(SCREENSHOT), camera=_camera,
                mask_enabled=False, capture_hdr=False, delay=3.0,
            )
            if not _screenshot_task.is_valid_task():
                raise RuntimeError("Unreal rejected the GS screenshot task")
            unreal.log("LINGTU_GSPOC_SCREENSHOT_REQUESTED")
        if _screenshot_task is not None and _screenshot_task.is_task_done():
            _screenshot_done = SCREENSHOT.is_file() and SCREENSHOT.stat().st_size > 0
            _screenshot_task = None
            if not _screenshot_done:
                raise RuntimeError("Screenshot task completed without an image")
        if _screenshot_done and time.monotonic() - _last_trace_at >= 2.0:
            _last_trace_at = time.monotonic()
            trace_hit, trace_result = _trace_collision()
            if trace_hit or elapsed > 180.0:
                payload = {
                    "engine": unreal.SystemLibrary.get_engine_version(),
                    "map": MAP_PATH,
                    "scene": SCENE_PATH,
                    "loaded": loaded,
                    "valid_splat_data": splats,
                    "lcc2_internal_collision_data": collision,
                    "proxy_mesh_asset": PROXY_ASSET,
                    "proxy_bounds_cm": _proxy_bounds,
                    "line_trace_hit": trace_hit,
                    "line_trace_result": trace_result,
                    "screenshot": str(SCREENSHOT),
                    "note": "UE visual/collision proof only; MuJoCo remains Thunder's physics authority.",
                }
                EVIDENCE.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
                if not trace_hit:
                    raise RuntimeError(f"The explicit UE proxy did not answer line traces: {trace_result}")
                _stop(f"LINGTU_GSPOC_VERIFIED {EVIDENCE}", True)
                return
        if elapsed > 600.0:
            raise TimeoutError(
                f"GS verification timed out: loaded={loaded} splats={splats} "
                f"collision={collision} screenshot={_screenshot_done}"
            )
    except Exception:
        _stop(traceback.format_exc(), False)


def _spawn() -> None:
    global _component, _camera, _proxy, _proxy_bounds, _tick_handle, _tick_callback
    source = PROJECT_ROOT / "Content" / pathlib.Path(SCENE_PATH)
    if not source.is_file():
        raise FileNotFoundError(f"LCC2 sample not staged: {source}")
    if not hasattr(unreal, "LCC2Actor"):
        raise RuntimeError("LCC4Unreal is not loaded; stage the UE 5.8 plugin first")
    if not PROXY_FBX.is_file():
        raise FileNotFoundError(f"Run export_lcc2_collision.py in Blender first: {PROXY_FBX}")

    level = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    if unreal.EditorAssetLibrary.does_asset_exist(MAP_PATH):
        if not level.load_level(MAP_PATH):
            raise RuntimeError(f"Could not load {MAP_PATH}")
        for actor in actors.get_all_level_actors():
            if actor.get_actor_label().startswith("GSPOC_"):
                if isinstance(actor, unreal.LCC2Actor):
                    actor.get_lcc_component().set_lcc_collision_enable(False)
                actors.destroy_actor(actor)
    elif not level.new_level(MAP_PATH, False):
        raise RuntimeError(f"Could not create {MAP_PATH}")

    task = unreal.AssetImportTask()
    task.set_editor_property("filename", str(PROXY_FBX))
    task.set_editor_property("destination_path", "/Game/RobotSim/GSPOC/Meshes")
    task.set_editor_property("automated", True)
    task.set_editor_property("replace_existing", True)
    task.set_editor_property("save", True)
    task.set_editor_property("async_", False)
    options = unreal.FbxImportUI()
    options.set_editor_property("import_mesh", True)
    options.set_editor_property("import_as_skeletal", False)
    options.set_editor_property("import_materials", False)
    options.set_editor_property("import_textures", False)
    options.set_editor_property("mesh_type_to_import", unreal.FBXImportType.FBXIT_STATIC_MESH)
    static_options = options.get_editor_property("static_mesh_import_data")
    static_options.set_editor_property("combine_meshes", True)
    static_options.set_editor_property("auto_generate_collision", False)
    static_options.set_editor_property("convert_scene", True)
    static_options.set_editor_property("convert_scene_unit", False)
    # FBX's meter-to-centimeter conversion already applies the required x100.
    static_options.set_editor_property("import_uniform_scale", 1.0)
    task.set_editor_property("options", options)
    unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
    proxy_mesh = unreal.load_asset(PROXY_ASSET)
    if proxy_mesh is None or not isinstance(proxy_mesh, unreal.StaticMesh):
        raise RuntimeError(f"FBX import did not produce {PROXY_ASSET}")
    bounds = proxy_mesh.get_bounding_box()
    _proxy_bounds = {
        "min": [bounds.min.x, bounds.min.y, bounds.min.z],
        "max": [bounds.max.x, bounds.max.y, bounds.max.z],
    }
    extents = [bounds.max.x - bounds.min.x, bounds.max.y - bounds.min.y, bounds.max.z - bounds.min.z]
    if not (6000.0 < extents[0] < 9000.0 and 3000.0 < extents[1] < 5000.0 and 1200.0 < extents[2] < 2000.0):
        raise RuntimeError(f"Proxy scale/axes do not match LCC2 metadata in centimeters: {_proxy_bounds}")
    body_setup = proxy_mesh.get_editor_property("body_setup")
    body_setup.set_editor_property(
        "collision_trace_flag", unreal.CollisionTraceFlag.CTF_USE_COMPLEX_AS_SIMPLE
    )
    unreal.EditorAssetLibrary.save_loaded_asset(proxy_mesh, only_if_is_dirty=False)
    unreal.log(f"LINGTU_GSPOC_PROXY_IMPORTED bounds={_proxy_bounds}")

    gs = actors.spawn_actor_from_class(unreal.LCC2Actor, unreal.Vector(0.0, 0.0, 0.0), unreal.Rotator(), False)
    gs.set_actor_label("GSPOC_LCC2_Visual_And_ProxyCollision")
    _component = gs.get_lcc_component()
    _component.set_editor_property("default_load_path", SCENE_PATH)
    _component.set_lcc_collision_enable(False)
    if not _component.load(SCENE_PATH):
        raise RuntimeError(f"XGRIDS rejected {SCENE_PATH}")

    _proxy = actors.spawn_actor_from_class(
        unreal.StaticMeshActor, unreal.Vector(0.0, 0.0, 0.0), unreal.Rotator(), False
    )
    _proxy.set_actor_label("GSPOC_LCC2_SameSource_CollisionProxy")
    _proxy.static_mesh_component.set_editor_property("static_mesh", proxy_mesh)
    _proxy.static_mesh_component.set_collision_profile_name("BlockAll")
    _proxy.static_mesh_component.set_collision_enabled(unreal.CollisionEnabled.QUERY_AND_PHYSICS)
    _proxy.static_mesh_component.set_visibility(False, True)

    marker_mesh = unreal.EditorAssetLibrary.load_asset("/Engine/BasicShapes/Cube.Cube")
    if marker_mesh is None:
        raise RuntimeError("The built-in UE cube mesh is unavailable")
    for name, position in (
        ("GSPOC_Foreground_DepthMarker", unreal.Vector(650.0, -900.0, 160.0)),
        ("GSPOC_Background_DepthMarker", unreal.Vector(650.0, 900.0, 160.0)),
    ):
        marker = actors.spawn_actor_from_class(unreal.StaticMeshActor, position, unreal.Rotator(), False)
        marker.set_actor_label(name)
        marker.static_mesh_component.set_editor_property("static_mesh", marker_mesh)
        marker.set_actor_scale3d(unreal.Vector(1.3, 0.5, 3.2))
        _markers.append(marker)

    sun = actors.spawn_actor_from_class(
        unreal.DirectionalLight, unreal.Vector(0.0, 0.0, 2000.0), unreal.Rotator(-42.0, -28.0, 0.0), False
    )
    sun.set_actor_label("GSPOC_DirectionalLight")
    sun.light_component.set_editor_property("intensity", 6.2)
    sky = actors.spawn_actor_from_class(
        unreal.SkyAtmosphere, unreal.Vector(0.0, 0.0, 0.0), unreal.Rotator(), False
    )
    sky.set_actor_label("GSPOC_SkyAtmosphere")
    skylight = actors.spawn_actor_from_class(
        unreal.SkyLight, unreal.Vector(0.0, 0.0, 1200.0), unreal.Rotator(), False
    )
    skylight.set_actor_label("GSPOC_SkyLight")
    skylight.light_component.set_editor_property("intensity", 1.0)
    skylight.light_component.set_editor_property("real_time_capture", True)

    target = unreal.Vector(0.0, 0.0, 0.0)
    location = unreal.Vector(0.0, -3900.0, 900.0)
    rotation = unreal.MathLibrary.find_look_at_rotation(location, target)
    _camera = actors.spawn_actor_from_class(unreal.CameraActor, location, rotation, False)
    _camera.set_actor_label("GSPOC_VerificationCamera")
    _camera.camera_component.set_editor_property("field_of_view", 74.0)
    unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).set_level_viewport_camera_info(location, rotation)
    if not level.save_current_level():
        raise RuntimeError(f"Could not save {MAP_PATH}")
    unreal.log(f"LINGTU_GSPOC_MAP_SAVED {MAP_PATH}")
    _tick_callback = _tick
    _tick_handle = unreal.register_slate_post_tick_callback(_tick_callback)


try:
    _spawn()
except Exception:
    _stop(traceback.format_exc(), False)

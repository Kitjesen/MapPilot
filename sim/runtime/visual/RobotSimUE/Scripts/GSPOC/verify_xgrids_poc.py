"""Reload the generated GS map and verify its saved splats and proxy collision."""

import json
import pathlib
import time
import traceback

import unreal

MAP_PATH = "/Game/RobotSim/Maps/GSPOC_XGrids_LCC2"
OUTPUT = pathlib.Path(unreal.Paths.project_dir()) / "Saved" / "GSPOC"
EVIDENCE = OUTPUT / "persisted_evidence.json"
ERROR = OUTPUT / "persisted_error.txt"
LABEL_GS = "GSPOC_LCC2_Visual_And_ProxyCollision"
LABEL_PROXY = "GSPOC_LCC2_SameSource_CollisionProxy"
_started = time.monotonic()
_tick_handle = None
_tick_callback = None


def _done(message: str, success: bool) -> None:
    global _tick_handle, _tick_callback
    if _tick_handle is not None:
        unreal.unregister_slate_post_tick_callback(_tick_handle)
        _tick_handle = None
    _tick_callback = None
    if success:
        unreal.log(message)
    else:
        ERROR.write_text(message + "\n", encoding="utf-8")
        unreal.log_error(message)
    unreal.SystemLibrary.quit_editor()


def _find_proxy_hit(world: object, marker_actors: list[object], proxy_actor: object) -> dict[str, object] | None:
    for x in (-1600.0, -800.0, 0.0, 800.0, 1600.0):
        for y in (-1600.0, -800.0, 0.0, 800.0, 1600.0):
            hit = unreal.SystemLibrary.line_trace_single(
                world,
                unreal.Vector(x, y, 5000.0),
                unreal.Vector(x, y, -5000.0),
                unreal.TraceTypeQuery.TRACE_TYPE_QUERY1,
                True,
                marker_actors,
                unreal.DrawDebugTrace.NONE,
                True,
            )
            if hit is None:
                continue
            without_proxy = unreal.SystemLibrary.line_trace_single(
                world,
                unreal.Vector(x, y, 5000.0),
                unreal.Vector(x, y, -5000.0),
                unreal.TraceTypeQuery.TRACE_TYPE_QUERY1,
                True,
                [*marker_actors, proxy_actor],
                unreal.DrawDebugTrace.NONE,
                True,
            )
            if without_proxy is None:
                return {
                    "actor": LABEL_PROXY,
                    "trace_xy_cm": [x, y],
                    "blocking_hit_with_proxy": True,
                    "blocking_hit_ignoring_proxy": False,
                }
    return None


try:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    EVIDENCE.unlink(missing_ok=True)
    ERROR.unlink(missing_ok=True)
    level = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    if not level.load_level(MAP_PATH):
        raise RuntimeError(f"Could not reload {MAP_PATH}")
    actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem).get_all_level_actors()
    by_label = {actor.get_actor_label(): actor for actor in actors}
    gs = by_label[LABEL_GS]
    proxy = by_label[LABEL_PROXY]
    markers = [actor for label, actor in by_label.items() if label.endswith("_DepthMarker")]
    component = gs.get_lcc_component()
    mesh_component = proxy.static_mesh_component
    if str(mesh_component.get_collision_profile_name()) != "BlockAll":
        raise RuntimeError("Saved proxy did not retain BlockAll collision")
    body_setup = mesh_component.get_editor_property("static_mesh").get_editor_property("body_setup")
    complexity = str(body_setup.get_editor_property("collision_trace_flag"))
    if "COMPLEX" not in complexity.upper():
        raise RuntimeError(f"Saved proxy did not retain complex-as-simple collision: {complexity}")
    world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()

    def tick(_delta_seconds: float) -> None:
        try:
            loaded = bool(component.check_if_loaded())
            splats = bool(component.have_valid_splat_data())
            if loaded and splats:
                hit = _find_proxy_hit(world, markers, proxy)
                if hit is not None:
                    payload = {
                        "map": MAP_PATH,
                        "loaded": loaded,
                        "valid_splat_data": splats,
                        "proxy_collision_profile": "BlockAll",
                        "proxy_collision_complexity": complexity,
                        "proxy_hit": hit,
                    }
                    EVIDENCE.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
                    _done(f"LINGTU_GSPOC_PERSISTED_VERIFIED {EVIDENCE}", True)
                    return
            if time.monotonic() - _started > 120.0:
                raise TimeoutError(f"Saved GS map failed validation: loaded={loaded} splats={splats}")
        except Exception:
            _done(traceback.format_exc(), False)

    _started = time.monotonic()
    _tick_callback = tick
    _tick_handle = unreal.register_slate_post_tick_callback(_tick_callback)
except Exception:
    _done(traceback.format_exc(), False)

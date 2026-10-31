"""Capture the licensed factory's selected corridor without editor overlays.

Run inside the isolated FactoryPreview editor. This does not save or migrate
the source map, execute gameplay, or create a simulation collision world.
"""

import json
import os
import time
from pathlib import Path

import unreal

output = Path(os.environ["LINGTU_FACTORY_CAPTURE_DIR"])
output.mkdir(parents=True, exist_ok=True)
started = time.monotonic()
requested = False
task = None
handle = None
finished = False
location = unreal.Vector(6502.645508, 108.841347, 253.603363)
rotation = unreal.Rotator(pitch=-1.000122, yaw=-196.991776, roll=0)
editor = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
editor.set_level_viewport_camera_info(location, rotation)
world = editor.get_editor_world()
for command in (
    "r.SceneColorFringeQuality 0", "r.MotionBlurQuality 0",
    "r.BloomQuality 3", "r.ScreenPercentage 100", "t.MaxFPS 30",
):
    unreal.SystemLibrary.execute_console_command(world, command)


def finish(error=None):
    global finished
    if finished:
        return
    finished = True
    (output / "capture-result.json").write_text(json.dumps({
        "source_project": unreal.Paths.project_dir(),
        "map": world.get_path_name(),
        "camera_position_cm": [location.x, location.y, location.z],
        "camera_rotation_degrees": [rotation.pitch, rotation.yaw, rotation.roll],
        "image": str(output / "factory-corridor.png"),
        "purpose": "static_frontend_background_not_drivable_map",
        "source_map_saved": False,
        "elapsed_seconds": time.monotonic() - started,
        "error": error,
    }, indent=2), encoding="utf-8")
    unreal.unregister_slate_post_tick_callback(handle)
    unreal.SystemLibrary.quit_editor()


def tick(_delta):
    global requested, task
    if finished:
        return
    elapsed = time.monotonic() - started
    if elapsed > 300:
        finish("factory screenshot timed out")
    elif not requested and elapsed > 18:
        # The screenshot API can pump Slate while waiting for textures.
        requested = True
        task = unreal.AutomationLibrary.take_high_res_screenshot(
            1920, 1080, str(output / "factory-corridor.png"),
            mask_enabled=False, capture_hdr=False, delay=1.0,
        )
    elif requested and task is not None and task.is_task_done() and (output / "factory-corridor.png").is_file():
        finish()


handle = unreal.register_slate_post_tick_callback(tick)
unreal.log("LINGTU_FACTORY_FRONTEND_CAPTURE_STARTED no_gameplay=true")

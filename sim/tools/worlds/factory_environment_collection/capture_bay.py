"""Read-only UE bay render review of a specified authored plan and its saved lights.

Execute in RobotSimUE editor via -ExecCmds=py. No level/assets are saved, no
physics or Product is started, and screenshots do not grant world qualification.
"""

import json
import os
import time
from pathlib import Path

import unreal as ue

plan = json.loads(Path(os.environ["LINGTU_FACTORY_BAY_VISUAL_PLAN"]).read_text(encoding="utf-8"))
LEVEL = plan["level_path"]
output = Path(os.environ["LINGTU_FACTORY_BAY_CAPTURE_DIR"])
output.mkdir(parents=True, exist_ok=True)
started = time.monotonic()
handle = None
finished = False
task = None
index = 0
ready_at = 0.0
views = [
    ("bay-overview", [6460, 30, 290], [5780, 260, 120]),
    ("control22-view", [5900, 300, 150], [6007.72, 379.13, 130]),
    ("engine-view", [5230, 350, 155], [5233.27, 513.10, 150]),
]
report = {
    "scope": "static_render_review",
    "map": LEVEL,
    "level_saved": False,
    "product_run": False,
    "spawn_approved": False,
    "qualified_world": False,
    "temporary_lighting": False,
    "views": [],
}


def finish(error=None):
    global finished
    if finished:
        return
    finished = True
    report.update(
        error=error,
        elapsed_s=time.monotonic() - started,
        capture_complete=error is None and len(report["views"]) == len(views),
    )
    (output / "capture-result.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    if handle is not None:
        ue.unregister_slate_post_tick_callback(handle)
    ue.SystemLibrary.quit_editor()


def tick(_delta):
    global index, task, ready_at
    if finished:
        return
    try:
        now = time.monotonic()
        if now - started > 300:
            raise TimeoutError("bounded bay screenshot capture exceeded 300 seconds")
        if task is not None:
            image = output / (views[index][0] + ".png")
            if not task.is_task_done() or not image.is_file():
                return
            name, position, target = views[index]
            report["views"].append({"name": name, "image": str(image), "position_cm": position, "look_at_cm": target})
            index += 1
            task = None
            if index == len(views):
                finish()
                return
            set_view(index)
            ready_at = now + 10
        elif now >= ready_at:
            # Mark in-flight before the screenshot API pumps Slate recursively.
            ready_at = float("inf")
            task = ue.AutomationLibrary.take_high_res_screenshot(
                1600,
                900,
                str(output / (views[index][0] + ".png")),
                mask_enabled=False,
                capture_hdr=False,
                delay=1.0,
            )
    except Exception as error:
        finish(f"{type(error).__name__}: {error}")


def set_view(view_index):
    _, position, target = views[view_index]
    location = ue.Vector(*position)
    editor.set_level_viewport_camera_info(location, ue.MathLibrary.find_look_at_rotation(location, ue.Vector(*target)))


try:
    levels = ue.get_editor_subsystem(ue.LevelEditorSubsystem)
    if not levels.load_level(LEVEL):
        raise RuntimeError("authored factory bay could not be loaded")
    editor = ue.get_editor_subsystem(ue.UnrealEditorSubsystem)
    actors = ue.get_editor_subsystem(ue.EditorActorSubsystem)
    components = []
    for actor in actors.get_all_level_actors():
        ids = [str(tag).split("=", 1)[1] for tag in actor.tags if str(tag).startswith("factory_source_component=")]
        if ids:
            component = actor.static_mesh_component
            if component.get_collision_enabled() != ue.CollisionEnabled.NO_COLLISION:
                raise RuntimeError("shared visual component has UE collision enabled")
            components.append(
                {
                    "source_id": ids[0],
                    "mesh": component.static_mesh.get_path_name(),
                    "materials": [
                        component.get_material(i).get_path_name() if component.get_material(i) else None
                        for i in range(component.get_num_materials())
                    ],
                }
            )
    if {c["source_id"] for c in components} != {c["source_id"] for c in plan["components"]}:
        raise RuntimeError("retained source component coverage differs from visual plan")
    report["components"] = components
    report["source_light_ids"] = [str(tag).split("=", 1)[1] for actor in actors.get_all_level_actors()
                                  for tag in actor.tags if str(tag).startswith("factory_source_light=")]
    if not report["source_light_ids"]:
        raise RuntimeError("authored source lighting is missing; no temporary-light fallback")
    world = editor.get_editor_world()
    for command in ("r.SceneColorFringeQuality 0", "r.MotionBlurQuality 0", "r.ScreenPercentage 100", "t.MaxFPS 30"):
        ue.SystemLibrary.execute_console_command(world, command)
    set_view(0)
    ready_at = time.monotonic() + 30
    handle = ue.register_slate_post_tick_callback(tick)
    ue.log("LINGTU_FACTORY_BAY_RENDER_STARTED authored_lighting=true no_product=true")
except Exception as error:
    finish(f"{type(error).__name__}: {error}")

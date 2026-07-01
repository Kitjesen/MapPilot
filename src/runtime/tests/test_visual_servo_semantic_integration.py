import numpy as np

from runtime.msgs.geometry import Vector3
from runtime.msgs.semantic import Detection3D, SceneGraph
from runtime.msgs.sensor import Image, ImageFormat
from decision.modules.visual_servo_module import VisualServoModule


def test_find_target_bbox_reads_bbox_2d():
    mod = VisualServoModule()
    mod._target_label = "chair"
    mod._latest_sg = SceneGraph(
        objects=[
            Detection3D(
                label="chair",
                confidence=0.9,
                bbox_2d=[10.0, 20.0, 30.0, 40.0],
            )
        ]
    )

    assert mod._find_target_bbox() == [10.0, 20.0, 30.0, 40.0]


def test_follow_mode_runs_without_depth_or_intrinsics():
    mod = VisualServoModule()
    mod._mode = "follow"

    called = []
    mod._tick_follow = lambda: called.append(True)

    mod._on_color(Image(data=np.zeros((8, 8, 3), dtype=np.uint8), format=ImageFormat.RGB))

    assert called == [True]


def test_release_servo_publishes_nav_resume():
    mod = VisualServoModule()
    mod._servo_active = True

    nav_stop = []
    cmd_vel = []
    mod.nav_stop._add_callback(nav_stop.append)
    mod.cmd_vel._add_callback(cmd_vel.append)

    mod._release_servo()

    assert nav_stop == [0]
    assert len(cmd_vel) == 1


def test_follow_path_uses_scene_graph_bbox_2d():
    mod = VisualServoModule()
    mod._latest_rgb = np.zeros((8, 8, 3), dtype=np.uint8)
    mod._latest_sg = SceneGraph(
        objects=[
            Detection3D(
                id="person-1",
                label="person",
                confidence=0.9,
                position=Vector3(1.0, 0.0, 0.0),
                bbox_2d=[5.0, 6.0, 20.0, 30.0],
            )
        ]
    )

    captured = {}
    mod._person_tracker.update = lambda scene_objects, rgb: captured.setdefault(
        "scene_objects", scene_objects
    )
    mod._person_tracker.get_follow_waypoint = lambda robot_pos: None

    mod._tick_follow()

    assert captured["scene_objects"][0]["bbox"] == [5.0, 6.0, 20.0, 30.0]

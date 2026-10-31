"""Regress the observed Habitat raster-center calibration error."""

import numpy as np
from tools.datasets.habitat_episode import (
    check_renderer_projection,
    has_approached_target,
    is_target,
    record_object_view,
    record_reload_observation,
)
from tools.datasets.habitat_objects import camera_transform

from runtime.msgs.sensor import CameraIntrinsics


def test_projection_agrees_with_independent_habitat_graphics_matrix_sample():
    # Recorded from Habitat's inverse projection/view matrices, not LingTu.
    position = [0.20659101009368896, 1.3193730115890503, 1.6761823892593384]
    sample = {"u": 320, "v": 240, "depth": 2.329545259475708,
              "renderer_world_point": [0.21023093163967133, 1.3157330751419067, -0.6533628702163696]}
    transform = camera_transform(position, [0, 0, 0, 1])
    intrinsics = CameraIntrinsics(width=640, height=480, fx=320, fy=320, cx=319.5, cy=239.5)

    row = check_renderer_projection([sample], transform, intrinsics)[0]

    assert row["error_m"] < 1e-6
    assert row["legacy_error_m"] > 0.005
    assert np.linalg.det(transform[:3, :3]) == 1


def test_exploration_distance_does_not_count_as_approaching_the_selected_target():
    selection = {"travelled_m": 10.0, "distance_m": 3.0}
    assert not has_approached_target(selection, travelled=10.0, distance=1.0)
    assert not has_approached_target(selection, travelled=10.4, distance=3.0)
    assert has_approached_target(selection, travelled=10.4, distance=1.0)


def test_empty_frame_after_reload_does_not_hide_later_real_match():
    from types import SimpleNamespace

    event = {"counts": {"2": 4}, "nonempty_observation_seen": False,
             "matched_existing_ids": None, "counts_incremented": None}
    record_reload_observation(event, 6, [0, 0, 0], [], [])
    assert event["next_frame_detection_count"] == 0
    assert event["matched_existing_ids"] is None
    assert event["counts_incremented"] is None
    obj = SimpleNamespace(object_id=2, detection_count=5)
    record_reload_observation(event, 9, [0.2, 0, 0], [object()], [obj])
    assert event["next_frame_step"] == 6
    assert event["matched_frame_step"] == 9
    assert event["matched_existing_ids"] == [2]
    assert event["counts_incremented"] is True


def test_context_objects_are_observed_but_not_selected_as_the_search_target():
    assert is_target("chair", "chair")
    assert is_target("bottle", "water bottle")
    assert not is_target("table", "water bottle")
    assert not is_target("cabinet", "chair")


def test_object_views_keep_distinct_camera_evidence_without_counting_duplicate_frames():
    from types import SimpleNamespace

    views = {}
    observation = SimpleNamespace(label="table", bbox_xyxy=np.array([10, 20, 40, 60]), confidence=0.8)
    obj = SimpleNamespace(object_id=2)
    current = {"rgb": "rgb/00000.jpg", "camera_position": [0, 1.2, 0],
               "camera_rotation_xyzw": [0, 0, 0, 1]}
    record_object_view(views, observation, obj, current, 0)
    record_object_view(views, observation, obj, current, 1)
    record_object_view(views, observation, obj, {**current, "camera_position": [0.4, 1.2, 0]}, 2)
    assert [view["step"] for view in views[2]] == [0, 2]
    assert views[2][0]["camera_pose_frame"] == "habitat_y_up"


def test_two_known_camera_positions_project_a_static_surface_to_the_same_world_point():
    from perception.tracking.projection import project_to_3d, transform_point

    intrinsics = CameraIntrinsics(width=640, height=480, fx=320, fy=320, cx=319.5, cy=239.5)
    first = project_to_3d(319.5+320/3, 207.5, 3.0, intrinsics)
    second = project_to_3d(399.5, 199.5, 2.4, intrinsics)
    a = transform_point(first, camera_transform([0, 1.2, 0], [0, 0, 0, 1]))
    b = transform_point(second, camera_transform([0.4, 1.2, -0.6], [0, 0, 0, 1]))
    np.testing.assert_allclose(a, [1, 3, 1.5], atol=1e-8)
    np.testing.assert_allclose(b, a, atol=1e-8)

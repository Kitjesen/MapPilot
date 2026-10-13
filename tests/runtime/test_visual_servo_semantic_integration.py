import math
import threading
import time

import numpy as np
import pytest

from decision.modules.visual_servo import VisualServoModule
from decision.vision import person as person_tracking
from message.generated.enums import NavigationGoalState
from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from runtime.msgs.nav import NavigationGoalStatus, VisualGoalCancel, VisualGoalRequest
from runtime.msgs.semantic import Detection3D
from runtime.msgs.sensor import Image, ImageFormat


def _goal(request: VisualGoalRequest) -> PoseStamped:
    assert isinstance(request, VisualGoalRequest)
    return request.goal


def _cancel_reasons(cancels: list[VisualGoalCancel]) -> list[str]:
    assert all(isinstance(cancel, VisualGoalCancel) for cancel in cancels)
    return [cancel.reason for cancel in cancels]


def _admit_current_goal(
    mod: VisualServoModule,
    *,
    accepted: bool = True,
    state: str | None = None,
    reason: str = "accepted",
) -> tuple[str, str]:
    task_id = mod._navigation_task_id
    assert task_id is not None
    request_id = mod._goal_requests[task_id]
    mod.goal_status._deliver({
        "action": "visual_servo",
        "accepted": accepted,
        "state": state or ("accepted" if accepted else "rejected"),
        "task_id": task_id,
        "request_id": request_id,
        "reason": reason,
    })
    return task_id, request_id


def _deliver_native_goal_status(
    mod: VisualServoModule,
    state: NavigationGoalState,
    *,
    task_id: str | None = None,
    request_id: str | None = None,
    reason: str = "",
    sequence: int = 1,
) -> None:
    task_id = task_id or mod._navigation_task_id
    assert task_id is not None
    request_id = request_id or mod._goal_requests[task_id]
    mod.navigation_goal_status._deliver(NavigationGoalStatus(
        boot_id="test-nav-boot",
        sequence=sequence,
        task_id=task_id,
        request_id=request_id,
        state=int(state),
        goal_epoch=1,
        reason=reason,
    ))


def test_follow_consumes_detection_dino_and_exposes_persistent_identity():
    module = VisualServoModule()
    feature = np.zeros(384, dtype=np.float32)
    feature[0] = 1.0
    objects = module._scene_objects_from_detections([
        Detection3D(
            id="track_7", label="person", confidence=0.9,
            position=Vector3(2.0, 0.0, 1.0), bbox_2d=[10, 10, 30, 40],
            dino_feature=feature, ts=10.0,
        ),
    ])
    assert objects[0]["dino_feature"] is feature
    module._person_tracker.lock_target(objects[0])
    status = module.get_servo_status()
    assert status["native_track_id"] == "track_7"
    assert status["target_identity_id"] == status["person"]["identity_id"]
    assert status["target_identity_id"]


def test_find_uses_current_map_detection_and_ground_standoff():
    mod = VisualServoModule()
    mod.setup()
    goals = []
    mod.goal_pose._add_callback(goals.append)
    mod.robot_pose._deliver(
        PoseStamped(
            pose=Pose(
                position=Vector3(0.0, 0.0, 0.25),
                orientation=Quaternion.from_yaw(0.0),
            ),
            ts=10.0,
            frame_id="map",
        )
    )
    mod._on_servo_target("find:chair")

    mod.detections_3d._deliver(
        [
            Detection3D(
                label="chair",
                confidence=0.9,
                position=Vector3(4.0, 0.0, 1.0),
                bbox_2d=[10.0, 20.0, 30.0, 40.0],
                ts=10.0,
            )
        ]
    )

    assert len(goals) == 1
    assert _goal(goals[0]).frame_id == "map"
    assert _goal(goals[0]).x == 2.5
    assert _goal(goals[0]).y == 0.0
    assert _goal(goals[0]).z == 0.25


def test_color_frame_alone_does_not_drive_follow_navigation():
    mod = VisualServoModule()
    mod._mode = "follow"

    called = []
    mod._tick_follow = lambda: called.append(True)

    mod._on_color(Image(data=np.zeros((8, 8, 3), dtype=np.uint8), format=ImageFormat.RGB))

    assert called == []


def test_stop_cancels_the_visual_navigation_task_and_waits_for_native_terminal():
    mod = VisualServoModule()
    cancels = []
    statuses = []
    mod.goal_cancel._add_callback(cancels.append)
    mod.servo_status._add_callback(statuses.append)

    assert mod._publish_goal_from_3d(np.array([2.0, 0.0, 0.0])) is True
    mod._on_servo_target("stop")

    assert _cancel_reasons(cancels) == ["visual_servo_stop"]
    assert statuses[-1]["mode"] == "idle"
    assert statuses[-1]["state"] == "stopping"


def test_stop_waits_for_the_native_visual_task_to_finish():
    mod = VisualServoModule()
    mod.setup()
    assert mod._publish_goal_from_3d(np.array([2.0, 0.0, 0.0])) is True
    task_id, request_id = _admit_current_goal(mod)

    mod._on_servo_target("stop")
    assert mod.get_servo_status()["state"] == "stopping"
    assert mod.get_servo_status()["navigation_state"] == "cancel_requested"

    _deliver_native_goal_status(
        mod,
        NavigationGoalState.CANCELLED,
        task_id=task_id,
        request_id=request_id,
        reason="operator_stop",
    )
    assert mod.get_servo_status()["state"] == "idle"
    assert mod.get_servo_status()["navigation_state"] == "cancelled"


def test_rejected_replacement_does_not_override_the_active_visual_task():
    mod = VisualServoModule()
    mod.setup()
    cancels = []
    mod.goal_cancel._add_callback(cancels.append)
    mod._mode = "follow"
    mod._target_visible = True
    assert mod._publish_goal_from_3d(np.array([2.0, 0.0, 0.0])) is True
    active_task_id, _ = _admit_current_goal(mod)

    mod.goal_status._deliver(
        {
            "action": "visual_servo",
            "accepted": False,
            "state": "rejected",
            "task_id": "visual-task-rejected",
            "reason": "no_path",
        }
    )

    status = mod.get_servo_status()
    assert status["navigation_state"] == "accepted"
    assert status["navigation_task_id"] == active_task_id
    assert mod._goal_published is True

    mod._on_servo_target("stop")
    assert _cancel_reasons(cancels) == ["visual_servo_stop"]


def test_first_native_goal_rejection_is_visible_and_allows_a_retry():
    mod = VisualServoModule()
    mod.setup()
    mod._mode = "follow"
    mod._target_visible = True
    mod._person_tracker.lock_target(
        {
            "id": "person-1",
            "label": "person",
            "position": [3.0, 0.0, 1.2],
            "confidence": 0.9,
            "ts": 20.0,
        }
    )
    assert mod._publish_goal_from_3d(np.array([2.0, 0.0, 0.0])) is True

    _admit_current_goal(mod, accepted=False, reason="no_path")

    status = mod.get_servo_status()
    assert status["state"] == "failed"
    assert status["navigation_state"] == "rejected"
    assert status["navigation_reason"] == "no_path"
    assert mod._goal_published is False
    assert mod._last_goal_position is None


def test_unowned_navigation_failure_cannot_change_the_current_follow():
    mod = VisualServoModule()
    mod._on_servo_target("follow:person in red")
    mod._on_goal_status({
        "action": "visual_servo", "accepted": False,
        "state": "rejected", "reason": "localization_not_tracking",
        "task_id": "not-owned", "request_id": "not-owned-request",
    })

    status = mod.get_servo_status()
    assert status["state"] == "selecting"
    assert status["navigation_state"] == "idle"
    assert status["navigation_reason"] == ""
    assert status["reason"] == "waiting_perception"


def test_follow_status_explains_selection_without_an_aligned_image():
    mod = VisualServoModule()
    mod._on_servo_target("follow_id:person-1")
    mod._person_tracker.set_dino_encoder(object())
    mod._on_detections([Detection3D(
        id="person-1", label="person", confidence=0.9,
        position=Vector3(4.0, 0.0, 1.2), ts=20.0,
    )])

    status = mod.get_servo_status()
    assert status["state"] == "selecting"
    assert status["reason"] == "waiting_selection_image"
    assert status["goal_position"] is None


def test_follow_status_explains_rejected_depth_and_recovers_on_valid_observation():
    mod = VisualServoModule()
    goals = []
    mod.goal_pose._add_callback(goals.append)
    mod._on_servo_target("follow_id:person-1")

    def observe(x, timestamp):
        mod._on_detections([Detection3D(
            id="person-1", label="person", confidence=0.9,
            position=Vector3(x, 0.0, 1.2), ts=timestamp,
        )])

    observe(4.0, 20.0)
    assert len(goals) == 1
    identity = mod.get_servo_status()["target_identity_id"]
    observe(8.0, 20.03)
    status = mod.get_servo_status()
    assert status["reason"] == "position_rejected"
    assert status["person"]["observation_reason"] == "position_rejected"
    assert status["person"]["position"] == [4.0, 0.0, 1.2]
    assert status["target_identity_id"] == identity
    assert len(goals) == 1

    observe(4.05, 20.06)
    assert mod.get_servo_status()["reason"] == ""
    assert mod.get_servo_status()["target_visible"] is True
    observe(4.05, 20.06)
    assert mod.get_servo_status()["reason"] == "stale_observation"


def test_follow_status_distinguishes_three_frame_evidence_from_missing_verifier():
    class Dino:
        def encode(self, _crop):
            return np.array([1.0, 0.0])

    mod = VisualServoModule()
    mod._person_tracker.set_dino_encoder(Dino())
    mod._on_servo_target("follow_id:original")
    frame = np.ones((16, 16, 3), dtype=np.uint8)

    def observe(identifier, timestamp):
        mod._on_color(Image(data=frame, format=ImageFormat.BGR, ts=timestamp))
        mod._on_detections([Detection3D(
            id=identifier, label="person", confidence=0.9,
            position=Vector3(4.0, 0.0, 1.2),
            bbox_2d=[0.0, 0.0, 16.0, 16.0], ts=timestamp,
        )])

    observe("original", 20.0)
    identity = mod.get_servo_status()["target_identity_id"]
    observe("returned", 28.0)
    assert mod.get_servo_status()["reason"] == "confirming"
    observe("returned", 28.03)
    assert mod.get_servo_status()["reason"] == "confirming"
    observe("returned", 28.06)
    status = mod.get_servo_status()
    assert status["reason"] == "identity_verification_unavailable"
    assert status["target_visible"] is False
    assert status["target_identity_id"] == identity
    assert status["native_track_id"] == "original"


def test_stop_skill_does_not_claim_physical_stop_on_cancel_request():
    mod = VisualServoModule()
    receipt = mod.follow_person("the visible person")
    assert receipt["accepted"] is True
    assert mod._publish_goal_from_3d(np.array([2.0, 0.0, 0.0])) is True
    _admit_current_goal(mod)

    reply = mod.stop_servo(receipt["follow_task_id"], receipt["product_session_id"])
    assert reply["accepted"] is True
    assert reply["reason"] == "cancel_requested"
    assert reply["execution_confirmed"] is False
    assert mod.get_servo_status()["state"] == "stopping"


def test_synchronous_goal_rejection_is_not_overwritten_by_local_publish_state():
    mod = VisualServoModule()
    mod.setup()

    def reject(goal_request):
        mod.goal_status._deliver(
            {
                "action": "visual_servo",
                "accepted": False,
                "state": "rejected",
                "task_id": goal_request.task_id,
                "request_id": goal_request.request_id,
                "reason": "no_path",
            }
        )

    mod.goal_pose._add_callback(reject)

    assert mod._publish_goal_from_3d(np.array([2.0, 0.0, 0.0])) is True
    assert mod._goal_published is False
    assert mod._last_goal_position is None
    assert mod.get_servo_status()["navigation_state"] == "rejected"
    assert mod._publish_goal_from_3d(np.array([2.0, 0.0, 0.0])) is False

    mod._last_goal_time -= mod._goal_interval_s + 0.01
    assert mod._publish_goal_from_3d(np.array([2.0, 0.0, 0.0])) is True


def test_old_cancel_receipt_does_not_override_the_current_visual_task():
    mod = VisualServoModule()
    mod.setup()
    assert mod._publish_goal_from_3d(np.array([2.0, 0.0, 0.0])) is True
    old_task_id, _ = _admit_current_goal(mod)
    mod._last_goal_time -= mod._goal_interval_s + 0.01
    assert mod._publish_goal_from_3d(np.array([3.0, 0.0, 0.0])) is True
    current_task_id, _ = _admit_current_goal(mod)

    mod.goal_status._deliver(
        {
            "action": "visual_servo_stop",
            "accepted": True,
            "state": "cancel_requested",
            "task_id": old_task_id,
            "reason": "superseded_by_new_goal",
        }
    )

    status = mod.get_servo_status()
    assert status["navigation_state"] == "accepted"
    assert status["navigation_task_id"] == current_task_id


def test_old_native_terminal_does_not_clear_the_current_visual_goal():
    mod = VisualServoModule()
    mod.setup()
    assert mod._publish_goal_from_3d(np.array([2.0, 0.0, 0.0])) is True
    old_task_id, old_request_id = _admit_current_goal(mod)
    mod._last_goal_time -= mod._goal_interval_s + 0.01
    assert mod._publish_goal_from_3d(np.array([3.0, 0.0, 0.0])) is True
    current_task_id, _ = _admit_current_goal(mod)

    _deliver_native_goal_status(
        mod,
        NavigationGoalState.CANCELLED,
        task_id=old_task_id,
        request_id=old_request_id,
        reason="superseded_by_new_goal",
    )

    status = mod.get_servo_status()
    assert status["navigation_task_id"] == current_task_id
    assert status["navigation_state"] == "accepted"
    assert mod._goal_published is True


def test_visual_goals_are_bounded_to_one_hz_and_ignore_small_motion():
    mod = VisualServoModule(goal_rate_hz=1.0, goal_deadband_m=0.25)
    goals = []
    mod.goal_pose._add_callback(goals.append)

    assert mod._publish_goal_from_3d(np.array([2.0, 0.0, 0.0])) is True
    assert mod._publish_goal_from_3d(np.array([3.0, 0.0, 0.0])) is False

    mod._last_goal_time -= 1.1
    assert mod._publish_goal_from_3d(np.array([2.1, 0.0, 0.0])) is False
    assert mod._publish_goal_from_3d(np.array([2.5, 0.0, 0.0])) is True
    assert len(goals) == 2


def test_follow_refuses_to_replace_an_active_visual_intent():
    mod = VisualServoModule()
    cancels = []
    mod.goal_cancel._add_callback(cancels.append)
    assert mod._publish_goal_from_3d(np.array([2.0, 0.0, 0.0])) is True

    mod._on_servo_target("follow:person in red")

    assert cancels == []
    assert mod._mode == "idle"
    assert mod._goal_published is True


def test_follow_uses_current_detection_and_ground_standoff():
    mod = VisualServoModule()
    mod.setup()

    class ImageSelector:
        def encode_text(self, texts):
            return np.array([[1.0, 0.0]])

        def encode_image(self, crop):
            return np.array([1.0, 0.0])

    mod._person_tracker.set_clip_encoder(ImageSelector())
    goals = []
    mod.goal_pose._add_callback(goals.append)
    mod.robot_pose._deliver(
        PoseStamped(
            pose=Pose(position=Vector3(0.0, 0.0, 0.2)),
            ts=20.0,
            frame_id="map",
        )
    )
    mod.servo_target._deliver("follow:person in red")
    mod.color_image._deliver(
        Image(
            data=np.ones((40, 40, 3), dtype=np.uint8),
            format=ImageFormat.RGB,
            ts=20.0,
        )
    )
    mod.detections_3d._deliver(
        [
            Detection3D(
                id="person-1",
                label="person",
                confidence=0.9,
                position=Vector3(4.0, 0.0, 1.2),
                bbox_2d=[0.0, 0.0, 30.0, 30.0],
                ts=20.0,
            )
        ]
    )

    assert len(goals) == 1
    assert _goal(goals[0]).x == 2.5
    assert _goal(goals[0]).y == 0.0
    assert _goal(goals[0]).z == 0.2


def test_follow_locks_the_only_visible_person_without_loading_a_semantic_model():
    mod = VisualServoModule()
    mod.setup()
    goals = []
    mod.goal_pose._add_callback(goals.append)
    mod.robot_pose._deliver(
        PoseStamped(
            pose=Pose(position=Vector3(0.0, 0.0, 0.2)),
            ts=20.0,
            frame_id="map",
        )
    )
    person = Detection3D(
        id="person-1",
        label="person",
        confidence=0.9,
        position=Vector3(4.0, 0.0, 1.2),
        bbox_2d=[0.0, 0.0, 30.0, 30.0],
        ts=20.0,
    )
    mod.detections_3d._deliver([person])

    receipt = mod.follow_person("the visible person")
    assert receipt["accepted"] is True
    assert receipt["follow_task_id"]
    assert receipt["state"] == "running"
    mod.detections_3d._deliver([person])

    assert len(goals) == 1
    assert _goal(goals[0]).x == 2.5
    assert _goal(goals[0]).z == 0.2
    assert mod.get_servo_status()["select"] == "single"


def test_single_person_fallback_confirms_the_same_track_before_locking():
    mod = VisualServoModule()
    mod.setup()
    goals = []
    mod.goal_pose._add_callback(goals.append)
    mod.robot_pose._deliver(
        PoseStamped(
            pose=Pose(position=Vector3(0.0, 0.0, 0.2)),
            ts=20.0,
            frame_id="map",
        )
    )

    def person(track_id: str) -> Detection3D:
        return Detection3D(
            id=track_id,
            label="person",
            confidence=0.9,
            position=Vector3(4.0, 0.0, 1.2),
            bbox_2d=[0.0, 0.0, 30.0, 30.0],
            ts=20.0,
        )

    mod.detections_3d._deliver([person("person-1")])
    assert mod.follow_person("the visible person")["accepted"] is True

    mod.detections_3d._deliver([person("person-2")])
    assert goals == []
    assert mod.get_servo_status()["select"] == "selecting"

    mod.detections_3d._deliver([person("person-2")])
    assert len(goals) == 1
    assert mod.get_servo_status()["person"]["id"] == "person-2"


def test_stale_detection_does_not_count_toward_single_person_confirmation():
    mod = VisualServoModule(lost_timeout=0.1)
    mod.setup()
    goals = []
    mod.goal_pose._add_callback(goals.append)
    mod.robot_pose._deliver(
        PoseStamped(
            pose=Pose(position=Vector3(0.0, 0.0, 0.2)),
            ts=20.0,
            frame_id="map",
        )
    )
    person = Detection3D(
        id="person-1",
        label="person",
        confidence=0.9,
        position=Vector3(4.0, 0.0, 1.2),
        bbox_2d=[0.0, 0.0, 30.0, 30.0],
        ts=20.0,
    )
    mod.detections_3d._deliver([person])
    assert mod.follow_person("the visible person")["accepted"] is True
    mod._last_perception_time -= 1.0

    mod.detections_3d._deliver([person])
    assert goals == []

    mod.detections_3d._deliver([person])
    assert len(goals) == 1


def test_follow_predicts_a_moving_person_before_publishing_the_native_nav_goal(monkeypatch):
    wall_time = [100.0]
    monkeypatch.setattr(person_tracking.time, "time", lambda: wall_time[0])

    mod = VisualServoModule()
    mod.setup()

    class ImageSelector:
        def encode_text(self, texts):
            return np.array([[1.0, 0.0]])

        def encode_image(self, crop):
            return np.array([1.0, 0.0])

    mod._person_tracker.set_clip_encoder(ImageSelector())
    goals = []
    mod.goal_pose._add_callback(goals.append)
    mod.robot_pose._deliver(
        PoseStamped(
            pose=Pose(position=Vector3(0.0, 0.0, 0.2)),
            ts=20.0,
            frame_id="map",
        )
    )
    mod.servo_target._deliver("follow:person in red")
    mod.color_image._deliver(
        Image(
            data=np.ones((40, 40, 3), dtype=np.uint8),
            format=ImageFormat.RGB,
            ts=20.0,
        )
    )
    mod.detections_3d._deliver(
        [
            Detection3D(
                id="person-1",
                label="person",
                confidence=0.9,
                position=Vector3(4.0, 0.0, 1.2),
                bbox_2d=[0.0, 0.0, 30.0, 30.0],
                ts=20.0,
            )
        ]
    )

    wall_time[0] = 101.0
    mod._last_goal_time -= 1.1
    mod.detections_3d._deliver(
        [
            Detection3D(
                id="person-1",
                label="person",
                confidence=0.9,
                position=Vector3(6.0, 0.0, 1.2),
                bbox_2d=[0.0, 0.0, 30.0, 30.0],
                ts=21.0,
            )
        ]
    )

    assert len(goals) == 2
    assert _goal(goals[-1]).frame_id == "map"
    assert _goal(goals[-1]).x == pytest.approx(3.48)
    assert _goal(goals[-1]).y == 0.0
    assert _goal(goals[-1]).z == 0.2

    status = mod.get_servo_status()
    assert status["frame_id"] == "map"
    assert status["person"]["id"] == "person-1"
    assert status["person"]["position"] == pytest.approx([4.8, 0.0, 1.2])
    assert status["person"]["velocity"] == pytest.approx([0.6, 0.0])
    assert status["person"]["last_seen"] == 101.0


def test_follow_loss_stops_the_owned_navigation_task_before_reselection():
    mod = VisualServoModule(lost_timeout=0.1)
    mod.setup()
    cancels = []
    mod.goal_cancel._add_callback(cancels.append)
    mod._mode = "follow"
    mod._target_label = "person in red"
    mod._follow_select_method = "clip"
    mod._person_tracker.lock_target(
        {
            "id": "person-1",
            "label": "person",
            "confidence": 0.9,
            "position": [4.0, 0.0, 1.2],
            "bbox": [0.0, 0.0, 30.0, 30.0],
        }
    )
    reference = np.full((20, 10, 3), 77, dtype=np.uint8)
    mod._follow_reference_crop = reference.copy()
    mod._person_tracker._person.last_seen -= 1.0
    assert mod._publish_goal_from_3d(np.array([2.5, 0.0, 0.2])) is True

    mod.detections_3d._deliver([])

    assert _cancel_reasons(cancels) == ["visual_servo_target_lost"]
    assert mod._goal_published is False
    assert mod._mode == "follow"
    assert mod._follow_select_pending is True
    np.testing.assert_array_equal(mod._follow_reference_crop, reference)


def test_follow_with_identity_reference_waits_for_the_same_person_and_resumes_goal():
    class _Dino:
        @staticmethod
        def encode(_crop):
            return np.array([1.0, 0.0])

    mod = VisualServoModule()
    mod.setup()
    goals = []
    cancels = []
    mod.goal_pose._add_callback(goals.append)
    mod.goal_cancel._add_callback(cancels.append)
    mod._mode = "follow"
    mod._target_label = "person in red"
    mod._follow_select_method = "clip"
    mod._person_tracker.set_dino_encoder(_Dino())
    mod._person_tracker.lock_target(
        {
            "id": "original",
            "label": "person",
            "position": [4.0, 0.0, 1.2],
            "bbox": [0.0, 0.0, 16.0, 16.0],
        },
        np.ones((16, 16, 3), dtype=np.uint8),
    )
    mod._person_tracker.is_lost = lambda: True
    mod._person_tracker.needs_vlm_reselect = lambda: True
    resets = []
    selections = []
    updates = []
    recovered = {"value": False}
    mod._person_tracker.reset = lambda: resets.append(True)
    mod._try_select_follow_target = lambda objects, **_kw: selections.append(objects)

    def update(objects, _frame):
        identifiers = [obj["id"] for obj in objects]
        updates.append(identifiers)
        recovered["value"] = identifiers == ["returned"]
        return recovered["value"]

    mod._person_tracker.update = update
    mod._person_tracker.get_follow_waypoint = lambda _pose: (
        {"x": 2.5, "y": 0.0, "z": 0.2} if recovered["value"] else None
    )
    mod._person_tracker.get_person_position = lambda: [4.0, 0.0, 1.2]
    assert mod._publish_goal_from_3d(np.array([2.5, 0.0, 0.2])) is True
    first_task_id = mod._navigation_task_id
    assert first_task_id is not None
    first_request_id = mod._goal_requests[first_task_id]

    mod._latest_detections = [
        Detection3D(
            id="wrong-person",
            label="person",
            confidence=0.9,
            position=Vector3(4.0, 0.0, 1.2),
            bbox_2d=[0.0, 0.0, 16.0, 16.0],
            ts=21.0,
        )
    ]
    mod._tick_follow()

    assert _cancel_reasons(cancels) == ["visual_servo_target_lost"]
    assert len(goals) == 1
    assert updates == [["wrong-person"]]
    assert resets == []
    assert selections == []
    assert mod._follow_select_pending is False
    assert mod.get_servo_status()["state"] == "reacquiring"

    _deliver_native_goal_status(
        mod,
        NavigationGoalState.CANCELLED,
        task_id=first_task_id,
        request_id=first_request_id,
        reason="visual_servo_target_lost",
    )

    mod._latest_detections = [
        Detection3D(
            id="returned",
            label="person",
            confidence=0.9,
            position=Vector3(4.0, 0.0, 1.2),
            bbox_2d=[0.0, 0.0, 16.0, 16.0],
            ts=22.0,
        )
    ]
    mod._tick_follow()

    assert updates[-1] == ["returned"]
    assert resets == []
    assert selections == []
    assert len(goals) == 2
    assert mod.get_servo_status()["state"] == "following"


def test_reacquisition_requires_three_distinct_observation_images():
    class _Dino:
        @staticmethod
        def encode(_crop):
            return np.array([1.0, 0.0])

    mod = VisualServoModule()
    mod.setup()
    mod._mode = "follow"
    mod._target_label = "person in red"
    mod._follow_select_method = "clip"
    mod._person_tracker.set_dino_encoder(_Dino())
    reference = np.ones((16, 16, 3), dtype=np.uint8)
    mod._person_tracker.lock_target(
        {
            "id": "original",
            "label": "person",
            "position": [4.0, 0.0, 1.2],
            "bbox": [0.0, 0.0, 16.0, 16.0],
            "ts": 20.0,
        },
        reference,
    )

    def image(timestamp: float) -> Image:
        return Image(data=reference.copy(), format=ImageFormat.BGR, ts=timestamp)

    def detection(timestamp: float) -> Detection3D:
        return Detection3D(
            id="returned",
            label="person",
            confidence=0.9,
            position=Vector3(4.0, 0.0, 1.2),
            bbox_2d=[0.0, 0.0, 16.0, 16.0],
            ts=timestamp,
        )

    mod.color_image._deliver(image(21.0))
    for timestamp in (21.0, 22.0, 23.0):
        mod.detections_3d._deliver([detection(timestamp)])

    assert mod._person_tracker.identity_candidate() is None

    for timestamp in (24.0, 25.0, 26.0):
        mod.color_image._deliver(image(timestamp))
        mod.detections_3d._deliver([detection(timestamp)])

    candidate = mod._person_tracker.identity_candidate()
    assert candidate is not None
    assert candidate["crop_timestamps"] == [24.0, 25.0, 26.0]


def test_follow_cancels_its_goal_when_the_locked_person_is_not_in_the_current_frame():
    mod = VisualServoModule()
    mod.setup()
    goals = []
    cancels = []
    mod.goal_pose._add_callback(goals.append)
    mod.goal_cancel._add_callback(cancels.append)
    mod.robot_pose._deliver(
        PoseStamped(
            pose=Pose(position=Vector3(0.0, 0.0, 0.2)),
            ts=20.0,
            frame_id="map",
        )
    )
    locked_person = Detection3D(
        id="person-1",
        label="person",
        confidence=0.9,
        position=Vector3(4.0, 0.0, 1.2),
        bbox_2d=[0.0, 0.0, 30.0, 30.0],
        ts=20.0,
    )
    mod.detections_3d._deliver([locked_person])
    assert mod.follow_person("the visible person")["accepted"] is True
    mod.detections_3d._deliver([locked_person])
    assert len(goals) == 1

    mod._last_goal_time -= 1.1
    mod._person_tracker._person.last_seen -= 1.0
    mod.detections_3d._deliver(
        [
            Detection3D(
                id="person-2",
                label="person",
                confidence=0.9,
                position=Vector3(10.0, 10.0, 1.2),
                bbox_2d=[0.0, 0.0, 30.0, 30.0],
                ts=21.0,
            )
        ]
    )

    assert len(goals) == 1
    assert _cancel_reasons(cancels) == ["visual_servo_target_lost"]
    assert mod.get_servo_status()["target_visible"] is False
    assert mod.get_servo_status()["person"]["id"] == "person-1"


def test_follow_cancels_its_goal_for_a_duplicate_detection_timestamp():
    mod = VisualServoModule()
    mod.setup()
    goals = []
    cancels = []
    mod.goal_pose._add_callback(goals.append)
    mod.goal_cancel._add_callback(cancels.append)
    mod.robot_pose._deliver(
        PoseStamped(
            pose=Pose(position=Vector3(0.0, 0.0, 0.2)),
            ts=20.0,
            frame_id="map",
        )
    )
    person = Detection3D(
        id="person-1",
        label="person",
        confidence=0.9,
        position=Vector3(4.0, 0.0, 1.2),
        bbox_2d=[0.0, 0.0, 30.0, 30.0],
        ts=20.0,
    )
    mod.detections_3d._deliver([person])
    assert mod.follow_person("the visible person")["accepted"] is True
    mod.detections_3d._deliver([person])
    assert len(goals) == 1
    status = mod.get_servo_status()
    assert status["robot_position"] == [0.0, 0.0, 0.2]
    assert status["goal_position"] == pytest.approx([2.5, 0.0, 0.2])
    assert status["distance_m"] == pytest.approx(4.0)
    assert status["desired_distance_m"] == pytest.approx(1.5)

    mod._last_goal_time -= 1.1
    mod._person_tracker._person.last_seen -= 1.0
    mod.detections_3d._deliver(
        [
            Detection3D(
                id="person-1",
                label="person",
                confidence=0.9,
                position=Vector3(8.0, 0.0, 1.2),
                bbox_2d=[0.0, 0.0, 30.0, 30.0],
                ts=20.0,
            )
        ]
    )

    assert len(goals) == 1
    assert _cancel_reasons(cancels) == ["visual_servo_target_lost"]
    assert mod.get_servo_status()["person"]["position"] == [4.0, 0.0, 1.2]


def test_idle_person_detection_publishes_follow_availability():
    mod = VisualServoModule()
    mod.setup()
    statuses = []
    mod.servo_status._add_callback(statuses.append)
    assert mod.can_select_follow_target() is False

    mod.detections_3d._deliver(
        [
            Detection3D(
                id="person-1",
                label="person",
                confidence=0.9,
                position=Vector3(4.0, 0.0, 1.2),
                bbox_2d=[0.0, 0.0, 30.0, 30.0],
                ts=20.0,
            )
        ]
    )

    assert mod.can_select_follow_target() is True
    assert statuses[-1]["follow_available"] is True


def test_follow_id_waits_for_the_exact_current_track_before_publishing_a_goal():
    mod = VisualServoModule()
    mod.setup()
    goals = []
    mod.goal_pose._add_callback(goals.append)
    mod.robot_pose._deliver(
        PoseStamped(
            pose=Pose(position=Vector3(0.0, 0.0, 0.2)),
            ts=20.0,
            frame_id="map",
        )
    )

    mod._on_servo_target("follow_id:person-7")
    assert mod.get_servo_status()["mode"] == "follow"
    assert mod.get_servo_status()["target_id"] == "person-7"
    assert goals == []

    mod.detections_3d._deliver(
        [
            Detection3D(
                id="person-8",
                label="person",
                confidence=0.9,
                position=Vector3(3.0, 0.0, 1.2),
                bbox_2d=[0.0, 0.0, 30.0, 30.0],
                ts=20.0,
            )
        ]
    )
    assert goals == []

    mod.detections_3d._deliver(
        [
            Detection3D(
                id="person-7",
                label="person",
                confidence=0.9,
                position=Vector3(4.0, 0.0, 1.2),
                bbox_2d=[0.0, 0.0, 30.0, 30.0],
                ts=21.0,
            )
        ]
    )

    assert len(goals) == 1
    assert mod.get_servo_status()["person"]["id"] == "person-7"


def test_follow_id_wrong_current_track_cancels_goal_without_changing_identity():
    mod = VisualServoModule()
    mod.setup()
    goals = []
    cancels = []
    mod.goal_pose._add_callback(goals.append)
    mod.goal_cancel._add_callback(cancels.append)
    mod._on_servo_target("follow_id:person-7")
    mod.detections_3d._deliver(
        [
            Detection3D(
                id="person-7",
                label="person",
                confidence=0.9,
                position=Vector3(4.0, 0.0, 1.2),
                bbox_2d=[0.0, 0.0, 30.0, 30.0],
                ts=20.0,
            )
        ]
    )
    assert len(goals) == 1

    mod._person_tracker._person.last_seen -= 1.0
    mod.detections_3d._deliver(
        [
            Detection3D(
                id="person-8",
                label="person",
                confidence=0.9,
                position=Vector3(4.1, 0.0, 1.2),
                bbox_2d=[0.0, 0.0, 30.0, 30.0],
                ts=21.0,
            )
        ]
    )

    assert _cancel_reasons(cancels) == ["visual_servo_target_lost"]
    assert mod.get_servo_status()["person"]["id"] == "person-7"
    assert mod.get_servo_status()["target_id"] == "person-7"


def test_follow_id_reacquires_the_same_person_after_the_tracker_changes_id():
    mod = VisualServoModule()
    mod.setup()
    goals = []
    cancels = []
    mod.goal_pose._add_callback(goals.append)
    mod.goal_cancel._add_callback(cancels.append)
    mod._on_servo_target("follow_id:person-7")
    mod.detections_3d._deliver(
        [
            Detection3D(
                id="person-7",
                label="person",
                confidence=0.9,
                position=Vector3(4.0, 0.0, 1.2),
                bbox_2d=[0.0, 0.0, 30.0, 30.0],
                ts=20.0,
            )
        ]
    )

    for timestamp, x in ((21.0, 4.1), (22.0, 4.2), (23.0, 4.3)):
        mod.detections_3d._deliver(
            [
                Detection3D(
                    id="person-8",
                    label="person",
                    confidence=0.9,
                    position=Vector3(x, 0.0, 1.2),
                    bbox_2d=[0.0, 0.0, 30.0, 30.0],
                    ts=timestamp,
                )
            ]
        )

    status = mod.get_servo_status()
    assert status["target_id"] == "person-8"
    assert status["person"]["id"] == "person-8"
    assert status["state"] == "following"
    assert cancels == []
    assert len(goals) == 1


@pytest.mark.parametrize("next_target", ["stop", "find:chair", "follow:person in red"])
def test_leaving_follow_id_clears_the_explicit_track(next_target):
    mod = VisualServoModule()
    receipt = mod.start_follow(target_id="person-7")
    assert receipt["accepted"] is True

    cancel = mod.cancel_follow_task(
        receipt["follow_task_id"], receipt["product_session_id"]
    )
    assert cancel["accepted"] is True
    assert cancel["terminal"] is True

    mod._on_servo_target(next_target)

    assert mod.get_servo_status()["target_id"] is None


def test_follow_updates_yaw_toward_the_person_while_holding_position(monkeypatch):
    wall_time = [100.0]
    monkeypatch.setattr(person_tracking.time, "time", lambda: wall_time[0])
    mod = VisualServoModule(follow_distance=1.5)
    mod.setup()
    goals = []
    mod.goal_pose._add_callback(goals.append)
    mod.robot_pose._deliver(
        PoseStamped(
            pose=Pose(position=Vector3(0.0, 0.0, 0.2)),
            ts=20.0,
            frame_id="map",
        )
    )
    person = Detection3D(
        id="person-1",
        label="person",
        confidence=0.9,
        position=Vector3(1.0, 0.0, 1.2),
        bbox_2d=[0.0, 0.0, 30.0, 30.0],
        ts=20.0,
    )
    mod.detections_3d._deliver([person])
    assert mod.follow_person("the visible person")["accepted"] is True
    mod.detections_3d._deliver([person])
    assert len(goals) == 1
    assert _goal(goals[-1]).x == 0.0
    assert _goal(goals[-1]).y == 0.0
    assert _goal(goals[-1]).yaw == pytest.approx(0.0)

    wall_time[0] = 101.0
    mod._last_goal_time -= 1.1
    mod.detections_3d._deliver(
        [
            Detection3D(
                id="person-1",
                label="person",
                confidence=0.9,
                position=Vector3(0.0, 1.0, 1.2),
                bbox_2d=[0.0, 0.0, 30.0, 30.0],
                ts=21.0,
            )
        ]
    )

    assert len(goals) == 2
    assert _goal(goals[-1]).x == 0.0
    assert _goal(goals[-1]).y == 0.0
    assert _goal(goals[-1]).yaw == pytest.approx(math.atan2(0.4, 0.6))


def test_every_stop_entry_wins_while_a_detection_callback_builds_a_goal():
    for invoke_stop in (
        lambda mod: mod._on_servo_target("stop"),
        lambda mod: mod.stop_servo(),
    ):
        mod = VisualServoModule()
        cancels = []
        entered = threading.Event()
        release = threading.Event()
        mod.goal_cancel._add_callback(cancels.append)
        mod._on_servo_target("find:chair")

        original_standoff = mod._standoff_position

        def blocked_standoff(
            target,
            distance,
            *,
            entered=entered,
            release=release,
            original_standoff=original_standoff,
        ):
            entered.set()
            assert release.wait(timeout=1.0)
            return original_standoff(target, distance)

        mod._standoff_position = blocked_standoff
        detection = Detection3D(
            label="chair",
            confidence=0.9,
            position=Vector3(4.0, 0.0, 1.0),
            bbox_2d=[0.0, 0.0, 20.0, 20.0],
            ts=30.0,
        )
        detector = threading.Thread(target=mod._on_detections, args=([detection],))
        stopper = threading.Thread(target=invoke_stop, args=(mod,))

        detector.start()
        assert entered.wait(timeout=1.0)
        stopper.start()
        time.sleep(0.02)
        release.set()
        detector.join(timeout=1.0)
        stopper.join(timeout=1.0)

        assert not detector.is_alive()
        assert not stopper.is_alive()
        assert mod._mode == "idle"
        assert mod._goal_published is False
        assert _cancel_reasons(cancels) == ["visual_servo_stop"]


def test_perception_stream_timeout_cancels_an_active_visual_goal():
    mod = VisualServoModule(lost_timeout=0.03)
    cancels = []
    mod.goal_cancel._add_callback(cancels.append)
    mod.setup()
    mod.start()
    try:
        mod._on_servo_target("find:chair")
        mod._on_detections(
            [
                Detection3D(
                    label="chair",
                    confidence=0.9,
                    position=Vector3(4.0, 0.0, 1.0),
                    bbox_2d=[0.0, 0.0, 20.0, 20.0],
                    ts=40.0,
                )
            ]
        )
        deadline = time.monotonic() + 0.5
        while not cancels and time.monotonic() < deadline:
            time.sleep(0.01)

        assert _cancel_reasons(cancels) == ["visual_servo_perception_stale"]
        assert mod._goal_published is False
        assert mod._mode == "find"
    finally:
        mod.stop()


def test_perception_timeout_preserves_a_locked_identity_for_reacquisition():
    class _Dino:
        @staticmethod
        def encode(_crop):
            return np.array([1.0, 0.0])

    mod = VisualServoModule(lost_timeout=0.03)
    cancels = []
    mod.goal_cancel._add_callback(cancels.append)
    mod.setup()
    mod.start()
    try:
        mod._mode = "follow"
        mod._target_label = "person in red"
        mod._follow_select_method = "clip"
        mod._person_tracker.set_dino_encoder(_Dino())
        mod._person_tracker.lock_target(
            {
                "id": "original",
                "label": "person",
                "position": [4.0, 0.0, 1.2],
                "bbox": [0.0, 0.0, 16.0, 16.0],
            },
            np.ones((16, 16, 3), dtype=np.uint8),
        )
        crop = np.ones((16, 16, 3), dtype=np.uint8)
        reacquisition = {
            "object": {
                "id": "returned",
                "label": "person",
                "position": [4.1, 0.0, 1.2],
                "bbox": [0.0, 0.0, 16.0, 16.0],
                "ts": 2.0,
            },
            "crop": crop.copy(),
            "reference_crop": crop.copy(),
            "feature": np.array([1.0, 0.0]),
        }
        mod._person_tracker._pending_identity = reacquisition
        mod._person_tracker._verified_identity = reacquisition
        mod._target_visible = True
        mod._last_perception_time = time.monotonic()
        assert mod._publish_goal_from_3d(np.array([2.5, 0.0, 0.2])) is True

        deadline = time.monotonic() + 0.5
        while not cancels and time.monotonic() < deadline:
            time.sleep(0.01)

        assert _cancel_reasons(cancels) == ["visual_servo_perception_stale"]
        assert mod._person_tracker.target_selected is True
        assert mod._person_tracker.has_identity_reference is True
        assert mod._person_tracker.identity_candidate() is None
        assert mod._person_tracker._verified_identity is None
        assert mod._follow_select_method == "clip"
        assert mod._follow_select_pending is False
        assert mod.get_servo_status()["state"] == "reacquiring"
    finally:
        mod.stop()

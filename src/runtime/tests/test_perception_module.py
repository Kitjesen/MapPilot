"""Tests for PerceptionModule -- core Module wrapper around perception.

Covers:
- Port declarations (5 In, 2 Out)
- Layer assignment (L3)
- Setup wires subscriptions
- Mock detector path: synthetic Detection2D -> 3D fallback -> scene_graph Out
- Depth/camera_info/odometry caching
- Skip-frame logic
- Graceful degradation (no detector/tracker available)
- Autoconnect with a downstream consumer module
"""

from __future__ import annotations

from dataclasses import dataclass, field
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from perception.perception_module import (
    PERCEPTION_MAP_FRAME_ID,
    PerceptionModule,
    _quat_to_rotation,
)
from runtime import Blueprint, In, Module
from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from runtime.msgs.nav import Odometry
from runtime.msgs.semantic import Detection3D as CoreDetection3D
from runtime.msgs.semantic import SceneGraph
from runtime.msgs.sensor import CameraIntrinsics, Image, ImageFormat

# -- helpers -------------------------------------------------------------------


def _make_bgr(
    h: int = 480,
    w: int = 640,
    *,
    ts: float | None = None,
    frame_id: str = "camera",
) -> Image:
    """Synthetic BGR image with non-zero data (passes Laplacian filter)."""
    data = np.random.randint(0, 255, (h, w, 3), dtype=np.uint8)
    kwargs = {"ts": ts} if ts is not None else {}
    return Image(data=data, format=ImageFormat.BGR, frame_id=frame_id, **kwargs)


def _make_depth(
    h: int = 480,
    w: int = 640,
    depth_mm: int = 2000,
    *,
    ts: float | None = None,
    frame_id: str = "camera",
) -> Image:
    """Synthetic depth image (uint16, millimetres)."""
    data = np.full((h, w), depth_mm, dtype=np.uint16)
    kwargs = {"ts": ts} if ts is not None else {}
    return Image(data=data, format=ImageFormat.DEPTH_U16, frame_id=frame_id, **kwargs)


def _make_intrinsics() -> CameraIntrinsics:
    return CameraIntrinsics(fx=600.0, fy=600.0, cx=320.0, cy=240.0, width=640, height=480)


def _make_odom(
    x: float = 1.0,
    y: float = 2.0,
    z: float = 0.35,
    *,
    ts: float | None = None,
    frame_id: str = PERCEPTION_MAP_FRAME_ID,
) -> Odometry:
    kwargs = {"ts": ts} if ts is not None else {}
    return Odometry(pose=Pose(Vector3(x, y, z), Quaternion()), frame_id=frame_id, **kwargs)


def _make_map_odom_tf(
    *,
    x: float = 0.0,
    y: float = 0.0,
    yaw: float = 0.0,
    ts: float = 5.0,
) -> dict:
    return {
        "valid": True,
        "frame_id": PERCEPTION_MAP_FRAME_ID,
        "child_frame_id": "odom",
        "tx": x,
        "ty": y,
        "tz": 0.0,
        "qx": 0.0,
        "qy": 0.0,
        "qz": np.sin(yaw / 2.0),
        "qw": np.cos(yaw / 2.0),
        "ts": ts,
    }


@dataclass
class FakeDetection2D:
    """Minimal Detection2D for testing the fallback projection path."""

    bbox: np.ndarray = field(default_factory=lambda: np.array([300, 220, 340, 260]))
    score: float = 0.9
    label: str = "chair"
    features: np.ndarray = field(default_factory=lambda: np.array([]))
    mask: object = None
    track_id: int | None = None


class FakeDetector:
    """Detector that returns canned 2D detections."""

    def __init__(self, detections=None):
        self._dets = [FakeDetection2D()] if detections is None else detections
        self._shutdown_called = False

    def detect(self, bgr, classes):
        return list(self._dets)

    def load_model(self):
        pass

    def shutdown(self):
        self._shutdown_called = True


class FakeSimObserver:
    def __init__(self, detections=None):
        self._dets = detections or [
            type(
                "Det",
                (),
                {
                    "position": np.array([16.25, 2.9, 0.9375], dtype=np.float32),
                    "label": "stairs",
                    "score": 1.0,
                    "bbox_2d": np.array([435.7, 173.6, 547.1, 285.0], dtype=np.float32),
                    "depth": 13.0,
                    "features": np.array([]),
                    "points": np.empty((0, 3), dtype=np.float32),
                },
            )()
        ]

    def observe(self, tf_camera_to_world, intrinsics, text_prompt=""):
        return list(self._dets)

    def shutdown(self):
        pass


# -- Port declaration tests ----------------------------------------------------


class TestPortDeclarations:
    def test_has_five_inputs(self):
        mod = PerceptionModule()
        assert len(mod.ports_in) == 5
        assert set(mod.ports_in.keys()) == {
            "color_image",
            "depth_image",
            "camera_info",
            "odometry",
            "map_odom_tf",
        }

    def test_has_three_outputs(self):
        mod = PerceptionModule()
        assert len(mod.ports_out) == 3
        assert set(mod.ports_out.keys()) == {
            "scene_graph",
            "detections_3d",
            "robot_pose",
        }

    def test_layer_is_3(self):
        mod = PerceptionModule()
        assert mod.layer == 3

    def test_output_types(self):
        mod = PerceptionModule()
        assert mod.scene_graph.msg_type is SceneGraph
        assert mod.detections_3d.msg_type is list
        assert mod.robot_pose.msg_type is PoseStamped

    def test_input_types(self):
        mod = PerceptionModule()
        assert mod.color_image.msg_type is Image
        assert mod.depth_image.msg_type is Image
        assert mod.camera_info.msg_type is CameraIntrinsics
        assert mod.odometry.msg_type is Odometry
        assert mod.map_odom_tf.msg_type is dict

    def test_localization_wires_map_odom_correction_to_perception(self):
        from types import SimpleNamespace

        from lingtu.assembly.wires.slam import localization_specs

        specs = localization_specs(
            SimpleNamespace(
                slam_module="SlamAdapterModule",
                camera_src="camera",
                color_out="color_image",
            )
        )
        assert any(
            spec.out_module == "SlamAdapterModule"
            and spec.out_port == "map_odom_tf"
            and spec.in_module == "PerceptionModule"
            and spec.in_port == "map_odom_tf"
            for spec in specs
        )

    def test_visual_servo_wires_current_map_observation_not_scene_memory(self):
        from types import SimpleNamespace

        from lingtu.assembly.wires.semantic import (
            semantic_scene_specs,
            visual_servo_specs,
        )

        specs = visual_servo_specs(
            SimpleNamespace(camera_src="camera", color_out="color_image")
        )
        assert any(
            spec.out_module == "PerceptionModule"
            and spec.out_port == "detections_3d"
            and spec.in_module == "VisualServoModule"
            and spec.in_port == "detections_3d"
            for spec in specs
        )
        assert any(
            spec.out_module == "PerceptionModule"
            and spec.out_port == "robot_pose"
            and spec.in_module == "VisualServoModule"
            and spec.in_port == "robot_pose"
            for spec in specs
        )
        assert not any(
            spec.in_module == "VisualServoModule"
            for spec in semantic_scene_specs()
        )


# -- Setup and lifecycle tests -------------------------------------------------


class TestSetupLifecycle:
    def test_setup_registers_subscriptions(self):
        """After setup(), all five In ports should have callbacks."""
        mod = PerceptionModule()
        mod.setup()
        assert mod.color_image.connected
        assert mod.depth_image.connected
        assert mod.camera_info.connected
        assert mod.odometry.connected
        assert mod.map_odom_tf.connected

    def test_stop_calls_detector_shutdown(self):
        mod = PerceptionModule()
        fake_det = FakeDetector()
        mod._detector = fake_det
        mod.stop()
        assert fake_det._shutdown_called

    def test_frame_count_starts_zero(self):
        mod = PerceptionModule()
        assert mod.frame_count == 0


# -- Data caching tests --------------------------------------------------------


class TestDataCaching:
    def test_depth_caching(self):
        mod = PerceptionModule()
        mod.setup()
        depth_img = _make_depth()
        mod.depth_image._deliver(depth_img)
        assert mod._latest_depth is not None
        assert mod._latest_depth.shape == (480, 640)

    def test_camera_info_one_shot(self):
        """Camera info should be stored only once."""
        mod = PerceptionModule()
        mod.setup()
        intr1 = _make_intrinsics()
        intr2 = CameraIntrinsics(fx=100.0, fy=100.0, cx=50.0, cy=50.0, width=100, height=100)
        mod.camera_info._deliver(intr1)
        mod.camera_info._deliver(intr2)
        # Should keep first
        stored_fx = getattr(mod._latest_intrinsics, "fx", None)
        assert stored_fx == pytest.approx(600.0)

    def test_odometry_builds_matrix(self):
        mod = PerceptionModule()
        mod.setup()
        mod.odometry._deliver(_make_odom(1.0, 2.0, 0.35))
        mat = mod._latest_odom_matrix
        assert mat is not None
        assert mat.shape == (4, 4)
        assert mat[0, 3] == pytest.approx(1.0)
        assert mat[1, 3] == pytest.approx(2.0)
        assert mat[2, 3] == pytest.approx(0.35)

    def test_odom_pose_is_composed_with_map_correction(self):
        mod = PerceptionModule()
        mod._on_map_odom_tf(_make_map_odom_tf(x=10.0, y=-2.0))
        mod._on_odometry(_make_odom(1.0, 2.0, 0.35, frame_id="odom"))

        mat = mod._latest_odom_matrix
        assert mat is not None
        assert mat[0, 3] == pytest.approx(11.0)
        assert mat[1, 3] == pytest.approx(0.0)
        assert mat[2, 3] == pytest.approx(0.35)

    def test_invalid_map_correction_clears_resolved_odom_pose(self):
        mod = PerceptionModule()
        mod._on_map_odom_tf(_make_map_odom_tf())
        mod._on_odometry(_make_odom(frame_id="odom"))
        assert mod._latest_odom_matrix is not None

        mod._on_map_odom_tf({"valid": False})

        assert mod._latest_odom_matrix is None


# -- Pipeline tests (mock detector, no real YOLO/CLIP) -------------------------


class TestPipeline:
    def _setup_module_with_fake_detector(self):
        """Create a PerceptionModule with a fake detector, feed intrinsics+depth."""
        mod = PerceptionModule(depth_scale=0.001, min_depth=0.3, max_depth=6.0)
        source_ts = 7.0
        mod._detector = FakeDetector()
        mod._on_camera_info(_make_intrinsics())
        mod._on_depth(_make_depth(depth_mm=2000, ts=source_ts))
        mod._on_odometry(_make_odom(ts=source_ts))
        return mod, source_ts

    def test_scene_graph_published_on_color_frame(self):
        mod, source_ts = self._setup_module_with_fake_detector()
        received_sg = []
        mod.scene_graph._add_callback(received_sg.append)

        mod._on_color_frame(_make_bgr(ts=source_ts))
        assert len(received_sg) == 1
        assert isinstance(received_sg[0], SceneGraph)
        assert len(received_sg[0].objects) >= 1
        assert received_sg[0].ts == pytest.approx(source_ts)

    def test_detections_3d_published(self):
        mod, source_ts = self._setup_module_with_fake_detector()
        received_dets = []
        mod.detections_3d._add_callback(received_dets.append)

        mod._on_color_frame(_make_bgr(ts=source_ts))
        assert len(received_dets) == 1
        dets = received_dets[0]
        assert isinstance(dets, list)
        assert len(dets) >= 1
        assert isinstance(dets[0], CoreDetection3D)
        assert dets[0].label == "chair"
        assert dets[0].ts == pytest.approx(source_ts)

    def test_synchronized_map_robot_pose_is_published_before_detections(self):
        mod, source_ts = self._setup_module_with_fake_detector()
        events = []
        mod.robot_pose._add_callback(lambda pose: events.append(("pose", pose)))
        mod.detections_3d._add_callback(
            lambda detections: events.append(("detections", detections))
        )

        mod._on_color_frame(_make_bgr(ts=source_ts))

        assert [event[0] for event in events] == ["pose", "detections"]
        pose = events[0][1]
        assert pose.frame_id == "map"
        assert pose.ts == pytest.approx(source_ts)
        assert pose.x == pytest.approx(1.0)
        assert pose.y == pytest.approx(2.0)

    def test_skip_frames(self):
        """With skip_frames=2, only every other frame produces output."""
        mod = PerceptionModule(skip_frames=2)
        source_ts = 9.0
        mod._detector = FakeDetector()
        mod._on_camera_info(_make_intrinsics())
        mod._on_depth(_make_depth(ts=source_ts))
        mod._on_odometry(_make_odom(ts=source_ts))

        received = []
        mod.detections_3d._add_callback(received.append)

        mod._on_color_frame(_make_bgr(ts=source_ts))  # frame 1 -> skipped
        assert len(received) == 0
        mod._on_color_frame(_make_bgr(ts=source_ts))  # frame 2 -> processed
        assert len(received) == 1
        mod._on_color_frame(_make_bgr(ts=source_ts))  # frame 3 -> skipped
        assert len(received) == 1

    def test_no_output_without_intrinsics(self):
        """Missing camera_info clears stale semantic outputs."""
        mod = PerceptionModule()
        mod._detector = FakeDetector()
        mod._on_depth(_make_depth(ts=6.0))
        mod._on_odometry(_make_odom(ts=6.0))

        received = []
        received_sg = []
        mod.detections_3d._add_callback(received.append)
        mod.scene_graph._add_callback(received_sg.append)
        mod._on_color_frame(_make_bgr(ts=6.0))
        assert received == [[]]
        assert len(received_sg) == 1
        assert received_sg[0].objects == []
        assert mod.health()["last_drop_reason"] == "missing_rgbd_calibration"

    def test_no_output_without_depth(self):
        """Missing depth clears stale semantic outputs."""
        mod = PerceptionModule()
        mod._detector = FakeDetector()
        mod._on_camera_info(_make_intrinsics())
        mod._on_odometry(_make_odom(ts=6.0))

        received = []
        received_sg = []
        mod.detections_3d._add_callback(received.append)
        mod.scene_graph._add_callback(received_sg.append)
        mod._on_color_frame(_make_bgr(ts=6.0))
        assert received == [[]]
        assert len(received_sg) == 1
        assert received_sg[0].objects == []
        assert mod.health()["last_drop_reason"] == "missing_rgbd_calibration"

    def test_sim_scene_backend_bypasses_blur_filter(self):
        mod = PerceptionModule(detector_type="sim_scene")
        mod._tracker = None
        mod._sim_scene_observer = FakeSimObserver()
        source_ts = 11.0
        mod._on_camera_info(_make_intrinsics())
        mod._on_depth(_make_depth(depth_mm=10000, ts=source_ts))
        mod._on_odometry(_make_odom(2.0, 3.0, 0.5, ts=source_ts))

        received = []
        mod.detections_3d._add_callback(received.append)

        blurry = Image(
            data=np.full((480, 640, 3), 127, dtype=np.uint8),
            format=ImageFormat.BGR,
            ts=source_ts,
            frame_id="camera",
        )
        mod._on_color_frame(blurry)

        assert len(received) == 1
        assert received[0][0].label == "stairs"
        assert mod.health()["detector_ready"] is True

    def test_fallback_projection_position(self):
        """Fallback 3D projection produces reasonable world-frame positions."""
        mod, source_ts = self._setup_module_with_fake_detector()
        received_dets = []
        mod.detections_3d._add_callback(received_dets.append)
        mod._on_color_frame(_make_bgr(ts=source_ts))

        det = received_dets[0][0]
        # Position should be non-zero with 2m depth
        pos = det.position
        assert pos.z != 0.0 or pos.x != 0.0 or pos.y != 0.0

    def test_missing_odometry_publishes_empty_scene_graph_without_3d_detections(self):
        mod = PerceptionModule(depth_scale=0.001, min_depth=0.3, max_depth=6.0)
        mod._detector = FakeDetector()
        mod._tracker = object()
        mod._tracking_service.tracker = mod._tracker
        mod._tracking_service.update = MagicMock(return_value=[])
        mod._on_camera_info(_make_intrinsics())
        mod._on_depth(_make_depth(depth_mm=2000))

        received_dets = []
        received_sg = []
        mod.detections_3d._add_callback(received_dets.append)
        mod.scene_graph._add_callback(received_sg.append)

        mod._on_color_frame(_make_bgr(ts=7.0))

        assert received_dets == [[]]
        assert len(received_sg) == 1
        assert received_sg[0].objects == []
        assert received_sg[0].frame_id == PERCEPTION_MAP_FRAME_ID
        assert received_sg[0].ts == pytest.approx(7.0)
        mod._tracking_service.update.assert_not_called()

    def test_stale_depth_publishes_empty_scene_graph(self):
        mod = PerceptionModule(
            depth_scale=0.001,
            min_depth=0.3,
            max_depth=6.0,
            max_rgbd_skew_s=0.1,
        )
        mod._detector = FakeDetector()
        mod._on_camera_info(_make_intrinsics())
        mod._on_depth(_make_depth(depth_mm=2000, ts=10.0))
        mod._on_odometry(_make_odom(ts=10.25))

        received_dets = []
        received_sg = []
        mod.detections_3d._add_callback(received_dets.append)
        mod.scene_graph._add_callback(received_sg.append)

        mod._on_color_frame(_make_bgr(ts=10.25))

        assert received_dets == [[]]
        assert len(received_sg) == 1
        assert received_sg[0].objects == []
        assert received_sg[0].ts == pytest.approx(10.25)

    def test_depth_frame_mismatch_publishes_empty_scene_graph(self):
        mod = PerceptionModule(depth_scale=0.001, min_depth=0.3, max_depth=6.0)
        mod._detector = FakeDetector()
        mod._on_camera_info(_make_intrinsics())
        mod._on_depth(_make_depth(depth_mm=2000, ts=5.0, frame_id="depth_camera"))
        mod._on_odometry(_make_odom(ts=5.0))

        received_dets = []
        received_sg = []
        mod.detections_3d._add_callback(received_dets.append)
        mod.scene_graph._add_callback(received_sg.append)

        mod._on_color_frame(_make_bgr(ts=5.0, frame_id="rgb_camera"))

        assert received_dets == [[]]
        assert len(received_sg) == 1
        assert received_sg[0].objects == []
        assert received_sg[0].ts == pytest.approx(5.0)

    def test_odom_frame_without_map_correction_publishes_empty_map_scene_graph(self):
        mod = PerceptionModule(depth_scale=0.001, min_depth=0.3, max_depth=6.0)
        mod._detector = FakeDetector()
        mod._on_camera_info(_make_intrinsics())
        mod._on_depth(_make_depth(depth_mm=2000, ts=5.0))
        mod._on_odometry(_make_odom(ts=5.0, frame_id="odom"))

        received_dets = []
        received_sg = []
        mod.detections_3d._add_callback(received_dets.append)
        mod.scene_graph._add_callback(received_sg.append)

        mod._on_color_frame(_make_bgr(ts=5.0))

        assert received_dets == [[]]
        assert len(received_sg) == 1
        assert received_sg[0].objects == []
        assert received_sg[0].frame_id == PERCEPTION_MAP_FRAME_ID
        assert received_sg[0].ts == pytest.approx(5.0)
        assert mod.health()["last_drop_reason"] == "missing_map_odom_transform"

    def test_odom_frame_with_map_correction_publishes_map_scene_graph(self):
        mod = PerceptionModule(depth_scale=0.001, min_depth=0.3, max_depth=6.0)
        mod._detector = FakeDetector()
        mod._on_camera_info(_make_intrinsics())
        mod._on_depth(_make_depth(depth_mm=2000, ts=5.0))
        mod._on_map_odom_tf(_make_map_odom_tf(x=3.0))
        mod._on_odometry(_make_odom(ts=5.0, frame_id="odom"))

        received_sg = []
        mod.scene_graph._add_callback(received_sg.append)
        mod._on_color_frame(_make_bgr(ts=5.0))

        assert len(received_sg) == 1
        assert len(received_sg[0].objects) == 1
        assert received_sg[0].frame_id == PERCEPTION_MAP_FRAME_ID

    def test_stale_map_odom_correction_publishes_empty_scene_graph(self):
        mod = PerceptionModule(
            depth_scale=0.001,
            min_depth=0.3,
            max_depth=6.0,
            max_map_odom_age_s=0.2,
        )
        mod._detector = FakeDetector()
        mod._on_camera_info(_make_intrinsics())
        mod._on_depth(_make_depth(depth_mm=2000, ts=5.0))
        mod._on_map_odom_tf(_make_map_odom_tf(ts=4.0))
        mod._on_odometry(_make_odom(ts=5.0, frame_id="odom"))

        received_sg = []
        mod.scene_graph._add_callback(received_sg.append)
        mod._on_color_frame(_make_bgr(ts=5.0))

        assert len(received_sg) == 1
        assert received_sg[0].objects == []
        assert mod.health()["last_drop_reason"] == "map_odom_time_skew"

    def test_frame_processing_uses_one_atomic_sample_snapshot(self):
        mod = PerceptionModule(depth_scale=0.001, min_depth=0.3, max_depth=6.0)
        mod._detector = FakeDetector()
        mod._on_camera_info(_make_intrinsics())
        original_depth = _make_depth(depth_mm=2000, ts=5.0)
        mod._on_depth(original_depth)
        mod._on_odometry(_make_odom(1.0, 2.0, 0.35, ts=5.0))
        original_pose = mod._latest_odom_matrix.copy()
        original_sync = mod._samples_are_synchronized

        def update_inputs_after_snapshot(*args, **kwargs):
            mod._on_depth(_make_depth(depth_mm=5000, ts=5.0))
            mod._on_odometry(_make_odom(99.0, 98.0, 0.35, ts=5.0))
            return original_sync(*args, **kwargs)

        mod._samples_are_synchronized = MagicMock(side_effect=update_inputs_after_snapshot)
        mod._process_frame = MagicMock()
        mod._on_color_frame(_make_bgr(ts=5.0))

        mod._process_frame.assert_called_once()
        args = mod._process_frame.call_args.args
        assert args[1] is original_depth.data
        np.testing.assert_allclose(args[3], original_pose @ mod._camera_body_transform())

    def test_empty_detector_result_advances_tracker_negative_observation(self):
        mod = PerceptionModule(depth_scale=0.001, min_depth=0.3, max_depth=6.0)
        mod._detector = FakeDetector(detections=[])
        mod._tracker = object()
        mod._tracking_service.tracker = mod._tracker
        mod._tracking_service.update = MagicMock(return_value=[])
        mod._on_camera_info(_make_intrinsics())
        mod._on_depth(_make_depth(depth_mm=2000, ts=8.0))
        mod._on_odometry(_make_odom(ts=8.0))

        received_sg = []
        received_dets = []
        mod.detections_3d._add_callback(received_dets.append)
        mod.scene_graph._add_callback(received_sg.append)

        mod._on_color_frame(_make_bgr(ts=8.0))

        mod._tracking_service.update.assert_called_once()
        args, kwargs = mod._tracking_service.update.call_args
        assert args == ([],)
        assert kwargs["camera_pos"].shape == (3,)
        assert kwargs["camera_forward"].shape == (3,)
        assert kwargs["intrinsics_fx"] == pytest.approx(600.0)
        assert mod._latest_core_detections == []
        assert received_dets == [[]]
        assert len(received_sg) == 1
        assert received_sg[0].objects == []
        assert received_sg[0].ts == pytest.approx(8.0)

    def test_processing_exception_clears_scene_graph_and_detections(self):
        mod = PerceptionModule(depth_scale=0.001, min_depth=0.3, max_depth=6.0)
        mod._detector = FakeDetector()
        mod._tracker = object()
        mod._tracking_service.tracker = mod._tracker
        mod._tracking_service.update = MagicMock(return_value=[])
        mod._process_frame = MagicMock(side_effect=RuntimeError("boom"))
        mod._on_camera_info(_make_intrinsics())
        mod._on_depth(_make_depth(depth_mm=2000, ts=13.0))
        mod._on_odometry(_make_odom(ts=13.0))

        received_dets = []
        received_sg = []
        mod.detections_3d._add_callback(received_dets.append)
        mod.scene_graph._add_callback(received_sg.append)

        mod._on_color_frame(_make_bgr(ts=13.0))

        assert received_dets == [[]]
        assert len(received_sg) == 1
        assert received_sg[0].objects == []
        assert received_sg[0].ts == pytest.approx(13.0)
        assert mod.health()["last_drop_reason"] == "frame_processing_error"
        mod._tracking_service.update.assert_not_called()

    def test_valid_scene_graph_uses_rgb_source_timestamp(self):
        mod = PerceptionModule(depth_scale=0.001, min_depth=0.3, max_depth=6.0)
        mod._detector = FakeDetector()
        mod._tracker = None
        mod._on_camera_info(_make_intrinsics())
        mod._on_depth(_make_depth(depth_mm=2000, ts=12.0))
        mod._on_odometry(_make_odom(ts=12.0))

        received_sg = []
        mod.scene_graph._add_callback(received_sg.append)

        mod._on_color_frame(_make_bgr(ts=12.0))

        assert len(received_sg) == 1
        assert len(received_sg[0].objects) == 1
        assert received_sg[0].ts == pytest.approx(12.0)


# -- Graceful degradation tests ------------------------------------------------


class TestGracefulDegradation:
    def test_no_detector_no_crash(self):
        """Module works even with no detector -- just produces no output."""
        mod = PerceptionModule()
        # _detector remains None
        mod._on_camera_info(_make_intrinsics())
        mod._on_depth(_make_depth(ts=14.0))
        mod._on_odometry(_make_odom(ts=14.0))

        received = []
        mod.detections_3d._add_callback(received.append)
        mod._on_color_frame(_make_bgr(ts=14.0))
        assert received == [[]]  # No crash; stale detections are cleared.

    def test_no_tracker_still_publishes_detections(self):
        """Without tracker, detections are still published."""
        mod = PerceptionModule()
        mod.setup()
        mod._detector = FakeDetector()
        mod._tracker = None  # Explicitly no tracker
        mod.camera_info._deliver(_make_intrinsics())
        mod.depth_image._deliver(_make_depth())
        mod.odometry._deliver(_make_odom())

        det_received = []
        sg_received = []
        mod.detections_3d._add_callback(det_received.append)
        mod.scene_graph._add_callback(sg_received.append)
        mod.color_image._deliver(_make_bgr())

        assert len(det_received) == 1
        assert len(sg_received) == 1
        # Fallback scene_graph should still preserve the latest detections.
        assert len(sg_received[0].objects) == 1
        assert sg_received[0].objects[0].bbox_2d == [300.0, 220.0, 340.0, 260.0]


class TestSceneGraphMetadata:
    def test_tracker_scene_graph_preserves_latest_bbox_metadata(self):
        mod = PerceptionModule()
        mod._latest_core_detections = [
            CoreDetection3D(
                id="det-1",
                label="chair",
                confidence=0.9,
                position=Vector3(1.0, 2.0, 0.5),
                bbox_2d=[10.0, 20.0, 30.0, 40.0],
            )
        ]

        class _FakeTracker:
            def get_scene_graph_json(self):
                return """
                {
                  "objects": [
                    {
                      "id": "track-1",
                      "label": "chair",
                      "score": 0.95,
                      "position": {"x": 1.0, "y": 2.0, "z": 0.5}
                    }
                  ],
                  "relations": [],
                  "rooms": []
                }
                """

        mod._tracker = _FakeTracker()

        sg = mod._scene_graph_service.build_scene_graph(
            mod._latest_core_detections,
            PERCEPTION_MAP_FRAME_ID,
            tracker=mod._tracker,
        )

        assert len(sg.objects) == 1
        assert sg.objects[0].bbox_2d == [10.0, 20.0, 30.0, 40.0]


# -- Autoconnect integration test ----------------------------------------------


class _DownstreamPlanner(Module, layer=5):
    """Dummy downstream consumer for autoconnect test."""

    scene_graph: In[SceneGraph]
    detections_3d: In[list]


class TestAutoconnect:
    def test_autoconnect_with_planner(self):
        """PerceptionModule.scene_graph -> PlannerModule.scene_graph via auto_wire."""
        bp = Blueprint().add(PerceptionModule).add(_DownstreamPlanner).auto_wire()
        system = bp.build()
        # SystemHandle.start() calls setup() + start() on all modules
        system.start()

        # Find modules by type from the name->module dict
        perception = None
        planner = None
        for _name, mod in system.modules.items():
            if isinstance(mod, PerceptionModule):
                perception = mod
            elif isinstance(mod, _DownstreamPlanner):
                planner = mod
        assert perception is not None
        assert planner is not None

        # Verify the wiring: publishing on perception.scene_graph should
        # deliver to planner.scene_graph
        received = []
        planner.scene_graph.subscribe(received.append)

        sg = SceneGraph(objects=[CoreDetection3D(label="test")])
        perception.scene_graph.publish(sg)

        assert len(received) == 1
        assert received[0].objects[0].label == "test"

        system.stop()


# -- Utility function test -----------------------------------------------------


class TestQuatToRotation:
    def test_identity_quaternion(self):
        """(0,0,0,1) should give identity rotation."""
        R = _quat_to_rotation(0.0, 0.0, 0.0, 1.0)
        np.testing.assert_allclose(R, np.eye(3), atol=1e-10)

    def test_90deg_z_rotation(self):
        """90-degree rotation around Z axis."""
        import math

        angle = math.pi / 2
        qw = math.cos(angle / 2)
        qz = math.sin(angle / 2)
        R = _quat_to_rotation(0.0, 0.0, qz, qw)
        expected = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)
        np.testing.assert_allclose(R, expected, atol=1e-10)


class TestDetectorConfiguration:
    def test_perception_backend_registry_names_are_visible(self):
        from runtime.registry import list_plugins

        assert {"yoloe", "yolo_world", "bpu", "sim_scene"} <= set(list_plugins("detector"))
        assert {"clip", "mobileclip"} <= set(list_plugins("encoder"))
        assert {"bpu"} <= set(list_plugins("perception_tracker"))

    def test_unknown_perception_backend_fails_fast(self):
        with pytest.raises(ValueError, match="Unknown detector backend 'bogus'"):
            PerceptionModule(detector_type="bogus")
        with pytest.raises(ValueError, match="Unknown encoder backend 'bogus'"):
            PerceptionModule(encoder_type="bogus")

    def test_constructor_accepts_registered_detector_and_encoder_plugins(self):
        from runtime.registry import register

        @register("detector", "constructor_test_detector")
        class _DetectorProvider:
            @staticmethod
            def create(_module):
                return object()

        @register("encoder", "constructor_test_encoder")
        class _EncoderProvider:
            @staticmethod
            def create(_module):
                return object()

        mod = PerceptionModule(
            detector_type="constructor_test_detector",
            encoder_type="constructor_test_encoder",
        )

        assert mod._detector_type == "constructor_test_detector"
        assert mod._encoder_type == "constructor_test_encoder"

    @patch("perception.tracking.instance_tracker.InstanceTracker")
    def test_setup_uses_tracking_iou_threshold(self, instance_tracker_cls):
        mod = PerceptionModule(
            merge_distance=0.8,
            max_objects=33,
            tracking_iou_threshold=0.27,
            confidence_threshold=0.91,
        )
        mod._backend_manager._init_detector = MagicMock(return_value=(None, None))
        mod._backend_manager._init_clip_encoder = MagicMock(return_value=None)

        mod.setup()

        instance_tracker_cls.assert_called_once_with(
            merge_distance=0.8,
            iou_threshold=0.27,
            max_objects=33,
        )

    @patch("perception.detection.yoloe_detector.YOLOEDetector")
    def test_init_yoloe_detector_forwards_recall_params(self, yoloe_cls):
        backend = MagicMock()
        yoloe_cls.return_value = backend

        mod = PerceptionModule(
            detector_type="yoloe",
            confidence_threshold=0.18,
            detector_iou_threshold=0.41,
            detector_max_detections=96,
            detector_model_size="m",
            detector_device="cuda:0",
        )

        created, observer = mod._backend_manager._init_detector()

        yoloe_cls.assert_called_once_with(
            model_size="m",
            confidence=0.18,
            iou_threshold=0.41,
            device="cuda:0",
            max_detections=96,
        )
        backend.load_model.assert_called_once_with()
        assert created is backend
        assert observer is None

    @patch("perception.detection.yolo_world_detector.YOLOWorldDetector")
    def test_init_yolo_world_detector_forwards_iou(self, yolo_world_cls):
        backend = MagicMock()
        yolo_world_cls.return_value = backend

        mod = PerceptionModule(
            detector_type="yolo_world",
            confidence_threshold=0.2,
            detector_iou_threshold=0.44,
            detector_model_size="s",
            detector_device="cpu",
        )

        created, observer = mod._backend_manager._init_detector()

        yolo_world_cls.assert_called_once_with(
            model_size="s",
            confidence=0.2,
            iou_threshold=0.44,
            device="cpu",
        )
        backend.load_model.assert_called_once_with()
        assert created is backend
        assert observer is None

    @patch("perception.detection.bpu_detector.BPUDetector")
    def test_init_bpu_detector_forwards_recall_params(self, bpu_cls):
        backend = MagicMock()
        bpu_cls.return_value = backend

        mod = PerceptionModule(
            detector_type="bpu",
            confidence_threshold=0.22,
            detector_iou_threshold=0.45,
            detector_max_detections=72,
            detector_min_box_size_px=10,
            detector_model_path="D:/models/people.hbm",
        )

        created, observer = mod._backend_manager._init_detector()

        bpu_cls.assert_called_once_with(
            model_path="D:/models/people.hbm",
            confidence=0.22,
            iou_threshold=0.45,
            max_detections=72,
            min_box_size_px=10,
        )
        backend.load_model.assert_called_once_with()
        assert created is backend
        assert observer is None

    @patch("perception.tracking.bpu_tracker.BPUTracker")
    def test_init_detector_tracker_for_bpu(self, tracker_cls):
        backend = MagicMock()
        tracker = MagicMock()
        tracker_cls.return_value = tracker

        mod = PerceptionModule(detector_type="bpu")
        mod._detector = backend

        created = mod._backend_manager._init_detector_tracker()

        tracker_cls.assert_called_once_with(backend, tracker_type="botsort")
        assert created is tracker

    def test_run_detector_prefers_detector_tracker(self):
        mod = PerceptionModule(detector_type="bpu")
        mod._default_classes = "person"
        mod._detector = MagicMock()
        mod._detector_tracker = MagicMock()
        mod._detector_tracker.track.return_value = ["tracked"]

        result = mod._run_detector(np.zeros((32, 32, 3), dtype=np.uint8))

        assert result == ["tracked"]
        mod._detector_tracker.track.assert_called_once()
        mod._detector.detect.assert_not_called()

    def test_convert_detections_preserves_detector_track_id(self):
        mod = PerceptionModule()
        det = type(
            "ProjDet",
            (),
            {
                "position": np.array([1.0, 2.0, 3.0]),
                "label": "person",
                "score": 0.87,
                "bbox_2d": np.array([10, 20, 30, 60], dtype=np.float32),
                "features": np.array([]),
                "track_id": 42,
            },
        )()

        converted = mod._detection_service.convert_to_core_detections([det])

        assert converted[0].id == "track_42"
        assert converted[0].label == "person"

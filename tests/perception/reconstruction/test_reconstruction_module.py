"""Contract tests for reconstruction modules.

Verifies module instantiation, port registration, and lifecycle for:
  - ReconstructionModule
  - DatasetRecorderModule
  - ReconKeyframeExporterModule

These are CONTRACT tests — they verify the module interface contract, not
internal implementation details or algorithmic correctness.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

# =============================================================================
# ReconstructionModule
# =============================================================================


class TestReconstructionModule:
    """Contract tests for ReconstructionModule (layer=3, RGB-D voxel map)."""

    def test_instantiation_with_defaults(self):
        """Creating a ReconstructionModule with default config should succeed."""
        from perception.reconstruction.reconstruction_module import ReconstructionModule

        mod = ReconstructionModule()
        assert mod._process_hz == 5.0
        assert mod._min_points == 1000
        assert mod._mask_dynamic is True
        assert mod._projector is not None
        assert mod._labeler is None
        mod._ensure_labeler()
        assert mod._labeler is not None

    def test_instantiation_with_custom_config(self):
        """Custom config parameters must be reflected in module state."""
        from perception.reconstruction.reconstruction_module import ReconstructionModule

        mod = ReconstructionModule(
            voxel_size=0.08,
            voxel_ttl=60.0,
            process_hz=10.0,
            min_points_to_publish=500,
            mask_dynamic=False,
            save_dir="/tmp/recon_test",
        )
        assert mod._process_hz == 10.0
        assert mod._min_points == 500
        assert mod._mask_dynamic is False
        assert mod._save_dir == "/tmp/recon_test"

    def test_ports(self):
        """All In/Out ports declared on the class must be registered."""
        from perception.reconstruction.reconstruction_module import ReconstructionModule
        from runtime.msgs.map import MapObservationFrame, SemanticLabelsFrame
        from runtime.msgs.nav import Odometry
        from runtime.msgs.semantic import SceneGraph
        from runtime.msgs.sensor import CameraIntrinsics, Image

        mod = ReconstructionModule()

        # -- Input ports --
        expected_in = {
            "color_image": Image,
            "depth_image": Image,
            "camera_info": CameraIntrinsics,
            "scene_graph": SceneGraph,
            "map_observation": MapObservationFrame,
            "odometry": Odometry,
        }
        assert len(mod._ports_in) == len(expected_in), (
            f"expected {len(expected_in)} In ports, got {list(mod._ports_in)}"
        )
        for name, expected_type in expected_in.items():
            assert name in mod._ports_in, f"missing In port: {name}"
            assert mod._ports_in[name].msg_type is expected_type, (
                f"In.{name}: expected {expected_type.__name__}, got {mod._ports_in[name].msg_type.__name__}"
            )

        # -- Output ports --
        expected_out = {
            "semantic_cloud": dict,
            "semantic_labels": SemanticLabelsFrame,
            "reconstruction_stats": dict,
        }
        assert len(mod._ports_out) == len(expected_out), (
            f"expected {len(expected_out)} Out ports, got {list(mod._ports_out)}"
        )
        for name, expected_type in expected_out.items():
            assert name in mod._ports_out, f"missing Out port: {name}"
            assert mod._ports_out[name].msg_type is expected_type, (
                f"Out.{name}: expected {expected_type.__name__}, got {mod._ports_out[name].msg_type.__name__}"
            )

    def test_lifecycle_setup_teardown(self):
        """setup() and teardown() must transition without error."""
        from perception.reconstruction.reconstruction_module import ReconstructionModule

        mod = ReconstructionModule()

        # setup subscribes ports and starts background thread
        mod.setup()
        assert mod._recon_active.is_set()
        assert mod._bg_thread is not None
        assert mod._bg_thread.is_alive()

        # teardown stops the background thread
        mod.teardown()
        assert not mod._recon_active.is_set()

    def test_save_ply_returns_no_data_message(self):
        """save_ply() with no data must return a failure dict, not crash."""
        from perception.reconstruction.reconstruction_module import ReconstructionModule

        mod = ReconstructionModule()
        result = mod.save_ply()
        assert isinstance(result, dict)
        assert result["success"] is False
        assert "no point cloud" in result["message"]

    def test_health_returns_module_info(self):
        """health() must return a dict with expected keys."""
        from perception.reconstruction.reconstruction_module import ReconstructionModule

        mod = ReconstructionModule()
        info = mod.health()
        assert isinstance(info, dict)
        assert "mesh_vertices" in info
        assert "frames_processed" in info
        assert "voxel_ttl_s" in info
        assert "mask_dynamic" in info

    def test_publish_cloud_returns_none_when_empty(self):
        """publish_cloud() must return None when below min_points threshold."""
        from perception.reconstruction.reconstruction_module import ReconstructionModule

        mod = ReconstructionModule()
        result = mod.publish_cloud()
        assert result is None

    def test_map_observation_publishes_exact_uint16_labels(self):
        from perception.reconstruction.reconstruction_module import ReconstructionModule
        from runtime.msgs.geometry import Pose, Transform, Vector3
        from runtime.msgs.map import MapObservationFrame
        from runtime.msgs.numpy_compat import np

        class _Labeler:
            def label_cloud(self, points):
                assert points.shape == (2, 3)
                return ["chair", "person"]

        mod = ReconstructionModule(semantic_scene_max_age_s=0.1)
        mod._labeler = _Labeler()
        mod._latest_sg = SimpleNamespace(frame_id="map", ts=10.0)
        observed = []
        mod.semantic_labels.subscribe(observed.append)
        transform = Transform(
            translation=Vector3(1.0, 2.0, 0.0),
            frame_id="map",
            child_frame_id="body",
            ts=10.0,
        )
        frame = MapObservationFrame(
            points=np.asarray([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dtype=np.float32),
            sequence=5,
            ts=10.0,
            frame_id="map",
            sensor_frame_id="body",
            sensor_origin=transform.translation,
            map_sensor_pose=Pose(transform.translation, transform.rotation),
            map_sensor_transform=transform,
        )

        mod._on_map_observation(frame)

        assert len(observed) == 1
        assert observed[0].labels.tolist() == [6, 0]
        assert observed[0].confidence.tolist() == pytest.approx([1.0, 0.0])
        assert observed[0].sequence == 5
        assert observed[0].frame_id == "body"


# =============================================================================
# DatasetRecorderModule
# =============================================================================


class TestDatasetRecorderModule:
    """Contract tests for DatasetRecorderModule (layer=3, keyframe recorder)."""

    def test_instantiation(self):
        """Creating a DatasetRecorderModule with default config should succeed."""
        from perception.reconstruction.dataset_recorder_module import (
            DatasetRecorderModule,
        )

        mod = DatasetRecorderModule()
        assert mod.recording is True
        assert mod._kf_dist == 0.15
        assert mod._kf_rot == 0.17
        assert mod._kf_time == 1.0
        assert mod._frame_count == 0
        assert mod._max_frames == 0  # unlimited

    def test_instantiation_with_custom_config(self):
        """Custom config parameters must be reflected in module state."""
        from perception.reconstruction.dataset_recorder_module import (
            DatasetRecorderModule,
        )

        mod = DatasetRecorderModule(
            keyframe_dist_m=0.5,
            keyframe_rot_rad=0.3,
            keyframe_time_s=2.0,
            max_depth_m=10.0,
            recording=False,
            max_frames=100,
        )
        assert mod._kf_dist == 0.5
        assert mod._kf_rot == 0.3
        assert mod._kf_time == 2.0
        assert mod._max_depth == 10.0
        assert mod.recording is False
        assert mod._max_frames == 100

    def test_ports(self):
        """All In/Out ports declared on the class must be registered."""
        from perception.reconstruction.dataset_recorder_module import (
            DatasetRecorderModule,
        )
        from runtime.msgs.nav import Odometry
        from runtime.msgs.sensor import CameraIntrinsics, Image

        mod = DatasetRecorderModule()

        # -- Input ports --
        expected_in = {
            "color_image": Image,
            "depth_image": Image,
            "camera_info": CameraIntrinsics,
            "odometry": Odometry,
        }
        assert len(mod._ports_in) == len(expected_in), (
            f"expected {len(expected_in)} In ports, got {list(mod._ports_in)}"
        )
        for name, expected_type in expected_in.items():
            assert name in mod._ports_in, f"missing In port: {name}"
            assert mod._ports_in[name].msg_type is expected_type, (
                f"In.{name}: expected {expected_type.__name__}, got {mod._ports_in[name].msg_type.__name__}"
            )

        # -- Output ports --
        assert "recorder_stats" in mod._ports_out
        assert mod._ports_out["recorder_stats"].msg_type is dict
        assert len(mod._ports_out) == 1, f"expected 1 Out port, got {list(mod._ports_out)}"

    def test_lifecycle_setup(self):
        """setup() must create directories and subscribe ports without error."""
        import tempfile

        from perception.reconstruction.dataset_recorder_module import (
            DatasetRecorderModule,
        )

        with tempfile.TemporaryDirectory() as tmpdir:
            mod = DatasetRecorderModule(save_dir=tmpdir)
            mod.setup()
            # setup creates images/ and depths/ subdirs
            assert (mod._save_dir / "images").exists()
            assert (mod._save_dir / "depths").exists()

    def test_recording_toggle(self):
        """start_recording() and stop_recording() must toggle the recording flag."""
        from perception.reconstruction.dataset_recorder_module import (
            DatasetRecorderModule,
        )

        mod = DatasetRecorderModule()
        assert mod.recording is True

        mod.stop_recording()
        assert mod.recording is False

        mod.start_recording()
        assert mod.recording is True

    def test_reset_clears_state(self):
        """reset() must clear frame count and create a new session directory."""
        import tempfile

        from perception.reconstruction.dataset_recorder_module import (
            DatasetRecorderModule,
        )

        with tempfile.TemporaryDirectory() as tmpdir:
            mod = DatasetRecorderModule(save_dir=tmpdir)
            mod.setup()
            mod._frame_count = 42
            mod._transforms_frames = [{"dummy": True}]

            mod.reset()
            assert mod._frame_count == 0
            assert len(mod._transforms_frames) == 0
            assert mod._last_kf_pos is None

    def test_health_returns_module_info(self):
        """health() must return a dict with expected keys."""
        from perception.reconstruction.dataset_recorder_module import (
            DatasetRecorderModule,
        )

        mod = DatasetRecorderModule()
        info = mod.health()
        assert isinstance(info, dict)
        assert "save_dir" in info
        assert "frames" in info
        assert "recording" in info


# =============================================================================
# ReconKeyframeExporterModule
# =============================================================================


class TestReconKeyframeExporterModule:
    """Contract tests for ReconKeyframeExporterModule (layer=3, server uploader)."""

    def test_instantiation(self):
        """Creating a ReconKeyframeExporterModule with default config should succeed."""
        from perception.reconstruction.keyframe_exporter_module import (
            ReconKeyframeExporterModule,
        )

        mod = ReconKeyframeExporterModule()
        assert mod._server_url == "http://localhost:7890"
        assert mod._kf_dist == 0.3
        assert mod._kf_rot == 0.26
        assert mod._kf_time == 2.0
        assert mod._batch_size == 5
        assert mod._enabled is True
        assert mod._total_exported == 0
        assert mod._upload_errors == 0

    def test_instantiation_with_custom_config(self):
        """Custom config parameters must be reflected in module state."""
        from perception.reconstruction.keyframe_exporter_module import (
            ReconKeyframeExporterModule,
        )

        mod = ReconKeyframeExporterModule(
            server_url="http://recon.example.com:8080",
            keyframe_dist_m=1.0,
            batch_size=10,
            enabled=False,
            jpeg_quality=95,
        )
        assert mod._server_url == "http://recon.example.com:8080"
        assert mod._kf_dist == 1.0
        assert mod._batch_size == 10
        assert mod._enabled is False
        assert mod._jpeg_quality == 95

    def test_ports(self):
        """All In/Out ports declared on the class must be registered."""
        from perception.reconstruction.keyframe_exporter_module import (
            ReconKeyframeExporterModule,
        )
        from runtime.msgs.nav import Odometry
        from runtime.msgs.sensor import CameraIntrinsics, Image

        mod = ReconKeyframeExporterModule()

        # -- Input ports --
        expected_in = {
            "color_image": Image,
            "depth_image": Image,
            "camera_info": CameraIntrinsics,
            "odometry": Odometry,
        }
        assert len(mod._ports_in) == len(expected_in), (
            f"expected {len(expected_in)} In ports, got {list(mod._ports_in)}"
        )
        for name, expected_type in expected_in.items():
            assert name in mod._ports_in, f"missing In port: {name}"
            assert mod._ports_in[name].msg_type is expected_type, (
                f"In.{name}: expected {expected_type.__name__}, got {mod._ports_in[name].msg_type.__name__}"
            )

        # -- Output ports --
        assert "export_stats" in mod._ports_out
        assert mod._ports_out["export_stats"].msg_type is dict
        assert len(mod._ports_out) == 1, f"expected 1 Out port, got {list(mod._ports_out)}"

    def test_lifecycle_setup_teardown(self):
        """setup() and teardown() must transition without error (no server needed)."""
        from perception.reconstruction.keyframe_exporter_module import (
            ReconKeyframeExporterModule,
        )

        mod = ReconKeyframeExporterModule(enabled=False)

        mod.setup()
        assert mod._recon_export_active.is_set()
        assert mod._upload_thread is not None
        assert mod._upload_thread.is_alive()

        mod.teardown()
        assert not mod._recon_export_active.is_set()

    def test_health_returns_module_info(self):
        """health() must return a dict with expected keys."""
        from perception.reconstruction.keyframe_exporter_module import (
            ReconKeyframeExporterModule,
        )

        mod = ReconKeyframeExporterModule()
        info = mod.health()
        assert isinstance(info, dict)
        assert "session_id" in info
        assert "frames_captured" in info
        assert "frames_exported" in info
        assert "server_url" in info
        assert "enabled" in info

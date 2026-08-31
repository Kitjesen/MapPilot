"""Tests for active service and reconstruction Module conversions.

Covers port declarations for the remaining Host service modules.
"""

from drivers.real.camera import dds_module as camera_module
from nav.services.goals import GoalService
from perception.reconstruction.reconstruction_module import ReconstructionModule
from runtime.msgs.semantic import SceneGraph
from runtime.msgs.sensor import CameraIntrinsics, Image


# ============================================================================
# Port declaration tests
# ============================================================================


class TestPortDeclarations:
    def test_goal_service_ports(self):
        m = GoalService()
        assert "goal_command" in m.ports_in
        assert "goal_request" in m.ports_in
        assert "cancel_request" in m.ports_in
        assert "goal_status" in m.ports_out
        assert m.ports_in["goal_command"].msg_type is str
        assert set(m.ports_out) == {"goal_status"}
        assert m.ports_out["goal_status"].msg_type is dict
        assert m.layer == 6

    def test_reconstruction_ports(self):
        m = ReconstructionModule()
        assert "color_image" in m.ports_in
        assert "depth_image" in m.ports_in
        assert "camera_info" in m.ports_in
        assert "scene_graph" in m.ports_in
        assert "odometry" in m.ports_in
        assert "semantic_cloud" in m.ports_out
        assert "reconstruction_stats" in m.ports_out
        assert m.ports_in["color_image"].msg_type is Image
        assert m.ports_in["scene_graph"].msg_type is SceneGraph
        assert m.ports_in["camera_info"].msg_type is CameraIntrinsics
        assert m.layer == 3


def test_camera_reader_starts_after_consumers_can_subscribe(monkeypatch, tmp_path):
    class Reader:
        def __init__(self, *_args, **_kwargs):
            pass

        def close(self):
            pass

    monkeypatch.setattr(camera_module, "ShmFrameReader", Reader)
    module = camera_module.DdsCameraModule(
        color_shm_path=str(tmp_path / "color.shm"),
        depth_shm_path=str(tmp_path / "depth.shm"),
        info_shm_path=str(tmp_path / "info.shm"),
    )
    received = []

    def publish_once():
        module.camera_info.publish(
            CameraIntrinsics(
                fx=600.0,
                fy=600.0,
                cx=320.0,
                cy=240.0,
                width=640,
                height=480,
            )
        )

    monkeypatch.setattr(module, "_read_loop", publish_once)

    module.setup()
    assert module._thread is None
    module.camera_info.subscribe(received.append)
    module.start()
    assert module._thread is not None
    module._thread.join(timeout=1.0)

    assert len(received) == 1
    module.stop()

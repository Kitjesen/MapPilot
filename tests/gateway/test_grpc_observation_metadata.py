"""Source calibration crosses the actual gRPC boundary without invented metadata."""

import socket
from contextlib import contextmanager
from types import SimpleNamespace

import grpc
import pytest

from gateway.rpc.generated import common_pb2 as common_pb
from gateway.rpc.generated import observations_pb2_grpc as observations_rpc
from gateway.rpc.server import start
from runtime.contracts import CAMERA_ROLE
from runtime.msgs.sensor import CameraIntrinsics


@contextmanager
def endpoint(source):
    gateway = SimpleNamespace(_all_modules={CAMERA_ROLE: source})
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        port = listener.getsockname()[1]
    server = start(gateway, f"127.0.0.1:{port}")
    with grpc.insecure_channel(f"127.0.0.1:{port}") as channel:
        try:
            yield observations_rpc.ObservationsStub(channel)
        finally:
            server.stop(0).wait(3)


@pytest.fixture(autouse=True)
def development_policy(monkeypatch):
    monkeypatch.delenv("LINGTU_MEDIA_REQUIRE_TOKEN", raising=False)
    monkeypatch.delenv("LINGTU_MEDIA_VIEWER_TOKEN", raising=False)


def test_calibration_preserves_source_optics_without_jpeg():
    from drivers.real.camera.module import OrbbecNativeCameraModule

    source = OrbbecNativeCameraModule(rotate=0, frame_id="source_color")
    source._publish_intrinsics({
        "width": 640, "height": 480, "fx": 410., "fy": 411., "cx": 319.5, "cy": 239.5,
        "depth_scale": .001, "dist_k1": .1, "dist_k2": .2,
        "dist_p1": .003, "dist_p2": .004, "dist_k3": .5,
    })
    with endpoint(source) as stub:
        reply = stub.CameraCalibration(common_pb.Empty(), timeout=2)
    assert (reply.width_px, reply.height_px) == (640, 480)
    assert (reply.fx_px, reply.fy_px, reply.cx_px, reply.cy_px) == (410., 411., 319.5, 239.5)
    assert reply.frame_id == "source_color"
    assert reply.distortion_model == "plumb_bob"
    assert list(reply.distortion) == [.1, .2, .003, .004, .5]


def test_default_dds_source_reads_shm_calibration_through_grpc(tmp_path):
    from drivers.real.camera.dds_module import DdsCameraModule
    from drivers.real.camera.shm import StreamKind
    from tests.drivers.test_camera_shm import write_committed_frame

    info_path = tmp_path / "camera_info.shm"
    source = DdsCameraModule(
        color_shm_path=str(tmp_path / "missing_color.shm"),
        depth_shm_path=str(tmp_path / "missing_depth.shm"),
        info_shm_path=str(info_path),
    )
    assert source.get_camera_intrinsics() is None
    write_committed_frame(info_path, stream_kind=StreamKind.INFO,
        encoding="camera_info", stride=0, payload=b"", source_generation=42,
        calibration=(500., 501., 1., 1.5, .001, .1, .2, .3, .4, .5))
    source.setup()
    try:
        assert source.poll_once() == 1
        with endpoint(source) as stub:
            reply = stub.CameraCalibration(common_pb.Empty(), timeout=2)
        assert (reply.width_px, reply.height_px) == (2, 2)
        assert (reply.fx_px, reply.fy_px, reply.cx_px, reply.cy_px) == (500., 501., 1., 1.5)
        assert reply.frame_id == "camera_link"
        assert reply.distortion_model == "plumb_bob"
        assert list(reply.distortion) == [.1, .2, .3, .4, .5]
        assert source.get_camera_intrinsics().source_generation == 42
    finally:
        source.stop()


@pytest.mark.parametrize("source", [None, SimpleNamespace(), SimpleNamespace(get_camera_intrinsics=lambda: None),
    SimpleNamespace(get_camera_intrinsics=lambda: CameraIntrinsics(width=640, height=480)),
    SimpleNamespace(get_camera_intrinsics=lambda: CameraIntrinsics(fx=float("nan"), fy=410, width=640, height=480)),
])
def test_missing_or_invalid_source_is_unavailable(source):
    with endpoint(source) as stub, pytest.raises(grpc.RpcError) as error:
        stub.CameraCalibration(common_pb.Empty(), timeout=2)
    assert error.value.code() == grpc.StatusCode.UNAVAILABLE


def test_calibration_obeys_media_authorization(monkeypatch):
    monkeypatch.setenv("LINGTU_MEDIA_REQUIRE_TOKEN", "1")
    monkeypatch.setenv("LINGTU_MEDIA_VIEWER_TOKEN", "viewer")
    source = SimpleNamespace(get_camera_intrinsics=lambda: CameraIntrinsics(fx=410, fy=410, width=640, height=480))
    with endpoint(source) as stub:
        with pytest.raises(grpc.RpcError) as error:
            stub.CameraCalibration(common_pb.Empty(), timeout=2)
        assert error.value.code() == grpc.StatusCode.UNAUTHENTICATED
        assert stub.CameraCalibration(common_pb.Empty(), metadata=(("authorization", "Bearer viewer"),), timeout=2).fx_px == 410


@pytest.mark.parametrize("rotation,expected", [
    (0, (640, 480, 410., 411., 100., 200.)),
    (90, (480, 640, 411., 410., 279., 100.)),
    (180, (640, 480, 410., 411., 539., 279.)),
    (270, (480, 640, 411., 410., 200., 539.)),
])
def test_source_intrinsics_rotate_pixel_centers(rotation, expected):
    from drivers.real.camera.module import OrbbecNativeCameraModule

    source = OrbbecNativeCameraModule(rotate=rotation, frame_id="source_color")
    source._publish_intrinsics({
        "width": 640, "height": 480, "fx": 410., "fy": 411., "cx": 100., "cy": 200.,
        "depth_scale": .001,
    })
    intrinsics = source.get_camera_intrinsics()
    assert (intrinsics.width, intrinsics.height, intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy) == expected
    assert intrinsics.frame_id == "source_color"

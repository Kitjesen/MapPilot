"""Localization feedback preserves measured fields and rejects unavailable caches."""

import socket
import threading
import time
from contextlib import contextmanager
from types import SimpleNamespace

import grpc
import pytest

from gateway.gateway_module import GatewayModule
from gateway.rpc import kinematics, services, telemetry
from gateway.rpc.generated import common_pb2 as common_pb
from gateway.rpc.generated import motion_pb2_grpc as motion_rpc
from gateway.rpc.server import start
from localization.adapters.status import _odometry_from_status_snapshot
from runtime.msgs import Odometry


class Abort(Exception):
    def __init__(self, code):
        self.code = code


def abort(code, _message):
    raise Abort(code)


@pytest.fixture
def context():
    return SimpleNamespace(abort=abort, add_callback=lambda _: True, is_active=lambda: True)


@pytest.fixture
def gateway():
    return SimpleNamespace(
        _state_lock=threading.Lock(),
        _odom={
            "x": 1.0, "y": 2.0, "z": 3.0, "orientation": [0.0, 0.0, 0.0, 1.0],
            "vx": 0.1, "vy": 0.2, "vz": 0.3, "wx": 0.4, "wy": 0.5, "wz": 0.6,
            "frame_id": "map", "child_frame_id": "body", "twist_available": True, "ts": 123.0,
        },
        _odom_timestamps=[time.time()], _localization_status={"state": "TRACKING"},
        _last_invalid_odometry=None,
    )


@contextmanager
def serve(gateway):
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        address = f"127.0.0.1:{listener.getsockname()[1]}"
    server = start(gateway, address)
    with grpc.insecure_channel(address) as channel:
        try:
            grpc.channel_ready_future(channel).result(timeout=3)
            yield motion_rpc.MotionStub(channel)
        finally:
            server.stop(0).wait(3)


def test_real_rpc_preserves_pose_six_axis_twist_and_unknown_clock(gateway):
    with serve(gateway) as client:
        velocity = client.Velocity(common_pb.Empty(), timeout=3)
        odometry = client.Odometry(common_pb.Empty(), timeout=3)
    assert (velocity.linear_m_s.x, velocity.linear_m_s.y, velocity.linear_m_s.z) == (0.1, 0.2, 0.3)
    assert (velocity.angular_rad_s.x, velocity.angular_rad_s.y, velocity.angular_rad_s.z) == (0.4, 0.5, 0.6)
    assert velocity.frame_id == "body"
    assert (odometry.position_m.x, odometry.position_m.y, odometry.position_m.z) == (1.0, 2.0, 3.0)
    assert odometry.orientation.w == 1.0
    assert odometry.parent_frame == "map" and odometry.child_frame == "body"
    assert odometry.linear_velocity_m_s == velocity.linear_m_s
    assert odometry.angular_velocity_rad_s == velocity.angular_rad_s
    for sample in (velocity, odometry):
        assert sample.source == "localization"
        assert sample.fresh and sample.HasField("received_age_ms")
        assert 0 <= sample.received_age_ms < 2000
        assert not sample.HasField("time")


def test_odometry_watch_uses_100ms_period(gateway, context, monkeypatch):
    waits = []
    stopped = SimpleNamespace(set=lambda: None, wait=lambda delay: waits.append(delay) or True)
    ticks = iter([0.0, 0.004])
    monkeypatch.setattr(telemetry, "time", SimpleNamespace(monotonic=lambda: next(ticks)))
    monkeypatch.setattr(telemetry, "threading", SimpleNamespace(Event=lambda: stopped))
    slots = threading.BoundedSemaphore(1)
    updates = list(services.Motion(gateway, slots).WatchOdometry(common_pb.Empty(), context))
    assert len(updates) == 1 and updates[0].source == "localization"
    assert waits == [pytest.approx(0.096)]
    assert slots.acquire(blocking=False)


@pytest.mark.parametrize("read", [kinematics.velocity, kinematics.odometry])
def test_stationary_feedback_keeps_real_zeroes(gateway, context, read):
    gateway._odom.update(dict.fromkeys(("vx", "vy", "vz", "wx", "wy", "wz"), 0.0))
    sample = read(gateway, context)
    linear = sample.linear_m_s if read is kinematics.velocity else sample.linear_velocity_m_s
    angular = sample.angular_rad_s if read is kinematics.velocity else sample.angular_velocity_rad_s
    assert linear.x == linear.y == linear.z == 0.0
    assert angular.x == angular.y == angular.z == 0.0


@pytest.mark.parametrize("read", [kinematics.velocity, kinematics.odometry])
@pytest.mark.parametrize("invalid", [
    "absent", "legacy", "missing_twist", "missing_axis", "nonfinite", "boolean", "stale",
    "future_receipt", "lost", "source_stalled", "quarantined", "wrong_frame", "wrong_child",
])
def test_unavailable_feedback_is_rejected(gateway, context, read, invalid):
    if invalid == "absent":
        gateway._odom = None
    elif invalid == "legacy":
        gateway._odom.pop("twist_available")
    elif invalid == "missing_twist":
        gateway._odom["twist_available"] = False
    elif invalid == "missing_axis":
        gateway._odom["wy"] = None
    elif invalid == "nonfinite":
        gateway._odom["wx"] = float("nan")
    elif invalid == "boolean":
        gateway._odom["vx"] = True
    elif invalid == "stale":
        gateway._odom_timestamps = [time.time() - 3]
    elif invalid == "future_receipt":
        gateway._odom_timestamps = [time.time() + 3]
    elif invalid == "lost":
        gateway._localization_status = {"state": "LOST"}
    elif invalid == "source_stalled":
        gateway._localization_status = {"source_data_stale": True}
    elif invalid == "quarantined":
        gateway._last_invalid_odometry = {"reason": "non_finite_odometry"}
    elif invalid == "wrong_frame":
        gateway._odom["frame_id"] = "odom"
    else:
        gateway._odom["child_frame_id"] = "unknown"
    with pytest.raises(Abort) as error:
        read(gateway, context)
    assert error.value.code == grpc.StatusCode.UNAVAILABLE


@pytest.mark.parametrize("orientation", [None, [0.0] * 4, [0.0, 0.0, 0.0, 2.0], [float("nan")] * 4])
def test_invalid_pose_orientation_is_unavailable(gateway, context, orientation):
    gateway._odom["orientation"] = orientation
    with pytest.raises(Abort) as error:
        kinematics.odometry(gateway, context)
    assert error.value.code == grpc.StatusCode.UNAVAILABLE


@pytest.mark.parametrize("method", ["WatchVelocity", "WatchOdometry"])
def test_real_stream_repeats_feedback_then_rejects_stale_source(gateway, method):
    with serve(gateway) as client:
        stream = getattr(client, method)(common_pb.Empty(), timeout=3)
        first, second = next(stream), next(stream)
        assert first.source == second.source == "localization"
        assert not first.HasField("time") and not second.HasField("time")
        with gateway._state_lock:
            gateway._localization_status = {"state": "LOST"}
        with pytest.raises(grpc.RpcError) as error:
            for _ in stream:
                pass
        assert error.value.code() == grpc.StatusCode.UNAVAILABLE


def native_odometry():
    return _odometry_from_status_snapshot({
        "has_odom": True, "stamp_s": 123.0,
        "map_odometry": {
            "frame_id": "map", "child_frame_id": "body",
            "pose": {"x": 1.0, "y": 2.0, "z": 3.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
            "twist": {"vx": 0.1, "vy": 0.2, "vz": 0.3, "wx": 0.4, "wy": 0.5, "wz": 0.6},
        },
    })


def test_native_snapshot_reaches_gateway_feedback_and_quarantines_bad_axis(context):
    robot = GatewayModule()
    robot.setup()
    odometry = native_odometry()
    robot._on_odometry(odometry)
    assert robot._odom["twist_available"] is True
    assert kinematics.odometry(robot, context).angular_velocity_rad_s.x == 0.4
    odometry.twist.angular.x = float("nan")
    robot._on_odometry(odometry)
    assert robot._last_invalid_odometry["fields"] == ["wx"]
    assert robot._odom["wx"] == 0.4
    with pytest.raises(Abort) as error:
        kinematics.velocity(robot, context)
    assert error.value.code == grpc.StatusCode.UNAVAILABLE
    robot._on_odometry(native_odometry())
    assert kinematics.velocity(robot, context).fresh


def test_missing_twist_keeps_legacy_zeroes_but_new_feedback_is_unavailable(context):
    robot = GatewayModule()
    robot.setup()
    odometry = native_odometry()
    odometry.twist = None
    robot._on_odometry(odometry)
    assert robot._odom["vx"] == robot._odom["vy"] == robot._odom["vz"] == robot._odom["wz"] == 0.0
    assert robot._odom["wx"] is None and robot._odom["wy"] is None
    assert robot._odom["twist_available"] is False
    with pytest.raises(Abort):
        kinematics.velocity(robot, context)


def test_default_dataclass_twist_does_not_claim_available_feedback(context):
    robot = GatewayModule()
    robot.setup()
    robot._on_odometry(Odometry(frame_id="map", child_frame_id="body"))
    assert robot._odom["twist_available"] is False
    with pytest.raises(Abort):
        kinematics.velocity(robot, context)


def test_feedback_streams_share_and_release_slots(gateway, context):
    slots = threading.BoundedSemaphore(1)
    service = services.Motion(gateway, slots)
    first = service.WatchVelocity(common_pb.Empty(), context)
    assert next(first).fresh
    with pytest.raises(Abort) as error:
        next(service.WatchOdometry(common_pb.Empty(), context))
    assert error.value.code == grpc.StatusCode.RESOURCE_EXHAUSTED
    first.close()
    second = service.WatchOdometry(common_pb.Empty(), context)
    assert next(second).fresh
    second.close()
    assert slots.acquire(blocking=False)

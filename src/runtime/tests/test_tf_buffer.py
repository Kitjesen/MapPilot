import math
import threading
import time
from types import SimpleNamespace

import pytest

from runtime.clock import Clock
from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Transform, Twist, Vector3
from runtime.msgs.nav import Odometry, Path
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import PointCloud2
from runtime.tf import (
    Buffer,
    StaticTransformBroadcaster,
    TFMessage,
    TfBus,
    TransformBroadcaster,
    TransformListener,
    tf_message_from_any,
    transform_from_stamped,
)


def test_listener_receives_latched_static_and_dynamic_tf() -> None:
    bus = TfBus()
    StaticTransformBroadcaster(bus).send_transform(
        Transform(
            translation=Vector3(0.2, 0.0, 0.0),
            frame_id="body",
            child_frame_id="lidar",
            ts=1.0,
        )
    )
    buffer = Buffer()
    listener = TransformListener(buffer, bus)
    try:
        TransformBroadcaster(bus).send_transform(
            Transform(
                translation=Vector3(1.0, 0.0, 0.0),
                frame_id="odom",
                child_frame_id="body",
                ts=2.0,
            )
        )

        assert buffer.can_transform("body", "lidar", time=100.0)
        assert buffer.lookup_transform("odom", "lidar", time=2.0).translation.x == pytest.approx(1.2)
    finally:
        listener.close()


def test_static_tf_replay_keeps_latest_per_edge() -> None:
    bus = TfBus()
    broadcaster = StaticTransformBroadcaster(bus)
    broadcaster.send_transform(
        Transform(
            translation=Vector3(1.0, 0.0, 0.0),
            frame_id="body",
            child_frame_id="camera",
            ts=1.0,
        )
    )
    broadcaster.send_transform(
        Transform(
            translation=Vector3(2.0, 0.0, 0.0),
            frame_id="body",
            child_frame_id="camera",
            ts=2.0,
        )
    )
    buffer = Buffer()
    listener = TransformListener(buffer, bus)
    try:
        transform = buffer.lookup_transform("body", "camera", time=100.0)
    finally:
        listener.close()

    assert transform.translation.x == pytest.approx(2.0)
    assert len(buffer.snapshot()["edges"]) == 1


def test_wait_for_transform_blocks_until_broadcast() -> None:
    bus = TfBus()
    buffer = Buffer()
    listener = TransformListener(buffer, bus)

    def publish_later() -> None:
        time.sleep(0.03)
        TransformBroadcaster(bus).send_transform(
            Transform(frame_id="odom", child_frame_id="body", ts=5.0)
        )

    thread = threading.Thread(target=publish_later)
    thread.start()
    try:
        assert buffer.wait_for_transform("odom", "body", time=5.0, timeout=1.0)
    finally:
        thread.join()
        listener.close()


def test_lookup_transform_full_uses_fixed_frame_times() -> None:
    buffer = Buffer()
    buffer.set_transform(
        Transform(
            translation=Vector3(20.0, 0.0, 0.0),
            frame_id="map",
            child_frame_id="odom",
            ts=20.0,
        )
    )
    buffer.set_transform(
        Transform(
            translation=Vector3(1.0, 0.0, 0.0),
            frame_id="odom",
            child_frame_id="body",
            ts=10.0,
        )
    )

    transform = buffer.lookup_transform_full(
        "map",
        20.0,
        "body",
        10.0,
        "odom",
    )

    assert transform.translation.x == pytest.approx(21.0)


def test_buffer_transforms_pose_and_cloud() -> None:
    buffer = Buffer()
    buffer.set_static_transform(
        Transform(
            translation=Vector3(1.0, 2.0, 0.0),
            rotation=Quaternion.from_yaw(math.pi / 2.0),
            frame_id="odom",
            child_frame_id="body",
            ts=1.0,
        )
    )
    pose = PoseStamped(Pose(1.0, 0.0, 0.0), ts=2.0, frame_id="body")
    cloud = PointCloud2(
        points=np.asarray([[1.0, 0.0, 0.0, 0.5]], dtype=np.float32),
        ts=2.0,
        frame_id="body",
    )

    out_pose = buffer.transform(pose, "odom")
    out_cloud = buffer.transform(cloud, "odom")

    assert out_pose.frame_id == "odom"
    assert out_pose.x == pytest.approx(1.0)
    assert out_pose.y == pytest.approx(3.0)
    np.testing.assert_allclose(
        out_cloud.points,
        np.asarray([[1.0, 3.0, 0.0, 0.5]], dtype=np.float32),
        atol=1e-6,
    )


def test_buffer_transforms_odometry_and_path() -> None:
    buffer = Buffer()
    buffer.set_static_transform(
        Transform(
            translation=Vector3(1.0, 0.0, 0.0),
            rotation=Quaternion.from_yaw(math.pi / 2.0),
            frame_id="map",
            child_frame_id="odom",
            ts=1.0,
        )
    )
    odom = Odometry(
        pose=Pose(2.0, 0.0, 0.0),
        twist=Twist(Vector3(1.0, 0.0, 0.0)),
        ts=2.0,
        frame_id="odom",
        child_frame_id="body",
    )
    path = Path(
        poses=[PoseStamped(Pose(2.0, 0.0, 0.0), ts=2.0, frame_id="odom")],
        ts=2.0,
        frame_id="odom",
    )

    out_odom = buffer.transform(odom, "map")
    out_path = buffer.transform(path, "map")

    assert out_odom.frame_id == "map"
    assert out_odom.child_frame_id == "body"
    assert out_odom.x == pytest.approx(1.0)
    assert out_odom.y == pytest.approx(2.0)
    assert out_odom.twist.linear.x == pytest.approx(1.0)
    assert out_odom.twist.linear.y == pytest.approx(0.0)
    assert out_path.frame_id == "map"
    assert out_path.poses[0].x == pytest.approx(1.0)
    assert out_path.poses[0].y == pytest.approx(2.0)


def test_all_frames_as_yaml_is_json_yaml_subset() -> None:
    buffer = Buffer()
    buffer.set_transform(Transform(frame_id="map", child_frame_id="odom", ts=5.0))

    text = buffer.all_frames_as_yaml()

    assert buffer.frame_exists("map")
    assert buffer.get_latest_common_time("map", "odom") == pytest.approx(5.0)
    assert "map" in buffer.all_frames_as_string()
    assert '"frames"' in text
    assert '"edges"' in text
    buffer.clear_dynamic()
    assert not buffer.can_transform("map", "odom")


def test_tf_message_round_trips_dict() -> None:
    msg = TFMessage(
        (
            Transform(
                translation=Vector3(1.0, 2.0, 3.0),
                frame_id="map",
                child_frame_id="odom",
                ts=9.0,
            ),
        )
    )

    decoded = TFMessage.from_dict(msg.to_dict())

    assert decoded.transforms[0].frame_id == "map"
    assert decoded.transforms[0].child_frame_id == "odom"
    assert decoded.transforms[0].translation.y == pytest.approx(2.0)
    assert decoded.transforms[0].ts == pytest.approx(9.0)


def test_duck_typed_tf_message_conversion() -> None:
    stamped = SimpleNamespace(
        header=SimpleNamespace(
            frame_id="map",
            stamp=SimpleNamespace(sec=12, nanosec=500_000_000),
        ),
        child_frame_id="odom",
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=1.0, y=2.0, z=3.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )
    msg = SimpleNamespace(transforms=[stamped])

    transform = transform_from_stamped(stamped)
    tf_msg = tf_message_from_any(msg)

    assert transform.frame_id == "map"
    assert transform.child_frame_id == "odom"
    assert transform.ts == pytest.approx(12.5)
    assert tf_msg.transforms[0].translation.z == pytest.approx(3.0)


def test_buffer_uses_sim_clock_now() -> None:
    sim_clock = Clock()
    sim_clock.set_sim_time(42.0)
    buffer = Buffer(clock=sim_clock)

    assert buffer.now() == pytest.approx(42.0)

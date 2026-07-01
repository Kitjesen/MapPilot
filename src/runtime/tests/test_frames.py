import math
from types import SimpleNamespace

import pytest

from runtime import FrameTree as ExportedFrameTree
from runtime.tf import ExtrapolationError, FrameTree, NoTransformError, UnknownFrameError
from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Transform, Vector3
from runtime.msgs.nav import Odometry
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import PointCloud2


def test_frame_tree_composes_sensor_mount_and_odometry() -> None:
    tree = FrameTree()
    tree.set_static_transform(
        Transform(
            translation=Vector3(1.0, 2.0, 0.0),
            rotation=Quaternion.from_yaw(math.pi / 2.0),
            frame_id="odom",
            child_frame_id="body",
        )
    )
    tree.set_static_transform(
        Transform(
            translation=Vector3(0.3, 0.0, 0.0),
            frame_id="body",
            child_frame_id="lidar",
        )
    )

    transform = tree.lookup("odom", "lidar")

    assert transform.frame_id == "odom"
    assert transform.child_frame_id == "lidar"
    assert transform.translation.x == pytest.approx(1.0)
    assert transform.translation.y == pytest.approx(2.3)
    assert transform.translation.z == pytest.approx(0.0)


def test_frame_tree_updates_odometry_and_round_trips_pose() -> None:
    tree = FrameTree()
    tree.update_odometry(
        Odometry(
            pose=Pose(
                position=Vector3(1.0, 2.0, 0.0),
                orientation=Quaternion.from_yaw(0.5),
            ),
            ts=12.0,
            frame_id="odom",
            child_frame_id="body",
        )
    )

    source = PoseStamped(
        pose=Pose(
            position=Vector3(0.5, -0.2, 0.0),
            orientation=Quaternion.from_yaw(0.25),
        ),
        ts=12.0,
        frame_id="body",
    )

    in_odom = tree.transform_pose_stamped(source, "odom")
    back_in_body = tree.transform_pose_stamped(in_odom, "body")

    assert in_odom.frame_id == "odom"
    assert in_odom.ts == pytest.approx(12.0)
    assert back_in_body.frame_id == "body"
    assert back_in_body.x == pytest.approx(source.x)
    assert back_in_body.y == pytest.approx(source.y)
    assert back_in_body.z == pytest.approx(source.z)
    assert back_in_body.yaw == pytest.approx(source.yaw)


def test_from_robot_config_registers_sensor_static_transforms_and_aliases() -> None:
    cfg = SimpleNamespace(
        lidar=SimpleNamespace(
            frame_id="livox_frame",
            offset_x=-0.1,
            offset_y=0.2,
            offset_z=0.3,
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
        ),
        camera=SimpleNamespace(
            position_x=0.4,
            position_y=0.0,
            position_z=0.5,
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
        ),
    )

    tree = FrameTree.from_robot_config(lambda: cfg)

    assert tree.can_transform("body", "livox_frame")
    assert tree.can_transform("body", "lidar_link")
    assert tree.can_transform("body", "base_link")
    assert tree.can_transform("body", "camera_link")

    lidar_to_body = tree.lookup("body", "lidar_link")
    camera_to_body = tree.lookup("body", "camera_link")
    assert lidar_to_body.translation.x == pytest.approx(-0.1)
    assert lidar_to_body.translation.y == pytest.approx(0.2)
    assert lidar_to_body.translation.z == pytest.approx(0.3)
    assert camera_to_body.translation.x == pytest.approx(0.4)
    assert camera_to_body.translation.z == pytest.approx(0.5)


def test_lookup_does_not_invent_unprovided_map_to_odom() -> None:
    tree = FrameTree()
    tree.set_alias("body", "base_link")

    with pytest.raises(UnknownFrameError):
        tree.lookup("map", "odom")


def test_lookup_raises_for_disconnected_known_frames() -> None:
    tree = FrameTree()
    tree.set_alias("body", "base_link")
    tree.set_alias("map", "odom")

    with pytest.raises(NoTransformError):
        tree.lookup("map", "body")


def test_replacing_child_parent_removes_stale_edge() -> None:
    tree = FrameTree()
    tree.set_transform(Transform(frame_id="body", child_frame_id="lidar"))
    tree.set_transform(Transform(frame_id="map", child_frame_id="lidar"))

    assert tree.lookup("map", "lidar").frame_id == "map"
    with pytest.raises(NoTransformError):
        tree.lookup("body", "lidar")


def test_transform_points_and_cloud_update_coordinates_and_frame() -> None:
    tree = FrameTree()
    tree.set_static_transform(
        Transform(
            translation=Vector3(1.0, 2.0, 3.0),
            frame_id="body",
            child_frame_id="lidar",
        )
    )
    points = np.array([[0.0, 0.0, 0.0, 7.0], [1.0, 0.0, 0.0, 8.0]], dtype=np.float32)

    converted_points = tree.transform_points(points, "body", "lidar")
    converted_cloud = tree.transform_cloud(PointCloud2(points=points, frame_id="lidar"), "body")

    np.testing.assert_allclose(
        converted_points,
        np.array([[1.0, 2.0, 3.0, 7.0], [2.0, 2.0, 3.0, 8.0]], dtype=np.float32),
    )
    np.testing.assert_allclose(converted_cloud.points, converted_points)
    assert converted_cloud.frame_id == "body"


def test_lookup_composes_rotation_translation_and_inverse() -> None:
    tree = FrameTree()
    tree.set_static_transform(
        Transform(
            translation=Vector3(10.0, 0.0, 0.0),
            rotation=Quaternion.from_yaw(math.pi / 2.0),
            frame_id="map",
            child_frame_id="odom",
        )
    )
    tree.set_static_transform(
        Transform(
            translation=Vector3(2.0, 0.0, 0.0),
            frame_id="odom",
            child_frame_id="body",
        )
    )

    in_map = tree.transform_point(Vector3(1.0, 0.0, 0.0), "map", "body")
    back_in_body = tree.transform_point(in_map, "body", "map")

    assert in_map.x == pytest.approx(10.0)
    assert in_map.y == pytest.approx(3.0)
    assert in_map.z == pytest.approx(0.0)
    assert back_in_body.x == pytest.approx(1.0)
    assert back_in_body.y == pytest.approx(0.0)
    assert back_in_body.z == pytest.approx(0.0)


def test_frame_tree_is_exported_from_core_package() -> None:
    assert ExportedFrameTree is FrameTree


def test_lookup_path_snapshot_parent_and_clear_dynamic() -> None:
    tree = FrameTree(cache_time_s=5.0)
    tree.set_transform(
        Transform(
            translation=Vector3(2.0, 0.0, 0.0),
            frame_id="odom",
            child_frame_id="body",
            ts=10.0,
        ),
        authority="slam",
    )
    tree.set_static_transform(
        Transform(
            translation=Vector3(0.2, 0.0, 0.0),
            frame_id="body",
            child_frame_id="lidar",
            ts=3.0,
        ),
        authority="robot_config",
    )

    assert tree.parent_of("body") == "odom"
    assert tree.lookup_path("odom", "lidar", ts=10.0) == ("lidar", "body", "odom")

    snapshot = tree.snapshot()
    edges = {(edge["parent"], edge["child"]): edge for edge in snapshot["edges"]}
    assert snapshot["cache_time_s"] == pytest.approx(5.0)
    assert edges[("odom", "body")]["authority"] == "slam"
    assert edges[("odom", "body")]["is_static"] is False
    assert edges[("body", "lidar")]["authority"] == "robot_config"
    assert edges[("body", "lidar")]["is_static"] is True

    tree.clear_dynamic()

    assert tree.parent_of("body") is None
    assert tree.can_transform("body", "lidar")
    with pytest.raises(NoTransformError):
        tree.lookup("odom", "body")


def test_static_transform_is_valid_for_any_lookup_time() -> None:
    tree = FrameTree()
    tree.set_static_transform(
        Transform(
            translation=Vector3(1.0, 0.0, 0.0),
            frame_id="body",
            child_frame_id="camera",
            ts=10.0,
        )
    )

    early = tree.lookup("body", "camera", ts=1.0)
    late = tree.lookup("body", "camera", ts=100.0)

    assert early.translation.x == pytest.approx(1.0)
    assert late.translation.x == pytest.approx(1.0)


def test_dynamic_lookup_interpolates_translation_and_rotation() -> None:
    tree = FrameTree()
    tree.set_transform(
        Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=Quaternion.from_yaw(0.0),
            frame_id="odom",
            child_frame_id="body",
            ts=1.0,
        )
    )
    tree.set_transform(
        Transform(
            translation=Vector3(10.0, 0.0, 0.0),
            rotation=Quaternion.from_yaw(math.pi),
            frame_id="odom",
            child_frame_id="body",
            ts=11.0,
        )
    )

    mid = tree.lookup("odom", "body", ts=6.0)

    assert mid.translation.x == pytest.approx(5.0)
    assert abs(mid.rotation.yaw) == pytest.approx(math.pi / 2.0)


def test_dynamic_lookup_rejects_time_outside_cache() -> None:
    tree = FrameTree()
    tree.set_transform(Transform(frame_id="odom", child_frame_id="body", ts=10.0))

    with pytest.raises(ExtrapolationError):
        tree.lookup("odom", "body", ts=9.0)


def test_dynamic_cache_prunes_old_samples() -> None:
    tree = FrameTree(cache_time_s=1.0)
    for ts in (1.0, 2.0, 4.0):
        tree.set_transform(
            Transform(
                translation=Vector3(ts, 0.0, 0.0),
                frame_id="odom",
                child_frame_id="body",
                ts=ts,
            )
        )

    latest = tree.lookup("odom", "body")

    assert latest.translation.x == pytest.approx(4.0)
    with pytest.raises(ExtrapolationError):
        tree.lookup("odom", "body", ts=1.0)

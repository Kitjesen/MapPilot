from __future__ import annotations

import sys
import types

import numpy as np

from runtime.msgs.geometry import (
    Pose,
    PoseStamped,
    Transform,
    Twist,
    TwistStamped,
    Vector3,
)
from runtime.msgs.nav import OccupancyGrid, Odometry, Path


def test_twist_ros_shape_is_cmd_vel_payload() -> None:
    cmd = Twist(
        linear=Vector3(0.4, -0.1, 0.0),
        angular=Vector3(0.0, 0.0, 0.2),
    )

    ros = cmd.to_ros_dict()

    assert set(ros) == {"linear", "angular"}
    assert ros["linear"] == {"x": 0.4, "y": -0.1, "z": 0.0}
    assert ros["angular"] == {"x": 0.0, "y": 0.0, "z": 0.2}


def test_twist_stamped_ros_shape_wraps_cmd_vel_with_header() -> None:
    msg = TwistStamped(
        linear=Vector3(0.5, 0.0, 0.0),
        angular=Vector3(0.0, 0.0, -0.3),
        ts=10.25,
        frame_id="body",
    )

    ros = msg.to_ros_dict()

    assert set(ros) == {"header", "twist"}
    assert ros["header"] == {
        "stamp": {"sec": 10, "nanosec": 250_000_000},
        "frame_id": "body",
    }
    assert ros["twist"]["linear"]["x"] == 0.5
    assert ros["twist"]["angular"]["z"] == -0.3


def test_pose_stamped_and_transform_ros_shapes() -> None:
    pose = PoseStamped(Pose(1.0, 2.0, 0.3), ts=2.5, frame_id="map")
    transform = Transform(
        translation=Vector3(1.0, 0.0, 0.2),
        frame_id="odom",
        child_frame_id="body",
        ts=3.75,
    )

    pose_ros = pose.to_ros_dict()
    transform_ros = transform.to_ros_dict()

    assert set(pose_ros) == {"header", "pose"}
    assert pose_ros["header"]["frame_id"] == "map"
    assert pose_ros["pose"]["position"] == {"x": 1.0, "y": 2.0, "z": 0.3}
    assert set(transform_ros) == {"header", "child_frame_id", "transform"}
    assert transform_ros["header"]["frame_id"] == "odom"
    assert transform_ros["child_frame_id"] == "body"
    assert transform_ros["transform"]["translation"]["z"] == 0.2


def test_odometry_ros_shape_contains_velocity_and_covariance_slots() -> None:
    odom = Odometry(
        pose=Pose(1.0, 2.0, 0.3),
        twist=Twist(
            linear=Vector3(0.7, 0.1, 0.0),
            angular=Vector3(0.0, 0.0, 0.4),
        ),
        ts=4.5,
        frame_id="odom",
        child_frame_id="body",
    )

    ros = odom.to_ros_dict()

    assert set(ros) == {"header", "child_frame_id", "pose", "twist"}
    assert ros["header"] == {
        "stamp": {"sec": 4, "nanosec": 500_000_000},
        "frame_id": "odom",
    }
    assert ros["child_frame_id"] == "body"
    assert ros["pose"]["pose"]["position"]["x"] == 1.0
    assert len(ros["pose"]["covariance"]) == 36
    assert ros["twist"]["twist"]["linear"] == {"x": 0.7, "y": 0.1, "z": 0.0}
    assert ros["twist"]["twist"]["angular"]["z"] == 0.4
    assert len(ros["twist"]["covariance"]) == 36


def test_path_and_occupancy_grid_ros_shapes() -> None:
    path = Path(
        poses=[PoseStamped(Pose(1.0, 2.0, 0.0), ts=1.0, frame_id="map")],
        ts=1.5,
        frame_id="map",
    )
    grid = OccupancyGrid(
        grid=np.array([[0, 100], [-1, 0]], dtype=np.int8),
        resolution=0.2,
        origin=Pose(1.0, 2.0, 0.0),
        ts=6.25,
        frame_id="map",
    )

    path_ros = path.to_ros_dict()
    grid_ros = grid.to_ros_dict()

    assert set(path_ros) == {"header", "poses"}
    assert path_ros["header"]["frame_id"] == "map"
    assert path_ros["poses"][0]["pose"]["position"]["y"] == 2.0
    assert set(grid_ros) == {"header", "info", "data"}
    assert grid_ros["info"]["resolution"] == 0.2
    assert grid_ros["info"]["width"] == 2
    assert grid_ros["info"]["height"] == 2
    assert grid_ros["info"]["origin"]["position"] == {"x": 1.0, "y": 2.0, "z": 0.0}
    assert grid_ros["data"] == [0, 100, -1, 0]


def test_ros2_nav_out_publishes_cmd_vel_as_twist_stamped(monkeypatch) -> None:
    class _Header:
        def __init__(self) -> None:
            self.frame_id = ""
            self.stamp = None

    class _Vec3:
        def __init__(self) -> None:
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

    class _Twist:
        def __init__(self) -> None:
            self.linear = _Vec3()
            self.angular = _Vec3()

    class _TwistStamped:
        def __init__(self) -> None:
            self.header = _Header()
            self.twist = _Twist()

    class _Pose:
        def __init__(self) -> None:
            self.position = _Vec3()
            self.orientation = _Vec3()
            self.orientation.w = 1.0

    class _PoseStamped:
        def __init__(self) -> None:
            self.header = _Header()
            self.pose = _Pose()

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.PoseStamped = _PoseStamped
    geometry_msgs_msg.TwistStamped = _TwistStamped
    monkeypatch.setitem(sys.modules, "geometry_msgs", geometry_msgs)
    monkeypatch.setitem(sys.modules, "geometry_msgs.msg", geometry_msgs_msg)

    from nav.adapters.ros2.nav.nav_out import ROS2NavOutModule

    adapter = ROS2NavOutModule(cmd_frame_id="body")
    msg = adapter._to_ros_twist(
        Twist(linear=Vector3(0.6, 0.1, 0.0), angular=Vector3(0.0, 0.0, 0.3))
    )

    assert isinstance(msg, _TwistStamped)
    assert msg.header.frame_id == "body"
    assert msg.twist.linear.x == 0.6
    assert msg.twist.linear.y == 0.1
    assert msg.twist.angular.z == 0.3

    waypoint = adapter._to_ros_waypoint(PoseStamped(Pose(1.0, 2.0, 0.3), frame_id="map"))
    assert isinstance(waypoint, _PoseStamped)
    assert waypoint.header.frame_id == "map"
    assert waypoint.pose.position.x == 1.0
    assert waypoint.pose.position.y == 2.0
    assert waypoint.pose.position.z == 0.3

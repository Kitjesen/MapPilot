from __future__ import annotations

import struct
import types

import numpy as np
import pytest

from drivers.sim.endpoint import SimEndpointDriverModule
from runtime.msgs.geometry import Twist, Vector3
from runtime.msgs.map import MapCloudFrame
from runtime.msgs.nav import Odometry
from runtime.msgs.sensor import CameraIntrinsics, Image, ImageFormat, PointCloud2
from runtime.registry import get

pytestmark = [pytest.mark.sim]


def _header(frame_id: str = "odom"):
    return types.SimpleNamespace(frame_id=frame_id)


def _odom_msg():
    return types.SimpleNamespace(
        header=_header("odom"),
        child_frame_id="base_link",
        pose=types.SimpleNamespace(
            pose=types.SimpleNamespace(
                position=types.SimpleNamespace(x=1.0, y=2.0, z=0.3),
                orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        ),
        twist=types.SimpleNamespace(
            twist=types.SimpleNamespace(
                linear=types.SimpleNamespace(x=0.4, y=0.1, z=0.0),
                angular=types.SimpleNamespace(x=0.0, y=0.0, z=0.2),
            )
        ),
    )


def _cloud_msg(*, frame_id: str = "odom", point_step: int = 16):
    points = np.array(
        [
            [1.0, 2.0, 3.0, 0.5],
            [4.0, 5.0, 6.0, 0.6],
        ],
        dtype=np.float32,
    )
    if point_step == 16:
        data = points.tobytes()
        fields = []
    else:
        data = b"".join(
            struct.pack(
                "<fffff",
                float(row[3]),
                float(row[0]),
                float(row[1]),
                float(row[2]),
                0.0,
            )
            for row in points
        )
        fields = [
            types.SimpleNamespace(name="intensity", offset=0),
            types.SimpleNamespace(name="x", offset=4),
            types.SimpleNamespace(name="y", offset=8),
            types.SimpleNamespace(name="z", offset=12),
        ]
    return types.SimpleNamespace(
        header=_header(frame_id),
        width=2,
        height=1,
        data=data,
        point_step=point_step,
        fields=fields,
    )


def _image_msg(*, encoding: str, data: bytes, width: int = 2, height: int = 2):
    return types.SimpleNamespace(
        header=_header("camera_link"),
        width=width,
        height=height,
        encoding=encoding,
        data=data,
    )


def _camera_info_msg():
    return types.SimpleNamespace(
        width=640,
        height=480,
        k=[500.0, 0.0, 320.0, 0.0, 510.0, 240.0, 0.0, 0.0, 1.0],
        d=[0.1, 0.2, 0.3, 0.4, 0.5],
    )


def test_sim_endpoint_driver_registers_without_ros2_runtime():
    assert get("driver", "sim_endpoint") is SimEndpointDriverModule
    assert get("driver_protocol", "sim_endpoint") is SimEndpointDriverModule


def test_sim_endpoint_driver_publishes_core_messages_from_ros_like_inputs():
    module = SimEndpointDriverModule()
    odom_messages: list[Odometry] = []
    lidar_messages: list[PointCloud2] = []
    map_messages: list[MapCloudFrame] = []
    module.odometry._add_callback(odom_messages.append)
    module.lidar_cloud._add_callback(lidar_messages.append)
    module.map_cloud_frame._add_callback(map_messages.append)

    module.publish_odometry_from_ros(_odom_msg())
    module.publish_registered_cloud_from_ros(_cloud_msg(frame_id="base_link"))
    module.publish_map_cloud_frame_from_ros(_cloud_msg(frame_id="odom", point_step=20))

    assert odom_messages[0].x == pytest.approx(1.0)
    assert odom_messages[0].vx == pytest.approx(0.4)
    assert odom_messages[0].wz == pytest.approx(0.2)
    assert odom_messages[0].frame_id == "odom"
    assert odom_messages[0].child_frame_id == "base_link"

    assert lidar_messages[0].frame_id == "base_link"
    assert map_messages[0].frame_id == "odom"
    assert map_messages[0].mode == "FULL"
    assert map_messages[0].source == "sim_endpoint"
    assert map_messages[0].sequence == 1
    np.testing.assert_allclose(
        lidar_messages[0].points[:, :3],
        np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32),
    )
    np.testing.assert_allclose(
        map_messages[0].points[:, :3],
        np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32),
    )


def test_sim_endpoint_driver_publishes_camera_messages_from_ros_like_inputs():
    module = SimEndpointDriverModule()
    color_messages: list[Image] = []
    depth_messages: list[Image] = []
    info_messages: list[CameraIntrinsics] = []
    module.camera_image._add_callback(color_messages.append)
    module.depth_image._add_callback(depth_messages.append)
    module.camera_info._add_callback(info_messages.append)

    color = np.arange(12, dtype=np.uint8).reshape(2, 2, 3)
    depth = np.array([[1.0, 2.0], [3.0, 4.0]], dtype=np.float32)
    module.publish_camera_image_from_ros(_image_msg(encoding="rgb8", data=color.tobytes()))
    module.publish_depth_image_from_ros(_image_msg(encoding="32FC1", data=depth.tobytes()))
    module.publish_camera_info_from_ros(_camera_info_msg())

    assert color_messages[0].format is ImageFormat.RGB
    assert color_messages[0].frame_id == "camera_link"
    np.testing.assert_array_equal(color_messages[0].data, color)
    assert depth_messages[0].format is ImageFormat.DEPTH_F32
    np.testing.assert_allclose(depth_messages[0].data, depth)
    assert info_messages[0].fx == pytest.approx(500.0)
    assert info_messages[0].fy == pytest.approx(510.0)
    assert info_messages[0].cx == pytest.approx(320.0)
    assert info_messages[0].cy == pytest.approx(240.0)
    assert info_messages[0].width == 640
    assert info_messages[0].height == 480


def test_sim_endpoint_driver_command_callback_and_stop_latch():
    commands: list[Twist] = []
    alive: list[bool] = []
    module = SimEndpointDriverModule(command_callback=commands.append)
    module.alive._add_callback(alive.append)

    module.setup()
    module.start()
    module.cmd_vel._deliver(Twist(Vector3(0.2, 0.1, 0.0), Vector3(0.0, 0.0, 0.3)))
    module.stop_signal._deliver(1)
    module.cmd_vel._deliver(Twist(Vector3(0.9, 0.0, 0.0), Vector3(0.0, 0.0, 0.9)))

    assert alive == [True]
    assert module.latest_command() == {"vx": 0.0, "vy": 0.0, "wz": 0.0}
    assert commands[0].linear.x == pytest.approx(0.2)
    assert commands[0].angular.z == pytest.approx(0.3)
    assert commands[1].is_zero()
    assert len(commands) == 2

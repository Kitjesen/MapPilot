from __future__ import annotations

from dataclasses import dataclass

from runtime.msgs.geometry import Twist, Vector3
from runtime.msgs.numpy_compat import np
from runtime.portable.contracts import PortableCommandFrame
from runtime.portable.topic_transport import PortableTopicTransport
from runtime.runtime_interface import TOPICS
from drivers.sim.mujoco.adapter import MujocoPortableAdapter, world_points_to_body_frame
from sim.engine.core.engine import CameraData, RobotState, VelocityCommand


class FakeEngine:
    def __init__(self) -> None:
        self.state = RobotState(
            position=np.asarray([1.0, 2.0, 0.5], dtype=np.float64),
            orientation=np.asarray([0.0, 0.0, 0.0, 1.0], dtype=np.float64),
            linear_velocity=np.asarray([0.1, 0.0, 0.0], dtype=np.float64),
            angular_velocity=np.asarray([0.0, 0.0, 0.2], dtype=np.float64),
            joint_positions=np.zeros(16, dtype=np.float64),
            joint_velocities=np.zeros(16, dtype=np.float64),
            imu_gyro=np.asarray([0.0, 0.1, 0.2], dtype=np.float64),
            imu_projected_gravity=np.asarray([0.0, 0.0, -1.0], dtype=np.float64),
        )
        self.last_cmd: VelocityCommand | None = None

    def get_robot_state(self) -> RobotState:
        return self.state

    def step(self, cmd: VelocityCommand | None = None) -> RobotState:
        self.last_cmd = cmd
        return self.state

    def set_cmd_vel(self, cmd: VelocityCommand) -> None:
        self.last_cmd = cmd

    def get_lidar_points(self):
        return np.asarray(
            [
                [1.0, 2.0, 0.5, 0.7],
                [2.0, 2.0, 0.5, 0.8],
            ],
            dtype=np.float32,
        )

    def get_camera_data(self, camera_name: str = "front_camera") -> CameraData:
        return CameraData(
            rgb=np.zeros((4, 6, 3), dtype=np.uint8),
            depth=np.ones((4, 6), dtype=np.float32),
            intrinsics=(10.0, 11.0, 3.0, 2.0),
            timestamp=1.0,
        )


def test_mujoco_adapter_maps_engine_state_to_sensor_frame() -> None:
    adapter = MujocoPortableAdapter(FakeEngine())

    frame = adapter.poll()

    assert frame.source == "mujoco"
    assert frame.odometry is not None
    assert frame.odometry.x == 1.0
    assert frame.imu is not None
    assert frame.imu.angular_velocity.y == 0.1
    assert frame.lidar_cloud is not None
    assert frame.lidar_cloud.points.shape == (2, 4)
    assert frame.map_cloud is not None
    assert frame.map_cloud.points.shape == (2, 4)
    assert frame.camera_image is not None
    assert frame.camera_image.data.shape == (4, 6, 3)
    assert frame.depth_image is not None
    assert frame.depth_image.data.shape == (4, 6)
    assert frame.camera_info is not None
    assert frame.camera_info.width == 6
    assert frame.camera_info.height == 4


def test_mujoco_adapter_applies_command_frame_to_engine() -> None:
    engine = FakeEngine()
    adapter = MujocoPortableAdapter(engine)
    command = PortableCommandFrame(
        cmd_vel=Twist(
            linear=Vector3(0.3, 0.1, 0.0),
            angular=Vector3(0.0, 0.0, 0.2),
        )
    )

    adapter.apply(command)

    assert engine.last_cmd is not None
    assert engine.last_cmd.linear_x == 0.3
    assert engine.last_cmd.linear_y == 0.1
    assert engine.last_cmd.angular_z == 0.2


def test_mujoco_adapter_publishes_to_transport_backend() -> None:
    adapter = MujocoPortableAdapter(FakeEngine())
    transport = PortableTopicTransport("local")
    received: dict[str, object] = {}
    transport.subscribe(TOPICS.odometry, lambda msg: received.setdefault("odometry", msg))
    transport.subscribe(TOPICS.map_cloud, lambda msg: received.setdefault("map_cloud", msg))
    transport.subscribe(TOPICS.camera_color, lambda msg: received.setdefault("camera_color", msg))
    transport.subscribe(TOPICS.camera_depth, lambda msg: received.setdefault("camera_depth", msg))
    transport.subscribe(TOPICS.camera_info, lambda msg: received.setdefault("camera_info", msg))

    try:
        adapter.publish(transport)

        assert received["odometry"] is not None
        assert received["map_cloud"] is not None
        assert received["camera_color"] is not None
        assert received["camera_depth"] is not None
        assert received["camera_info"] is not None
    finally:
        transport.close()


def test_world_points_to_body_frame_preserves_intensity_columns() -> None:
    points = np.asarray([[2.0, 2.0, 0.5, 0.9]], dtype=np.float32)

    body = world_points_to_body_frame(
        points,
        np.asarray([1.0, 2.0, 0.5], dtype=np.float64),
        np.asarray([0.0, 0.0, 0.0, 1.0], dtype=np.float64),
    )

    assert np.allclose(body[0], [1.0, 0.0, 0.0, 0.9])

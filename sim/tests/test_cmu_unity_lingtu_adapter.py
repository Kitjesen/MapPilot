from __future__ import annotations

import os
import sys
import types

import pytest
from runtime.tests.numpy_guard import import_numpy_or_skip

pytestmark = [pytest.mark.sim]

np = import_numpy_or_skip()

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from runtime.runtime_interface import TOPICS
from sim.engine.bridge import cmu_unity_lingtu_adapter as adapter


def _relay_map(relay_cmd_vel_to_sim: bool = True) -> dict[str, str]:
    return adapter.relay_contract(relay_cmd_vel_to_sim=relay_cmd_vel_to_sim)


def test_cmu_unity_adapter_maps_cmu_sensor_and_exploration_topics_to_lingtu():
    relays = _relay_map()

    assert relays[f"/state_estimation->{TOPICS.odometry}"] == "nav_msgs/msg/Odometry"
    assert (
        relays[f"/state_estimation_at_scan->{TOPICS.state_estimation_at_scan}"]
        == "nav_msgs/msg/Odometry"
    )
    assert f"/registered_scan->{TOPICS.registered_cloud}" not in relays
    assert relays[f"/registered_scan->{TOPICS.map_cloud}"] == "sensor_msgs/msg/PointCloud2"
    assert relays[f"/terrain_map->{TOPICS.terrain_map}"] == "sensor_msgs/msg/PointCloud2"
    assert (
        relays[f"/terrain_map_ext->{TOPICS.terrain_map_ext}"]
        == "sensor_msgs/msg/PointCloud2"
    )
    assert (
        relays["/way_point->/exploration/way_point"]
        == "geometry_msgs/msg/PointStamped"
    )
    assert relays["/global_path_full->/exploration/global_path_full"] == "nav_msgs/msg/Path"
    assert relays["/global_path->/exploration/global_path"] == "nav_msgs/msg/Path"
    assert relays["/local_path->/exploration/local_path"] == "nav_msgs/msg/Path"
    assert relays["/path->/exploration/cmu_local_planner_path"] == "nav_msgs/msg/Path"
    assert relays["/runtime->/exploration/runtime"] == "std_msgs/msg/Float32"


def test_cmu_unity_adapter_maps_lingtu_control_topics_back_to_cmu():
    relays = _relay_map()

    assert relays["/exploration/start->/start_exploration"] == "std_msgs/msg/Bool"
    assert relays["/nav/goal_point->/goal_point"] == "geometry_msgs/msg/PointStamped"
    assert (
        relays["/nav/navigation_boundary->/navigation_boundary"]
        == "geometry_msgs/msg/PolygonStamped"
    )


def test_cmu_unity_adapter_requires_explicit_cmd_vel_relay():
    without_cmd = _relay_map(relay_cmd_vel_to_sim=False)
    with_cmd = _relay_map(relay_cmd_vel_to_sim=True)

    assert "/nav/cmd_vel->/cmd_vel" not in without_cmd
    assert with_cmd["/nav/cmd_vel->/cmd_vel"] == "geometry_msgs/msg/TwistStamped"
    assert adapter.SIM_CMD_VEL_RELAY.target_topic == "/cmd_vel"
    assert "hardware ROS domain" in adapter.SIM_CMD_VEL_RELAY.note


def test_cmu_unity_adapter_required_relay_contract_excludes_optional_paths():
    required = adapter.required_relay_contract(relay_cmd_vel_to_sim=True)

    assert required[f"/state_estimation->{TOPICS.odometry}"] == "nav_msgs/msg/Odometry"
    assert required[f"/registered_scan->{TOPICS.map_cloud}"] == "sensor_msgs/msg/PointCloud2"
    assert required["/nav/cmd_vel->/cmd_vel"] == "geometry_msgs/msg/TwistStamped"
    assert "/global_path->/exploration/global_path" not in required
    assert "/local_path->/exploration/local_path" not in required
    assert "/path->/exploration/cmu_local_planner_path" not in required


def test_cmu_unity_adapter_can_replace_full_registered_scan_with_local_cloud():
    relays = {
        f"{spec.source_topic}->{spec.target_topic}": spec.msg_type
        for spec in adapter.build_relay_specs(
            relay_cmd_vel_to_sim=True,
            local_registered_cloud=True,
        )
    }

    assert f"/registered_scan->{TOPICS.registered_cloud}" not in relays
    assert relays[f"/registered_scan->{TOPICS.map_cloud}"] == "sensor_msgs/msg/PointCloud2"
    assert relays["/nav/cmd_vel->/cmd_vel"] == "geometry_msgs/msg/TwistStamped"


def test_cmu_unity_adapter_legacy_full_registered_relay_is_explicit_opt_in():
    relays = {
        f"{spec.source_topic}->{spec.target_topic}": spec.msg_type
        for spec in adapter.build_relay_specs(
            relay_cmd_vel_to_sim=True,
            local_registered_cloud=False,
        )
    }

    assert relays[f"/registered_scan->{TOPICS.registered_cloud}"] == "sensor_msgs/msg/PointCloud2"


def test_cmu_unity_adapter_recrops_cached_registered_scan_by_default():
    args = adapter._build_parser().parse_args([])

    assert args.local_scan_republish_hz == 2.0
    assert args.local_registered_cloud is True


def test_cmu_unity_adapter_filters_registered_scan_around_robot_xy():
    points = np.asarray(
        [
            [0.0, 0.0, 0.2],
            [1.0, 0.0, 0.3],
            [4.0, 0.0, 0.4],
            [float("nan"), 0.0, 0.5],
        ],
        dtype=np.float32,
    )

    filtered = adapter._filter_xyz_points_by_radius(
        points,
        center_xy=(0.0, 0.0),
        radius_m=1.5,
    )

    assert np.allclose(filtered, np.asarray([[0.0, 0.0, 0.2], [1.0, 0.0, 0.3]]))


def test_cmu_unity_adapter_filters_nav_cloud_to_obstacle_height_band():
    points = np.asarray(
        [
            [0.0, 0.0, 0.1],
            [1.0, 0.0, 0.5],
            [2.0, 0.0, 2.5],
        ],
        dtype=np.float32,
    )

    filtered = adapter._filter_xyz_points_by_height(points, z_min=0.3, z_max=2.0)

    assert np.allclose(filtered, np.asarray([[1.0, 0.0, 0.5]]))


def test_cmu_unity_adapter_transforms_global_scan_crop_to_body_frame():
    points = np.asarray([[0.0, 1.0, 0.4]], dtype=np.float32)

    body = adapter._global_xyz_points_to_body(
        points,
        pose_xyzyaw=(0.0, 0.0, 0.0, np.pi / 2.0),
    )

    assert body[0] == pytest.approx([1.0, 0.0, 0.4], abs=1e-6)


def test_cmu_unity_adapter_uses_reliable_qos_for_relayed_pointclouds(monkeypatch):
    class _Reliability:
        BEST_EFFORT = "best_effort"
        RELIABLE = "reliable"

    class _QoSProfile:
        def __init__(self, *, depth, reliability):
            self.depth = depth
            self.reliability = reliability

    qos_module = types.SimpleNamespace(
        QoSProfile=_QoSProfile,
        ReliabilityPolicy=_Reliability,
    )
    rclpy_module = types.ModuleType("rclpy")
    monkeypatch.setitem(sys.modules, "rclpy", rclpy_module)
    monkeypatch.setitem(sys.modules, "rclpy.qos", qos_module)

    qos = adapter._qos_for_msg("sensor_msgs/msg/PointCloud2")

    assert qos.depth == 5
    assert qos.reliability == _Reliability.RELIABLE

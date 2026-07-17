"""Round-trip coverage for the replay/DDS route backends.

This module complements ``test_dds_typed_wire_adapter.py`` (which covers
LivoxFrame / CameraInfo / Image / TFMessage) by exercising the remaining
typed DDS payloads that the codec round-trips (Odometry, PointCloud2,
OccupancyGrid, TwistStamped, PoseStamped, Text, Float32, Imu).

It also verifies the ``replay()`` route preset:

* every LCM channel binding declared for replay maps to a topic that owns a
  typed DDS spec (registry consistency);
* the ``_REPLAY_LCM_BINDINGS`` structure is well-formed (channel + type names);
* ``robot()`` routes those topics onto the DDS backend.

The replay LCM bindings are protocol-neutral declarations. The runtime does not
ship a live LCM transport backend (the historical LCM adapters were removed), so
any test that would require the ``lcm`` library is skipped gracefully while the
declarative structure is still validated.
"""

from __future__ import annotations

import importlib.util
import json

import pytest

from message.dds import from_dds_message, to_dds_message, topic_spec
from message.dds_types import (
    DDS_FinalVelocityCommand,
    DDS_Twist,
    DDS_Vector3,
)
from runtime.msgs.geometry import (
    Pose,
    PoseStamped,
    Quaternion,
    Twist,
    TwistStamped,
    Vector3,
)
from runtime.msgs.nav import OccupancyGrid, Odometry
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import Imu, PointCloud2
from runtime.route_contract import load_route_contract, validate_route_contract
from runtime.route_contract.routes import _REPLAY_LCM_BINDINGS, replay, robot
from runtime.runtime_interface import TOPICS

# ---------------------------------------------------------------------------
# Section A: DDS round-trip extended coverage
# ---------------------------------------------------------------------------


def test_odometry_round_trips_through_registered_dds_payload() -> None:
    odom = Odometry(
        pose=Pose(Vector3(1.0, 2.0, 3.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        twist=Twist(Vector3(0.5, 0.0, 0.0), Vector3(0.0, 0.0, 0.25)),
        ts=12.5,
        frame_id="odom",
        child_frame_id="base_link",
    )

    dds_msg = to_dds_message(TOPICS.odometry, odom)
    roundtrip = from_dds_message(TOPICS.odometry, dds_msg)

    assert topic_spec(TOPICS.odometry).cpp_type == "lingtu::dds::Odometry"
    assert dds_msg.child_frame_id == "base_link"
    assert dds_msg.pose.pose.position.x == pytest.approx(1.0)
    assert roundtrip.x == pytest.approx(1.0)
    assert roundtrip.y == pytest.approx(2.0)
    assert roundtrip.z == pytest.approx(3.0)
    assert roundtrip.vx == pytest.approx(0.5)
    assert roundtrip.wz == pytest.approx(0.25)
    assert roundtrip.frame_id == "odom"
    assert roundtrip.child_frame_id == "base_link"
    assert roundtrip.ts == pytest.approx(12.5)


def test_pointcloud2_round_trips_through_registered_dds_payload() -> None:
    points = np.asarray(
        [[1.0, 2.0, 3.0, 10.0], [4.0, 5.0, 6.0, 20.0]],
        dtype=np.float32,
    )
    cloud = PointCloud2(points=points, ts=5.25, frame_id="lidar")

    dds_msg = to_dds_message(TOPICS.map_cloud, cloud)
    roundtrip = from_dds_message(TOPICS.map_cloud, dds_msg)

    assert topic_spec(TOPICS.map_cloud).cpp_type == "lingtu::dds::PointCloud2"
    assert dds_msg.width == 2
    assert dds_msg.height == 1
    assert dds_msg.point_step == 16
    assert roundtrip.num_points == 2
    assert roundtrip.points[0].tolist() == pytest.approx([1.0, 2.0, 3.0, 10.0])
    assert roundtrip.points[1].tolist() == pytest.approx([4.0, 5.0, 6.0, 20.0])
    assert roundtrip.frame_id == "lidar"
    assert roundtrip.ts == pytest.approx(5.25)


def test_occupancy_grid_round_trips_through_registered_dds_payload() -> None:
    grid = OccupancyGrid(
        grid=[[0, 100], [-1, 50]],
        resolution=0.1,
        ts=7.5,
        frame_id="map",
    )

    dds_msg = to_dds_message(TOPICS.traversability, grid)
    roundtrip = from_dds_message(TOPICS.traversability, dds_msg)

    assert topic_spec(TOPICS.traversability).cpp_type == "lingtu::dds::OccupancyGrid"
    assert dds_msg.info.width == 2
    assert dds_msg.info.height == 2
    assert roundtrip["width"] == 2
    assert roundtrip["height"] == 2
    assert roundtrip["grid"] == [[0, 100], [-1, 50]]
    assert roundtrip["resolution"] == pytest.approx(0.1)
    assert roundtrip["frame_id"] == "map"
    assert roundtrip["origin"] == [0.0, 0.0]
    assert roundtrip["ts"] == pytest.approx(7.5)


def final_velocity_command(
    *,
    output_seq: int = 7,
) -> DDS_FinalVelocityCommand:
    return DDS_FinalVelocityCommand(
        host_boot_id="test-host-boot",
        producer_boot_id="test-endpoint-process",
        output_seq=output_seq,
        source_boottime_ns=3_000_000_000,
        source_wall_ns=1_700_000_000_000_000_000,
        twist=DDS_Twist(
            linear=DDS_Vector3(0.4, 0.0, 0.0),
            angular=DDS_Vector3(0.0, 0.0, -0.2),
        ),
    )


def test_final_velocity_round_trips_through_registered_dds_payload() -> None:
    dds_msg = to_dds_message(TOPICS.cmd_vel, final_velocity_command())
    roundtrip = from_dds_message(TOPICS.cmd_vel, dds_msg)

    assert (
        topic_spec(TOPICS.cmd_vel).cpp_type
        == "lingtu::dds::FinalVelocityCommand"
    )
    assert dds_msg.host_boot_id == "test-host-boot"
    assert dds_msg.producer_boot_id == "test-endpoint-process"
    assert dds_msg.output_seq == 7
    assert dds_msg.twist.linear.x == pytest.approx(0.4)
    assert dds_msg.twist.angular.z == pytest.approx(-0.2)
    assert isinstance(roundtrip, Twist)
    assert roundtrip.linear.x == pytest.approx(0.4)
    assert roundtrip.angular.z == pytest.approx(-0.2)


def test_raw_twist_cannot_publish_the_final_hardware_topic() -> None:
    twist = TwistStamped(
        Vector3(0.4, 0.0, 0.0),
        Vector3(0.0, 0.0, -0.2),
        ts=3.0,
        frame_id="base_link",
    )
    with pytest.raises(ValueError, match="FinalVelocityCommand envelope"):
        to_dds_message(TOPICS.cmd_vel, twist)


def test_pose_stamped_round_trips_through_registered_dds_payload() -> None:
    pose = PoseStamped(
        pose=Pose(Vector3(2.0, 3.0, 0.0), Quaternion.from_yaw(0.5)),
        ts=9.0,
        frame_id="map",
    )

    dds_msg = to_dds_message(TOPICS.goal_pose, pose)
    roundtrip = from_dds_message(TOPICS.goal_pose, dds_msg)

    assert topic_spec(TOPICS.goal_pose).cpp_type == "lingtu::dds::PoseStamped"
    assert dds_msg.header.frame_id == "map"
    assert roundtrip.x == pytest.approx(2.0)
    assert roundtrip.y == pytest.approx(3.0)
    assert roundtrip.frame_id == "map"
    assert roundtrip.ts == pytest.approx(9.0)
    assert roundtrip.yaw == pytest.approx(0.5)


def test_text_round_trips_through_registered_dds_payload() -> None:
    dds_msg = to_dds_message(TOPICS.cancel, "stop_now")
    roundtrip = from_dds_message(TOPICS.cancel, dds_msg)

    assert topic_spec(TOPICS.cancel).cpp_type == "lingtu::dds::Text"
    assert dds_msg.data == "stop_now"
    assert roundtrip == "stop_now"


def test_float32_round_trips_through_registered_dds_payload() -> None:
    dds_msg = to_dds_message(TOPICS.localization_quality, 0.5)
    roundtrip = from_dds_message(TOPICS.localization_quality, dds_msg)

    assert topic_spec(TOPICS.localization_quality).cpp_type == "lingtu::dds::Float32"
    assert float(dds_msg.data) == pytest.approx(0.5)
    # localization_quality has no dedicated decoder; from_dds_message returns the
    # DDS payload unchanged, so the scalar remains recoverable via ``.data``.
    assert float(getattr(roundtrip, "data", roundtrip)) == pytest.approx(0.5)


def test_imu_round_trips_through_registered_dds_payload() -> None:
    imu = Imu(
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        angular_velocity=Vector3(0.1, 0.2, 0.3),
        linear_acceleration=Vector3(0.0, 0.0, 9.8),
        ts=4.0,
        frame_id="imu_link",
    )

    dds_msg = to_dds_message(TOPICS.imu, imu)
    roundtrip = from_dds_message(TOPICS.imu, dds_msg)

    assert topic_spec(TOPICS.imu).cpp_type == "lingtu::dds::Imu"
    assert roundtrip.angular_velocity.x == pytest.approx(0.1)
    assert roundtrip.angular_velocity.y == pytest.approx(0.2)
    assert roundtrip.angular_velocity.z == pytest.approx(0.3)
    assert roundtrip.linear_acceleration.z == pytest.approx(9.8)
    assert roundtrip.frame_id == "imu_link"


# ---------------------------------------------------------------------------
# Section B: Replay backend
# ---------------------------------------------------------------------------

_LCM_AVAILABLE = importlib.util.find_spec("lcm") is not None

# Expected replay channel bindings (topic -> LCM channel / generated type name).
_EXPECTED_REPLAY_CHANNELS = {
    TOPICS.lidar_scan: ("LT_LIDAR_RAW_FRAME", "lingtu_lidar_frame_t"),
    TOPICS.imu: ("LT_IMU_RAW", "lingtu_imu_t"),
    TOPICS.odometry: ("LT_SLAM_ODOMETRY", "lingtu_odometry_t"),
    TOPICS.map_cloud: ("LT_SLAM_MAP_CLOUD", "lingtu_pointcloud2_t"),
    TOPICS.localization_health: (
        "LT_SLAM_LOCALIZATION_HEALTH",
        "lingtu_text_t",
    ),
    TOPICS.cmd_vel: ("LT_NAV_CMD_VEL", "lingtu_twist_stamped_t"),
}


def test_replay_lcm_bindings_have_valid_structure() -> None:
    assert set(_REPLAY_LCM_BINDINGS) == set(_EXPECTED_REPLAY_CHANNELS)

    seen_channels: set[str] = set()
    for topic, binding in _REPLAY_LCM_BINDINGS.items():
        channel = binding.get("channel")
        type_name = binding.get("type")
        # Channel names must be unique, non-empty, upper snake-case identifiers.
        assert isinstance(channel, str) and channel
        assert channel == channel.upper()
        assert channel not in seen_channels
        seen_channels.add(channel)
        # Type names must be non-empty and follow the ``lingtu_*_t`` convention.
        assert isinstance(type_name, str) and type_name
        assert type_name.startswith("lingtu_") and type_name.endswith("_t")
        assert (channel, type_name) == _EXPECTED_REPLAY_CHANNELS[topic]

    # cmd_vel is the only single-writer channel in the replay preset.
    assert _REPLAY_LCM_BINDINGS[TOPICS.cmd_vel].get("single_writer") is True


def test_replay_channels_reference_topics_with_typed_dds_specs() -> None:
    # Every replay LCM channel must correspond to a topic that also owns a typed
    # DDS spec, keeping the LCM and DDS registries consistent.
    for topic in _REPLAY_LCM_BINDINGS:
        spec = topic_spec(topic)
        assert spec is not None, f"replay topic {topic} lacks a typed DDS spec"
        assert spec.cpp_type.startswith("lingtu::dds::")


def test_replay_channel_message_types_round_trip_through_dds_codec() -> None:
    # The replay channels carry the same payload schemas as the DDS registry.
    # Verify the codec encodes/decodes each supported schema cleanly.
    imu = Imu(
        angular_velocity=Vector3(0.1, 0.2, 0.3),
        linear_acceleration=Vector3(0.0, 0.0, 9.8),
        ts=4.0,
        frame_id="imu_link",
    )
    assert from_dds_message(TOPICS.imu, to_dds_message(TOPICS.imu, imu)).frame_id == "imu_link"

    odom = Odometry(pose=Pose(Vector3(1.0, 2.0, 3.0)), ts=1.0, frame_id="odom")
    assert from_dds_message(TOPICS.odometry, to_dds_message(TOPICS.odometry, odom)).x == pytest.approx(1.0)

    cloud = PointCloud2(
        points=np.asarray([[1.0, 2.0, 3.0]], dtype=np.float32),
        ts=1.0,
        frame_id="lidar",
    )
    assert from_dds_message(TOPICS.map_cloud, to_dds_message(TOPICS.map_cloud, cloud)).num_points == 1

    command = final_velocity_command(output_seq=8)
    assert (
        from_dds_message(
            TOPICS.cmd_vel,
            to_dds_message(TOPICS.cmd_vel, command),
        ).linear.x
        == pytest.approx(0.4)
    )

    health = {"state": "healthy", "score": 0.9}
    health_dds = to_dds_message(TOPICS.localization_health, health)
    assert json.loads(health_dds.data) == health


@pytest.mark.skipif(
    not _LCM_AVAILABLE,
    reason="lcm library not installed; replay LCM bindings are declarative only",
)
def test_replay_lcm_library_backend_is_importable() -> None:
    # When the optional ``lcm`` library is present we assert only that it is a
    # usable module. The runtime does not register a live LCM transport backend,
    # so there is no generated ``lingtu_*_t`` codec to exercise here.
    import lcm

    assert hasattr(lcm, "LCM")


# ---------------------------------------------------------------------------
# Section C: route_contract replay preset validation
# ---------------------------------------------------------------------------


def test_replay_preset_declares_expected_lcm_channel_bindings() -> None:
    contract = load_route_contract(replay())

    assert validate_route_contract(contract) == []
    for topic, (channel, type_name) in _EXPECTED_REPLAY_CHANNELS.items():
        assert contract.route_for(topic) == "lcm"
        binding = contract.binding_for(topic)
        assert binding["channel"] == channel
        assert binding["type"] == type_name


def test_replay_preset_routes_declared_topics_to_lcm_backend() -> None:
    spec = replay()

    assert spec.name == "replay"
    assert spec.default == "local"
    for topic in _EXPECTED_REPLAY_CHANNELS:
        assert spec.backend_for(topic) == "lcm"
    # Topics without an explicit replay binding fall back to local delivery.
    assert spec.backend_for(TOPICS.global_path) == "local"


def test_robot_preset_uses_dds_backend_for_the_same_topics() -> None:
    contract = load_route_contract(robot())

    assert validate_route_contract(contract) == []
    assert contract.route.endpoint_contract == "thunder_field_dds_v1"
    # cmd_vel/odometry are DDS on the robot but LCM on replay.
    assert contract.route_for(TOPICS.cmd_vel) == "dds"
    assert contract.route_for(TOPICS.odometry) == "dds"

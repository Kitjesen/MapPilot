from __future__ import annotations

import json
import sys

import pytest

from runtime.adapters.lcm.contracts import THUNDER_FIELD_LCM_CONTRACT_NAME, binding_for_topic
from runtime.adapters.lcm.endpoint_codec import dumps_endpoint_message, loads_endpoint_message
from runtime.adapters.lcm.localization_adapter import LCMLocalizationAdapterModule
from runtime.blueprints.profile_builder import blueprint_for_resolved_profile
from runtime.blueprints.stacks.slam import slam
from runtime.msgs.geometry import Pose, PoseStamped, Twist, Vector3
from runtime.msgs.nav import Odometry, Path
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import Imu, PointCloud2
from runtime.profiles.resolver import resolve_profile_config
from runtime.runtime_interface import TOPICS, topic_default_frame_id


class _FakeLCMTransport:
    def __init__(self) -> None:
        self.callbacks = {}
        self.closed = False

    def subscribe(self, channel, callback):
        self.callbacks[channel] = callback
        return _FakeSubscription(channel)

    def emit(self, channel, payload) -> None:
        self.callbacks[channel](payload)

    def close(self) -> None:
        self.closed = True


class _FakeSubscription:
    def __init__(self, channel: str) -> None:
        self.channel = channel
        self.closed = False

    def close(self) -> None:
        self.closed = True


def test_endpoint_codec_preserves_point_cloud_payload() -> None:
    binding = binding_for_topic(THUNDER_FIELD_LCM_CONTRACT_NAME, TOPICS.map_cloud)
    cloud = PointCloud2(
        points=np.asarray([[1.0, 2.0, 3.0, 0.5], [4.0, 5.0, 6.0, 0.8]], dtype=np.float32),
        frame_id="map",
        ts=123.0,
    )

    decoded = loads_endpoint_message(binding, dumps_endpoint_message(binding, cloud))

    assert isinstance(decoded, PointCloud2)
    assert decoded.frame_id == "map"
    assert decoded.ts == 123.0
    assert decoded.points.shape == (2, 4)
    assert np.allclose(decoded.points, cloud.points)


def test_endpoint_codec_roundtrips_navigation_messages() -> None:
    odom_binding = binding_for_topic(THUNDER_FIELD_LCM_CONTRACT_NAME, TOPICS.odometry)
    path_binding = binding_for_topic(THUNDER_FIELD_LCM_CONTRACT_NAME, TOPICS.global_path)
    twist_binding = binding_for_topic(THUNDER_FIELD_LCM_CONTRACT_NAME, TOPICS.cmd_vel)

    odom = Odometry(pose=Pose(1.0, 2.0, 0.0), twist=Twist(Vector3(0.5, 0, 0)), ts=7.0)
    path = Path(poses=[PoseStamped(Pose(1.0, 2.0, 0.0), frame_id="map")], frame_id="map")
    twist = Twist(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.1))

    assert loads_endpoint_message(odom_binding, dumps_endpoint_message(odom_binding, odom)).x == 1.0
    assert len(loads_endpoint_message(path_binding, dumps_endpoint_message(path_binding, path))) == 1
    decoded_twist = loads_endpoint_message(
        twist_binding,
        dumps_endpoint_message(twist_binding, twist),
    )
    assert decoded_twist.linear.x == 0.2
    assert decoded_twist.angular.z == 0.1


def test_endpoint_codec_rejects_bad_envelope_contract() -> None:
    binding = binding_for_topic(THUNDER_FIELD_LCM_CONTRACT_NAME, TOPICS.cmd_vel)
    twist = Twist(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.1))

    envelope = json.loads(dumps_endpoint_message(binding, twist))
    envelope["schema_version"] = 999
    with pytest.raises(ValueError, match="schema_version mismatch"):
        loads_endpoint_message(binding, json.dumps(envelope).encode("utf-8"))

    envelope = json.loads(dumps_endpoint_message(binding, twist))
    envelope["frame_id"] = "map"
    with pytest.raises(ValueError, match="frame_id mismatch"):
        loads_endpoint_message(binding, json.dumps(envelope).encode("utf-8"))

    envelope = json.loads(dumps_endpoint_message(binding, twist))
    envelope.pop("ts")
    with pytest.raises(ValueError, match="timestamp missing"):
        loads_endpoint_message(binding, json.dumps(envelope).encode("utf-8"))


def test_lcm_localization_adapter_publishes_odometry_and_tracking_status() -> None:
    transport = _FakeLCMTransport()
    adapter = LCMLocalizationAdapterModule(transport=transport)
    odometry_seen = []
    status_seen = []
    alive_seen = []
    adapter.odometry.subscribe(odometry_seen.append)
    adapter.localization_status.subscribe(status_seen.append)
    adapter.alive.subscribe(alive_seen.append)
    adapter.setup()

    binding = binding_for_topic(THUNDER_FIELD_LCM_CONTRACT_NAME, TOPICS.odometry)
    odom = Odometry(pose=Pose(1.0, 2.0, 0.0), ts=42.0)
    transport.emit(binding.channel, dumps_endpoint_message(binding, odom))

    assert odometry_seen and odometry_seen[-1].x == 1.0
    assert alive_seen[-1] is True
    assert status_seen[-1]["backend"] == "lcm_endpoint"
    assert status_seen[-1]["endpoint_contract"] == THUNDER_FIELD_LCM_CONTRACT_NAME
    assert status_seen[-1]["source_topic"] == TOPICS.odometry
    assert adapter.health()["message_counts"][TOPICS.odometry] == 1

    adapter.stop()
    assert transport.closed is False


def test_lcm_localization_adapter_publishes_map_cloud_without_data_loss() -> None:
    transport = _FakeLCMTransport()
    adapter = LCMLocalizationAdapterModule(transport=transport)
    clouds = []
    adapter.map_cloud.subscribe(clouds.append)
    adapter.setup()

    binding = binding_for_topic(THUNDER_FIELD_LCM_CONTRACT_NAME, TOPICS.map_cloud)
    cloud = PointCloud2(points=np.asarray([[3.0, 2.0, 1.0]], dtype=np.float32), frame_id="map")
    transport.emit(binding.channel, dumps_endpoint_message(binding, cloud))

    assert clouds
    assert clouds[-1].frame_id == "map"
    assert np.allclose(clouds[-1].points, cloud.points)


def test_lcm_localization_adapter_accepts_health_payloads() -> None:
    transport = _FakeLCMTransport()
    adapter = LCMLocalizationAdapterModule(transport=transport)
    statuses = []
    qualities = []
    scenes = []
    adapter.localization_status.subscribe(statuses.append)
    adapter.localization_quality.subscribe(qualities.append)
    adapter.scene_mode.subscribe(scenes.append)
    adapter.setup()

    binding = binding_for_topic(THUNDER_FIELD_LCM_CONTRACT_NAME, TOPICS.localization_health)
    payload = {
        "state": "LOCKED",
        "quality": 0.82,
        "scene_mode": "outdoor",
    }
    transport.emit(binding.channel, dumps_endpoint_message(binding, payload))

    assert statuses[-1]["state"] == "LOCKED"
    assert statuses[-1]["health_source"] == THUNDER_FIELD_LCM_CONTRACT_NAME
    assert qualities[-1] == 0.82
    assert scenes[-1] == "outdoor"


def test_lcm_localization_adapter_updates_frame_tree_from_health_tf() -> None:
    transport = _FakeLCMTransport()
    adapter = LCMLocalizationAdapterModule(transport=transport)
    published_tf = []
    adapter.map_odom_tf.subscribe(published_tf.append)
    adapter.setup()

    binding = binding_for_topic(THUNDER_FIELD_LCM_CONTRACT_NAME, TOPICS.localization_health)
    payload = {
        "state": "LOCKED",
        "map_odom_tf": {
            "tx": 1.0,
            "ty": 2.0,
            "tz": 3.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.0,
            "qw": 1.0,
            "ts": 321.0,
            "valid": True,
        },
    }
    transport.emit(binding.channel, dumps_endpoint_message(binding, payload))

    transform = adapter._frame_tree.lookup(
        topic_default_frame_id(TOPICS.map_cloud),
        topic_default_frame_id(TOPICS.odometry),
        ts=321.0,
    )

    assert published_tf and published_tf[-1]["valid"] is True
    assert transform.ts == 321.0
    assert transform.translation.x == 1.0
    assert transform.translation.y == 2.0
    assert transform.translation.z == 3.0


def test_slam_stack_can_select_lcm_localization_adapter() -> None:
    bp = slam(
        "bridge",
        enable_visual_backup=False,
        localization_adapter="lcm_endpoint",
        endpoint_contract=THUNDER_FIELD_LCM_CONTRACT_NAME,
    )

    assert bp._entries[0].name == "SlamAdapterModule"
    assert bp._entries[0].module_cls is LCMLocalizationAdapterModule
    assert bp._entries[0].config["endpoint_contract"] == THUNDER_FIELD_LCM_CONTRACT_NAME


def test_thunder_field_product_blueprints_can_explicitly_use_lcm_localization_adapter() -> None:
    for profile in (
        "map",
        "nav",
        "explore",
        "tare_explore",
        "super_lio",
        "super_lio_relocation",
    ):
        config = resolve_profile_config(
            profile,
            localization_adapter="lcm_endpoint",
            endpoint_contract=THUNDER_FIELD_LCM_CONTRACT_NAME,
        )
        bp = blueprint_for_resolved_profile(profile, config)
        slam_entry = next(
            entry for entry in bp._entries if entry.name == "SlamAdapterModule"
        )

        assert config["_runtime_endpoint"] == "thunder_field"
        assert config["localization_adapter"] == "lcm_endpoint"
        assert config["endpoint_contract"] == THUNDER_FIELD_LCM_CONTRACT_NAME
        assert slam_entry.module_cls is LCMLocalizationAdapterModule
        assert not slam_entry.module_cls.__module__.startswith("runtime.adapters.ros2")
        assert slam_entry.config["endpoint_contract"] == THUNDER_FIELD_LCM_CONTRACT_NAME


def test_thunder_field_product_blueprints_do_not_import_ros2_slam_bridge() -> None:
    ros2_bridge_before = sys.modules.get("localization.adapters.ros2.slam_bridge")
    had_ros2_bridge = "localization.adapters.ros2.slam_bridge" in sys.modules
    try:
        sys.modules.pop("localization.adapters.ros2.slam_bridge", None)
        for profile in (
            "map",
            "nav",
            "explore",
            "tare_explore",
            "super_lio",
            "super_lio_relocation",
        ):
            config = resolve_profile_config(profile)
            bp = blueprint_for_resolved_profile(profile, config)
            slam_entry = next(
                entry for entry in bp._entries if entry.name == "SlamAdapterModule"
            )

            assert not slam_entry.module_cls.__module__.startswith("runtime.adapters.ros2")

        assert "localization.adapters.ros2.slam_bridge" not in sys.modules
    finally:
        if had_ros2_bridge:
            sys.modules["localization.adapters.ros2.slam_bridge"] = ros2_bridge_before
        else:
            sys.modules.pop("localization.adapters.ros2.slam_bridge", None)


def test_thunder_field_product_blueprints_do_not_contain_ros2_modules() -> None:
    for profile in (
        "nav",
        "explore",
        "tare_explore",
    ):
        config = resolve_profile_config(profile)
        bp = blueprint_for_resolved_profile(profile, config)
        ros2_like = [
            (entry.name, entry.module_cls.__module__)
            for entry in bp._entries
            if "ros2" in entry.name.lower()
            or "ros2" in entry.module_cls.__module__.lower()
            or "ros2" in entry.module_cls.__name__.lower()
        ]

        assert ros2_like == []

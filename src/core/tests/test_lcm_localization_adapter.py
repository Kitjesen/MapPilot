from __future__ import annotations

from compat.lcm.contracts import THUNDER_FIELD_LCM_CONTRACT_NAME, binding_for_topic
from compat.lcm.endpoint_codec import dumps_endpoint_message, loads_endpoint_message
from compat.lcm.localization_adapter import LCMLocalizationAdapterModule
from core.blueprints.profile_builder import blueprint_for_resolved_profile
from core.blueprints.stacks.slam import slam
from core.msgs.geometry import Pose, PoseStamped, Twist, Vector3
from core.msgs.nav import Odometry, Path
from core.msgs.numpy_compat import np
from core.msgs.sensor import Imu, PointCloud2
from core.runtime.resolver import resolve_profile_config
from core.runtime_interface import TOPICS


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


def test_slam_stack_can_select_lcm_localization_adapter() -> None:
    bp = slam(
        "bridge",
        enable_visual_backup=False,
        localization_adapter="lcm_endpoint",
        endpoint_contract=THUNDER_FIELD_LCM_CONTRACT_NAME,
    )

    assert bp._entries[0].name == "SlamBridgeModule"
    assert bp._entries[0].module_cls is LCMLocalizationAdapterModule
    assert bp._entries[0].config["endpoint_contract"] == THUNDER_FIELD_LCM_CONTRACT_NAME


def test_thunder_field_nav_blueprint_uses_lcm_localization_adapter() -> None:
    config = resolve_profile_config("nav")
    bp = blueprint_for_resolved_profile("nav", config)
    slam_entry = next(entry for entry in bp._entries if entry.name == "SlamBridgeModule")

    assert config["localization_adapter"] == "lcm_endpoint"
    assert config["_endpoint_contract"] == THUNDER_FIELD_LCM_CONTRACT_NAME
    assert slam_entry.module_cls is LCMLocalizationAdapterModule
    assert slam_entry.config["endpoint_contract"] == THUNDER_FIELD_LCM_CONTRACT_NAME

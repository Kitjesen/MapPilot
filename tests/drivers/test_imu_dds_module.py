from __future__ import annotations

from types import SimpleNamespace

from runtime.msgs.geometry import Quaternion, Vector3
from runtime.msgs.sensor import Imu
from runtime.runtime_interface import TOPICS


class FakeDDSReader:
    instances: list[FakeDDSReader] = []

    def __init__(self, domain_id=None):
        self.domain_id = domain_id
        self.subscriptions = []
        self.started = False
        FakeDDSReader.instances.append(self)

    def subscribe(self, ros2_topic, dds_type, callback, *, dds_topic=None):
        self.subscriptions.append((ros2_topic, dds_type, callback, dds_topic))

    def spin_background(self):
        self.started = True
        return True

    def stop(self):
        self.started = False


def _dds_imu():
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=10, nanosec=250_000_000),
            frame_id="imu_link",
        ),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        orientation_covariance=[0.0] * 9,
        angular_velocity=SimpleNamespace(x=1.0, y=2.0, z=3.0),
        angular_velocity_covariance=[0.0] * 9,
        linear_acceleration=SimpleNamespace(x=4.0, y=5.0, z=6.0),
        linear_acceleration_covariance=[0.0] * 9,
    )


def test_dds_imu_module_subscribes_native_raw_imu_topic_and_publishes_runtime_imu():
    from drivers.real.imu.dds_module import DdsImuModule

    module = DdsImuModule(reader_factory=FakeDDSReader, domain_id=7)
    samples = []
    module.imu.subscribe(samples.append)

    module.setup()
    module.start()

    reader = FakeDDSReader.instances[-1]
    assert reader.domain_id == 7
    assert reader.started is True
    assert len(reader.subscriptions) == 1
    topic, dds_type, callback, dds_topic = reader.subscriptions[0]
    assert topic == TOPICS.raw_imu
    assert dds_type is not None
    assert dds_topic == "rt/imu/raw"

    callback(_dds_imu())

    assert len(samples) == 1
    assert isinstance(samples[0], Imu)
    assert samples[0].frame_id == "imu_link"
    assert samples[0].ts == 10.25
    assert samples[0].angular_velocity == Vector3(1.0, 2.0, 3.0)
    assert samples[0].linear_acceleration == Vector3(4.0, 5.0, 6.0)
    assert samples[0].orientation == Quaternion.identity()
    health = module.health()
    assert health["role"] == "imu"
    assert health["backend"] == "dds"
    assert health["source"] == "rt/imu/raw"
    assert health["diagnostic_reader"] is True
    assert health["publishes_samples"] is True
    assert health["duplicates_lidar_imu_topic"] is True
    assert "field SLAM prefers lidar.imu" in health["note"]
    assert health["samples"] == 1


def test_livox_imu_backend_is_facade_for_lidar_imu_not_second_publisher():
    from drivers.real.imu.module import ImuModule

    module = ImuModule()
    samples = []
    alive = []
    module.imu.subscribe(samples.append)
    module.alive.subscribe(alive.append)

    module.setup()
    health = module.health()

    assert alive[-1] is False
    assert samples == []
    assert health["role"] == "imu"
    assert health["backend"] == "livox"
    assert health["source"] == "lidar.imu"
    assert health["status"] == "facade"
    assert health["publishes_samples"] is False
    assert health["samples"] == 0
    assert "field IMU is synchronized and published by the lidar role" in health["note"]


def test_livox_imu_facade_and_dds_reader_expose_different_risk_boundaries():
    from drivers.real.imu.dds_module import DdsImuModule
    from drivers.real.imu.module import ImuModule

    facade = ImuModule()
    dds = DdsImuModule(reader_factory=FakeDDSReader)

    facade.setup()
    dds.setup()
    dds.start()

    facade_health = facade.health()
    dds_health = dds.health()

    assert facade_health["backend"] == "livox"
    assert facade_health["source"] == "lidar.imu"
    assert facade_health["publishes_samples"] is False
    assert dds_health["backend"] == "dds"
    assert dds_health["source"] == "rt/imu/raw"
    assert dds_health["diagnostic_reader"] is True
    assert dds_health["duplicates_lidar_imu_topic"] is True


def test_imu_dds_backend_is_registered():
    import lingtu.plugin_seed as plugin_seed
    from drivers.real.imu.dds_module import DdsImuModule
    from runtime.blueprints.stacks.imu import imu
    from runtime.registry import get

    plugin_seed.seed_builtin_plugins(("imu",))

    assert get("imu", "dds") is DdsImuModule
    bp = imu(enabled=True, backend="dds")
    assert [entry.name for entry in bp._entries] == ["imu"]
    assert [entry.module_cls for entry in bp._entries] == [DdsImuModule]

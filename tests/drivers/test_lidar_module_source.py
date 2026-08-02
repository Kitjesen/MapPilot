from __future__ import annotations

import io
import sys
import threading
from pathlib import Path
from types import SimpleNamespace

from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import PointCloud2


class _FakeHealth:
    def to_dict(self) -> dict:
        return {"state": "connected", "source": "fake"}


class _FakeLidarSource:
    def __init__(self) -> None:
        self.ip = "192.0.2.10"
        self.state = SimpleNamespace(value="disconnected")
        self.connected = False
        self.disconnected = False
        self.cloud_callbacks = []
        self.raw_cloud_callbacks = []
        self.imu_callbacks = []

    @property
    def health(self) -> _FakeHealth:
        return _FakeHealth()

    def on_cloud(self, callback):
        self.cloud_callbacks.append(callback)
        return self

    def on_raw_cloud(self, callback):
        self.raw_cloud_callbacks.append(callback)
        return self

    def on_imu(self, callback):
        self.imu_callbacks.append(callback)
        return self

    def connect(self, ip=None):
        self.connected = True
        self.state.value = "connected"
        pts = np.asarray([[1.0, 2.0, 3.0, 0.5]], dtype=np.float32)
        raw = SimpleNamespace(
            points=np.asarray(
                [(1.0, 2.0, 3.0, 0.5, 42, 7, 3, 0)],
                dtype=[
                    ("x", "<f4"),
                    ("y", "<f4"),
                    ("z", "<f4"),
                    ("intensity", "<f4"),
                    ("offset_time_ns", "<u4"),
                    ("tag", "u1"),
                    ("line", "u1"),
                    ("flags", "<u2"),
                ],
            ),
        )
        for callback in self.raw_cloud_callbacks:
            callback(raw)
        for callback in self.cloud_callbacks:
            callback(pts)
        return self

    def disconnect(self) -> None:
        self.disconnected = True
        self.state.value = "disconnected"

    def emit_imu(self, imu) -> None:
        for callback in self.imu_callbacks:
            callback(imu)


class _FailingLidarSource(_FakeLidarSource):
    def connect(self, ip=None):
        raise RuntimeError("connect failed")


def test_lidar_module_accepts_injected_source_without_ros_or_dds() -> None:
    from drivers.real.lidar import LidarModule

    source = _FakeLidarSource()
    module = LidarModule(source=source)
    scans: list[PointCloud2] = []
    alive: list[bool] = []
    module.scan.subscribe(scans.append)
    module.alive.subscribe(alive.append)

    module.setup()
    module.start()

    assert source.connected is True
    assert alive == [True]
    assert len(scans) == 1
    assert isinstance(scans[0], PointCloud2)
    assert scans[0].points.shape[0] == 1
    assert module.health()["lidar"] == {"state": "connected", "source": "fake"}

    module.stop()

    assert source.disconnected is True
    assert alive[-1] is False


def test_lidar_module_publishes_lossless_raw_scan() -> None:
    from drivers.real.lidar import LidarModule

    source = _FakeLidarSource()
    module = LidarModule(source=source)
    raw_scans = []
    module.raw_scan.subscribe(raw_scans.append)

    module.setup()
    module.start()

    assert len(raw_scans) == 1
    assert raw_scans[0].points["offset_time_ns"][0] == 42
    assert raw_scans[0].points["tag"][0] == 7
    assert raw_scans[0].points["line"][0] == 3


def test_lidar_module_publishes_injected_source_imu() -> None:
    from drivers.real.lidar import LidarModule

    source = _FakeLidarSource()
    module = LidarModule(source=source)
    received = []
    imu_msg = object()
    module.imu.subscribe(received.append)

    module.setup()
    source.emit_imu(imu_msg)

    assert received == [imu_msg]


def test_lidar_module_reads_mock_sdk2_process_stream(monkeypatch, tmp_path) -> None:
    from drivers.real.lidar import LidarModule
    from drivers.real.lidar.impl.livox import sdk2_stream_source
    from drivers.real.lidar.impl.livox.sdk2_stream_source import (
        _HEADER,
        _IMU_PAYLOAD,
        _MAGIC,
        _RECORD_CLOUD,
        _RECORD_IMU,
        Sdk2Source,
    )

    points = np.zeros(2, dtype=sdk2_stream_source.POINT_DTYPE)
    points["x"] = [1.0, 2.0]
    points["y"] = [3.0, 4.0]
    points["z"] = [5.0, 6.0]
    points["intensity"] = [7.0, 8.0]
    points["offset_time_ns"] = [0, 1000]
    points["tag"] = [9, 10]
    points["line"] = [0, 1]
    cloud_payload = points.tobytes()
    stream = (
        _HEADER.pack(_MAGIC, _RECORD_CLOUD, 1_000_000_000, 11, len(points), len(cloud_payload))
        + cloud_payload
        + _HEADER.pack(_MAGIC, _RECORD_IMU, 1_001_000_000, 12, 1, _IMU_PAYLOAD.size)
        + _IMU_PAYLOAD.pack(0.1, 0.2, 0.3, 9.8, 0.0, -0.1)
    )

    class FakeProcess:
        stdout = io.BytesIO(stream)
        stderr = io.BytesIO()
        pid = 12345

        def poll(self):
            return 0

        def terminate(self):
            pass

        def wait(self, timeout=None):
            return 0

        def kill(self):
            pass

    monkeypatch.setattr(sdk2_stream_source.subprocess, "Popen", lambda *_args, **_kw: FakeProcess())

    fake_bin = tmp_path / "livox_sdk2_stream"
    fake_cfg = tmp_path / "MID360_config.json"
    fake_bin.write_text("", encoding="utf-8")
    fake_cfg.write_text("{}", encoding="utf-8")

    def factory(**kwargs):
        return Sdk2Source(
            ip=kwargs.get("ip"),
            executable=str(fake_bin),
            config_path=str(fake_cfg),
        )

    module = LidarModule(ip="192.0.2.30", source_factory=factory)
    scans = []
    raw_scans = []
    imus = []
    got_imu = threading.Event()
    module.scan.subscribe(scans.append)
    module.raw_scan.subscribe(raw_scans.append)
    module.imu.subscribe(lambda msg: (imus.append(msg), got_imu.set()))

    module.setup()
    try:
        module.start()
        assert got_imu.wait(1.0)
    finally:
        module.stop()

    assert len(scans) == 1
    assert isinstance(scans[0], PointCloud2)
    assert scans[0].points.shape == (2, 4)
    assert raw_scans[0].points["offset_time_ns"].tolist() == [0, 1000]
    assert raw_scans[0].points["tag"].tolist() == [9, 10]
    assert abs(imus[0].ts - 1.001) < 1e-9
    assert abs(imus[0].angular_velocity.z - 0.3) < 1e-6
    assert imus[0].orientation_covariance[0] == -1.0


def test_lidar_module_reads_software_livox_device_process(monkeypatch, tmp_path) -> None:
    from drivers.real.lidar import LidarModule
    from runtime.config import reset_config

    repo_root = Path(__file__).resolve().parents[2]
    simulator = repo_root / "scripts" / "diagnostics" / "livox_stream_sim.py"
    monkeypatch.setenv("LINGTU_LIVOX_SDK2_STREAM_BIN", str(simulator))
    monkeypatch.setenv("LINGTU_LIVOX_SIM_FRAMES", "2")
    monkeypatch.setenv("LINGTU_LIVOX_SIM_POINTS", "32")
    monkeypatch.setenv("LINGTU_LIVOX_SIM_HZ", "1000")
    monkeypatch.setenv("HOME", str(tmp_path))
    monkeypatch.setenv("USERPROFILE", str(tmp_path))
    reset_config()

    module = LidarModule(ip="192.0.2.30")
    scans = []
    raw_scans = []
    imus = []
    got_raw = threading.Event()
    got_imu = threading.Event()
    module.scan.subscribe(scans.append)
    module.raw_scan.subscribe(lambda msg: (raw_scans.append(msg), got_raw.set()))
    module.imu.subscribe(lambda msg: (imus.append(msg), got_imu.set()))

    module.setup()
    try:
        module.start()
        assert got_raw.wait(5.0)
        assert got_imu.wait(5.0)
    finally:
        module.stop()
        reset_config()

    assert scans[0].points.shape == (32, 4)
    assert raw_scans[0].point_count == 32
    assert raw_scans[0].points["line"].tolist()[:4] == [0, 1, 2, 3]
    assert imus[0].linear_acceleration.z > 9.0
    assert imus[0].orientation_covariance[0] == -1.0


def test_lidar_module_passes_ip_to_source_factory() -> None:
    from drivers.real.lidar import LidarModule

    calls = []
    source = _FakeLidarSource()

    def factory(**kwargs):
        calls.append(kwargs)
        return source

    module = LidarModule(
        ip="192.0.2.20",
        source_factory=factory,
    )

    assert module._lidar is source
    assert calls == [
        {
            "ip": "192.0.2.20",
        }
    ]


def test_lidar_module_start_failure_publishes_not_alive() -> None:
    from drivers.real.lidar import LidarModule

    module = LidarModule(source=_FailingLidarSource())
    alive: list[bool] = []
    module.alive.subscribe(alive.append)

    module.setup()
    module.start()

    assert alive == [False]


def test_create_lidar_source_builds_default_sdk2_source_without_connecting() -> None:
    from drivers.real.lidar.native.sdk import create_lidar_source

    sys.modules.pop("drivers.adapters.ros2.livox_driver", None)
    source = create_lidar_source(
        ip="192.0.2.30",
    )

    assert source.__class__.__name__ == "Sdk2Source"
    assert source.ip == "192.0.2.30"
    assert source.state.value == "disconnected"
    assert "drivers.adapters.ros2.livox_driver" not in sys.modules


def test_lidar_host_apis_do_not_expose_process_start_switch() -> None:
    import inspect

    from drivers.real.lidar import LidarModule
    from drivers.real.lidar.native.sdk import create_lidar_source
    from lingtu.assembly.stacks.lidar import lidar

    assert "start_driver" not in inspect.signature(LidarModule.__init__).parameters
    assert "start_driver" not in inspect.signature(create_lidar_source).parameters
    assert "start_driver" not in inspect.signature(lidar).parameters
    assert "scan_topic" not in inspect.signature(LidarModule.__init__).parameters
    assert "imu_topic" not in inspect.signature(LidarModule.__init__).parameters


def test_lidar_ros2_process_factory_is_removed() -> None:
    root = Path(__file__).resolve().parents[2]
    stack_source = (root / "src" / "lingtu" / "assembly" / "stacks" / "lidar.py").read_text(encoding="utf-8")
    module_source = (root / "src" / "drivers" / "real" / "lidar" / "module.py").read_text(encoding="utf-8")

    assert not (root / "src/drivers/real/lidar/impl/livox/native_factory.py").exists()
    assert not (root / "src/drivers/real/lidar/compat").exists()
    assert "from drivers.adapters.ros2" not in stack_source
    assert "from drivers.adapters.ros2" not in module_source


def test_lidar_module_rejects_ambiguous_source_configuration() -> None:
    from drivers.real.lidar import LidarModule

    source = _FakeLidarSource()
    try:
        LidarModule(source=source, source_factory=lambda **_: source)
    except ValueError as exc:
        assert "either source or source_factory" in str(exc)
    else:
        raise AssertionError("LidarModule accepted both source and source_factory")

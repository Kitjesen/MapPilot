from __future__ import annotations

import importlib
import sys
from types import SimpleNamespace

import pytest


class _FakeNative:
    def __init__(self, setup_error: Exception | None = None) -> None:
        self.setup_error = setup_error
        self.setup_called = False
        self.start_called = False

    def setup(self) -> None:
        self.setup_called = True
        if self.setup_error is not None:
            raise self.setup_error

    def start(self) -> None:
        self.start_called = True


def test_lidar_native_factory_does_not_import_ros2_livox_on_module_import() -> None:
    sys.modules.pop("runtime.native_module", None)

    import drivers.real.lidar.native_factory as native_factory

    importlib.reload(native_factory)

    assert "runtime.native_module" not in sys.modules


def test_lidar_start_native_driver_uses_driver_factory(monkeypatch) -> None:
    from drivers.real.lidar import lidar as lidar_module
    from drivers.real.lidar import native_factory

    native = _FakeNative()
    cfg = object()
    monkeypatch.setattr(native_factory, "livox_driver_process", lambda _cfg: native)

    lidar = lidar_module.Lidar()
    lidar._start_native_driver(cfg)

    assert lidar._native is native
    assert native.setup_called is True
    assert native.start_called is True


def test_lidar_connect_skips_native_driver_by_default(monkeypatch) -> None:
    from drivers.real.lidar import lidar as lidar_module

    calls = []
    lidar = lidar_module.Lidar(scan_topic="/test/scan", imu_topic="/test/imu")
    monkeypatch.setattr(
        lidar,
        "_build_config",
        lambda: SimpleNamespace(lidar=SimpleNamespace(lidar_ip="192.0.2.40")),
    )
    monkeypatch.setattr(lidar, "_start_native_driver", lambda _cfg: calls.append("native"))
    monkeypatch.setattr(lidar, "_start_dds_bridge", lambda: calls.append("dds"))

    lidar.connect()

    assert calls == ["dds"]
    assert lidar.is_connected is True


def test_lidar_connect_starts_native_driver_when_explicit(monkeypatch) -> None:
    from drivers.real.lidar import lidar as lidar_module

    calls = []
    lidar = lidar_module.Lidar(start_driver=True)
    monkeypatch.setattr(
        lidar,
        "_build_config",
        lambda: SimpleNamespace(lidar=SimpleNamespace(lidar_ip="192.0.2.40")),
    )
    monkeypatch.setattr(lidar, "_start_native_driver", lambda _cfg: calls.append("native"))
    monkeypatch.setattr(lidar, "_start_dds_bridge", lambda: calls.append("dds"))

    lidar.connect()

    assert calls == ["native", "dds"]
    assert lidar.is_connected is True


def test_lidar_start_native_driver_wraps_missing_binary(monkeypatch) -> None:
    from drivers.real.lidar import lidar as lidar_module
    from drivers.real.lidar import native_factory

    native = _FakeNative(setup_error=FileNotFoundError("missing livox"))
    monkeypatch.setattr(native_factory, "livox_driver_process", lambda _cfg: native)

    lidar = lidar_module.Lidar()
    with pytest.raises(RuntimeError, match="Build with: source /opt/ros/humble/setup.bash"):
        lidar._start_native_driver(object())

    assert native.setup_called is True
    assert native.start_called is False


def test_lidar_dds_bridge_subscribes_scan_and_imu_topics(monkeypatch) -> None:
    from drivers.real.lidar import _dds as livox_dds
    from drivers.real.lidar import lidar as lidar_module

    class FakeDDSReader:
        instances = []

        def __init__(self) -> None:
            self.subscriptions = []
            self.started = False
            FakeDDSReader.instances.append(self)

        def subscribe(self, topic, msg_type, callback) -> None:
            self.subscriptions.append((topic, msg_type, callback))

        def spin_background(self) -> None:
            self.started = True

    monkeypatch.setitem(sys.modules, "runtime.dds", SimpleNamespace(DDSReader=FakeDDSReader))
    monkeypatch.setattr(livox_dds, "HAS_LIVOX_IDL", True)
    monkeypatch.setattr(livox_dds, "LivoxCustomMsg", object())
    monkeypatch.setattr(livox_dds, "DDS_Imu", object())

    lidar = lidar_module.Lidar(scan_topic="/test/scan", imu_topic="/test/imu")
    lidar._start_dds_bridge()

    reader = FakeDDSReader.instances[0]
    assert lidar._dds is reader
    assert reader.started is True
    assert [sub[0] for sub in reader.subscriptions] == ["/test/scan", "/test/imu"]
    assert reader.subscriptions[0][2] == lidar._dds_adapter.on_scan
    assert reader.subscriptions[1][2] == lidar._dds_adapter.on_imu


def test_lidar_dds_bridge_skips_when_livox_idl_is_missing(monkeypatch) -> None:
    from drivers.real.lidar import _dds as livox_dds
    from drivers.real.lidar import lidar as lidar_module

    monkeypatch.setattr(livox_dds, "HAS_LIVOX_IDL", False)

    lidar = lidar_module.Lidar()
    lidar._start_dds_bridge()

    assert lidar._dds is None

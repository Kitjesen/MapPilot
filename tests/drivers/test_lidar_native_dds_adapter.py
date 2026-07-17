from __future__ import annotations

import sys
from types import SimpleNamespace

import pytest


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
    monkeypatch.setattr(lidar, "_start_dds_bridge", lambda: calls.append("dds") or True)

    lidar.connect()

    assert calls == ["dds"]
    assert lidar.is_connected is True


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

        def spin_background(self) -> bool:
            self.started = True
            return True

    monkeypatch.setitem(sys.modules, "runtime.adapters.dds.reader", SimpleNamespace(DDSReader=FakeDDSReader))
    monkeypatch.setattr(livox_dds, "HAS_LIVOX_IDL", True)
    monkeypatch.setattr(livox_dds, "LivoxCustomMsg", object())
    monkeypatch.setattr(livox_dds, "DDS_Imu", object())

    lidar = lidar_module.Lidar(scan_topic="/test/scan", imu_topic="/test/imu")
    assert lidar._start_dds_bridge() is True

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
    assert lidar._start_dds_bridge() is False

    assert lidar._dds is None


def test_lidar_connect_fails_when_dds_bridge_is_unavailable(monkeypatch) -> None:
    from drivers.real.lidar import _dds as livox_dds
    from drivers.real.lidar import lidar as lidar_module

    monkeypatch.setattr(livox_dds, "HAS_LIVOX_IDL", False)

    lidar = lidar_module.Lidar()
    with pytest.raises(RuntimeError, match="DDS bridge unavailable"):
        lidar.connect()

    assert lidar.is_connected is False

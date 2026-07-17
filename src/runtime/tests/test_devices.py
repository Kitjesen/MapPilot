"""Unit tests for the hardware device framework."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest
import yaml

_SRC = Path(__file__).resolve().parent.parent.parent
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from runtime.devices import (
    DeviceManager,
    DeviceStatus,
    DeviceType,
    Hw,
    decoder_registry,
    load_device_specs,
)
from runtime.devices.decoders.nmea import NmeaDecoder, _checksum
from runtime.devices.drivers.serial_nmea0183 import (
    SerialNmea0183Device,
    driver_registry,
)
from runtime.devices.spec import DeviceSpec
from runtime.msgs.gnss import GnssFixType
from runtime.registry import get

# ═══════════════════════════════════════════════════════════════════
# Spec loading
# ═══════════════════════════════════════════════════════════════════


class TestDeviceSpec:
    def test_load_devices_yaml(self):
        # Walk up to repo root: tests -> core -> src -> repo
        repo = _SRC.parent
        path = repo / "config" / "devices.yaml"
        if not path.exists():
            pytest.skip("config/devices.yaml not found")
        specs = load_device_specs(path)
        assert len(specs) >= 1
        ids = {s.id for s in specs}
        assert "wtrtk980_main" in ids

    def test_devices_yaml_is_inventory_not_sensor_wiring(self):
        repo = _SRC.parent
        path = repo / "config" / "devices.yaml"
        if not path.exists():
            pytest.skip("config/devices.yaml not found")
        text = path.read_text(encoding="utf-8")
        assert "physical hardware inventory" in text
        assert "must not decide runtime sensor wiring" in text
        assert "DeviceManager" not in text
        raw = yaml.safe_load(text)
        wiring_keys = {
            "publish_topics",
            "subscribe_topics",
            "topics",
            "wires",
            "inputs",
            "outputs",
            "enable_camera",
            "enable_lidar",
            "enable_imu",
            "enable_gnss",
            "camera_backend",
            "lidar_backend",
            "imu_backend",
            "gnss_backend",
        }

        for device in raw.get("devices", []):
            assert wiring_keys.isdisjoint(device), device["id"]
            config = device.get("config", {})
            if isinstance(config, dict):
                assert wiring_keys.isdisjoint(config), device["id"]

    def test_spec_from_dict(self):
        d = {
            "id": "test_gnss",
            "type": "gnss",
            "driver": "serial_nmea0183",
            "enabled": True,
            "udev_match": {"vendor_id": "1a86", "product_id": "7523", "port_path": "3-1"},
            "serial": {"device": "/dev/foo", "baud": 115200},
            "publish_topics": [{"name": "gnss_fix", "type": "GnssFix", "rate_hz": 1.0}],
        }
        spec = DeviceSpec.from_dict(d)
        assert spec.id == "test_gnss"
        assert spec.driver == "serial_nmea0183"
        assert spec.udev_match.vendor_id == "1a86"
        assert spec.serial["baud"] == 115200
        assert len(spec.publish_topics) == 1
        assert spec.publish_topics[0].rate_hz == 1.0


# ═══════════════════════════════════════════════════════════════════
# NMEA decoder
# ═══════════════════════════════════════════════════════════════════


class TestNmeaDecoder:
    def test_no_fix(self):
        d = NmeaDecoder()
        out = list(d.decode(b"$GNGGA,,,,,,0,,,,,,,,*78\r\n"))
        assert len(out) == 1
        assert out[0].fix_type == GnssFixType.NO_FIX

    def test_dgps_fix(self):
        d = NmeaDecoder()
        # Real-world DGPS sample with valid checksum
        s = b"$GPGGA,134658.00,5106.9792,N,11402.3003,W,2,09,1.0,1048.47,M,-16.27,M,08,AAAA*60\r\n"
        out = list(d.decode(s))
        assert len(out) == 1
        assert out[0].fix_type == GnssFixType.DGPS
        assert abs(out[0].lat - 51.116) < 0.01
        assert out[0].lon < 0  # West hemisphere

    def test_invalid_checksum_dropped(self):
        d = NmeaDecoder()
        out = list(d.decode(b"$GNGGA,,,,,,0,,,,,,,,*FF\r\n"))
        assert len(out) == 0

    def test_partial_line_buffered(self):
        d = NmeaDecoder()
        # Line split across two reads — should reconstruct
        out1 = list(d.decode(b"$GNGGA,,,,,"))
        out2 = list(d.decode(b",0,,,,,,,,*78\r\n"))
        assert len(out1) == 0
        assert len(out2) == 1

    def test_non_gga_ignored(self):
        d = NmeaDecoder()
        out = list(d.decode(b"$GNRMC,,V,,,,,,,,,,N,V*37\r\n"))
        assert len(out) == 0  # Only GGA decoded for now

    def test_checksum(self):
        assert _checksum("$GNGGA,,,,,,0,,,,,,,,*78")
        assert not _checksum("$GNGGA,,,,,,0,,,,,,,,*FF")


# ═══════════════════════════════════════════════════════════════════
# Serial NMEA driver
# ═══════════════════════════════════════════════════════════════════


class TestSerialNmea0183Device:
    def _spec(self, **over):
        base = dict(
            id="test_dev",
            type="gnss",
            driver="serial_nmea0183",
            enabled=True,
            serial={"device": "/dev/null", "baud": 115200},
        )
        base.update(over)
        return DeviceSpec(**base)

    def test_construct(self):
        d = SerialNmea0183Device(self._spec())
        assert d.id == "test_dev"
        assert d.type == DeviceType.GNSS
        assert d.status == DeviceStatus.OFFLINE

    def test_disabled_status(self):
        d = SerialNmea0183Device(self._spec(enabled=False))
        assert d.status == DeviceStatus.DISABLED

    def test_detect_missing_device(self):
        d = SerialNmea0183Device(self._spec(serial={"device": "/dev/nonexistent_xyz"}))
        assert d.detect() is False
        assert d.status == DeviceStatus.OFFLINE

    def test_open_fails_without_hardware(self):
        d = SerialNmea0183Device(self._spec(serial={"device": "/dev/nonexistent_xyz"}))
        assert d.open() is False


# ═══════════════════════════════════════════════════════════════════
# Decoder + driver registries
# ═══════════════════════════════════════════════════════════════════


class TestRegistries:
    def test_decoder_registered(self):
        assert "nmea0183" in decoder_registry()

    def test_driver_registered(self):
        assert "serial_nmea0183" in driver_registry()


# ═══════════════════════════════════════════════════════════════════
# Hw
# ═══════════════════════════════════════════════════════════════════


class _FakePort:
    def __init__(self):
        self.items = []

    def publish(self, v):
        self.items.append(v)


class TestHw:
    def test_hw_short_role_and_device_manager_alias_resolve_same_module(self):
        assert DeviceManager is Hw
        assert get("hw", "default") is Hw
        assert get("device_manager", "default") is Hw

    def test_blueprint_stack_exports_hw_not_device_manager(self):
        import runtime.blueprints.stacks as stacks
        from runtime.blueprints.stacks.system import device_manager, hw

        assert "hw" in stacks.__all__
        assert "device_manager" not in stacks.__all__
        assert [entry.name for entry in hw()._entries] == [entry.name for entry in device_manager()._entries]

    def _make(self, tmp_path):
        cfg = tmp_path / "devices.yaml"
        cfg.write_text(
            """
version: 1
devices:
  - id: test1
    type: gnss
    driver: serial_nmea0183
    enabled: true
    description: "test"
    serial:
      device: /dev/nonexistent_xyz
      baud: 115200
  - id: test2_disabled
    type: lidar
    driver: livox_sdk2
    enabled: false
    description: "disabled"
""",
            encoding="utf-8",
        )
        m = Hw(config_path=str(cfg), enable_hotplug=False)
        m.device_status = _FakePort()
        m.device_event = _FakePort()
        m.alive = _FakePort()
        return m

    def test_load_specs(self, tmp_path):
        m = self._make(tmp_path)
        m.setup()
        assert len(m._specs) == 2

    def test_skip_disabled(self, tmp_path):
        m = self._make(tmp_path)
        m.setup()
        # test1 created (driver exists, hardware missing); test2 skipped
        assert "test1" in m._devices
        assert "test2_disabled" not in m._devices

    def test_subscribe(self, tmp_path):
        m = self._make(tmp_path)
        m.setup()
        received = []
        m.subscribe("test1", received.append)
        # Manually trigger dispatch (simulating data from device)
        dispatcher = m._make_dispatch("test1")
        dispatcher("hello")
        assert received == ["hello"]

    def test_health(self, tmp_path):
        m = self._make(tmp_path)
        m.setup()
        h = m.health()
        assert h["role"] == "hw"
        assert h["backend"] == "inventory"
        assert h["sensor_streams"] is False
        assert h["scope"] == "hardware inventory/status only"
        assert h["spec_count"] == 2
        assert h["opened_count"] == 1  # test1 created (open failed but device exists)

import json
import subprocess

import pytest

from diagnostics.field import ble


@pytest.fixture
def controllers(monkeypatch, tmp_path):
    monkeypatch.setattr(ble, "CONTROLLERS_PATH", tmp_path)
    return tmp_path


def fake_commands(monkeypatch, management="", *, service="active", service_code=0, error=None):
    calls = []

    def run(command, **options):
        calls.append(command)
        assert options == {"capture_output": True, "text": True, "timeout": 5, "check": False}
        if command == ["systemctl", "is-active", "bluetooth"]:
            return subprocess.CompletedProcess(command, service_code, service, "")
        assert command == ["btmgmt", "info"]
        if isinstance(error, Exception):
            raise error
        if error:
            return subprocess.CompletedProcess(command, 1, "", error)
        return subprocess.CompletedProcess(command, 0, management, "")

    monkeypatch.setattr(ble.subprocess, "run", run)
    return calls


def test_running_bluez_without_controllers(monkeypatch, controllers):
    calls = fake_commands(monkeypatch)
    report = ble.collect()
    assert report == {
        "controller_scan": "ok", "controllers": [], "bluetooth_service": "active",
        "issues": [{"probe": "controllers", "reason": "no_controller"}],
    }
    assert calls == [["systemctl", "is-active", "bluetooth"]]


def test_capabilities_are_reported_per_controller_without_addresses(monkeypatch, controllers):
    (controllers / "hci0").mkdir()
    (controllers / "hci1").mkdir()
    fake_commands(monkeypatch, """hci0: Primary controller
    addr AA:BB:CC:DD:EE:FF version 11
    supported settings: powered br/edr le advertising secure-conn
    current settings: powered
hci1: Primary controller
    supported settings: powered br/edr
    current settings: br/edr
""")
    report = ble.collect()
    assert report["controllers"] == [
        {"name": "hci0", "le": "yes", "advertising": "yes", "powered": True},
        {"name": "hci1", "le": "no", "advertising": "no", "powered": False},
    ]
    assert report["issues"] == []
    assert "AA:BB:CC:DD:EE:FF" not in json.dumps(report)


@pytest.mark.parametrize("error, reason", [
    (FileNotFoundError(), "command_missing"),
    (PermissionError(), "permission_denied"),
    (subprocess.TimeoutExpired(["btmgmt", "info"], 5), "command_timeout"),
    ("Failed to read info: Operation not permitted", "permission_denied"),
    ("Failed to read info", "command_failed"),
])
def test_failed_capability_query_stays_unknown(monkeypatch, controllers, error, reason):
    (controllers / "hci0").mkdir()
    fake_commands(monkeypatch, error=error)
    report = ble.collect()
    assert report["controllers"] == [{"name": "hci0", "le": "unknown", "advertising": "unknown", "powered": "unknown"}]
    assert report["issues"] == [{"probe": "controller_capabilities", "reason": reason}]


def test_unrecognized_capabilities_stay_unknown(monkeypatch, controllers):
    (controllers / "hci0").mkdir()
    fake_commands(monkeypatch, "hci0: Primary controller\n    current settings: powered le advertising\n")
    report = ble.collect()
    assert report["controllers"] == [{"name": "hci0", "le": "unknown", "advertising": "unknown", "powered": True}]
    assert report["issues"] == [{"probe": "hci0", "reason": "capabilities_unconfirmed"}]


def test_missing_sysfs_is_distinct_from_zero_controllers(monkeypatch, controllers):
    monkeypatch.setattr(ble, "CONTROLLERS_PATH", controllers / "missing")
    fake_commands(monkeypatch, service="inactive", service_code=3)
    report = ble.collect()
    assert report["controller_scan"] == "unknown"
    assert report["controllers"] == []
    assert report["bluetooth_service"] == "inactive"
    assert report["issues"] == [{"probe": "controllers", "reason": "sysfs_unavailable"}]


def test_failed_service_probe_stays_unknown(monkeypatch, controllers):
    fake_commands(monkeypatch, service="", service_code=1)
    report = ble.collect()
    assert report["bluetooth_service"] == "unknown"
    assert report["issues"] == [
        {"probe": "controllers", "reason": "no_controller"},
        {"probe": "bluetooth_service", "reason": "state_unconfirmed"},
    ]


def test_supported_advertising_does_not_imply_powered(monkeypatch, controllers):
    (controllers / "hci0").mkdir()
    fake_commands(monkeypatch, "hci0: Primary controller\n    supported settings: le advertising\n    current settings:\n")
    report = ble.collect()
    assert report["controllers"] == [{"name": "hci0", "le": "yes", "advertising": "yes", "powered": False}]
    assert report["issues"] == []


def test_missing_power_state_stays_unknown(monkeypatch, controllers):
    (controllers / "hci0").mkdir()
    fake_commands(monkeypatch, "hci0: Primary controller\n    supported settings: le advertising\n")
    report = ble.collect()
    assert report["controllers"] == [{"name": "hci0", "le": "yes", "advertising": "yes", "powered": "unknown"}]
    assert report["issues"] == [{"probe": "hci0", "reason": "power_state_unconfirmed"}]

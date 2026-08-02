from __future__ import annotations

import struct

from gateway import ble_peripheral


def _status(packet: bytes) -> tuple[int, int, int]:
    parsed = ble_peripheral.parse_packet(packet)
    assert parsed is not None
    cmd, payload = parsed
    _battery, mode, error_code = struct.unpack_from("<BBH", payload)
    return cmd, mode, error_code


def test_estop_rejection_preserves_mode_and_reports_error(monkeypatch):
    state = ble_peripheral.RobotState()
    state.mode = ble_peripheral.MODE_MANUAL
    monkeypatch.setattr(state, "_send_estop_http", lambda: False)
    monkeypatch.setattr(ble_peripheral, "robot_state", state)
    notifications: list[bytes] = []

    ble_peripheral.handle_command(
        ble_peripheral.build_packet(ble_peripheral.CMD_ESTOP),
        notifications.append,
    )

    assert state.mode == ble_peripheral.MODE_MANUAL
    assert _status(notifications[0]) == (
        ble_peripheral.CMD_STATUS_RESP,
        ble_peripheral.MODE_MANUAL,
        ble_peripheral.ERROR_COMMAND_REJECTED,
    )


def test_mode_switch_without_product_control_is_fail_closed(monkeypatch):
    state = ble_peripheral.RobotState()
    state.mode = ble_peripheral.MODE_MANUAL
    monkeypatch.setattr(ble_peripheral, "robot_state", state)
    notifications: list[bytes] = []

    ble_peripheral.handle_command(
        ble_peripheral.build_packet(
            ble_peripheral.CMD_MODE_SWITCH,
            bytes([ble_peripheral.MODE_AUTONOMOUS]),
        ),
        notifications.append,
    )

    assert state.mode == ble_peripheral.MODE_MANUAL
    assert _status(notifications[0]) == (
        ble_peripheral.CMD_STATUS_RESP,
        ble_peripheral.MODE_MANUAL,
        ble_peripheral.ERROR_MODE_SWITCH_UNAVAILABLE,
    )


def test_estop_requires_strict_native_gateway_ack(monkeypatch):
    import urllib.request

    class GatewayResponse:
        status = 200

        def __init__(self, body: bytes):
            self.body = body

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def read(self) -> bytes:
            return self.body

    responses = iter(
        [
            GatewayResponse(
                b'{"ok":true,"status":"stopped","stage":"local_published",'
                b'"dds":false,"native_control":"local_compat",'
                b'"command":{"accepted":true}}'
            ),
            GatewayResponse(
                b'{"ok":true,"status":"stopped","stage":"native_acknowledged",'
                b'"dds":true,"native_control":"estop",'
                b'"command":{"accepted":true}}'
            ),
        ]
    )

    def urlopen(_request, timeout):
        assert timeout == 2.0
        return next(responses)

    monkeypatch.setattr(urllib.request, "urlopen", urlopen)
    state = ble_peripheral.RobotState()
    state.mode = ble_peripheral.MODE_MANUAL

    assert state.emergency_stop() is False
    assert state.mode == ble_peripheral.MODE_MANUAL
    assert state.error_code == ble_peripheral.ERROR_COMMAND_REJECTED

    assert state.emergency_stop() is True
    assert state.mode == ble_peripheral.MODE_ESTOP
    assert state.error_code == ble_peripheral.ERROR_NONE


def test_wifi_ack_requires_valid_payload_and_zero_nmcli_returncode(monkeypatch):
    class RunResult:
        def __init__(self, returncode: int):
            self.returncode = returncode
            self.stdout = ""
            self.stderr = "nmcli failed" if returncode else ""

    results = iter([RunResult(10), RunResult(0)])
    calls: list[list[str]] = []

    def run(command, **_kwargs):
        calls.append(command)
        return next(results)

    state = ble_peripheral.RobotState()
    monkeypatch.setattr(ble_peripheral, "robot_state", state)
    monkeypatch.setattr(ble_peripheral.subprocess, "run", run)

    invalid_notifications: list[bytes] = []
    ble_peripheral.handle_command(
        ble_peripheral.build_packet(ble_peripheral.CMD_WIFI_CONFIG, b"\x04abc"),
        invalid_notifications.append,
    )
    assert calls == []
    assert _status(invalid_notifications[0]) == (
        ble_peripheral.CMD_STATUS_RESP,
        ble_peripheral.MODE_IDLE,
        ble_peripheral.ERROR_INVALID_WIFI_PAYLOAD,
    )

    ssid = b"field-net"
    password = b"password"
    payload = bytes([len(ssid)]) + ssid + bytes([len(password)]) + password

    failed_notifications: list[bytes] = []
    ble_peripheral.handle_command(
        ble_peripheral.build_packet(ble_peripheral.CMD_WIFI_CONFIG, payload),
        failed_notifications.append,
    )
    assert _status(failed_notifications[0]) == (
        ble_peripheral.CMD_STATUS_RESP,
        ble_peripheral.MODE_IDLE,
        ble_peripheral.ERROR_WIFI_CONFIG_FAILED,
    )

    success_notifications: list[bytes] = []
    ble_peripheral.handle_command(
        ble_peripheral.build_packet(ble_peripheral.CMD_WIFI_CONFIG, payload),
        success_notifications.append,
    )
    assert ble_peripheral.parse_packet(success_notifications[0]) == (
        ble_peripheral.CMD_WIFI_CONFIG_ACK,
        b"",
    )
    assert state.error_code == ble_peripheral.ERROR_NONE

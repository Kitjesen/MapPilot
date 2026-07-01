#!/usr/bin/env python3
"""
ble_peripheral.py - BLE peripheral service (BlueZ D-Bus)

Runs on the robot as a BLE Peripheral (GATT Server), responding to client commands.

Protocol matches the Flutter-side BleProtocol / BleRobotClient:
  Service UUID:     0000FFF0-0000-1000-8000-00805F9B34FB
  Command Char:     0000FFF1  (Write)      - receive commands from client
  Status Char:      0000FFF2  (Read/Notify) - send status data to client
  WiFi Config Char: 0000FFF3  (Write)      - WiFi configuration

Packet format: [0xA5] [CMD] [LEN_LO] [LEN_HI] [PAYLOAD...] [CRC8]

Dependencies:
  sudo apt-get install -y bluetooth bluez python3-dbus python3-gi
  pip3 install dbus-python PyGObject

Usage:
  sudo python3 ble_peripheral.py

systemd service:
  sudo cp scripts/gateway/ble_peripheral.service /etc/systemd/system/
  sudo systemctl enable --now ble_peripheral
"""

import logging
import signal
import struct
import subprocess
import sys
import time

try:
    from bluezero import peripheral as ble_peripheral
except ImportError:
    ble_peripheral = None  # type: ignore[assignment]

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("ble_peripheral")

# ============================================================
# Protocol constants
# ============================================================
HEADER = 0xA5
CMD_PING          = 0x01
CMD_PONG          = 0x02
CMD_ESTOP         = 0x10
CMD_MODE_SWITCH   = 0x11
CMD_WIFI_CONFIG   = 0x20
CMD_WIFI_CONFIG_ACK = 0x21
CMD_STATUS_REQ    = 0x30
CMD_STATUS_RESP   = 0x31

MODE_IDLE       = 0x00
MODE_MANUAL     = 0x01
MODE_TELEOP     = 0x02
MODE_AUTONOMOUS = 0x03
MODE_MAPPING    = 0x04
MODE_ESTOP      = 0xFF

# Service UUID
SERVICE_UUID     = "0000fff0-0000-1000-8000-00805f9b34fb"
COMMAND_CHAR_UUID = "0000fff1-0000-1000-8000-00805f9b34fb"
STATUS_CHAR_UUID  = "0000fff2-0000-1000-8000-00805f9b34fb"
WIFI_CHAR_UUID    = "0000fff3-0000-1000-8000-00805f9b34fb"


# ============================================================
# CRC8 (Dallas/Maxim)
# ============================================================
def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x31) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def build_packet(cmd: int, payload: bytes = b"") -> bytes:
    plen = len(payload)
    header = struct.pack("BBBB", HEADER, cmd, plen & 0xFF, (plen >> 8) & 0xFF)
    raw = header + payload
    return raw + bytes([crc8(raw)])


def parse_packet(data: bytes):
    """Parse a packet; returns (cmd, payload) or None on error."""
    if len(data) < 5 or data[0] != HEADER:
        return None
    cmd = data[1]
    plen = data[2] | (data[3] << 8)
    if len(data) < 5 + plen:
        return None
    expected_crc = crc8(data[:4 + plen])
    if data[4 + plen] != expected_crc:
        return None
    payload = data[4:4 + plen]
    return cmd, payload


# ============================================================
# Robot state (replace with real SDK calls as needed)
# ============================================================
class RobotState:
    def __init__(self):
        self.mode = MODE_IDLE
        self.battery_percent = 75
        self.error_code = 0
        self.cpu_temp = 45
        self.start_time = time.time()

    @property
    def uptime_seconds(self):
        return int(time.time() - self.start_time)

    def get_status_payload(self) -> bytes:
        """
        payload: [battery(1)] [mode(1)] [error_code(2 LE)] [cpu_temp(1)]
                 [uptime_s(4 LE)] [rssi(1)]
        """
        return struct.pack(
            "<BBHBI b",
            self.battery_percent,
            self.mode,
            self.error_code,
            self.cpu_temp,
            self.uptime_seconds,
            -50,  # RSSI placeholder
        )

    def set_mode(self, mode: int):
        log.info(f"Mode switch: {self.mode:#x} -> {mode:#x}")
        self.mode = mode

    def emergency_stop(self):
        log.warning("EMERGENCY STOP received!")
        self.mode = MODE_ESTOP
        # Call GatewayModule HTTP stop endpoint (best-effort, non-blocking)
        import threading
        threading.Thread(target=self._send_estop_http, daemon=True).start()

    def _send_estop_http(self, gateway_url: str = "http://127.0.0.1:5050"):
        """POST /api/v1/stop to the GatewayModule HTTP server."""
        try:
            import urllib.request
            req = urllib.request.Request(
                f"{gateway_url}/api/v1/stop",
                data=b"{}",
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=2.0) as resp:  # noqa: S310  # trusted localhost gateway
                log.info("E-stop sent to gateway: HTTP %d", resp.status)
        except Exception as e:
            log.error("E-stop HTTP call failed: %s — "
                      "ensure GatewayModule is running on %s", e, gateway_url)

    def configure_wifi(self, ssid: str, password: str):
        log.info(f"WiFi config: SSID='{ssid}'")
        try:
            subprocess.run(
                ["nmcli", "dev", "wifi", "connect", ssid,
                 "password", password],
                timeout=30,
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
            )
            log.info("WiFi configured via nmcli")
        except Exception as e:
            log.error(f"WiFi config failed: {e}")


robot_state = RobotState()


# ============================================================
# Command dispatcher
# ============================================================
def handle_command(data: bytes, notify_func) -> None:
    """Dispatch an incoming BLE command packet."""
    result = parse_packet(data)
    if result is None:
        log.warning(f"Invalid packet: {data.hex()}")
        return

    cmd, payload = result
    log.info(f"Received cmd=0x{cmd:02x}, payload={payload.hex()}")

    if cmd == CMD_PING:
        resp = build_packet(CMD_PONG)
        notify_func(resp)

    elif cmd == CMD_ESTOP:
        robot_state.emergency_stop()
        resp = build_packet(CMD_STATUS_RESP, robot_state.get_status_payload())
        notify_func(resp)

    elif cmd == CMD_MODE_SWITCH:
        if len(payload) >= 1:
            robot_state.set_mode(payload[0])
        resp = build_packet(CMD_STATUS_RESP, robot_state.get_status_payload())
        notify_func(resp)

    elif cmd == CMD_STATUS_REQ:
        resp = build_packet(CMD_STATUS_RESP, robot_state.get_status_payload())
        notify_func(resp)

    elif cmd == CMD_WIFI_CONFIG:
        if len(payload) >= 2:
            ssid_len = payload[0]
            ssid = payload[1:1 + ssid_len].decode("utf-8", errors="replace")
            pass_len = payload[1 + ssid_len] if len(payload) > 1 + ssid_len else 0
            password = payload[2 + ssid_len:2 + ssid_len + pass_len].decode(
                "utf-8", errors="replace")
            robot_state.configure_wifi(ssid, password)
        resp = build_packet(CMD_WIFI_CONFIG_ACK)
        notify_func(resp)

    else:
        log.warning(f"Unknown cmd: 0x{cmd:02x}")


# ============================================================
# BlueZ advertisement & GATT server
# ============================================================

def setup_advertising():
    """Configure BLE advertising via bluetoothctl."""
    cmds = [
        "power on",
        "discoverable on",
        "discoverable-timeout 0",
        "advertise.name \"DaSuanRobot\"",
        f"advertise.uuids {SERVICE_UUID}",
        "advertise on",
    ]
    for cmd in cmds:
        try:
            subprocess.run(
                ["bluetoothctl", "--", cmd],
                timeout=5,
                capture_output=True,
            )
        except Exception as e:
            log.warning(f"bluetoothctl '{cmd}' failed: {e}")

    log.info("BLE advertising started as 'DaSuanRobot'")


def main():
    """BLE peripheral daemon entry point.

    Registers a GATT server using the bluezero library if available,
    otherwise falls back to stub mode (keeps the process alive for signals).

    GATT registration options:
      Option A: pip3 install bluezero  (used here)
      Option B: BlueZ D-Bus API via python3-dbus
      Option C: btgatt-server C tool from BlueZ source tree
    """
    log.info("=" * 50)
    log.info("BLE Peripheral Daemon starting...")
    log.info(f"Service UUID: {SERVICE_UUID}")
    log.info("=" * 50)

    setup_advertising()

    try:
        from bluezero import peripheral as ble_peripheral

        robot_ble = ble_peripheral.Peripheral(
            adapter_address=get_adapter_address(),
            local_name="DaSuanRobot",
        )

        robot_ble.add_service(srv_id=1, uuid=SERVICE_UUID, primary=True)

        # Command characteristic (Write)
        robot_ble.add_characteristic(
            srv_id=1, chr_id=1, uuid=COMMAND_CHAR_UUID,
            value=[], notifying=False,
            flags=["write"],
            write_callback=on_command_write,
        )

        # Status characteristic (Read + Notify)
        robot_ble.add_characteristic(
            srv_id=1, chr_id=2, uuid=STATUS_CHAR_UUID,
            value=list(build_packet(CMD_STATUS_RESP,
                                     robot_state.get_status_payload())),
            notifying=False,
            flags=["read", "notify"],
            read_callback=on_status_read,
            notify_callback=on_status_notify,
        )

        # WiFi config characteristic (Write)
        robot_ble.add_characteristic(
            srv_id=1, chr_id=3, uuid=WIFI_CHAR_UUID,
            value=[], notifying=False,
            flags=["write"],
            write_callback=on_wifi_write,
        )

        log.info("GATT server registered, starting event loop...")
        robot_ble.publish()

    except ImportError:
        log.warning("bluezero not installed. Running in stub mode.")
        log.info("Install: pip3 install bluezero")
        log.info("Daemon running (waiting for signals)...")

        # Stub mode: keep alive until SIGTERM/SIGINT
        def sigterm_handler(signum, frame):
            log.info("Received SIGTERM, shutting down...")
            sys.exit(0)

        signal.signal(signal.SIGTERM, sigterm_handler)
        signal.signal(signal.SIGINT, sigterm_handler)

        while True:
            time.sleep(1)

    except Exception as e:
        log.error(f"BLE peripheral failed: {e}")
        sys.exit(1)


# ---- bluezero callbacks ----

_notify_characteristic = None


def get_adapter_address() -> str:
    """Return the local Bluetooth adapter MAC address."""
    try:
        result = subprocess.run(
            ["hciconfig", "hci0"],
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=5,
        )
        for line in result.stdout.split("\n"):
            if "BD Address:" in line:
                return line.split("BD Address:")[1].split()[0].strip()
    except (subprocess.CalledProcessError, OSError):
        pass
    return "00:00:00:00:00:00"


def on_command_write(value, options):
    """bluezero callback: Command characteristic written by client."""
    data = bytes(value)
    log.info(f"Command write: {data.hex()}")

    def notify(resp_data):
        global _notify_characteristic
        if _notify_characteristic:
            _notify_characteristic.set_value(list(resp_data))
            _notify_characteristic.changed(
                ble_peripheral.DBUS_PROP_IFACE)

    handle_command(data, notify)


def on_status_read():
    """bluezero callback: Status characteristic read by client."""
    resp = build_packet(CMD_STATUS_RESP, robot_state.get_status_payload())
    return list(resp)


def on_status_notify(notifying, characteristic):
    """bluezero callback: Status characteristic notification toggled."""
    global _notify_characteristic
    if notifying:
        _notify_characteristic = characteristic
        log.info("Status notifications enabled")
    else:
        _notify_characteristic = None
        log.info("Status notifications disabled")


def on_wifi_write(value, options):
    """bluezero callback: WiFi Config characteristic written by client."""
    data = bytes(value)
    log.info(f"WiFi config write: {data.hex()}")

    def notify(resp_data):
        global _notify_characteristic
        if _notify_characteristic:
            _notify_characteristic.set_value(list(resp_data))

    handle_command(data, notify)


if __name__ == "__main__":
    main()

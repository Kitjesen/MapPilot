#!/usr/bin/env python3
"""
ble_peripheral.py - BLE 外设服务 (BlueZ D-Bus)

在机器人端运行，作为 BLE Peripheral (GATT Server) 响应客户端命令。

协议与 Flutter 端 BleProtocol / BleRobotClient 一致:
  Service UUID:     0000FFF0-0000-1000-8000-00805F9B34FB
  Command Char:     0000FFF1  (Write)     - 接收客户端命令
  Status Char:      0000FFF2  (Read/Notify) - 发送状态数据
  WiFi Config Char: 0000FFF3  (Write)     - WiFi 配置

数据包格式: [0xA5] [CMD] [LEN_LO] [LEN_HI] [PAYLOAD...] [CRC8]

依赖:
  sudo apt-get install -y bluetooth bluez python3-dbus python3-gi
  pip3 install dbus-python PyGObject

用法:
  sudo python3 ble_peripheral.py

systemd 服务:
  sudo cp ble_peripheral.service /etc/systemd/system/
  sudo systemctl enable --now ble_peripheral
"""

import struct
import time
import subprocess
import logging
import signal
import sys
import os

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("ble_peripheral")

# ============================================================
# 协议常量
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
    """解析数据包，返回 (cmd, payload) 或 None"""
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
# 机器人状态模拟 (替换为实际 SDK 调用)
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
        # TODO: 调用实际急停逻辑
        # subprocess.run(["ros2", "topic", "pub", "--once", "/stop",
        #                 "std_msgs/msg/Bool", "{data: true}"])

    def configure_wifi(self, ssid: str, password: str):
        log.info(f"WiFi config: SSID='{ssid}'")
        # 使用 nmcli 配置 WiFi
        try:
            subprocess.run(
                ["nmcli", "dev", "wifi", "connect", ssid,
                 "password", password],
                timeout=30,
                capture_output=True,
                text=True,
            )
            log.info("WiFi configured via nmcli")
        except Exception as e:
            log.error(f"WiFi config failed: {e}")


robot_state = RobotState()


# ============================================================
# 命令处理
# ============================================================
def handle_command(data: bytes, notify_func) -> None:
    """处理收到的 BLE 命令"""
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
# BlueZ Advertisement & GATT Server (bluetoothctl 简化版)
# ============================================================

def setup_advertising():
    """使用 bluetoothctl 设置 BLE 广播"""
    cmds = [
        "power on",
        "discoverable on",
        "discoverable-timeout 0",
        f"advertise.name \"DaSuanRobot\"",
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
    """
    简化的 BLE peripheral 实现。
    
    完整实现需要使用 BlueZ D-Bus API 注册 GATT 服务。
    此脚本提供了核心协议处理逻辑，实际 GATT 注册需要:
    
    方案 A: 使用 Python bluezero 库
        pip3 install bluezero
        from bluezero import peripheral
    
    方案 B: 使用 BlueZ D-Bus API (python3-dbus)
        import dbus
        # 注册 GATT Application + Advertisement
    
    方案 C: 使用 btgatt-server (C 程序)
        编译 BlueZ 源码中的 tools/btgatt-server
    
    以下是启动流程的框架代码:
    """
    log.info("=" * 50)
    log.info("BLE Peripheral Daemon starting...")
    log.info(f"Service UUID: {SERVICE_UUID}")
    log.info("=" * 50)

    # 尝试设置广播
    setup_advertising()

    try:
        # 尝试导入 bluezero
        from bluezero import peripheral as ble_peripheral

        robot_ble = ble_peripheral.Peripheral(
            adapter_address=get_adapter_address(),
            local_name="DaSuanRobot",
        )

        # 添加服务
        robot_ble.add_service(srv_id=1, uuid=SERVICE_UUID, primary=True)

        # Command 特征 (Write)
        robot_ble.add_characteristic(
            srv_id=1, chr_id=1, uuid=COMMAND_CHAR_UUID,
            value=[], notifying=False,
            flags=["write"],
            write_callback=on_command_write,
        )

        # Status 特征 (Read + Notify)
        robot_ble.add_characteristic(
            srv_id=1, chr_id=2, uuid=STATUS_CHAR_UUID,
            value=list(build_packet(CMD_STATUS_RESP,
                                     robot_state.get_status_payload())),
            notifying=False,
            flags=["read", "notify"],
            read_callback=on_status_read,
            notify_callback=on_status_notify,
        )

        # WiFi Config 特征 (Write)
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

        # 在 stub 模式下保持运行，等待信号终止
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


# ---- bluezero 回调 ----

_notify_characteristic = None


def get_adapter_address() -> str:
    """获取蓝牙适配器地址"""
    try:
        result = subprocess.run(
            ["hciconfig", "hci0"],
            capture_output=True, text=True, timeout=5,
        )
        for line in result.stdout.split("\n"):
            if "BD Address:" in line:
                return line.split("BD Address:")[1].split()[0].strip()
    except Exception:
        pass
    return "00:00:00:00:00:00"


def on_command_write(value, options):
    """Command 特征写入回调"""
    data = bytes(value)
    log.info(f"Command write: {data.hex()}")

    def notify(resp_data):
        # 更新 Status 特征并发送通知
        global _notify_characteristic
        if _notify_characteristic:
            _notify_characteristic.set_value(list(resp_data))
            _notify_characteristic.changed(
                ble_peripheral.DBUS_PROP_IFACE)

    handle_command(data, notify)


def on_status_read():
    """Status 特征读取回调"""
    resp = build_packet(CMD_STATUS_RESP, robot_state.get_status_payload())
    return list(resp)


def on_status_notify(notifying, characteristic):
    """Status 特征 Notify 开启/关闭回调"""
    global _notify_characteristic
    if notifying:
        _notify_characteristic = characteristic
        log.info("Status notifications enabled")
    else:
        _notify_characteristic = None
        log.info("Status notifications disabled")


def on_wifi_write(value, options):
    """WiFi Config 特征写入回调"""
    data = bytes(value)
    log.info(f"WiFi config write: {data.hex()}")

    def notify(resp_data):
        global _notify_characteristic
        if _notify_characteristic:
            _notify_characteristic.set_value(list(resp_data))

    handle_command(data, notify)


if __name__ == "__main__":
    main()

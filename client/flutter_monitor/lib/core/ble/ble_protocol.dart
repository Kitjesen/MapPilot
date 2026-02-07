import 'dart:typed_data';

/// BLE 通信协议定义
///
/// 数据包格式: [HEADER(1)] [CMD(1)] [LENGTH(2, little-endian)] [PAYLOAD(0~N)] [CRC8(1)]
///
/// 服务和特征 UUID:
/// - Service:     0000FFF0-0000-1000-8000-00805F9B34FB  (主服务)
/// - Command:     0000FFF1-0000-1000-8000-00805F9B34FB  (Write, 发送命令)
/// - Status:      0000FFF2-0000-1000-8000-00805F9B34FB  (Read/Notify, 接收状态)
/// - WiFi Config: 0000FFF3-0000-1000-8000-00805F9B34FB  (Write, WiFi 配置)

class BleProtocol {
  // ---- Service & Characteristic UUIDs ----
  static const String serviceUuid = '0000fff0-0000-1000-8000-00805f9b34fb';
  static const String commandCharUuid = '0000fff1-0000-1000-8000-00805f9b34fb';
  static const String statusCharUuid = '0000fff2-0000-1000-8000-00805f9b34fb';
  static const String wifiConfigCharUuid = '0000fff3-0000-1000-8000-00805f9b34fb';

  // ---- 包头 ----
  static const int header = 0xA5;

  // ---- 命令类型 ----
  static const int cmdPing = 0x01;
  static const int cmdPong = 0x02;
  static const int cmdEmergencyStop = 0x10;
  static const int cmdModeSwitch = 0x11;
  static const int cmdWifiConfig = 0x20;
  static const int cmdWifiConfigAck = 0x21;
  static const int cmdStatusRequest = 0x30;
  static const int cmdStatusResponse = 0x31;

  // ---- 机器人模式 ----
  static const int modeIdle = 0x00;
  static const int modeManual = 0x01;
  static const int modeTeleop = 0x02;
  static const int modeAutonomous = 0x03;
  static const int modeMapping = 0x04;
  static const int modeEstop = 0xFF;

  // ---- 编码 ----

  /// 构建数据包 (自动计算长度和 CRC)
  static Uint8List buildPacket(int cmd, [Uint8List? payload]) {
    final payloadLen = payload?.length ?? 0;
    final packet = Uint8List(5 + payloadLen); // header + cmd + len(2) + payload + crc
    packet[0] = header;
    packet[1] = cmd;
    packet[2] = payloadLen & 0xFF; // length low byte
    packet[3] = (payloadLen >> 8) & 0xFF; // length high byte
    if (payload != null) {
      packet.setRange(4, 4 + payloadLen, payload);
    }
    packet[4 + payloadLen] = _crc8(packet, 0, 4 + payloadLen);
    return packet;
  }

  /// 构建 Ping 包
  static Uint8List buildPing() => buildPacket(cmdPing);

  /// 构建紧急停止包
  static Uint8List buildEmergencyStop() => buildPacket(cmdEmergencyStop);

  /// 构建模式切换包
  static Uint8List buildModeSwitch(int mode) {
    return buildPacket(cmdModeSwitch, Uint8List.fromList([mode]));
  }

  /// 构建状态请求包
  static Uint8List buildStatusRequest() => buildPacket(cmdStatusRequest);

  /// 构建 WiFi 配置包
  /// payload: [ssid_len(1)] [ssid(N)] [pass_len(1)] [pass(M)]
  static Uint8List buildWifiConfig(String ssid, String password) {
    final ssidBytes = Uint8List.fromList(ssid.codeUnits);
    final passBytes = Uint8List.fromList(password.codeUnits);
    final payload = Uint8List(2 + ssidBytes.length + passBytes.length);
    payload[0] = ssidBytes.length;
    payload.setRange(1, 1 + ssidBytes.length, ssidBytes);
    payload[1 + ssidBytes.length] = passBytes.length;
    payload.setRange(
        2 + ssidBytes.length, 2 + ssidBytes.length + passBytes.length, passBytes);
    return buildPacket(cmdWifiConfig, payload);
  }

  // ---- 解码 ----

  /// 解析数据包，返回 null 如果无效
  static BlePacket? parsePacket(Uint8List data) {
    if (data.length < 5) return null;
    if (data[0] != header) return null;

    final cmd = data[1];
    final payloadLen = data[2] | (data[3] << 8);

    if (data.length < 5 + payloadLen) return null;

    // CRC 校验
    final expectedCrc = _crc8(data, 0, 4 + payloadLen);
    if (data[4 + payloadLen] != expectedCrc) return null;

    final payload =
        payloadLen > 0 ? data.sublist(4, 4 + payloadLen) : Uint8List(0);

    return BlePacket(cmd: cmd, payload: payload);
  }

  /// 解析状态响应包
  /// payload 格式: [battery(1)] [mode(1)] [error_code(2 LE)] [cpu_temp(1)] [uptime_s(4 LE)]
  static BleStatusData? parseStatusResponse(Uint8List payload) {
    if (payload.length < 10) return null;
    return BleStatusData(
      batteryPercent: payload[0],
      mode: payload[1],
      errorCode: payload[2] | (payload[3] << 8),
      cpuTemp: payload[4],
      uptimeSeconds:
          payload[5] | (payload[6] << 8) | (payload[7] << 16) | (payload[8] << 24),
      rssi: payload.length > 9 ? payload[9].toSigned(8) : 0,
    );
  }

  // ---- CRC8 (Dallas/Maxim) ----
  static int _crc8(Uint8List data, int start, int end) {
    int crc = 0;
    for (int i = start; i < end; i++) {
      crc ^= data[i];
      for (int j = 0; j < 8; j++) {
        if ((crc & 0x80) != 0) {
          crc = ((crc << 1) ^ 0x31) & 0xFF;
        } else {
          crc = (crc << 1) & 0xFF;
        }
      }
    }
    return crc;
  }
}

/// 解析后的 BLE 数据包
class BlePacket {
  final int cmd;
  final Uint8List payload;

  BlePacket({required this.cmd, required this.payload});
}

/// 机器人 BLE 状态数据
class BleStatusData {
  final int batteryPercent;
  final int mode;
  final int errorCode;
  final int cpuTemp;
  final int uptimeSeconds;
  final int rssi;

  BleStatusData({
    required this.batteryPercent,
    required this.mode,
    required this.errorCode,
    required this.cpuTemp,
    required this.uptimeSeconds,
    this.rssi = 0,
  });

  String get modeString {
    switch (mode) {
      case BleProtocol.modeIdle:
        return 'idle';
      case BleProtocol.modeManual:
        return 'manual';
      case BleProtocol.modeTeleop:
        return 'teleop';
      case BleProtocol.modeAutonomous:
        return 'autonomous';
      case BleProtocol.modeMapping:
        return 'mapping';
      case BleProtocol.modeEstop:
        return 'e-stop';
      default:
        return 'unknown($mode)';
    }
  }

  bool get hasError => errorCode != 0;
}

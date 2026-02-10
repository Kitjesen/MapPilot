import 'dart:async';
import 'dart:typed_data';
import 'package:flutter/foundation.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:flutter_monitor/core/ble/ble_protocol.dart';

/// BLE 机器人客户端
///
/// 通过 BLE 特征值 读/写 与机器人进行简单通信。
/// 功能有限（相比 gRPC），主要用于：
/// - 紧急停止
/// - 模式切换
/// - 状态查询
/// - WiFi 配置（首次设置时使用）
class BleRobotClient extends ChangeNotifier {
  BluetoothDevice? _device;
  BluetoothCharacteristic? _commandChar;
  BluetoothCharacteristic? _statusChar;
  BluetoothCharacteristic? _wifiConfigChar;

  bool _isReady = false;
  StreamSubscription? _statusNotifySub;

  // ---- 状态 ----
  BleStatusData? _latestStatus;
  BleStatusData? get latestStatus => _latestStatus;
  bool get isReady => _isReady;
  BluetoothDevice? get device => _device;

  // ---- 状态流（多个监听者） ----
  final StreamController<BleStatusData> _statusController =
      StreamController<BleStatusData>.broadcast();
  Stream<BleStatusData> get statusStream => _statusController.stream;

  // ---- Pong 回调 ----
  DateTime? _lastPongTime;
  DateTime? get lastPongTime => _lastPongTime;

  /// 连接设备后，发现服务和特征
  Future<bool> discoverAndBind(BluetoothDevice device) async {
    _device = device;
    _isReady = false;
    notifyListeners();

    try {
      final services = await device.discoverServices();
      debugPrint('[BLE Client] Discovered ${services.length} services');

      // 查找主服务
      BluetoothService? robotService;
      for (final svc in services) {
        if (svc.uuid.toString().toLowerCase() == BleProtocol.serviceUuid) {
          robotService = svc;
          break;
        }
      }

      if (robotService == null) {
        debugPrint('[BLE Client] Robot service not found: ${BleProtocol.serviceUuid}');
        return false;
      }

      // 绑定特征
      for (final char in robotService.characteristics) {
        final uuid = char.uuid.toString().toLowerCase();
        if (uuid == BleProtocol.commandCharUuid) {
          _commandChar = char;
          debugPrint('[BLE Client] Command characteristic found');
        } else if (uuid == BleProtocol.statusCharUuid) {
          _statusChar = char;
          debugPrint('[BLE Client] Status characteristic found');
        } else if (uuid == BleProtocol.wifiConfigCharUuid) {
          _wifiConfigChar = char;
          debugPrint('[BLE Client] WiFi config characteristic found');
        }
      }

      if (_commandChar == null || _statusChar == null) {
        debugPrint('[BLE Client] Required characteristics not found');
        return false;
      }

      // 订阅状态通知
      await _statusChar!.setNotifyValue(true);
      _statusNotifySub = _statusChar!.onValueReceived.listen(_onStatusData);

      _isReady = true;
      notifyListeners();
      debugPrint('[BLE Client] Bound and ready');

      // 初始化状态查询
      await requestStatus();

      return true;
    } catch (e) {
      debugPrint('[BLE Client] Discover failed: $e');
      return false;
    }
  }

  /// 处理收到的状态数据
  void _onStatusData(List<int> data) {
    final packet = BleProtocol.parsePacket(Uint8List.fromList(data));
    if (packet == null) {
      debugPrint('[BLE Client] Invalid packet received');
      return;
    }

    switch (packet.cmd) {
      case BleProtocol.cmdPong:
        _lastPongTime = DateTime.now();
        debugPrint('[BLE Client] Pong received');
        notifyListeners();
        break;

      case BleProtocol.cmdStatusResponse:
        final status = BleProtocol.parseStatusResponse(packet.payload);
        if (status != null) {
          _latestStatus = status;
          _statusController.add(status);
          notifyListeners();
          debugPrint(
              '[BLE Client] Status: battery=${status.batteryPercent}% '
              'mode=${status.modeString} temp=${status.cpuTemp}°C');
        }
        break;

      case BleProtocol.cmdWifiConfigAck:
        debugPrint('[BLE Client] WiFi config acknowledged');
        break;

      default:
        debugPrint('[BLE Client] Unknown cmd: 0x${packet.cmd.toRadixString(16)}');
    }
  }

  // ================================================================
  // 命令发送
  // ================================================================

  /// 发送 Ping
  Future<void> sendPing() async {
    await _writeCommand(BleProtocol.buildPing());
  }

  /// 发送紧急停止
  Future<void> sendEmergencyStop() async {
    debugPrint('[BLE Client] EMERGENCY STOP');
    await _writeCommand(BleProtocol.buildEmergencyStop());
  }

  /// 发送模式切换
  Future<void> sendModeSwitch(int mode) async {
    debugPrint('[BLE Client] Mode switch -> $mode');
    await _writeCommand(BleProtocol.buildModeSwitch(mode));
  }

  /// 请求状态
  Future<void> requestStatus() async {
    await _writeCommand(BleProtocol.buildStatusRequest());
  }

  /// 发送 WiFi 配置
  Future<void> sendWifiConfig(String ssid, String password) async {
    debugPrint('[BLE Client] WiFi config: SSID=$ssid');
    final packet = BleProtocol.buildWifiConfig(ssid, password);
    if (_wifiConfigChar != null) {
      await _wifiConfigChar!.write(packet.toList(), withoutResponse: false);
    } else {
      // Fallback: 通过 command char 发送
      await _writeCommand(packet);
    }
  }

  /// 周期性拉取状态（每 N 秒）
  Timer? _statusPollTimer;
  void startStatusPolling({Duration interval = const Duration(seconds: 3)}) {
    _statusPollTimer?.cancel();
    _statusPollTimer = Timer.periodic(interval, (_) {
      if (_isReady) requestStatus();
    });
  }

  void stopStatusPolling() {
    _statusPollTimer?.cancel();
    _statusPollTimer = null;
  }

  // ================================================================
  // 内部
  // ================================================================

  Future<void> _writeCommand(Uint8List data) async {
    if (_commandChar == null || !_isReady) {
      debugPrint('[BLE Client] Not ready, cannot write');
      return;
    }
    try {
      await _commandChar!.write(data.toList(), withoutResponse: false);
    } catch (e) {
      debugPrint('[BLE Client] Write failed: $e');
    }
  }

  bool _disposed = false;

  /// 断开并清理
  void unbind() {
    stopStatusPolling();
    _statusNotifySub?.cancel();
    _statusNotifySub = null;
    _commandChar = null;
    _statusChar = null;
    _wifiConfigChar = null;
    _device = null;
    _isReady = false;
    _latestStatus = null;
    if (!_disposed) notifyListeners();
  }

  @override
  void dispose() {
    _disposed = true;
    unbind();
    _statusController.close();
    super.dispose();
  }
}

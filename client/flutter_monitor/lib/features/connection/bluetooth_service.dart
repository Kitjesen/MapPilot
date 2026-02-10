import 'dart:async';
import 'package:flutter/foundation.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:flutter_monitor/core/ble/ble_protocol.dart';
import 'package:flutter_monitor/core/ble/ble_robot_client.dart';

/// BLE connection states
enum BleConnectionState {
  disconnected,
  scanning,
  connecting,
  discovering,
  connected,
  error,
}

/// A discovered BLE robot device
class BleRobotDevice {
  final BluetoothDevice device;
  final String name;
  final int rssi;
  final DateTime discoveredAt;
  final bool hasRobotService;

  BleRobotDevice({
    required this.device,
    required this.name,
    required this.rssi,
    this.hasRobotService = false,
    DateTime? discoveredAt,
  }) : discoveredAt = discoveredAt ?? DateTime.now();

  String get id => device.remoteId.str;
  String get displayName =>
      name.isNotEmpty ? name : 'Unknown (${id.substring(0, 8)})';

  /// Signal quality: 0.0 to 1.0
  double get signalQuality {
    final clamped = rssi.clamp(-100, -30);
    return (clamped + 100) / 70.0;
  }
}

/// Manages Bluetooth Low Energy connections for robot communication.
///
/// 使用 [BleRobotClient] 进行通信，支持：
/// - 设备扫描 & 连接
/// - 紧急停止 / 模式切换
/// - WiFi 配置
/// - 状态查询
class BluetoothService extends ChangeNotifier {
  BleConnectionState _state = BleConnectionState.disconnected;
  BleConnectionState get state => _state;

  final List<BleRobotDevice> _discoveredDevices = [];
  List<BleRobotDevice> get discoveredDevices =>
      List.unmodifiable(_discoveredDevices);

  BluetoothDevice? _connectedDevice;
  BluetoothDevice? get connectedDevice => _connectedDevice;

  /// BLE 机器人通信客户端（连接后可用）
  final BleRobotClient _robotClient = BleRobotClient();
  BleRobotClient get robotClient => _robotClient;
  bool get isRobotReady => _robotClient.isReady;

  StreamSubscription? _scanSubscription;
  StreamSubscription? _connectionSubscription;

  String? _errorMessage;
  String? get errorMessage => _errorMessage;

  /// Check if Bluetooth is available and enabled
  Future<bool> get isAvailable async {
    try {
      final supported = await FlutterBluePlus.isSupported;
      if (!supported) return false;
      final state = FlutterBluePlus.adapterStateNow;
      return state == BluetoothAdapterState.on;
    } catch (_) {
      return false;
    }
  }

  /// Start scanning for BLE robot devices
  Future<void> startScan(
      {Duration timeout = const Duration(seconds: 10)}) async {
    if (_state == BleConnectionState.scanning) return;

    _discoveredDevices.clear();
    _state = BleConnectionState.scanning;
    _errorMessage = null;
    notifyListeners();

    try {
      final available = await isAvailable;
      if (!available) {
        _state = BleConnectionState.error;
        _errorMessage = '蓝牙未开启或不可用';
        notifyListeners();
        return;
      }

      await FlutterBluePlus.startScan(
        timeout: timeout,
        androidUsesFineLocation: true,
      );

      _scanSubscription = FlutterBluePlus.scanResults.listen((results) {
        for (final r in results) {
          final name = r.device.platformName;
          if (name.isNotEmpty) {
            // 检查是否包含机器人 BLE 服务 UUID
            final hasRobotSvc = r.advertisementData.serviceUuids
                .any((u) => u.toString().toLowerCase() == BleProtocol.serviceUuid);

            final existing = _discoveredDevices.indexWhere(
              (d) => d.id == r.device.remoteId.str,
            );
            final device = BleRobotDevice(
              device: r.device,
              name: name,
              rssi: r.rssi,
              hasRobotService: hasRobotSvc,
            );
            if (existing >= 0) {
              _discoveredDevices[existing] = device;
            } else {
              _discoveredDevices.add(device);
            }
          }
        }
        // 优先显示包含机器人服务的设备，然后按信号强度排序
        _discoveredDevices.sort((a, b) {
          if (a.hasRobotService != b.hasRobotService) {
            return a.hasRobotService ? -1 : 1;
          }
          return b.rssi.compareTo(a.rssi);
        });
        notifyListeners();
      });

      Future.delayed(timeout, () {
        if (_state == BleConnectionState.scanning) {
          stopScan();
        }
      });
    } catch (e) {
      debugPrint('[BLE] Scan error: $e');
      _state = BleConnectionState.error;
      _errorMessage = '扫描失败: $e';
      notifyListeners();
    }
  }

  /// Stop scanning
  void stopScan() {
    FlutterBluePlus.stopScan();
    _scanSubscription?.cancel();
    _scanSubscription = null;
    if (_state == BleConnectionState.scanning) {
      _state = BleConnectionState.disconnected;
      notifyListeners();
    }
  }

  /// Connect to a BLE device and discover robot services
  Future<bool> connectDevice(BleRobotDevice robot) async {
    _state = BleConnectionState.connecting;
    _errorMessage = null;
    notifyListeners();

    try {
      await robot.device.connect(
        timeout: const Duration(seconds: 10),
        autoConnect: false,
      );

      _connectedDevice = robot.device;
      _state = BleConnectionState.discovering;
      notifyListeners();

      // 发现服务并绑定特征
      debugPrint('[BLE] Connected to ${robot.displayName}, discovering services...');
      final success = await _robotClient.discoverAndBind(robot.device);

      if (success) {
        _state = BleConnectionState.connected;
        _robotClient.startStatusPolling();
        debugPrint('[BLE] Robot BLE service bound successfully');
      } else {
        _state = BleConnectionState.connected;
        debugPrint('[BLE] Connected but robot service not found (basic BLE only)');
      }

      // Listen for disconnection
      _connectionSubscription =
          robot.device.connectionState.listen((connState) {
        if (connState == BluetoothConnectionState.disconnected) {
          _robotClient.unbind();
          _connectedDevice = null;
          _state = BleConnectionState.disconnected;
          notifyListeners();
        }
      });

      notifyListeners();
      return true;
    } catch (e) {
      debugPrint('[BLE] Connection failed: $e');
      _state = BleConnectionState.error;
      _errorMessage = '连接失败: $e';
      notifyListeners();
      return false;
    }
  }

  /// Disconnect from the current device
  Future<void> disconnectDevice() async {
    _robotClient.unbind();
    try {
      await _connectedDevice?.disconnect();
    } catch (_) {}
    _connectionSubscription?.cancel();
    _connectionSubscription = null;
    _connectedDevice = null;
    _state = BleConnectionState.disconnected;
    notifyListeners();
  }

  @override
  void dispose() {
    stopScan();
    // 必须先 disconnect（会调 unbind → notifyListeners），
    // 再 dispose robotClient，否则 dispose 后 notifyListeners 会崩
    disconnectDevice();
    _robotClient.dispose();
    super.dispose();
  }
}

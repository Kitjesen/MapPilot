import 'dart:async';
import 'package:flutter/foundation.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';
import 'package:flutter_monitor/core/services/notification_service.dart';
import 'package:flutter_monitor/core/storage/settings_preferences.dart';

/// 告警级别
enum AlertLevel { info, warning, critical }

/// 告警类型
enum AlertType { batteryLow, tempHigh, commLost }

/// 一条告警记录
class AlertRecord {
  final AlertType type;
  final AlertLevel level;
  final String title;
  final String message;
  final DateTime timestamp;
  bool dismissed;

  AlertRecord({
    required this.type,
    required this.level,
    required this.title,
    required this.message,
    DateTime? timestamp,
    this.dismissed = false,
  }) : timestamp = timestamp ?? DateTime.now();
}

/// 告警监控服务
///
/// 监听 [RobotConnectionProvider] 状态变化，根据 [SettingsPreferences]
/// 中的用户配置，在满足阈值时生成告警记录。
/// 上层 UI 通过 [alertStream] 收到新告警后自行展示 SnackBar / 通知。
class AlertMonitorService extends ChangeNotifier {
  // ---- 阈值 ----
  static const double batteryThreshold = 20.0; // %
  static const double tempThreshold = 70.0; // °C
  static const Duration commLostDelay = Duration(seconds: 8);
  static const Duration _cooldown = Duration(seconds: 60);

  // ---- 依赖 ----
  RobotConnectionProvider? _connProvider;
  SettingsPreferences? _settingsPrefs;

  // ---- 内部状态 ----
  final Map<AlertType, DateTime> _lastFired = {};
  Timer? _pollTimer;
  ConnectionStatus? _prevConnectionStatus;

  // ---- 告警队列（新告警从这里弹出） ----
  final StreamController<AlertRecord> _alertController =
      StreamController<AlertRecord>.broadcast();

  /// Widget 层监听此流来展示 SnackBar / 通知
  Stream<AlertRecord> get alertStream => _alertController.stream;

  /// 历史告警（最多保留 200 条）
  final List<AlertRecord> _history = [];
  List<AlertRecord> get history => List.unmodifiable(_history);

  // ---- 生命周期 ----

  /// 绑定依赖 —— 在 main.dart 中 Provider 就绪后调用
  void bind({
    required RobotConnectionProvider connProvider,
    required SettingsPreferences settingsPrefs,
  }) {
    _connProvider = connProvider;
    _settingsPrefs = settingsPrefs;

    // 每 2 秒轮询一次状态
    _pollTimer?.cancel();
    _pollTimer = Timer.periodic(const Duration(seconds: 2), (_) => _evaluate());

    // 同时监听连接状态变化（即时感知断连）
    _connProvider!.addListener(_onProviderChanged);
  }

  void _onProviderChanged() {
    _evaluate();
  }

  /// 核心评估逻辑
  void _evaluate() {
    final conn = _connProvider;
    final prefs = _settingsPrefs;
    if (conn == null || prefs == null) return;

    // ---- 1. 通信异常 ----
    if (prefs.alertCommLost) {
      final currentStatus = conn.status;
      // 从 connected/reconnecting 变成 error/disconnected
      if (_prevConnectionStatus == ConnectionStatus.connected &&
          (currentStatus == ConnectionStatus.error ||
           currentStatus == ConnectionStatus.disconnected ||
           currentStatus == ConnectionStatus.reconnecting)) {
        _fire(AlertRecord(
          type: AlertType.commLost,
          level: AlertLevel.critical,
          title: '通信异常',
          message: '与机器人的连接已断开',
        ));
      }
      _prevConnectionStatus = currentStatus;
    }

    // 以下检查需要连接中
    if (!conn.isConnected) return;
    final slow = conn.latestSlowState;
    if (slow == null) return;

    // ---- 2. 电量低 ----
    if (prefs.alertBatteryLow) {
      final battery = slow.resources.batteryPercent;
      if (battery > 0 && battery < batteryThreshold) {
        _fire(AlertRecord(
          type: AlertType.batteryLow,
          level: AlertLevel.warning,
          title: '电量低',
          message: '电池电量 ${battery.toStringAsFixed(0)}%，请及时充电',
        ));
      }
    }

    // ---- 3. 温度过高 ----
    if (prefs.alertTempHigh) {
      final temp = slow.resources.cpuTemp;
      if (temp > tempThreshold) {
        _fire(AlertRecord(
          type: AlertType.tempHigh,
          level: AlertLevel.warning,
          title: '温度过高',
          message: 'CPU 温度 ${temp.toStringAsFixed(1)}°C，超过 ${tempThreshold.toStringAsFixed(0)}°C 阈值',
        ));
      }
    }
  }

  /// 发射告警（带冷却时间防抖）
  void _fire(AlertRecord record) {
    final last = _lastFired[record.type];
    if (last != null && DateTime.now().difference(last) < _cooldown) {
      return; // 冷却中，不重复告警
    }
    _lastFired[record.type] = DateTime.now();

    _history.insert(0, record);
    if (_history.length > 200) _history.removeLast();

    _alertController.add(record);

    // OS 级通知（当用户开启时）
    if (_settingsPrefs?.alertSystem ?? false) {
      _sendSystemNotification(record);
    }

    debugPrint('[AlertMonitor] Fired: ${record.title} - ${record.message}');
    notifyListeners();
  }

  /// 发送系统级推送通知
  void _sendSystemNotification(AlertRecord record) {
    final notifService = NotificationService();
    if (!notifService.isInitialized) return;

    final id = record.type.index * 100 +
        (DateTime.now().millisecondsSinceEpoch % 100);

    if (record.level == AlertLevel.critical) {
      notifService.showCriticalAlert(
        id: id,
        title: record.title,
        body: record.message,
        payload: record.type.name,
      );
    } else {
      notifService.showAlert(
        id: id,
        title: record.title,
        body: record.message,
        payload: record.type.name,
      );
    }
  }

  /// 清除某条告警
  void dismiss(AlertRecord record) {
    record.dismissed = true;
    notifyListeners();
  }

  /// 清空历史
  void clearHistory() {
    _history.clear();
    notifyListeners();
  }

  @override
  void dispose() {
    _pollTimer?.cancel();
    _connProvider?.removeListener(_onProviderChanged);
    _alertController.close();
    super.dispose();
  }
}

import 'dart:async';
import 'package:flutter/foundation.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';

/// 一条采样记录
class StateLogEntry {
  final DateTime timestamp;
  final double posX;
  final double posY;
  final double posZ;
  final double linearVel;
  final double angularVel;
  final double roll;
  final double pitch;
  final double yaw;
  final double batteryPercent;
  final double cpuPercent;
  final double cpuTemp;
  final String mode;
  final String connectionStatus;

  StateLogEntry({
    required this.timestamp,
    this.posX = 0,
    this.posY = 0,
    this.posZ = 0,
    this.linearVel = 0,
    this.angularVel = 0,
    this.roll = 0,
    this.pitch = 0,
    this.yaw = 0,
    this.batteryPercent = 0,
    this.cpuPercent = 0,
    this.cpuTemp = 0,
    this.mode = '',
    this.connectionStatus = '',
  });

  /// CSV header
  static const String csvHeader =
      'timestamp,pos_x,pos_y,pos_z,linear_vel,angular_vel,'
      'roll,pitch,yaw,battery_pct,cpu_pct,cpu_temp,mode,connection_status';

  /// 转成 CSV 行
  String toCsvRow() {
    return '${timestamp.toIso8601String()},'
        '${posX.toStringAsFixed(4)},${posY.toStringAsFixed(4)},${posZ.toStringAsFixed(4)},'
        '${linearVel.toStringAsFixed(4)},${angularVel.toStringAsFixed(4)},'
        '${roll.toStringAsFixed(2)},${pitch.toStringAsFixed(2)},${yaw.toStringAsFixed(2)},'
        '${batteryPercent.toStringAsFixed(1)},${cpuPercent.toStringAsFixed(1)},'
        '${cpuTemp.toStringAsFixed(1)},$mode,$connectionStatus';
  }
}

/// 状态日志记录服务
///
/// 以 1Hz 频率从 [RobotConnectionProvider] 采样 FastState / SlowState，
/// 写入环形缓冲区（默认 10 000 条 ≈ 2.7 小时）。
/// [LogExportPage] 可按时间范围筛选并导出 CSV。
class StateLoggerService extends ChangeNotifier {
  static const int maxEntries = 10000;

  final List<StateLogEntry> _buffer = [];
  Timer? _sampleTimer;
  RobotConnectionProvider? _connProvider;

  List<StateLogEntry> get buffer => List.unmodifiable(_buffer);
  int get entryCount => _buffer.length;

  /// 绑定连接 Provider 后开始采样
  void bind(RobotConnectionProvider connProvider) {
    _connProvider = connProvider;
    _sampleTimer?.cancel();
    _sampleTimer = Timer.periodic(const Duration(seconds: 1), (_) => _sample());
    debugPrint('[StateLogger] Started sampling at 1Hz (buffer max=$maxEntries)');
  }

  void _sample() {
    final conn = _connProvider;
    if (conn == null) return;

    final fast = conn.latestFastState;
    final slow = conn.latestSlowState;

    final entry = StateLogEntry(
      timestamp: DateTime.now(),
      posX: fast?.pose.position.x ?? 0,
      posY: fast?.pose.position.y ?? 0,
      posZ: fast?.pose.position.z ?? 0,
      linearVel: fast?.velocity.linear.x ?? 0,
      angularVel: fast?.velocity.angular.z ?? 0,
      roll: fast?.rpyDeg.x ?? 0,
      pitch: fast?.rpyDeg.y ?? 0,
      yaw: fast?.rpyDeg.z ?? 0,
      batteryPercent: slow?.resources.batteryPercent ?? 0,
      cpuPercent: slow?.resources.cpuPercent ?? 0,
      cpuTemp: slow?.resources.cpuTemp ?? 0,
      mode: slow?.currentMode ?? '',
      connectionStatus: conn.status.name,
    );

    _buffer.add(entry);
    if (_buffer.length > maxEntries) {
      _buffer.removeAt(0);
    }
  }

  /// 按时间范围筛选日志
  List<StateLogEntry> query(DateTime start, DateTime end) {
    return _buffer
        .where((e) =>
            !e.timestamp.isBefore(start) && !e.timestamp.isAfter(end))
        .toList();
  }

  /// 生成 CSV 内容
  String exportCsv(DateTime start, DateTime end) {
    final entries = query(start, end);
    final buf = StringBuffer();
    buf.writeln(StateLogEntry.csvHeader);
    for (final e in entries) {
      buf.writeln(e.toCsvRow());
    }
    return buf.toString();
  }

  /// 清空缓冲区
  void clear() {
    _buffer.clear();
    notifyListeners();
  }

  @override
  void dispose() {
    _sampleTimer?.cancel();
    super.dispose();
  }
}

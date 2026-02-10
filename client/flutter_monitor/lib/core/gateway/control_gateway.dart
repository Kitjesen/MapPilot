import 'dart:async';
import 'package:flutter/foundation.dart';
import 'package:flutter_monitor/core/grpc/robot_client_base.dart';
import 'package:flutter_monitor/core/grpc/dog_direct_client.dart';
import 'package:flutter_monitor/core/services/app_logger.dart';
import 'package:robot_proto/robot_proto.dart';

/// 控制网关：封装双模控制（Nav Board / Dog Direct）+ 租约管理 + 遥操作流。
///
/// UI 通过 `context.watch<ControlGateway>()` 读取状态，
/// 摇杆输入调用 [sendVelocity]，按钮调用 dog 命令。
class ControlGateway extends ChangeNotifier {
  RobotClientBase? _client;
  DogDirectClient? _dogClient;

  ControlGateway({RobotClientBase? client, DogDirectClient? dogClient})
      : _client = client,
        _dogClient = dogClient;

  /// 更新底层客户端引用
  void updateClients({RobotClientBase? client, DogDirectClient? dogClient}) {
    _client = client;
    _dogClient = dogClient;

    // 如果连接断开，重置控制状态
    if (client == null && dogClient == null) {
      _stopTeleop();
      _linearX = 0;
      _linearY = 0;
      _angularZ = 0;
    }

    // 如果只有 Dog Board 连上了，自动切换
    if (client == null && dogClient != null && dogClient.isConnected) {
      _useDogDirect = true;
    }

    notifyListeners();
  }

  // ================================================================
  // 状态
  // ================================================================

  bool _useDogDirect = false;
  bool _hasLease = false;
  bool _isTeleopActive = false;
  double _linearX = 0.0;
  double _linearY = 0.0;
  double _angularZ = 0.0;
  String? _statusMessage;

  bool get useDogDirect => _useDogDirect;
  bool get hasLease => _hasLease;
  bool get isTeleopActive => _isTeleopActive;
  double get linearX => _linearX;
  double get linearY => _linearY;
  double get angularZ => _angularZ;
  String? get statusMessage => _statusMessage;
  DogDirectClient? get dogClient => _dogClient;

  // 遥操作流
  StreamController<Twist>? _velocityController;
  StreamSubscription? _teleopSubscription;

  // 发送节流
  DateTime _lastTwistSend = DateTime(2000);
  static const _twistSendInterval = Duration(milliseconds: 50); // 20Hz max

  // ================================================================
  // 控制模式切换
  // ================================================================

  /// 切换 Dog Direct / Nav Board 模式
  void toggleDogDirect() {
    _useDogDirect = !_useDogDirect;
    notifyListeners();
  }

  /// 设置控制模式
  void setDogDirect(bool value) {
    _useDogDirect = value;
    notifyListeners();
  }

  // ================================================================
  // 租约管理
  // ================================================================

  /// 切换租约状态
  ///
  /// 返回操作是否成功。
  Future<bool> toggleLease() async {
    if (_hasLease) {
      await _releaseLease();
      return true;
    } else {
      return await _acquireLease();
    }
  }

  Future<bool> _acquireLease() async {
    final client = _client;
    if (client == null) return false;

    try {
      final success = await client.acquireLease();
      _hasLease = success;
      if (success) {
        _startTeleopStream();
      } else {
        _statusMessage = '无法获取控制权';
      }
      notifyListeners();
      return success;
    } catch (e) {
      _statusMessage = '获取租约失败: $e';
      notifyListeners();
      return false;
    }
  }

  Future<void> _releaseLease() async {
    final client = _client;
    if (client == null) return;

    _stopTeleop();
    try {
      await client.releaseLease();
    } catch (_) {}
    _hasLease = false;
    notifyListeners();
  }

  // ================================================================
  // 遥操作
  // ================================================================

  void _startTeleopStream() {
    final client = _client;
    if (client == null) return;

    _velocityController?.close();
    _velocityController = StreamController<Twist>.broadcast();
    _isTeleopActive = true;

    try {
      _teleopSubscription =
          client.streamTeleop(_velocityController!.stream).listen(
        (feedback) {
          // 可扩展：处理 safety feedback
        },
        onError: (e) {
          AppLogger.control.error('Teleop stream error', error: e);
          _hasLease = false;
          _isTeleopActive = false;
          notifyListeners();
        },
      );
    } catch (e) {
      AppLogger.control.error('Failed to start teleop', error: e);
      _isTeleopActive = false;
    }
    notifyListeners();
  }

  void _stopTeleop() {
    _teleopSubscription?.cancel();
    _teleopSubscription = null;
    _velocityController?.close();
    _velocityController = null;
    _isTeleopActive = false;
  }

  /// 发送速度命令（带节流，20Hz）
  ///
  /// 由 UI 摇杆回调调用。[vx], [vy] 为线速度，[wz] 为角速度。
  /// Dog Direct 模式下为归一化值 [-1, 1]，Nav Board 模式下为物理单位。
  void sendVelocity(double vx, double vy, double wz) {
    _linearX = vx;
    _linearY = vy;
    _angularZ = wz;

    final now = DateTime.now();
    if (now.difference(_lastTwistSend) >= _twistSendInterval) {
      _lastTwistSend = now;
      if (_useDogDirect) {
        _dogClient?.walk(_linearX, _linearY, _angularZ);
      } else if (_velocityController != null) {
        final twist = Twist()
          ..linear = (Vector3()
            ..x = _linearX
            ..y = _linearY)
          ..angular = (Vector3()..z = _angularZ);
        _velocityController!.add(twist);
      }
    }
    notifyListeners();
  }

  /// 停止速度（归零）
  void stopVelocity() {
    sendVelocity(0, 0, 0);
  }

  // ================================================================
  // 模式切换 (Nav Board)
  // ================================================================

  /// 设置机器人运行模式
  Future<(bool, String)> setMode(RobotMode mode) async {
    final client = _client;
    if (client == null) return (false, '未连接');

    try {
      final success = await client.setMode(mode);
      return (success, success ? '模式已切换: $mode' : '模式切换失败');
    } catch (e) {
      return (false, '$e');
    }
  }

  // ================================================================
  // 紧急停止
  // ================================================================

  /// 紧急停止所有连接的设备
  Future<void> emergencyStop() async {
    _linearX = 0;
    _linearY = 0;
    _angularZ = 0;

    // 停 Dog Board
    if (_dogClient != null && _dogClient!.isConnected) {
      await _dogClient!.emergencyStop();
    }

    // 停 Nav Board
    if (_client != null && _client!.isConnected) {
      await _client!.emergencyStop(hardStop: false);
    }

    _statusMessage = '紧急停止已触发';
    notifyListeners();
  }

  // ================================================================
  // Dog Direct 命令
  // ================================================================

  /// 使能电机
  Future<(bool, String?)> dogEnable() async {
    if (_dogClient == null) return (false, 'Dog 未连接');
    final ok = await _dogClient!.enable();
    if (!ok) return (false, _dogClient!.errorMessage);
    notifyListeners();
    return (true, null);
  }

  /// 关闭电机
  Future<void> dogDisable() async {
    await _dogClient?.disable();
    notifyListeners();
  }

  /// 站立
  Future<(bool, String?)> dogStandUp() async {
    if (_dogClient == null) return (false, 'Dog 未连接');
    final ok = await _dogClient!.standUp();
    if (!ok) return (false, _dogClient!.errorMessage);
    notifyListeners();
    return (true, null);
  }

  /// 趴下
  Future<void> dogSitDown() async {
    await _dogClient?.sitDown();
    notifyListeners();
  }

  // ================================================================
  // Cleanup
  // ================================================================

  void clearStatusMessage() {
    _statusMessage = null;
    notifyListeners();
  }

  @override
  void dispose() {
    _stopTeleop();
    super.dispose();
  }
}

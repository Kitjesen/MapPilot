import 'dart:async';
import 'package:flutter/foundation.dart';
import 'package:flutter_monitor/core/grpc/robot_client_base.dart';
import 'package:flutter_monitor/core/services/app_logger.dart';
import 'package:robot_proto/src/common.pb.dart';
import 'package:robot_proto/src/control.pb.dart';

/// 任务管理网关：封装任务生命周期 + 状态轮询。
///
/// UI 通过 `context.watch<TaskGateway>()` 读取状态、调用方法，
/// 不再直接操作 [RobotClientBase] 或管理轮询 Timer。
class TaskGateway extends ChangeNotifier {
  RobotClientBase? _client;

  /// Called when a mapping task completes successfully.
  /// Wired in main.dart → triggers MapGateway.saveMap().
  VoidCallback? onMappingComplete;

  /// Called when any task reaches a terminal state (completed/failed/cancelled).
  /// Useful for cross-gateway coordination (e.g. re-enable OTA after task ends).
  void Function(TaskType type, TaskStatus status)? onTaskTerminated;

  TaskGateway({RobotClientBase? client}) : _client = client;

  /// 更新底层 gRPC 客户端
  void updateClient(RobotClientBase? client) {
    _client = client;
    if (client == null) {
      _stopPolling();
      _isRunning = false;
      _isPaused = false;
      _activeTaskId = null;
      _activeTaskType = TaskType.TASK_TYPE_UNSPECIFIED;
      _progress = 0.0;
      _statusMessage = null;
    }
    notifyListeners();
  }

  // ================================================================
  // 状态
  // ================================================================

  bool _isRunning = false;
  bool _isPaused = false;
  String? _activeTaskId;
  TaskType _activeTaskType = TaskType.TASK_TYPE_UNSPECIFIED;
  double _progress = 0.0;
  TaskStatus _taskStatus = TaskStatus.TASK_STATUS_UNSPECIFIED;
  String? _statusMessage;
  Timer? _statusTimer;
  int _pollFailCount = 0;
  static const _pollWarnThreshold = 3;
  static const _pollErrorThreshold = 5;

  bool get isRunning => _isRunning;
  bool get isPaused => _isPaused;
  String? get activeTaskId => _activeTaskId;
  TaskType get activeTaskType => _activeTaskType;
  double get progress => _progress;
  TaskStatus get taskStatus => _taskStatus;
  String? get statusMessage => _statusMessage;

  /// 清除状态消息
  void clearStatusMessage() {
    _statusMessage = null;
    notifyListeners();
  }

  /// 重连后恢复任务状态。
  ///
  /// 如果断线前有活跃任务，重新查询后端状态并恢复轮询。
  /// 如果没有活跃任务，查询后端航点来源以同步显示。
  Future<void> recoverState() async {
    final client = _client;
    if (client == null) return;

    // 如果断线前有 activeTaskId，尝试恢复
    if (_activeTaskId != null) {
      try {
        final r = await client.getTaskStatus(taskId: _activeTaskId!);
        _taskStatus = r.task.status;
        _progress = r.task.progressPercent;
        final terminal =
            _taskStatus == TaskStatus.TASK_STATUS_COMPLETED ||
            _taskStatus == TaskStatus.TASK_STATUS_FAILED ||
            _taskStatus == TaskStatus.TASK_STATUS_CANCELLED;
        if (terminal) {
          _isRunning = false;
          _isPaused = false;
          _stopPolling();

          // 断线期间任务结束 → 补发回调 (否则建图保存等联动逻辑会丢失)
          if (_taskStatus == TaskStatus.TASK_STATUS_COMPLETED &&
              _activeTaskType == TaskType.TASK_TYPE_MAPPING) {
            AppLogger.task.info(
                'Recovery: mapping completed during disconnect → firing onMappingComplete');
            onMappingComplete?.call();
          }
          onTaskTerminated?.call(_activeTaskType, _taskStatus);
        } else {
          _isRunning = true;
          _isPaused = _taskStatus == TaskStatus.TASK_STATUS_PAUSED;
          _startPolling();
        }
        notifyListeners();
      } catch (_) {
        // 查不到就清除本地状态
        _isRunning = false;
        _isPaused = false;
        _activeTaskId = null;
        _stopPolling();
        notifyListeners();
      }
    }
  }

  // ================================================================
  // 业务方法
  // ================================================================

  /// 启动导航任务（统一入口）
  ///
  /// 所有导航任务应优先通过该方法发起，避免页面重复拼装参数。
  Future<bool> startNavigationTask(
    List<NavigationGoal> waypoints, {
    bool loop = false,
    double maxSpeed = 0,
  }) async {
    if (waypoints.isEmpty) {
      _statusMessage = '请先设置导航目标';
      notifyListeners();
      return false;
    }
    final params = NavigationParams()
      ..waypoints.addAll(waypoints)
      ..loop = loop
      ..maxSpeed = maxSpeed;
    return startTask(
      TaskType.TASK_TYPE_NAVIGATION,
      navigationParams: params,
    );
  }

  /// 启动单目标点导航。
  Future<bool> startSingleGoalNavigation({
    required double x,
    required double y,
    double z = 0,
    double yaw = 0,
    double arrivalRadius = 1.0,
    String label = '',
    double maxSpeed = 0,
  }) async {
    final goal = NavigationGoal()
      ..position = (Vector3()
        ..x = x
        ..y = y
        ..z = z)
      ..yaw = yaw
      ..arrivalRadius = arrivalRadius
      ..label = label;
    return startNavigationTask([goal], maxSpeed: maxSpeed);
  }

  /// Whether current task execution should block cold OTA actions.
  bool get blocksColdOta => _isRunning;

  // ================================================================
  // 航点管理
  // ================================================================

  /// 查询当前活跃航点 (来源 / 列表 / 进度)
  Future<GetActiveWaypointsResponse?> getActiveWaypoints() async {
    final client = _client;
    if (client == null) return null;
    try {
      return await client.getActiveWaypoints();
    } catch (e) {
      _statusMessage = '查询航点失败: $e';
      notifyListeners();
      return null;
    }
  }

  /// 清除所有航点并立即停车
  Future<bool> clearWaypoints() async {
    final client = _client;
    if (client == null) return false;
    try {
      final resp = await client.clearWaypoints();
      if (resp.base.errorCode == ErrorCode.ERROR_CODE_OK) {
        _statusMessage = '已清除 ${resp.clearedCount} 个航点';
        notifyListeners();
        return true;
      } else {
        _statusMessage = resp.base.errorMessage;
        notifyListeners();
        return false;
      }
    } catch (e) {
      _statusMessage = '清除航点失败: $e';
      notifyListeners();
      return false;
    }
  }

  /// 启动任务
  ///
  /// 内部自动完成前置条件:
  /// - 建图模式 → SetMode(MAPPING)
  /// - 导航/巡检/循迹 → AcquireLease + SetMode(AUTONOMOUS)
  Future<bool> startTask(
    TaskType type, {
    NavigationParams? navigationParams,
    MappingParams? mappingParams,
    FollowPathParams? followPathParams,
  }) async {
    final client = _client;
    if (client == null) {
      _statusMessage = '未连接';
      notifyListeners();
      return false;
    }

    try {
      if (type == TaskType.TASK_TYPE_MAPPING) {
        // 建图模式 → 切换到 MAPPING mode
        final modeOk = await client.setMode(RobotMode.ROBOT_MODE_MAPPING);
        if (!modeOk) {
          _statusMessage = '切换建图模式失败';
          notifyListeners();
          return false;
        }
      } else {
        // 导航/巡检/循迹 → 需要 Lease + AUTONOMOUS
        final leaseOk = await client.ensureLease();
        if (!leaseOk) {
          _statusMessage = '获取控制租约失败（可能其他客户端正在控制）';
          notifyListeners();
          return false;
        }
        final modeOk = await client.setMode(RobotMode.ROBOT_MODE_AUTONOMOUS);
        if (!modeOk) {
          _statusMessage = '切换自主模式失败';
          notifyListeners();
          return false;
        }
      }

      final resp = await client.startTask(
        taskType: type,
        navigationParams: navigationParams,
        mappingParams: mappingParams,
        followPathParams: followPathParams,
      );

      if (resp.base.errorCode == ErrorCode.ERROR_CODE_OK) {
        _activeTaskId = resp.taskId;
        _activeTaskType = type;
        _isRunning = true;
        _isPaused = false;
        _progress = 0.0;
        _taskStatus = TaskStatus.TASK_STATUS_RUNNING;
        _statusMessage = '${_taskTypeLabel(type)}任务已启动';
        _startPolling();
        notifyListeners();
        return true;
      } else {
        _statusMessage = resp.base.errorMessage;
        notifyListeners();
        return false;
      }
    } catch (e) {
      _statusMessage = '$e';
      notifyListeners();
      return false;
    }
  }

  /// 暂停任务
  Future<bool> pauseTask() async {
    final client = _client;
    if (client == null || _activeTaskId == null) return false;

    try {
      await client.pauseTask(taskId: _activeTaskId!);
      _isPaused = true;
      notifyListeners();
      return true;
    } catch (e) {
      _statusMessage = '$e';
      notifyListeners();
      return false;
    }
  }

  /// 恢复任务
  Future<bool> resumeTask() async {
    final client = _client;
    if (client == null || _activeTaskId == null) return false;

    try {
      await client.resumeTask(taskId: _activeTaskId!);
      _isPaused = false;
      notifyListeners();
      return true;
    } catch (e) {
      _statusMessage = '$e';
      notifyListeners();
      return false;
    }
  }

  /// 取消任务
  Future<bool> cancelTask() async {
    final client = _client;
    if (client == null || _activeTaskId == null) return false;

    try {
      await client.cancelTask(taskId: _activeTaskId!);
      _isRunning = false;
      _isPaused = false;
      _stopPolling();
      _statusMessage = '任务已取消';
      notifyListeners();
      return true;
    } catch (e) {
      _statusMessage = '$e';
      notifyListeners();
      return false;
    }
  }

  // ================================================================
  // 状态轮询（内部）
  // ================================================================

  void _startPolling() {
    _stopPolling();
    _pollFailCount = 0;
    _statusTimer = Timer.periodic(const Duration(seconds: 2), (_) async {
      if (_activeTaskId == null || _client == null) return;
      try {
        final r = await _client!.getTaskStatus(taskId: _activeTaskId!);
        _progress = r.task.progressPercent;
        final prevStatus = _taskStatus;
        _taskStatus = r.task.status;

        if (_taskStatus == TaskStatus.TASK_STATUS_COMPLETED ||
            _taskStatus == TaskStatus.TASK_STATUS_FAILED ||
            _taskStatus == TaskStatus.TASK_STATUS_CANCELLED) {
          _isRunning = false;
          _isPaused = false;
          _stopPolling();

          // --- Cross-gateway callbacks ---
          if (prevStatus != _taskStatus) {
            // Mapping auto-save: trigger when mapping task completes
            if (_taskStatus == TaskStatus.TASK_STATUS_COMPLETED &&
                _activeTaskType == TaskType.TASK_TYPE_MAPPING) {
              onMappingComplete?.call();
            }
            // Generic terminal callback
            onTaskTerminated?.call(_activeTaskType, _taskStatus);
          }
        }
        _pollFailCount = 0; // 成功时重置
        notifyListeners();
      } catch (e) {
        _pollFailCount++;
        if (_pollFailCount == _pollWarnThreshold) {
          AppLogger.task.warning('Task polling failed $_pollFailCount times: $e');
        }
        if (_pollFailCount >= _pollErrorThreshold) {
          AppLogger.task.error(
              'Task polling failed $_pollFailCount consecutive times, '
              'status sync may be stale: $e');
          _statusMessage = '状态同步异常，请检查网络连接';
          notifyListeners();
        }
      }
    });
  }

  void _stopPolling() {
    _statusTimer?.cancel();
    _statusTimer = null;
  }

  String _taskTypeLabel(TaskType type) {
    switch (type) {
      case TaskType.TASK_TYPE_NAVIGATION:
        return '导航';
      case TaskType.TASK_TYPE_MAPPING:
        return '建图';
      case TaskType.TASK_TYPE_INSPECTION:
        return '巡检';
      case TaskType.TASK_TYPE_RETURN_HOME:
        return '回家';
      case TaskType.TASK_TYPE_FOLLOW_PATH:
        return '循迹';
      default:
        return '';
    }
  }

  @override
  void dispose() {
    _stopPolling();
    super.dispose();
  }
}

import 'dart:async';
import 'package:flutter/foundation.dart';
import 'package:flutter_monitor/core/grpc/robot_client_base.dart';
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

  // ================================================================
  // 业务方法
  // ================================================================

  /// 启动导航任务（统一入口）
  ///
  /// 所有导航任务应优先通过该方法发起，避免页面重复拼装参数。
  Future<bool> startNavigationTask(
    List<NavigationGoal> waypoints, {
    bool loop = false,
  }) async {
    if (waypoints.isEmpty) {
      _statusMessage = '请先设置导航目标';
      notifyListeners();
      return false;
    }
    final params = NavigationParams()
      ..waypoints.addAll(waypoints)
      ..loop = loop;
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
  }) async {
    final goal = NavigationGoal()
      ..position = (Vector3()
        ..x = x
        ..y = y
        ..z = z)
      ..yaw = yaw
      ..arrivalRadius = arrivalRadius
      ..label = label;
    return startNavigationTask([goal]);
  }

  /// Whether current task execution should block cold OTA actions.
  bool get blocksColdOta => _isRunning;

  /// 启动任务
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
      // 建图模式需要先切换到 MAPPING mode
      if (type == TaskType.TASK_TYPE_MAPPING) {
        await client.setMode(RobotMode.ROBOT_MODE_MAPPING);
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
        notifyListeners();
      } catch (_) {
        // 网络抖动时静默忽略，下次轮询重试
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

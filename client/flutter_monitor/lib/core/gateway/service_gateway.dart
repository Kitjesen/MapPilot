import 'dart:async';
import 'package:flutter/foundation.dart';
import 'package:flutter_monitor/core/grpc/robot_client_base.dart';
import 'package:flutter_monitor/core/services/app_logger.dart';
import 'package:robot_proto/src/data.pb.dart';
import 'package:robot_proto/src/common.pb.dart';

/// 服务管理网关：统一管理建图/导航 systemd 服务的生命周期。
///
/// 核心职责:
///   - 启动/停止 mapping.service 和 navigation.service (通过 OTA ManageService)
///   - 互斥保证: 建图和导航不能同时运行 (共用 LiDAR)
///   - 服务状态轮询
///
/// UI 通过 `context.watch<ServiceGateway>()` 读取状态、调用方法。
class ServiceGateway extends ChangeNotifier {
  RobotClientBase? _client;

  static const String kMappingService = 'mapping.service';
  static const String kNavigationService = 'navigation.service';

  ServiceGateway({RobotClientBase? client}) : _client = client;

  void updateClient(RobotClientBase? client) {
    _client = client;
    if (client == null) {
      _stopPolling();
      _activeService = null;
      _mappingStatus = null;
      _navigationStatus = null;
      _isTransitioning = false;
      _error = null;
    }
    notifyListeners();
  }

  // ================================================================
  // 状态
  // ================================================================

  /// 当前活跃的服务: 'mapping' | 'navigation' | null
  String? _activeService;
  ServiceStatus? _mappingStatus;
  ServiceStatus? _navigationStatus;
  bool _isTransitioning = false;
  String? _error;
  Timer? _pollTimer;

  String? get activeService => _activeService;
  ServiceStatus? get mappingStatus => _mappingStatus;
  ServiceStatus? get navigationStatus => _navigationStatus;
  bool get isTransitioning => _isTransitioning;
  String? get error => _error;

  bool get isMappingRunning =>
      _mappingStatus?.state == 'active' || _activeService == 'mapping';
  bool get isNavigationRunning =>
      _navigationStatus?.state == 'active' || _activeService == 'navigation';
  bool get isAnyServiceRunning => isMappingRunning || isNavigationRunning;

  // ================================================================
  // 服务启动/停止
  // ================================================================

  /// 启动建图服务 — 自动停止导航 (互斥)
  Future<bool> startMapping() async {
    final client = _client;
    if (client == null) {
      _error = '未连接机器人';
      notifyListeners();
      return false;
    }

    _isTransitioning = true;
    _error = null;
    notifyListeners();

    try {
      // 互斥: 先停导航
      if (isNavigationRunning) {
        AppLogger.system.info('停止导航服务 (互斥)...');
        await _stopService(client, kNavigationService);
        await Future.delayed(const Duration(seconds: 1));
      }

      // 启动建图
      AppLogger.system.info('启动建图服务...');
      final resp = await client.manageService(
        serviceName: kMappingService,
        action: ServiceAction.SERVICE_ACTION_START,
      );

      if (resp.success) {
        _activeService = 'mapping';
        _mappingStatus = resp.status;
        _startPolling();
        AppLogger.system.info('建图服务已启动');
      } else {
        _error = '启动建图失败: ${resp.message}';
        AppLogger.system.error(_error!);
      }

      _isTransitioning = false;
      notifyListeners();
      return resp.success;
    } catch (e) {
      _error = '启动建图异常: $e';
      _isTransitioning = false;
      AppLogger.system.error(_error!);
      notifyListeners();
      return false;
    }
  }

  /// 停止建图服务
  Future<bool> stopMapping() async {
    final client = _client;
    if (client == null) return false;

    _isTransitioning = true;
    _error = null;
    notifyListeners();

    try {
      final success = await _stopService(client, kMappingService);
      if (success) {
        _activeService = null;
        _mappingStatus = null;
        AppLogger.system.info('建图服务已停止');
      }
      _isTransitioning = false;
      notifyListeners();
      return success;
    } catch (e) {
      _error = '停止建图异常: $e';
      _isTransitioning = false;
      AppLogger.system.error(_error!);
      notifyListeners();
      return false;
    }
  }

  /// 启动导航服务 — 自动停止建图 (互斥)
  Future<bool> startNavigation() async {
    final client = _client;
    if (client == null) {
      _error = '未连接机器人';
      notifyListeners();
      return false;
    }

    _isTransitioning = true;
    _error = null;
    notifyListeners();

    try {
      // 互斥: 先停建图
      if (isMappingRunning) {
        AppLogger.system.info('停止建图服务 (互斥)...');
        await _stopService(client, kMappingService);
        await Future.delayed(const Duration(seconds: 1));
      }

      // 启动导航
      AppLogger.system.info('启动导航服务...');
      final resp = await client.manageService(
        serviceName: kNavigationService,
        action: ServiceAction.SERVICE_ACTION_START,
      );

      if (resp.success) {
        _activeService = 'navigation';
        _navigationStatus = resp.status;
        _startPolling();
        AppLogger.system.info('导航服务已启动');
      } else {
        _error = '启动导航失败: ${resp.message}';
        AppLogger.system.error(_error!);
      }

      _isTransitioning = false;
      notifyListeners();
      return resp.success;
    } catch (e) {
      _error = '启动导航异常: $e';
      _isTransitioning = false;
      AppLogger.system.error(_error!);
      notifyListeners();
      return false;
    }
  }

  /// 停止导航服务
  Future<bool> stopNavigation() async {
    final client = _client;
    if (client == null) return false;

    _isTransitioning = true;
    _error = null;
    notifyListeners();

    try {
      final success = await _stopService(client, kNavigationService);
      if (success) {
        _activeService = null;
        _navigationStatus = null;
        AppLogger.system.info('导航服务已停止');
      }
      _isTransitioning = false;
      notifyListeners();
      return success;
    } catch (e) {
      _error = '停止导航异常: $e';
      _isTransitioning = false;
      AppLogger.system.error(_error!);
      notifyListeners();
      return false;
    }
  }

  /// 停止所有服务
  Future<void> stopAll() async {
    final client = _client;
    if (client == null) return;

    if (isMappingRunning) await _stopService(client, kMappingService);
    if (isNavigationRunning) await _stopService(client, kNavigationService);
    _activeService = null;
    _mappingStatus = null;
    _navigationStatus = null;
    _stopPolling();
    notifyListeners();
  }

  // ================================================================
  // 状态查询
  // ================================================================

  /// 刷新两个服务的状态 (通过 ManageService + status action)
  Future<void> refreshStatus() async {
    final client = _client;
    if (client == null) return;

    try {
      // 查询建图服务状态
      final mappingResp = await client.manageService(
        serviceName: kMappingService,
        action: ServiceAction.SERVICE_ACTION_STATUS,
      );
      _mappingStatus = mappingResp.status;

      // 查询导航服务状态
      final navResp = await client.manageService(
        serviceName: kNavigationService,
        action: ServiceAction.SERVICE_ACTION_STATUS,
      );
      _navigationStatus = navResp.status;

      // 更新 activeService
      if (_mappingStatus?.state == 'active') {
        _activeService = 'mapping';
      } else if (_navigationStatus?.state == 'active') {
        _activeService = 'navigation';
      } else {
        _activeService = null;
      }

      notifyListeners();
    } catch (e) {
      AppLogger.system.error('刷新服务状态失败: $e');
    }
  }

  // ================================================================
  // 内部方法
  // ================================================================

  Future<bool> _stopService(RobotClientBase client, String serviceName) async {
    try {
      final resp = await client.manageService(
        serviceName: serviceName,
        action: ServiceAction.SERVICE_ACTION_STOP,
      );
      return resp.success;
    } catch (e) {
      _error = '停止 $serviceName 失败: $e';
      AppLogger.system.error(_error!);
      return false;
    }
  }

  void _startPolling() {
    _stopPolling();
    _pollTimer = Timer.periodic(const Duration(seconds: 3), (_) {
      refreshStatus();
    });
  }

  void _stopPolling() {
    _pollTimer?.cancel();
    _pollTimer = null;
  }

  void clearError() {
    _error = null;
    notifyListeners();
  }

  @override
  void dispose() {
    _stopPolling();
    super.dispose();
  }
}

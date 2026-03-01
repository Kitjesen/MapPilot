import 'package:flutter/foundation.dart';
import 'package:grpc/grpc.dart';
import 'package:flutter_monitor/core/grpc/robot_client_base.dart';
import 'package:flutter_monitor/core/services/app_logger.dart';
import 'package:robot_proto/robot_proto.dart';

/// 系统服务网关：查询 systemd 服务状态 + 一键重启。
///
/// 通过 OTA daemon (port 50052) 的 GetDeviceInfo / ManageService RPC 实现。
class SystemGateway extends ChangeNotifier {
  RobotClientBase? _client;

  SystemGateway({RobotClientBase? client}) : _client = client;

  void updateClient(RobotClientBase? client) {
    _client = client;
    if (client == null) {
      _services = [];
      _error = null;
      _loading = false;
    }
    notifyListeners();
  }

  // ── State ──

  List<ServiceStatus> _services = [];
  bool _loading = false;
  String? _error;
  /// 正在重启的服务名（用于 UI 进度指示）
  String? _restartingService;

  List<ServiceStatus> get services => _services;
  bool get loading => _loading;
  String? get error => _error;
  String? get restartingService => _restartingService;

  /// 硬编码的关注服务列表（与 systemd 单元名对应）
  static const knownServices = [
    'nav-slam',
    'nav-planning',
    'nav-autonomy',
    'nav-grpc',
    'nav-lidar',
    'ota-daemon',
  ];

  // ── Actions ──

  /// 从 GetDeviceInfo 拉取全部服务状态
  Future<void> fetchServiceStatuses() async {
    final client = _client;
    if (client == null) return;
    if (!client.otaAvailable) {
      _error = '需要固件 v1.6+ 支持此功能';
      notifyListeners();
      return;
    }

    _loading = true;
    _error = null;
    notifyListeners();

    try {
      final resp = await client.getDeviceInfo();
      _services = List<ServiceStatus>.from(resp.services);
      _error = null;
    } on GrpcError catch (e) {
      if (e.code == StatusCode.unimplemented) {
        _error = '需要固件 v1.6+ 支持此功能';
      } else {
        _error = 'gRPC 错误: ${e.message}';
      }
      AppLogger.system.warning('fetchServiceStatuses failed: $e');
    } catch (e) {
      _error = '查询失败: $e';
      AppLogger.system.warning('fetchServiceStatuses failed: $e');
    } finally {
      _loading = false;
      notifyListeners();
    }
  }

  /// 重启指定服务。返回是否成功。
  Future<bool> restartService(String serviceName) async {
    final client = _client;
    if (client == null) return false;

    _restartingService = serviceName;
    notifyListeners();

    try {
      final resp = await client.manageService(
        serviceName: serviceName,
        action: ServiceAction.SERVICE_ACTION_RESTART,
      );

      if (resp.success) {
        // 更新本地缓存中该服务的状态
        if (resp.hasStatus()) {
          final idx = _services.indexWhere((s) => s.name == serviceName);
          if (idx >= 0) {
            _services[idx] = resp.status;
          } else {
            _services.add(resp.status);
          }
        }
        // 延迟后刷新全部状态（等服务启动）
        Future.delayed(const Duration(seconds: 2), fetchServiceStatuses);
        return true;
      } else {
        _error = resp.message.isNotEmpty ? resp.message : '重启失败';
        return false;
      }
    } on GrpcError catch (e) {
      if (e.code == StatusCode.unimplemented) {
        _error = '需要固件 v1.6+ 支持此功能';
      } else {
        _error = '重启失败: ${e.message}';
      }
      AppLogger.system.warning('restartService($serviceName) failed: $e');
      return false;
    } catch (e) {
      _error = '重启失败: $e';
      AppLogger.system.warning('restartService($serviceName) failed: $e');
      return false;
    } finally {
      _restartingService = null;
      notifyListeners();
    }
  }
}

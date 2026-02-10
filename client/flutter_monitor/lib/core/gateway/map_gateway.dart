import 'dart:typed_data';

import 'package:flutter/foundation.dart';
import 'package:flutter_monitor/core/grpc/robot_client_base.dart';
import 'package:robot_proto/src/common.pb.dart';
import 'package:robot_proto/src/system.pb.dart';

/// 地图管理网关：封装地图 CRUD + relocalize 业务逻辑。
///
/// UI 通过 `context.watch<MapGateway>()` 读取状态、调用方法，
/// 不再直接操作 [RobotClientBase]。
class MapGateway extends ChangeNotifier {
  RobotClientBase? _client;

  MapGateway({RobotClientBase? client}) : _client = client;

  /// 更新底层 gRPC 客户端（连接/断开时由 Provider 调用）
  void updateClient(RobotClientBase? client) {
    _client = client;
    if (client == null) {
      _maps = [];
      _error = null;
    }
    notifyListeners();
  }

  // ================================================================
  // 状态
  // ================================================================

  List<MapInfo> _maps = [];
  bool _isLoading = false;
  bool _isDownloading = false;
  double _downloadProgress = 0.0;
  String? _error;

  List<MapInfo> get maps => _maps;
  bool get isLoading => _isLoading;
  bool get isDownloading => _isDownloading;
  double get downloadProgress => _downloadProgress;
  String? get error => _error;

  // ================================================================
  // 业务方法
  // ================================================================

  /// 加载/刷新地图列表
  Future<void> refreshMaps() async {
    final client = _client;
    if (client == null) {
      _error = '未连接';
      notifyListeners();
      return;
    }

    _isLoading = true;
    _error = null;
    notifyListeners();

    try {
      final r = await client.listMaps();
      if (r.base.errorCode == ErrorCode.ERROR_CODE_OK) {
        _maps = r.maps.toList()
          ..sort((a, b) => b.modifiedAt.compareTo(a.modifiedAt));
        _error = null;
      } else {
        _error = r.base.errorMessage;
      }
    } catch (e) {
      _error = '$e';
    }

    _isLoading = false;
    notifyListeners();
  }

  /// 保存当前地图
  ///
  /// 返回 (success, message) 元组。
  Future<(bool, String)> saveMap(String filePath) async {
    final client = _client;
    if (client == null) return (false, '未连接');

    try {
      final r = await client.saveMap(filePath: filePath);
      if (r.success) {
        // 保存成功后自动刷新列表
        await refreshMaps();
      }
      return (r.success, r.success ? '已保存' : r.message);
    } catch (e) {
      return (false, '$e');
    }
  }

  /// 删除地图
  Future<(bool, String)> deleteMap(String path) async {
    final client = _client;
    if (client == null) return (false, '未连接');

    try {
      final r = await client.deleteMap(path: path);
      if (r.success) {
        await refreshMaps();
      }
      return (r.success, r.success ? '已删除' : r.message);
    } catch (e) {
      return (false, '$e');
    }
  }

  /// 重命名地图
  Future<(bool, String)> renameMap(String oldPath, String newName) async {
    final client = _client;
    if (client == null) return (false, '未连接');

    try {
      final r = await client.renameMap(oldPath: oldPath, newName: newName);
      if (r.success) {
        await refreshMaps();
      }
      return (r.success, r.success ? '已重命名' : r.message);
    } catch (e) {
      return (false, '$e');
    }
  }

  /// 重定位（加载地图并设置初始位姿）
  Future<(bool, String)> relocalize(
    String pcdPath, {
    double x = 0,
    double y = 0,
    double z = 0,
    double yaw = 0,
  }) async {
    final client = _client;
    if (client == null) return (false, '未连接');

    try {
      final r = await client.relocalize(
        pcdPath: pcdPath,
        x: x,
        y: y,
        z: z,
        yaw: yaw,
      );
      return (r.success, r.success ? '已加载' : r.message);
    } catch (e) {
      return (false, '$e');
    }
  }

  /// 从机器人下载地图文件
  ///
  /// 返回文件字节内容，失败返回 null。
  /// [remotePath] 机器人上的绝对路径（如 `/maps/xxx.pcd`）。
  Future<Uint8List?> downloadMap(String remotePath) async {
    final client = _client;
    if (client == null) return null;

    _isDownloading = true;
    _downloadProgress = 0.0;
    notifyListeners();

    try {
      final chunks = <int>[];
      int totalSize = 0;

      await for (final chunk in client.downloadFile(filePath: remotePath)) {
        chunks.addAll(chunk.data);
        if (chunk.totalSize.toInt() > 0) totalSize = chunk.totalSize.toInt();
        if (totalSize > 0) {
          _downloadProgress = chunks.length / totalSize;
          notifyListeners();
        }
        if (chunk.isLast) break;
      }

      _isDownloading = false;
      _downloadProgress = 1.0;
      notifyListeners();
      return Uint8List.fromList(chunks);
    } catch (e) {
      _isDownloading = false;
      _downloadProgress = 0.0;
      notifyListeners();
      return null;
    }
  }
}

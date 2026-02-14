import 'dart:io';

import 'package:flutter/foundation.dart';
import 'package:flutter_monitor/core/grpc/robot_client_base.dart';
import 'package:robot_proto/src/common.pb.dart';
import 'package:robot_proto/src/system.pb.dart';

/// 地图管理网关：封装地图 CRUD + relocalize 业务逻辑。
///
/// UI 通过 `context.watch<MapGateway>()` 读取状态、调用方法，
/// 不再直接操作 [RobotClientBase]。
///
/// 高频下载进度通过独立的 [downloadProgressNotifier] 发布，
/// 避免整个列表跟着重建。
class MapGateway extends ChangeNotifier {
  RobotClientBase? _client;

  MapGateway({RobotClientBase? client}) : _client = client;

  /// 更新底层 gRPC 客户端（连接/断开时由 Provider 调用）
  void updateClient(RobotClientBase? client) {
    _client = client;
    if (client == null) {
      _maps = [];
      _error = null;
      _isDownloading = false;
    }
    notifyListeners();
  }

  // ================================================================
  // 状态
  // ================================================================

  List<MapInfo> _maps = [];
  bool _isLoading = false;
  bool _isDownloading = false;
  String? _error;

  /// 用于跟踪 refresh 并发的代际计数器
  int _refreshGeneration = 0;

  List<MapInfo> get maps => _maps;
  bool get isLoading => _isLoading;
  bool get isDownloading => _isDownloading;
  String? get error => _error;

  /// 独立的下载进度通知器 — UI 应使用 [ValueListenableBuilder] 监听，
  /// 而非 context.watch，避免 1600 次/下载 的全页面重建。
  final ValueNotifier<double> downloadProgressNotifier = ValueNotifier(0.0);

  @override
  void dispose() {
    downloadProgressNotifier.dispose();
    super.dispose();
  }

  // ================================================================
  // 业务方法
  // ================================================================

  /// 加载/刷新地图列表。
  ///
  /// 内置并发保护：如果正在刷新，后续调用直接返回。
  /// 代际计数器确保旧的异步结果不会覆盖新的。
  Future<void> refreshMaps() async {
    final client = _client;
    if (client == null) {
      _error = '未连接';
      notifyListeners();
      return;
    }

    // 并发保护
    if (_isLoading) return;

    final generation = ++_refreshGeneration;

    _isLoading = true;
    _error = null;
    notifyListeners();

    try {
      final r = await client.listMaps();
      // 旧代际的结果丢弃
      if (generation != _refreshGeneration) return;
      if (r.base.errorCode == ErrorCode.ERROR_CODE_OK) {
        _maps = r.maps.toList()
          ..sort((a, b) => b.modifiedAt.compareTo(a.modifiedAt));
        _error = null;
      } else {
        _error = r.base.errorMessage;
      }
    } catch (e) {
      if (generation != _refreshGeneration) return;
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

  /// 检查给定文件名是否已存在于本地缓存的地图列表中。
  ///
  /// 用于 SaveMap 前的客户端侧冲突预检。
  bool mapNameExists(String fileName) {
    return _maps.any((m) => m.name == fileName);
  }

  /// 判断地图文件是否可用于 Relocalize（只有 PCD 可以）。
  static bool canRelocalize(MapInfo m) {
    return m.name.toLowerCase().endsWith('.pcd');
  }

  /// 从机器人下载地图文件 — 流式写入本地文件，避免大文件 OOM。
  ///
  /// [remotePath] 机器人上的绝对路径（如 `/maps/xxx.pcd`）。
  /// [localFile] 本地写入目标。
  /// 返回写入的字节总数，失败返回 null。
  ///
  /// 进度更新发布在 [downloadProgressNotifier]（不触发 notifyListeners）。
  Future<int?> downloadMapToFile(String remotePath, File localFile) async {
    final client = _client;
    if (client == null) return null;

    _isDownloading = true;
    downloadProgressNotifier.value = 0.0;
    notifyListeners();

    IOSink? sink;
    try {
      sink = localFile.openWrite();
      int receivedBytes = 0;
      int totalSize = 0;

      await for (final chunk in client.downloadFile(filePath: remotePath)) {
        sink.add(chunk.data);
        receivedBytes += chunk.data.length;
        if (chunk.totalSize.toInt() > 0) totalSize = chunk.totalSize.toInt();
        if (totalSize > 0) {
          downloadProgressNotifier.value = receivedBytes / totalSize;
        }
        if (chunk.isLast) break;
      }

      await sink.flush();
      await sink.close();
      sink = null;

      _isDownloading = false;
      downloadProgressNotifier.value = 1.0;
      notifyListeners();
      return receivedBytes;
    } catch (e) {
      await sink?.close();
      // 清理不完整的文件，避免留下损坏数据
      try {
        if (await localFile.exists()) {
          await localFile.delete();
        }
      } catch (_) {}
      _isDownloading = false;
      downloadProgressNotifier.value = 0.0;
      notifyListeners();
      return null;
    }
  }
}

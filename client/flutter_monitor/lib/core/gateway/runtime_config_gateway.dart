import 'dart:async';
import 'dart:convert';
import 'package:flutter/foundation.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:flutter_monitor/core/grpc/robot_client_base.dart';
import 'package:flutter_monitor/core/models/runtime_config.dart';
import 'package:flutter_monitor/core/services/app_logger.dart';
import 'package:robot_proto/src/common.pb.dart';

/// RuntimeConfigGateway — 运行参数管理（在线推送 + 本地持久化）。
///
/// 工作模式:
/// 1. **离线**: 加载本地 SharedPreferences 缓存的配置
/// 2. **在线**: 从后端 GetRuntimeConfig 拉取 → 用户修改 → SetRuntimeConfig 推送
/// 3. **回读**: SlowState 心跳自动携带 runtime_config，gateway 据此刷新
///
/// UI 通过 `context.watch<RuntimeConfigGateway>()` 订阅变化。
class RuntimeConfigGateway extends ChangeNotifier {
  RobotClientBase? _client;

  RuntimeConfig _config = RuntimeConfig();
  RuntimeConfig? _serverConfig;   // 服务端确认的配置 (null=未连接)
  int _configVersion = 0;
  bool _syncing = false;
  bool _dirty = false;            // 本地有未推送的修改
  String? _error;
  List<String> _lastClampedFields = [];

  // ── Public getters ──
  RuntimeConfig get config => _config;
  RuntimeConfig? get serverConfig => _serverConfig;
  int get configVersion => _configVersion;
  bool get isSyncing => _syncing;
  bool get isDirty => _dirty;
  bool get isOnline => _client != null;
  String? get error => _error;
  List<String> get lastClampedFields => _lastClampedFields;

  static const _cacheKey = 'runtime_config_json';

  RuntimeConfigGateway() {
    _loadLocal();
  }

  /// 注入/更新 gRPC 客户端引用
  void updateClient(RobotClientBase? client) {
    _client = client;
    _error = null;

    if (client != null) {
      // 连接时从后端拉取
      fetchFromServer();
    } else {
      _serverConfig = null;
      _dirty = false;
    }

    notifyListeners();
  }

  // ══════════════════════════════════════════════════════════
  //  LOCAL PERSISTENCE
  // ══════════════════════════════════════════════════════════

  Future<void> _loadLocal() async {
    try {
      final prefs = await SharedPreferences.getInstance();
      final json = prefs.getString(_cacheKey);
      if (json != null && json.isNotEmpty) {
        _config = RuntimeConfig.fromJsonString(json);
        notifyListeners();
      }
    } catch (e) {
      AppLogger.system.warning('Failed to load local runtime config: $e');
    }
  }

  Future<void> _saveLocal() async {
    try {
      final prefs = await SharedPreferences.getInstance();
      await prefs.setString(_cacheKey, _config.toJsonString());
    } catch (e) {
      AppLogger.system.warning('Failed to save local runtime config: $e');
    }
  }

  // ══════════════════════════════════════════════════════════
  //  SERVER SYNC
  // ══════════════════════════════════════════════════════════

  /// 从后端拉取当前配置
  Future<void> fetchFromServer() async {
    if (_client == null) return;

    _syncing = true;
    _error = null;
    notifyListeners();

    try {
      final response = await _client!.getRuntimeConfig();
      if (response.base.errorCode == ErrorCode.ERROR_CODE_OK &&
          response.configJson.isNotEmpty) {
        final json = jsonDecode(response.configJson) as Map<String, dynamic>;
        _config = RuntimeConfig.fromJson(json);
        _configVersion = response.configVersion.toInt();
        _serverConfig = _config.copyWith();
        _dirty = false;
        await _saveLocal();
        AppLogger.system.info(
          'Runtime config fetched from server (version=$_configVersion)',
        );
      } else {
        // 服务端暂无配置 → 使用本地缓存
        _serverConfig = _config.copyWith();
        _dirty = false;
        AppLogger.system.info(
          'Server returned empty config, using local cache',
        );
      }
      _syncing = false;
      notifyListeners();
    } catch (e) {
      _syncing = false;
      _error = '获取配置失败: $e';
      notifyListeners();
      AppLogger.system.warning('Failed to fetch runtime config: $e');
    }
  }

  /// 将当前配置推送到后端
  Future<bool> pushToServer() async {
    if (_client == null) {
      _error = '未连接机器人';
      notifyListeners();
      return false;
    }

    _syncing = true;
    _error = null;
    _lastClampedFields = [];
    notifyListeners();

    try {
      final changedFields = _serverConfig != null
          ? _diffFields(_config, _serverConfig!)
          : <String>[];

      final response = await _client!.setRuntimeConfig(
        configJson: jsonEncode(_config.toJson()),
        expectedVersion: _configVersion,
        changedFields: changedFields,
      );

      if (response.base.errorCode == ErrorCode.ERROR_CODE_OK) {
        // 服务端返回了实际应用的配置 (可能 clamp 了某些值)
        if (response.appliedConfigJson.isNotEmpty) {
          final applied = jsonDecode(response.appliedConfigJson) as Map<String, dynamic>;
          _config = RuntimeConfig.fromJson(applied);
        }
        _configVersion = response.newVersion.toInt();
        _lastClampedFields = List<String>.from(response.clampedFields);
        _serverConfig = _config.copyWith();
        _dirty = false;
        await _saveLocal();
      } else {
        _error = response.base.errorMessage;
      }

      _syncing = false;
      notifyListeners();

      AppLogger.system.info(
        'Runtime config pushed (version=$_configVersion, '
        'changed=${changedFields.length} fields, '
        'clamped=${_lastClampedFields.length})',
      );
      return response.base.errorCode == ErrorCode.ERROR_CODE_OK;
    } catch (e) {
      _syncing = false;
      _error = '推送配置失败: $e';
      notifyListeners();
      AppLogger.system.warning('Failed to push runtime config: $e');
      return false;
    }
  }

  // ══════════════════════════════════════════════════════════
  //  HEARTBEAT READ-BACK
  // ══════════════════════════════════════════════════════════

  /// 从 SlowState 心跳更新配置 (被 RobotConnectionProvider 调用)
  void updateFromHeartbeat(RuntimeConfig config, int version) {
    if (version > _configVersion) {
      _config = config;
      _serverConfig = config.copyWith();
      _configVersion = version;
      _dirty = false;
      _saveLocal();
      notifyListeners();
    }
  }

  // ══════════════════════════════════════════════════════════
  //  PARAMETER MODIFICATION (from UI)
  // ══════════════════════════════════════════════════════════

  /// 修改单个参数并标记 dirty (通过 JSON round-trip 实现通用更新)
  void updateParam(String fieldName, dynamic value) {
    final json = _config.toJson();
    if (!json.containsKey(fieldName)) return;
    json[fieldName] = value;
    _config = RuntimeConfig.fromJson(json);
    _dirty = true;
    _saveLocal();
    notifyListeners();
  }

  /// 批量更新并推送
  Future<bool> updateAndPush(RuntimeConfig newConfig) async {
    _config = newConfig;
    _dirty = true;
    notifyListeners();
    return pushToServer();
  }

  /// 获取指定字段的当前值 (通过 JSON 映射)
  dynamic getParamValue(String fieldName) {
    return _config.toJson()[fieldName];
  }

  /// 重置为默认值
  void resetToDefaults() {
    _config = RuntimeConfig();
    _dirty = true;
    _saveLocal();
    notifyListeners();
  }

  /// 从服务端重新加载 (丢弃本地修改)
  Future<void> revertToServer() async {
    if (_serverConfig != null) {
      _config = _serverConfig!.copyWith();
      _dirty = false;
      _saveLocal();
      notifyListeners();
    } else {
      await fetchFromServer();
    }
  }

  /// 计算两个配置之间的差异字段列表
  static List<String> _diffFields(RuntimeConfig a, RuntimeConfig b) {
    final ja = a.toJson();
    final jb = b.toJson();
    final changed = <String>[];
    for (final key in ja.keys) {
      if (ja[key] != jb[key]) changed.add(key);
    }
    return changed;
  }
}

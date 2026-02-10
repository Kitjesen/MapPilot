import 'dart:async';
import 'package:flutter/foundation.dart';
import 'package:flutter_monitor/core/grpc/robot_client_base.dart';
import 'package:flutter_monitor/core/grpc/dog_direct_client.dart';
import 'package:flutter_monitor/core/storage/settings_preferences.dart';
import 'package:flutter_monitor/core/providers/robot_profile_provider.dart';
import 'package:robot_proto/robot_proto.dart';

/// 连接状态枚举
enum ConnectionStatus {
  disconnected,
  connecting,
  connected,
  reconnecting,
  error,
}

/// 集中式状态管理 Provider
/// 统一管理连接状态、租约状态、数据流缓存
///
/// 支持双连接模式:
///   - Primary: Nav Board (RobotClient, robot::v1 协议, :50051)
///   - Secondary: Dog Board (DogDirectClient, han_dog CMS 协议, :13145)
/// 两个连接完全独立，可同时使用或单独使用。
class RobotConnectionProvider extends ChangeNotifier {
  RobotClientBase? _client;
  ConnectionStatus _status = ConnectionStatus.disconnected;
  String? _errorMessage;

  // ─── Dog Board 直连 (secondary connection) ───
  DogDirectClient? _dogClient;

  // — Lease state —
  bool _hasLease = false;

  // — Cached latest state (shared across screens) —
  FastState? _latestFastState;
  SlowState? _latestSlowState;

  // — Stream subscriptions (centralized) —
  StreamSubscription<FastState>? _fastStateSub;
  StreamSubscription<SlowState>? _slowStateSub;

  // — Stream controllers for broadcasting to multiple listeners —
  final StreamController<FastState> _fastStateBroadcast =
      StreamController<FastState>.broadcast();
  final StreamController<SlowState> _slowStateBroadcast =
      StreamController<SlowState>.broadcast();

  // — Reconnect logic —
  Timer? _reconnectTimer;
  int _reconnectAttempts = 0;
  static const int _maxReconnectAttempts = 10;
  static const Duration _baseReconnectDelay = Duration(seconds: 2);

  // — Settings reference (optional, injected after construction) —
  SettingsPreferences? _settingsPrefs;
  RobotProfileProvider? _profileProvider;
  void bindSettings(SettingsPreferences prefs) => _settingsPrefs = prefs;
  void bindProfileProvider(RobotProfileProvider p) => _profileProvider = p;

  /// Called after a successful reconnection (not the initial connect).
  /// Used for cross-gateway orchestration: re-query deployment/task states.
  VoidCallback? onReconnected;

  // — Connection health —
  DateTime? _lastFastStateTime;
  Timer? _healthCheckTimer;
  static const Duration _healthTimeout = Duration(seconds: 10);

  // — Heartbeat / RTT —
  Timer? _heartbeatTimer;
  double? _lastRttMs;
  int _heartbeatFailCount = 0;
  static const Duration _heartbeatInterval = Duration(seconds: 5);
  static const int _heartbeatMaxFails = 3;

  // — Capabilities —
  CapabilitiesResponse? _capabilities;

  // — Available resources —
  ListResourcesResponse? _resources;

  // — Throttle for fast state notifications —
  DateTime _lastFastStateNotify = DateTime(2000);
  static const Duration _fastStateNotifyInterval = Duration(milliseconds: 100);

  // ============ Getters ============

  RobotClientBase? get client => _client;
  ConnectionStatus get status => _status;
  String? get errorMessage => _errorMessage;
  bool get hasLease => _hasLease;
  bool get isConnected => _status == ConnectionStatus.connected;

  FastState? get latestFastState => _latestFastState;
  SlowState? get latestSlowState => _latestSlowState;

  /// 广播流：多个 Widget 可以同时监听
  Stream<FastState> get fastStateStream => _fastStateBroadcast.stream;
  Stream<SlowState> get slowStateStream => _slowStateBroadcast.stream;

  // ─── Dog Board 直连 getters ───
  DogDirectClient? get dogClient => _dogClient;
  bool get isDogConnected => _dogClient?.isConnected ?? false;

  /// Latest measured RTT in milliseconds (null if no heartbeat yet).
  double? get connectionRttMs => _lastRttMs;

  /// OTA daemon (port 50052) 是否可用
  bool get otaAvailable => _client?.otaAvailable ?? false;

  /// Robot capabilities (null until fetched).
  CapabilitiesResponse? get capabilities => _capabilities;

  /// Available resources (cameras, pointclouds, etc.). Null until fetched.
  ListResourcesResponse? get resources => _resources;

  /// Convenience: list of available camera resource IDs.
  List<String> get availableCameras {
    if (_resources == null) return [];
    return _resources!.resources
        .where((r) => r.id.type == ResourceType.RESOURCE_TYPE_CAMERA && r.available)
        .map((r) => r.id.name)
        .toList();
  }

  // ============ Connection ============

  /// 设置客户端并连接
  Future<bool> connect(RobotClientBase client) async {
    _client = client;
    _status = ConnectionStatus.connecting;
    _errorMessage = null;
    _reconnectAttempts = 0;
    notifyListeners();

    try {
      final success = await client.connect();
      if (success) {
        _status = ConnectionStatus.connected;
        _startCentralStreams();
        _startHealthCheck();
        _startHeartbeat();
        _fetchCapabilities();
        _fetchResources();
        notifyListeners();
        return true;
      } else {
        _status = ConnectionStatus.error;
        _errorMessage = '无法连接到机器人';
        notifyListeners();
        return false;
      }
    } catch (e) {
      _status = ConnectionStatus.error;
      _errorMessage = '连接错误: $e';
      notifyListeners();
      return false;
    }
  }

  /// 断开所有连接 (Nav Board + Dog Board)
  Future<void> disconnect() async {
    _stopCentralStreams();
    _stopHealthCheck();
    _stopHeartbeat();
    _cancelReconnect();

    if (_hasLease) {
      try {
        await _client?.releaseLease();
      } catch (_) {}
      _hasLease = false;
    }

    try {
      await _client?.disconnect();
    } catch (_) {}

    // 同时断开 Dog Board
    await disconnectDog();

    _client = null;
    _status = ConnectionStatus.disconnected;
    _latestFastState = null;
    _latestSlowState = null;
    _errorMessage = null;
    notifyListeners();
  }

  // ============ Centralized Streams ============

  void _startCentralStreams() {
    _stopCentralStreams();

    if (_client == null || !_client!.isConnected) return;

    // FastState stream
    _fastStateSub = _client!.streamFastState(desiredHz: 10.0).listen(
      (state) {
        _latestFastState = state;
        _lastFastStateTime = DateTime.now();
        _fastStateBroadcast.add(state);

        // 节流通知：避免 10Hz 频率刷新 UI
        final now = DateTime.now();
        if (now.difference(_lastFastStateNotify) >= _fastStateNotifyInterval) {
          _lastFastStateNotify = now;
          notifyListeners();
        }
      },
      onError: (error) {
        debugPrint('[Provider] FastState stream error: $error');
        _handleStreamError();
      },
      onDone: () {
        debugPrint('[Provider] FastState stream done');
        _handleStreamError();
      },
    );

    // SlowState stream
    _slowStateSub = _client!.streamSlowState().listen(
      (state) {
        _latestSlowState = state;
        _slowStateBroadcast.add(state);
        notifyListeners();
      },
      onError: (error) {
        debugPrint('[Provider] SlowState stream error: $error');
      },
    );
  }

  void _stopCentralStreams() {
    _fastStateSub?.cancel();
    _fastStateSub = null;
    _slowStateSub?.cancel();
    _slowStateSub = null;
  }

  // ============ Lease Management ============

  Future<bool> acquireLease() async {
    if (_client == null || !isConnected) return false;

    try {
      final success = await _client!.acquireLease();
      _hasLease = success;
      notifyListeners();
      return success;
    } catch (e) {
      debugPrint('[Provider] Acquire lease failed: $e');
      return false;
    }
  }

  Future<void> releaseLease() async {
    if (_client == null) return;

    try {
      await _client!.releaseLease();
    } catch (e) {
      debugPrint('[Provider] Release lease failed: $e');
    }

    _hasLease = false;
    notifyListeners();
  }

  // ============ Auto-Reconnect ============

  void _handleStreamError() {
    if (_status == ConnectionStatus.reconnecting) return;

    // Check if auto-reconnect is enabled
    final autoReconnect = _settingsPrefs?.autoReconnect ?? true;
    if (!autoReconnect) {
      _status = ConnectionStatus.error;
      _errorMessage = '连接已断开（自动重连已关闭）';
      _stopCentralStreams();
      notifyListeners();
      return;
    }

    _status = ConnectionStatus.reconnecting;
    _stopCentralStreams();
    notifyListeners();
    _scheduleReconnect();
  }

  void _scheduleReconnect() {
    _cancelReconnect();

    if (_reconnectAttempts >= _maxReconnectAttempts) {
      _status = ConnectionStatus.error;
      _errorMessage = '重连失败，已达最大重试次数';
      notifyListeners();
      return;
    }

    // 指数退避: 2s, 4s, 8s, 16s ... 最大 30s
    final delay = Duration(
      milliseconds: (_baseReconnectDelay.inMilliseconds *
              (1 << _reconnectAttempts.clamp(0, 4)))
          .clamp(2000, 30000),
    );

    debugPrint('[Provider] Reconnect attempt ${_reconnectAttempts + 1} in ${delay.inSeconds}s');

    _reconnectTimer = Timer(delay, () async {
      _reconnectAttempts++;
      if (_client != null) {
        try {
          final success = await _client!.connect();
          if (success) {
            _status = ConnectionStatus.connected;
            _reconnectAttempts = 0;
            _startCentralStreams();
            _startHealthCheck();
            _startHeartbeat();
            notifyListeners();
            // Fire reconnection hook for cross-gateway state recovery
            onReconnected?.call();
            return;
          }
        } catch (_) {}
      }
      // 继续重连
      _scheduleReconnect();
    });
  }

  void _cancelReconnect() {
    _reconnectTimer?.cancel();
    _reconnectTimer = null;
  }

  // ============ Health Check ============

  void _startHealthCheck() {
    _stopHealthCheck();
    _healthCheckTimer = Timer.periodic(const Duration(seconds: 5), (_) {
      if (_lastFastStateTime != null &&
          DateTime.now().difference(_lastFastStateTime!) > _healthTimeout) {
        debugPrint('[Provider] Health check: no data for ${_healthTimeout.inSeconds}s');
        _handleStreamError();
      }
    });
  }

  void _stopHealthCheck() {
    _healthCheckTimer?.cancel();
    _healthCheckTimer = null;
  }

  // ============ Heartbeat (RTT measurement) ============

  void _startHeartbeat() {
    _stopHeartbeat();
    _heartbeatFailCount = 0;
    _heartbeatTimer = Timer.periodic(_heartbeatInterval, (_) => _sendHeartbeat());
  }

  void _stopHeartbeat() {
    _heartbeatTimer?.cancel();
    _heartbeatTimer = null;
    _lastRttMs = null;
  }

  Future<void> _sendHeartbeat() async {
    final c = _client;
    if (c == null || !c.isConnected) return;

    final start = DateTime.now();
    try {
      await c.heartbeat();
      final rtt = DateTime.now().difference(start).inMicroseconds / 1000.0;
      _lastRttMs = rtt;
      _heartbeatFailCount = 0;
      notifyListeners();
    } catch (_) {
      _heartbeatFailCount++;
      if (_heartbeatFailCount >= _heartbeatMaxFails) {
        debugPrint('[Provider] Heartbeat failed ${_heartbeatMaxFails}x → triggering reconnect');
        _handleStreamError();
      }
    }
  }

  // ============ Capabilities ============

  Future<void> _fetchCapabilities() async {
    final c = _client;
    if (c == null) return;
    try {
      _capabilities = await c.getCapabilities();
      notifyListeners();
    } catch (_) {
      // Server may not support GetCapabilities; gracefully ignore.
      _capabilities = null;
    }
  }

  // ============ Resources ============

  Future<void> _fetchResources() async {
    final c = _client;
    if (c == null) return;
    try {
      _resources = await c.listResources();
      notifyListeners();
    } catch (_) {
      _resources = null;
    }
  }

  // ============ Dog Board Direct Connection ============

  /// 连接到 Dog Board (可在 Nav Board 连接后调用, 也可独立使用)
  Future<bool> connectDog({required String host, int port = 13145}) async {
    // 断开已有连接
    await disconnectDog();

    _dogClient = DogDirectClient(host: host, port: port);
    _dogClient!.addListener(_onDogChanged);

    final success = await _dogClient!.connect();
    if (!success) {
      debugPrint('[Provider] Dog Board connection failed: ${_dogClient!.errorMessage}');
    } else {
      // Auto-detect robot profile from GetParams response
      final params = _dogClient!.robotParams;
      if (params != null && _profileProvider != null) {
        final typeName = params.robot.type.name;
        _profileProvider!.autoDetect(typeName);
      }
    }
    notifyListeners();
    return success;
  }

  /// 断开 Dog Board 连接
  Future<void> disconnectDog() async {
    _dogClient?.removeListener(_onDogChanged);
    await _dogClient?.disconnect();
    _dogClient?.dispose();
    _dogClient = null;
    notifyListeners();
  }

  void _onDogChanged() {
    notifyListeners();
  }

  // ============ Cleanup ============

  @override
  void dispose() {
    _stopCentralStreams();
    _stopHealthCheck();
    _stopHeartbeat();
    _cancelReconnect();
    _fastStateBroadcast.close();
    _slowStateBroadcast.close();
    _dogClient?.removeListener(_onDogChanged);
    _dogClient?.dispose();
    super.dispose();
  }
}

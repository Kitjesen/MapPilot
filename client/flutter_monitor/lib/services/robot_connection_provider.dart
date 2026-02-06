import 'dart:async';
import 'package:flutter/foundation.dart';
import 'robot_client_base.dart';
import '../generated/telemetry.pb.dart';

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
class RobotConnectionProvider extends ChangeNotifier {
  RobotClientBase? _client;
  ConnectionStatus _status = ConnectionStatus.disconnected;
  String? _errorMessage;

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

  // — Connection health —
  DateTime? _lastFastStateTime;
  Timer? _healthCheckTimer;
  static const Duration _healthTimeout = Duration(seconds: 10);

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

  /// 断开连接
  Future<void> disconnect() async {
    _stopCentralStreams();
    _stopHealthCheck();
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
            notifyListeners();
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

  // ============ Cleanup ============

  @override
  void dispose() {
    _stopCentralStreams();
    _stopHealthCheck();
    _cancelReconnect();
    _fastStateBroadcast.close();
    _slowStateBroadcast.close();
    super.dispose();
  }
}

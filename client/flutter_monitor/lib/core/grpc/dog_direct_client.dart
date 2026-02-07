import 'dart:async';
import 'package:flutter/foundation.dart';
import 'package:grpc/grpc.dart';
import 'package:han_dog_message/han_dog_message.dart' as dog;

/// Dog Board 直连客户端
///
/// 通过 han_dog CMS gRPC 协议直接与机器狗控制板通信。
/// 可与 Nav Board 连接(RobotClient) 同时使用，提供:
///   - 低延迟底层传感器流 (IMU, Joint)
///   - RL 推理历史流 (History)
///   - Nav Board 不可用时的降级直连控制
///
/// 架构:
///   Flutter App
///     ├── RobotClient     → Nav Board :50051  (导航/遥测/地图)
///     └── DogDirectClient → Dog Board :13145  (底层传感器/直连控制)  ← 本类
class DogDirectClient extends ChangeNotifier {
  final String host;
  final int port;

  ClientChannel? _channel;
  dog.CmsClient? _stub;
  bool _connected = false;
  bool _standing = false;
  String? _errorMessage;

  // ─── 传感器数据缓存 ───
  dog.Imu? _latestImu;
  dog.Joint? _latestJoint;
  dog.History? _latestHistory;
  dog.Params? _robotParams;

  // ─── 流订阅 ───
  StreamSubscription? _imuSub;
  StreamSubscription? _jointSub;
  StreamSubscription? _historySub;

  // ─── 广播流 (多个 Widget 可监听) ───
  final _imuController = StreamController<dog.Imu>.broadcast();
  final _jointController = StreamController<dog.Joint>.broadcast();
  final _historyController = StreamController<dog.History>.broadcast();

  DogDirectClient({required this.host, this.port = 13145});

  // ============ Getters ============

  bool get isConnected => _connected;
  bool get isStanding => _standing;
  String? get errorMessage => _errorMessage;

  dog.Imu? get latestImu => _latestImu;
  dog.Joint? get latestJoint => _latestJoint;
  dog.History? get latestHistory => _latestHistory;
  dog.Params? get robotParams => _robotParams;

  Stream<dog.Imu> get imuStream => _imuController.stream;
  Stream<dog.Joint> get jointStream => _jointController.stream;
  Stream<dog.History> get historyStream => _historyController.stream;

  // ============ Connection ============

  /// 连接到 Dog Board CMS gRPC
  Future<bool> connect() async {
    try {
      _errorMessage = null;
      _channel = ClientChannel(
        host,
        port: port,
        options: const ChannelOptions(
          credentials: ChannelCredentials.insecure(),
        ),
      );

      _stub = dog.CmsClient(_channel!);

      // 测试连接: 获取参数
      _robotParams = await _stub!.getParams(
        dog.Empty(),
        options: CallOptions(timeout: const Duration(seconds: 3)),
      );

      _connected = true;
      debugPrint('[DogDirect] Connected to $host:$port '
          '(robot type: ${_robotParams?.robot.type})');
      notifyListeners();

      // 自动启动传感器流
      _startStreams();

      return true;
    } catch (e) {
      _connected = false;
      _errorMessage = '无法连接到机器狗: $e';
      debugPrint('[DogDirect] Connection failed: $e');
      notifyListeners();
      return false;
    }
  }

  /// 断开连接
  Future<void> disconnect() async {
    _stopStreams();
    try {
      await _channel?.shutdown();
    } catch (_) {}
    _channel = null;
    _stub = null;
    _connected = false;
    _standing = false;
    _latestImu = null;
    _latestJoint = null;
    _latestHistory = null;
    _errorMessage = null;
    notifyListeners();
  }

  // ============ Motor Control ============

  /// 使能电机
  Future<bool> enable() async {
    if (!_connected) return false;
    try {
      await _stub!.enable(dog.Empty());
      debugPrint('[DogDirect] Motors ENABLED');
      return true;
    } catch (e) {
      _errorMessage = 'Enable failed: $e';
      notifyListeners();
      return false;
    }
  }

  /// 禁用电机
  Future<bool> disable() async {
    if (!_connected) return false;
    try {
      await _stub!.disable(dog.Empty());
      _standing = false;
      debugPrint('[DogDirect] Motors DISABLED');
      notifyListeners();
      return true;
    } catch (e) {
      _errorMessage = 'Disable failed: $e';
      notifyListeners();
      return false;
    }
  }

  /// 站立
  Future<bool> standUp() async {
    if (!_connected) return false;
    try {
      await _stub!.standUp(dog.Empty());
      _standing = true;
      debugPrint('[DogDirect] STAND UP');
      notifyListeners();
      return true;
    } catch (e) {
      _errorMessage = 'StandUp failed: $e';
      notifyListeners();
      return false;
    }
  }

  /// 坐下
  Future<bool> sitDown() async {
    if (!_connected) return false;
    try {
      // 先停止行走
      await _stub!.walk(dog.Vector3(x: 0, y: 0, z: 0));
      await _stub!.sitDown(dog.Empty());
      _standing = false;
      debugPrint('[DogDirect] SIT DOWN');
      notifyListeners();
      return true;
    } catch (e) {
      _errorMessage = 'SitDown failed: $e';
      notifyListeners();
      return false;
    }
  }

  /// 行走 (归一化向量, 范围 [-1, 1])
  ///   x = 前进/后退
  ///   y = 左移/右移
  ///   z = 逆时针/顺时针旋转
  Future<void> walk(double x, double y, double z) async {
    if (!_connected || !_standing) return;
    try {
      await _stub!.walk(dog.Vector3(x: x, y: y, z: z));
    } catch (e) {
      debugPrint('[DogDirect] Walk failed: $e');
    }
  }

  /// 发送速度指令 (m/s, rad/s), 内部归一化
  Future<void> walkVelocity(
    double vx, double vy, double wz, {
    double maxLinear = 1.0,
    double maxAngular = 1.0,
  }) async {
    final nx = (vx / maxLinear).clamp(-1.0, 1.0);
    final ny = (vy / maxLinear).clamp(-1.0, 1.0);
    final nz = (wz / maxAngular).clamp(-1.0, 1.0);
    await walk(nx, ny, nz);
  }

  /// 紧急停止 (零速)
  Future<void> emergencyStop() async {
    if (!_connected) return;
    try {
      await _stub!.walk(dog.Vector3(x: 0, y: 0, z: 0));
    } catch (_) {}
  }

  // ============ Sensor Streams ============

  void _startStreams() {
    _stopStreams();
    if (_stub == null) return;

    // IMU stream
    _imuSub = _stub!.listenImu(dog.Empty()).listen(
      (imu) {
        _latestImu = imu;
        _imuController.add(imu);
      },
      onError: (e) => debugPrint('[DogDirect] IMU stream error: $e'),
      onDone: () => debugPrint('[DogDirect] IMU stream ended'),
    );

    // Joint stream
    _jointSub = _stub!.listenJoint(dog.Empty()).listen(
      (joint) {
        _latestJoint = joint;
        _jointController.add(joint);
      },
      onError: (e) => debugPrint('[DogDirect] Joint stream error: $e'),
      onDone: () => debugPrint('[DogDirect] Joint stream ended'),
    );

    // History stream (RL policy)
    _historySub = _stub!.listenHistory(dog.Empty()).listen(
      (history) {
        _latestHistory = history;
        _historyController.add(history);
      },
      onError: (e) => debugPrint('[DogDirect] History stream error: $e'),
      onDone: () => debugPrint('[DogDirect] History stream ended'),
    );

    debugPrint('[DogDirect] All sensor streams started');
  }

  void _stopStreams() {
    _imuSub?.cancel();
    _imuSub = null;
    _jointSub?.cancel();
    _jointSub = null;
    _historySub?.cancel();
    _historySub = null;
  }

  // ============ Cleanup ============

  @override
  void dispose() {
    _stopStreams();
    _imuController.close();
    _jointController.close();
    _historyController.close();
    _channel?.shutdown();
    super.dispose();
  }
}

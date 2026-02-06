import 'dart:async';
import 'dart:math' as math;

import 'package:fixnum/fixnum.dart';
import 'package:protobuf/well_known_types/google/protobuf/duration.pb.dart' as pb;
import 'package:protobuf/well_known_types/google/protobuf/timestamp.pb.dart';

import 'package:robot_proto/src/common.pb.dart';
import 'package:robot_proto/src/control.pb.dart';
import 'package:robot_proto/src/telemetry.pb.dart';
import 'package:robot_proto/src/system.pb.dart';
import 'package:robot_proto/src/data.pb.dart';
import 'package:robot_proto/src/data.pbgrpc.dart';
import 'robot_client_base.dart';

class MockRobotClient implements RobotClientBase {
  bool _connected = false;
  bool _hasLease = false;
  RobotMode _currentMode = RobotMode.ROBOT_MODE_IDLE;
  int _eventCounter = 0;
  int _teleopSeq = 0;

  @override
  Future<bool> connect() async {
    _connected = true;
    return true;
  }

  @override
  Stream<FastState> streamFastState({double desiredHz = 10.0}) {
    if (!_connected) {
      throw Exception('Not connected (mock)');
    }

    final periodMs = (1000 / desiredHz).clamp(10, 1000).round();
    final start = DateTime.now();

    return Stream.periodic(Duration(milliseconds: periodMs), (count) {
      final t = DateTime.now().difference(start).inMilliseconds / 1000.0;
      const r = 2.0;
      final x = r * math.cos(t);
      final y = r * math.sin(t);
      final yaw = t % (2 * math.pi);

      final q = Quaternion()
        ..w = math.cos(yaw / 2)
        ..z = math.sin(yaw / 2);

      final pose = Pose()
        ..position = (Vector3()
          ..x = x
          ..y = y
          ..z = 0.0)
        ..orientation = q;

      final twist = Twist()
        ..linear = (Vector3()
          ..x = -r * math.sin(t)
          ..y = r * math.cos(t)
          ..z = 0.0)
        ..angular = (Vector3()..z = 0.5);

      return FastState()
        ..header = _header()
        ..pose = pose
        ..velocity = twist
        ..linearAcceleration = (Vector3()..z = 9.8)
        ..angularVelocity = (Vector3()..z = 0.5)
        ..rpyDeg = (Vector3()..z = yaw * 180 / math.pi)
        ..tfOk = true;
    });
  }

  @override
  Stream<SlowState> streamSlowState() {
    if (!_connected) {
      throw Exception('Not connected (mock)');
    }

    final start = DateTime.now();
    return Stream.periodic(const Duration(seconds: 1), (count) {
      final t = DateTime.now().difference(start).inSeconds.toDouble();
      final cpu = 40 + 20 * math.sin(t / 5);
      final mem = 50 + 10 * math.cos(t / 7);
      final temp = 45 + 5 * math.sin(t / 9);
      final battery = 80 - (t % 60) * 0.2;

      return SlowState()
        ..header = _header()
        ..currentMode = _currentMode.name
        ..resources = (SystemResource()
          ..cpuPercent = cpu
          ..memPercent = mem
          ..cpuTemp = temp
          ..batteryPercent = battery
          ..batteryVoltage = 24.5)
        ..topicRates = (TopicRates()
          ..odomHz = 10
          ..terrainMapHz = 5
          ..pathHz = 2
          ..lidarHz = 15
          ..cmdVelHz = 0
          ..globalPathHz = 0)
        ..navigation = (NavigationStatus()
          ..globalPlannerStatus = 'IDLE'
          ..localizationValid = true
          ..slowDownLevel = 0);
    });
  }

  @override
  Stream<Event> streamEvents({String lastEventId = ''}) {
    if (!_connected) {
      throw Exception('Not connected (mock)');
    }

    return Stream.periodic(const Duration(seconds: 5), (count) {
      final severity = EventSeverity.values[count % EventSeverity.values.length];
      final type = EventType.values[count % EventType.values.length];
      _eventCounter++;

      return Event()
        ..eventId = 'mock-${_eventCounter.toString().padLeft(4, '0')}'
        ..severity = severity
        ..type = type
        ..title = 'Mock Event ${_eventCounter}'
        ..description = 'Generated mock event for testing'
        ..timestamp = Timestamp.fromDateTime(DateTime.now());
    });
  }

  @override
  Stream<DataChunk> subscribeToResource(ResourceId resourceId) {
    if (!_connected) {
      throw Exception('Not connected (mock)');
    }

    // Different behavior based on resource type
    if (resourceId.type == ResourceType.RESOURCE_TYPE_CAMERA) {
      // Camera: emit empty chunks at 30Hz (simulating video stream)
      // The UI will show "No Camera Feed" placeholder for empty data
      return Stream.periodic(const Duration(milliseconds: 33), (count) {
        return DataChunk()
          ..header = _header()
          ..resourceId = resourceId
          ..data = [] // Empty - no mock camera image data
          ..compression = CompressionType.COMPRESSION_TYPE_NONE;
      });
    } else {
      // Point cloud / map data: emit dummy chunks every 2 seconds
      return Stream.periodic(const Duration(seconds: 2), (count) {
        return DataChunk()
          ..header = _header()
          ..resourceId = resourceId
          ..data = [] // Empty dummy data
          ..compression = CompressionType.COMPRESSION_TYPE_NONE;
      });
    }
  }

  @override
  Future<bool> acquireLease() async {
    _hasLease = true;
    return true;
  }

  @override
  Future<void> releaseLease() async {
    _hasLease = false;
  }

  @override
  Stream<TeleopFeedback> streamTeleop(Stream<Twist> velocityStream) {
    if (!_hasLease) {
      throw Exception('Lease required for teleop (mock)');
    }

    final controller = StreamController<TeleopFeedback>();
    late StreamSubscription<Twist> sub;
    sub = velocityStream.listen((twist) {
      _teleopSeq++;
      controller.add(
        TeleopFeedback()
          ..timestamp = Timestamp.fromDateTime(DateTime.now())
          ..commandSequence = Int64(_teleopSeq)
          ..actualVelocity = twist
          ..safetyStatus = (SafetyStatus()
            ..estopActive = false
            ..deadmanActive = false
            ..tiltLimitActive = false
            ..speedLimited = false
            ..maxAllowedSpeed = 1.5
            ..safetyMessage = 'OK')
          ..controlLatency = (pb.Duration()..nanos = 1000000),
      );
    }, onError: controller.addError, onDone: () async {
      await controller.close();
    });

    controller.onCancel = () async {
      await sub.cancel();
    };

    return controller.stream;
  }

  @override
  Future<bool> setMode(RobotMode mode) async {
    _currentMode = mode;
    return true;
  }

  @override
  Future<bool> emergencyStop({bool hardStop = false}) async {
    _currentMode = RobotMode.ROBOT_MODE_ESTOP;
    return true;
  }

  @override
  Future<void> ackEvent(String eventId) async {
    // no-op for mock
  }

  @override
  Future<RelocalizeResponse> relocalize({
    required String pcdPath,
    double x = 0, double y = 0, double z = 0,
    double yaw = 0, double pitch = 0, double roll = 0,
  }) async {
    return RelocalizeResponse()
      ..base = (ResponseBase()..errorCode = ErrorCode.ERROR_CODE_OK)
      ..success = true
      ..message = 'Mock relocalize OK';
  }

  @override
  Future<SaveMapResponse> saveMap({required String filePath, bool savePatches = false}) async {
    return SaveMapResponse()
      ..base = (ResponseBase()..errorCode = ErrorCode.ERROR_CODE_OK)
      ..success = true
      ..message = 'Mock save map OK';
  }

  @override
  Future<StartTaskResponse> startTask({required TaskType taskType, String paramsJson = ''}) async {
    return StartTaskResponse()
      ..base = (ResponseBase()..errorCode = ErrorCode.ERROR_CODE_OK)
      ..taskId = 'mock-task-001'
      ..task = (Task()
        ..taskId = 'mock-task-001'
        ..type = taskType
        ..status = TaskStatus.TASK_STATUS_RUNNING);
  }

  @override
  Future<CancelTaskResponse> cancelTask({required String taskId}) async {
    return CancelTaskResponse()
      ..base = (ResponseBase()..errorCode = ErrorCode.ERROR_CODE_OK)
      ..task = (Task()
        ..taskId = taskId
        ..status = TaskStatus.TASK_STATUS_CANCELLED);
  }

  @override
  Future<void> disconnect() async {
    _connected = false;
    _hasLease = false;
  }

  @override
  bool get isConnected => _connected;

  @override
  DataServiceClient? get dataServiceClient => null;

  Header _header() {
    return Header()
      ..timestamp = Timestamp.fromDateTime(DateTime.now())
      ..frameId = 'mock'
      ..sequence = Int64(DateTime.now().millisecondsSinceEpoch);
  }
}

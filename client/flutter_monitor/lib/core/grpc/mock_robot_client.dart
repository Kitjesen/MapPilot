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
import 'package:flutter_monitor/core/grpc/robot_client_base.dart';


class MockRobotClient implements RobotClientBase {
  bool _connected = false;
  bool _hasLease = false;
  RobotMode _currentMode = RobotMode.ROBOT_MODE_IDLE;

  // Mock task state
  String _mockTaskId = '';
  TaskStatus _mockTaskStatus = TaskStatus.TASK_STATUS_UNSPECIFIED;
  TaskType _mockTaskType = TaskType.TASK_TYPE_UNSPECIFIED;
  double _mockTaskProgress = 0.0;
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
  Future<StartTaskResponse> startTask({
    required TaskType taskType,
    String paramsJson = '',
    NavigationParams? navigationParams,
    MappingParams? mappingParams,
    FollowPathParams? followPathParams,
  }) async {
    _mockTaskId = 'mock-task-${DateTime.now().millisecondsSinceEpoch}';
    _mockTaskStatus = TaskStatus.TASK_STATUS_RUNNING;
    _mockTaskType = taskType;
    _mockTaskProgress = 0.0;
    return StartTaskResponse()
      ..base = (ResponseBase()..errorCode = ErrorCode.ERROR_CODE_OK)
      ..taskId = _mockTaskId
      ..task = (Task()
        ..taskId = _mockTaskId
        ..type = taskType
        ..status = TaskStatus.TASK_STATUS_RUNNING);
  }

  @override
  Future<CancelTaskResponse> cancelTask({required String taskId}) async {
    _mockTaskStatus = TaskStatus.TASK_STATUS_CANCELLED;
    return CancelTaskResponse()
      ..base = (ResponseBase()..errorCode = ErrorCode.ERROR_CODE_OK)
      ..task = (Task()
        ..taskId = taskId
        ..status = TaskStatus.TASK_STATUS_CANCELLED);
  }

  @override
  Future<PauseTaskResponse> pauseTask({required String taskId}) async {
    _mockTaskStatus = TaskStatus.TASK_STATUS_PAUSED;
    return PauseTaskResponse()
      ..base = (ResponseBase()..errorCode = ErrorCode.ERROR_CODE_OK)
      ..task = (Task()
        ..taskId = taskId
        ..status = TaskStatus.TASK_STATUS_PAUSED
        ..progressPercent = _mockTaskProgress);
  }

  @override
  Future<ResumeTaskResponse> resumeTask({required String taskId}) async {
    _mockTaskStatus = TaskStatus.TASK_STATUS_RUNNING;
    return ResumeTaskResponse()
      ..base = (ResponseBase()..errorCode = ErrorCode.ERROR_CODE_OK)
      ..task = (Task()
        ..taskId = taskId
        ..status = TaskStatus.TASK_STATUS_RUNNING
        ..progressPercent = _mockTaskProgress);
  }

  @override
  Future<GetTaskStatusResponse> getTaskStatus({required String taskId}) async {
    // Simulate progress
    if (_mockTaskStatus == TaskStatus.TASK_STATUS_RUNNING) {
      _mockTaskProgress = math.min(1.0, _mockTaskProgress + 0.05);
      if (_mockTaskProgress >= 1.0) {
        _mockTaskStatus = TaskStatus.TASK_STATUS_COMPLETED;
      }
    }
    return GetTaskStatusResponse()
      ..base = (ResponseBase()..errorCode = ErrorCode.ERROR_CODE_OK)
      ..task = (Task()
        ..taskId = _mockTaskId
        ..type = _mockTaskType
        ..status = _mockTaskStatus
        ..progressPercent = _mockTaskProgress);
  }

  // ==================== 地图管理 Mock ====================

  @override
  Future<ListMapsResponse> listMaps({String directory = '/maps'}) async {
    return ListMapsResponse()
      ..base = (ResponseBase()..errorCode = ErrorCode.ERROR_CODE_OK)
      ..maps.addAll([
        MapInfo()
          ..name = 'office_floor1.pcd'
          ..path = '/maps/office_floor1.pcd'
          ..sizeBytes = Int64(52428800)
          ..modifiedAt = '2026-01-15T10:30:00'
          ..pointCount = 1250000,
        MapInfo()
          ..name = 'warehouse.pcd'
          ..path = '/maps/warehouse.pcd'
          ..sizeBytes = Int64(104857600)
          ..modifiedAt = '2026-01-20T14:00:00'
          ..pointCount = 3500000,
        MapInfo()
          ..name = 'outdoor_campus.pcd'
          ..path = '/maps/outdoor_campus.pcd'
          ..sizeBytes = Int64(209715200)
          ..modifiedAt = '2026-02-01T09:15:00'
          ..pointCount = 8200000,
      ]);
  }

  @override
  Future<DeleteMapResponse> deleteMap({required String path}) async {
    return DeleteMapResponse()
      ..base = (ResponseBase()..errorCode = ErrorCode.ERROR_CODE_OK)
      ..success = true
      ..message = 'Deleted: $path';
  }

  @override
  Future<RenameMapResponse> renameMap({required String oldPath, required String newName}) async {
    final dir = oldPath.substring(0, oldPath.lastIndexOf('/') + 1);
    return RenameMapResponse()
      ..base = (ResponseBase()..errorCode = ErrorCode.ERROR_CODE_OK)
      ..success = true
      ..newPath = '$dir$newName'
      ..message = 'Renamed to: $dir$newName';
  }

  // ==================== 文件管理 (OTA) Mock ====================

  @override
  Future<UploadFileResponse> uploadFile({
    required List<int> localBytes,
    required String remotePath,
    required String filename,
    String category = 'model',
    bool overwrite = true,
    int resumeFromOffset = 0,
    String sha256 = '',
    void Function(double progress)? onProgress,
  }) async {
    // 模拟分块上传进度
    const steps = 10;
    for (int i = 1; i <= steps; i++) {
      await Future.delayed(const Duration(milliseconds: 100));
      onProgress?.call(i / steps);
    }
    return UploadFileResponse()
      ..base = (ResponseBase()..errorCode = ErrorCode.ERROR_CODE_OK)
      ..success = true
      ..remotePath = remotePath
      ..bytesReceived = Int64(localBytes.length)
      ..message = 'Mock upload OK: $filename';
  }

  @override
  Future<ListRemoteFilesResponse> listRemoteFiles({
    required String directory,
    String category = '',
  }) async {
    final now = DateTime.now().toIso8601String();
    return ListRemoteFilesResponse()
      ..base = (ResponseBase()..errorCode = ErrorCode.ERROR_CODE_OK)
      ..files.addAll([
        RemoteFileInfo()
          ..path = '$directory/yolov8n.pt'
          ..filename = 'yolov8n.pt'
          ..size = Int64(6 * 1024 * 1024)
          ..modifiedTime = now
          ..category = 'model',
        RemoteFileInfo()
          ..path = '$directory/terrain_model.onnx'
          ..filename = 'terrain_model.onnx'
          ..size = Int64(12 * 1024 * 1024)
          ..modifiedTime = now
          ..category = 'model',
        RemoteFileInfo()
          ..path = '$directory/slam_config.yaml'
          ..filename = 'slam_config.yaml'
          ..size = Int64(2048)
          ..modifiedTime = now
          ..category = 'config',
      ])
      ..totalSize = Int64(18 * 1024 * 1024)
      ..freeSpace = Int64(8 * 1024 * 1024 * 1024);
  }

  @override
  Future<DeleteRemoteFileResponse> deleteRemoteFile({required String remotePath}) async {
    return DeleteRemoteFileResponse()
      ..base = (ResponseBase()..errorCode = ErrorCode.ERROR_CODE_OK)
      ..success = true
      ..message = 'Mock delete OK: $remotePath';
  }

  // ==================== OTA Mock ====================

  @override
  Future<ApplyUpdateResponse> applyUpdate({
    required OtaArtifact artifact,
    required String stagedPath,
    bool force = false,
  }) async {
    await Future.delayed(const Duration(seconds: 1));
    return ApplyUpdateResponse()
      ..base = (ResponseBase()..errorCode = ErrorCode.ERROR_CODE_OK)
      ..success = true
      ..status = OtaUpdateStatus.OTA_UPDATE_STATUS_SUCCESS
      ..message = 'Mock apply update OK: ${artifact.name} v${artifact.version}';
  }

  @override
  Future<GetInstalledVersionsResponse> getInstalledVersions({
    OtaCategory categoryFilter = OtaCategory.OTA_CATEGORY_UNSPECIFIED,
  }) async {
    return GetInstalledVersionsResponse()
      ..base = (ResponseBase()..errorCode = ErrorCode.ERROR_CODE_OK)
      ..installed.addAll([
        InstalledArtifact()
          ..name = 'navigation_firmware'
          ..version = '1.0.0'
          ..category = OtaCategory.OTA_CATEGORY_FIRMWARE
          ..installedAt = DateTime.now().toIso8601String(),
        InstalledArtifact()
          ..name = 'yolov8n'
          ..version = '8.0.1'
          ..category = OtaCategory.OTA_CATEGORY_MODEL
          ..installedAt = DateTime.now().toIso8601String(),
      ]);
  }

  @override
  Future<RollbackResponse> rollback({required String artifactName}) async {
    await Future.delayed(const Duration(seconds: 1));
    return RollbackResponse()
      ..base = (ResponseBase()..errorCode = ErrorCode.ERROR_CODE_OK)
      ..success = true
      ..restoredVersion = '1.0.0'
      ..message = 'Mock rollback OK: $artifactName';
  }

  @override
  Stream<OtaProgress> downloadFromUrl({
    required String url,
    required String stagingPath,
    String expectedSha256 = '',
    int expectedSize = 0,
    Map<String, String> headers = const {},
  }) async* {
    const steps = 10;
    for (int i = 1; i <= steps; i++) {
      await Future.delayed(const Duration(milliseconds: 200));
      yield OtaProgress()
        ..status = OtaUpdateStatus.OTA_UPDATE_STATUS_PENDING
        ..progressPercent = (i * 100.0 / steps)
        ..bytesCompleted = Int64(i * 1024 * 1024)
        ..bytesTotal = Int64(steps * 1024 * 1024)
        ..speedBytesPerSec = 5 * 1024 * 1024
        ..message = 'Downloading... ${(i * 100 / steps).toInt()}%';
    }
    yield OtaProgress()
      ..status = OtaUpdateStatus.OTA_UPDATE_STATUS_SUCCESS
      ..progressPercent = 100.0
      ..message = 'Mock download complete';
  }

  @override
  Future<CheckUpdateReadinessResponse> checkUpdateReadiness({
    required List<OtaArtifact> artifacts,
    String manifestSignature = '',
  }) async {
    return CheckUpdateReadinessResponse()
      ..base = (ResponseBase()..errorCode = ErrorCode.ERROR_CODE_OK)
      ..ready = true
      ..checks.addAll([
        ReadinessCheck()
          ..checkName = 'disk_space'
          ..passed = true
          ..message = 'Sufficient disk space'
          ..detail = 'free=8GB required=500MB',
        ReadinessCheck()
          ..checkName = 'battery'
          ..passed = true
          ..message = 'Battery level OK'
          ..detail = 'level=85% required=20%',
      ]);
  }

  @override
  Future<GetUpgradeHistoryResponse> getUpgradeHistory({
    String artifactFilter = '',
    int limit = 50,
  }) async {
    return GetUpgradeHistoryResponse()
      ..base = (ResponseBase()..errorCode = ErrorCode.ERROR_CODE_OK)
      ..entries.addAll([
        UpgradeHistoryEntry()
          ..timestamp = DateTime.now().subtract(const Duration(days: 1)).toIso8601String()
          ..action = 'install'
          ..artifactName = 'navigation_firmware'
          ..fromVersion = '0.9.0'
          ..toVersion = '1.0.0'
          ..success = true,
        UpgradeHistoryEntry()
          ..timestamp = DateTime.now().subtract(const Duration(days: 3)).toIso8601String()
          ..action = 'install'
          ..artifactName = 'yolov8n'
          ..fromVersion = '8.0.0'
          ..toVersion = '8.0.1'
          ..success = true,
      ]);
  }

  @override
  Future<ValidateSystemVersionResponse> validateSystemVersion({
    String expectedSystemVersion = '',
    List<ComponentVersion> expectedComponents = const [],
  }) async {
    return ValidateSystemVersionResponse()
      ..base = (ResponseBase()..errorCode = ErrorCode.ERROR_CODE_OK)
      ..consistent = true
      ..actualSystemVersion = '1.0.0';
  }

  @override
  Future<RobotInfoResponse> getRobotInfo() async {
    return RobotInfoResponse()
      ..base = (ResponseBase()..errorCode = ErrorCode.ERROR_CODE_OK)
      ..robotId = 'mock-robot-001'
      ..displayName = 'Mock Robot'
      ..firmwareVersion = '1.0.0-mock'
      ..softwareVersion = '2.0.0-mock';
  }

  @override
  Future<CapabilitiesResponse> getCapabilities() async {
    return CapabilitiesResponse()
      ..base = (ResponseBase()..errorCode = ErrorCode.ERROR_CODE_OK)
      ..supportedResources.addAll(['camera', 'pointcloud', 'map'])
      ..supportedTasks.addAll([
        TaskType.TASK_TYPE_NAVIGATION,
        TaskType.TASK_TYPE_MAPPING,
      ])
      ..teleopSupported = true
      ..mappingSupported = true;
  }

  @override
  Future<HeartbeatResponse> heartbeat() async {
    return HeartbeatResponse()
      ..serverTimestamp = Timestamp.fromDateTime(DateTime.now())
      ..activeSessions = 1;
  }

  @override
  Future<DeviceInfoResponse> getDeviceInfo() async {
    return DeviceInfoResponse()
      ..base = (ResponseBase()..errorCode = ErrorCode.ERROR_CODE_OK)
      ..hostname = 'mock-robot'
      ..robotId = 'mock-001'
      ..hwId = 'hw-0001'
      ..ipAddresses.addAll(['192.168.1.100', '10.0.0.1'])
      ..diskTotalBytes = Int64(64 * 1024 * 1024 * 1024) // 64 GB
      ..diskFreeBytes = Int64(32 * 1024 * 1024 * 1024)  // 32 GB
      ..batteryPercent = 85
      ..uptimeSeconds = Int64(86400) // 1 day
      ..osVersion = 'Ubuntu 22.04 LTS'
      ..otaDaemonVersion = '1.2.0'
      ..services.addAll([
        ServiceStatus()..name = 'navigation.service'..state = 'active'..subState = 'running'..uptimeSeconds = Int64(3600),
        ServiceStatus()..name = 'slam.service'..state = 'active'..subState = 'running'..uptimeSeconds = Int64(3600),
        ServiceStatus()..name = 'camera.service'..state = 'inactive'..subState = 'dead',
      ]);
  }

  @override
  Future<ManageServiceResponse> manageService({
    required String serviceName,
    required ServiceAction action,
  }) async {
    return ManageServiceResponse()
      ..base = (ResponseBase()..errorCode = ErrorCode.ERROR_CODE_OK)
      ..success = true
      ..message = 'Mock: $serviceName ${action.name}'
      ..status = (ServiceStatus()..name = serviceName..state = 'active'..subState = 'running');
  }

  @override
  Future<ListResourcesResponse> listResources() async {
    return ListResourcesResponse()
      ..base = (ResponseBase()..errorCode = ErrorCode.ERROR_CODE_OK)
      ..resources.addAll([
        ResourceInfo()..id = (ResourceId()..name = 'front_camera'..type = ResourceType.RESOURCE_TYPE_CAMERA)..description = '前置摄像头'..available = true,
        ResourceInfo()..id = (ResourceId()..name = 'rear_camera'..type = ResourceType.RESOURCE_TYPE_CAMERA)..description = '后置摄像头'..available = true,
        ResourceInfo()..id = (ResourceId()..name = 'pointcloud'..type = ResourceType.RESOURCE_TYPE_POINTCLOUD)..description = '3D 点云'..available = true,
      ]);
  }

  @override
  Stream<FileChunk> downloadFile({
    required String filePath,
    int chunkSize = 65536,
  }) async* {
    // Mock: emit a single small chunk
    yield FileChunk()
      ..offset = Int64.ZERO
      ..data = [0x4D, 0x4F, 0x43, 0x4B] // "MOCK"
      ..totalSize = Int64(4)
      ..isLast = true;
  }

  @override
  Future<UnsubscribeResponse> unsubscribe({required String subscriptionId}) async {
    return UnsubscribeResponse()
      ..base = (ResponseBase()..errorCode = ErrorCode.ERROR_CODE_OK);
  }

  @override
  Future<void> disconnect() async {
    _connected = false;
    _hasLease = false;
  }

  @override
  bool get isConnected => _connected;

  @override
  bool get otaAvailable => _connected; // Mock mode: always available when connected

  @override
  DataServiceClient? get dataServiceClient => null;

  Header _header() {
    return Header()
      ..timestamp = Timestamp.fromDateTime(DateTime.now())
      ..frameId = 'mock'
      ..sequence = Int64(DateTime.now().millisecondsSinceEpoch);
  }
}

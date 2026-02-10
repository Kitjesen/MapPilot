import 'dart:async';
import 'package:fixnum/fixnum.dart';
import 'package:grpc/grpc.dart';
import 'package:robot_proto/src/telemetry.pbgrpc.dart';
import 'package:robot_proto/src/system.pbgrpc.dart';
import 'package:robot_proto/src/system.pb.dart';
import 'package:robot_proto/src/control.pbgrpc.dart';
import 'package:robot_proto/src/data.pbgrpc.dart';
import 'package:robot_proto/src/common.pb.dart';
import 'package:protobuf/well_known_types/google/protobuf/empty.pb.dart';
import 'package:protobuf/well_known_types/google/protobuf/timestamp.pb.dart';
import 'package:uuid/uuid.dart';
import 'package:flutter_monitor/core/grpc/robot_client_base.dart';

class RobotClient implements RobotClientBase {
  final String host;
  final int port;
  /// OTA Daemon 端口 (独立守护进程, 不依赖 ROS2)
  final int otaPort;
  final bool useTls;
  /// Optional PEM-encoded server certificate for trust-on-first-use verification.
  /// When null/empty, system defaults are used (or all certs trusted for self-signed).
  final List<int>? trustedCertificateBytes;
  
  late ClientChannel _channel;
  late ClientChannel _otaChannel;
  late TelemetryServiceClient _telemetryClient;
  late SystemServiceClient _systemClient;
  late ControlServiceClient _controlClient;
  late DataServiceClient _dataClient;
  late OtaServiceClient _otaClient;
  
  bool _isConnected = false;
  StreamSubscription? _fastStateSubscription;
  StreamSubscription? _slowStateSubscription;
  
  // Lease token for control
  String? _currentLeaseToken;
  Timer? _leaseRenewalTimer;

  RobotClient({
    required this.host,
    this.port = 50051,
    this.otaPort = 50052,
    this.useTls = false,
    this.trustedCertificateBytes,
  });

  /// 连接到机器人
  Future<bool> connect() async {
    try {
      final ChannelCredentials credentials;
      if (useTls) {
        credentials = ChannelCredentials.secure(
          certificates: trustedCertificateBytes,
          // For self-signed certs (trust-on-first-use), override authority
          authority: host,
        );
      } else {
        credentials = const ChannelCredentials.insecure();
      }

      _channel = ClientChannel(
        host,
        port: port,
        options: ChannelOptions(
          credentials: credentials,
        ),
      );

      _telemetryClient = TelemetryServiceClient(_channel);
      _systemClient = SystemServiceClient(_channel);
      _controlClient = ControlServiceClient(_channel);
      _dataClient = DataServiceClient(_channel);

      // OTA Daemon 独立通道 (:50052) — 同一 host, 不同端口
      _otaChannel = ClientChannel(
        host,
        port: otaPort,
        options: ChannelOptions(credentials: credentials),
      );
      _otaClient = OtaServiceClient(_otaChannel);

      // 测试主连接 (50051)
      final info = await _systemClient.getRobotInfo(
        Empty(),
        options: CallOptions(timeout: const Duration(seconds: 3)),
      );
      print('Connected to robot: ${info.robotId} ($host:$port)');

      // 测试 OTA 连接 (50052) — 非阻塞, 失败不影响主连接
      try {
        await _otaClient.getDeviceInfo(
          Empty(),
          options: CallOptions(timeout: const Duration(seconds: 2)),
        );
        _otaAvailable = true;
        print('OTA daemon available ($host:$otaPort)');
      } catch (e) {
        _otaAvailable = false;
        print('OTA daemon not available ($host:$otaPort): $e');
      }

      _isConnected = true;
      return true;
    } catch (e) {
      print('Connection failed: $e');
      _isConnected = false;
      return false;
    }
  }

  /// OTA daemon 是否可用
  bool _otaAvailable = false;
  bool get otaAvailable => _otaAvailable;

  /// 订阅快速状态流（位姿、速度、IMU）
  Stream<FastState> streamFastState({double desiredHz = 10.0}) {
    if (!_isConnected) {
      throw Exception('Not connected to robot');
    }

    final request = FastStateRequest()..desiredHz = desiredHz;
    
    return _telemetryClient.streamFastState(request).map((state) {
      return state;
    }).handleError((error) {
      print('FastState stream error: $error');
      _isConnected = false;
    });
  }

  /// 订阅慢速状态流（系统资源、模式）
  Stream<SlowState> streamSlowState() {
    if (!_isConnected) {
      throw Exception('Not connected to robot');
    }

    final request = SlowStateRequest();
    
    return _telemetryClient.streamSlowState(request).map((state) {
      return state;
    }).handleError((error) {
      print('SlowState stream error: $error');
      _isConnected = false;
    });
  }

  /// 订阅事件流
  Stream<Event> streamEvents({String lastEventId = ''}) {
    if (!_isConnected) {
      throw Exception('Not connected to robot');
    }

    final request = EventStreamRequest()
      ..lastEventId = lastEventId
      ..minSeverity = EventSeverity.EVENT_SEVERITY_INFO;

    return _telemetryClient.streamEvents(request);
  }

  /// 订阅资源 (地图/点云)
  Stream<DataChunk> subscribeToResource(ResourceId resourceId) {
    if (!_isConnected) {
      throw Exception('Not connected to robot');
    }

    final request = SubscribeRequest()
      ..base = _createRequestBase()
      ..resourceId = resourceId
      ..profile = (SubscribeProfile()..frequency = 30.0);

    return _dataClient.subscribe(request);
  }

  /// 获取租约
  Future<bool> acquireLease() async {
    try {
      final request = AcquireLeaseRequest()
        ..base = _createRequestBase();
      
      final response = await _controlClient.acquireLease(request);
      if (response.base.errorCode == ErrorCode.ERROR_CODE_OK) {
        _currentLeaseToken = response.lease.leaseToken;
        _startLeaseRenewal(response.lease.ttl.seconds.toInt());
        return true;
      }
      return false;
    } catch (e) {
      print('Acquire lease failed: $e');
      return false;
    }
  }

  /// 释放租约
  Future<void> releaseLease() async {
    if (_currentLeaseToken == null) return;
    
    try {
      _stopLeaseRenewal();
      final request = ReleaseLeaseRequest()
        ..base = _createRequestBase()
        ..leaseToken = _currentLeaseToken!;
      
      await _controlClient.releaseLease(request);
      _currentLeaseToken = null;
    } catch (e) {
      print('Release lease failed: $e');
    }
  }

  /// 续约逻辑
  void _startLeaseRenewal(int ttlSeconds) {
    _leaseRenewalTimer?.cancel();
    // 提前续约（例如 TTL 的一半时间）
    final renewalDuration = Duration(seconds: ttlSeconds > 2 ? ttlSeconds ~/ 2 : 1);
    
    _leaseRenewalTimer = Timer.periodic(renewalDuration, (_) async {
      if (_currentLeaseToken == null) {
        _stopLeaseRenewal();
        return;
      }

      try {
        final request = RenewLeaseRequest()
          ..base = _createRequestBase()
          ..leaseToken = _currentLeaseToken!;
        
        final response = await _controlClient.renewLease(request);
        if (response.base.errorCode != ErrorCode.ERROR_CODE_OK) {
          print('Lease renewal failed: ${response.base.errorMessage}');
          _currentLeaseToken = null;
          _stopLeaseRenewal();
        }
      } catch (e) {
        print('Lease renewal error: $e');
      }
    });
  }

  void _stopLeaseRenewal() {
    _leaseRenewalTimer?.cancel();
    _leaseRenewalTimer = null;
  }

  /// 设置模式
  Future<bool> setMode(RobotMode mode) async {
    try {
      final request = SetModeRequest()
        ..base = _createRequestBase()
        ..mode = mode;
      
      final response = await _controlClient.setMode(request);
      return response.base.errorCode == ErrorCode.ERROR_CODE_OK;
    } catch (e) {
      print('Set mode failed: $e');
      return false;
    }
  }

  /// 急停
  Future<bool> emergencyStop({bool hardStop = false}) async {
    try {
      final request = EmergencyStopRequest()
        ..base = _createRequestBase()
        ..hardStop = hardStop;
      
      final response = await _controlClient.emergencyStop(request);
      return response.base.errorCode == ErrorCode.ERROR_CODE_OK;
    } catch (e) {
      print('Emergency stop failed: $e');
      return false;
    }
  }

  /// 遥操作流
  Stream<TeleopFeedback> streamTeleop(Stream<Twist> velocityStream) {
    if (_currentLeaseToken == null) {
      throw Exception('Lease required for teleop');
    }

    final outgoingStream = velocityStream.map((twist) {
      return TeleopCommand()
        ..leaseToken = _currentLeaseToken!
        ..sequence = Int64(DateTime.now().millisecondsSinceEpoch)
        ..targetVelocity = twist
        ..enableObstacleAvoidance = true;
    });

    return _controlClient.streamTeleop(outgoingStream);
  }

  /// 确认事件
  Future<void> ackEvent(String eventId) async {
    try {
      final request = AckEventRequest()
        ..base = _createRequestBase()
        ..eventId = eventId;
      
      await _telemetryClient.ackEvent(request);
    } catch (e) {
      print('Ack event failed: $e');
    }
  }

  RequestBase _createRequestBase() {
    return RequestBase()
      ..requestId = const Uuid().v4()
      ..clientTimestamp = Timestamp.fromDateTime(DateTime.now());
  }

  /// 获取机器人信息
  @override
  Future<RobotInfoResponse> getRobotInfo() async {
    return await _systemClient.getRobotInfo(Empty());
  }

  /// 获取机器人能力
  @override
  Future<CapabilitiesResponse> getCapabilities() async {
    return await _systemClient.getCapabilities(Empty());
  }

  /// 心跳检测
  @override
  Future<HeartbeatResponse> heartbeat() async {
    final request = HeartbeatRequest()
      ..clientTimestamp = Timestamp.fromDateTime(DateTime.now());
    return await _systemClient.heartbeat(request);
  }

  /// 获取设备信息 (OTA daemon)
  @override
  Future<DeviceInfoResponse> getDeviceInfo() async {
    return await _otaClient.getDeviceInfo(Empty());
  }

  /// 管理系统服务 (OTA daemon)
  @override
  Future<ManageServiceResponse> manageService({
    required String serviceName,
    required ServiceAction action,
  }) async {
    final request = ManageServiceRequest()
      ..base = _createRequestBase()
      ..serviceName = serviceName
      ..action = action;
    return await _otaClient.manageService(request);
  }

  /// 列出可用资源
  @override
  Future<ListResourcesResponse> listResources() async {
    return await _dataClient.listResources(Empty());
  }

  /// 下载文件 (流式)
  @override
  Stream<FileChunk> downloadFile({
    required String filePath,
    int chunkSize = 65536,
  }) {
    final request = DownloadFileRequest()
      ..base = _createRequestBase()
      ..filePath = filePath
      ..chunkSize = chunkSize;
    return _dataClient.downloadFile(request);
  }

  /// 取消订阅资源
  @override
  Future<UnsubscribeResponse> unsubscribe({required String subscriptionId}) async {
    final request = UnsubscribeRequest()
      ..base = _createRequestBase()
      ..subscriptionId = subscriptionId;
    return await _dataClient.unsubscribe(request);
  }

  /// 重定位
  @override
  Future<RelocalizeResponse> relocalize({
    required String pcdPath,
    double x = 0, double y = 0, double z = 0,
    double yaw = 0, double pitch = 0, double roll = 0,
  }) async {
    final request = RelocalizeRequest()
      ..base = _createRequestBase()
      ..pcdPath = pcdPath
      ..x = x ..y = y ..z = z
      ..yaw = yaw ..pitch = pitch ..roll = roll;
    return await _systemClient.relocalize(request);
  }

  /// 保存地图
  @override
  Future<SaveMapResponse> saveMap({required String filePath, bool savePatches = false}) async {
    final request = SaveMapRequest()
      ..base = _createRequestBase()
      ..filePath = filePath
      ..savePatches = savePatches;
    return await _systemClient.saveMap(request);
  }

  /// 启动任务（支持结构化参数）
  @override
  Future<StartTaskResponse> startTask({
    required TaskType taskType,
    String paramsJson = '',
    NavigationParams? navigationParams,
    MappingParams? mappingParams,
    FollowPathParams? followPathParams,
  }) async {
    final request = StartTaskRequest()
      ..base = _createRequestBase()
      ..taskType = taskType
      ..paramsJson = paramsJson;
    if (navigationParams != null) request.navigationParams = navigationParams;
    if (mappingParams != null) request.mappingParams = mappingParams;
    if (followPathParams != null) request.followPathParams = followPathParams;
    return await _controlClient.startTask(request);
  }

  /// 取消任务
  @override
  Future<CancelTaskResponse> cancelTask({required String taskId}) async {
    final request = CancelTaskRequest()
      ..base = _createRequestBase()
      ..taskId = taskId;
    return await _controlClient.cancelTask(request);
  }

  /// 暂停任务
  @override
  Future<PauseTaskResponse> pauseTask({required String taskId}) async {
    final request = PauseTaskRequest()
      ..base = _createRequestBase()
      ..taskId = taskId;
    return await _controlClient.pauseTask(request);
  }

  /// 恢复任务
  @override
  Future<ResumeTaskResponse> resumeTask({required String taskId}) async {
    final request = ResumeTaskRequest()
      ..base = _createRequestBase()
      ..taskId = taskId;
    return await _controlClient.resumeTask(request);
  }

  /// 查询任务状态
  @override
  Future<GetTaskStatusResponse> getTaskStatus({required String taskId}) async {
    final request = GetTaskStatusRequest()
      ..base = _createRequestBase()
      ..taskId = taskId;
    return await _controlClient.getTaskStatus(request);
  }

  // ==================== 地图管理 ====================

  @override
  Future<ListMapsResponse> listMaps({String directory = '/maps'}) async {
    final request = ListMapsRequest()
      ..base = _createRequestBase()
      ..directory = directory;
    return await _systemClient.listMaps(request);
  }

  @override
  Future<DeleteMapResponse> deleteMap({required String path}) async {
    final request = DeleteMapRequest()
      ..base = _createRequestBase()
      ..path = path;
    return await _systemClient.deleteMap(request);
  }

  @override
  Future<RenameMapResponse> renameMap({required String oldPath, required String newName}) async {
    final request = RenameMapRequest()
      ..base = _createRequestBase()
      ..oldPath = oldPath
      ..newName = newName;
    return await _systemClient.renameMap(request);
  }

  // ==================== 文件管理 (支持断点续传) ====================

  /// 上传文件到机器人
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
    const chunkSize = 64 * 1024; // 64KB per chunk
    final totalSize = localBytes.length;

    // 断点续传: 跳过已上传部分
    final startOffset = resumeFromOffset.clamp(0, totalSize);

    final controller = StreamController<UploadFileChunk>();
    final responseFuture = _otaClient.uploadFile(controller.stream);

    // 发送第一个 chunk (包含 metadata)
    int offset = startOffset;
    final firstEnd = (offset + chunkSize) < totalSize
        ? offset + chunkSize
        : totalSize;
    controller.add(UploadFileChunk()
      ..metadata = (UploadFileMetadata()
        ..base = _createRequestBase()
        ..remotePath = remotePath
        ..filename = filename
        ..totalSize = Int64(totalSize)
        ..sha256 = sha256
        ..overwrite = overwrite
        ..category = category
        ..resumeFromOffset = Int64(startOffset))
      ..offset = Int64(offset)
      ..data = localBytes.sublist(offset, firstEnd)
      ..isLast = (firstEnd >= totalSize));
    offset = firstEnd;
    onProgress?.call(offset / totalSize);

    // 发送后续 chunks
    while (offset < totalSize) {
      final end = (offset + chunkSize) < totalSize ? offset + chunkSize : totalSize;
      controller.add(UploadFileChunk()
        ..offset = Int64(offset)
        ..data = localBytes.sublist(offset, end)
        ..isLast = (end >= totalSize));
      offset = end;
      onProgress?.call(offset / totalSize);
    }

    await controller.close();
    return await responseFuture;
  }

  /// 列出远程目录文件
  @override
  Future<ListRemoteFilesResponse> listRemoteFiles({
    required String directory,
    String category = '',
  }) async {
    final request = ListRemoteFilesRequest()
      ..base = _createRequestBase()
      ..directory = directory
      ..category = category;
    return await _otaClient.listRemoteFiles(request);
  }

  /// 删除远程文件
  @override
  Future<DeleteRemoteFileResponse> deleteRemoteFile({required String remotePath}) async {
    final request = DeleteRemoteFileRequest()
      ..base = _createRequestBase()
      ..remotePath = remotePath;
    return await _otaClient.deleteRemoteFile(request);
  }

  // ==================== OTA 更新管理 ====================

  /// 应用 OTA 更新
  @override
  Future<ApplyUpdateResponse> applyUpdate({
    required OtaArtifact artifact,
    required String stagedPath,
    bool force = false,
  }) async {
    final request = ApplyUpdateRequest()
      ..base = _createRequestBase()
      ..artifact = artifact
      ..stagedPath = stagedPath
      ..force = force;
    return await _otaClient.applyUpdate(request);
  }

  /// 查询已安装版本
  @override
  Future<GetInstalledVersionsResponse> getInstalledVersions({
    OtaCategory categoryFilter = OtaCategory.OTA_CATEGORY_UNSPECIFIED,
  }) async {
    final request = GetInstalledVersionsRequest()
      ..base = _createRequestBase()
      ..categoryFilter = categoryFilter;
    return await _otaClient.getInstalledVersions(request);
  }

  /// 回滚到上一版本
  @override
  Future<RollbackResponse> rollback({required String artifactName}) async {
    final request = RollbackRequest()
      ..base = _createRequestBase()
      ..artifactName = artifactName;
    return await _otaClient.rollback(request);
  }

  /// 机器人直接从 URL 下载（免手机中转）
  @override
  Stream<OtaProgress> downloadFromUrl({
    required String url,
    required String stagingPath,
    String expectedSha256 = '',
    int expectedSize = 0,
    Map<String, String> headers = const {},
  }) {
    final request = DownloadFromUrlRequest()
      ..base = _createRequestBase()
      ..url = url
      ..stagingPath = stagingPath
      ..expectedSha256 = expectedSha256
      ..expectedSize = Int64(expectedSize);

    for (final entry in headers.entries) {
      request.headers[entry.key] = entry.value;
    }

    return _otaClient.downloadFromUrl(request);
  }

  /// 安装前预检查
  @override
  Future<CheckUpdateReadinessResponse> checkUpdateReadiness({
    required List<OtaArtifact> artifacts,
    String manifestSignature = '',
  }) async {
    final request = CheckUpdateReadinessRequest()
      ..base = _createRequestBase()
      ..artifacts.addAll(artifacts)
      ..manifestSignature = manifestSignature;
    return await _otaClient.checkUpdateReadiness(request);
  }

  /// 查询升级历史
  @override
  Future<GetUpgradeHistoryResponse> getUpgradeHistory({
    String artifactFilter = '',
    int limit = 50,
  }) async {
    final request = GetUpgradeHistoryRequest()
      ..base = _createRequestBase()
      ..artifactFilter = artifactFilter
      ..limit = limit;
    return await _otaClient.getUpgradeHistory(request);
  }

  /// 版本一致性校验
  @override
  Future<ValidateSystemVersionResponse> validateSystemVersion({
    String expectedSystemVersion = '',
    List<ComponentVersion> expectedComponents = const [],
  }) async {
    final request = ValidateSystemVersionRequest()
      ..base = _createRequestBase()
      ..expectedSystemVersion = expectedSystemVersion
      ..expectedComponents.addAll(expectedComponents);
    return await _otaClient.validateSystemVersion(request);
  }

  /// 断开连接
  Future<void> disconnect() async {
    _stopLeaseRenewal();
    if (_currentLeaseToken != null) {
      await releaseLease();
    }
    await _fastStateSubscription?.cancel();
    await _slowStateSubscription?.cancel();
    await _channel.shutdown();
    await _otaChannel.shutdown();
    _isConnected = false;
  }

  bool get isConnected => _isConnected;
  
  /// DataServiceClient for WebRTC signaling
  @override
  DataServiceClient? get dataServiceClient => _isConnected ? _dataClient : null;
}

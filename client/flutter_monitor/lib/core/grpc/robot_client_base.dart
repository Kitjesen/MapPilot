import 'package:robot_proto/src/common.pb.dart';
import 'package:robot_proto/src/control.pb.dart';
import 'package:robot_proto/src/telemetry.pb.dart';
import 'package:robot_proto/src/system.pb.dart';
import 'package:robot_proto/src/data.pb.dart';
import 'package:robot_proto/src/data.pbgrpc.dart';

abstract class RobotClientBase {
  Future<bool> connect();

  Stream<FastState> streamFastState({double desiredHz = 10.0});
  Stream<SlowState> streamSlowState();
  Stream<Event> streamEvents({String lastEventId = ''});
  Stream<DataChunk> subscribeToResource(ResourceId resourceId);

  Future<bool> acquireLease();
  Future<void> releaseLease();
  Stream<TeleopFeedback> streamTeleop(Stream<Twist> velocityStream);
  Future<bool> setMode(RobotMode mode);
  Future<bool> emergencyStop({bool hardStop = false});
  Future<void> ackEvent(String eventId);

  /// 重定位（加载地图并设置初始位姿）
  Future<RelocalizeResponse> relocalize({
    required String pcdPath,
    double x = 0, double y = 0, double z = 0,
    double yaw = 0, double pitch = 0, double roll = 0,
  });

  /// 保存地图
  Future<SaveMapResponse> saveMap({required String filePath, bool savePatches = false});

  /// 启动任务（导航/建图）— 支持结构化参数
  Future<StartTaskResponse> startTask({
    required TaskType taskType,
    String paramsJson = '',
    NavigationParams? navigationParams,
    MappingParams? mappingParams,
    FollowPathParams? followPathParams,
  });

  /// 取消任务
  Future<CancelTaskResponse> cancelTask({required String taskId});

  /// 暂停任务
  Future<PauseTaskResponse> pauseTask({required String taskId});

  /// 恢复任务
  Future<ResumeTaskResponse> resumeTask({required String taskId});

  /// 查询任务状态
  Future<GetTaskStatusResponse> getTaskStatus({required String taskId});

  // ==================== 地图管理 ====================

  /// 列出已保存的地图
  Future<ListMapsResponse> listMaps({String directory = '/maps'});

  /// 删除地图
  Future<DeleteMapResponse> deleteMap({required String path});

  /// 重命名地图
  Future<RenameMapResponse> renameMap({required String oldPath, required String newName});

  // ==================== 文件管理 ====================

  /// 上传文件到机器人 (支持断点续传)
  /// [localBytes] 文件二进制内容
  /// [remotePath] 目标路径
  /// [filename] 文件名
  /// [category] 分类: "model", "map", "config", "firmware"
  /// [resumeFromOffset] 断点续传偏移量, 0=全新上传
  /// [onProgress] 上传进度回调 (0.0 ~ 1.0)
  Future<UploadFileResponse> uploadFile({
    required List<int> localBytes,
    required String remotePath,
    required String filename,
    String category = 'model',
    bool overwrite = true,
    int resumeFromOffset = 0,
    String sha256 = '',
    void Function(double progress)? onProgress,
  });

  /// 列出远程目录文件
  Future<ListRemoteFilesResponse> listRemoteFiles({
    required String directory,
    String category = '',
  });

  /// 删除远程文件
  Future<DeleteRemoteFileResponse> deleteRemoteFile({required String remotePath});

  // ==================== OTA 更新管理 ====================

  /// 应用 OTA 更新（SHA256 校验、备份、安装、回滚支持）
  Future<ApplyUpdateResponse> applyUpdate({
    required OtaArtifact artifact,
    required String stagedPath,
    bool force = false,
  });

  /// 查询已安装版本列表
  Future<GetInstalledVersionsResponse> getInstalledVersions({
    OtaCategory categoryFilter = OtaCategory.OTA_CATEGORY_UNSPECIFIED,
  });

  /// 回滚到上一版本
  Future<RollbackResponse> rollback({required String artifactName});

  /// 机器人直接从 URL 下载（免手机中转，适合大文件如 ONNX 模型）
  /// 返回进度流，客户端可实时显示进度条
  Stream<OtaProgress> downloadFromUrl({
    required String url,
    required String stagingPath,
    String expectedSha256 = '',
    int expectedSize = 0,
    Map<String, String> headers = const {},
  });

  /// 安装前预检查（磁盘空间、电量、硬件兼容性）
  Future<CheckUpdateReadinessResponse> checkUpdateReadiness({
    required List<OtaArtifact> artifacts,
    String manifestSignature = '',
  });

  /// 查询升级历史
  Future<GetUpgradeHistoryResponse> getUpgradeHistory({
    String artifactFilter = '',
    int limit = 50,
  });

  /// 版本一致性校验
  Future<ValidateSystemVersionResponse> validateSystemVersion({
    String expectedSystemVersion = '',
    List<ComponentVersion> expectedComponents = const [],
  });

  // ==================== 系统信息 ====================

  /// 获取机器人信息 (ID, firmware/software versions)
  Future<RobotInfoResponse> getRobotInfo();

  /// 获取机器人能力 (支持的资源/任务)
  Future<CapabilitiesResponse> getCapabilities();

  /// 心跳检测 (RTT 测量, 保活)
  Future<HeartbeatResponse> heartbeat();

  // ==================== 设备管理 (OTA daemon) ====================

  /// 获取设备信息 (IP, 磁盘, 运行时间, 服务状态)
  Future<DeviceInfoResponse> getDeviceInfo();

  /// 管理系统服务 (start/stop/restart)
  Future<ManageServiceResponse> manageService({
    required String serviceName,
    required ServiceAction action,
  });

  // ==================== 资源发现 ====================

  /// 列出可用资源 (相机, 点云, 地图等)
  Future<ListResourcesResponse> listResources();

  /// 下载文件 (流式)
  Stream<FileChunk> downloadFile({
    required String filePath,
    int chunkSize = 65536,
  });

  /// 取消订阅资源
  Future<UnsubscribeResponse> unsubscribe({required String subscriptionId});

  Future<void> disconnect();
  bool get isConnected;

  /// OTA daemon (port 50052) 是否可用
  bool get otaAvailable => false;
  
  /// DataServiceClient for WebRTC signaling (null if mock/unsupported)
  DataServiceClient? get dataServiceClient => null;
}

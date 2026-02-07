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

  /// 启动任务（导航/建图）
  Future<StartTaskResponse> startTask({required TaskType taskType, String paramsJson = ''});

  /// 取消任务
  Future<CancelTaskResponse> cancelTask({required String taskId});

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
  });

  /// 应用固件（上传完成后触发刷写脚本）
  Future<ApplyFirmwareResponse> applyFirmware({required String firmwarePath});

  Future<void> disconnect();
  bool get isConnected;
  
  /// DataServiceClient for WebRTC signaling (null if mock/unsupported)
  DataServiceClient? get dataServiceClient => null;
}

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

  // ==================== 文件管理 (OTA) ====================

  /// 上传文件到机器人 (模型/地图/配置)
  /// [localBytes] 文件二进制内容
  /// [remotePath] 目标路径
  /// [filename] 文件名
  /// [category] 分类: "model", "map", "config", "firmware"
  /// [onProgress] 上传进度回调 (0.0 ~ 1.0)
  Future<UploadFileResponse> uploadFile({
    required List<int> localBytes,
    required String remotePath,
    required String filename,
    String category = 'model',
    bool overwrite = true,
    void Function(double progress)? onProgress,
  });

  /// 列出远程目录文件
  Future<ListRemoteFilesResponse> listRemoteFiles({
    required String directory,
    String category = '',
  });

  /// 删除远程文件
  Future<DeleteRemoteFileResponse> deleteRemoteFile({required String remotePath});

  Future<void> disconnect();
  bool get isConnected;
  
  /// DataServiceClient for WebRTC signaling (null if mock/unsupported)
  DataServiceClient? get dataServiceClient => null;
}

import 'package:fixnum/fixnum.dart';
import 'package:flutter/foundation.dart';
import 'package:uuid/uuid.dart';
import 'package:flutter_monitor/core/grpc/robot_client_base.dart';
import 'package:flutter_monitor/core/gateway/cloud_ota_client.dart';
import 'package:flutter_monitor/core/services/app_logger.dart';
import 'package:robot_proto/src/common.pb.dart';
import 'package:robot_proto/src/data.pb.dart';
import 'package:robot_proto/src/data.pbenum.dart';

// ================================================================
// Deployment Transaction Model (事务式部署)
// ================================================================

/// Deployment phase — state machine for a single OTA deployment.
enum DeployPhase {
  idle,
  downloading,  // Cloud → App memory
  uploading,    // App → Robot staging area
  checking,     // Readiness verification (battery, disk, hw compat, signature)
  applying,     // Robot applying update (backup → install → health check)
  validating,   // Post-apply version consistency check
  completed,    // All done
  failed,       // Error with recovery info
}

/// A single deployment session tracking the entire cloud→robot OTA flow.
///
/// Holds all state needed for progress display, error recovery, and resume.
/// Created by [OtaGateway.deployFromCloud] or [OtaGateway.deployFromLocal].
class DeploymentSession {
  final String id;
  final String artifactName;
  final OtaArtifact artifact;
  final CloudAsset? cloudAsset; // null for local file deploy

  DeployPhase phase;
  double progress; // 0.0 – 1.0 unified across all phases
  String? errorMessage;
  OtaFailureCode? failureCode;
  List<ReadinessCheck> readinessResults;

  // --- Resume state (survives retry) ---
  List<int>? cachedBytes;         // downloaded / local bytes — avoids re-download
  String? uploadedRemotePath;     // robot-side staged path after upload
  DeployPhase? failedAtPhase;     // which phase to resume from on retry

  bool _cancelled = false;

  DeploymentSession({
    required this.id,
    required this.artifactName,
    required this.artifact,
    this.cloudAsset,
    this.phase = DeployPhase.idle,
    this.progress = 0.0,
    this.errorMessage,
    this.failureCode,
    List<ReadinessCheck>? readinessResults,
    this.cachedBytes,
    this.uploadedRemotePath,
    this.failedAtPhase,
  }) : readinessResults = readinessResults ?? [];

  /// Can retry from the failure point?
  bool get canRetry => phase == DeployPhase.failed && failedAtPhase != null;

  /// Can rollback? (only if failure was during/after apply)
  bool get canRollback =>
      phase == DeployPhase.failed &&
      artifact.rollbackSafe &&
      (failedAtPhase == DeployPhase.applying ||
       failedAtPhase == DeployPhase.validating);

  /// Currently executing a phase?
  bool get isBusy =>
      phase != DeployPhase.idle &&
      phase != DeployPhase.completed &&
      phase != DeployPhase.failed;

  /// Human-readable label for current phase.
  String get phaseLabel {
    switch (phase) {
      case DeployPhase.idle:        return '准备中';
      case DeployPhase.downloading: return '正在从云端下载';
      case DeployPhase.uploading:   return '正在上传到机器人';
      case DeployPhase.checking:    return '安装预检查';
      case DeployPhase.applying:    return '正在应用更新';
      case DeployPhase.validating:  return '版本验证中';
      case DeployPhase.completed:   return '部署完成';
      case DeployPhase.failed:      return '部署失败';
    }
  }
}

// ================================================================
// OTA 数据模型（从 firmware_ota_page.dart 提取）
// ================================================================

/// 已安装的制品信息
class InstalledItem {
  final String name;
  final String version;
  final String category;

  InstalledItem({required this.name, required this.version, required this.category});
}

/// 可回滚的制品信息
class RollbackItem {
  final String name;
  final String version;

  RollbackItem({required this.name, required this.version});
}

/// 升级历史条目
class HistoryEntry {
  final String timestamp;
  final String action;
  final String artifactName;
  final String fromVersion;
  final String toVersion;
  final String status;

  HistoryEntry({
    required this.timestamp,
    required this.action,
    required this.artifactName,
    required this.fromVersion,
    required this.toVersion,
    required this.status,
  });
}

// ================================================================
// OtaGateway — OTA 业务逻辑 + 状态管理（从 Widget 解耦）
// ================================================================

/// OTA 网关：封装所有 OTA 业务逻辑和状态。
///
/// 通过 [ChangeNotifier] 驱动 UI 更新，Widget 只需 `context.watch<OtaGateway>()`
/// 读取状态并调用方法，不再直接操作 [RobotClientBase]。
class OtaGateway extends ChangeNotifier {
  RobotClientBase? _client;
  final CloudOtaClient cloud;

  /// Safety interlock: called before COLD apply.
  /// Returns true if safe to proceed (robot in maintenance mode).
  /// Wired in main.dart → checks ControlGateway state + transitions mode.
  Future<bool> Function()? safetyCheck;

  OtaGateway({RobotClientBase? client, CloudOtaClient? cloud})
      : _client = client,
        cloud = cloud ?? CloudOtaClient();

  /// 更新底层 gRPC 客户端（连接/断开时由 Provider 调用）
  void updateClient(RobotClientBase? client) {
    _client = client;
    if (client == null) {
      // 连接断开时重置与机器人相关的状态
      _installedVersions = [];
      _rollbackEntries = [];
      _historyEntries = [];
      _robotSystemVersion = null;
      // Active deployment in a robot-dependent phase → mark failed
      final s = _activeDeployment;
      if (s != null && s.isBusy && s.phase != DeployPhase.downloading) {
        s.failedAtPhase = s.phase;
        s.phase = DeployPhase.failed;
        s.errorMessage = '连接断开，可重连后重试';
      }
    }
    notifyListeners();
  }

  // ================================================================
  // 状态
  // ================================================================

  String? _statusMessage;
  String? get statusMessage => _statusMessage;

  // ---- 云端更新检查 ----
  bool _isCheckingCloud = false;
  CloudRelease? _latestRelease;
  String? _cloudError;

  bool get isCheckingCloud => _isCheckingCloud;
  CloudRelease? get latestRelease => _latestRelease;
  String? get cloudError => _cloudError;

  // ---- 已安装版本 ----
  bool _isLoadingVersions = false;
  List<InstalledItem> _installedVersions = [];
  List<RollbackItem> _rollbackEntries = [];
  String? _robotSystemVersion;

  bool get isLoadingVersions => _isLoadingVersions;
  List<InstalledItem> get installedVersions => _installedVersions;
  List<RollbackItem> get rollbackEntries => _rollbackEntries;
  String? get robotSystemVersion => _robotSystemVersion;

  // ---- 升级历史 ----
  bool _isLoadingHistory = false;
  List<HistoryEntry> _historyEntries = [];

  bool get isLoadingHistory => _isLoadingHistory;
  List<HistoryEntry> get historyEntries => _historyEntries;

  /// 清除当前状态消息
  void clearStatusMessage() {
    _statusMessage = null;
    notifyListeners();
  }

  // ================================================================
  // Deployment Transaction — 事务式部署编排
  //
  // 单入口 API：deployFromCloud / deployFromLocal
  // 内部按 downloading → uploading → checking → applying → validating
  // 推进状态机，UI 只需 watch activeDeployment 展示状态。
  // ================================================================

  DeploymentSession? _activeDeployment;

  /// The current active deployment session (null = idle).
  /// UI watches this for progress, phase label, error, retry state.
  DeploymentSession? get activeDeployment => _activeDeployment;

  /// One-click cloud deploy: download → upload → readiness → apply → validate.
  ///
  /// Idempotent: if a deployment is already in progress, this is a no-op.
  Future<void> deployFromCloud(CloudAsset asset) async {
    if (_activeDeployment?.isBusy == true) return;

    final artifact = _buildArtifact(asset);
    _activeDeployment = DeploymentSession(
      id: const Uuid().v4(),
      artifactName: asset.name,
      artifact: artifact,
      cloudAsset: asset,
      phase: DeployPhase.downloading,
    );
    notifyListeners();

    await _advanceDeployment();
  }

  /// One-click local deploy: upload → readiness → apply → validate.
  ///
  /// Skips the download phase; bytes are provided directly.
  Future<void> deployFromLocal(List<int> bytes, String filename) async {
    if (_activeDeployment?.isBusy == true) return;

    final artifact = _buildLocalArtifact(filename);
    _activeDeployment = DeploymentSession(
      id: const Uuid().v4(),
      artifactName: filename,
      artifact: artifact,
      phase: DeployPhase.uploading,
      cachedBytes: bytes,
    );
    notifyListeners();

    await _advanceDeployment();
  }

  /// Deploy by having the robot download directly from URL (robot-side download).
  ///
  /// Bypasses the phone completely — the robot pulls the file itself.
  /// Ideal for large files (ONNX models, firmware images) to avoid phone memory limits.
  /// Flow: robot downloads → check → apply → validate (no upload phase).
  Future<void> deployDirectFromUrl(CloudAsset asset) async {
    if (_activeDeployment?.isBusy == true) return;

    final artifact = _buildArtifact(asset);
    _activeDeployment = DeploymentSession(
      id: const Uuid().v4(),
      artifactName: asset.name,
      artifact: artifact,
      cloudAsset: asset,
      phase: DeployPhase.downloading,
    );
    notifyListeners();

    // Execute: robot downloads → check → apply → validate
    await _advanceDirectDownload();
  }

  /// Orchestrator for robot-direct-download path.
  Future<void> _advanceDirectDownload() async {
    final s = _activeDeployment;
    if (s == null) return;

    final client = _client;
    if (client == null) {
      s.phase = DeployPhase.failed;
      s.errorMessage = '未连接机器人';
      notifyListeners();
      return;
    }

    try {
      // Phase 1: Robot downloads from URL
      s.phase = DeployPhase.downloading;
      s.progress = 0.0;
      _statusMessage = '机器人正在下载 ${s.artifactName}…';
      notifyListeners();

      final asset = s.cloudAsset!;
      final category = _categoryStr(s.artifact.category.value);
      final stagingPath = '/staging/$category/${s.artifact.filename}';

      await for (final progress in client.downloadFromUrl(
        url: asset.downloadUrl,
        stagingPath: stagingPath,
        expectedSha256: s.artifact.sha256,
        expectedSize: asset.size,
      )) {
        if (s._cancelled) { _activeDeployment = null; notifyListeners(); return; }

        final total = progress.bytesTotal.toInt();
        final downloaded = progress.bytesCompleted.toInt();
        if (total > 0) {
          s.progress = _phaseProgress(DeployPhase.downloading, downloaded / total, hasDownload: true);
        }
        // Error: status == FAILED or message indicates failure
        if (progress.status == OtaUpdateStatus.OTA_UPDATE_STATUS_FAILED) {
          throw Exception('下载失败: ${progress.message}');
        }
        notifyListeners();
      }

      s.uploadedRemotePath = stagingPath;

      // Phase 2: Readiness check
      s.phase = DeployPhase.checking;
      notifyListeners();
      await _doReadinessCheck(s);
      if (s._cancelled) { _activeDeployment = null; notifyListeners(); return; }

      // Phase 3: Apply
      s.phase = DeployPhase.applying;
      notifyListeners();
      await _doApply(s);
      if (s._cancelled) { _activeDeployment = null; notifyListeners(); return; }

      // Phase 4: Validate
      s.phase = DeployPhase.validating;
      notifyListeners();
      await _doValidate(s);

      s.phase = DeployPhase.completed;
      s.progress = 1.0;
      _statusMessage = '${s.artifactName} 部署成功 (直接下载)';
      notifyListeners();

      try { await fetchInstalledVersions(); } catch (_) {}
    } catch (e) {
      s.failedAtPhase = s.phase;
      s.phase = DeployPhase.failed;
      s.errorMessage = e.toString();
      notifyListeners();
    }
  }

  /// Resume from the failure point (uses cached bytes / upload offset).
  Future<void> retryDeploy() async {
    final s = _activeDeployment;
    if (s == null || !s.canRetry) return;

    s.phase = s.failedAtPhase!;
    s.errorMessage = null;
    s.failureCode = null;
    s.readinessResults = [];
    s._cancelled = false;
    notifyListeners();

    await _advanceDeployment();
  }

  /// Cancel in-progress deployment or dismiss completed/failed state.
  void cancelDeploy() {
    final s = _activeDeployment;
    if (s == null) return;

    if (s.isBusy) {
      s._cancelled = true;
      // Orchestrator checks _cancelled between phases
    } else {
      _activeDeployment = null;
      notifyListeners();
    }
  }

  // ---- Internal Orchestrator ----

  /// Advances the deployment state machine through phases sequentially.
  /// Each phase method throws on failure; catch sets the session to [DeployPhase.failed].
  Future<void> _advanceDeployment() async {
    final s = _activeDeployment;
    if (s == null) return;

    if (s._cancelled) {
      _activeDeployment = null;
      notifyListeners();
      return;
    }

    try {
      switch (s.phase) {
        case DeployPhase.downloading:
          await _doDownload(s);
          if (s._cancelled) { _activeDeployment = null; notifyListeners(); return; }
          s.phase = DeployPhase.uploading;
          notifyListeners();
          await _advanceDeployment();

        case DeployPhase.uploading:
          await _doUpload(s);
          if (s._cancelled) { _activeDeployment = null; notifyListeners(); return; }
          s.phase = DeployPhase.checking;
          notifyListeners();
          await _advanceDeployment();

        case DeployPhase.checking:
          await _doReadinessCheck(s);
          if (s._cancelled) { _activeDeployment = null; notifyListeners(); return; }
          s.phase = DeployPhase.applying;
          notifyListeners();
          await _advanceDeployment();

        case DeployPhase.applying:
          await _doApply(s);
          if (s._cancelled) { _activeDeployment = null; notifyListeners(); return; }
          s.phase = DeployPhase.validating;
          notifyListeners();
          await _advanceDeployment();

        case DeployPhase.validating:
          await _doValidate(s);
          s.phase = DeployPhase.completed;
          s.progress = 1.0;
          _statusMessage = '${s.artifactName} 部署成功';
          notifyListeners();
          // Auto-refresh installed versions after success
          try { await fetchInstalledVersions(); } catch (_) {}

        default:
          return;
      }
    } catch (e) {
      s.failedAtPhase = s.phase;
      s.phase = DeployPhase.failed;
      s.errorMessage = e.toString();
      notifyListeners();
    }
  }

  // ---- Phase Implementations ----

  Future<void> _doDownload(DeploymentSession s) async {
    if (s.cachedBytes != null) return; // Already downloaded (resume case)

    final asset = s.cloudAsset;
    if (asset == null) throw Exception('无云端资源信息');

    s.progress = 0.0;
    notifyListeners();

    final bytes = await cloud.downloadAsset(
      asset,
      onProgress: (p) {
        s.progress = _phaseProgress(DeployPhase.downloading, p, hasDownload: true);
        notifyListeners();
      },
    );

    s.cachedBytes = bytes;
  }

  Future<void> _doUpload(DeploymentSession s) async {
    final client = _client;
    if (client == null) throw Exception('未连接机器人');

    final bytes = s.cachedBytes;
    if (bytes == null) throw Exception('无可上传数据');

    final hasDownload = s.cloudAsset != null;
    final category = _categoryStr(s.artifact.category.value);
    final remotePath = '/$category/${s.artifact.filename}';

    s.progress = _phaseProgress(DeployPhase.uploading, 0.0, hasDownload: hasDownload);
    notifyListeners();

    final response = await client.uploadFile(
      localBytes: bytes,
      remotePath: remotePath,
      filename: s.artifact.filename,
      category: category,
      overwrite: true,
      sha256: s.artifact.sha256,
      onProgress: (p) {
        s.progress = _phaseProgress(DeployPhase.uploading, p, hasDownload: hasDownload);
        notifyListeners();
      },
    );

    if (!response.success) {
      throw Exception('上传失败: ${response.message}');
    }

    s.uploadedRemotePath =
        response.remotePath.isNotEmpty ? response.remotePath : remotePath;
  }

  Future<void> _doReadinessCheck(DeploymentSession s) async {
    final client = _client;
    if (client == null) throw Exception('未连接机器人');

    final hasDownload = s.cloudAsset != null;
    s.progress = _phaseProgress(DeployPhase.checking, 0.0, hasDownload: hasDownload);
    notifyListeners();

    try {
      final response = await client.checkUpdateReadiness(
        artifacts: [s.artifact],
      );

      s.readinessResults = response.checks.toList();
      s.progress = _phaseProgress(DeployPhase.checking, 1.0, hasDownload: hasDownload);
      notifyListeners();

      if (!response.ready) {
        final failedChecks = response.checks
            .where((c) => !c.passed)
            .map((c) => c.message)
            .join('; ');
        throw Exception('预检查未通过: $failedChecks');
      }
    } catch (e) {
      if (e.toString().contains('UNIMPLEMENTED') ||
          e.toString().contains('Unimplemented')) {
        // ota_daemon 尚未实现 readiness check → 跳过（向后兼容）
        AppLogger.ota.info('readiness check not implemented, skipping');
        s.progress = _phaseProgress(DeployPhase.checking, 1.0, hasDownload: hasDownload);
        notifyListeners();
        return;
      }
      rethrow;
    }
  }

  Future<void> _doApply(DeploymentSession s) async {
    final client = _client;
    if (client == null) throw Exception('未连接机器人');
    if (s.uploadedRemotePath == null) throw Exception('无 staged 路径');

    final hasDownload = s.cloudAsset != null;
    s.progress = _phaseProgress(DeployPhase.applying, 0.0, hasDownload: hasDownload);
    notifyListeners();

    // Safety interlock: COLD updates require robot in maintenance mode
    if (s.artifact.safetyLevel == OtaSafetyLevel.OTA_SAFETY_LEVEL_COLD) {
      final check = safetyCheck;
      if (check != null) {
        final safe = await check();
        if (!safe) {
          s.failureCode = OtaFailureCode.OTA_FAILURE_SAFETY_MODE;
          throw Exception('安全检查未通过：机器人需进入维护模式后才能执行固件更新');
        }
      }
    }

    final response = await client.applyUpdate(
      artifact: s.artifact,
      stagedPath: s.uploadedRemotePath!,
    );

    s.progress = _phaseProgress(DeployPhase.applying, 1.0, hasDownload: hasDownload);
    notifyListeners();

    if (response.base.errorCode != ErrorCode.ERROR_CODE_OK) {
      s.failureCode = response.failureCode;
      throw Exception('应用更新失败: ${response.message}');
    }
  }

  Future<void> _doValidate(DeploymentSession s) async {
    final client = _client;
    if (client == null) throw Exception('未连接机器人');

    final hasDownload = s.cloudAsset != null;
    s.progress = _phaseProgress(DeployPhase.validating, 0.0, hasDownload: hasDownload);
    notifyListeners();

    try {
      final response = await client.validateSystemVersion(
        expectedSystemVersion: s.artifact.version,
      );

      s.progress = _phaseProgress(DeployPhase.validating, 1.0, hasDownload: hasDownload);
      notifyListeners();

      if (!response.consistent) {
        final details = response.mismatches.isNotEmpty
            ? response.mismatches
                .map((m) =>
                    '${m.componentName}: 期望${m.expectedVersion} 实际${m.actualVersion}')
                .join('; ')
            : '期望 ${s.artifact.version}, 实际 ${response.actualSystemVersion}';
        throw Exception('版本验证不一致: $details');
      }
    } catch (e) {
      if (e.toString().contains('UNIMPLEMENTED') ||
          e.toString().contains('Unimplemented')) {
        // ota_daemon 尚未实现 validate → 跳过（向后兼容）
        AppLogger.ota.info('validate not implemented, skipping');
        s.progress = _phaseProgress(DeployPhase.validating, 1.0, hasDownload: hasDownload);
        notifyListeners();
        return;
      }
      rethrow;
    }
  }

  // ---- Artifact Builders ----

  /// Build a fully-populated [OtaArtifact] from [CloudAsset] + manifest metadata.
  OtaArtifact _buildArtifact(CloudAsset asset) {
    final meta = asset.otaMeta;
    final artifact = OtaArtifact()
      ..name = meta?.name ?? asset.name
      ..category = _parseCategoryEnum(meta?.category ?? asset.category)
      ..version = meta?.version ?? _latestRelease?.tagName ?? 'unknown'
      ..filename = asset.name
      ..sha256 = meta?.sha256 ?? ''
      ..size = Int64(asset.size)
      ..targetPath = meta?.targetPath ?? ''
      ..targetBoard = meta?.targetBoard ?? 'nav'
      ..applyAction = _parseApplyAction(meta?.applyAction ?? 'copy_only')
      ..requiresReboot = meta?.requiresReboot ?? false
      ..minBatteryPercent = meta?.minBatteryPercent ?? 20
      ..changelog = meta?.changelog ?? ''
      ..rollbackSafe = meta?.rollbackSafe ?? true
      ..safetyLevel = _deriveSafetyLevel(meta?.category ?? asset.category);

    if (meta != null) {
      artifact.hwCompat.addAll(meta.hwCompat);
    }

    return artifact;
  }

  /// Build a minimal [OtaArtifact] for a local file (no manifest metadata).
  OtaArtifact _buildLocalArtifact(String filename) {
    final ext = filename.split('.').last.toLowerCase();
    final category = ['bin', 'hex', 'img', 'deb'].contains(ext)
        ? 'firmware'
        : ['pt', 'pth', 'onnx', 'tflite', 'engine'].contains(ext)
            ? 'model'
            : ['pcd', 'ply', 'pickle', 'pgm'].contains(ext)
                ? 'map'
                : ['yaml', 'yml', 'json', 'xml', 'cfg', 'ini'].contains(ext)
                    ? 'config'
                    : 'other';
    return OtaArtifact()
      ..name = filename
      ..category = _parseCategoryEnum(category)
      ..filename = filename
      ..targetBoard = 'nav'
      ..applyAction = OtaApplyAction.OTA_APPLY_ACTION_COPY_ONLY
      ..rollbackSafe = true
      ..safetyLevel = _deriveSafetyLevel(category)
      ..minBatteryPercent = (category == 'firmware' ? 30 : 20);
  }

  // ---- Progress Helpers ----

  /// Compute unified progress (0.0-1.0) given the current phase and intra-phase progress.
  static double _phaseProgress(
    DeployPhase phase, double intra, {required bool hasDownload}) {
    if (hasDownload) {
      switch (phase) {
        case DeployPhase.downloading: return intra * 0.30;
        case DeployPhase.uploading:   return 0.30 + intra * 0.30;
        case DeployPhase.checking:    return 0.60 + intra * 0.10;
        case DeployPhase.applying:    return 0.70 + intra * 0.20;
        case DeployPhase.validating:  return 0.90 + intra * 0.10;
        default: return 0.0;
      }
    } else {
      // Local deploy: no download phase, re-distribute weights
      switch (phase) {
        case DeployPhase.uploading:   return intra * 0.50;
        case DeployPhase.checking:    return 0.50 + intra * 0.10;
        case DeployPhase.applying:    return 0.60 + intra * 0.25;
        case DeployPhase.validating:  return 0.85 + intra * 0.15;
        default: return 0.0;
      }
    }
  }

  // ---- Enum Parsers ----

  static OtaCategory _parseCategoryEnum(String cat) {
    switch (cat) {
      case 'model':    return OtaCategory.OTA_CATEGORY_MODEL;
      case 'firmware': return OtaCategory.OTA_CATEGORY_FIRMWARE;
      case 'map':      return OtaCategory.OTA_CATEGORY_MAP;
      case 'config':   return OtaCategory.OTA_CATEGORY_CONFIG;
      default:         return OtaCategory.OTA_CATEGORY_UNSPECIFIED;
    }
  }

  static OtaApplyAction _parseApplyAction(String action) {
    switch (action) {
      case 'copy_only':      return OtaApplyAction.OTA_APPLY_ACTION_COPY_ONLY;
      case 'reload_model':   return OtaApplyAction.OTA_APPLY_ACTION_RELOAD_MODEL;
      case 'install_deb':    return OtaApplyAction.OTA_APPLY_ACTION_INSTALL_DEB;
      case 'flash_mcu':      return OtaApplyAction.OTA_APPLY_ACTION_FLASH_MCU;
      case 'install_script': return OtaApplyAction.OTA_APPLY_ACTION_INSTALL_SCRIPT;
      default:               return OtaApplyAction.OTA_APPLY_ACTION_UNSPECIFIED;
    }
  }

  static OtaSafetyLevel _deriveSafetyLevel(String category) {
    switch (category) {
      case 'firmware': return OtaSafetyLevel.OTA_SAFETY_LEVEL_COLD;
      case 'model':    return OtaSafetyLevel.OTA_SAFETY_LEVEL_WARM;
      case 'map':
      case 'config':   return OtaSafetyLevel.OTA_SAFETY_LEVEL_HOT;
      default:         return OtaSafetyLevel.OTA_SAFETY_LEVEL_UNSPECIFIED;
    }
  }

  // ================================================================
  // Cloud & Version Methods
  // ================================================================

  /// 从 GitHub 检查最新版本
  Future<void> checkCloudUpdate() async {
    _isCheckingCloud = true;
    _cloudError = null;
    _latestRelease = null;
    notifyListeners();

    try {
      final release = await cloud.fetchLatestRelease();
      _isCheckingCloud = false;
      _latestRelease = release;
      if (release == null) {
        _cloudError = '未找到任何 Release';
      }
      notifyListeners();
    } catch (e) {
      _isCheckingCloud = false;
      _cloudError = '检查更新失败: $e';
      notifyListeners();
    }
  }

  /// 获取已安装版本列表
  Future<void> fetchInstalledVersions() async {
    final client = _client;
    if (client == null) throw Exception('未连接');

    _isLoadingVersions = true;
    notifyListeners();

    try {
      final response = await client.getInstalledVersions();
      _isLoadingVersions = false;
      _robotSystemVersion = response.systemVersion;
      _installedVersions = response.installed
          .map((a) => InstalledItem(
                name: a.name,
                version: a.version,
                category: _categoryStr(a.category.value),
              ))
          .toList();
      _rollbackEntries = response.rollbackAvailable
          .map((r) => RollbackItem(
                name: r.name,
                version: r.version,
              ))
          .toList();
      notifyListeners();
    } catch (e) {
      _isLoadingVersions = false;
      _statusMessage = '获取版本信息失败: $e';
      notifyListeners();
    }
  }

  /// 执行回滚
  ///
  /// 注意：确认对话框由 UI 层处理，这里只执行回滚操作。
  Future<bool> performRollback(String artifactName) async {
    final client = _client;
    if (client == null) throw Exception('未连接');

    try {
      final response = await client.rollback(artifactName: artifactName);
      _statusMessage = response.success
          ? '$artifactName 已回滚到 v${response.restoredVersion}'
          : '回滚失败: ${response.message}';
      notifyListeners();

      // 回滚成功后自动刷新版本列表
      if (response.success) {
        await fetchInstalledVersions();
      }
      return response.success;
    } catch (e) {
      _statusMessage = '回滚失败: $e';
      notifyListeners();
      return false;
    }
  }

  /// 获取升级历史
  Future<void> fetchUpgradeHistory() async {
    final client = _client;
    if (client == null) throw Exception('未连接');

    _isLoadingHistory = true;
    notifyListeners();

    try {
      final response = await client.getUpgradeHistory(limit: 20);
      _isLoadingHistory = false;
      _historyEntries = response.entries
          .map((e) => HistoryEntry(
                timestamp: e.timestamp,
                action: e.action,
                artifactName: e.artifactName,
                fromVersion: e.fromVersion,
                toVersion: e.toVersion,
                status: e.status,
              ))
          .toList();
      notifyListeners();
    } catch (e) {
      _isLoadingHistory = false;
      _statusMessage = '获取升级历史失败: $e';
      notifyListeners();
    }
  }

  // ================================================================
  // 工具方法
  // ================================================================

  static String _categoryStr(int value) {
    switch (value) {
      case 1: return 'model';
      case 2: return 'firmware';
      case 3: return 'map';
      case 4: return 'config';
      default: return 'other';
    }
  }
}

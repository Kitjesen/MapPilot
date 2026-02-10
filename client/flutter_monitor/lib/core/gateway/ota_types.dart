/// 统一 OTA 类型导出。
///
/// 将 proto 生成的 OTA 类型和云端 OTA 模型集中到一个入口，
/// 方便其他文件 import。

// Proto OTA request/response types
export 'package:robot_proto/src/data.pb.dart'
    show
        ApplyUpdateRequest,
        ApplyUpdateResponse,
        GetInstalledVersionsRequest,
        GetInstalledVersionsResponse,
        RollbackRequest,
        RollbackResponse,
        DownloadFromUrlRequest,
        OtaProgress,
        CheckUpdateReadinessRequest,
        CheckUpdateReadinessResponse,
        ReadinessCheck,
        OtaArtifact,
        InstalledArtifact,
        RollbackEntry,
        GetUpgradeHistoryRequest,
        GetUpgradeHistoryResponse,
        ValidateSystemVersionRequest,
        ValidateSystemVersionResponse,
        ComponentVersion,
        ApplyFirmwareRequest,
        ApplyFirmwareResponse;

// Proto OTA enums
export 'package:robot_proto/src/data.pbenum.dart'
    show OtaCategory, OtaApplyAction, OtaUpdateStatus;

// Cloud OTA models
export 'package:flutter_monitor/core/gateway/cloud_ota_client.dart'
    show
        CloudAsset,
        CloudRelease,
        CloudOtaManifest,
        CloudOtaArtifactMeta,
        CloudOtaClient;

// OTA Gateway
export 'package:flutter_monitor/core/gateway/ota_gateway.dart'
    show OtaGateway, InstalledItem, RollbackItem, HistoryEntry;

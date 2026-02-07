// Re-export OTA types from generated proto.
// All OTA types are defined in data.proto.

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
        RollbackEntry;

export 'package:robot_proto/src/data.pbenum.dart'
    show
        OtaCategory,
        OtaApplyAction,
        OtaUpdateStatus;

/// **Deprecated** — Use `package:flutter_monitor/core/gateway/cloud_ota_client.dart` instead.
///
/// This file is kept only for backward compatibility and will be removed
/// in a future version. The class has been renamed from `CloudOtaService` to
/// `CloudOtaClient`.
@Deprecated('Use package:flutter_monitor/core/gateway/cloud_ota_client.dart instead')
library cloud_ota_service;

export 'package:flutter_monitor/core/gateway/cloud_ota_client.dart';

// Backward-compatible alias: CloudOtaService -> CloudOtaClient
import 'package:flutter_monitor/core/gateway/cloud_ota_client.dart';

/// @nodoc
@Deprecated('Use CloudOtaClient instead')
typedef CloudOtaService = CloudOtaClient;

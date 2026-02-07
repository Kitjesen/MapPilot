import 'package:robot_proto/src/common.pb.dart';

/// Temporary placeholder for ApplyFirmwareRequest proto message.
/// TODO: Remove this file once data.proto is updated with the ApplyFirmware RPC
/// and the generated Dart code is available.
class ApplyFirmwareRequest {
  RequestBase base = RequestBase();
  String firmwarePath = '';
}

/// Temporary placeholder for ApplyFirmwareResponse proto message.
class ApplyFirmwareResponse {
  ResponseBase base = ResponseBase();
  bool success = false;
  String message = '';
}

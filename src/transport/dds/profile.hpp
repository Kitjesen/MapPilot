#pragma once

namespace lingtu::dds {

enum class QosProfile {
  Default,
  SensorStream,
  RawLidarStream,
  HighFreqState,
  LocalizationHealth,
  ControlCommand,
  FinalVelocityCommand,
  CommandRequest,
  CommandAck,
  OperatorMotionControl,
  OperatorMotionSample,
  OperatorMotionAck,
  OperatorMotionStatus,
  InspectionEvidence,
  TaskEvent,
  CameraStream,
  CameraInfo,
  GlobalPath,
  SystemStatus,
  Event,
  MapGrid,
  LocalRiskGrid,
  TfDynamic,
  TfStatic,
  LidarPointcloud,
  MapCloud,
  MapScene,
};

}  // namespace lingtu::dds

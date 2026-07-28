"""Native DDS wire types owned by the message package.

The package-level namespace preserves the historical ``message.dds_types``
exports without importing every message family up front. In particular, Livox
conversion helpers depend on NumPy-backed frame utilities, so those names must
stay lazy for control-plane imports that only need navigation or scalar types.
"""

from __future__ import annotations

from importlib import import_module
from typing import Any

_EXPORTS: dict[str, str] = {
    "Bool": ".scalar",
    "CameraInfo": ".camera",
    "DDS_Bool": ".scalar",
    "DDS_CameraInfo": ".camera",
    "DDS_Float32": ".scalar",
    "DDS_ExplorationCommandAck": ".nav",
    "DDS_ExplorationCommandRequest": ".nav",
    "DDS_ExplorationGrid": ".nav",
    "DDS_ExplorationExecutionGrid": ".nav",
    "DDS_ExplorationSegmentAck": ".nav",
    "DDS_ExplorationSegmentRequest": ".nav",
    "DDS_ExplorationSegmentStatus": ".nav",
    "DDS_FinalVelocityCommand": ".geometry",
    "DDS_GnssFix": ".gnss",
    "DDS_GnssStatus": ".gnss",
    "DDS_Header": ".common",
    "DDS_Image": ".camera",
    "DDS_Imu": ".imu",
    "DDS_InspectionCommandAck": ".nav",
    "DDS_InspectionCommandRequest": ".nav",
    "DDS_InspectionTaskAck": ".nav",
    "DDS_InspectionTaskRequest": ".nav",
    "DDS_InspectionEvidenceRequest": ".nav",
    "DDS_InspectionEvidenceResult": ".nav",
    "DDS_InspectionStatus": ".nav",
    "DDS_InspectionTaskEvent": ".nav",
    "DDS_MapMetaData": ".nav",
    "DDS_MapCloudLayer": ".nav",
    "DDS_MapGrid": ".nav",
    "DDS_MapObservation": ".nav",
    "DDS_MapRuntimeState": ".nav",
    "DDS_MapScene": ".nav",
    "DDS_NavigationCommandAck": ".nav",
    "DDS_NavigationGoalStatus": ".nav",
    "DDS_NavigationState": ".nav",
    "DDS_NavigationCommandRequest": ".nav",
    "DDS_OperatorMotionAck": ".nav",
    "DDS_OperatorMotionControl": ".nav",
    "DDS_OperatorMotionSample": ".nav",
    "DDS_OperatorMotionStatus": ".nav",
    "DDS_OccupancyGrid": ".nav",
    "DDS_Odometry": ".nav",
    "DDS_Path": ".nav",
    "DDS_Point": ".common",
    "DDS_PointCloud2": ".pointcloud",
    "DDS_PointField": ".pointcloud",
    "DDS_Pose": ".geometry",
    "DDS_PoseStamped": ".geometry",
    "DDS_PoseWithCovariance": ".geometry",
    "DDS_Quaternion": ".common",
    "DDS_RelocalizationRequest": ".slam",
    "DDS_RelocalizationResponse": ".slam",
    "DDS_String": ".scalar",
    "DDS_TFMessage": ".geometry",
    "DDS_Time": ".common",
    "DDS_Transform": ".geometry",
    "DDS_TransformStamped": ".geometry",
    "DDS_Twist": ".geometry",
    "DDS_TwistStamped": ".geometry",
    "DDS_TwistWithCovariance": ".geometry",
    "ExplorationCommandAck": ".nav",
    "ExplorationCommandRequest": ".nav",
    "ExplorationGrid": ".nav",
    "ExplorationExecutionGrid": ".nav",
    "ExplorationSegmentAck": ".nav",
    "ExplorationSegmentRequest": ".nav",
    "ExplorationSegmentStatus": ".nav",
    "DDS_Vector3": ".common",
    "Float32": ".scalar",
    "FinalVelocityCommand": ".geometry",
    "GnssFix": ".gnss",
    "GnssStatus": ".gnss",
    "Header": ".common",
    "Image": ".camera",
    "Imu": ".imu",
    "InspectionCommandAck": ".nav",
    "InspectionCommandRequest": ".nav",
    "InspectionTaskAck": ".nav",
    "InspectionTaskRequest": ".nav",
    "InspectionEvidenceRequest": ".nav",
    "InspectionEvidenceResult": ".nav",
    "InspectionStatus": ".nav",
    "InspectionTaskEvent": ".nav",
    "LivoxCustomMsg": ".livox",
    "LivoxFrame": ".livox",
    "LivoxPoint": ".livox",
    "MapMetaData": ".nav",
    "MapCloudLayer": ".nav",
    "MapGrid": ".nav",
    "MapObservation": ".nav",
    "MapRuntimeState": ".nav",
    "MapScene": ".nav",
    "NavigationCommandAck": ".nav",
    "NavigationGoalStatus": ".nav",
    "NavigationState": ".nav",
    "NavigationCommandRequest": ".nav",
    "OperatorMotionAck": ".nav",
    "OperatorMotionControl": ".nav",
    "OperatorMotionSample": ".nav",
    "OperatorMotionStatus": ".nav",
    "OccupancyGrid": ".nav",
    "Odometry": ".nav",
    "Path": ".nav",
    "Point": ".common",
    "PointCloud2": ".pointcloud",
    "PointField": ".pointcloud",
    "Pose": ".geometry",
    "PoseStamped": ".geometry",
    "PoseWithCovariance": ".geometry",
    "Quaternion": ".common",
    "RelocalizationRequest": ".slam",
    "RelocalizationResponse": ".slam",
    "String": ".scalar",
    "TFMessage": ".geometry",
    "Text": ".scalar",
    "Time": ".common",
    "Transform": ".geometry",
    "TransformStamped": ".geometry",
    "Twist": ".geometry",
    "TwistStamped": ".geometry",
    "TwistWithCovariance": ".geometry",
    "Vector3": ".common",
    "dds_imu_to_imu": ".imu",
    "livox_frame_to_msg": ".livox",
    "livox_msg_to_frame": ".livox",
    "livox_msg_to_numpy": ".livox",
}

__all__ = list(_EXPORTS)


def __getattr__(name: str) -> Any:
    try:
        module_name = _EXPORTS[name]
    except KeyError as exc:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}") from exc

    value = getattr(import_module(module_name, __name__), name)
    globals()[name] = value
    return value


def __dir__() -> list[str]:
    return sorted([*globals(), *__all__])

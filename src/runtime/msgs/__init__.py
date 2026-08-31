"""runtime.msgs — unified message types for inter-module data flow.

All Module In/Out ports use these types. No ROS2 dependency.

Every message class satisfies the :class:`LingtuMsg` protocol:
``msg_name`` + ``encode()`` + ``decode()``.
"""

from .geometry import Pose, PoseStamped, Quaternion, Transform, Twist, TwistStamped, Vector3
from .map import (
    MapCloudFrame,
    MapObservationFrame,
    SemanticLabelsFrame,
    SemanticSaveRequest,
    SemanticSaveResult,
)
from .nav import (
    ExplorationRunEvent,
    ExplorationRunEventKind,
    ExplorationRunState,
    InspectionTaskEvent,
    InspectionTaskEventKind,
    InspectionTaskState,
    NavigationCommandKind,
    NavigationCommandReceipt,
    NavigationControlMode,
    NavigationExecutionState,
    NavigationGoalState,
    NavigationGoalStatus,
    NavigationLifecycle,
    NavigationPlanningState,
    NavigationRecoveryState,
    NavigationState,
    OccupancyGrid,
    Odometry,
    OperatorMotionAction,
    OperatorMotionReceipt,
    Path,
)
from .protocol import LingtuMsg, is_lingtu_msg, resolve_msg_type
from .semantic import (
    Detection3D,
    GoalResult,
    NavigationCommand,
    Region,
    Relation,
    SceneGraph,
)
from .sensor import CameraIntrinsics, Image, ImageFormat, Imu, PointCloud2, PointField

__all__ = [
    "CameraIntrinsics",
    "Detection3D",
    "ExplorationRunEvent",
    "ExplorationRunEventKind",
    "ExplorationRunState",
    "GoalResult",
    "Image",
    "ImageFormat",
    "Imu",
    "InspectionTaskEvent",
    "InspectionTaskEventKind",
    "InspectionTaskState",
    "LingtuMsg",
    "MapCloudFrame",
    "MapObservationFrame",
    "NavigationCommand",
    "NavigationCommandKind",
    "NavigationCommandReceipt",
    "NavigationControlMode",
    "NavigationExecutionState",
    "NavigationGoalState",
    "NavigationGoalStatus",
    "NavigationLifecycle",
    "NavigationPlanningState",
    "NavigationRecoveryState",
    "NavigationState",
    "OccupancyGrid",
    "Odometry",
    "OperatorMotionAction",
    "OperatorMotionReceipt",
    "Path",
    "PointCloud2",
    "PointField",
    "Pose",
    "PoseStamped",
    "Quaternion",
    "Region",
    "Relation",
    "SceneGraph",
    "SemanticLabelsFrame",
    "SemanticSaveRequest",
    "SemanticSaveResult",
    "Transform",
    "Twist",
    "TwistStamped",
    "Vector3",
    "is_lingtu_msg",
    "resolve_msg_type",
]

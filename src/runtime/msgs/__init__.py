"""runtime.msgs — unified message types for inter-module data flow.

All Module In/Out ports use these types. No ROS2 dependency.

Every message class satisfies the :class:`LingtuMsg` protocol:
``msg_name`` + ``encode()`` + ``decode()``.
"""

from .geometry import Pose, PoseStamped, Quaternion, Transform, Twist, TwistStamped, Vector3
from .map import MapCloudFrame
from .nav import OccupancyGrid, Odometry, Path
from .protocol import LingtuMsg, is_lingtu_msg, resolve_msg_type
from .robot import BatteryState, FootForces, JointState, RobotState
from .semantic import (
    Detection3D,
    DialogueState,
    ExecutionEval,
    GoalResult,
    MissionStatus,
    NavigationCommand,
    Region,
    Relation,
    SafetyState,
    SceneGraph,
)
from .sensor import CameraIntrinsics, Image, ImageFormat, Imu, PointCloud2, PointField

__all__ = [
    "BatteryState",
    "CameraIntrinsics",
    "Detection3D",
    "DialogueState",
    "ExecutionEval",
    "FootForces",
    "GoalResult",
    "Image",
    "ImageFormat",
    "Imu",
    "JointState",
    "LingtuMsg",
    "MapCloudFrame",
    "MissionStatus",
    "NavigationCommand",
    "OccupancyGrid",
    "Odometry",
    "Path",
    "PointCloud2",
    "PointField",
    "Pose",
    "PoseStamped",
    "Quaternion",
    "Region",
    "Relation",
    "RobotState",
    "SafetyState",
    "SceneGraph",
    "Transform",
    "Twist",
    "TwistStamped",
    "Vector3",
    "is_lingtu_msg",
    "resolve_msg_type",
]

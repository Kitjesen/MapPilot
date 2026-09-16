# Generated from message/idl/constants.idl. Do not edit.
from enum import IntEnum


class NavigationControlMode(IntEnum):
    UNKNOWN = 0
    AUTONOMY = 1
    TELEOP = 2
    TELEOP_AVOID = 3


class NavigationLifecycle(IntEnum):
    IDLE = 0
    PLANNING = 1
    EXECUTING = 2
    PAUSED = 3
    RECOVERING = 4
    SUCCESS = 5
    FAILED = 6
    CANCELLED = 7


class NavigationPlanningState(IntEnum):
    IDLE = 0
    PLANNING = 1
    READY = 2
    FAILED = 3


class NavigationExecutionState(IntEnum):
    IDLE = 0
    FOLLOWING = 1
    REACHED = 2
    BLOCKED = 3


class NavigationRecoveryState(IntEnum):
    IDLE = 0
    ACTIVE = 1
    SUCCEEDED = 2
    FAILED = 3


class NavigationGoalState(IntEnum):
    PLANNING = 1
    PATH_ACTIVE = 2
    FAILED = 3
    REACHED = 4
    CANCELLED = 5
    PAUSED = 6


class InspectionTaskEventKind(IntEnum):
    TASK_ACCEPTED = 1
    STATE_CHANGED = 2
    MILESTONE = 3
    STOP_CONFIRMATION_FAILED = 4
    EVIDENCE_RECORDED = 5


class InspectionTaskState(IntEnum):
    IDLE = 0
    VALIDATING = 1
    PLANNING = 2
    NAVIGATING = 3
    DWELLING = 4
    PAUSED = 5
    RECOVERING = 6
    SUCCEEDED = 7
    FAILED = 8
    CANCELLED = 9
    SETTLING = 10
    ACTION_PENDING = 11
    PAUSING = 12
    CANCELLING = 13


class ExplorationRunEventKind(IntEnum):
    ADMITTED = 1
    STATE_CHANGED = 2
    STOP_CONFIRMATION_FAILED = 3


class ExplorationRunState(IntEnum):
    ADMITTED = 1
    RUNNING = 2
    PAUSING = 3
    PAUSED = 4
    CANCELLING = 5
    COMPLETED = 6
    CANCELLED = 7
    FAILED = 8


class NavigationCommandKind(IntEnum):
    GOAL = 1
    TASK_CANCEL = 2
    STOP = 4
    ESTOP = 5
    CLEAR_ESTOP = 6
    RESUME_AUTONOMY = 7
    PAUSE_TASK = 8
    RESUME_TASK = 9


class OperatorMotionAction(IntEnum):
    CLAIM = 1
    RELEASE = 2
    HOLD = 3


class ExplorationCommandKind(IntEnum):
    START = 1
    PAUSE = 2
    RESUME = 3
    STOP = 4
    SET_DIRECTED_TARGET = 5
    CLEAR_DIRECTED_TARGET = 6


class InspectionCommandKind(IntEnum):
    START = 1
    PAUSE = 2
    RESUME = 3
    CANCEL = 4


class GeofenceAction(IntEnum):
    ADD = 1
    REMOVE = 2
    CLEAR = 3
    ENABLE = 4
    DISABLE = 5
    LIST = 6

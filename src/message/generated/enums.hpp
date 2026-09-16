// Generated from message/idl/constants.idl. Do not edit.
#pragma once
#include <cstdint>
namespace lingtu::message {
enum class NavigationControlState : std::int32_t {
  kUnknown = 0,
  kAutonomy = 1,
  kTeleop = 2,
  kTeleopAvoid = 3,
};
enum class NavigationLifecycleState : std::int32_t {
  kIdle = 0,
  kPlanning = 1,
  kExecuting = 2,
  kPaused = 3,
  kRecovering = 4,
  kSuccess = 5,
  kFailed = 6,
  kCancelled = 7,
};
enum class NavigationPlanningState : std::int32_t {
  kIdle = 0,
  kPlanning = 1,
  kReady = 2,
  kFailed = 3,
};
enum class NavigationExecutionState : std::int32_t {
  kIdle = 0,
  kFollowing = 1,
  kReached = 2,
  kBlocked = 3,
};
enum class NavigationRecoveryState : std::int32_t {
  kIdle = 0,
  kActive = 1,
  kSucceeded = 2,
  kFailed = 3,
};
enum class NavigationGoalState : std::int32_t {
  Planning = 1,
  PathActive = 2,
  Failed = 3,
  Reached = 4,
  Cancelled = 5,
  Paused = 6,
};
enum class InspectionTaskEventKind : std::int32_t {
  kTaskAccepted = 1,
  kStateChanged = 2,
  kMilestone = 3,
  kStopConfirmationFailed = 4,
  kEvidenceRecorded = 5,
};
enum class InspectionTaskState : std::int32_t {
  kIdle = 0,
  kValidating = 1,
  kPlanning = 2,
  kNavigating = 3,
  kDwelling = 4,
  kPaused = 5,
  kRecovering = 6,
  kSucceeded = 7,
  kFailed = 8,
  kCancelled = 9,
  kSettling = 10,
  kActionPending = 11,
  kPausing = 12,
  kCancelling = 13,
};
enum class ExplorationRunEventKind : std::int32_t {
  kAdmitted = 1,
  kStateChanged = 2,
  kStopConfirmationFailed = 3,
};
enum class ExplorationRunState : std::int32_t {
  kAdmitted = 1,
  kRunning = 2,
  kPausing = 3,
  kPaused = 4,
  kCancelling = 5,
  kCompleted = 6,
  kCancelled = 7,
  kFailed = 8,
};
enum class NavigationCommandKind : std::int32_t {
  Goal = 1,
  TaskCancel = 2,
  Stop = 4,
  Estop = 5,
  ClearEstop = 6,
  ResumeAutonomy = 7,
  TaskPause = 8,
  TaskResume = 9,
};
enum class OperatorMotionAction : std::int32_t {
  Claim = 1,
  Release = 2,
  Hold = 3,
};
enum class ExplorationCommandKind : std::int32_t {
  kStart = 1,
  kPause = 2,
  kResume = 3,
  kStop = 4,
  kSetDirectedTarget = 5,
  kClearDirectedTarget = 6,
};
enum class InspectionCommandKind : std::int32_t {
  kStart = 1,
  kPause = 2,
  kResume = 3,
  kCancel = 4,
};
enum class GeofenceAction : std::int32_t {
  kAdd = 1,
  kRemove = 2,
  kClear = 3,
  kEnable = 4,
  kDisable = 5,
  kList = 6,
};
}  // namespace lingtu::message

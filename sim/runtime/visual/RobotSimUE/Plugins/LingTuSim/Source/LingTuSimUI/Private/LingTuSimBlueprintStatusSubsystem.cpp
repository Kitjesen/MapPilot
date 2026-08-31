#include "LingTuSimBlueprintStatusSubsystem.h"

#include "Engine/World.h"
#include "LingTuSimControlTransport.h"
#include "LingTuSimRuntimeUIStatus.h"
#include "LingTuSimRuntimeUIWorldSubsystem.h"

namespace {
constexpr float BlueprintStatusRefreshIntervalSeconds = 0.1F;

int64 ToBlueprintInt64(const uint64 Value) {
  return Value > static_cast<uint64>(MAX_int64) ? MAX_int64 : static_cast<int64>(Value);
}

const TCHAR *RuntimeStateName(const LingTuSim::EControlStatusRuntimeState State) {
  switch (State) {
    case LingTuSim::EControlStatusRuntimeState::New:
      return TEXT("New");
    case LingTuSim::EControlStatusRuntimeState::Preparing:
      return TEXT("Preparing");
    case LingTuSim::EControlStatusRuntimeState::Ready:
      return TEXT("Ready");
    case LingTuSim::EControlStatusRuntimeState::Running:
      return TEXT("Running");
    case LingTuSim::EControlStatusRuntimeState::Paused:
      return TEXT("Paused");
    case LingTuSim::EControlStatusRuntimeState::Stopped:
      return TEXT("Stopped");
    case LingTuSim::EControlStatusRuntimeState::Failed:
      return TEXT("Failed");
    default:
      return TEXT("Unavailable");
  }
}

const TCHAR *SafeStopStateName(const LingTuSim::EControlSafeStopState State) {
  switch (State) {
    case LingTuSim::EControlSafeStopState::Clear:
      return TEXT("Clear");
    case LingTuSim::EControlSafeStopState::Pending:
      return TEXT("Pending");
    case LingTuSim::EControlSafeStopState::Zeroed:
      return TEXT("Zeroed");
    case LingTuSim::EControlSafeStopState::Blocked:
      return TEXT("Blocked");
    case LingTuSim::EControlSafeStopState::Unavailable:
    default:
      return TEXT("Unavailable");
  }
}

const TCHAR *RecordingStateName(const LingTuSim::EControlRecordingState State) {
  switch (State) {
    case LingTuSim::EControlRecordingState::Idle:
      return TEXT("Idle");
    case LingTuSim::EControlRecordingState::Requested:
      return TEXT("Requested");
    case LingTuSim::EControlRecordingState::Recording:
      return TEXT("Recording");
    case LingTuSim::EControlRecordingState::Committed:
      return TEXT("Committed");
    case LingTuSim::EControlRecordingState::Rejected:
      return TEXT("Rejected");
    case LingTuSim::EControlRecordingState::Failed:
      return TEXT("Failed");
    case LingTuSim::EControlRecordingState::Unavailable:
    default:
      return TEXT("Unavailable");
  }
}
}  // namespace

bool FLingTuSimBlueprintRuntimeStatus::operator==(
    const FLingTuSimBlueprintRuntimeStatus &Other) const {
  return bPresentationReady == Other.bPresentationReady &&
         bSessionAvailable == Other.bSessionAvailable && SessionState == Other.SessionState &&
         SessionId == Other.SessionId && ModelGeneration == Other.ModelGeneration &&
         bControlBindingAvailable == Other.bControlBindingAvailable &&
         ControlState == Other.ControlState && RunId == Other.RunId &&
         ResetGeneration == Other.ResetGeneration && bIdentityCoherent == Other.bIdentityCoherent &&
         Blocker == Other.Blocker && RuntimeState == Other.RuntimeState &&
         ControlOwner == Other.ControlOwner && bDeadman == Other.bDeadman &&
         SafeStopState == Other.SafeStopState &&
         bLatestControlAckAvailable == Other.bLatestControlAckAvailable &&
         bLatestControlAckAccepted == Other.bLatestControlAckAccepted &&
         LatestControlAckState == Other.LatestControlAckState &&
         LatestControlAckReason == Other.LatestControlAckReason &&
         bFullStatusAvailable == Other.bFullStatusAvailable &&
         bFullStatusFresh == Other.bFullStatusFresh &&
         FullStatusSequence == Other.FullStatusSequence && VisualState == Other.VisualState &&
         BodyBindingCount == Other.BodyBindingCount &&
         ScenarioActorCount == Other.ScenarioActorCount &&
         bLatestAppliedTruthAvailable == Other.bLatestAppliedTruthAvailable &&
         TruthSequence == Other.TruthSequence && SimTimeNanoseconds == Other.SimTimeNanoseconds &&
         bRequestedAxesAvailable == Other.bRequestedAxesAvailable &&
         RequestedForward == Other.RequestedForward && RequestedLeft == Other.RequestedLeft &&
         RequestedYawLeft == Other.RequestedYawLeft &&
         bAdmittedTwistAvailable == Other.bAdmittedTwistAvailable &&
         AdmittedLinearVelocityMps == Other.AdmittedLinearVelocityMps &&
         AdmittedAngularVelocityRadps == Other.AdmittedAngularVelocityRadps &&
         bObservedBaseVelocityAvailable == Other.bObservedBaseVelocityAvailable &&
         ObservedBaseStableId == Other.ObservedBaseStableId &&
         ObservedBaseLinearVelocityMps == Other.ObservedBaseLinearVelocityMps &&
         ObservedBaseAngularVelocityRadps == Other.ObservedBaseAngularVelocityRadps &&
         RecordingState == Other.RecordingState && ActualUIMode == Other.ActualUIMode &&
         ActualCameraMode == Other.ActualCameraMode;
}

FLingTuSimBlueprintRuntimeStatus
LingTuSim::UI::FBlueprintRuntimeStatusProjection::Project(const FRuntimeUIStatusSnapshot &Source) {
  FLingTuSimBlueprintRuntimeStatus Result;
  Result.bPresentationReady = Source.bSessionAvailable && Source.bControlBindingAvailable &&
                              Source.bIdentityCoherent && Source.VisualState == TEXT("Active") &&
                              Source.bLatestAppliedTruthAvailable && Source.bFullStatusAvailable &&
                              Source.bFullStatusFresh && Source.bFullStatusIdentityCoherent;

  Result.bSessionAvailable = Source.bSessionAvailable;
  Result.SessionState = Source.SessionState;
  Result.SessionId = Source.SessionId;
  Result.ModelGeneration = ToBlueprintInt64(Source.ModelGeneration);
  Result.bControlBindingAvailable = Source.bControlBindingAvailable;
  Result.ControlState = Source.ControlState;
  Result.RunId = Source.RunId;
  Result.ResetGeneration = ToBlueprintInt64(Source.ResetGeneration);
  Result.bIdentityCoherent = Source.bIdentityCoherent;
  Result.Blocker = Source.Blocker;

  Result.bLatestControlAckAvailable = Source.bControlAckAvailable;
  Result.bLatestControlAckAccepted = Source.bControlAckAccepted;
  Result.LatestControlAckState = Source.ControlAckState;
  Result.LatestControlAckReason = Source.ControlAckReason;

  Result.bFullStatusAvailable = Source.bFullStatusAvailable;
  Result.bFullStatusFresh = Source.bFullStatusFresh;
  if (Source.bFullStatusAvailable) {
    Result.FullStatusSequence = ToBlueprintInt64(Source.FullStatus.ServerStatusSequence);
    Result.RuntimeState = RuntimeStateName(Source.FullStatus.Runtime.RuntimeState);
    Result.ControlOwner = Source.FullStatus.Runtime.ControlOwner;
    Result.bDeadman = Source.FullStatus.Runtime.bDeadman;
    Result.SafeStopState = SafeStopStateName(Source.FullStatus.Runtime.SafeStopState);

    Result.bRequestedAxesAvailable = Source.FullStatus.Motion.RequestedAxes.bAvailable;
    Result.RequestedForward = Source.FullStatus.Motion.RequestedAxes.Forward;
    Result.RequestedLeft = Source.FullStatus.Motion.RequestedAxes.Left;
    Result.RequestedYawLeft = Source.FullStatus.Motion.RequestedAxes.YawLeft;

    Result.bAdmittedTwistAvailable = Source.FullStatus.Motion.AdmittedTwist.bAvailable;
    Result.AdmittedLinearVelocityMps = FVector2D(Source.FullStatus.Motion.AdmittedTwist.LinearX,
                                                 Source.FullStatus.Motion.AdmittedTwist.LinearY);
    Result.AdmittedAngularVelocityRadps = Source.FullStatus.Motion.AdmittedTwist.AngularZ;
    Result.RecordingState = RecordingStateName(Source.FullStatus.Recording.State);
  }

  Result.VisualState = Source.VisualState;
  Result.BodyBindingCount = Source.BodyBindingCount;
  Result.ScenarioActorCount = Source.ScenarioActorCount;
  Result.bLatestAppliedTruthAvailable = Source.bLatestAppliedTruthAvailable;
  Result.TruthSequence = ToBlueprintInt64(Source.TruthSequence);
  Result.SimTimeNanoseconds = Source.SimTimeNs;
  Result.bObservedBaseVelocityAvailable = Source.bObservedBaseVelocityAvailable;
  Result.ObservedBaseStableId = Source.ObservedBaseStableId;
  Result.ObservedBaseLinearVelocityMps = Source.ObservedBaseLinearVelocityMps;
  Result.ObservedBaseAngularVelocityRadps = Source.ObservedBaseAngularVelocityRadps;
  Result.ActualUIMode = Source.ActualUIMode;
  Result.ActualCameraMode = Source.ActualCameraMode;
  return Result;
}

bool ULingTuSimBlueprintStatusSubsystem::ShouldCreateSubsystem(UObject *Outer) const {
  if (!Super::ShouldCreateSubsystem(Outer)) {
    return false;
  }

  const UWorld *World = Cast<UWorld>(Outer);
  return World != nullptr &&
         (World->WorldType == EWorldType::Game || World->WorldType == EWorldType::PIE ||
          World->WorldType == EWorldType::GamePreview);
}

void ULingTuSimBlueprintStatusSubsystem::OnWorldBeginPlay(UWorld &InWorld) {
  Super::OnWorldBeginPlay(InWorld);
  LatestStatus = ReadCurrentStatus();
  bHasLatestStatus = true;
  bInitialBroadcastPending = true;
  InWorld.GetTimerManager().SetTimer(
      StatusRefreshTimer, this, &ULingTuSimBlueprintStatusSubsystem::RefreshStatus,
      BlueprintStatusRefreshIntervalSeconds, true, BlueprintStatusRefreshIntervalSeconds);
}

void ULingTuSimBlueprintStatusSubsystem::Deinitialize() {
  if (UWorld *World = GetWorld()) {
    World->GetTimerManager().ClearTimer(StatusRefreshTimer);
  }
  OnRuntimeStatusChanged.Clear();
  bHasLatestStatus = false;
  bInitialBroadcastPending = false;
  Super::Deinitialize();
}

FLingTuSimBlueprintRuntimeStatus ULingTuSimBlueprintStatusSubsystem::GetLatestStatus() const {
  return bHasLatestStatus ? LatestStatus : ReadCurrentStatus();
}

FLingTuSimBlueprintRuntimeStatus ULingTuSimBlueprintStatusSubsystem::ReadCurrentStatus() const {
  UWorld *World = GetWorld();
  if (World == nullptr) {
    return FLingTuSimBlueprintRuntimeStatus{};
  }

  const ULingTuSimRuntimeUIWorldSubsystem *RuntimeUI =
      World->GetSubsystem<ULingTuSimRuntimeUIWorldSubsystem>();
  const LingTuSim::UI::FRuntimeUIStatusSnapshot Source =
      RuntimeUI != nullptr ? RuntimeUI->ReadStatusSnapshot()
                           : LingTuSim::UI::FRuntimeUIStatusReader::Read(World);
  return LingTuSim::UI::FBlueprintRuntimeStatusProjection::Project(Source);
}

void ULingTuSimBlueprintStatusSubsystem::RefreshStatus() {
  FLingTuSimBlueprintRuntimeStatus NextStatus = ReadCurrentStatus();
  const bool bChanged = !bHasLatestStatus || NextStatus != LatestStatus;
  LatestStatus = MoveTemp(NextStatus);
  bHasLatestStatus = true;

  if (bChanged || bInitialBroadcastPending) {
    bInitialBroadcastPending = false;
    OnRuntimeStatusChanged.Broadcast(LatestStatus);
  }
}

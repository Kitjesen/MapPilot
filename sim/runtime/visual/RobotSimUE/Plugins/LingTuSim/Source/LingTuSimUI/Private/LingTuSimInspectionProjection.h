#pragma once

#include "CoreMinimal.h"
#include "LingTuSimInspectionGuidance.h"

class FJsonObject;

namespace LingTuSim::UI {

struct FInspectionExpectedBinding final {
  FString ProductSessionId;
  // Launcher assertion used to fail closed; this is not automatic proof of the loaded UE world.
  FString MapId;
  int64 MapContentEpoch = 0;
  int64 RouteRevision = 0;
  bool bConfigured = false;
};

struct FInspectionProjection final {
  FString ProductSessionId;
  FInspectionGuidance Guidance;
  bool bCanPause = false;
  bool bCanResume = false;
  bool bCanCancel = false;
  bool bCommandPending = false;
  FString CommandFeedback;
  FString TaskId;
  FString RouteId;
  FString MapId;
  int64 RouteRevision = 0;
  int64 MapContentEpoch = 0;
  FString State;
  FString Phase;
  FString StateSource;
  FString Reason;
  int32 CompletedPoints = 0;
  int32 PointCount = 0;
  int32 CurrentPointNumber = 0;
  FString CurrentPointId;
  FString Action;
  FString ReportStatus;
  FString Acceptance;
  int32 RequiredEvidence = 0;
  int32 VerifiedEvidence = 0;
  bool bBound = false;
  bool bTaskAvailable = false;
  bool bExecutionConfirmed = false;
  bool bHistoryComplete = false;
  bool bProgressVerified = false;
  bool bTerminal = false;
  bool bReportVerified = false;
  bool bReportStale = false;
  bool bStale = false;
  FString TransportError;
  FString IdentityBlocker;
  bool bLauncherBindingVerified = false;

  bool CanRequestControl(const FString& Command) const {
    if (!bBound || !bTaskAvailable || !bProgressVerified || !bLauncherBindingVerified ||
        bStale || bTerminal || bCommandPending || !Guidance.bSimulationRuntime ||
        ProductSessionId.IsEmpty() || Guidance.ProductSessionId != ProductSessionId) return false;
    return (Command == TEXT("pause") && bCanPause) ||
      (Command == TEXT("resume") && bCanResume) || (Command == TEXT("cancel") && bCanCancel);
  }
};

class FInspectionProjectionParser final {
 public:
  static bool ParseTask(const FString &Json, const FString &ExpectedTaskId,
                        const FInspectionExpectedBinding &ExpectedBinding,
                        FInspectionProjection &OutProjection, FString &OutError);
  static bool ParseReport(const FString &Json, const FInspectionProjection &TaskProjection,
                          FInspectionProjection &OutProjection, FString &OutError);

 private:
  static bool ParseObject(const FString &Json, TSharedPtr<FJsonObject> &OutObject,
                          FString &OutError);
};

}  // namespace LingTuSim::UI

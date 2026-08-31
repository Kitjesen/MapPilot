#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "TimerManager.h"

#include "LingTuSimBlueprintStatusSubsystem.generated.h"

namespace LingTuSim::UI {
struct FRuntimeUIStatusSnapshot;
}

/**
 * Read-only runtime status projected for Blueprint presentation logic.
 * Requested input, admitted motion, and observed MuJoCo truth remain distinct.
 */
USTRUCT(BlueprintType)
struct LINGTUSIMUI_API FLingTuSimBlueprintRuntimeStatus {
  GENERATED_BODY()

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status")
  bool bPresentationReady = false;

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Session")
  bool bSessionAvailable = false;

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Session")
  FString SessionState = TEXT("Unavailable");

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Session")
  FString SessionId;

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Session")
  int64 ModelGeneration = 0;

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Control")
  bool bControlBindingAvailable = false;

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Control")
  FString ControlState = TEXT("Unavailable");

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Control")
  FString RunId;

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Control")
  int64 ResetGeneration = 0;

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Control")
  bool bIdentityCoherent = false;

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Control")
  FString Blocker = TEXT("Session binding unavailable");

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Control")
  FString RuntimeState = TEXT("Unavailable");

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Control")
  FString ControlOwner = TEXT("unavailable");

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Control")
  bool bDeadman = false;

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Control")
  FString SafeStopState = TEXT("Unavailable");

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Control")
  bool bLatestControlAckAvailable = false;

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Control")
  bool bLatestControlAckAccepted = false;

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Control")
  FString LatestControlAckState = TEXT("Unavailable");

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Control")
  FString LatestControlAckReason;

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Control")
  bool bFullStatusAvailable = false;

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Control")
  bool bFullStatusFresh = false;

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Control")
  int64 FullStatusSequence = 0;

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Visual")
  FString VisualState = TEXT("Unavailable");

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Visual")
  int32 BodyBindingCount = 0;

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Visual")
  int32 ScenarioActorCount = 0;

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Truth")
  bool bLatestAppliedTruthAvailable = false;

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Truth")
  int64 TruthSequence = 0;

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Truth")
  int64 SimTimeNanoseconds = 0;

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Motion")
  bool bRequestedAxesAvailable = false;

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Motion")
  double RequestedForward = 0.0;

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Motion")
  double RequestedLeft = 0.0;

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Motion")
  double RequestedYawLeft = 0.0;

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Motion")
  bool bAdmittedTwistAvailable = false;

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Motion")
  FVector2D AdmittedLinearVelocityMps = FVector2D::ZeroVector;

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Motion")
  double AdmittedAngularVelocityRadps = 0.0;

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Truth")
  bool bObservedBaseVelocityAvailable = false;

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Truth")
  FString ObservedBaseStableId;

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Truth")
  FVector ObservedBaseLinearVelocityMps = FVector::ZeroVector;

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Truth")
  FVector ObservedBaseAngularVelocityRadps = FVector::ZeroVector;

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|Recording")
  FString RecordingState = TEXT("Unavailable");

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|UI")
  FString ActualUIMode = TEXT("drive");

  UPROPERTY(BlueprintReadOnly, Category = "LingTuSim|Status|UI")
  FString ActualCameraMode = TEXT("unavailable");

  bool operator==(const FLingTuSimBlueprintRuntimeStatus &Other) const;
  bool operator!=(const FLingTuSimBlueprintRuntimeStatus &Other) const { return !(*this == Other); }
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FLingTuSimBlueprintRuntimeStatusChanged,
                                            const FLingTuSimBlueprintRuntimeStatus &, Status);

namespace LingTuSim::UI {
/** Pure adapter from the centralized C++ status snapshot to the Blueprint view. */
class LINGTUSIMUI_API FBlueprintRuntimeStatusProjection final {
 public:
  static FLingTuSimBlueprintRuntimeStatus Project(const FRuntimeUIStatusSnapshot &Source);

 private:
  FBlueprintRuntimeStatusProjection() = delete;
};
}  // namespace LingTuSim::UI

/**
 * Blueprint presentation seam for one game world.
 * It samples the existing centralized status at a bounded rate and broadcasts
 * structured changes; it never owns simulation or control state.
 */
UCLASS(BlueprintType, meta = (DisplayName = "LingTu Sim Blueprint Status"))
class LINGTUSIMUI_API ULingTuSimBlueprintStatusSubsystem final : public UWorldSubsystem {
  GENERATED_BODY()

 public:
  virtual bool ShouldCreateSubsystem(UObject *Outer) const override;
  virtual void OnWorldBeginPlay(UWorld &InWorld) override;
  virtual void Deinitialize() override;

  UPROPERTY(BlueprintAssignable, Category = "LingTuSim|Status")
  FLingTuSimBlueprintRuntimeStatusChanged OnRuntimeStatusChanged;

  UFUNCTION(BlueprintPure, Category = "LingTuSim|Status")
  FLingTuSimBlueprintRuntimeStatus GetLatestStatus() const;

 private:
  FLingTuSimBlueprintRuntimeStatus ReadCurrentStatus() const;
  void RefreshStatus();

  FTimerHandle StatusRefreshTimer;
  FLingTuSimBlueprintRuntimeStatus LatestStatus;
  bool bHasLatestStatus = false;
  bool bInitialBroadcastPending = false;
};

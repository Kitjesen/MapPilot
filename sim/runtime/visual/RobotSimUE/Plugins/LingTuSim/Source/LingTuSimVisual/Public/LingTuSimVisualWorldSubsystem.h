#pragma once

#include "LingTuSimScenarioVisualRegistry.h"
#include "LingTuSimSnapshotMailbox.h"
#include "LingTuSimVisualBoundary.h"
#include "LingTuSimVisualSnapshotGate.h"
#include "Subsystems/WorldSubsystem.h"
#include "LingTuSimVisualWorldSubsystem.generated.h"

class ULingTuSimBodyBindingComponent;
class ALingTuSimBodyActor;
class ALingTuSimWorldEntityActor;
class ACameraActor;
class AActor;

enum class ELingTuSimRuntimeCameraMode : uint8 {
  Unavailable,
  Follow,
  Inspection,
  Free,
};

UCLASS()
class LINGTUSIMVISUAL_API ULingTuSimVisualWorldSubsystem : public UTickableWorldSubsystem,
                                                           public ILingTuSimVisualBoundary {
  GENERATED_BODY()

 public:
  bool RebindSession(const FString &SessionId, uint64 ModelGeneration);
  bool ConfigureReadinessEvidence(const FString &EvidenceDirectory, const FString &RunId,
                                  const FString &SessionId, uint64 ModelGeneration,
                                  uint64 ResetGeneration, FString &OutError);
  virtual LingTuSim::ESnapshotPublishResult
  SubmitSnapshot(const LingTuSim::FSnapshotEnvelope &Snapshot) override;
  virtual bool SubmitScenarioSnapshotJson(const FString &SnapshotJson,
                                          LingTuSim::ESnapshotPublishResult &OutPublishResult,
                                          FString &OutError) override;

  bool RegisterBinding(ULingTuSimBodyBindingComponent *Binding);
  void UnregisterBinding(const FString &StableId, const ULingTuSimBodyBindingComponent *Binding);
  int32 GetRegisteredBindingCount() const { return Bindings.Num(); }
  int32 GetScenarioActorCount() const { return ScenarioActorRegistry.Num(); }
  /** Copies the most recent truth frame only after every required binding accepted it. */
  bool GetLatestAppliedSnapshot(LingTuSim::FSnapshotEnvelope &OutSnapshot) const;
  ALingTuSimScenarioActor *FindScenarioActor(const FString &StableId) const {
    return ScenarioActorRegistry.FindActor(StableId);
  }
  bool IsWaitingForRebind() const;
  static bool AreStaticMeshRenderResourcesReady(const AActor *Actor, FString &OutReason);
  bool SetSessionCameraViewTargetForPlayer0();
  /** Changes only the player's real UE view target and verifies readback. */
  bool SetRuntimeCameraMode(ELingTuSimRuntimeCameraMode RequestedMode, FString &OutError);
  bool CycleRuntimeCameraMode(FString &OutModeName, FString &OutError);
  bool ApplyRuntimeFreeCameraLook(float CameraYaw, float CameraPitch, FString &OutError);
  FString GetRuntimeCameraModeName() const;

  virtual void Tick(float DeltaTime) override;
  virtual TStatId GetStatId() const override;
  virtual void Initialize(FSubsystemCollectionBase &Collection) override;
  virtual void OnWorldBeginPlay(UWorld &InWorld) override;
  virtual void Deinitialize() override;

  bool StartVisualPlanFromCommandLine(FString &OutError);
  virtual bool StartVisualPlan(const FString &BundleDirectory, const FString &ArtifactRoot,
                               uint64 ModelGeneration, uint64 ResetGeneration,
                               FString &OutError) override;

 private:
  void BeginCandidate();
  void RollbackCandidate();
  bool CommitCandidate(const FString &SessionId, uint64 ModelGeneration, uint64 ResetGeneration,
                       int32 ExpectedBodyCount, int32 ExpectedWorldEntityCount, FString &OutError);
  void DestroyActors(TArray<TObjectPtr<ALingTuSimBodyActor>> &Actors);
  void DestroyWorldActors(TArray<TObjectPtr<ALingTuSimWorldEntityActor>> &Actors);
  void RevealActiveActors();
  void CaptureCommandLineVisualPlanRequest();
  void TryStartPendingCommandLineVisualPlan();
  bool CanAutoStartVisualPlan() const;
  void EnterWaitingForRebind(const LingTuSim::FSnapshotEnvelope &FutureSnapshot);
  bool TryApplyCompleteFrame(const LingTuSim::FSnapshotEnvelope &Snapshot);
  bool AreActiveActorRenderResourcesReady();
  void InitializeReadinessEvidenceFromCommandLine();
  void TryWritePreparedReadinessEvidence();
  bool WriteReadinessEvidence(const TCHAR *VisualState, FString &OutError) const;
  void InitializeFirstFrameScreenshotFromCommandLine();
  void RequestFirstFrameScreenshotIfReady(const LingTuSim::FSnapshotEnvelope &Snapshot);
  void InitializeFrameCaptureFromCommandLine();
  void RequestFrameCaptureIfReady(const LingTuSim::FSnapshotEnvelope &Snapshot);
  void InitializeMotionCameraFromCommandLine();
  void ReassertSessionCameraViewTargetAfterActorBeginPlay();
  bool EnsureMotionCameraForAppliedSnapshot(const LingTuSim::FSnapshotEnvelope &Snapshot);
  void DestroyMotionCamera();
  bool UpdateMotionCameraTransform();
  ACameraActor *FindUniqueInspectionCamera(FString &OutError) const;
  ACameraActor *EnsureFreeCamera(FString &OutError);
  APlayerController *FindPlayerController0() const;
  void ApplyLatestScenarioSnapshot();
  void WriteScenarioVisualEvidence();

  TMap<FString, TWeakObjectPtr<ULingTuSimBodyBindingComponent>> Bindings;
  TMap<FString, TWeakObjectPtr<ULingTuSimBodyBindingComponent>> CandidateBindings;
  TArray<TObjectPtr<ALingTuSimBodyActor>> ActiveActors;
  TArray<TObjectPtr<ALingTuSimBodyActor>> CandidateActors;
  TArray<TObjectPtr<ALingTuSimWorldEntityActor>> ActiveWorldActors;
  TArray<TObjectPtr<ALingTuSimWorldEntityActor>> CandidateWorldActors;
  LingTuSim::Visual::FSnapshotGate SnapshotGate;
  LingTuSim::Visual::FScenarioVisualActorRegistry ScenarioActorRegistry;

  mutable FCriticalSection SubmissionCriticalSection;
  FString BoundSessionId;
  uint64 BoundModelGeneration = 0;
  uint64 ReadinessResetGeneration = 0;
  bool bSessionBound = false;
  bool bWaitingForRebind = false;
  bool bCandidateActive = false;
  bool bLoggedFirstAppliedFrame = false;
  bool bReadinessEvidenceConfigured = false;
  bool bVisualReadinessActive = false;
  bool bActiveActorsRevealed = false;
  bool bPendingCommandLineVisualPlan = false;
  bool bAttemptedCommandLineVisualPlan = false;
  bool bLoggedCommandLineVisualPlanDeferral = false;
  bool bFirstFrameScreenshotRequested = false;
  bool bFrameCaptureEnabled = false;
  bool bFrameCaptureDue = false;
  bool bLoggedFrameCapturePending = false;
  bool bLoggedFrameCaptureComplete = false;
  bool bMotionCameraRequested = false;
  bool bMotionCameraReady = false;
  bool bLoggedMotionCameraBindingMissing = false;
  bool bLoggedMotionCameraCaptureBlocked = false;
  bool bLoggedRenderResourcesDeferred = false;
  bool bLoggedRenderResourcesReady = false;
  bool bHasRenderDeferredSnapshot = false;
  bool bHasLatestAppliedSnapshot = false;
  LingTuSim::FSnapshotEnvelope RenderDeferredSnapshot;
  LingTuSim::FSnapshotEnvelope LatestAppliedSnapshot;
  FString PendingBundleDirectory;
  FString PendingArtifactRoot;
  uint64 PendingModelGeneration = 0;
  uint64 PendingResetGeneration = 0;
  FString ReadinessEvidenceDirectory;
  FString ReadinessRunId;
  FString ReadinessSessionId;
  FString FirstFrameScreenshotPath;
  FString FrameCaptureDirectory;
  FString MotionCameraStableId;
  TWeakObjectPtr<ACameraActor> MotionCamera;
  TWeakObjectPtr<ACameraActor> FreeCamera;
  ELingTuSimRuntimeCameraMode RuntimeCameraMode = ELingTuSimRuntimeCameraMode::Unavailable;
  int32 FrameCaptureEvery = 0;
  int32 FrameCaptureMax = 0;
  int32 FrameCaptureRequested = 0;
  uint64 FrameCaptureAppliedSnapshotCount = 0;
};

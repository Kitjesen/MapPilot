#include "LingTuSimVisualWorldSubsystem.h"

#include "Camera/CameraActor.h"
#include "Camera/CameraComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "EngineUtils.h"
#include "GameFramework/PlayerController.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformProcess.h"
#include "LingTuSimBodyActor.h"
#include "LingTuSimBodyBindingComponent.h"
#include "LingTuSimRobotVisualProjection.h"
#include "LingTuSimScenarioActor.h"
#include "LingTuSimSessionService.h"
#include "LingTuSimVisualTransform.h"
#include "LingTuSimWorldEntityActor.h"
#include "Math/NumericLimits.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/ScopeLock.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "TimerManager.h"
#include "UnrealClient.h"

DEFINE_LOG_CATEGORY_STATIC(LogLingTuSimVisual, Log, All);

namespace {
struct FPreparedBindingTransform {
  ULingTuSimBodyBindingComponent *Binding = nullptr;
  FTransform Transform;
};

bool ParseJsonObject(const FString &Json, TSharedPtr<FJsonObject> &OutObject, FString &OutError) {
  TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Json);
  if (!FJsonSerializer::Deserialize(Reader, OutObject) || !OutObject.IsValid()) {
    OutError = TEXT("invalid JSON object");
    return false;
  }
  return true;
}

FString SafeStringField(const TSharedPtr<FJsonObject> &Object, const TCHAR *Key) {
  FString Value;
  if (Object.IsValid()) {
    Object->TryGetStringField(Key, Value);
  }
  return Value;
}

bool HasRelativePathComponent(const FString &Path) {
  FString StandardPath = Path;
  FPaths::NormalizeFilename(StandardPath);
  TArray<FString> Components;
  StandardPath.ParseIntoArray(Components, TEXT("/"), false);
  return Components.Contains(TEXT(".")) || Components.Contains(TEXT(".."));
}

bool TryNormalizeAbsolutePath(const FString &Path, const bool bDirectory, FString &OutPath) {
  OutPath.Reset();
  if (Path.IsEmpty() || FPaths::IsRelative(Path) || HasRelativePathComponent(Path)) {
    return false;
  }
  OutPath = FPaths::ConvertRelativePathToFull(Path);
  if (bDirectory) {
    FPaths::NormalizeDirectoryName(OutPath);
  } else {
    FPaths::NormalizeFilename(OutPath);
  }
  return !OutPath.IsEmpty() && FPaths::CollapseRelativeDirectories(OutPath, true);
}

bool PathTraversesSymlink(const FString &Path) {
  FString Current = Path;
  while (!Current.IsEmpty()) {
    if (IFileManager::Get().IsSymlink(*Current)) {
      return true;
    }
    FString Parent = FPaths::GetPath(Current);
    FPaths::NormalizeDirectoryName(Parent);
    if (Parent.IsEmpty() || FPaths::IsSamePath(Current, Parent)) {
      break;
    }
    Current = MoveTemp(Parent);
  }
  return false;
}

bool ValidateRunAllocationFilePath(const FString &AllocationPath, FString &OutCanonicalPath,
                                   FString &OutError) {
  if (!TryNormalizeAbsolutePath(AllocationPath, false, OutCanonicalPath) ||
      FPaths::GetCleanFilename(OutCanonicalPath) != TEXT("run-allocation.json")) {
    OutError = TEXT("run allocation path must be an absolute canonical run-allocation.json path");
    return false;
  }
  if (!IFileManager::Get().FileExists(*OutCanonicalPath) ||
      PathTraversesSymlink(OutCanonicalPath)) {
    OutError = TEXT(
        "run allocation path must be a plain existing file without symlink or reparse traversal");
    return false;
  }
  return true;
}

bool ValidateRunAllocationEvidencePaths(const FString &AllocationPath, const FString &RunId,
                                        const FString &LogDirectory,
                                        FString &OutCanonicalLogDirectory, FString &OutError) {
  const FString RunDirectory = FPaths::GetPath(AllocationPath);
  if (FPaths::GetCleanFilename(RunDirectory) != RunId) {
    OutError = TEXT("run allocation directory identity must exactly match run_id");
    return false;
  }
  if (!TryNormalizeAbsolutePath(LogDirectory, true, OutCanonicalLogDirectory)) {
    OutError = TEXT("RunAllocation.log_dir must be an absolute canonical directory");
    return false;
  }
  FString ExpectedLogDirectory = FPaths::Combine(RunDirectory, TEXT("logs"));
  FPaths::NormalizeDirectoryName(ExpectedLogDirectory);
  if (!FPaths::IsSamePath(OutCanonicalLogDirectory, ExpectedLogDirectory)) {
    OutError =
        TEXT("RunAllocation.log_dir must exactly equal the allocation run directory logs path");
    return false;
  }
  if (!IFileManager::Get().DirectoryExists(*OutCanonicalLogDirectory) ||
      PathTraversesSymlink(AllocationPath) || PathTraversesSymlink(OutCanonicalLogDirectory)) {
    OutError = TEXT(
        "run allocation evidence path traverses a symlink, reparse point, or missing directory");
    return false;
  }
  return true;
}

bool ReadRequiredCommandLineGeneration(const TCHAR *Key, uint64 &OutGeneration) {
  FString Text;
  const FString DashedKey = FString(TEXT("-")) + Key;
  if ((!FParse::Value(FCommandLine::Get(), Key, Text) &&
       !FParse::Value(FCommandLine::Get(), *DashedKey, Text)) ||
      Text.IsEmpty()) {
    return false;
  }
  uint64 Value = 0;
  for (const TCHAR Character : Text) {
    if (Character < TEXT('0') || Character > TEXT('9')) {
      return false;
    }
    const uint64 Digit = static_cast<uint64>(Character - TEXT('0'));
    if (Value > (MAX_uint64 - Digit) / 10) {
      return false;
    }
    Value = Value * 10 + Digit;
  }
  OutGeneration = Value;
  return true;
}

bool ReadCommandLineGeneration(const TCHAR *Key, const uint64 DefaultValue, uint64 &OutGeneration) {
  FString Text;
  if (!FParse::Value(FCommandLine::Get(), Key, Text) || Text.IsEmpty()) {
    OutGeneration = DefaultValue;
    return true;
  }
  if (!Text.IsNumeric()) {
    return false;
  }
  OutGeneration = FCString::Strtoui64(*Text, nullptr, 10);
  return true;
}

bool ReplaceReadinessFileWithRetry(const FString &TargetPath, const FString &TempPath) {
#if PLATFORM_WINDOWS
  constexpr int32 MaxAttempts = 5;
#else
  constexpr int32 MaxAttempts = 1;
#endif
  for (int32 Attempt = 0; Attempt < MaxAttempts; ++Attempt) {
    if (IFileManager::Get().Move(*TargetPath, *TempPath, true, true, false, true)) {
      return true;
    }
#if PLATFORM_WINDOWS
    if (Attempt + 1 < MaxAttempts) {
      FPlatformProcess::SleepNoStats(0.005F * static_cast<float>(Attempt + 1));
    }
#endif
  }
  return false;
}

const TCHAR *WorldTypeToText(const EWorldType::Type WorldType) {
  switch (WorldType) {
    case EWorldType::Game:
      return TEXT("Game");
    case EWorldType::PIE:
      return TEXT("PIE");
    case EWorldType::GamePreview:
      return TEXT("GamePreview");
    case EWorldType::Editor:
      return TEXT("Editor");
    case EWorldType::EditorPreview:
      return TEXT("EditorPreview");
    case EWorldType::Inactive:
      return TEXT("Inactive");
    case EWorldType::None:
      return TEXT("None");
    default:
      return TEXT("Unknown");
  }
}
}  // namespace

bool ULingTuSimVisualWorldSubsystem::RebindSession(const FString &SessionId,
                                                   const uint64 ModelGeneration) {
  if (!ensure(IsInGameThread()) || SessionId.IsEmpty()) {
    return false;
  }

  UWorld *World = GetWorld();
  if (World == nullptr) {
    return false;
  }
  FScopeLock Lock(&SubmissionCriticalSection);
  if (!LingTuSim::FSessionService::RebindSession(SessionId, ModelGeneration)) {
    return false;
  }
  DestroyMotionCamera();
  if (!ScenarioActorRegistry.BindSession(*World, SessionId, ModelGeneration)) {
    return false;
  }

  BoundSessionId = SessionId;
  BoundModelGeneration = ModelGeneration;
  bSessionBound = true;
  bWaitingForRebind = false;
  bLoggedFirstAppliedFrame = false;
  bVisualReadinessActive = false;
  bLoggedRenderResourcesDeferred = false;
  bLoggedRenderResourcesReady = false;
  bHasRenderDeferredSnapshot = false;
  bHasLatestAppliedSnapshot = false;
  RenderDeferredSnapshot = LingTuSim::FSnapshotEnvelope{};
  LatestAppliedSnapshot = LingTuSim::FSnapshotEnvelope{};
  ReadinessSessionId = SessionId;
  TryWritePreparedReadinessEvidence();
  SnapshotGate.BindSession(SessionId, ModelGeneration);
  return true;
}

bool ULingTuSimVisualWorldSubsystem::ConfigureReadinessEvidence(
    const FString &EvidenceDirectory, const FString &RunId, const FString &SessionId,
    const uint64 ModelGeneration, const uint64 ResetGeneration, FString &OutError) {
  OutError.Reset();
  if (EvidenceDirectory.IsEmpty()) {
    OutError = TEXT("RunAllocation.log_dir is required for visual readiness evidence");
    return false;
  }
  if (SessionId.IsEmpty()) {
    OutError = TEXT("RunAllocation.session_id is required for visual readiness evidence");
    return false;
  }
  if (!ScenarioActorRegistry.BindRunId(RunId)) {
    OutError = TEXT("RunAllocation.run_id must be a non-empty canonical run ID");
    return false;
  }
  ReadinessEvidenceDirectory = EvidenceDirectory;
  ReadinessRunId = RunId;
  ReadinessSessionId = SessionId;
  BoundModelGeneration = ModelGeneration;
  ReadinessResetGeneration = ResetGeneration;
  bReadinessEvidenceConfigured = true;
  bVisualReadinessActive = false;
  TryWritePreparedReadinessEvidence();
  return true;
}

LingTuSim::ESnapshotPublishResult
ULingTuSimVisualWorldSubsystem::SubmitSnapshot(const LingTuSim::FSnapshotEnvelope &Snapshot) {
  FScopeLock Lock(&SubmissionCriticalSection);
  if (!bSessionBound) {
    return LingTuSim::ESnapshotPublishResult::SessionMismatch;
  }
  if (bWaitingForRebind) {
    return LingTuSim::ESnapshotPublishResult::ModelMismatch;
  }
  if (bHasRenderDeferredSnapshot &&
      Snapshot.SessionId == RenderDeferredSnapshot.SessionId &&
      Snapshot.ModelGeneration == RenderDeferredSnapshot.ModelGeneration &&
      Snapshot.ResetGeneration == RenderDeferredSnapshot.ResetGeneration &&
      Snapshot.Sequence == RenderDeferredSnapshot.Sequence) {
    RenderDeferredSnapshot = Snapshot;
    return LingTuSim::ESnapshotPublishResult::Accepted;
  }
  return LingTuSim::FSessionService::PublishSnapshot(Snapshot);
}

bool ULingTuSimVisualWorldSubsystem::SubmitScenarioSnapshotJson(
    const FString &SnapshotJson, LingTuSim::ESnapshotPublishResult &OutPublishResult,
    FString &OutError) {
  return LingTuSim::FSessionService::PublishScenarioSnapshotJson(SnapshotJson, OutPublishResult,
                                                                 OutError);
}

bool ULingTuSimVisualWorldSubsystem::RegisterBinding(ULingTuSimBodyBindingComponent *Binding) {
  if (!ensure(IsInGameThread()) || Binding == nullptr || Binding->StableId.IsEmpty()) {
    return false;
  }

  TMap<FString, TWeakObjectPtr<ULingTuSimBodyBindingComponent>> &TargetBindings =
      bCandidateActive ? CandidateBindings : Bindings;

  if (TWeakObjectPtr<ULingTuSimBodyBindingComponent> *Existing =
          TargetBindings.Find(Binding->StableId)) {
    if (Existing->Get() == Binding) {
      return true;
    }
    if (Existing->IsValid()) {
      return false;
    }
    *Existing = Binding;
    if (!bCandidateActive) {
      TryWritePreparedReadinessEvidence();
    }
    return true;
  }

  TargetBindings.Add(Binding->StableId, Binding);
  if (!bCandidateActive) {
    TryWritePreparedReadinessEvidence();
  }
  return true;
}

void ULingTuSimVisualWorldSubsystem::UnregisterBinding(
    const FString &StableId, const ULingTuSimBodyBindingComponent *Binding) {
  if (!ensure(IsInGameThread())) {
    return;
  }

  TMap<FString, TWeakObjectPtr<ULingTuSimBodyBindingComponent>> &TargetBindings =
      bCandidateActive ? CandidateBindings : Bindings;
  if (const TWeakObjectPtr<ULingTuSimBodyBindingComponent> *Existing =
          TargetBindings.Find(StableId)) {
    if (Existing->Get() == Binding) {
      TargetBindings.Remove(StableId);
    }
  }
}

bool ULingTuSimVisualWorldSubsystem::IsWaitingForRebind() const {
  FScopeLock Lock(&SubmissionCriticalSection);
  return bWaitingForRebind;
}

bool ULingTuSimVisualWorldSubsystem::GetLatestAppliedSnapshot(
    LingTuSim::FSnapshotEnvelope &OutSnapshot) const {
  FScopeLock Lock(&SubmissionCriticalSection);
  if (!bHasLatestAppliedSnapshot) {
    return false;
  }
  OutSnapshot = LatestAppliedSnapshot;
  return true;
}

void ULingTuSimVisualWorldSubsystem::Tick(const float DeltaTime) {
  (void)DeltaTime;
  check(IsInGameThread());
  TryStartPendingCommandLineVisualPlan();
  ApplyLatestScenarioSnapshot();

  LingTuSim::FSnapshotEnvelope FutureSnapshot;
  LingTuSim::FSnapshotEnvelope Snapshot;
  {
    FScopeLock Lock(&SubmissionCriticalSection);
    if (bWaitingForRebind) {
      return;
    }
    if (bHasRenderDeferredSnapshot) {
      Snapshot = RenderDeferredSnapshot;
    } else if (LingTuSim::FSessionService::TryTakeLatestFutureSnapshot(FutureSnapshot)) {
      const LingTuSim::Visual::ESnapshotGateResult FutureResult =
          SnapshotGate.Evaluate(FutureSnapshot);
      if (FutureResult == LingTuSim::Visual::ESnapshotGateResult::FutureModel) {
        EnterWaitingForRebind(FutureSnapshot);
        return;
      }
    }
    if (!bHasRenderDeferredSnapshot &&
        !LingTuSim::FSessionService::TryTakeLatestSnapshot(Snapshot)) {
      return;
    }
  }

  const LingTuSim::Visual::ESnapshotGateResult GateResult = SnapshotGate.Evaluate(Snapshot);
  if (GateResult == LingTuSim::Visual::ESnapshotGateResult::FutureModel) {
    FScopeLock Lock(&SubmissionCriticalSection);
    EnterWaitingForRebind(Snapshot);
    return;
  }
  if (GateResult != LingTuSim::Visual::ESnapshotGateResult::Accept) {
    return;
  }

  if (TryApplyCompleteFrame(Snapshot)) {
    FScopeLock Lock(&SubmissionCriticalSection);
    bHasRenderDeferredSnapshot = false;
    RenderDeferredSnapshot = LingTuSim::FSnapshotEnvelope{};
  }
}

TStatId ULingTuSimVisualWorldSubsystem::GetStatId() const {
  RETURN_QUICK_DECLARE_CYCLE_STAT(ULingTuSimVisualWorldSubsystem, STATGROUP_Tickables);
}

void ULingTuSimVisualWorldSubsystem::Initialize(FSubsystemCollectionBase &Collection) {
  Super::Initialize(Collection);
  InitializeReadinessEvidenceFromCommandLine();
  InitializeFirstFrameScreenshotFromCommandLine();
  InitializeFrameCaptureFromCommandLine();
  InitializeMotionCameraFromCommandLine();
  FString SessionId;
  uint64 ModelGeneration = 0;
  if (LingTuSim::FSessionService::GetBoundSession(SessionId, ModelGeneration)) {
    RebindSession(SessionId, ModelGeneration);
  }
  CaptureCommandLineVisualPlanRequest();
}

void ULingTuSimVisualWorldSubsystem::OnWorldBeginPlay(UWorld &InWorld) {
  Super::OnWorldBeginPlay(InWorld);
  SetSessionCameraViewTargetForPlayer0();
  InWorld.GetTimerManager().SetTimerForNextTick(
      this, &ULingTuSimVisualWorldSubsystem::ReassertSessionCameraViewTargetAfterActorBeginPlay);
  TryStartPendingCommandLineVisualPlan();
}

void ULingTuSimVisualWorldSubsystem::ReassertSessionCameraViewTargetAfterActorBeginPlay() {
  SetSessionCameraViewTargetForPlayer0();
}

void ULingTuSimVisualWorldSubsystem::Deinitialize() {
  check(IsInGameThread());
  RollbackCandidate();
  DestroyMotionCamera();
  if (UWorld *World = GetWorld()) {
    ScenarioActorRegistry.Clear(*World);
  }
  DestroyActors(ActiveActors);
  DestroyWorldActors(ActiveWorldActors);
  Bindings.Reset();
  CandidateBindings.Reset();
  SnapshotGate.Clear();
  {
    FScopeLock Lock(&SubmissionCriticalSection);
    BoundSessionId.Reset();
    BoundModelGeneration = 0;
    ReadinessResetGeneration = 0;
    bSessionBound = false;
    bWaitingForRebind = false;
    bCandidateActive = false;
    bLoggedFirstAppliedFrame = false;
    bReadinessEvidenceConfigured = false;
    bVisualReadinessActive = false;
    bActiveActorsRevealed = false;
    bPendingCommandLineVisualPlan = false;
    bAttemptedCommandLineVisualPlan = false;
    bLoggedCommandLineVisualPlanDeferral = false;
    bFirstFrameScreenshotRequested = false;
    bFrameCaptureEnabled = false;
    bFrameCaptureDue = false;
    bLoggedFrameCapturePending = false;
    bLoggedFrameCaptureComplete = false;
    bMotionCameraRequested = false;
    bMotionCameraReady = false;
    bLoggedMotionCameraBindingMissing = false;
    bLoggedMotionCameraCaptureBlocked = false;
    bLoggedRenderResourcesDeferred = false;
    bLoggedRenderResourcesReady = false;
    bHasRenderDeferredSnapshot = false;
    bHasLatestAppliedSnapshot = false;
    RenderDeferredSnapshot = LingTuSim::FSnapshotEnvelope{};
    LatestAppliedSnapshot = LingTuSim::FSnapshotEnvelope{};
    PendingBundleDirectory.Reset();
    PendingArtifactRoot.Reset();
    PendingModelGeneration = 0;
    PendingResetGeneration = 0;
    ReadinessEvidenceDirectory.Reset();
    ReadinessRunId.Reset();
    ReadinessSessionId.Reset();
    FirstFrameScreenshotPath.Reset();
    FrameCaptureDirectory.Reset();
    MotionCameraStableId.Reset();
    FrameCaptureEvery = 0;
    FrameCaptureMax = 0;
    FrameCaptureRequested = 0;
    FrameCaptureAppliedSnapshotCount = 0;
  }
  Super::Deinitialize();
}

void ULingTuSimVisualWorldSubsystem::ApplyLatestScenarioSnapshot() {
  check(IsInGameThread());
  UWorld *World = GetWorld();
  if (World == nullptr) {
    return;
  }

  FString SnapshotJson;
  if (!LingTuSim::FSessionService::TryTakeLatestScenarioSnapshotJson(SnapshotJson)) {
    return;
  }
  FString ApplyError;
  const LingTuSim::Visual::EScenarioVisualApplyResult Result =
      ScenarioActorRegistry.ApplySnapshotJson(*World, SnapshotJson, ApplyError);
  if (Result != LingTuSim::Visual::EScenarioVisualApplyResult::Accepted) {
    UE_LOG(LogLingTuSimVisual, Warning, TEXT("LINGTU_SCENARIO_VISUAL_REJECTED result=%d reason=%s"),
           static_cast<int32>(Result),
           ApplyError.IsEmpty() ? TEXT("generation_or_identity_gate") : *ApplyError);
    return;
  }
  WriteScenarioVisualEvidence();
}

void ULingTuSimVisualWorldSubsystem::WriteScenarioVisualEvidence() {
  const FString &Json = ScenarioActorRegistry.GetLastEvidenceJson();
  if (ReadinessEvidenceDirectory.IsEmpty() || Json.IsEmpty()) {
    return;
  }
  IFileManager::Get().MakeDirectory(*ReadinessEvidenceDirectory, true);
  const FString EvidencePath =
      FPaths::Combine(ReadinessEvidenceDirectory, TEXT("scenario-visual-evidence.json"));
  const FString TempPath =
      FString::Printf(TEXT("%s.tmp.%u"), *EvidencePath, FPlatformProcess::GetCurrentProcessId());
  if (!FFileHelper::SaveStringToFile(Json, *TempPath,
                                     FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM) ||
      !ReplaceReadinessFileWithRetry(EvidencePath, TempPath)) {
    IFileManager::Get().Delete(*TempPath, false, true, true);
    UE_LOG(LogLingTuSimVisual, Error, TEXT("LINGTU_SCENARIO_VISUAL_EVIDENCE_WRITE_FAILED path=%s"),
           *EvidencePath);
  }
}

bool ULingTuSimVisualWorldSubsystem::StartVisualPlanFromCommandLine(FString &OutError) {
  OutError.Reset();
  FString BundleDirectory;
  FString ArtifactRoot;
  if (!FParse::Value(FCommandLine::Get(), TEXT("LingTuBundle="), BundleDirectory) &&
      !FParse::Value(FCommandLine::Get(), TEXT("-LingTuBundle="), BundleDirectory)) {
    return true;
  }
  if (!FParse::Value(FCommandLine::Get(), TEXT("LingTuArtifactRoot="), ArtifactRoot) &&
      !FParse::Value(FCommandLine::Get(), TEXT("-LingTuArtifactRoot="), ArtifactRoot)) {
    OutError = TEXT("LingTuArtifactRoot is required when LingTuBundle is set");
    return false;
  }

  uint64 ModelGeneration = 0;
  uint64 ResetGeneration = 0;
  if (!ReadCommandLineGeneration(TEXT("LingTuModelGeneration="), 0, ModelGeneration) ||
      !ReadCommandLineGeneration(TEXT("LingTuResetGeneration="), 0, ResetGeneration)) {
    OutError =
        TEXT("LingTuModelGeneration and LingTuResetGeneration must be non-negative integers");
    return false;
  }
  return StartVisualPlan(BundleDirectory, ArtifactRoot, ModelGeneration, ResetGeneration, OutError);
}

void ULingTuSimVisualWorldSubsystem::CaptureCommandLineVisualPlanRequest() {
  PendingBundleDirectory.Reset();
  PendingArtifactRoot.Reset();
  PendingModelGeneration = 0;
  PendingResetGeneration = 0;
  bPendingCommandLineVisualPlan = false;
  bAttemptedCommandLineVisualPlan = false;
  bLoggedCommandLineVisualPlanDeferral = false;

  if (!FParse::Value(FCommandLine::Get(), TEXT("LingTuBundle="), PendingBundleDirectory) &&
      !FParse::Value(FCommandLine::Get(), TEXT("-LingTuBundle="), PendingBundleDirectory)) {
    return;
  }
  bPendingCommandLineVisualPlan = true;
  if (!FParse::Value(FCommandLine::Get(), TEXT("LingTuArtifactRoot="), PendingArtifactRoot) &&
      !FParse::Value(FCommandLine::Get(), TEXT("-LingTuArtifactRoot="), PendingArtifactRoot)) {
    UE_LOG(LogLingTuSimVisual, Error,
           TEXT("LINGTU_VISUAL_PLAN_REQUEST_INVALID reason=LingTuArtifactRoot is required"));
    bAttemptedCommandLineVisualPlan = true;
    return;
  }
  if (!ReadCommandLineGeneration(TEXT("LingTuModelGeneration="), 0, PendingModelGeneration) ||
      !ReadCommandLineGeneration(TEXT("LingTuResetGeneration="), 0, PendingResetGeneration)) {
    UE_LOG(LogLingTuSimVisual, Error,
           TEXT("LINGTU_VISUAL_PLAN_REQUEST_INVALID reason=invalid generation"));
    bAttemptedCommandLineVisualPlan = true;
  }
}

bool ULingTuSimVisualWorldSubsystem::CanAutoStartVisualPlan() const {
  const UWorld *World = GetWorld();
  if (World == nullptr || !World->HasBegunPlay()) {
    return false;
  }
  return World->WorldType == EWorldType::Game || World->WorldType == EWorldType::PIE ||
         World->WorldType == EWorldType::GamePreview;
}

void ULingTuSimVisualWorldSubsystem::TryStartPendingCommandLineVisualPlan() {
  const UWorld *World = GetWorld();
  const bool bWorldIsNull = World == nullptr;
  const bool bWorldHasBegunPlay = World != nullptr && World->HasBegunPlay();
  const bool bCanAutoStartVisualPlan = CanAutoStartVisualPlan();
  if (!bPendingCommandLineVisualPlan || bAttemptedCommandLineVisualPlan ||
      !bCanAutoStartVisualPlan) {
    const FString CommandLine(FCommandLine::Get());
    if (!bLoggedCommandLineVisualPlanDeferral &&
        (bPendingCommandLineVisualPlan || bAttemptedCommandLineVisualPlan ||
         CommandLine.Contains(TEXT("LingTuBundle")))) {
      bLoggedCommandLineVisualPlanDeferral = true;
      UE_LOG(LogLingTuSimVisual, Warning,
             TEXT("LINGTU_VISUAL_PLAN_DEFERRED pending=%d attempted=%d world_null=%d "
                  "has_begun_play=%d world_type=%s can_auto_start=%d"),
             bPendingCommandLineVisualPlan ? 1 : 0, bAttemptedCommandLineVisualPlan ? 1 : 0,
             bWorldIsNull ? 1 : 0, bWorldHasBegunPlay ? 1 : 0,
             World != nullptr ? WorldTypeToText(World->WorldType) : TEXT("None"),
             bCanAutoStartVisualPlan ? 1 : 0);
    }
    return;
  }
  bAttemptedCommandLineVisualPlan = true;
  FString VisualStartError;
  if (!StartVisualPlan(PendingBundleDirectory, PendingArtifactRoot, PendingModelGeneration,
                       PendingResetGeneration, VisualStartError) &&
      !VisualStartError.IsEmpty()) {
    UE_LOG(LogLingTuSimVisual, Error, TEXT("LINGTU_VISUAL_PLAN_START_FAILED reason=%s"),
           *VisualStartError);
  }
}

bool ULingTuSimVisualWorldSubsystem::StartVisualPlan(const FString &BundleDirectory,
                                                     const FString &ArtifactRoot,
                                                     const uint64 ModelGeneration,
                                                     const uint64 ResetGeneration,
                                                     FString &OutError) {
  OutError.Reset();
  if (!ensure(IsInGameThread())) {
    OutError = TEXT("visual plan can only start on the game thread");
    return false;
  }
  if (GetWorld() == nullptr) {
    OutError = TEXT("visual plan requires an initialized world");
    return false;
  }
  if (bSessionBound) {
    if (ModelGeneration < BoundModelGeneration) {
      OutError = TEXT("stale model generation rejected");
      return false;
    }
  }

  BeginCandidate();
  LingTuSim::Visual::FVisualMaterializationResult Result;
  LingTuSim::Visual::FVisualMaterializationError Error;
  if (!LingTuSim::Visual::FRobotVisualProjectionMaterializer::MaterializeBundle(
          *GetWorld(), BundleDirectory, ArtifactRoot, ModelGeneration, ResetGeneration, Result,
          Error)) {
    RollbackCandidate();
    OutError = Error.Message.IsEmpty()
                   ? TEXT("visual materialization failed")
                   : FString::Printf(TEXT("%s: %s"), *Error.Source, *Error.Message);
    return false;
  }
  CandidateActors = MoveTemp(Result.Actors);
  CandidateWorldActors = MoveTemp(Result.WorldActors);

  if (bSessionBound && ModelGeneration == BoundModelGeneration &&
      Result.SessionId == BoundSessionId && ActiveActors.Num() > 0) {
    RollbackCandidate();
    return true;
  }
  if (bSessionBound && ModelGeneration == BoundModelGeneration &&
      Result.SessionId != BoundSessionId) {
    RollbackCandidate();
    OutError = TEXT("same model generation cannot switch to a different session");
    return false;
  }

  return CommitCandidate(Result.SessionId, Result.ModelGeneration, Result.ResetGeneration,
                         Result.ExpectedBodyCount, Result.ExpectedWorldEntityCount, OutError);
}

void ULingTuSimVisualWorldSubsystem::BeginCandidate() {
  check(IsInGameThread());
  RollbackCandidate();
  CandidateBindings.Reset();
  CandidateActors.Reset();
  CandidateWorldActors.Reset();
  bCandidateActive = true;
}

void ULingTuSimVisualWorldSubsystem::RollbackCandidate() {
  check(IsInGameThread());
  bCandidateActive = false;
  CandidateBindings.Reset();
  DestroyActors(CandidateActors);
  DestroyWorldActors(CandidateWorldActors);
}

bool ULingTuSimVisualWorldSubsystem::CommitCandidate(
    const FString &SessionId, const uint64 ModelGeneration, const uint64 ResetGeneration,
    const int32 ExpectedBodyCount, const int32 ExpectedWorldEntityCount, FString &OutError) {
  check(IsInGameThread());
  if (!bCandidateActive) {
    OutError = TEXT("no candidate visual transaction is active");
    return false;
  }
  if (CandidateBindings.Num() != ExpectedBodyCount || CandidateActors.Num() != ExpectedBodyCount) {
    RollbackCandidate();
    OutError = FString::Printf(TEXT("candidate visual transaction incomplete: expected %d bodies, "
                                    "got %d bindings and %d actors"),
                               ExpectedBodyCount, CandidateBindings.Num(), CandidateActors.Num());
    return false;
  }
  if (CandidateWorldActors.Num() != ExpectedWorldEntityCount) {
    RollbackCandidate();
    OutError = FString::Printf(
        TEXT("candidate visual transaction incomplete: expected %d world entities, got %d actors"),
        ExpectedWorldEntityCount, CandidateWorldActors.Num());
    return false;
  }

  TArray<TObjectPtr<ALingTuSimBodyActor>> OldActors = MoveTemp(ActiveActors);
  TArray<TObjectPtr<ALingTuSimWorldEntityActor>> OldWorldActors = MoveTemp(ActiveWorldActors);
  TMap<FString, TWeakObjectPtr<ULingTuSimBodyBindingComponent>> OldBindings = MoveTemp(Bindings);
  const uint64 OldReadinessResetGeneration = ReadinessResetGeneration;
  const bool bOldActiveActorsRevealed = bActiveActorsRevealed;

  Bindings = MoveTemp(CandidateBindings);
  ActiveActors = MoveTemp(CandidateActors);
  ActiveWorldActors = MoveTemp(CandidateWorldActors);
  bCandidateActive = false;
  bActiveActorsRevealed = false;
  bLoggedRenderResourcesDeferred = false;
  bLoggedRenderResourcesReady = false;
  bHasRenderDeferredSnapshot = false;
  RenderDeferredSnapshot = LingTuSim::FSnapshotEnvelope{};
  ReadinessResetGeneration = ResetGeneration;
  if (!RebindSession(SessionId, ModelGeneration)) {
    DestroyActors(ActiveActors);
    DestroyWorldActors(ActiveWorldActors);
    ActiveActors = MoveTemp(OldActors);
    ActiveWorldActors = MoveTemp(OldWorldActors);
    Bindings = MoveTemp(OldBindings);
    ReadinessResetGeneration = OldReadinessResetGeneration;
    bActiveActorsRevealed = bOldActiveActorsRevealed;
    OutError = TEXT("session rebind failed after candidate visual materialization");
    return false;
  }
  TryWritePreparedReadinessEvidence();
  DestroyActors(OldActors);
  DestroyWorldActors(OldWorldActors);
  return true;
}

void ULingTuSimVisualWorldSubsystem::DestroyActors(
    TArray<TObjectPtr<ALingTuSimBodyActor>> &Actors) {
  for (ALingTuSimBodyActor *Actor : Actors) {
    if (IsValid(Actor)) {
      Actor->Destroy();
    }
  }
  Actors.Reset();
}

void ULingTuSimVisualWorldSubsystem::DestroyWorldActors(
    TArray<TObjectPtr<ALingTuSimWorldEntityActor>> &Actors) {
  for (ALingTuSimWorldEntityActor *Actor : Actors) {
    if (IsValid(Actor)) {
      Actor->Destroy();
    }
  }
  Actors.Reset();
}

void ULingTuSimVisualWorldSubsystem::RevealActiveActors() {
  if (bActiveActorsRevealed) {
    return;
  }
  for (ALingTuSimBodyActor *Actor : ActiveActors) {
    if (IsValid(Actor)) {
      Actor->SetActorHiddenInGame(false);
#if WITH_EDITOR
      Actor->SetIsTemporarilyHiddenInEditor(false);
#endif
    }
  }
  for (ALingTuSimWorldEntityActor *Actor : ActiveWorldActors) {
    if (IsValid(Actor)) {
      Actor->SetActorHiddenInGame(false);
#if WITH_EDITOR
      Actor->SetIsTemporarilyHiddenInEditor(false);
#endif
    }
  }
  bActiveActorsRevealed = true;
}

bool ULingTuSimVisualWorldSubsystem::AreStaticMeshRenderResourcesReady(const AActor *Actor,
                                                                       FString &OutReason) {
  OutReason.Reset();
  if (!IsValid(Actor)) {
    OutReason = TEXT("invalid_actor");
    return false;
  }

  TArray<UStaticMeshComponent *> StaticMeshComponents;
  Actor->GetComponents<UStaticMeshComponent>(StaticMeshComponents);
  for (const UStaticMeshComponent *Component : StaticMeshComponents) {
    if (!IsValid(Component)) {
      OutReason = TEXT("invalid_static_mesh_component");
      return false;
    }
    if (!Component->IsRegistered()) {
      OutReason = FString::Printf(TEXT("component_not_registered actor=%s component=%s"),
                                  *Actor->GetName(), *Component->GetName());
      return false;
    }
    if (!Component->IsRenderStateCreated()) {
      OutReason = FString::Printf(TEXT("render_state_not_created actor=%s component=%s"),
                                  *Actor->GetName(), *Component->GetName());
      return false;
    }

    UStaticMesh *Mesh = Component->GetStaticMesh();
    if (Mesh == nullptr) {
      OutReason = FString::Printf(TEXT("missing_static_mesh actor=%s component=%s"),
                                  *Actor->GetName(), *Component->GetName());
      return false;
    }
#if WITH_EDITOR
    if (Mesh->IsCompiling()) {
      OutReason = FString::Printf(TEXT("static_mesh_compiling actor=%s component=%s mesh=%s"),
                                  *Actor->GetName(), *Component->GetName(), *Mesh->GetName());
      return false;
    }
#endif
    if (!Mesh->HasValidRenderData()) {
      OutReason = FString::Printf(TEXT("invalid_render_data actor=%s component=%s mesh=%s"),
                                  *Actor->GetName(), *Component->GetName(), *Mesh->GetName());
      return false;
    }
  }
  return true;
}

bool ULingTuSimVisualWorldSubsystem::AreActiveActorRenderResourcesReady() {
  for (const ALingTuSimBodyActor *Actor : ActiveActors) {
    FString Reason;
    if (!AreStaticMeshRenderResourcesReady(Actor, Reason)) {
      if (!bLoggedRenderResourcesDeferred) {
        bLoggedRenderResourcesDeferred = true;
        UE_LOG(
            LogLingTuSimVisual, Warning,
            TEXT("LINGTU_VISUAL_RENDER_RESOURCES_DEFERRED reason=%s active_actors=%d bindings=%d"),
            *Reason, ActiveActors.Num(), Bindings.Num());
      }
      return false;
    }
  }
  for (const ALingTuSimWorldEntityActor *Actor : ActiveWorldActors) {
    FString Reason;
    if (!AreStaticMeshRenderResourcesReady(Actor, Reason)) {
      if (!bLoggedRenderResourcesDeferred) {
        bLoggedRenderResourcesDeferred = true;
        UE_LOG(LogLingTuSimVisual, Warning,
               TEXT("LINGTU_VISUAL_RENDER_RESOURCES_DEFERRED reason=%s active_actors=%d "
                    "world_actors=%d bindings=%d"),
               *Reason, ActiveActors.Num(), ActiveWorldActors.Num(), Bindings.Num());
      }
      return false;
    }
  }

  if (!bLoggedRenderResourcesReady) {
    bLoggedRenderResourcesReady = true;
    UE_LOG(
        LogLingTuSimVisual, Display,
        TEXT("LINGTU_VISUAL_RENDER_RESOURCES_READY active_actors=%d world_actors=%d bindings=%d"),
        ActiveActors.Num(), ActiveWorldActors.Num(), Bindings.Num());
  }
  return true;
}

void ULingTuSimVisualWorldSubsystem::InitializeMotionCameraFromCommandLine() {
  MotionCameraStableId.Reset();
  bMotionCameraRequested = false;
  bMotionCameraReady = false;
  bLoggedMotionCameraBindingMissing = false;
  bLoggedMotionCameraCaptureBlocked = false;

  FString StableId;
  const bool bHasStableId =
      FParse::Value(FCommandLine::Get(), TEXT("LingTuMotionCameraStableId="), StableId) ||
      FParse::Value(FCommandLine::Get(), TEXT("-LingTuMotionCameraStableId="), StableId);
  if (!bHasStableId) {
    return;
  }

  bMotionCameraRequested = true;
  StableId.TrimStartAndEndInline();
  if (StableId.IsEmpty()) {
    UE_LOG(LogLingTuSimVisual, Error,
           TEXT("LINGTU_VISUAL_MOTION_CAMERA_INVALID reason=empty_stable_id"));
    return;
  }

  MotionCameraStableId = MoveTemp(StableId);
  UE_LOG(LogLingTuSimVisual, Display,
         TEXT("LINGTU_VISUAL_MOTION_CAMERA_CONFIGURED stable_id=%s offset_cm=(-300,300,180) "
              "look_at_cm=(100,0,20) fov_deg=55"),
         *MotionCameraStableId);
}

bool ULingTuSimVisualWorldSubsystem::EnsureMotionCameraForAppliedSnapshot(
    const LingTuSim::FSnapshotEnvelope &Snapshot) {
  if (!bMotionCameraRequested) {
    return true;
  }
  if (MotionCameraStableId.IsEmpty()) {
    return false;
  }
  if (MotionCamera.IsValid()) {
    UpdateMotionCameraTransform();
    if (!bMotionCameraReady) {
      FString CameraError;
      bMotionCameraReady = SetRuntimeCameraMode(ELingTuSimRuntimeCameraMode::Follow, CameraError);
    }
    return bMotionCameraReady;
  }

  const TWeakObjectPtr<ULingTuSimBodyBindingComponent> *BindingPtr =
      Bindings.Find(MotionCameraStableId);
  ULingTuSimBodyBindingComponent *Binding = BindingPtr != nullptr ? BindingPtr->Get() : nullptr;
  if (!IsValid(Binding)) {
    if (!bLoggedMotionCameraBindingMissing) {
      bLoggedMotionCameraBindingMissing = true;
      UE_LOG(LogLingTuSimVisual, Error,
             TEXT("LINGTU_VISUAL_MOTION_CAMERA_BLOCKED reason=exact_binding_not_found stable_id=%s "
                  "bindings=%d model_generation=%llu reset_generation=%llu sequence=%llu"),
             *MotionCameraStableId, Bindings.Num(),
             static_cast<unsigned long long>(Snapshot.ModelGeneration),
             static_cast<unsigned long long>(Snapshot.ResetGeneration),
             static_cast<unsigned long long>(Snapshot.Sequence));
    }
    return false;
  }
  bLoggedMotionCameraBindingMissing = false;

  UWorld *World = GetWorld();
  if (World == nullptr) {
    return false;
  }

  const FTransform RootTransform = Binding->GetComponentTransform();
  const FVector CameraLocation =
      RootTransform.GetLocation() +
      RootTransform.GetRotation().RotateVector(FVector(-300.0, 300.0, 180.0));
  const FVector LookAtLocation =
      RootTransform.GetLocation() +
      RootTransform.GetRotation().RotateVector(FVector(100.0, 0.0, 20.0));
  const FRotator CameraRotation = (LookAtLocation - CameraLocation).Rotation();

  FActorSpawnParameters SpawnParameters;
  SpawnParameters.Name =
      MakeUniqueObjectName(World, ACameraActor::StaticClass(), FName(TEXT("LingTuMotionCamera")));
  SpawnParameters.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
  SpawnParameters.ObjectFlags |= RF_Transient;
  ACameraActor *Camera = World->SpawnActor<ACameraActor>(
      ACameraActor::StaticClass(), FTransform(CameraRotation, CameraLocation), SpawnParameters);
  if (!IsValid(Camera) || !IsValid(Camera->GetCameraComponent())) {
    if (IsValid(Camera)) {
      Camera->Destroy();
    }
    UE_LOG(LogLingTuSimVisual, Error,
           TEXT("LINGTU_VISUAL_MOTION_CAMERA_BLOCKED reason=spawn_failed stable_id=%s"),
           *MotionCameraStableId);
    return false;
  }

  Camera->Tags.AddUnique(FName(TEXT("LingTuMotionCamera")));
  Camera->GetCameraComponent()->SetFieldOfView(55.0F);
  MotionCamera = Camera;
  FString CameraError;
  bMotionCameraReady = SetRuntimeCameraMode(ELingTuSimRuntimeCameraMode::Follow, CameraError);
  if (!bMotionCameraReady) {
    UE_LOG(LogLingTuSimVisual, Warning,
           TEXT("LINGTU_VISUAL_MOTION_CAMERA_DEFERRED reason=player_view_target_unavailable "
                "stable_id=%s camera=%s"),
           *MotionCameraStableId, *Camera->GetName());
    return false;
  }

  UE_LOG(LogLingTuSimVisual, Display,
         TEXT("LINGTU_VISUAL_MOTION_CAMERA_READY stable_id=%s camera=%s "
              "location_cm=(%.3f,%.3f,%.3f) look_at_cm=(%.3f,%.3f,%.3f) fov_deg=55 "
              "model_generation=%llu reset_generation=%llu sequence=%llu"),
         *MotionCameraStableId, *Camera->GetName(), CameraLocation.X, CameraLocation.Y,
         CameraLocation.Z, LookAtLocation.X, LookAtLocation.Y, LookAtLocation.Z,
         static_cast<unsigned long long>(Snapshot.ModelGeneration),
         static_cast<unsigned long long>(Snapshot.ResetGeneration),
         static_cast<unsigned long long>(Snapshot.Sequence));
  return true;
}

void ULingTuSimVisualWorldSubsystem::DestroyMotionCamera() {
  if (ACameraActor *Camera = FreeCamera.Get()) {
    Camera->Destroy();
  }
  FreeCamera.Reset();
  if (ACameraActor *Camera = MotionCamera.Get()) {
    Camera->Destroy();
  }
  MotionCamera.Reset();
  bMotionCameraReady = false;
  bLoggedMotionCameraBindingMissing = false;
  bLoggedMotionCameraCaptureBlocked = false;
  RuntimeCameraMode = ELingTuSimRuntimeCameraMode::Unavailable;
}

FString ULingTuSimVisualWorldSubsystem::GetRuntimeCameraModeName() const {
  switch (RuntimeCameraMode) {
    case ELingTuSimRuntimeCameraMode::Follow:
      return TEXT("follow");
    case ELingTuSimRuntimeCameraMode::Inspection:
      return TEXT("inspection");
    case ELingTuSimRuntimeCameraMode::Free:
      return TEXT("free");
    case ELingTuSimRuntimeCameraMode::Unavailable:
    default:
      return TEXT("unavailable");
  }
}

APlayerController *ULingTuSimVisualWorldSubsystem::FindPlayerController0() const {
  UWorld *World = GetWorld();
  if (World == nullptr) {
    return nullptr;
  }
  for (FConstPlayerControllerIterator It = World->GetPlayerControllerIterator(); It; ++It) {
    if (APlayerController *PlayerController = It->Get(); IsValid(PlayerController)) {
      return PlayerController;
    }
  }
  return nullptr;
}

bool ULingTuSimVisualWorldSubsystem::UpdateMotionCameraTransform() {
  ACameraActor *Camera = MotionCamera.Get();
  const TWeakObjectPtr<ULingTuSimBodyBindingComponent> *BindingPtr =
      Bindings.Find(MotionCameraStableId);
  ULingTuSimBodyBindingComponent *Binding = BindingPtr != nullptr ? BindingPtr->Get() : nullptr;
  if (!IsValid(Camera) || !IsValid(Binding)) {
    return false;
  }
  const FTransform RootTransform = Binding->GetComponentTransform();
  const FVector CameraLocation =
      RootTransform.GetLocation() +
      RootTransform.GetRotation().RotateVector(FVector(-300.0, 300.0, 180.0));
  const FVector LookAtLocation =
      RootTransform.GetLocation() +
      RootTransform.GetRotation().RotateVector(FVector(100.0, 0.0, 20.0));
  Camera->SetActorLocationAndRotation(CameraLocation, (LookAtLocation - CameraLocation).Rotation(),
                                      false, nullptr, ETeleportType::TeleportPhysics);
  return Camera->GetActorLocation().Equals(CameraLocation, 0.01);
}

ACameraActor *ULingTuSimVisualWorldSubsystem::FindUniqueInspectionCamera(FString &OutError) const {
  OutError.Reset();
  FString RequestedCameraTag;
  if ((!FParse::Value(FCommandLine::Get(), TEXT("LingTuSessionCameraTag="), RequestedCameraTag) &&
       !FParse::Value(FCommandLine::Get(), TEXT("-LingTuSessionCameraTag="), RequestedCameraTag)) ||
      RequestedCameraTag.TrimStartAndEnd().IsEmpty()) {
    OutError = TEXT("inspection_camera_tag_unavailable");
    return nullptr;
  }
  const FName RequiredTag(*RequestedCameraTag);
  ACameraActor *Match = nullptr;
  int32 MatchCount = 0;
  UWorld *World = GetWorld();
  if (World == nullptr) {
    OutError = TEXT("visual_world_unavailable");
    return nullptr;
  }
  for (TActorIterator<ACameraActor> It(World); It; ++It) {
    ACameraActor *Candidate = *It;
    if (IsValid(Candidate) && Candidate->ActorHasTag(RequiredTag)) {
      ++MatchCount;
      Match = Candidate;
    }
  }
  if (MatchCount != 1) {
    OutError =
        MatchCount == 0 ? TEXT("inspection_camera_not_found") : TEXT("inspection_camera_ambiguous");
    return nullptr;
  }
  return Match;
}

ACameraActor *ULingTuSimVisualWorldSubsystem::EnsureFreeCamera(FString &OutError) {
  OutError.Reset();
  if (ACameraActor *Existing = FreeCamera.Get()) {
    return Existing;
  }
  APlayerController *PlayerController = FindPlayerController0();
  AActor *CurrentViewTarget =
      IsValid(PlayerController) ? PlayerController->GetViewTarget() : nullptr;
  UWorld *World = GetWorld();
  if (!IsValid(CurrentViewTarget) || World == nullptr) {
    OutError = TEXT("free_camera_source_view_unavailable");
    return nullptr;
  }
  FActorSpawnParameters SpawnParameters;
  SpawnParameters.Name =
      MakeUniqueObjectName(World, ACameraActor::StaticClass(), FName(TEXT("LingTuFreeCamera")));
  SpawnParameters.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
  SpawnParameters.ObjectFlags |= RF_Transient;
  ACameraActor *Camera = World->SpawnActor<ACameraActor>(
      ACameraActor::StaticClass(), CurrentViewTarget->GetActorTransform(), SpawnParameters);
  if (!IsValid(Camera) || !IsValid(Camera->GetCameraComponent())) {
    if (IsValid(Camera)) {
      Camera->Destroy();
    }
    OutError = TEXT("free_camera_spawn_failed");
    return nullptr;
  }
  if (const ACameraActor *SourceCamera = Cast<ACameraActor>(CurrentViewTarget)) {
    if (const UCameraComponent *SourceComponent = SourceCamera->GetCameraComponent()) {
      Camera->GetCameraComponent()->SetFieldOfView(SourceComponent->FieldOfView);
    }
  }
  Camera->Tags.AddUnique(FName(TEXT("LingTuFreeCamera")));
  FreeCamera = Camera;
  return Camera;
}

bool ULingTuSimVisualWorldSubsystem::SetRuntimeCameraMode(
    const ELingTuSimRuntimeCameraMode RequestedMode, FString &OutError) {
  check(IsInGameThread());
  OutError.Reset();
  APlayerController *PlayerController = FindPlayerController0();
  if (!IsValid(PlayerController)) {
    OutError = TEXT("player_controller_0_unavailable");
    return false;
  }
  AActor *Candidate = nullptr;
  switch (RequestedMode) {
    case ELingTuSimRuntimeCameraMode::Follow:
      if (!UpdateMotionCameraTransform()) {
        OutError = TEXT("follow_camera_not_bound_to_motion_base");
        return false;
      }
      Candidate = MotionCamera.Get();
      break;
    case ELingTuSimRuntimeCameraMode::Inspection:
      Candidate = FindUniqueInspectionCamera(OutError);
      break;
    case ELingTuSimRuntimeCameraMode::Free:
      Candidate = EnsureFreeCamera(OutError);
      break;
    case ELingTuSimRuntimeCameraMode::Unavailable:
    default:
      OutError = TEXT("camera_mode_unavailable_cannot_be_activated");
      return false;
  }
  if (!IsValid(Candidate)) {
    if (OutError.IsEmpty()) {
      OutError = TEXT("camera_mode_target_unavailable");
    }
    return false;
  }
  AActor *PreviousViewTarget = PlayerController->GetViewTarget();
  PlayerController->SetViewTarget(Candidate);
  if (PlayerController->GetViewTarget() != Candidate) {
    if (IsValid(PreviousViewTarget)) {
      PlayerController->SetViewTarget(PreviousViewTarget);
    }
    OutError = TEXT("camera_view_target_readback_failed");
    return false;
  }
  RuntimeCameraMode = RequestedMode;
  UE_LOG(LogLingTuSimVisual, Display,
         TEXT("LINGTU_VISUAL_RUNTIME_CAMERA_MODE mode=%s target=%s readback=confirmed"),
         *GetRuntimeCameraModeName(), *Candidate->GetName());
  return true;
}

bool ULingTuSimVisualWorldSubsystem::CycleRuntimeCameraMode(FString &OutModeName,
                                                            FString &OutError) {
  const ELingTuSimRuntimeCameraMode NextMode =
      RuntimeCameraMode == ELingTuSimRuntimeCameraMode::Follow
          ? ELingTuSimRuntimeCameraMode::Inspection
      : RuntimeCameraMode == ELingTuSimRuntimeCameraMode::Inspection
          ? ELingTuSimRuntimeCameraMode::Free
          : ELingTuSimRuntimeCameraMode::Follow;
  if (!SetRuntimeCameraMode(NextMode, OutError)) {
    OutModeName = GetRuntimeCameraModeName();
    return false;
  }
  OutModeName = GetRuntimeCameraModeName();
  return true;
}

bool ULingTuSimVisualWorldSubsystem::ApplyRuntimeFreeCameraLook(const float CameraYaw,
                                                                const float CameraPitch,
                                                                FString &OutError) {
  check(IsInGameThread());
  OutError.Reset();
  if (RuntimeCameraMode != ELingTuSimRuntimeCameraMode::Free) {
    return true;
  }
  ACameraActor *Camera = FreeCamera.Get();
  APlayerController *PlayerController = FindPlayerController0();
  if (!IsValid(Camera) || !IsValid(PlayerController) ||
      PlayerController->GetViewTarget() != Camera) {
    OutError = TEXT("free_camera_view_target_not_confirmed");
    return false;
  }
  const float Yaw = FMath::Clamp(CameraYaw, -1.0F, 1.0F);
  const float Pitch = FMath::Clamp(CameraPitch, -1.0F, 1.0F);
  if (FMath::IsNearlyZero(Yaw) && FMath::IsNearlyZero(Pitch)) {
    return true;
  }
  FRotator Rotation = Camera->GetActorRotation();
  Rotation.Yaw += Yaw * 2.0F;
  Rotation.Pitch = FMath::Clamp(Rotation.Pitch - Pitch * 2.0F, -85.0F, 85.0F);
  Camera->SetActorRotation(Rotation, ETeleportType::TeleportPhysics);
  if (!Camera->GetActorRotation().Equals(Rotation, 0.01F)) {
    OutError = TEXT("free_camera_rotation_readback_failed");
    return false;
  }
  return true;
}

bool ULingTuSimVisualWorldSubsystem::SetSessionCameraViewTargetForPlayer0() {
  UWorld *World = GetWorld();
  if (World == nullptr) {
    return false;
  }

  FString SelectedCameraSource(TEXT("motion_camera"));
  ACameraActor *SessionCamera = MotionCamera.Get();
  if (!IsValid(SessionCamera)) {
    FString RequestedCameraTag;
    const bool bHasRequestedCameraTag =
        FParse::Value(FCommandLine::Get(), TEXT("LingTuSessionCameraTag="), RequestedCameraTag) ||
        FParse::Value(FCommandLine::Get(), TEXT("-LingTuSessionCameraTag="), RequestedCameraTag);
    RequestedCameraTag.TrimStartAndEndInline();
    if (bHasRequestedCameraTag && RequestedCameraTag.IsEmpty()) {
      UE_LOG(LogLingTuSimVisual, Error,
             TEXT("LINGTU_VISUAL_SESSION_CAMERA_REQUEST_INVALID reason=empty_tag"));
      return false;
    }

    const FName RequiredCameraTag =
        bHasRequestedCameraTag ? FName(*RequestedCameraTag) : FName(TEXT("SessionCamera"));
    SelectedCameraSource = RequiredCameraTag.ToString();
    int32 MatchingCameraCount = 0;
    for (TActorIterator<ACameraActor> It(World); It; ++It) {
      ACameraActor *Candidate = *It;
      if (IsValid(Candidate) && Candidate->ActorHasTag(RequiredCameraTag)) {
        ++MatchingCameraCount;
        if (SessionCamera == nullptr) {
          SessionCamera = Candidate;
        }
      }
    }
    if (bHasRequestedCameraTag && MatchingCameraCount > 1) {
      UE_LOG(LogLingTuSimVisual, Error,
             TEXT("LINGTU_VISUAL_SESSION_CAMERA_AMBIGUOUS tag=%s matches=%d"), *RequestedCameraTag,
             MatchingCameraCount);
      return false;
    }
    if (SessionCamera == nullptr) {
      if (bHasRequestedCameraTag) {
        UE_LOG(LogLingTuSimVisual, Error, TEXT("LINGTU_VISUAL_SESSION_CAMERA_NOT_FOUND tag=%s"),
               *RequestedCameraTag);
      }
      return false;
    }
  }

  if (!IsValid(SessionCamera)) {
    if (bMotionCameraRequested) {
      UE_LOG(LogLingTuSimVisual, Error,
             TEXT("LINGTU_VISUAL_MOTION_CAMERA_BLOCKED reason=camera_invalid stable_id=%s"),
             *MotionCameraStableId);
    }
    return false;
  }

  APlayerController *PlayerController = nullptr;
  for (FConstPlayerControllerIterator It = World->GetPlayerControllerIterator(); It; ++It) {
    PlayerController = It->Get();
    if (IsValid(PlayerController)) {
      break;
    }
  }
  if (!IsValid(PlayerController)) {
    return false;
  }

  PlayerController->SetViewTarget(SessionCamera);
  if (PlayerController->GetViewTarget() != SessionCamera) {
    UE_LOG(LogLingTuSimVisual, Warning,
           TEXT("LINGTU_VISUAL_SESSION_CAMERA_BIND_FAILED camera=%s player=0"),
           *SessionCamera->GetName());
    return false;
  }
  UE_LOG(LogLingTuSimVisual, Display,
         TEXT("LINGTU_VISUAL_SESSION_CAMERA_BOUND camera=%s source=%s player=0"),
         *SessionCamera->GetName(), *SelectedCameraSource);
  return true;
}

void ULingTuSimVisualWorldSubsystem::EnterWaitingForRebind(
    const LingTuSim::FSnapshotEnvelope &FutureSnapshot) {
  check(IsInGameThread());
  bWaitingForRebind = true;
  LingTuSim::FSessionService::ClearSnapshots();
  UE_LOG(LogLingTuSimVisual, Warning,
         TEXT("LINGTU_VISUAL_WAITING_FOR_REBIND bound_digest=%s bound_model_generation=%llu "
              "observed_digest=%s observed_model_generation=%llu"),
         *BoundSessionId, static_cast<unsigned long long>(BoundModelGeneration),
         *FutureSnapshot.SessionId,
         static_cast<unsigned long long>(FutureSnapshot.ModelGeneration));
}

bool ULingTuSimVisualWorldSubsystem::TryApplyCompleteFrame(
    const LingTuSim::FSnapshotEnvelope &Snapshot) {
  for (auto It = Bindings.CreateIterator(); It; ++It) {
    if (!It.Value().IsValid()) {
      It.RemoveCurrent();
    }
  }

  if (Bindings.Num() == 0) {
    return false;
  }

  TMap<FString, const LingTuSim::FEntityState *> RequiredEntities;
  RequiredEntities.Reserve(Bindings.Num());
  for (const LingTuSim::FEntityState &Entity : Snapshot.Entities) {
    if (!Bindings.Contains(Entity.Id.StableId)) {
      continue;
    }
    if (RequiredEntities.Contains(Entity.Id.StableId)) {
      return false;
    }
    RequiredEntities.Add(Entity.Id.StableId, &Entity);
  }

  if (RequiredEntities.Num() != Bindings.Num()) {
    return false;
  }

  TArray<FPreparedBindingTransform> PreparedTransforms;
  PreparedTransforms.Reserve(Bindings.Num());
  for (const TPair<FString, TWeakObjectPtr<ULingTuSimBodyBindingComponent>> &Pair : Bindings) {
    ULingTuSimBodyBindingComponent *Binding = Pair.Value.Get();
    const LingTuSim::FEntityState *const *Entity = RequiredEntities.Find(Pair.Key);
    if (Binding == nullptr || Entity == nullptr || *Entity == nullptr) {
      return false;
    }

    FPreparedBindingTransform &Prepared = PreparedTransforms.AddDefaulted_GetRef();
    Prepared.Binding = Binding;
    if (!LingTuSim::Visual::FCoordinateConverter::TryMakeWorldTransform(
            **Entity, Binding->GetComponentTransform().GetScale3D(), Prepared.Transform)) {
      return false;
    }
  }

  if (!AreActiveActorRenderResourcesReady()) {
    FScopeLock Lock(&SubmissionCriticalSection);
    bHasRenderDeferredSnapshot = true;
    RenderDeferredSnapshot = Snapshot;
    return false;
  }

  for (const FPreparedBindingTransform &Prepared : PreparedTransforms) {
    Prepared.Binding->SetWorldTransform(Prepared.Transform, false, nullptr,
                                        ETeleportType::TeleportPhysics);
  }
  RevealActiveActors();
  EnsureMotionCameraForAppliedSnapshot(Snapshot);

  SnapshotGate.Commit(Snapshot);
  {
    FScopeLock Lock(&SubmissionCriticalSection);
    LatestAppliedSnapshot = Snapshot;
    bHasLatestAppliedSnapshot = true;
  }
  ReadinessSessionId = Snapshot.SessionId;
  BoundModelGeneration = Snapshot.ModelGeneration;
  ReadinessResetGeneration = Snapshot.ResetGeneration;
  if (bReadinessEvidenceConfigured && !bVisualReadinessActive) {
    FString EvidenceError;
    if (WriteReadinessEvidence(TEXT("ACTIVE"), EvidenceError)) {
      bVisualReadinessActive = true;
    } else {
      UE_LOG(LogLingTuSimVisual, Error, TEXT("LINGTU_VISUAL_READINESS_WRITE_FAILED reason=%s"),
             *EvidenceError);
    }
  }
  if (!bLoggedFirstAppliedFrame) {
    bLoggedFirstAppliedFrame = true;
    UE_LOG(LogLingTuSimVisual, Display,
           TEXT("LINGTU_VISUAL_FRAME_APPLIED reset_generation=%llu sequence=%llu bindings=%d"),
           static_cast<unsigned long long>(Snapshot.ResetGeneration),
           static_cast<unsigned long long>(Snapshot.Sequence), PreparedTransforms.Num());
  }
  RequestFirstFrameScreenshotIfReady(Snapshot);
  RequestFrameCaptureIfReady(Snapshot);
  return true;
}

void ULingTuSimVisualWorldSubsystem::InitializeReadinessEvidenceFromCommandLine() {
  FString AllocationPath;
  if (!FParse::Value(FCommandLine::Get(), TEXT("LingTuRunAllocation="), AllocationPath) ||
      AllocationPath.IsEmpty()) {
    return;
  }

  FString AllocationJson;
  FString Error;
  FString CanonicalAllocationPath;
  if (!ValidateRunAllocationFilePath(AllocationPath, CanonicalAllocationPath, Error)) {
    UE_LOG(LogLingTuSimVisual, Warning,
           TEXT("LINGTU_VISUAL_READINESS_ALLOCATION_IGNORED path=%s reason=%s"), *AllocationPath,
           *Error);
    return;
  }
  TSharedPtr<FJsonObject> Root;
  if (!FFileHelper::LoadFileToString(AllocationJson, *CanonicalAllocationPath) ||
      !ParseJsonObject(AllocationJson, Root, Error)) {
    UE_LOG(LogLingTuSimVisual, Warning,
           TEXT("LINGTU_VISUAL_READINESS_ALLOCATION_IGNORED path=%s reason=%s"),
           *CanonicalAllocationPath, Error.IsEmpty() ? TEXT("read failed") : *Error);
    return;
  }
  if (SafeStringField(Root, TEXT("schema")) != TEXT("lingtu.sim.run-allocation.v1")) {
    UE_LOG(LogLingTuSimVisual, Warning,
           TEXT("LINGTU_VISUAL_READINESS_ALLOCATION_IGNORED path=%s reason=schema mismatch"),
           *CanonicalAllocationPath);
    return;
  }

  const FString AllocationRunId = SafeStringField(Root, TEXT("run_id"));
  FString CommandLineRunId;
  if ((!FParse::Value(FCommandLine::Get(), TEXT("LingTuRunId="), CommandLineRunId) &&
       !FParse::Value(FCommandLine::Get(), TEXT("-LingTuRunId="), CommandLineRunId)) ||
      AllocationRunId.IsEmpty() || CommandLineRunId != AllocationRunId) {
    UE_LOG(LogLingTuSimVisual, Warning,
           TEXT("LINGTU_VISUAL_READINESS_ALLOCATION_IGNORED path=%s reason=run_id mismatch"),
           *CanonicalAllocationPath);
    return;
  }

  FString CanonicalLogDirectory;
  if (!ValidateRunAllocationEvidencePaths(CanonicalAllocationPath, AllocationRunId,
                                          SafeStringField(Root, TEXT("log_dir")),
                                          CanonicalLogDirectory, Error)) {
    UE_LOG(LogLingTuSimVisual, Warning,
           TEXT("LINGTU_VISUAL_READINESS_ALLOCATION_IGNORED path=%s reason=%s"),
           *CanonicalAllocationPath, *Error);
    return;
  }

  uint64 ModelGeneration = 0;
  uint64 ResetGeneration = 0;
  if (!ReadRequiredCommandLineGeneration(TEXT("LingTuModelGeneration="), ModelGeneration) ||
      !ReadRequiredCommandLineGeneration(TEXT("LingTuResetGeneration="), ResetGeneration)) {
    UE_LOG(LogLingTuSimVisual, Warning,
           TEXT("LINGTU_VISUAL_READINESS_ALLOCATION_IGNORED path=%s reason=missing or invalid "
                "generation"),
           *CanonicalAllocationPath);
    return;
  }

  FString ConfigureError;
  if (!ConfigureReadinessEvidence(CanonicalLogDirectory, AllocationRunId,
                                  SafeStringField(Root, TEXT("session_id")), ModelGeneration,
                                  ResetGeneration, ConfigureError)) {
    UE_LOG(LogLingTuSimVisual, Warning,
           TEXT("LINGTU_VISUAL_READINESS_ALLOCATION_IGNORED path=%s reason=%s"),
           *CanonicalAllocationPath, *ConfigureError);
  }
}

void ULingTuSimVisualWorldSubsystem::TryWritePreparedReadinessEvidence() {
  if (!bReadinessEvidenceConfigured || Bindings.Num() == 0 || ReadinessSessionId.IsEmpty() ||
      bVisualReadinessActive) {
    return;
  }

  FString EvidenceError;
  if (!WriteReadinessEvidence(TEXT("PREPARED"), EvidenceError)) {
    UE_LOG(LogLingTuSimVisual, Error, TEXT("LINGTU_VISUAL_READINESS_WRITE_FAILED reason=%s"),
           *EvidenceError);
  }
}

void ULingTuSimVisualWorldSubsystem::InitializeFirstFrameScreenshotFromCommandLine() {
  FirstFrameScreenshotPath.Reset();
  bFirstFrameScreenshotRequested = false;
  FString ScreenshotPath;
  if (!FParse::Value(FCommandLine::Get(), TEXT("LingTuScreenshot="), ScreenshotPath) &&
      !FParse::Value(FCommandLine::Get(), TEXT("-LingTuScreenshot="), ScreenshotPath)) {
    return;
  }
  if (ScreenshotPath.IsEmpty()) {
    UE_LOG(LogLingTuSimVisual, Warning, TEXT("LINGTU_VISUAL_SCREENSHOT_IGNORED reason=empty_path"));
    return;
  }
  FirstFrameScreenshotPath = FPaths::ConvertRelativePathToFull(ScreenshotPath);
}

void ULingTuSimVisualWorldSubsystem::RequestFirstFrameScreenshotIfReady(
    const LingTuSim::FSnapshotEnvelope &Snapshot) {
  if (bFirstFrameScreenshotRequested || FirstFrameScreenshotPath.IsEmpty()) {
    return;
  }
  if (!bActiveActorsRevealed || Bindings.Num() == 0) {
    return;
  }
  SetSessionCameraViewTargetForPlayer0();

  IFileManager::Get().MakeDirectory(*FPaths::GetPath(FirstFrameScreenshotPath), true);
  FScreenshotRequest::RequestScreenshot(FirstFrameScreenshotPath, false, false);
  bFirstFrameScreenshotRequested = true;
  UE_LOG(LogLingTuSimVisual, Display,
         TEXT("LINGTU_VISUAL_FIRST_FRAME_SCREENSHOT_REQUESTED path=%s reset_generation=%llu "
              "sequence=%llu bindings=%d"),
         *FirstFrameScreenshotPath, static_cast<unsigned long long>(Snapshot.ResetGeneration),
         static_cast<unsigned long long>(Snapshot.Sequence), Bindings.Num());
}

void ULingTuSimVisualWorldSubsystem::InitializeFrameCaptureFromCommandLine() {
  FrameCaptureDirectory.Reset();
  FrameCaptureEvery = 0;
  FrameCaptureMax = 0;
  FrameCaptureRequested = 0;
  FrameCaptureAppliedSnapshotCount = 0;
  bFrameCaptureEnabled = false;
  bFrameCaptureDue = false;
  bLoggedFrameCapturePending = false;
  bLoggedFrameCaptureComplete = false;

  FString CaptureDirectory;
  if (!FParse::Value(FCommandLine::Get(), TEXT("LingTuFrameCaptureDir="), CaptureDirectory) &&
      !FParse::Value(FCommandLine::Get(), TEXT("-LingTuFrameCaptureDir="), CaptureDirectory)) {
    return;
  }

  FString EveryText;
  FString MaxText;
  const bool bHasEvery =
      FParse::Value(FCommandLine::Get(), TEXT("LingTuFrameCaptureEvery="), EveryText) ||
      FParse::Value(FCommandLine::Get(), TEXT("-LingTuFrameCaptureEvery="), EveryText);
  const bool bHasMax =
      FParse::Value(FCommandLine::Get(), TEXT("LingTuFrameCaptureMax="), MaxText) ||
      FParse::Value(FCommandLine::Get(), TEXT("-LingTuFrameCaptureMax="), MaxText);
  if (CaptureDirectory.IsEmpty() || !bHasEvery || !bHasMax || !EveryText.IsNumeric() ||
      !MaxText.IsNumeric()) {
    UE_LOG(LogLingTuSimVisual, Error,
           TEXT("LINGTU_VISUAL_FRAME_CAPTURE_DISABLED reason=invalid_configuration dir=%s every=%s "
                "max=%s"),
           *CaptureDirectory, *EveryText, *MaxText);
    return;
  }

  const uint64 ParsedEvery = FCString::Strtoui64(*EveryText, nullptr, 10);
  const uint64 ParsedMax = FCString::Strtoui64(*MaxText, nullptr, 10);
  if (ParsedEvery == 0 || ParsedEvery > static_cast<uint64>(MAX_int32) || ParsedMax == 0 ||
      ParsedMax > static_cast<uint64>(MAX_int32)) {
    UE_LOG(LogLingTuSimVisual, Error,
           TEXT("LINGTU_VISUAL_FRAME_CAPTURE_DISABLED reason=non_positive_or_oversized_bound "
                "every=%s max=%s"),
           *EveryText, *MaxText);
    return;
  }

  FrameCaptureDirectory = FPaths::ConvertRelativePathToFull(CaptureDirectory);
  if (!IFileManager::Get().DirectoryExists(*FrameCaptureDirectory) &&
      !IFileManager::Get().MakeDirectory(*FrameCaptureDirectory, true)) {
    UE_LOG(LogLingTuSimVisual, Error,
           TEXT("LINGTU_VISUAL_FRAME_CAPTURE_DISABLED reason=create_directory_failed dir=%s"),
           *FrameCaptureDirectory);
    FrameCaptureDirectory.Reset();
    return;
  }

  FrameCaptureEvery = static_cast<int32>(ParsedEvery);
  FrameCaptureMax = static_cast<int32>(ParsedMax);
  bFrameCaptureEnabled = true;
  UE_LOG(LogLingTuSimVisual, Display,
         TEXT("LINGTU_VISUAL_FRAME_CAPTURE_CONFIGURED dir=%s every=%d max=%d"),
         *FrameCaptureDirectory, FrameCaptureEvery, FrameCaptureMax);
}

void ULingTuSimVisualWorldSubsystem::RequestFrameCaptureIfReady(
    const LingTuSim::FSnapshotEnvelope &Snapshot) {
  if (!bFrameCaptureEnabled) {
    return;
  }
  if (bMotionCameraRequested && !bMotionCameraReady) {
    if (!bLoggedMotionCameraCaptureBlocked) {
      bLoggedMotionCameraCaptureBlocked = true;
      UE_LOG(LogLingTuSimVisual, Error,
             TEXT("LINGTU_VISUAL_FRAME_CAPTURE_BLOCKED reason=motion_camera_not_ready stable_id=%s "
                  "model_generation=%llu reset_generation=%llu sequence=%llu"),
             *MotionCameraStableId, static_cast<unsigned long long>(Snapshot.ModelGeneration),
             static_cast<unsigned long long>(Snapshot.ResetGeneration),
             static_cast<unsigned long long>(Snapshot.Sequence));
    }
    return;
  }
  bLoggedMotionCameraCaptureBlocked = false;

  ++FrameCaptureAppliedSnapshotCount;
  if ((FrameCaptureAppliedSnapshotCount - 1) % static_cast<uint64>(FrameCaptureEvery) == 0) {
    bFrameCaptureDue = true;
  }
  if (!bFrameCaptureDue) {
    return;
  }
  if (FrameCaptureRequested >= FrameCaptureMax) {
    if (!bLoggedFrameCaptureComplete) {
      bLoggedFrameCaptureComplete = true;
      UE_LOG(LogLingTuSimVisual, Display,
             TEXT("LINGTU_VISUAL_FRAME_CAPTURE_COMPLETE requested=%d applied_snapshots=%llu"),
             FrameCaptureRequested,
             static_cast<unsigned long long>(FrameCaptureAppliedSnapshotCount));
    }
    return;
  }
  if (FScreenshotRequest::IsScreenshotRequested()) {
    if (!bLoggedFrameCapturePending) {
      bLoggedFrameCapturePending = true;
      UE_LOG(LogLingTuSimVisual, Warning,
             TEXT("LINGTU_VISUAL_FRAME_CAPTURE_DEFERRED reason=screenshot_request_pending "
                  "reset_generation=%llu sequence=%llu"),
             static_cast<unsigned long long>(Snapshot.ResetGeneration),
             static_cast<unsigned long long>(Snapshot.Sequence));
    }
    return;
  }

  if (!IFileManager::Get().DirectoryExists(*FrameCaptureDirectory) &&
      !IFileManager::Get().MakeDirectory(*FrameCaptureDirectory, true)) {
    bFrameCaptureEnabled = false;
    UE_LOG(LogLingTuSimVisual, Error,
           TEXT("LINGTU_VISUAL_FRAME_CAPTURE_DISABLED reason=create_directory_failed dir=%s"),
           *FrameCaptureDirectory);
    return;
  }

  const int32 CaptureIndex = FrameCaptureRequested;
  const FString ScreenshotPath =
      FPaths::Combine(FrameCaptureDirectory, FString::Printf(TEXT("frame_%06d.png"), CaptureIndex));
  if (IFileManager::Get().FileExists(*ScreenshotPath)) {
    bFrameCaptureEnabled = false;
    UE_LOG(
        LogLingTuSimVisual, Error,
        TEXT("LINGTU_VISUAL_FRAME_CAPTURE_DISABLED reason=target_exists path=%s capture_index=%d"),
        *ScreenshotPath, CaptureIndex);
    return;
  }

  FScreenshotRequest::RequestScreenshot(ScreenshotPath, false, false, false, FIntRect(), true);
  if (!FScreenshotRequest::IsScreenshotRequested()) {
    bFrameCaptureEnabled = false;
    UE_LOG(LogLingTuSimVisual, Error,
           TEXT("LINGTU_VISUAL_FRAME_CAPTURE_DISABLED reason=request_rejected path=%s"),
           *ScreenshotPath);
    return;
  }

  ++FrameCaptureRequested;
  bFrameCaptureDue = false;
  bLoggedFrameCapturePending = false;
  UE_LOG(LogLingTuSimVisual, Display,
         TEXT("LINGTU_VISUAL_FRAME_CAPTURE_REQUESTED capture_index=%d model_generation=%llu "
              "reset_generation=%llu sequence=%llu sim_time_ns=%lld path=%s requested=%d max=%d"),
         CaptureIndex, static_cast<unsigned long long>(Snapshot.ModelGeneration),
         static_cast<unsigned long long>(Snapshot.ResetGeneration),
         static_cast<unsigned long long>(Snapshot.Sequence),
         static_cast<long long>(Snapshot.SimTimeNs), *ScreenshotPath, FrameCaptureRequested,
         FrameCaptureMax);
}

bool ULingTuSimVisualWorldSubsystem::WriteReadinessEvidence(const TCHAR *VisualState,
                                                            FString &OutError) const {
  OutError.Reset();
  if (ReadinessEvidenceDirectory.IsEmpty()) {
    OutError = TEXT("visual readiness evidence directory is not configured");
    return false;
  }
  if (ReadinessSessionId.IsEmpty()) {
    OutError = TEXT("visual readiness session id is not configured");
    return false;
  }

  IFileManager::Get().MakeDirectory(*ReadinessEvidenceDirectory, true);
  const FString EvidencePath =
      FPaths::Combine(ReadinessEvidenceDirectory, TEXT("visual-readiness.json"));
  const FString TempEvidencePath =
      FString::Printf(TEXT("%s.tmp.%u"), *EvidencePath, FPlatformProcess::GetCurrentProcessId());
  const FString Json = FString::Printf(
      TEXT("{\n"
           "  \"schema\": \"lingtu.sim.sensor-readiness-evidence.v1\",\n"
           "  \"session_id\": \"%s\",\n"
           "  \"model_generation\": %llu,\n"
           "  \"reset_generation\": %llu,\n"
           "  \"source_id\": \"robotsimue-visual\",\n"
           "  \"basis\": \"truth_snapshot_applied_to_visual_bindings\",\n"
           "  \"visual\": {\"state\": \"%s\"},\n"
           "  \"sensors\": {\"camera_streams\": \"PREPARING\", \"overall\": \"PREPARING\"},\n"
           "  \"streams\": []\n"
           "}\n"),
      *ReadinessSessionId, static_cast<unsigned long long>(BoundModelGeneration),
      static_cast<unsigned long long>(ReadinessResetGeneration), VisualState);
  if (!FFileHelper::SaveStringToFile(Json, *TempEvidencePath,
                                     FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM) ||
      !ReplaceReadinessFileWithRetry(EvidencePath, TempEvidencePath)) {
    IFileManager::Get().Delete(*TempEvidencePath, false, true, true);
    OutError =
        FString::Printf(TEXT("failed to write visual readiness evidence '%s'"), *EvidencePath);
    return false;
  }
  return true;
}

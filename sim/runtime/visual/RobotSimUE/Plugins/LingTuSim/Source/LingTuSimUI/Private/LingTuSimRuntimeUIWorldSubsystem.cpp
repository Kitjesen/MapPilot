#include "LingTuSimRuntimeUIWorldSubsystem.h"

#include "Engine/GameInstance.h"
#include "Engine/GameViewportClient.h"
#include "Engine/World.h"
#include "Framework/Application/SlateApplication.h"
#include "GameFramework/PlayerController.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformMisc.h"
#include "HAL/PlatformTime.h"
#include "LingTuSimControlTransport.h"
#include "LingTuSimGameSelection.h"
#include "LingTuSimHudScreenshotContract.h"
#include "LingTuSimRuntimeUIInputProcessor.h"
#include "LingTuSimRuntimeUIModel.h"
#include "LingTuSimRuntimeUIPolicy.h"
#include "LingTuSimSessionService.h"
#include "LingTuSimVisualWorldSubsystem.h"
#include "Misc/App.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "SLingTuSimRuntimeHUD.h"
#include "UnrealClient.h"
#include "Widgets/SViewport.h"

DEFINE_LOG_CATEGORY_STATIC(LogLingTuSimUI, Log, All);

namespace {
TArray<TWeakObjectPtr<ULingTuSimRuntimeUIWorldSubsystem>> GEligibleRuntimeUIContexts;

struct FPublishedInputLifecycle final {
  LingTuSim::UI::ERuntimeUIMode Mode = LingTuSim::UI::ERuntimeUIMode::Drive;
  bool bDeadman = false;
};

bool SamePackageBinding(const LingTuSim::UI::FGameSelectionPackageSummary &Left,
                        const LingTuSim::UI::FGameSelectionPackageSummary &Right) {
  return Left.Id == Right.Id && Left.Version == Right.Version;
}

bool SameScenarioBinding(const TOptional<LingTuSim::UI::FGameSelectionPackageSummary> &Left,
                         const TOptional<LingTuSim::UI::FGameSelectionPackageSummary> &Right) {
  return Left.IsSet() == Right.IsSet() &&
         (!Left.IsSet() || SamePackageBinding(Left.GetValue(), Right.GetValue()));
}

bool SameCompiledSelectionBinding(const LingTuSim::UI::FGameSelectionOption &Left,
                                  const LingTuSim::UI::FGameSelectionOption &Right) {
  return Left.Id == Right.Id && Left.Availability == Right.Availability &&
         Left.Mode == Right.Mode && Left.SessionId == Right.SessionId &&
         FPaths::IsSamePath(Left.BundleDirectory, Right.BundleDirectory) &&
         SamePackageBinding(Left.Robot, Right.Robot) &&
         SamePackageBinding(Left.World, Right.World) &&
         SameScenarioBinding(Left.Scenario, Right.Scenario);
}

uint64 SourceMonotonicNowNs() {
  static const double SourceClockOriginSeconds = FPlatformTime::Seconds();
  static uint64 LastSourceTimeNs = 0;
  const double ElapsedSeconds = FPlatformTime::Seconds() - SourceClockOriginSeconds;
  const uint64 Candidate = ElapsedSeconds >= 0.0 && FMath::IsFinite(ElapsedSeconds)
                               ? static_cast<uint64>(ElapsedSeconds * 1'000'000'000.0)
                               : 0;
  LastSourceTimeNs = FMath::Max(LastSourceTimeNs + 1, Candidate);
  return LastSourceTimeNs;
}

int32 RegisterEligibleContext(ULingTuSimRuntimeUIWorldSubsystem *Context) {
  GEligibleRuntimeUIContexts.RemoveAll(
      [](const TWeakObjectPtr<ULingTuSimRuntimeUIWorldSubsystem> &Candidate) {
        return !Candidate.IsValid();
      });
  GEligibleRuntimeUIContexts.AddUnique(TWeakObjectPtr<ULingTuSimRuntimeUIWorldSubsystem>(Context));
  return GEligibleRuntimeUIContexts.Num();
}

void UnregisterEligibleContext(ULingTuSimRuntimeUIWorldSubsystem *Context) {
  GEligibleRuntimeUIContexts.RemoveAll(
      [Context](const TWeakObjectPtr<ULingTuSimRuntimeUIWorldSubsystem> &Candidate) {
        return !Candidate.IsValid() || Candidate.Get() == Context;
      });
}

uint64 CaptureMonotonicNowNs() {
  const double Nanoseconds = FPlatformTime::Seconds() * 1'000'000'000.0;
  return FMath::IsFinite(Nanoseconds) && Nanoseconds > 0.0 ? static_cast<uint64>(Nanoseconds) : 0;
}

bool IsCaptureReadyForTarget(const LingTuSim::UI::FHudScreenshotTarget &Target,
                             const LingTuSim::UI::FRuntimeUIStatusSnapshot &Status) {
  switch (Target.Mode) {
    case LingTuSim::UI::ERuntimeUIMode::Drive:
      return Status.IsDriveCaptureReady();
    case LingTuSim::UI::ERuntimeUIMode::Tactical:
      return Status.IsTacticalCaptureReady();
    case LingTuSim::UI::ERuntimeUIMode::Pause:
      return Status.IsMenuRecordingCaptureReady();
    default:
      return false;
  }
}
}  // namespace

bool ULingTuSimRuntimeUIWorldSubsystem::ShouldCreateSubsystem(UObject *Outer) const {
  if (!Super::ShouldCreateSubsystem(Outer)) {
    return false;
  }

  const UWorld *World = Cast<UWorld>(Outer);
  if (World == nullptr ||
      (World->WorldType != EWorldType::Game && World->WorldType != EWorldType::PIE &&
       World->WorldType != EWorldType::GamePreview)) {
    return false;
  }

  return LingTuSim::UI::FRuntimeUIPolicy::ShouldEnable(FCommandLine::Get(), FApp::IsUnattended(),
                                                       FApp::IsGame());
}

LingTuSim::UI::FRuntimeUIStatusSnapshot
ULingTuSimRuntimeUIWorldSubsystem::ReadStatusSnapshot() const {
  return LingTuSim::UI::FRuntimeUIStatusReader::Read(GetWorld(), LocalState.Get());
}

void ULingTuSimRuntimeUIWorldSubsystem::OnWorldBeginPlay(UWorld &InWorld) {
  Super::OnWorldBeginPlay(InWorld);
  if (HUDWidget.IsValid() || !FSlateApplication::IsInitialized()) {
    return;
  }

  UGameInstance *GameInstance = InWorld.GetGameInstance();
  UGameViewportClient *Viewport =
      GameInstance != nullptr ? GameInstance->GetGameViewportClient() : nullptr;
  if (Viewport == nullptr) {
    UE_LOG(LogLingTuSimUI, Warning,
           TEXT("LINGTU_RUNTIME_UI_UNAVAILABLE reason=game_viewport_missing"));
    return;
  }

  const int32 EligibleContextCount = RegisterEligibleContext(this);
  bEligibleContextRegistered = true;
  const int32 LocalPlayerCount = GameInstance->GetNumLocalPlayers();
  if (!LingTuSim::UI::FRuntimeUIPolicy::CanOwnInputContext(EligibleContextCount,
                                                           LocalPlayerCount)) {
    for (const TWeakObjectPtr<ULingTuSimRuntimeUIWorldSubsystem> &Candidate :
         GEligibleRuntimeUIContexts) {
      if (ULingTuSimRuntimeUIWorldSubsystem *Other = Candidate.Get()) {
        Other->DetachRuntimeUI();
      }
    }
    UE_LOG(LogLingTuSimUI, Error,
           TEXT("LINGTU_RUNTIME_UI_INPUT_UNAVAILABLE reason=multiple_eligible_game_worlds "
                "eligible_worlds=%d local_players=%d"),
           EligibleContextCount, LocalPlayerCount);
    return;
  }

  const TSharedPtr<SViewport> GameViewportWidget = Viewport->GetGameViewportWidget();
  if (!GameViewportWidget.IsValid()) {
    UE_LOG(LogLingTuSimUI, Warning,
           TEXT("LINGTU_RUNTIME_UI_UNAVAILABLE reason=game_viewport_widget_missing"));
    return;
  }

  ModeController = MakeShared<LingTuSim::UI::FRuntimeUIModeController>();
  LocalState = MakeShared<LingTuSim::UI::FRuntimeUILocalState>();
  FrontEndLoginModel = MakeShared<LingTuSim::UI::FFrontEndLoginModel>();
  GameSelectionModel = MakeShared<LingTuSim::UI::FGameSelectionModel>();
  AssetReviewModel = MakeShared<LingTuSim::UI::FAssetReviewModel>();
  GameSelectionFeedback = MakeShared<FString>();
  InitializeGameSelectionFromCommandLine();
  FrontEndLoginModel->BindSessionModel(*GameSelectionModel);
  AssetReviewModel->BindSessionModel(*GameSelectionModel);
  AssetReviewModel->SetCatalog(GameSelectionModel->GetCatalog().AssetReview);
  if (bGameSelector) {
    ModeController->TogglePause();
  }
  const TSharedPtr<LingTuSim::UI::SLingTuSimRuntimeHUD> RuntimeHUD =
      SNew(LingTuSim::UI::SLingTuSimRuntimeHUD)
          .World(&InWorld)
          .ModeController(ModeController)
          .LocalState(LocalState)
          .LoginModel(FrontEndLoginModel)
          .SelectionModel(GameSelectionModel)
          .AssetReviewModel(AssetReviewModel)
          .SelectionFeedback(GameSelectionFeedback)
          .FrontEndLoginRequired(bGameSelector)
          .SelectionIntentConfigured(!GameSelectionIntentPath.IsEmpty())
          .OnSelectionPrevious(FSimpleDelegate::CreateUObject(
              this, &ULingTuSimRuntimeUIWorldSubsystem::HandlePreviousGameSelection))
          .OnSelectionNext(FSimpleDelegate::CreateUObject(
              this, &ULingTuSimRuntimeUIWorldSubsystem::HandleNextGameSelection))
          .OnSelectionConfirmed(FSimpleDelegate::CreateUObject(
              this, &ULingTuSimRuntimeUIWorldSubsystem::HandleGameSelectionConfirm))
          .OnAssetReviewPrevious(FSimpleDelegate::CreateUObject(
              this, &ULingTuSimRuntimeUIWorldSubsystem::HandlePreviousAssetReview))
          .OnAssetReviewNext(FSimpleDelegate::CreateUObject(
              this, &ULingTuSimRuntimeUIWorldSubsystem::HandleNextAssetReview));
  RuntimeHUDWidget = RuntimeHUD;
  HUDWidget = RuntimeHUD;
  Viewport->AddViewportWidgetContent(HUDWidget.ToSharedRef(), 1000);
  AttachedViewport = Viewport;
  InitializeHudScreenshotFromCommandLine();
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
  FrontEndScreenshotDriver.Initialize(
      FCommandLine::Get(), bGameSelector, FApp::IsUnattended(), FApp::CanEverRender(), HUDWidget,
      Viewport, FrontEndLoginModel, GameSelectionModel, AssetReviewModel, GameSelectionIntentPath);
#endif

  InputProcessor = MakeShared<LingTuSim::UI::FLingTuSimRuntimeUIInputProcessor>(
      ModeController.ToSharedRef(), TWeakPtr<SWidget>(HUDWidget),
      TWeakPtr<SWidget>(GameViewportWidget),
      [WeakModeController = TWeakPtr<LingTuSim::UI::FRuntimeUIModeController>(ModeController),
       WeakWorld = TWeakObjectPtr<UWorld>(&InWorld), SharedLocalState = LocalState,
       Lifecycle = MakeShared<FPublishedInputLifecycle>()](
          const LingTuSim::UI::FRobotDriveInputSnapshot &Snapshot,
          const LingTuSim::UI::ERobotDrivePublishReason Reason) {
        const TSharedPtr<LingTuSim::UI::FRuntimeUIModeController> Controller =
            WeakModeController.Pin();
        const LingTuSim::UI::ERuntimeUIMode CurrentMode =
            Controller.IsValid() ? Controller->GetMode() : LingTuSim::UI::ERuntimeUIMode::Pause;
        const uint64 SourceTimeNs = SourceMonotonicNowNs();

        SharedLocalState->bInputObserved = true;
        SharedLocalState->RequestedInput = Snapshot;
        SharedLocalState->ActualUIMode = LingTuSim::UI::RuntimeUIModeWireName(CurrentMode);

        if (UWorld *World = WeakWorld.Get()) {
          if (ULingTuSimVisualWorldSubsystem *Visual =
                  World->GetSubsystem<ULingTuSimVisualWorldSubsystem>()) {
            FString CameraLookError;
            if (!Visual->ApplyRuntimeFreeCameraLook(Snapshot.CameraYaw, Snapshot.CameraPitch,
                                                    CameraLookError) &&
                Reason != LingTuSim::UI::ERobotDrivePublishReason::Periodic) {
              UE_LOG(LogLingTuSimUI, Warning,
                     TEXT("LINGTU_RUNTIME_UI_CAMERA_LOOK_REJECTED reason=%s"), *CameraLookError);
            }
          }
        }

        LingTuSim::FOperatorIntentSample Sample;
        // Motion intent v1 is Drive-only. Mode transitions are represented by
        // zero/deadman release plus explicit lifecycle requests below.
        Sample.InputMode = TEXT("drive");
        Sample.UIMode = SharedLocalState->ActualUIMode;
        Sample.CameraMode = SharedLocalState->ActualCameraMode;
        const TCHAR *DeviceName =
            LingTuSim::UI::FRobotDriveInputState::InputDeviceName(Snapshot.InputDevice);
        Sample.InputDevice =
            FCString::Strcmp(DeviceName, TEXT("unknown")) == 0 ? TEXT("keyboard") : DeviceName;
        Sample.bViewportFocused = Snapshot.bViewportFocused;
        Sample.bDeadman = Snapshot.bDeadman;
        Sample.Forward = Snapshot.Forward;
        Sample.Left = Snapshot.Left;
        Sample.YawLeft = Snapshot.YawLeft;
        Sample.CameraYaw = Snapshot.CameraYaw;
        Sample.CameraPitch = Snapshot.CameraPitch;
        Sample.ActiveControls = Snapshot.ActiveControls;
        Sample.SourceMonotonicNs = SourceTimeNs;

        FString IntentEventId;
        FString IntentError;
        const bool bIntentPublished =
            LingTuSim::FSessionService::PublishOperatorIntent(Sample, IntentEventId, IntentError);
        if (!bIntentPublished && Reason != LingTuSim::UI::ERobotDrivePublishReason::Periodic) {
          UE_LOG(LogLingTuSimUI, Warning, TEXT("LINGTU_RUNTIME_UI_INTENT_REJECTED reason=%s"),
                 *IntentError);
        }

        const auto PublishRequest = [SourceTimeNs, Reason, SharedLocalState](
                                        const LingTuSim::EOperatorRuntimeRequestType RequestType,
                                        const TCHAR *RequestName) {
          LingTuSim::FOperatorRuntimeRequest Request;
          Request.Request = RequestType;
          Request.UIMode = SharedLocalState->ActualUIMode;
          Request.CameraMode = SharedLocalState->ActualCameraMode;
          Request.SourceMonotonicNs = SourceTimeNs;
          FString EventId;
          FString Error;
          const bool bPublished =
              LingTuSim::FSessionService::PublishRuntimeRequest(Request, EventId, Error);
          SharedLocalState->LatestRuntimeRequest = RequestName;
          SharedLocalState->LatestRuntimeRequestEventId = bPublished ? EventId : FString();
          SharedLocalState->LatestRuntimeRequestError = bPublished ? FString() : Error;
          SharedLocalState->bLatestRuntimeRequestPublished = bPublished;
          if (!bPublished && Reason != LingTuSim::UI::ERobotDrivePublishReason::Periodic) {
            UE_LOG(LogLingTuSimUI, Warning,
                   TEXT("LINGTU_RUNTIME_UI_REQUEST_REJECTED request=%s reason=%s"), RequestName,
                   *Error);
          }
          return bPublished;
        };

        if (!Lifecycle->bDeadman && Snapshot.bDeadman) {
          PublishRequest(LingTuSim::EOperatorRuntimeRequestType::ControlClaim,
                         TEXT("control_claim"));
        } else if ((Lifecycle->bDeadman && !Snapshot.bDeadman) ||
                   Reason == LingTuSim::UI::ERobotDrivePublishReason::Release) {
          PublishRequest(LingTuSim::EOperatorRuntimeRequestType::ControlRelease,
                         TEXT("control_release"));
        }

        if (CurrentMode != Lifecycle->Mode) {
          if (CurrentMode == LingTuSim::UI::ERuntimeUIMode::Pause) {
            PublishRequest(LingTuSim::EOperatorRuntimeRequestType::Pause, TEXT("pause"));
          } else if (Lifecycle->Mode == LingTuSim::UI::ERuntimeUIMode::Pause) {
            PublishRequest(LingTuSim::EOperatorRuntimeRequestType::Resume, TEXT("resume"));
          }
          PublishRequest(LingTuSim::EOperatorRuntimeRequestType::UIStateUpdate,
                         TEXT("ui_state_update"));
        }

        Lifecycle->Mode = CurrentMode;
        Lifecycle->bDeadman = Snapshot.bDeadman;
      },
      [WeakThis = TWeakObjectPtr<ULingTuSimRuntimeUIWorldSubsystem>(this)](
          const LingTuSim::UI::ERuntimeUIAction Action) {
        if (ULingTuSimRuntimeUIWorldSubsystem *Subsystem = WeakThis.Get()) {
          Subsystem->HandleUIAction(Action);
        }
      },
      [WeakThis = TWeakObjectPtr<ULingTuSimRuntimeUIWorldSubsystem>(this)]() {
        const ULingTuSimRuntimeUIWorldSubsystem *Subsystem = WeakThis.Get();
        return Subsystem != nullptr && Subsystem->bGameSelector;
      },
      [WeakHUD = TWeakPtr<LingTuSim::UI::SLingTuSimRuntimeHUD>(RuntimeHUD)]() {
        const TSharedPtr<LingTuSim::UI::SLingTuSimRuntimeHUD> HUD = WeakHUD.Pin();
        return HUD.IsValid() && HUD->IsOperatorNameEditing();
      });
  bInputProcessorRegistered = FSlateApplication::Get().RegisterInputPreProcessor(InputProcessor, 0);
  if (!bInputProcessorRegistered) {
    UE_LOG(LogLingTuSimUI, Error,
           TEXT("LINGTU_RUNTIME_UI_INPUT_UNAVAILABLE reason=registration_failed"));
    InputProcessor.Reset();
  }
  SynchronizePlayerInputMode(true);

  UE_LOG(LogLingTuSimUI, Display,
         TEXT("LINGTU_RUNTIME_UI_ATTACHED modes=drive,build,tactical,menu"));
}

void ULingTuSimRuntimeUIWorldSubsystem::Tick(const float DeltaTime) {
  (void)DeltaTime;
  check(IsInGameThread());
  if (HUDWidget.IsValid()) {
    ++HudFramesSinceAttach;
  }
  if (LocalState.IsValid() && ModeController.IsValid()) {
    LocalState->ActualUIMode = LingTuSim::UI::RuntimeUIModeWireName(ModeController->GetMode());
    if (LocalState->ActualCameraMode == TEXT("unavailable")) {
      if (ULingTuSimVisualWorldSubsystem *Visual =
              GetWorld() != nullptr ? GetWorld()->GetSubsystem<ULingTuSimVisualWorldSubsystem>()
                                    : nullptr) {
        FString CameraError;
        if (Visual->SetRuntimeCameraMode(ELingTuSimRuntimeCameraMode::Follow, CameraError)) {
          LocalState->ActualCameraMode = Visual->GetRuntimeCameraModeName();
        }
      }
    }
    if (!bInitialCameraEchoPublished && LocalState->ActualCameraMode != TEXT("unavailable")) {
      FString EchoError;
      bInitialCameraEchoPublished =
          PublishRuntimeRequest(LingTuSim::EOperatorRuntimeRequestType::UIStateUpdate,
                                TEXT("ui_state_update"), EchoError);
    }
  }
  SynchronizePlayerInputMode();
  TickPendingExit();
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
  FrontEndScreenshotDriver.Tick();
#endif
  TickHudScreenshot();
}

TStatId ULingTuSimRuntimeUIWorldSubsystem::GetStatId() const {
  RETURN_QUICK_DECLARE_CYCLE_STAT(ULingTuSimRuntimeUIWorldSubsystem, STATGROUP_Tickables);
}

void ULingTuSimRuntimeUIWorldSubsystem::Deinitialize() {
  DetachRuntimeUI();
  if (bEligibleContextRegistered) {
    UnregisterEligibleContext(this);
    bEligibleContextRegistered = false;
  }
  bHudScreenshotConfigured = false;
  bHudScreenshotComplete = false;
  bHudScreenshotRejected = false;
  bHudScreenshotTargetsValidated = false;
  bEngineExitRequested = false;
  bGameSelector = false;
  bGameSelectorExitOnConfirm = false;
  bPlayerInputModeInitialized = false;
  bInitialCameraEchoPublished = false;
  HudFramesSinceAttach = 0;
  NextHudScreenshotIndex = 0;
  ActiveHudScreenshotIndex = INDEX_NONE;
  HudScreenshotControlLogDirectory.Reset();
  PendingExitEventId.Reset();
  GameSelectionCatalogPath.Reset();
  GameSelectionIntentPath.Reset();
  HudScreenshotCaptures.Reset();
  Super::Deinitialize();
}

void ULingTuSimRuntimeUIWorldSubsystem::DetachRuntimeUI() {
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
  FrontEndScreenshotDriver.Detach();
#endif
  if (bInputProcessorRegistered && InputProcessor.IsValid() && FSlateApplication::IsInitialized()) {
    FSlateApplication::Get().UnregisterInputPreProcessor(InputProcessor);
  }
  bInputProcessorRegistered = false;
  InputProcessor.Reset();

  if (APlayerController *PlayerController =
          GetWorld() != nullptr ? GetWorld()->GetFirstPlayerController() : nullptr) {
    PlayerController->SetInputMode(FInputModeGameOnly());
    PlayerController->bShowMouseCursor = false;
  }
  bPlayerInputModeInitialized = false;

  if (HUDWidget.IsValid()) {
    if (UGameViewportClient *Viewport = AttachedViewport.Get()) {
      Viewport->RemoveViewportWidgetContent(HUDWidget.ToSharedRef());
    }
  }
  AttachedViewport.Reset();
  RuntimeHUDWidget.Reset();
  HUDWidget.Reset();
  ModeController.Reset();
  LocalState.Reset();
  FrontEndLoginModel.Reset();
  GameSelectionModel.Reset();
  AssetReviewModel.Reset();
  GameSelectionFeedback.Reset();
}

void ULingTuSimRuntimeUIWorldSubsystem::InitializeGameSelectionFromCommandLine() {
  check(GameSelectionModel.IsValid());
  bGameSelector = FParse::Param(FCommandLine::Get(), TEXT("LingTuGameSelector"));
  bGameSelectorExitOnConfirm =
      bGameSelector && FParse::Param(FCommandLine::Get(), TEXT("LingTuGameSelectorExitOnConfirm"));
  GameSelectionIntentPath.Reset();
  FParse::Value(FCommandLine::Get(), TEXT("LingTuGameSelectionIntent="), GameSelectionIntentPath);

  GameSelectionCatalogPath.Reset();
  FString CatalogPath;
  if (!FParse::Value(FCommandLine::Get(), TEXT("LingTuGameSelectionCatalog="), CatalogPath)) {
    if (bGameSelector) {
      *GameSelectionFeedback = TEXT("BLOCKED  ·  catalog path missing");
      UE_LOG(LogLingTuSimUI, Error,
             TEXT("LINGTU_GAME_SELECTION_UNAVAILABLE reason=catalog_path_missing"));
    }
    return;
  }
  GameSelectionCatalogPath = FPaths::ConvertRelativePathToFull(CatalogPath);

  LingTuSim::UI::FGameSelectionCatalog Catalog;
  FString Error;
  if (!LingTuSim::UI::FGameSelectionCatalogLoader::LoadFromFile(GameSelectionCatalogPath, Catalog,
                                                                Error)) {
    *GameSelectionFeedback = FString::Printf(TEXT("BLOCKED  ·  %s"), *Error);
    UE_LOG(LogLingTuSimUI, Error, TEXT("LINGTU_GAME_SELECTION_UNAVAILABLE catalog=%s reason=%s"),
           *GameSelectionCatalogPath, *Error);
    return;
  }
  GameSelectionModel->SetCatalog(MoveTemp(Catalog));
  GameSelectionFeedback->Reset();
  UE_LOG(LogLingTuSimUI, Display,
         TEXT("LINGTU_GAME_SELECTION_READY catalog=%s options=%d intent=%s"),
         *GameSelectionCatalogPath, GameSelectionModel->GetCatalog().Options.Num(),
         GameSelectionIntentPath.IsEmpty() ? TEXT("unconfigured") : *GameSelectionIntentPath);
}

void ULingTuSimRuntimeUIWorldSubsystem::HandleGameSelectionConfirm() {
  check(IsInGameThread());
  if (!GameSelectionModel.IsValid()) {
    return;
  }
  if (GameSelectionModel->IsConfirmed()) {
    return;
  }
  FString Blocker;
  if (bGameSelector) {
    if (!FrontEndLoginModel.IsValid()) {
      Blocker = TEXT("Local Operator login model is unavailable.");
    } else if (!FrontEndLoginModel->CanConfirmSession(Blocker)) {
      // The login model owns the user-facing blocker.
    }
    if (!Blocker.IsEmpty()) {
      *GameSelectionFeedback = FString::Printf(TEXT("BLOCKED  ·  %s"), *Blocker);
      UE_LOG(LogLingTuSimUI, Warning,
             TEXT("LINGTU_GAME_SELECTION_REJECTED reason=local_operator_gate:%s"), *Blocker);
      return;
    }
  }
  if (GameSelectionIntentPath.IsEmpty()) {
    *GameSelectionFeedback = TEXT("BLOCKED  ·  launcher intent path missing");
    UE_LOG(LogLingTuSimUI, Warning,
           TEXT("LINGTU_GAME_SELECTION_REJECTED reason=intent_path_missing"));
    return;
  }
  if (!GameSelectionModel->CanConfirm(Blocker)) {
    *GameSelectionFeedback = FString::Printf(TEXT("BLOCKED  ·  %s"), *Blocker);
    UE_LOG(LogLingTuSimUI, Warning, TEXT("LINGTU_GAME_SELECTION_REJECTED reason=%s"), *Blocker);
    return;
  }
  const LingTuSim::UI::FGameSelectionOption *Option = GameSelectionModel->GetSelectedOption();
  check(Option != nullptr);
  LingTuSim::UI::FGameSelectionCatalog RevalidatedCatalog;
  if (!LingTuSim::UI::FGameSelectionCatalogLoader::LoadFromFile(
          GameSelectionCatalogPath, RevalidatedCatalog, Blocker)) {
    *GameSelectionFeedback = FString::Printf(TEXT("BLOCKED  ·  %s"), *Blocker);
    UE_LOG(LogLingTuSimUI, Error,
           TEXT("LINGTU_GAME_SELECTION_REJECTED option=%s reason=revalidation_failed:%s"),
           *Option->Id, *Blocker);
    return;
  }
  const LingTuSim::UI::FGameSelectionOption *RevalidatedOption =
      RevalidatedCatalog.Options.FindByPredicate(
          [Option](const LingTuSim::UI::FGameSelectionOption &Candidate) {
            return Candidate.Id == Option->Id;
          });
  if (RevalidatedOption == nullptr || !SameCompiledSelectionBinding(*Option, *RevalidatedOption)) {
    Blocker = TEXT("selected compiled binding changed during confirmation");
    *GameSelectionFeedback = FString::Printf(TEXT("BLOCKED  ·  %s"), *Blocker);
    UE_LOG(LogLingTuSimUI, Error,
           TEXT("LINGTU_GAME_SELECTION_REJECTED option=%s reason=binding_changed"), *Option->Id);
    return;
  }
  if (!LingTuSim::UI::FGameSelectionIntentWriter::WriteNew(GameSelectionIntentPath,
                                                           *RevalidatedOption, Blocker)) {
    *GameSelectionFeedback = FString::Printf(TEXT("BLOCKED  ·  %s"), *Blocker);
    UE_LOG(LogLingTuSimUI, Error, TEXT("LINGTU_GAME_SELECTION_REJECTED option=%s reason=%s"),
           *Option->Id, *Blocker);
    return;
  }
  if (!GameSelectionModel->MarkConfirmed(Blocker)) {
    *GameSelectionFeedback = FString::Printf(TEXT("BLOCKED  ·  %s"), *Blocker);
    UE_LOG(LogLingTuSimUI, Error, TEXT("LINGTU_GAME_SELECTION_REJECTED option=%s reason=%s"),
           *Option->Id, *Blocker);
    return;
  }
  UE_LOG(LogLingTuSimUI, Display,
         TEXT("LINGTU_GAME_SELECTION_COMMITTED option=%s session=%s intent=%s"), *Option->Id,
         *Option->SessionId, *GameSelectionIntentPath);
  *GameSelectionFeedback = TEXT("SELECTED  ·  launcher handoff committed");
  if (HUDWidget.IsValid()) {
    HUDWidget->Invalidate(EInvalidateWidgetReason::Layout | EInvalidateWidgetReason::Paint);
  }
  if (bGameSelectorExitOnConfirm) {
    FPlatformMisc::RequestExit(false);
  }
}

void ULingTuSimRuntimeUIWorldSubsystem::HandlePreviousGameSelection() {
  if (GameSelectionModel.IsValid() && GameSelectionModel->SelectPrevious()) {
    GameSelectionFeedback->Reset();
    if (HUDWidget.IsValid()) {
      HUDWidget->Invalidate(EInvalidateWidgetReason::Layout | EInvalidateWidgetReason::Paint);
    }
  }
}

void ULingTuSimRuntimeUIWorldSubsystem::HandleNextGameSelection() {
  if (GameSelectionModel.IsValid() && GameSelectionModel->SelectNext()) {
    GameSelectionFeedback->Reset();
    if (HUDWidget.IsValid()) {
      HUDWidget->Invalidate(EInvalidateWidgetReason::Layout | EInvalidateWidgetReason::Paint);
    }
  }
}

void ULingTuSimRuntimeUIWorldSubsystem::HandlePreviousAssetReview() {
  if (AssetReviewModel.IsValid() && AssetReviewModel->SelectPrevious() && HUDWidget.IsValid()) {
    HUDWidget->Invalidate(EInvalidateWidgetReason::Layout | EInvalidateWidgetReason::Paint);
  }
}

void ULingTuSimRuntimeUIWorldSubsystem::HandleNextAssetReview() {
  if (AssetReviewModel.IsValid() && AssetReviewModel->SelectNext() && HUDWidget.IsValid()) {
    HUDWidget->Invalidate(EInvalidateWidgetReason::Layout | EInvalidateWidgetReason::Paint);
  }
}

void ULingTuSimRuntimeUIWorldSubsystem::SynchronizePlayerInputMode(const bool bForce) {
  if (!ModeController.IsValid() || !HUDWidget.IsValid() || GetWorld() == nullptr) {
    return;
  }
  const LingTuSim::UI::ERuntimeUIMode CurrentMode = ModeController->GetMode();
  if (!bForce && bPlayerInputModeInitialized && AppliedPlayerInputMode == CurrentMode) {
    return;
  }
  APlayerController *PlayerController = GetWorld()->GetFirstPlayerController();
  if (PlayerController == nullptr) {
    return;
  }
  if (CurrentMode == LingTuSim::UI::ERuntimeUIMode::Pause) {
    FInputModeGameAndUI InputMode;
    InputMode.SetWidgetToFocus(HUDWidget);
    InputMode.SetHideCursorDuringCapture(false);
    InputMode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);
    PlayerController->SetInputMode(InputMode);
    PlayerController->bShowMouseCursor = true;
    FSlateApplication::Get().SetKeyboardFocus(HUDWidget, EFocusCause::SetDirectly);
  } else {
    PlayerController->SetInputMode(FInputModeGameOnly());
    PlayerController->bShowMouseCursor = false;
  }
  AppliedPlayerInputMode = CurrentMode;
  bPlayerInputModeInitialized = true;
}

bool ULingTuSimRuntimeUIWorldSubsystem::PublishRuntimeRequest(
    const LingTuSim::EOperatorRuntimeRequestType RequestType, const TCHAR *RequestName,
    FString &OutError) {
  check(IsInGameThread());
  OutError.Reset();
  if (!LocalState.IsValid() || !ModeController.IsValid()) {
    OutError = TEXT("runtime_ui_state_unavailable");
    return false;
  }
  LingTuSim::FOperatorRuntimeRequest Request;
  Request.Request = RequestType;
  Request.UIMode = LingTuSim::UI::RuntimeUIModeWireName(ModeController->GetMode());
  Request.CameraMode = LocalState->ActualCameraMode;
  Request.SourceMonotonicNs = SourceMonotonicNowNs();
  FString EventId;
  const bool bPublished =
      LingTuSim::FSessionService::PublishRuntimeRequest(Request, EventId, OutError);
  LocalState->LatestRuntimeRequest = RequestName;
  LocalState->LatestRuntimeRequestEventId = bPublished ? EventId : FString();
  LocalState->LatestRuntimeRequestError = bPublished ? FString() : OutError;
  LocalState->bLatestRuntimeRequestPublished = bPublished;
  if (!bPublished) {
    UE_LOG(LogLingTuSimUI, Warning, TEXT("LINGTU_RUNTIME_UI_REQUEST_REJECTED request=%s reason=%s"),
           RequestName, *OutError);
  }
  return bPublished;
}

void ULingTuSimRuntimeUIWorldSubsystem::HandleUIAction(
    const LingTuSim::UI::ERuntimeUIAction Action) {
  check(IsInGameThread());
  if (!LocalState.IsValid() || !ModeController.IsValid()) {
    return;
  }
  if (bGameSelector) {
    if (!FrontEndLoginModel.IsValid() || !FrontEndLoginModel->IsLoggedIn()) {
      return;
    }
    const bool bAdmissibleFrontEndAction =
        Action == LingTuSim::UI::ERuntimeUIAction::SelectPreviousGame ||
        Action == LingTuSim::UI::ERuntimeUIAction::SelectNextGame ||
        Action == LingTuSim::UI::ERuntimeUIAction::SelectPreviousAsset ||
        Action == LingTuSim::UI::ERuntimeUIAction::SelectNextAsset ||
        Action == LingTuSim::UI::ERuntimeUIAction::ConfirmGameSelection ||
        Action == LingTuSim::UI::ERuntimeUIAction::ReturnFromFrontEnd;
    if (!bAdmissibleFrontEndAction) {
      return;
    }
  }
  FString Error;
  switch (Action) {
    case LingTuSim::UI::ERuntimeUIAction::SelectPreviousGame:
      if (ModeController->GetMode() == LingTuSim::UI::ERuntimeUIMode::Pause) {
        HandlePreviousGameSelection();
      }
      return;
    case LingTuSim::UI::ERuntimeUIAction::SelectNextGame:
      if (ModeController->GetMode() == LingTuSim::UI::ERuntimeUIMode::Pause) {
        HandleNextGameSelection();
      }
      return;
    case LingTuSim::UI::ERuntimeUIAction::SelectPreviousAsset:
      if (ModeController->GetMode() == LingTuSim::UI::ERuntimeUIMode::Pause) {
        HandlePreviousAssetReview();
      }
      return;
    case LingTuSim::UI::ERuntimeUIAction::SelectNextAsset:
      if (ModeController->GetMode() == LingTuSim::UI::ERuntimeUIMode::Pause) {
        HandleNextAssetReview();
      }
      return;
    case LingTuSim::UI::ERuntimeUIAction::ConfirmGameSelection:
      if (ModeController->GetMode() == LingTuSim::UI::ERuntimeUIMode::Pause) {
        HandleGameSelectionConfirm();
      }
      return;
    case LingTuSim::UI::ERuntimeUIAction::ReturnFromFrontEnd:
      if (!bGameSelector || !FrontEndLoginModel.IsValid() ||
          ModeController->GetMode() != LingTuSim::UI::ERuntimeUIMode::Pause) {
        return;
      }
      if (!FrontEndLoginModel->Logout(Error)) {
        if (GameSelectionFeedback.IsValid()) {
          *GameSelectionFeedback = FString::Printf(TEXT("BLOCKED  ·  %s"), *Error);
        }
        return;
      }
      if (GameSelectionFeedback.IsValid()) {
        GameSelectionFeedback->Reset();
      }
      if (RuntimeHUDWidget.IsValid()) {
        RuntimeHUDWidget->FocusFrontEndLoginCTA();
      }
      return;
    case LingTuSim::UI::ERuntimeUIAction::CycleCamera: {
      if (ModeController->GetMode() == LingTuSim::UI::ERuntimeUIMode::Pause) {
        return;
      }
      ULingTuSimVisualWorldSubsystem *Visual =
          GetWorld() != nullptr ? GetWorld()->GetSubsystem<ULingTuSimVisualWorldSubsystem>()
                                : nullptr;
      FString ActualMode;
      if (Visual == nullptr || !Visual->CycleRuntimeCameraMode(ActualMode, Error)) {
        LocalState->LatestRuntimeRequest = TEXT("ui_state_update");
        LocalState->LatestRuntimeRequestEventId.Reset();
        LocalState->LatestRuntimeRequestError =
            Error.IsEmpty() ? TEXT("runtime_camera_cycle_unavailable") : Error;
        LocalState->bLatestRuntimeRequestPublished = false;
        return;
      }
      LocalState->ActualCameraMode = ActualMode;
      bInitialCameraEchoPublished = PublishRuntimeRequest(
          LingTuSim::EOperatorRuntimeRequestType::UIStateUpdate, TEXT("ui_state_update"), Error);
      return;
    }
    case LingTuSim::UI::ERuntimeUIAction::ToggleRecording: {
      const LingTuSim::UI::FRuntimeUIStatusSnapshot Status =
          LingTuSim::UI::FRuntimeUIStatusReader::Read(GetWorld(), LocalState.Get());
      if (Status.CanRequestRecordStart()) {
        PublishRuntimeRequest(LingTuSim::EOperatorRuntimeRequestType::RecordStart,
                              TEXT("record_start"), Error);
        return;
      }
      if (Status.CanRequestRecordStopCommit()) {
        PublishRuntimeRequest(LingTuSim::EOperatorRuntimeRequestType::RecordStopCommit,
                              TEXT("record_stop_commit"), Error);
        return;
      }
      LocalState->LatestRuntimeRequest = TEXT("record_toggle");
      LocalState->LatestRuntimeRequestEventId.Reset();
      LocalState->LatestRuntimeRequestError =
          Status.FullStatusBlocker.IsEmpty() ? TEXT("authoritative_recording_state_not_actionable")
                                             : Status.FullStatusBlocker;
      LocalState->bLatestRuntimeRequestPublished = false;
      return;
    }
    case LingTuSim::UI::ERuntimeUIAction::Exit:
      if (ModeController->GetMode() != LingTuSim::UI::ERuntimeUIMode::Pause) {
        return;
      }
      if (PublishRuntimeRequest(LingTuSim::EOperatorRuntimeRequestType::Exit, TEXT("exit"),
                                Error)) {
        PendingExitEventId = LocalState->LatestRuntimeRequestEventId;
        bEngineExitRequested = false;
      }
      return;
    default:
      return;
  }
}

void ULingTuSimRuntimeUIWorldSubsystem::TickPendingExit() {
  if (PendingExitEventId.IsEmpty() || bEngineExitRequested || !LocalState.IsValid()) {
    return;
  }
  const LingTuSim::UI::FRuntimeUIStatusSnapshot Status =
      LingTuSim::UI::FRuntimeUIStatusReader::Read(GetWorld(), LocalState.Get());
  FString Blocker;
  if (!LingTuSim::UI::FRuntimeUIExitPolicy::CanRequestEngineExit(
          PendingExitEventId, bEngineExitRequested, Status, Blocker)) {
    return;
  }
  bEngineExitRequested = true;
  UE_LOG(LogLingTuSimUI, Display,
         TEXT("LINGTU_RUNTIME_UI_EXIT_CONFIRMED event_id=%s source_epoch=%llu source_sequence=%llu "
              "server_status_sequence=%llu runtime=STOPPED safe_stop=ZEROED admitted_zero=true "
              "request_exit=once"),
         *PendingExitEventId, static_cast<unsigned long long>(Status.FullStatus.SourceEpoch),
         static_cast<unsigned long long>(Status.FullStatus.SourceSequence),
         static_cast<unsigned long long>(Status.FullStatus.ServerStatusSequence));
  FPlatformMisc::RequestExit(false);
}

void ULingTuSimRuntimeUIWorldSubsystem::InitializeHudScreenshotFromCommandLine() {
  HudScreenshotCaptures.Reset();
  HudScreenshotControlLogDirectory.Reset();
  bHudScreenshotConfigured = false;
  bHudScreenshotComplete = false;
  bHudScreenshotRejected = false;
  bHudScreenshotTargetsValidated = false;
  HudFramesSinceAttach = 0;
  NextHudScreenshotIndex = 0;
  ActiveHudScreenshotIndex = INDEX_NONE;

  TArray<LingTuSim::UI::FHudScreenshotTarget> Targets;
  FString ParseError;
  if (!LingTuSim::UI::FHudScreenshotContract::ParseCommandLine(
          FCommandLine::Get(), Targets, bHudScreenshotConfigured, ParseError)) {
    RejectHudScreenshots(*ParseError);
    return;
  }
  if (!bHudScreenshotConfigured) {
    return;
  }
  HudScreenshotCaptures.Reserve(Targets.Num());
  for (LingTuSim::UI::FHudScreenshotTarget &Target : Targets) {
    FLingTuSimHudScreenshotRuntimeState Capture;
    Capture.Target = MoveTemp(Target);
    HudScreenshotCaptures.Add(MoveTemp(Capture));
  }
}

void ULingTuSimRuntimeUIWorldSubsystem::TickHudScreenshot() {
  if (!bHudScreenshotConfigured || bHudScreenshotComplete || bHudScreenshotRejected) {
    return;
  }

  LingTuSim::FControlTransportBinding ControlBinding;
  if (!LingTuSim::FSessionService::GetControlTransportBinding(ControlBinding)) {
    return;
  }
  if (!bHudScreenshotTargetsValidated) {
    HudScreenshotControlLogDirectory = ControlBinding.LogDirectory;
    for (const FLingTuSimHudScreenshotRuntimeState &Capture : HudScreenshotCaptures) {
      FString ValidationError;
      if (!LingTuSim::UI::FHudScreenshotContract::ValidateHudScreenshotTarget(
              Capture.Target, HudScreenshotControlLogDirectory,
              LingTuSim::UI::EHudScreenshotValidationPhase::BeforeRequest, ValidationError)) {
        RejectHudScreenshots(*ValidationError, Capture.Target.ScreenshotPath);
        return;
      }
    }
    bHudScreenshotTargetsValidated = true;
  }

  if (ActiveHudScreenshotIndex != INDEX_NONE) {
    if (ActiveHudScreenshotIndex != NextHudScreenshotIndex ||
        !HudScreenshotCaptures.IsValidIndex(ActiveHudScreenshotIndex)) {
      RejectHudScreenshots(TEXT("active_capture_index_invalid"));
      return;
    }
    FLingTuSimHudScreenshotRuntimeState &Capture = HudScreenshotCaptures[ActiveHudScreenshotIndex];
    const int64 ScreenshotBytes = IFileManager::Get().FileSize(*Capture.Target.ScreenshotPath);
    if (ScreenshotBytes <= 0) {
      return;
    }
    FString ValidationError;
    if (!LingTuSim::UI::FHudScreenshotContract::ValidateHudScreenshotTarget(
            Capture.Target, HudScreenshotControlLogDirectory,
            LingTuSim::UI::EHudScreenshotValidationPhase::ScreenshotWritten, ValidationError)) {
      RejectHudScreenshots(*ValidationError, Capture.Target.ScreenshotPath);
      return;
    }
    int32 ScreenshotWidth = 0;
    int32 ScreenshotHeight = 0;
    if (!LingTuSim::UI::FHudScreenshotContract::ReadPngDimensions(
            Capture.Target.ScreenshotPath, ScreenshotWidth, ScreenshotHeight, ValidationError) ||
        ScreenshotWidth != 1920 || ScreenshotHeight != 1080) {
      RejectHudScreenshots(ValidationError.IsEmpty() ? TEXT("screenshot_media_not_exact_1920x1080")
                                                     : *ValidationError,
                           Capture.Target.ScreenshotPath);
      return;
    }
    if (!ModeController.IsValid() || !LocalState.IsValid() ||
        !LingTuSim::UI::FHudScreenshotContract::IsNextCaptureMode(
            Capture.Target, NextHudScreenshotIndex, ModeController->GetMode())) {
      RejectHudScreenshots(TEXT("ui_mode_changed_before_screenshot_commit"),
                           Capture.Target.ScreenshotPath);
      return;
    }
    const LingTuSim::UI::FRuntimeUIStatusSnapshot CommitStatus =
        LingTuSim::UI::FRuntimeUIStatusReader::Read(GetWorld(), LocalState.Get());
    if (!IsCaptureReadyForTarget(Capture.Target, CommitStatus)) {
      RejectHudScreenshots(TEXT("authoritative_status_changed_before_screenshot_commit"),
                           Capture.Target.ScreenshotPath);
      return;
    }
    FString EvidenceError;
    if (!WriteHudScreenshotEvidence(Capture, ScreenshotBytes, ScreenshotWidth, ScreenshotHeight,
                                    EvidenceError)) {
      RejectHudScreenshots(*EvidenceError, Capture.Target.ScreenshotPath);
      return;
    }
    Capture.bComplete = true;
    UE_LOG(LogLingTuSimUI, Display,
           TEXT("LINGTU_RUNTIME_UI_HUD_SCREENSHOT_COMPLETE path=%s evidence=%s bytes=%lld mode=%s"),
           *Capture.Target.ScreenshotPath, *Capture.Target.EvidencePath,
           static_cast<long long>(ScreenshotBytes), *Capture.Target.ModeName);
    ActiveHudScreenshotIndex = INDEX_NONE;
    ++NextHudScreenshotIndex;
    bHudScreenshotComplete =
        HudScreenshotCaptures.Num() == 3 && NextHudScreenshotIndex == HudScreenshotCaptures.Num();
    if (bHudScreenshotComplete) {
      UE_LOG(LogLingTuSimUI, Display, TEXT("LINGTU_RUNTIME_UI_HUD_SCREENSHOTS_COMPLETE count=3"));
    }
    return;
  }

  if (HudFramesSinceAttach < 2 || !HUDWidget.IsValid() || !AttachedViewport.IsValid() ||
      !bInputProcessorRegistered || !ModeController.IsValid() || !LocalState.IsValid() ||
      FScreenshotRequest::IsScreenshotRequested()) {
    return;
  }

  UGameViewportClient *Viewport = AttachedViewport.Get();
  if (Viewport == nullptr || Viewport->Viewport == nullptr ||
      Viewport->Viewport->GetSizeXY() != FIntPoint(1920, 1080)) {
    return;
  }

  const LingTuSim::UI::FRuntimeUIStatusSnapshot Status =
      LingTuSim::UI::FRuntimeUIStatusReader::Read(GetWorld(), LocalState.Get());
  if (!HudScreenshotCaptures.IsValidIndex(NextHudScreenshotIndex)) {
    RejectHudScreenshots(TEXT("next_capture_index_invalid"));
    return;
  }
  FLingTuSimHudScreenshotRuntimeState &Capture = HudScreenshotCaptures[NextHudScreenshotIndex];
  if (Capture.bComplete || Capture.bRequested) {
    RejectHudScreenshots(TEXT("next_capture_state_invalid"), Capture.Target.ScreenshotPath);
    return;
  }
  if (!LingTuSim::UI::FHudScreenshotContract::IsNextCaptureMode(
          Capture.Target, NextHudScreenshotIndex, ModeController->GetMode()) ||
      !IsCaptureReadyForTarget(Capture.Target, Status)) {
    Capture.ConsecutiveReadyFrames = 0;
    return;
  }
  ++Capture.ConsecutiveReadyFrames;
  if (Capture.ConsecutiveReadyFrames < 2) {
    return;
  }
  FString ValidationError;
  if (!LingTuSim::UI::FHudScreenshotContract::ValidateHudScreenshotTarget(
          Capture.Target, HudScreenshotControlLogDirectory,
          LingTuSim::UI::EHudScreenshotValidationPhase::BeforeRequest, ValidationError)) {
    RejectHudScreenshots(*ValidationError, Capture.Target.ScreenshotPath);
    return;
  }

  Capture.StatusAtCaptureRequest = Status;
  Capture.CapturedMonotonicNs = CaptureMonotonicNowNs();
  FScreenshotRequest::RequestScreenshot(Capture.Target.ScreenshotPath, true, false);
  Capture.bRequested = true;
  ActiveHudScreenshotIndex = NextHudScreenshotIndex;
  if (!FScreenshotRequest::IsScreenshotRequested()) {
    RejectHudScreenshots(TEXT("screenshot_request_rejected"), Capture.Target.ScreenshotPath);
    return;
  }
  UE_LOG(LogLingTuSimUI, Display,
         TEXT("LINGTU_RUNTIME_UI_HUD_SCREENSHOT_REQUESTED path=%s mode=%s camera=%s show_ui=true "
              "server_status_sequence=%llu"),
         *Capture.Target.ScreenshotPath, *Capture.Target.ModeName, *Status.ActualCameraMode,
         static_cast<unsigned long long>(Status.FullStatus.ServerStatusSequence));
}

void ULingTuSimRuntimeUIWorldSubsystem::RejectHudScreenshots(const TCHAR *Reason,
                                                             const FString &TargetPath) {
  bHudScreenshotRejected = true;
  bHudScreenshotConfigured = false;
  ActiveHudScreenshotIndex = INDEX_NONE;
  UE_LOG(LogLingTuSimUI, Error, TEXT("LINGTU_RUNTIME_UI_HUD_SCREENSHOT_REJECTED path=%s reason=%s"),
         TargetPath.IsEmpty() ? TEXT("unavailable") : *TargetPath,
         Reason == nullptr || Reason[0] == 0 ? TEXT("unknown") : Reason);
}

bool ULingTuSimRuntimeUIWorldSubsystem::WriteHudScreenshotEvidence(
    const FLingTuSimHudScreenshotRuntimeState &Capture, const int64 ScreenshotBytes,
    const int32 ScreenshotWidth, const int32 ScreenshotHeight, FString &OutError) const {
  OutError.Reset();
  if (!LingTuSim::UI::FHudScreenshotContract::ValidateHudScreenshotTarget(
          Capture.Target, HudScreenshotControlLogDirectory,
          LingTuSim::UI::EHudScreenshotValidationPhase::ScreenshotWritten, OutError)) {
    return false;
  }

  FString Evidence;
  if (!LingTuSim::UI::FHudScreenshotContract::SerializeEvidenceJson(
          Capture.Target, Capture.StatusAtCaptureRequest, Capture.CapturedMonotonicNs,
          ScreenshotBytes, ScreenshotWidth, ScreenshotHeight, Evidence, OutError)) {
    return false;
  }
  const FString TempPath = Capture.Target.EvidencePath + TEXT(".tmp");
  if (!LingTuSim::UI::FHudScreenshotContract::WriteEvidenceTempFileNoFollow(Capture.Target,
                                                                            Evidence, OutError)) {
    return false;
  }
  if (!IFileManager::Get().Move(*Capture.Target.EvidencePath, *TempPath, false, false)) {
    IFileManager::Get().Delete(*TempPath, false, true, true);
    OutError = TEXT("evidence_commit_failed");
    return false;
  }
  return LingTuSim::UI::FHudScreenshotContract::ValidateHudScreenshotTarget(
      Capture.Target, HudScreenshotControlLogDirectory,
      LingTuSim::UI::EHudScreenshotValidationPhase::EvidenceCommitted, OutError);
}

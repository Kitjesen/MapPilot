#pragma once

#include "LingTuSimFrontEndLogin.h"
#include "LingTuSimGameSelection.h"
#include "LingTuSimHudScreenshotContract.h"
#include "LingTuSimInspectionProjection.h"
#include "LingTuSimRuntimeUIModel.h"
#include "LingTuSimRuntimeUIStatus.h"
#include "Subsystems/WorldSubsystem.h"
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "LingTuSimFrontEndScreenshotDriver.h"
#endif
#include "LingTuSimRuntimeUIWorldSubsystem.generated.h"

class IInputProcessor;
class IHttpRequest;
class SWidget;
class UGameViewportClient;

namespace LingTuSim::UI {
class FRuntimeUIModeController;
class SLingTuSimRuntimeHUD;
}  // namespace LingTuSim::UI

struct FLingTuSimHudScreenshotRuntimeState final {
  LingTuSim::UI::FHudScreenshotTarget Target;
  LingTuSim::UI::FRuntimeUIStatusSnapshot StatusAtCaptureRequest;
  uint64 CapturedMonotonicNs = 0;
  int32 ConsecutiveReadyFrames = 0;
  bool bRequested = false;
  bool bComplete = false;
};

/** Owns the Slate overlay for exactly one interactive game world. */
UCLASS()
class ULingTuSimRuntimeUIWorldSubsystem final : public UTickableWorldSubsystem {
  GENERATED_BODY()

 public:
  virtual bool ShouldCreateSubsystem(UObject *Outer) const override;
  virtual void OnWorldBeginPlay(UWorld &InWorld) override;
  virtual void Tick(float DeltaTime) override;
  virtual TStatId GetStatId() const override;
  virtual void Deinitialize() override;

  /** Copies the centralized read-only status together with this world's local UI state. */
  LingTuSim::UI::FRuntimeUIStatusSnapshot ReadStatusSnapshot() const;

 private:
  void DetachRuntimeUI();
  void InitializeGameSelectionFromCommandLine();
  void InitializeInspectionProjectionFromCommandLine();
  void TickInspectionProjection(float DeltaTime);
  void RequestInspectionTask();
  void RequestInspectionReport();
  void HandleInspectionTaskResponse(bool bTransportOk, int32 ResponseCode, FString Body);
  void HandleInspectionReportResponse(bool bTransportOk, int32 ResponseCode, FString Body);
  void HandlePreviousGameSelection();
  void HandleNextGameSelection();
  void HandleGameSelectionConfirm();
  void HandlePreviousAssetReview();
  void HandleNextAssetReview();
  void SynchronizePlayerInputMode(bool bForce = false);
  void HandleUIAction(LingTuSim::UI::ERuntimeUIAction Action);
  void TickPendingExit();
  bool PublishRuntimeRequest(LingTuSim::EOperatorRuntimeRequestType RequestType,
                             const TCHAR *RequestName, FString &OutError);
  void InitializeHudScreenshotFromCommandLine();
  void TickHudScreenshot();
  void RejectHudScreenshots(const TCHAR *Reason, const FString &TargetPath = FString());
  bool WriteHudScreenshotEvidence(const FLingTuSimHudScreenshotRuntimeState &Capture,
                                  int64 ScreenshotBytes, int32 ScreenshotWidth,
                                  int32 ScreenshotHeight, FString &OutError) const;

  TSharedPtr<LingTuSim::UI::FRuntimeUIModeController> ModeController;
  TSharedPtr<LingTuSim::UI::FRuntimeUILocalState> LocalState;
  TSharedPtr<LingTuSim::UI::FFrontEndLoginModel> FrontEndLoginModel;
  TSharedPtr<LingTuSim::UI::FGameSelectionModel> GameSelectionModel;
  TSharedPtr<LingTuSim::UI::FAssetReviewModel> AssetReviewModel;
  TSharedPtr<LingTuSim::UI::FInspectionProjection> InspectionProjection;
  TSharedPtr<FString> GameSelectionFeedback;
  TSharedPtr<LingTuSim::UI::SLingTuSimRuntimeHUD> RuntimeHUDWidget;
  TSharedPtr<SWidget> HUDWidget;
  TSharedPtr<IInputProcessor> InputProcessor;
  TSharedPtr<IHttpRequest> InspectionRequest;
  TWeakObjectPtr<UGameViewportClient> AttachedViewport;
  bool bInputProcessorRegistered = false;
  bool bEligibleContextRegistered = false;
  bool bInitialCameraEchoPublished = false;
  bool bHudScreenshotConfigured = false;
  bool bHudScreenshotComplete = false;
  bool bHudScreenshotRejected = false;
  bool bHudScreenshotTargetsValidated = false;
  bool bEngineExitRequested = false;
  bool bGameSelector = false;
  bool bGameSelectorExitOnConfirm = false;
  bool bPlayerInputModeInitialized = false;
  bool bInspectionRequestInFlight = false;
  LingTuSim::UI::ERuntimeUIMode AppliedPlayerInputMode = LingTuSim::UI::ERuntimeUIMode::Drive;
  int32 HudFramesSinceAttach = 0;
  int32 NextHudScreenshotIndex = 0;
  int32 ActiveHudScreenshotIndex = INDEX_NONE;
  FString HudScreenshotControlLogDirectory;
  FString PendingExitEventId;
  FString GameSelectionCatalogPath;
  FString GameSelectionIntentPath;
  FString InspectionGatewayUrl;
  FString InspectionTaskId;
  LingTuSim::UI::FInspectionExpectedBinding InspectionExpectedBinding;
  float InspectionPollElapsedSeconds = 0.0F;
  double LastSuccessfulInspectionTaskResponseSeconds = 0.0;
  TArray<FLingTuSimHudScreenshotRuntimeState> HudScreenshotCaptures;
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
  LingTuSim::UI::FFrontEndScreenshotDriver FrontEndScreenshotDriver;
#endif
};

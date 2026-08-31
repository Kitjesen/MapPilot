#pragma once

#include "CoreMinimal.h"

#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING

class SWidget;
class UGameViewportClient;

namespace LingTuSim::UI {
class FAssetReviewModel;
class FFrontEndLoginModel;
class FGameSelectionModel;

struct FFrontEndScreenshotPaths final {
  FString RunDirectory;
  FString LoginScreenshot;
  FString AssetLibraryScreenshot;
  FString SuccessSentinel;
  FString ErrorSentinel;
};

/** Development-only, unattended visual QA driver for the selector front end. */
class FFrontEndScreenshotDriver final {
 public:
  static bool ParseCommandLine(const FString &CommandLine, FString &OutRunId, bool &bOutConfigured,
                               FString &OutError);
  static bool IsSafeRunId(const FString &RunId);
  static FFrontEndScreenshotPaths BuildOutputPaths(const FString &RunId);
  static bool ValidateStartPolicy(bool bGameSelector, bool bUnattended, bool bCanEverRender,
                                  bool bNullRHI, FString &OutError);

  void Initialize(const FString &CommandLine, bool bGameSelector, bool bUnattended,
                  bool bCanEverRender, const TSharedPtr<SWidget> &HUDWidget,
                  UGameViewportClient *Viewport, const TSharedPtr<FFrontEndLoginModel> &LoginModel,
                  const TSharedPtr<FGameSelectionModel> &SelectionModel,
                  const TSharedPtr<FAssetReviewModel> &AssetReviewModel,
                  const FString &SelectionIntentPath);
  void Tick();
  void Detach();

 private:
  enum class EState : uint8 {
    Disabled,
    AwaitLoginStable,
    AwaitLoginFile,
    AwaitAssetLibraryStable,
    AwaitAssetLibraryFile,
    Complete,
    Failed,
  };

  bool IsViewportReady() const;
  bool ValidateScreenshot(const FString &Path, int64 &OutBytes, FString &OutError) const;
  bool ValidateInvariantState(FString &OutError) const;
  bool RequestCapture(const FString &Path, EState AwaitFileState);
  bool WriteSuccess(FString &OutError) const;
  void Fail(const TCHAR *Reason);
  void ExitWithStatus(uint8 Status);

  EState State = EState::Disabled;
  FFrontEndScreenshotPaths Paths;
  FString RunId;
  FString SelectionIntentPath;
  TWeakPtr<SWidget> HUDWidget;
  TWeakObjectPtr<UGameViewportClient> Viewport;
  TWeakPtr<FFrontEndLoginModel> LoginModel;
  TWeakPtr<FGameSelectionModel> SelectionModel;
  TWeakPtr<FAssetReviewModel> AssetReviewModel;
  double StartedSeconds = 0.0;
  int32 ConsecutiveStableFrames = 0;
  int64 LoginScreenshotBytes = 0;
  int64 AssetLibraryScreenshotBytes = 0;
  bool bExitRequested = false;
};
}  // namespace LingTuSim::UI

#endif

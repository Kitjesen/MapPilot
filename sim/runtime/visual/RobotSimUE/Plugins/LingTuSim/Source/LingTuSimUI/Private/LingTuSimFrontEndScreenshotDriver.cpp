#include "LingTuSimFrontEndScreenshotDriver.h"

#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING

#include "Engine/GameViewportClient.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformMisc.h"
#include "HAL/PlatformTime.h"
#include "LingTuSimAssetReview.h"
#include "LingTuSimFrontEndLogin.h"
#include "LingTuSimGameSelection.h"
#include "LingTuSimHudScreenshotContract.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "UnrealClient.h"
#include "Widgets/SWidget.h"
#include "SLingTuSimRuntimeHUD.h"

DEFINE_LOG_CATEGORY_STATIC(LogLingTuSimFrontEndScreenshot, Log, All);

namespace LingTuSim::UI {
namespace {
constexpr double ScreenshotTimeoutSeconds = 120.0;
// Allow the presentation-only capture's temporal history to converge before QA capture.
constexpr int32 StableFrameCount = 48;

bool DecodeAssignmentValue(const FString &RawValue, FString &OutValue) {
  OutValue = RawValue;
  const bool bLeadingQuote = OutValue.StartsWith(TEXT("\""));
  const bool bTrailingQuote = OutValue.EndsWith(TEXT("\""));
  if (bLeadingQuote != bTrailingQuote || (bLeadingQuote && OutValue.Len() < 2)) {
    return false;
  }
  if (bLeadingQuote) {
    OutValue = OutValue.Mid(1, OutValue.Len() - 2);
  }
  return !OutValue.Contains(TEXT("\""));
}

bool FileDoesNotExist(const FString &Path) {
  return !Path.IsEmpty() && !IFileManager::Get().FileExists(*Path) &&
         !IFileManager::Get().DirectoryExists(*Path);
}

bool IsAsciiAlphaNumeric(const TCHAR Character) {
  return (Character >= TEXT('A') && Character <= TEXT('Z')) ||
         (Character >= TEXT('a') && Character <= TEXT('z')) ||
         (Character >= TEXT('0') && Character <= TEXT('9'));
}
}  // namespace

bool FFrontEndScreenshotDriver::ParseCommandLine(const FString &CommandLine, FString &OutRunId,
                                                 bool &bOutConfigured, FString &OutError) {
  OutRunId.Reset();
  bOutConfigured = false;
  OutError.Reset();
  TArray<FString> Tokens;
  TArray<FString> Switches;
  FCommandLine::Parse(*CommandLine, Tokens, Switches);
  const FString Prefix = TEXT("LingTuDevFrontEndScreenshots=");
  TArray<FString> Values;
  for (const FString &Switch : Switches) {
    if (Switch.StartsWith(Prefix, ESearchCase::CaseSensitive)) {
      FString Value;
      if (!DecodeAssignmentValue(Switch.Mid(Prefix.Len()), Value)) {
        OutError = TEXT("front_end_screenshot_argument_quotes_invalid");
        return false;
      }
      Values.Add(MoveTemp(Value));
    }
  }
  if (Values.IsEmpty()) {
    return true;
  }
  bOutConfigured = true;
  if (Values.Num() != 1) {
    OutError = TEXT("front_end_screenshot_argument_must_appear_exactly_once");
    return false;
  }
  OutRunId = MoveTemp(Values[0]);
  if (!IsSafeRunId(OutRunId)) {
    OutError = TEXT("front_end_screenshot_run_id_invalid");
    return false;
  }
  return true;
}

bool FFrontEndScreenshotDriver::IsSafeRunId(const FString &RunId) {
  if (RunId.IsEmpty() || RunId.Len() > 128 || !IsAsciiAlphaNumeric(RunId[0])) {
    return false;
  }
  for (const TCHAR Character : RunId) {
    if (!IsAsciiAlphaNumeric(Character) && Character != TEXT('_') && Character != TEXT('.') &&
        Character != TEXT('-')) {
      return false;
    }
  }
  return true;
}

FFrontEndScreenshotPaths FFrontEndScreenshotDriver::BuildOutputPaths(const FString &RunId) {
  FFrontEndScreenshotPaths Result;
  if (!IsSafeRunId(RunId)) {
    return Result;
  }
  Result.RunDirectory = FPaths::ConvertRelativePathToFull(FPaths::Combine(
      FPaths::ProjectSavedDir(), TEXT("Automation"), TEXT("LingTuSimUI"), TEXT("FrontEnd"), RunId));
  FPaths::NormalizeDirectoryName(Result.RunDirectory);
  Result.LoginScreenshot = FPaths::Combine(Result.RunDirectory, TEXT("login.png"));
  Result.AssetLibraryScreenshot = FPaths::Combine(Result.RunDirectory, TEXT("asset-library.png"));
  Result.CategoryScreenshot = FPaths::Combine(Result.RunDirectory, TEXT("category-humanoid.png"));
  Result.WorldScreenshot = FPaths::Combine(Result.RunDirectory, TEXT("map-selection.png"));
  Result.SuccessSentinel = FPaths::Combine(Result.RunDirectory, TEXT("success.json"));
  Result.ErrorSentinel = FPaths::Combine(Result.RunDirectory, TEXT("error.txt"));
  return Result;
}

bool FFrontEndScreenshotDriver::ValidateStartPolicy(const bool bGameSelector,
                                                    const bool bUnattended,
                                                    const bool bCanEverRender, const bool bNullRHI,
                                                    FString &OutError) {
  OutError.Reset();
  if (!bGameSelector) {
    OutError = TEXT("game_selector_required");
    return false;
  }
  if (!bUnattended) {
    OutError = TEXT("unattended_required");
    return false;
  }
  if (!bCanEverRender || bNullRHI) {
    OutError = TEXT("real_rhi_required_nullrhi_rejected");
    return false;
  }
  return true;
}

void FFrontEndScreenshotDriver::Initialize(const FString &CommandLine, const bool bGameSelector,
                                           const bool bUnattended, const bool bCanEverRender,
                                           const TSharedPtr<SWidget> &InHUDWidget,
                                           UGameViewportClient *InViewport,
                                           const TSharedPtr<FFrontEndLoginModel> &InLoginModel,
                                           const TSharedPtr<FGameSelectionModel> &InSelectionModel,
                                           const TSharedPtr<FAssetReviewModel> &InAssetReviewModel,
                                           const FString &InSelectionIntentPath) {
  Detach();
  FString ParseError;
  bool bConfigured = false;
  if (!ParseCommandLine(CommandLine, RunId, bConfigured, ParseError)) {
    UE_LOG(LogLingTuSimFrontEndScreenshot, Error,
           TEXT("LINGTU_FRONT_END_SCREENSHOT_REJECTED reason=%s"), *ParseError);
    ExitWithStatus(2);
    State = EState::Failed;
    return;
  }
  if (!bConfigured) {
    return;
  }

  Paths = BuildOutputPaths(RunId);
  if (IFileManager::Get().DirectoryExists(*Paths.RunDirectory) ||
      IFileManager::Get().FileExists(*Paths.RunDirectory) ||
      !IFileManager::Get().MakeDirectory(*Paths.RunDirectory, true)) {
    UE_LOG(LogLingTuSimFrontEndScreenshot, Error,
           TEXT("LINGTU_FRONT_END_SCREENSHOT_REJECTED run_id=%s "
                "reason=front_end_screenshot_run_directory_not_new"),
           *RunId);
    ExitWithStatus(2);
    State = EState::Failed;
    return;
  }
  FString PolicyError;
  const bool bNullRHI = FParse::Param(*CommandLine, TEXT("nullrhi"));
  if (!ValidateStartPolicy(bGameSelector, bUnattended, bCanEverRender, bNullRHI, PolicyError)) {
    Fail(*PolicyError);
    return;
  }
  if (!InHUDWidget.IsValid() || InViewport == nullptr || !InLoginModel.IsValid() ||
      !InSelectionModel.IsValid() || !InAssetReviewModel.IsValid()) {
    Fail(TEXT("front_end_runtime_models_unavailable"));
    return;
  }
  if (InSelectionIntentPath.IsEmpty() || FPaths::IsRelative(InSelectionIntentPath)) {
    Fail(TEXT("front_end_selection_intent_absolute_path_required"));
    return;
  }
  HUDWidget = InHUDWidget;
  Viewport = InViewport;
  LoginModel = InLoginModel;
  SelectionModel = InSelectionModel;
  AssetReviewModel = InAssetReviewModel;
  SelectionIntentPath = InSelectionIntentPath;
  FPaths::NormalizeFilename(SelectionIntentPath);
  StartedSeconds = FPlatformTime::Seconds();
  State = EState::AwaitLoginStable;

  FString InvariantError;
  if (!ValidateInvariantState(InvariantError)) {
    Fail(*InvariantError);
    return;
  }
  UE_LOG(LogLingTuSimFrontEndScreenshot, Display,
         TEXT("LINGTU_FRONT_END_SCREENSHOT_STARTED run_id=%s directory=%s"), *RunId,
         *Paths.RunDirectory);
}

void FFrontEndScreenshotDriver::Tick() {
  if (State == EState::Disabled || State == EState::Complete || State == EState::Failed) {
    return;
  }
  if (FPlatformTime::Seconds() - StartedSeconds > ScreenshotTimeoutSeconds) {
    Fail(TEXT("front_end_screenshot_timeout"));
    return;
  }

  FString InvariantError;
  if (!ValidateInvariantState(InvariantError)) {
    Fail(*InvariantError);
    return;
  }

  if (State == EState::AwaitLoginFile) {
    FString MediaError;
    int64 Bytes = 0;
    if (IFileManager::Get().FileSize(*Paths.LoginScreenshot) <= 0) {
      return;
    }
    if (!ValidateScreenshot(Paths.LoginScreenshot, Bytes, MediaError)) {
      Fail(*MediaError);
      return;
    }
    LoginScreenshotBytes = Bytes;
    const TSharedPtr<FFrontEndLoginModel> Login = LoginModel.Pin();
    FString LoginError;
    if (!Login.IsValid() || !Login->SubmitLocalOperator(TEXT("Visual QA"), LoginError)) {
      Fail(LoginError.IsEmpty() ? TEXT("visual_qa_local_login_failed") : *LoginError);
      return;
    }
    if (const TSharedPtr<SWidget> HUD = HUDWidget.Pin()) {
      HUD->Invalidate(EInvalidateWidgetReason::LayoutAndVolatility);
    }
    ConsecutiveStableFrames = 0;
    State = EState::AwaitAssetLibraryStable;
    return;
  }

  if (State == EState::AwaitAssetLibraryFile) {
    FString MediaError;
    int64 Bytes = 0;
    if (IFileManager::Get().FileSize(*Paths.AssetLibraryScreenshot) <= 0) {
      return;
    }
    if (!ValidateScreenshot(Paths.AssetLibraryScreenshot, Bytes, MediaError)) {
      Fail(*MediaError);
      return;
    }
    AssetLibraryScreenshotBytes = Bytes;
    const auto Selection = SelectionModel.Pin();
    if (Selection.IsValid() && Selection->GetSelectedOption() &&
        Selection->GetRobotChoices().ContainsByPredicate([](const auto& Robot) { return Robot.Category == TEXT("humanoid"); })) {
      OriginalSelectionId = Selection->GetSelectedOption()->Id;
      StaticCastSharedPtr<SLingTuSimRuntimeHUD>(HUDWidget.Pin())->PreviewRobotCategory(TEXT("humanoid"));
      ConsecutiveStableFrames = 0;
      State = EState::AwaitCategoryStable;
      return;
    }
    StaticCastSharedPtr<SLingTuSimRuntimeHUD>(HUDWidget.Pin())->AdvanceFrontEndSelection();
    ConsecutiveStableFrames = 0;
    State = EState::AwaitWorldStable;
    return;
  }

  if (State == EState::AwaitCategoryFile) {
    if (IFileManager::Get().FileSize(*Paths.CategoryScreenshot) <= 0) return;
    FString Error;
    if (!ValidateScreenshot(Paths.CategoryScreenshot, CategoryScreenshotBytes, Error)) { Fail(*Error); return; }
    StaticCastSharedPtr<SLingTuSimRuntimeHUD>(HUDWidget.Pin())->PreviewShowroomInspection();
    ConsecutiveStableFrames = 0;
    State = EState::AwaitInspectionStable;
    return;
  }
  if (State == EState::AwaitInspectionFile) {
    const FString Path = FPaths::Combine(Paths.RunDirectory, TEXT("robot-inspection.png"));
    if (IFileManager::Get().FileSize(*Path) <= 0) return;
    FString Error;
    int64 Bytes = 0;
    if (!ValidateScreenshot(Path, Bytes, Error)) { Fail(*Error); return; }
    const auto HUD = StaticCastSharedPtr<SLingTuSimRuntimeHUD>(HUDWidget.Pin());
    HUD->PreviewRobotCategory(TEXT("all"));
    if (!SelectionModel.Pin()->SelectById(OriginalSelectionId)) { Fail(TEXT("category_qa_restore_failed")); return; }
    HUD->AdvanceFrontEndSelection();
    ConsecutiveStableFrames = 0;
    State = EState::AwaitWorldStable;
    return;
  }

  if (State == EState::AwaitWorldFile) {
    const FString& Path = Paths.WorldScreenshot;
    if (IFileManager::Get().FileSize(*Path) <= 0) return;
    FString MediaError;
    int64 Bytes = 0;
    if (!ValidateScreenshot(Path, Bytes, MediaError)) { Fail(*MediaError); return; }
    WorldScreenshotBytes = Bytes;
    if (!WriteSuccess(MediaError)) {
      Fail(*MediaError);
      return;
    }
    State = EState::Complete;
    UE_LOG(LogLingTuSimFrontEndScreenshot, Display,
           TEXT("LINGTU_FRONT_END_SCREENSHOTS_COMPLETE run_id=%s login=%s asset_library=%s"),
           *RunId, *Paths.LoginScreenshot, *Paths.AssetLibraryScreenshot);
    ExitWithStatus(0);
    return;
  }

  const bool bAwaitShowroom = State == EState::AwaitLoginStable || State == EState::AwaitAssetLibraryStable || State == EState::AwaitCategoryStable || State == EState::AwaitInspectionStable;
  const auto HUD = StaticCastSharedPtr<SLingTuSimRuntimeHUD>(HUDWidget.Pin());
  if (!IsViewportReady() || FScreenshotRequest::IsScreenshotRequested() ||
      (bAwaitShowroom && (!HUD.IsValid() || !HUD->IsFrontEndShowroomReady()))) {
    ConsecutiveStableFrames = 0;
    return;
  }
  ++ConsecutiveStableFrames;
  if (ConsecutiveStableFrames < StableFrameCount) {
    return;
  }

  if (State == EState::AwaitLoginStable) {
    RequestCapture(Paths.LoginScreenshot, EState::AwaitLoginFile);
  } else if (State == EState::AwaitAssetLibraryStable) {
    const TSharedPtr<FFrontEndLoginModel> Login = LoginModel.Pin();
    if (!Login.IsValid() || !Login->IsLoggedIn() || Login->GetDisplayName() != TEXT("Visual QA")) {
      Fail(TEXT("asset_library_login_state_invalid"));
      return;
    }
    RequestCapture(Paths.AssetLibraryScreenshot, EState::AwaitAssetLibraryFile);
  } else if (State == EState::AwaitCategoryStable) {
    RequestCapture(Paths.CategoryScreenshot, EState::AwaitCategoryFile);
  } else if (State == EState::AwaitInspectionStable) {
    RequestCapture(FPaths::Combine(Paths.RunDirectory, TEXT("robot-inspection.png")), EState::AwaitInspectionFile);
  } else if (State == EState::AwaitWorldStable) {
    RequestCapture(Paths.WorldScreenshot, EState::AwaitWorldFile);
  }
}

void FFrontEndScreenshotDriver::Detach() {
  State = EState::Disabled;
  Paths = {};
  RunId.Reset();
  SelectionIntentPath.Reset();
  OriginalSelectionId.Reset();
  CategoryScreenshotBytes = 0;
  HUDWidget.Reset();
  Viewport.Reset();
  LoginModel.Reset();
  SelectionModel.Reset();
  AssetReviewModel.Reset();
  StartedSeconds = 0.0;
  ConsecutiveStableFrames = 0;
  LoginScreenshotBytes = 0;
  AssetLibraryScreenshotBytes = 0;
  WorldScreenshotBytes = 0;
  bExitRequested = false;
}

bool FFrontEndScreenshotDriver::IsViewportReady() const {
  const UGameViewportClient *GameViewport = Viewport.Get();
  return HUDWidget.IsValid() && GameViewport != nullptr && GameViewport->Viewport != nullptr &&
         GameViewport->Viewport->GetSizeXY() == FIntPoint(1920, 1080);
}

bool FFrontEndScreenshotDriver::ValidateScreenshot(const FString &Path, int64 &OutBytes,
                                                   FString &OutError) const {
  OutBytes = IFileManager::Get().FileSize(*Path);
  if (OutBytes <= 0) {
    OutError = TEXT("front_end_screenshot_empty");
    return false;
  }
  int32 Width = 0;
  int32 Height = 0;
  if (!FHudScreenshotContract::ReadPngDimensions(Path, Width, Height, OutError)) {
    return false;
  }
  if (Width != 1920 || Height != 1080) {
    OutError = TEXT("front_end_screenshot_media_not_exact_1920x1080");
    return false;
  }
  return true;
}

bool FFrontEndScreenshotDriver::ValidateInvariantState(FString &OutError) const {
  OutError.Reset();
  const TSharedPtr<FGameSelectionModel> Selection = SelectionModel.Pin();
  const TSharedPtr<FAssetReviewModel> AssetReview = AssetReviewModel.Pin();
  if (!Selection.IsValid() || !AssetReview.IsValid()) {
    OutError = TEXT("front_end_models_detached");
    return false;
  }
  if (Selection->IsConfirmed()) {
    OutError = TEXT("session_confirmed_during_visual_qa");
    return false;
  }
  const FAssetReviewCatalog &Catalog = AssetReview->GetCatalog();
  if (!Catalog.bAvailable || Catalog.Cards.IsEmpty()) {
    OutError = TEXT("asset_review_catalog_unavailable_or_empty");
    return false;
  }
  if (!FileDoesNotExist(SelectionIntentPath)) {
    OutError = TEXT("selection_intent_exists_during_visual_qa");
    return false;
  }
  if (!FileDoesNotExist(SelectionIntentPath + TEXT(".tmp"))) {
    OutError = TEXT("selection_intent_temporary_exists_during_visual_qa");
    return false;
  }
  return true;
}

bool FFrontEndScreenshotDriver::RequestCapture(const FString &Path, const EState AwaitFileState) {
  if (!FileDoesNotExist(Path)) {
    Fail(TEXT("front_end_screenshot_target_not_new"));
    return false;
  }
  FScreenshotRequest::RequestScreenshot(Path, true, false);
  if (!FScreenshotRequest::IsScreenshotRequested()) {
    Fail(TEXT("front_end_screenshot_request_rejected"));
    return false;
  }
  ConsecutiveStableFrames = 0;
  State = AwaitFileState;
  UE_LOG(LogLingTuSimFrontEndScreenshot, Display,
         TEXT("LINGTU_FRONT_END_SCREENSHOT_REQUESTED path=%s show_ui=true"), *Path);
  return true;
}

bool FFrontEndScreenshotDriver::WriteSuccess(FString &OutError) const {
  if (!ValidateInvariantState(OutError)) {
    return false;
  }
  int64 VerifiedLoginBytes = 0;
  int64 VerifiedAssetBytes = 0;
  int64 VerifiedWorldBytes = 0;
  int64 VerifiedCategoryBytes = 0;
  if (CategoryScreenshotBytes > 0 &&
      (!ValidateScreenshot(Paths.CategoryScreenshot, VerifiedCategoryBytes, OutError) || VerifiedCategoryBytes != CategoryScreenshotBytes)) return false;
  if (!ValidateScreenshot(Paths.LoginScreenshot, VerifiedLoginBytes, OutError) ||
      !ValidateScreenshot(Paths.AssetLibraryScreenshot, VerifiedAssetBytes, OutError) ||
      !ValidateScreenshot(Paths.WorldScreenshot, VerifiedWorldBytes, OutError) ||
      VerifiedLoginBytes != LoginScreenshotBytes ||
      VerifiedAssetBytes != AssetLibraryScreenshotBytes || VerifiedWorldBytes != WorldScreenshotBytes) {
    if (OutError.IsEmpty()) {
      OutError = TEXT("front_end_screenshot_changed_before_commit");
    }
    return false;
  }
  const TSharedPtr<FAssetReviewModel> AssetReview = AssetReviewModel.Pin();
  const int32 CardCount = AssetReview.IsValid() ? AssetReview->GetCatalog().Cards.Num() : 0;
  const FString Json = FString::Printf(
      TEXT("{\"schema\":\"lingtu.sim.ue-front-end-screenshots.v1\",\"state\":\"CAPTURED\","
           "\"run_id\":\"%s\",\"operator\":{\"method\":\"local_operator\","
           "\"display_name\":\"Visual QA\"},\"session\":{\"confirmed\":false,"
           "\"intent_exists\":false},\"asset_catalog\":{\"available\":true,"
           "\"card_count\":%d},\"category_preview\":{\"basename\":\"category-humanoid.png\",\"bytes\":%lld},\"screenshots\":[{\"basename\":\"login.png\","
           "\"bytes\":%lld,\"width\":1920,\"height\":1080,\"show_ui\":true},"
           "{\"basename\":\"asset-library.png\",\"bytes\":%lld,\"width\":1920,"
           "\"height\":1080,\"show_ui\":true},"
           "{\"basename\":\"map-selection.png\",\"bytes\":%lld,\"width\":1920,\"height\":1080,\"show_ui\":true}]}\n"),
      *RunId, CardCount, static_cast<long long>(VerifiedCategoryBytes), static_cast<long long>(VerifiedLoginBytes),
      static_cast<long long>(VerifiedAssetBytes), static_cast<long long>(VerifiedWorldBytes));
  const FString Temporary = Paths.SuccessSentinel + TEXT(".tmp");
  if (!FileDoesNotExist(Paths.SuccessSentinel) || !FileDoesNotExist(Temporary) ||
      !FFileHelper::SaveStringToFile(Json, *Temporary,
                                     FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM,
                                     &IFileManager::Get(), FILEWRITE_NoReplaceExisting)) {
    OutError = TEXT("front_end_success_sentinel_temporary_write_failed");
    return false;
  }
  if (!IFileManager::Get().Move(*Paths.SuccessSentinel, *Temporary, false, false, false, true)) {
    IFileManager::Get().Delete(*Temporary, false, true, true);
    OutError = TEXT("front_end_success_sentinel_atomic_publish_failed");
    return false;
  }
  return true;
}

void FFrontEndScreenshotDriver::Fail(const TCHAR *Reason) {
  const FString StableReason = Reason == nullptr || Reason[0] == 0 ? TEXT("unknown") : Reason;
  State = EState::Failed;
  if (!Paths.RunDirectory.IsEmpty() && IFileManager::Get().DirectoryExists(*Paths.RunDirectory)) {
    const FString Payload = StableReason + TEXT("\n");
    FFileHelper::SaveStringToFile(Payload, *Paths.ErrorSentinel,
                                  FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM,
                                  &IFileManager::Get(), FILEWRITE_NoReplaceExisting);
  }
  UE_LOG(LogLingTuSimFrontEndScreenshot, Error,
         TEXT("LINGTU_FRONT_END_SCREENSHOT_FAILED run_id=%s reason=%s"),
         RunId.IsEmpty() ? TEXT("unavailable") : *RunId, *StableReason);
  ExitWithStatus(2);
}

void FFrontEndScreenshotDriver::ExitWithStatus(const uint8 Status) {
  if (!bExitRequested) {
    bExitRequested = true;
    FPlatformMisc::RequestExitWithStatus(false, Status, TEXT("LingTuSimFrontEndScreenshotDriver"));
  }
}
}  // namespace LingTuSim::UI

#endif

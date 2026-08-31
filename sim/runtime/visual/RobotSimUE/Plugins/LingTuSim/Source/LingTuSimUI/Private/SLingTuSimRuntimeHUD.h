#pragma once

#include "LingTuSimFrontEndLogin.h"
#include "LingTuSimGameSelection.h"
#include "LingTuSimRuntimeUIStatus.h"
#include "Widgets/SCompoundWidget.h"

class UWorld;
class SButton;
class SEditableTextBox;
struct FSlateDynamicImageBrush;

namespace LingTuSim::UI {
class FRuntimeUIModeController;

/** In-game overlay with staged presentation resources; runtime values remain copied read-only. */
class SLingTuSimRuntimeHUD final : public SCompoundWidget {
 public:
  SLATE_BEGIN_ARGS(SLingTuSimRuntimeHUD)
      : _World(nullptr), _FrontEndLoginRequired(false), _SelectionIntentConfigured(false) {}
  SLATE_ARGUMENT(UWorld *, World)
  SLATE_ARGUMENT(TSharedPtr<FRuntimeUIModeController>, ModeController)
  SLATE_ARGUMENT(TSharedPtr<FRuntimeUILocalState>, LocalState)
  SLATE_ARGUMENT(TSharedPtr<FFrontEndLoginModel>, LoginModel)
  SLATE_ARGUMENT(TSharedPtr<FGameSelectionModel>, SelectionModel)
  SLATE_ARGUMENT(TSharedPtr<FAssetReviewModel>, AssetReviewModel)
  SLATE_ARGUMENT(TSharedPtr<FString>, SelectionFeedback)
  SLATE_ARGUMENT(bool, FrontEndLoginRequired)
  SLATE_ARGUMENT(bool, SelectionIntentConfigured)
  SLATE_EVENT(FSimpleDelegate, OnSelectionPrevious)
  SLATE_EVENT(FSimpleDelegate, OnSelectionNext)
  SLATE_EVENT(FSimpleDelegate, OnSelectionConfirmed)
  SLATE_EVENT(FSimpleDelegate, OnAssetReviewPrevious)
  SLATE_EVENT(FSimpleDelegate, OnAssetReviewNext)
  SLATE_END_ARGS()

  void Construct(const FArguments &InArgs);
  virtual bool SupportsKeyboardFocus() const override { return true; }
  virtual void Tick(const FGeometry &AllottedGeometry, double InCurrentTime,
                    float InDeltaTime) override;
  bool IsOperatorNameEditing() const;
  void FocusFrontEndLoginCTA();

 private:
  TSharedRef<SWidget> BuildLoginScreen();
  TSharedRef<SWidget> BuildWorkspaceScreen();
  TSharedRef<SWidget> BuildAssetHeroPreview();
  TSharedRef<SWidget> BuildAssetFilmstrip();
  FText GetIdentityText() const;
  FText GetReadinessText() const;
  FText GetModeText() const;
  FText GetInputText() const;
  FText GetRequestedAxesText() const;
  FText GetAckText() const;
  FText GetAcceptedMotionText() const;
  FText GetObservedVelocityText() const;
  FText GetSensorsText() const;
  FText GetRecordingText() const;
  FText GetCameraText() const;
  FText GetTruthFrameText() const;
  FText GetModeHintText() const;
  FText GetMenuRequestText() const;
  FText GetFrontEndIdentityText() const;
  FText GetLoginCatalogStatusText() const;
  FText GetLoginFeedbackText() const;
  FText GetSelectionPositionText() const;
  FText GetSelectionTitleText() const;
  FText GetSelectionDescriptionText() const;
  FText GetSelectionAvailabilityText() const;
  FText GetSelectionPackagesText() const;
  FText GetSelectionAssetsText() const;
  FText GetSelectionStatusText() const;
  FText GetAssetReviewPositionText() const;
  FText GetAssetReviewTitleText() const;
  FText GetAssetReviewDescriptionText() const;
  FText GetAssetReviewDispositionText() const;
  FText GetAssetReviewReasonText() const;
  FText GetAssetReviewClassStageText() const;
  FText GetAssetReviewEvidenceText() const;
  FText GetAssetReviewPolicyText() const;
  FSlateColor GetAssetReviewDispositionColor() const;
  FSlateColor GetSelectionAvailabilityColor() const;
  FSlateColor GetSelectionStatusColor() const;
  bool CanConfirmSelection() const;
  FReply HandlePreviousSelection();
  FReply HandleNextSelection();
  FReply HandleConfirmSelection();
  FReply HandlePreviousAssetReview();
  FReply HandleNextAssetReview();
  FReply HandleSelectAssetReview(FString CardId);
  FReply HandleEnterLocalWorkspace();
  FReply HandleLogout();
  void HandleOperatorNameChanged(const FText &NewText);
  bool CanEnterLocalWorkspace() const;
  bool CanLogout() const;
  bool CanBrowseSessions() const;
  bool CanBrowseAssets() const;
  bool CanSelectAssetReview(FString CardId) const;
  const FSlateBrush *GetAssetPreviewBrush(const FString &CardId) const;
  FSlateColor GetAssetFilmstripCardColor(FString CardId) const;
  FSlateColor GetAssetFilmstripPreviewColor(FString CardId) const;
  EVisibility GetBuildVisibility() const;
  EVisibility GetTacticalVisibility() const;
  EVisibility GetPauseVisibility() const;
  EVisibility GetLoginVisibility() const;
  EVisibility GetWorkspaceVisibility() const;
  EVisibility GetLogoutVisibility() const;
  EVisibility GetFrontEndStaticBackdropVisibility() const;
  EVisibility GetRuntimeHudChromeVisibility() const;
  EVisibility GetNonPauseVisibility() const;

  TWeakObjectPtr<UWorld> World;
  TSharedPtr<FRuntimeUIModeController> ModeController;
  TSharedPtr<FRuntimeUILocalState> LocalState;
  TSharedPtr<FFrontEndLoginModel> LoginModel;
  TSharedPtr<FGameSelectionModel> SelectionModel;
  TSharedPtr<FAssetReviewModel> AssetReviewModel;
  TSharedPtr<FString> SelectionFeedback;
  FSimpleDelegate OnSelectionPrevious;
  FSimpleDelegate OnSelectionNext;
  FSimpleDelegate OnSelectionConfirmed;
  FSimpleDelegate OnAssetReviewPrevious;
  FSimpleDelegate OnAssetReviewNext;
  TSharedPtr<SEditableTextBox> OperatorNameTextBox;
  TSharedPtr<SButton> LocalWorkspaceButton;
  TSharedPtr<SButton> ConfirmSelectionButton;
  TSharedPtr<FSlateDynamicImageBrush> FrontEndBackgroundBrush;
  TMap<FString, TSharedPtr<FSlateDynamicImageBrush>> AssetPreviewBrushes;
  FString PendingOperatorName = TEXT("Operator");
  FString LoginFeedback;
  bool bFrontEndLoginRequired = false;
  bool bSelectionIntentConfigured = false;
  FRuntimeUIStatusSnapshot CachedStatus;
};
}  // namespace LingTuSim::UI

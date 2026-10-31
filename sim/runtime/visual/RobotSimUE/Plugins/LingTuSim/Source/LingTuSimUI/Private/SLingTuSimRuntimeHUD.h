#pragma once

#include "LingTuSimFrontEndLogin.h"
#include "LingTuSimGameSelection.h"
#include "LingTuSimInspectionProjection.h"
#include "LingTuSimRuntimeUIStatus.h"
#include "LingTuSimPlayerSettings.h"
#include "LingTuSimGraphicsSettings.h"
#include "LingTuSimMenuFeedback.h"
#include "LingTuSimMenuAudio.h"
#include "Widgets/SCompoundWidget.h"

class UWorld;
class SButton;
class SScrollBox;
class SEditableTextBox;
struct FSlateDynamicImageBrush;

namespace LingTuSim::UI {
class FRuntimeUIModeController;
class SLingTuSimRobotMapPicker;
class SLingTuSimShowroom;
class SLingTuSimShowroomInput;
class FMeasurementClient;
class SLingTuSimMeasurements;

/** In-game overlay with staged presentation resources; runtime values remain copied read-only. */
class SLingTuSimRuntimeHUD final : public SCompoundWidget {
 public:
  SLATE_BEGIN_ARGS(SLingTuSimRuntimeHUD)
      : _World(nullptr), _FrontEndLoginRequired(false), _SelectionIntentConfigured(false) {}
  SLATE_ARGUMENT(UWorld *, World)
  SLATE_ARGUMENT(TSharedPtr<FRuntimeUIModeController>, ModeController)
  SLATE_ARGUMENT(TSharedPtr<FRuntimeUILocalState>, LocalState)
  SLATE_ARGUMENT(TSharedPtr<FPlayerSettings>, PlayerSettings)
  SLATE_ARGUMENT(TSharedPtr<FFrontEndLoginModel>, LoginModel)
  SLATE_ARGUMENT(TSharedPtr<FGameSelectionModel>, SelectionModel)
  SLATE_ARGUMENT(TSharedPtr<FAssetReviewModel>, AssetReviewModel)
  SLATE_ARGUMENT(TSharedPtr<FInspectionProjection>, InspectionProjection)
  SLATE_ARGUMENT(TSharedPtr<FMeasurementClient>, Measurements)
  SLATE_ARGUMENT(TSharedPtr<FString>, SelectionFeedback)
  SLATE_ARGUMENT(bool, FrontEndLoginRequired)
  SLATE_ARGUMENT(bool, SelectionIntentConfigured)
  SLATE_EVENT(FSimpleDelegate, OnSelectionPrevious)
  SLATE_EVENT(FSimpleDelegate, OnSelectionNext)
  SLATE_EVENT(FSimpleDelegate, OnSelectionConfirmed)
  SLATE_EVENT(FSimpleDelegate, OnAssetReviewPrevious)
  SLATE_EVENT(FSimpleDelegate, OnAssetReviewNext)
  SLATE_EVENT(FSimpleDelegate, OnCycleCamera)
  SLATE_EVENT(FSimpleDelegate, OnToggleRecording)
  SLATE_EVENT(FSimpleDelegate, OnRequestExit)
  SLATE_EVENT(FSimpleDelegate, OnRequestRestart)
  SLATE_EVENT(FSimpleDelegate, OnRequestMainMenu)
  SLATE_EVENT(FSimpleDelegate, OnInspectionPause)
  SLATE_EVENT(FSimpleDelegate, OnInspectionResume)
  SLATE_EVENT(FSimpleDelegate, OnInspectionCancel)
  SLATE_END_ARGS()

  void Construct(const FArguments &InArgs);
  virtual ~SLingTuSimRuntimeHUD() override;
  virtual bool SupportsKeyboardFocus() const override { return true; }
  virtual FReply OnKeyDown(const FGeometry& Geometry, const FKeyEvent& Event) override;
  virtual void Tick(const FGeometry &AllottedGeometry, double InCurrentTime,
                    float InDeltaTime) override;
  bool IsOperatorNameEditing() const;
  bool IsTextEntryEditing() const;
  bool IsMeasurementPageOpen() const;
  void FocusFrontEndLoginCTA();
  void FocusRuntimeMenuCTA();
  bool HandleRuntimeMenuBack();
  void AdvanceFrontEndSelection();
  void RestoreSelectionFocus();
  bool HandleKeyBindingCapture(const FKeyEvent& Event);
  void CancelKeyBindingCapture(bool bRestoreFocus = false);
  void OpenQuitConfirmation();
  void TickGraphicsPreview();
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
  void PreviewRobotCategory(const FString& Category);
  bool IsFrontEndShowroomReady() const;
  void PreviewShowroomInspection();
  bool ProbeWelcomeInspection(bool bReset);
  void PreviewPlayerMenuPage(const FString& Page);
  void ProbeGraphicsPreview(bool bVerifyRollback);
  bool HasGraphicsDraftChangesForTest() const { return HasGraphicsDraftChanges(); }
  bool IsGraphicsPreviewPendingForTest() const { return GraphicsPreview.IsPending(); }
  void ApplyGraphicsSettingsForTest() { ApplyGraphicsSettings(); }
#endif
  static FText FormatFieldSpeed(const FRuntimeUIStatusSnapshot& Status);
  static FText FormatDriveSpeedValue(const FRuntimeUIStatusSnapshot& Status);
  static FText FormatDriveNotice(const FRuntimeUIStatusSnapshot& Status);
  static FText FormatFieldTurnRate(const FRuntimeUIStatusSnapshot& Status);
  static FText FormatDriveLink(const FRuntimeUIStatusSnapshot& Status);
  static FText FormatFieldTime(const FRuntimeUIStatusSnapshot& Status);

 private:
  static bool HasHealthyDriveLink(const FRuntimeUIStatusSnapshot& Status);
  TSharedRef<SWidget> BuildLoginScreen();
  TSharedRef<SWidget> BuildWorkspaceScreen();
  TSharedRef<SWidget> BuildAssetReviewWorkspace();
  TSharedRef<SWidget> BuildRobotMapPicker(bool bFrontEnd);
  TSharedRef<SWidget> BuildDriveInterface();
  TSharedRef<SWidget> BuildDriveHelp();
  FText GetDriveSpeedValueText() const;
  FText GetDriveNoticeText() const;
  FSlateColor GetDriveNoticeColor() const;
  TSharedRef<SWidget> BuildTacticalInterface();
  TSharedRef<SWidget> BuildRuntimeMenu();
  TSharedRef<SWidget> BuildTaskJournal();
  TSharedRef<SWidget> BuildPlayerSettings();
  bool HasSettingsChanges() const;
  FReply RequestSettingsBack();
  FReply ContinueSettingsEdit();
  TSharedRef<SWidget> BuildGraphicsSettingsPage();
  TSharedRef<SWidget> BuildControlsSettingsPage();
  TSharedRef<SWidget> BuildDriveKeyBindingRow(EDriveKeyAction Action);
  void FinishKeyBindingCapture(bool bRestoreFocus);
  FText GetDriveKeysText(bool bTurn) const;
  TSharedRef<SWidget> BuildInterfaceSettingsPage();
  TSharedRef<SWidget> BuildAudioSettingsPage();
  enum class ESettingsTab : uint8 { Graphics, Controls, Interface, Audio, Count };
  void SelectSettingsTab(ESettingsTab Tab);
  void BeginPlayerSettingsEdit();
  void RestoreUnsavedPlayerSettings(bool bKeepEditing = false);
  void AdvancePlayerSettingsCheckpoint();
  FReply ApplyGraphicsSettings();
  FReply ConfirmGraphicsSettings();
  FReply RevertGraphicsSettings();
  bool CanEditGraphicsSettings() const;
  bool HasGraphicsDraftChanges() const;
  bool CanApplyGraphicsSettings() const;
  TSharedRef<SWidget> BuildMenuConfirmation();
  enum class EPlayerMenuPage : uint8 { Root, Journal, Measurements, Library, Settings, Restart, MainMenu, Quit };
  void ShowMenuPage(EPlayerMenuPage Page);
  FReply ConfirmMenuAction();
  bool CanConfirmMenuAction() const;
  FText GetMenuConfirmationTitle() const;
  FText GetMenuConfirmationBody() const;
  FText GetFieldSpeedText() const;
  FText GetFieldTurnRateText() const;
  FText GetDriveLinkText() const;
  FText GetDriveCameraText() const;
  FText GetDriveRecordingText() const;
  FText GetPlayerFrameRateText() const;
  FText GetFieldTimeText() const;
  FText GetFieldSensorText() const;
  FText GetFieldRecordText() const;
  FReply HandleMenuResume();
  FReply HandleMenuTactical();
  FReply HandleMenuBuild();
  EVisibility GetRuntimeMenuVisibility() const;
  TSharedRef<SWidget> BuildAssetHeroPreview();
  TSharedRef<SWidget> BuildAssetFilmstrip();
  FText GetIdentityText() const;
  FText GetReadinessText() const;
  FText GetDriveMissionText() const;
  FText GetInspectionProgressText() const;
  FText GetInspectionReportText() const;
  FSlateColor GetInspectionStatusColor() const;
  FText GetDriveSessionText() const;
  FText GetDriveTelemetryText() const;
  FSlateColor GetDriveStatusColor() const;
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
  EVisibility GetRuntimeHudChromeVisibility() const;
  EVisibility GetDriveHudVisibility() const;
  EVisibility GetNonPauseVisibility() const;

  TWeakObjectPtr<UWorld> World;
  TSharedPtr<FRuntimeUIModeController> ModeController;
  TSharedPtr<FRuntimeUILocalState> LocalState;
  TSharedPtr<FPlayerSettings> PlayerSettings;
  FFrontEndMusicPlayer FrontEndMusic;
  TSharedPtr<FSlateDynamicImageBrush> FactoryBackgroundBrush;
  TSharedPtr<FFrontEndLoginModel> LoginModel;
  TSharedPtr<FGameSelectionModel> SelectionModel;
  TSharedPtr<FAssetReviewModel> AssetReviewModel;
  TSharedPtr<FInspectionProjection> InspectionProjection;
  TSharedPtr<FMeasurementClient> Measurements;
  TSharedPtr<SLingTuSimMeasurements> MeasurementPanel;
  TSharedPtr<FString> SelectionFeedback;
  FSimpleDelegate OnSelectionPrevious;
  FSimpleDelegate OnSelectionNext;
  FSimpleDelegate OnSelectionConfirmed;
  FSimpleDelegate OnAssetReviewPrevious;
  FSimpleDelegate OnAssetReviewNext;
  FSimpleDelegate OnCycleCamera;
  FSimpleDelegate OnToggleRecording;
  FSimpleDelegate OnRequestExit;
  FSimpleDelegate OnRequestRestart;
  FSimpleDelegate OnRequestMainMenu;
  FSimpleDelegate OnInspectionPause;
  FSimpleDelegate OnInspectionResume;
  FSimpleDelegate OnInspectionCancel;
  EPlayerMenuPage MenuPage = EPlayerMenuPage::Root;
  TWeakPtr<SWidget> RuntimeMenuFocusTarget;
  ESettingsTab SettingsTab = ESettingsTab::Graphics;
  FText SettingsHelpTitle;
  FText SettingsHelpBody;
  FMenuFeedbackState MenuMotion;
  TSharedPtr<SWidget> SettingsAnimatedContent;
  double LastMenuCueAt = -1.;
  void NotifySettingChanged(const FText& Name, const FText& Value, bool bNeedsApply);
  void EmitMenuCue(EMenuCue Cue);
  TSharedPtr<SScrollBox> SettingsScrollBoxes[static_cast<int32>(ESettingsTab::Count)];
  FPlayerSettings PlayerSettingsCheckpoint;
  bool bPlayerSettingsEditActive = false;
  bool bSettingsExitDecision = false;
  TWeakPtr<SWidget> SettingsExitReturnFocus;
  TSharedPtr<SButton> SettingsContinueButton;
  TOptional<EDriveKeyAction> PendingDriveKey;
  TWeakPtr<SWidget> KeyBindingReturnFocus;
  double KeyBindingErrorUntil = 0.0;
  FPlayerGraphicsSettings GraphicsDraft;
  FPlayerGraphicsSettings GraphicsSettingsCheckpoint;
  FPlayerGraphicsPreview GraphicsPreview;
  TArray<FIntPoint> GraphicsResolutions;
  TSharedPtr<SButton> GraphicsTabButton;
  TSharedPtr<SButton> ControlsTabButton;
  TSharedPtr<SButton> InterfaceTabButton;
  TSharedPtr<SButton> AudioTabButton;
  TSharedPtr<SButton> SettingsMenuButton;
  TSharedPtr<SButton> JournalMenuButton;
  TSharedPtr<SButton> MeasurementsMenuButton;
  TSharedPtr<SButton> JournalBackButton;
  TSharedPtr<SButton> LibraryMenuButton;
  TSharedPtr<SLingTuSimRobotMapPicker> WorkspaceRobotMapPicker;
  TSharedPtr<SLingTuSimRobotMapPicker> RuntimeRobotMapPicker;
  bool bAssetReviewWorkspace = false;
  TSharedPtr<SButton> ConfirmationCancelButton;
  TSharedPtr<SButton> SettingsBackButton;
  FString MenuFeedback;
  TSharedPtr<SButton> ResumeMenuButton;
  bool bRuntimeMenuWasVisible = false;
  float SmoothedFrameTimeSeconds = 0.F;
  TSharedPtr<SEditableTextBox> OperatorNameTextBox;
  TSharedPtr<SButton> LocalWorkspaceButton;
  TSharedPtr<SButton> WelcomeSettingsButton;
  TSharedPtr<SButton> WelcomeQuitButton;
  TSharedPtr<SLingTuSimShowroom> WelcomeShowroom;
  TSharedPtr<SLingTuSimShowroomInput> WelcomeShowroomInput;
  TSharedPtr<SButton> ConfirmSelectionButton;
  TMap<FString, TSharedPtr<FSlateDynamicImageBrush>> AssetPreviewBrushes;
  FString PendingOperatorName = TEXT("Operator");
  FString LoginFeedback;
  bool bFrontEndLoginRequired = false;
  bool bSelectionIntentConfigured = false;
  FRuntimeUIStatusSnapshot CachedStatus;
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
  FPlayerGraphicsSettings GraphicsProbeBefore;
  float GraphicsProbeBeforeScreenPercentage = 0.F;
  bool bGraphicsProbeStarted = false;
#endif
};
}  // namespace LingTuSim::UI

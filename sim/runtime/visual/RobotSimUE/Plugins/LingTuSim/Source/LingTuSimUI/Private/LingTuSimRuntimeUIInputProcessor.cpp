#include "LingTuSimRuntimeUIInputProcessor.h"

#include "Framework/Application/SlateApplication.h"
#include "Input/Events.h"
#include "LingTuSimRuntimeUIModel.h"
#include "Widgets/SWidget.h"

namespace LingTuSim::UI {
namespace {
constexpr float PublishIntervalSeconds = 1.0f / 30.0f;

bool IsRuntimeUIKey(const FKey &Key) {
  return Key == EKeys::B || Key == EKeys::Tab || Key == EKeys::Escape ||
         Key == EKeys::Gamepad_FaceButton_Right;
}

bool IsConfirmKey(const FKey &Key) {
  return Key == EKeys::Enter || Key == EKeys::Gamepad_FaceButton_Bottom;
}
}  // namespace

FLingTuSimRuntimeUIInputProcessor::FLingTuSimRuntimeUIInputProcessor(
    TSharedRef<FRuntimeUIModeController> InModeController, TWeakPtr<SWidget> InHUDWidget,
    TWeakPtr<SWidget> InViewportWidget, FRobotDriveInputPublisher InPublisher,
    FRuntimeUIActionPublisher InActionPublisher, FRuntimeUIFrontEndLockQuery InFrontEndLockQuery,
    FRuntimeUITextEntryFocusQuery InTextEntryFocusQuery)
    : ModeController(MoveTemp(InModeController)),
      HUDWidget(MoveTemp(InHUDWidget)),
      ViewportWidget(MoveTemp(InViewportWidget)),
      Publisher(MoveTemp(InPublisher)),
      ActionPublisher(MoveTemp(InActionPublisher)),
      FrontEndLockQuery(MoveTemp(InFrontEndLockQuery)),
      TextEntryFocusQuery(MoveTemp(InTextEntryFocusQuery)) {
  DriveInput.SetDriveMode(ModeController->GetMode() == ERuntimeUIMode::Drive);
}

FLingTuSimRuntimeUIInputProcessor::~FLingTuSimRuntimeUIInputProcessor() {
  ReleaseInput();
}

void FLingTuSimRuntimeUIInputProcessor::Tick(const float DeltaTime,
                                             FSlateApplication &SlateApplication,
                                             TSharedRef<ICursor> Cursor) {
  (void)Cursor;
  PublishForUpdate(SynchronizeContext(SlateApplication));
  const FRobotDriveInputSnapshot Snapshot = DriveInput.GetSnapshot();
  if (!DriveInput.IsEligible() || !Snapshot.bDeadman) {
    PublishAccumulatorSeconds = 0.0f;
    return;
  }

  PublishAccumulatorSeconds += FMath::Clamp(DeltaTime, 0.0f, 0.25f);
  if (PublishAccumulatorSeconds >= PublishIntervalSeconds) {
    PublishAccumulatorSeconds = FMath::Fmod(PublishAccumulatorSeconds, PublishIntervalSeconds);
    Publish(ERobotDrivePublishReason::Periodic);
  }
}

bool FLingTuSimRuntimeUIInputProcessor::HandleKeyDownEvent(FSlateApplication &SlateApplication,
                                                           const FKeyEvent &KeyEvent) {
  PublishForUpdate(SynchronizeContext(SlateApplication));
  const FKey &Key = KeyEvent.GetKey();
  if (!DriveInput.GetSnapshot().bViewportFocused) {
    return false;
  }
  ERuntimeUIAction Action = ERuntimeUIAction::ToggleRecording;
  const TSharedPtr<SWidget> FocusedWidget = SlateApplication.GetKeyboardFocusedWidget();
  const TSharedPtr<SWidget> HUD = HUDWidget.Pin();
  if (IsConfirmKey(Key) && HUD.IsValid() && FocusedWidget.IsValid() && FocusedWidget != HUD &&
      FocusedWidget->GetVisibility().IsVisible() && HUD->HasFocusedDescendants()) {
    // Let the focused SButton handle keyboard or gamepad accept through Slate's
    // normal activation path. The action policy remains the fallback when no
    // child control owns focus.
    return false;
  }
  if (TextEntryFocusQuery && TextEntryFocusQuery()) {
    // Text editing owns cursor, deletion, and commit keys. Do not reinterpret
    // them as hidden session or asset navigation.
    return false;
  }
  if (FRuntimeUIActionPolicy::ResolveKey(ModeController->GetMode(), Key, Action)) {
    if (!KeyEvent.IsRepeat() && ActionPublisher) {
      ActionPublisher(Action);
      InvalidateHUD();
    }
    return true;
  }
  if (IsRuntimeUIKey(Key)) {
    if (FrontEndLockQuery && FrontEndLockQuery()) {
      // A selector process is a full-screen front end, never a gameplay mode.
      // B/Escape uses the same authority-resolved action path as every other
      // selector mutation; it never toggles a hidden runtime mode.
      if (!KeyEvent.IsRepeat() &&
          (Key == EKeys::Escape || Key == EKeys::Gamepad_FaceButton_Right) && ActionPublisher) {
        ActionPublisher(ERuntimeUIAction::ReturnFromFrontEnd);
        InvalidateHUD();
      }
      return true;
    }
    if (!KeyEvent.IsRepeat()) {
      ModeController->HandleKey(Key);
      const ERobotDriveInputUpdate ModeUpdate =
          DriveInput.SetDriveMode(ModeController->GetMode() == ERuntimeUIMode::Drive);
      if (ModeUpdate == ERobotDriveInputUpdate::Handled) {
        Publish(ERobotDrivePublishReason::StateChanged);
      } else {
        PublishForUpdate(ModeUpdate);
      }
      InvalidateHUD();
    }
    return true;
  }

  if (FrontEndLockQuery && FrontEndLockQuery()) {
    // The full-screen selector owns the whole input surface. Swallow gameplay
    // keys that are not front-end actions so they cannot accumulate in the
    // robot drive state or enter a runtime mode behind the front end.
    return true;
  }

  const ERobotDriveInputUpdate Update = DriveInput.HandleKeyDown(Key, KeyEvent.IsRepeat());
  if (Update == ERobotDriveInputUpdate::Unhandled) {
    return false;
  }
  PublishForUpdate(Update);
  return true;
}

bool FLingTuSimRuntimeUIInputProcessor::HandleKeyUpEvent(FSlateApplication &SlateApplication,
                                                         const FKeyEvent &KeyEvent) {
  PublishForUpdate(SynchronizeContext(SlateApplication));
  if (!DriveInput.GetSnapshot().bViewportFocused) {
    return false;
  }
  if (FrontEndLockQuery && FrontEndLockQuery()) {
    return true;
  }
  const ERobotDriveInputUpdate Update = DriveInput.HandleKeyUp(KeyEvent.GetKey());
  if (Update == ERobotDriveInputUpdate::Unhandled) {
    return false;
  }
  PublishForUpdate(Update);
  return true;
}

bool FLingTuSimRuntimeUIInputProcessor::HandleAnalogInputEvent(
    FSlateApplication &SlateApplication, const FAnalogInputEvent &AnalogInputEvent) {
  PublishForUpdate(SynchronizeContext(SlateApplication));
  if (!DriveInput.GetSnapshot().bViewportFocused) {
    return false;
  }
  if (FrontEndLockQuery && FrontEndLockQuery()) {
    return true;
  }
  const ERobotDriveInputUpdate Update =
      DriveInput.HandleAnalog(AnalogInputEvent.GetKey(), AnalogInputEvent.GetAnalogValue());
  if (Update == ERobotDriveInputUpdate::Unhandled) {
    return false;
  }
  PublishForUpdate(Update);
  return true;
}

void FLingTuSimRuntimeUIInputProcessor::ReleaseInput() {
  if (bTeardownReleasePublished) {
    return;
  }
  bTeardownReleasePublished = true;
  DriveInput.ReleaseAll();
  Publish(ERobotDrivePublishReason::Release);
}

ERobotDriveInputUpdate
FLingTuSimRuntimeUIInputProcessor::SynchronizeContext(FSlateApplication &SlateApplication) {
  const ERobotDriveInputUpdate ModeUpdate =
      DriveInput.SetDriveMode(ModeController->GetMode() == ERuntimeUIMode::Drive);
  const TSharedPtr<SWidget> Viewport = ViewportWidget.Pin();
  const TSharedPtr<SWidget> HUD = HUDWidget.Pin();
  const bool bViewportOwnsFocus =
      Viewport.IsValid() &&
      (Viewport->HasKeyboardFocus() || Viewport->HasFocusedDescendants() ||
       Viewport->HasUserFocus(0).IsSet() || Viewport->HasUserFocusedDescendants(0));
  const bool bMenuOwnsFocus = ModeController->GetMode() == ERuntimeUIMode::Pause && HUD.IsValid() &&
                              (HUD->HasKeyboardFocus() || HUD->HasFocusedDescendants() ||
                               HUD->HasUserFocus(0).IsSet() || HUD->HasUserFocusedDescendants(0));
  const bool bViewportFocused =
      SlateApplication.IsActive() && (bViewportOwnsFocus || bMenuOwnsFocus);
  const ERobotDriveInputUpdate FocusUpdate = DriveInput.SetViewportFocused(bViewportFocused);
  if (ModeUpdate == ERobotDriveInputUpdate::Released ||
      FocusUpdate == ERobotDriveInputUpdate::Released) {
    return ERobotDriveInputUpdate::Released;
  }
  if (ModeUpdate == ERobotDriveInputUpdate::Changed ||
      FocusUpdate == ERobotDriveInputUpdate::Changed) {
    return ERobotDriveInputUpdate::Changed;
  }
  return ERobotDriveInputUpdate::Handled;
}

void FLingTuSimRuntimeUIInputProcessor::PublishForUpdate(const ERobotDriveInputUpdate Update) {
  if (Update == ERobotDriveInputUpdate::Released) {
    Publish(ERobotDrivePublishReason::Release);
  } else if (Update == ERobotDriveInputUpdate::Changed) {
    Publish(ERobotDrivePublishReason::StateChanged);
  }
}

void FLingTuSimRuntimeUIInputProcessor::Publish(const ERobotDrivePublishReason Reason) {
  if (Publisher) {
    Publisher(DriveInput.GetSnapshot(), Reason);
  }
  PublishAccumulatorSeconds = 0.0f;
  InvalidateHUD();
}

void FLingTuSimRuntimeUIInputProcessor::InvalidateHUD() const {
  if (const TSharedPtr<SWidget> Widget = HUDWidget.Pin()) {
    Widget->Invalidate(EInvalidateWidgetReason::Layout | EInvalidateWidgetReason::Paint);
  }
}
}  // namespace LingTuSim::UI

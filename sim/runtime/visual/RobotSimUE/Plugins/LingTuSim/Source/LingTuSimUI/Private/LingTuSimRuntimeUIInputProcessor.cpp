#include "LingTuSimRuntimeUIInputProcessor.h"

#include "Framework/Application/SlateApplication.h"
#include "Framework/Application/SlateUser.h"
#include "Input/Events.h"
#include "LingTuSimRuntimeUIModel.h"
#include "LingTuSimPlayerSettings.h"
#include "SLingTuSimShowroomInput.h"
#include "Widgets/SWidget.h"

namespace LingTuSim::UI {
namespace {
constexpr float PublishIntervalSeconds = 1.0f / 50.0f;
constexpr float MouseLookDegreesPerPixel = 0.16f;

bool IsRuntimeUIKey(const FKey &Key) {
  return Key == EKeys::B || Key == EKeys::Tab || Key == EKeys::Escape ||
         Key == EKeys::Gamepad_FaceButton_Right;
}

bool IsConfirmKey(const FKey &Key) {
  return Key == EKeys::Enter || Key == EKeys::Gamepad_FaceButton_Bottom;
}

bool IsDriveHelpKey(const FKey& Key) {
  return Key == EKeys::F1 || Key == EKeys::Gamepad_Special_Left;
}

bool IsMenuNavigationKey(const FKey &Key) {
  return Key == EKeys::Up || Key == EKeys::Down || Key == EKeys::Left || Key == EKeys::Right ||
         Key == EKeys::Gamepad_DPad_Up || Key == EKeys::Gamepad_DPad_Down ||
         Key == EKeys::Gamepad_DPad_Left || Key == EKeys::Gamepad_DPad_Right ||
         Key == EKeys::Gamepad_LeftShoulder || Key == EKeys::Gamepad_RightShoulder;
}
}  // namespace

FLingTuSimRuntimeUIInputProcessor::FLingTuSimRuntimeUIInputProcessor(
    TSharedRef<FRuntimeUIModeController> InModeController, TWeakPtr<SWidget> InHUDWidget,
    TWeakPtr<SWidget> InViewportWidget, FRobotDriveInputPublisher InPublisher,
    FRuntimeUIActionPublisher InActionPublisher, FRuntimeUIFrontEndLockQuery InFrontEndLockQuery,
    FRuntimeUITextEntryFocusQuery InTextEntryFocusQuery,
    FRuntimeCameraInputPublisher InCameraPublisher, TFunction<bool()> InMenuBackHandler,
    TSharedPtr<FPlayerSettings> InPlayerSettings,
    TFunction<bool(const FKeyEvent&)> InKeyCaptureHandler,
    TFunction<void()> InCancelKeyCapture, TFunction<void()> InRestoreMenuFocus)
    : ModeController(MoveTemp(InModeController)),
      HUDWidget(MoveTemp(InHUDWidget)),
      ViewportWidget(MoveTemp(InViewportWidget)),
      Publisher(MoveTemp(InPublisher)),
      ActionPublisher(MoveTemp(InActionPublisher)),
      FrontEndLockQuery(MoveTemp(InFrontEndLockQuery)),
      TextEntryFocusQuery(MoveTemp(InTextEntryFocusQuery)),
      CameraPublisher(MoveTemp(InCameraPublisher)), MenuBackHandler(MoveTemp(InMenuBackHandler)),
      PlayerSettings(MoveTemp(InPlayerSettings)), KeyCaptureHandler(MoveTemp(InKeyCaptureHandler)),
      CancelKeyCapture(MoveTemp(InCancelKeyCapture)), RestoreMenuFocus(MoveTemp(InRestoreMenuFocus)) {
  if (PlayerSettings.IsValid()) DriveInput.SetKeyBindings(PlayerSettings->DriveKeys);
  PublishedMode = ModeController->GetMode();
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
  // Capture precedes menu shortcuts and action policy, so R cannot start a
  // recording and Escape cancels the binding rather than closing the menu.
  if (ModeController->GetMode() == ERuntimeUIMode::Pause &&
      KeyCaptureHandler && KeyCaptureHandler(KeyEvent)) {
    InvalidateHUD();
    return true;
  }
  if ((Key == EKeys::Escape || Key == EKeys::Gamepad_FaceButton_Right) &&
      MenuBackHandler) {
    if (KeyEvent.IsRepeat()) return true;
    if (MenuBackHandler()) { InvalidateHUD(); return true; }
  }
  ERuntimeUIAction Action = ERuntimeUIAction::ToggleRecording;
  const TSharedPtr<SWidget> FocusedWidget = SlateApplication.GetKeyboardFocusedWidget();
  const TSharedPtr<SWidget> HUD = HUDWidget.Pin();
  if (FrontEndLockQuery && FrontEndLockQuery() && FocusedWidget.IsValid() &&
      FocusedWidget->GetTag() == TEXT("LingTu.Selection.Model") &&
      SLingTuSimShowroomInput::IsViewKey(Key)) return false;
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
  if (IsDriveHelpKey(Key) && !bTeardownReleasePublished && DriveInput.IsEligible() &&
      !(FrontEndLockQuery && FrontEndLockQuery()) && !KeyEvent.IsAltDown() &&
      !KeyEvent.IsControlDown() && !KeyEvent.IsCommandDown()) {
    if (!KeyEvent.IsRepeat() && ModeController->SetDriveHelpVisible(true)) InvalidateHUD();
    return true;
  }
  if (ModeController->GetMode() == ERuntimeUIMode::Pause && IsMenuNavigationKey(Key)) {
    // Focused controls own navigation in both the runtime menu and selector.
    if (FrontEndLockQuery && FrontEndLockQuery() &&
        !(HUD.IsValid() && HUD->HasFocusedDescendants()) && RestoreMenuFocus) RestoreMenuFocus();
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
      if (ModeController->GetMode() != ERuntimeUIMode::Drive) bMouseLooking = false;
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

  if (KeyEvent.IsAltDown() || KeyEvent.IsControlDown() || KeyEvent.IsCommandDown()) return false;
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
  if (IsDriveHelpKey(KeyEvent.GetKey())) {
    if (ModeController->SetDriveHelpVisible(false)) InvalidateHUD();
    return ModeController->GetMode() == ERuntimeUIMode::Drive;
  }
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

bool FLingTuSimRuntimeUIInputProcessor::CanAdjustCamera() const {
  return !bTeardownReleasePublished && DriveInput.IsEligible() && !(FrontEndLockQuery && FrontEndLockQuery()) &&
      !(TextEntryFocusQuery && TextEntryFocusQuery());
}

bool FLingTuSimRuntimeUIInputProcessor::ViewportOwnsMouseCapture(FSlateApplication& App) const {
  const auto Viewport = ViewportWidget.Pin();
  const auto User = App.GetCursorUser();
  return Viewport.IsValid() && User.IsValid() && User->HasCursorCapture() &&
      User->GetCursorCaptorPath().ContainsWidget(Viewport.Get());
}

bool FLingTuSimRuntimeUIInputProcessor::OwnsMousePointer(
    FSlateApplication& App, const FPointerEvent& Event) const {
  const auto Viewport = ViewportWidget.Pin();
  const auto User = App.GetCursorUser();
  if (!Viewport.IsValid() || !User.IsValid()) return false;
  if (User->HasCursorCapture()) return ViewportOwnsMouseCapture(App);
  return App.LocateWindowUnderMouse(Event.GetScreenSpacePosition(),
      App.GetInteractiveTopLevelWindows(), false, Event.GetUserIndex()).ContainsWidget(Viewport.Get());
}

bool FLingTuSimRuntimeUIInputProcessor::HandleMouseButtonDownEvent(
    FSlateApplication& App, const FPointerEvent& Event) {
  PublishForUpdate(SynchronizeContext(App));
  if (!CanAdjustCamera() || !OwnsMousePointer(App, Event)) return false;
  if (Event.GetEffectingButton() == EKeys::RightMouseButton && CameraPublisher) {
    bMouseLooking = true;
    // The first uncaptured click must still reach SViewport's normal capture path.
    return ViewportOwnsMouseCapture(App);
  }
  if (Event.GetEffectingButton() == EKeys::MiddleMouseButton && ActionPublisher) {
    bMouseLooking = false;
    ActionPublisher(ERuntimeUIAction::CycleCamera);
    InvalidateHUD();
    return true;
  }
  return false;
}

bool FLingTuSimRuntimeUIInputProcessor::HandleMouseButtonDoubleClickEvent(
    FSlateApplication& App, const FPointerEvent& Event) {
  return HandleMouseButtonDownEvent(App, Event);
}

bool FLingTuSimRuntimeUIInputProcessor::HandleMouseButtonUpEvent(
    FSlateApplication& App, const FPointerEvent& Event) {
  const bool bWasLooking = bMouseLooking;
  PublishForUpdate(SynchronizeContext(App));
  if (Event.GetEffectingButton() == EKeys::RightMouseButton) {
    bMouseLooking = false;
    return bWasLooking && CanAdjustCamera() && ViewportOwnsMouseCapture(App);
  }
  return false;
}

bool FLingTuSimRuntimeUIInputProcessor::HandleMouseMoveEvent(
    FSlateApplication& App, const FPointerEvent& Event) {
  PublishForUpdate(SynchronizeContext(App));
  if (!CanAdjustCamera() || !bMouseLooking || !CameraPublisher || !ViewportOwnsMouseCapture(App)) {
    bMouseLooking = false;
    return false;
  }
  if (!Event.IsMouseButtonDown(EKeys::RightMouseButton)) {
    bMouseLooking = false;
    return false;
  }
  const FVector2D Delta = Event.GetCursorDelta();
  if (!FMath::IsFinite(Delta.X) || !FMath::IsFinite(Delta.Y)) return true;
  FRuntimeCameraInput Input;
  Input.YawDeltaDegrees = FMath::Clamp(static_cast<float>(Delta.X) * MouseLookDegreesPerPixel, -90.0f, 90.0f);
  Input.PitchDeltaDegrees = FMath::Clamp(static_cast<float>(-Delta.Y) * MouseLookDegreesPerPixel, -90.0f, 90.0f);
  if (!FMath::IsNearlyZero(Input.YawDeltaDegrees) || !FMath::IsNearlyZero(Input.PitchDeltaDegrees))
    CameraPublisher(Input);
  return true;
}

bool FLingTuSimRuntimeUIInputProcessor::HandleMouseWheelOrGestureEvent(
    FSlateApplication& App, const FPointerEvent& Event, const FPointerEvent* Gesture) {
  PublishForUpdate(SynchronizeContext(App));
  if (Gesture != nullptr || !CanAdjustCamera() || !CameraPublisher || !OwnsMousePointer(App, Event)) return false;
  const float Wheel = Event.GetWheelDelta();
  if (!FMath::IsFinite(Wheel) || FMath::IsNearlyZero(Wheel)) return false;
  FRuntimeCameraInput Input;
  Input.ZoomSteps = FMath::Clamp(Wheel, -5.0f, 5.0f);
  CameraPublisher(Input);
  return true;
}

void FLingTuSimRuntimeUIInputProcessor::ReleaseInput() {
  if (bTeardownReleasePublished) {
    return;
  }
  bTeardownReleasePublished = true;
  ModeController->SetDriveHelpVisible(false);
  bMouseLooking = false;
  if (CancelKeyCapture) CancelKeyCapture();
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
  const bool bMenuOwnsFocus = (ModeController->GetMode() == ERuntimeUIMode::Pause ||
                              (TextEntryFocusQuery && TextEntryFocusQuery())) && HUD.IsValid() &&
                              (HUD->HasKeyboardFocus() || HUD->HasFocusedDescendants() ||
                               HUD->HasUserFocus(0).IsSet() || HUD->HasUserFocusedDescendants(0));
  const bool bViewportFocused =
      SlateApplication.IsActive() && (bViewportOwnsFocus || bMenuOwnsFocus);
  const ERobotDriveInputUpdate FocusUpdate = DriveInput.SetViewportFocused(bViewportFocused);
  const ERobotDriveInputUpdate BindingUpdate = PlayerSettings.IsValid()
      ? DriveInput.SetKeyBindings(PlayerSettings->DriveKeys) : ERobotDriveInputUpdate::Handled;
  if ((!bViewportFocused || ModeController->GetMode() != ERuntimeUIMode::Pause) && CancelKeyCapture) CancelKeyCapture();
  if ((!DriveInput.IsEligible() || (FrontEndLockQuery && FrontEndLockQuery()) ||
       (TextEntryFocusQuery && TextEntryFocusQuery())) && ModeController->SetDriveHelpVisible(false)) InvalidateHUD();
  if (!CanAdjustCamera() || !ViewportOwnsMouseCapture(SlateApplication)) bMouseLooking = false;
  if (ModeUpdate == ERobotDriveInputUpdate::Released ||
      FocusUpdate == ERobotDriveInputUpdate::Released || BindingUpdate == ERobotDriveInputUpdate::Released) {
    return ERobotDriveInputUpdate::Released;
  }
  if (ModeController->GetMode() != PublishedMode || ModeUpdate == ERobotDriveInputUpdate::Changed ||
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
  PublishedMode = ModeController->GetMode();
  if (Publisher) {
    Publisher(DriveInput.GetSnapshot(), Reason);
  }
  if (Reason != ERobotDrivePublishReason::Periodic) PublishAccumulatorSeconds = 0.0f;
  InvalidateHUD();
}

void FLingTuSimRuntimeUIInputProcessor::InvalidateHUD() const {
  if (const TSharedPtr<SWidget> Widget = HUDWidget.Pin()) {
    Widget->Invalidate(EInvalidateWidgetReason::Layout | EInvalidateWidgetReason::Paint);
  }
}
}  // namespace LingTuSim::UI

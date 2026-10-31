#pragma once

#include "Framework/Application/IInputProcessor.h"
#include "LingTuSimRobotDriveInput.h"
#include "LingTuSimRuntimeUIModel.h"
#include "Templates/Function.h"

class SWidget;

namespace LingTuSim::UI {
struct FPlayerSettings;
enum class ERobotDrivePublishReason : uint8 {
  StateChanged,
  Periodic,
  Release,
};

using FRobotDriveInputPublisher =
    TFunction<void(const FRobotDriveInputSnapshot &, ERobotDrivePublishReason)>;
using FRuntimeUIActionPublisher = TFunction<void(ERuntimeUIAction)>;
using FRuntimeUIFrontEndLockQuery = TFunction<bool()>;
using FRuntimeUITextEntryFocusQuery = TFunction<bool()>;

/** Local player-view deltas only; these are never robot motion intent. */
struct FRuntimeCameraInput {
  float YawDeltaDegrees = 0.0f;
  float PitchDeltaDegrees = 0.0f;
  float ZoomSteps = 0.0f;
};
using FRuntimeCameraInputPublisher = TFunction<void(const FRuntimeCameraInput&)>;

/** Viewport-scoped input adapter. Transport ownership remains in Session. */
class FLingTuSimRuntimeUIInputProcessor final : public IInputProcessor {
 public:
  FLingTuSimRuntimeUIInputProcessor(TSharedRef<FRuntimeUIModeController> InModeController,
                                    TWeakPtr<SWidget> InHUDWidget,
                                    TWeakPtr<SWidget> InViewportWidget,
                                    FRobotDriveInputPublisher InPublisher,
                                    FRuntimeUIActionPublisher InActionPublisher,
                                    FRuntimeUIFrontEndLockQuery InFrontEndLockQuery,
                                    FRuntimeUITextEntryFocusQuery InTextEntryFocusQuery,
                                    FRuntimeCameraInputPublisher InCameraPublisher = {},
                                    TFunction<bool()> InMenuBackHandler = {},
                                    TSharedPtr<FPlayerSettings> InPlayerSettings = {},
                                    TFunction<bool(const FKeyEvent&)> InKeyCaptureHandler = {},
                                    TFunction<void()> InCancelKeyCapture = {},
                                    TFunction<void()> InRestoreMenuFocus = {});
  virtual ~FLingTuSimRuntimeUIInputProcessor() override;

  virtual void Tick(float DeltaTime, FSlateApplication &SlateApplication,
                    TSharedRef<ICursor> Cursor) override;
  virtual bool HandleKeyDownEvent(FSlateApplication &SlateApplication,
                                  const FKeyEvent &KeyEvent) override;
  virtual bool HandleKeyUpEvent(FSlateApplication &SlateApplication,
                                const FKeyEvent &KeyEvent) override;
  virtual bool HandleAnalogInputEvent(FSlateApplication &SlateApplication,
                                      const FAnalogInputEvent &AnalogInputEvent) override;
  virtual bool HandleMouseMoveEvent(FSlateApplication& SlateApplication, const FPointerEvent& Event) override;
  virtual bool HandleMouseButtonDownEvent(FSlateApplication& SlateApplication, const FPointerEvent& Event) override;
  virtual bool HandleMouseButtonUpEvent(FSlateApplication& SlateApplication, const FPointerEvent& Event) override;
  virtual bool HandleMouseButtonDoubleClickEvent(FSlateApplication& SlateApplication, const FPointerEvent& Event) override;
  virtual bool HandleMouseWheelOrGestureEvent(FSlateApplication& SlateApplication, const FPointerEvent& Event,
                                            const FPointerEvent* Gesture) override;

  void ReleaseInput();

 private:
  ERobotDriveInputUpdate SynchronizeContext(FSlateApplication &SlateApplication);
  void PublishForUpdate(ERobotDriveInputUpdate Update);
  void Publish(ERobotDrivePublishReason Reason);
  void InvalidateHUD() const;
  bool CanAdjustCamera() const;
  bool ViewportOwnsMouseCapture(FSlateApplication& App) const;
  bool OwnsMousePointer(FSlateApplication& App, const FPointerEvent& Event) const;

  TSharedRef<FRuntimeUIModeController> ModeController;
  TWeakPtr<SWidget> HUDWidget;
  TWeakPtr<SWidget> ViewportWidget;
  FRobotDriveInputPublisher Publisher;
  FRuntimeUIActionPublisher ActionPublisher;
  FRuntimeUIFrontEndLockQuery FrontEndLockQuery;
  FRuntimeUITextEntryFocusQuery TextEntryFocusQuery;
  FRuntimeCameraInputPublisher CameraPublisher;
  TFunction<bool()> MenuBackHandler;
  TSharedPtr<FPlayerSettings> PlayerSettings;
  TFunction<bool(const FKeyEvent&)> KeyCaptureHandler;
  TFunction<void()> CancelKeyCapture;
  TFunction<void()> RestoreMenuFocus;
  FRobotDriveInputState DriveInput;
  ERuntimeUIMode PublishedMode;
  float PublishAccumulatorSeconds = 0.0f;
  bool bTeardownReleasePublished = false;
  bool bMouseLooking = false;
};
}  // namespace LingTuSim::UI

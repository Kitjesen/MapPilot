#pragma once

#include "Framework/Application/IInputProcessor.h"
#include "LingTuSimRobotDriveInput.h"
#include "LingTuSimRuntimeUIModel.h"
#include "Templates/Function.h"

class SWidget;

namespace LingTuSim::UI {
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

/** Viewport-scoped input adapter. Transport ownership remains in Session. */
class FLingTuSimRuntimeUIInputProcessor final : public IInputProcessor {
 public:
  FLingTuSimRuntimeUIInputProcessor(TSharedRef<FRuntimeUIModeController> InModeController,
                                    TWeakPtr<SWidget> InHUDWidget,
                                    TWeakPtr<SWidget> InViewportWidget,
                                    FRobotDriveInputPublisher InPublisher,
                                    FRuntimeUIActionPublisher InActionPublisher,
                                    FRuntimeUIFrontEndLockQuery InFrontEndLockQuery,
                                    FRuntimeUITextEntryFocusQuery InTextEntryFocusQuery);
  virtual ~FLingTuSimRuntimeUIInputProcessor() override;

  virtual void Tick(float DeltaTime, FSlateApplication &SlateApplication,
                    TSharedRef<ICursor> Cursor) override;
  virtual bool HandleKeyDownEvent(FSlateApplication &SlateApplication,
                                  const FKeyEvent &KeyEvent) override;
  virtual bool HandleKeyUpEvent(FSlateApplication &SlateApplication,
                                const FKeyEvent &KeyEvent) override;
  virtual bool HandleAnalogInputEvent(FSlateApplication &SlateApplication,
                                      const FAnalogInputEvent &AnalogInputEvent) override;

  void ReleaseInput();

 private:
  ERobotDriveInputUpdate SynchronizeContext(FSlateApplication &SlateApplication);
  void PublishForUpdate(ERobotDriveInputUpdate Update);
  void Publish(ERobotDrivePublishReason Reason);
  void InvalidateHUD() const;

  TSharedRef<FRuntimeUIModeController> ModeController;
  TWeakPtr<SWidget> HUDWidget;
  TWeakPtr<SWidget> ViewportWidget;
  FRobotDriveInputPublisher Publisher;
  FRuntimeUIActionPublisher ActionPublisher;
  FRuntimeUIFrontEndLockQuery FrontEndLockQuery;
  FRuntimeUITextEntryFocusQuery TextEntryFocusQuery;
  FRobotDriveInputState DriveInput;
  float PublishAccumulatorSeconds = 0.0f;
  bool bTeardownReleasePublished = false;
};
}  // namespace LingTuSim::UI

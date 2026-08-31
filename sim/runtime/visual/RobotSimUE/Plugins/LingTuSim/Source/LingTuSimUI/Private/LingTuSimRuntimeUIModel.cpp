#include "LingTuSimRuntimeUIModel.h"

namespace LingTuSim::UI {
const TCHAR *RuntimeUIModeWireName(const ERuntimeUIMode Mode) {
  switch (Mode) {
    case ERuntimeUIMode::Build:
      return TEXT("build");
    case ERuntimeUIMode::Tactical:
      return TEXT("tactical");
    case ERuntimeUIMode::Pause:
      return TEXT("menu");
    case ERuntimeUIMode::Drive:
    default:
      return TEXT("drive");
  }
}

bool FRuntimeUIActionPolicy::ResolveKey(const ERuntimeUIMode Mode, const FKey &Key,
                                        ERuntimeUIAction &OutAction) {
  if (Mode == ERuntimeUIMode::Pause && (Key == EKeys::Up || Key == EKeys::Gamepad_DPad_Up)) {
    OutAction = ERuntimeUIAction::SelectPreviousGame;
    return true;
  }
  if (Mode == ERuntimeUIMode::Pause && (Key == EKeys::Down || Key == EKeys::Gamepad_DPad_Down)) {
    OutAction = ERuntimeUIAction::SelectNextGame;
    return true;
  }
  if (Mode == ERuntimeUIMode::Pause && (Key == EKeys::Left || Key == EKeys::Gamepad_DPad_Left ||
                                        Key == EKeys::Gamepad_LeftShoulder)) {
    OutAction = ERuntimeUIAction::SelectPreviousAsset;
    return true;
  }
  if (Mode == ERuntimeUIMode::Pause && (Key == EKeys::Right || Key == EKeys::Gamepad_DPad_Right ||
                                        Key == EKeys::Gamepad_RightShoulder)) {
    OutAction = ERuntimeUIAction::SelectNextAsset;
    return true;
  }
  if (Mode == ERuntimeUIMode::Pause &&
      (Key == EKeys::Enter || Key == EKeys::Gamepad_FaceButton_Bottom)) {
    OutAction = ERuntimeUIAction::ConfirmGameSelection;
    return true;
  }
  if (Mode != ERuntimeUIMode::Pause && Key == EKeys::C) {
    OutAction = ERuntimeUIAction::CycleCamera;
    return true;
  }
  if (Key == EKeys::R) {
    OutAction = ERuntimeUIAction::ToggleRecording;
    return true;
  }
  if (Mode == ERuntimeUIMode::Pause && Key == EKeys::X) {
    OutAction = ERuntimeUIAction::Exit;
    return true;
  }
  return false;
}

bool FRuntimeUIModeController::HandleKey(const FKey &Key) {
  if (Key == EKeys::B) {
    ToggleBuild();
    return true;
  }
  if (Key == EKeys::Tab) {
    ToggleTactical();
    return true;
  }
  if (Key == EKeys::Escape || Key == EKeys::Gamepad_FaceButton_Right) {
    TogglePause();
    return true;
  }
  return false;
}

void FRuntimeUIModeController::ToggleBuild() {
  ToggleExclusive(ERuntimeUIMode::Build);
}

void FRuntimeUIModeController::ToggleTactical() {
  ToggleExclusive(ERuntimeUIMode::Tactical);
}

void FRuntimeUIModeController::TogglePause() {
  if (Mode == ERuntimeUIMode::Pause) {
    Mode = ModeBeforePause;
    return;
  }
  ModeBeforePause = Mode;
  Mode = ERuntimeUIMode::Pause;
}

void FRuntimeUIModeController::ToggleExclusive(const ERuntimeUIMode RequestedMode) {
  Mode = Mode == RequestedMode ? ERuntimeUIMode::Drive : RequestedMode;
}
}  // namespace LingTuSim::UI

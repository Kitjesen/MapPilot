#include "LingTuSimRobotDriveInput.h"

namespace LingTuSim::UI {
namespace {
bool SnapshotsMatch(const FRobotDriveInputSnapshot &Left, const FRobotDriveInputSnapshot &Right) {
  return Left.InputDevice == Right.InputDevice && Left.bDriveMode == Right.bDriveMode &&
         Left.bViewportFocused == Right.bViewportFocused && Left.bDeadman == Right.bDeadman &&
         FMath::IsNearlyEqual(Left.Forward, Right.Forward) &&
         FMath::IsNearlyEqual(Left.Left, Right.Left) &&
         FMath::IsNearlyEqual(Left.YawLeft, Right.YawLeft) &&
         FMath::IsNearlyEqual(Left.CameraYaw, Right.CameraYaw) &&
         FMath::IsNearlyEqual(Left.CameraPitch, Right.CameraPitch) &&
         Left.ActiveControls == Right.ActiveControls;
}
}  // namespace

ERobotDriveInputUpdate FRobotDriveInputState::SetDriveMode(const bool bInDriveMode) {
  if (bDriveMode == bInDriveMode) {
    return ERobotDriveInputUpdate::Handled;
  }

  bDriveMode = bInDriveMode;
  bRelease = false;
  if (!bDriveMode) {
    ClearPhysicalState();
    bRelease = true;
    return ERobotDriveInputUpdate::Released;
  }
  return ERobotDriveInputUpdate::Changed;
}

ERobotDriveInputUpdate FRobotDriveInputState::SetViewportFocused(const bool bInViewportFocused) {
  if (bViewportFocused == bInViewportFocused) {
    return ERobotDriveInputUpdate::Handled;
  }

  bViewportFocused = bInViewportFocused;
  bRelease = false;
  if (!bViewportFocused) {
    ClearPhysicalState();
    bRelease = true;
    return ERobotDriveInputUpdate::Released;
  }
  return ERobotDriveInputUpdate::Changed;
}

ERobotDriveInputUpdate FRobotDriveInputState::HandleKeyDown(const FKey &Key, const bool bIsRepeat) {
  if (!IsMappedKey(Key)) {
    return ERobotDriveInputUpdate::Unhandled;
  }
  if (bIsRepeat || !IsEligible()) {
    return ERobotDriveInputUpdate::Handled;
  }
  return SetMappedKey(Key, true);
}

ERobotDriveInputUpdate FRobotDriveInputState::HandleKeyUp(const FKey &Key) {
  if (!IsMappedKey(Key)) {
    return ERobotDriveInputUpdate::Unhandled;
  }
  if (!IsEligible()) {
    return ERobotDriveInputUpdate::Handled;
  }
  return SetMappedKey(Key, false);
}

ERobotDriveInputUpdate FRobotDriveInputState::HandleAnalog(const FKey &Key, const float Value) {
  if (!IsMappedAnalog(Key)) {
    return ERobotDriveInputUpdate::Unhandled;
  }
  const bool bCameraAxis = Key == EKeys::Gamepad_RightX || Key == EKeys::Gamepad_RightY;
  if (!bViewportFocused || (!bDriveMode && !bCameraAxis)) {
    return ERobotDriveInputUpdate::Handled;
  }

  const FRobotDriveInputSnapshot Before = GetSnapshot();
  const float SafeValue = FMath::IsFinite(Value) ? FMath::Clamp(Value, -1.0f, 1.0f) : 0.0f;
  if (Key == EKeys::Gamepad_LeftX) {
    GamepadLeftX = SafeValue;
  } else if (Key == EKeys::Gamepad_LeftY) {
    GamepadLeftY = SafeValue;
  } else if (Key == EKeys::Gamepad_LeftTriggerAxis) {
    GamepadLeftTrigger = FMath::Clamp(SafeValue, 0.0f, 1.0f);
  } else if (Key == EKeys::Gamepad_RightTriggerAxis) {
    GamepadRightTrigger = FMath::Clamp(SafeValue, 0.0f, 1.0f);
  } else if (Key == EKeys::Gamepad_RightX) {
    GamepadRightX = SafeValue;
  } else if (Key == EKeys::Gamepad_RightY) {
    GamepadRightY = SafeValue;
  }
  return FinishMutation(Before, ERobotDriveInputDevice::Gamepad);
}

ERobotDriveInputUpdate FRobotDriveInputState::ReleaseAll() {
  ClearPhysicalState();
  bViewportFocused = false;
  bRelease = true;
  return ERobotDriveInputUpdate::Released;
}

FRobotDriveInputSnapshot FRobotDriveInputState::GetSnapshot() const {
  FRobotDriveInputSnapshot Snapshot;
  Snapshot.InputDevice = LastInputDevice;
  Snapshot.bDriveMode = bDriveMode;
  Snapshot.bViewportFocused = bViewportFocused;
  Snapshot.bRelease = bRelease;

  if (!bViewportFocused) {
    return Snapshot;
  }

  const FVector2D RightStick = ApplyRadialDeadZone(GamepadRightX, GamepadRightY);
  if (!RightStick.IsNearlyZero()) {
    Snapshot.ActiveControls.Add(TEXT("gamepad.right_stick"));
  }
  Snapshot.CameraYaw = static_cast<float>(RightStick.X);
  Snapshot.CameraPitch = static_cast<float>(RightStick.Y);
  if (!bDriveMode) {
    return Snapshot;
  }

  Snapshot.bDeadman = bKeyboardDeadman || bGamepadDeadman;
  if (bKeyboardDeadman) {
    Snapshot.ActiveControls.Add(TEXT("keyboard.left_shift"));
  }
  if (bW) {
    Snapshot.ActiveControls.Add(TEXT("keyboard.w"));
  }
  if (bS) {
    Snapshot.ActiveControls.Add(TEXT("keyboard.s"));
  }
  if (bA) {
    Snapshot.ActiveControls.Add(TEXT("keyboard.a"));
  }
  if (bD) {
    Snapshot.ActiveControls.Add(TEXT("keyboard.d"));
  }
  if (bQ) {
    Snapshot.ActiveControls.Add(TEXT("keyboard.q"));
  }
  if (bE) {
    Snapshot.ActiveControls.Add(TEXT("keyboard.e"));
  }
  if (bGamepadDeadman) {
    Snapshot.ActiveControls.Add(TEXT("gamepad.left_shoulder"));
  }

  const FVector2D LeftStick = ApplyRadialDeadZone(GamepadLeftX, GamepadLeftY);
  const float LeftTrigger = ApplyScalarDeadZone(GamepadLeftTrigger);
  const float RightTrigger = ApplyScalarDeadZone(GamepadRightTrigger);

  if (!LeftStick.IsNearlyZero()) {
    Snapshot.ActiveControls.Add(TEXT("gamepad.left_stick"));
  }
  if (!FMath::IsNearlyZero(LeftTrigger)) {
    Snapshot.ActiveControls.Add(TEXT("gamepad.left_trigger"));
  }
  if (!FMath::IsNearlyZero(RightTrigger)) {
    Snapshot.ActiveControls.Add(TEXT("gamepad.right_trigger"));
  }
  if (Snapshot.bDeadman) {
    const float KeyboardForward = static_cast<float>(bW) - static_cast<float>(bS);
    const float KeyboardLeft = static_cast<float>(bA) - static_cast<float>(bD);
    const float KeyboardYawLeft = static_cast<float>(bQ) - static_cast<float>(bE);
    Snapshot.Forward = FMath::Clamp(KeyboardForward + static_cast<float>(LeftStick.Y), -1.0f, 1.0f);
    Snapshot.Left = FMath::Clamp(KeyboardLeft - static_cast<float>(LeftStick.X), -1.0f, 1.0f);
    Snapshot.YawLeft = FMath::Clamp(KeyboardYawLeft + LeftTrigger - RightTrigger, -1.0f, 1.0f);
  }
  return Snapshot;
}

bool FRobotDriveInputState::IsMappedKey(const FKey &Key) {
  return Key == EKeys::W || Key == EKeys::S || Key == EKeys::A || Key == EKeys::D ||
         Key == EKeys::Q || Key == EKeys::E || Key == EKeys::LeftShift ||
         Key == EKeys::Gamepad_LeftShoulder;
}

bool FRobotDriveInputState::IsMappedAnalog(const FKey &Key) {
  return Key == EKeys::Gamepad_LeftX || Key == EKeys::Gamepad_LeftY ||
         Key == EKeys::Gamepad_LeftTriggerAxis || Key == EKeys::Gamepad_RightTriggerAxis ||
         Key == EKeys::Gamepad_RightX || Key == EKeys::Gamepad_RightY;
}

const TCHAR *FRobotDriveInputState::InputDeviceName(const ERobotDriveInputDevice Device) {
  switch (Device) {
    case ERobotDriveInputDevice::Keyboard:
      return TEXT("keyboard");
    case ERobotDriveInputDevice::Gamepad:
      return TEXT("gamepad");
    default:
      return TEXT("unknown");
  }
}

ERobotDriveInputUpdate FRobotDriveInputState::SetMappedKey(const FKey &Key, const bool bPressed) {
  const FRobotDriveInputSnapshot Before = GetSnapshot();
  ERobotDriveInputDevice Device = ERobotDriveInputDevice::Keyboard;
  if (Key == EKeys::W) {
    bW = bPressed;
  } else if (Key == EKeys::S) {
    bS = bPressed;
  } else if (Key == EKeys::A) {
    bA = bPressed;
  } else if (Key == EKeys::D) {
    bD = bPressed;
  } else if (Key == EKeys::Q) {
    bQ = bPressed;
  } else if (Key == EKeys::E) {
    bE = bPressed;
  } else if (Key == EKeys::LeftShift) {
    bKeyboardDeadman = bPressed;
  } else if (Key == EKeys::Gamepad_LeftShoulder) {
    bGamepadDeadman = bPressed;
    Device = ERobotDriveInputDevice::Gamepad;
  }
  return FinishMutation(Before, Device);
}

ERobotDriveInputUpdate FRobotDriveInputState::FinishMutation(const FRobotDriveInputSnapshot &Before,
                                                             const ERobotDriveInputDevice Device) {
  LastInputDevice = Device;
  bRelease = false;
  const FRobotDriveInputSnapshot After = GetSnapshot();
  if (Before.bDeadman && !After.bDeadman) {
    bRelease = true;
    return ERobotDriveInputUpdate::Released;
  }
  return SnapshotsMatch(Before, After) ? ERobotDriveInputUpdate::Handled
                                       : ERobotDriveInputUpdate::Changed;
}

void FRobotDriveInputState::ClearPhysicalState() {
  bW = false;
  bS = false;
  bA = false;
  bD = false;
  bQ = false;
  bE = false;
  bKeyboardDeadman = false;
  bGamepadDeadman = false;
  GamepadLeftX = 0.0f;
  GamepadLeftY = 0.0f;
  GamepadLeftTrigger = 0.0f;
  GamepadRightTrigger = 0.0f;
  GamepadRightX = 0.0f;
  GamepadRightY = 0.0f;
}

FVector2D FRobotDriveInputState::ApplyRadialDeadZone(const float X, const float Y) {
  const FVector2D Clamped(FMath::Clamp(X, -1.0f, 1.0f), FMath::Clamp(Y, -1.0f, 1.0f));
  const double Magnitude = Clamped.Size();
  if (!FMath::IsFinite(Magnitude) || Magnitude <= RobotDriveGamepadDeadZone) {
    return FVector2D::ZeroVector;
  }

  const double ClampedMagnitude = FMath::Min(Magnitude, 1.0);
  const double RemappedMagnitude =
      (ClampedMagnitude - RobotDriveGamepadDeadZone) / (1.0 - RobotDriveGamepadDeadZone);
  return Clamped.GetSafeNormal() * RemappedMagnitude;
}

float FRobotDriveInputState::ApplyScalarDeadZone(const float Value) {
  const float Clamped = FMath::Clamp(Value, 0.0f, 1.0f);
  if (Clamped <= RobotDriveGamepadDeadZone) {
    return 0.0f;
  }
  return (Clamped - RobotDriveGamepadDeadZone) / (1.0f - RobotDriveGamepadDeadZone);
}
}  // namespace LingTuSim::UI

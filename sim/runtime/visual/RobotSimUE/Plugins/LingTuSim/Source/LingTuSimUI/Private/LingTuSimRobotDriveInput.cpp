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

FString KeyboardControlName(const FKey &Key) {
  FString KeyName = Key.GetFName().ToString().ToLower();
  const FKey DigitKeys[] = {EKeys::Zero, EKeys::One, EKeys::Two,   EKeys::Three, EKeys::Four,
                            EKeys::Five, EKeys::Six, EKeys::Seven, EKeys::Eight, EKeys::Nine};
  for (int32 Index = 0; Index < UE_ARRAY_COUNT(DigitKeys); ++Index) {
    if (Key == DigitKeys[Index]) {
      KeyName = FString::FromInt(Index);
      break;
    }
  }
  return FString::Printf(TEXT("keyboard.%s"), *KeyName);
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

ERobotDriveInputUpdate FRobotDriveInputState::SetKeyBindings(
    const FDriveKeyBindings &InKeyBindings) {
  FDriveKeyBindings Normalized = InKeyBindings;
  Normalized.Normalize();
  if (KeyBindings == Normalized) {
    return ERobotDriveInputUpdate::Handled;
  }
  KeyBindings = Normalized;
  ClearPhysicalState();
  bRelease = true;
  return ERobotDriveInputUpdate::Released;
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

  // Keyboard movement itself is the live permit; gamepad axes still require LB.
  Snapshot.bDeadman = bForward || bBackward || bLeft || bRight || bTurnLeft || bTurnRight ||
                      bGamepadDeadman;
  if (bForward) {
    Snapshot.ActiveControls.Add(KeyboardControlName(KeyBindings.GetKey(EDriveKeyAction::Forward)));
  }
  if (bBackward) {
    Snapshot.ActiveControls.Add(KeyboardControlName(KeyBindings.GetKey(EDriveKeyAction::Backward)));
  }
  if (bLeft) {
    Snapshot.ActiveControls.Add(KeyboardControlName(KeyBindings.GetKey(EDriveKeyAction::Left)));
  }
  if (bRight) {
    Snapshot.ActiveControls.Add(KeyboardControlName(KeyBindings.GetKey(EDriveKeyAction::Right)));
  }
  if (bTurnLeft) {
    Snapshot.ActiveControls.Add(KeyboardControlName(KeyBindings.GetKey(EDriveKeyAction::TurnLeft)));
  }
  if (bTurnRight) {
    Snapshot.ActiveControls.Add(KeyboardControlName(KeyBindings.GetKey(EDriveKeyAction::TurnRight)));
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
    const float KeyboardForward = static_cast<float>(bForward) - static_cast<float>(bBackward);
    const float KeyboardLeft = static_cast<float>(bLeft) - static_cast<float>(bRight);
    const float KeyboardYawLeft = static_cast<float>(bTurnLeft) - static_cast<float>(bTurnRight);
    const float GamepadForward = bGamepadDeadman ? static_cast<float>(LeftStick.Y) : 0.0f;
    const float GamepadLeft = bGamepadDeadman ? -static_cast<float>(LeftStick.X) : 0.0f;
    const float GamepadYawLeft = bGamepadDeadman ? LeftTrigger - RightTrigger : 0.0f;
    Snapshot.Forward = FMath::Clamp(KeyboardForward + GamepadForward, -1.0f, 1.0f);
    Snapshot.Left = FMath::Clamp(KeyboardLeft + GamepadLeft, -1.0f, 1.0f);
    Snapshot.YawLeft = FMath::Clamp(KeyboardYawLeft + GamepadYawLeft, -1.0f, 1.0f);
  }
  return Snapshot;
}

bool FRobotDriveInputState::IsMappedKey(const FKey &Key) const {
  if (Key == EKeys::Gamepad_LeftShoulder) {
    return true;
  }
  for (uint8 Index = 0; Index < static_cast<uint8>(EDriveKeyAction::Count); ++Index) {
    if (Key == KeyBindings.GetKey(static_cast<EDriveKeyAction>(Index))) {
      return true;
    }
  }
  return false;
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
  if (Key == KeyBindings.GetKey(EDriveKeyAction::Forward)) {
    bForward = bPressed;
  } else if (Key == KeyBindings.GetKey(EDriveKeyAction::Backward)) {
    bBackward = bPressed;
  } else if (Key == KeyBindings.GetKey(EDriveKeyAction::Left)) {
    bLeft = bPressed;
  } else if (Key == KeyBindings.GetKey(EDriveKeyAction::Right)) {
    bRight = bPressed;
  } else if (Key == KeyBindings.GetKey(EDriveKeyAction::TurnLeft)) {
    bTurnLeft = bPressed;
  } else if (Key == KeyBindings.GetKey(EDriveKeyAction::TurnRight)) {
    bTurnRight = bPressed;
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
  bForward = false;
  bBackward = false;
  bLeft = false;
  bRight = false;
  bTurnLeft = false;
  bTurnRight = false;
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

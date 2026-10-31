#pragma once

#include "CoreMinimal.h"
#include "InputCoreTypes.h"
#include "LingTuSimDriveKeyBindings.h"

namespace LingTuSim::UI {
constexpr float RobotDriveGamepadDeadZone = 0.15f;

enum class ERobotDriveInputDevice : uint8 {
  Unknown,
  Keyboard,
  Gamepad,
};

enum class ERobotDriveInputUpdate : uint8 {
  Unhandled,
  Handled,
  Changed,
  Released,
};

/**
 * Normalized, local operator input. Robot axes and camera axes are deliberately
 * separate; this value is intent, never accepted control or observed truth.
 */
struct LINGTUSIMUI_API FRobotDriveInputSnapshot final {
  ERobotDriveInputDevice InputDevice = ERobotDriveInputDevice::Unknown;
  bool bDriveMode = false;
  bool bViewportFocused = false;
  // Live keyboard movement or the gamepad LB permit; retained at the intent boundary.
  bool bDeadman = false;
  bool bRelease = false;
  float Forward = 0.0f;
  float Left = 0.0f;
  float YawLeft = 0.0f;
  float CameraYaw = 0.0f;
  float CameraPitch = 0.0f;
  TArray<FString> ActiveControls;
};

/** Pure, deterministic input mapping and safety gate used by Slate and tests. */
class LINGTUSIMUI_API FRobotDriveInputState final {
 public:
  ERobotDriveInputUpdate SetDriveMode(bool bInDriveMode);
  ERobotDriveInputUpdate SetViewportFocused(bool bInViewportFocused);
  ERobotDriveInputUpdate SetKeyBindings(const FDriveKeyBindings &InKeyBindings);
  ERobotDriveInputUpdate HandleKeyDown(const FKey &Key, bool bIsRepeat);
  ERobotDriveInputUpdate HandleKeyUp(const FKey &Key);
  ERobotDriveInputUpdate HandleAnalog(const FKey &Key, float Value);
  ERobotDriveInputUpdate ReleaseAll();

  FRobotDriveInputSnapshot GetSnapshot() const;
  bool IsEligible() const { return bDriveMode && bViewportFocused; }

  bool IsMappedKey(const FKey &Key) const;
  static bool IsMappedAnalog(const FKey &Key);
  static const TCHAR *InputDeviceName(ERobotDriveInputDevice Device);

 private:
  ERobotDriveInputUpdate SetMappedKey(const FKey &Key, bool bPressed);
  ERobotDriveInputUpdate FinishMutation(const FRobotDriveInputSnapshot &Before,
                                        ERobotDriveInputDevice Device);
  void ClearPhysicalState();
  static FVector2D ApplyRadialDeadZone(float X, float Y);
  static float ApplyScalarDeadZone(float Value);

  FDriveKeyBindings KeyBindings;
  bool bDriveMode = false;
  bool bViewportFocused = false;
  bool bRelease = false;
  ERobotDriveInputDevice LastInputDevice = ERobotDriveInputDevice::Unknown;

  bool bForward = false;
  bool bBackward = false;
  bool bLeft = false;
  bool bRight = false;
  bool bTurnLeft = false;
  bool bTurnRight = false;
  bool bGamepadDeadman = false;

  float GamepadLeftX = 0.0f;
  float GamepadLeftY = 0.0f;
  float GamepadLeftTrigger = 0.0f;
  float GamepadRightTrigger = 0.0f;
  float GamepadRightX = 0.0f;
  float GamepadRightY = 0.0f;
};
}  // namespace LingTuSim::UI

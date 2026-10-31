#pragma once

#include "CoreMinimal.h"
#include "InputCoreTypes.h"

namespace LingTuSim::UI {
enum class EDriveKeyAction : uint8 {
  Forward,
  Backward,
  Left,
  Right,
  TurnLeft,
  TurnRight,
  Count,
};

struct FPlayerSettings;

struct LINGTUSIMUI_API FDriveKeyBindings final {
  FKey GetKey(EDriveKeyAction Action) const;
  bool TryBind(EDriveKeyAction Action, const FKey &Key, FString &Error);
  void Normalize();

  bool operator==(const FDriveKeyBindings &Other) const;
  bool operator!=(const FDriveKeyBindings &Other) const { return !(*this == Other); }

  static const TCHAR *ActionLabel(EDriveKeyAction Action);
  static const TCHAR *ConfigKey(EDriveKeyAction Action);

 private:
  friend struct FPlayerSettings;

  FKey Keys[6] = {EKeys::W, EKeys::S, EKeys::A, EKeys::D, EKeys::Q, EKeys::E};
};
}  // namespace LingTuSim::UI

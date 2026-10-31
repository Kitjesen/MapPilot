#include "LingTuSimDriveKeyBindings.h"

namespace LingTuSim::UI {
namespace {
constexpr int32 DriveKeyActionCount = static_cast<int32>(EDriveKeyAction::Count);

bool IsValidAction(const EDriveKeyAction Action) {
  return static_cast<uint8>(Action) < static_cast<uint8>(EDriveKeyAction::Count);
}

bool IsSupportedKey(const FKey &Key) {
  const FKey SupportedKeys[] = {
      EKeys::A,     EKeys::B,     EKeys::C,     EKeys::D,     EKeys::E,    EKeys::F,
      EKeys::G,     EKeys::H,     EKeys::I,     EKeys::J,     EKeys::K,    EKeys::L,
      EKeys::M,     EKeys::N,     EKeys::O,     EKeys::P,     EKeys::Q,    EKeys::R,
      EKeys::S,     EKeys::T,     EKeys::U,     EKeys::V,     EKeys::W,    EKeys::X,
      EKeys::Y,     EKeys::Z,     EKeys::Zero,  EKeys::One,   EKeys::Two,  EKeys::Three,
      EKeys::Four,  EKeys::Five,  EKeys::Six,   EKeys::Seven, EKeys::Eight, EKeys::Nine,
  };
  for (const FKey &SupportedKey : SupportedKeys) {
    if (Key == SupportedKey) {
      return true;
    }
  }
  return false;
}

bool IsReservedKey(const FKey &Key) {
  return Key == EKeys::B || Key == EKeys::C || Key == EKeys::R || Key == EKeys::X;
}
}  // namespace

FKey FDriveKeyBindings::GetKey(const EDriveKeyAction Action) const {
  return IsValidAction(Action) ? Keys[static_cast<int32>(Action)] : EKeys::Invalid;
}

bool FDriveKeyBindings::TryBind(const EDriveKeyAction Action, const FKey &Key, FString &Error) {
  Error.Reset();
  if (!IsValidAction(Action)) {
    Error = TEXT("未知的移动操作");
    return false;
  }

  const int32 ActionIndex = static_cast<int32>(Action);
  if (Keys[ActionIndex] == Key) {
    return true;
  }
  if (!IsSupportedKey(Key)) {
    Error = TEXT("仅支持单个字母 A-Z 或数字 0-9");
    return false;
  }
  if (IsReservedKey(Key)) {
    Error = FString::Printf(TEXT("按键 %s 已被界面操作占用"), *Key.GetDisplayName(false).ToString());
    return false;
  }

  for (int32 Index = 0; Index < DriveKeyActionCount; ++Index) {
    if (Index != ActionIndex && Keys[Index] == Key) {
      Error = FString::Printf(TEXT("按键 %s 已绑定到%s"), *Key.GetDisplayName(false).ToString(),
                              ActionLabel(static_cast<EDriveKeyAction>(Index)));
      return false;
    }
  }

  Keys[ActionIndex] = Key;
  return true;
}

void FDriveKeyBindings::Normalize() {
  for (int32 Index = 0; Index < DriveKeyActionCount; ++Index) {
    if (!IsSupportedKey(Keys[Index]) || IsReservedKey(Keys[Index])) {
      *this = FDriveKeyBindings();
      return;
    }
    for (int32 OtherIndex = Index + 1; OtherIndex < DriveKeyActionCount; ++OtherIndex) {
      if (Keys[Index] == Keys[OtherIndex]) {
        *this = FDriveKeyBindings();
        return;
      }
    }
  }
}

bool FDriveKeyBindings::operator==(const FDriveKeyBindings &Other) const {
  for (int32 Index = 0; Index < DriveKeyActionCount; ++Index) {
    if (Keys[Index] != Other.Keys[Index]) {
      return false;
    }
  }
  return true;
}

const TCHAR *FDriveKeyBindings::ActionLabel(const EDriveKeyAction Action) {
  switch (Action) {
    case EDriveKeyAction::Forward:
      return TEXT("前进");
    case EDriveKeyAction::Backward:
      return TEXT("后退");
    case EDriveKeyAction::Left:
      return TEXT("左移");
    case EDriveKeyAction::Right:
      return TEXT("右移");
    case EDriveKeyAction::TurnLeft:
      return TEXT("左转");
    case EDriveKeyAction::TurnRight:
      return TEXT("右转");
    default:
      return TEXT("未知操作");
  }
}

const TCHAR *FDriveKeyBindings::ConfigKey(const EDriveKeyAction Action) {
  switch (Action) {
    case EDriveKeyAction::Forward:
      return TEXT("KeyForward");
    case EDriveKeyAction::Backward:
      return TEXT("KeyBackward");
    case EDriveKeyAction::Left:
      return TEXT("KeyLeft");
    case EDriveKeyAction::Right:
      return TEXT("KeyRight");
    case EDriveKeyAction::TurnLeft:
      return TEXT("KeyTurnLeft");
    case EDriveKeyAction::TurnRight:
      return TEXT("KeyTurnRight");
    default:
      return TEXT("");
  }
}
}  // namespace LingTuSim::UI

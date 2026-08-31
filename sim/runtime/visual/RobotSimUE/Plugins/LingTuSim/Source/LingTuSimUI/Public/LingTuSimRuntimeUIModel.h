#pragma once

#include "CoreMinimal.h"
#include "InputCoreTypes.h"

namespace LingTuSim::UI {
enum class ERuntimeUIMode : uint8 {
  Drive,
  Build,
  Tactical,
  Pause,
};

enum class ERuntimeUIAction : uint8 {
  CycleCamera,
  ToggleRecording,
  Exit,
  SelectPreviousGame,
  SelectNextGame,
  SelectPreviousAsset,
  SelectNextAsset,
  ConfirmGameSelection,
  ReturnFromFrontEnd,
};

LINGTUSIMUI_API const TCHAR *RuntimeUIModeWireName(ERuntimeUIMode Mode);

/** Pure key policy. Runtime authority still decides whether an action is admissible. */
class LINGTUSIMUI_API FRuntimeUIActionPolicy final {
 public:
  static bool ResolveKey(ERuntimeUIMode Mode, const FKey &Key, ERuntimeUIAction &OutAction);

 private:
  FRuntimeUIActionPolicy() = delete;
};

/** Small deterministic state machine shared by Slate input and tests. */
class LINGTUSIMUI_API FRuntimeUIModeController final {
 public:
  ERuntimeUIMode GetMode() const { return Mode; }
  bool HandleKey(const FKey &Key);
  void ToggleBuild();
  void ToggleTactical();
  void TogglePause();

 private:
  void ToggleExclusive(ERuntimeUIMode RequestedMode);

  ERuntimeUIMode Mode = ERuntimeUIMode::Drive;
  ERuntimeUIMode ModeBeforePause = ERuntimeUIMode::Drive;
};
}  // namespace LingTuSim::UI

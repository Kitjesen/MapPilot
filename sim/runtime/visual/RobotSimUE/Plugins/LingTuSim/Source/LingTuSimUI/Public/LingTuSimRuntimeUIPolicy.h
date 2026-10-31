#pragma once

#include "CoreMinimal.h"

namespace LingTuSim::UI {
/** Pure startup policy for the asset-free runtime interface. */
class LINGTUSIMUI_API FRuntimeUIPolicy final {
 public:
  /**
   * -LingTuDisableRuntimeUI always wins. -LingTuRuntimeUI explicitly
   * enables the interface; otherwise only interactive game runs enable it.
   */
  static bool ShouldEnable(const TCHAR *CommandLine, bool bIsUnattended, bool bIsGame);

  static bool CanOwnInputContext(int32 EligibleGameWorldCount, int32 LocalPlayerCount);
  static bool ShouldStartFrontEnd(const TCHAR* CommandLine, bool bHasSession, bool bHasControl);

 private:
  FRuntimeUIPolicy() = delete;
};
}  // namespace LingTuSim::UI

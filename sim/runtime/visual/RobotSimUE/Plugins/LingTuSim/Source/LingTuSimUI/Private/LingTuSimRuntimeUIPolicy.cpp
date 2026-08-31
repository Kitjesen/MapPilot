#include "LingTuSimRuntimeUIPolicy.h"

#include "Misc/Parse.h"

namespace LingTuSim::UI {
bool FRuntimeUIPolicy::ShouldEnable(const TCHAR *CommandLine, const bool bIsUnattended,
                                    const bool bIsGame) {
  const TCHAR *Args = CommandLine != nullptr ? CommandLine : TEXT("");
  if (FParse::Param(Args, TEXT("LingTuDisableRuntimeUI"))) {
    return false;
  }
  if (FParse::Param(Args, TEXT("LingTuRuntimeUI"))) {
    return true;
  }
  return bIsGame && !bIsUnattended;
}

bool FRuntimeUIPolicy::CanOwnInputContext(const int32 EligibleGameWorldCount,
                                          const int32 LocalPlayerCount) {
  return EligibleGameWorldCount == 1 && LocalPlayerCount == 1;
}
}  // namespace LingTuSim::UI

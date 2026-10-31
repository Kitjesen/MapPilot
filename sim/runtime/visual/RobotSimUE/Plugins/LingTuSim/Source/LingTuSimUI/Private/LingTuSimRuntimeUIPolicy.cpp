#include "LingTuSimRuntimeUIPolicy.h"

#include "Misc/Parse.h"
#include "Misc/CommandLine.h"

namespace LingTuSim::UI {
bool FRuntimeUIPolicy::ShouldStartFrontEnd(const TCHAR* CommandLine, bool bHasSession, bool bHasControl) {
  const TCHAR* Args = CommandLine ? CommandLine : TEXT("");
  if (bHasSession || bHasControl || FParse::Param(Args, TEXT("LingTuDevRuntimeUIRender"))) return false;
  // Presence matters: an empty or invalid runtime argument is still a failed runtime
  // request, never permission to conceal that failure behind the welcome screen.
  TArray<FString> Tokens, Switches;
  FCommandLine::Parse(Args, Tokens, Switches);
  for (const FString& Switch : Switches) {
    FString Name, Value;
    if (!Switch.Split(TEXT("="), &Name, &Value)) Name = Switch;
    if (Name.Equals(TEXT("LingTuBundle"), ESearchCase::IgnoreCase) ||
        Name.Equals(TEXT("LingTuSnapshotPort"), ESearchCase::IgnoreCase) ||
        Name.Equals(TEXT("LingTuRunAllocation"), ESearchCase::IgnoreCase) ||
        Name.Equals(TEXT("LingTuRunId"), ESearchCase::IgnoreCase) ||
        Name.StartsWith(TEXT("LingTuControl"), ESearchCase::IgnoreCase)) return false;
  }
  return true;
}

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

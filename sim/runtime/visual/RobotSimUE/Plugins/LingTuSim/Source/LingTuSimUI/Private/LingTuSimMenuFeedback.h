#pragma once

#include "CoreMinimal.h"

namespace LingTuSim::UI {
/** Presentation deadlines only; never gate menu input or apply settings. */
class FMenuFeedbackState {
 public:
  void BeginPage(double Now) {
    PageStarted = Now;
    ChangeStarted = -1.;
    LastChange = FText::GetEmpty();
  }
  void Changed(const FText& Name, const FText& Value, bool bNeedsApply, double Now) {
    ChangeStarted = Now;
    LastChange = FText::Format(FText::FromString(TEXT("{0} → {1} · {2}")), Name, Value,
        FText::FromString(bNeedsApply ? TEXT("待应用") : TEXT("待保存")));
  }
  float PageOpacity(double Now) const { return 1.F - .55F * PageRemainder(Now); }
  float PageOffset(double Now) const { return 8.F * PageRemainder(Now); }
  FText ChangeText(double Now) const { return ChangeOpacity(Now) > 0.F ? LastChange : FText::GetEmpty(); }
  float ChangeOpacity(double Now) const {
    if (ChangeStarted < 0.) return 0.F;
    return static_cast<float>(FMath::Clamp((1.4 - (Now - ChangeStarted)) / .5, 0., 1.));
  }
  bool IsActive(double Now) const { return PageRemainder(Now) > 0.F || ChangeOpacity(Now) > 0.F; }
 private:
  float PageRemainder(double Now) const {
    if (PageStarted < 0.) return 0.F;
    const float Remaining = static_cast<float>(FMath::Clamp(1. - (Now - PageStarted) / .14, 0., 1.));
    return Remaining * Remaining;
  }
  double PageStarted = -1.;
  double ChangeStarted = -1.;
  FText LastChange;
};
}

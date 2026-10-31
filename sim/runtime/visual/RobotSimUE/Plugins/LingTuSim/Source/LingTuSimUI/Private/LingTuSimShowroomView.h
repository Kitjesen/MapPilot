#pragma once

#include "CoreMinimal.h"

namespace LingTuSim::UI {

enum class EShowroomView { Perspective, Front, Side, Back, Top };

struct FShowroomViewState {
  double YawDegrees = -25.0;
  double PitchDegrees = 12.0;
  double Zoom = 1.0;
  FVector2D Pan = FVector2D::ZeroVector;

  void Reset() {
    YawDegrees = -25.0;
    PitchDegrees = 12.0;
    Zoom = 1.0;
    Pan = FVector2D::ZeroVector;
  }

  void SetPreset(const EShowroomView View) {
    Reset();
    switch (View) {
      case EShowroomView::Perspective:
        break;
      case EShowroomView::Front:
        YawDegrees = 0.0;
        break;
      case EShowroomView::Side:
        YawDegrees = 90.0;
        break;
      case EShowroomView::Back:
        YawDegrees = 180.0;
        break;
      case EShowroomView::Top:
        YawDegrees = 0.0;
        PitchDegrees = 70.0;
        break;
    }
  }

  void Orbit(const FVector2D Delta) {
    YawDegrees = FMath::UnwindDegrees(YawDegrees + Delta.X * 240.0);
    PitchDegrees = FMath::Clamp(PitchDegrees + Delta.Y * 120.0, 5.0, 70.0);
  }

  void Dolly(const double WheelDelta) {
    Zoom = FMath::Clamp(Zoom * FMath::Pow(1.12, WheelDelta), 0.65, 1.8);
  }

  void Translate(const FVector2D Delta) {
    Pan.X = FMath::Clamp(Pan.X + Delta.X * 0.15, -0.10, 0.10);
    Pan.Y = FMath::Clamp(Pan.Y + Delta.Y * 0.15, -0.10, 0.10);
  }
};

}  // namespace LingTuSim::UI

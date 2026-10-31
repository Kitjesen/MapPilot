#pragma once
#include "LingTuSimInspectionGuidance.h"
#include "Widgets/SLeafWidget.h"

class UWorld;
namespace LingTuSim::UI {
struct FInspectionProjection;

class SLingTuSimNavigationOverlay final : public SLeafWidget {
 public:
  SLATE_BEGIN_ARGS(SLingTuSimNavigationOverlay) : _World(nullptr) {}
    SLATE_ARGUMENT(UWorld*, World)
    SLATE_ARGUMENT(TSharedPtr<FInspectionProjection>, Task)
    SLATE_ATTRIBUTE(FNavigationPose, Pose)
  SLATE_END_ARGS()
  void Construct(const FArguments& Args);
  virtual FVector2D ComputeDesiredSize(float) const override { return FVector2D::ZeroVector; }
  virtual int32 OnPaint(const FPaintArgs&, const FGeometry&, const FSlateRect&,
    FSlateWindowElementList&, int32, const FWidgetStyle&, bool) const override;
 private:
  TWeakObjectPtr<UWorld> World;
  TSharedPtr<FInspectionProjection> Task;
  TAttribute<FNavigationPose> Pose;
};
}

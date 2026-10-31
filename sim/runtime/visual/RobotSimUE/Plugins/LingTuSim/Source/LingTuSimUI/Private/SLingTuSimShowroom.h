#pragma once

#include "Widgets/SCompoundWidget.h"
#include "LingTuSimShowroomView.h"

namespace LingTuSim::UI {
class FRobotShowroomScene;
enum class EShowroomEnvironment { Gallery, OperationsBase };
TSharedRef<SWidget> MakeShowroomVeil();

/** Presentation-only world: no physics scene, navigation, controller or session. */
class SLingTuSimShowroom final : public SCompoundWidget {
 public:
  SLATE_BEGIN_ARGS(SLingTuSimShowroom) : _Environment(EShowroomEnvironment::Gallery) {}
  SLATE_ATTRIBUTE(FString, RobotId)
  SLATE_ARGUMENT(EShowroomEnvironment, Environment)
  SLATE_END_ARGS()
  SLingTuSimShowroom();
  void Construct(const FArguments& Args);
  virtual ~SLingTuSimShowroom() override;
  virtual void Tick(const FGeometry&, double, float) override;
  bool HasRobot() const;
  void Orbit(FVector2D Delta);
  void Pan(FVector2D Delta);
  void Zoom(double Steps);
  void SetView(EShowroomView Preset);
  const FShowroomViewState& GetView() const { return View; }
  bool IsReady() const { return Scene.IsValid() && WarmupFrames == 0 && LoadedId == RobotId.Get(); }
  static FString MeshPath(const FString& RobotId);

 private:
  TAttribute<FString> RobotId;
  EShowroomEnvironment Environment = EShowroomEnvironment::Gallery;
  TUniquePtr<FRobotShowroomScene> Scene;
  FSlateBrush RenderBrush;
  FString LoadedId;
  int32 WarmupFrames = 0;
  FShowroomViewState View;
  FVector2D RenderSize = FVector2D::ZeroVector;
  void ViewChanged();
};
}

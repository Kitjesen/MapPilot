#include "SLingTuSimNavigationOverlay.h"
#include "LingTuSimInspectionProjection.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "HAL/PlatformTime.h"
#include "Rendering/DrawElements.h"
#include "Styling/CoreStyle.h"

namespace LingTuSim::UI {
void SLingTuSimNavigationOverlay::Construct(const FArguments& Args) {
  World = Args._World; Task = Args._Task; Pose = Args._Pose;
  SetVisibility(EVisibility::HitTestInvisible);
}

int32 SLingTuSimNavigationOverlay::OnPaint(const FPaintArgs&, const FGeometry& G,
    const FSlateRect&, FSlateWindowElementList& E, int32 Layer, const FWidgetStyle&, bool) const {
  const FNavigationPose CurrentPose = Pose.Get(FNavigationPose());
  if (!Task.IsValid() || !CurrentPose.bAvailable || !Task->Guidance.SimAnchorPose.bAvailable ||
      !Task->Guidance.IsCurrent(*Task, FPlatformTime::Seconds())) return Layer;
  const auto& Guide = Task->Guidance;
  const FVector2D Size = G.GetLocalSize();
  if (Size.X < 320 || Size.Y < 240) return Layer;
  const FLinearColor Ink(.94F, .93F, .89F, .9F), Amber(.92F, .68F, .26F, .95F);
  const auto Line = [&](TArray<FVector2D> Points, FLinearColor Color, float Width = 1) {
    FSlateDrawElement::MakeLines(E, Layer + 1, G.ToPaintGeometry(), Points, ESlateDrawEffect::None, Color, true, Width);
  };
  const auto Text = [&](FVector2D At, const FString& Value, FLinearColor Color) {
    const auto Font = FCoreStyle::GetDefaultFontStyle(TEXT("Regular"), 11);
    FSlateDrawElement::MakeText(E, Layer, G.ToPaintGeometry(FVector2D(200, 24), FSlateLayoutTransform(At + FVector2D(1, 1))),
      Value, Font, ESlateDrawEffect::None, FLinearColor(0, 0, 0, .85F));
    FSlateDrawElement::MakeText(E, Layer + 1, G.ToPaintGeometry(FVector2D(200, 24), FSlateLayoutTransform(At)),
      Value, Font, ESlateDrawEffect::None, Color);
  };
  const double Centre = Size.X * .5;
  const double HalfWidth = FMath::Min(190.0, Size.X * .16);
  const FVector Target = Guide.EstimatedTargetUnrealCm(Guide.SimAnchorPose);
  const FVector TargetSim(Target.X * .01, -Target.Y * .01, Target.Z * .01);
  const FVector ToTarget = TargetSim - CurrentPose.PositionMeters;
  const double Heading = FMath::Fmod(FMath::RadiansToDegrees(Guide.RobotMapYawRadians) +
    CurrentPose.Rotation.Rotator().Yaw - Guide.SimAnchorPose.Rotation.Rotator().Yaw + 3600, 360);
  Line({FVector2D(Centre-HalfWidth, 46), FVector2D(Centre+HalfWidth, 46)}, Ink);
  for (int32 Offset = -60; Offset <= 60; Offset += 15) {
    const double X = Centre + Offset / 60.0 * HalfWidth;
    Line({FVector2D(X, 46), FVector2D(X, Offset % 30 == 0 ? 37 : 41)}, Ink);
    if (Offset % 30 == 0) Text(FVector2D(X-13, 52), FString::Printf(TEXT("%03.0f"), FMath::Fmod(Heading-Offset+360, 360)), Ink);
  }
  const double Bearing = FMath::UnwindDegrees(FMath::RadiansToDegrees(FMath::Atan2(ToTarget.Y, ToTarget.X)) - CurrentPose.Rotation.Rotator().Yaw);
  const double TargetX = Centre - FMath::Clamp(Bearing / 60.0, -1.0, 1.0) * HalfWidth;
  Line({FVector2D(TargetX-4, 30), FVector2D(TargetX, 26), FVector2D(TargetX+4, 30), FVector2D(TargetX, 34), FVector2D(TargetX-4, 30)}, Amber, 1.5);
  Text(FVector2D(Centre-38, 75), TEXT("地图航向"), Ink);

  APlayerController* PC = World.IsValid() ? World->GetFirstPlayerController() : nullptr;
  if (!PC) return Layer + 1;
  int32 Width = 0, Height = 0; PC->GetViewportSize(Width, Height);
  if (Width <= 0 || Height <= 0) return Layer + 1;
  FVector Camera; FRotator CameraRotation; PC->GetPlayerViewPoint(Camera, CameraRotation);
  FVector2D Pixel;
  const bool bProjected = PC->ProjectWorldLocationToScreen(Target, Pixel, true);
  const FVector Direction = Target - Camera;
  const bool bFront = FVector::DotProduct(CameraRotation.Vector(), Direction) > 0;
  FVector2D Unclamped;
  if (bProjected && bFront) Unclamped = FVector2D(Pixel.X * Size.X / Width, Pixel.Y * Size.Y / Height);
  else {
    const FRotationMatrix Basis(CameraRotation);
    FVector2D Edge(FVector::DotProduct(Basis.GetUnitAxis(EAxis::Y), Direction),
      -FVector::DotProduct(Basis.GetUnitAxis(EAxis::Z), Direction));
    if (Edge.IsNearlyZero()) Edge = FVector2D(0, 1);
    Unclamped = Size * .5 + Edge.GetSafeNormal() * Size.Size();
  }
  const FVector2D At = FInspectionGuidance::PlaceMarker(Unclamped, Size);
  const bool bOffscreen = !bFront || !At.Equals(Unclamped, .1);
  if (bOffscreen) {
    const FVector2D D = (At - Size * .5).GetSafeNormal();
    const FVector2D Side(-D.Y, D.X);
    Line({At-D*9+Side*6, At+D*5, At-D*9-Side*6}, Amber, 2);
  } else {
    Line({At+FVector2D(-7, 0), At+FVector2D(0,-7), At+FVector2D(7,0), At+FVector2D(0,7), At+FVector2D(-7,0)}, Amber, 2);
  }
  // A localized direction estimate, not a visibility/occlusion or traversability claim.
  const FVector2D Label(FMath::Clamp(At.X + 15, 30.0, Size.X - 185),
    FMath::Clamp(At.Y + 10, 110.0, Size.Y - 120));
  Text(Label, FString::Printf(TEXT("目标方位 · 巡检点 %02d"), Task->CurrentPointNumber), Ink);
  Text(Label+FVector2D(0,20), FString::Printf(TEXT("估计直线 %.1f m"), ToTarget.Size()), Ink);
  return Layer + 1;
}
}

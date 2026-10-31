#pragma once

#include "CoreMinimal.h"

namespace LingTuSim::UI {
struct FInspectionProjection;

/** Copied MuJoCo pose used only to anchor a localized directional overlay. */
struct FNavigationPose final {
  bool bAvailable = false;
  FVector PositionMeters = FVector::ZeroVector;
  FQuat Rotation = FQuat::Identity;
};

/** Read-only guidance, never a planner, collision query or arrival detector. */
struct FInspectionGuidance final {
  bool bAvailable = false;
  bool bSimulationRuntime = false;
  FString ProductSessionId;
  FString PointId;
  FString Phase;
  FString Reason;
  FVector GoalMapMeters = FVector::ZeroVector;
  FVector RobotMapMeters = FVector::ZeroVector;
  double RobotMapYawRadians = 0;
  double ReceivedSeconds = 0;
  FNavigationPose SimAnchorPose;

  static bool Parse(const FString& Json, const FInspectionProjection& Task,
                    FInspectionGuidance& Out, FString& Error);
  bool IsCurrent(const FInspectionProjection& Task, double NowSeconds) const;
  double DistanceMeters() const;
  double BearingDegrees() const;
  FVector EstimatedTargetUnrealCm(const FNavigationPose& Pose) const;
  static FVector2D PlaceMarker(FVector2D Position, FVector2D ViewSize);
  static FText PhaseText(const FInspectionProjection& Task);
};
}

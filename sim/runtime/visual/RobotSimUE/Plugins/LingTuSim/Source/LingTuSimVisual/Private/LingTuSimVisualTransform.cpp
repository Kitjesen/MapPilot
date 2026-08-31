#include "LingTuSimVisualTransform.h"

namespace
{
    bool IsFiniteVector(const FVector& Value)
    {
        return FMath::IsFinite(Value.X)
            && FMath::IsFinite(Value.Y)
            && FMath::IsFinite(Value.Z);
    }

    bool IsFiniteQuat(const FQuat& Value)
    {
        return FMath::IsFinite(Value.X)
            && FMath::IsFinite(Value.Y)
            && FMath::IsFinite(Value.Z)
            && FMath::IsFinite(Value.W);
    }
}

bool LingTuSim::Visual::FCoordinateConverter::TryMakeWorldTransform(
    const FEntityState& Entity,
    const FVector& WorldScale,
    FTransform& OutTransform)
{
    if (!IsFiniteVector(Entity.PositionMeters)
        || !IsFiniteQuat(Entity.Rotation)
        || !IsFiniteVector(WorldScale))
    {
        return false;
    }

    const FVector UnrealPosition(
        100.0 * Entity.PositionMeters.X,
        -100.0 * Entity.PositionMeters.Y,
        100.0 * Entity.PositionMeters.Z);
    FQuat UnrealRotation(
        -Entity.Rotation.X,
        Entity.Rotation.Y,
        -Entity.Rotation.Z,
        Entity.Rotation.W);

    const double RotationSizeSquared = UnrealRotation.SizeSquared();
    if (!IsFiniteVector(UnrealPosition)
        || !FMath::IsFinite(RotationSizeSquared)
        || RotationSizeSquared <= UE_SMALL_NUMBER)
    {
        return false;
    }

    UnrealRotation.Normalize();
    if (!IsFiniteQuat(UnrealRotation))
    {
        return false;
    }

    OutTransform = FTransform(UnrealRotation, UnrealPosition, WorldScale);
    return !OutTransform.ContainsNaN();
}

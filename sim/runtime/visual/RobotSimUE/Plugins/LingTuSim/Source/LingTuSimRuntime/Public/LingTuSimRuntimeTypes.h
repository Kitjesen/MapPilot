#pragma once

#include "CoreMinimal.h"

namespace LingTuSim
{
    struct LINGTUSIMRUNTIME_API FCompiledArtifactNames
    {
        static const TCHAR* SessionLock() { return TEXT("session.lock.json"); }
        static const TCHAR* PhysicsPlan() { return TEXT("physics.plan.json"); }
        static const TCHAR* VisualPlan() { return TEXT("visual.plan.json"); }
        static const TCHAR* SensorPlan() { return TEXT("sensor.plan.json"); }
        static const TCHAR* ControlPlan() { return TEXT("control.plan.json"); }
        static const TCHAR* ScenarioPlan() { return TEXT("scenario.plan.json"); }
        static const TCHAR* TransportIntent() { return TEXT("transport.intent.json"); }
    };

    struct LINGTUSIMRUNTIME_API FEntityId
    {
        FString StableId;
        FString InstanceId;
        FString FrameId;

        bool IsValid() const
        {
            return !StableId.IsEmpty() && !InstanceId.IsEmpty() && !FrameId.IsEmpty();
        }
    };

    struct LINGTUSIMRUNTIME_API FEntityState
    {
        FEntityId Id;
        FVector PositionMeters = FVector::ZeroVector;
        FQuat Rotation = FQuat::Identity;
        FVector LinearVelocityMetersPerSecond = FVector::ZeroVector;
        FVector AngularVelocityRadiansPerSecond = FVector::ZeroVector;
    };

    struct LINGTUSIMRUNTIME_API FSessionBundleView
    {
        FString SessionDigest;
        FString SessionLockPath;
        FString PhysicsPlanPath;
        FString VisualPlanPath;
        FString SensorPlanPath;
        FString ControlPlanPath;
        FString ScenarioPlanPath;
        FString TransportIntentPath;
        FString PhysicsPlanDigest;
        FString VisualPlanDigest;
        FString SensorPlanDigest;
        FString ControlPlanDigest;
        FString ScenarioPlanDigest;
        FString TransportIntentDigest;

        bool IsBound() const
        {
            return !SessionDigest.IsEmpty()
                && !SessionLockPath.IsEmpty()
                && !PhysicsPlanPath.IsEmpty()
                && !VisualPlanPath.IsEmpty()
                && !SensorPlanPath.IsEmpty()
                && !ControlPlanPath.IsEmpty()
                && !TransportIntentPath.IsEmpty()
                && !PhysicsPlanDigest.IsEmpty()
                && !VisualPlanDigest.IsEmpty()
                && !SensorPlanDigest.IsEmpty()
                && !ControlPlanDigest.IsEmpty()
                && !TransportIntentDigest.IsEmpty()
                && (ScenarioPlanPath.IsEmpty() == ScenarioPlanDigest.IsEmpty());
        }
    };

    struct LINGTUSIMRUNTIME_API FSnapshotEnvelope
    {
        FString SessionDigest;
        uint64 ModelGeneration = 0;
        uint64 ResetGeneration = 0;
        uint64 Sequence = 0;
        int64 SimTimeNs = 0;
        TArray<FEntityState> Entities;
    };

    struct LINGTUSIMRUNTIME_API FCommandEnvelope
    {
        FString SessionDigest;
        FName InstanceId;
        FString StableId;
        FString FrameId;
        uint64 ModelGeneration = 0;
        uint64 ResetGeneration = 0;
        uint64 Sequence = 0;
        int64 ApplyTimeNs = 0;
    };
}

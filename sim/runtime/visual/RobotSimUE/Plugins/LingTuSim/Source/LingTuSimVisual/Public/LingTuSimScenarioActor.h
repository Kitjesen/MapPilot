#pragma once

#include "GameFramework/Actor.h"

#include "LingTuSimScenarioActor.generated.h"

class USceneComponent;
class UStaticMeshComponent;

/** Snapshot-driven visual representation of one routed scenario entity. */
UCLASS()
class LINGTUSIMVISUAL_API ALingTuSimScenarioActor final : public AActor
{
    GENERATED_BODY()

public:
    ALingTuSimScenarioActor();

    bool CanConfigureSnapshotIdentity(
        const FString& InStableId,
        const FString& InAuthority,
        uint64 InSourceEpoch,
        const FString& InSemanticClass,
        const FString& InMotionState,
        const FString& InPhysicsProxyMode) const;

    bool ConfigureSnapshotIdentity(
        const FString& InStableId,
        const FString& InAuthority,
        uint64 InSourceEpoch,
        const FString& InSemanticClass,
        const FString& InMotionState,
        const FString& InPhysicsProxyMode,
        const FString& InBodyStableId);

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="LingTuSim|Scenario")
    FString StableId;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="LingTuSim|Scenario")
    FString Authority;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="LingTuSim|Scenario")
    int64 SourceEpoch = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="LingTuSim|Scenario")
    FString SemanticClass;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="LingTuSim|Scenario")
    FString MotionState;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="LingTuSim|Scenario")
    FString PhysicsProxyMode;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="LingTuSim|Scenario")
    FString BodyStableId;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="LingTuSim|Scenario")
    TObjectPtr<USceneComponent> SceneRoot;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="LingTuSim|Scenario")
    TObjectPtr<UStaticMeshComponent> VisualMesh;
};

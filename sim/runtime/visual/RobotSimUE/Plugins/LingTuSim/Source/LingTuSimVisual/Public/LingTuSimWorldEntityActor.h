#pragma once

#include "GameFramework/Actor.h"

#include "LingTuSimWorldEntityActor.generated.h"

class USceneComponent;

/** Plan-driven visual representation of one static WorldPackage entity. */
UCLASS()
class LINGTUSIMVISUAL_API ALingTuSimWorldEntityActor final : public AActor
{
    GENERATED_BODY()

public:
    ALingTuSimWorldEntityActor();

    bool ConfigureIdentity(
        const FString& InStableId,
        const FString& InSemanticClass,
        const FString& InAuthority);

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="LingTuSim|World")
    FString StableId;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="LingTuSim|World")
    FString SemanticClass;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="LingTuSim|World")
    FString Authority;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="LingTuSim|World")
    TObjectPtr<USceneComponent> SceneRoot;
};

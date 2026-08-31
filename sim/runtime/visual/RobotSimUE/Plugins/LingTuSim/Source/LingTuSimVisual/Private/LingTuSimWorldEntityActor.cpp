#include "LingTuSimWorldEntityActor.h"

#include "Components/SceneComponent.h"

ALingTuSimWorldEntityActor::ALingTuSimWorldEntityActor()
{
    PrimaryActorTick.bCanEverTick = false;
    SetActorEnableCollision(false);
    SetActorHiddenInGame(true);

    SceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("WorldEntityRoot"));
    SceneRoot->SetMobility(EComponentMobility::Movable);
    RootComponent = SceneRoot;
}

bool ALingTuSimWorldEntityActor::ConfigureIdentity(
    const FString& InStableId,
    const FString& InSemanticClass,
    const FString& InAuthority)
{
    const bool bAuthorityValid = InAuthority == TEXT("mujoco")
        || InAuthority == TEXT("scenario")
        || InAuthority == TEXT("ue_animation");
    if (InStableId.IsEmpty() || InSemanticClass.IsEmpty() || !bAuthorityValid)
    {
        return false;
    }

    StableId = InStableId;
    SemanticClass = InSemanticClass;
    Authority = InAuthority;
    Tags.Reset();
    Tags.Add(TEXT("LingTuWorldEntityProjection"));
    Tags.Add(FName(*FString::Printf(TEXT("StableId:%s"), *StableId)));
    Tags.Add(FName(*FString::Printf(TEXT("SemanticClass:%s"), *SemanticClass)));
    Tags.Add(FName(*FString::Printf(TEXT("Authority:%s"), *Authority)));
    return true;
}

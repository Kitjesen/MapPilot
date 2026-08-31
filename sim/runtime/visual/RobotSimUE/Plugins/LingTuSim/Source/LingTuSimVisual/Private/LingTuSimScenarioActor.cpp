#include "LingTuSimScenarioActor.h"

#include "LingTuSimPresentationPolicy.h"

#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "UObject/ConstructorHelpers.h"

ALingTuSimScenarioActor::ALingTuSimScenarioActor()
{
    PrimaryActorTick.bCanEverTick = false;
    SetActorEnableCollision(false);
    SetActorHiddenInGame(true);

    SceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("ScenarioRoot"));
    SceneRoot->SetMobility(EComponentMobility::Movable);
    RootComponent = SceneRoot;

    VisualMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("ScenarioVisual"));
    VisualMesh->SetupAttachment(SceneRoot);
    VisualMesh->SetMobility(EComponentMobility::Movable);
    LingTuSim::Visual::ApplyPresentationPolicy(*VisualMesh);
    VisualMesh->SetRelativeScale3D(FVector(0.35, 0.35, 0.9));

    static ConstructorHelpers::FObjectFinder<UStaticMesh> CylinderMesh(
        TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
    if (CylinderMesh.Succeeded())
    {
        VisualMesh->SetStaticMesh(CylinderMesh.Object);
    }
}

bool ALingTuSimScenarioActor::CanConfigureSnapshotIdentity(
    const FString& InStableId,
    const FString& InAuthority,
    const uint64 InSourceEpoch,
    const FString& InSemanticClass,
    const FString& InMotionState,
    const FString& InPhysicsProxyMode) const
{
    return !InStableId.IsEmpty()
        && (InAuthority == TEXT("scenario") || InAuthority == TEXT("ue_animation"))
        && !InSemanticClass.IsEmpty()
        && !InMotionState.IsEmpty()
        && !InPhysicsProxyMode.IsEmpty()
        && VisualMesh != nullptr
        && VisualMesh->GetStaticMesh() != nullptr
        && InSourceEpoch <= static_cast<uint64>(MAX_int64);
}

bool ALingTuSimScenarioActor::ConfigureSnapshotIdentity(
    const FString& InStableId,
    const FString& InAuthority,
    const uint64 InSourceEpoch,
    const FString& InSemanticClass,
    const FString& InMotionState,
    const FString& InPhysicsProxyMode,
    const FString& InBodyStableId)
{
    if (!CanConfigureSnapshotIdentity(
            InStableId,
            InAuthority,
            InSourceEpoch,
            InSemanticClass,
            InMotionState,
            InPhysicsProxyMode))
    {
        return false;
    }

    StableId = InStableId;
    Authority = InAuthority;
    SourceEpoch = static_cast<int64>(InSourceEpoch);
    SemanticClass = InSemanticClass;
    MotionState = InMotionState;
    PhysicsProxyMode = InPhysicsProxyMode;
    BodyStableId = InBodyStableId;
    Tags.Reset();
    Tags.Add(TEXT("LingTuScenarioSnapshotProjection"));
    Tags.Add(FName(*FString::Printf(TEXT("StableId:%s"), *StableId)));
    Tags.Add(FName(*FString::Printf(TEXT("Authority:%s"), *Authority)));
    Tags.Add(FName(*FString::Printf(TEXT("SemanticClass:%s"), *SemanticClass)));
    return true;
}

#include "LingTuSimVisualInstanceBatchActor.h"

#include "LingTuSimPresentationPolicy.h"

#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#include "Components/SceneComponent.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInterface.h"
#include "UObject/UObjectGlobals.h"

ALingTuSimVisualInstanceBatchActor::ALingTuSimVisualInstanceBatchActor()
{
    PrimaryActorTick.bCanEverTick = false;
    SetActorEnableCollision(false);

    IdentityRoot = CreateDefaultSubobject<USceneComponent>(TEXT("IdentityRoot"));
    IdentityRoot->SetMobility(EComponentMobility::Static);
    RootComponent = IdentityRoot;
}

bool ALingTuSimVisualInstanceBatchActor::AddVisualGroup(const FString& GroupId, UStaticMesh* Mesh,
                                                        UMaterialInterface* Material,
                                                        const TArray<FTransform>& Transforms,
                                                        const TArray<FString>& StableIds,
                                                        const bool bCastShadow)
{
    if (GroupId.IsEmpty() || GroupId.TrimStartAndEnd() != GroupId || Mesh == nullptr ||
        Material == nullptr || Transforms.IsEmpty() || Transforms.Num() != StableIds.Num())
    {
        return false;
    }

    TSet<FString> KnownStableIds;
    for (const FLingTuSimVisualInstanceGroup& Group : VisualGroups)
    {
        if (Group.GroupId == GroupId)
        {
            return false;
        }
        for (const FString& StableId : Group.StableIds)
        {
            KnownStableIds.Add(StableId);
        }
    }
    for (const FString& StableId : StableIds)
    {
        if (StableId.IsEmpty() || StableId.TrimStartAndEnd() != StableId ||
            KnownStableIds.Contains(StableId))
        {
            return false;
        }
        KnownStableIds.Add(StableId);
    }
    for (const FTransform& Transform : Transforms)
    {
        const FVector Scale = Transform.GetScale3D();
        if (!Transform.IsValid() || FMath::IsNearlyZero(Scale.X) || FMath::IsNearlyZero(Scale.Y) ||
            FMath::IsNearlyZero(Scale.Z))
        {
            return false;
        }
    }

    const FName ComponentName = MakeUniqueObjectName(
        this, UHierarchicalInstancedStaticMeshComponent::StaticClass(), FName(TEXT("VisualGroup")));
    UHierarchicalInstancedStaticMeshComponent* InstanceComponent =
        NewObject<UHierarchicalInstancedStaticMeshComponent>(this, ComponentName, RF_Transactional);
    if (InstanceComponent == nullptr)
    {
        return false;
    }

    InstanceComponent->SetupAttachment(IdentityRoot);
    InstanceComponent->SetMobility(EComponentMobility::Static);
    InstanceComponent->SetStaticMesh(Mesh);
    InstanceComponent->SetMaterial(0, Material);
    InstanceComponent->SetCastShadow(bCastShadow);
    InstanceComponent->SetEnableGravity(false);
    LingTuSim::Visual::ApplyPresentationPolicy(*InstanceComponent);
    InstanceComponent->PreAllocateInstancesMemory(Transforms.Num());
    const TArray<int32> AddedIndices =
        InstanceComponent->AddInstances(Transforms, true, false, false);
    if (AddedIndices.Num() != Transforms.Num() ||
        InstanceComponent->GetInstanceCount() != Transforms.Num())
    {
        InstanceComponent->DestroyComponent();
        return false;
    }

    Modify();
    AddInstanceComponent(InstanceComponent);
    InstanceComponent->RegisterComponent();

    FLingTuSimVisualInstanceGroup& Group = VisualGroups.AddDefaulted_GetRef();
    Group.GroupId = GroupId;
    Group.Mesh = Mesh;
    Group.Material = Material;
    Group.StableIds = StableIds;
    Group.InstanceTransforms = Transforms;
    Group.bCastShadow = bCastShadow;
    Group.InstanceComponent = InstanceComponent;
    MarkPackageDirty();
    return true;
}

bool ALingTuSimVisualInstanceBatchActor::ValidateVisualGroup(
    const FString& GroupId, UStaticMesh* ExpectedMesh, UMaterialInterface* ExpectedMaterial,
    const TArray<FTransform>& ExpectedTransforms, const TArray<FString>& ExpectedStableIds,
    const bool bExpectedCastShadow) const
{
    if (GroupId.IsEmpty() || GroupId.TrimStartAndEnd() != GroupId || ExpectedMesh == nullptr ||
        ExpectedMaterial == nullptr || ExpectedTransforms.IsEmpty() ||
        ExpectedTransforms.Num() != ExpectedStableIds.Num())
    {
        return false;
    }
    const FLingTuSimVisualInstanceGroup* Match = VisualGroups.FindByPredicate(
        [&GroupId](const FLingTuSimVisualInstanceGroup& Group) { return Group.GroupId == GroupId; });
    if (Match == nullptr || Match->Mesh != ExpectedMesh || Match->Material != ExpectedMaterial ||
        Match->StableIds != ExpectedStableIds ||
        Match->InstanceTransforms.Num() != ExpectedTransforms.Num() ||
        Match->bCastShadow != bExpectedCastShadow || !IsValid(Match->InstanceComponent))
    {
        return false;
    }
    const UHierarchicalInstancedStaticMeshComponent& Component = *Match->InstanceComponent;
    if (Component.GetStaticMesh() != ExpectedMesh || Component.GetMaterial(0) != ExpectedMaterial ||
        Component.CastShadow != bExpectedCastShadow ||
        Component.GetInstanceCount() != ExpectedTransforms.Num() ||
        !LingTuSim::Visual::HasPresentationPolicy(Component))
    {
        return false;
    }
    for (int32 Index = 0; Index < ExpectedTransforms.Num(); ++Index)
    {
        const FTransform& Expected = ExpectedTransforms[Index];
        if (!Match->InstanceTransforms[Index].TranslationEquals(Expected, 0.01) ||
            !Match->InstanceTransforms[Index].RotationEquals(Expected, 1.0e-5) ||
            !Match->InstanceTransforms[Index].Scale3DEquals(Expected, 1.0e-6))
        {
            return false;
        }
        FTransform Actual;
        if (!Component.GetInstanceTransform(Index, Actual, false) ||
            !Actual.TranslationEquals(Expected, 0.01) ||
            !Actual.RotationEquals(Expected, 1.0e-5) ||
            !Actual.Scale3DEquals(Expected, 1.0e-6))
        {
            return false;
        }
    }
    return true;
}

void ALingTuSimVisualInstanceBatchActor::ResetGroups()
{
    Modify();
    for (FLingTuSimVisualInstanceGroup& Group : VisualGroups)
    {
        if (IsValid(Group.InstanceComponent))
        {
            Group.InstanceComponent->Modify();
            Group.InstanceComponent->ClearInstances();
            Group.InstanceComponent->DestroyComponent();
        }
    }
    VisualGroups.Reset();
    MarkPackageDirty();
}

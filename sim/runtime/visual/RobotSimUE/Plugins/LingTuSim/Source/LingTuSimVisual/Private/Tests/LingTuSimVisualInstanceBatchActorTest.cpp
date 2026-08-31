#if WITH_DEV_AUTOMATION_TESTS

#include "LingTuSimPresentationPolicy.h"
#include "LingTuSimVisualInstanceBatchActor.h"

#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#include "Components/SceneComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "Engine/WorldInitializationValues.h"
#include "Materials/Material.h"
#include "Misc/AutomationTest.h"
#include "Misc/Guid.h"
#include "UObject/UnrealType.h"

namespace
{
    class FScopedInstanceBatchTestWorld final
    {
    public:
        FScopedInstanceBatchTestWorld()
        {
            const FWorldInitializationValues InitializationValues =
                FWorldInitializationValues()
                    .AllowAudioPlayback(false)
                    .RequiresHitProxies(false)
                    .CreatePhysicsScene(false)
                    .CreateNavigation(false)
                    .CreateAISystem(false)
                    .ShouldSimulatePhysics(false)
                    .SetTransactional(false);
            World = UWorld::CreateWorld(
                EWorldType::Game, false,
                FName(*FString::Printf(TEXT("LingTuSimInstanceBatchTestWorld_%s"),
                                       *FGuid::NewGuid().ToString(EGuidFormats::Digits))),
                nullptr, true, ERHIFeatureLevel::Num, &InitializationValues);
        }

        ~FScopedInstanceBatchTestWorld()
        {
            if (World != nullptr)
            {
                World->DestroyWorld(false);
            }
        }

        ALingTuSimVisualInstanceBatchActor* SpawnActor() const
        {
            return World != nullptr ? World->SpawnActor<ALingTuSimVisualInstanceBatchActor>()
                                    : nullptr;
        }

    private:
        UWorld* World = nullptr;
    };

    TArray<FTransform> MakeTransforms()
    {
        return {
            FTransform(FRotator::ZeroRotator, FVector(0.0, 0.0, 0.0)),
            FTransform(FRotator(0.0, 90.0, 0.0), FVector(200.0, 0.0, 0.0)),
        };
    }

    TArray<FString> MakeStableIds()
    {
        return {TEXT("factory_park/cone_001"), TEXT("factory_park/cone_002")};
    }

    void GetHismComponents(ALingTuSimVisualInstanceBatchActor& Actor,
                           TArray<UHierarchicalInstancedStaticMeshComponent*>& OutComponents)
    {
        Actor.GetComponents<UHierarchicalInstancedStaticMeshComponent>(OutComponents);
    }
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuSimVisualInstanceBatchActorSuccessTest,
                                 "LingTuSim.Visual.FactoryPark.InstanceBatch.SuccessAndMetadata",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuSimVisualInstanceBatchActorSuccessTest::RunTest(const FString& Parameters)
{
    FScopedInstanceBatchTestWorld TestWorld;
    ALingTuSimVisualInstanceBatchActor* Actor = TestWorld.SpawnActor();
    TestNotNull(TEXT("batch actor spawns"), Actor);
    if (Actor == nullptr)
    {
        return false;
    }

    UStaticMesh* Mesh = NewObject<UStaticMesh>(Actor);
    UMaterialInterface* Material = UMaterial::GetDefaultMaterial(MD_Surface);
    const TArray<FTransform> Transforms = MakeTransforms();
    const TArray<FString> StableIds = MakeStableIds();
    TestTrue(
        TEXT("valid visual group is accepted"),
        Actor->AddVisualGroup(TEXT("traffic_cones"), Mesh, Material, Transforms, StableIds, true));

    TArray<UHierarchicalInstancedStaticMeshComponent*> Components;
    GetHismComponents(*Actor, Components);
    TestEqual(TEXT("one HISM exists for one group"), Components.Num(), 1);
    TestEqual(TEXT("one metadata record exists"), Actor->VisualGroups.Num(), 1);
    TestTrue(TEXT("identity root remains the actor root"),
             Actor->GetRootComponent() == Actor->IdentityRoot);
    TestEqual(TEXT("identity root mobility is static"), Actor->IdentityRoot->GetMobility(),
              EComponentMobility::Static);
    if (Components.Num() == 1 && Actor->VisualGroups.Num() == 1)
    {
        UHierarchicalInstancedStaticMeshComponent* Component = Components[0];
        const FLingTuSimVisualInstanceGroup& Metadata = Actor->VisualGroups[0];
        TestEqual(TEXT("bulk instance count is preserved"), Component->GetInstanceCount(), 2);
        TestTrue(TEXT("component is visual-only"),
                 LingTuSim::Visual::HasPresentationPolicy(*Component));
        TestFalse(TEXT("visual-only component has no gravity"), Component->IsGravityEnabled());
        TestEqual(TEXT("component mobility is static"), Component->GetMobility(),
                  EComponentMobility::Static);
        TestTrue(TEXT("requested cast-shadow value is applied"), Component->CastShadow);
        TestTrue(TEXT("actor collision is disabled"), !Actor->GetActorEnableCollision());
        TestEqual(TEXT("group ID is serialized metadata"), Metadata.GroupId,
                  FString(TEXT("traffic_cones")));
        TestTrue(TEXT("mesh metadata is retained"), Metadata.Mesh == Mesh);
        TestTrue(TEXT("material metadata is retained"), Metadata.Material == Material);
        TestEqual(TEXT("stable ID count is retained"), Metadata.StableIds.Num(), StableIds.Num());
        TestEqual(TEXT("serialized transform count is retained"),
                  Metadata.InstanceTransforms.Num(), Transforms.Num());
        TestTrue(TEXT("first serialized transform value is retained"),
                 Metadata.InstanceTransforms[0].Equals(Transforms[0]));
        TestTrue(TEXT("second serialized transform value is retained"),
                 Metadata.InstanceTransforms[1].Equals(Transforms[1]));
        TestEqual(TEXT("first stable ID is retained"), Metadata.StableIds[0], StableIds[0]);
        TestEqual(TEXT("second stable ID is retained"), Metadata.StableIds[1], StableIds[1]);
        TestTrue(TEXT("cast-shadow metadata is retained"), Metadata.bCastShadow);
        TestTrue(TEXT("HISM metadata points at the owned component"),
                 Metadata.InstanceComponent == Component);
        TestTrue(TEXT("HISM is an instance component for actor serialization"),
                 Actor->GetInstanceComponents().Contains(Component));
        TestFalse(TEXT("HISM is not transient"), Component->HasAnyFlags(RF_Transient));
        TestTrue(TEXT("strict live/serialized group validation succeeds"),
                 Actor->ValidateVisualGroup(TEXT("traffic_cones"), Mesh, Material, Transforms,
                                            StableIds, true));
        Component->UpdateInstanceTransform(
            0, FTransform(FRotator::ZeroRotator, FVector(25.0, 0.0, 0.0)), false, true, true);
        TestFalse(TEXT("live transform drift is rejected"),
                  Actor->ValidateVisualGroup(TEXT("traffic_cones"), Mesh, Material, Transforms,
                                             StableIds, true));
    }

    const FProperty* GroupsProperty = FindFProperty<FProperty>(
        ALingTuSimVisualInstanceBatchActor::StaticClass(),
        GET_MEMBER_NAME_CHECKED(ALingTuSimVisualInstanceBatchActor, VisualGroups));
    TestNotNull(TEXT("group metadata is reflected"), GroupsProperty);
    TestTrue(TEXT("group metadata participates in UObject serialization"),
             GroupsProperty != nullptr && !GroupsProperty->HasAnyPropertyFlags(CPF_Transient));
    const FProperty* StableIdsProperty =
        FindFProperty<FProperty>(FLingTuSimVisualInstanceGroup::StaticStruct(),
                                 GET_MEMBER_NAME_CHECKED(FLingTuSimVisualInstanceGroup, StableIds));
    TestTrue(TEXT("per-instance stable IDs participate in UObject serialization"),
             StableIdsProperty != nullptr &&
                 !StableIdsProperty->HasAnyPropertyFlags(CPF_Transient));
    const FProperty* InstanceTransformsProperty = FindFProperty<FProperty>(
        FLingTuSimVisualInstanceGroup::StaticStruct(),
        GET_MEMBER_NAME_CHECKED(FLingTuSimVisualInstanceGroup, InstanceTransforms));
    TestTrue(TEXT("per-instance transforms participate in UObject serialization"),
             InstanceTransformsProperty != nullptr &&
                 !InstanceTransformsProperty->HasAnyPropertyFlags(CPF_Transient));
    UFunction* AddFunction = Actor->FindFunction(TEXT("AddVisualGroup"));
    TestTrue(TEXT("AddVisualGroup is Blueprint/Python callable"),
             AddFunction != nullptr && AddFunction->HasAnyFunctionFlags(FUNC_BlueprintCallable));
    UFunction* ValidateFunction = Actor->FindFunction(TEXT("ValidateVisualGroup"));
    TestTrue(TEXT("ValidateVisualGroup is Blueprint/Python callable"),
             ValidateFunction != nullptr &&
                 ValidateFunction->HasAnyFunctionFlags(FUNC_BlueprintCallable));
    UFunction* ResetFunction = Actor->FindFunction(TEXT("ResetGroups"));
    TestTrue(TEXT("ResetGroups is Blueprint/Python callable"),
             ResetFunction != nullptr &&
                 ResetFunction->HasAnyFunctionFlags(FUNC_BlueprintCallable));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuSimVisualInstanceBatchActorFailureTest,
                                 "LingTuSim.Visual.FactoryPark.InstanceBatch.FailClosed",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuSimVisualInstanceBatchActorFailureTest::RunTest(const FString& Parameters)
{
    FScopedInstanceBatchTestWorld TestWorld;
    ALingTuSimVisualInstanceBatchActor* Actor = TestWorld.SpawnActor();
    TestNotNull(TEXT("batch actor spawns"), Actor);
    if (Actor == nullptr)
    {
        return false;
    }

    UStaticMesh* Mesh = NewObject<UStaticMesh>(Actor);
    UMaterialInterface* Material = UMaterial::GetDefaultMaterial(MD_Surface);
    const TArray<FTransform> Transforms = MakeTransforms();
    const TArray<FString> StableIds = MakeStableIds();
    TestTrue(TEXT("baseline group succeeds"),
             Actor->AddVisualGroup(TEXT("baseline"), Mesh, Material, Transforms, StableIds, false));

    auto TestRejectedWithoutResidue = [this, Actor](const TCHAR* Label, const bool bResult)
    {
        TArray<UHierarchicalInstancedStaticMeshComponent*> Components;
        GetHismComponents(*Actor, Components);
        TestFalse(Label, bResult);
        TestEqual(TEXT("failed add leaves group metadata unchanged"), Actor->VisualGroups.Num(), 1);
        TestEqual(TEXT("failed add leaves HISM component count unchanged"), Components.Num(), 1);
        if (Components.Num() == 1)
        {
            TestEqual(TEXT("failed add leaves instance count unchanged"),
                      Components[0]->GetInstanceCount(), 2);
        }
    };

    TestRejectedWithoutResidue(
        TEXT("duplicate group is rejected"),
        Actor->AddVisualGroup(TEXT("baseline"), Mesh, Material, Transforms, StableIds, false));
    TestRejectedWithoutResidue(
        TEXT("empty group ID is rejected"),
        Actor->AddVisualGroup(TEXT(""), Mesh, Material, Transforms, StableIds, false));
    TestRejectedWithoutResidue(
        TEXT("null mesh is rejected"),
        Actor->AddVisualGroup(TEXT("null_mesh"), nullptr, Material, Transforms, StableIds, false));
    TestRejectedWithoutResidue(
        TEXT("null material is rejected"),
        Actor->AddVisualGroup(TEXT("null_material"), Mesh, nullptr, Transforms, StableIds, false));
    TestRejectedWithoutResidue(TEXT("count mismatch is rejected"),
                               Actor->AddVisualGroup(TEXT("mismatch"), Mesh, Material, Transforms,
                                                     {TEXT("factory_park/one")}, false));
    TestRejectedWithoutResidue(TEXT("empty stable ID is rejected"),
                               Actor->AddVisualGroup(TEXT("empty_stable_id"), Mesh, Material,
                                                     Transforms,
                                                     {TEXT("factory_park/new"), TEXT("")}, false));
    TArray<FTransform> ZeroScaleTransforms = Transforms;
    ZeroScaleTransforms[0].SetScale3D(FVector::ZeroVector);
    TestRejectedWithoutResidue(
        TEXT("zero-scale transform is rejected"),
        Actor->AddVisualGroup(TEXT("zero_scale"), Mesh, Material, ZeroScaleTransforms,
                              {TEXT("factory_park/new_1"), TEXT("factory_park/new_2")}, false));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuSimVisualInstanceBatchActorResetTest,
                                 "LingTuSim.Visual.FactoryPark.InstanceBatch.Reset",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)

bool FLingTuSimVisualInstanceBatchActorResetTest::RunTest(const FString& Parameters)
{
    FScopedInstanceBatchTestWorld TestWorld;
    ALingTuSimVisualInstanceBatchActor* Actor = TestWorld.SpawnActor();
    TestNotNull(TEXT("batch actor spawns"), Actor);
    if (Actor == nullptr)
    {
        return false;
    }

    UStaticMesh* Mesh = NewObject<UStaticMesh>(Actor);
    UMaterialInterface* Material = UMaterial::GetDefaultMaterial(MD_Surface);
    TestTrue(TEXT("group succeeds before reset"),
             Actor->AddVisualGroup(TEXT("reset_group"), Mesh, Material, MakeTransforms(),
                                   MakeStableIds(), false));
    Actor->ResetGroups();

    TArray<UHierarchicalInstancedStaticMeshComponent*> Components;
    GetHismComponents(*Actor, Components);
    TestEqual(TEXT("reset removes all metadata"), Actor->VisualGroups.Num(), 0);
    TestEqual(TEXT("reset removes all HISM components"), Components.Num(), 0);
    TestTrue(TEXT("same group can be recreated after reset"),
             Actor->AddVisualGroup(TEXT("reset_group"), Mesh, Material, MakeTransforms(),
                                   MakeStableIds(), false));
    return true;
}

#endif

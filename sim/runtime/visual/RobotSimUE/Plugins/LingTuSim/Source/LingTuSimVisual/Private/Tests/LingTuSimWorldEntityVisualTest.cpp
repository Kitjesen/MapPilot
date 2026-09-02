#if WITH_DEV_AUTOMATION_TESTS

#include "LingTuSimBodyActor.h"
#include "LingTuSimPresentationPolicy.h"
#include "LingTuSimRobotVisualProjection.h"
#include "LingTuSimWorldEntityActor.h"

#include "Components/StaticMeshComponent.h"
#include "Engine/World.h"
#include "Engine/WorldInitializationValues.h"
#include "HAL/FileManager.h"
#include "Misc/AutomationTest.h"
#include "Misc/FileHelper.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"

namespace
{
    FString MinimalArtifactJson(const TCHAR* Schema, const FString& SessionId)
    {
        return FString::Printf(
            TEXT("{\"schema\":\"%s\",\"session_id\":\"%s\"}"),
            Schema,
            *SessionId);
    }

    UWorld* CreateWorldEntityTestWorld()
    {
        const FWorldInitializationValues InitializationValues = FWorldInitializationValues()
            .AllowAudioPlayback(false)
            .RequiresHitProxies(false)
            .CreatePhysicsScene(false)
            .CreateNavigation(false)
            .CreateAISystem(false)
            .ShouldSimulatePhysics(false)
            .SetTransactional(false);
        return UWorld::CreateWorld(
            EWorldType::Game,
            false,
            FName(*FString::Printf(
                TEXT("LingTuWorldEntityVisualTest_%s"),
                *FGuid::NewGuid().ToString(EGuidFormats::Digits))),
            nullptr,
            true,
            ERHIFeatureLevel::Num,
            &InitializationValues);
    }

    bool WriteWorldEntityBundle(
        const FString& Directory,
        const FString& SessionId,
        FString& OutError)
    {
        IFileManager::Get().MakeDirectory(*Directory, true);
        const TMap<FString, FString> Artifacts = {
            {TEXT("physics.plan.json"), MinimalArtifactJson(TEXT("lingtu.sim.physics-plan.v1"), SessionId)},
            {TEXT("sensor.plan.json"), MinimalArtifactJson(TEXT("lingtu.sim.sensor-plan.v1"), SessionId)},
            {TEXT("control.plan.json"), MinimalArtifactJson(TEXT("lingtu.sim.control-plan.v1"), SessionId)},
            {TEXT("transport.intent.json"), MinimalArtifactJson(TEXT("lingtu.sim.transport-intent.v1"), SessionId)},
        };
        for (const TPair<FString, FString>& Artifact : Artifacts)
        {
            if (!FFileHelper::SaveStringToFile(
                    Artifact.Value,
                    *FPaths::Combine(Directory, Artifact.Key),
                    FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))
            {
                OutError = Artifact.Key;
                return false;
            }
        }

        FString ProjectionRoot = FPaths::Combine(
            FPaths::ProjectDir(),
            TEXT("Saved"),
            TEXT("Automation"),
            TEXT("LingTuSimVisual"),
            TEXT("WorldEntityProjection"),
            FGuid::NewGuid().ToString(EGuidFormats::Digits));
        IFileManager::Get().MakeDirectory(*ProjectionRoot, true);
        FString RobotProjectionPath = FPaths::Combine(ProjectionRoot, TEXT("robot.visual-projection.json"));
        FString WorldProjectionPath = FPaths::Combine(ProjectionRoot, TEXT("world.visual-projection.json"));
        const FString RobotProjection = FString::Printf(
            TEXT("{\"schema\":\"lingtu.sim.robot-visual-projection.v1\",")
            TEXT("\"binding\":\"RobotVisual:OmniCart\",")
            TEXT("\"package\":{\"id\":\"omni_cart\",\"version\":\"1.0.0\",\"manifest\":\"robot.package.yaml\"},")
            TEXT("\"mjcf\":{\"path\":\"mjcf/robot.xml\"},")
            TEXT("\"components\":[{\"local_body_id\":\"base\",\"body_frame_id\":\"base\",")
            TEXT("\"visual_id\":\"base_visual\",\"visual_frame_id\":\"base/visual/base\",\"asset_key\":\"base_visual\",")
            TEXT("\"geometry\":{\"kind\":\"primitive\",\"primitive\":\"box\",\"size\":[0.2,0.1,0.05]},")
            TEXT("\"unreal\":{\"representation\":\"static_mesh\",\"component_class\":\"/Script/Engine.StaticMeshComponent\",\"asset_path\":\"/Engine/BasicShapes/Cube.Cube\",\"dimensions_m\":[0.4,0.2,0.1]},")
            TEXT("\"local_transform\":{\"position_m\":[0.0,0.0,0.0],\"quaternion_wxyz\":[1.0,0.0,0.0,0.0],\"scale\":[1.0,1.0,1.0]},")
            TEXT("\"material_key\":null,\"material\":{\"source\":\"compiler_default\",\"key\":null,\"pbr\":{\"base_color_rgba\":[0.4,0.4,0.4,1.0],\"metallic\":0.0,\"roughness\":0.65}},")
            TEXT("\"source\":{\"manifest_schema\":\"lingtu.sim.robot-visual-manifest.v1\",\"package\":{\"id\":\"omni_cart\",\"version\":\"1.0.0\",\"manifest\":\"robot.package.yaml\"},\"mjcf\":{\"path\":\"mjcf/robot.xml\"},\"geometry_source\":{\"kind\":\"primitive\",\"primitive\":\"box\"}}}]}"));
        const FString WorldProjection = FString::Printf(
            TEXT("{\"schema\":\"lingtu.sim.world-visual-projection.v1\",")
            TEXT("\"package\":{\"id\":\"factory_layout\",\"version\":\"1.0.0\",\"manifest\":\"world.package.yaml\",\"provenance\":\"provenance/world.provenance.json\"},")
            TEXT("\"binding\":\"WorldVisual:FactoryLayout\",\"level\":\"/Game/RobotSim/Maps/Test\",")
            TEXT("\"units\":{\"length\":\"m\",\"up_axis\":\"Z\",\"handedness\":\"RH\"},")
            TEXT("\"terrain\":{\"grid_px\":[2,2],\"extent_m\":[2.0,2.0],\"sample_spacing_m\":[2.0,2.0],\"physics_bounds_m\":{\"min_m\":[-1.0,-1.0,0.0],\"max_m\":[1.0,1.0,1.0]},\"visual_bounds_m\":{\"min_m\":[-1.0,-1.0,0.0],\"max_m\":[1.0,1.0,1.0]},")
            TEXT("\"assets\":[{\"role\":\"heightmap\",\"path\":\"artifacts/heightmap.png\",\"bytes\":4},{\"role\":\"render_mesh\",\"path\":\"artifacts/terrain.fbx\",\"bytes\":4},{\"role\":\"collision\",\"path\":\"artifacts/collision.obj\",\"bytes\":4,\"collision\":true}]},")
            TEXT("\"entities\":[{\"entity_id\":\"traffic_cone_01\",\"semantic_class\":\"traffic_cone\",\"authority\":\"mujoco\",")
            TEXT("\"transform\":{\"position_m\":[4.0,-6.0,0.35],\"quaternion_wxyz\":[1.0,0.0,0.0,0.0]},")
            TEXT("\"geometry\":{\"shape\":\"cylinder\",\"radius_m\":0.2,\"half_height_m\":0.35},")
            TEXT("\"unreal\":{\"representation\":\"static_mesh\",\"component_class\":\"/Script/Engine.StaticMeshComponent\",\"asset_path\":\"/Engine/BasicShapes/Cylinder.Cylinder\",\"dimensions_m\":[0.4,0.4,0.7]},")
            TEXT("\"material\":{\"source\":\"world_package\",\"key\":\"container_orange\",\"pbr\":{\"base_color_rgba\":[0.82,0.32,0.06,1.0],\"metallic\":0.0,\"roughness\":0.4}}}],")
            TEXT("\"spawn_alignment\":{\"position_m\":[0.0,0.0,0.0],\"aligned_to_heightmap\":true}}"));
        if (!FFileHelper::SaveStringToFile(RobotProjection, *RobotProjectionPath)
            || !FFileHelper::SaveStringToFile(WorldProjection, *WorldProjectionPath))
        {
            OutError = TEXT("projection_write");
            return false;
        }
        FString RobotRelative = RobotProjectionPath;
        FString WorldRelative = WorldProjectionPath;
        if (!FPaths::MakePathRelativeTo(RobotRelative, *FPaths::ProjectDir())
            || !FPaths::MakePathRelativeTo(WorldRelative, *FPaths::ProjectDir()))
        {
            OutError = TEXT("projection_relative_path");
            return false;
        }
        FPaths::NormalizeFilename(RobotRelative);
        FPaths::NormalizeFilename(WorldRelative);
        const FString VisualPlan = FString::Printf(
            TEXT("{\"schema\":\"lingtu.sim.visual-plan.v1\",\"session_id\":\"%s\",")
            TEXT("\"backends\":{\"physics\":\"mujoco\",\"visual\":\"unreal\"},")
            TEXT("\"coordinate_system\":{\"source\":\"mujoco_rh_z_up_m\",\"target\":\"unreal_lh_z_up_cm\",\"position_scale\":100.0,\"axis_mapping\":[\"x\",\"-y\",\"z\"],\"quaternion_order\":\"wxyz\"},")
            TEXT("\"binding_policy\":{\"missing_asset\":\"fail\",\"data_asset_is_projection\":true},")
            TEXT("\"world\":{\"package\":{\"id\":\"factory_layout\",\"version\":\"1.0.0\",\"kind\":\"world\",\"manifest\":\"sim/packages/worlds/factory_layout/1.0.0/world.package.yaml\"},\"binding\":\"WorldVisual:FactoryLayout\",\"level\":\"/Game/RobotSim/Maps/Test\",\"projection\":{\"schema\":\"lingtu.sim.world-visual-projection.v1\",\"path\":\"%s\"}},")
            TEXT("\"robots\":[{\"instance_id\":\"cart_01\",\"namespace\":\"cart_01\",\"package\":{\"id\":\"omni_cart\",\"version\":\"1.0.0\",\"kind\":\"robot\",\"manifest\":\"sim/packages/robots/omni_cart/robot.package.yaml\"},\"binding\":\"RobotVisual:OmniCart\",\"projection\":{\"schema\":\"lingtu.sim.robot-visual-projection.v1\",\"path\":\"%s\"},\"spawn\":{\"position_m\":[0.0,0.0,0.0],\"quaternion_wxyz\":[1.0,0.0,0.0,0.0]}}]}"),
            *SessionId,
            *WorldRelative,
            *RobotRelative);
        return FFileHelper::SaveStringToFile(
            VisualPlan,
            *FPaths::Combine(Directory, TEXT("visual.plan.json")),
            FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuWorldEntityProjectionMaterializesTest,
    "LingTuSim.Visual.Runtime.WorldProjection.MaterializesStaticEntity",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuWorldEntityProjectionMaterializesTest::RunTest(const FString& Parameters)
{
    const FString Directory = FPaths::Combine(
        FPaths::ProjectSavedDir(),
        TEXT("Automation"),
        TEXT("LingTuSimVisual"),
        TEXT("WorldEntityBundle"),
        FGuid::NewGuid().ToString(EGuidFormats::Digits));
    const FString SessionId = TEXT("world-entity-session");
    FString FixtureError;
    TestTrue(
        TEXT("world entity SessionBundle fixture writes"),
        WriteWorldEntityBundle(Directory, SessionId, FixtureError));
    if (!FixtureError.IsEmpty())
    {
        AddError(FixtureError);
        return false;
    }

    UWorld* World = CreateWorldEntityTestWorld();
    TestNotNull(TEXT("world entity test world exists"), World);
    LingTuSim::Visual::FVisualMaterializationResult Result;
    LingTuSim::Visual::FVisualMaterializationError Error;
    const bool bMaterialized = World != nullptr
        && LingTuSim::Visual::FRobotVisualProjectionMaterializer::MaterializeBundle(
            *World,
            Directory,
            FPaths::ProjectDir(),
            3,
            2,
            Result,
            Error);
    TestTrue(
        *FString::Printf(TEXT("world projection materializes; error='%s:%s'"), *Error.Source, *Error.Message),
        bMaterialized);
    TestEqual(TEXT("robot body count remains independent"), Result.ExpectedBodyCount, 1);
    TestEqual(TEXT("one runtime world entity materializes"), Result.ExpectedWorldEntityCount, 1);
    TestEqual(TEXT("one world entity actor is returned"), Result.WorldActors.Num(), 1);
    if (Result.WorldActors.Num() == 1 && IsValid(Result.WorldActors[0]))
    {
        ALingTuSimWorldEntityActor* Actor = Result.WorldActors[0];
        TestEqual(TEXT("stable world identity"), Actor->StableId, FString(TEXT("world/traffic_cone_01")));
        TestEqual(TEXT("semantic class survives"), Actor->SemanticClass, FString(TEXT("traffic_cone")));
        TestEqual(TEXT("physics authority survives"), Actor->Authority, FString(TEXT("mujoco")));
        TestTrue(TEXT("candidate world entity starts hidden"), Actor->IsHidden());
        TestTrue(
            TEXT("MuJoCo RH metres convert to Unreal LH centimetres"),
            Actor->GetActorLocation().Equals(FVector(400.0, 600.0, 35.0), 0.01));
        TArray<UStaticMeshComponent*> Meshes;
        Actor->GetComponents<UStaticMeshComponent>(Meshes);
        TestEqual(TEXT("one world visual mesh exists"), Meshes.Num(), 1);
        if (Meshes.Num() == 1)
        {
            TestTrue(
                TEXT("primitive dimensions become one exact UE scale"),
                Meshes[0]->GetRelativeScale3D().Equals(FVector(0.4, 0.4, 0.7), 1.0e-6));
            TestEqual(
                TEXT("MuJoCo remains collision authority"),
                Meshes[0]->GetCollisionEnabled(),
                ECollisionEnabled::NoCollision);
            TestTrue(
                TEXT("world mesh satisfies the complete presentation-only policy"),
                LingTuSim::Visual::HasPresentationPolicy(*Meshes[0]));
            TestNotNull(TEXT("runtime material is assigned"), Meshes[0]->GetMaterial(0));
        }
    }

    for (ALingTuSimWorldEntityActor* Actor : Result.WorldActors)
    {
        if (IsValid(Actor))
        {
            Actor->Destroy();
        }
    }
    for (ALingTuSimBodyActor* Actor : Result.Actors)
    {
        if (IsValid(Actor))
        {
            Actor->Destroy();
        }
    }
    if (World != nullptr)
    {
        World->DestroyWorld(false);
    }
    IFileManager::Get().DeleteDirectory(*Directory, false, true);
    return true;
}

#endif

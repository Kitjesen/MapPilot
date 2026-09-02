#if WITH_DEV_AUTOMATION_TESTS

#include "LingTuSimBodyBindingComponent.h"
#include "LingTuSimBundleLoader.h"
#include "LingTuSimBodyActor.h"
#include "LingTuSimPresentationPolicy.h"
#include "LingTuSimRobotVisualProjection.h"
#include "LingTuSimSessionService.h"
#include "LingTuSimVisualBoundary.h"
#include "LingTuSimVisualSnapshotGate.h"
#include "LingTuSimVisualTransform.h"
#include "LingTuSimVisualWorldSubsystem.h"

#include "Misc/AutomationTest.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/CommandLine.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "Engine/WorldInitializationValues.h"
#include "GameFramework/WorldSettings.h"
#include "Camera/CameraActor.h"
#include "GameFramework/PlayerController.h"
#include "EngineUtils.h"
#include "Components/CapsuleComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

#include <type_traits>

static_assert(
    std::is_base_of<ILingTuSimVisualBoundary, ULingTuSimVisualWorldSubsystem>::value,
    "ULingTuSimVisualWorldSubsystem must be the concrete visual boundary implementation.");

namespace
{
    LingTuSim::FSnapshotEnvelope MakeSnapshot(
        const FString& SessionId,
        const uint64 Model,
        const uint64 Reset,
        const uint64 Sequence)
    {
        LingTuSim::FSnapshotEnvelope Snapshot;
        Snapshot.SessionId = SessionId;
        Snapshot.ModelGeneration = Model;
        Snapshot.ResetGeneration = Reset;
        Snapshot.Sequence = Sequence;
        Snapshot.PhysicsStep = Sequence;
        return Snapshot;
    }

    FString MakeSnapshotJson(
        const FString& SessionId,
        const uint64 Model,
        const uint64 Reset,
        const uint64 Sequence)
    {
        return FString::Printf(
            TEXT("{\"schema\":\"lingtu.sim.truth-snapshot.v1\",\"session_id\":\"%s\",\"model_generation\":%llu,\"reset_generation\":%llu,\"sequence\":%llu,\"physics_step\":%llu,\"sim_time_ns\":0,\"bodies\":[]}"),
            *SessionId,
            static_cast<unsigned long long>(Model),
            static_cast<unsigned long long>(Reset),
            static_cast<unsigned long long>(Sequence),
            static_cast<unsigned long long>(Sequence));
    }

    LingTuSim::FSnapshotEnvelope MakeBodySnapshot(
        const FString& SessionId,
        const uint64 Model,
        const uint64 Reset,
        const uint64 Sequence,
        const FString& StableId)
    {
        TArray<FString> StableIds;
        StableIds.Add(StableId);
        LingTuSim::FSnapshotEnvelope Snapshot = MakeSnapshot(SessionId, Model, Reset, Sequence);
        for (const FString& BodyStableId : StableIds)
        {
            LingTuSim::FEntityState& Entity = Snapshot.Entities.AddDefaulted_GetRef();
            Entity.Id.StableId = BodyStableId;
            Entity.Id.InstanceId = BodyStableId;
            Entity.Id.FrameId = BodyStableId;
            Entity.PositionMeters = FVector(1.0, 2.0, 0.5);
            Entity.Rotation = FQuat::Identity;
        }
        return Snapshot;
    }

    LingTuSim::FSnapshotEnvelope MakeBodySnapshot(
        const FString& SessionId,
        const uint64 Model,
        const uint64 Reset,
        const uint64 Sequence,
        const TArray<FString>& StableIds)
    {
        LingTuSim::FSnapshotEnvelope Snapshot = MakeSnapshot(SessionId, Model, Reset, Sequence);
        for (int32 Index = 0; Index < StableIds.Num(); ++Index)
        {
            LingTuSim::FEntityState& Entity = Snapshot.Entities.AddDefaulted_GetRef();
            Entity.Id.StableId = StableIds[Index];
            Entity.Id.InstanceId = StableIds[Index];
            Entity.Id.FrameId = StableIds[Index];
            Entity.PositionMeters = FVector(1.0 + static_cast<double>(Index), 2.0, 0.5);
            Entity.Rotation = FQuat::Identity;
        }
        return Snapshot;
    }

    FString MakeEvidenceDirectory(const FString& TestName)
    {
        const FString Directory = FPaths::Combine(
            FPaths::ProjectSavedDir(),
            TEXT("Automation"),
            TEXT("LingTuSimVisual"),
            TestName + TEXT("-") + FGuid::NewGuid().ToString(EGuidFormats::Digits));
        IFileManager::Get().MakeDirectory(*Directory, true);
        return Directory;
    }

    bool LoadEvidenceObject(
        const FString& Directory,
        TSharedPtr<FJsonObject>& OutObject)
    {
        FString Json;
        if (!FFileHelper::LoadFileToString(
                Json,
                *FPaths::Combine(Directory, TEXT("visual-readiness.json"))))
        {
            return false;
        }
        TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Json);
        return FJsonSerializer::Deserialize(Reader, OutObject) && OutObject.IsValid();
    }

    void CheckVisualEvidence(
        FAutomationTestBase& Test,
        const TSharedPtr<FJsonObject>& Evidence,
        const FString& ExpectedState,
        const FString& ExpectedSessionId,
        const int32 ExpectedModelGeneration,
        const int32 ExpectedResetGeneration)
    {
        Test.TestTrue(TEXT("readiness evidence JSON is an object"), Evidence.IsValid());
        if (!Evidence.IsValid())
        {
            return;
        }
        Test.TestEqual(TEXT("top-level field count is strict"), Evidence->Values.Num(), 9);
        Test.TestEqual(TEXT("schema matches ExternalRuntimeEvidence"), Evidence->GetStringField(TEXT("schema")), FString(TEXT("lingtu.sim.sensor-readiness-evidence.v1")));
        Test.TestEqual(TEXT("session_id is current"), Evidence->GetStringField(TEXT("session_id")), ExpectedSessionId);
        Test.TestEqual(TEXT("model_generation is current"), static_cast<int32>(Evidence->GetNumberField(TEXT("model_generation"))), ExpectedModelGeneration);
        Test.TestEqual(TEXT("reset_generation is current"), static_cast<int32>(Evidence->GetNumberField(TEXT("reset_generation"))), ExpectedResetGeneration);
        Test.TestEqual(TEXT("source_id is visual-owned"), Evidence->GetStringField(TEXT("source_id")), FString(TEXT("robotsimue-visual")));
        Test.TestEqual(TEXT("basis proves truth snapshot application"), Evidence->GetStringField(TEXT("basis")), FString(TEXT("truth_snapshot_applied_to_visual_bindings")));

        const TSharedPtr<FJsonObject>* Visual = nullptr;
        Test.TestTrue(TEXT("visual object exists"), Evidence->TryGetObjectField(TEXT("visual"), Visual) && Visual != nullptr && Visual->IsValid());
        if (Visual != nullptr && Visual->IsValid())
        {
            Test.TestEqual(TEXT("visual has only state"), (*Visual)->Values.Num(), 1);
            Test.TestEqual(TEXT("visual state matches"), (*Visual)->GetStringField(TEXT("state")), ExpectedState);
            Test.TestFalse(TEXT("visual.reason is absent outside FAILED"), (*Visual)->HasField(TEXT("reason")));
        }

        const TSharedPtr<FJsonObject>* Sensors = nullptr;
        Test.TestTrue(TEXT("sensors object exists"), Evidence->TryGetObjectField(TEXT("sensors"), Sensors) && Sensors != nullptr && Sensors->IsValid());
        if (Sensors != nullptr && Sensors->IsValid())
        {
            Test.TestEqual(TEXT("sensors has only camera_streams and overall"), (*Sensors)->Values.Num(), 2);
            Test.TestEqual(TEXT("camera streams stay preparing"), (*Sensors)->GetStringField(TEXT("camera_streams")), FString(TEXT("PREPARING")));
            Test.TestEqual(TEXT("sensor overall stays preparing"), (*Sensors)->GetStringField(TEXT("overall")), FString(TEXT("PREPARING")));
        }

        const TArray<TSharedPtr<FJsonValue>>* Streams = nullptr;
        Test.TestTrue(TEXT("streams array exists"), Evidence->TryGetArrayField(TEXT("streams"), Streams) && Streams != nullptr);
        if (Streams != nullptr)
        {
            Test.TestEqual(TEXT("visual evidence owns no sensor streams"), Streams->Num(), 0);
        }
    }

    FString MinimalArtifactJson(const TCHAR* Schema, const FString& Session)
    {
        return FString::Printf(
            TEXT("{\"schema\":\"%s\",\"session_id\":\"%s\"}"),
            Schema,
            *Session);
    }

    FString ProjectionJson(
        const FString& Binding,
        const FString& PackageManifest,
        const FString& GeometryKind,
        const FString& Primitive,
        const FString& VisualFrameId,
        const FString& AssetPath,
        const FString& GeometryScale,
        const FString& LocalScale)
    {
        const FString Geometry = GeometryKind == TEXT("mesh")
            ? FString::Printf(
                TEXT("\"geometry\":{\"kind\":\"mesh\",\"source_mesh\":\"meshes/cube.stl\",\"mesh\":\"cube\",\"scale\":%s},"),
                *GeometryScale)
            : FString::Printf(
                TEXT("\"geometry\":{\"kind\":\"primitive\",\"primitive\":\"%s\",\"size\":[1.0,2.0,0.5]},"),
                *Primitive);
        const FString Unreal = GeometryKind == TEXT("mesh")
            ? FString::Printf(
                TEXT("\"unreal\":{\"representation\":\"static_mesh\",\"component_class\":\"/Script/Engine.StaticMeshComponent\",\"asset_path\":\"%s\"},"),
                *AssetPath)
            : TEXT("\"unreal\":{\"representation\":\"static_mesh\",\"component_class\":\"/Script/Engine.StaticMeshComponent\",\"asset_path\":\"/Engine/BasicShapes/Cube.Cube\",\"dimensions_m\":[1.0,2.0,0.5]},");
        return FString::Printf(
            TEXT("{\"schema\":\"lingtu.sim.robot-visual-projection.v1\",")
            TEXT("\"binding\":\"%s\",")
            TEXT("\"package\":{\"id\":\"omni_cart\",\"version\":\"1.0.0\",\"manifest\":\"%s\"},")
            TEXT("\"mjcf\":{\"path\":\"mjcf/robot.xml\"},")
            TEXT("\"components\":[{\"local_body_id\":\"base\",")
            TEXT("\"body_frame_id\":\"base\",")
            TEXT("\"visual_id\":\"base_visual\",")
            TEXT("\"visual_frame_id\":\"%s\",")
            TEXT("\"asset_key\":\"base_visual\",")
            TEXT("%s%s")
            TEXT("\"local_transform\":{\"position_m\":[0.0,0.0,0.0],\"quaternion_wxyz\":[1.0,0.0,0.0,0.0],\"scale\":%s},")
            TEXT("\"material_key\":null,")
            TEXT("\"material\":{\"source\":\"compiler_default\",\"key\":null,\"pbr\":{\"base_color_rgba\":[1.0,1.0,1.0,1.0],\"metallic\":0.0,\"roughness\":0.65}},")
            TEXT("\"source\":{\"manifest_schema\":\"lingtu.sim.robot-visual-manifest.v1\",\"package\":{\"id\":\"omni_cart\",\"version\":\"1.0.0\",\"manifest\":\"%s\"},\"mjcf\":{\"path\":\"mjcf/robot.xml\"},\"geometry_source\":{\"kind\":\"primitive\",\"primitive\":\"box\"}}}]}"),
            *Binding,
            *PackageManifest,
            *VisualFrameId,
            *Geometry,
            *Unreal,
            *LocalScale,
            *PackageManifest);
    }

    FString ProjectionJsonWithMaterial(const FString& MaterialJson)
    {
        FString Json = ProjectionJson(
            TEXT("RobotVisual:OmniCart"),
            TEXT("robot.package.yaml"),
            TEXT("primitive"),
            TEXT("box"),
            TEXT("base_visual_frame"),
            TEXT("/Engine/BasicShapes/Cube.Cube"),
            TEXT("[1.0,1.0,1.0]"),
            TEXT("[1.0,1.0,1.0]"));
        Json.ReplaceInline(
            TEXT("\"material\":{\"source\":\"compiler_default\",\"key\":null,\"pbr\":{\"base_color_rgba\":[1.0,1.0,1.0,1.0],\"metallic\":0.0,\"roughness\":0.65}},"),
            *FString::Printf(TEXT("\"material\":%s,"), *MaterialJson));
        return Json;
    }

    FString CapsuleProjectionJsonWithMaterial(const FString& MaterialJson)
    {
        return FString::Printf(
            TEXT("{\"schema\":\"lingtu.sim.robot-visual-projection.v1\",")
            TEXT("\"binding\":\"RobotVisual:OmniCart\",")
            TEXT("\"package\":{\"id\":\"omni_cart\",\"version\":\"1.0.0\",\"manifest\":\"robot.package.yaml\"},")
            TEXT("\"mjcf\":{\"path\":\"mjcf/robot.xml\"},")
            TEXT("\"components\":[{\"local_body_id\":\"base\",")
            TEXT("\"body_frame_id\":\"base\",")
            TEXT("\"visual_id\":\"base_capsule\",")
            TEXT("\"visual_frame_id\":\"base_capsule_frame\",")
            TEXT("\"asset_key\":\"base_capsule\",")
            TEXT("\"geometry\":{\"kind\":\"primitive\",\"primitive\":\"capsule\",\"size\":[0.25,1.0]},")
            TEXT("\"unreal\":{\"representation\":\"component\",\"component_class\":\"/Script/Engine.CapsuleComponent\",\"radius_m\":0.25,\"capsule_half_height_m\":0.5,\"cylinder_half_length_m\":0.5,\"dimensions_m\":[0.5,0.5,1.0]},")
            TEXT("\"local_transform\":{\"position_m\":[0.0,0.0,0.0],\"quaternion_wxyz\":[1.0,0.0,0.0,0.0],\"scale\":[1.0,1.0,1.0]},")
            TEXT("\"material_key\":null,\"material\":%s,")
            TEXT("\"source\":{\"manifest_schema\":\"lingtu.sim.robot-visual-manifest.v1\",\"package\":{\"id\":\"omni_cart\",\"version\":\"1.0.0\",\"manifest\":\"robot.package.yaml\"},\"mjcf\":{\"path\":\"mjcf/robot.xml\"},\"geometry_source\":{\"kind\":\"primitive\",\"primitive\":\"capsule\"}}}]}"),
            *MaterialJson);
    }

    FString PrimitiveProjectionComponentJson(
        const FString& LocalBodyId,
        const FString& VisualFrameId,
        const FString& VisualId)
    {
        return FString::Printf(
            TEXT("{\"local_body_id\":\"%s\",")
            TEXT("\"body_frame_id\":\"%s\",")
            TEXT("\"visual_id\":\"%s\",")
            TEXT("\"visual_frame_id\":\"%s\",")
            TEXT("\"asset_key\":\"%s\",")
            TEXT("\"geometry\":{\"kind\":\"primitive\",\"primitive\":\"box\",\"size\":[1.0,2.0,0.5]},")
            TEXT("\"unreal\":{\"representation\":\"static_mesh\",\"component_class\":\"/Script/Engine.StaticMeshComponent\",\"asset_path\":\"/Engine/BasicShapes/Cube.Cube\",\"dimensions_m\":[1.0,2.0,0.5]},")
            TEXT("\"local_transform\":{\"position_m\":[0.0,0.0,0.0],\"quaternion_wxyz\":[1.0,0.0,0.0,0.0],\"scale\":[1.0,1.0,1.0]},")
            TEXT("\"material_key\":null,")
            TEXT("\"material\":{\"source\":\"compiler_default\",\"key\":null,\"pbr\":{\"base_color_rgba\":[1.0,1.0,1.0,1.0],\"metallic\":0.0,\"roughness\":0.65}},")
            TEXT("\"source\":{\"manifest_schema\":\"lingtu.sim.robot-visual-manifest.v1\",\"package\":{\"id\":\"omni_cart\",\"version\":\"1.0.0\",\"manifest\":\"robot.package.yaml\"},\"mjcf\":{\"path\":\"mjcf/robot.xml\"},\"geometry_source\":{\"kind\":\"primitive\",\"primitive\":\"box\"}}}"),
            *LocalBodyId,
            *LocalBodyId,
            *VisualId,
            *VisualFrameId,
            *VisualId);
    }

    FString MultiBodyProjectionJson(const TArray<FString>& LocalBodyIds)
    {
        TArray<FString> Components;
        for (const FString& LocalBodyId : LocalBodyIds)
        {
            Components.Add(PrimitiveProjectionComponentJson(
                LocalBodyId,
                LocalBodyId + TEXT("_visual_frame"),
                LocalBodyId + TEXT("_visual")));
        }
        return FString::Printf(
            TEXT("{\"schema\":\"lingtu.sim.robot-visual-projection.v1\",")
            TEXT("\"binding\":\"RobotVisual:OmniCart\",")
            TEXT("\"package\":{\"id\":\"omni_cart\",\"version\":\"1.0.0\",\"manifest\":\"robot.package.yaml\"},")
            TEXT("\"mjcf\":{\"path\":\"mjcf/robot.xml\"},")
            TEXT("\"components\":[%s]}"),
            *FString::Join(Components, TEXT(",")));
    }

    FString VisualPlanRobotEntryJson(
        const FString& InstanceId,
        const FString& ProjectionRelativePath)
    {
        return FString::Printf(
            TEXT("{\"instance_id\":\"%s\",\"namespace\":\"%s\",")
            TEXT("\"package\":{\"id\":\"omni_cart\",\"version\":\"1.0.0\",\"kind\":\"robot\",\"manifest\":\"sim/packages/robots/omni_cart/robot.package.yaml\"},")
            TEXT("\"binding\":\"RobotVisual:OmniCart\",")
            TEXT("\"projection\":{\"schema\":\"lingtu.sim.robot-visual-projection.v1\",\"path\":\"%s\"},")
            TEXT("\"spawn\":{\"position_m\":[0.0,0.0,0.0],\"quaternion_wxyz\":[1.0,0.0,0.0,0.0]}}"),
            *InstanceId,
            *InstanceId,
            *ProjectionRelativePath);
    }

    FString PayloadProjectionJson(const bool bAddUnknownField)
    {
        return FString::Printf(
            TEXT("{\"schema\":\"lingtu.sim.payload-visual-projection.v1\",")
            TEXT("\"package\":{\"id\":\"test_payload\",\"version\":\"1.0.0\",\"manifest\":\"payload.package.yaml\"},")
            TEXT("\"binding\":\"PayloadVisual:Test\",\"authority\":\"mujoco\",")
            TEXT("\"visual_policy\":{\"collision_enabled\":\"no_collision\",\"simulate_physics\":false,\"generate_overlap_events\":false},")
            TEXT("\"source_asset\":{\"path\":\"assets/test.glb\"},")
            TEXT("\"material_contract\":{\"source_format\":\"gltf2\",\"material_count\":1,\"required_channels\":[\"base_color\",\"normal\",\"metallic_roughness\"]},")
            TEXT("\"components\":[{\"mesh\":\"SM_RWS01_MountBase\",\"body_frame\":\"payload_base\",\"unreal_asset\":\"/Game/RobotSim/Payloads/FictionalRWS01/Runtime/rws-01-v002-runtime/StaticMeshes/SM_RWS01_MountBase.SM_RWS01_MountBase\",\"local_transform\":{\"position_m\":[0.0,0.0,0.0],\"quaternion_wxyz\":[1.0,0.0,0.0,0.0],\"scale\":[1.0,1.0,1.0]}}]%s}"),
            bAddUnknownField ? TEXT(",\"unexpected\":true") : TEXT(""));
    }

    bool RewriteVisualPlanWithPayloadV2(
        const FString& Directory,
        const FString& Session,
        const FString& RobotProjectionRelativePath,
        const FString& PayloadProjectionRelativePath,
        const bool bAddUnknownPayloadField)
    {
        const FString PayloadExtra = bAddUnknownPayloadField
            ? TEXT(",\"unexpected\":true")
            : TEXT("");
        const FString VisualPlan = FString::Printf(
            TEXT("{\"schema\":\"lingtu.sim.visual-plan.v2\",")
            TEXT("\"session_id\":\"%s\",")
            TEXT("\"backends\":{\"physics\":\"mujoco\",\"visual\":\"unreal\"},")
            TEXT("\"coordinate_system\":{\"source\":\"mujoco_rh_z_up_m\",\"target\":\"unreal_lh_z_up_cm\",\"position_scale\":100.0,\"axis_mapping\":[\"x\",\"-y\",\"z\"],\"quaternion_order\":\"wxyz\"},")
            TEXT("\"binding_policy\":{\"missing_asset\":\"fail\",\"data_asset_is_projection\":true},")
            TEXT("\"world\":{\"package\":{\"id\":\"open_field\",\"version\":\"1.0.0\",\"kind\":\"world\",\"manifest\":\"sim/packages/worlds/open_field/world.package.yaml\"},\"binding\":\"WorldVisual:OpenField\",\"level\":\"/Game/RobotSim/Maps/Test\"},")
            TEXT("\"robots\":[{\"instance_id\":\"cart_01\",\"namespace\":\"cart_01\",\"package\":{\"id\":\"omni_cart\",\"version\":\"1.0.0\",\"kind\":\"robot\",\"manifest\":\"sim/packages/robots/omni_cart/robot.package.yaml\"},\"binding\":\"RobotVisual:OmniCart\",\"projection\":{\"schema\":\"lingtu.sim.robot-visual-projection.v1\",\"path\":\"%s\"},\"spawn\":{\"position_m\":[0.0,0.0,0.0],\"quaternion_wxyz\":[1.0,0.0,0.0,0.0]},")
            TEXT("\"payloads\":[{\"instance_id\":\"payload_01\",\"namespace\":\"payload_01\",\"robot_instance_id\":\"cart_01\",\"package\":{\"id\":\"test_payload\",\"version\":\"1.0.0\",\"kind\":\"payload\",\"manifest\":\"sim/packages/payloads/test_payload/1.0.0/payload.package.yaml\"},\"parent_frame\":\"payload_top\",\"mount_transform\":{\"position_m\":[0.0,0.0,0.1],\"quaternion_wxyz\":[1.0,0.0,0.0,0.0]},\"binding\":\"PayloadVisual:Test\",\"projection\":{\"schema\":\"lingtu.sim.payload-visual-projection.v1\",\"path\":\"%s\"},\"authority\":\"mujoco\",\"ue_collision\":\"disabled\",\"frames\":[{\"name\":\"payload_base\",\"role\":\"payload_root\"}]%s}]}]}"),
            *Session,
            *RobotProjectionRelativePath,
            *PayloadProjectionRelativePath,
            *PayloadExtra);
        return FFileHelper::SaveStringToFile(
                   MinimalArtifactJson(TEXT("lingtu.sim.physics-plan.v2"), Session),
                   *FPaths::Combine(Directory, TEXT("physics.plan.json")),
                   FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM)
            && FFileHelper::SaveStringToFile(
                   VisualPlan,
                   *FPaths::Combine(Directory, TEXT("visual.plan.json")),
                   FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
    }

    bool RewriteVisualPlanRobots(
        const FString& Directory,
        const FString& Session,
        const TArray<FString>& RobotEntries)
    {
        const FString VisualPlan = FString::Printf(
            TEXT("{\"schema\":\"lingtu.sim.visual-plan.v1\",")
            TEXT("\"session_id\":\"%s\",")
            TEXT("\"backends\":{\"physics\":\"mujoco\",\"visual\":\"unreal\"},")
            TEXT("\"coordinate_system\":{\"source\":\"mujoco_rh_z_up_m\",\"target\":\"unreal_lh_z_up_cm\",\"position_scale\":100.0,\"axis_mapping\":[\"x\",\"-y\",\"z\"],\"quaternion_order\":\"wxyz\"},")
            TEXT("\"binding_policy\":{\"missing_asset\":\"fail\",\"data_asset_is_projection\":true},")
            TEXT("\"world\":{\"package\":{\"id\":\"open_field\",\"version\":\"1.0.0\",\"kind\":\"world\",\"manifest\":\"sim/packages/worlds/open_field/world.package.yaml\"},\"binding\":\"WorldVisual:OpenField\",\"level\":\"/Game/RobotSim/Maps/Test\"},")
            TEXT("\"robots\":[%s]}"),
            *Session,
            *FString::Join(RobotEntries, TEXT(",")));
        return FFileHelper::SaveStringToFile(
            VisualPlan,
            *FPaths::Combine(Directory, TEXT("visual.plan.json")),
            FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
    }

    FString ScenarioEntityProjectionJson()
    {
        return FString::Printf(
            TEXT("{\"schema\":\"lingtu.sim.entity-visual-projection.v1\",")
            TEXT("\"binding\":\"EntityVisual:PedestrianCapsule\",")
            TEXT("\"package\":{\"id\":\"crossing\",\"version\":\"1.0.0\",\"manifest\":\"scenario.package.yaml\"},")
            TEXT("\"mjcf\":{\"path\":\"physics/pedestrian.xml\"},")
            TEXT("\"components\":[{\"local_body_id\":\"proxy_root\",")
            TEXT("\"body_frame_id\":\"proxy_root\",")
            TEXT("\"visual_id\":\"pedestrian_capsule\",")
            TEXT("\"visual_frame_id\":\"proxy_root/visual/pedestrian_capsule\",")
            TEXT("\"asset_key\":\"EntityVisual:PedestrianCapsule/proxy_root/visual/pedestrian_capsule\",")
            TEXT("\"geometry\":{\"kind\":\"primitive\",\"primitive\":\"capsule\",\"size\":[0.3,0.6]},")
            TEXT("\"unreal\":{\"representation\":\"component\",\"component_class\":\"/Script/Engine.CapsuleComponent\",\"radius_m\":0.3,\"capsule_half_height_m\":0.9,\"cylinder_half_length_m\":0.6,\"dimensions_m\":[0.6,0.6,1.8]},")
            TEXT("\"local_transform\":{\"position_m\":[0.0,0.0,0.9],\"quaternion_wxyz\":[1.0,0.0,0.0,0.0],\"scale\":[1.0,1.0,1.0]},")
            TEXT("\"material_key\":null,")
            TEXT("\"material\":{\"source\":\"compiler_default\",\"key\":null,\"pbr\":{\"base_color_rgba\":[0.85,0.22,0.08,1.0],\"metallic\":0.0,\"roughness\":0.72}},")
            TEXT("\"source\":{\"manifest_schema\":\"lingtu.sim.scenario-package.v1\",\"package\":{\"id\":\"crossing\",\"version\":\"1.0.0\",\"manifest\":\"scenario.package.yaml\"},\"mjcf\":{\"path\":\"physics/pedestrian.xml\"},\"geometry_source\":{\"kind\":\"primitive\",\"primitive\":\"capsule\"}}}]}"));
    }

    bool RewriteVisualPlanWithScenarioEntity(
        const FString& Directory,
        const FString& Session,
        const FString& RobotEntry,
        const FString& ProjectionRelativePath)
    {
        const FString VisualPlan = FString::Printf(
            TEXT("{\"schema\":\"lingtu.sim.visual-plan.v1\",")
            TEXT("\"session_id\":\"%s\",")
            TEXT("\"backends\":{\"physics\":\"mujoco\",\"visual\":\"unreal\"},")
            TEXT("\"coordinate_system\":{\"source\":\"mujoco_rh_z_up_m\",\"target\":\"unreal_lh_z_up_cm\",\"position_scale\":100.0,\"axis_mapping\":[\"x\",\"-y\",\"z\"],\"quaternion_order\":\"wxyz\"},")
            TEXT("\"binding_policy\":{\"missing_asset\":\"fail\",\"data_asset_is_projection\":true},")
            TEXT("\"world\":{\"package\":{\"id\":\"open_field\",\"version\":\"1.0.0\",\"kind\":\"world\",\"manifest\":\"sim/packages/worlds/open_field/world.package.yaml\"},\"binding\":\"WorldVisual:OpenField\",\"level\":\"/Game/RobotSim/Maps/Test\"},")
            TEXT("\"robots\":[%s],")
            TEXT("\"scenario_entities\":[{\"entity_id\":\"pedestrian_01\",\"namespace\":\"pedestrian_01\",\"package\":{\"id\":\"crossing\",\"version\":\"1.0.0\",\"kind\":\"scenario\",\"manifest\":\"sim/packages/scenarios/crossing/scenario.package.yaml\"},\"binding\":\"EntityVisual:PedestrianCapsule\",\"projection\":{\"schema\":\"lingtu.sim.entity-visual-projection.v1\",\"path\":\"%s\"},\"spawn\":{\"position_m\":[4.0,-6.0,0.0],\"quaternion_wxyz\":[1.0,0.0,0.0,0.0]}}]}"),
            *Session,
            *RobotEntry,
            *ProjectionRelativePath);
        return FFileHelper::SaveStringToFile(
            VisualPlan,
            *FPaths::Combine(Directory, TEXT("visual.plan.json")),
            FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
    }

    bool WriteVisualBundle(
        const FString& Directory,
        const FString& ProjectionRelativePath,
        const FString& ProjectionJsonText,
        FString& OutError,
        const FString& Session = TEXT("session-e"))
    {
        IFileManager::Get().MakeDirectory(*Directory, true);
        const TMap<FString, FString> Artifacts = {
            {TEXT("physics.plan.json"), MinimalArtifactJson(TEXT("lingtu.sim.physics-plan.v1"), Session)},
            {TEXT("sensor.plan.json"), MinimalArtifactJson(TEXT("lingtu.sim.sensor-plan.v1"), Session)},
            {TEXT("control.plan.json"), MinimalArtifactJson(TEXT("lingtu.sim.control-plan.v1"), Session)},
            {TEXT("transport.intent.json"), MinimalArtifactJson(TEXT("lingtu.sim.transport-intent.v1"), Session)},
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

        const FString ProjectionPath = FPaths::Combine(FPaths::ProjectDir(), ProjectionRelativePath);
        IFileManager::Get().MakeDirectory(*FPaths::GetPath(ProjectionPath), true);
        if (!FFileHelper::SaveStringToFile(
                ProjectionJsonText,
                *ProjectionPath,
                FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))
        {
            OutError = ProjectionPath;
            return false;
        }
        const FString VisualPlan = FString::Printf(
            TEXT("{\"schema\":\"lingtu.sim.visual-plan.v1\",")
            TEXT("\"session_id\":\"%s\",")
            TEXT("\"backends\":{\"physics\":\"mujoco\",\"visual\":\"unreal\"},")
            TEXT("\"coordinate_system\":{\"source\":\"mujoco_rh_z_up_m\",\"target\":\"unreal_lh_z_up_cm\",\"position_scale\":100.0,\"axis_mapping\":[\"x\",\"-y\",\"z\"],\"quaternion_order\":\"wxyz\"},")
            TEXT("\"binding_policy\":{\"missing_asset\":\"fail\",\"data_asset_is_projection\":true},")
            TEXT("\"world\":{\"package\":{\"id\":\"open_field\",\"version\":\"1.0.0\",\"kind\":\"world\",\"manifest\":\"sim/packages/worlds/open_field/world.package.yaml\"},\"binding\":\"WorldVisual:OpenField\",\"level\":\"/Game/RobotSim/Maps/Test\"},")
            TEXT("\"robots\":[{\"instance_id\":\"cart_01\",\"namespace\":\"cart_01\",\"package\":{\"id\":\"omni_cart\",\"version\":\"1.0.0\",\"kind\":\"robot\",\"manifest\":\"sim/packages/robots/omni_cart/robot.package.yaml\"},\"binding\":\"RobotVisual:OmniCart\",\"projection\":{\"schema\":\"lingtu.sim.robot-visual-projection.v1\",\"path\":\"%s\"},\"spawn\":{\"position_m\":[0.0,0.0,0.0],\"quaternion_wxyz\":[1.0,0.0,0.0,0.0]}}]}"),
            *Session,
            *ProjectionRelativePath);
        return FFileHelper::SaveStringToFile(
            VisualPlan,
            *FPaths::Combine(Directory, TEXT("visual.plan.json")),
            FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
    }

    UWorld* CreateVisualTestWorld()
    {
        const FWorldInitializationValues InitializationValues = FWorldInitializationValues()
            .AllowAudioPlayback(false)
            .RequiresHitProxies(false)
            .CreatePhysicsScene(false)
            .CreateNavigation(false)
            .CreateAISystem(false)
            .ShouldSimulatePhysics(false)
            .SetTransactional(false);
        const FName WorldName(*FString::Printf(
            TEXT("LingTuSimVisualTestWorld_%s"),
            *FGuid::NewGuid().ToString(EGuidFormats::Digits)));
        return UWorld::CreateWorld(
            EWorldType::Game,
            false,
            WorldName,
            nullptr,
            true,
            ERHIFeatureLevel::Num,
            &InitializationValues);
    }

    class FScopedSessionServiceBinding final
    {
    public:
        FScopedSessionServiceBinding()
        {
            LingTuSim::FSessionService::UnbindSession();
        }

        ~FScopedSessionServiceBinding()
        {
            LingTuSim::FSessionService::UnbindSession();
        }
    };

    class FScopedVisualTestWorld final
    {
    public:
        FScopedVisualTestWorld()
            : World(CreateVisualTestWorld())
        {
        }

        ~FScopedVisualTestWorld()
        {
            if (World != nullptr)
            {
                World->DestroyWorld(false);
            }
        }

        UWorld* Get() const
        {
            return World;
        }

        ULingTuSimVisualWorldSubsystem* GetVisualSubsystem() const
        {
            return World != nullptr
                ? World->GetSubsystem<ULingTuSimVisualWorldSubsystem>()
                : nullptr;
        }

    private:
        UWorld* World = nullptr;
    };

    bool PathTraversesSymlinkForTest(const FString& Path)
    {
        FString Current = Path;
        while (!Current.IsEmpty())
        {
            if (IFileManager::Get().IsSymlink(*Current))
            {
                return true;
            }
            FString Parent = FPaths::GetPath(Current);
            FPaths::NormalizeDirectoryName(Parent);
            if (Parent.IsEmpty() || FPaths::IsSamePath(Current, Parent))
            {
                break;
            }
            Current = MoveTemp(Parent);
        }
        return false;
    }

    class FRunEvidenceFixture final
    {
    public:
        explicit FRunEvidenceFixture(const FString& Prefix)
        {
            RunId = Prefix + TEXT("-")
                + FGuid::NewGuid().ToString(EGuidFormats::Digits);
            RunDirectory = FPaths::ConvertRelativePathToFull(FPaths::Combine(
                FPaths::ProjectSavedDir(),
                TEXT("Automation"),
                TEXT("LingTuSimVisual"),
                TEXT("runs"),
                RunId));
            FPaths::NormalizeDirectoryName(RunDirectory);
            LogDirectory = FPaths::Combine(RunDirectory, TEXT("logs"));
            FPaths::NormalizeDirectoryName(LogDirectory);
            bCreated = IFileManager::Get().MakeDirectory(*LogDirectory, true);
        }

        ~FRunEvidenceFixture()
        {
            if (!RunDirectory.IsEmpty())
            {
                IFileManager::Get().DeleteDirectory(*RunDirectory, false, true);
            }
        }

        bool IsValid() const
        {
            return bCreated
                && !FPaths::IsRelative(RunDirectory)
                && !FPaths::IsRelative(LogDirectory)
                && FPaths::GetCleanFilename(RunDirectory) == RunId
                && FPaths::GetCleanFilename(LogDirectory) == TEXT("logs")
                && FPaths::IsSamePath(FPaths::GetPath(LogDirectory), RunDirectory)
                && IFileManager::Get().DirectoryExists(*LogDirectory)
                && !PathTraversesSymlinkForTest(LogDirectory);
        }

        FString RunId;
        FString RunDirectory;
        FString LogDirectory;

    private:
        bool bCreated = false;
    };

    int32 CountValidBodyActors(UWorld* World)
    {
        int32 Count = 0;
        if (World == nullptr)
        {
            return Count;
        }
        for (TActorIterator<ALingTuSimBodyActor> It(World); It; ++It)
        {
            if (IsValid(*It))
            {
                ++Count;
            }
        }
        return Count;
    }

    TMap<FString, ALingTuSimBodyActor*> CollectBodyActorsByStableId(UWorld* World)
    {
        TMap<FString, ALingTuSimBodyActor*> ActorsByStableId;
        if (World == nullptr)
        {
            return ActorsByStableId;
        }
        for (TActorIterator<ALingTuSimBodyActor> It(World); It; ++It)
        {
            ALingTuSimBodyActor* Actor = *It;
            if (IsValid(Actor) && Actor->BodyBinding != nullptr)
            {
                ActorsByStableId.Add(Actor->BodyBinding->StableId, Actor);
            }
        }
        return ActorsByStableId;
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimVisualCoordinateConversionTest,
    "LingTuSim.Visual.Runtime.CoordinateConversion",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuSimVisualCoordinateConversionTest::RunTest(const FString& Parameters)
{
    (void)Parameters;

    LingTuSim::FEntityState Entity;
    Entity.PositionMeters = FVector(1.0, 2.0, 3.0);
    Entity.Rotation = FQuat(1.0, 2.0, 3.0, 4.0);

    FTransform Converted;
    TestTrue(
        TEXT("A finite non-zero source pose converts"),
        LingTuSim::Visual::FCoordinateConverter::TryMakeWorldTransform(
            Entity,
            FVector(2.0, 3.0, 4.0),
            Converted));
    TestTrue(
        TEXT("Meters and handedness convert to Unreal centimeters"),
        Converted.GetLocation().Equals(FVector(100.0, -200.0, 300.0)));

    FQuat ExpectedRotation(-1.0, 2.0, -3.0, 4.0);
    ExpectedRotation.Normalize();
    TestTrue(
        TEXT("Quaternion axes convert once and normalize"),
        Converted.GetRotation().Equals(ExpectedRotation));
    TestTrue(
        TEXT("The converted quaternion is normalized"),
        Converted.GetRotation().IsNormalized());
    TestTrue(
        TEXT("Visual scale is preserved"),
        Converted.GetScale3D().Equals(FVector(2.0, 3.0, 4.0)));

    Entity.Rotation = FQuat(0.0, 0.0, 0.0, 0.0);
    TestFalse(
        TEXT("A zero quaternion cannot produce a frame transform"),
        LingTuSim::Visual::FCoordinateConverter::TryMakeWorldTransform(
            Entity,
            FVector::OneVector,
            Converted));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimVisualSnapshotGateTest,
    "LingTuSim.Visual.Runtime.SnapshotGate",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuSimVisualSnapshotGateTest::RunTest(const FString& Parameters)
{
    (void)Parameters;
    using LingTuSim::Visual::ESnapshotGateResult;

    LingTuSim::Visual::FSnapshotGate Gate;
    Gate.BindSession(TEXT("session-a"), 7);

    LingTuSim::FSnapshotEnvelope Snapshot = MakeSnapshot(TEXT("session-a"), 7, 4, 1);
    TestTrue(TEXT("Sequence 1 is accepted"), Gate.Evaluate(Snapshot) == ESnapshotGateResult::Accept);
    Gate.Commit(Snapshot);
    Snapshot.Sequence = 3;
    TestTrue(TEXT("Sequence 3 is accepted"), Gate.Evaluate(Snapshot) == ESnapshotGateResult::Accept);
    Gate.Commit(Snapshot);
    Snapshot.Sequence = 2;
    TestTrue(TEXT("Sequence 2 is rejected after 3"), Gate.Evaluate(Snapshot) == ESnapshotGateResult::OldSequence);

    Snapshot = MakeSnapshot(TEXT("session-a"), 7, 5, 0);
    TestTrue(TEXT("A reset increment accepts sequence 0"), Gate.Evaluate(Snapshot) == ESnapshotGateResult::Accept);
    Gate.Commit(Snapshot);
    Snapshot.ResetGeneration = 4;
    Snapshot.Sequence = 100;
    TestTrue(TEXT("An old reset is rejected"), Gate.Evaluate(Snapshot) == ESnapshotGateResult::OldReset);

    Snapshot = MakeSnapshot(TEXT("next-session"), 8, 0, 0);
    TestTrue(TEXT("A future model from the next session waits for rebind"), Gate.Evaluate(Snapshot) == ESnapshotGateResult::FutureModel);
    Gate.BindSession(TEXT("next-session"), 8);
    TestTrue(TEXT("Rebind clears ordering history"), Gate.Evaluate(Snapshot) == ESnapshotGateResult::Accept);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimVisualUdpFutureGenerationTest,
    "LingTuSim.Visual.Runtime.UdpFutureGenerationWaitsForRebind",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuSimVisualUdpFutureGenerationTest::RunTest(const FString& Parameters)
{
    (void)Parameters;

    const FString SessionA = TEXT("session-a");
    const FString SessionB = TEXT("session-b");
    constexpr uint64 ModelGeneration = 7;
    FScopedSessionServiceBinding SessionScope;
    FScopedVisualTestWorld TestWorld;
    if (!TestNotNull(TEXT("The UDP future-generation test world exists"), TestWorld.Get()))
    {
        return false;
    }
    ULingTuSimVisualWorldSubsystem* Subsystem = TestWorld.GetVisualSubsystem();
    if (!TestNotNull(TEXT("The UDP future-generation visual subsystem exists"), Subsystem))
    {
        return false;
    }
    if (!TestTrue(
        TEXT("The visual runtime binds the current model generation"),
        Subsystem->RebindSession(SessionA, ModelGeneration)))
    {
        return false;
    }

    LingTuSim::ESnapshotPublishResult PublishResult =
        LingTuSim::ESnapshotPublishResult::Stale;
    LingTuSim::FSnapshotEnvelope ParsedSnapshot;
    LingTuSim::FRuntimeLoadError LoadError;
    const bool bParsedAndPublished =
        LingTuSim::FSessionService::PublishSnapshotJson(
            MakeSnapshotJson(SessionB, ModelGeneration + 1, 0, 1),
            ParsedSnapshot,
            PublishResult,
            LoadError);
    if (!TestTrue(
        TEXT("UDP ingress parses a well-formed snapshot for the next session"),
        bParsedAndPublished))
    {
        return false;
    }
    if (!TestTrue(
        TEXT("The next session is not accepted into the current session mailbox"),
        PublishResult == LingTuSim::ESnapshotPublishResult::SessionMismatch))
    {
        return false;
    }

    Subsystem->Tick(0.0F);
    TestTrue(
        TEXT("The visual runtime exposes waiting-for-rebind evidence"),
        Subsystem->IsWaitingForRebind());

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimVisualAtomicRebindTest,
    "LingTuSim.Visual.Runtime.AtomicRebindAtoB",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuSimVisualAtomicRebindTest::RunTest(const FString& Parameters)
{
    (void)Parameters;

    const FString SessionA = TEXT("session-a");
    const FString SessionB = TEXT("session-b");
    constexpr uint64 ModelA = 7;
    constexpr uint64 ModelB = ModelA + 1;
    FScopedSessionServiceBinding SessionScope;
    FScopedVisualTestWorld TestWorld;
    if (!TestNotNull(TEXT("The atomic-rebind test world exists"), TestWorld.Get()))
    {
        return false;
    }
    ULingTuSimVisualWorldSubsystem* Subsystem = TestWorld.GetVisualSubsystem();
    if (!TestNotNull(TEXT("The atomic-rebind visual subsystem exists"), Subsystem))
    {
        return false;
    }
    if (!TestTrue(
        TEXT("Session A binds at model generation 7"),
        Subsystem->RebindSession(SessionA, ModelA)))
    {
        return false;
    }
    if (!TestTrue(
        TEXT("Session A can queue a current snapshot"),
        Subsystem->SubmitSnapshot(MakeSnapshot(SessionA, ModelA, 3, 9))
            == LingTuSim::ESnapshotPublishResult::Accepted))
    {
        return false;
    }

    LingTuSim::ESnapshotPublishResult FuturePublishResult =
        LingTuSim::ESnapshotPublishResult::Stale;
    LingTuSim::FSnapshotEnvelope FutureSnapshot;
    LingTuSim::FRuntimeLoadError LoadError;
    if (!TestTrue(
        TEXT("A well-formed session B generation +1 UDP payload reaches ingress classification"),
        LingTuSim::FSessionService::PublishSnapshotJson(
            MakeSnapshotJson(SessionB, ModelB, 0, 1),
            FutureSnapshot,
            FuturePublishResult,
            LoadError)))
    {
        return false;
    }
    if (!TestTrue(
        TEXT("Session B is rejected from session A's mailbox before rebind"),
        FuturePublishResult == LingTuSim::ESnapshotPublishResult::SessionMismatch))
    {
        return false;
    }
    Subsystem->Tick(0.0F);
    if (!TestTrue(
        TEXT("The future snapshot pauses visual consumption until rebind"),
        Subsystem->IsWaitingForRebind()))
    {
        return false;
    }

    if (!TestTrue(
        TEXT("One rebind transaction accepts session B and generation +1"),
        Subsystem->RebindSession(SessionB, ModelB)))
    {
        return false;
    }
    if (!TestFalse(
        TEXT("Rebind clears waiting-for-rebind evidence"),
        Subsystem->IsWaitingForRebind()))
    {
        return false;
    }

    LingTuSim::FSnapshotEnvelope Taken;
    if (!TestFalse(
        TEXT("Rebind clears every pending session A snapshot"),
        LingTuSim::FSessionService::TryTakeLatestSnapshot(Taken)))
    {
        return false;
    }
    if (!TestTrue(
        TEXT("An old session A snapshot is rejected after rebind"),
        Subsystem->SubmitSnapshot(MakeSnapshot(SessionA, ModelA, 4, 10))
            == LingTuSim::ESnapshotPublishResult::SessionMismatch))
    {
        return false;
    }
    if (!TestTrue(
        TEXT("An old generation is rejected even with the new session"),
        Subsystem->SubmitSnapshot(MakeSnapshot(SessionB, ModelA, 0, 1))
            == LingTuSim::ESnapshotPublishResult::ModelMismatch))
    {
        return false;
    }
    if (!TestTrue(
        TEXT("A session B generation +1 snapshot is accepted"),
        Subsystem->SubmitSnapshot(MakeSnapshot(SessionB, ModelB, 0, 1))
            == LingTuSim::ESnapshotPublishResult::Accepted))
    {
        return false;
    }
    if (!TestTrue(
        TEXT("The accepted session B snapshot is available to the visual consumer"),
        LingTuSim::FSessionService::TryTakeLatestSnapshot(Taken)))
    {
        return false;
    }
    TestEqual(TEXT("The new session is retained"), Taken.SessionId, SessionB);
    TestEqual(TEXT("The new generation is retained"), Taken.ModelGeneration, ModelB);

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimVisualDuplicateBindingTest,
    "LingTuSim.Visual.Runtime.DuplicateBinding",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuSimVisualDuplicateBindingTest::RunTest(const FString& Parameters)
{
    (void)Parameters;

    ULingTuSimVisualWorldSubsystem* Subsystem =
        NewObject<ULingTuSimVisualWorldSubsystem>();
    ULingTuSimBodyBindingComponent* First =
        NewObject<ULingTuSimBodyBindingComponent>();
    ULingTuSimBodyBindingComponent* Duplicate =
        NewObject<ULingTuSimBodyBindingComponent>();
    First->StableId = TEXT("robot/base_link");
    Duplicate->StableId = First->StableId;

    TestTrue(TEXT("The first stable ID registers"), Subsystem->RegisterBinding(First));
    TestFalse(TEXT("A second component cannot claim the same stable ID"), Subsystem->RegisterBinding(Duplicate));
    TestEqual(TEXT("Duplicate registration leaves one required binding"), Subsystem->GetRegisteredBindingCount(), 1);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimVisualReadinessEvidenceTransitionTest,
    "LingTuSim.Visual.Runtime.ReadinessEvidenceTransition",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuSimVisualReadinessEvidenceTransitionTest::RunTest(const FString& Parameters)
{
    (void)Parameters;

    const FString Session = TEXT("session-c");
    const FString StableId = TEXT("robot/base_link");
    FScopedSessionServiceBinding SessionScope;
    FRunEvidenceFixture EvidenceFixture(TEXT("visual-readiness-transition"));
    if (!TestTrue(
            TEXT("The readiness fixture is an absolute plain <run_id>/logs directory"),
            EvidenceFixture.IsValid()))
    {
        return false;
    }
    FScopedVisualTestWorld TestWorld;
    if (!TestNotNull(TEXT("The readiness-transition test world exists"), TestWorld.Get()))
    {
        return false;
    }
    ULingTuSimVisualWorldSubsystem* Subsystem = TestWorld.GetVisualSubsystem();
    if (!TestNotNull(TEXT("The readiness-transition visual subsystem exists"), Subsystem))
    {
        return false;
    }
    FString Error;
    if (!TestTrue(
        TEXT("Generation 0 readiness configuration is accepted"),
        Subsystem->ConfigureReadinessEvidence(
            EvidenceFixture.LogDirectory,
            EvidenceFixture.RunId,
            Session,
            0,
            0,
            Error)))
    {
        return false;
    }
    if (!TestTrue(
            TEXT("The visual runtime binds generation 0"),
            Subsystem->RebindSession(Session, 0)))
    {
        return false;
    }

    ULingTuSimBodyBindingComponent* Binding =
        NewObject<ULingTuSimBodyBindingComponent>();
    Binding->StableId = StableId;
    if (!TestTrue(TEXT("An actual visual binding registers"), Subsystem->RegisterBinding(Binding)))
    {
        return false;
    }

    TSharedPtr<FJsonObject> Evidence;
    if (!TestTrue(
            TEXT("PREPARED evidence is written after binding registration"),
            LoadEvidenceObject(EvidenceFixture.LogDirectory, Evidence)))
    {
        return false;
    }
    CheckVisualEvidence(*this, Evidence, TEXT("PREPARED"), Session, 0, 0);

    LingTuSim::FSnapshotEnvelope CurrentSnapshot =
        MakeBodySnapshot(Session, 0, 0, 1, StableId);
    if (CurrentSnapshot.Entities.Num() != 1)
    {
        TestEqual(
            TEXT("current truth fixture entity count is exact"),
            CurrentSnapshot.Entities.Num(),
            1);
        return false;
    }
    CurrentSnapshot.SimTimeNs = 125'000'000;
    CurrentSnapshot.Entities[0].Id.FrameId = TEXT("base_link");
    CurrentSnapshot.Entities[0].LinearVelocityMetersPerSecond = FVector(0.4, -0.2, 0.0);
    CurrentSnapshot.Entities[0].AngularVelocityRadiansPerSecond = FVector(0.0, 0.0, 0.3);
    if (!TestTrue(
        TEXT("Current truth snapshot queues"),
        Subsystem->SubmitSnapshot(CurrentSnapshot)
            == LingTuSim::ESnapshotPublishResult::Accepted))
    {
        return false;
    }
    Subsystem->Tick(0.0F);

    LingTuSim::FSnapshotEnvelope LatestApplied;
    if (!TestTrue(
        TEXT("The latest-applied accessor exposes only the committed visual frame"),
        Subsystem->GetLatestAppliedSnapshot(LatestApplied)))
    {
        return false;
    }
    TestEqual(TEXT("Latest-applied sequence is copied"), LatestApplied.Sequence,
              static_cast<uint64>(1));
    TestEqual(TEXT("Latest-applied sim time is copied"), LatestApplied.SimTimeNs,
              static_cast<int64>(125'000'000));
    if (LatestApplied.Entities.Num() != 1)
    {
        TestEqual(
            TEXT("latest-applied entity count is exact"),
            LatestApplied.Entities.Num(),
            1);
        return false;
    }
    TestEqual(
        TEXT("Latest-applied truth velocity is copied"),
        LatestApplied.Entities[0].LinearVelocityMetersPerSecond,
        FVector(0.4, -0.2, 0.0));

    Evidence.Reset();
    if (!TestTrue(
            TEXT("ACTIVE evidence is written after a real binding transform is applied"),
            LoadEvidenceObject(EvidenceFixture.LogDirectory, Evidence)))
    {
        return false;
    }
    CheckVisualEvidence(*this, Evidence, TEXT("ACTIVE"), Session, 0, 0);

    const FString EvidencePath =
        FPaths::Combine(EvidenceFixture.LogDirectory, TEXT("visual-readiness.json"));
    const FString Sentinel = TEXT("ACTIVE evidence must not be rewritten");
    if (!TestTrue(
        TEXT("The test can replace ACTIVE evidence with a write sentinel"),
        FFileHelper::SaveStringToFile(Sentinel, *EvidencePath)))
    {
        return false;
    }
    if (!TestTrue(
        TEXT("A later current truth snapshot queues"),
        Subsystem->SubmitSnapshot(MakeBodySnapshot(Session, 0, 0, 2, StableId))
            == LingTuSim::ESnapshotPublishResult::Accepted))
    {
        return false;
    }
    Subsystem->Tick(0.0F);
    if (!TestTrue(
            TEXT("A later committed frame remains readable"),
            Subsystem->GetLatestAppliedSnapshot(LatestApplied)))
    {
        return false;
    }
    TestEqual(TEXT("Latest-applied accessor advances with visual commit"),
              LatestApplied.Sequence, static_cast<uint64>(2));
    FString EvidenceAfterSecondSnapshot;
    if (!TestTrue(
        TEXT("Evidence remains readable after a later snapshot"),
        FFileHelper::LoadFileToString(EvidenceAfterSecondSnapshot, *EvidencePath)))
    {
        return false;
    }
    TestEqual(
        TEXT("A later snapshot does not rewrite successful ACTIVE evidence"),
        EvidenceAfterSecondSnapshot,
        Sentinel);
    if (!TestTrue(TEXT("Explicit rebind succeeds"), Subsystem->RebindSession(Session, 1)))
    {
        return false;
    }
    TestFalse(TEXT("Rebind clears the prior generation's latest-applied snapshot"),
              Subsystem->GetLatestAppliedSnapshot(LatestApplied));

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimVisualReadinessRejectsStaleAndFutureTest,
    "LingTuSim.Visual.Runtime.ReadinessRejectsStaleAndFutureSnapshots",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuSimVisualReadinessRejectsStaleAndFutureTest::RunTest(const FString& Parameters)
{
    (void)Parameters;

    const FString Session = TEXT("session-d");
    const FString StableId = TEXT("robot/base_link");
    FScopedSessionServiceBinding SessionScope;
    FRunEvidenceFixture EvidenceFixture(TEXT("visual-readiness-stale-future"));
    if (!TestTrue(
            TEXT("The stale/future fixture is an absolute plain <run_id>/logs directory"),
            EvidenceFixture.IsValid()))
    {
        return false;
    }
    FScopedVisualTestWorld TestWorld;
    if (!TestNotNull(TEXT("The stale/future test world exists"), TestWorld.Get()))
    {
        return false;
    }
    ULingTuSimVisualWorldSubsystem* Subsystem = TestWorld.GetVisualSubsystem();
    if (!TestNotNull(TEXT("The stale/future visual subsystem exists"), Subsystem))
    {
        return false;
    }
    FString Error;
    if (!TestTrue(
        TEXT("Readiness evidence configures for reset generation 1"),
        Subsystem->ConfigureReadinessEvidence(
            EvidenceFixture.LogDirectory,
            EvidenceFixture.RunId,
            Session,
            3,
            1,
            Error)))
    {
        return false;
    }
    if (!TestTrue(
            TEXT("The visual runtime binds model generation 3"),
            Subsystem->RebindSession(Session, 3)))
    {
        return false;
    }

    ULingTuSimBodyBindingComponent* Binding =
        NewObject<ULingTuSimBodyBindingComponent>();
    Binding->StableId = StableId;
    if (!TestTrue(TEXT("Binding registers"), Subsystem->RegisterBinding(Binding)))
    {
        return false;
    }

    if (!TestTrue(
        TEXT("A current reset snapshot queues"),
        Subsystem->SubmitSnapshot(MakeBodySnapshot(Session, 3, 1, 4, StableId))
            == LingTuSim::ESnapshotPublishResult::Accepted))
    {
        return false;
    }
    Subsystem->Tick(0.0F);
    TSharedPtr<FJsonObject> Evidence;
    if (!TestTrue(
            TEXT("Initial ACTIVE evidence is present"),
            LoadEvidenceObject(EvidenceFixture.LogDirectory, Evidence)))
    {
        return false;
    }
    CheckVisualEvidence(*this, Evidence, TEXT("ACTIVE"), Session, 3, 1);

    if (!TestTrue(
        TEXT("An old reset snapshot is rejected before visual activation"),
        Subsystem->SubmitSnapshot(MakeBodySnapshot(Session, 3, 0, 99, StableId))
            == LingTuSim::ESnapshotPublishResult::Stale))
    {
        return false;
    }
    Subsystem->Tick(0.0F);
    Evidence.Reset();
    if (!TestTrue(
        TEXT("Evidence remains on the applied reset"),
        LoadEvidenceObject(EvidenceFixture.LogDirectory, Evidence)))
    {
        return false;
    }
    CheckVisualEvidence(*this, Evidence, TEXT("ACTIVE"), Session, 3, 1);

    if (!TestTrue(
        TEXT("A future model snapshot is rejected by the current binding"),
        Subsystem->SubmitSnapshot(MakeBodySnapshot(Session, 4, 1, 100, StableId))
            == LingTuSim::ESnapshotPublishResult::ModelMismatch))
    {
        return false;
    }
    Subsystem->Tick(0.0F);
    Evidence.Reset();
    if (!TestTrue(
        TEXT("Future model evidence does not become active"),
        LoadEvidenceObject(EvidenceFixture.LogDirectory, Evidence)))
    {
        return false;
    }
    CheckVisualEvidence(*this, Evidence, TEXT("ACTIVE"), Session, 3, 1);

    if (!TestTrue(
            TEXT("Rebind to the future model is explicit"),
            Subsystem->RebindSession(Session, 4)))
    {
        return false;
    }
    Evidence.Reset();
    if (!TestTrue(
            TEXT("Rebind rewrites PREPARED evidence for the new generation"),
            LoadEvidenceObject(EvidenceFixture.LogDirectory, Evidence)))
    {
        return false;
    }
    CheckVisualEvidence(*this, Evidence, TEXT("PREPARED"), Session, 4, 1);

    if (!TestTrue(
        TEXT("A post-reset current snapshot queues"),
        Subsystem->SubmitSnapshot(MakeBodySnapshot(Session, 4, 2, 0, StableId))
            == LingTuSim::ESnapshotPublishResult::Accepted))
    {
        return false;
    }
    Subsystem->Tick(0.0F);
    Evidence.Reset();
    if (!TestTrue(
        TEXT("Reset generation is updated only after apply"),
        LoadEvidenceObject(EvidenceFixture.LogDirectory, Evidence)))
    {
        return false;
    }
    CheckVisualEvidence(*this, Evidence, TEXT("ACTIVE"), Session, 4, 2);

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimVisualRenderResourcesGateFirstFrameTest,
    "LingTuSim.Visual.Runtime.RenderResourcesGateFirstFrame",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuSimVisualRenderResourcesGateFirstFrameTest::RunTest(const FString& Parameters)
{
    (void)Parameters;

    const FString Session = TEXT("session-e");
    const FString Directory = MakeEvidenceDirectory(TEXT("render-resources-gate"));
    FRunEvidenceFixture EvidenceFixture(TEXT("visual-render-resource-gate"));
    if (!TestTrue(
            TEXT("The render-resource fixture is an absolute plain <run_id>/logs directory"),
            EvidenceFixture.IsValid()))
    {
        return false;
    }
    const FString ProjectionRelativePath =
        TEXT("Saved/Automation/LingTuSimVisual/render-resources-gate/robot.visual-projection.json");
    FString Error;
    LingTuSim::FSessionService::UnbindSession();

    TestTrue(
        TEXT("render resource fixture writes"),
        WriteVisualBundle(
            Directory,
            ProjectionRelativePath,
            ProjectionJson(
                TEXT("RobotVisual:OmniCart"),
                TEXT("robot.package.yaml"),
                TEXT("primitive"),
                TEXT("box"),
                TEXT("base_visual_frame"),
                TEXT("/Engine/BasicShapes/Cube.Cube"),
                TEXT("[1.0,1.0,1.0]"),
                TEXT("[1.0,1.0,1.0]")),
            Error,
            Session));

    UWorld* World = CreateVisualTestWorld();
    TestNotNull(TEXT("render resource world exists"), World);
    ULingTuSimVisualWorldSubsystem* Subsystem =
        World != nullptr ? World->GetSubsystem<ULingTuSimVisualWorldSubsystem>() : nullptr;
    TestNotNull(TEXT("visual subsystem exists"), Subsystem);
    FString StartError;
    TestTrue(
        *FString::Printf(TEXT("visual plan starts; StartError='%s'"), *StartError),
        Subsystem != nullptr
            && Subsystem->StartVisualPlan(Directory, FPaths::ProjectDir(), 2, 0, StartError));
    TestTrue(
        TEXT("readiness evidence configures after plan start"),
        Subsystem != nullptr
            && Subsystem->ConfigureReadinessEvidence(
                EvidenceFixture.LogDirectory,
                EvidenceFixture.RunId,
                Session,
                2,
                0,
                Error));

    const TMap<FString, ALingTuSimBodyActor*> ActorsByStableId =
        CollectBodyActorsByStableId(World);
    ALingTuSimBodyActor* BodyActor =
        ActorsByStableId.FindRef(TEXT("cart_01/base"));
    TestNotNull(TEXT("body actor exists"), BodyActor);
    TArray<UStaticMeshComponent*> Components;
    if (BodyActor != nullptr)
    {
        BodyActor->GetComponents<UStaticMeshComponent>(Components);
    }
    TestEqual(TEXT("one static mesh component exists"), Components.Num(), 1);

    UStaticMeshComponent* VisualComponent = Components.Num() == 1 ? Components[0] : nullptr;
    if (VisualComponent != nullptr)
    {
        UStaticMesh* TransientMesh = NewObject<UStaticMesh>(VisualComponent);
        VisualComponent->SetStaticMesh(TransientMesh);
    }
    if (BodyActor != nullptr)
    {
        FString NotReadyReason;
        TestFalse(
            TEXT("transient static mesh without render data is not ready"),
            ULingTuSimVisualWorldSubsystem::AreStaticMeshRenderResourcesReady(
                BodyActor,
                NotReadyReason));
    }

    TSharedPtr<FJsonObject> Evidence;
    TestTrue(
        TEXT("PREPARED evidence exists before first truth frame"),
        LoadEvidenceObject(EvidenceFixture.LogDirectory, Evidence));
    CheckVisualEvidence(*this, Evidence, TEXT("PREPARED"), Session, 2, 0);
    TestTrue(
        TEXT("first truth frame queues"),
        Subsystem != nullptr
            && Subsystem->SubmitSnapshot(MakeBodySnapshot(Session, 2, 0, 1, TEXT("cart_01/base")))
                == LingTuSim::ESnapshotPublishResult::Accepted);
    if (Subsystem != nullptr)
    {
        Subsystem->Tick(0.0F);
    }
    Evidence.Reset();
    TestTrue(
        TEXT("deferred frame leaves evidence readable"),
        LoadEvidenceObject(EvidenceFixture.LogDirectory, Evidence));
    CheckVisualEvidence(*this, Evidence, TEXT("PREPARED"), Session, 2, 0);
    if (BodyActor != nullptr)
    {
        TestTrue(TEXT("actor remains hidden while render resources are deferred"), BodyActor->IsHidden());
    }
    TestTrue(
        TEXT("same deferred first frame can be resent while resources are still pending"),
        Subsystem != nullptr
            && Subsystem->SubmitSnapshot(MakeBodySnapshot(Session, 2, 0, 1, TEXT("cart_01/base")))
                == LingTuSim::ESnapshotPublishResult::Accepted);

    if (VisualComponent != nullptr)
    {
        VisualComponent->DestroyComponent();
    }
    if (Subsystem != nullptr)
    {
        Subsystem->Tick(0.0F);
    }
    Evidence.Reset();
    TestTrue(
        TEXT("deferred first frame applies once render resources are ready"),
        LoadEvidenceObject(EvidenceFixture.LogDirectory, Evidence));
    CheckVisualEvidence(*this, Evidence, TEXT("ACTIVE"), Session, 2, 0);
    if (BodyActor != nullptr)
    {
        TestFalse(TEXT("actor is revealed after render resources are ready"), BodyActor->IsHidden());
    }

    if (World != nullptr)
    {
        World->DestroyWorld(false);
    }
    IFileManager::Get().DeleteDirectory(*Directory, false, true);
    LingTuSim::FSessionService::UnbindSession();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimVisualProjectionPayloadV2StrictParsingTest,
    "LingTuSim.Visual.Runtime.Projection.PayloadV2StrictParsing",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuSimVisualProjectionPayloadV2StrictParsingTest::RunTest(const FString& Parameters)
{
    (void)Parameters;

    const FString Session = TEXT("payload-v2-session");
    const FString Directory = MakeEvidenceDirectory(TEXT("projection-payload-v2"));
    FString RelativeDirectory = Directory;
    TestTrue(
        TEXT("payload v2 test directory is repository-relative"),
        FPaths::MakePathRelativeTo(RelativeDirectory, *FPaths::ProjectDir()));
    FPaths::NormalizeFilename(RelativeDirectory);
    const FString RobotProjectionRelativePath =
        FPaths::Combine(RelativeDirectory, TEXT("robot.visual-projection.json"));
    const FString PayloadProjectionRelativePath =
        FPaths::Combine(RelativeDirectory, TEXT("payload.visual-projection.json"));
    FString Error;
    TestTrue(
        TEXT("payload v2 base bundle writes"),
        WriteVisualBundle(
            Directory,
            RobotProjectionRelativePath,
            ProjectionJson(
                TEXT("RobotVisual:OmniCart"),
                TEXT("robot.package.yaml"),
                TEXT("primitive"),
                TEXT("box"),
                TEXT("base_visual_frame"),
                TEXT("/Engine/BasicShapes/Cube.Cube"),
                TEXT("[1.0,1.0,1.0]"),
                TEXT("[1.0,1.0,1.0]")),
            Error,
            Session));
    TestTrue(
        TEXT("valid payload projection writes"),
        FFileHelper::SaveStringToFile(
            PayloadProjectionJson(false),
            *FPaths::Combine(FPaths::ProjectDir(), PayloadProjectionRelativePath),
            FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM));
    TestTrue(
        TEXT("payload-aware visual v2 plan writes"),
        RewriteVisualPlanWithPayloadV2(
            Directory,
            Session,
            RobotProjectionRelativePath,
            PayloadProjectionRelativePath,
            false));

    UWorld* World = CreateVisualTestWorld();
    LingTuSim::Visual::FVisualMaterializationResult Result;
    LingTuSim::Visual::FVisualMaterializationError LoadError;
    TestTrue(
        TEXT("valid payload-aware visual v2 bundle materializes"),
        World != nullptr
            && LingTuSim::Visual::FRobotVisualProjectionMaterializer::MaterializeBundle(
                *World,
                Directory,
                FPaths::ProjectDir(),
                1,
                0,
                Result,
                LoadError));
    const TMap<FString, ALingTuSimBodyActor*> Actors = CollectBodyActorsByStableId(World);
    ALingTuSimBodyActor* PayloadActor = Actors.FindRef(TEXT("payload_01/payload_base"));
    TestNotNull(TEXT("payload stable body actor is materialized"), PayloadActor);
    TArray<UStaticMeshComponent*> PayloadComponents;
    if (PayloadActor != nullptr)
    {
        PayloadActor->GetComponents<UStaticMeshComponent>(PayloadComponents);
    }
    TestEqual(TEXT("payload actor owns its projected mesh component"), PayloadComponents.Num(), 1);
    for (ALingTuSimBodyActor* Actor : Result.Actors)
    {
        if (IsValid(Actor))
        {
            Actor->Destroy();
        }
    }

    TestTrue(
        TEXT("payload v2 plan with colon-bearing projection path writes"),
        RewriteVisualPlanWithPayloadV2(
            Directory,
            Session,
            RobotProjectionRelativePath,
            PayloadProjectionRelativePath + TEXT(":alternate"),
            false));
    Result = LingTuSim::Visual::FVisualMaterializationResult();
    LoadError.Reset();
    TestFalse(
        TEXT("colon-bearing repository-relative paths are rejected"),
        World != nullptr
            && LingTuSim::Visual::FRobotVisualProjectionMaterializer::MaterializeBundle(
                *World,
                Directory,
                FPaths::ProjectDir(),
                1,
                0,
                Result,
                LoadError));

    TestTrue(
        TEXT("payload projection with explicit unknown field writes"),
        FFileHelper::SaveStringToFile(
            PayloadProjectionJson(true),
            *FPaths::Combine(FPaths::ProjectDir(), PayloadProjectionRelativePath),
            FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM));
    TestTrue(
        TEXT("payload-aware visual v2 plan is restored"),
        RewriteVisualPlanWithPayloadV2(
            Directory,
            Session,
            RobotProjectionRelativePath,
            PayloadProjectionRelativePath,
            false));
    Result = LingTuSim::Visual::FVisualMaterializationResult();
    LoadError.Reset();
    TestFalse(
        TEXT("unknown payload projection fields fail closed after v2 plan parsing"),
        World != nullptr
            && LingTuSim::Visual::FRobotVisualProjectionMaterializer::MaterializeBundle(
                *World,
                Directory,
                FPaths::ProjectDir(),
                1,
                0,
                Result,
                LoadError));
    TestTrue(
        TEXT("failure identifies the payload projection rather than rejecting visual v2"),
        LoadError.Source.EndsWith(TEXT("payload.visual-projection.json")));
    TestTrue(
        TEXT("payload projection rejects unknown fields explicitly"),
        LoadError.Message.Contains(TEXT("exact expected field set")));

    TestTrue(
        TEXT("visual v2 plan with unknown payload field writes"),
        RewriteVisualPlanWithPayloadV2(
            Directory,
            Session,
            RobotProjectionRelativePath,
            PayloadProjectionRelativePath,
            true));
    LoadError.Reset();
    TestFalse(
        TEXT("unknown payload plan fields are not ignored"),
        World != nullptr
            && LingTuSim::Visual::FRobotVisualProjectionMaterializer::MaterializeBundle(
                *World,
                Directory,
                FPaths::ProjectDir(),
                1,
                0,
                Result,
                LoadError));
    TestTrue(
        TEXT("unknown payload plan field is rejected at the visual plan"),
        LoadError.Source.Contains(TEXT("visual.plan.json.robots[0].payloads[0]")));
    TestTrue(
        TEXT("payload plan rejection is strict"),
        LoadError.Message.Contains(TEXT("exact expected field set")));

    if (World != nullptr)
    {
        World->DestroyWorld(false);
    }
    IFileManager::Get().DeleteDirectory(*Directory, false, true);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimVisualProjectionPrimitiveMaterializationTest,
    "LingTuSim.Visual.Runtime.Projection.PrimitiveMaterialization",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuSimVisualProjectionPrimitiveMaterializationTest::RunTest(const FString& Parameters)
{
    (void)Parameters;

    const FString Directory = MakeEvidenceDirectory(TEXT("projection-primitive"));
    const FString ProjectionRelativePath =
        TEXT("Saved/Automation/LingTuSimVisual/projection-primitive/robot.visual-projection.json");
    FString Error;
    TestTrue(
        TEXT("primitive bundle fixture writes"),
        WriteVisualBundle(
            Directory,
            ProjectionRelativePath,
            ProjectionJson(
                TEXT("RobotVisual:OmniCart"),
                TEXT("robot.package.yaml"),
                TEXT("primitive"),
                TEXT("box"),
                TEXT("base_visual_frame"),
                TEXT("/Engine/BasicShapes/Cube.Cube"),
                TEXT("[1.0,1.0,1.0]"),
                TEXT("[1.0,1.0,1.0]")),
            Error));

    UWorld* World = CreateVisualTestWorld();
    TestNotNull(TEXT("test world exists"), World);
    LingTuSim::Visual::FVisualMaterializationResult Result;
    LingTuSim::Visual::FVisualMaterializationError LoadError;
    TestTrue(
        TEXT("primitive visual plan materializes"),
        World != nullptr
            && LingTuSim::Visual::FRobotVisualProjectionMaterializer::MaterializeBundle(
                *World,
                Directory,
                FPaths::ProjectDir(),
                3,
                4,
                Result,
                LoadError));
    TestEqual(TEXT("one body actor spawned"), Result.ExpectedBodyCount, 1);
    if (Result.Actors.Num() == 1 && IsValid(Result.Actors[0]))
    {
        TestEqual(
            TEXT("body stable ID is instance-prefixed"),
            Result.Actors[0]->BodyBinding->StableId,
            FString(TEXT("cart_01/base")));
        TArray<UStaticMeshComponent*> Components;
        Result.Actors[0]->GetComponents<UStaticMeshComponent>(Components);
        TestEqual(TEXT("one primitive mesh visual attached"), Components.Num(), 1);
        if (Components.Num() == 1)
        {
            TestTrue(TEXT("visual-only mesh has no collision"), Components[0]->GetCollisionEnabled() == ECollisionEnabled::NoCollision);
            TestTrue(
                TEXT("robot mesh satisfies the complete presentation-only policy"),
                LingTuSim::Visual::HasPresentationPolicy(*Components[0]));
            TestFalse(TEXT("visual actor cannot become a Chaos collision owner"), Result.Actors[0]->GetActorEnableCollision());
            TestTrue(TEXT("box scale uses UE basic shape meter semantics"), Components[0]->GetRelativeScale3D().Equals(FVector(1.0, 2.0, 0.5)));
            TestTrue(TEXT("dynamic visual is owned as an instance component"), Result.Actors[0]->GetInstanceComponents().Contains(Components[0]));
        }
        TestTrue(TEXT("destroying actor removes dynamic components"), Result.Actors[0]->Destroy());
    }
    if (World != nullptr)
    {
        World->DestroyWorld(false);
    }
    IFileManager::Get().DeleteDirectory(*Directory, false, true);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimVisualProjectionComponentMaterialTest,
    "LingTuSim.Visual.Runtime.Projection.ComponentMaterial",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuSimVisualProjectionComponentMaterialTest::RunTest(const FString& Parameters)
{
    (void)Parameters;

    UMaterialInterface* BaseMaterial = LoadObject<UMaterialInterface>(
        nullptr,
        TEXT("/Game/RobotSim/Materials/M_LingTuVisualSurface.M_LingTuVisualSurface"));
    TestNotNull(TEXT("generic visual surface base material is available for cook"), BaseMaterial);

    const FString MaterialJson =
        TEXT("{\"source\":\"mjcf_material_rgba\",\"key\":\"paint\",")
        TEXT("\"pbr\":{\"base_color_rgba\":[0.25,0.5,0.75,1.0],\"metallic\":0.2,\"roughness\":0.8}}");
    const FString Directory = MakeEvidenceDirectory(TEXT("projection-component-material"));
    const FString ProjectionRelativePath =
        TEXT("Saved/Automation/LingTuSimVisual/projection-component-material/robot.visual-projection.json");
    FString Error;
    TestTrue(
        TEXT("component material bundle fixture writes"),
        WriteVisualBundle(
            Directory,
            ProjectionRelativePath,
            ProjectionJsonWithMaterial(MaterialJson),
            Error));

    UWorld* World = CreateVisualTestWorld();
    TestNotNull(TEXT("material test world exists"), World);
    LingTuSim::Visual::FVisualMaterializationResult Result;
    LingTuSim::Visual::FVisualMaterializationError LoadError;
    TestTrue(
        *FString::Printf(TEXT("component material projection materializes; error='%s:%s'"), *LoadError.Source, *LoadError.Message),
        World != nullptr
            && LingTuSim::Visual::FRobotVisualProjectionMaterializer::MaterializeBundle(
                *World,
                Directory,
                FPaths::ProjectDir(),
                3,
                4,
                Result,
                LoadError));
    if (Result.Actors.Num() == 1 && IsValid(Result.Actors[0]))
    {
        TArray<UStaticMeshComponent*> Components;
        Result.Actors[0]->GetComponents<UStaticMeshComponent>(Components);
        TestEqual(TEXT("one materialized mesh component exists"), Components.Num(), 1);
        if (Components.Num() == 1)
        {
            UMaterialInstanceDynamic* DynamicMaterial =
                Cast<UMaterialInstanceDynamic>(Components[0]->GetMaterial(0));
            TestNotNull(TEXT("mesh component gets a dynamic material instance"), DynamicMaterial);
            if (DynamicMaterial != nullptr)
            {
                TestTrue(
                    TEXT("dynamic material parent is generic base material"),
                    DynamicMaterial->Parent.Get() == BaseMaterial);
            }
        }
        Result.Actors[0]->Destroy();
    }
    if (World != nullptr)
    {
        World->DestroyWorld(false);
    }
    IFileManager::Get().DeleteDirectory(*Directory, false, true);

    const FString MissingMaterialDirectory = MakeEvidenceDirectory(TEXT("projection-material-missing"));
    const FString MissingMaterialProjectionRelativePath =
        TEXT("Saved/Automation/LingTuSimVisual/projection-material-missing/robot.visual-projection.json");
    FString MissingMaterialJson = ProjectionJsonWithMaterial(MaterialJson);
    const int32 RemovedMaterialFields = MissingMaterialJson.ReplaceInline(
        *FString::Printf(TEXT("\"material\":%s,"), *MaterialJson),
        TEXT(""));
    TestEqual(TEXT("missing material fixture removes one canonical material field"), RemovedMaterialFields, 1);
    TestTrue(
        TEXT("missing material fixture writes"),
        WriteVisualBundle(
            MissingMaterialDirectory,
            MissingMaterialProjectionRelativePath,
            MissingMaterialJson,
            Error));
    World = CreateVisualTestWorld();
    LingTuSim::Visual::FVisualMaterializationResult MissingMaterialResult;
    LingTuSim::Visual::FVisualMaterializationError MissingMaterialError;
    TestFalse(
        TEXT("v1 projection rejects a component without resolved material"),
        World != nullptr
            && LingTuSim::Visual::FRobotVisualProjectionMaterializer::MaterializeBundle(
                *World,
                MissingMaterialDirectory,
                FPaths::ProjectDir(),
                3,
                4,
                MissingMaterialResult,
                MissingMaterialError));
    if (World != nullptr)
    {
        World->DestroyWorld(false);
    }
    IFileManager::Get().DeleteDirectory(*MissingMaterialDirectory, false, true);

    const FString InvalidEnumDirectory = MakeEvidenceDirectory(TEXT("projection-material-invalid-source"));
    const FString InvalidEnumProjectionRelativePath =
        TEXT("Saved/Automation/LingTuSimVisual/projection-material-invalid-source/robot.visual-projection.json");
    TestTrue(
        TEXT("invalid source material fixture writes"),
        WriteVisualBundle(
            InvalidEnumDirectory,
            InvalidEnumProjectionRelativePath,
            ProjectionJsonWithMaterial(
                TEXT("{\"source\":\"unknown\",\"key\":null,\"pbr\":{\"base_color_rgba\":[1.0,1.0,1.0,1.0],\"metallic\":0.0,\"roughness\":0.5}}")),
            Error));
    World = CreateVisualTestWorld();
    TestFalse(
        TEXT("invalid canonical material.source fails closed"),
        World != nullptr
            && LingTuSim::Visual::FRobotVisualProjectionMaterializer::MaterializeBundle(
                *World,
                InvalidEnumDirectory,
                FPaths::ProjectDir(),
                3,
                4,
                Result,
                LoadError));
    if (World != nullptr)
    {
        World->DestroyWorld(false);
    }
    IFileManager::Get().DeleteDirectory(*InvalidEnumDirectory, false, true);

    const FString UnknownFieldDirectory = MakeEvidenceDirectory(TEXT("projection-material-unknown-field"));
    const FString UnknownFieldProjectionRelativePath =
        TEXT("Saved/Automation/LingTuSimVisual/projection-material-unknown-field/robot.visual-projection.json");
    TestTrue(
        TEXT("unknown field material fixture writes"),
        WriteVisualBundle(
            UnknownFieldDirectory,
            UnknownFieldProjectionRelativePath,
            ProjectionJsonWithMaterial(
                TEXT("{\"source\":\"compiler_default\",\"key\":null,\"pbr\":{\"base_color_rgba\":[1.0,1.0,1.0,1.0],\"metallic\":0.0,\"roughness\":0.5},\"extra\":true}")),
            Error));
    World = CreateVisualTestWorld();
    TestFalse(
        TEXT("unknown canonical material field fails closed"),
        World != nullptr
            && LingTuSim::Visual::FRobotVisualProjectionMaterializer::MaterializeBundle(
                *World,
                UnknownFieldDirectory,
                FPaths::ProjectDir(),
                3,
                4,
                Result,
                LoadError));
    if (World != nullptr)
    {
        World->DestroyWorld(false);
    }
    IFileManager::Get().DeleteDirectory(*UnknownFieldDirectory, false, true);

    const FString LegacyFlatDirectory = MakeEvidenceDirectory(TEXT("projection-material-legacy-flat"));
    const FString LegacyFlatProjectionRelativePath =
        TEXT("Saved/Automation/LingTuSimVisual/projection-material-legacy-flat/robot.visual-projection.json");
    TestTrue(
        TEXT("legacy flat material fixture writes"),
        WriteVisualBundle(
            LegacyFlatDirectory,
            LegacyFlatProjectionRelativePath,
            ProjectionJsonWithMaterial(
                TEXT("{\"BaseColor\":[1.0,1.0,1.0,1.0],\"Metallic\":0.0,\"Roughness\":0.5}")),
            Error));
    World = CreateVisualTestWorld();
    TestFalse(
        TEXT("v1 projection rejects legacy flat material"),
        World != nullptr
            && LingTuSim::Visual::FRobotVisualProjectionMaterializer::MaterializeBundle(
                *World,
                LegacyFlatDirectory,
                FPaths::ProjectDir(),
                3,
                4,
                Result,
                LoadError));
    if (World != nullptr)
    {
        World->DestroyWorld(false);
    }
    IFileManager::Get().DeleteDirectory(*LegacyFlatDirectory, false, true);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimVisualProjectionCapsuleMaterialTest,
    "LingTuSim.Visual.Runtime.Projection.CapsuleMaterial",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuSimVisualProjectionCapsuleMaterialTest::RunTest(const FString& Parameters)
{
    (void)Parameters;

    const FString Directory = MakeEvidenceDirectory(TEXT("projection-capsule-material"));
    const FString ProjectionRelativePath =
        TEXT("Saved/Automation/LingTuSimVisual/projection-capsule-material/robot.visual-projection.json");
    FString Error;
    TestTrue(
        TEXT("capsule material bundle fixture writes"),
        WriteVisualBundle(
            Directory,
            ProjectionRelativePath,
            CapsuleProjectionJsonWithMaterial(
                TEXT("{\"source\":\"mjcf_geom_rgba\",\"key\":null,\"pbr\":{\"base_color_rgba\":[0.7,0.2,0.1,1.0],\"metallic\":0.0,\"roughness\":0.6}}")),
            Error));

    UWorld* World = CreateVisualTestWorld();
    TestNotNull(TEXT("capsule material test world exists"), World);
    LingTuSim::Visual::FVisualMaterializationResult Result;
    LingTuSim::Visual::FVisualMaterializationError LoadError;
    TestTrue(
        *FString::Printf(TEXT("capsule material projection materializes; error='%s:%s'"), *LoadError.Source, *LoadError.Message),
        World != nullptr
            && LingTuSim::Visual::FRobotVisualProjectionMaterializer::MaterializeBundle(
                *World,
                Directory,
                FPaths::ProjectDir(),
                3,
                4,
                Result,
                LoadError));
    if (Result.Actors.Num() == 1 && IsValid(Result.Actors[0]))
    {
        TArray<UCapsuleComponent*> Components;
        Result.Actors[0]->GetComponents<UCapsuleComponent>(Components);
        TestEqual(TEXT("one capsule visual component exists"), Components.Num(), 1);
        if (Components.Num() == 1)
        {
            TArray<UStaticMeshComponent*> RenderComponents;
            Result.Actors[0]->GetComponents<UStaticMeshComponent>(RenderComponents);
            TestEqual(
                TEXT("capsule has cylinder and two spherical render proxies"),
                RenderComponents.Num(),
                3);
            for (UStaticMeshComponent* RenderComponent : RenderComponents)
            {
                TestNotNull(
                    TEXT("capsule render proxy gets a dynamic material instance"),
                    RenderComponent != nullptr
                        ? Cast<UMaterialInstanceDynamic>(RenderComponent->GetMaterial(0))
                        : nullptr);
            }
        }
        Result.Actors[0]->Destroy();
    }
    if (World != nullptr)
    {
        World->DestroyWorld(false);
    }
    IFileManager::Get().DeleteDirectory(*Directory, false, true);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimVisualProjectionMultiBodyMaterializationTest,
    "LingTuSim.Visual.Runtime.Projection.MultiBodyMaterialization",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuSimVisualProjectionMultiBodyMaterializationTest::RunTest(const FString& Parameters)
{
    (void)Parameters;

    const FString Directory = MakeEvidenceDirectory(TEXT("projection-multi-body"));
    const FString ProjectionRelativePath =
        TEXT("Saved/Automation/LingTuSimVisual/projection-multi-body/robot.visual-projection.json");
    const TArray<FString> LocalBodies = {TEXT("base"), TEXT("front_axle"), TEXT("rear_axle")};
    FString Error;
    TestTrue(
        TEXT("multi-body bundle fixture writes"),
        WriteVisualBundle(
            Directory,
            ProjectionRelativePath,
            MultiBodyProjectionJson(LocalBodies),
            Error));

    UWorld* World = CreateVisualTestWorld();
    TestNotNull(TEXT("multi-body world exists"), World);
    LingTuSim::Visual::FVisualMaterializationResult Result;
    LingTuSim::Visual::FVisualMaterializationError LoadError;
    TestTrue(
        TEXT("multi-body projection materializes"),
        World != nullptr
            && LingTuSim::Visual::FRobotVisualProjectionMaterializer::MaterializeBundle(
                *World,
                Directory,
                FPaths::ProjectDir(),
                6,
                0,
                Result,
                LoadError));
    TestEqual(TEXT("ExpectedBodyCount matches local body count"), Result.ExpectedBodyCount, LocalBodies.Num());

    TSet<FString> ExpectedStableIds;
    for (const FString& LocalBody : LocalBodies)
    {
        ExpectedStableIds.Add(TEXT("cart_01/") + LocalBody);
    }
    TSet<FString> ObservedStableIds;
    for (ALingTuSimBodyActor* Actor : Result.Actors)
    {
        if (!IsValid(Actor) || Actor->BodyBinding == nullptr)
        {
            continue;
        }
        ObservedStableIds.Add(Actor->BodyBinding->StableId);
        TArray<UStaticMeshComponent*> Components;
        Actor->GetComponents<UStaticMeshComponent>(Components);
        TestEqual(
            *FString::Printf(TEXT("%s has one visual component"), *Actor->BodyBinding->StableId),
            Components.Num(),
            1);
    }
    TestEqual(TEXT("all expected stable body IDs materialize"), ObservedStableIds.Num(), ExpectedStableIds.Num());
    for (const FString& StableId : ExpectedStableIds)
    {
        TestTrue(
            *FString::Printf(TEXT("stable ID exists: %s"), *StableId),
            ObservedStableIds.Contains(StableId));
    }

    if (World != nullptr)
    {
        World->DestroyWorld(false);
    }
    IFileManager::Get().DeleteDirectory(*Directory, false, true);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimVisualProjectionScenarioEntityMaterializationTest,
    "LingTuSim.Visual.Runtime.Projection.ScenarioEntityMaterialization",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuSimVisualProjectionScenarioEntityMaterializationTest::RunTest(const FString& Parameters)
{
    (void)Parameters;

    const FString Session = TEXT("session-d");
    const FString Directory = MakeEvidenceDirectory(TEXT("projection-scenario-entity"));
    const FString RobotProjectionRelativePath =
        TEXT("Saved/Automation/LingTuSimVisual/projection-scenario-entity/robot.visual-projection.json");
    const FString EntityProjectionRelativePath =
        TEXT("Saved/Automation/LingTuSimVisual/projection-scenario-entity/entity.visual-projection.json");
    FString Error;
    TestTrue(
        TEXT("robot fixture writes before scenario entity extension"),
        WriteVisualBundle(
            Directory,
            RobotProjectionRelativePath,
            ProjectionJson(
                TEXT("RobotVisual:OmniCart"),
                TEXT("robot.package.yaml"),
                TEXT("primitive"),
                TEXT("box"),
                TEXT("base_visual_frame"),
                TEXT("/Engine/BasicShapes/Cube.Cube"),
                TEXT("[1.0,1.0,1.0]"),
                TEXT("[1.0,1.0,1.0]")),
            Error,
            Session));

    const FString EntityProjectionPath =
        FPaths::Combine(FPaths::ProjectDir(), EntityProjectionRelativePath);
    TestTrue(
        TEXT("scenario entity projection writes"),
        FFileHelper::SaveStringToFile(
            ScenarioEntityProjectionJson(),
            *EntityProjectionPath,
            FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM));
    const FString RobotEntry = VisualPlanRobotEntryJson(
        TEXT("cart_01"),
        RobotProjectionRelativePath);
    TestTrue(
        TEXT("visual plan declares a generic scenario entity"),
        RewriteVisualPlanWithScenarioEntity(
            Directory,
            Session,
            RobotEntry,
            EntityProjectionRelativePath));

    UWorld* World = CreateVisualTestWorld();
    TestNotNull(TEXT("scenario entity visual world exists"), World);
    LingTuSim::Visual::FVisualMaterializationResult Result;
    LingTuSim::Visual::FVisualMaterializationError LoadError;
    TestTrue(
        *FString::Printf(
            TEXT("robot and scenario entity materialize from one VisualPlan; error='%s:%s'"),
            *LoadError.Source,
            *LoadError.Message),
        World != nullptr
            && LingTuSim::Visual::FRobotVisualProjectionMaterializer::MaterializeBundle(
                *World,
                Directory,
                FPaths::ProjectDir(),
                7,
                0,
                Result,
                LoadError));
    TestEqual(TEXT("robot and scenario entity create two body actors"), Result.ExpectedBodyCount, 2);
    const TMap<FString, ALingTuSimBodyActor*> Actors = CollectBodyActorsByStableId(World);
    TestTrue(TEXT("robot body is present"), Actors.Contains(TEXT("cart_01/base")));
    TestTrue(
        TEXT("scenario body uses the same stable ID as MuJoCo truth"),
        Actors.Contains(TEXT("pedestrian_01/proxy_root")));
    if (ALingTuSimBodyActor* const* Pedestrian = Actors.Find(TEXT("pedestrian_01/proxy_root")))
    {
        TArray<UCapsuleComponent*> Capsules;
        (*Pedestrian)->GetComponents<UCapsuleComponent>(Capsules);
        TestEqual(TEXT("scenario projection owns one capsule component"), Capsules.Num(), 1);
    }

    if (World != nullptr)
    {
        World->DestroyWorld(false);
    }
    IFileManager::Get().DeleteDirectory(*Directory, false, true);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimVisualProjectionMultiRobotAndAtomicFailureTest,
    "LingTuSim.Visual.Runtime.Projection.MultiRobotAndAtomicFailure",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuSimVisualProjectionMultiRobotAndAtomicFailureTest::RunTest(const FString& Parameters)
{
    (void)Parameters;

    const FString SessionA = TEXT("session-a");
    const FString SessionB = TEXT("session-b");
    const FString SessionC = TEXT("session-c");
    const FString DirectoryA = MakeEvidenceDirectory(TEXT("projection-multi-robot-a"));
    const FString DirectoryDuplicate = MakeEvidenceDirectory(TEXT("projection-multi-robot-duplicate"));
    const FString DirectoryC = MakeEvidenceDirectory(TEXT("projection-multi-robot-c"));
    const FString ProjectionRelativePathA =
        TEXT("Saved/Automation/LingTuSimVisual/projection-multi-robot-a/robot.visual-projection.json");
    const FString ProjectionRelativePathDuplicate =
        TEXT("Saved/Automation/LingTuSimVisual/projection-multi-robot-duplicate/robot.visual-projection.json");
    const FString ProjectionRelativePathC =
        TEXT("Saved/Automation/LingTuSimVisual/projection-multi-robot-c/robot.visual-projection.json");
    FString Error;
    LingTuSim::FSessionService::UnbindSession();

    TestTrue(
        TEXT("two-robot projection fixture writes"),
        WriteVisualBundle(
            DirectoryA,
            ProjectionRelativePathA,
            ProjectionJson(
                TEXT("RobotVisual:OmniCart"),
                TEXT("robot.package.yaml"),
                TEXT("primitive"),
                TEXT("box"),
                TEXT("base_visual_frame"),
                TEXT("/Engine/BasicShapes/Cube.Cube"),
                TEXT("[1.0,1.0,1.0]"),
                TEXT("[1.0,1.0,1.0]")),
            Error,
            SessionA));
    TestTrue(
        TEXT("two distinct robot entries share one compiled projection"),
        [&]()
        {
            TArray<FString> RobotEntries;
            RobotEntries.Add(VisualPlanRobotEntryJson(TEXT("cart_01"), ProjectionRelativePathA));
            RobotEntries.Add(VisualPlanRobotEntryJson(TEXT("cart_02"), ProjectionRelativePathA));
            return RewriteVisualPlanRobots(DirectoryA, SessionA, RobotEntries);
        }());

    UWorld* World = CreateVisualTestWorld();
    TestNotNull(TEXT("multi-robot world exists"), World);
    LingTuSim::Visual::FVisualMaterializationResult Result;
    LingTuSim::Visual::FVisualMaterializationError LoadError;
    TestTrue(
        TEXT("two robots sharing a projection materialize"),
        World != nullptr
            && LingTuSim::Visual::FRobotVisualProjectionMaterializer::MaterializeBundle(
                *World,
                DirectoryA,
                FPaths::ProjectDir(),
                7,
                0,
                Result,
                LoadError));
    TestEqual(TEXT("two robot entries produce two body actors"), Result.ExpectedBodyCount, 2);
    TSet<FString> SharedProjectionStableIds;
    for (ALingTuSimBodyActor* Actor : Result.Actors)
    {
        if (IsValid(Actor) && Actor->BodyBinding != nullptr)
        {
            SharedProjectionStableIds.Add(Actor->BodyBinding->StableId);
        }
    }
    TestTrue(TEXT("cart_01 stable ID exists"), SharedProjectionStableIds.Contains(TEXT("cart_01/base")));
    TestTrue(TEXT("cart_02 stable ID exists"), SharedProjectionStableIds.Contains(TEXT("cart_02/base")));
    TestEqual(TEXT("prefixed stable IDs do not collide"), SharedProjectionStableIds.Num(), 2);
    for (ALingTuSimBodyActor* Actor : Result.Actors)
    {
        if (IsValid(Actor))
        {
            Actor->Destroy();
        }
    }

    ULingTuSimVisualWorldSubsystem* Subsystem =
        World != nullptr ? World->GetSubsystem<ULingTuSimVisualWorldSubsystem>() : nullptr;
    TestNotNull(TEXT("visual subsystem exists"), Subsystem);
    FString StartError;
    TestTrue(
        *FString::Printf(TEXT("initial two-robot plan starts; StartError='%s'"), *StartError),
        Subsystem != nullptr
            && Subsystem->StartVisualPlan(DirectoryA, FPaths::ProjectDir(), 7, 0, StartError));
    TestEqual(TEXT("registered binding count matches two robot bodies"), Subsystem != nullptr ? Subsystem->GetRegisteredBindingCount() : 0, 2);
    TestEqual(TEXT("active actor count is two"), CountValidBodyActors(World), 2);

    TestTrue(
        TEXT("duplicate instance fixture writes"),
        WriteVisualBundle(
            DirectoryDuplicate,
            ProjectionRelativePathDuplicate,
            ProjectionJson(
                TEXT("RobotVisual:OmniCart"),
                TEXT("robot.package.yaml"),
                TEXT("primitive"),
                TEXT("box"),
                TEXT("base_visual_frame"),
                TEXT("/Engine/BasicShapes/Cube.Cube"),
                TEXT("[1.0,1.0,1.0]"),
                TEXT("[1.0,1.0,1.0]")),
            Error,
            SessionB));
    TestTrue(
        TEXT("duplicate robot entries share one instance ID"),
        [&]()
        {
            TArray<FString> RobotEntries;
            RobotEntries.Add(VisualPlanRobotEntryJson(TEXT("cart_03"), ProjectionRelativePathDuplicate));
            RobotEntries.Add(VisualPlanRobotEntryJson(TEXT("cart_03"), ProjectionRelativePathDuplicate));
            return RewriteVisualPlanRobots(DirectoryDuplicate, SessionB, RobotEntries);
        }());

    StartError.Reset();
    TestFalse(
        TEXT("duplicate instance_id fails atomically"),
        Subsystem != nullptr
            && Subsystem->StartVisualPlan(DirectoryDuplicate, FPaths::ProjectDir(), 8, 0, StartError));
    TestTrue(TEXT("duplicate instance failure is explicit"), StartError.Contains(TEXT("duplicate instance_id")));
    TestEqual(TEXT("failed duplicate plan preserves previous actor count"), CountValidBodyActors(World), 2);
    TestEqual(TEXT("failed duplicate plan preserves previous bindings"), Subsystem != nullptr ? Subsystem->GetRegisteredBindingCount() : 0, 2);
    TMap<FString, ALingTuSimBodyActor*> ActorsAfterFailure = CollectBodyActorsByStableId(World);
    TestTrue(TEXT("cart_01/base remains active after duplicate failure"), ActorsAfterFailure.Contains(TEXT("cart_01/base")));
    TestTrue(TEXT("cart_02/base remains active after duplicate failure"), ActorsAfterFailure.Contains(TEXT("cart_02/base")));

    TestTrue(
        TEXT("higher-generation three-body fixture writes"),
        [&]()
        {
            TArray<FString> LocalBodies;
            LocalBodies.Add(TEXT("base"));
            LocalBodies.Add(TEXT("front_axle"));
            LocalBodies.Add(TEXT("rear_axle"));
            return WriteVisualBundle(
                DirectoryC,
                ProjectionRelativePathC,
                MultiBodyProjectionJson(LocalBodies),
                Error,
                SessionC);
        }());
    StartError.Reset();
    TestTrue(
        *FString::Printf(TEXT("higher generation plan changes body count; StartError='%s'"), *StartError),
        Subsystem != nullptr
            && Subsystem->StartVisualPlan(DirectoryC, FPaths::ProjectDir(), 9, 0, StartError));
    TestEqual(TEXT("higher generation leaves three active actors"), CountValidBodyActors(World), 3);
    TestEqual(TEXT("higher generation leaves three registered bindings"), Subsystem != nullptr ? Subsystem->GetRegisteredBindingCount() : 0, 3);
    const TMap<FString, ALingTuSimBodyActor*> ActorsAfterSwitch = CollectBodyActorsByStableId(World);
    TestFalse(TEXT("old cart_02/base actor is gone"), ActorsAfterSwitch.Contains(TEXT("cart_02/base")));
    TestTrue(TEXT("new cart_01/base actor exists"), ActorsAfterSwitch.Contains(TEXT("cart_01/base")));
    TestTrue(TEXT("new cart_01/front_axle actor exists"), ActorsAfterSwitch.Contains(TEXT("cart_01/front_axle")));
    TestTrue(TEXT("new cart_01/rear_axle actor exists"), ActorsAfterSwitch.Contains(TEXT("cart_01/rear_axle")));

    if (World != nullptr)
    {
        World->DestroyWorld(false);
    }
    IFileManager::Get().DeleteDirectory(*DirectoryA, false, true);
    IFileManager::Get().DeleteDirectory(*DirectoryDuplicate, false, true);
    IFileManager::Get().DeleteDirectory(*DirectoryC, false, true);
    LingTuSim::FSessionService::UnbindSession();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimVisualProjectionMeshScaleOnceTest,
    "LingTuSim.Visual.Runtime.Projection.MeshScaleOnce",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuSimVisualProjectionMeshScaleOnceTest::RunTest(const FString& Parameters)
{
    (void)Parameters;

    const FString Directory = MakeEvidenceDirectory(TEXT("projection-mesh"));
    const FString ProjectionRelativePath =
        TEXT("Saved/Automation/LingTuSimVisual/projection-mesh/robot.visual-projection.json");
    FString Error;
    TestTrue(
        TEXT("mesh bundle fixture writes"),
        WriteVisualBundle(
            Directory,
            ProjectionRelativePath,
            ProjectionJson(
                TEXT("RobotVisual:OmniCart"),
                TEXT("robot.package.yaml"),
                TEXT("mesh"),
                TEXT(""),
                TEXT("base_visual_frame"),
                TEXT("/Engine/BasicShapes/Cube.Cube"),
                TEXT("[2.0,2.0,2.0]"),
                TEXT("[2.0,2.0,2.0]")),
            Error));

    UWorld* World = CreateVisualTestWorld();
    LingTuSim::Visual::FVisualMaterializationResult Result;
    LingTuSim::Visual::FVisualMaterializationError LoadError;
    TestTrue(
        TEXT("mesh visual plan materializes"),
        World != nullptr
            && LingTuSim::Visual::FRobotVisualProjectionMaterializer::MaterializeBundle(
                *World,
                Directory,
                FPaths::ProjectDir(),
                3,
                4,
                Result,
                LoadError));
    if (Result.Actors.Num() == 1 && IsValid(Result.Actors[0]))
    {
        TArray<UStaticMeshComponent*> Components;
        Result.Actors[0]->GetComponents<UStaticMeshComponent>(Components);
        TestEqual(TEXT("one mesh visual attached"), Components.Num(), 1);
        if (Components.Num() == 1)
        {
            TestTrue(TEXT("mesh scale is applied once"), Components[0]->GetRelativeScale3D().Equals(FVector(2.0, 2.0, 2.0)));
        }
    }
    if (World != nullptr)
    {
        World->DestroyWorld(false);
    }
    IFileManager::Get().DeleteDirectory(*Directory, false, true);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimVisualProjectionRejectsUnsafePathAndDuplicateTest,
    "LingTuSim.Visual.Runtime.Projection.RejectsUnsafePathAndDuplicate",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuSimVisualProjectionRejectsUnsafePathAndDuplicateTest::RunTest(const FString& Parameters)
{
    (void)Parameters;

    const FString BaseDirectory = MakeEvidenceDirectory(TEXT("projection-failures"));
    FString Error;

    const FString GoodProjection = ProjectionJson(
        TEXT("RobotVisual:OmniCart"),
        TEXT("robot.package.yaml"),
        TEXT("primitive"),
        TEXT("box"),
        TEXT("base_visual_frame"),
        TEXT("/Engine/BasicShapes/Cube.Cube"),
        TEXT("[1.0,1.0,1.0]"),
        TEXT("[1.0,1.0,1.0]"));

    FString UnsafeDirectory = FPaths::Combine(BaseDirectory, TEXT("unsafe"));
    TestTrue(
        TEXT("unsafe fixture writes"),
        WriteVisualBundle(
            UnsafeDirectory,
            TEXT("Saved/Automation/LingTuSimVisual/projection failures/robot.visual-projection.json"),
            GoodProjection,
            Error));
    UWorld* World = CreateVisualTestWorld();
    LingTuSim::Visual::FVisualMaterializationResult Result;
    LingTuSim::Visual::FVisualMaterializationError LoadError;
    TestFalse(
        TEXT("unsafe projection path is rejected before materialization"),
        World != nullptr
            && LingTuSim::Visual::FRobotVisualProjectionMaterializer::MaterializeBundle(
                *World,
                UnsafeDirectory,
                FPaths::ProjectDir(),
                1,
                0,
                Result,
                LoadError));
    TestTrue(TEXT("unsafe path failure is explicit"), LoadError.Message.Contains(TEXT("safe repository-relative")));

    FString DuplicateDirectory = FPaths::Combine(BaseDirectory, TEXT("duplicate"));
    FString DuplicateProjectionWithTwoComponents = GoodProjection.LeftChop(2)
        + TEXT(",")
        + PrimitiveProjectionComponentJson(TEXT("base2"), TEXT("base_visual_frame"), TEXT("other_visual"))
        + TEXT("]}");
    TestTrue(
        TEXT("duplicate fixture writes"),
        WriteVisualBundle(
            DuplicateDirectory,
            TEXT("Saved/Automation/LingTuSimVisual/projection-duplicate/robot.visual-projection.json"),
            DuplicateProjectionWithTwoComponents,
            Error));
    LoadError = LingTuSim::Visual::FVisualMaterializationError();
    TestFalse(
        TEXT("duplicate visual_frame_id rejects"),
        World != nullptr
            && LingTuSim::Visual::FRobotVisualProjectionMaterializer::MaterializeBundle(
                *World,
                DuplicateDirectory,
                FPaths::ProjectDir(),
                1,
                0,
                Result,
                LoadError));
    TestTrue(TEXT("duplicate failure is explicit"), LoadError.Message.Contains(TEXT("duplicate visual_frame_id")));

    if (World != nullptr)
    {
        World->DestroyWorld(false);
    }
    IFileManager::Get().DeleteDirectory(*BaseDirectory, false, true);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimVisualProjectionCandidateActorsRollbackTest,
    "LingTuSim.Visual.Runtime.Projection.CandidateActorsRollback",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuSimVisualProjectionCandidateActorsRollbackTest::RunTest(const FString& Parameters)
{
    (void)Parameters;

    const FString SessionA = TEXT("session-a");
    const FString SessionB = TEXT("session-b");
    const FString DirectoryA = MakeEvidenceDirectory(TEXT("projection-candidate-a"));
    const FString DirectoryB = MakeEvidenceDirectory(TEXT("projection-candidate-b"));
    const FString ProjectionRelativePathA =
        TEXT("Saved/Automation/LingTuSimVisual/projection-candidate-a/robot.visual-projection.json");
    const FString ProjectionRelativePathB =
        TEXT("Saved/Automation/LingTuSimVisual/projection-candidate-b/robot.visual-projection.json");
    FString Error;
    LingTuSim::FSessionService::UnbindSession();

    TestTrue(
        TEXT("candidate bundle A fixture writes"),
        WriteVisualBundle(
            DirectoryA,
            ProjectionRelativePathA,
            ProjectionJson(
                TEXT("RobotVisual:OmniCart"),
                TEXT("robot.package.yaml"),
                TEXT("primitive"),
                TEXT("box"),
                TEXT("candidate_visual_frame"),
                TEXT("/Engine/BasicShapes/Cube.Cube"),
                TEXT("[1.0,1.0,1.0]"),
                TEXT("[1.0,1.0,1.0]")),
            Error,
            SessionA));
    TestTrue(
        TEXT("candidate bundle B fixture writes"),
        WriteVisualBundle(
            DirectoryB,
            ProjectionRelativePathB,
            ProjectionJson(
                TEXT("RobotVisual:OmniCart"),
                TEXT("robot.package.yaml"),
                TEXT("primitive"),
                TEXT("box"),
                TEXT("candidate_visual_frame"),
                TEXT("/Engine/BasicShapes/Cube.Cube"),
                TEXT("[1.0,1.0,1.0]"),
                TEXT("[1.0,1.0,1.0]")),
            Error,
            SessionB));

    UWorld* World = CreateVisualTestWorld();
    TestNotNull(TEXT("candidate lifecycle world exists"), World);
    ULingTuSimVisualWorldSubsystem* Subsystem =
        World != nullptr ? World->GetSubsystem<ULingTuSimVisualWorldSubsystem>() : nullptr;
    TestNotNull(TEXT("visual subsystem exists"), Subsystem);

    FString StartError;
    const bool bInitialVisualPlanStarted = Subsystem != nullptr
        && Subsystem->StartVisualPlan(
            DirectoryA,
            FPaths::ProjectDir(),
            11,
            0,
            StartError);
    TestTrue(
        *FString::Printf(
            TEXT("initial visual plan commits one active body actor; StartError='%s'"),
            *StartError),
        bInitialVisualPlanStarted);
    TestEqual(TEXT("initial commit leaves one body actor"), CountValidBodyActors(World), 1);

    StartError.Reset();
    TestTrue(
        TEXT("same session and model generation is idempotent"),
        Subsystem != nullptr
            && Subsystem->StartVisualPlan(
                DirectoryA,
                FPaths::ProjectDir(),
                11,
                0,
                StartError));
    TestEqual(TEXT("idempotent candidate rollback leaves one body actor"), CountValidBodyActors(World), 1);

    StartError.Reset();
    TestFalse(
        TEXT("same model generation cannot switch sessions"),
        Subsystem != nullptr
            && Subsystem->StartVisualPlan(
                DirectoryB,
                FPaths::ProjectDir(),
                11,
                0,
                StartError));
    TestTrue(TEXT("session switch failure is explicit"), StartError.Contains(TEXT("different session")));
    TestEqual(TEXT("failed switch candidate rollback leaves no orphan body actor"), CountValidBodyActors(World), 1);

    if (World != nullptr)
    {
        World->DestroyWorld(false);
    }
    IFileManager::Get().DeleteDirectory(*DirectoryA, false, true);
    IFileManager::Get().DeleteDirectory(*DirectoryB, false, true);
    LingTuSim::FSessionService::UnbindSession();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimVisualProjectionCommandLineDefersUntilBeginPlayTest,
    "LingTuSim.Visual.Runtime.Projection.CommandLineDefersUntilBeginPlay",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuSimVisualProjectionCommandLineDefersUntilBeginPlayTest::RunTest(const FString& Parameters)
{
    (void)Parameters;

    const FString Directory = MakeEvidenceDirectory(TEXT("projection-lifecycle"));
    const FString ProjectionRelativePath =
        TEXT("Saved/Automation/LingTuSimVisual/projection-lifecycle/robot.visual-projection.json");
    FString Error;
    TestTrue(
        TEXT("lifecycle bundle fixture writes"),
        WriteVisualBundle(
            Directory,
            ProjectionRelativePath,
            ProjectionJson(
                TEXT("RobotVisual:OmniCart"),
                TEXT("robot.package.yaml"),
                TEXT("primitive"),
                TEXT("box"),
                TEXT("base_visual_frame"),
                TEXT("/Engine/BasicShapes/Cube.Cube"),
                TEXT("[1.0,1.0,1.0]"),
                TEXT("[1.0,1.0,1.0]")),
            Error));

    const FString OriginalCommandLine = FCommandLine::Get();
    const FString TestCommandLine = FString::Printf(
        TEXT("-LingTuBundle=\"%s\" -LingTuArtifactRoot=\"%s\" -LingTuModelGeneration=9 -LingTuResetGeneration=1"),
        *Directory,
        *FPaths::ProjectDir());
    FCommandLine::Set(*TestCommandLine);

    UWorld* World = CreateVisualTestWorld();
    TestNotNull(TEXT("lifecycle world exists"), World);
    ULingTuSimVisualWorldSubsystem* Subsystem =
        World != nullptr ? World->GetSubsystem<ULingTuSimVisualWorldSubsystem>() : nullptr;
    TestNotNull(TEXT("visual subsystem exists"), Subsystem);
    if (Subsystem != nullptr)
    {
        Subsystem->Tick(0.0F);
    }
    int32 ActorCountBeforeBeginPlay = 0;
    if (World != nullptr)
    {
        for (TActorIterator<ALingTuSimBodyActor> It(World); It; ++It)
        {
            ++ActorCountBeforeBeginPlay;
        }
    }
    TestEqual(TEXT("command-line materialization is deferred before BeginPlay"), ActorCountBeforeBeginPlay, 0);

    if (World != nullptr)
    {
        World->InitializeActorsForPlay(FURL());
        World->BeginPlay();
        World->GetWorldSettings()->NotifyBeginPlay();
    }
    if (Subsystem != nullptr)
    {
        Subsystem->Tick(0.0F);
    }
    int32 ActorCountAfterBeginPlay = 0;
    if (World != nullptr)
    {
        for (TActorIterator<ALingTuSimBodyActor> It(World); It; ++It)
        {
            ++ActorCountAfterBeginPlay;
        }
    }
    TestEqual(TEXT("command-line materialization starts after BeginPlay"), ActorCountAfterBeginPlay, 1);

    FCommandLine::Set(*OriginalCommandLine);
    if (World != nullptr)
    {
        World->DestroyWorld(false);
    }
    IFileManager::Get().DeleteDirectory(*Directory, false, true);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimVisualSessionCameraViewTargetTest,
    "LingTuSim.Visual.Runtime.SessionCameraViewTarget",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuSimVisualSessionCameraViewTargetTest::RunTest(const FString& Parameters)
{
    (void)Parameters;

    UWorld* World = CreateVisualTestWorld();
    TestNotNull(TEXT("session camera test world exists"), World);
    ULingTuSimVisualWorldSubsystem* Subsystem =
        World != nullptr ? World->GetSubsystem<ULingTuSimVisualWorldSubsystem>() : nullptr;
    TestNotNull(TEXT("visual subsystem exists"), Subsystem);

    APlayerController* PlayerController =
        World != nullptr ? World->SpawnActor<APlayerController>() : nullptr;
    TestNotNull(TEXT("player 0 controller exists"), PlayerController);
    if (World != nullptr && PlayerController != nullptr)
    {
        World->AddController(PlayerController);
        PlayerController->SpawnPlayerCameraManager();
    }

    ACameraActor* UntaggedCamera =
        World != nullptr ? World->SpawnActor<ACameraActor>() : nullptr;
    ACameraActor* SessionCamera =
        World != nullptr ? World->SpawnActor<ACameraActor>() : nullptr;
    TestNotNull(TEXT("untagged camera exists"), UntaggedCamera);
    TestNotNull(TEXT("session camera exists"), SessionCamera);
    if (SessionCamera != nullptr)
    {
        SessionCamera->Tags.Add(TEXT("SessionCamera"));
    }

    TestTrue(
        TEXT("SessionCamera actor tag is selected for player 0"),
        Subsystem != nullptr && Subsystem->SetSessionCameraViewTargetForPlayer0());
    if (PlayerController != nullptr)
    {
        TestEqual(
            TEXT("player 0 view target is tagged session camera"),
            PlayerController->GetViewTarget(),
            Cast<AActor>(SessionCamera));
    }

    if (World != nullptr)
    {
        World->DestroyWorld(false);
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimVisualRuntimeCameraModesUseConfirmedViewTargetsTest,
    "LingTuSim.Visual.Runtime.CameraModesUseConfirmedViewTargets",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuSimVisualRuntimeCameraModesUseConfirmedViewTargetsTest::RunTest(
    const FString& Parameters)
{
    (void)Parameters;
    const FString OriginalCommandLine(FCommandLine::Get());
    FCommandLine::Set(*FString::Printf(
        TEXT("%s -LingTuMotionCameraStableId=robot/base_link ")
        TEXT("-LingTuSessionCameraTag=InspectionCamera"),
        *OriginalCommandLine));
    LingTuSim::FSessionService::UnbindSession();

    UWorld* World = CreateVisualTestWorld();
    TestNotNull(TEXT("runtime camera test world exists"), World);
    ULingTuSimVisualWorldSubsystem* Subsystem =
        World != nullptr ? World->GetSubsystem<ULingTuSimVisualWorldSubsystem>() : nullptr;
    TestNotNull(TEXT("visual subsystem exists"), Subsystem);
    APlayerController* PlayerController =
        World != nullptr ? World->SpawnActor<APlayerController>() : nullptr;
    TestNotNull(TEXT("player 0 controller exists"), PlayerController);
    if (World != nullptr && PlayerController != nullptr)
    {
        World->AddController(PlayerController);
        PlayerController->SpawnPlayerCameraManager();
    }

    const FString Session = TEXT("session-f");
    TestTrue(TEXT("camera fixture session binds"),
             Subsystem != nullptr && Subsystem->RebindSession(Session, 1));
    ULingTuSimBodyBindingComponent* Binding =
        World != nullptr ? NewObject<ULingTuSimBodyBindingComponent>(World) : nullptr;
    TestNotNull(TEXT("camera fixture body binding exists"), Binding);
    if (Binding != nullptr)
    {
        Binding->StableId = TEXT("robot/base_link");
    }
    TestTrue(TEXT("motion-camera body binding registers"),
             Subsystem != nullptr && Binding != nullptr
                 && Subsystem->RegisterBinding(Binding));
    TestEqual(TEXT("motion-camera truth queues"),
              Subsystem != nullptr
                  ? Subsystem->SubmitSnapshot(
                        MakeBodySnapshot(Session, 1, 0, 1, TEXT("robot/base_link")))
                  : LingTuSim::ESnapshotPublishResult::SessionMismatch,
              LingTuSim::ESnapshotPublishResult::Accepted);
    if (Subsystem != nullptr)
    {
        Subsystem->Tick(0.0F);
    }
    TestEqual(TEXT("applied body truth binds the real follow camera"),
              Subsystem != nullptr ? Subsystem->GetRuntimeCameraModeName() : FString(),
              FString(TEXT("follow")));
    AActor* FollowTarget =
        PlayerController != nullptr ? PlayerController->GetViewTarget() : nullptr;
    TestTrue(TEXT("follow mode readback names the motion camera"),
             IsValid(FollowTarget)
                 && FollowTarget->ActorHasTag(FName(TEXT("LingTuMotionCamera"))));

    ACameraActor* InspectionCamera =
        World != nullptr ? World->SpawnActor<ACameraActor>() : nullptr;
    TestNotNull(TEXT("inspection camera exists"), InspectionCamera);
    if (InspectionCamera != nullptr)
    {
        InspectionCamera->Tags.Add(FName(TEXT("InspectionCamera")));
    }
    FString Error;
    TestTrue(TEXT("unique inspection tag binds the actual view target"),
             Subsystem != nullptr
                 && Subsystem->SetRuntimeCameraMode(
                     ELingTuSimRuntimeCameraMode::Inspection, Error));
    TestEqual(TEXT("inspection view-target readback is exact"),
              PlayerController != nullptr ? PlayerController->GetViewTarget() : nullptr,
              Cast<AActor>(InspectionCamera));

    TestTrue(TEXT("free camera is spawned from the confirmed current view"),
             Subsystem != nullptr
                 && Subsystem->SetRuntimeCameraMode(
                     ELingTuSimRuntimeCameraMode::Free, Error));
    AActor* FreeTarget =
        PlayerController != nullptr ? PlayerController->GetViewTarget() : nullptr;
    TestTrue(TEXT("free mode readback names the free camera"),
             IsValid(FreeTarget)
                 && FreeTarget->ActorHasTag(FName(TEXT("LingTuFreeCamera"))));
    const FRotator RotationBefore =
        IsValid(FreeTarget) ? FreeTarget->GetActorRotation() : FRotator::ZeroRotator;
    TestTrue(TEXT("free-look axes update only the confirmed free view target"),
             Subsystem != nullptr
                 && Subsystem->ApplyRuntimeFreeCameraLook(0.75F, -0.50F, Error));
    TestTrue(TEXT("free-look rotation has real readback"),
             IsValid(FreeTarget)
                 && !FreeTarget->GetActorRotation().Equals(RotationBefore, 0.01F));

    ACameraActor* AmbiguousInspectionCamera =
        World != nullptr ? World->SpawnActor<ACameraActor>() : nullptr;
    if (AmbiguousInspectionCamera != nullptr)
    {
        AmbiguousInspectionCamera->Tags.Add(FName(TEXT("InspectionCamera")));
    }
    TestFalse(TEXT("ambiguous inspection tag fails closed"),
              Subsystem != nullptr
                  && Subsystem->SetRuntimeCameraMode(
                      ELingTuSimRuntimeCameraMode::Inspection, Error));
    TestEqual(TEXT("failed camera switch preserves the prior free target"),
              PlayerController != nullptr ? PlayerController->GetViewTarget() : nullptr,
              FreeTarget);
    TestEqual(TEXT("failed camera switch preserves the prior mode"),
              Subsystem != nullptr ? Subsystem->GetRuntimeCameraModeName() : FString(),
              FString(TEXT("free")));

    FCommandLine::Set(*OriginalCommandLine);
    if (World != nullptr)
    {
        World->DestroyWorld(false);
    }
    LingTuSim::FSessionService::UnbindSession();
    return true;
}

#endif

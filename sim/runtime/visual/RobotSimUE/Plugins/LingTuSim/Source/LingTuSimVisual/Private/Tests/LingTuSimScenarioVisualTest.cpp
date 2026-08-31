#if WITH_DEV_AUTOMATION_TESTS

#include "LingTuSimScenarioActor.h"
#include "LingTuSimPresentationPolicy.h"
#include "LingTuSimScenarioVisualRegistry.h"
#include "LingTuSimSessionService.h"
#include "LingTuSimVisualWorldSubsystem.h"

#include "Engine/World.h"
#include "Engine/WorldInitializationValues.h"
#include "Components/StaticMeshComponent.h"
#include "HAL/FileManager.h"
#include "Misc/AutomationTest.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

namespace
{
    UWorld* CreateScenarioVisualTestWorld()
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
                TEXT("LingTuScenarioVisualTest_%s"),
                *FGuid::NewGuid().ToString(EGuidFormats::Digits))),
            nullptr,
            true,
            ERHIFeatureLevel::Num,
            &InitializationValues);
    }

    FString ScenarioEntityJson(
        const FString& StableId,
        const FString& Authority,
        const FString& PositionJson,
        const FString& SemanticClass,
        const FString& PhysicsProxyMode,
        const FString& BodyStableIdJson)
    {
        return FString::Printf(
            TEXT("{\"entity_id\":\"%s\",\"transform\":{\"position_m\":%s,\"quaternion_wxyz\":[1.0,0.0,0.0,0.0]},\"authority\":\"%s\",\"source_epoch\":0,\"semantic_class\":\"%s\",\"motion_state\":\"active\",\"physics_proxy_mode\":\"%s\",\"body_stable_id\":%s}"),
            *StableId,
            *PositionJson,
            *Authority,
            *SemanticClass,
            *PhysicsProxyMode,
            *BodyStableIdJson);
    }

    FString ScenarioSnapshotJson(
        const FString& SessionId,
        const uint64 ModelGeneration,
        const uint64 ResetGeneration,
        const uint64 Sequence,
        const int64 SimTimeNs,
        const FString& EntitiesJson)
    {
        return FString::Printf(
            TEXT("{\"schema\":\"lingtu.sim.scenario-snapshot.v1\",\"session_id\":\"%s\",\"model_generation\":%llu,\"reset_generation\":%llu,\"sequence\":%llu,\"sim_time_ns\":%lld,\"entities\":[%s]}"),
            *SessionId,
            static_cast<unsigned long long>(ModelGeneration),
            static_cast<unsigned long long>(ResetGeneration),
            static_cast<unsigned long long>(Sequence),
            static_cast<long long>(SimTimeNs),
            *EntitiesJson);
    }

    bool WriteRunAllocationDocument(
        const FString& AllocationPath,
        const FString& RunId,
        const FString& LogDirectory)
    {
        TSharedPtr<FJsonObject> Root = MakeShared<FJsonObject>();
        Root->SetStringField(TEXT("schema"), TEXT("lingtu.sim.run-allocation.v1"));
        Root->SetStringField(TEXT("run_id"), RunId);
        Root->SetStringField(TEXT("session_id"), TEXT("session-d"));
        Root->SetStringField(TEXT("log_dir"), LogDirectory);
        FString Json;
        const TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&Json);
        return FJsonSerializer::Serialize(Root.ToSharedRef(), Writer)
            && FFileHelper::SaveStringToFile(Json, *AllocationPath);
    }

    ULingTuSimVisualWorldSubsystem* CreateSubsystemForCommandLine(
        const FString& CommandLine,
        UWorld*& OutWorld)
    {
        FCommandLine::Set(*CommandLine);
        OutWorld = CreateScenarioVisualTestWorld();
        return OutWorld != nullptr
            ? OutWorld->GetSubsystem<ULingTuSimVisualWorldSubsystem>()
            : nullptr;
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuScenarioVisualCreatesActorsFromCanonicalSnapshotTest,
    "LingTuSim.Visual.Scenario.CreatesActorsFromCanonicalSnapshot",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuScenarioVisualCreatesActorsFromCanonicalSnapshotTest::RunTest(
    const FString& Parameters)
{
    (void)Parameters;
    using LingTuSim::Visual::EScenarioVisualApplyResult;

    const FString SessionId = TEXT("session-a");
    UWorld* World = CreateScenarioVisualTestWorld();
    TestNotNull(TEXT("scenario visual test world exists"), World);
    LingTuSim::Visual::FScenarioVisualActorRegistry Registry;
    TestTrue(
        TEXT("registry binds the current session"),
        World != nullptr
            && Registry.BindSession(*World, SessionId, 7)
            && Registry.BindRunId(TEXT("scenario-canonical-snapshot")));

    const FString Pedestrian = ScenarioEntityJson(
        TEXT("pedestrian_01"),
        TEXT("scenario"),
        TEXT("[1.25,-2.0,0.9]"),
        TEXT("person"),
        TEXT("kinematic"),
        TEXT("\"pedestrian_01/proxy_root\""));
    const FString Flag = ScenarioEntityJson(
        TEXT("signage/flag_01"),
        TEXT("ue_animation"),
        TEXT("[3.0,4.0,1.0]"),
        TEXT("flag"),
        TEXT("none"),
        TEXT("null"));
    FString Error;
    TestTrue(
        TEXT("canonical snapshot is accepted"),
        World != nullptr
            && Registry.ApplySnapshotJson(
                *World,
                ScenarioSnapshotJson(
                    SessionId,
                    7,
                    2,
                    5,
                    500'000'000,
                    Pedestrian + TEXT(",") + Flag),
                Error)
                == EScenarioVisualApplyResult::Accepted);
    TestEqual(TEXT("one actor is created per stable_id"), Registry.Num(), 2);
    TestNotNull(
        TEXT("stable_id is preserved when its actor object name needs sanitizing"),
        Registry.FindActor(TEXT("signage/flag_01")));

    const ALingTuSimScenarioActor* Actor = Registry.FindActor(TEXT("pedestrian_01"));
    TestNotNull(TEXT("pedestrian actor is bound by stable_id"), Actor);
    if (Actor != nullptr)
    {
        TestTrue(
            TEXT("snapshot x converts from metres to centimetres"),
            FMath::IsNearlyEqual(Actor->GetActorLocation().X, 125.0, 0.01));
        TestTrue(
            TEXT("snapshot y changes handedness"),
            FMath::IsNearlyEqual(Actor->GetActorLocation().Y, 200.0, 0.01));
        TestTrue(
            TEXT("snapshot z converts from metres to centimetres"),
            FMath::IsNearlyEqual(Actor->GetActorLocation().Z, 90.0, 0.01));
        TestEqual(TEXT("actor remains a snapshot projection"), Actor->Authority, FString(TEXT("scenario")));
        TestFalse(TEXT("scenario visual actor collision remains disabled"), Actor->GetActorEnableCollision());
        TestTrue(
            TEXT("scenario mesh satisfies the complete presentation-only policy"),
            Actor->VisualMesh != nullptr
                && LingTuSim::Visual::HasPresentationPolicy(*Actor->VisualMesh));
        TestFalse(TEXT("accepted actor generation is revealed"), Actor->IsHidden());
    }

    TSharedPtr<FJsonObject> Evidence;
    const TSharedRef<TJsonReader<>> Reader =
        TJsonReaderFactory<>::Create(Registry.GetLastEvidenceJson());
    TestTrue(
        TEXT("scenario visual evidence is valid JSON"),
        FJsonSerializer::Deserialize(Reader, Evidence) && Evidence.IsValid());
    if (Evidence.IsValid())
    {
        TestEqual(
            TEXT("evidence source proves UE registry application"),
            Evidence->GetStringField(TEXT("source")),
            FString(TEXT("ue_registry_applied")));
        TestEqual(
            TEXT("evidence identifies the canonical scenario input"),
            Evidence->GetStringField(TEXT("input_source")),
            FString(TEXT("canonical_scenario_snapshot")));
        TestEqual(
            TEXT("evidence run_id comes from the bound run context"),
            Evidence->GetStringField(TEXT("run_id")),
            FString(TEXT("scenario-canonical-snapshot")));
        TestEqual(
            TEXT("evidence keeps source sequence"),
            static_cast<int32>(Evidence->GetNumberField(TEXT("sequence"))),
            5);
        TestTrue(
            TEXT("maximum_position_error_m is within 2 cm"),
            Evidence->GetNumberField(TEXT("maximum_position_error_m")) <= 0.02);
        TestEqual(
            TEXT("evidence expected actor count matches the snapshot"),
            static_cast<int32>(Evidence->GetNumberField(TEXT("expected_actor_count"))),
            2);
        TestTrue(
            TEXT("evidence covers the complete actor set"),
            Evidence->GetBoolField(TEXT("complete_actor_set")));
        TestTrue(
            TEXT("evidence proves every accepted actor is visible"),
            Evidence->GetBoolField(TEXT("all_actors_visible")));
        const TArray<TSharedPtr<FJsonValue>>* Actors = nullptr;
        TestTrue(
            TEXT("evidence actors array exists"),
            Evidence->TryGetArrayField(TEXT("actors"), Actors) && Actors != nullptr);
        if (Actors != nullptr && !Actors->IsEmpty())
        {
            const TSharedPtr<FJsonObject>* ActorObject = nullptr;
            TestTrue(
                TEXT("evidence actor is an object"),
                (*Actors)[0]->TryGetObject(ActorObject)
                    && ActorObject != nullptr
                    && ActorObject->IsValid());
            if (ActorObject != nullptr && ActorObject->IsValid())
            {
                TestEqual(
                    TEXT("evidence actor contains canonical entity_id"),
                    (*ActorObject)->GetStringField(TEXT("entity_id")),
                    FString(TEXT("pedestrian_01")));
            }
        }
        TestTrue(TEXT("evidence passes tolerance"), Evidence->GetBoolField(TEXT("within_tolerance")));
    }

    if (World != nullptr)
    {
        Registry.Clear(*World);
        World->DestroyWorld(false);
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuScenarioVisualRejectsWrongIdentityAndOutOfOrderTest,
    "LingTuSim.Visual.Scenario.RejectsWrongIdentityAndOutOfOrder",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuScenarioVisualRejectsWrongIdentityAndOutOfOrderTest::RunTest(
    const FString& Parameters)
{
    (void)Parameters;
    using LingTuSim::Visual::EScenarioVisualApplyResult;

    const FString SessionId = TEXT("session-b");
    const FString WrongSessionId = TEXT("session-c");
    const FString Entity = ScenarioEntityJson(
        TEXT("pedestrian_order"),
        TEXT("scenario"),
        TEXT("[0.0,0.0,0.9]"),
        TEXT("person"),
        TEXT("kinematic"),
        TEXT("\"pedestrian_order/proxy_root\""));
    UWorld* World = CreateScenarioVisualTestWorld();
    LingTuSim::Visual::FScenarioVisualActorRegistry Registry;
    FString Error;
    TestTrue(
        TEXT("registry binds ordering test session"),
        World != nullptr
            && Registry.BindSession(*World, SessionId, 4)
            && Registry.BindRunId(TEXT("scenario-ordering")));
    TestTrue(
        TEXT("first observed sequence may be non-zero"),
        World != nullptr
            && Registry.ApplySnapshotJson(
                *World,
                ScenarioSnapshotJson(SessionId, 4, 3, 7, 700, Entity),
                Error)
                == EScenarioVisualApplyResult::Accepted);
    TestTrue(
        TEXT("duplicate sequence is rejected"),
        World != nullptr
            && Registry.ApplySnapshotJson(
                *World,
                ScenarioSnapshotJson(SessionId, 4, 3, 7, 701, Entity),
                Error)
                == EScenarioVisualApplyResult::OldSequence);
    TestTrue(
        TEXT("wrong session id is rejected"),
        World != nullptr
            && Registry.ApplySnapshotJson(
                *World,
                ScenarioSnapshotJson(WrongSessionId, 4, 3, 8, 800, Entity),
                Error)
                == EScenarioVisualApplyResult::SessionMismatch);
    TestTrue(
        TEXT("old model generation is rejected"),
        World != nullptr
            && Registry.ApplySnapshotJson(
                *World,
                ScenarioSnapshotJson(SessionId, 3, 3, 8, 800, Entity),
                Error)
                == EScenarioVisualApplyResult::OldModel);
    TestTrue(
        TEXT("future model generation is rejected until rebind"),
        World != nullptr
            && Registry.ApplySnapshotJson(
                *World,
                ScenarioSnapshotJson(SessionId, 5, 0, 0, 0, Entity),
                Error)
                == EScenarioVisualApplyResult::FutureModel);
    TestTrue(
        TEXT("old reset generation is rejected"),
        World != nullptr
            && Registry.ApplySnapshotJson(
                *World,
                ScenarioSnapshotJson(SessionId, 4, 2, 100, 10'000, Entity),
                Error)
                == EScenarioVisualApplyResult::OldReset);

    if (World != nullptr)
    {
        Registry.Clear(*World);
        World->DestroyWorld(false);
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuScenarioVisualResetGenerationRebuildsActorsFromFirstObservedSequenceTest,
    "LingTuSim.Visual.Scenario.ResetGenerationRebuildsActorsFromFirstObservedSequence",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuScenarioVisualResetGenerationRebuildsActorsFromFirstObservedSequenceTest::RunTest(
    const FString& Parameters)
{
    (void)Parameters;
    using LingTuSim::Visual::EScenarioVisualApplyResult;

    const FString SessionId = TEXT("session-d");
    const FString Entity = ScenarioEntityJson(
        TEXT("pedestrian_reset"),
        TEXT("scenario"),
        TEXT("[1.0,1.0,0.9]"),
        TEXT("person"),
        TEXT("kinematic"),
        TEXT("\"pedestrian_reset/proxy_root\""));
    UWorld* World = CreateScenarioVisualTestWorld();
    LingTuSim::Visual::FScenarioVisualActorRegistry Registry;
    FString Error;
    TestTrue(
        TEXT("registry binds reset test session"),
        World != nullptr
            && Registry.BindSession(*World, SessionId, 8)
            && Registry.BindRunId(TEXT("scenario-reset")));
    TestTrue(
        TEXT("initial reset generation applies"),
        World != nullptr
            && Registry.ApplySnapshotJson(
                *World,
                ScenarioSnapshotJson(SessionId, 8, 0, 9, 900, Entity),
                Error)
                == EScenarioVisualApplyResult::Accepted);
    const ALingTuSimScenarioActor* OriginalActor =
        Registry.FindActor(TEXT("pedestrian_reset"));
    TestNotNull(TEXT("initial actor exists"), OriginalActor);

    TestTrue(
        TEXT("first observed packet after reset may skip sequence zero"),
        World != nullptr
            && Registry.ApplySnapshotJson(
                *World,
                ScenarioSnapshotJson(SessionId, 8, 1, 4, 400, Entity),
                Error)
                == EScenarioVisualApplyResult::Accepted);
    const ALingTuSimScenarioActor* ResetActor = Registry.FindActor(TEXT("pedestrian_reset"));
    TestNotNull(TEXT("reset actor exists"), ResetActor);
    TestTrue(
        TEXT("reset rebuild replaces actor identity"),
        ResetActor != nullptr && ResetActor != OriginalActor);

    if (World != nullptr)
    {
        Registry.Clear(*World);
        World->DestroyWorld(false);
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuScenarioVisualRejectsDuplicateStableIdAndMalformedEntityTest,
    "LingTuSim.Visual.Scenario.RejectsDuplicateStableIdAndMalformedEntity",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuScenarioVisualRejectsDuplicateStableIdAndMalformedEntityTest::RunTest(
    const FString& Parameters)
{
    (void)Parameters;
    using LingTuSim::Visual::EScenarioVisualApplyResult;

    const FString SessionId = TEXT("session-e");
    const FString Entity = ScenarioEntityJson(
        TEXT("pedestrian_strict"),
        TEXT("scenario"),
        TEXT("[0.0,0.0,0.9]"),
        TEXT("person"),
        TEXT("kinematic"),
        TEXT("\"pedestrian_strict/proxy_root\""));
    UWorld* World = CreateScenarioVisualTestWorld();
    LingTuSim::Visual::FScenarioVisualActorRegistry Registry;
    FString Error;
    TestTrue(
        TEXT("registry binds strict validation session"),
        World != nullptr && Registry.BindSession(*World, SessionId, 2));
    TestFalse(TEXT("empty run_id is rejected"), Registry.BindRunId(FString{}));
    TestFalse(
        TEXT("path-like run_id is rejected"),
        Registry.BindRunId(TEXT("../scenario-run")));
    TestTrue(
        TEXT("canonical run_id binds"),
        Registry.BindRunId(TEXT("scenario-strict-validation")));
    TestFalse(
        TEXT("run_id cannot change after binding"),
        Registry.BindRunId(TEXT("scenario-other-run")));

    TestTrue(
        TEXT("duplicate stable_id is rejected"),
        World != nullptr
            && Registry.ApplySnapshotJson(
                *World,
                ScenarioSnapshotJson(
                    SessionId,
                    2,
                    0,
                    0,
                    0,
                    Entity + TEXT(",") + Entity),
                Error)
                == EScenarioVisualApplyResult::DuplicateStableId);

    const FString UnknownFieldEntity = Entity.Replace(
        TEXT("\"transform\":"),
        TEXT("\"unexpected\":true,\"transform\":"));
    TestTrue(
        TEXT("unknown entity field is rejected"),
        World != nullptr
            && Registry.ApplySnapshotJson(
                *World,
                ScenarioSnapshotJson(SessionId, 2, 0, 0, 0, UnknownFieldEntity),
                Error)
                == EScenarioVisualApplyResult::InvalidSnapshot);

    const FString UnsupportedAuthorityEntity = ScenarioEntityJson(
        TEXT("pedestrian_wrong_owner"),
        TEXT("mujoco"),
        TEXT("[0.0,0.0,0.9]"),
        TEXT("person"),
        TEXT("kinematic"),
        TEXT("\"pedestrian_wrong_owner/proxy_root\""));
    TestTrue(
        TEXT("unsupported authority is rejected"),
        World != nullptr
            && Registry.ApplySnapshotJson(
                *World,
                ScenarioSnapshotJson(SessionId, 2, 0, 0, 0, UnsupportedAuthorityEntity),
                Error)
                == EScenarioVisualApplyResult::InvalidSnapshot);
    TestEqual(TEXT("rejected batches create no actors"), Registry.Num(), 0);

    if (World != nullptr)
    {
        Registry.Clear(*World);
        World->DestroyWorld(false);
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuScenarioVisualEntitySetChangesOnlyOnResetTest,
    "LingTuSim.Visual.Scenario.EntitySetChangesOnlyOnReset",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuScenarioVisualEntitySetChangesOnlyOnResetTest::RunTest(
    const FString& Parameters)
{
    (void)Parameters;
    using LingTuSim::Visual::EScenarioVisualApplyResult;

    const FString SessionId = TEXT("session-f");
    const FString EntityA = ScenarioEntityJson(
        TEXT("pedestrian_a"),
        TEXT("scenario"),
        TEXT("[0.0,0.0,0.9]"),
        TEXT("person"),
        TEXT("kinematic"),
        TEXT("\"pedestrian_a/proxy_root\""));
    const FString EntityB = ScenarioEntityJson(
        TEXT("pedestrian_b"),
        TEXT("scenario"),
        TEXT("[2.0,0.0,0.9]"),
        TEXT("person"),
        TEXT("kinematic"),
        TEXT("\"pedestrian_b/proxy_root\""));
    UWorld* World = CreateScenarioVisualTestWorld();
    LingTuSim::Visual::FScenarioVisualActorRegistry Registry;
    FString Error;
    TestTrue(
        TEXT("registry binds entity set session"),
        World != nullptr
            && Registry.BindSession(*World, SessionId, 6)
            && Registry.BindRunId(TEXT("scenario-entity-set")));
    TestTrue(
        TEXT("initial entity set applies"),
        World != nullptr
            && Registry.ApplySnapshotJson(
                *World,
                ScenarioSnapshotJson(SessionId, 6, 0, 0, 0, EntityA),
                Error)
                == EScenarioVisualApplyResult::Accepted);
    TestTrue(
        TEXT("entity set change outside reset is rejected"),
        World != nullptr
            && Registry.ApplySnapshotJson(
                *World,
                ScenarioSnapshotJson(SessionId, 6, 0, 1, 100, EntityB),
                Error)
                == EScenarioVisualApplyResult::EntitySetMismatch);
    TestNotNull(TEXT("original entity remains after rejection"), Registry.FindActor(TEXT("pedestrian_a")));
    TestNull(TEXT("replacement entity was not partially spawned"), Registry.FindActor(TEXT("pedestrian_b")));

    TestTrue(
        TEXT("new reset generation may replace the stable identity set"),
        World != nullptr
            && Registry.ApplySnapshotJson(
                *World,
                ScenarioSnapshotJson(SessionId, 6, 1, 3, 300, EntityB),
                Error)
                == EScenarioVisualApplyResult::Accepted);
    TestNull(TEXT("old reset actor is destroyed"), Registry.FindActor(TEXT("pedestrian_a")));
    TestNotNull(TEXT("new reset actor exists"), Registry.FindActor(TEXT("pedestrian_b")));

    if (World != nullptr)
    {
        Registry.Clear(*World);
        World->DestroyWorld(false);
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuScenarioVisualSameResetFailureKeepsCompletePreviousFrameTest,
    "LingTuSim.Visual.Scenario.SameResetFailureKeepsCompletePreviousFrame",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuScenarioVisualSameResetFailureKeepsCompletePreviousFrameTest::RunTest(
    const FString& Parameters)
{
    (void)Parameters;
    using LingTuSim::Visual::EScenarioVisualApplyResult;

    const FString SessionId = TEXT("session-7");
    const FString InitialA = ScenarioEntityJson(
        TEXT("pedestrian_transaction_a"),
        TEXT("scenario"),
        TEXT("[1.0,0.0,0.9]"),
        TEXT("person"),
        TEXT("kinematic"),
        TEXT("\"pedestrian_transaction_a/proxy_root\""));
    const FString InitialB = ScenarioEntityJson(
        TEXT("pedestrian_transaction_b"),
        TEXT("scenario"),
        TEXT("[2.0,0.0,0.9]"),
        TEXT("person"),
        TEXT("kinematic"),
        TEXT("\"pedestrian_transaction_b/proxy_root\""));
    UWorld* World = CreateScenarioVisualTestWorld();
    LingTuSim::Visual::FScenarioVisualActorRegistry Registry;
    FString Error;
    TestTrue(
        TEXT("registry binds transaction test session"),
        World != nullptr
            && Registry.BindSession(*World, SessionId, 13)
            && Registry.BindRunId(TEXT("scenario-frame-transaction")));
    TestTrue(
        TEXT("complete initial frame applies"),
        World != nullptr
            && Registry.ApplySnapshotJson(
                *World,
                ScenarioSnapshotJson(
                    SessionId,
                    13,
                    0,
                    0,
                    0,
                    InitialA + TEXT(",") + InitialB),
                Error)
                == EScenarioVisualApplyResult::Accepted);

    ALingTuSimScenarioActor* ActorA = Registry.FindActor(TEXT("pedestrian_transaction_a"));
    ALingTuSimScenarioActor* ActorB = Registry.FindActor(TEXT("pedestrian_transaction_b"));
    TestNotNull(TEXT("transaction actor A exists"), ActorA);
    TestNotNull(TEXT("transaction actor B exists"), ActorB);
    if (ActorB != nullptr && ActorB->VisualMesh != nullptr)
    {
        ActorB->VisualMesh->SetStaticMesh(nullptr);
    }

    const FString UpdatedA = ScenarioEntityJson(
        TEXT("pedestrian_transaction_a"),
        TEXT("scenario"),
        TEXT("[4.0,0.0,0.9]"),
        TEXT("person"),
        TEXT("kinematic"),
        TEXT("\"pedestrian_transaction_a/proxy_root\""));
    const FString UpdatedB = ScenarioEntityJson(
        TEXT("pedestrian_transaction_b"),
        TEXT("scenario"),
        TEXT("[5.0,0.0,0.9]"),
        TEXT("person"),
        TEXT("kinematic"),
        TEXT("\"pedestrian_transaction_b/proxy_root\""));
    TestTrue(
        TEXT("incomplete same-reset frame is rejected"),
        World != nullptr
            && Registry.ApplySnapshotJson(
                *World,
                ScenarioSnapshotJson(
                    SessionId,
                    13,
                    0,
                    1,
                    100,
                    UpdatedA + TEXT(",") + UpdatedB),
                Error)
                == EScenarioVisualApplyResult::TransformRejected);
    if (ActorA != nullptr)
    {
        TestTrue(
            TEXT("failed same-reset frame leaves earlier actor at previous pose"),
            FMath::IsNearlyEqual(ActorA->GetActorLocation().X, 100.0, 0.01));
    }

    if (World != nullptr)
    {
        Registry.Clear(*World);
        World->DestroyWorld(false);
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuScenarioVisualMailboxToGameThreadRegistryWritesEvidenceTest,
    "LingTuSim.Visual.Scenario.MailboxToGameThreadRegistryWritesEvidence",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuScenarioVisualMailboxToGameThreadRegistryWritesEvidenceTest::RunTest(
    const FString& Parameters)
{
    (void)Parameters;
    const FString SessionId = TEXT("session-9");
    const FString EvidenceDirectory = FPaths::Combine(
        FPaths::ProjectSavedDir(),
        TEXT("Automation"),
        TEXT("LingTuScenarioVisual"),
        FGuid::NewGuid().ToString(EGuidFormats::Digits));
    const FString Entity = ScenarioEntityJson(
        TEXT("pedestrian_mailbox"),
        TEXT("scenario"),
        TEXT("[1.0,-1.0,0.9]"),
        TEXT("person"),
        TEXT("kinematic"),
        TEXT("\"pedestrian_mailbox/proxy_root\""));

    LingTuSim::FSessionService::UnbindSession();
    UWorld* World = CreateScenarioVisualTestWorld();
    ULingTuSimVisualWorldSubsystem* Subsystem =
        World != nullptr ? World->GetSubsystem<ULingTuSimVisualWorldSubsystem>() : nullptr;
    TestNotNull(TEXT("visual subsystem exists"), Subsystem);
    TestTrue(
        TEXT("visual subsystem binds scenario source identity"),
        Subsystem != nullptr && Subsystem->RebindSession(SessionId, 12));
    FString Error;
    TestTrue(
        TEXT("scenario evidence output is configured"),
        Subsystem != nullptr
            && Subsystem->ConfigureReadinessEvidence(
                EvidenceDirectory,
                TEXT("scenario-mailbox-evidence"),
                SessionId,
                12,
                3,
                Error));

    LingTuSim::ESnapshotPublishResult PublishResult =
        LingTuSim::ESnapshotPublishResult::Stale;
    TestTrue(
        TEXT("canonical JSON enters the shared raw mailbox"),
        Subsystem != nullptr
            && Subsystem->SubmitScenarioSnapshotJson(
                ScenarioSnapshotJson(SessionId, 12, 3, 6, 654'321, Entity),
                PublishResult,
                Error)
            && PublishResult == LingTuSim::ESnapshotPublishResult::Accepted);
    if (Subsystem != nullptr)
    {
        Subsystem->Tick(0.0F);
    }
    TestEqual(
        TEXT("game-thread tick creates one scenario actor"),
        Subsystem != nullptr ? Subsystem->GetScenarioActorCount() : 0,
        1);

    const FString EvidencePath = FPaths::Combine(
        EvidenceDirectory,
        TEXT("scenario-visual-evidence.json"));
    FString EvidenceJson;
    TestTrue(
        TEXT("scenario-visual-evidence.json is written"),
        FFileHelper::LoadFileToString(EvidenceJson, *EvidencePath));
    TSharedPtr<FJsonObject> Evidence;
    const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(EvidenceJson);
    TestTrue(
        TEXT("written scenario evidence parses"),
        FJsonSerializer::Deserialize(Reader, Evidence) && Evidence.IsValid());
    if (Evidence.IsValid())
    {
        TestEqual(
            TEXT("written evidence proves UE registry application"),
            Evidence->GetStringField(TEXT("source")),
            FString(TEXT("ue_registry_applied")));
        TestEqual(
            TEXT("written evidence run_id matches authoritative allocation context"),
            Evidence->GetStringField(TEXT("run_id")),
            FString(TEXT("scenario-mailbox-evidence")));
        TestEqual(
            TEXT("evidence session_id matches source snapshot"),
            Evidence->GetStringField(TEXT("session_id")),
            SessionId);
        TestEqual(
            TEXT("evidence model generation matches source snapshot"),
            static_cast<int32>(Evidence->GetNumberField(TEXT("model_generation"))),
            12);
        TestEqual(
            TEXT("evidence reset generation matches source snapshot"),
            static_cast<int32>(Evidence->GetNumberField(TEXT("reset_generation"))),
            3);
        TestEqual(
            TEXT("evidence sequence matches source snapshot"),
            static_cast<int32>(Evidence->GetNumberField(TEXT("sequence"))),
            6);
        TestEqual(
            TEXT("evidence sim_time_ns matches source snapshot"),
            static_cast<int64>(Evidence->GetNumberField(TEXT("sim_time_ns"))),
            int64{654'321});
        const TArray<TSharedPtr<FJsonValue>>* Actors = nullptr;
        if (Evidence->TryGetArrayField(TEXT("actors"), Actors)
            && Actors != nullptr
            && !Actors->IsEmpty())
        {
            const TSharedPtr<FJsonObject>* ActorObject = nullptr;
            if ((*Actors)[0]->TryGetObject(ActorObject)
                && ActorObject != nullptr
                && ActorObject->IsValid())
            {
                TestEqual(
                    TEXT("written evidence actor keeps canonical entity_id"),
                    (*ActorObject)->GetStringField(TEXT("entity_id")),
                    FString(TEXT("pedestrian_mailbox")));
            }
        }
    }

    if (World != nullptr)
    {
        World->DestroyWorld(false);
    }
    IFileManager::Get().DeleteDirectory(*EvidenceDirectory, false, true);
    LingTuSim::FSessionService::UnbindSession();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuScenarioVisualRejectsMissingAllocationGenerationArgumentsTest,
    "LingTuSim.Visual.Scenario.RejectsMissingAllocationGenerationArguments",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuScenarioVisualRejectsMissingAllocationGenerationArgumentsTest::RunTest(
    const FString& Parameters)
{
    (void)Parameters;
    const FString OriginalCommandLine = FCommandLine::Get();
    const FString BaseDirectory = FPaths::ConvertRelativePathToFull(FPaths::Combine(
        FPaths::ProjectSavedDir(),
        TEXT("Automation/LingTuSimVisual"),
        TEXT("required-generations-") + FGuid::NewGuid().ToString(EGuidFormats::Digits)));
    const FString RunId = TEXT("scenario-required-generations");
    const FString RunDirectory = FPaths::Combine(BaseDirectory, RunId);
    const FString LogDirectory = FPaths::Combine(RunDirectory, TEXT("logs"));
    const FString AllocationPath = FPaths::Combine(
        RunDirectory,
        TEXT("run-allocation.json"));
    IFileManager::Get().MakeDirectory(*LogDirectory, true);
    TestTrue(
        TEXT("valid allocation fixture writes"),
        WriteRunAllocationDocument(AllocationPath, RunId, LogDirectory));

    AddExpectedError(
        TEXT("reason=missing or invalid generation"),
        EAutomationExpectedErrorFlags::Contains,
        1);
    UWorld* MissingModelWorld = nullptr;
    ULingTuSimVisualWorldSubsystem* MissingModelSubsystem =
        CreateSubsystemForCommandLine(
            FString::Printf(
                TEXT("-LingTuRunAllocation=\"%s\" -LingTuRunId=%s -LingTuResetGeneration=0"),
                *AllocationPath,
                *RunId),
            MissingModelWorld);
    TestNotNull(TEXT("missing model generation is rejected"), MissingModelSubsystem);
    if (MissingModelWorld != nullptr)
    {
        MissingModelWorld->DestroyWorld(false);
    }

    AddExpectedError(
        TEXT("reason=missing or invalid generation"),
        EAutomationExpectedErrorFlags::Contains,
        1);
    UWorld* MissingResetWorld = nullptr;
    ULingTuSimVisualWorldSubsystem* MissingResetSubsystem =
        CreateSubsystemForCommandLine(
            FString::Printf(
                TEXT("-LingTuRunAllocation=\"%s\" -LingTuRunId=%s -LingTuModelGeneration=0"),
                *AllocationPath,
                *RunId),
            MissingResetWorld);
    TestNotNull(TEXT("missing reset generation is rejected"), MissingResetSubsystem);
    if (MissingResetWorld != nullptr)
    {
        MissingResetWorld->DestroyWorld(false);
    }

    FCommandLine::Set(*OriginalCommandLine);
    IFileManager::Get().DeleteDirectory(*BaseDirectory, false, true);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuScenarioVisualRejectsUnsafeRunAllocationEvidencePathsTest,
    "LingTuSim.Visual.Scenario.RejectsUnsafeRunAllocationEvidencePaths",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuScenarioVisualRejectsUnsafeRunAllocationEvidencePathsTest::RunTest(
    const FString& Parameters)
{
    (void)Parameters;
    const FString OriginalCommandLine = FCommandLine::Get();
    const FString BaseDirectory = FPaths::ConvertRelativePathToFull(FPaths::Combine(
        FPaths::ProjectSavedDir(),
        TEXT("Automation/LingTuSimVisual"),
        TEXT("safe-allocation-paths-") + FGuid::NewGuid().ToString(EGuidFormats::Digits)));
    const FString RunId = TEXT("scenario-safe-allocation-paths");
    const FString RunDirectory = FPaths::Combine(BaseDirectory, RunId);
    const FString LogDirectory = FPaths::Combine(RunDirectory, TEXT("logs"));
    const FString AllocationPath = FPaths::Combine(
        RunDirectory,
        TEXT("run-allocation.json"));
    const FString ForeignLogDirectory = FPaths::Combine(BaseDirectory, TEXT("foreign/logs"));
    IFileManager::Get().MakeDirectory(*LogDirectory, true);
    IFileManager::Get().MakeDirectory(*ForeignLogDirectory, true);

    struct FUnsafeLogCase
    {
        const TCHAR* Label;
        FString LogPath;
        const TCHAR* ExpectedReason;
    };
    const TArray<FUnsafeLogCase> Cases = {
        {TEXT("relative log_dir is rejected"), TEXT("logs"), TEXT("must be an absolute canonical directory")},
        {TEXT("traversal log_dir is rejected"), RunDirectory + TEXT("/nested/../logs"), TEXT("must be an absolute canonical directory")},
        {TEXT("foreign log_dir is rejected"), ForeignLogDirectory, TEXT("must exactly equal the allocation run directory logs path")},
    };
    for (const FUnsafeLogCase& TestCase : Cases)
    {
        TestTrue(
            TEXT("unsafe allocation fixture writes"),
            WriteRunAllocationDocument(AllocationPath, RunId, TestCase.LogPath));
        AddExpectedError(
            TestCase.ExpectedReason,
            EAutomationExpectedErrorFlags::Contains,
            1);
        UWorld* World = nullptr;
        ULingTuSimVisualWorldSubsystem* Subsystem = CreateSubsystemForCommandLine(
            FString::Printf(
                TEXT("-LingTuRunAllocation=\"%s\" -LingTuRunId=%s -LingTuModelGeneration=0 -LingTuResetGeneration=0"),
                *AllocationPath,
                *RunId),
            World);
        TestNotNull(TestCase.Label, Subsystem);
        if (World != nullptr)
        {
            World->DestroyWorld(false);
        }
    }

    AddExpectedError(
        TEXT("must be an absolute canonical run-allocation.json path"),
        EAutomationExpectedErrorFlags::Contains,
        1);
    UWorld* RelativeAllocationWorld = nullptr;
    TestNotNull(
        TEXT("relative allocation path is rejected"),
        CreateSubsystemForCommandLine(
            FString::Printf(
                TEXT("-LingTuRunAllocation=run-allocation.json -LingTuRunId=%s -LingTuModelGeneration=0 -LingTuResetGeneration=0"),
                *RunId),
            RelativeAllocationWorld));
    if (RelativeAllocationWorld != nullptr)
    {
        RelativeAllocationWorld->DestroyWorld(false);
    }

    FCommandLine::Set(*OriginalCommandLine);
    IFileManager::Get().DeleteDirectory(*BaseDirectory, false, true);
    return true;
}

#endif

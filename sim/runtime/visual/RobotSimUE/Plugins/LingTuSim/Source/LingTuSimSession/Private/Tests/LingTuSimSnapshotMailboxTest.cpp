#if WITH_DEV_AUTOMATION_TESTS

#include "LingTuSimSnapshotMailbox.h"
#include "LingTuSimSessionService.h"

#include "Async/ParallelFor.h"
#include "Misc/AutomationTest.h"

namespace
{
    constexpr uint64 BoundModelGeneration = 7;

    LingTuSim::FSnapshotEnvelope MakeSnapshot(
        const FString& SessionId,
        const uint64 ModelGeneration,
        const uint64 ResetGeneration,
        const uint64 Sequence)
    {
        LingTuSim::FSnapshotEnvelope Snapshot;
        Snapshot.SessionId = SessionId;
        Snapshot.ModelGeneration = ModelGeneration;
        Snapshot.ResetGeneration = ResetGeneration;
        Snapshot.Sequence = Sequence;
        Snapshot.SimTimeNs = static_cast<int64>(Sequence) * 1'000'000;

        LingTuSim::FEntityState Entity;
        Entity.Id.StableId = TEXT("robot/base_link");
        Entity.Id.InstanceId = TEXT("robot");
        Entity.Id.FrameId = TEXT("base_link");
        Snapshot.Entities.Add(MoveTemp(Entity));
        return Snapshot;
    }

    FString MakeScenarioSnapshotJson(
        const FString& SessionId,
        const uint64 ModelGeneration,
        const uint64 ResetGeneration,
        const uint64 Sequence,
        const int64 SimTimeNs)
    {
        return FString::Printf(
            TEXT("{\"schema\":\"lingtu.sim.scenario-snapshot.v1\",\"session_id\":\"%s\",\"model_generation\":%llu,\"reset_generation\":%llu,\"sequence\":%llu,\"sim_time_ns\":%lld,\"entities\":[]}"),
            *SessionId,
            static_cast<unsigned long long>(ModelGeneration),
            static_cast<unsigned long long>(ResetGeneration),
            static_cast<unsigned long long>(Sequence),
            static_cast<long long>(SimTimeNs));
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimSnapshotMailboxLatestWinsTest,
    "LingTuSim.Session.SnapshotMailbox.LatestWins",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuSimSnapshotMailboxLatestWinsTest::RunTest(const FString& Parameters)
{
    (void)Parameters;

    const FString SessionId = TEXT("session-a");
    LingTuSim::FSnapshotMailbox Mailbox;
    Mailbox.BindSession(SessionId, BoundModelGeneration);

    LingTuSim::FSnapshotEnvelope First = MakeSnapshot(
        SessionId,
        BoundModelGeneration,
        0,
        1);
    LingTuSim::FSnapshotEnvelope Third = MakeSnapshot(
        SessionId,
        BoundModelGeneration,
        0,
        3);
    LingTuSim::FSnapshotEnvelope Second = MakeSnapshot(
        SessionId,
        BoundModelGeneration,
        0,
        2);

    TestTrue(
        TEXT("Sequence 1 is accepted into an empty mailbox"),
        Mailbox.Publish(First) == LingTuSim::ESnapshotPublishResult::Accepted);
    TestTrue(
        TEXT("Sequence 3 replaces sequence 1"),
        Mailbox.Publish(Third) == LingTuSim::ESnapshotPublishResult::Replaced);

    Third.Entities[0].Id.StableId = TEXT("mutated-after-publish");
    TestTrue(
        TEXT("Sequence 2 is stale after sequence 3"),
        Mailbox.Publish(Second) == LingTuSim::ESnapshotPublishResult::Stale);
    TestEqual(TEXT("Only one snapshot is pending"), Mailbox.PendingCount(), 1);

    LingTuSim::FSnapshotEnvelope Taken;
    TestTrue(TEXT("The latest snapshot can be taken"), Mailbox.TryTakeLatest(Taken));
    TestEqual(TEXT("The mailbox yields sequence 3"), Taken.Sequence, uint64{3});
    TestEqual(
        TEXT("The mailbox owns a full snapshot copy"),
        Taken.Entities[0].Id.StableId,
        FString{TEXT("robot/base_link")});
    TestEqual(TEXT("Taking consumes the pending snapshot"), Mailbox.PendingCount(), 0);
    TestFalse(TEXT("A second take finds no snapshot"), Mailbox.TryTakeLatest(Taken));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimSnapshotMailboxCapacityTest,
    "LingTuSim.Session.SnapshotMailbox.ConcurrentCapacityOne",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuSimSnapshotMailboxCapacityTest::RunTest(const FString& Parameters)
{
    (void)Parameters;

    const FString SessionId = TEXT("session-capacity");
    LingTuSim::FSnapshotMailbox Mailbox;
    Mailbox.BindSession(SessionId, BoundModelGeneration);

    constexpr int32 PublishCount = 10'000;
    ParallelFor(
        PublishCount,
        [&Mailbox, &SessionId](const int32 Index)
        {
            Mailbox.Publish(MakeSnapshot(
                SessionId,
                BoundModelGeneration,
                0,
                static_cast<uint64>(Index)));
        });

    TestEqual(
        TEXT("Ten thousand concurrent publishes leave one pending snapshot"),
        Mailbox.PendingCount(),
        1);

    LingTuSim::FSnapshotEnvelope Taken;
    TestTrue(TEXT("The flood result can be taken"), Mailbox.TryTakeLatest(Taken));
    TestEqual(
        TEXT("The flood keeps the logically latest sequence"),
        Taken.Sequence,
        uint64{PublishCount - 1});
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimSnapshotMailboxResetGenerationTest,
    "LingTuSim.Session.SnapshotMailbox.ResetGenerationRestartsSequence",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuSimSnapshotMailboxResetGenerationTest::RunTest(const FString& Parameters)
{
    (void)Parameters;

    const FString SessionId = TEXT("session-reset");
    LingTuSim::FSnapshotMailbox Mailbox;
    Mailbox.BindSession(SessionId, BoundModelGeneration);

    TestTrue(
        TEXT("A snapshot in reset generation 4 is accepted"),
        Mailbox.Publish(MakeSnapshot(SessionId, BoundModelGeneration, 4, 91))
            == LingTuSim::ESnapshotPublishResult::Accepted);
    TestTrue(
        TEXT("A newer reset generation accepts sequence 0"),
        Mailbox.Publish(MakeSnapshot(SessionId, BoundModelGeneration, 5, 0))
            == LingTuSim::ESnapshotPublishResult::Replaced);
    TestTrue(
        TEXT("An older reset generation is stale even with a larger sequence"),
        Mailbox.Publish(MakeSnapshot(SessionId, BoundModelGeneration, 4, 1000))
            == LingTuSim::ESnapshotPublishResult::Stale);

    LingTuSim::FSnapshotEnvelope Taken;
    TestTrue(TEXT("The new reset generation remains pending"), Mailbox.TryTakeLatest(Taken));
    TestEqual(TEXT("Reset generation 5 is retained"), Taken.ResetGeneration, uint64{5});
    TestEqual(TEXT("Sequence restarts at 0"), Taken.Sequence, uint64{0});
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimSnapshotMailboxIdentityTest,
    "LingTuSim.Session.SnapshotMailbox.RejectsWrongIdentity",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuSimSnapshotMailboxIdentityTest::RunTest(const FString& Parameters)
{
    (void)Parameters;

    const FString SessionId = TEXT("session-bound");
    LingTuSim::FSnapshotMailbox Mailbox;
    Mailbox.BindSession(SessionId, BoundModelGeneration);

    TestTrue(
        TEXT("A different session is rejected"),
        Mailbox.Publish(MakeSnapshot(TEXT("session-old"), BoundModelGeneration, 0, 1))
            == LingTuSim::ESnapshotPublishResult::SessionMismatch);
    TestTrue(
        TEXT("A stale model generation is rejected"),
        Mailbox.Publish(MakeSnapshot(SessionId, BoundModelGeneration - 1, 0, 1))
            == LingTuSim::ESnapshotPublishResult::ModelMismatch);
    TestTrue(
        TEXT("A future model generation is rejected"),
        Mailbox.Publish(MakeSnapshot(SessionId, BoundModelGeneration + 1, 0, 1))
            == LingTuSim::ESnapshotPublishResult::ModelMismatch);
    TestEqual(TEXT("Rejected snapshots do not become pending"), Mailbox.PendingCount(), 0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimScenarioRawMailboxLatestWinsTest,
    "LingTuSim.Session.ScenarioRawMailboxLatestWins",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuSimScenarioRawMailboxLatestWinsTest::RunTest(const FString& Parameters)
{
    (void)Parameters;
    const FString SessionId = TEXT("session-a");
    const FString WrongSessionId = TEXT("session-b");
    constexpr uint64 ModelGeneration = 17;
    LingTuSim::FSessionService::UnbindSession();
    TestTrue(
        TEXT("scenario raw mailbox session binds"),
        LingTuSim::FSessionService::RebindSession(SessionId, ModelGeneration));

    LingTuSim::ESnapshotPublishResult Result = LingTuSim::ESnapshotPublishResult::Stale;
    FString Error;
    TestTrue(
        TEXT("first observed reset sequence can be non-zero"),
        LingTuSim::FSessionService::PublishScenarioSnapshotJson(
            MakeScenarioSnapshotJson(SessionId, ModelGeneration, 2, 5, 500),
            Result,
            Error)
            && Result == LingTuSim::ESnapshotPublishResult::Accepted);
    TestTrue(
        TEXT("newer scenario sequence replaces pending raw JSON"),
        LingTuSim::FSessionService::PublishScenarioSnapshotJson(
            MakeScenarioSnapshotJson(SessionId, ModelGeneration, 2, 7, 700),
            Result,
            Error)
            && Result == LingTuSim::ESnapshotPublishResult::Replaced);
    TestTrue(
        TEXT("stale scenario sequence is rejected"),
        LingTuSim::FSessionService::PublishScenarioSnapshotJson(
            MakeScenarioSnapshotJson(SessionId, ModelGeneration, 2, 6, 800),
            Result,
            Error)
            && Result == LingTuSim::ESnapshotPublishResult::Stale);
    TestTrue(
        TEXT("wrong scenario model generation is rejected"),
        LingTuSim::FSessionService::PublishScenarioSnapshotJson(
            MakeScenarioSnapshotJson(SessionId, ModelGeneration + 1, 0, 0, 0),
            Result,
            Error)
            && Result == LingTuSim::ESnapshotPublishResult::ModelMismatch);
    TestTrue(
        TEXT("wrong scenario session id is rejected"),
        LingTuSim::FSessionService::PublishScenarioSnapshotJson(
            MakeScenarioSnapshotJson(WrongSessionId, ModelGeneration, 2, 8, 800),
            Result,
            Error)
            && Result == LingTuSim::ESnapshotPublishResult::SessionMismatch);

    FString TakenJson;
    TestTrue(
        TEXT("raw scenario JSON is copied and consumed"),
        LingTuSim::FSessionService::TryTakeLatestScenarioSnapshotJson(TakenJson));
    TestTrue(TEXT("latest raw sequence is retained"), TakenJson.Contains(TEXT("\"sequence\":7")));
    TestFalse(
        TEXT("capacity-one mailbox is empty after take"),
        LingTuSim::FSessionService::TryTakeLatestScenarioSnapshotJson(TakenJson));

    TestTrue(
        TEXT("first observed packet of a newer reset may also skip zero"),
        LingTuSim::FSessionService::PublishScenarioSnapshotJson(
            MakeScenarioSnapshotJson(SessionId, ModelGeneration, 3, 4, 400),
            Result,
            Error)
            && Result == LingTuSim::ESnapshotPublishResult::Accepted);
    LingTuSim::FSessionService::UnbindSession();
    return true;
}

#endif

#if WITH_DEV_AUTOMATION_TESTS

#include "LingTuSimBundleLoader.h"

#include "Misc/AutomationTest.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"
#include "Misc/Parse.h"

namespace
{
    FString DescribeLoadResult(
        const TCHAR* ArtifactName,
        const LingTuSim::FRuntimeLoadError& Error)
    {
        return FString::Printf(
            TEXT("%s loads successfully (error code: %d, source: '%s', message: '%s')"),
            ArtifactName,
            static_cast<int32>(Error.Code),
            *Error.Source,
            *Error.Message);
    }

    FString MinimalArtifactJson(const TCHAR* Schema, const FString& SessionId)
    {
        return FString::Printf(
            TEXT("{\"schema\":\"%s\",\"session_id\":\"%s\"}"),
            Schema,
            *SessionId);
    }

    bool WriteBundleArtifact(
        const FString& Directory,
        const TCHAR* Filename,
        const TCHAR* Schema,
        const FString& SessionId)
    {
        return FFileHelper::SaveStringToFile(
            MinimalArtifactJson(Schema, SessionId),
            *FPaths::Combine(Directory, Filename),
            FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimRuntimeBundleLoaderContractTest,
    "LingTuSim.Runtime.BundleLoader.Contract",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuSimRuntimeBundleLoaderContractTest::RunTest(const FString& Parameters)
{
    (void)Parameters;
    const FString SessionId = TEXT("bundle-test");
    const FString Directory = FPaths::Combine(
        FPaths::ProjectSavedDir(),
        TEXT("Automation"),
        TEXT("LingTuSimRuntime"),
        FGuid::NewGuid().ToString(EGuidFormats::Digits));
    IFileManager::Get().MakeDirectory(*Directory, true);

    TestTrue(TEXT("writes physics plan"), WriteBundleArtifact(
        Directory, TEXT("physics.plan.json"), TEXT("lingtu.sim.physics-plan.v1"), SessionId));
    TestTrue(TEXT("writes visual plan"), WriteBundleArtifact(
        Directory, TEXT("visual.plan.json"), TEXT("lingtu.sim.visual-plan.v1"), SessionId));
    TestTrue(TEXT("writes sensor plan"), WriteBundleArtifact(
        Directory, TEXT("sensor.plan.json"), TEXT("lingtu.sim.sensor-plan.v1"), SessionId));
    TestTrue(TEXT("writes control plan"), WriteBundleArtifact(
        Directory, TEXT("control.plan.json"), TEXT("lingtu.sim.control-plan.v1"), SessionId));
    TestTrue(TEXT("writes transport intent"), WriteBundleArtifact(
        Directory, TEXT("transport.intent.json"), TEXT("lingtu.sim.transport-intent.v1"), SessionId));

    LingTuSim::FSessionBundleView Bundle;
    LingTuSim::FRuntimeLoadError Error;
    TestTrue(
        TEXT("path-only bundle loads"),
        LingTuSim::FSessionBundleLoader::LoadSessionBundle(Directory, Bundle, Error));
    TestEqual(TEXT("bundle keeps session_id"), Bundle.SessionId, SessionId);

    TestTrue(TEXT("writes physics plan v2"), WriteBundleArtifact(
        Directory, TEXT("physics.plan.json"), TEXT("lingtu.sim.physics-plan.v2"), SessionId));
    TestTrue(TEXT("writes visual plan v2"), WriteBundleArtifact(
        Directory, TEXT("visual.plan.json"), TEXT("lingtu.sim.visual-plan.v2"), SessionId));
    TestTrue(
        TEXT("payload-aware v2 bundle loads without weakening the other artifact contracts"),
        LingTuSim::FSessionBundleLoader::LoadSessionBundle(Directory, Bundle, Error));

    TestTrue(TEXT("writes unsupported visual plan version"), WriteBundleArtifact(
        Directory, TEXT("visual.plan.json"), TEXT("lingtu.sim.visual-plan.v3"), SessionId));
    TestFalse(
        TEXT("unknown visual plan versions remain rejected"),
        LingTuSim::FSessionBundleLoader::LoadSessionBundle(Directory, Bundle, Error));
    TestEqual(TEXT("unknown version reports schema mismatch"), Error.Code,
        LingTuSim::ERuntimeLoadErrorCode::SchemaMismatch);

    TestTrue(TEXT("restores physics plan v1"), WriteBundleArtifact(
        Directory, TEXT("physics.plan.json"), TEXT("lingtu.sim.physics-plan.v1"), SessionId));
    TestTrue(TEXT("restores visual plan v1 after version checks"), WriteBundleArtifact(
        Directory, TEXT("visual.plan.json"), TEXT("lingtu.sim.visual-plan.v1"), SessionId));

    TestTrue(TEXT("writes session id with a space"), WriteBundleArtifact(
        Directory, TEXT("visual.plan.json"), TEXT("lingtu.sim.visual-plan.v1"), TEXT("bad id")));
    TestFalse(
        TEXT("session_id with a space is rejected"),
        LingTuSim::FSessionBundleLoader::LoadSessionBundle(Directory, Bundle, Error));
    TestEqual(TEXT("invalid session_id reports invalid field"), Error.Code,
        LingTuSim::ERuntimeLoadErrorCode::InvalidField);

    TestTrue(TEXT("writes 64-character session id"), WriteBundleArtifact(
        Directory, TEXT("visual.plan.json"), TEXT("lingtu.sim.visual-plan.v1"),
        FString::ChrN(64, TEXT('a'))));
    TestFalse(
        TEXT("64-character session_id is rejected"),
        LingTuSim::FSessionBundleLoader::LoadSessionBundle(Directory, Bundle, Error));
    TestEqual(TEXT("overlong session_id reports invalid field"), Error.Code,
        LingTuSim::ERuntimeLoadErrorCode::InvalidField);

    TestTrue(TEXT("writes mismatched visual plan"), WriteBundleArtifact(
        Directory, TEXT("visual.plan.json"), TEXT("lingtu.sim.visual-plan.v1"), TEXT("other-session")));
    TestFalse(
        TEXT("mismatched plan session_id is rejected"),
        LingTuSim::FSessionBundleLoader::LoadSessionBundle(Directory, Bundle, Error));
    TestEqual(
        TEXT("mismatched session_id reports invalid field"),
        Error.Code,
        LingTuSim::ERuntimeLoadErrorCode::InvalidField);

    TestTrue(TEXT("restores visual plan"), WriteBundleArtifact(
        Directory, TEXT("visual.plan.json"), TEXT("lingtu.sim.visual-plan.v1"), SessionId));
    TestTrue(TEXT("writes invalid sensor JSON"), FFileHelper::SaveStringToFile(
        TEXT("{"), *FPaths::Combine(Directory, TEXT("sensor.plan.json")),
        FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM));
    TestFalse(TEXT("invalid plan JSON is rejected"),
        LingTuSim::FSessionBundleLoader::LoadSessionBundle(Directory, Bundle, Error));
    TestEqual(TEXT("invalid JSON reports invalid JSON"), Error.Code,
        LingTuSim::ERuntimeLoadErrorCode::InvalidJson);

    TestTrue(TEXT("restores sensor plan"), WriteBundleArtifact(
        Directory, TEXT("sensor.plan.json"), TEXT("lingtu.sim.sensor-plan.v1"), SessionId));
    TestTrue(TEXT("removes required control plan"),
        IFileManager::Get().Delete(*FPaths::Combine(Directory, TEXT("control.plan.json"))));
    TestFalse(TEXT("missing required plan is rejected"),
        LingTuSim::FSessionBundleLoader::LoadSessionBundle(Directory, Bundle, Error));
    TestEqual(TEXT("missing plan reports missing artifact"), Error.Code,
        LingTuSim::ERuntimeLoadErrorCode::MissingArtifact);

    IFileManager::Get().DeleteDirectory(*Directory, false, true);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimRuntimeBundleLoaderTest,
    "LingTuSim.Runtime.BundleLoader",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuSimRuntimeBundleLoaderTest::RunTest(const FString& Parameters)
{
    (void)Parameters;

    FString BundleDirectory;
    FString SnapshotPath;
    const bool bHasBundleDirectory =
        FParse::Value(FCommandLine::Get(), TEXT("LingTuTestBundle="), BundleDirectory)
        && !BundleDirectory.IsEmpty();
    const bool bHasSnapshotPath =
        FParse::Value(FCommandLine::Get(), TEXT("LingTuTestSnapshot="), SnapshotPath)
        && !SnapshotPath.IsEmpty();

    TestTrue(
        TEXT("Command line provides non-empty LingTuTestBundle=<bundle-directory>"),
        bHasBundleDirectory);
    TestTrue(
        TEXT("Command line provides non-empty LingTuTestSnapshot=<snapshot-file>"),
        bHasSnapshotPath);
    if (!bHasBundleDirectory || !bHasSnapshotPath)
    {
        return false;
    }

    LingTuSim::FSessionBundleView Bundle;
    LingTuSim::FRuntimeLoadError LoadError;
    const bool bBundleLoaded = LingTuSim::FSessionBundleLoader::LoadSessionBundle(
        BundleDirectory,
        Bundle,
        LoadError);
    TestTrue(DescribeLoadResult(TEXT("SessionBundle"), LoadError), bBundleLoaded);
    if (!bBundleLoaded)
    {
        return false;
    }

    if (!TestTrue(TEXT("Loaded SessionBundle reports IsBound()"), Bundle.IsBound()))
    {
        return false;
    }

    LingTuSim::FSnapshotEnvelope Snapshot;
    const bool bSnapshotLoaded = LingTuSim::FSessionBundleLoader::LoadSnapshotFile(
        SnapshotPath,
        Bundle.SessionId,
        Snapshot,
        LoadError);
    TestTrue(DescribeLoadResult(TEXT("Truth snapshot"), LoadError), bSnapshotLoaded);
    if (!bSnapshotLoaded)
    {
        return false;
    }

    TestEqual(
        TEXT("Snapshot session_id matches the loaded SessionBundle"),
        Snapshot.SessionId,
        Bundle.SessionId);
    TestEqual(
        TEXT("Snapshot model_generation is the initial model generation"),
        Snapshot.ModelGeneration,
        uint64{0});
    TestEqual(
        TEXT("Snapshot reset_generation is the initial reset generation"),
        Snapshot.ResetGeneration,
        uint64{0});
    TestTrue(
        TEXT("Snapshot sequence describes the supplied runtime point"),
        Snapshot.SimTimeNs == 0 || Snapshot.Sequence > 0);

    TestTrue(
        FString::Printf(
            TEXT("Snapshot contains dynamic entities (actual count: %d)"),
            Snapshot.Entities.Num()),
        !Snapshot.Entities.IsEmpty());
    const LingTuSim::FEntityState* ThunderBase = Snapshot.Entities.FindByPredicate(
        [](const LingTuSim::FEntityState& Entity)
        {
            return Entity.Id.StableId == TEXT("thunder_01/base_link");
        });
    TestTrue(
        TEXT("Snapshot entities contain stable_id 'thunder_01/base_link'"),
        ThunderBase != nullptr);

    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimRuntimeSnapshotSchemaContractTest,
    "LingTuSim.Runtime.BundleLoader.SnapshotSchemaContract",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuSimRuntimeSnapshotSchemaContractTest::RunTest(const FString& Parameters)
{
    (void)Parameters;
    const FString SessionId = TEXT("snapshot-contract");
    const FString ValidBody =
        TEXT("{\"body_id\":0,\"name\":\"base\",\"stable_id\":\"cart_01/base\",")
        TEXT("\"instance_id\":\"cart_01\",\"frame_id\":\"base\",")
        TEXT("\"position_m\":[0.0,0.0,0.0],\"quaternion_wxyz\":[1.0,0.0,0.0,0.0],")
        TEXT("\"linear_velocity_mps\":[0.0,0.0,0.0],\"angular_velocity_rps\":[0.0,0.0,0.0]}");
    const FString ValidSnapshot = FString::Printf(
        TEXT("{\"schema\":\"lingtu.sim.truth-snapshot.v1\",\"session_id\":\"%s\",")
        TEXT("\"model_generation\":1,\"reset_generation\":2,\"sequence\":3,\"physics_step\":4,")
        TEXT("\"sim_time_ns\":5,\"bodies\":[%s],\"joints\":[],\"actuators\":[],\"sensors\":[]}"),
        *SessionId,
        *ValidBody);

    LingTuSim::FSnapshotEnvelope Snapshot;
    LingTuSim::FRuntimeLoadError Error;
    TestTrue(
        TEXT("schema-shaped truth snapshot parses"),
        LingTuSim::FSessionBundleLoader::ParseSnapshotJson(
            ValidSnapshot,
            SessionId,
            Snapshot,
            Error));
    TestEqual(TEXT("physics_step is retained"), Snapshot.PhysicsStep, uint64{4});
    TestEqual(TEXT("one body is retained"), Snapshot.Entities.Num(), 1);

    FString InvalidSnapshot = ValidSnapshot.Replace(TEXT("\"physics_step\":4,"), TEXT(""));
    TestFalse(
        TEXT("physics_step is required"),
        LingTuSim::FSessionBundleLoader::ParseSnapshotJson(
            InvalidSnapshot,
            SessionId,
            Snapshot,
            Error));

    InvalidSnapshot = ValidSnapshot.Replace(TEXT("\"bodies\":"), TEXT("\"entities\":"));
    TestFalse(
        TEXT("legacy entities field is rejected"),
        LingTuSim::FSessionBundleLoader::ParseSnapshotJson(
            InvalidSnapshot,
            SessionId,
            Snapshot,
            Error));

    InvalidSnapshot = ValidSnapshot.Replace(TEXT("\"joints\":[]"), TEXT("\"unexpected\":true,\"joints\":[]"));
    TestFalse(
        TEXT("unknown snapshot root fields are rejected"),
        LingTuSim::FSessionBundleLoader::ParseSnapshotJson(
            InvalidSnapshot,
            SessionId,
            Snapshot,
            Error));

    InvalidSnapshot = ValidSnapshot.Replace(
        TEXT("\"linear_velocity_mps\":[0.0,0.0,0.0],"),
        TEXT(""));
    TestFalse(
        TEXT("body linear velocity is required"),
        LingTuSim::FSessionBundleLoader::ParseSnapshotJson(
            InvalidSnapshot,
            SessionId,
            Snapshot,
            Error));

    InvalidSnapshot = ValidSnapshot.Replace(
        TEXT(",\"angular_velocity_rps\":[0.0,0.0,0.0]"),
        TEXT(""));
    TestFalse(
        TEXT("body angular velocity is required"),
        LingTuSim::FSessionBundleLoader::ParseSnapshotJson(
            InvalidSnapshot,
            SessionId,
            Snapshot,
            Error));

    InvalidSnapshot = ValidSnapshot.Replace(
        TEXT("\"linear_velocity_mps\":[0.0,0.0,0.0],"),
        TEXT("\"linear_velocity_mps\":[0.0,0.0,0.0],\"unexpected\":true,"));
    TestFalse(
        TEXT("unknown body fields are rejected"),
        LingTuSim::FSessionBundleLoader::ParseSnapshotJson(
            InvalidSnapshot,
            SessionId,
            Snapshot,
            Error));

    return true;
}

#endif

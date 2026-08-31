#if WITH_DEV_AUTOMATION_TESTS

#include "LingTuSimControlTransport.h"
#include "LingTuSimSessionService.h"

#include "Misc/AutomationTest.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

namespace
{
    FString Digest(const TCHAR Character)
    {
        return FString::ChrN(64, Character);
    }

    FString AllocationJson(
        const uint16 IntentPort = 25124,
        const uint16 StatusPort = 25125)
    {
        return FString::Printf(
            TEXT("{\"schema\":\"lingtu.sim.run-allocation.v1\",\"run_id\":\"playable-run-001\",\"session_id\":\"%s\",\"artifact_root\":\"C:/artifact-root\",\"boot_id\":\"boot-001\",\"dds_domain\":1,\"ports\":{\"visual_snapshot_udp\":25123,\"control_intent_udp\":%u,\"control_status_udp\":%u},\"shm\":{},\"log_dir\":\"C:/runs/playable-run-001/logs\"}"),
            *Digest(TEXT('a')),
            IntentPort,
            StatusPort);
    }

    LingTuSim::FControlTransportBinding Binding()
    {
        LingTuSim::FControlTransportBinding Result;
        FString Error;
        const bool bParsed = LingTuSim::FControlTransportProtocol::ParseRunAllocationJson(
            AllocationJson(),
            TEXT("playable-run-001"),
            Digest(TEXT('a')),
            TEXT("C:/runs/playable-run-001/logs"),
            25124,
            25125,
            7,
            3,
            TEXT("robotsimue.local_player.0"),
            Result,
            Error);
        checkf(bParsed, TEXT("test binding must parse: %s"), *Error);
        return Result;
    }

    FString DatagramJson(const TArray<uint8>& Datagram)
    {
        const FUTF8ToTCHAR Converted(
            reinterpret_cast<const ANSICHAR*>(Datagram.GetData()),
            Datagram.Num());
        return FString(Converted.Length(), Converted.Get());
    }

    FString AckJson(
        const uint64 ModelGeneration = 7,
        const uint64 ResetGeneration = 3,
        const uint64 ServerStatusSequence = 9,
        const uint64 SourceEpoch = 1,
        const uint64 SourceSequence = 4,
        const FString& DatagramSha256 = Digest(TEXT('b')),
        const FString& Status = TEXT("accepted"),
        const FString& Reason = TEXT(""))
    {
        return FString::Printf(
            TEXT("{\"schema\":\"lingtu.sim.ue-control-ack.v1\",\"run_id\":\"playable-run-001\",\"session_id\":\"%s\",\"boot_id\":\"boot-001\",\"model_generation\":%llu,\"reset_generation\":%llu,\"server_status_sequence\":%llu,\"source_id\":\"robotsimue.local_player.0\",\"source_epoch\":%llu,\"source_sequence\":%llu,\"event_id\":\"boot-001:%llu:%llu\",\"intent_datagram_sha256\":\"%s\",\"status\":\"%s\",\"reason\":\"%s\"}"),
            *Digest(TEXT('a')),
            static_cast<unsigned long long>(ModelGeneration),
            static_cast<unsigned long long>(ResetGeneration),
            static_cast<unsigned long long>(ServerStatusSequence),
            static_cast<unsigned long long>(SourceEpoch),
            static_cast<unsigned long long>(SourceSequence),
            static_cast<unsigned long long>(SourceEpoch),
            static_cast<unsigned long long>(SourceSequence),
            *DatagramSha256,
            *Status,
            *Reason);
    }

    FString StatusJson(
        const FString& DatagramSha256 = Digest(TEXT('b')),
        const uint64 ServerStatusSequence = 10,
        const uint64 SourceEpoch = 1,
        const uint64 SourceSequence = 4)
    {
        return FString::Printf(
            TEXT("{\"schema\":\"lingtu.sim.ue-control-status.v1\",\"run_id\":\"playable-run-001\",\"session_id\":\"%s\",\"boot_id\":\"boot-001\",\"model_generation\":7,\"reset_generation\":3,\"server_status_sequence\":%llu,\"server_monotonic_ns\":123456789,\"sim_time_ns\":2000000000,\"truth_sequence\":600,\"source_id\":\"robotsimue.local_player.0\",\"source_epoch\":%llu,\"source_sequence\":%llu,\"event_id\":\"boot-001:%llu:%llu\",\"intent_datagram_sha256\":\"%s\",\"status\":\"accepted\",\"reason\":\"\",\"runtime\":{\"runtime_state\":\"RUNNING\",\"control_owner\":\"robotsimue.local_player.0\",\"deadman\":true,\"sample_age_ns\":10000000,\"safe_stop_state\":\"clear\"},\"motion\":{\"requested_axes\":{\"available\":true,\"forward\":1,\"left\":0,\"yaw_left\":0},\"admitted_twist_mps_radps\":{\"available\":true,\"linear_x\":0.1,\"linear_y\":0,\"angular_z\":0},\"observed_base_velocity_mps_radps\":{\"available\":true,\"linear_x\":0.08,\"linear_y\":0.01,\"angular_z\":0.02}},\"readiness\":{\"physics\":{\"state\":\"ACTIVE\",\"required\":true,\"source_id\":\"runtime.physics\",\"blocker\":\"\"},\"control\":{\"state\":\"ACTIVE\",\"required\":true,\"source_id\":\"runtime.control\",\"blocker\":\"\"},\"visual\":{\"state\":\"ACTIVE\",\"required\":true,\"source_id\":\"runtime.visual\",\"blocker\":\"\"},\"sensors\":{\"state\":\"ACTIVE\",\"required\":true,\"source_id\":\"runtime.sensors\",\"blocker\":\"\"}},\"sensors\":[{\"stream_id\":\"thunder_01.front_depth\",\"state\":\"ACTIVE\",\"sample_count\":7,\"blocker\":\"\"},{\"stream_id\":\"thunder_01.front_rgb\",\"state\":\"ACTIVE\",\"sample_count\":8,\"blocker\":\"\"},{\"stream_id\":\"thunder_01.imu\",\"state\":\"ACTIVE\",\"sample_count\":9,\"blocker\":\"\"},{\"stream_id\":\"thunder_01.mid360\",\"state\":\"ACTIVE\",\"sample_count\":10,\"blocker\":\"\"},{\"stream_id\":\"thunder_01.truth_odom\",\"state\":\"ACTIVE\",\"sample_count\":11,\"blocker\":\"\"}],\"recording\":{\"state\":\"recording\",\"elapsed_sim_time_ns\":900000000,\"artifact_id\":\"\",\"blocker\":\"\"},\"ui\":{\"ui_mode\":\"tactical\",\"camera_mode\":\"free\"}}"),
            *Digest(TEXT('a')),
            static_cast<unsigned long long>(ServerStatusSequence),
            static_cast<unsigned long long>(SourceEpoch),
            static_cast<unsigned long long>(SourceSequence),
            static_cast<unsigned long long>(SourceEpoch),
            static_cast<unsigned long long>(SourceSequence),
            *DatagramSha256);
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimControlSha256KnownVectorTest,
    "LingTuSim.Session.ControlTransport.Sha256KnownVector",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuSimControlSha256KnownVectorTest::RunTest(const FString& Parameters)
{
    (void)Parameters;
    const TArray<uint8> Datagram = {0x61, 0x62, 0x63};
    FString Sha256;
    FString Error;
    TestTrue(
        TEXT("SHA-256 hashes a non-empty datagram"),
        LingTuSim::FControlTransportProtocol::Sha256Hex(
            Datagram,
            Sha256,
            Error));
    TestEqual(
        TEXT("SHA-256 matches the standard abc vector"),
        Sha256,
        FString(TEXT("ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad")));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimControlAllocationBindingTest,
    "LingTuSim.Session.ControlTransport.RunAllocationBinding",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuSimControlAllocationBindingTest::RunTest(const FString& Parameters)
{
    (void)Parameters;
    LingTuSim::FControlTransportBinding Parsed;
    FString Error;
    TestTrue(
        TEXT("exact allocation identity and ports bind"),
        LingTuSim::FControlTransportProtocol::ParseRunAllocationJson(
            AllocationJson(),
            TEXT("playable-run-001"),
            Digest(TEXT('a')),
            TEXT("C:/runs/playable-run-001/logs"),
            25124,
            25125,
            7,
            3,
            TEXT("robotsimue.local_player.0"),
            Parsed,
            Error));
    TestEqual(TEXT("intent port comes from allocation"), Parsed.IntentPort, uint16{25124});
    TestEqual(TEXT("status port comes from allocation"), Parsed.StatusPort, uint16{25125});
    TestEqual(TEXT("boot identity comes from allocation"), Parsed.BootId, FString{TEXT("boot-001")});
    TestTrue(
        TEXT("origin evidence stays in allocation logs"),
        Parsed.OriginEvidencePath.EndsWith(TEXT("/logs/ue-control-origin.jsonl")));

    TestFalse(
        TEXT("command-line port mismatch is rejected"),
        LingTuSim::FControlTransportProtocol::ParseRunAllocationJson(
            AllocationJson(),
            TEXT("playable-run-001"),
            Digest(TEXT('a')),
            TEXT("C:/runs/playable-run-001/logs"),
            25126,
            25125,
            7,
            3,
            TEXT("robotsimue.local_player.0"),
            Parsed,
            Error));
    TestFalse(
        TEXT("port collision with visual ingress is rejected"),
        LingTuSim::FControlTransportProtocol::ParseRunAllocationJson(
            AllocationJson(25123, 25125),
            TEXT("playable-run-001"),
            Digest(TEXT('a')),
            TEXT("C:/runs/playable-run-001/logs"),
            25123,
            25125,
            7,
            3,
            TEXT("robotsimue.local_player.0"),
            Parsed,
            Error));

    FString AllocationWithNestedString = AllocationJson();
    AllocationWithNestedString.ReplaceInline(
        TEXT("\"shm\":{}"),
        TEXT("\"shm\":{\"note\":\"nested } text\"}"));
    TestTrue(
        TEXT("braces inside nested allocation strings do not corrupt top-level fields"),
        LingTuSim::FControlTransportProtocol::ParseRunAllocationJson(
            AllocationWithNestedString,
            TEXT("playable-run-001"),
            Digest(TEXT('a')),
            TEXT("C:/runs/playable-run-001/logs"),
            25124,
            25125,
            7,
            3,
            TEXT("robotsimue.local_player.0"),
            Parsed,
            Error));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimControlIntentSerializationTest,
    "LingTuSim.Session.ControlTransport.IntentSerialization",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuSimControlIntentSerializationTest::RunTest(const FString& Parameters)
{
    (void)Parameters;
    LingTuSim::FOperatorIntentSample Sample;
    Sample.InputMode = TEXT("drive");
    Sample.InputDevice = TEXT("keyboard");
    Sample.bViewportFocused = true;
    Sample.bDeadman = true;
    Sample.Forward = 1.0;
    Sample.Left = -0.25;
    Sample.YawLeft = 0.5;
    Sample.CameraYaw = 0.75;
    Sample.CameraPitch = -0.5;
    Sample.ActiveControls = {TEXT("keyboard.left_shift"), TEXT("keyboard.w")};
    Sample.SourceMonotonicNs = 123456789;

    TArray<uint8> Datagram;
    FString EventId;
    FString Error;
    TestTrue(
        TEXT("operator intent serializes"),
        LingTuSim::FControlTransportProtocol::SerializeOperatorIntent(
            Binding(),
            Sample,
            1,
            4,
            Datagram,
            EventId,
            Error));
    const FString Json = DatagramJson(Datagram);
    TestEqual(TEXT("event ID is boot/epoch/sequence bound"), EventId, FString{TEXT("boot-001:1:4")});
    TestTrue(TEXT("wire schema is exact"), Json.Contains(TEXT("\"schema\":\"lingtu.sim.ue-control-intent.v1\"")));
    TestTrue(TEXT("robot axes are serialized"), Json.Contains(TEXT("\"forward\":1")));
    TestTrue(TEXT("actual UI mode is serialized"), Json.Contains(TEXT("\"ui_mode\":\"drive\"")));
    TestTrue(TEXT("degraded camera echo is explicit"), Json.Contains(TEXT("\"camera_mode\":\"unavailable\"")));
    TSharedPtr<FJsonObject> IntentRoot;
    const TSharedRef<TJsonReader<>> IntentReader = TJsonReaderFactory<>::Create(Json);
    TestTrue(TEXT("intent datagram JSON is parseable"), FJsonSerializer::Deserialize(IntentReader, IntentRoot));
    const TSharedPtr<FJsonObject>* Axes = nullptr;
    if (TestTrue(
            TEXT("intent datagram contains robot axes object"),
            IntentRoot.IsValid() && IntentRoot->TryGetObjectField(TEXT("axes"), Axes) && Axes != nullptr))
    {
        TestFalse(TEXT("camera yaw never enters robot intent axes"), (*Axes)->HasField(TEXT("camera_yaw")));
        TestFalse(TEXT("camera pitch never enters robot intent axes"), (*Axes)->HasField(TEXT("camera_pitch")));
    }
    TestFalse(TEXT("datagram hash is not self-referential"), Json.Contains(TEXT("datagram_sha256")));

    Sample.InputMode = TEXT("tactical");
    TestFalse(
        TEXT("motion intent v1 is drive-only"),
        LingTuSim::FControlTransportProtocol::SerializeOperatorIntent(
            Binding(),
            Sample,
            1,
            5,
            Datagram,
            EventId,
            Error));
    Sample.InputMode = TEXT("drive");
    Sample.InputDevice = TEXT("automation");
    TestFalse(
        TEXT("motion intent v1 accepts keyboard/gamepad only"),
        LingTuSim::FControlTransportProtocol::SerializeOperatorIntent(
            Binding(),
            Sample,
            1,
            6,
            Datagram,
            EventId,
            Error));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimControlOriginAfterSendTest,
    "LingTuSim.Session.ControlTransport.OriginEvidenceAfterSuccessfulSend",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuSimControlOriginAfterSendTest::RunTest(const FString& Parameters)
{
    (void)Parameters;
    int32 SendCalls = 0;
    int32 AppendCalls = 0;
    LingTuSim::FOperatorIntentSample Sample;
    Sample.bViewportFocused = true;
    Sample.bDeadman = true;
    Sample.SourceMonotonicNs = 123456789;

    LingTuSim::FLingTuSimOperatorIntentSender FailedSender(
        Binding(),
        [&SendCalls](const TArray<uint8>& Datagram, FString& Error)
        {
            ++SendCalls;
            Error = TEXT("synthetic send failure");
            return false;
        },
        [&AppendCalls](const FString& EvidenceJson, FString& Error)
        {
            ++AppendCalls;
            return true;
        });
    FString EventId;
    FString Error;
    TestFalse(
        TEXT("failed send fails publication"),
        FailedSender.PublishOperatorIntent(Sample, EventId, Error));
    TestEqual(TEXT("send is attempted once"), SendCalls, 1);
    TestEqual(TEXT("failed send writes no origin evidence"), AppendCalls, 0);
    LingTuSim::FControlAckEnvelope FailedSendAck;
    TestTrue(
        TEXT("ACK for a failed-send sequence is structurally valid"),
        LingTuSim::FControlTransportProtocol::ParseControlAckJson(
            Binding(),
            AckJson(7, 3, 1, 1, 1),
            FailedSendAck,
            Error));
    TestFalse(
        TEXT("ACK for a datagram that was never sent successfully is rejected"),
        FailedSender.ValidateAckAgainstSuccessfulSend(FailedSendAck, Error));

    TArray<uint8> EvidenceFailureDatagram;
    LingTuSim::FLingTuSimOperatorIntentSender EvidenceFailureSender(
        Binding(),
        [&EvidenceFailureDatagram](const TArray<uint8>& Datagram, FString& Error)
        {
            EvidenceFailureDatagram = Datagram;
            return true;
        },
        [](const FString& EvidenceJson, FString& Error)
        {
            Error = TEXT("synthetic durable append failure");
            return false;
        });
    TestFalse(
        TEXT("durable origin evidence failure rejects publication"),
        EvidenceFailureSender.PublishOperatorIntent(Sample, EventId, Error));
    FString EvidenceFailureHash;
    TestTrue(
        TEXT("evidence-failed sent bytes still have a test hash"),
        LingTuSim::FControlTransportProtocol::Sha256Hex(
            EvidenceFailureDatagram,
            EvidenceFailureHash,
            Error));
    LingTuSim::FControlAckEnvelope EvidenceFailureAck;
    TestTrue(
        TEXT("evidence-failed ACK remains structurally parseable"),
        LingTuSim::FControlTransportProtocol::ParseControlAckJson(
            Binding(),
            AckJson(7, 3, 1, 1, 1, EvidenceFailureHash),
            EvidenceFailureAck,
            Error));
    TestFalse(
        TEXT("ACK is rejected when durable origin evidence append failed"),
        EvidenceFailureSender.ValidateAckAgainstSuccessfulSend(
            EvidenceFailureAck,
            Error));
    LingTuSim::FControlStatusEnvelope EvidenceFailureStatus;
    TestTrue(
        TEXT("evidence-failed status remains structurally parseable"),
        LingTuSim::FControlTransportProtocol::ParseControlStatusJson(
            Binding(),
            StatusJson(EvidenceFailureHash, 1, 1, 1),
            EvidenceFailureStatus,
            Error));
    TestFalse(
        TEXT("status is rejected when durable origin evidence append failed"),
        EvidenceFailureSender.ValidateStatusAgainstSuccessfulSend(
            EvidenceFailureStatus,
            Error));

    TArray<uint8> SentDatagram;
    FString EvidenceLine;
    LingTuSim::FLingTuSimOperatorIntentSender SuccessfulSender(
        Binding(),
        [&SentDatagram](const TArray<uint8>& Datagram, FString& Error)
        {
            SentDatagram = Datagram;
            return true;
        },
        [&EvidenceLine](const FString& EvidenceJson, FString& Error)
        {
            EvidenceLine = EvidenceJson;
            return true;
        });
    TestTrue(
        TEXT("complete send records origin evidence"),
        SuccessfulSender.PublishOperatorIntent(Sample, EventId, Error));
    FString ExpectedHash;
    TestTrue(
        TEXT("sent raw bytes have a SHA-256"),
        LingTuSim::FControlTransportProtocol::Sha256Hex(
            SentDatagram,
            ExpectedHash,
            Error));
    TestTrue(TEXT("origin line carries sent-byte hash"), EvidenceLine.Contains(ExpectedHash));
    TestTrue(TEXT("origin line carries event correlation"), EvidenceLine.Contains(EventId));
    TestTrue(
        TEXT("origin line explicitly records complete-send success"),
        EvidenceLine.Contains(TEXT("\"successful_send\":true")));
    TestFalse(TEXT("origin evidence has no synthetic maneuver label"), EvidenceLine.Contains(TEXT("maneuver")));

    LingTuSim::FControlAckEnvelope ParsedAck;
    TestTrue(
        TEXT("matching sent-datagram ACK parses"),
        LingTuSim::FControlTransportProtocol::ParseControlAckJson(
            Binding(),
            AckJson(7, 3, 1, 1, 1, ExpectedHash),
            ParsedAck,
            Error));
    TestTrue(
        TEXT("ACK for the exact successfully sent datagram is admitted"),
        SuccessfulSender.ValidateAckAgainstSuccessfulSend(ParsedAck, Error));

    TestTrue(
        TEXT("spoofed-hash ACK parses before sender correlation"),
        LingTuSim::FControlTransportProtocol::ParseControlAckJson(
            Binding(),
            AckJson(7, 3, 2, 1, 1, Digest(TEXT('c'))),
            ParsedAck,
            Error));
    TestFalse(
        TEXT("spoofed-hash ACK is rejected by successful-send correlation"),
        SuccessfulSender.ValidateAckAgainstSuccessfulSend(ParsedAck, Error));

    TestTrue(
        TEXT("future-sequence ACK parses before sender correlation"),
        LingTuSim::FControlTransportProtocol::ParseControlAckJson(
            Binding(),
            AckJson(7, 3, 3, 1, 2, ExpectedHash),
            ParsedAck,
            Error));
    TestFalse(
        TEXT("future-sequence ACK without a successful send is rejected"),
        SuccessfulSender.ValidateAckAgainstSuccessfulSend(ParsedAck, Error));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimControlCommandLineShapeTest,
    "LingTuSim.Session.ControlTransport.CommandLineShape",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuSimControlCommandLineShapeTest::RunTest(const FString& Parameters)
{
    (void)Parameters;
    bool bControlRequested = true;
    FString Error;
    TestTrue(
        TEXT("viewer command line has no control request"),
        LingTuSim::FControlTransportProtocol::ValidateControlCommandLineShape(
            TEXT("-game"),
            bControlRequested,
            Error));
    TestFalse(TEXT("viewer remains disabled"), bControlRequested);

    TestFalse(
        TEXT("orphan source ID cannot silently become viewer mode"),
        LingTuSim::FControlTransportProtocol::ValidateControlCommandLineShape(
            TEXT("-LingTuControlSourceId=robotsimue.local_player.0"),
            bControlRequested,
            Error));
    TestFalse(
        TEXT("legacy HUD evidence path cannot silently become viewer mode"),
        LingTuSim::FControlTransportProtocol::ValidateControlCommandLineShape(
            TEXT("-LingTuHudScreenshot=C:/run/logs/hud.png"),
            bControlRequested,
            Error));
    TestFalse(
        TEXT("intent port without status port cannot silently become viewer mode"),
        LingTuSim::FControlTransportProtocol::ValidateControlCommandLineShape(
            TEXT("-LingTuControlIntentPort=25124"),
            bControlRequested,
            Error));
    TestFalse(
        TEXT("status port without intent port cannot silently become viewer mode"),
        LingTuSim::FControlTransportProtocol::ValidateControlCommandLineShape(
            TEXT("-LingTuControlStatusPort=25125"),
            bControlRequested,
            Error));
    TestTrue(
        TEXT("one exact intent/status/source/three-screenshot shape is admitted"),
        LingTuSim::FControlTransportProtocol::ValidateControlCommandLineShape(
            TEXT("-LingTuControlIntentPort=25124 -LingTuControlStatusPort=25125 -LingTuControlSourceId=robotsimue.local_player.0 -LingTuHudDriveScreenshot=C:/run/screenshots/hud-drive.png -LingTuHudTacticalScreenshot=C:/run/screenshots/hud-tactical.png -LingTuHudMenuRecordingScreenshot=C:/run/screenshots/hud-menu-recording.png"),
            bControlRequested,
            Error));
    TestTrue(TEXT("valid control shape requests transport"), bControlRequested);

    TestFalse(
        TEXT("partial three-screenshot shape is rejected"),
        LingTuSim::FControlTransportProtocol::ValidateControlCommandLineShape(
            TEXT("-LingTuControlIntentPort=25124 -LingTuControlStatusPort=25125 -LingTuControlSourceId=robotsimue.local_player.0 -LingTuHudDriveScreenshot=C:/run/screenshots/hud-drive.png"),
            bControlRequested,
            Error));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimControlAckStrictTest,
    "LingTuSim.Session.ControlTransport.AckStrictIdentity",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuSimControlAckStrictTest::RunTest(const FString& Parameters)
{
    (void)Parameters;
    LingTuSim::FControlAckEnvelope Ack;
    FString Error;
    TestTrue(
        TEXT("minimal exact ACK parses"),
        LingTuSim::FControlTransportProtocol::ParseControlAckJson(
            Binding(),
            AckJson(),
            Ack,
            Error));
    TestEqual(TEXT("ACK sequence is retained"), Ack.ServerStatusSequence, uint64{9});
    TestEqual(TEXT("ACK source event is retained"), Ack.EventId, FString{TEXT("boot-001:1:4")});

    TestFalse(
        TEXT("wrong model generation is rejected"),
        LingTuSim::FControlTransportProtocol::ParseControlAckJson(
            Binding(),
            AckJson(8, 3),
            Ack,
            Error));
    TestFalse(
        TEXT("wrong reset generation is rejected"),
        LingTuSim::FControlTransportProtocol::ParseControlAckJson(
            Binding(),
            AckJson(7, 4),
            Ack,
            Error));
    TestFalse(
        TEXT("server status sequence must be positive"),
        LingTuSim::FControlTransportProtocol::ParseControlAckJson(
            Binding(),
            AckJson(7, 3, 0),
            Ack,
            Error));

    FString RejectedWithoutReason = AckJson();
    RejectedWithoutReason.ReplaceInline(
        TEXT("\"status\":\"accepted\""),
        TEXT("\"status\":\"rejected\""));
    TestFalse(
        TEXT("negative ACK state requires a reason"),
        LingTuSim::FControlTransportProtocol::ParseControlAckJson(
            Binding(),
            RejectedWithoutReason,
            Ack,
            Error));

    FString UnknownField = AckJson();
    UnknownField.RemoveFromEnd(TEXT("}"));
    UnknownField += TEXT(",\"optimistic_motion\":true}");
    TestFalse(
        TEXT("unknown ACK fields fail closed"),
        LingTuSim::FControlTransportProtocol::ParseControlAckJson(
            Binding(),
            UnknownField,
            Ack,
            Error));

    FString DuplicateField = AckJson();
    DuplicateField.RemoveFromEnd(TEXT("}"));
    DuplicateField += TEXT(",\"status\":\"accepted\"}");
    TestFalse(
        TEXT("duplicate ACK fields fail closed"),
        LingTuSim::FControlTransportProtocol::ParseControlAckJson(
            Binding(),
            DuplicateField,
            Ack,
            Error));

    const FString EncodedStructuredReason =
        TEXT("controller said {\\\"status\\\":\\\"held\\\"}");
    const FString DecodedStructuredReason =
        TEXT("controller said {\"status\":\"held\"}");
    TestTrue(
        TEXT("braces and escaped quotes inside a reason string remain legal"),
        LingTuSim::FControlTransportProtocol::ParseControlAckJson(
            Binding(),
            AckJson(
                7,
                3,
                10,
                1,
                4,
                Digest(TEXT('b')),
                TEXT("rejected"),
                EncodedStructuredReason),
            Ack,
            Error));
    TestEqual(TEXT("structured reason is decoded"), Ack.Reason, DecodedStructuredReason);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimControlStatusStrictTest,
    "LingTuSim.Session.ControlTransport.FullStatusStrict",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuSimControlStatusStrictTest::RunTest(const FString& Parameters)
{
    (void)Parameters;
    LingTuSim::FControlStatusEnvelope Status;
    FString Error;
    TestTrue(
        TEXT("exact full status parses"),
        LingTuSim::FControlTransportProtocol::ParseControlStatusJson(
            Binding(),
            StatusJson(),
            Status,
            Error));
    TestEqual(TEXT("truth sequence is retained"), Status.TruthSequence, uint64{600});
    TestEqual(TEXT("exact five sensors are retained"), Status.Sensors.Num(), 5);
    TestEqual(
        TEXT("canonical sensor order is retained"),
        Status.Sensors[0].StreamId,
        FString(TEXT("thunder_01.front_depth")));
    TestTrue(TEXT("requested axes stay separate"), Status.Motion.RequestedAxes.bAvailable);
    TestTrue(TEXT("admitted twist stays separate"), Status.Motion.AdmittedTwist.bAvailable);
    TestTrue(TEXT("observed truth stays separate"), Status.Motion.ObservedBaseVelocity.bAvailable);
    TestTrue(
        TEXT("recording state is authoritative"),
        Status.Recording.State == LingTuSim::EControlRecordingState::Recording);
    TestTrue(
        TEXT("UE mode echo is retained"),
        Status.UI.UIMode == LingTuSim::EControlStatusUIMode::Tactical);

    FString UnknownNested = StatusJson();
    UnknownNested.ReplaceInline(
        TEXT("\"camera_mode\":\"free\"}"),
        TEXT("\"camera_mode\":\"free\",\"optimistic\":true}"));
    TestFalse(
        TEXT("unknown nested fields fail closed"),
        LingTuSim::FControlTransportProtocol::ParseControlStatusJson(
            Binding(), UnknownNested, Status, Error));

    FString DuplicateNested = StatusJson();
    DuplicateNested.ReplaceInline(
        TEXT("\"ui_mode\":\"tactical\",\"camera_mode\":\"free\""),
        TEXT("\"ui_mode\":\"tactical\",\"ui_mode\":\"tactical\",\"camera_mode\":\"free\""));
    TestFalse(
        TEXT("duplicate nested fields fail closed"),
        LingTuSim::FControlTransportProtocol::ParseControlStatusJson(
            Binding(), DuplicateNested, Status, Error));

    FString WrongSensorOrder = StatusJson();
    WrongSensorOrder.ReplaceInline(
        TEXT("\"stream_id\":\"thunder_01.front_depth\""),
        TEXT("\"stream_id\":\"thunder_01.front_rgb\""),
        ESearchCase::CaseSensitive);
    TestFalse(
        TEXT("non-canonical sensor order fails closed"),
        LingTuSim::FControlTransportProtocol::ParseControlStatusJson(
            Binding(), WrongSensorOrder, Status, Error));

    FString FabricatedUnavailable = StatusJson();
    FabricatedUnavailable.ReplaceInline(
        TEXT("\"observed_base_velocity_mps_radps\":{\"available\":true,\"linear_x\":0.08"),
        TEXT("\"observed_base_velocity_mps_radps\":{\"available\":false,\"linear_x\":0.08"));
    TestFalse(
        TEXT("unavailable observed motion cannot carry invented values"),
        LingTuSim::FControlTransportProtocol::ParseControlStatusJson(
            Binding(), FabricatedUnavailable, Status, Error));

    TestFalse(TEXT("parsed status is not fresh before service receive"), Status.IsFresh(100, 50));
    Status.ReceivedMonotonicNs = 100;
    TestTrue(TEXT("received status is fresh within the explicit bound"), Status.IsFresh(150, 50));
    TestFalse(TEXT("received status expires outside the explicit bound"), Status.IsFresh(151, 50));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuSimControlStatusLedgerAndServiceTest,
    "LingTuSim.Session.ControlTransport.FullStatusLedgerAndService",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuSimControlStatusLedgerAndServiceTest::RunTest(const FString& Parameters)
{
    (void)Parameters;
    TArray<uint8> SentDatagram;
    const TSharedRef<LingTuSim::FLingTuSimOperatorIntentSender, ESPMode::ThreadSafe> Sender =
        MakeShared<LingTuSim::FLingTuSimOperatorIntentSender, ESPMode::ThreadSafe>(
            Binding(),
            [&SentDatagram](const TArray<uint8>& Datagram, FString& Error)
            {
                SentDatagram = Datagram;
                return true;
            },
            [](const FString& EvidenceJson, FString& Error)
            {
                return true;
            });
    LingTuSim::FOperatorIntentSample Sample;
    Sample.bViewportFocused = true;
    Sample.bDeadman = true;
    Sample.SourceMonotonicNs = 123456789;
    FString EventId;
    FString Error;
    TestTrue(
        TEXT("one local intent is successfully sent"),
        Sender->PublishOperatorIntent(Sample, EventId, Error));
    FString Hash;
    TestTrue(
        TEXT("sent intent bytes are hashed"),
        LingTuSim::FControlTransportProtocol::Sha256Hex(
            SentDatagram, Hash, Error));

    LingTuSim::FControlStatusEnvelope Status;
    TestTrue(
        TEXT("matching status parses"),
        LingTuSim::FControlTransportProtocol::ParseControlStatusJson(
            Binding(), StatusJson(Hash, 1, 1, 1), Status, Error));
    TestTrue(
        TEXT("matching status names a successful local send"),
        Sender->ValidateStatusAgainstSuccessfulSend(Status, Error));
    TestTrue(
        TEXT("spoofed status parses before ledger validation"),
        LingTuSim::FControlTransportProtocol::ParseControlStatusJson(
            Binding(), StatusJson(Digest(TEXT('c')), 2, 1, 1), Status, Error));
    TestFalse(
        TEXT("spoofed status hash is rejected by the sender ledger"),
        Sender->ValidateStatusAgainstSuccessfulSend(Status, Error));

    LingTuSim::FSessionService::UnbindSession();
    TestTrue(
        TEXT("test session binding is installed"),
        LingTuSim::FSessionService::RebindSession(Digest(TEXT('a')), 7));
    TestTrue(
        TEXT("test control sender is bound"),
        LingTuSim::FSessionService::BindControlTransport(Sender));
    TestTrue(
        TEXT("same-port schema switch accepts correlated full status"),
        LingTuSim::FSessionService::PublishControlResponseJson(
            StatusJson(Hash, 3, 1, 1), Error));
    TestTrue(
        TEXT("service exposes the latest validated full status"),
        LingTuSim::FSessionService::GetLatestControlStatus(Status));
    TestTrue(TEXT("service receive stamps freshness time"), Status.ReceivedMonotonicNs > 0);
    TestFalse(
        TEXT("unknown response schema fails closed"),
        LingTuSim::FSessionService::PublishControlResponseJson(
            TEXT("{\"schema\":\"lingtu.sim.unknown.v1\"}"), Error));
    LingTuSim::FSessionService::UnbindSession();
    return true;
}

#endif

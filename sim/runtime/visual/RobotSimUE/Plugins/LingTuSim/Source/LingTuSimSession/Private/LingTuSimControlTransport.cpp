#include "LingTuSimControlTransport.h"

#include "Misc/Paths.h"
#include "Misc/ScopeLock.h"
#include "PlatformCryptoContextIncludes.h"
#include "Policies/CondensedJsonPrintPolicy.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"

namespace LingTuSim
{
    namespace
    {
        constexpr int32 MaxControlDatagramBytes = 4'096;
        constexpr int32 MaxTrackedSuccessfulSends = 256;
        constexpr double MaxExactJsonInteger = 9'007'199'254'740'991.0;

        bool IsLowerHex(const FString& Value, const int32 ExpectedLength)
        {
            if (Value.Len() != ExpectedLength)
            {
                return false;
            }
            for (const TCHAR Character : Value)
            {
                if (!((Character >= TEXT('0') && Character <= TEXT('9'))
                        || (Character >= TEXT('a') && Character <= TEXT('f'))))
                {
                    return false;
                }
            }
            return true;
        }

        bool IsCanonicalId(const FString& Value, const int32 MaxLength = 128)
        {
            if (Value.IsEmpty() || Value.Len() > MaxLength)
            {
                return false;
            }
            for (int32 Index = 0; Index < Value.Len(); ++Index)
            {
                const TCHAR Character = Value[Index];
                const bool bAlphaNumeric =
                    (Character >= TEXT('a') && Character <= TEXT('z'))
                    || (Character >= TEXT('A') && Character <= TEXT('Z'))
                    || (Character >= TEXT('0') && Character <= TEXT('9'));
                if (!bAlphaNumeric
                    && (Index == 0
                        || (Character != TEXT('_')
                            && Character != TEXT('-')
                            && Character != TEXT('.'))))
                {
                    return false;
                }
            }
            return true;
        }

        bool IsSafeControlName(const FString& Value)
        {
            if (Value.IsEmpty() || Value.Len() > 64)
            {
                return false;
            }
            for (int32 Index = 0; Index < Value.Len(); ++Index)
            {
                const TCHAR Character = Value[Index];
                const bool bLowerAlphaNumeric =
                    (Character >= TEXT('a') && Character <= TEXT('z'))
                    || (Character >= TEXT('0') && Character <= TEXT('9'));
                if (!bLowerAlphaNumeric
                    && (Index == 0
                        || (Character != TEXT('_')
                            && Character != TEXT('-')
                            && Character != TEXT('.'))))
                {
                    return false;
                }
            }
            return true;
        }

        bool IsWireUIMode(const FString& Value)
        {
            return Value == TEXT("drive")
                || Value == TEXT("build")
                || Value == TEXT("tactical")
                || Value == TEXT("menu");
        }

        bool IsWireCameraMode(const FString& Value)
        {
            return Value == TEXT("unavailable")
                || Value == TEXT("follow")
                || Value == TEXT("inspection")
                || Value == TEXT("free");
        }

        bool ReadExactUint64(
            const TSharedPtr<FJsonObject>& Object,
            const TCHAR* Field,
            uint64& OutValue,
            FString& OutError)
        {
            double Number = 0.0;
            if (!Object.IsValid()
                || !Object->TryGetNumberField(Field, Number)
                || !FMath::IsFinite(Number)
                || Number < 0.0
                || Number > MaxExactJsonInteger
                || FMath::FloorToDouble(Number) != Number)
            {
                OutError = FString::Printf(
                    TEXT("%s must be a non-negative exact JSON integer"),
                    Field);
                return false;
            }
            OutValue = static_cast<uint64>(Number);
            return true;
        }

        bool ReadPort(
            const TSharedPtr<FJsonObject>& Ports,
            const TCHAR* Field,
            uint16& OutPort,
            FString& OutError)
        {
            uint64 Value = 0;
            if (!ReadExactUint64(Ports, Field, Value, OutError)
                || Value == 0
                || Value > MAX_uint16)
            {
                OutError = FString::Printf(
                    TEXT("RunAllocation ports.%s must be in [1, 65535]"),
                    Field);
                return false;
            }
            OutPort = static_cast<uint16>(Value);
            return true;
        }

        bool HasExactTopLevelFields(
            const FString& Json,
            const TSet<FString>& Expected,
            FString& OutError)
        {
            TSet<FString> Found;
            int32 Depth = 0;
            int32 StringDepth = 0;
            int32 StringStart = INDEX_NONE;
            bool bInString = false;
            bool bEscaped = false;
            for (int32 Index = 0; Index < Json.Len(); ++Index)
            {
                const TCHAR Character = Json[Index];
                if (bInString)
                {
                    if (bEscaped)
                    {
                        bEscaped = false;
                        continue;
                    }
                    if (Character == TEXT('\\'))
                    {
                        bEscaped = true;
                        continue;
                    }
                    if (Character != TEXT('"'))
                    {
                        continue;
                    }

                    bInString = false;
                    if (StringDepth != 1)
                    {
                        continue;
                    }
                    const FString Candidate = Json.Mid(
                        StringStart,
                        Index - StringStart);
                    int32 After = Index + 1;
                    while (After < Json.Len() && FChar::IsWhitespace(Json[After]))
                    {
                        ++After;
                    }
                    if (After < Json.Len() && Json[After] == TEXT(':'))
                    {
                        if (Found.Contains(Candidate))
                        {
                            OutError = FString::Printf(
                                TEXT("duplicate top-level JSON field '%s'"),
                                *Candidate);
                            return false;
                        }
                        Found.Add(Candidate);
                    }
                    continue;
                }

                if (Character == TEXT('"'))
                {
                    bInString = true;
                    bEscaped = false;
                    StringDepth = Depth;
                    StringStart = Index + 1;
                    continue;
                }
                if (Character == TEXT('{'))
                {
                    ++Depth;
                    continue;
                }
                if (Character == TEXT('}'))
                {
                    --Depth;
                    if (Depth < 0)
                    {
                        OutError = TEXT("JSON object depth is invalid");
                        return false;
                    }
                }
            }
            if (bInString)
            {
                OutError = TEXT("JSON contains an unterminated string");
                return false;
            }

            if (Found.Num() != Expected.Num())
            {
                OutError = TEXT("JSON must contain exactly the canonical top-level fields");
                return false;
            }
            for (const FString& Field : Expected)
            {
                if (!Found.Contains(Field))
                {
                    OutError = FString::Printf(
                        TEXT("JSON is missing required top-level field '%s'"),
                        *Field);
                    return false;
                }
            }
            return true;
        }

        bool SerializeObject(
            const TSharedRef<FJsonObject>& Root,
            FString& OutJson,
            FString& OutError)
        {
            OutJson.Reset();
            const TSharedRef<TJsonWriter<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>> Writer =
                TJsonWriterFactory<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>::Create(&OutJson);
            if (!FJsonSerializer::Serialize(Root, Writer))
            {
                OutError = TEXT("cannot serialize control JSON");
                return false;
            }
            return true;
        }

        bool ToDatagram(
            const FString& Json,
            TArray<uint8>& OutDatagram,
            FString& OutError)
        {
            const FTCHARToUTF8 Utf8(*Json);
            if (Utf8.Length() <= 0 || Utf8.Length() > MaxControlDatagramBytes)
            {
                OutError = FString::Printf(
                    TEXT("control datagram must contain 1..%d UTF-8 bytes"),
                    MaxControlDatagramBytes);
                return false;
            }
            OutDatagram.SetNumUninitialized(Utf8.Length());
            FMemory::Memcpy(OutDatagram.GetData(), Utf8.Get(), Utf8.Length());
            return true;
        }

        FString RuntimeRequestName(const EOperatorRuntimeRequestType Request)
        {
            switch (Request)
            {
            case EOperatorRuntimeRequestType::ControlClaim:
                return TEXT("control_claim");
            case EOperatorRuntimeRequestType::ControlRelease:
                return TEXT("control_release");
            case EOperatorRuntimeRequestType::Pause:
                return TEXT("pause");
            case EOperatorRuntimeRequestType::Resume:
                return TEXT("resume");
            case EOperatorRuntimeRequestType::RecordStart:
                return TEXT("record_start");
            case EOperatorRuntimeRequestType::RecordStopCommit:
                return TEXT("record_stop_commit");
            case EOperatorRuntimeRequestType::SafeStop:
                return TEXT("safe_stop");
            case EOperatorRuntimeRequestType::Exit:
                return TEXT("exit");
            case EOperatorRuntimeRequestType::UIStateUpdate:
                return TEXT("ui_state_update");
            default:
                return FString{};
            }
        }

        bool ParseAckStatus(
            const FString& Value,
            EControlAckStatus& OutStatus)
        {
            if (Value == TEXT("pending"))
            {
                OutStatus = EControlAckStatus::Pending;
            }
            else if (Value == TEXT("accepted"))
            {
                OutStatus = EControlAckStatus::Accepted;
            }
            else if (Value == TEXT("rejected"))
            {
                OutStatus = EControlAckStatus::Rejected;
            }
            else if (Value == TEXT("released"))
            {
                OutStatus = EControlAckStatus::Released;
            }
            else if (Value == TEXT("timeout_zero"))
            {
                OutStatus = EControlAckStatus::TimeoutZero;
            }
            else if (Value == TEXT("confirmed"))
            {
                OutStatus = EControlAckStatus::Confirmed;
            }
            else
            {
                return false;
            }
            return true;
        }

        FString MakeEventId(
            const FControlTransportBinding& Binding,
            const uint64 SourceEpoch,
            const uint64 SourceSequence)
        {
            return FString::Printf(
                TEXT("%s:%llu:%llu"),
                *Binding.BootId,
                static_cast<unsigned long long>(SourceEpoch),
                static_cast<unsigned long long>(SourceSequence));
        }

        void AddCommonIdentity(
            const TSharedRef<FJsonObject>& Root,
            const TCHAR* Schema,
            const FControlTransportBinding& Binding,
            const uint64 SourceEpoch,
            const uint64 SourceSequence,
            const FString& EventId)
        {
            Root->SetStringField(TEXT("schema"), Schema);
            Root->SetStringField(TEXT("run_id"), Binding.RunId);
            Root->SetStringField(TEXT("session_id"), Binding.SessionId);
            Root->SetStringField(TEXT("boot_id"), Binding.BootId);
            Root->SetNumberField(
                TEXT("model_generation"),
                static_cast<double>(Binding.ModelGeneration));
            Root->SetNumberField(
                TEXT("reset_generation"),
                static_cast<double>(Binding.ResetGeneration));
            Root->SetStringField(TEXT("source_id"), Binding.SourceId);
            Root->SetNumberField(TEXT("source_epoch"), static_cast<double>(SourceEpoch));
            Root->SetNumberField(TEXT("source_sequence"), static_cast<double>(SourceSequence));
            Root->SetStringField(TEXT("event_id"), EventId);
        }
    }

    bool FControlTransportBinding::IsValid() const
    {
        const FString ExpectedOriginEvidencePath = FPaths::Combine(
            LogDirectory,
            TEXT("ue-control-origin.jsonl"));
        return IsCanonicalId(RunId)
            && IsCanonicalId(SessionId, 63)
            && IsCanonicalId(BootId)
            && SourceId == TEXT("robotsimue.local_player.0")
            && !FPaths::IsRelative(LogDirectory)
            && !OriginEvidencePath.IsEmpty()
            && !FPaths::IsRelative(OriginEvidencePath)
            && FPaths::IsSamePath(OriginEvidencePath, ExpectedOriginEvidencePath)
            && ModelGeneration <= static_cast<uint64>(MaxExactJsonInteger)
            && ResetGeneration <= static_cast<uint64>(MaxExactJsonInteger)
            && IntentPort > 0
            && StatusPort > 0
            && IntentPort != StatusPort;
    }

    bool FControlTransportProtocol::ParseRunAllocationJson(
        const FString& AllocationJson,
        const FString& ExpectedRunId,
        const FString& ExpectedSessionId,
        const FString& ExpectedLogDirectory,
        const uint16 ExpectedIntentPort,
        const uint16 ExpectedStatusPort,
        const uint64 ModelGeneration,
        const uint64 ResetGeneration,
        const FString& SourceId,
        FControlTransportBinding& OutBinding,
        FString& OutError)
    {
        OutBinding = FControlTransportBinding{};
        OutError.Reset();
        static const TSet<FString> AllocationFields = {
            TEXT("schema"),
            TEXT("run_id"),
            TEXT("session_id"),
            TEXT("artifact_root"),
            TEXT("boot_id"),
            TEXT("dds_domain"),
            TEXT("ports"),
            TEXT("shm"),
            TEXT("log_dir"),
        };
        if (!HasExactTopLevelFields(AllocationJson, AllocationFields, OutError))
        {
            return false;
        }

        TSharedPtr<FJsonObject> Root;
        const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(AllocationJson);
        if (!FJsonSerializer::Deserialize(Reader, Root) || !Root.IsValid())
        {
            OutError = TEXT("RunAllocation is not a valid JSON object");
            return false;
        }

        FString Schema;
        FString RunId;
        FString SessionId;
        FString BootId;
        FString LogDirectory;
        const TSharedPtr<FJsonObject>* Ports = nullptr;
        if (!Root->TryGetStringField(TEXT("schema"), Schema)
            || Schema != TEXT("lingtu.sim.run-allocation.v1")
            || !Root->TryGetStringField(TEXT("run_id"), RunId)
            || !Root->TryGetStringField(TEXT("session_id"), SessionId)
            || !Root->TryGetStringField(TEXT("boot_id"), BootId)
            || !Root->TryGetStringField(TEXT("log_dir"), LogDirectory)
            || !Root->TryGetObjectField(TEXT("ports"), Ports)
            || Ports == nullptr
            || !Ports->IsValid())
        {
            OutError = TEXT("RunAllocation identity or ports are invalid");
            return false;
        }
        if (!IsCanonicalId(RunId) || RunId != ExpectedRunId)
        {
            OutError = TEXT("RunAllocation run_id does not match LingTuRunId");
            return false;
        }
        if (!IsCanonicalId(SessionId, 63)
            || SessionId != ExpectedSessionId)
        {
            OutError = TEXT("RunAllocation session_id does not match SessionBundle");
            return false;
        }
        if (!IsCanonicalId(BootId))
        {
            OutError = TEXT("RunAllocation boot_id is not canonical");
            return false;
        }
        if (!IsCanonicalId(SourceId)
            || SourceId != TEXT("robotsimue.local_player.0"))
        {
            OutError = TEXT("LingTuControlSourceId is not the first-slice source ID");
            return false;
        }
        if (FPaths::IsRelative(LogDirectory)
            || !FPaths::IsSamePath(LogDirectory, ExpectedLogDirectory))
        {
            OutError = TEXT("RunAllocation log_dir does not match the allocation logs directory");
            return false;
        }

        uint16 IntentPort = 0;
        uint16 StatusPort = 0;
        if (!ReadPort(*Ports, TEXT("control_intent_udp"), IntentPort, OutError)
            || !ReadPort(*Ports, TEXT("control_status_udp"), StatusPort, OutError))
        {
            return false;
        }
        if (IntentPort != ExpectedIntentPort || StatusPort != ExpectedStatusPort)
        {
            OutError = TEXT("command-line control ports do not match RunAllocation");
            return false;
        }
        if (IntentPort == StatusPort)
        {
            OutError = TEXT("control intent and status ports must be distinct");
            return false;
        }
        if ((*Ports)->HasField(TEXT("visual_snapshot_udp")))
        {
            uint16 VisualSnapshotPort = 0;
            if (!ReadPort(
                    *Ports,
                    TEXT("visual_snapshot_udp"),
                    VisualSnapshotPort,
                    OutError))
            {
                return false;
            }
            if (VisualSnapshotPort == IntentPort || VisualSnapshotPort == StatusPort)
            {
                OutError = TEXT("visual snapshot and control ports must be distinct");
                return false;
            }
        }

        const FString OriginEvidencePath = FPaths::Combine(
            ExpectedLogDirectory,
            TEXT("ue-control-origin.jsonl"));
        OutBinding.RunId = RunId;
        OutBinding.SessionId = SessionId;
        OutBinding.BootId = BootId;
        OutBinding.SourceId = SourceId;
        OutBinding.LogDirectory = ExpectedLogDirectory;
        OutBinding.OriginEvidencePath = OriginEvidencePath;
        OutBinding.ModelGeneration = ModelGeneration;
        OutBinding.ResetGeneration = ResetGeneration;
        OutBinding.IntentPort = IntentPort;
        OutBinding.StatusPort = StatusPort;
        if (!OutBinding.IsValid())
        {
            OutError = TEXT("control transport binding is incomplete");
            OutBinding = FControlTransportBinding{};
            return false;
        }
        return true;
    }

    bool FControlTransportProtocol::SerializeOperatorIntent(
        const FControlTransportBinding& Binding,
        const FOperatorIntentSample& Sample,
        const uint64 SourceEpoch,
        const uint64 SourceSequence,
        TArray<uint8>& OutDatagram,
        FString& OutEventId,
        FString& OutError)
    {
        OutDatagram.Reset();
        OutEventId.Reset();
        OutError.Reset();
        if (!Binding.IsValid()
            || SourceEpoch == 0
            || SourceSequence == 0
            || SourceEpoch > static_cast<uint64>(MaxExactJsonInteger)
            || SourceSequence > static_cast<uint64>(MaxExactJsonInteger))
        {
            OutError = TEXT("control transport identity/sequence is invalid");
            return false;
        }
        if (Sample.InputMode != TEXT("drive")
            || Sample.UIMode != TEXT("drive")
            || !IsWireCameraMode(Sample.CameraMode)
            || (Sample.InputDevice != TEXT("keyboard")
                && Sample.InputDevice != TEXT("gamepad")))
        {
            OutError = TEXT("operator intent input mode/device is unsupported");
            return false;
        }
        if (!FMath::IsFinite(Sample.Forward)
            || !FMath::IsFinite(Sample.Left)
            || !FMath::IsFinite(Sample.YawLeft)
            || !FMath::IsFinite(Sample.CameraYaw)
            || !FMath::IsFinite(Sample.CameraPitch)
            || FMath::Abs(Sample.Forward) > 1.0
            || FMath::Abs(Sample.Left) > 1.0
            || FMath::Abs(Sample.YawLeft) > 1.0
            || FMath::Abs(Sample.CameraYaw) > 1.0
            || FMath::Abs(Sample.CameraPitch) > 1.0)
        {
            OutError = TEXT("operator intent axes must be finite values in [-1, 1]");
            return false;
        }
        if (Sample.SourceMonotonicNs == 0
            || Sample.SourceMonotonicNs > static_cast<uint64>(MaxExactJsonInteger)
            || Sample.ActiveControls.Num() > 16)
        {
            OutError = TEXT("operator intent timestamp/control count is invalid");
            return false;
        }
        TSet<FString> UniqueControls;
        TArray<TSharedPtr<FJsonValue>> Controls;
        for (const FString& Control : Sample.ActiveControls)
        {
            if (!IsSafeControlName(Control) || UniqueControls.Contains(Control))
            {
                OutError = TEXT("operator intent active controls must be unique canonical names");
                return false;
            }
            UniqueControls.Add(Control);
            Controls.Add(MakeShared<FJsonValueString>(Control));
        }

        OutEventId = MakeEventId(Binding, SourceEpoch, SourceSequence);
        const TSharedRef<FJsonObject> Root = MakeShared<FJsonObject>();
        AddCommonIdentity(
            Root,
            TEXT("lingtu.sim.ue-control-intent.v1"),
            Binding,
            SourceEpoch,
            SourceSequence,
            OutEventId);
        Root->SetStringField(TEXT("input_mode"), Sample.InputMode);
        Root->SetStringField(TEXT("ui_mode"), Sample.UIMode);
        Root->SetStringField(TEXT("camera_mode"), Sample.CameraMode);
        Root->SetStringField(TEXT("input_device"), Sample.InputDevice);
        Root->SetBoolField(TEXT("viewport_focused"), Sample.bViewportFocused);
        Root->SetBoolField(TEXT("deadman"), Sample.bDeadman);
        const TSharedRef<FJsonObject> Axes = MakeShared<FJsonObject>();
        Axes->SetNumberField(TEXT("forward"), Sample.Forward);
        Axes->SetNumberField(TEXT("left"), Sample.Left);
        Axes->SetNumberField(TEXT("yaw_left"), Sample.YawLeft);
        Root->SetObjectField(TEXT("axes"), Axes);
        Root->SetArrayField(TEXT("active_controls"), MoveTemp(Controls));
        Root->SetNumberField(
            TEXT("source_monotonic_ns"),
            static_cast<double>(Sample.SourceMonotonicNs));

        FString Json;
        return SerializeObject(Root, Json, OutError)
            && ToDatagram(Json, OutDatagram, OutError);
    }

    bool FControlTransportProtocol::SerializeRuntimeRequest(
        const FControlTransportBinding& Binding,
        const FOperatorRuntimeRequest& Request,
        const uint64 SourceEpoch,
        const uint64 SourceSequence,
        TArray<uint8>& OutDatagram,
        FString& OutEventId,
        FString& OutError)
    {
        OutDatagram.Reset();
        OutEventId.Reset();
        OutError.Reset();
        const FString RequestName = RuntimeRequestName(Request.Request);
        if (!Binding.IsValid()
            || RequestName.IsEmpty()
            || !IsWireUIMode(Request.UIMode)
            || !IsWireCameraMode(Request.CameraMode)
            || SourceEpoch == 0
            || SourceSequence == 0
            || Request.SourceMonotonicNs == 0
            || SourceEpoch > static_cast<uint64>(MaxExactJsonInteger)
            || SourceSequence > static_cast<uint64>(MaxExactJsonInteger)
            || Request.SourceMonotonicNs > static_cast<uint64>(MaxExactJsonInteger))
        {
            OutError = TEXT("runtime request identity, sequence, or timestamp is invalid");
            return false;
        }

        OutEventId = MakeEventId(Binding, SourceEpoch, SourceSequence);
        const TSharedRef<FJsonObject> Root = MakeShared<FJsonObject>();
        AddCommonIdentity(
            Root,
            TEXT("lingtu.sim.ue-runtime-request.v1"),
            Binding,
            SourceEpoch,
            SourceSequence,
            OutEventId);
        Root->SetStringField(TEXT("request"), RequestName);
        Root->SetStringField(TEXT("ui_mode"), Request.UIMode);
        Root->SetStringField(TEXT("camera_mode"), Request.CameraMode);
        Root->SetNumberField(
            TEXT("source_monotonic_ns"),
            static_cast<double>(Request.SourceMonotonicNs));

        FString Json;
        return SerializeObject(Root, Json, OutError)
            && ToDatagram(Json, OutDatagram, OutError);
    }

    bool FControlTransportProtocol::ParseControlAckJson(
        const FControlTransportBinding& Binding,
        const FString& AckJson,
        FControlAckEnvelope& OutAck,
        FString& OutError)
    {
        OutAck = FControlAckEnvelope{};
        OutError.Reset();
        if (!Binding.IsValid())
        {
            OutError = TEXT("control transport is not bound");
            return false;
        }
        const FTCHARToUTF8 Utf8(*AckJson);
        if (Utf8.Length() <= 0 || Utf8.Length() > MaxControlDatagramBytes)
        {
            OutError = TEXT("control ACK is empty or oversized");
            return false;
        }
        static const TSet<FString> AckFields = {
            TEXT("schema"),
            TEXT("run_id"),
            TEXT("session_id"),
            TEXT("boot_id"),
            TEXT("model_generation"),
            TEXT("reset_generation"),
            TEXT("server_status_sequence"),
            TEXT("source_id"),
            TEXT("source_epoch"),
            TEXT("source_sequence"),
            TEXT("event_id"),
            TEXT("intent_datagram_sha256"),
            TEXT("status"),
            TEXT("reason"),
        };
        if (!HasExactTopLevelFields(AckJson, AckFields, OutError))
        {
            return false;
        }

        TSharedPtr<FJsonObject> Root;
        const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(AckJson);
        if (!FJsonSerializer::Deserialize(Reader, Root) || !Root.IsValid())
        {
            OutError = TEXT("control ACK is not a valid JSON object");
            return false;
        }

        FString Schema;
        FString Status;
        if (!Root->TryGetStringField(TEXT("schema"), Schema)
            || Schema != TEXT("lingtu.sim.ue-control-ack.v1")
            || !Root->TryGetStringField(TEXT("run_id"), OutAck.RunId)
            || !Root->TryGetStringField(TEXT("session_id"), OutAck.SessionId)
            || !Root->TryGetStringField(TEXT("boot_id"), OutAck.BootId)
            || !Root->TryGetStringField(TEXT("source_id"), OutAck.SourceId)
            || !Root->TryGetStringField(TEXT("event_id"), OutAck.EventId)
            || !Root->TryGetStringField(
                TEXT("intent_datagram_sha256"),
                OutAck.IntentDatagramSha256)
            || !Root->TryGetStringField(TEXT("status"), Status)
            || !Root->TryGetStringField(TEXT("reason"), OutAck.Reason)
            || !ReadExactUint64(
                Root,
                TEXT("model_generation"),
                OutAck.ModelGeneration,
                OutError)
            || !ReadExactUint64(
                Root,
                TEXT("reset_generation"),
                OutAck.ResetGeneration,
                OutError)
            || !ReadExactUint64(
                Root,
                TEXT("server_status_sequence"),
                OutAck.ServerStatusSequence,
                OutError)
            || !ReadExactUint64(
                Root,
                TEXT("source_epoch"),
                OutAck.SourceEpoch,
                OutError)
            || !ReadExactUint64(
                Root,
                TEXT("source_sequence"),
                OutAck.SourceSequence,
                OutError))
        {
            if (OutError.IsEmpty())
            {
                OutError = TEXT("control ACK fields are malformed");
            }
            return false;
        }

        if (OutAck.RunId != Binding.RunId
            || OutAck.SessionId != Binding.SessionId
            || OutAck.BootId != Binding.BootId
            || OutAck.ModelGeneration != Binding.ModelGeneration
            || OutAck.ResetGeneration != Binding.ResetGeneration
            || OutAck.SourceId != Binding.SourceId)
        {
            OutError = TEXT("control ACK identity or generation does not match the binding");
            return false;
        }
        if (OutAck.SourceEpoch == 0
            || OutAck.SourceSequence == 0
            || OutAck.EventId != MakeEventId(
                Binding,
                OutAck.SourceEpoch,
                OutAck.SourceSequence))
        {
            OutError = TEXT("control ACK source correlation is invalid");
            return false;
        }
        if (!IsLowerHex(OutAck.IntentDatagramSha256, 64))
        {
            OutError = TEXT("control ACK intent_datagram_sha256 must be lowercase SHA-256");
            return false;
        }
        if (!ParseAckStatus(Status, OutAck.Status))
        {
            OutError = TEXT("control ACK status is unsupported");
            return false;
        }
        const FTCHARToUTF8 ReasonUtf8(*OutAck.Reason);
        bool bReasonHasControlCharacter = false;
        for (const TCHAR Character : OutAck.Reason)
        {
            if (Character < 32 || Character == 127)
            {
                bReasonHasControlCharacter = true;
                break;
            }
        }
        if (OutAck.ServerStatusSequence == 0
            || ReasonUtf8.Length() > 512
            || OutAck.Reason.TrimStartAndEnd() != OutAck.Reason
            || bReasonHasControlCharacter)
        {
            OutError = TEXT("control ACK sequence/reason is invalid");
            return false;
        }
        if (OutAck.Status != EControlAckStatus::Accepted
            && OutAck.Status != EControlAckStatus::Confirmed
            && OutAck.Reason.IsEmpty())
        {
            OutError = TEXT("control ACK reason is required for this status");
            return false;
        }
        return true;
    }

    bool FControlTransportProtocol::Sha256Hex(
        const TArray<uint8>& Datagram,
        FString& OutSha256,
        FString& OutError)
    {
        OutSha256.Reset();
        OutError.Reset();
        if (Datagram.IsEmpty() || Datagram.Num() > MaxControlDatagramBytes)
        {
            OutError = TEXT("cannot hash an empty or oversized control datagram");
            return false;
        }
        FEncryptionContext EncryptionContext;
        TArray<uint8> Digest;
        if (!EncryptionContext.CalcSHA256(
                MakeArrayView(Datagram),
                Digest)
            || Digest.Num() != 32)
        {
            OutError = TEXT("SHA-256 computation failed");
            return false;
        }
        static constexpr TCHAR HexDigits[] = TEXT("0123456789abcdef");
        OutSha256.Reserve(64);
        for (const uint8 Byte : Digest)
        {
            OutSha256.AppendChar(HexDigits[Byte >> 4]);
            OutSha256.AppendChar(HexDigits[Byte & 0x0f]);
        }
        if (!IsLowerHex(OutSha256, 64))
        {
            OutError = TEXT("SHA-256 result is not canonical");
            OutSha256.Reset();
            return false;
        }
        return true;
    }

    bool FControlTransportProtocol::BuildOriginEvidenceJson(
        const FControlTransportBinding& Binding,
        const FString& EventId,
        const uint64 SourceEpoch,
        const uint64 SourceSequence,
        const FString& DatagramSha256,
        const int32 DatagramBytes,
        FString& OutEvidenceJson,
        FString& OutError)
    {
        OutEvidenceJson.Reset();
        OutError.Reset();
        if (!Binding.IsValid()
            || EventId != MakeEventId(Binding, SourceEpoch, SourceSequence)
            || !IsLowerHex(DatagramSha256, 64)
            || DatagramBytes <= 0
            || DatagramBytes > MaxControlDatagramBytes)
        {
            OutError = TEXT("UE control origin evidence identity/hash/size is invalid");
            return false;
        }

        const TSharedRef<FJsonObject> Root = MakeShared<FJsonObject>();
        Root->SetStringField(TEXT("schema"), TEXT("lingtu.sim.ue-control-origin.v1"));
        Root->SetStringField(TEXT("run_id"), Binding.RunId);
        Root->SetStringField(TEXT("session_id"), Binding.SessionId);
        Root->SetStringField(TEXT("boot_id"), Binding.BootId);
        Root->SetNumberField(
            TEXT("model_generation"),
            static_cast<double>(Binding.ModelGeneration));
        Root->SetNumberField(
            TEXT("reset_generation"),
            static_cast<double>(Binding.ResetGeneration));
        Root->SetStringField(TEXT("source_id"), Binding.SourceId);
        Root->SetNumberField(TEXT("source_epoch"), static_cast<double>(SourceEpoch));
        Root->SetNumberField(TEXT("source_sequence"), static_cast<double>(SourceSequence));
        Root->SetStringField(TEXT("event_id"), EventId);
        Root->SetStringField(TEXT("datagram_sha256"), DatagramSha256);
        Root->SetNumberField(TEXT("datagram_bytes"), DatagramBytes);
        Root->SetBoolField(TEXT("successful_send"), true);
        return SerializeObject(Root, OutEvidenceJson, OutError);
    }

    FLingTuSimOperatorIntentSender::FLingTuSimOperatorIntentSender(
        FControlTransportBinding InBinding,
        FSendDatagram InSendDatagram,
        FAppendOriginEvidence InAppendOriginEvidence)
        : Binding(MoveTemp(InBinding))
        , SendDatagram(MoveTemp(InSendDatagram))
        , AppendOriginEvidence(MoveTemp(InAppendOriginEvidence))
    {
    }

    bool FLingTuSimOperatorIntentSender::PublishOperatorIntent(
        const FOperatorIntentSample& Sample,
        FString& OutEventId,
        FString& OutError)
    {
        FScopeLock Lock(&CriticalSection);
        const uint64 Sequence = NextSourceSequence++;
        TArray<uint8> Datagram;
        if (!FControlTransportProtocol::SerializeOperatorIntent(
                Binding,
                Sample,
                SourceEpoch,
                Sequence,
                Datagram,
                OutEventId,
                OutError))
        {
            return false;
        }
        return SendAndRecord(Datagram, OutEventId, Sequence, OutError);
    }

    bool FLingTuSimOperatorIntentSender::PublishRuntimeRequest(
        const FOperatorRuntimeRequest& Request,
        FString& OutEventId,
        FString& OutError)
    {
        FScopeLock Lock(&CriticalSection);
        const uint64 Sequence = NextSourceSequence++;
        TArray<uint8> Datagram;
        if (!FControlTransportProtocol::SerializeRuntimeRequest(
                Binding,
                Request,
                SourceEpoch,
                Sequence,
                Datagram,
                OutEventId,
                OutError))
        {
            return false;
        }
        return SendAndRecord(Datagram, OutEventId, Sequence, OutError);
    }

    bool FLingTuSimOperatorIntentSender::ValidateAckAgainstSuccessfulSend(
        const FControlAckEnvelope& Ack,
        FString& OutError)
    {
        FScopeLock Lock(&CriticalSection);
        OutError.Reset();
        if (Ack.RunId != Binding.RunId
            || Ack.SessionId != Binding.SessionId
            || Ack.BootId != Binding.BootId
            || Ack.ModelGeneration != Binding.ModelGeneration
            || Ack.ResetGeneration != Binding.ResetGeneration
            || Ack.SourceId != Binding.SourceId
            || Ack.SourceEpoch == 0
            || Ack.SourceSequence == 0
            || Ack.EventId != MakeEventId(
                Binding,
                Ack.SourceEpoch,
                Ack.SourceSequence))
        {
            OutError = TEXT("control ACK identity does not match this sender");
            return false;
        }

        const FSuccessfulSendKey Key{
            Ack.SourceEpoch,
            Ack.SourceSequence,
            Ack.EventId,
        };
        const FString* SuccessfulSendSha256 = SuccessfulSendSha256ByKey.Find(Key);
        if (SuccessfulSendSha256 == nullptr
            || *SuccessfulSendSha256 != Ack.IntentDatagramSha256)
        {
            OutError = TEXT("control ACK does not match a successful control send");
            return false;
        }
        return true;
    }

    bool FLingTuSimOperatorIntentSender::SendAndRecord(
        const TArray<uint8>& Datagram,
        const FString& EventId,
        const uint64 SourceSequence,
        FString& OutError)
    {
        if (!SendDatagram || !AppendOriginEvidence)
        {
            OutError = TEXT("control sender callbacks are not bound");
            return false;
        }
        if (!SendDatagram(Datagram, OutError))
        {
            return false;
        }

        FString DatagramSha256;
        FString EvidenceJson;
        if (!FControlTransportProtocol::Sha256Hex(
                Datagram,
                DatagramSha256,
                OutError))
        {
            return false;
        }
        if (!FControlTransportProtocol::BuildOriginEvidenceJson(
                Binding,
                EventId,
                SourceEpoch,
                SourceSequence,
                DatagramSha256,
                Datagram.Num(),
                EvidenceJson,
                OutError))
        {
            return false;
        }
        if (!AppendOriginEvidence(EvidenceJson, OutError))
        {
            return false;
        }
        RememberSuccessfulSend(EventId, SourceSequence, DatagramSha256);
        return true;
    }

    void FLingTuSimOperatorIntentSender::RememberSuccessfulSend(
        const FString& EventId,
        const uint64 SourceSequence,
        const FString& DatagramSha256)
    {
        const FSuccessfulSendKey Key{
            SourceEpoch,
            SourceSequence,
            EventId,
        };
        if (!SuccessfulSendSha256ByKey.Contains(Key))
        {
            SuccessfulSendOrder.Add(Key);
        }
        SuccessfulSendSha256ByKey.Add(Key, DatagramSha256);
        while (SuccessfulSendOrder.Num() > MaxTrackedSuccessfulSends)
        {
            const FSuccessfulSendKey Oldest = SuccessfulSendOrder[0];
            SuccessfulSendOrder.RemoveAt(0, 1, EAllowShrinking::No);
            SuccessfulSendSha256ByKey.Remove(Oldest);
        }
    }
}

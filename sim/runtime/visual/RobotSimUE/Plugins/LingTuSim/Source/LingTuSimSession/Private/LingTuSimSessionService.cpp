#include "LingTuSimSessionService.h"

#include "Misc/ScopeLock.h"
#include "HAL/PlatformTime.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

namespace LingTuSim
{
    namespace
    {
        struct FSessionState final
        {
            FCriticalSection CriticalSection;
            FSnapshotMailbox SnapshotMailbox;
            TSharedPtr<FLingTuSimOperatorIntentSender, ESPMode::ThreadSafe> ControlSender;
            FControlTransportBinding ControlBinding;
            FControlAckEnvelope LatestControlAck;
            FControlStatusEnvelope LatestControlStatus;
            FString BoundSessionId;
            uint64 BoundModelGeneration = 0;
            FSnapshotEnvelope LatestFutureSnapshot;
            FString PendingScenarioSnapshotJson;
            uint64 LastScenarioResetGeneration = 0;
            uint64 LastScenarioSequence = 0;
            int64 LastScenarioSimTimeNs = 0;
            bool bHasBoundSession = false;
            bool bHasFutureSnapshot = false;
            bool bHasScenarioOrderingWatermark = false;
            bool bHasPendingScenarioSnapshot = false;
            bool bHasControlBinding = false;
            bool bHasControlAck = false;
            bool bHasControlStatus = false;
        };

        FSessionState& GetSessionState()
        {
            static FSessionState State;
            return State;
        }

        bool IsValidSessionId(const FString& Value)
        {
            if (Value.IsEmpty() || Value.Len() > 63)
            {
                return false;
            }
            for (const TCHAR Character : Value)
            {
                if (FChar::IsWhitespace(Character) || Character == TEXT('\0'))
                {
                    return false;
                }
            }
            return true;
        }

        bool ReadNonNegativeInteger(
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
                || FMath::FloorToDouble(Number) != Number
                || Number > 9'007'199'254'740'991.0)
            {
                OutError = FString::Printf(
                    TEXT("scenario snapshot %s must be a non-negative integer"),
                    Field);
                return false;
            }
            OutValue = static_cast<uint64>(Number);
            return true;
        }

        bool ReadNonNegativeSimTime(
            const TSharedPtr<FJsonObject>& Object,
            int64& OutValue,
            FString& OutError)
        {
            uint64 Value = 0;
            if (!ReadNonNegativeInteger(Object, TEXT("sim_time_ns"), Value, OutError)
                || Value > static_cast<uint64>(MAX_int64))
            {
                if (OutError.IsEmpty())
                {
                    OutError = TEXT("scenario snapshot sim_time_ns exceeds int64");
                }
                return false;
            }
            OutValue = static_cast<int64>(Value);
            return true;
        }

        bool HasExactScenarioTopLevelFields(
            const TSharedPtr<FJsonObject>& Object,
            FString& OutError)
        {
            static const TSet<FString> RequiredFields = {
                TEXT("schema"),
                TEXT("session_id"),
                TEXT("model_generation"),
                TEXT("reset_generation"),
                TEXT("sequence"),
                TEXT("sim_time_ns"),
                TEXT("entities"),
            };
            if (!Object.IsValid() || Object->Values.Num() != RequiredFields.Num())
            {
                OutError = TEXT("scenario snapshot must contain exactly the canonical top-level fields");
                return false;
            }
            for (const FString& Field : RequiredFields)
            {
                if (!Object->HasField(Field))
                {
                    OutError = FString::Printf(
                        TEXT("scenario snapshot is missing required field '%s'"),
                        *Field);
                    return false;
                }
            }
            return true;
        }

        uint64 ControlReceiveMonotonicNs()
        {
            const double Nanoseconds = FPlatformTime::Seconds() * 1'000'000'000.0;
            if (!FMath::IsFinite(Nanoseconds) || Nanoseconds <= 0.0)
            {
                return 0;
            }
            return static_cast<uint64>(Nanoseconds);
        }
    }

    bool FSessionService::RebindSession(
        FString SessionId,
        const uint64 ModelGeneration)
    {
        if (SessionId.IsEmpty())
        {
            return false;
        }

        FSessionState& State = GetSessionState();
        FScopeLock Lock(&State.CriticalSection);
        if (State.bHasBoundSession)
        {
            const bool bSameBinding = State.BoundSessionId == SessionId
                && State.BoundModelGeneration == ModelGeneration;
            if (bSameBinding)
            {
                return true;
            }
            if (ModelGeneration <= State.BoundModelGeneration)
            {
                return false;
            }
        }

        State.BoundSessionId = MoveTemp(SessionId);
        State.BoundModelGeneration = ModelGeneration;
        State.LatestFutureSnapshot = FSnapshotEnvelope{};
        State.PendingScenarioSnapshotJson.Reset();
        State.ControlSender.Reset();
        State.ControlBinding = FControlTransportBinding{};
        State.LatestControlAck = FControlAckEnvelope{};
        State.LatestControlStatus = FControlStatusEnvelope{};
        State.LastScenarioResetGeneration = 0;
        State.LastScenarioSequence = 0;
        State.LastScenarioSimTimeNs = 0;
        State.bHasBoundSession = true;
        State.bHasFutureSnapshot = false;
        State.bHasScenarioOrderingWatermark = false;
        State.bHasPendingScenarioSnapshot = false;
        State.bHasControlBinding = false;
        State.bHasControlAck = false;
        State.bHasControlStatus = false;
        State.SnapshotMailbox.BindSession(
            State.BoundSessionId,
            State.BoundModelGeneration);
        return true;
    }

    void FSessionService::UnbindSession()
    {
        FSessionState& State = GetSessionState();
        FScopeLock Lock(&State.CriticalSection);
        State.BoundSessionId.Reset();
        State.BoundModelGeneration = 0;
        State.LatestFutureSnapshot = FSnapshotEnvelope{};
        State.PendingScenarioSnapshotJson.Reset();
        State.ControlSender.Reset();
        State.ControlBinding = FControlTransportBinding{};
        State.LatestControlAck = FControlAckEnvelope{};
        State.LatestControlStatus = FControlStatusEnvelope{};
        State.LastScenarioResetGeneration = 0;
        State.LastScenarioSequence = 0;
        State.LastScenarioSimTimeNs = 0;
        State.bHasBoundSession = false;
        State.bHasFutureSnapshot = false;
        State.bHasScenarioOrderingWatermark = false;
        State.bHasPendingScenarioSnapshot = false;
        State.bHasControlBinding = false;
        State.bHasControlAck = false;
        State.bHasControlStatus = false;
        State.SnapshotMailbox.BindSession(FString{}, 0);
    }

    bool FSessionService::GetBoundSession(
        FString& OutSessionId,
        uint64& OutModelGeneration)
    {
        FSessionState& State = GetSessionState();
        FScopeLock Lock(&State.CriticalSection);
        if (!State.bHasBoundSession)
        {
            return false;
        }
        OutSessionId = State.BoundSessionId;
        OutModelGeneration = State.BoundModelGeneration;
        return true;
    }

    ESnapshotPublishResult FSessionService::PublishSnapshot(
        const FSnapshotEnvelope& Snapshot)
    {
        FSessionState& State = GetSessionState();
        FScopeLock Lock(&State.CriticalSection);
        const ESnapshotPublishResult Result = State.SnapshotMailbox.Publish(Snapshot);
        if ((Result == ESnapshotPublishResult::ModelMismatch
                || Result == ESnapshotPublishResult::SessionMismatch)
            && State.bHasBoundSession
            && !Snapshot.SessionId.IsEmpty()
            && Snapshot.ModelGeneration > State.BoundModelGeneration
            && (!State.bHasFutureSnapshot
                || Snapshot.ModelGeneration >= State.LatestFutureSnapshot.ModelGeneration))
        {
            State.LatestFutureSnapshot = Snapshot;
            State.bHasFutureSnapshot = true;
        }
        return Result;
    }

    bool FSessionService::PublishSnapshotJson(
        const FString& SnapshotJson,
        FSnapshotEnvelope& OutSnapshot,
        ESnapshotPublishResult& OutPublishResult,
        FRuntimeLoadError& OutError)
    {
        OutPublishResult = ESnapshotPublishResult::Stale;
        if (!FSessionBundleLoader::ParseSnapshotJson(
                SnapshotJson,
                OutSnapshot,
                OutError))
        {
            return false;
        }
        OutPublishResult = PublishSnapshot(OutSnapshot);
        return true;
    }

    bool FSessionService::PublishScenarioSnapshotJson(
        const FString& SnapshotJson,
        ESnapshotPublishResult& OutPublishResult,
        FString& OutError)
    {
        OutPublishResult = ESnapshotPublishResult::Stale;
        OutError.Reset();

        TSharedPtr<FJsonObject> RootObject;
        const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(SnapshotJson);
        if (!FJsonSerializer::Deserialize(Reader, RootObject) || !RootObject.IsValid())
        {
            OutError = Reader->GetErrorMessage().IsEmpty()
                ? TEXT("scenario snapshot is not a JSON object")
                : FString::Printf(TEXT("invalid scenario snapshot JSON: %s"), *Reader->GetErrorMessage());
            return false;
        }
        if (!HasExactScenarioTopLevelFields(RootObject, OutError))
        {
            return false;
        }

        FString Schema;
        if (!RootObject->TryGetStringField(TEXT("schema"), Schema)
            || Schema != TEXT("lingtu.sim.scenario-snapshot.v1"))
        {
            OutError = TEXT("scenario snapshot schema must be lingtu.sim.scenario-snapshot.v1");
            return false;
        }

        FString SessionId;
        uint64 ModelGeneration = 0;
        uint64 ResetGeneration = 0;
        uint64 Sequence = 0;
        int64 SimTimeNs = 0;
        const TArray<TSharedPtr<FJsonValue>>* Entities = nullptr;
        if (!RootObject->TryGetStringField(TEXT("session_id"), SessionId)
            || !IsValidSessionId(SessionId))
        {
            OutError = TEXT("scenario snapshot session_id is invalid");
            return false;
        }
        if (!ReadNonNegativeInteger(RootObject, TEXT("model_generation"), ModelGeneration, OutError)
            || !ReadNonNegativeInteger(RootObject, TEXT("reset_generation"), ResetGeneration, OutError)
            || !ReadNonNegativeInteger(RootObject, TEXT("sequence"), Sequence, OutError)
            || !ReadNonNegativeSimTime(RootObject, SimTimeNs, OutError)
            || !RootObject->TryGetArrayField(TEXT("entities"), Entities)
            || Entities == nullptr)
        {
            if (OutError.IsEmpty())
            {
                OutError = TEXT("scenario snapshot entities must be an array");
            }
            return false;
        }

        FSessionState& State = GetSessionState();
        FScopeLock Lock(&State.CriticalSection);
        if (!State.bHasBoundSession || SessionId != State.BoundSessionId)
        {
            OutPublishResult = ESnapshotPublishResult::SessionMismatch;
            return true;
        }
        if (ModelGeneration != State.BoundModelGeneration)
        {
            OutPublishResult = ESnapshotPublishResult::ModelMismatch;
            return true;
        }
        if (State.bHasScenarioOrderingWatermark)
        {
            // The visual UDP sink is a lossy projection endpoint: sequence 0 may
            // be emitted before UE starts, and the capacity-one mailbox may
            // coalesce a reset boundary. Accept the first observed higher reset
            // at any sequence; the authoritative Python dispatcher enforces
            // contiguous reset epochs and emits sequence 0.
            if (ResetGeneration < State.LastScenarioResetGeneration)
            {
                OutPublishResult = ESnapshotPublishResult::Stale;
                return true;
            }
            if (ResetGeneration == State.LastScenarioResetGeneration
                && (Sequence <= State.LastScenarioSequence
                    || SimTimeNs <= State.LastScenarioSimTimeNs))
            {
                OutPublishResult = ESnapshotPublishResult::Stale;
                return true;
            }
        }

        const bool bReplaced = State.bHasPendingScenarioSnapshot;
        State.PendingScenarioSnapshotJson = SnapshotJson;
        State.LastScenarioResetGeneration = ResetGeneration;
        State.LastScenarioSequence = Sequence;
        State.LastScenarioSimTimeNs = SimTimeNs;
        State.bHasScenarioOrderingWatermark = true;
        State.bHasPendingScenarioSnapshot = true;
        OutPublishResult = bReplaced
            ? ESnapshotPublishResult::Replaced
            : ESnapshotPublishResult::Accepted;
        return true;
    }

    bool FSessionService::TryTakeLatestSnapshot(FSnapshotEnvelope& OutSnapshot)
    {
        FSessionState& State = GetSessionState();
        FScopeLock Lock(&State.CriticalSection);
        return State.SnapshotMailbox.TryTakeLatest(OutSnapshot);
    }

    bool FSessionService::TryTakeLatestFutureSnapshot(
        FSnapshotEnvelope& OutSnapshot)
    {
        FSessionState& State = GetSessionState();
        FScopeLock Lock(&State.CriticalSection);
        if (!State.bHasFutureSnapshot)
        {
            return false;
        }
        OutSnapshot = State.LatestFutureSnapshot;
        State.LatestFutureSnapshot = FSnapshotEnvelope{};
        State.bHasFutureSnapshot = false;
        return true;
    }

    bool FSessionService::TryTakeLatestScenarioSnapshotJson(FString& OutSnapshotJson)
    {
        FSessionState& State = GetSessionState();
        FScopeLock Lock(&State.CriticalSection);
        if (!State.bHasPendingScenarioSnapshot)
        {
            return false;
        }
        OutSnapshotJson = State.PendingScenarioSnapshotJson;
        State.PendingScenarioSnapshotJson.Reset();
        State.bHasPendingScenarioSnapshot = false;
        return true;
    }

    void FSessionService::ClearSnapshots()
    {
        FSessionState& State = GetSessionState();
        FScopeLock Lock(&State.CriticalSection);
        State.SnapshotMailbox.Clear();
        State.LatestFutureSnapshot = FSnapshotEnvelope{};
        State.PendingScenarioSnapshotJson.Reset();
        State.bHasFutureSnapshot = false;
        State.bHasPendingScenarioSnapshot = false;
    }

    bool FSessionService::BindControlTransport(
        const TSharedRef<FLingTuSimOperatorIntentSender, ESPMode::ThreadSafe>& Sender)
    {
        const FControlTransportBinding& Binding = Sender->GetBinding();
        if (!Binding.IsValid())
        {
            return false;
        }

        FSessionState& State = GetSessionState();
        FScopeLock Lock(&State.CriticalSection);
        if (!State.bHasBoundSession
            || Binding.SessionId != State.BoundSessionId
            || Binding.ModelGeneration != State.BoundModelGeneration)
        {
            return false;
        }
        State.ControlSender = Sender;
        State.ControlBinding = Binding;
        State.LatestControlAck = FControlAckEnvelope{};
        State.LatestControlStatus = FControlStatusEnvelope{};
        State.bHasControlBinding = true;
        State.bHasControlAck = false;
        State.bHasControlStatus = false;
        return true;
    }

    void FSessionService::UnbindControlTransport()
    {
        FSessionState& State = GetSessionState();
        FScopeLock Lock(&State.CriticalSection);
        State.ControlSender.Reset();
        State.ControlBinding = FControlTransportBinding{};
        State.LatestControlAck = FControlAckEnvelope{};
        State.LatestControlStatus = FControlStatusEnvelope{};
        State.bHasControlBinding = false;
        State.bHasControlAck = false;
        State.bHasControlStatus = false;
    }

    bool FSessionService::PublishOperatorIntent(
        const FOperatorIntentSample& Sample,
        FString& OutEventId,
        FString& OutError)
    {
        TSharedPtr<FLingTuSimOperatorIntentSender, ESPMode::ThreadSafe> Sender;
        {
            FSessionState& State = GetSessionState();
            FScopeLock Lock(&State.CriticalSection);
            if (!State.bHasControlBinding || !State.ControlSender.IsValid())
            {
                OutEventId.Reset();
                OutError = TEXT("playable control transport is not bound");
                return false;
            }
            Sender = State.ControlSender;
        }
        return Sender->PublishOperatorIntent(Sample, OutEventId, OutError);
    }

    bool FSessionService::PublishRuntimeRequest(
        const FOperatorRuntimeRequest& Request,
        FString& OutEventId,
        FString& OutError)
    {
        TSharedPtr<FLingTuSimOperatorIntentSender, ESPMode::ThreadSafe> Sender;
        {
            FSessionState& State = GetSessionState();
            FScopeLock Lock(&State.CriticalSection);
            if (!State.bHasControlBinding || !State.ControlSender.IsValid())
            {
                OutEventId.Reset();
                OutError = TEXT("playable control transport is not bound");
                return false;
            }
            Sender = State.ControlSender;
        }
        return Sender->PublishRuntimeRequest(Request, OutEventId, OutError);
    }

    bool FSessionService::PublishControlAckJson(
        const FString& AckJson,
        FString& OutError)
    {
        FControlTransportBinding Binding;
        TSharedPtr<FLingTuSimOperatorIntentSender, ESPMode::ThreadSafe> Sender;
        {
            FSessionState& State = GetSessionState();
            FScopeLock Lock(&State.CriticalSection);
            if (!State.bHasControlBinding || !State.ControlSender.IsValid())
            {
                OutError = TEXT("playable control transport is not bound");
                return false;
            }
            Binding = State.ControlBinding;
            Sender = State.ControlSender;
        }

        FControlAckEnvelope Ack;
        if (!FControlTransportProtocol::ParseControlAckJson(
                Binding,
                AckJson,
                Ack,
                OutError))
        {
            return false;
        }
        if (!Sender->ValidateAckAgainstSuccessfulSend(Ack, OutError))
        {
            return false;
        }

        FSessionState& State = GetSessionState();
        FScopeLock Lock(&State.CriticalSection);
        if (!State.bHasControlBinding
            || State.ControlSender != Sender
            || State.ControlBinding.RunId != Binding.RunId
            || State.ControlBinding.SessionId != Binding.SessionId
            || State.ControlBinding.BootId != Binding.BootId
            || State.ControlBinding.ModelGeneration != Binding.ModelGeneration
            || State.ControlBinding.ResetGeneration != Binding.ResetGeneration)
        {
            OutError = TEXT("control transport binding changed while ACK was parsed");
            return false;
        }
        if (State.bHasControlAck
            && Ack.ServerStatusSequence <= State.LatestControlAck.ServerStatusSequence)
        {
            OutError = TEXT("control ACK server_status_sequence is stale");
            return false;
        }
        State.LatestControlAck = MoveTemp(Ack);
        State.bHasControlAck = true;
        return true;
    }

    bool FSessionService::PublishControlResponseJson(
        const FString& ResponseJson,
        FString& OutError)
    {
        TSharedPtr<FJsonObject> Root;
        const TSharedRef<TJsonReader<>> Reader =
            TJsonReaderFactory<>::Create(ResponseJson);
        FString Schema;
        if (!FJsonSerializer::Deserialize(Reader, Root)
            || !Root.IsValid()
            || !Root->TryGetStringField(TEXT("schema"), Schema))
        {
            OutError = TEXT("control response is not a JSON object with a schema");
            return false;
        }
        if (Schema == TEXT("lingtu.sim.ue-control-ack.v1"))
        {
            return PublishControlAckJson(ResponseJson, OutError);
        }
        if (Schema == TEXT("lingtu.sim.ue-control-status.v1"))
        {
            return PublishControlStatusJson(ResponseJson, OutError);
        }
        OutError = TEXT("control response schema is unsupported");
        return false;
    }

    bool FSessionService::PublishControlStatusJson(
        const FString& StatusJson,
        FString& OutError)
    {
        FControlTransportBinding Binding;
        TSharedPtr<FLingTuSimOperatorIntentSender, ESPMode::ThreadSafe> Sender;
        {
            FSessionState& State = GetSessionState();
            FScopeLock Lock(&State.CriticalSection);
            if (!State.bHasControlBinding || !State.ControlSender.IsValid())
            {
                OutError = TEXT("playable control transport is not bound");
                return false;
            }
            Binding = State.ControlBinding;
            Sender = State.ControlSender;
        }

        FControlStatusEnvelope Status;
        if (!FControlTransportProtocol::ParseControlStatusJson(
                Binding,
                StatusJson,
                Status,
                OutError)
            || !Sender->ValidateStatusAgainstSuccessfulSend(Status, OutError))
        {
            return false;
        }
        Status.ReceivedMonotonicNs = ControlReceiveMonotonicNs();
        if (Status.ReceivedMonotonicNs == 0)
        {
            OutError = TEXT("cannot timestamp the received control status");
            return false;
        }

        FSessionState& State = GetSessionState();
        FScopeLock Lock(&State.CriticalSection);
        if (!State.bHasControlBinding
            || State.ControlSender != Sender
            || State.ControlBinding.RunId != Binding.RunId
            || State.ControlBinding.SessionId != Binding.SessionId
            || State.ControlBinding.BootId != Binding.BootId
            || State.ControlBinding.ModelGeneration != Binding.ModelGeneration
            || State.ControlBinding.ResetGeneration != Binding.ResetGeneration)
        {
            OutError = TEXT("control transport binding changed while status was parsed");
            return false;
        }
        if (State.bHasControlStatus
            && Status.ServerStatusSequence
                <= State.LatestControlStatus.ServerStatusSequence)
        {
            OutError = TEXT("control status server_status_sequence is stale");
            return false;
        }
        State.LatestControlStatus = MoveTemp(Status);
        State.bHasControlStatus = true;
        return true;
    }

    bool FSessionService::GetLatestControlAck(FControlAckEnvelope& OutAck)
    {
        FSessionState& State = GetSessionState();
        FScopeLock Lock(&State.CriticalSection);
        if (!State.bHasControlBinding || !State.bHasControlAck)
        {
            return false;
        }
        OutAck = State.LatestControlAck;
        return true;
    }

    bool FSessionService::GetLatestControlStatus(
        FControlStatusEnvelope& OutStatus)
    {
        FSessionState& State = GetSessionState();
        FScopeLock Lock(&State.CriticalSection);
        if (!State.bHasControlBinding || !State.bHasControlStatus)
        {
            return false;
        }
        OutStatus = State.LatestControlStatus;
        return true;
    }

    bool FSessionService::GetControlTransportBinding(FControlTransportBinding& OutBinding)
    {
        FSessionState& State = GetSessionState();
        FScopeLock Lock(&State.CriticalSection);
        if (!State.bHasControlBinding)
        {
            return false;
        }
        OutBinding = State.ControlBinding;
        return true;
    }

    FSnapshotMailbox& FSessionService::GetSnapshotMailbox()
    {
        return GetSessionState().SnapshotMailbox;
    }
}

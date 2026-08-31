#include "LingTuSimControlTransport.h"

#include "Misc/ScopeLock.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

namespace LingTuSim
{
    namespace
    {
        constexpr int32 MaxControlDatagramBytes = 4'096;
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

        bool HasNoDuplicateObjectFields(const FString& Json, FString& OutError)
        {
            TArray<TSet<FString>> ObjectFields;
            bool bInString = false;
            bool bEscaped = false;
            int32 StringStart = INDEX_NONE;
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
                    int32 After = Index + 1;
                    while (After < Json.Len() && FChar::IsWhitespace(Json[After]))
                    {
                        ++After;
                    }
                    if (After < Json.Len() && Json[After] == TEXT(':'))
                    {
                        if (ObjectFields.IsEmpty())
                        {
                            OutError = TEXT("JSON field appears outside an object");
                            return false;
                        }
                        const FString Field = Json.Mid(StringStart, Index - StringStart);
                        if (Field.Contains(TEXT("\\")))
                        {
                            OutError = TEXT("canonical JSON field names must not use escapes");
                            return false;
                        }
                        if (ObjectFields.Last().Contains(Field))
                        {
                            OutError = FString::Printf(
                                TEXT("duplicate JSON object field '%s'"),
                                *Field);
                            return false;
                        }
                        ObjectFields.Last().Add(Field);
                    }
                    continue;
                }
                if (Character == TEXT('"'))
                {
                    bInString = true;
                    bEscaped = false;
                    StringStart = Index + 1;
                }
                else if (Character == TEXT('{'))
                {
                    ObjectFields.Emplace();
                }
                else if (Character == TEXT('}'))
                {
                    if (ObjectFields.IsEmpty())
                    {
                        OutError = TEXT("JSON object depth is invalid");
                        return false;
                    }
                    ObjectFields.Pop();
                }
            }
            if (bInString || !ObjectFields.IsEmpty())
            {
                OutError = TEXT("JSON string/object is unterminated");
                return false;
            }
            return true;
        }

        bool HasExactFields(
            const TSharedPtr<FJsonObject>& Object,
            const TSet<FString>& Expected,
            const TCHAR* Context,
            FString& OutError)
        {
            if (!Object.IsValid() || Object->Values.Num() != Expected.Num())
            {
                OutError = FString::Printf(
                    TEXT("%s must contain exactly the canonical fields"),
                    Context);
                return false;
            }
            for (const FString& Field : Expected)
            {
                if (!Object->HasField(Field))
                {
                    OutError = FString::Printf(
                        TEXT("%s is missing field '%s'"),
                        Context,
                        *Field);
                    return false;
                }
            }
            return true;
        }

        bool ReadObject(
            const TSharedPtr<FJsonObject>& Parent,
            const TCHAR* Field,
            TSharedPtr<FJsonObject>& OutObject,
            FString& OutError)
        {
            const TSharedPtr<FJsonObject>* Object = nullptr;
            if (!Parent.IsValid()
                || !Parent->TryGetObjectField(Field, Object)
                || Object == nullptr
                || !Object->IsValid())
            {
                OutError = FString::Printf(TEXT("%s must be an object"), Field);
                return false;
            }
            OutObject = *Object;
            return true;
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

        bool ReadFiniteDouble(
            const TSharedPtr<FJsonObject>& Object,
            const TCHAR* Field,
            double& OutValue,
            FString& OutError)
        {
            if (!Object.IsValid()
                || !Object->TryGetNumberField(Field, OutValue)
                || !FMath::IsFinite(OutValue))
            {
                OutError = FString::Printf(TEXT("%s must be a finite number"), Field);
                return false;
            }
            return true;
        }

        bool ValidateReason(
            const FString& Value,
            const bool bRequired,
            const TCHAR* Context,
            FString& OutError)
        {
            const FTCHARToUTF8 Utf8(*Value);
            bool bControlCharacter = false;
            for (const TCHAR Character : Value)
            {
                if (Character < 32 || Character == 127)
                {
                    bControlCharacter = true;
                    break;
                }
            }
            if (Utf8.Length() > 512
                || Value.TrimStartAndEnd() != Value
                || bControlCharacter
                || (bRequired && Value.IsEmpty()))
            {
                OutError = FString::Printf(TEXT("%s is invalid"), Context);
                return false;
            }
            return true;
        }

        bool ParseAckStatus(const FString& Value, EControlAckStatus& OutStatus)
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

        bool ParseRuntimeState(
            const FString& Value,
            EControlStatusRuntimeState& OutState)
        {
            if (Value == TEXT("NEW")) OutState = EControlStatusRuntimeState::New;
            else if (Value == TEXT("PREPARING")) OutState = EControlStatusRuntimeState::Preparing;
            else if (Value == TEXT("READY")) OutState = EControlStatusRuntimeState::Ready;
            else if (Value == TEXT("RUNNING")) OutState = EControlStatusRuntimeState::Running;
            else if (Value == TEXT("PAUSED")) OutState = EControlStatusRuntimeState::Paused;
            else if (Value == TEXT("STOPPED")) OutState = EControlStatusRuntimeState::Stopped;
            else if (Value == TEXT("FAILED")) OutState = EControlStatusRuntimeState::Failed;
            else return false;
            return true;
        }

        bool ParseSafeStopState(
            const FString& Value,
            EControlSafeStopState& OutState)
        {
            if (Value == TEXT("unavailable")) OutState = EControlSafeStopState::Unavailable;
            else if (Value == TEXT("clear")) OutState = EControlSafeStopState::Clear;
            else if (Value == TEXT("pending")) OutState = EControlSafeStopState::Pending;
            else if (Value == TEXT("zeroed")) OutState = EControlSafeStopState::Zeroed;
            else if (Value == TEXT("blocked")) OutState = EControlSafeStopState::Blocked;
            else return false;
            return true;
        }

        bool ParseBindingState(
            const FString& Value,
            EControlBindingState& OutState)
        {
            if (Value == TEXT("UNAVAILABLE")) OutState = EControlBindingState::Unavailable;
            else if (Value == TEXT("UNBOUND")) OutState = EControlBindingState::Unbound;
            else if (Value == TEXT("PREPARED")) OutState = EControlBindingState::Prepared;
            else if (Value == TEXT("ACTIVE")) OutState = EControlBindingState::Active;
            else if (Value == TEXT("FAILED")) OutState = EControlBindingState::Failed;
            else return false;
            return true;
        }

        bool ParseSensorState(
            const FString& Value,
            EControlSensorState& OutState)
        {
            if (Value == TEXT("UNAVAILABLE")) OutState = EControlSensorState::Unavailable;
            else if (Value == TEXT("MISSING")) OutState = EControlSensorState::Missing;
            else if (Value == TEXT("UNBOUND")) OutState = EControlSensorState::Unbound;
            else if (Value == TEXT("PREPARED")) OutState = EControlSensorState::Prepared;
            else if (Value == TEXT("ACTIVE")) OutState = EControlSensorState::Active;
            else if (Value == TEXT("FAILED")) OutState = EControlSensorState::Failed;
            else return false;
            return true;
        }

        bool ParseRecordingState(
            const FString& Value,
            EControlRecordingState& OutState)
        {
            if (Value == TEXT("unavailable")) OutState = EControlRecordingState::Unavailable;
            else if (Value == TEXT("idle")) OutState = EControlRecordingState::Idle;
            else if (Value == TEXT("requested")) OutState = EControlRecordingState::Requested;
            else if (Value == TEXT("recording")) OutState = EControlRecordingState::Recording;
            else if (Value == TEXT("committed")) OutState = EControlRecordingState::Committed;
            else if (Value == TEXT("rejected")) OutState = EControlRecordingState::Rejected;
            else if (Value == TEXT("failed")) OutState = EControlRecordingState::Failed;
            else return false;
            return true;
        }

        bool ParseUIMode(const FString& Value, EControlStatusUIMode& OutMode)
        {
            if (Value == TEXT("unavailable")) OutMode = EControlStatusUIMode::Unavailable;
            else if (Value == TEXT("drive")) OutMode = EControlStatusUIMode::Drive;
            else if (Value == TEXT("build")) OutMode = EControlStatusUIMode::Build;
            else if (Value == TEXT("tactical")) OutMode = EControlStatusUIMode::Tactical;
            else if (Value == TEXT("menu")) OutMode = EControlStatusUIMode::Menu;
            else return false;
            return true;
        }

        bool ParseCameraMode(
            const FString& Value,
            EControlStatusCameraMode& OutMode)
        {
            if (Value == TEXT("unavailable")) OutMode = EControlStatusCameraMode::Unavailable;
            else if (Value == TEXT("follow")) OutMode = EControlStatusCameraMode::Follow;
            else if (Value == TEXT("inspection")) OutMode = EControlStatusCameraMode::Inspection;
            else if (Value == TEXT("free")) OutMode = EControlStatusCameraMode::Free;
            else return false;
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

        bool ParseRequestedAxes(
            const TSharedPtr<FJsonObject>& Object,
            FControlStatusRequestedAxes& OutAxes,
            FString& OutError)
        {
            static const TSet<FString> Fields = {
                TEXT("available"), TEXT("forward"), TEXT("left"), TEXT("yaw_left")};
            if (!HasExactFields(Object, Fields, TEXT("motion.requested_axes"), OutError)
                || !Object->TryGetBoolField(TEXT("available"), OutAxes.bAvailable)
                || !ReadFiniteDouble(Object, TEXT("forward"), OutAxes.Forward, OutError)
                || !ReadFiniteDouble(Object, TEXT("left"), OutAxes.Left, OutError)
                || !ReadFiniteDouble(Object, TEXT("yaw_left"), OutAxes.YawLeft, OutError))
            {
                if (OutError.IsEmpty()) OutError = TEXT("requested axes are malformed");
                return false;
            }
            if (!OutAxes.bAvailable
                && (OutAxes.Forward != 0.0 || OutAxes.Left != 0.0 || OutAxes.YawLeft != 0.0))
            {
                OutError = TEXT("unavailable requested axes must be exact zeros");
                return false;
            }
            return true;
        }

        bool ParseVelocity(
            const TSharedPtr<FJsonObject>& Object,
            const TCHAR* Context,
            FControlStatusVelocity& OutVelocity,
            FString& OutError)
        {
            static const TSet<FString> Fields = {
                TEXT("available"), TEXT("linear_x"), TEXT("linear_y"), TEXT("angular_z")};
            if (!HasExactFields(Object, Fields, Context, OutError)
                || !Object->TryGetBoolField(TEXT("available"), OutVelocity.bAvailable)
                || !ReadFiniteDouble(Object, TEXT("linear_x"), OutVelocity.LinearX, OutError)
                || !ReadFiniteDouble(Object, TEXT("linear_y"), OutVelocity.LinearY, OutError)
                || !ReadFiniteDouble(Object, TEXT("angular_z"), OutVelocity.AngularZ, OutError))
            {
                if (OutError.IsEmpty()) OutError = FString::Printf(TEXT("%s is malformed"), Context);
                return false;
            }
            if (!OutVelocity.bAvailable
                && (OutVelocity.LinearX != 0.0
                    || OutVelocity.LinearY != 0.0
                    || OutVelocity.AngularZ != 0.0))
            {
                OutError = FString::Printf(TEXT("unavailable %s must be exact zeros"), Context);
                return false;
            }
            return true;
        }

        bool ParseReadinessFacet(
            const TSharedPtr<FJsonObject>& Object,
            const TCHAR* Context,
            FControlStatusReadinessFacet& OutFacet,
            FString& OutError)
        {
            static const TSet<FString> Fields = {
                TEXT("state"), TEXT("required"), TEXT("source_id"), TEXT("blocker")};
            FString State;
            if (!HasExactFields(Object, Fields, Context, OutError)
                || !Object->TryGetStringField(TEXT("state"), State)
                || !Object->TryGetBoolField(TEXT("required"), OutFacet.bRequired)
                || !Object->TryGetStringField(TEXT("source_id"), OutFacet.SourceId)
                || !Object->TryGetStringField(TEXT("blocker"), OutFacet.Blocker)
                || !ParseBindingState(State, OutFacet.State))
            {
                if (OutError.IsEmpty()) OutError = FString::Printf(TEXT("%s is malformed"), Context);
                return false;
            }
            if ((OutFacet.SourceId != TEXT("unavailable") && !IsCanonicalId(OutFacet.SourceId))
                || !ValidateReason(
                    OutFacet.Blocker,
                    OutFacet.State != EControlBindingState::Active,
                    *FString::Printf(TEXT("%s.blocker"), Context),
                    OutError))
            {
                if (OutError.IsEmpty()) OutError = FString::Printf(TEXT("%s source is invalid"), Context);
                return false;
            }
            return true;
        }

        bool SameCorrelation(
            const FControlTransportBinding& Binding,
            const FString& RunId,
            const FString& SessionId,
            const FString& BootId,
            const uint64 ModelGeneration,
            const uint64 ResetGeneration,
            const FString& SourceId,
            const uint64 SourceEpoch,
            const uint64 SourceSequence,
            const FString& EventId)
        {
            return RunId == Binding.RunId
                && SessionId == Binding.SessionId
                && BootId == Binding.BootId
                && ModelGeneration == Binding.ModelGeneration
                && ResetGeneration == Binding.ResetGeneration
                && SourceId == Binding.SourceId
                && SourceEpoch > 0
                && SourceSequence > 0
                && EventId == MakeEventId(Binding, SourceEpoch, SourceSequence);
        }
    }

    bool FControlStatusEnvelope::IsFresh(
        const uint64 NowMonotonicNs,
        const uint64 MaxAgeNs) const
    {
        return ReceivedMonotonicNs > 0
            && MaxAgeNs > 0
            && NowMonotonicNs >= ReceivedMonotonicNs
            && NowMonotonicNs - ReceivedMonotonicNs <= MaxAgeNs;
    }

    bool FControlTransportProtocol::ParseControlStatusJson(
        const FControlTransportBinding& Binding,
        const FString& StatusJson,
        FControlStatusEnvelope& OutStatus,
        FString& OutError)
    {
        OutStatus = FControlStatusEnvelope{};
        OutError.Reset();
        if (!Binding.IsValid())
        {
            OutError = TEXT("control transport is not bound");
            return false;
        }
        const FTCHARToUTF8 Utf8(*StatusJson);
        if (Utf8.Length() <= 0 || Utf8.Length() > MaxControlDatagramBytes)
        {
            OutError = TEXT("control status is empty or oversized");
            return false;
        }
        if (!HasNoDuplicateObjectFields(StatusJson, OutError))
        {
            return false;
        }
        static const TSet<FString> TopFields = {
            TEXT("schema"), TEXT("run_id"), TEXT("session_id"), TEXT("boot_id"),
            TEXT("model_generation"), TEXT("reset_generation"),
            TEXT("server_status_sequence"), TEXT("server_monotonic_ns"),
            TEXT("sim_time_ns"), TEXT("truth_sequence"), TEXT("source_id"),
            TEXT("source_epoch"), TEXT("source_sequence"), TEXT("event_id"),
            TEXT("intent_datagram_sha256"), TEXT("status"), TEXT("reason"),
            TEXT("runtime"), TEXT("motion"), TEXT("readiness"), TEXT("sensors"),
            TEXT("recording"), TEXT("ui")};

        TSharedPtr<FJsonObject> Root;
        const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(StatusJson);
        if (!FJsonSerializer::Deserialize(Reader, Root)
            || !HasExactFields(Root, TopFields, TEXT("control status"), OutError))
        {
            if (OutError.IsEmpty()) OutError = TEXT("control status is not a valid exact JSON object");
            return false;
        }

        FString Schema;
        FString AckStatus;
        if (!Root->TryGetStringField(TEXT("schema"), Schema)
            || Schema != TEXT("lingtu.sim.ue-control-status.v1")
            || !Root->TryGetStringField(TEXT("run_id"), OutStatus.RunId)
            || !Root->TryGetStringField(TEXT("session_id"), OutStatus.SessionId)
            || !Root->TryGetStringField(TEXT("boot_id"), OutStatus.BootId)
            || !Root->TryGetStringField(TEXT("source_id"), OutStatus.SourceId)
            || !Root->TryGetStringField(TEXT("event_id"), OutStatus.EventId)
            || !Root->TryGetStringField(
                TEXT("intent_datagram_sha256"), OutStatus.IntentDatagramSha256)
            || !Root->TryGetStringField(TEXT("status"), AckStatus)
            || !Root->TryGetStringField(TEXT("reason"), OutStatus.Reason)
            || !ReadExactUint64(Root, TEXT("model_generation"), OutStatus.ModelGeneration, OutError)
            || !ReadExactUint64(Root, TEXT("reset_generation"), OutStatus.ResetGeneration, OutError)
            || !ReadExactUint64(Root, TEXT("server_status_sequence"), OutStatus.ServerStatusSequence, OutError)
            || !ReadExactUint64(Root, TEXT("server_monotonic_ns"), OutStatus.ServerMonotonicNs, OutError)
            || !ReadExactUint64(Root, TEXT("sim_time_ns"), OutStatus.SimTimeNs, OutError)
            || !ReadExactUint64(Root, TEXT("truth_sequence"), OutStatus.TruthSequence, OutError)
            || !ReadExactUint64(Root, TEXT("source_epoch"), OutStatus.SourceEpoch, OutError)
            || !ReadExactUint64(Root, TEXT("source_sequence"), OutStatus.SourceSequence, OutError)
            || !ParseAckStatus(AckStatus, OutStatus.Status))
        {
            if (OutError.IsEmpty()) OutError = TEXT("control status identity/correlation is malformed");
            return false;
        }
        const bool bReasonRequired = OutStatus.Status != EControlAckStatus::Accepted
            && OutStatus.Status != EControlAckStatus::Confirmed;
        if (OutStatus.ServerStatusSequence == 0
            || !SameCorrelation(
                Binding,
                OutStatus.RunId,
                OutStatus.SessionId,
                OutStatus.BootId,
                OutStatus.ModelGeneration,
                OutStatus.ResetGeneration,
                OutStatus.SourceId,
                OutStatus.SourceEpoch,
                OutStatus.SourceSequence,
                OutStatus.EventId)
            || !IsLowerHex(OutStatus.IntentDatagramSha256, 64)
            || !ValidateReason(OutStatus.Reason, bReasonRequired, TEXT("status.reason"), OutError))
        {
            if (OutError.IsEmpty()) OutError = TEXT("control status identity or correlation is invalid");
            return false;
        }

        TSharedPtr<FJsonObject> Runtime;
        static const TSet<FString> RuntimeFields = {
            TEXT("runtime_state"), TEXT("control_owner"), TEXT("deadman"),
            TEXT("sample_age_ns"), TEXT("safe_stop_state")};
        FString RuntimeState;
        FString SafeStopState;
        if (!ReadObject(Root, TEXT("runtime"), Runtime, OutError)
            || !HasExactFields(Runtime, RuntimeFields, TEXT("runtime"), OutError)
            || !Runtime->TryGetStringField(TEXT("runtime_state"), RuntimeState)
            || !Runtime->TryGetStringField(TEXT("control_owner"), OutStatus.Runtime.ControlOwner)
            || !Runtime->TryGetBoolField(TEXT("deadman"), OutStatus.Runtime.bDeadman)
            || !ReadExactUint64(Runtime, TEXT("sample_age_ns"), OutStatus.Runtime.SampleAgeNs, OutError)
            || !Runtime->TryGetStringField(TEXT("safe_stop_state"), SafeStopState)
            || !ParseRuntimeState(RuntimeState, OutStatus.Runtime.RuntimeState)
            || !ParseSafeStopState(SafeStopState, OutStatus.Runtime.SafeStopState)
            || (OutStatus.Runtime.ControlOwner != TEXT("unavailable")
                && !IsCanonicalId(OutStatus.Runtime.ControlOwner)))
        {
            if (OutError.IsEmpty()) OutError = TEXT("control status runtime is malformed");
            return false;
        }

        TSharedPtr<FJsonObject> Motion;
        TSharedPtr<FJsonObject> Requested;
        TSharedPtr<FJsonObject> Admitted;
        TSharedPtr<FJsonObject> Observed;
        static const TSet<FString> MotionFields = {
            TEXT("requested_axes"), TEXT("admitted_twist_mps_radps"),
            TEXT("observed_base_velocity_mps_radps")};
        if (!ReadObject(Root, TEXT("motion"), Motion, OutError)
            || !HasExactFields(Motion, MotionFields, TEXT("motion"), OutError)
            || !ReadObject(Motion, TEXT("requested_axes"), Requested, OutError)
            || !ReadObject(Motion, TEXT("admitted_twist_mps_radps"), Admitted, OutError)
            || !ReadObject(Motion, TEXT("observed_base_velocity_mps_radps"), Observed, OutError)
            || !ParseRequestedAxes(Requested, OutStatus.Motion.RequestedAxes, OutError)
            || !ParseVelocity(Admitted, TEXT("motion.admitted_twist_mps_radps"), OutStatus.Motion.AdmittedTwist, OutError)
            || !ParseVelocity(Observed, TEXT("motion.observed_base_velocity_mps_radps"), OutStatus.Motion.ObservedBaseVelocity, OutError))
        {
            return false;
        }

        TSharedPtr<FJsonObject> Readiness;
        static const TSet<FString> ReadinessFields = {
            TEXT("physics"), TEXT("control"), TEXT("visual"), TEXT("sensors")};
        TSharedPtr<FJsonObject> Facet;
        if (!ReadObject(Root, TEXT("readiness"), Readiness, OutError)
            || !HasExactFields(Readiness, ReadinessFields, TEXT("readiness"), OutError)
            || !ReadObject(Readiness, TEXT("physics"), Facet, OutError)
            || !ParseReadinessFacet(Facet, TEXT("readiness.physics"), OutStatus.Readiness.Physics, OutError)
            || !ReadObject(Readiness, TEXT("control"), Facet, OutError)
            || !ParseReadinessFacet(Facet, TEXT("readiness.control"), OutStatus.Readiness.Control, OutError)
            || !ReadObject(Readiness, TEXT("visual"), Facet, OutError)
            || !ParseReadinessFacet(Facet, TEXT("readiness.visual"), OutStatus.Readiness.Visual, OutError)
            || !ReadObject(Readiness, TEXT("sensors"), Facet, OutError)
            || !ParseReadinessFacet(Facet, TEXT("readiness.sensors"), OutStatus.Readiness.Sensors, OutError))
        {
            return false;
        }

        const TArray<TSharedPtr<FJsonValue>>* SensorValues = nullptr;
        static const TArray<FString> ExpectedStreams = {
            TEXT("thunder_01.front_depth"), TEXT("thunder_01.front_rgb"),
            TEXT("thunder_01.imu"), TEXT("thunder_01.mid360"),
            TEXT("thunder_01.truth_odom")};
        static const TSet<FString> SensorFields = {
            TEXT("stream_id"), TEXT("state"), TEXT("sample_count"), TEXT("blocker")};
        if (!Root->TryGetArrayField(TEXT("sensors"), SensorValues)
            || SensorValues == nullptr
            || SensorValues->Num() != ExpectedStreams.Num())
        {
            OutError = TEXT("control status sensors must contain the exact five streams");
            return false;
        }
        OutStatus.Sensors.Reserve(ExpectedStreams.Num());
        for (int32 Index = 0; Index < ExpectedStreams.Num(); ++Index)
        {
            const TSharedPtr<FJsonObject> SensorObject = (*SensorValues)[Index]->AsObject();
            FControlStatusSensor Sensor;
            FString SensorState;
            if (!HasExactFields(SensorObject, SensorFields, TEXT("sensor"), OutError)
                || !SensorObject->TryGetStringField(TEXT("stream_id"), Sensor.StreamId)
                || Sensor.StreamId != ExpectedStreams[Index]
                || !SensorObject->TryGetStringField(TEXT("state"), SensorState)
                || !ParseSensorState(SensorState, Sensor.State)
                || !ReadExactUint64(SensorObject, TEXT("sample_count"), Sensor.SampleCount, OutError)
                || !SensorObject->TryGetStringField(TEXT("blocker"), Sensor.Blocker)
                || !ValidateReason(
                    Sensor.Blocker,
                    Sensor.State != EControlSensorState::Active || Sensor.SampleCount == 0,
                    TEXT("sensor.blocker"),
                    OutError))
            {
                if (OutError.IsEmpty()) OutError = TEXT("control status sensor is malformed or out of order");
                return false;
            }
            OutStatus.Sensors.Add(MoveTemp(Sensor));
        }

        TSharedPtr<FJsonObject> Recording;
        static const TSet<FString> RecordingFields = {
            TEXT("state"), TEXT("elapsed_sim_time_ns"), TEXT("artifact_id"), TEXT("blocker")};
        FString RecordingState;
        if (!ReadObject(Root, TEXT("recording"), Recording, OutError)
            || !HasExactFields(Recording, RecordingFields, TEXT("recording"), OutError)
            || !Recording->TryGetStringField(TEXT("state"), RecordingState)
            || !ParseRecordingState(RecordingState, OutStatus.Recording.State)
            || !ReadExactUint64(Recording, TEXT("elapsed_sim_time_ns"), OutStatus.Recording.ElapsedSimTimeNs, OutError)
            || !Recording->TryGetStringField(TEXT("artifact_id"), OutStatus.Recording.ArtifactId)
            || !Recording->TryGetStringField(TEXT("blocker"), OutStatus.Recording.Blocker))
        {
            if (OutError.IsEmpty()) OutError = TEXT("control status recording is malformed");
            return false;
        }
        const bool bRecordingBlocked =
            OutStatus.Recording.State == EControlRecordingState::Unavailable
            || OutStatus.Recording.State == EControlRecordingState::Rejected
            || OutStatus.Recording.State == EControlRecordingState::Failed;
        if ((OutStatus.Recording.State == EControlRecordingState::Unavailable
                && (OutStatus.Recording.ElapsedSimTimeNs != 0
                    || !OutStatus.Recording.ArtifactId.IsEmpty()))
            || (OutStatus.Recording.State == EControlRecordingState::Committed
                && OutStatus.Recording.ArtifactId.IsEmpty())
            || (!OutStatus.Recording.ArtifactId.IsEmpty()
                && !IsCanonicalId(OutStatus.Recording.ArtifactId))
            || !ValidateReason(
                OutStatus.Recording.Blocker,
                bRecordingBlocked,
                TEXT("recording.blocker"),
                OutError)
            || (!bRecordingBlocked && !OutStatus.Recording.Blocker.IsEmpty()))
        {
            if (OutError.IsEmpty()) OutError = TEXT("control status recording invariants failed");
            return false;
        }

        TSharedPtr<FJsonObject> UI;
        static const TSet<FString> UIFields = {TEXT("ui_mode"), TEXT("camera_mode")};
        FString UIMode;
        FString CameraMode;
        if (!ReadObject(Root, TEXT("ui"), UI, OutError)
            || !HasExactFields(UI, UIFields, TEXT("ui"), OutError)
            || !UI->TryGetStringField(TEXT("ui_mode"), UIMode)
            || !UI->TryGetStringField(TEXT("camera_mode"), CameraMode)
            || !ParseUIMode(UIMode, OutStatus.UI.UIMode)
            || !ParseCameraMode(CameraMode, OutStatus.UI.CameraMode))
        {
            if (OutError.IsEmpty()) OutError = TEXT("control status UI echo is malformed");
            return false;
        }
        return true;
    }

    bool FLingTuSimOperatorIntentSender::ValidateStatusAgainstSuccessfulSend(
        const FControlStatusEnvelope& Status,
        FString& OutError)
    {
        FScopeLock Lock(&CriticalSection);
        OutError.Reset();
        if (!SameCorrelation(
                Binding,
                Status.RunId,
                Status.SessionId,
                Status.BootId,
                Status.ModelGeneration,
                Status.ResetGeneration,
                Status.SourceId,
                Status.SourceEpoch,
                Status.SourceSequence,
                Status.EventId))
        {
            OutError = TEXT("control status identity does not match this sender");
            return false;
        }
        const FSuccessfulSendKey Key{
            Status.SourceEpoch,
            Status.SourceSequence,
            Status.EventId,
        };
        const FString* SuccessfulSendSha256 = SuccessfulSendSha256ByKey.Find(Key);
        if (SuccessfulSendSha256 == nullptr
            || *SuccessfulSendSha256 != Status.IntentDatagramSha256)
        {
            OutError = TEXT("control status does not match a successful control send");
            return false;
        }
        return true;
    }
}

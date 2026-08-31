#include "LingTuSimRuntimeUIStatus.h"

#include "Engine/World.h"
#include "HAL/PlatformTime.h"
#include "LingTuSimControlTransport.h"
#include "LingTuSimSessionService.h"
#include "LingTuSimVisualWorldSubsystem.h"

namespace LingTuSim::UI
{
    namespace
    {
        constexpr uint64 MaxHudControlStatusAgeNs = 100'000'000;

        bool IsLowerHexSha256(const FString& Value)
        {
            if (Value.Len() != 64)
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

        uint64 CurrentMonotonicNs()
        {
            const double Nanoseconds = FPlatformTime::Seconds() * 1'000'000'000.0;
            if (!FMath::IsFinite(Nanoseconds) || Nanoseconds <= 0.0)
            {
                return 0;
            }
            return static_cast<uint64>(Nanoseconds);
        }

        const TCHAR* UIModeName(const LingTuSim::EControlStatusUIMode Mode)
        {
            switch (Mode)
            {
            case LingTuSim::EControlStatusUIMode::Drive:
                return TEXT("drive");
            case LingTuSim::EControlStatusUIMode::Build:
                return TEXT("build");
            case LingTuSim::EControlStatusUIMode::Tactical:
                return TEXT("tactical");
            case LingTuSim::EControlStatusUIMode::Menu:
                return TEXT("menu");
            case LingTuSim::EControlStatusUIMode::Unavailable:
            default:
                return TEXT("unavailable");
            }
        }

        const TCHAR* CameraModeName(const LingTuSim::EControlStatusCameraMode Mode)
        {
            switch (Mode)
            {
            case LingTuSim::EControlStatusCameraMode::Follow:
                return TEXT("follow");
            case LingTuSim::EControlStatusCameraMode::Inspection:
                return TEXT("inspection");
            case LingTuSim::EControlStatusCameraMode::Free:
                return TEXT("free");
            case LingTuSim::EControlStatusCameraMode::Unavailable:
            default:
                return TEXT("unavailable");
            }
        }

        bool IsActiveRequiredFacet(const LingTuSim::FControlStatusReadinessFacet& Facet)
        {
            return Facet.bRequired
                && Facet.State == LingTuSim::EControlBindingState::Active
                && Facet.Blocker.IsEmpty();
        }

        bool HasExactActiveSensors(const TArray<LingTuSim::FControlStatusSensor>& Sensors)
        {
            static const TCHAR* RequiredStreamIds[] = {
                TEXT("front_rgb"),
                TEXT("front_depth"),
                TEXT("imu"),
                TEXT("mid360"),
                TEXT("truth_odom"),
            };
            if (Sensors.Num() != static_cast<int32>(UE_ARRAY_COUNT(RequiredStreamIds)))
            {
                return false;
            }
            for (int32 Index = 0; Index < Sensors.Num(); ++Index)
            {
                const LingTuSim::FControlStatusSensor& Sensor = Sensors[Index];
                if (Sensor.StreamId != RequiredStreamIds[Index]
                    || Sensor.State != LingTuSim::EControlSensorState::Active
                    || Sensor.SampleCount == 0
                    || !Sensor.Blocker.IsEmpty())
                {
                    return false;
                }
            }
            return true;
        }

        const TCHAR* AckStatusName(const LingTuSim::EControlAckStatus Status)
        {
            switch (Status)
            {
            case LingTuSim::EControlAckStatus::Pending:
                return TEXT("Pending");
            case LingTuSim::EControlAckStatus::Accepted:
                return TEXT("Accepted");
            case LingTuSim::EControlAckStatus::Rejected:
                return TEXT("Rejected");
            case LingTuSim::EControlAckStatus::Released:
                return TEXT("Released");
            case LingTuSim::EControlAckStatus::TimeoutZero:
                return TEXT("Timeout / zeroed");
            case LingTuSim::EControlAckStatus::Confirmed:
                return TEXT("Confirmed");
            default:
                return TEXT("Unavailable");
            }
        }

        bool IsBaseEntity(const LingTuSim::FEntityState& Entity)
        {
            return Entity.Id.FrameId.Equals(TEXT("base_link"), ESearchCase::IgnoreCase)
                || Entity.Id.StableId.EndsWith(TEXT("/base_link"), ESearchCase::IgnoreCase);
        }

        void ReadObservedBaseVelocity(
            const LingTuSim::FSnapshotEnvelope& Snapshot,
            FRuntimeUIStatusSnapshot& Status)
        {
            const LingTuSim::FEntityState* BaseEntity = nullptr;
            for (const LingTuSim::FEntityState& Entity : Snapshot.Entities)
            {
                if (!IsBaseEntity(Entity))
                {
                    continue;
                }
                if (BaseEntity != nullptr)
                {
                    Status.ObservedBaseVelocityState = TEXT("Unavailable: multiple base_link entities");
                    return;
                }
                BaseEntity = &Entity;
            }

            if (BaseEntity == nullptr)
            {
                Status.ObservedBaseVelocityState = TEXT("Unavailable: base_link absent");
                return;
            }
            Status.bObservedBaseVelocityAvailable = true;
            Status.ObservedBaseStableId = BaseEntity->Id.StableId;
            Status.ObservedBaseVelocityState = TEXT("Observed truth");
            Status.ObservedBaseLinearVelocityMps = BaseEntity->LinearVelocityMetersPerSecond;
            Status.ObservedBaseAngularVelocityRadps =
                BaseEntity->AngularVelocityRadiansPerSecond;
        }
    }

    FRuntimeUIStatusSnapshot FRuntimeUIStatusReader::Read(
        UWorld* World,
        const FRuntimeUILocalState* LocalState)
    {
        FRuntimeUIStatusSnapshot Status;
        if (LocalState != nullptr)
        {
            Status.bInputObserved = LocalState->bInputObserved;
            Status.RequestedInput = LocalState->RequestedInput;
            Status.ActualUIMode = LocalState->ActualUIMode;
            Status.ActualCameraMode = LocalState->ActualCameraMode;
            Status.LatestRuntimeRequest = LocalState->LatestRuntimeRequest;
            Status.LatestRuntimeRequestEventId = LocalState->LatestRuntimeRequestEventId;
            Status.LatestRuntimeRequestError = LocalState->LatestRuntimeRequestError;
            Status.bLatestRuntimeRequestPublished =
                LocalState->bLatestRuntimeRequestPublished;
        }

        if (!LingTuSim::FSessionService::GetBoundSession(
                Status.SessionId,
                Status.ModelGeneration))
        {
            return Status;
        }

        Status.bSessionAvailable = true;
        Status.SessionState = TEXT("Bound");
        Status.Blocker.Reset();

        LingTuSim::FControlTransportBinding ControlBinding;
        if (LingTuSim::FSessionService::GetControlTransportBinding(ControlBinding))
        {
            Status.bControlBindingAvailable = true;
            Status.ControlState = TEXT("Bound");
            Status.RunId = ControlBinding.RunId;
            Status.ResetGeneration = ControlBinding.ResetGeneration;
            Status.bIdentityCoherent =
                ControlBinding.SessionId == Status.SessionId
                && ControlBinding.ModelGeneration == Status.ModelGeneration;
            if (!Status.bIdentityCoherent)
            {
                Status.ControlState = TEXT("Blocked: identity mismatch");
                Status.Blocker = TEXT("Session and Control identity disagree");
            }
        }
        else
        {
            Status.bIdentityCoherent = true;
            Status.Blocker = TEXT("Control binding unavailable");
        }

        LingTuSim::FControlAckEnvelope Ack;
        if (LingTuSim::FSessionService::GetLatestControlAck(Ack))
        {
            Status.bControlAckAvailable = true;
            Status.ControlAckStatus = Ack.Status;
            Status.bControlAckAccepted =
                Ack.Status == LingTuSim::EControlAckStatus::Accepted
                || Ack.Status == LingTuSim::EControlAckStatus::Confirmed;
            Status.ControlAckState = AckStatusName(Ack.Status);
            Status.ControlAckReason = Ack.Reason;
            Status.ControlAckEventId = Ack.EventId;
            Status.ControlAckSourceSequence = Ack.SourceSequence;
            Status.bAckMatchesLatestRuntimeRequest =
                !Status.LatestRuntimeRequestEventId.IsEmpty()
                && Ack.EventId == Status.LatestRuntimeRequestEventId;
        }

        LingTuSim::FControlStatusEnvelope FullStatus;
        if (LingTuSim::FSessionService::GetLatestControlStatus(FullStatus))
        {
            Status.bFullStatusAvailable = true;
            Status.FullStatus = FullStatus;
            const uint64 NowNs = CurrentMonotonicNs();
            Status.bFullStatusFresh = FullStatus.IsFresh(
                NowNs,
                MaxHudControlStatusAgeNs);
            Status.FullStatusAgeNs = NowNs >= FullStatus.ReceivedMonotonicNs
                ? NowNs - FullStatus.ReceivedMonotonicNs
                : MAX_uint64;
            Status.bFullStatusIdentityCoherent =
                Status.bControlBindingAvailable
                && FullStatus.RunId == Status.RunId
                && FullStatus.SessionId == Status.SessionId
                && FullStatus.BootId == ControlBinding.BootId
                && FullStatus.SourceId == ControlBinding.SourceId
                && FullStatus.ModelGeneration == Status.ModelGeneration
                && FullStatus.ResetGeneration == Status.ResetGeneration;
            if (!Status.bFullStatusIdentityCoherent)
            {
                Status.FullStatusState = TEXT("Blocked: identity mismatch");
                Status.FullStatusBlocker = TEXT("Full status identity disagrees with Session/Control");
            }
            else if (!Status.bFullStatusFresh)
            {
                Status.FullStatusState = TEXT("Blocked: stale");
                Status.FullStatusBlocker = TEXT("Full status is older than 100 ms");
            }
            else
            {
                Status.FullStatusState = TEXT("Current");
                Status.FullStatusBlocker.Reset();
            }
        }

        if (World == nullptr)
        {
            if (Status.Blocker.IsEmpty())
            {
                Status.Blocker = TEXT("Visual world unavailable");
            }
            return Status;
        }

        const ULingTuSimVisualWorldSubsystem* Visual =
            World->GetSubsystem<ULingTuSimVisualWorldSubsystem>();
        if (Visual == nullptr)
        {
            if (Status.Blocker.IsEmpty())
            {
                Status.Blocker = TEXT("Visual subsystem unavailable");
            }
            return Status;
        }

        Status.BodyBindingCount = Visual->GetRegisteredBindingCount();
        Status.ScenarioActorCount = Visual->GetScenarioActorCount();
        if (Visual->IsWaitingForRebind())
        {
            Status.VisualState = TEXT("Waiting for rebind");
            Status.Blocker = TEXT("Visual is waiting for a session rebind");
            return Status;
        }

        LingTuSim::FSnapshotEnvelope LatestApplied;
        if (!Visual->GetLatestAppliedSnapshot(LatestApplied))
        {
            Status.VisualState = Status.BodyBindingCount > 0 || Status.ScenarioActorCount > 0
                ? TEXT("Preparing")
                : TEXT("Unavailable");
            if (Status.Blocker.IsEmpty())
            {
                Status.Blocker = TEXT("No truth snapshot has been applied to Visual");
            }
            return Status;
        }

        Status.bLatestAppliedTruthAvailable = true;
        Status.TruthSequence = LatestApplied.Sequence;
        Status.SimTimeNs = LatestApplied.SimTimeNs;
        Status.VisualState = TEXT("Active");
        ReadObservedBaseVelocity(LatestApplied, Status);

        const bool bTruthMatchesSession =
            LatestApplied.SessionId == Status.SessionId
            && LatestApplied.ModelGeneration == Status.ModelGeneration;
        const bool bTruthMatchesControl =
            !Status.bControlBindingAvailable
            || LatestApplied.ResetGeneration == Status.ResetGeneration;
        if (!bTruthMatchesSession || !bTruthMatchesControl)
        {
            Status.bIdentityCoherent = false;
            Status.VisualState = TEXT("Blocked: identity mismatch");
            Status.Blocker = TEXT("Latest applied Visual truth has mismatched identity");
            return Status;
        }

        if (Status.bFullStatusAvailable
            && Status.bFullStatusIdentityCoherent
            && (Status.FullStatus.TruthSequence != Status.TruthSequence
                || Status.FullStatus.SimTimeNs != static_cast<uint64>(Status.SimTimeNs)))
        {
            Status.bFullStatusIdentityCoherent = false;
            Status.FullStatusState = TEXT("Blocked: truth mismatch");
            Status.FullStatusBlocker = TEXT("Full status and latest applied Visual truth disagree");
            Status.Blocker = Status.FullStatusBlocker;
            return Status;
        }

        if (!Status.bFullStatusAvailable
            || !Status.bFullStatusFresh
            || !Status.bFullStatusIdentityCoherent)
        {
            Status.Blocker = Status.FullStatusBlocker;
            return Status;
        }

        if (Status.Blocker == TEXT("Control binding unavailable"))
        {
            return Status;
        }
        if (!Status.bIdentityCoherent)
        {
            return Status;
        }
        Status.Blocker.Reset();
        return Status;
    }

    bool FRuntimeUIStatusSnapshot::CanRequestRecordStart() const
    {
        return bFullStatusAvailable
            && bFullStatusFresh
            && bFullStatusIdentityCoherent
            && FullStatus.Recording.State == LingTuSim::EControlRecordingState::Idle;
    }

    bool FRuntimeUIStatusSnapshot::CanRequestRecordStopCommit() const
    {
        return bFullStatusAvailable
            && bFullStatusFresh
            && bFullStatusIdentityCoherent
            && FullStatus.Recording.State == LingTuSim::EControlRecordingState::Recording;
    }

    bool FRuntimeUIStatusSnapshot::IsFullStatusCaptureReady(
        const FString& ExpectedUIMode,
        const bool bRequireRecording) const
    {
        if (!bSessionAvailable
            || !bControlBindingAvailable
            || !bIdentityCoherent
            || VisualState != TEXT("Active")
            || !bLatestAppliedTruthAvailable
            || !bObservedBaseVelocityAvailable
            || !bFullStatusAvailable
            || !bFullStatusFresh
            || !bFullStatusIdentityCoherent
            || (FullStatus.Status != LingTuSim::EControlAckStatus::Accepted
                && FullStatus.Status != LingTuSim::EControlAckStatus::Confirmed)
            || !FullStatus.Motion.AdmittedTwist.bAvailable
            || !FullStatus.Motion.ObservedBaseVelocity.bAvailable
            || !IsActiveRequiredFacet(FullStatus.Readiness.Physics)
            || !IsActiveRequiredFacet(FullStatus.Readiness.Control)
            || !IsActiveRequiredFacet(FullStatus.Readiness.Visual)
            || !IsActiveRequiredFacet(FullStatus.Readiness.Sensors)
            || !HasExactActiveSensors(FullStatus.Sensors))
        {
            return false;
        }
        const FString StatusUIMode = UIModeName(FullStatus.UI.UIMode);
        const FString StatusCameraMode = CameraModeName(FullStatus.UI.CameraMode);
        if (ActualUIMode != ExpectedUIMode
            || StatusUIMode != ActualUIMode
            || ActualCameraMode == TEXT("unavailable")
            || StatusCameraMode != ActualCameraMode)
        {
            return false;
        }
        return !bRequireRecording
            || FullStatus.Recording.State == LingTuSim::EControlRecordingState::Recording;
    }

    bool FRuntimeUIExitPolicy::CanRequestEngineExit(
        const FString& PendingExitEventId,
        const bool bEngineExitAlreadyRequested,
        const FRuntimeUIStatusSnapshot& Status,
        FString& OutBlocker)
    {
        OutBlocker.Reset();
        if (bEngineExitAlreadyRequested)
        {
            OutBlocker = TEXT("engine_exit_already_requested");
            return false;
        }
        if (PendingExitEventId.IsEmpty())
        {
            OutBlocker = TEXT("exit_event_not_published");
            return false;
        }
        if (!Status.bFullStatusAvailable
            || !Status.bFullStatusFresh
            || !Status.bFullStatusIdentityCoherent)
        {
            OutBlocker = TEXT("exit_full_status_unavailable_stale_or_incoherent");
            return false;
        }
        const LingTuSim::FControlStatusEnvelope& Full = Status.FullStatus;
        const FString CorrelatedEventId = FString::Printf(
            TEXT("%s:%llu:%llu"),
            *Full.BootId,
            static_cast<unsigned long long>(Full.SourceEpoch),
            static_cast<unsigned long long>(Full.SourceSequence));
        // GetLatestControlStatus exposes only successful-send-ledger validated
        // statuses. Rechecking the encoded epoch/sequence and SHA shape keeps
        // this pure policy fail-closed when used with hand-built snapshots.
        if (Full.SourceEpoch == 0
            || Full.SourceSequence == 0
            || Full.EventId != CorrelatedEventId
            || !IsLowerHexSha256(Full.IntentDatagramSha256))
        {
            OutBlocker = TEXT("exit_successful_send_correlation_invalid");
            return false;
        }
        if (Full.EventId != PendingExitEventId)
        {
            OutBlocker = TEXT("exit_event_correlation_mismatch");
            return false;
        }
        if (Full.Status != LingTuSim::EControlAckStatus::Confirmed)
        {
            OutBlocker = TEXT("exit_not_confirmed");
            return false;
        }
        if (Full.Runtime.RuntimeState != LingTuSim::EControlStatusRuntimeState::Stopped)
        {
            OutBlocker = TEXT("runtime_not_stopped");
            return false;
        }
        if (!Full.Runtime.ControlOwner.Equals(TEXT("unavailable"), ESearchCase::CaseSensitive)
            || Full.Runtime.bDeadman)
        {
            OutBlocker = TEXT("terminal_control_ownership_not_released");
            return false;
        }
        if (Full.Runtime.SafeStopState != LingTuSim::EControlSafeStopState::Zeroed)
        {
            OutBlocker = TEXT("safe_stop_not_zeroed");
            return false;
        }
        const LingTuSim::FControlStatusRequestedAxes& Requested =
            Full.Motion.RequestedAxes;
        if (Requested.bAvailable
            || Requested.Forward != 0.0
            || Requested.Left != 0.0
            || Requested.YawLeft != 0.0)
        {
            OutBlocker = TEXT("terminal_requested_axes_not_unavailable_zero");
            return false;
        }
        const LingTuSim::FControlStatusVelocity& Admitted = Full.Motion.AdmittedTwist;
        if (!Admitted.bAvailable
            || Admitted.LinearX != 0.0
            || Admitted.LinearY != 0.0
            || Admitted.AngularZ != 0.0)
        {
            OutBlocker = TEXT("admitted_twist_not_exact_zero");
            return false;
        }
        const FString EchoedUIMode = UIModeName(Full.UI.UIMode);
        const FString EchoedCameraMode = CameraModeName(Full.UI.CameraMode);
        if (Status.ActualUIMode != TEXT("menu")
            || EchoedUIMode != Status.ActualUIMode
            || Status.ActualCameraMode == TEXT("unavailable")
            || EchoedCameraMode != Status.ActualCameraMode)
        {
            OutBlocker = TEXT("terminal_ui_or_camera_echo_mismatch");
            return false;
        }
        return true;
    }
}

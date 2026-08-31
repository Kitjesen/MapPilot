#pragma once

#include "CoreMinimal.h"
#include "LingTuSimControlTransport.h"
#include "LingTuSimRobotDriveInput.h"

class UWorld;

namespace LingTuSim::UI
{
    /** UI-owned facts that are not transport or simulator authority. */
    struct LINGTUSIMUI_API FRuntimeUILocalState final
    {
        bool bInputObserved = false;
        FRobotDriveInputSnapshot RequestedInput;
        FString ActualUIMode = TEXT("drive");
        FString ActualCameraMode = TEXT("unavailable");
        FString LatestRuntimeRequest;
        FString LatestRuntimeRequestEventId;
        FString LatestRuntimeRequestError;
        bool bLatestRuntimeRequestPublished = false;
    };

    /**
     * One copied, read-only HUD view. Missing public runtime facts stay
     * unavailable; requested input, ACK state, and observed truth never collapse.
     */
    struct LINGTUSIMUI_API FRuntimeUIStatusSnapshot final
    {
        bool bSessionAvailable = false;
        FString SessionState = TEXT("Unavailable");
        FString SessionId;
        uint64 ModelGeneration = 0;

        bool bControlBindingAvailable = false;
        FString ControlState = TEXT("Unavailable");
        FString RunId;
        uint64 ResetGeneration = 0;

        bool bIdentityCoherent = false;
        FString Blocker = TEXT("Session binding unavailable");

        FString VisualState = TEXT("Unavailable");
        int32 BodyBindingCount = 0;
        int32 ScenarioActorCount = 0;

        bool bLatestAppliedTruthAvailable = false;
        uint64 TruthSequence = 0;
        int64 SimTimeNs = 0;
        bool bObservedBaseVelocityAvailable = false;
        FString ObservedBaseStableId;
        FString ObservedBaseVelocityState = TEXT("Unavailable");
        FVector ObservedBaseLinearVelocityMps = FVector::ZeroVector;
        FVector ObservedBaseAngularVelocityRadps = FVector::ZeroVector;

        bool bInputObserved = false;
        FRobotDriveInputSnapshot RequestedInput;

        bool bControlAckAvailable = false;
        LingTuSim::EControlAckStatus ControlAckStatus =
            LingTuSim::EControlAckStatus::Rejected;
        bool bControlAckAccepted = false;
        FString ControlAckState = TEXT("Unavailable");
        FString ControlAckReason;
        FString ControlAckEventId;
        uint64 ControlAckSourceSequence = 0;
        bool bAckMatchesLatestRuntimeRequest = false;

        bool bFullStatusAvailable = false;
        bool bFullStatusFresh = false;
        bool bFullStatusIdentityCoherent = false;
        uint64 FullStatusAgeNs = 0;
        FString FullStatusState = TEXT("Unavailable");
        FString FullStatusBlocker = TEXT("Full control status unavailable");
        LingTuSim::FControlStatusEnvelope FullStatus;

        FString ActualUIMode = TEXT("drive");
        FString ActualCameraMode = TEXT("unavailable");

        FString LatestRuntimeRequest;
        FString LatestRuntimeRequestEventId;
        FString LatestRuntimeRequestError;
        bool bLatestRuntimeRequestPublished = false;

        bool IsDriveCaptureReady() const
        {
            return IsFullStatusCaptureReady(TEXT("drive"), false)
                && bInputObserved
                && FullStatus.Motion.RequestedAxes.bAvailable;
        }

        bool IsTacticalCaptureReady() const
        {
            return IsFullStatusCaptureReady(TEXT("tactical"), false);
        }

        bool IsMenuRecordingCaptureReady() const
        {
            return IsFullStatusCaptureReady(TEXT("menu"), true);
        }

        bool CanRequestRecordStart() const;
        bool CanRequestRecordStopCommit() const;
        bool IsFullStatusCaptureReady(
            const FString& ExpectedUIMode,
            bool bRequireRecording) const;
    };

    // Compatibility name retained for existing callers while the HUD moves to one snapshot.
    using FRuntimeUIStatus = FRuntimeUIStatusSnapshot;

    class LINGTUSIMUI_API FRuntimeUIStatusReader final
    {
    public:
        static FRuntimeUIStatusSnapshot Read(
            UWorld* World,
            const FRuntimeUILocalState* LocalState = nullptr);

    private:
        FRuntimeUIStatusReader() = delete;
    };

    /** Pure fail-closed gate for the one normal UE process-exit request. */
    class LINGTUSIMUI_API FRuntimeUIExitPolicy final
    {
    public:
        static bool CanRequestEngineExit(
            const FString& PendingExitEventId,
            bool bEngineExitAlreadyRequested,
            const FRuntimeUIStatusSnapshot& Status,
            FString& OutBlocker);

    private:
        FRuntimeUIExitPolicy() = delete;
    };
}

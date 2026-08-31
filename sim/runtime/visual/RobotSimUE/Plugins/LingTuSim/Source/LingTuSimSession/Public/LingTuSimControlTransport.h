#pragma once

#include "CoreMinimal.h"

namespace LingTuSim
{
    enum class EOperatorRuntimeRequestType : uint8
    {
        ControlClaim,
        ControlRelease,
        Pause,
        Resume,
        RecordStart,
        RecordStopCommit,
        SafeStop,
        Exit,
        UIStateUpdate,
    };

    enum class EControlAckStatus : uint8
    {
        Pending,
        Accepted,
        Rejected,
        Released,
        TimeoutZero,
        Confirmed,
    };

    enum class EControlStatusRuntimeState : uint8
    {
        New,
        Preparing,
        Ready,
        Running,
        Paused,
        Stopped,
        Failed,
    };

    enum class EControlSafeStopState : uint8
    {
        Unavailable,
        Clear,
        Pending,
        Zeroed,
        Blocked,
    };

    enum class EControlBindingState : uint8
    {
        Unavailable,
        Unbound,
        Prepared,
        Active,
        Failed,
    };

    enum class EControlSensorState : uint8
    {
        Unavailable,
        Missing,
        Unbound,
        Prepared,
        Active,
        Failed,
    };

    enum class EControlRecordingState : uint8
    {
        Unavailable,
        Idle,
        Requested,
        Recording,
        Committed,
        Rejected,
        Failed,
    };

    enum class EControlStatusUIMode : uint8
    {
        Unavailable,
        Drive,
        Build,
        Tactical,
        Menu,
    };

    enum class EControlStatusCameraMode : uint8
    {
        Unavailable,
        Follow,
        Inspection,
        Free,
    };

    /** Allocation-owned identity for the loopback UE control adapter. */
    struct LINGTUSIMSESSION_API FControlTransportBinding
    {
        FString RunId;
        FString SessionId;
        FString BootId;
        FString SourceId;
        FString LogDirectory;
        FString OriginEvidencePath;
        uint64 ModelGeneration = 0;
        uint64 ResetGeneration = 0;
        uint16 IntentPort = 0;
        uint16 StatusPort = 0;

        bool IsValid() const;
    };

    /** UI-owned values. Session adds allocation identity, epoch, sequence, and event ID. */
    struct LINGTUSIMSESSION_API FOperatorIntentSample
    {
        FString InputMode = TEXT("drive");
        FString UIMode = TEXT("drive");
        FString CameraMode = TEXT("unavailable");
        FString InputDevice = TEXT("keyboard");
        bool bViewportFocused = false;
        bool bDeadman = false;
        double Forward = 0.0;
        double Left = 0.0;
        double YawLeft = 0.0;

        // Camera axes are intentionally not serialized into robot motion axes.
        double CameraYaw = 0.0;
        double CameraPitch = 0.0;

        TArray<FString> ActiveControls;
        uint64 SourceMonotonicNs = 0;
    };

    struct LINGTUSIMSESSION_API FOperatorRuntimeRequest
    {
        EOperatorRuntimeRequestType Request = EOperatorRuntimeRequestType::SafeStop;
        FString UIMode = TEXT("drive");
        FString CameraMode = TEXT("unavailable");
        uint64 SourceMonotonicNs = 0;
    };

    /** Minimal correlated response; the complete runtime status uses a later schema. */
    struct LINGTUSIMSESSION_API FControlAckEnvelope
    {
        FString RunId;
        FString SessionId;
        FString BootId;
        FString SourceId;
        FString EventId;
        FString IntentDatagramSha256;
        FString Reason;
        uint64 ModelGeneration = 0;
        uint64 ResetGeneration = 0;
        uint64 ServerStatusSequence = 0;
        uint64 SourceEpoch = 0;
        uint64 SourceSequence = 0;
        EControlAckStatus Status = EControlAckStatus::Rejected;
    };

    struct LINGTUSIMSESSION_API FControlStatusRequestedAxes
    {
        bool bAvailable = false;
        double Forward = 0.0;
        double Left = 0.0;
        double YawLeft = 0.0;
    };

    struct LINGTUSIMSESSION_API FControlStatusVelocity
    {
        bool bAvailable = false;
        double LinearX = 0.0;
        double LinearY = 0.0;
        double AngularZ = 0.0;
    };

    struct LINGTUSIMSESSION_API FControlStatusRuntime
    {
        EControlStatusRuntimeState RuntimeState = EControlStatusRuntimeState::New;
        FString ControlOwner = TEXT("unavailable");
        bool bDeadman = false;
        uint64 SampleAgeNs = 0;
        EControlSafeStopState SafeStopState = EControlSafeStopState::Unavailable;
    };

    struct LINGTUSIMSESSION_API FControlStatusMotion
    {
        FControlStatusRequestedAxes RequestedAxes;
        FControlStatusVelocity AdmittedTwist;
        FControlStatusVelocity ObservedBaseVelocity;
    };

    struct LINGTUSIMSESSION_API FControlStatusReadinessFacet
    {
        EControlBindingState State = EControlBindingState::Unavailable;
        bool bRequired = false;
        FString SourceId = TEXT("unavailable");
        FString Blocker;
    };

    struct LINGTUSIMSESSION_API FControlStatusReadiness
    {
        FControlStatusReadinessFacet Physics;
        FControlStatusReadinessFacet Control;
        FControlStatusReadinessFacet Visual;
        FControlStatusReadinessFacet Sensors;
    };

    struct LINGTUSIMSESSION_API FControlStatusSensor
    {
        FString StreamId;
        EControlSensorState State = EControlSensorState::Unavailable;
        uint64 SampleCount = 0;
        FString Blocker;
    };

    struct LINGTUSIMSESSION_API FControlStatusRecording
    {
        EControlRecordingState State = EControlRecordingState::Unavailable;
        uint64 ElapsedSimTimeNs = 0;
        FString ArtifactId;
        FString Blocker;
    };

    struct LINGTUSIMSESSION_API FControlStatusUIEcho
    {
        EControlStatusUIMode UIMode = EControlStatusUIMode::Unavailable;
        EControlStatusCameraMode CameraMode = EControlStatusCameraMode::Unavailable;
    };

    /** Exact correlated full status, received only after successful-send validation. */
    struct LINGTUSIMSESSION_API FControlStatusEnvelope
    {
        FString RunId;
        FString SessionId;
        FString BootId;
        FString SourceId;
        FString EventId;
        FString IntentDatagramSha256;
        FString Reason;
        uint64 ModelGeneration = 0;
        uint64 ResetGeneration = 0;
        uint64 ServerStatusSequence = 0;
        uint64 ServerMonotonicNs = 0;
        uint64 ReceivedMonotonicNs = 0;
        uint64 SimTimeNs = 0;
        uint64 TruthSequence = 0;
        uint64 SourceEpoch = 0;
        uint64 SourceSequence = 0;
        EControlAckStatus Status = EControlAckStatus::Rejected;
        FControlStatusRuntime Runtime;
        FControlStatusMotion Motion;
        FControlStatusReadiness Readiness;
        TArray<FControlStatusSensor> Sensors;
        FControlStatusRecording Recording;
        FControlStatusUIEcho UI;

        bool IsFresh(uint64 NowMonotonicNs, uint64 MaxAgeNs) const;
    };

    /** Pure allocation/protocol functions used by module startup and Automation. */
    class LINGTUSIMSESSION_API FControlTransportProtocol final
    {
    public:
        static bool ParseRunAllocationJson(
            const FString& AllocationJson,
            const FString& ExpectedRunId,
            const FString& ExpectedSessionId,
            const FString& ExpectedLogDirectory,
            uint16 ExpectedIntentPort,
            uint16 ExpectedStatusPort,
            uint64 ModelGeneration,
            uint64 ResetGeneration,
            const FString& SourceId,
            FControlTransportBinding& OutBinding,
            FString& OutError);

        /** Classifies viewer versus playable control without ignoring orphan control argv. */
        static bool ValidateControlCommandLineShape(
            const FString& CommandLine,
            bool& bOutControlRequested,
            FString& OutError);

        static bool SerializeOperatorIntent(
            const FControlTransportBinding& Binding,
            const FOperatorIntentSample& Sample,
            uint64 SourceEpoch,
            uint64 SourceSequence,
            TArray<uint8>& OutDatagram,
            FString& OutEventId,
            FString& OutError);

        static bool SerializeRuntimeRequest(
            const FControlTransportBinding& Binding,
            const FOperatorRuntimeRequest& Request,
            uint64 SourceEpoch,
            uint64 SourceSequence,
            TArray<uint8>& OutDatagram,
            FString& OutEventId,
            FString& OutError);

        static bool ParseControlAckJson(
            const FControlTransportBinding& Binding,
            const FString& AckJson,
            FControlAckEnvelope& OutAck,
            FString& OutError);

        static bool ParseControlStatusJson(
            const FControlTransportBinding& Binding,
            const FString& StatusJson,
            FControlStatusEnvelope& OutStatus,
            FString& OutError);

        static bool Sha256Hex(
            const TArray<uint8>& Datagram,
            FString& OutSha256,
            FString& OutError);

        static bool BuildOriginEvidenceJson(
            const FControlTransportBinding& Binding,
            const FString& EventId,
            uint64 SourceEpoch,
            uint64 SourceSequence,
            const FString& DatagramSha256,
            int32 DatagramBytes,
            FString& OutEvidenceJson,
            FString& OutError);

    private:
        FControlTransportProtocol() = delete;
    };

    /**
     * Serializes, sends, then appends UE-origin evidence. Evidence is never
     * appended before a complete datagram send succeeds.
     */
    class LINGTUSIMSESSION_API FLingTuSimOperatorIntentSender final
    {
    public:
        using FSendDatagram = TFunction<bool(const TArray<uint8>&, FString&)>;
        using FAppendOriginEvidence = TFunction<bool(const FString&, FString&)>;

        FLingTuSimOperatorIntentSender(
            FControlTransportBinding InBinding,
            FSendDatagram InSendDatagram,
            FAppendOriginEvidence InAppendOriginEvidence);

        bool PublishOperatorIntent(
            const FOperatorIntentSample& Sample,
            FString& OutEventId,
            FString& OutError);

        bool PublishRuntimeRequest(
            const FOperatorRuntimeRequest& Request,
            FString& OutEventId,
            FString& OutError);

        /** Rejects ACKs that do not name the exact bytes of a successful local send. */
        bool ValidateAckAgainstSuccessfulSend(
            const FControlAckEnvelope& Ack,
            FString& OutError);

        /** Rejects full status that does not name exact bytes sent by this sender. */
        bool ValidateStatusAgainstSuccessfulSend(
            const FControlStatusEnvelope& Status,
            FString& OutError);

        const FControlTransportBinding& GetBinding() const { return Binding; }

    private:
        struct FSuccessfulSendKey
        {
            uint64 SourceEpoch = 0;
            uint64 SourceSequence = 0;
            FString EventId;

            bool operator==(const FSuccessfulSendKey& Other) const
            {
                return SourceEpoch == Other.SourceEpoch
                    && SourceSequence == Other.SourceSequence
                    && EventId == Other.EventId;
            }

            friend uint32 GetTypeHash(const FSuccessfulSendKey& Key)
            {
                return HashCombineFast(
                    ::GetTypeHash(Key.SourceEpoch),
                    HashCombineFast(
                        ::GetTypeHash(Key.SourceSequence),
                        GetTypeHash(Key.EventId)));
            }
        };

        bool SendAndRecord(
            const TArray<uint8>& Datagram,
            const FString& EventId,
            uint64 SourceSequence,
            FString& OutError);

        void RememberSuccessfulSend(
            const FString& EventId,
            uint64 SourceSequence,
            const FString& DatagramSha256);

        FControlTransportBinding Binding;
        FSendDatagram SendDatagram;
        FAppendOriginEvidence AppendOriginEvidence;
        FCriticalSection CriticalSection;
        uint64 SourceEpoch = 1;
        uint64 NextSourceSequence = 1;
        TMap<FSuccessfulSendKey, FString> SuccessfulSendSha256ByKey;
        TArray<FSuccessfulSendKey> SuccessfulSendOrder;
    };
}

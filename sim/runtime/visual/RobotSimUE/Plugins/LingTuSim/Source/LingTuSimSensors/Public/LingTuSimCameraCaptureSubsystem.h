#pragma once

#include "CoreMinimal.h"
#include "Engine/EngineTypes.h"
#include "LingTuSimRuntimeTypes.h"
#include "Subsystems/WorldSubsystem.h"

#include "LingTuSimCameraCaptureSubsystem.generated.h"

struct FEngineShowFlags;

namespace LingTuSim::Sensors::Camera
{
    enum class EStreamKind : uint8
    {
        Color,
        Depth,
    };

    struct LINGTUSIMSENSORS_API FStreamPlan
    {
        FString SensorId;
        FString FrameId;
        FString ParentFrameId;
        FString SessionId;
        FString Encoding;
        FString Source;
        FString Transport;
        EStreamKind Kind = EStreamKind::Color;
        int32 Width = 0;
        int32 Height = 0;
        double RateHz = 0.0;
        double DepthScale = 0.001;
        FVector ExtrinsicTranslationMeters = FVector::ZeroVector;
        FQuat ExtrinsicRotation = FQuat::Identity;
        bool bHasExtrinsic = false;
    };

    struct LINGTUSIMSENSORS_API FCameraPlan
    {
        FString SessionId;
        int32 TotalStreamCount = 0;
        TArray<FStreamPlan> Streams;
    };

    struct LINGTUSIMSENSORS_API FSharedColorDepthPair
    {
        int32 ColorStreamIndex = INDEX_NONE;
        int32 DepthStreamIndex = INDEX_NONE;
    };

    struct LINGTUSIMSENSORS_API FRunAllocation
    {
        FString RunId;
        FString SessionId;
        FString EvidenceDirectory;
        TMap<FString, FString> SharedMemoryBySensor;
    };

    struct LINGTUSIMSENSORS_API FStreamReadiness
    {
        FString SensorId;
        FString State;
        uint64 PublishedFrames = 0;
        uint64 LastSampleTruthSequence = 0;
        int64 LastSampleSimTimeNs = 0;
        FString Reason;
    };

    /** Identity and simulation clock copied from one coherent applied Visual frame. */
    struct LINGTUSIMSENSORS_API FTruthSampleStamp
    {
        FString SessionId;
        uint64 ModelGeneration = 0;
        uint64 ResetGeneration = 0;
        uint64 TruthSequence = 0;
        int64 SimTimeNs = 0;
    };

    enum class EDeferredCaptureAction : uint8
    {
        Wait,
        Promote,
        Discard,
        Fail,
    };

    enum class EReadbackDeadlineStage : uint8
    {
        EnqueuePending,
        GpuPending,
        CpuDecodePending,
        CpuReady,
    };

    enum class EReadbackDeadlineAction : uint8
    {
        Wait,
        Complete,
        Fail,
    };

    /** Freezes the startup or steady-state budget when a capture job is created. */
    LINGTUSIMSENSORS_API double SelectCapturePipelineTimeoutSeconds(
        uint64 PublishedFrames,
        double StartupTimeoutSeconds,
        double SteadyTimeoutSeconds);

    /**
     * Evaluates one readback stage without touching UObject or RHI state.
     * Deferred, render-command enqueue, and GPU-pending are separate bounded
     * stages; each receives the job's frozen startup or steady-state budget.
     */
    LINGTUSIMSENSORS_API EReadbackDeadlineAction EvaluateReadbackStageDeadline(
        EReadbackDeadlineStage Stage,
        double StageStartedAtSeconds,
        double NowSeconds,
        double TimeoutSeconds,
        FString& OutError);

    /**
     * Decides whether an end-of-frame capture may be copied to an asynchronous
     * GPU readback. A capture is never promoted in its submission frame, across
     * a runtime identity boundary, or after its bounded pipeline deadline.
     */
    LINGTUSIMSENSORS_API EDeferredCaptureAction EvaluateDeferredCapturePromotion(
        const FTruthSampleStamp& DeferredTruth,
        const FString& ExpectedSessionId,
        uint64 ExpectedModelGeneration,
        uint64 ExpectedResetGeneration,
        uint64 SubmittedFrame,
        uint64 CurrentFrame,
        double SubmittedAtSeconds,
        double NowSeconds,
        int32 PendingReadbackCount,
        int32 MaxPendingReadbackCount,
        double TimeoutSeconds,
        FString& OutError);

    /** Main-renderer integration is an explicit depth/SceneDepth-only option. */
    LINGTUSIMSENSORS_API bool ShouldRenderCameraCaptureInMainRenderer(
        EStreamKind StreamKind,
        ESceneCaptureSource CaptureSource,
        bool bDepthOptionEnabled);

    /** Removes lighting-only work from geometric depth captures without changing RGB. */
    LINGTUSIMSENSORS_API void ConfigureCameraCaptureShowFlags(
        EStreamKind StreamKind,
        FEngineShowFlags& ShowFlags);

    /**
     * Validates one copied latest-applied Visual snapshot against the camera binding.
     * The output is cleared on every rejection so stale truth cannot be reused.
     */
    LINGTUSIMSENSORS_API bool ResolveAppliedTruthSample(
        const LingTuSim::FSnapshotEnvelope& Snapshot,
        const FString& ExpectedSessionId,
        uint64 ExpectedModelGeneration,
        uint64 ExpectedResetGeneration,
        FTruthSampleStamp& OutStamp,
        FString& OutError);

    LINGTUSIMSENSORS_API bool ParseCameraPlanJson(
        const FString& Json,
        FCameraPlan& OutPlan,
        FString& OutError);

    LINGTUSIMSENSORS_API bool ParseRunAllocationJson(
        const FString& Json,
        FRunAllocation& OutAllocation,
        FString& OutError);

    LINGTUSIMSENSORS_API bool ConvertColorPixels(
        const TArray<FColor>& Pixels,
        int32 Width,
        int32 Height,
        const FString& Encoding,
        TArray<uint8>& OutPayload,
        FString& OutError);

    LINGTUSIMSENSORS_API bool ConvertDepthPixels(
        const TArray<FLinearColor>& Pixels,
        int32 Width,
        int32 Height,
        double DepthScale,
        TArray<uint8>& OutPayload,
        FString& OutError);

    LINGTUSIMSENSORS_API bool ConvertDepthValues(
        const TArray<float>& DepthValues,
        int32 Width,
        int32 Height,
        double DepthScale,
        TArray<uint8>& OutPayload,
        FString& OutError);

    /**
     * Finds unique one-to-one RGB/depth streams that may share one scene render.
     * Ambiguous or geometrically different streams deliberately remain separate.
     */
    LINGTUSIMSENSORS_API TArray<FSharedColorDepthPair>
    ResolveSharedColorDepthCapturePairs(const TArray<FStreamPlan>& Streams);

    /**
     * Splits RGBA16F SceneColor/SceneDepth pixels into sRGB color and metres.
     * UE SceneDepth is linear in world units; RobotSimUE uses centimetres.
     */
    LINGTUSIMSENSORS_API bool DecodeSharedColorDepthPixels(
        const TArray<FLinearColor>& Pixels,
        int32 Width,
        int32 Height,
        TArray<FColor>& OutColorPixels,
        TArray<float>& OutDepthMeters,
        FString& OutError);

    /**
     * Encodes packed RGBA16F SceneColor/SceneDepth readback without creating a
     * second full-size FLinearColor image. Intended for background execution.
     */
    LINGTUSIMSENSORS_API bool EncodeSharedColorDepthHalfPixels(
        const TArray<FFloat16Color>& Pixels,
        int32 Width,
        int32 Height,
        const FString& ColorEncoding,
        double DepthScale,
        TArray<uint8>& OutColorPayload,
        TArray<uint8>& OutDepthPayload,
        FString& OutError);

    LINGTUSIMSENSORS_API bool AdvanceSchedule(
        float DeltaSeconds,
        double RateHz,
        double& InOutAccumulatorSeconds);

    /**
     * Rate-limits the filesystem projection of camera readiness. Stream state
     * changes stay immediate, ordinary frame counters are projected on a low-
     * frequency heartbeat, and transient sharing failures retry with backoff.
     */
    class LINGTUSIMSENSORS_API FReadinessEvidenceCadence final
    {
    public:
        explicit FReadinessEvidenceCadence(
            double InHeartbeatIntervalSeconds = 0.5,
            double InRetryIntervalSeconds = 0.1);

        void MarkDirty();
        bool ShouldAttempt(double NowSeconds) const;
        void RecordAttempt(double NowSeconds, bool bSucceeded);
        void Reset();

    private:
        double HeartbeatIntervalSeconds = 0.5;
        double RetryIntervalSeconds = 0.1;
        double LastSuccessfulWriteSeconds = -1.0;
        double NextAttemptSeconds = 0.0;
        bool bDirty = true;
    };

    LINGTUSIMSENSORS_API bool ResolveMountTransform(
        const FStreamPlan& Stream,
        bool bPreviewOverride,
        const FVector& PreviewLocationCm,
        const FQuat& PreviewRotation,
        FVector& OutRelativeLocationCm,
        FQuat& OutRelativeRotation,
        bool& OutUsedPlanExtrinsic,
        FString& OutError);

    LINGTUSIMSENSORS_API FString BuildReadinessEvidenceJson(
        const FString& SessionId,
        uint64 ModelGeneration,
        uint64 ResetGeneration,
        const TArray<FStreamReadiness>& Streams);
}

/**
 * Real Unreal scene-capture producer for RGB and depth camera streams.
 *
 * Scene captures use a bounded asynchronous GPU-readback pipeline. Rendering,
 * GPU-to-staging copies, CPU conversion, and SHM publication are separated
 * across ticks so the game thread never waits for a per-frame RHI flush. A
 * pre-ACTIVE job freezes a bounded startup budget; steady-state jobs keep the
 * stricter runtime budget, and render-command/GPU stages own separate clocks.
 */
UCLASS()
class LINGTUSIMSENSORS_API ULingTuSimCameraCaptureSubsystem final
    : public UTickableWorldSubsystem
{
    GENERATED_BODY()

public:
    ~ULingTuSimCameraCaptureSubsystem();
    virtual void Initialize(FSubsystemCollectionBase& Collection) override;
    virtual void OnWorldBeginPlay(UWorld& InWorld) override;
    virtual void OnWorldEndPlay(UWorld& InWorld) override;
    virtual void PreDeinitialize() override;
    virtual void Deinitialize() override;
    virtual void Tick(float DeltaTime) override;
    virtual TStatId GetStatId() const override;
    virtual bool IsTickable() const override;

    /** Explicit binding entry point for a host that already loaded the bundle. */
    bool BindPlan(
        const LingTuSim::Sensors::Camera::FCameraPlan& Plan,
        const LingTuSim::Sensors::Camera::FRunAllocation& Allocation,
        uint64 ModelGeneration,
        uint64 ResetGeneration,
        FString& OutError);

    bool IsBound() const { return bBound; }
    bool HasActiveFrame() const { return bHasActiveFrame; }
    FString LastError() const { return LastFailure; }

private:
    enum class ECaptureLayout : uint8
    {
        Separate,
        SharedColorLeader,
        SharedDepthFollower,
    };

    struct FAsyncReadbackJob;
    struct FDeferredCaptureJob;
    struct FCaptureState;

    bool BindFromCommandLine(FString& OutError);
    bool CreateCaptureState(
        const LingTuSim::Sensors::Camera::FStreamPlan& Stream,
        const FString& MappingName,
        uint64 ModelGeneration,
        ECaptureLayout Layout,
        FCaptureState*& OutState,
        FString& OutError);
    bool CanQueueDeferredCaptureBatch(
        const TArray<FCaptureState*>& DueCaptures,
        bool& OutCanQueue,
        FString& OutError) const;
    bool QueueDeferredCapture(
        FCaptureState& State,
        const LingTuSim::Sensors::Camera::FTruthSampleStamp& TruthSample,
        uint64 SubmittedFrame,
        double SubmittedAtSeconds,
        FString& OutError);
    bool PromoteDeferredCaptureToReadback(
        FCaptureState& State,
        uint64 CurrentFrame,
        double NowSeconds,
        FString& OutError);
    bool PublishCompletedReadback(
        FCaptureState& State,
        FAsyncReadbackJob& Readback,
        FString& OutError);
    bool PumpCaptureReadbacks(
        FCaptureState& State,
        bool bAllowPublish,
        double NowSeconds,
        bool& OutReadinessChanged,
        FString& OutError);
    void DiscardDeferredCapture(FCaptureState& State);
    void DiscardPendingReadbacks(FCaptureState& State);
    void DrainReadbacksForRelease(FCaptureState& State);
    void FlushReadinessEvidenceIfDue(double NowSeconds);
    bool WriteReadinessEvidence(FString& OutError) const;
    void FailClosed(const FString& Error);
    void ReleaseCaptureState(FCaptureState& State);
    void ReleaseCaptures();

    TArray<FCaptureState*> Captures;
    FString SessionId;
    FString EvidenceDirectory;
    uint64 ModelGeneration = 0;
    uint64 ResetGeneration = 0;
    int32 TotalPlanStreamCount = 0;
    LingTuSim::Sensors::Camera::FReadinessEvidenceCadence ReadinessEvidenceCadence;
    double VisualBindingWaitStartSeconds = -1.0;
    bool bHasBoundOnce = false;
    bool bCommandLineBindingRequested = false;
    bool bWorldBindingLifecycleActive = false;
    bool bDepthCaptureInMainRenderer = false;
    bool bSharedColorDepthCapture = false;
    bool bBound = false;
    bool bHasActiveFrame = false;
    FString LastFailure;
};

namespace LingTuSim::Sensors::Camera
{
    enum class EWorldBindingAction : uint8
    {
        Skip,
        Wait,
        Bind,
    };

    /** Classifies command-line camera ownership for one Unreal world. */
    LINGTUSIMSENSORS_API EWorldBindingAction EvaluateWorldBindingAction(
        EWorldType::Type WorldType,
        bool bHasBegunPlay,
        bool bWorldInitialized,
        bool bWorldTearingDown,
        TFunctionRef<int32()> VisualBindingCountProvider);

    LINGTUSIMSENSORS_API bool HasWorldBindingWaitExpired(
        bool bHasBegunPlay,
        double ElapsedSeconds,
        double TimeoutSeconds);
}

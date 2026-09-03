from __future__ import annotations

from pathlib import Path

# ruff: noqa: S101

_SENSORS = (
    Path(__file__).resolve().parents[2]
    / "sim"
    / "runtime"
    / "visual"
    / "RobotSimUE"
    / "Plugins"
    / "LingTuSim"
    / "Source"
    / "LingTuSimSensors"
)


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_camera_capture_latches_one_applied_truth_snapshot_only_after_shm_publish() -> None:
    header = _read(_SENSORS / "Public" / "LingTuSimCameraCaptureSubsystem.h")
    source = _read(_SENSORS / "Private" / "LingTuSimCameraCaptureSubsystem.cpp")

    assert "FTruthSampleStamp" in header
    assert "ResolveAppliedTruthSample" in header
    assert "GetLatestAppliedSnapshot" in source

    capture = source[
        source.index("bool ULingTuSimCameraCaptureSubsystem::PublishCompletedReadback") :
        source.index("bool ULingTuSimCameraCaptureSubsystem::PumpCaptureReadbacks")
    ]
    publish = capture.index("TryPublish")
    assert publish < capture.index("LastPublishedTruth = Readback.TruthSample")
    assert publish < capture.index("++State.PublishedFrames")
    assert publish < capture.index("State.bActive = true")


def test_camera_readiness_exports_truth_sequence_and_sim_time_without_wall_clock() -> None:
    source = _read(_SENSORS / "Private" / "LingTuSimCameraCaptureSubsystem.cpp")

    readiness = source[
        source.index("FString BuildReadinessEvidenceJson") :
        source.index("struct ULingTuSimCameraCaptureSubsystem::FCaptureState")
    ]
    assert "last_sample_truth_sequence" in readiness
    assert "last_sample_sim_time_ns" in readiness
    assert "UnixTimeNs" not in readiness

    projection = source[
        source.index("bool ULingTuSimCameraCaptureSubsystem::WriteReadinessEvidence") :
        source.index("void ULingTuSimCameraCaptureSubsystem::FailClosed")
    ]
    assert "State->LastPublishedTruth.TruthSequence" in projection
    assert "State->LastPublishedTruth.SimTimeNs" in projection


def test_camera_reset_and_failures_cannot_leave_active_truth_progress() -> None:
    source = _read(_SENSORS / "Private" / "LingTuSimCameraCaptureSubsystem.cpp")

    state = source[
        source.index("struct ULingTuSimCameraCaptureSubsystem::FCaptureState") :
        source.index("void ULingTuSimCameraCaptureSubsystem::Initialize")
    ]
    assert "FTruthSampleStamp LastPublishedTruth" in state
    assert "PublishedFrames = 0" in state
    assert "bActive = false" in state

    failure = source[
        source.index("void ULingTuSimCameraCaptureSubsystem::FailClosed") :
        source.index("void ULingTuSimCameraCaptureSubsystem::ReleaseCaptures")
    ]
    assert "State->bActive = false" in failure
    assert "State->FailureReason = Error" in failure
    assert "WriteReadinessEvidence" in failure


def test_next_visual_reset_clears_camera_truth_before_any_new_publish() -> None:
    source = _read(_SENSORS / "Private" / "LingTuSimCameraCaptureSubsystem.cpp")
    tick = source[
        source.index("void ULingTuSimCameraCaptureSubsystem::Tick") :
        source.index("TStatId ULingTuSimCameraCaptureSubsystem::GetStatId")
    ]

    reset = tick.index("AppliedSnapshot.ResetGeneration - ResetGeneration == 1")
    publish = tick.index("QueueDeferredCapture")
    assert reset < publish
    assert "State->PublishedFrames = 0" in tick
    assert "State->LastPublishedTruth =" in tick
    assert "State->bActive = false" in tick
    assert "WriteReadinessEvidence" in tick


def test_camera_does_not_publish_twice_against_one_applied_truth_frame() -> None:
    source = _read(_SENSORS / "Private" / "LingTuSimCameraCaptureSubsystem.cpp")
    tick = source[
        source.index("void ULingTuSimCameraCaptureSubsystem::Tick") :
        source.index("TStatId ULingTuSimCameraCaptureSubsystem::GetStatId")
    ]

    duplicate_gate = tick.index(
        "TruthSample.TruthSequence == LastSubmitted.TruthSequence"
    )
    publish = tick.index("QueueDeferredCapture")
    assert duplicate_gate < publish
    assert "TruthSample.SimTimeNs == LastSubmitted.SimTimeNs" in tick
    assert "bBatchTruthEligible = false;" in tick[duplicate_gate:publish]
    assert "break;" in tick[duplicate_gate:publish]
    assert "latest applied Visual snapshot did not advance monotonically" in tick


def test_missing_applied_snapshot_retracts_active_without_advancing_truth() -> None:
    source = _read(_SENSORS / "Private" / "LingTuSimCameraCaptureSubsystem.cpp")
    tick = source[
        source.index("void ULingTuSimCameraCaptureSubsystem::Tick") :
        source.index("TStatId ULingTuSimCameraCaptureSubsystem::GetStatId")
    ]
    missing = tick[
        tick.index("VisualSubsystem == nullptr") :
        tick.index("AppliedSnapshot.SessionId == SessionId")
    ]

    assert "State->bActive = false" in missing
    assert "bHasActiveFrame = false" in missing
    assert "WriteReadinessEvidence" in missing
    assert "State->PublishedFrames = 0" not in missing
    assert "State->LastPublishedTruth =" not in missing


def test_camera_capture_state_destroys_transient_owner_on_late_failure() -> None:
    source = _read(_SENSORS / "Private" / "LingTuSimCameraCaptureSubsystem.cpp")
    create = source[
        source.index("bool ULingTuSimCameraCaptureSubsystem::CreateCaptureState") :
        source.index("bool ULingTuSimCameraCaptureSubsystem::CanQueueDeferredCaptureBatch")
    ]

    resolve = create.index("ResolveMountTransform")
    parent_lookup = create.index("ParentFrame == nullptr")
    spawn = create.index("SpawnActor<AActor>")
    cleanup = create.index("FTransientOwnerCleanup")
    release = create.index("OwnerCleanup.Release()")
    state_release = create.index("State.Release()", release)
    assert resolve < spawn
    assert parent_lookup < spawn
    assert spawn < cleanup < release < state_release
    assert "Owner->Destroy()" in create


def test_partial_camera_bind_releases_every_prepared_ue_capture_state() -> None:
    source = _read(_SENSORS / "Private" / "LingTuSimCameraCaptureSubsystem.cpp")
    bind = source[
        source.index("bool ULingTuSimCameraCaptureSubsystem::BindPlan") :
        source.index("bool ULingTuSimCameraCaptureSubsystem::BindFromCommandLine")
    ]

    assert "ReleaseCaptureState(*PreparedState)" in bind
    assert bind.index("ReleaseCaptureState(*PreparedState)") < bind.index(
        "delete PreparedState"
    )
    mapping_failure = bind[
        bind.index("Mapping == nullptr") : bind.index("FCaptureState* State")
    ]
    create_failure = bind[
        bind.index("if (!CreateCaptureState") : bind.index("Prepared.Add(State)")
    ]
    assert "ReleasePreparedCaptures();" in mapping_failure
    assert "ReleasePreparedCaptures();" in create_failure


def test_camera_teardown_cancels_and_drains_deferred_capture_before_owner_destroy() -> None:
    source = _read(_SENSORS / "Private" / "LingTuSimCameraCaptureSubsystem.cpp")
    release_one = source[
        source.index("void ULingTuSimCameraCaptureSubsystem::ReleaseCaptureState") :
        source.index("void ULingTuSimCameraCaptureSubsystem::ReleaseCaptures")
    ]

    deferred = release_one.index("State.DeferredCapture.IsSet()")
    unregister = release_one.index("State.Capture->UnregisterComponent()")
    drain = release_one.index("DrainReadbacksForRelease(State)")
    flush = release_one.index("FlushRenderingCommands()")
    destroy = release_one.index("State.Owner->Destroy()")
    assert deferred < unregister < drain
    assert unregister < flush < destroy


def test_camera_transport_and_truth_evidence_keep_their_two_clock_domains() -> None:
    source = _read(_SENSORS / "Private" / "LingTuSimCameraCaptureSubsystem.cpp")
    capture = source[
        source.index("bool ULingTuSimCameraCaptureSubsystem::PublishCompletedReadback") :
        source.index("bool ULingTuSimCameraCaptureSubsystem::PumpCaptureReadbacks")
    ]

    assert "Metadata.TimestampNs = LingTuSim::Sensors::CameraShm::UnixTimeNs()" in capture
    assert "transport freshness clock" in capture
    assert "State.LastPublishedTruth = Readback.TruthSample" in capture


def test_camera_capture_uses_a_bounded_nonblocking_gpu_readback_pipeline() -> None:
    source = _read(_SENSORS / "Private" / "LingTuSimCameraCaptureSubsystem.cpp")

    tick = source[
        source.index("void ULingTuSimCameraCaptureSubsystem::Tick") :
        source.index("TStatId ULingTuSimCameraCaptureSubsystem::GetStatId")
    ]
    submit = source[
        source.index("bool ULingTuSimCameraCaptureSubsystem::PromoteDeferredCaptureToReadback") :
        source.index("bool ULingTuSimCameraCaptureSubsystem::PublishCompletedReadback")
    ]

    assert '#include "RHIGPUReadback.h"' in source
    assert "FRHIGPUTextureReadback" in source
    assert "MaxPendingReadbacksPerStream" in source
    assert "State.PendingReadbacks.Num()" in submit
    assert "MaxPendingReadbacksPerStream" in submit
    assert "ENQUEUE_RENDER_COMMAND" in submit
    assert "EnqueueCopy" in submit
    assert "FlushRenderingCommands" not in tick


def test_camera_capture_defers_the_paired_scene_render_before_next_frame_readback() -> None:
    header = _read(_SENSORS / "Public" / "LingTuSimCameraCaptureSubsystem.h")
    source = _read(_SENSORS / "Private" / "LingTuSimCameraCaptureSubsystem.cpp")

    tick = source[
        source.index("void ULingTuSimCameraCaptureSubsystem::Tick") :
        source.index("TStatId ULingTuSimCameraCaptureSubsystem::GetStatId")
    ]
    queue = source[
        source.index("bool ULingTuSimCameraCaptureSubsystem::QueueDeferredCapture") :
        source.index("bool ULingTuSimCameraCaptureSubsystem::PromoteDeferredCaptureToReadback")
    ]
    promote = source[
        source.index("bool ULingTuSimCameraCaptureSubsystem::PromoteDeferredCaptureToReadback") :
        source.index("bool ULingTuSimCameraCaptureSubsystem::PublishCompletedReadback")
    ]

    assert "EDeferredCaptureAction" in header
    assert "FDeferredCaptureJob" in source
    assert "DeferredCapture" in source
    assert "CaptureSceneDeferred()" in queue
    assert "CaptureScene()" not in queue
    assert "->CaptureScene();" not in source
    assert "ISceneRenderBuilder" not in source
    assert "EvaluateDeferredCapturePromotion" in promote
    assert "EnqueueCopy" in promote
    assert tick.index("PromoteDeferredCaptureToReadback") < tick.index(
        "QueueDeferredCapture"
    )
    assert "CanQueueDeferredCaptureBatch" in tick


def test_deferred_camera_capture_is_generation_checked_and_time_bounded() -> None:
    source = _read(_SENSORS / "Private" / "LingTuSimCameraCaptureSubsystem.cpp")
    evaluate = source[
        source.index("EDeferredCaptureAction EvaluateDeferredCapturePromotion") :
        source.index("bool ParseCameraPlanJson")
    ]

    assert "EDeferredCaptureAction::Discard" in evaluate
    assert "EDeferredCaptureAction::Wait" in evaluate
    assert "EDeferredCaptureAction::Promote" in evaluate
    assert "EDeferredCaptureAction::Fail" in evaluate
    assert "ExpectedSessionId" in evaluate
    assert "ExpectedModelGeneration" in evaluate
    assert "ExpectedResetGeneration" in evaluate
    assert "TimeoutSeconds" in evaluate
    assert "current frame regressed" in evaluate

    pump = source[
        source.index("bool ULingTuSimCameraCaptureSubsystem::PumpCaptureReadbacks") :
        source.index("void ULingTuSimCameraCaptureSubsystem::DiscardDeferredCapture")
    ]
    assert "Readback->PipelineTimeoutSeconds" in pump
    assert "EvaluateReadbackStageDeadline" in pump
    assert "Readback->Stage.Load() == Stage" in pump
    assert "EReadbackDeadlineStage::EnqueuePending" in pump
    assert "EReadbackDeadlineStage::GpuPending" in pump


def test_camera_first_frame_pipeline_budget_is_bounded_and_stage_specific() -> None:
    header = _read(_SENSORS / "Public" / "LingTuSimCameraCaptureSubsystem.h")
    source = _read(_SENSORS / "Private" / "LingTuSimCameraCaptureSubsystem.cpp")
    automation = _read(
        _SENSORS / "Private" / "Tests" / "LingTuSimCameraCaptureTest.cpp"
    )

    assert "EReadbackDeadlineStage" in header
    assert "EReadbackDeadlineAction" in header
    assert "SelectCapturePipelineTimeoutSeconds" in header
    assert "EvaluateReadbackStageDeadline" in header
    assert "EvaluateReadbackStageDeadline" in automation
    assert "a reset generation receives the startup budget again" in automation
    assert "CpuReady never becomes a GPU timeout" in automation

    assert "CameraFirstFramePipelineTimeoutSeconds = 30.0" in source
    assert "CameraCapturePipelineTimeoutSeconds = 1.0" in source
    assert "startup_deadline_s=%.3f steady_deadline_s=%.3f" in source

    deferred = source[
        source.index("bool ULingTuSimCameraCaptureSubsystem::QueueDeferredCapture") :
        source.index("bool ULingTuSimCameraCaptureSubsystem::PromoteDeferredCaptureToReadback")
    ]
    assert "State.PublishedFrames" in deferred
    assert "Deferred.PipelineTimeoutSeconds" in deferred
    assert "SelectCapturePipelineTimeoutSeconds" in deferred

    promote = source[
        source.index("bool ULingTuSimCameraCaptureSubsystem::PromoteDeferredCaptureToReadback") :
        source.index("bool ULingTuSimCameraCaptureSubsystem::PublishCompletedReadback")
    ]
    assert "Readback->PipelineTimeoutSeconds = Deferred.PipelineTimeoutSeconds" in promote
    gpu_clock = promote.index(
        "Readback->GpuSubmittedAtSeconds.Store(FPlatformTime::Seconds())"
    )
    assert promote.index("ENQUEUE_RENDER_COMMAND") < gpu_clock
    assert gpu_clock < promote.index("EnqueueCopy", gpu_clock)
    assert promote.index("EnqueueCopy", gpu_clock) < promote.index(
        "Stage.Store(ELingTuCameraReadbackStage::GpuPending)", gpu_clock
    )

    pump = source[
        source.index("bool ULingTuSimCameraCaptureSubsystem::PumpCaptureReadbacks") :
        source.index("void ULingTuSimCameraCaptureSubsystem::DiscardDeferredCapture")
    ]
    enqueue_timeout = pump.index(
        "Stage == ELingTuCameraReadbackStage::EnqueuePending"
    )
    gpu_timeout = pump.index("Stage == ELingTuCameraReadbackStage::GpuPending")
    cpu_ready = pump.index("Stage == ELingTuCameraReadbackStage::CpuReady")
    assert "Readback->QueuedAtSeconds" in pump[enqueue_timeout:gpu_timeout]
    assert "Readback->GpuSubmittedAtSeconds.Load()" in pump[gpu_timeout:cpu_ready]
    assert "EvaluateReadbackStageDeadline" in pump
    assert "ReadbackAgeSeconds" not in pump


def test_depth_main_renderer_path_is_explicit_opt_in_and_scene_depth_only() -> None:
    header = _read(_SENSORS / "Public" / "LingTuSimCameraCaptureSubsystem.h")
    source = _read(_SENSORS / "Private" / "LingTuSimCameraCaptureSubsystem.cpp")

    assert "ShouldRenderCameraCaptureInMainRenderer" in header
    assert "bool bDepthCaptureInMainRenderer = false" in header
    assert "LingTuDepthCaptureInMainRenderer" in source
    assert "SCS_SceneDepth" in source
    assert "SCS_FinalColorLDR" in source
    assert "State->Capture->bRenderInMainRenderer" in source

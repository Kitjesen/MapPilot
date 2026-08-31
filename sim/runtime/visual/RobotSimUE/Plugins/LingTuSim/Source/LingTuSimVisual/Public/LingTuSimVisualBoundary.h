#pragma once

#include "LingTuSimRuntimeTypes.h"
#include "LingTuSimSnapshotMailbox.h"

class LINGTUSIMVISUAL_API ILingTuSimVisualBoundary
{
public:
    virtual ~ILingTuSimVisualBoundary() = default;
    virtual bool StartVisualPlan(
        const FString& BundleDirectory,
        const FString& ArtifactRoot,
        uint64 ModelGeneration,
        uint64 ResetGeneration,
        FString& OutError) = 0;
    virtual LingTuSim::ESnapshotPublishResult SubmitSnapshot(
        const LingTuSim::FSnapshotEnvelope& Snapshot) = 0;
    virtual bool SubmitScenarioSnapshotJson(
        const FString& SnapshotJson,
        LingTuSim::ESnapshotPublishResult& OutPublishResult,
        FString& OutError) = 0;
};

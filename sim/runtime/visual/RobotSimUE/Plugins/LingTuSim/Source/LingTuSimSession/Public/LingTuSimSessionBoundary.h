#pragma once
#include "LingTuSimRuntimeTypes.h"

class LINGTUSIMSESSION_API ILingTuSimSessionBoundary
{
public:
    virtual ~ILingTuSimSessionBoundary() = default;
    virtual bool BindCompiledSession(const LingTuSim::FSessionBundleView& Bundle) = 0;
    virtual void ResetSession(uint64 ResetGeneration) = 0;
};

class LINGTUSIMSESSION_API ILingTuSimPhysicsBoundary
{
public:
    virtual ~ILingTuSimPhysicsBoundary() = default;
    virtual bool BindPhysicsPlan(const LingTuSim::FSessionBundleView& Bundle) = 0;
    virtual bool PublishSnapshot(const LingTuSim::FSnapshotEnvelope& Snapshot) = 0;
};

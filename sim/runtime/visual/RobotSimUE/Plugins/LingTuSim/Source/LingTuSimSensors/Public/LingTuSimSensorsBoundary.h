#pragma once
#include "LingTuSimRuntimeTypes.h"

class LINGTUSIMSENSORS_API ILingTuSimSensorBoundary
{
public:
    virtual ~ILingTuSimSensorBoundary() = default;
    virtual bool BindSensorPlan(const LingTuSim::FSessionBundleView& Bundle) = 0;
    virtual bool TickSensors(const LingTuSim::FSnapshotEnvelope& Snapshot) = 0;
};

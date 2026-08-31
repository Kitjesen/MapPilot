#pragma once

#include "LingTuSimRuntimeTypes.h"

namespace LingTuSim::Visual
{
    /** The only simulation-to-Unreal pose conversion used by the visual runtime. */
    class LINGTUSIMVISUAL_API FCoordinateConverter final
    {
    public:
        static bool TryMakeWorldTransform(
            const FEntityState& Entity,
            const FVector& WorldScale,
            FTransform& OutTransform);
    };
}

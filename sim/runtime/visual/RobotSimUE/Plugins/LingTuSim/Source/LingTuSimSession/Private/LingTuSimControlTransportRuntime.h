#pragma once

#include "CoreMinimal.h"
#include "LingTuSimRuntimeTypes.h"

namespace LingTuSim
{
    /** Module-owned loopback sockets and allocation binding for playable control. */
    class FLingTuSimControlTransportRuntime final
    {
    public:
        FLingTuSimControlTransportRuntime();
        ~FLingTuSimControlTransportRuntime();

        FLingTuSimControlTransportRuntime(const FLingTuSimControlTransportRuntime&) = delete;
        FLingTuSimControlTransportRuntime& operator=(
            const FLingTuSimControlTransportRuntime&) = delete;

        /**
         * Returns true for a valid viewer launch with no control switches and
         * for a fully bound playable launch. Partial/mismatched control fails.
         */
        bool StartFromCommandLine(
            const FSessionBundleView& Bundle,
            FString& OutError);

        void Stop();
        bool IsEnabled() const { return bEnabled; }

    private:
        class FImpl;
        TUniquePtr<FImpl> Impl;
        bool bEnabled = false;
    };
}

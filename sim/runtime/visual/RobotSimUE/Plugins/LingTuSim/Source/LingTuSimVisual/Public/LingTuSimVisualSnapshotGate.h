#pragma once

#include "LingTuSimRuntimeTypes.h"

namespace LingTuSim::Visual
{
    enum class ESnapshotGateResult : uint8
    {
        Accept,
        Unbound,
        SessionMismatch,
        OldModel,
        FutureModel,
        OldReset,
        OldSequence,
    };

    /** Commits ordering history only after a complete visual frame is applied. */
    class LINGTUSIMVISUAL_API FSnapshotGate final
    {
    public:
        void BindSession(const FString& SessionId, uint64 ModelGeneration);
        void Clear();
        ESnapshotGateResult Evaluate(const FSnapshotEnvelope& Snapshot) const;
        void Commit(const FSnapshotEnvelope& Snapshot);

    private:
        FString BoundSessionId;
        uint64 BoundModelGeneration = 0;
        uint64 CommittedResetGeneration = 0;
        uint64 CommittedSequence = 0;
        bool bIsBound = false;
        bool bHasCommittedSnapshot = false;
    };
}

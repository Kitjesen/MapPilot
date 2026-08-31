#pragma once

#include "HAL/CriticalSection.h"
#include "LingTuSimRuntimeTypes.h"

namespace LingTuSim
{
    enum class ESnapshotPublishResult : uint8
    {
        Accepted,
        Replaced,
        Stale,
        SessionMismatch,
        ModelMismatch,
    };

    /**
     * A session-bound, capacity-one handoff for immutable simulation snapshots.
     *
     * The mailbox owns a value copy of every accepted snapshot. Consumers also
     * receive a value copy, so no producer-owned storage crosses thread boundaries.
     */
    class LINGTUSIMSESSION_API FSnapshotMailbox final
    {
    public:
        FSnapshotMailbox() = default;
        FSnapshotMailbox(const FSnapshotMailbox&) = delete;
        FSnapshotMailbox& operator=(const FSnapshotMailbox&) = delete;

        /** Rebinds the mailbox and resets its pending snapshot and ordering watermark. */
        void BindSession(FString SessionId, uint64 ModelGeneration);

        /** Publishes only snapshots matching the bound session and model generation. */
        ESnapshotPublishResult Publish(const FSnapshotEnvelope& Snapshot);

        /** Copies and consumes the latest pending snapshot, if one exists. */
        bool TryTakeLatest(FSnapshotEnvelope& OutSnapshot);

        /** Drops the pending snapshot while preserving binding and stale-frame protection. */
        void Clear();

        /** Returns either zero or one. */
        int32 PendingCount() const;

    private:
        mutable FCriticalSection CriticalSection;
        FString BoundSessionId;
        uint64 BoundModelGeneration = 0;
        uint64 LastResetGeneration = 0;
        uint64 LastSequence = 0;
        FSnapshotEnvelope PendingSnapshot;
        bool bIsBound = false;
        bool bHasOrderingWatermark = false;
        bool bHasPendingSnapshot = false;
    };
}

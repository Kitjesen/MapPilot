#include "LingTuSimSnapshotMailbox.h"

#include "Misc/ScopeLock.h"

namespace LingTuSim
{
    void FSnapshotMailbox::BindSession(FString SessionId, const uint64 ModelGeneration)
    {
        FScopeLock Lock(&CriticalSection);

        BoundSessionId = MoveTemp(SessionId);
        BoundModelGeneration = ModelGeneration;
        LastResetGeneration = 0;
        LastSequence = 0;
        PendingSnapshot = FSnapshotEnvelope{};
        bIsBound = !BoundSessionId.IsEmpty();
        bHasOrderingWatermark = false;
        bHasPendingSnapshot = false;
    }

    ESnapshotPublishResult FSnapshotMailbox::Publish(const FSnapshotEnvelope& Snapshot)
    {
        FScopeLock Lock(&CriticalSection);

        if (!bIsBound || Snapshot.SessionId != BoundSessionId)
        {
            return ESnapshotPublishResult::SessionMismatch;
        }

        if (Snapshot.ModelGeneration != BoundModelGeneration)
        {
            return ESnapshotPublishResult::ModelMismatch;
        }

        if (bHasOrderingWatermark)
        {
            const bool bNewerResetGeneration =
                Snapshot.ResetGeneration > LastResetGeneration;
            const bool bNewerSequenceInCurrentReset =
                Snapshot.ResetGeneration == LastResetGeneration
                && Snapshot.Sequence > LastSequence;
            if (!bNewerResetGeneration && !bNewerSequenceInCurrentReset)
            {
                return ESnapshotPublishResult::Stale;
            }
        }

        const bool bReplacedPendingSnapshot = bHasPendingSnapshot;
        PendingSnapshot = Snapshot;
        LastResetGeneration = Snapshot.ResetGeneration;
        LastSequence = Snapshot.Sequence;
        bHasOrderingWatermark = true;
        bHasPendingSnapshot = true;

        return bReplacedPendingSnapshot
            ? ESnapshotPublishResult::Replaced
            : ESnapshotPublishResult::Accepted;
    }

    bool FSnapshotMailbox::TryTakeLatest(FSnapshotEnvelope& OutSnapshot)
    {
        FScopeLock Lock(&CriticalSection);

        if (!bHasPendingSnapshot)
        {
            return false;
        }

        OutSnapshot = PendingSnapshot;
        PendingSnapshot = FSnapshotEnvelope{};
        bHasPendingSnapshot = false;
        return true;
    }

    void FSnapshotMailbox::Clear()
    {
        FScopeLock Lock(&CriticalSection);

        PendingSnapshot = FSnapshotEnvelope{};
        bHasPendingSnapshot = false;
    }

    int32 FSnapshotMailbox::PendingCount() const
    {
        FScopeLock Lock(&CriticalSection);
        return bHasPendingSnapshot ? 1 : 0;
    }
}

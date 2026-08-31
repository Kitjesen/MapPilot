#include "LingTuSimVisualSnapshotGate.h"

void LingTuSim::Visual::FSnapshotGate::BindSession(
    const FString& SessionId,
    const uint64 ModelGeneration)
{
    const bool bBindingChanged = !bIsBound
        || BoundSessionId != SessionId
        || BoundModelGeneration != ModelGeneration;
    BoundSessionId = SessionId;
    BoundModelGeneration = ModelGeneration;
    bIsBound = !SessionId.IsEmpty();
    if (bBindingChanged)
    {
        bHasCommittedSnapshot = false;
        CommittedResetGeneration = 0;
        CommittedSequence = 0;
    }
}

void LingTuSim::Visual::FSnapshotGate::Clear()
{
    BoundSessionId.Reset();
    BoundModelGeneration = 0;
    CommittedResetGeneration = 0;
    CommittedSequence = 0;
    bIsBound = false;
    bHasCommittedSnapshot = false;
}

LingTuSim::Visual::ESnapshotGateResult LingTuSim::Visual::FSnapshotGate::Evaluate(
    const FSnapshotEnvelope& Snapshot) const
{
    if (!bIsBound)
    {
        return ESnapshotGateResult::Unbound;
    }
    if (Snapshot.ModelGeneration > BoundModelGeneration)
    {
        return ESnapshotGateResult::FutureModel;
    }
    if (Snapshot.SessionId != BoundSessionId)
    {
        return ESnapshotGateResult::SessionMismatch;
    }
    if (Snapshot.ModelGeneration < BoundModelGeneration)
    {
        return ESnapshotGateResult::OldModel;
    }
    if (!bHasCommittedSnapshot)
    {
        return ESnapshotGateResult::Accept;
    }
    if (Snapshot.ResetGeneration < CommittedResetGeneration)
    {
        return ESnapshotGateResult::OldReset;
    }
    if (Snapshot.ResetGeneration > CommittedResetGeneration)
    {
        return ESnapshotGateResult::Accept;
    }
    if (Snapshot.Sequence <= CommittedSequence)
    {
        return ESnapshotGateResult::OldSequence;
    }
    return ESnapshotGateResult::Accept;
}

void LingTuSim::Visual::FSnapshotGate::Commit(const FSnapshotEnvelope& Snapshot)
{
    check(Evaluate(Snapshot) == ESnapshotGateResult::Accept);
    CommittedResetGeneration = Snapshot.ResetGeneration;
    CommittedSequence = Snapshot.Sequence;
    bHasCommittedSnapshot = true;
}

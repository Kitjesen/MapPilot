#pragma once

#include "LingTuSimRuntimeTypes.h"
#include "LingTuSimVisualSnapshotGate.h"

class ALingTuSimScenarioActor;
class UWorld;

namespace LingTuSim::Visual
{
    enum class EScenarioVisualApplyResult : uint8
    {
        Accepted,
        InvalidSnapshot,
        Unbound,
        SessionMismatch,
        OldModel,
        FutureModel,
        OldReset,
        OldSequence,
        DuplicateStableId,
        EntitySetMismatch,
        ActorSpawnFailed,
        TransformRejected,
    };

    /**
     * Game-thread registry for actors driven by canonical ScenarioSnapshot JSON.
     *
     * The serialized Python ScenarioSnapshot remains the only cross-process
     * payload. This module never evaluates behavior or advances simulation time.
     */
    class LINGTUSIMVISUAL_API FScenarioVisualActorRegistry final
    {
    public:
        bool BindSession(UWorld& World, const FString& SessionId, uint64 ModelGeneration);
        bool BindRunId(const FString& RunId);
        void Clear(UWorld& World);

        EScenarioVisualApplyResult ApplySnapshotJson(
            UWorld& World,
            const FString& SnapshotJson,
            FString& OutError);

        int32 Num() const { return Actors.Num(); }
        ALingTuSimScenarioActor* FindActor(const FString& StableId) const;
        const FString& GetLastEvidenceJson() const { return LastEvidenceJson; }

    private:
        void DestroyActors(UWorld& World);

        FString BoundSessionId;
        FString BoundRunId;
        uint64 BoundModelGeneration = 0;
        uint64 LastAppliedResetGeneration = 0;
        int64 LastAppliedSimTimeNs = 0;
        bool bIsBound = false;
        bool bHasAppliedSnapshot = false;
        TMap<FString, TWeakObjectPtr<ALingTuSimScenarioActor>> Actors;
        FSnapshotGate SnapshotGate;
        FString LastEvidenceJson;
    };
}

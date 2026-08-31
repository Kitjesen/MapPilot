#include "LingTuSimScenarioVisualRegistry.h"

#include "LingTuSimScenarioActor.h"
#include "LingTuSimVisualTransform.h"
#include "Engine/World.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "UObject/NameTypes.h"

namespace
{
    constexpr double PositionToleranceMeters = 0.02;
    constexpr double MaximumExactJsonInteger = 9'007'199'254'740'991.0;

    struct FParsedScenarioEntity
    {
        LingTuSim::FEntityState State;
        FString Authority;
        uint64 SourceEpoch = 0;
        FString SemanticClass;
        FString MotionState;
        FString PhysicsProxyMode;
        FString BodyStableId;
    };

    struct FPreparedScenarioTransform
    {
        FParsedScenarioEntity Entity;
        FTransform WorldTransform;
    };

    struct FPreviousScenarioActorState
    {
        ALingTuSimScenarioActor* Actor = nullptr;
        FTransform WorldTransform;
        FString StableId;
        FString Authority;
        uint64 SourceEpoch = 0;
        FString SemanticClass;
        FString MotionState;
        FString PhysicsProxyMode;
        FString BodyStableId;
    };

    bool Fail(FString& OutError, const FString& Message)
    {
        OutError = Message;
        return false;
    }

    bool IsAsciiAlphaNumeric(const TCHAR Character)
    {
        return (Character >= TEXT('0') && Character <= TEXT('9'))
            || (Character >= TEXT('A') && Character <= TEXT('Z'))
            || (Character >= TEXT('a') && Character <= TEXT('z'));
    }

    bool IsCanonicalRunId(const FString& RunId)
    {
        if (RunId.IsEmpty() || RunId.Len() > 128 || !IsAsciiAlphaNumeric(RunId[0]))
        {
            return false;
        }
        for (const TCHAR Character : RunId)
        {
            if (!IsAsciiAlphaNumeric(Character)
                && Character != TEXT('_')
                && Character != TEXT('.')
                && Character != TEXT('-'))
            {
                return false;
            }
        }
        return true;
    }

    FName MakeScenarioActorBaseName(const FString& StableId)
    {
        FString ObjectName = TEXT("LingTuScenario_") + StableId;
        for (const TCHAR* Invalid = INVALID_OBJECTNAME_CHARACTERS; *Invalid != 0; ++Invalid)
        {
            ObjectName.ReplaceCharInline(*Invalid, TEXT('_'));
        }
        return FName(*ObjectName);
    }

    bool HasExactFields(
        const TSharedPtr<FJsonObject>& Object,
        const TArray<FString>& RequiredFields,
        const FString& Context,
        FString& OutError)
    {
        if (!Object.IsValid() || Object->Values.Num() != RequiredFields.Num())
        {
            return Fail(
                OutError,
                FString::Printf(TEXT("%s must contain exactly the canonical fields"), *Context));
        }
        for (const FString& Field : RequiredFields)
        {
            if (!Object->HasField(Field))
            {
                return Fail(
                    OutError,
                    FString::Printf(TEXT("%s is missing required field '%s'"), *Context, *Field));
            }
        }
        return true;
    }

    bool ReadTrimmedString(
        const TSharedPtr<FJsonObject>& Object,
        const TCHAR* Field,
        const FString& Context,
        FString& OutValue,
        FString& OutError)
    {
        if (!Object.IsValid()
            || !Object->TryGetStringField(Field, OutValue)
            || OutValue.IsEmpty()
            || OutValue.TrimStartAndEnd() != OutValue)
        {
            return Fail(
                OutError,
                FString::Printf(TEXT("%s.%s must be a non-empty trimmed string"), *Context, Field));
        }
        return true;
    }

    bool ReadNonNegativeInteger(
        const TSharedPtr<FJsonObject>& Object,
        const TCHAR* Field,
        const FString& Context,
        uint64& OutValue,
        FString& OutError)
    {
        double Number = 0.0;
        if (!Object.IsValid()
            || !Object->TryGetNumberField(Field, Number)
            || !FMath::IsFinite(Number)
            || Number < 0.0
            || Number > MaximumExactJsonInteger
            || FMath::FloorToDouble(Number) != Number)
        {
            return Fail(
                OutError,
                FString::Printf(TEXT("%s.%s must be an exact non-negative integer"), *Context, Field));
        }
        OutValue = static_cast<uint64>(Number);
        return true;
    }

    bool ReadFiniteArray(
        const TSharedPtr<FJsonObject>& Object,
        const TCHAR* Field,
        const int32 ExpectedCount,
        const FString& Context,
        TArray<double>& OutValues,
        FString& OutError)
    {
        const TArray<TSharedPtr<FJsonValue>>* Values = nullptr;
        if (!Object.IsValid()
            || !Object->TryGetArrayField(Field, Values)
            || Values == nullptr
            || Values->Num() != ExpectedCount)
        {
            return Fail(
                OutError,
                FString::Printf(
                    TEXT("%s.%s must contain exactly %d finite numbers"),
                    *Context,
                    Field,
                    ExpectedCount));
        }
        OutValues.Reset(ExpectedCount);
        for (int32 Index = 0; Index < Values->Num(); ++Index)
        {
            double Number = 0.0;
            if (!(*Values)[Index].IsValid()
                || !(*Values)[Index]->TryGetNumber(Number)
                || !FMath::IsFinite(Number))
            {
                return Fail(
                    OutError,
                    FString::Printf(TEXT("%s.%s[%d] must be finite"), *Context, Field, Index));
            }
            OutValues.Add(Number);
        }
        return true;
    }

    bool ParseEntity(
        const TSharedPtr<FJsonValue>& Value,
        const int32 Index,
        FParsedScenarioEntity& OutEntity,
        FString& OutError)
    {
        const FString Context = FString::Printf(TEXT("scenario snapshot entities[%d]"), Index);
        const TSharedPtr<FJsonObject>* ObjectPointer = nullptr;
        if (!Value.IsValid()
            || !Value->TryGetObject(ObjectPointer)
            || ObjectPointer == nullptr
            || !ObjectPointer->IsValid())
        {
            return Fail(OutError, FString::Printf(TEXT("%s must be an object"), *Context));
        }
        const TSharedPtr<FJsonObject>& Object = *ObjectPointer;
        static const TArray<FString> EntityFields = {
            TEXT("entity_id"),
            TEXT("transform"),
            TEXT("authority"),
            TEXT("source_epoch"),
            TEXT("semantic_class"),
            TEXT("motion_state"),
            TEXT("physics_proxy_mode"),
            TEXT("body_stable_id"),
        };
        if (!HasExactFields(Object, EntityFields, Context, OutError))
        {
            return false;
        }

        FParsedScenarioEntity Candidate;
        if (!ReadTrimmedString(
                Object,
                TEXT("entity_id"),
                Context,
                Candidate.State.Id.StableId,
                OutError)
            || !ReadTrimmedString(
                Object,
                TEXT("authority"),
                Context,
                Candidate.Authority,
                OutError)
            || !ReadNonNegativeInteger(
                Object,
                TEXT("source_epoch"),
                Context,
                Candidate.SourceEpoch,
                OutError)
            || !ReadTrimmedString(
                Object,
                TEXT("semantic_class"),
                Context,
                Candidate.SemanticClass,
                OutError)
            || !ReadTrimmedString(
                Object,
                TEXT("motion_state"),
                Context,
                Candidate.MotionState,
                OutError)
            || !ReadTrimmedString(
                Object,
                TEXT("physics_proxy_mode"),
                Context,
                Candidate.PhysicsProxyMode,
                OutError))
        {
            return false;
        }
        if (Candidate.Authority != TEXT("scenario")
            && Candidate.Authority != TEXT("ue_animation"))
        {
            return Fail(
                OutError,
                FString::Printf(TEXT("%s.authority is not routed to UE visual"), *Context));
        }

        const TSharedPtr<FJsonValue> BodyStableIdValue = Object->TryGetField(TEXT("body_stable_id"));
        if (!BodyStableIdValue.IsValid())
        {
            return Fail(OutError, FString::Printf(TEXT("%s.body_stable_id is unavailable"), *Context));
        }
        if (BodyStableIdValue->Type != EJson::Null
            && (!BodyStableIdValue->TryGetString(Candidate.BodyStableId)
                || Candidate.BodyStableId.IsEmpty()
                || Candidate.BodyStableId.TrimStartAndEnd() != Candidate.BodyStableId))
        {
            return Fail(
                OutError,
                FString::Printf(TEXT("%s.body_stable_id must be null or a trimmed string"), *Context));
        }
        if (!Candidate.BodyStableId.IsEmpty()
            && (!Candidate.BodyStableId.StartsWith(Candidate.State.Id.StableId + TEXT("/"))
                || Candidate.BodyStableId == Candidate.State.Id.StableId + TEXT("/")))
        {
            return Fail(
                OutError,
                FString::Printf(TEXT("%s.body_stable_id must belong to entity_id"), *Context));
        }

        const TSharedPtr<FJsonObject>* TransformPointer = nullptr;
        if (!Object->TryGetObjectField(TEXT("transform"), TransformPointer)
            || TransformPointer == nullptr
            || !TransformPointer->IsValid())
        {
            return Fail(OutError, FString::Printf(TEXT("%s.transform must be an object"), *Context));
        }
        static const TArray<FString> TransformFields = {
            TEXT("position_m"),
            TEXT("quaternion_wxyz"),
        };
        if (!HasExactFields(*TransformPointer, TransformFields, Context + TEXT(".transform"), OutError))
        {
            return false;
        }
        TArray<double> Position;
        TArray<double> Quaternion;
        if (!ReadFiniteArray(
                *TransformPointer,
                TEXT("position_m"),
                3,
                Context + TEXT(".transform"),
                Position,
                OutError)
            || !ReadFiniteArray(
                *TransformPointer,
                TEXT("quaternion_wxyz"),
                4,
                Context + TEXT(".transform"),
                Quaternion,
                OutError))
        {
            return false;
        }
        const double QuaternionNormSquared =
            Quaternion[0] * Quaternion[0]
            + Quaternion[1] * Quaternion[1]
            + Quaternion[2] * Quaternion[2]
            + Quaternion[3] * Quaternion[3];
        if (!FMath::IsFinite(QuaternionNormSquared) || QuaternionNormSquared <= UE_SMALL_NUMBER)
        {
            return Fail(
                OutError,
                FString::Printf(TEXT("%s.transform quaternion must have non-zero norm"), *Context));
        }

        Candidate.State.Id.InstanceId = Candidate.State.Id.StableId;
        Candidate.State.Id.FrameId = TEXT("scenario_root");
        Candidate.State.PositionMeters = FVector(Position[0], Position[1], Position[2]);
        Candidate.State.Rotation = FQuat(
            Quaternion[1],
            Quaternion[2],
            Quaternion[3],
            Quaternion[0]);
        OutEntity = MoveTemp(Candidate);
        return true;
    }

    TSharedPtr<FJsonValue> PositionJson(const FVector& PositionMeters)
    {
        TArray<TSharedPtr<FJsonValue>> Values;
        Values.Add(MakeShared<FJsonValueNumber>(PositionMeters.X));
        Values.Add(MakeShared<FJsonValueNumber>(PositionMeters.Y));
        Values.Add(MakeShared<FJsonValueNumber>(PositionMeters.Z));
        return MakeShared<FJsonValueArray>(MoveTemp(Values));
    }

    FString BuildEvidenceJson(
        const FString& RunId,
        const LingTuSim::FSnapshotEnvelope& Snapshot,
        const TArray<FPreparedScenarioTransform>& Prepared,
        const TMap<FString, TWeakObjectPtr<ALingTuSimScenarioActor>>& Actors)
    {
        TArray<TSharedPtr<FJsonValue>> ActorEvidence;
        double MaximumErrorMeters = 0.0;
        bool bAllActorsVisible = true;
        for (const FPreparedScenarioTransform& Item : Prepared)
        {
            const ALingTuSimScenarioActor* Actor = Actors.FindRef(Item.Entity.State.Id.StableId).Get();
            if (!IsValid(Actor))
            {
                bAllActorsVisible = false;
                continue;
            }
            const bool bVisible = !Actor->IsHidden();
            bAllActorsVisible = bAllActorsVisible && bVisible;
            const FVector ObservedLocationCm = Actor->GetActorLocation();
            const FVector ObservedPositionMeters(
                ObservedLocationCm.X / 100.0,
                -ObservedLocationCm.Y / 100.0,
                ObservedLocationCm.Z / 100.0);
            const double ErrorMeters = FVector::Distance(
                Actor->GetActorLocation(),
                Item.WorldTransform.GetLocation()) / 100.0;
            MaximumErrorMeters = FMath::Max(MaximumErrorMeters, ErrorMeters);

            TSharedPtr<FJsonObject> ActorObject = MakeShared<FJsonObject>();
            ActorObject->SetStringField(TEXT("entity_id"), Item.Entity.State.Id.StableId);
            ActorObject->SetStringField(TEXT("stable_id"), Item.Entity.State.Id.StableId);
            ActorObject->SetStringField(TEXT("authority"), Item.Entity.Authority);
            ActorObject->SetStringField(TEXT("semantic_class"), Item.Entity.SemanticClass);
            ActorObject->SetNumberField(TEXT("source_epoch"), static_cast<double>(Item.Entity.SourceEpoch));
            ActorObject->SetBoolField(TEXT("visible"), bVisible);
            ActorObject->SetField(TEXT("expected_position_m"), PositionJson(Item.Entity.State.PositionMeters));
            ActorObject->SetField(TEXT("observed_position_m"), PositionJson(ObservedPositionMeters));
            ActorObject->SetNumberField(TEXT("position_error_m"), ErrorMeters);
            ActorEvidence.Add(MakeShared<FJsonValueObject>(MoveTemp(ActorObject)));
        }

        TSharedPtr<FJsonObject> Root = MakeShared<FJsonObject>();
        Root->SetStringField(TEXT("schema"), TEXT("lingtu.sim.scenario-visual-evidence.v1"));
        Root->SetStringField(TEXT("session_id"), Snapshot.SessionId);
        Root->SetNumberField(TEXT("model_generation"), static_cast<double>(Snapshot.ModelGeneration));
        Root->SetNumberField(TEXT("reset_generation"), static_cast<double>(Snapshot.ResetGeneration));
        Root->SetNumberField(TEXT("sequence"), static_cast<double>(Snapshot.Sequence));
        Root->SetNumberField(TEXT("sim_time_ns"), static_cast<double>(Snapshot.SimTimeNs));
        Root->SetStringField(TEXT("source"), TEXT("ue_registry_applied"));
        Root->SetStringField(TEXT("input_source"), TEXT("canonical_scenario_snapshot"));
        Root->SetStringField(TEXT("run_id"), RunId);
        Root->SetStringField(TEXT("basis"), TEXT("snapshot_pose_applied_to_unreal_actor"));
        Root->SetNumberField(TEXT("position_tolerance_m"), PositionToleranceMeters);
        Root->SetNumberField(TEXT("maximum_position_error_m"), MaximumErrorMeters);
        const bool bCompleteActorSet = ActorEvidence.Num() == Prepared.Num();
        Root->SetBoolField(
            TEXT("within_tolerance"),
            bCompleteActorSet && MaximumErrorMeters <= PositionToleranceMeters);
        Root->SetNumberField(TEXT("expected_actor_count"), Prepared.Num());
        Root->SetNumberField(TEXT("actor_count"), ActorEvidence.Num());
        Root->SetBoolField(TEXT("complete_actor_set"), bCompleteActorSet);
        Root->SetBoolField(TEXT("all_actors_visible"), bAllActorsVisible && bCompleteActorSet);
        Root->SetArrayField(TEXT("actors"), MoveTemp(ActorEvidence));

        FString Json;
        const TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&Json);
        FJsonSerializer::Serialize(Root.ToSharedRef(), Writer);
        return Json + TEXT("\n");
    }
}

bool LingTuSim::Visual::FScenarioVisualActorRegistry::BindSession(
    UWorld& World,
    const FString& SessionId,
    const uint64 ModelGeneration)
{
    check(IsInGameThread());
    if (SessionId.IsEmpty())
    {
        return false;
    }
    const bool bChanged = !bIsBound
        || SessionId != BoundSessionId
        || ModelGeneration != BoundModelGeneration;
    if (bChanged)
    {
        DestroyActors(World);
        LastAppliedResetGeneration = 0;
        LastAppliedSimTimeNs = 0;
        bHasAppliedSnapshot = false;
        LastEvidenceJson.Reset();
    }
    BoundSessionId = SessionId;
    BoundModelGeneration = ModelGeneration;
    bIsBound = true;
    SnapshotGate.BindSession(SessionId, ModelGeneration);
    return true;
}

bool LingTuSim::Visual::FScenarioVisualActorRegistry::BindRunId(
    const FString& RunId)
{
    check(IsInGameThread());
    if (!IsCanonicalRunId(RunId))
    {
        return false;
    }
    if (!BoundRunId.IsEmpty() && BoundRunId != RunId)
    {
        return false;
    }
    BoundRunId = RunId;
    return true;
}

void LingTuSim::Visual::FScenarioVisualActorRegistry::Clear(UWorld& World)
{
    check(IsInGameThread());
    DestroyActors(World);
    BoundSessionId.Reset();
    BoundRunId.Reset();
    BoundModelGeneration = 0;
    LastAppliedResetGeneration = 0;
    LastAppliedSimTimeNs = 0;
    bIsBound = false;
    bHasAppliedSnapshot = false;
    SnapshotGate.Clear();
    LastEvidenceJson.Reset();
}

LingTuSim::Visual::EScenarioVisualApplyResult
LingTuSim::Visual::FScenarioVisualActorRegistry::ApplySnapshotJson(
    UWorld& World,
    const FString& SnapshotJson,
    FString& OutError)
{
    check(IsInGameThread());
    OutError.Reset();
    if (BoundRunId.IsEmpty())
    {
        OutError = TEXT("scenario visual run_id is not bound");
        return EScenarioVisualApplyResult::Unbound;
    }

    TSharedPtr<FJsonObject> Root;
    const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(SnapshotJson);
    if (!FJsonSerializer::Deserialize(Reader, Root) || !Root.IsValid())
    {
        OutError = TEXT("scenario snapshot is not valid JSON");
        return EScenarioVisualApplyResult::InvalidSnapshot;
    }
    static const TArray<FString> RootFields = {
        TEXT("schema"),
        TEXT("session_id"),
        TEXT("model_generation"),
        TEXT("reset_generation"),
        TEXT("sequence"),
        TEXT("sim_time_ns"),
        TEXT("entities"),
    };
    if (!HasExactFields(Root, RootFields, TEXT("scenario snapshot"), OutError))
    {
        return EScenarioVisualApplyResult::InvalidSnapshot;
    }

    FString Schema;
    LingTuSim::FSnapshotEnvelope Snapshot;
    uint64 SimTimeNs = 0;
    if (!ReadTrimmedString(Root, TEXT("schema"), TEXT("scenario snapshot"), Schema, OutError)
        || Schema != TEXT("lingtu.sim.scenario-snapshot.v1")
        || !ReadTrimmedString(
            Root,
            TEXT("session_id"),
            TEXT("scenario snapshot"),
            Snapshot.SessionId,
            OutError)
        || !ReadNonNegativeInteger(
            Root,
            TEXT("model_generation"),
            TEXT("scenario snapshot"),
            Snapshot.ModelGeneration,
            OutError)
        || !ReadNonNegativeInteger(
            Root,
            TEXT("reset_generation"),
            TEXT("scenario snapshot"),
            Snapshot.ResetGeneration,
            OutError)
        || !ReadNonNegativeInteger(
            Root,
            TEXT("sequence"),
            TEXT("scenario snapshot"),
            Snapshot.Sequence,
            OutError)
        || !ReadNonNegativeInteger(
            Root,
            TEXT("sim_time_ns"),
            TEXT("scenario snapshot"),
            SimTimeNs,
            OutError)
        || SimTimeNs > static_cast<uint64>(MAX_int64))
    {
        if (OutError.IsEmpty())
        {
            OutError = TEXT("scenario snapshot has invalid schema or sim_time_ns");
        }
        return EScenarioVisualApplyResult::InvalidSnapshot;
    }
    Snapshot.SimTimeNs = static_cast<int64>(SimTimeNs);
    if (Snapshot.SessionId.IsEmpty() || Snapshot.SessionId.Len() > 63)
    {
        OutError = TEXT("scenario snapshot session_id is invalid");
        return EScenarioVisualApplyResult::InvalidSnapshot;
    }
    for (const TCHAR Character : Snapshot.SessionId)
    {
        if (FChar::IsWhitespace(Character) || Character == TEXT('\0'))
        {
            OutError = TEXT("scenario snapshot session_id is invalid");
            return EScenarioVisualApplyResult::InvalidSnapshot;
        }
    }

    // This presentation sink can start after the dispatcher emitted sequence 0
    // and can observe only the latest value from its capacity-one mailbox. The
    // gate therefore accepts the first observed snapshot and the first packet
    // of a higher reset at any sequence, then rejects all duplicate/backward
    // packets. ScenarioRuntime/CompositeScenarioDispatcher remain authority for
    // contiguous reset epochs and sequence-zero emission.
    const ESnapshotGateResult GateResult = SnapshotGate.Evaluate(Snapshot);
    switch (GateResult)
    {
    case ESnapshotGateResult::Unbound:
        return EScenarioVisualApplyResult::Unbound;
    case ESnapshotGateResult::SessionMismatch:
        return EScenarioVisualApplyResult::SessionMismatch;
    case ESnapshotGateResult::OldModel:
        return EScenarioVisualApplyResult::OldModel;
    case ESnapshotGateResult::FutureModel:
        return EScenarioVisualApplyResult::FutureModel;
    case ESnapshotGateResult::OldReset:
        return EScenarioVisualApplyResult::OldReset;
    case ESnapshotGateResult::OldSequence:
        return EScenarioVisualApplyResult::OldSequence;
    case ESnapshotGateResult::Accept:
        break;
    }
    if (bHasAppliedSnapshot
        && Snapshot.ResetGeneration == LastAppliedResetGeneration
        && Snapshot.SimTimeNs <= LastAppliedSimTimeNs)
    {
        OutError = TEXT("scenario snapshot sim_time_ns is stale");
        return EScenarioVisualApplyResult::OldSequence;
    }

    const TArray<TSharedPtr<FJsonValue>>* EntityValues = nullptr;
    if (!Root->TryGetArrayField(TEXT("entities"), EntityValues)
        || EntityValues == nullptr
        || EntityValues->IsEmpty())
    {
        OutError = TEXT("scenario snapshot entities must be a non-empty array");
        return EScenarioVisualApplyResult::InvalidSnapshot;
    }

    TArray<FPreparedScenarioTransform> Prepared;
    TSet<FString> StableIds;
    Prepared.Reserve(EntityValues->Num());
    for (int32 Index = 0; Index < EntityValues->Num(); ++Index)
    {
        FPreparedScenarioTransform& Item = Prepared.AddDefaulted_GetRef();
        if (!ParseEntity((*EntityValues)[Index], Index, Item.Entity, OutError))
        {
            return EScenarioVisualApplyResult::InvalidSnapshot;
        }
        const FString& StableId = Item.Entity.State.Id.StableId;
        if (StableIds.Contains(StableId))
        {
            OutError = FString::Printf(TEXT("duplicate scenario stable_id '%s'"), *StableId);
            return EScenarioVisualApplyResult::DuplicateStableId;
        }
        StableIds.Add(StableId);
        Snapshot.Entities.Add(Item.Entity.State);
        if (!FCoordinateConverter::TryMakeWorldTransform(
                Item.Entity.State,
                FVector::OneVector,
                Item.WorldTransform))
        {
            OutError = FString::Printf(TEXT("scenario transform rejected for '%s'"), *StableId);
            return EScenarioVisualApplyResult::TransformRejected;
        }
    }

    const bool bRequiresRebuild = !bHasAppliedSnapshot
        || Snapshot.ResetGeneration != LastAppliedResetGeneration;
    if (!bRequiresRebuild)
    {
        if (StableIds.Num() != Actors.Num())
        {
            OutError = TEXT("scenario entity set changed outside reset");
            return EScenarioVisualApplyResult::EntitySetMismatch;
        }
        for (const FString& StableId : StableIds)
        {
            if (!IsValid(Actors.FindRef(StableId).Get()))
            {
                OutError = TEXT("scenario entity set changed outside reset");
                return EScenarioVisualApplyResult::EntitySetMismatch;
            }
        }
    }

    if (bRequiresRebuild)
    {
        TMap<FString, TWeakObjectPtr<ALingTuSimScenarioActor>> CandidateActors;
        for (const FPreparedScenarioTransform& Item : Prepared)
        {
            FActorSpawnParameters Parameters;
            Parameters.Name = MakeUniqueObjectName(
                &World,
                ALingTuSimScenarioActor::StaticClass(),
                MakeScenarioActorBaseName(Item.Entity.State.Id.StableId));
            Parameters.SpawnCollisionHandlingOverride =
                ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
            ALingTuSimScenarioActor* Actor = World.SpawnActor<ALingTuSimScenarioActor>(
                ALingTuSimScenarioActor::StaticClass(),
                Item.WorldTransform,
                Parameters);
            if (!IsValid(Actor)
                || !Actor->ConfigureSnapshotIdentity(
                    Item.Entity.State.Id.StableId,
                    Item.Entity.Authority,
                    Item.Entity.SourceEpoch,
                    Item.Entity.SemanticClass,
                    Item.Entity.MotionState,
                    Item.Entity.PhysicsProxyMode,
                    Item.Entity.BodyStableId))
            {
                for (const TPair<FString, TWeakObjectPtr<ALingTuSimScenarioActor>>& Pair : CandidateActors)
                {
                    if (ALingTuSimScenarioActor* Candidate = Pair.Value.Get())
                    {
                        Candidate->Destroy();
                    }
                }
                if (IsValid(Actor))
                {
                    Actor->Destroy();
                }
                OutError = TEXT("failed to spawn a complete scenario actor generation");
                return EScenarioVisualApplyResult::ActorSpawnFailed;
            }
            CandidateActors.Add(Item.Entity.State.Id.StableId, Actor);
        }
        DestroyActors(World);
        Actors = MoveTemp(CandidateActors);
        for (const TPair<FString, TWeakObjectPtr<ALingTuSimScenarioActor>>& Pair : Actors)
        {
            if (ALingTuSimScenarioActor* Actor = Pair.Value.Get())
            {
                Actor->SetActorHiddenInGame(false);
            }
        }
    }
    else
    {
        TArray<FPreviousScenarioActorState> PreviousActorStates;
        PreviousActorStates.Reserve(Prepared.Num());
        for (const FPreparedScenarioTransform& Item : Prepared)
        {
            ALingTuSimScenarioActor* Actor = Actors.FindRef(Item.Entity.State.Id.StableId).Get();
            if (!IsValid(Actor)
                || Actor->GetRootComponent() == nullptr
                || !Actor->CanConfigureSnapshotIdentity(
                    Item.Entity.State.Id.StableId,
                    Item.Entity.Authority,
                    Item.Entity.SourceEpoch,
                    Item.Entity.SemanticClass,
                    Item.Entity.MotionState,
                    Item.Entity.PhysicsProxyMode))
            {
                OutError = TEXT("scenario actor frame failed preflight validation");
                return EScenarioVisualApplyResult::TransformRejected;
            }

            FPreviousScenarioActorState& Previous = PreviousActorStates.AddDefaulted_GetRef();
            Previous.Actor = Actor;
            Previous.WorldTransform = Actor->GetActorTransform();
            Previous.StableId = Actor->StableId;
            Previous.Authority = Actor->Authority;
            Previous.SourceEpoch = static_cast<uint64>(Actor->SourceEpoch);
            Previous.SemanticClass = Actor->SemanticClass;
            Previous.MotionState = Actor->MotionState;
            Previous.PhysicsProxyMode = Actor->PhysicsProxyMode;
            Previous.BodyStableId = Actor->BodyStableId;
        }

        bool bAppliedCompleteFrame = true;
        for (const FPreparedScenarioTransform& Item : Prepared)
        {
            ALingTuSimScenarioActor* Actor = Actors.FindRef(Item.Entity.State.Id.StableId).Get();
            if (!Actor->ConfigureSnapshotIdentity(
                    Item.Entity.State.Id.StableId,
                    Item.Entity.Authority,
                    Item.Entity.SourceEpoch,
                    Item.Entity.SemanticClass,
                    Item.Entity.MotionState,
                    Item.Entity.PhysicsProxyMode,
                    Item.Entity.BodyStableId)
                || !Actor->SetActorTransform(
                    Item.WorldTransform,
                    false,
                    nullptr,
                    ETeleportType::TeleportPhysics))
            {
                bAppliedCompleteFrame = false;
                break;
            }
        }
        if (!bAppliedCompleteFrame)
        {
            bool bRestoredPreviousFrame = true;
            for (const FPreviousScenarioActorState& Previous : PreviousActorStates)
            {
                if (!IsValid(Previous.Actor)
                    || !Previous.Actor->ConfigureSnapshotIdentity(
                        Previous.StableId,
                        Previous.Authority,
                        Previous.SourceEpoch,
                        Previous.SemanticClass,
                        Previous.MotionState,
                        Previous.PhysicsProxyMode,
                        Previous.BodyStableId)
                    || !Previous.Actor->SetActorTransform(
                        Previous.WorldTransform,
                        false,
                        nullptr,
                        ETeleportType::TeleportPhysics))
                {
                    bRestoredPreviousFrame = false;
                }
            }
            OutError = bRestoredPreviousFrame
                ? TEXT("failed to apply a complete scenario actor frame; previous frame restored")
                : TEXT("failed to apply a complete scenario actor frame and restore previous frame");
            return EScenarioVisualApplyResult::TransformRejected;
        }
    }

    LastEvidenceJson = BuildEvidenceJson(BoundRunId, Snapshot, Prepared, Actors);
    SnapshotGate.Commit(Snapshot);
    LastAppliedResetGeneration = Snapshot.ResetGeneration;
    LastAppliedSimTimeNs = Snapshot.SimTimeNs;
    bHasAppliedSnapshot = true;
    return EScenarioVisualApplyResult::Accepted;
}

ALingTuSimScenarioActor* LingTuSim::Visual::FScenarioVisualActorRegistry::FindActor(
    const FString& StableId) const
{
    return Actors.FindRef(StableId).Get();
}

void LingTuSim::Visual::FScenarioVisualActorRegistry::DestroyActors(UWorld& World)
{
    (void)World;
    for (const TPair<FString, TWeakObjectPtr<ALingTuSimScenarioActor>>& Pair : Actors)
    {
        if (ALingTuSimScenarioActor* Actor = Pair.Value.Get())
        {
            Actor->Destroy();
        }
    }
    Actors.Reset();
}

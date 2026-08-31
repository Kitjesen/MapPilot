#include "LingTuSimBundleLoader.h"

#include "Dom/JsonObject.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

namespace LingTuSim
{
    namespace
    {
        struct FParsedArtifact
        {
            FString Path;
            FString SessionId;
        };

        struct FArtifactContract
        {
            const TCHAR* Filename;
            const TCHAR* PrimarySchema;
            const TCHAR* AlternateSchema;
            bool bRequired;
            FString FSessionBundleView::* PathMember;
        };

        bool Fail(
            FRuntimeLoadError& OutError,
            const ERuntimeLoadErrorCode Code,
            const FString& Source,
            const FString& Message)
        {
            OutError.Code = Code;
            OutError.Source = Source;
            OutError.Message = Message;
            return false;
        }

        bool IsValidSessionId(const FString& Value)
        {
            if (Value.IsEmpty() || Value.Len() > 63)
            {
                return false;
            }

            for (int32 Index = 0; Index < Value.Len(); ++Index)
            {
                const TCHAR Character = Value[Index];
                const bool bAsciiAlphaNumeric =
                    (Character >= TEXT('A') && Character <= TEXT('Z'))
                    || (Character >= TEXT('a') && Character <= TEXT('z'))
                    || (Character >= TEXT('0') && Character <= TEXT('9'));
                if (!bAsciiAlphaNumeric
                    && (Index == 0
                        || (Character != TEXT('_')
                            && Character != TEXT('.')
                            && Character != TEXT('-'))))
                {
                    return false;
                }
            }
            return true;
        }

        bool LoadJsonArtifact(
            const FString& Path,
            const FString& PrimarySchema,
            const FString& AlternateSchema,
            FParsedArtifact& OutArtifact,
            FRuntimeLoadError& OutError)
        {
            if (!IFileManager::Get().FileExists(*Path))
            {
                return Fail(
                    OutError,
                    ERuntimeLoadErrorCode::MissingArtifact,
                    Path,
                    FString::Printf(TEXT("Required SessionBundle artifact is missing: %s"), *Path));
            }

            FString JsonText;
            if (!FFileHelper::LoadFileToString(JsonText, *Path))
            {
                return Fail(
                    OutError,
                    ERuntimeLoadErrorCode::ReadFailed,
                    Path,
                    FString::Printf(TEXT("Unable to read SessionBundle artifact: %s"), *Path));
            }

            TSharedPtr<FJsonObject> RootObject;
            const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(JsonText);
            if (!FJsonSerializer::Deserialize(Reader, RootObject) || !RootObject.IsValid())
            {
                const FString Detail = Reader->GetErrorMessage();
                return Fail(
                    OutError,
                    ERuntimeLoadErrorCode::InvalidJson,
                    Path,
                    Detail.IsEmpty()
                        ? FString::Printf(TEXT("Artifact is not a JSON object: %s"), *Path)
                        : FString::Printf(TEXT("Invalid JSON in %s: %s"), *Path, *Detail));
            }

            FString ActualSchema;
            if (!RootObject->TryGetStringField(TEXT("schema"), ActualSchema)
                || (ActualSchema != PrimarySchema
                    && (AlternateSchema.IsEmpty() || ActualSchema != AlternateSchema)))
            {
                const FString ExpectedSchemas = AlternateSchema.IsEmpty()
                    ? FString::Printf(TEXT("'%s'"), *PrimarySchema)
                    : FString::Printf(TEXT("'%s' or '%s'"), *PrimarySchema, *AlternateSchema);
                return Fail(
                    OutError,
                    ERuntimeLoadErrorCode::SchemaMismatch,
                    Path,
                    FString::Printf(
                        TEXT("Artifact schema mismatch in %s: expected %s, got '%s'."),
                        *Path,
                        *ExpectedSchemas,
                        ActualSchema.IsEmpty() ? TEXT("<missing>") : *ActualSchema));
            }

            FString SessionId;
            if (!RootObject->TryGetStringField(TEXT("session_id"), SessionId)
                || !IsValidSessionId(SessionId))
            {
                return Fail(
                    OutError,
                    ERuntimeLoadErrorCode::InvalidField,
                    Path,
                    FString::Printf(
                        TEXT("Artifact session_id in %s is invalid."),
                        *Path));
            }

            FParsedArtifact Candidate;
            Candidate.Path = Path;
            Candidate.SessionId = MoveTemp(SessionId);
            OutArtifact = MoveTemp(Candidate);
            return true;
        }
    }

    bool FSessionBundleLoader::LoadSessionBundle(
        const FString& BundleDirectory,
        FSessionBundleView& OutBundle,
        FRuntimeLoadError& OutError)
    {
        OutError.Reset();
        if (BundleDirectory.IsEmpty())
        {
            return Fail(
                OutError,
                ERuntimeLoadErrorCode::InvalidArgument,
                TEXT("SessionBundle"),
                TEXT("SessionBundle directory must not be empty."));
        }

        FString NormalizedDirectory = FPaths::ConvertRelativePathToFull(BundleDirectory);
        FPaths::NormalizeDirectoryName(NormalizedDirectory);
        if (!IFileManager::Get().DirectoryExists(*NormalizedDirectory))
        {
            return Fail(
                OutError,
                ERuntimeLoadErrorCode::MissingArtifact,
                NormalizedDirectory,
                FString::Printf(TEXT("SessionBundle directory does not exist: %s"), *NormalizedDirectory));
        }

        FSessionBundleView Candidate;
        static const FArtifactContract Contracts[] = {
            {
                TEXT("physics.plan.json"),
                TEXT("lingtu.sim.physics-plan.v1"),
                TEXT("lingtu.sim.physics-plan.v2"),
                true,
                &FSessionBundleView::PhysicsPlanPath,
            },
            {
                TEXT("visual.plan.json"),
                TEXT("lingtu.sim.visual-plan.v1"),
                TEXT("lingtu.sim.visual-plan.v2"),
                true,
                &FSessionBundleView::VisualPlanPath,
            },
            {
                TEXT("sensor.plan.json"),
                TEXT("lingtu.sim.sensor-plan.v1"),
                nullptr,
                true,
                &FSessionBundleView::SensorPlanPath,
            },
            {
                TEXT("control.plan.json"),
                TEXT("lingtu.sim.control-plan.v1"),
                nullptr,
                true,
                &FSessionBundleView::ControlPlanPath,
            },
            {
                TEXT("scenario.plan.json"),
                TEXT("lingtu.sim.scenario-plan.v1"),
                nullptr,
                false,
                &FSessionBundleView::ScenarioPlanPath,
            },
            {
                TEXT("transport.intent.json"),
                TEXT("lingtu.sim.transport-intent.v1"),
                nullptr,
                true,
                &FSessionBundleView::TransportIntentPath,
            },
        };

        for (const FArtifactContract& Contract : Contracts)
        {
            const FString Path = FPaths::Combine(NormalizedDirectory, Contract.Filename);
            if (!Contract.bRequired && !IFileManager::Get().FileExists(*Path))
            {
                continue;
            }

            FParsedArtifact Artifact;
            if (!LoadJsonArtifact(
                    Path,
                    Contract.PrimarySchema,
                    Contract.AlternateSchema != nullptr ? Contract.AlternateSchema : TEXT(""),
                    Artifact,
                    OutError))
            {
                return false;
            }
            if (Candidate.SessionId.IsEmpty())
            {
                Candidate.SessionId = Artifact.SessionId;
            }
            else if (Artifact.SessionId != Candidate.SessionId)
            {
                return Fail(
                    OutError,
                    ERuntimeLoadErrorCode::InvalidField,
                    Path,
                    FString::Printf(
                        TEXT("Artifact session_id in %s does not match the other plans."),
                        *Path));
            }

            Candidate.*Contract.PathMember = MoveTemp(Artifact.Path);
        }

        if (!Candidate.IsBound())
        {
            return Fail(
                OutError,
                ERuntimeLoadErrorCode::InvalidField,
                NormalizedDirectory,
                TEXT("SessionBundle did not bind every required compiled artifact."));
        }

        OutBundle = MoveTemp(Candidate);
        return true;
    }

    namespace
    {
        bool ReadNonNegativeUint64Field(
            const TSharedPtr<FJsonObject>& Object,
            const TCHAR* FieldName,
            const FString& Source,
            uint64& OutValue,
            FRuntimeLoadError& OutError)
        {
            const TSharedPtr<FJsonValue> Value = Object->TryGetField(FieldName);
            uint64 Candidate = 0;
            if (!Value.IsValid()
                || Value->Type != EJson::Number
                || !Value->TryGetNumber(Candidate))
            {
                return Fail(
                    OutError,
                    ERuntimeLoadErrorCode::InvalidField,
                    Source,
                    FString::Printf(
                        TEXT("%s must be a non-negative integer in the uint64 range."),
                        FieldName));
            }
            OutValue = Candidate;
            return true;
        }

        bool ReadNonNegativeInt64Field(
            const TSharedPtr<FJsonObject>& Object,
            const TCHAR* FieldName,
            const FString& Source,
            int64& OutValue,
            FRuntimeLoadError& OutError)
        {
            const TSharedPtr<FJsonValue> Value = Object->TryGetField(FieldName);
            int64 Candidate = 0;
            if (!Value.IsValid()
                || Value->Type != EJson::Number
                || !Value->TryGetNumber(Candidate)
                || Candidate < 0)
            {
                return Fail(
                    OutError,
                    ERuntimeLoadErrorCode::InvalidField,
                    Source,
                    FString::Printf(
                        TEXT("%s must be a non-negative integer in the int64 range."),
                        FieldName));
            }
            OutValue = Candidate;
            return true;
        }

        bool ReadFiniteArray(
            const TSharedPtr<FJsonObject>& Object,
            const TCHAR* FieldName,
            const int32 ExpectedCount,
            const bool bRequired,
            const FString& Source,
            TArray<double>& OutValues,
            FRuntimeLoadError& OutError)
        {
            const TArray<TSharedPtr<FJsonValue>>* Values = nullptr;
            if (!Object->TryGetArrayField(FieldName, Values))
            {
                if (!bRequired && !Object->HasField(FieldName))
                {
                    OutValues.Reset();
                    return true;
                }
                return Fail(
                    OutError,
                    ERuntimeLoadErrorCode::InvalidField,
                    Source,
                    FString::Printf(TEXT("%s must be an array."), FieldName));
            }
            if (Values == nullptr || Values->Num() != ExpectedCount)
            {
                return Fail(
                    OutError,
                    ERuntimeLoadErrorCode::InvalidField,
                    Source,
                    FString::Printf(
                        TEXT("%s must contain exactly %d numeric values."),
                        FieldName,
                        ExpectedCount));
            }

            TArray<double> Candidate;
            Candidate.Reserve(ExpectedCount);
            for (int32 ValueIndex = 0; ValueIndex < Values->Num(); ++ValueIndex)
            {
                double Value = 0.0;
                if (!(*Values)[ValueIndex].IsValid()
                    || !(*Values)[ValueIndex]->TryGetNumber(Value)
                    || !FMath::IsFinite(Value))
                {
                    return Fail(
                        OutError,
                        ERuntimeLoadErrorCode::InvalidField,
                        Source,
                        FString::Printf(
                            TEXT("%s[%d] must be finite numeric data."),
                            FieldName,
                            ValueIndex));
                }
                Candidate.Add(Value);
            }
            OutValues = MoveTemp(Candidate);
            return true;
        }

        bool ReadRequiredEntityId(
            const TSharedPtr<FJsonObject>& Object,
            const TCHAR* FieldName,
            const FString& Source,
            FString& OutValue,
            FRuntimeLoadError& OutError)
        {
            FString Candidate;
            if (!Object->TryGetStringField(FieldName, Candidate) || Candidate.IsEmpty())
            {
                return Fail(
                    OutError,
                    ERuntimeLoadErrorCode::InvalidField,
                    Source,
                    FString::Printf(TEXT("%s must be a non-empty string."), FieldName));
            }
            OutValue = MoveTemp(Candidate);
            return true;
        }

        bool HasOnlyFields(
            const TSharedPtr<FJsonObject>& Object,
            const TSet<FString>& AllowedFields,
            const FString& Source,
            FRuntimeLoadError& OutError)
        {
            for (const TPair<FString, TSharedPtr<FJsonValue>>& Field : Object->Values)
            {
                if (!AllowedFields.Contains(Field.Key))
                {
                    return Fail(
                        OutError,
                        ERuntimeLoadErrorCode::InvalidField,
                        Source,
                        FString::Printf(TEXT("Unknown field '%s'."), *Field.Key));
                }
            }
            return true;
        }

        bool ParseEntity(
            const TSharedPtr<FJsonValue>& EntityValue,
            const FString& Source,
            FEntityState& OutEntity,
            FRuntimeLoadError& OutError)
        {
            const TSharedPtr<FJsonObject>* EntityObjectPointer = nullptr;
            if (!EntityValue.IsValid()
                || !EntityValue->TryGetObject(EntityObjectPointer)
                || EntityObjectPointer == nullptr
                || !EntityObjectPointer->IsValid())
            {
                return Fail(
                    OutError,
                    ERuntimeLoadErrorCode::InvalidField,
                    Source,
                    TEXT("Snapshot entity must be a JSON object."));
            }
            const TSharedPtr<FJsonObject>& EntityObject = *EntityObjectPointer;

            static const TSet<FString> AllowedBodyFields = {
                TEXT("body_id"),
                TEXT("name"),
                TEXT("stable_id"),
                TEXT("instance_id"),
                TEXT("frame_id"),
                TEXT("position_m"),
                TEXT("quaternion_wxyz"),
                TEXT("linear_velocity_mps"),
                TEXT("angular_velocity_rps"),
            };
            if (!HasOnlyFields(EntityObject, AllowedBodyFields, Source, OutError))
            {
                return false;
            }

            uint64 BodyId = 0;
            FString BodyName;
            if ((EntityObject->HasField(TEXT("body_id"))
                    && !ReadNonNegativeUint64Field(
                        EntityObject,
                        TEXT("body_id"),
                        Source,
                        BodyId,
                        OutError))
                || (EntityObject->HasField(TEXT("name"))
                    && !ReadRequiredEntityId(
                        EntityObject,
                        TEXT("name"),
                        Source,
                        BodyName,
                        OutError)))
            {
                return false;
            }

            FEntityState Candidate;
            if (!ReadRequiredEntityId(
                    EntityObject,
                    TEXT("stable_id"),
                    Source,
                    Candidate.Id.StableId,
                    OutError)
                || !ReadRequiredEntityId(
                    EntityObject,
                    TEXT("instance_id"),
                    Source,
                    Candidate.Id.InstanceId,
                    OutError)
                || !ReadRequiredEntityId(
                    EntityObject,
                    TEXT("frame_id"),
                    Source,
                    Candidate.Id.FrameId,
                    OutError))
            {
                return false;
            }

            TArray<double> Position;
            TArray<double> Quaternion;
            TArray<double> LinearVelocity;
            TArray<double> AngularVelocity;
            if (!ReadFiniteArray(
                    EntityObject,
                    TEXT("position_m"),
                    3,
                    true,
                    Source,
                    Position,
                    OutError)
                || !ReadFiniteArray(
                    EntityObject,
                    TEXT("quaternion_wxyz"),
                    4,
                    true,
                    Source,
                    Quaternion,
                    OutError)
                || !ReadFiniteArray(
                    EntityObject,
                    TEXT("linear_velocity_mps"),
                    3,
                    true,
                    Source,
                    LinearVelocity,
                    OutError)
                || !ReadFiniteArray(
                    EntityObject,
                    TEXT("angular_velocity_rps"),
                    3,
                    true,
                    Source,
                    AngularVelocity,
                    OutError))
            {
                return false;
            }

            const double QuaternionNormSquared =
                Quaternion[0] * Quaternion[0]
                + Quaternion[1] * Quaternion[1]
                + Quaternion[2] * Quaternion[2]
                + Quaternion[3] * Quaternion[3];
            if (!FMath::IsFinite(QuaternionNormSquared) || QuaternionNormSquared <= 0.0)
            {
                return Fail(
                    OutError,
                    ERuntimeLoadErrorCode::InvalidField,
                    Source,
                    TEXT("quaternion_wxyz must have a finite, non-zero norm."));
            }

            Candidate.PositionMeters = FVector(Position[0], Position[1], Position[2]);
            Candidate.Rotation = FQuat(
                Quaternion[1],
                Quaternion[2],
                Quaternion[3],
                Quaternion[0]);
            if (LinearVelocity.Num() != 0)
            {
                Candidate.LinearVelocityMetersPerSecond = FVector(
                    LinearVelocity[0],
                    LinearVelocity[1],
                    LinearVelocity[2]);
            }
            if (AngularVelocity.Num() != 0)
            {
                Candidate.AngularVelocityRadiansPerSecond = FVector(
                    AngularVelocity[0],
                    AngularVelocity[1],
                    AngularVelocity[2]);
            }

            OutEntity = MoveTemp(Candidate);
            return true;
        }

        bool ParseSnapshotJsonInternal(
            const FString& SnapshotJson,
            const FString& Source,
            const FString* ExpectedSessionId,
            FSnapshotEnvelope& OutSnapshot,
            FRuntimeLoadError& OutError)
        {
            if (ExpectedSessionId != nullptr
                && !IsValidSessionId(*ExpectedSessionId))
            {
                return Fail(
                    OutError,
                    ERuntimeLoadErrorCode::InvalidArgument,
                    Source,
                    TEXT("Expected session_id is invalid."));
            }

            TSharedPtr<FJsonObject> RootObject;
            const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(SnapshotJson);
            if (!FJsonSerializer::Deserialize(Reader, RootObject) || !RootObject.IsValid())
            {
                const FString Detail = Reader->GetErrorMessage();
                return Fail(
                    OutError,
                    ERuntimeLoadErrorCode::InvalidJson,
                    Source,
                    Detail.IsEmpty()
                        ? TEXT("Snapshot is not a JSON object.")
                        : FString::Printf(TEXT("Invalid snapshot JSON: %s"), *Detail));
            }

            if (RootObject->HasField(TEXT("entities")))
            {
                return Fail(
                    OutError,
                    ERuntimeLoadErrorCode::InvalidField,
                    Source,
                    TEXT("Legacy snapshot field 'entities' is not supported; use 'bodies'."));
            }

            static const TSet<FString> AllowedSnapshotFields = {
                TEXT("schema"),
                TEXT("session_id"),
                TEXT("model_generation"),
                TEXT("reset_generation"),
                TEXT("sequence"),
                TEXT("physics_step"),
                TEXT("sim_time_ns"),
                TEXT("bodies"),
                TEXT("joints"),
                TEXT("actuators"),
                TEXT("sensors"),
            };
            if (!HasOnlyFields(RootObject, AllowedSnapshotFields, Source, OutError))
            {
                return false;
            }

            FString Schema;
            if (!RootObject->TryGetStringField(TEXT("schema"), Schema)
                || Schema != TEXT("lingtu.sim.truth-snapshot.v1"))
            {
                return Fail(
                    OutError,
                    ERuntimeLoadErrorCode::SchemaMismatch,
                    Source,
                    FString::Printf(
                        TEXT("Snapshot schema must be 'lingtu.sim.truth-snapshot.v1', got '%s'."),
                        Schema.IsEmpty() ? TEXT("<missing>") : *Schema));
            }

            FSnapshotEnvelope Candidate;
            if (!RootObject->TryGetStringField(TEXT("session_id"), Candidate.SessionId)
                || !IsValidSessionId(Candidate.SessionId))
            {
                return Fail(
                    OutError,
                    ERuntimeLoadErrorCode::InvalidField,
                    Source,
                    TEXT("Snapshot session_id is invalid."));
            }
            if (ExpectedSessionId != nullptr
                && Candidate.SessionId != *ExpectedSessionId)
            {
                return Fail(
                    OutError,
                    ERuntimeLoadErrorCode::InvalidField,
                    Source,
                    TEXT("Snapshot session_id does not match the loaded SessionBundle."));
            }

            if (!ReadNonNegativeUint64Field(
                    RootObject,
                    TEXT("model_generation"),
                    Source,
                    Candidate.ModelGeneration,
                    OutError)
                || !ReadNonNegativeUint64Field(
                    RootObject,
                    TEXT("reset_generation"),
                    Source,
                    Candidate.ResetGeneration,
                    OutError)
                || !ReadNonNegativeUint64Field(
                    RootObject,
                    TEXT("sequence"),
                    Source,
                    Candidate.Sequence,
                    OutError)
                || !ReadNonNegativeUint64Field(
                    RootObject,
                    TEXT("physics_step"),
                    Source,
                    Candidate.PhysicsStep,
                    OutError)
                || !ReadNonNegativeInt64Field(
                    RootObject,
                    TEXT("sim_time_ns"),
                    Source,
                    Candidate.SimTimeNs,
                    OutError))
            {
                return false;
            }

            const TArray<TSharedPtr<FJsonValue>>* EntityValues = nullptr;
            if (!RootObject->TryGetArrayField(TEXT("bodies"), EntityValues)
                || EntityValues == nullptr)
            {
                return Fail(
                    OutError,
                    ERuntimeLoadErrorCode::InvalidField,
                    Source,
                    TEXT("Snapshot bodies must be an array."));
            }

            // UE consumes body poses only. Keep the other collections type-safe
            // without walking every unused element on each presentation frame.
            static const TCHAR* OptionalArrayFields[] = {
                TEXT("joints"),
                TEXT("actuators"),
                TEXT("sensors"),
            };
            for (const TCHAR* OptionalArrayField : OptionalArrayFields)
            {
                const TArray<TSharedPtr<FJsonValue>>* Values = nullptr;
                if (RootObject->HasField(OptionalArrayField)
                    && !RootObject->TryGetArrayField(OptionalArrayField, Values))
                {
                    return Fail(
                        OutError,
                        ERuntimeLoadErrorCode::InvalidField,
                        Source,
                        FString::Printf(TEXT("Snapshot %s must be an array."), OptionalArrayField));
                }
            }

            Candidate.Entities.Reserve(EntityValues->Num());
            TSet<FString> StableIds;
            for (int32 EntityIndex = 0; EntityIndex < EntityValues->Num(); ++EntityIndex)
            {
                FEntityState Entity;
                const FString EntitySource = FString::Printf(
                    TEXT("%s.bodies[%d]"),
                    *Source,
                    EntityIndex);
                if (!ParseEntity((*EntityValues)[EntityIndex], EntitySource, Entity, OutError))
                {
                    return false;
                }
                if (StableIds.Contains(Entity.Id.StableId))
                {
                    return Fail(
                        OutError,
                        ERuntimeLoadErrorCode::InvalidField,
                        EntitySource,
                        FString::Printf(
                            TEXT("Duplicate snapshot stable_id '%s'."),
                            *Entity.Id.StableId));
                }
                StableIds.Add(Entity.Id.StableId);
                Candidate.Entities.Add(MoveTemp(Entity));
            }

            OutSnapshot = MoveTemp(Candidate);
            return true;
        }
    }

    bool FSessionBundleLoader::LoadSnapshotFile(
        const FString& SnapshotPath,
        const FString& ExpectedSessionId,
        FSnapshotEnvelope& OutSnapshot,
        FRuntimeLoadError& OutError)
    {
        OutError.Reset();
        if (SnapshotPath.IsEmpty())
        {
            return Fail(
                OutError,
                ERuntimeLoadErrorCode::InvalidArgument,
                TEXT("Snapshot"),
                TEXT("Snapshot path must not be empty."));
        }

        const FString NormalizedPath = FPaths::ConvertRelativePathToFull(SnapshotPath);
        if (!IFileManager::Get().FileExists(*NormalizedPath))
        {
            return Fail(
                OutError,
                ERuntimeLoadErrorCode::MissingArtifact,
                NormalizedPath,
                FString::Printf(TEXT("Snapshot file does not exist: %s"), *NormalizedPath));
        }

        FString SnapshotJson;
        if (!FFileHelper::LoadFileToString(SnapshotJson, *NormalizedPath))
        {
            return Fail(
                OutError,
                ERuntimeLoadErrorCode::ReadFailed,
                NormalizedPath,
                FString::Printf(TEXT("Unable to read snapshot file: %s"), *NormalizedPath));
        }
        return ParseSnapshotJsonInternal(
            SnapshotJson,
            NormalizedPath,
            &ExpectedSessionId,
            OutSnapshot,
            OutError);
    }

    bool FSessionBundleLoader::ParseSnapshotJson(
        const FString& SnapshotJson,
        const FString& ExpectedSessionId,
        FSnapshotEnvelope& OutSnapshot,
        FRuntimeLoadError& OutError)
    {
        OutError.Reset();
        return ParseSnapshotJsonInternal(
            SnapshotJson,
            TEXT("<snapshot-json>"),
            &ExpectedSessionId,
            OutSnapshot,
            OutError);
    }

    bool FSessionBundleLoader::ParseSnapshotJson(
        const FString& SnapshotJson,
        FSnapshotEnvelope& OutSnapshot,
        FRuntimeLoadError& OutError)
    {
        OutError.Reset();
        return ParseSnapshotJsonInternal(
            SnapshotJson,
            TEXT("<snapshot-json>"),
            nullptr,
            OutSnapshot,
            OutError);
    }
}

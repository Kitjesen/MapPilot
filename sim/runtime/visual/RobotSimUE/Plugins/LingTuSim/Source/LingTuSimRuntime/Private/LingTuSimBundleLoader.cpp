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
        constexpr int32 Sha256BlockBytes = 64;

        struct FParsedArtifact
        {
            FString Path;
            FString SessionDigest;
            FString Sha256;
        };

        struct FArtifactContract
        {
            const TCHAR* Filename;
            const TCHAR* Schema;
            bool bRequired;
            FString FSessionBundleView::* PathMember;
            FString FSessionBundleView::* DigestMember;
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

        bool IsLowerHexDigest(const FString& Value)
        {
            if (Value.Len() != 64)
            {
                return false;
            }

            for (const TCHAR Character : Value)
            {
                if (!((Character >= TEXT('0') && Character <= TEXT('9'))
                    || (Character >= TEXT('a') && Character <= TEXT('f'))))
                {
                    return false;
                }
            }
            return true;
        }

        uint32 RotateRight(const uint32 Value, const uint32 Count)
        {
            return (Value >> Count) | (Value << (32U - Count));
        }

        bool ComputeSha256(
            const FString& Path,
            FString& OutDigest,
            FRuntimeLoadError& OutError)
        {
            TArray<uint8> Bytes;
            if (!FFileHelper::LoadFileToArray(Bytes, *Path))
            {
                return Fail(
                    OutError,
                    ERuntimeLoadErrorCode::ReadFailed,
                    Path,
                    TEXT("Unable to read artifact bytes for SHA-256."));
            }
            if (Bytes.Num() > MAX_int32 - 72)
            {
                return Fail(
                    OutError,
                    ERuntimeLoadErrorCode::HashFailed,
                    Path,
                    TEXT("Artifact is too large for the in-memory SHA-256 implementation."));
            }

            const uint64 BitLength = static_cast<uint64>(Bytes.Num()) * 8ULL;
            TArray<uint8> Padded = Bytes;
            Padded.Add(0x80U);
            while ((Padded.Num() % Sha256BlockBytes) != 56)
            {
                Padded.Add(0U);
            }
            for (int32 Shift = 56; Shift >= 0; Shift -= 8)
            {
                Padded.Add(static_cast<uint8>((BitLength >> Shift) & 0xffULL));
            }

            static constexpr uint32 RoundConstants[64] = {
                0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U,
                0x3956c25bU, 0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U,
                0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U,
                0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U,
                0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU,
                0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
                0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U,
                0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U,
                0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U,
                0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
                0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U,
                0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
                0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U,
                0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U,
                0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
                0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U,
            };

            uint32 Hash[8] = {
                0x6a09e667U,
                0xbb67ae85U,
                0x3c6ef372U,
                0xa54ff53aU,
                0x510e527fU,
                0x9b05688cU,
                0x1f83d9abU,
                0x5be0cd19U,
            };

            for (int32 Offset = 0; Offset < Padded.Num(); Offset += Sha256BlockBytes)
            {
                uint32 Words[64] = {};
                for (int32 Index = 0; Index < 16; ++Index)
                {
                    const int32 WordOffset = Offset + Index * 4;
                    Words[Index] = (static_cast<uint32>(Padded[WordOffset]) << 24U)
                        | (static_cast<uint32>(Padded[WordOffset + 1]) << 16U)
                        | (static_cast<uint32>(Padded[WordOffset + 2]) << 8U)
                        | static_cast<uint32>(Padded[WordOffset + 3]);
                }
                for (int32 Index = 16; Index < 64; ++Index)
                {
                    const uint32 Sigma0 = RotateRight(Words[Index - 15], 7U)
                        ^ RotateRight(Words[Index - 15], 18U)
                        ^ (Words[Index - 15] >> 3U);
                    const uint32 Sigma1 = RotateRight(Words[Index - 2], 17U)
                        ^ RotateRight(Words[Index - 2], 19U)
                        ^ (Words[Index - 2] >> 10U);
                    Words[Index] = Words[Index - 16] + Sigma0 + Words[Index - 7] + Sigma1;
                }

                uint32 A = Hash[0];
                uint32 B = Hash[1];
                uint32 C = Hash[2];
                uint32 D = Hash[3];
                uint32 E = Hash[4];
                uint32 F = Hash[5];
                uint32 G = Hash[6];
                uint32 H = Hash[7];

                for (int32 Index = 0; Index < 64; ++Index)
                {
                    const uint32 Sum1 = RotateRight(E, 6U)
                        ^ RotateRight(E, 11U)
                        ^ RotateRight(E, 25U);
                    const uint32 Choose = (E & F) ^ ((~E) & G);
                    const uint32 Temp1 = H + Sum1 + Choose + RoundConstants[Index] + Words[Index];
                    const uint32 Sum0 = RotateRight(A, 2U)
                        ^ RotateRight(A, 13U)
                        ^ RotateRight(A, 22U);
                    const uint32 Majority = (A & B) ^ (A & C) ^ (B & C);
                    const uint32 Temp2 = Sum0 + Majority;

                    H = G;
                    G = F;
                    F = E;
                    E = D + Temp1;
                    D = C;
                    C = B;
                    B = A;
                    A = Temp1 + Temp2;
                }

                Hash[0] += A;
                Hash[1] += B;
                Hash[2] += C;
                Hash[3] += D;
                Hash[4] += E;
                Hash[5] += F;
                Hash[6] += G;
                Hash[7] += H;
            }

            OutDigest.Reset(64);
            for (const uint32 Word : Hash)
            {
                OutDigest += FString::Printf(TEXT("%08x"), Word);
            }
            if (!IsLowerHexDigest(OutDigest))
            {
                return Fail(
                    OutError,
                    ERuntimeLoadErrorCode::HashFailed,
                    Path,
                    TEXT("SHA-256 did not produce a 64-character lowercase hexadecimal digest."));
            }
            return true;
        }

        bool LoadJsonArtifact(
            const FString& Path,
            const FString& ExpectedSchema,
            const bool bComputeDigest,
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
                || ActualSchema != ExpectedSchema)
            {
                return Fail(
                    OutError,
                    ERuntimeLoadErrorCode::SchemaMismatch,
                    Path,
                    FString::Printf(
                        TEXT("Artifact schema mismatch in %s: expected '%s', got '%s'."),
                        *Path,
                        *ExpectedSchema,
                        ActualSchema.IsEmpty() ? TEXT("<missing>") : *ActualSchema));
            }

            FString SessionDigest;
            if (!RootObject->TryGetStringField(TEXT("session_digest"), SessionDigest)
                || !IsLowerHexDigest(SessionDigest))
            {
                return Fail(
                    OutError,
                    ERuntimeLoadErrorCode::InvalidDigest,
                    Path,
                    FString::Printf(
                        TEXT("Artifact session_digest in %s must be 64 lowercase hexadecimal characters."),
                        *Path));
            }

            FParsedArtifact Candidate;
            Candidate.Path = Path;
            Candidate.SessionDigest = MoveTemp(SessionDigest);
            if (bComputeDigest && !ComputeSha256(Path, Candidate.Sha256, OutError))
            {
                return false;
            }
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
        FParsedArtifact SessionLock;
        Candidate.SessionLockPath = FPaths::Combine(NormalizedDirectory, TEXT("session.lock.json"));
        if (!LoadJsonArtifact(
                Candidate.SessionLockPath,
                TEXT("lingtu.sim.session-lock.v1"),
                false,
                SessionLock,
                OutError))
        {
            return false;
        }
        Candidate.SessionDigest = SessionLock.SessionDigest;

        static const FArtifactContract Contracts[] = {
            {
                TEXT("physics.plan.json"),
                TEXT("lingtu.sim.physics-plan.v1"),
                true,
                &FSessionBundleView::PhysicsPlanPath,
                &FSessionBundleView::PhysicsPlanDigest,
            },
            {
                TEXT("visual.plan.json"),
                TEXT("lingtu.sim.visual-plan.v1"),
                true,
                &FSessionBundleView::VisualPlanPath,
                &FSessionBundleView::VisualPlanDigest,
            },
            {
                TEXT("sensor.plan.json"),
                TEXT("lingtu.sim.sensor-plan.v1"),
                true,
                &FSessionBundleView::SensorPlanPath,
                &FSessionBundleView::SensorPlanDigest,
            },
            {
                TEXT("control.plan.json"),
                TEXT("lingtu.sim.control-plan.v1"),
                true,
                &FSessionBundleView::ControlPlanPath,
                &FSessionBundleView::ControlPlanDigest,
            },
            {
                TEXT("scenario.plan.json"),
                TEXT("lingtu.sim.scenario-plan.v1"),
                false,
                &FSessionBundleView::ScenarioPlanPath,
                &FSessionBundleView::ScenarioPlanDigest,
            },
            {
                TEXT("transport.intent.json"),
                TEXT("lingtu.sim.transport-intent.v1"),
                true,
                &FSessionBundleView::TransportIntentPath,
                &FSessionBundleView::TransportIntentDigest,
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
            if (!LoadJsonArtifact(Path, Contract.Schema, true, Artifact, OutError))
            {
                return false;
            }
            if (Artifact.SessionDigest != Candidate.SessionDigest)
            {
                return Fail(
                    OutError,
                    ERuntimeLoadErrorCode::DigestMismatch,
                    Path,
                    FString::Printf(
                        TEXT("Artifact session_digest in %s does not match session.lock.json."),
                        *Path));
            }

            Candidate.*Contract.PathMember = MoveTemp(Artifact.Path);
            Candidate.*Contract.DigestMember = MoveTemp(Artifact.Sha256);
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
                    false,
                    Source,
                    LinearVelocity,
                    OutError)
                || !ReadFiniteArray(
                    EntityObject,
                    TEXT("angular_velocity_rps"),
                    3,
                    false,
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
            const FString& ExpectedSessionDigest,
            FSnapshotEnvelope& OutSnapshot,
            FRuntimeLoadError& OutError)
        {
            if (!IsLowerHexDigest(ExpectedSessionDigest))
            {
                return Fail(
                    OutError,
                    ERuntimeLoadErrorCode::InvalidArgument,
                    Source,
                    TEXT("Expected session_digest must be 64 lowercase hexadecimal characters."));
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
            if (!RootObject->TryGetStringField(TEXT("session_digest"), Candidate.SessionDigest)
                || !IsLowerHexDigest(Candidate.SessionDigest))
            {
                return Fail(
                    OutError,
                    ERuntimeLoadErrorCode::InvalidDigest,
                    Source,
                    TEXT("Snapshot session_digest must be 64 lowercase hexadecimal characters."));
            }
            if (Candidate.SessionDigest != ExpectedSessionDigest)
            {
                return Fail(
                    OutError,
                    ERuntimeLoadErrorCode::DigestMismatch,
                    Source,
                    TEXT("Snapshot session_digest does not match the loaded SessionBundle."));
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
            const TArray<TSharedPtr<FJsonValue>>* BodyValues = nullptr;
            const bool bHasEntities = RootObject->TryGetArrayField(TEXT("entities"), EntityValues);
            const bool bHasBodies = RootObject->TryGetArrayField(TEXT("bodies"), BodyValues);
            if (bHasEntities == bHasBodies)
            {
                return Fail(
                    OutError,
                    ERuntimeLoadErrorCode::InvalidField,
                    Source,
                    TEXT("Snapshot must contain exactly one entity array: 'entities' or 'bodies'."));
            }
            EntityValues = bHasEntities ? EntityValues : BodyValues;
            if (EntityValues == nullptr)
            {
                return Fail(
                    OutError,
                    ERuntimeLoadErrorCode::InvalidField,
                    Source,
                    TEXT("Snapshot entity array is unavailable."));
            }

            Candidate.Entities.Reserve(EntityValues->Num());
            TSet<FString> StableIds;
            for (int32 EntityIndex = 0; EntityIndex < EntityValues->Num(); ++EntityIndex)
            {
                FEntityState Entity;
                const FString EntitySource = FString::Printf(
                    TEXT("%s.entities[%d]"),
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
        const FString& ExpectedSessionDigest,
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
            ExpectedSessionDigest,
            OutSnapshot,
            OutError);
    }

    bool FSessionBundleLoader::ParseSnapshotJson(
        const FString& SnapshotJson,
        const FString& ExpectedSessionDigest,
        FSnapshotEnvelope& OutSnapshot,
        FRuntimeLoadError& OutError)
    {
        OutError.Reset();
        return ParseSnapshotJsonInternal(
            SnapshotJson,
            TEXT("<snapshot-json>"),
            ExpectedSessionDigest,
            OutSnapshot,
            OutError);
    }
}

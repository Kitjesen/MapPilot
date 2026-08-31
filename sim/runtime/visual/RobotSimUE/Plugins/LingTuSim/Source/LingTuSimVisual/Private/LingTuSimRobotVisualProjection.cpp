#include "LingTuSimRobotVisualProjection.h"

#include "Components/CapsuleComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Dom/JsonObject.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "HAL/FileManager.h"
#include "LingTuSimBodyActor.h"
#include "LingTuSimBundleLoader.h"
#include "LingTuSimPresentationPolicy.h"
#include "LingTuSimWorldEntityActor.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/ScopeExit.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

#include <initializer_list>

namespace LingTuSim::Visual
{
    namespace
    {
        struct FPackageRef
        {
            FString Id;
            FString Version;
            FString Kind;
            FString Manifest;
            FString Provenance;
        };

        struct FLocalTransform
        {
            FVector PositionMeters = FVector::ZeroVector;
            FQuat Rotation = FQuat::Identity;
            FVector Scale = FVector::OneVector;
        };

        struct FPayloadFrame
        {
            FString Name;
            FString Role;
            FString ParentFrame;
        };

        struct FVisualPlanInstance
        {
            FString InstanceId;
            FPackageRef Package;
            FString Binding;
            FString ProjectionSchema;
            FString ProjectionPath;
            bool bPayload = false;
            FString RobotInstanceId;
            FString ParentFrame;
            FLocalTransform MountTransform;
            TArray<FPayloadFrame> Frames;
        };

        struct FWorldProjectionPlan
        {
            bool bPresent = false;
            FPackageRef Package;
            FString Binding;
            FString Level;
            FString ProjectionSchema;
            FString ProjectionPath;
        };

        struct FProjectionComponent
        {
            FString LocalBodyId;
            FString VisualId;
            FString VisualFrameId;
            FString GeometryKind;
            FString Primitive;
            FString AssetPath;
            FVector MeshScale = FVector::OneVector;
            FVector DimensionsMeters = FVector::OneVector;
            double CapsuleRadiusMeters = 0.0;
            double CapsuleHalfHeightMeters = 0.0;
            double CapsuleCylinderHalfLengthMeters = 0.0;
            bool bHasMaterial = false;
            FLinearColor BaseColor = FLinearColor::White;
            double Metallic = 0.0;
            double Roughness = 0.5;
            FLocalTransform LocalTransform;
        };

        struct FProjectionDocument
        {
            FString Binding;
            FPackageRef Package;
            TArray<FProjectionComponent> Components;
        };

        struct FWorldEntityProjection
        {
            FString EntityId;
            FString SemanticClass;
            FString Authority;
            FLocalTransform WorldTransform;
            FProjectionComponent Visual;
        };

        struct FWorldProjectionDocument
        {
            FString Binding;
            FString Level;
            FPackageRef Package;
            TArray<FWorldEntityProjection> Entities;
        };

        bool Fail(
            FVisualMaterializationError& OutError,
            const FString& Source,
            const FString& Message)
        {
            OutError.Source = Source;
            OutError.Message = Message;
            return false;
        }

        bool ReadJsonObject(
            const FString& Path,
            TSharedPtr<FJsonObject>& OutObject,
            FVisualMaterializationError& OutError)
        {
            FString Json;
            if (!FFileHelper::LoadFileToString(Json, *Path))
            {
                return Fail(OutError, Path, TEXT("unable to read JSON file"));
            }
            const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Json);
            if (!FJsonSerializer::Deserialize(Reader, OutObject) || !OutObject.IsValid())
            {
                const FString Detail = Reader->GetErrorMessage();
                return Fail(
                    OutError,
                    Path,
                    Detail.IsEmpty() ? TEXT("invalid JSON object") : Detail);
            }
            return true;
        }

        bool RequireFieldCount(
            const TSharedPtr<FJsonObject>& Object,
            const int32 ExpectedCount,
            const FString& Source,
            FVisualMaterializationError& OutError)
        {
            return (Object.IsValid() && Object->Values.Num() == ExpectedCount)
                || Fail(
                    OutError,
                    Source,
                    FString::Printf(
                        TEXT("expected exactly %d fields, got %d"),
                        ExpectedCount,
                        Object.IsValid() ? Object->Values.Num() : -1));
        }

        bool RequireExactFields(
            const TSharedPtr<FJsonObject>& Object,
            const FString& Source,
            std::initializer_list<const TCHAR*> ExpectedFields,
            FVisualMaterializationError& OutError)
        {
            if (!Object.IsValid() || Object->Values.Num() != static_cast<int32>(ExpectedFields.size()))
            {
                return Fail(OutError, Source, TEXT("JSON object does not contain the exact expected field set"));
            }
            for (const TCHAR* Field : ExpectedFields)
            {
                if (!Object->HasField(Field))
                {
                    return Fail(
                        OutError,
                        Source,
                        FString::Printf(TEXT("missing required field %s"), Field));
                }
            }
            return true;
        }

        bool RequireString(
            const TSharedPtr<FJsonObject>& Object,
            const TCHAR* Field,
            const FString& Source,
            FString& OutValue,
            FVisualMaterializationError& OutError)
        {
            FString Value;
            if (!Object.IsValid() || !Object->TryGetStringField(Field, Value) || Value.IsEmpty())
            {
                return Fail(OutError, Source, FString::Printf(TEXT("%s must be a non-empty string"), Field));
            }
            OutValue = MoveTemp(Value);
            return true;
        }

        bool RequireNumberArray(
            const TSharedPtr<FJsonObject>& Object,
            const TCHAR* Field,
            const int32 ExpectedCount,
            const bool bPositive,
            const FString& Source,
            TArray<double>& OutValues,
            FVisualMaterializationError& OutError)
        {
            const TArray<TSharedPtr<FJsonValue>>* Values = nullptr;
            if (!Object.IsValid() || !Object->TryGetArrayField(Field, Values) || Values == nullptr || Values->Num() != ExpectedCount)
            {
                return Fail(
                    OutError,
                    Source,
                    FString::Printf(TEXT("%s must contain exactly %d numeric values"), Field, ExpectedCount));
            }
            TArray<double> Candidate;
            Candidate.Reserve(ExpectedCount);
            for (int32 Index = 0; Index < Values->Num(); ++Index)
            {
                double Value = 0.0;
                if (!(*Values)[Index].IsValid()
                    || !(*Values)[Index]->TryGetNumber(Value)
                    || !FMath::IsFinite(Value)
                    || (bPositive && Value <= 0.0))
                {
                    return Fail(
                        OutError,
                        Source,
                        FString::Printf(TEXT("%s[%d] is not valid finite numeric data"), Field, Index));
                }
                Candidate.Add(Value);
            }
            OutValues = MoveTemp(Candidate);
            return true;
        }

        bool RequireFinitePositiveNumber(
            const TSharedPtr<FJsonObject>& Object,
            const TCHAR* Field,
            const FString& Source,
            double& OutValue,
            FVisualMaterializationError& OutError)
        {
            double Value = 0.0;
            if (!Object.IsValid()
                || !Object->TryGetNumberField(Field, Value)
                || !FMath::IsFinite(Value)
                || Value <= 0.0)
            {
                return Fail(OutError, Source, FString::Printf(TEXT("%s must be positive finite numeric data"), Field));
            }
            OutValue = Value;
            return true;
        }

        bool ReadRgbaColor(
            const TSharedPtr<FJsonObject>& Object,
            const TCHAR* Field,
            const FString& Source,
            FLinearColor& OutColor,
            FVisualMaterializationError& OutError)
        {
            const TArray<TSharedPtr<FJsonValue>>* Values = nullptr;
            if (!Object.IsValid()
                || !Object->TryGetArrayField(Field, Values)
                || Values == nullptr
                || Values->Num() != 4)
            {
                return Fail(OutError, Source, FString::Printf(TEXT("%s must contain exactly 4 numeric values"), Field));
            }

            double Channels[4] = {};
            for (int32 Index = 0; Index < Values->Num(); ++Index)
            {
                if (!(*Values)[Index].IsValid()
                    || !(*Values)[Index]->TryGetNumber(Channels[Index])
                    || !FMath::IsFinite(Channels[Index]))
                {
                    return Fail(OutError, Source, FString::Printf(TEXT("%s contains invalid numeric data"), Field));
                }
            }
            OutColor = FLinearColor(
                static_cast<float>(Channels[0]),
                static_cast<float>(Channels[1]),
                static_cast<float>(Channels[2]),
                static_cast<float>(Channels[3]));
            return true;
        }

        bool ValidateUnitScalar(
            const double Value,
            const TCHAR* Field,
            const FString& Source,
            FVisualMaterializationError& OutError)
        {
            return (FMath::IsFinite(Value) && Value >= 0.0 && Value <= 1.0)
                || Fail(OutError, Source, FString::Printf(TEXT("%s must be finite numeric data in [0,1]"), Field));
        }

        bool ParseCanonicalComponentMaterial(
            const TSharedPtr<FJsonObject>& MaterialObject,
            const FString& MaterialSource,
            FProjectionComponent& Component,
            FVisualMaterializationError& OutError)
        {
            if (!RequireFieldCount(MaterialObject, 3, MaterialSource, OutError))
            {
                return false;
            }

            FString Source;
            if (!RequireString(MaterialObject, TEXT("source"), MaterialSource, Source, OutError)
                || (Source != TEXT("mjcf_material_rgba")
                    && Source != TEXT("mjcf_geom_rgba")
                    && Source != TEXT("compiler_default")
                    && Source != TEXT("world_package")))
            {
                return Fail(OutError, MaterialSource, TEXT("material.source is not a supported canonical source"));
            }

            const TSharedPtr<FJsonValue> KeyValue = MaterialObject->TryGetField(TEXT("key"));
            if (!KeyValue.IsValid()
                || (KeyValue->Type != EJson::String && KeyValue->Type != EJson::Null))
            {
                return Fail(OutError, MaterialSource, TEXT("material.key must be string or null"));
            }

            const TSharedPtr<FJsonObject>* PbrObjectPointer = nullptr;
            if (!MaterialObject->TryGetObjectField(TEXT("pbr"), PbrObjectPointer)
                || PbrObjectPointer == nullptr
                || !PbrObjectPointer->IsValid())
            {
                return Fail(OutError, MaterialSource, TEXT("material.pbr must be a JSON object"));
            }
            const TSharedPtr<FJsonObject>& PbrObject = *PbrObjectPointer;
            double Metallic = 0.0;
            double Roughness = 0.0;
            FLinearColor BaseColor = FLinearColor::White;
            if (!RequireFieldCount(PbrObject, 3, MaterialSource + TEXT(".pbr"), OutError)
                || !ReadRgbaColor(PbrObject, TEXT("base_color_rgba"), MaterialSource + TEXT(".pbr"), BaseColor, OutError)
                || !PbrObject->TryGetNumberField(TEXT("metallic"), Metallic)
                || !ValidateUnitScalar(Metallic, TEXT("metallic"), MaterialSource + TEXT(".pbr"), OutError)
                || !PbrObject->TryGetNumberField(TEXT("roughness"), Roughness)
                || !ValidateUnitScalar(Roughness, TEXT("roughness"), MaterialSource + TEXT(".pbr"), OutError))
            {
                return false;
            }

            Component.bHasMaterial = true;
            Component.BaseColor = BaseColor;
            Component.Metallic = Metallic;
            Component.Roughness = Roughness;
            return true;
        }

        bool ParseComponentMaterial(
            const TSharedPtr<FJsonObject>& ComponentObject,
            const FString& ComponentSource,
            FProjectionComponent& Component,
            FVisualMaterializationError& OutError)
        {
            const TSharedPtr<FJsonObject>* MaterialObjectPointer = nullptr;
            if (!ComponentObject->TryGetObjectField(TEXT("material"), MaterialObjectPointer)
                || MaterialObjectPointer == nullptr
                || !MaterialObjectPointer->IsValid())
            {
                return Fail(OutError, ComponentSource, TEXT("material is required and must be an object"));
            }

            return ParseCanonicalComponentMaterial(
                *MaterialObjectPointer,
                ComponentSource + TEXT(".material"),
                Component,
                OutError);
        }

        bool RequireObject(
            const TSharedPtr<FJsonObject>& Object,
            const TCHAR* Field,
            const FString& Source,
            TSharedPtr<FJsonObject>& OutObject,
            FVisualMaterializationError& OutError)
        {
            const TSharedPtr<FJsonObject>* Pointer = nullptr;
            if (!Object.IsValid()
                || !Object->TryGetObjectField(Field, Pointer)
                || Pointer == nullptr
                || !Pointer->IsValid())
            {
                return Fail(OutError, Source, FString::Printf(TEXT("%s must be a JSON object"), Field));
            }
            OutObject = *Pointer;
            return true;
        }

        bool RequirePackageRef(
            const TSharedPtr<FJsonObject>& Object,
            const TCHAR* Field,
            const bool bRequireKind,
            const FString& Source,
            FPackageRef& OutPackage,
            FVisualMaterializationError& OutError)
        {
            TSharedPtr<FJsonObject> PackageObject;
            if (!RequireObject(Object, Field, Source, PackageObject, OutError)
                || !RequireFieldCount(PackageObject, bRequireKind ? 4 : 3, Source + TEXT(".package"), OutError)
                || !RequireString(PackageObject, TEXT("id"), Source + TEXT(".package"), OutPackage.Id, OutError)
                || !RequireString(PackageObject, TEXT("version"), Source + TEXT(".package"), OutPackage.Version, OutError)
                || !RequireString(PackageObject, TEXT("manifest"), Source + TEXT(".package"), OutPackage.Manifest, OutError))
            {
                return false;
            }
            if (bRequireKind
                && (!RequireString(PackageObject, TEXT("kind"), Source + TEXT(".package"), OutPackage.Kind, OutError)
                    || (OutPackage.Kind != TEXT("robot")
                        && OutPackage.Kind != TEXT("world")
                        && OutPackage.Kind != TEXT("scenario")
                        && OutPackage.Kind != TEXT("payload"))))
            {
                return Fail(
                    OutError,
                    Source + TEXT(".package"),
                    TEXT("package.kind must be robot, world, scenario, or payload"));
            }
            return true;
        }

        bool RequireWorldProjectionPackageRef(
            const TSharedPtr<FJsonObject>& Object,
            const TCHAR* Field,
            const FString& Source,
            FPackageRef& OutPackage,
            FVisualMaterializationError& OutError)
        {
            TSharedPtr<FJsonObject> PackageObject;
            return RequireObject(Object, Field, Source, PackageObject, OutError)
                && RequireExactFields(
                    PackageObject,
                    Source + TEXT(".package"),
                    {TEXT("id"), TEXT("version"), TEXT("manifest"), TEXT("provenance")},
                    OutError)
                && RequireString(PackageObject, TEXT("id"), Source + TEXT(".package"), OutPackage.Id, OutError)
                && RequireString(PackageObject, TEXT("version"), Source + TEXT(".package"), OutPackage.Version, OutError)
                && RequireString(PackageObject, TEXT("manifest"), Source + TEXT(".package"), OutPackage.Manifest, OutError)
                && RequireString(PackageObject, TEXT("provenance"), Source + TEXT(".package"), OutPackage.Provenance, OutError);
        }

        bool IsSafeRepositoryRelativePath(const FString& RelativePath)
        {
            if (RelativePath.IsEmpty()
                || FPaths::IsRelative(RelativePath) == false
                || RelativePath.Contains(TEXT("\\"))
                || RelativePath.Contains(TEXT(":"))
                || RelativePath.Contains(TEXT("//"))
                || RelativePath.EndsWith(TEXT("/")))
            {
                return false;
            }
            for (const TCHAR Character : RelativePath)
            {
                if (FChar::IsWhitespace(Character))
                {
                    return false;
                }
            }

            TArray<FString> Parts;
            RelativePath.ParseIntoArray(Parts, TEXT("/"), false);
            for (const FString& Part : Parts)
            {
                if (Part.IsEmpty() || Part == TEXT(".") || Part == TEXT(".."))
                {
                    return false;
                }
            }
            return true;
        }

        bool IsStableLocalId(const FString& Value)
        {
            if (Value.IsEmpty() || !FChar::IsAlnum(Value[0]))
            {
                return false;
            }
            for (const TCHAR Character : Value)
            {
                if (!FChar::IsAlnum(Character)
                    && Character != TEXT('_')
                    && Character != TEXT('.')
                    && Character != TEXT('-'))
                {
                    return false;
                }
            }
            return true;
        }

        bool ResolveContainedPath(
            const FString& ArtifactRoot,
            const FString& RelativePath,
            FString& OutPath,
            FVisualMaterializationError& OutError)
        {
            if (!IsSafeRepositoryRelativePath(RelativePath))
            {
                return Fail(OutError, RelativePath, TEXT("projection path is not safe repository-relative data"));
            }
            FString Root = FPaths::ConvertRelativePathToFull(ArtifactRoot);
            FPaths::NormalizeDirectoryName(Root);
            FString Candidate = FPaths::ConvertRelativePathToFull(FPaths::Combine(Root, RelativePath));
            FPaths::NormalizeFilename(Candidate);
            const FString RootPrefix = Root.EndsWith(TEXT("/")) ? Root : Root + TEXT("/");
            if (!Candidate.StartsWith(RootPrefix) && Candidate != Root)
            {
                return Fail(OutError, Candidate, TEXT("projection path escapes the artifact root"));
            }
            if (!IFileManager::Get().FileExists(*Candidate))
            {
                return Fail(OutError, Candidate, TEXT("projection file does not exist"));
            }
            OutPath = MoveTemp(Candidate);
            return true;
        }

        bool ReadQuaternionWxyz(
            const TSharedPtr<FJsonObject>& Object,
            const TCHAR* Field,
            const FString& Source,
            FQuat& OutQuat,
            FVisualMaterializationError& OutError)
        {
            TArray<double> Values;
            if (!RequireNumberArray(Object, Field, 4, false, Source, Values, OutError))
            {
                return false;
            }
            const double SizeSquared =
                Values[0] * Values[0] + Values[1] * Values[1] + Values[2] * Values[2] + Values[3] * Values[3];
            if (!FMath::IsFinite(SizeSquared) || FMath::Abs(SizeSquared - 1.0) > 1.0e-6)
            {
                return Fail(OutError, Source, FString::Printf(TEXT("%s must be a finite normalized quaternion"), Field));
            }
            OutQuat = FQuat(Values[1], Values[2], Values[3], Values[0]);
            return true;
        }

        FTransform MakeLocalUnrealTransform(const FLocalTransform& LocalTransform, const FVector& Scale)
        {
            const FVector UnrealPosition(
                LocalTransform.PositionMeters.X * 100.0,
                -LocalTransform.PositionMeters.Y * 100.0,
                LocalTransform.PositionMeters.Z * 100.0);
            FQuat UnrealRotation(
                -LocalTransform.Rotation.X,
                LocalTransform.Rotation.Y,
                -LocalTransform.Rotation.Z,
                LocalTransform.Rotation.W);
            UnrealRotation.Normalize();
            return FTransform(UnrealRotation, UnrealPosition, Scale);
        }

        bool ParseLocalTransform(
            const TSharedPtr<FJsonObject>& Object,
            const FString& Source,
            FLocalTransform& OutTransform,
            FVisualMaterializationError& OutError)
        {
            TArray<double> Position;
            TArray<double> Scale;
            if (!RequireFieldCount(Object, 3, Source, OutError)
                || !RequireNumberArray(Object, TEXT("position_m"), 3, false, Source, Position, OutError)
                || !ReadQuaternionWxyz(Object, TEXT("quaternion_wxyz"), Source, OutTransform.Rotation, OutError)
                || !RequireNumberArray(Object, TEXT("scale"), 3, false, Source, Scale, OutError))
            {
                return false;
            }
            OutTransform.PositionMeters = FVector(Position[0], Position[1], Position[2]);
            OutTransform.Scale = FVector(Scale[0], Scale[1], Scale[2]);
            return true;
        }

        bool ParsePoseTransform(
            const TSharedPtr<FJsonObject>& Object,
            const FString& Source,
            FLocalTransform& OutTransform,
            FVisualMaterializationError& OutError)
        {
            TArray<double> Position;
            if (!RequireExactFields(
                    Object,
                    Source,
                    {TEXT("position_m"), TEXT("quaternion_wxyz")},
                    OutError)
                || !RequireNumberArray(
                    Object,
                    TEXT("position_m"),
                    3,
                    false,
                    Source,
                    Position,
                    OutError)
                || !ReadQuaternionWxyz(
                    Object,
                    TEXT("quaternion_wxyz"),
                    Source,
                    OutTransform.Rotation,
                    OutError))
            {
                return false;
            }
            OutTransform.PositionMeters = FVector(Position[0], Position[1], Position[2]);
            OutTransform.Scale = FVector::OneVector;
            return true;
        }

        bool ParsePayloadPlanInstance(
            const TSharedPtr<FJsonObject>& Object,
            const FString& Source,
            const FString& ExpectedRobotInstanceId,
            FVisualPlanInstance& OutInstance,
            FVisualMaterializationError& OutError)
        {
            if (!RequireExactFields(
                    Object,
                    Source,
                    {
                        TEXT("instance_id"),
                        TEXT("namespace"),
                        TEXT("robot_instance_id"),
                        TEXT("package"),
                        TEXT("parent_frame"),
                        TEXT("mount_transform"),
                        TEXT("binding"),
                        TEXT("projection"),
                        TEXT("authority"),
                        TEXT("ue_collision"),
                        TEXT("frames"),
                    },
                    OutError))
            {
                return false;
            }

            FVisualPlanInstance Candidate;
            Candidate.bPayload = true;
            FString Namespace;
            FString Authority;
            FString UeCollision;
            if (!RequireString(Object, TEXT("instance_id"), Source, Candidate.InstanceId, OutError)
                || !IsStableLocalId(Candidate.InstanceId)
                || !RequireString(Object, TEXT("namespace"), Source, Namespace, OutError)
                || Namespace != Candidate.InstanceId
                || !RequireString(
                    Object,
                    TEXT("robot_instance_id"),
                    Source,
                    Candidate.RobotInstanceId,
                    OutError)
                || Candidate.RobotInstanceId != ExpectedRobotInstanceId
                || !RequirePackageRef(Object, TEXT("package"), true, Source, Candidate.Package, OutError)
                || Candidate.Package.Kind != TEXT("payload")
                || !RequireString(Object, TEXT("parent_frame"), Source, Candidate.ParentFrame, OutError)
                || !IsStableLocalId(Candidate.ParentFrame)
                || !RequireString(Object, TEXT("binding"), Source, Candidate.Binding, OutError)
                || !RequireString(Object, TEXT("authority"), Source, Authority, OutError)
                || Authority != TEXT("mujoco")
                || !RequireString(Object, TEXT("ue_collision"), Source, UeCollision, OutError)
                || UeCollision != TEXT("disabled"))
            {
                return Fail(OutError, Source, TEXT("visual payload identity or authority is invalid"));
            }

            TSharedPtr<FJsonObject> MountTransform;
            TSharedPtr<FJsonObject> Projection;
            if (!RequireObject(Object, TEXT("mount_transform"), Source, MountTransform, OutError)
                || !ParsePoseTransform(
                    MountTransform,
                    Source + TEXT(".mount_transform"),
                    Candidate.MountTransform,
                    OutError)
                || !RequireObject(Object, TEXT("projection"), Source, Projection, OutError)
                || !RequireExactFields(
                    Projection,
                    Source + TEXT(".projection"),
                    {TEXT("schema"), TEXT("path")},
                    OutError)
                || !RequireString(
                    Projection,
                    TEXT("schema"),
                    Source + TEXT(".projection"),
                    Candidate.ProjectionSchema,
                    OutError)
                || Candidate.ProjectionSchema != TEXT("lingtu.sim.payload-visual-projection.v1")
                || !RequireString(
                    Projection,
                    TEXT("path"),
                    Source + TEXT(".projection"),
                    Candidate.ProjectionPath,
                    OutError)
                || !IsSafeRepositoryRelativePath(Candidate.ProjectionPath))
            {
                return Fail(OutError, Source, TEXT("visual payload projection is invalid"));
            }

            const TArray<TSharedPtr<FJsonValue>>* Frames = nullptr;
            if (!Object->TryGetArrayField(TEXT("frames"), Frames)
                || Frames == nullptr
                || Frames->Num() == 0)
            {
                return Fail(OutError, Source, TEXT("visual payload frames must be a non-empty array"));
            }
            TSet<FString> FrameNames;
            for (int32 FrameIndex = 0; FrameIndex < Frames->Num(); ++FrameIndex)
            {
                const FString FrameSource = FString::Printf(
                    TEXT("%s.frames[%d]"),
                    *Source,
                    FrameIndex);
                const TSharedPtr<FJsonObject>* FrameObjectPointer = nullptr;
                if (!(*Frames)[FrameIndex].IsValid()
                    || !(*Frames)[FrameIndex]->TryGetObject(FrameObjectPointer)
                    || FrameObjectPointer == nullptr
                    || !FrameObjectPointer->IsValid())
                {
                    return Fail(OutError, FrameSource, TEXT("visual payload frame must be an object"));
                }
                const TSharedPtr<FJsonObject>& FrameObject = *FrameObjectPointer;
                const bool bHasParent = FrameObject->HasField(TEXT("parent_frame"));
                if (!(bHasParent
                        ? RequireExactFields(
                            FrameObject,
                            FrameSource,
                            {TEXT("name"), TEXT("role"), TEXT("parent_frame")},
                            OutError)
                        : RequireExactFields(
                            FrameObject,
                            FrameSource,
                            {TEXT("name"), TEXT("role")},
                            OutError)))
                {
                    return false;
                }
                FPayloadFrame Frame;
                if (!RequireString(FrameObject, TEXT("name"), FrameSource, Frame.Name, OutError)
                    || !IsStableLocalId(Frame.Name)
                    || !RequireString(FrameObject, TEXT("role"), FrameSource, Frame.Role, OutError)
                    || (bHasParent
                        && (!RequireString(
                                FrameObject,
                                TEXT("parent_frame"),
                                FrameSource,
                                Frame.ParentFrame,
                                OutError)
                            || !IsStableLocalId(Frame.ParentFrame))))
                {
                    return Fail(OutError, FrameSource, TEXT("visual payload frame identity is invalid"));
                }
                if (FrameNames.Contains(Frame.Name))
                {
                    return Fail(OutError, FrameSource, TEXT("duplicate visual payload frame name"));
                }
                FrameNames.Add(Frame.Name);
                Candidate.Frames.Add(MoveTemp(Frame));
            }
            OutInstance = MoveTemp(Candidate);
            return true;
        }

        bool ParseVisualPlan(
            const FString& Path,
            const FString& ExpectedSessionId,
            TArray<FVisualPlanInstance>& OutInstances,
            FWorldProjectionPlan& OutWorldProjection,
            FVisualMaterializationError& OutError)
        {
            TSharedPtr<FJsonObject> Root;
            if (!ReadJsonObject(Path, Root, OutError))
            {
                return false;
            }
            const bool bHasScenarioEntities = Root->HasField(TEXT("scenario_entities"));
            if (bHasScenarioEntities)
            {
                if (!RequireExactFields(
                        Root,
                        Path,
                        {
                            TEXT("schema"),
                            TEXT("session_id"),
                            TEXT("backends"),
                            TEXT("coordinate_system"),
                            TEXT("binding_policy"),
                            TEXT("world"),
                            TEXT("robots"),
                            TEXT("scenario_entities"),
                        },
                        OutError))
                {
                    return false;
                }
            }
            else if (!RequireExactFields(
                         Root,
                         Path,
                         {
                             TEXT("schema"),
                             TEXT("session_id"),
                             TEXT("backends"),
                             TEXT("coordinate_system"),
                             TEXT("binding_policy"),
                             TEXT("world"),
                             TEXT("robots"),
                         },
                         OutError))
            {
                return false;
            }
            FString Schema;
            FString SessionId;
            if (!RequireString(Root, TEXT("schema"), Path, Schema, OutError))
            {
                return false;
            }
            const bool bPayloadAwareV2 = Schema == TEXT("lingtu.sim.visual-plan.v2");
            if (Schema != TEXT("lingtu.sim.visual-plan.v1") && !bPayloadAwareV2)
            {
                return Fail(OutError, Path, TEXT("visual plan schema mismatch"));
            }
            if (!RequireString(Root, TEXT("session_id"), Path, SessionId, OutError))
            {
                return false;
            }
            if (SessionId != ExpectedSessionId)
            {
                return Fail(OutError, Path, TEXT("visual plan session_id does not match SessionBundle"));
            }

            TSharedPtr<FJsonObject> Backends;
            if (!RequireObject(Root, TEXT("backends"), Path, Backends, OutError)
                || !RequireExactFields(Backends, Path + TEXT(".backends"), {TEXT("physics"), TEXT("visual")}, OutError))
            {
                return false;
            }
            FString PhysicsBackend;
            FString VisualBackend;
            if (!RequireString(Backends, TEXT("physics"), Path, PhysicsBackend, OutError)
                || !RequireString(Backends, TEXT("visual"), Path, VisualBackend, OutError)
                || PhysicsBackend != TEXT("mujoco")
                || VisualBackend != TEXT("unreal"))
            {
                return Fail(OutError, Path, TEXT("visual plan backends must be {physics:mujoco, visual:unreal}"));
            }

            TSharedPtr<FJsonObject> World;
            if (!RequireObject(Root, TEXT("world"), Path, World, OutError))
            {
                return false;
            }
            const bool bHasWorldProjection = World->HasField(TEXT("projection"));
            if (bHasWorldProjection)
            {
                if (!RequireExactFields(
                        World,
                        Path + TEXT(".world"),
                        {
                            TEXT("package"),
                            TEXT("binding"),
                            TEXT("level"),
                            TEXT("projection"),
                        },
                        OutError))
                {
                    return false;
                }
            }
            else if (!RequireExactFields(
                         World,
                         Path + TEXT(".world"),
                         {TEXT("package"), TEXT("binding"), TEXT("level")},
                         OutError))
            {
                return false;
            }
            FPackageRef WorldPackage;
            FString WorldBinding;
            FString WorldLevel;
            if (!RequirePackageRef(World, TEXT("package"), true, Path + TEXT(".world"), WorldPackage, OutError)
                || WorldPackage.Kind != TEXT("world")
                || !RequireString(World, TEXT("binding"), Path + TEXT(".world"), WorldBinding, OutError)
                || !RequireString(World, TEXT("level"), Path + TEXT(".world"), WorldLevel, OutError)
                || !WorldLevel.StartsWith(TEXT("/Game/"))
                || WorldLevel.Contains(TEXT("\\"))
                || WorldLevel.Contains(TEXT("//"))
                || WorldLevel.Contains(TEXT("/../"))
                || WorldLevel.Contains(TEXT("/./")))
            {
                return Fail(OutError, Path, TEXT("visual plan world binding is invalid"));
            }
            for (const TCHAR Character : WorldLevel)
            {
                if (FChar::IsWhitespace(Character))
                {
                    return Fail(OutError, Path, TEXT("visual plan world level contains whitespace"));
                }
            }

            FWorldProjectionPlan ParsedWorldProjection;
            ParsedWorldProjection.Package = WorldPackage;
            ParsedWorldProjection.Binding = WorldBinding;
            ParsedWorldProjection.Level = WorldLevel;
            if (bHasWorldProjection)
            {
                TSharedPtr<FJsonObject> ProjectionObject;
                if (!RequireObject(
                        World,
                        TEXT("projection"),
                        Path + TEXT(".world"),
                        ProjectionObject,
                        OutError)
                    || !RequireExactFields(
                        ProjectionObject,
                        Path + TEXT(".world.projection"),
                        {
                            TEXT("schema"),
                            TEXT("path"),
                        },
                        OutError)
                    || !RequireString(
                        ProjectionObject,
                        TEXT("schema"),
                        Path + TEXT(".world.projection"),
                        ParsedWorldProjection.ProjectionSchema,
                        OutError)
                    || ParsedWorldProjection.ProjectionSchema
                        != TEXT("lingtu.sim.world-visual-projection.v1")
                    || !RequireString(
                        ProjectionObject,
                        TEXT("path"),
                        Path + TEXT(".world.projection"),
                        ParsedWorldProjection.ProjectionPath,
                        OutError)
                    || !IsSafeRepositoryRelativePath(ParsedWorldProjection.ProjectionPath))
                {
                    return Fail(OutError, Path, TEXT("visual plan world projection is invalid"));
                }
                ParsedWorldProjection.bPresent = true;
            }

            TSharedPtr<FJsonObject> Coordinates;
            TSharedPtr<FJsonObject> Policy;
            if (!RequireObject(Root, TEXT("coordinate_system"), Path, Coordinates, OutError)
                || !RequireFieldCount(Coordinates, 5, Path + TEXT(".coordinate_system"), OutError)
                || !RequireObject(Root, TEXT("binding_policy"), Path, Policy, OutError)
                || !RequireFieldCount(Policy, 2, Path + TEXT(".binding_policy"), OutError))
            {
                return false;
            }
            FString Source;
            FString Target;
            FString QuaternionOrder;
            if (!RequireString(Coordinates, TEXT("source"), Path, Source, OutError)
                || !RequireString(Coordinates, TEXT("target"), Path, Target, OutError)
                || !RequireString(Coordinates, TEXT("quaternion_order"), Path, QuaternionOrder, OutError)
                || Source != TEXT("mujoco_rh_z_up_m")
                || Target != TEXT("unreal_lh_z_up_cm")
                || QuaternionOrder != TEXT("wxyz"))
            {
                return Fail(OutError, Path, TEXT("visual plan coordinate system is unsupported"));
            }
            double PositionScale = 0.0;
            if (!Coordinates->TryGetNumberField(TEXT("position_scale"), PositionScale)
                || PositionScale != 100.0)
            {
                return Fail(OutError, Path, TEXT("visual plan position_scale must be 100.0"));
            }
            const TArray<TSharedPtr<FJsonValue>>* AxisMapping = nullptr;
            if (!Coordinates->TryGetArrayField(TEXT("axis_mapping"), AxisMapping)
                || AxisMapping == nullptr
                || AxisMapping->Num() != 3
                || (*AxisMapping)[0]->AsString() != TEXT("x")
                || (*AxisMapping)[1]->AsString() != TEXT("-y")
                || (*AxisMapping)[2]->AsString() != TEXT("z"))
            {
                return Fail(OutError, Path, TEXT("visual plan axis_mapping is unsupported"));
            }
            FString MissingAssetPolicy;
            bool bDataAssetIsProjection = false;
            if (!RequireString(Policy, TEXT("missing_asset"), Path, MissingAssetPolicy, OutError)
                || MissingAssetPolicy != TEXT("fail")
                || !Policy->TryGetBoolField(TEXT("data_asset_is_projection"), bDataAssetIsProjection)
                || !bDataAssetIsProjection)
            {
                return Fail(OutError, Path, TEXT("visual plan binding_policy must fail closed"));
            }

            TArray<FVisualPlanInstance> ParsedInstances;
            TSet<FString> InstanceIds;
            const auto ParseInstances = [
                                            &Root,
                                            &Path,
                                            &OutError,
                                            &ParsedInstances,
                                            &InstanceIds,
                                            bPayloadAwareV2](
                                            const TCHAR* ArrayField,
                                            const TCHAR* IdentityField,
                                            const TCHAR* ExpectedPackageKind,
                                            const TCHAR* ExpectedProjectionSchema,
                                            const bool bRobotInstances) -> bool
            {
                const TArray<TSharedPtr<FJsonValue>>* Values = nullptr;
                if (!Root->TryGetArrayField(ArrayField, Values) || Values == nullptr || Values->Num() == 0)
                {
                    return Fail(
                        OutError,
                        Path,
                        FString::Printf(TEXT("visual plan %s must be a non-empty array"), ArrayField));
                }
                for (int32 Index = 0; Index < Values->Num(); ++Index)
                {
                    const FString InstanceSource = FString::Printf(
                        TEXT("%s.%s[%d]"),
                        *Path,
                        ArrayField,
                        Index);
                    const TSharedPtr<FJsonObject>* InstanceObjectPointer = nullptr;
                    if (!(*Values)[Index].IsValid()
                        || !(*Values)[Index]->TryGetObject(InstanceObjectPointer)
                        || InstanceObjectPointer == nullptr
                        || !InstanceObjectPointer->IsValid())
                    {
                        return Fail(OutError, InstanceSource, TEXT("visual instance must be a strict JSON object"));
                    }
                    const TSharedPtr<FJsonObject>& InstanceObject = *InstanceObjectPointer;
                    const bool bHasPayloads = bRobotInstances
                        && InstanceObject->HasField(TEXT("payloads"));
                    if (bHasPayloads && !bPayloadAwareV2)
                    {
                        return Fail(
                            OutError,
                            InstanceSource,
                            TEXT("visual plan v1 does not support robot payloads"));
                    }
                    if (!(bHasPayloads
                            ? RequireExactFields(
                                InstanceObject,
                                InstanceSource,
                                {
                                    IdentityField,
                                    TEXT("namespace"),
                                    TEXT("package"),
                                    TEXT("binding"),
                                    TEXT("projection"),
                                    TEXT("spawn"),
                                    TEXT("payloads"),
                                },
                                OutError)
                            : RequireExactFields(
                                InstanceObject,
                                InstanceSource,
                                {
                                    IdentityField,
                                    TEXT("namespace"),
                                    TEXT("package"),
                                    TEXT("binding"),
                                    TEXT("projection"),
                                    TEXT("spawn"),
                                },
                                OutError)))
                    {
                        return false;
                    }
                    FVisualPlanInstance Instance;
                    FString Namespace;
                    if (!RequireString(InstanceObject, IdentityField, InstanceSource, Instance.InstanceId, OutError)
                        || !RequireString(InstanceObject, TEXT("namespace"), InstanceSource, Namespace, OutError)
                        || !RequirePackageRef(InstanceObject, TEXT("package"), true, InstanceSource, Instance.Package, OutError)
                        || !RequireString(InstanceObject, TEXT("binding"), InstanceSource, Instance.Binding, OutError))
                    {
                        return false;
                    }
                    if (Namespace != Instance.InstanceId)
                    {
                        return Fail(OutError, InstanceSource, TEXT("namespace must equal the stable instance identity"));
                    }
                    if (Instance.Package.Kind != ExpectedPackageKind)
                    {
                        return Fail(OutError, InstanceSource, TEXT("visual instance package kind is invalid"));
                    }
                    if (InstanceIds.Contains(Instance.InstanceId))
                    {
                        return Fail(
                            OutError,
                            InstanceSource,
                            FString::Printf(TEXT("duplicate %s visual identity"), IdentityField));
                    }
                    InstanceIds.Add(Instance.InstanceId);

                    TSharedPtr<FJsonObject> ProjectionObject;
                    if (!RequireObject(InstanceObject, TEXT("projection"), InstanceSource, ProjectionObject, OutError)
                        || !RequireFieldCount(ProjectionObject, 2, InstanceSource + TEXT(".projection"), OutError)
                        || !RequireString(
                            ProjectionObject,
                            TEXT("schema"),
                            InstanceSource,
                            Instance.ProjectionSchema,
                            OutError))
                    {
                        return false;
                    }
                    if (Instance.ProjectionSchema != ExpectedProjectionSchema)
                    {
                        return Fail(OutError, InstanceSource, TEXT("projection schema mismatch"));
                    }
                    if (!RequireString(ProjectionObject, TEXT("path"), InstanceSource, Instance.ProjectionPath, OutError)
                        || !IsSafeRepositoryRelativePath(Instance.ProjectionPath))
                    {
                        return false;
                    }

                    TSharedPtr<FJsonObject> SpawnObject;
                    TArray<double> SpawnPosition;
                    FQuat SpawnRotation;
                    if (!RequireObject(InstanceObject, TEXT("spawn"), InstanceSource, SpawnObject, OutError)
                        || !RequireFieldCount(SpawnObject, 2, InstanceSource + TEXT(".spawn"), OutError)
                        || !RequireNumberArray(SpawnObject, TEXT("position_m"), 3, false, InstanceSource, SpawnPosition, OutError)
                        || !ReadQuaternionWxyz(SpawnObject, TEXT("quaternion_wxyz"), InstanceSource, SpawnRotation, OutError))
                    {
                        return false;
                    }
                    const FString ParentRobotInstanceId = Instance.InstanceId;
                    ParsedInstances.Add(MoveTemp(Instance));

                    if (bHasPayloads)
                    {
                        const TArray<TSharedPtr<FJsonValue>>* PayloadValues = nullptr;
                        if (!InstanceObject->TryGetArrayField(TEXT("payloads"), PayloadValues)
                            || PayloadValues == nullptr
                            || PayloadValues->Num() == 0)
                        {
                            return Fail(
                                OutError,
                                InstanceSource,
                                TEXT("visual plan robot payloads must be a non-empty array"));
                        }
                        for (int32 PayloadIndex = 0; PayloadIndex < PayloadValues->Num(); ++PayloadIndex)
                        {
                            const FString PayloadSource = FString::Printf(
                                TEXT("%s.payloads[%d]"),
                                *InstanceSource,
                                PayloadIndex);
                            const TSharedPtr<FJsonObject>* PayloadObjectPointer = nullptr;
                            if (!(*PayloadValues)[PayloadIndex].IsValid()
                                || !(*PayloadValues)[PayloadIndex]->TryGetObject(PayloadObjectPointer)
                                || PayloadObjectPointer == nullptr
                                || !PayloadObjectPointer->IsValid())
                            {
                                return Fail(
                                    OutError,
                                    PayloadSource,
                                    TEXT("visual payload must be a strict JSON object"));
                            }
                            FVisualPlanInstance Payload;
                            if (!ParsePayloadPlanInstance(
                                    *PayloadObjectPointer,
                                    PayloadSource,
                                    ParentRobotInstanceId,
                                    Payload,
                                    OutError))
                            {
                                return false;
                            }
                            if (InstanceIds.Contains(Payload.InstanceId))
                            {
                                return Fail(
                                    OutError,
                                    PayloadSource,
                                    TEXT("duplicate payload visual identity"));
                            }
                            InstanceIds.Add(Payload.InstanceId);
                            ParsedInstances.Add(MoveTemp(Payload));
                        }
                    }
                }
                return true;
            };

            if (!ParseInstances(
                    TEXT("robots"),
                    TEXT("instance_id"),
                    TEXT("robot"),
                    TEXT("lingtu.sim.robot-visual-projection.v1"),
                    true))
            {
                return false;
            }
            if (bHasScenarioEntities
                && !ParseInstances(
                    TEXT("scenario_entities"),
                    TEXT("entity_id"),
                    TEXT("scenario"),
                    TEXT("lingtu.sim.entity-visual-projection.v1"),
                    false))
            {
                return false;
            }

            OutInstances = MoveTemp(ParsedInstances);
            OutWorldProjection = MoveTemp(ParsedWorldProjection);
            return true;
        }

        bool ParsePayloadProjection(
            const FString& Path,
            FProjectionDocument& OutProjection,
            FVisualMaterializationError& OutError)
        {
            TSharedPtr<FJsonObject> Root;
            if (!ReadJsonObject(Path, Root, OutError)
                || !RequireExactFields(
                    Root,
                    Path,
                    {
                        TEXT("schema"),
                        TEXT("package"),
                        TEXT("binding"),
                        TEXT("authority"),
                        TEXT("visual_policy"),
                        TEXT("source_asset"),
                        TEXT("material_contract"),
                        TEXT("components"),
                    },
                    OutError))
            {
                return false;
            }
            FString Schema;
            FString Authority;
            if (!RequireString(Root, TEXT("schema"), Path, Schema, OutError)
                || Schema != TEXT("lingtu.sim.payload-visual-projection.v1")
                || !RequirePackageRef(Root, TEXT("package"), false, Path, OutProjection.Package, OutError)
                || !RequireString(Root, TEXT("binding"), Path, OutProjection.Binding, OutError)
                || !RequireString(Root, TEXT("authority"), Path, Authority, OutError)
                || Authority != TEXT("mujoco"))
            {
                return Fail(OutError, Path, TEXT("payload projection identity or authority is invalid"));
            }

            TSharedPtr<FJsonObject> Policy;
            FString CollisionEnabled;
            bool bSimulatePhysics = true;
            bool bGenerateOverlapEvents = true;
            if (!RequireObject(Root, TEXT("visual_policy"), Path, Policy, OutError)
                || !RequireExactFields(
                    Policy,
                    Path + TEXT(".visual_policy"),
                    {
                        TEXT("collision_enabled"),
                        TEXT("simulate_physics"),
                        TEXT("generate_overlap_events"),
                    },
                    OutError)
                || !RequireString(
                    Policy,
                    TEXT("collision_enabled"),
                    Path + TEXT(".visual_policy"),
                    CollisionEnabled,
                    OutError)
                || CollisionEnabled != TEXT("no_collision")
                || !Policy->TryGetBoolField(TEXT("simulate_physics"), bSimulatePhysics)
                || bSimulatePhysics
                || !Policy->TryGetBoolField(TEXT("generate_overlap_events"), bGenerateOverlapEvents)
                || bGenerateOverlapEvents)
            {
                return Fail(OutError, Path, TEXT("payload projection visual_policy must preserve MuJoCo authority"));
            }

            TSharedPtr<FJsonObject> SourceAsset;
            FString SourceAssetPath;
            if (!RequireObject(Root, TEXT("source_asset"), Path, SourceAsset, OutError)
                || !RequireExactFields(
                    SourceAsset,
                    Path + TEXT(".source_asset"),
                    {TEXT("path")},
                    OutError)
                || !RequireString(
                    SourceAsset,
                    TEXT("path"),
                    Path + TEXT(".source_asset"),
                    SourceAssetPath,
                    OutError)
                || !IsSafeRepositoryRelativePath(SourceAssetPath))
            {
                return Fail(OutError, Path, TEXT("payload projection source_asset is invalid"));
            }

            TSharedPtr<FJsonObject> MaterialContract;
            FString SourceFormat;
            int32 MaterialCount = 0;
            const TArray<TSharedPtr<FJsonValue>>* RequiredChannels = nullptr;
            static const TCHAR* ExpectedChannels[] = {
                TEXT("base_color"),
                TEXT("normal"),
                TEXT("metallic_roughness"),
            };
            if (!RequireObject(Root, TEXT("material_contract"), Path, MaterialContract, OutError)
                || !RequireExactFields(
                    MaterialContract,
                    Path + TEXT(".material_contract"),
                    {TEXT("source_format"), TEXT("material_count"), TEXT("required_channels")},
                    OutError)
                || !RequireString(
                    MaterialContract,
                    TEXT("source_format"),
                    Path + TEXT(".material_contract"),
                    SourceFormat,
                    OutError)
                || SourceFormat != TEXT("gltf2")
                || !MaterialContract->TryGetNumberField(TEXT("material_count"), MaterialCount)
                || MaterialCount <= 0
                || !MaterialContract->TryGetArrayField(TEXT("required_channels"), RequiredChannels)
                || RequiredChannels == nullptr
                || RequiredChannels->Num() != UE_ARRAY_COUNT(ExpectedChannels))
            {
                return Fail(OutError, Path, TEXT("payload projection material_contract is invalid"));
            }
            for (int32 ChannelIndex = 0; ChannelIndex < RequiredChannels->Num(); ++ChannelIndex)
            {
                FString Channel;
                if (!(*RequiredChannels)[ChannelIndex].IsValid()
                    || !(*RequiredChannels)[ChannelIndex]->TryGetString(Channel)
                    || Channel != ExpectedChannels[ChannelIndex])
                {
                    return Fail(OutError, Path, TEXT("payload projection required_channels are invalid"));
                }
            }

            const TArray<TSharedPtr<FJsonValue>>* Components = nullptr;
            if (!Root->TryGetArrayField(TEXT("components"), Components)
                || Components == nullptr
                || Components->Num() == 0)
            {
                return Fail(OutError, Path, TEXT("payload projection components must be a non-empty array"));
            }
            TSet<FString> VisualFrameIds;
            for (int32 ComponentIndex = 0; ComponentIndex < Components->Num(); ++ComponentIndex)
            {
                const FString ComponentSource = FString::Printf(
                    TEXT("%s.components[%d]"),
                    *Path,
                    ComponentIndex);
                const TSharedPtr<FJsonObject>* ComponentObjectPointer = nullptr;
                if (!(*Components)[ComponentIndex].IsValid()
                    || !(*Components)[ComponentIndex]->TryGetObject(ComponentObjectPointer)
                    || ComponentObjectPointer == nullptr
                    || !ComponentObjectPointer->IsValid()
                    || !RequireExactFields(
                        *ComponentObjectPointer,
                        ComponentSource,
                        {
                            TEXT("mesh"),
                            TEXT("body_frame"),
                            TEXT("unreal_asset"),
                            TEXT("local_transform"),
                        },
                        OutError))
                {
                    return Fail(OutError, ComponentSource, TEXT("payload projection component must be strict"));
                }
                const TSharedPtr<FJsonObject>& ComponentObject = *ComponentObjectPointer;
                FProjectionComponent Component;
                Component.GeometryKind = TEXT("mesh");
                if (!RequireString(ComponentObject, TEXT("mesh"), ComponentSource, Component.VisualId, OutError)
                    || !IsStableLocalId(Component.VisualId)
                    || !RequireString(
                        ComponentObject,
                        TEXT("body_frame"),
                        ComponentSource,
                        Component.LocalBodyId,
                        OutError)
                    || !IsStableLocalId(Component.LocalBodyId)
                    || !RequireString(
                        ComponentObject,
                        TEXT("unreal_asset"),
                        ComponentSource,
                        Component.AssetPath,
                        OutError)
                    || !Component.AssetPath.StartsWith(TEXT("/Game/"))
                    || Component.AssetPath.Contains(TEXT("\\"))
                    || Component.AssetPath.Contains(TEXT(" "))
                    || !Component.AssetPath.Contains(TEXT(".")))
                {
                    return Fail(OutError, ComponentSource, TEXT("payload projection mesh binding is invalid"));
                }
                Component.VisualFrameId = Component.LocalBodyId + TEXT("/visual/") + Component.VisualId;
                if (VisualFrameIds.Contains(Component.VisualFrameId))
                {
                    return Fail(OutError, ComponentSource, TEXT("duplicate payload visual frame"));
                }
                VisualFrameIds.Add(Component.VisualFrameId);

                TSharedPtr<FJsonObject> LocalTransform;
                if (!RequireObject(
                        ComponentObject,
                        TEXT("local_transform"),
                        ComponentSource,
                        LocalTransform,
                        OutError)
                    || !ParseLocalTransform(
                        LocalTransform,
                        ComponentSource + TEXT(".local_transform"),
                        Component.LocalTransform,
                        OutError)
                    || Component.LocalTransform.Scale.X <= 0.0
                    || Component.LocalTransform.Scale.Y <= 0.0
                    || Component.LocalTransform.Scale.Z <= 0.0)
                {
                    return Fail(OutError, ComponentSource, TEXT("payload projection local_transform is invalid"));
                }
                Component.MeshScale = Component.LocalTransform.Scale;
                OutProjection.Components.Add(MoveTemp(Component));
            }
            return true;
        }

        bool ParseProjection(
            const FString& Path,
            const FString& ExpectedSchema,
            FProjectionDocument& OutProjection,
            FVisualMaterializationError& OutError)
        {
            TSharedPtr<FJsonObject> Root;
            if (!ReadJsonObject(Path, Root, OutError)
                || !RequireFieldCount(Root, 5, Path, OutError))
            {
                return false;
            }
            FString Schema;
            if (!RequireString(Root, TEXT("schema"), Path, Schema, OutError))
            {
                return false;
            }
            if (Schema != ExpectedSchema)
            {
                return Fail(OutError, Path, TEXT("projection schema mismatch"));
            }
            if (!RequireString(Root, TEXT("binding"), Path, OutProjection.Binding, OutError)
                || !RequirePackageRef(Root, TEXT("package"), false, Path, OutProjection.Package, OutError)
                )
            {
                return false;
            }

            TSharedPtr<FJsonObject> Mjcf;
            FString MjcfPath;
            if (!RequireObject(Root, TEXT("mjcf"), Path, Mjcf, OutError)
                || !RequireFieldCount(Mjcf, 1, Path + TEXT(".mjcf"), OutError)
                || !RequireString(Mjcf, TEXT("path"), Path, MjcfPath, OutError))
            {
                return false;
            }

            const TArray<TSharedPtr<FJsonValue>>* Components = nullptr;
            if (!Root->TryGetArrayField(TEXT("components"), Components)
                || Components == nullptr
                || Components->Num() == 0)
            {
                return Fail(OutError, Path, TEXT("projection components must be a non-empty array"));
            }

            TArray<FProjectionComponent> ParsedComponents;
            TSet<FString> BodyIds;
            TSet<FString> VisualIds;
            for (int32 Index = 0; Index < Components->Num(); ++Index)
            {
                const FString ComponentSource = FString::Printf(TEXT("%s.components[%d]"), *Path, Index);
                const TSharedPtr<FJsonObject>* ComponentObjectPointer = nullptr;
                if (!(*Components)[Index].IsValid()
                    || !(*Components)[Index]->TryGetObject(ComponentObjectPointer)
                    || ComponentObjectPointer == nullptr
                    || !ComponentObjectPointer->IsValid())
                {
                    return Fail(OutError, ComponentSource, TEXT("projection component must be a strict JSON object"));
                }
                if ((*ComponentObjectPointer)->Values.Num() != 11)
                {
                    return Fail(OutError, ComponentSource, TEXT("projection component must be a strict JSON object"));
                }
                const TSharedPtr<FJsonObject>& ComponentObject = *ComponentObjectPointer;
                FProjectionComponent Component;
                FString BodyFrameId;
                FString AssetKey;
                if (!RequireString(ComponentObject, TEXT("local_body_id"), ComponentSource, Component.LocalBodyId, OutError)
                    || !RequireString(ComponentObject, TEXT("body_frame_id"), ComponentSource, BodyFrameId, OutError)
                    || !RequireString(ComponentObject, TEXT("visual_id"), ComponentSource, Component.VisualId, OutError)
                    || !RequireString(ComponentObject, TEXT("visual_frame_id"), ComponentSource, Component.VisualFrameId, OutError)
                    || !RequireString(ComponentObject, TEXT("asset_key"), ComponentSource, AssetKey, OutError))
                {
                    return false;
                }
                BodyIds.Add(Component.LocalBodyId);
                if (VisualIds.Contains(Component.VisualFrameId))
                {
                    return Fail(OutError, ComponentSource, TEXT("duplicate visual_frame_id"));
                }
                VisualIds.Add(Component.VisualFrameId);

                TSharedPtr<FJsonObject> GeometryObject;
                if (!RequireObject(ComponentObject, TEXT("geometry"), ComponentSource, GeometryObject, OutError)
                    || !RequireString(GeometryObject, TEXT("kind"), ComponentSource, Component.GeometryKind, OutError))
                {
                    return false;
                }
                if (Component.GeometryKind == TEXT("mesh"))
                {
                    TArray<double> MeshScale;
                    FString SourceMesh;
                    FString MeshName;
                    if (!RequireFieldCount(GeometryObject, 4, ComponentSource + TEXT(".geometry"), OutError)
                        || !RequireString(GeometryObject, TEXT("source_mesh"), ComponentSource, SourceMesh, OutError)
                        || !RequireString(GeometryObject, TEXT("mesh"), ComponentSource, MeshName, OutError)
                        || !RequireNumberArray(GeometryObject, TEXT("scale"), 3, false, ComponentSource, MeshScale, OutError))
                    {
                        return false;
                    }
                    Component.MeshScale = FVector(MeshScale[0], MeshScale[1], MeshScale[2]);
                }
                else if (Component.GeometryKind == TEXT("primitive"))
                {
                    TArray<double> Size;
                    if (!RequireFieldCount(GeometryObject, 3, ComponentSource + TEXT(".geometry"), OutError)
                        || !RequireString(GeometryObject, TEXT("primitive"), ComponentSource, Component.Primitive, OutError))
                    {
                        return false;
                    }
                    const int32 ExpectedSize =
                        Component.Primitive == TEXT("box") ? 3
                        : Component.Primitive == TEXT("sphere") ? 1
                        : (Component.Primitive == TEXT("cylinder") || Component.Primitive == TEXT("capsule")) ? 2
                        : 0;
                    if (ExpectedSize == 0
                        || !RequireNumberArray(GeometryObject, TEXT("size"), ExpectedSize, true, ComponentSource, Size, OutError))
                    {
                        return Fail(OutError, ComponentSource, TEXT("unsupported primitive geometry"));
                    }
                    if (Component.Primitive == TEXT("box"))
                    {
                        Component.DimensionsMeters = FVector(Size[0], Size[1], Size[2]);
                    }
                    else if (Component.Primitive == TEXT("sphere"))
                    {
                        Component.DimensionsMeters = FVector(Size[0] * 2.0);
                    }
                    else
                    {
                        Component.DimensionsMeters = FVector(Size[0] * 2.0, Size[0] * 2.0, Size[1] * 2.0);
                    }
                }
                else
                {
                    return Fail(OutError, ComponentSource, TEXT("unsupported geometry kind"));
                }

                TSharedPtr<FJsonObject> UnrealObject;
                if (!RequireObject(ComponentObject, TEXT("unreal"), ComponentSource, UnrealObject, OutError))
                {
                    return false;
                }
                FString Representation;
                FString ComponentClass;
                if (!RequireString(UnrealObject, TEXT("representation"), ComponentSource, Representation, OutError)
                    || !RequireString(UnrealObject, TEXT("component_class"), ComponentSource, ComponentClass, OutError))
                {
                    return false;
                }
                if (Component.Primitive == TEXT("capsule"))
                {
                    if (!RequireFieldCount(UnrealObject, 6, ComponentSource + TEXT(".unreal"), OutError)
                        || Representation != TEXT("component")
                        || ComponentClass != TEXT("/Script/Engine.CapsuleComponent")
                        || !RequireFinitePositiveNumber(UnrealObject, TEXT("radius_m"), ComponentSource, Component.CapsuleRadiusMeters, OutError)
                        || !RequireFinitePositiveNumber(UnrealObject, TEXT("capsule_half_height_m"), ComponentSource, Component.CapsuleHalfHeightMeters, OutError))
                    {
                        return Fail(OutError, ComponentSource, TEXT("invalid capsule Unreal binding"));
                    }
                    TArray<double> Dimensions;
                    if (!RequireFinitePositiveNumber(
                            UnrealObject,
                            TEXT("cylinder_half_length_m"),
                            ComponentSource,
                            Component.CapsuleCylinderHalfLengthMeters,
                            OutError)
                        || !RequireNumberArray(UnrealObject, TEXT("dimensions_m"), 3, true, ComponentSource, Dimensions, OutError))
                    {
                        return false;
                    }
                    Component.DimensionsMeters = FVector(Dimensions[0], Dimensions[1], Dimensions[2]);
                }
                else
                {
                    const int32 ExpectedUnrealFields = Component.GeometryKind == TEXT("mesh") ? 3 : 4;
                    if (!RequireFieldCount(UnrealObject, ExpectedUnrealFields, ComponentSource + TEXT(".unreal"), OutError)
                        || Representation != TEXT("static_mesh")
                        || ComponentClass != TEXT("/Script/Engine.StaticMeshComponent")
                        || !RequireString(UnrealObject, TEXT("asset_path"), ComponentSource, Component.AssetPath, OutError))
                    {
                        return Fail(OutError, ComponentSource, TEXT("invalid static mesh Unreal binding"));
                    }
                    if (Component.GeometryKind == TEXT("primitive"))
                    {
                        TArray<double> Dimensions;
                        if (!RequireNumberArray(UnrealObject, TEXT("dimensions_m"), 3, true, ComponentSource, Dimensions, OutError))
                        {
                            return false;
                        }
                        Component.DimensionsMeters = FVector(Dimensions[0], Dimensions[1], Dimensions[2]);
                    }
                }

                TSharedPtr<FJsonObject> LocalTransformObject;
                if (!RequireObject(ComponentObject, TEXT("local_transform"), ComponentSource, LocalTransformObject, OutError)
                    || !ParseLocalTransform(LocalTransformObject, ComponentSource + TEXT(".local_transform"), Component.LocalTransform, OutError))
                {
                    return false;
                }
                if (Component.GeometryKind == TEXT("mesh")
                    && !Component.MeshScale.Equals(Component.LocalTransform.Scale, 1.0e-6))
                {
                    return Fail(
                        OutError,
                        ComponentSource,
                        TEXT("mesh geometry.scale and local_transform.scale must match"));
                }

                if (!ComponentObject->HasField(TEXT("material_key")))
                {
                    return Fail(OutError, ComponentSource, TEXT("material_key field is required"));
                }
                const TSharedPtr<FJsonValue> MaterialValue = ComponentObject->TryGetField(TEXT("material_key"));
                if (!MaterialValue.IsValid()
                    || (MaterialValue->Type != EJson::String && MaterialValue->Type != EJson::Null))
                {
                    return Fail(OutError, ComponentSource, TEXT("material_key must be string or null metadata"));
                }
                if (!ParseComponentMaterial(ComponentObject, ComponentSource, Component, OutError))
                {
                    return false;
                }

                TSharedPtr<FJsonObject> SourceObject;
                if (!RequireObject(ComponentObject, TEXT("source"), ComponentSource, SourceObject, OutError)
                    || !RequireFieldCount(SourceObject, 4, ComponentSource + TEXT(".source"), OutError))
                {
                    return false;
                }
                ParsedComponents.Add(MoveTemp(Component));
            }

            if (BodyIds.Num() == 0)
            {
                return Fail(OutError, Path, TEXT("projection produced no unique body IDs"));
            }
            OutProjection.Components = MoveTemp(ParsedComponents);
            return true;
        }

        bool ParseWorldProjection(
            const FString& Path,
            const FString& ExpectedSchema,
            FWorldProjectionDocument& OutProjection,
            FVisualMaterializationError& OutError)
        {
            TSharedPtr<FJsonObject> Root;
            if (!ReadJsonObject(Path, Root, OutError)
                || !RequireExactFields(
                    Root,
                    Path,
                    {
                        TEXT("schema"),
                        TEXT("package"),
                        TEXT("binding"),
                        TEXT("level"),
                        TEXT("units"),
                        TEXT("terrain"),
                        TEXT("entities"),
                        TEXT("spawn_alignment"),
                    },
                    OutError))
            {
                return false;
            }

            FString Schema;
            if (!RequireString(Root, TEXT("schema"), Path, Schema, OutError)
                || Schema != ExpectedSchema
                || !RequireWorldProjectionPackageRef(
                    Root,
                    TEXT("package"),
                    Path,
                    OutProjection.Package,
                    OutError)
                || !IsSafeRepositoryRelativePath(OutProjection.Package.Manifest)
                || !IsSafeRepositoryRelativePath(OutProjection.Package.Provenance)
                || !RequireString(Root, TEXT("binding"), Path, OutProjection.Binding, OutError)
                || !RequireString(Root, TEXT("level"), Path, OutProjection.Level, OutError)
                || !OutProjection.Level.StartsWith(TEXT("/Game/")))
            {
                return Fail(OutError, Path, TEXT("world projection identity is invalid"));
            }

            TSharedPtr<FJsonObject> Units;
            FString Length;
            FString UpAxis;
            FString Handedness;
            if (!RequireObject(Root, TEXT("units"), Path, Units, OutError)
                || !RequireExactFields(
                    Units,
                    Path + TEXT(".units"),
                    {TEXT("length"), TEXT("up_axis"), TEXT("handedness")},
                    OutError)
                || !RequireString(Units, TEXT("length"), Path + TEXT(".units"), Length, OutError)
                || !RequireString(Units, TEXT("up_axis"), Path + TEXT(".units"), UpAxis, OutError)
                || !RequireString(Units, TEXT("handedness"), Path + TEXT(".units"), Handedness, OutError)
                || Length != TEXT("m")
                || UpAxis != TEXT("Z")
                || Handedness != TEXT("RH"))
            {
                return Fail(OutError, Path, TEXT("world projection coordinate units are unsupported"));
            }

            TSharedPtr<FJsonObject> Terrain;
            if (!RequireObject(Root, TEXT("terrain"), Path, Terrain, OutError)
                || !RequireExactFields(
                    Terrain,
                    Path + TEXT(".terrain"),
                    {
                        TEXT("grid_px"),
                        TEXT("extent_m"),
                        TEXT("sample_spacing_m"),
                        TEXT("physics_bounds_m"),
                        TEXT("visual_bounds_m"),
                        TEXT("assets"),
                    },
                    OutError))
            {
                return false;
            }
            TArray<double> Grid;
            TArray<double> Extent;
            TArray<double> Spacing;
            if (!RequireNumberArray(Terrain, TEXT("grid_px"), 2, true, Path + TEXT(".terrain"), Grid, OutError)
                || Grid[0] < 2.0
                || Grid[1] < 2.0
                || FMath::FloorToDouble(Grid[0]) != Grid[0]
                || FMath::FloorToDouble(Grid[1]) != Grid[1]
                || !RequireNumberArray(Terrain, TEXT("extent_m"), 2, true, Path + TEXT(".terrain"), Extent, OutError)
                || !RequireNumberArray(Terrain, TEXT("sample_spacing_m"), 2, true, Path + TEXT(".terrain"), Spacing, OutError))
            {
                return Fail(OutError, Path, TEXT("world projection terrain dimensions are invalid"));
            }
            for (const TCHAR* BoundsField : {TEXT("physics_bounds_m"), TEXT("visual_bounds_m")})
            {
                TSharedPtr<FJsonObject> Bounds;
                TArray<double> Min;
                TArray<double> Max;
                if (!RequireObject(Terrain, BoundsField, Path + TEXT(".terrain"), Bounds, OutError)
                    || !RequireExactFields(
                        Bounds,
                        Path + TEXT(".terrain.") + BoundsField,
                        {TEXT("min_m"), TEXT("max_m")},
                        OutError)
                    || !RequireNumberArray(Bounds, TEXT("min_m"), 3, false, Path + TEXT(".terrain"), Min, OutError)
                    || !RequireNumberArray(Bounds, TEXT("max_m"), 3, false, Path + TEXT(".terrain"), Max, OutError)
                    || Min[0] > Max[0]
                    || Min[1] > Max[1]
                    || Min[2] > Max[2])
                {
                    return Fail(OutError, Path, TEXT("world projection terrain bounds are invalid"));
                }
            }

            const TArray<TSharedPtr<FJsonValue>>* Assets = nullptr;
            if (!Terrain->TryGetArrayField(TEXT("assets"), Assets)
                || Assets == nullptr
                || Assets->Num() < 3)
            {
                return Fail(OutError, Path, TEXT("world projection terrain must bind at least three assets"));
            }
            for (int32 Index = 0; Index < Assets->Num(); ++Index)
            {
                const FString AssetSource = FString::Printf(TEXT("%s.terrain.assets[%d]"), *Path, Index);
                const TSharedPtr<FJsonObject>* AssetPointer = nullptr;
                if (!(*Assets)[Index].IsValid()
                    || !(*Assets)[Index]->TryGetObject(AssetPointer)
                    || AssetPointer == nullptr
                    || !AssetPointer->IsValid())
                {
                    return Fail(OutError, AssetSource, TEXT("terrain asset must be a JSON object"));
                }
                const TSharedPtr<FJsonObject>& Asset = *AssetPointer;
                const bool bHasCollision = Asset->HasField(TEXT("collision"));
                const bool bFieldsValid = bHasCollision
                    ? RequireExactFields(
                        Asset,
                        AssetSource,
                        {TEXT("role"), TEXT("path"), TEXT("bytes"), TEXT("collision")},
                        OutError)
                    : RequireExactFields(
                        Asset,
                        AssetSource,
                        {TEXT("role"), TEXT("path"), TEXT("bytes")},
                        OutError);
                if (!bFieldsValid)
                {
                    return false;
                }
                FString Role;
                FString AssetPath;
                double Bytes = 0.0;
                bool bCollision = false;
                if (!RequireString(Asset, TEXT("role"), AssetSource, Role, OutError)
                    || !RequireString(Asset, TEXT("path"), AssetSource, AssetPath, OutError)
                    || !IsSafeRepositoryRelativePath(AssetPath)
                    || !Asset->TryGetNumberField(TEXT("bytes"), Bytes)
                    || !FMath::IsFinite(Bytes)
                    || Bytes < 1.0
                    || FMath::FloorToDouble(Bytes) != Bytes
                    || (bHasCollision && !Asset->TryGetBoolField(TEXT("collision"), bCollision)))
                {
                    return Fail(OutError, AssetSource, TEXT("terrain asset metadata is invalid"));
                }
            }

            TSharedPtr<FJsonObject> SpawnAlignment;
            TArray<double> SpawnPosition;
            bool bAlignedToHeightmap = false;
            if (!RequireObject(Root, TEXT("spawn_alignment"), Path, SpawnAlignment, OutError)
                || !RequireExactFields(
                    SpawnAlignment,
                    Path + TEXT(".spawn_alignment"),
                    {TEXT("position_m"), TEXT("aligned_to_heightmap")},
                    OutError)
                || !RequireNumberArray(
                    SpawnAlignment,
                    TEXT("position_m"),
                    3,
                    false,
                    Path + TEXT(".spawn_alignment"),
                    SpawnPosition,
                    OutError)
                || !SpawnAlignment->TryGetBoolField(TEXT("aligned_to_heightmap"), bAlignedToHeightmap)
                || !bAlignedToHeightmap)
            {
                return Fail(OutError, Path, TEXT("world projection spawn alignment is invalid"));
            }

            const TArray<TSharedPtr<FJsonValue>>* Entities = nullptr;
            if (!Root->TryGetArrayField(TEXT("entities"), Entities) || Entities == nullptr)
            {
                return Fail(OutError, Path, TEXT("world projection entities must be an array"));
            }

            FString PreviousEntityId;
            TArray<FWorldEntityProjection> ParsedEntities;
            for (int32 Index = 0; Index < Entities->Num(); ++Index)
            {
                const FString EntitySource = FString::Printf(TEXT("%s.entities[%d]"), *Path, Index);
                const TSharedPtr<FJsonObject>* EntityPointer = nullptr;
                if (!(*Entities)[Index].IsValid()
                    || !(*Entities)[Index]->TryGetObject(EntityPointer)
                    || EntityPointer == nullptr
                    || !EntityPointer->IsValid()
                    || !RequireExactFields(
                        *EntityPointer,
                        EntitySource,
                        {
                            TEXT("entity_id"),
                            TEXT("semantic_class"),
                            TEXT("authority"),
                            TEXT("transform"),
                            TEXT("geometry"),
                            TEXT("unreal"),
                            TEXT("material"),
                        },
                        OutError))
                {
                    return Fail(OutError, EntitySource, TEXT("world entity projection must be a strict JSON object"));
                }
                const TSharedPtr<FJsonObject>& EntityObject = *EntityPointer;
                FWorldEntityProjection Entity;
                if (!RequireString(EntityObject, TEXT("entity_id"), EntitySource, Entity.EntityId, OutError)
                    || !IsStableLocalId(Entity.EntityId)
                    || (!PreviousEntityId.IsEmpty() && Entity.EntityId.Compare(PreviousEntityId) <= 0)
                    || !RequireString(EntityObject, TEXT("semantic_class"), EntitySource, Entity.SemanticClass, OutError)
                    || !RequireString(EntityObject, TEXT("authority"), EntitySource, Entity.Authority, OutError)
                    || (Entity.Authority != TEXT("mujoco")
                        && Entity.Authority != TEXT("scenario")
                        && Entity.Authority != TEXT("ue_animation")))
                {
                    return Fail(OutError, EntitySource, TEXT("world entity identity is invalid or not canonical"));
                }
                PreviousEntityId = Entity.EntityId;

                TSharedPtr<FJsonObject> Transform;
                TArray<double> Position;
                if (!RequireObject(EntityObject, TEXT("transform"), EntitySource, Transform, OutError)
                    || !RequireExactFields(
                        Transform,
                        EntitySource + TEXT(".transform"),
                        {TEXT("position_m"), TEXT("quaternion_wxyz")},
                        OutError)
                    || !RequireNumberArray(
                        Transform,
                        TEXT("position_m"),
                        3,
                        false,
                        EntitySource + TEXT(".transform"),
                        Position,
                        OutError)
                    || !ReadQuaternionWxyz(
                        Transform,
                        TEXT("quaternion_wxyz"),
                        EntitySource + TEXT(".transform"),
                        Entity.WorldTransform.Rotation,
                        OutError))
                {
                    return false;
                }
                Entity.WorldTransform.PositionMeters = FVector(Position[0], Position[1], Position[2]);

                TSharedPtr<FJsonObject> Geometry;
                if (!RequireObject(EntityObject, TEXT("geometry"), EntitySource, Geometry, OutError)
                    || !RequireString(Geometry, TEXT("shape"), EntitySource + TEXT(".geometry"), Entity.Visual.Primitive, OutError))
                {
                    return false;
                }
                Entity.Visual.GeometryKind = TEXT("primitive");
                if (Entity.Visual.Primitive == TEXT("box"))
                {
                    TArray<double> Size;
                    if (!RequireExactFields(
                            Geometry,
                            EntitySource + TEXT(".geometry"),
                            {TEXT("shape"), TEXT("size_m")},
                            OutError)
                        || !RequireNumberArray(
                            Geometry,
                            TEXT("size_m"),
                            3,
                            true,
                            EntitySource + TEXT(".geometry"),
                            Size,
                            OutError))
                    {
                        return false;
                    }
                    Entity.Visual.DimensionsMeters = FVector(Size[0], Size[1], Size[2]);
                }
                else if (Entity.Visual.Primitive == TEXT("cylinder"))
                {
                    double Radius = 0.0;
                    double HalfHeight = 0.0;
                    if (!RequireExactFields(
                            Geometry,
                            EntitySource + TEXT(".geometry"),
                            {TEXT("shape"), TEXT("radius_m"), TEXT("half_height_m")},
                            OutError)
                        || !RequireFinitePositiveNumber(
                            Geometry,
                            TEXT("radius_m"),
                            EntitySource + TEXT(".geometry"),
                            Radius,
                            OutError)
                        || !RequireFinitePositiveNumber(
                            Geometry,
                            TEXT("half_height_m"),
                            EntitySource + TEXT(".geometry"),
                            HalfHeight,
                            OutError))
                    {
                        return false;
                    }
                    Entity.Visual.DimensionsMeters = FVector(
                        Radius * 2.0,
                        Radius * 2.0,
                        HalfHeight * 2.0);
                }
                else
                {
                    return Fail(OutError, EntitySource, TEXT("unsupported world entity geometry"));
                }

                TSharedPtr<FJsonObject> Unreal;
                FString Representation;
                FString ComponentClass;
                TArray<double> Dimensions;
                if (!RequireObject(EntityObject, TEXT("unreal"), EntitySource, Unreal, OutError)
                    || !RequireExactFields(
                        Unreal,
                        EntitySource + TEXT(".unreal"),
                        {TEXT("representation"), TEXT("component_class"), TEXT("asset_path"), TEXT("dimensions_m")},
                        OutError)
                    || !RequireString(Unreal, TEXT("representation"), EntitySource, Representation, OutError)
                    || Representation != TEXT("static_mesh")
                    || !RequireString(Unreal, TEXT("component_class"), EntitySource, ComponentClass, OutError)
                    || ComponentClass != TEXT("/Script/Engine.StaticMeshComponent")
                    || !RequireString(Unreal, TEXT("asset_path"), EntitySource, Entity.Visual.AssetPath, OutError)
                    || !RequireNumberArray(
                        Unreal,
                        TEXT("dimensions_m"),
                        3,
                        true,
                        EntitySource + TEXT(".unreal"),
                        Dimensions,
                        OutError))
                {
                    return Fail(OutError, EntitySource, TEXT("world entity Unreal projection is invalid"));
                }
                const FString ExpectedAsset = Entity.Visual.Primitive == TEXT("box")
                    ? TEXT("/Engine/BasicShapes/Cube.Cube")
                    : TEXT("/Engine/BasicShapes/Cylinder.Cylinder");
                const FVector ProjectedDimensions(Dimensions[0], Dimensions[1], Dimensions[2]);
                if (Entity.Visual.AssetPath != ExpectedAsset
                    || !ProjectedDimensions.Equals(Entity.Visual.DimensionsMeters, 1.0e-6))
                {
                    return Fail(OutError, EntitySource, TEXT("world entity visual geometry does not match physics geometry"));
                }

                TSharedPtr<FJsonObject> Material;
                FString MaterialSource;
                FString MaterialKey;
                if (!RequireObject(EntityObject, TEXT("material"), EntitySource, Material, OutError)
                    || !RequireString(Material, TEXT("source"), EntitySource + TEXT(".material"), MaterialSource, OutError)
                    || MaterialSource != TEXT("world_package")
                    || !RequireString(Material, TEXT("key"), EntitySource + TEXT(".material"), MaterialKey, OutError)
                    || !ParseCanonicalComponentMaterial(
                        Material,
                        EntitySource + TEXT(".material"),
                        Entity.Visual,
                        OutError)
                    || !ValidateUnitScalar(Entity.Visual.BaseColor.R, TEXT("base_color_rgba[0]"), EntitySource, OutError)
                    || !ValidateUnitScalar(Entity.Visual.BaseColor.G, TEXT("base_color_rgba[1]"), EntitySource, OutError)
                    || !ValidateUnitScalar(Entity.Visual.BaseColor.B, TEXT("base_color_rgba[2]"), EntitySource, OutError)
                    || !ValidateUnitScalar(Entity.Visual.BaseColor.A, TEXT("base_color_rgba[3]"), EntitySource, OutError))
                {
                    return false;
                }

                Entity.Visual.VisualId = Entity.EntityId + TEXT("_visual");
                Entity.Visual.VisualFrameId = Entity.EntityId + TEXT("/visual");
                ParsedEntities.Add(MoveTemp(Entity));
            }

            OutProjection.Entities = MoveTemp(ParsedEntities);
            return true;
        }

        bool PackageMatches(const FPackageRef& PlanPackage, const FPackageRef& ProjectionPackage)
        {
            if (!IsSafeRepositoryRelativePath(PlanPackage.Manifest)
                || !IsSafeRepositoryRelativePath(ProjectionPackage.Manifest))
            {
                return false;
            }
            FString PlanManifest = PlanPackage.Manifest;
            FString ProjectionManifest = ProjectionPackage.Manifest;
            FPaths::NormalizeFilename(PlanManifest);
            FPaths::NormalizeFilename(ProjectionManifest);
            return PlanPackage.Id == ProjectionPackage.Id
                && PlanPackage.Version == ProjectionPackage.Version
                && (PlanManifest == ProjectionManifest
                    || PlanManifest.EndsWith(TEXT("/") + ProjectionManifest));
        }

        FVector BasicShapeScaleFromDimensions(const FVector& DimensionsMeters)
        {
            return FVector(
                DimensionsMeters.X,
                DimensionsMeters.Y,
                DimensionsMeters.Z);
        }

        bool ConfigureStaticMeshComponent(
            UStaticMeshComponent& Component,
            const FProjectionComponent& ProjectionComponent,
            FVisualMaterializationError& OutError)
        {
            UStaticMesh* Mesh = Cast<UStaticMesh>(
                StaticLoadObject(UStaticMesh::StaticClass(), nullptr, *ProjectionComponent.AssetPath));
            if (Mesh == nullptr)
            {
                return Fail(
                    OutError,
                    ProjectionComponent.AssetPath,
                    TEXT("static mesh asset failed to load"));
            }
            Component.SetStaticMesh(Mesh);
            Component.SetMobility(EComponentMobility::Movable);
            ApplyPresentationPolicy(Component);
            Component.SetCastShadow(true);
            return true;
        }

        UMaterialInstanceDynamic* CreateComponentMaterialInstance(
            UObject& Outer,
            const FProjectionComponent& ProjectionComponent,
            FVisualMaterializationError& OutError)
        {
            if (!ProjectionComponent.bHasMaterial)
            {
                return nullptr;
            }

            static constexpr const TCHAR* BaseMaterialPath =
                TEXT("/Game/RobotSim/Materials/M_LingTuVisualSurface.M_LingTuVisualSurface");
            UMaterialInterface* BaseMaterial = Cast<UMaterialInterface>(
                StaticLoadObject(UMaterialInterface::StaticClass(), nullptr, BaseMaterialPath));
            if (BaseMaterial == nullptr)
            {
                Fail(OutError, BaseMaterialPath, TEXT("base visual surface material failed to load"));
                return nullptr;
            }

            UMaterialInstanceDynamic* DynamicMaterial =
                UMaterialInstanceDynamic::Create(BaseMaterial, &Outer);
            if (DynamicMaterial == nullptr)
            {
                Fail(OutError, BaseMaterialPath, TEXT("failed to create dynamic visual surface material"));
                return nullptr;
            }
            DynamicMaterial->SetVectorParameterValue(TEXT("BaseColor"), ProjectionComponent.BaseColor);
            DynamicMaterial->SetScalarParameterValue(TEXT("Metallic"), ProjectionComponent.Metallic);
            DynamicMaterial->SetScalarParameterValue(TEXT("Roughness"), ProjectionComponent.Roughness);
            return DynamicMaterial;
        }

        bool ApplyProjectionMaterial(
            UPrimitiveComponent& Component,
            const FProjectionComponent& ProjectionComponent,
            FVisualMaterializationError& OutError)
        {
            if (!ProjectionComponent.bHasMaterial)
            {
                return true;
            }
            UMaterialInstanceDynamic* DynamicMaterial =
                CreateComponentMaterialInstance(Component, ProjectionComponent, OutError);
            if (DynamicMaterial == nullptr)
            {
                return false;
            }

            const int32 SlotCount = FMath::Max(1, Component.GetNumMaterials());
            for (int32 SlotIndex = 0; SlotIndex < SlotCount; ++SlotIndex)
            {
                Component.SetMaterial(SlotIndex, DynamicMaterial);
            }
            return true;
        }

        bool AttachCapsuleRenderProxy(
            ALingTuSimBodyActor& BodyActor,
            UCapsuleComponent& Capsule,
            const FProjectionComponent& ProjectionComponent,
            FVisualMaterializationError& OutError)
        {
            struct FCapsuleRenderPart
            {
                const TCHAR* Suffix;
                const TCHAR* AssetPath;
                FVector LocationCentimeters;
                FVector Scale;
            };

            const double RadiusMeters = ProjectionComponent.CapsuleRadiusMeters;
            const double CylinderHalfLengthMeters =
                ProjectionComponent.CapsuleCylinderHalfLengthMeters;
            const FCapsuleRenderPart Parts[] = {
                {
                    TEXT("RenderCylinder"),
                    TEXT("/Engine/BasicShapes/Cylinder.Cylinder"),
                    FVector::ZeroVector,
                    FVector(
                        2.0 * RadiusMeters,
                        2.0 * RadiusMeters,
                        2.0 * CylinderHalfLengthMeters)},
                {
                    TEXT("RenderCapTop"),
                    TEXT("/Engine/BasicShapes/Sphere.Sphere"),
                    FVector(0.0, 0.0, CylinderHalfLengthMeters * 100.0),
                    FVector(2.0 * RadiusMeters)},
                {
                    TEXT("RenderCapBottom"),
                    TEXT("/Engine/BasicShapes/Sphere.Sphere"),
                    FVector(0.0, 0.0, -CylinderHalfLengthMeters * 100.0),
                    FVector(2.0 * RadiusMeters)},
            };

            for (const FCapsuleRenderPart& Part : Parts)
            {
                const FName PartName(*FString::Printf(
                    TEXT("%s_%s"),
                    *ProjectionComponent.VisualId,
                    Part.Suffix));
                UStaticMeshComponent* RenderComponent =
                    NewObject<UStaticMeshComponent>(&BodyActor, PartName);
                if (RenderComponent == nullptr)
                {
                    return Fail(
                        OutError,
                        ProjectionComponent.VisualId,
                        TEXT("failed to create capsule render component"));
                }

                FProjectionComponent RenderProjection = ProjectionComponent;
                RenderProjection.AssetPath = Part.AssetPath;
                RenderComponent->SetupAttachment(&Capsule);
                if (!ConfigureStaticMeshComponent(
                        *RenderComponent,
                        RenderProjection,
                        OutError)
                    || !ApplyProjectionMaterial(
                        *RenderComponent,
                        RenderProjection,
                        OutError))
                {
                    RenderComponent->DestroyComponent();
                    return false;
                }
                RenderComponent->SetRelativeTransform(FTransform(
                    FQuat::Identity,
                    Part.LocationCentimeters,
                    Part.Scale));
                BodyActor.AddInstanceComponent(RenderComponent);
                RenderComponent->RegisterComponent();
            }
            return true;
        }

        bool AttachVisualComponent(
            ALingTuSimBodyActor& BodyActor,
            const FProjectionComponent& ProjectionComponent,
            FVisualMaterializationError& OutError)
        {
            const FName ComponentName(*ProjectionComponent.VisualId);
            if (ProjectionComponent.Primitive == TEXT("capsule"))
            {
                UCapsuleComponent* Capsule = NewObject<UCapsuleComponent>(&BodyActor, ComponentName);
                if (Capsule == nullptr)
                {
                    return Fail(OutError, ProjectionComponent.VisualId, TEXT("failed to create capsule component"));
                }
                Capsule->SetupAttachment(BodyActor.GetRootComponent());
                Capsule->SetMobility(EComponentMobility::Movable);
                ApplyPresentationPolicy(*Capsule);
                Capsule->SetCastShadow(true);
                Capsule->SetCapsuleSize(
                    ProjectionComponent.CapsuleRadiusMeters * 100.0,
                    ProjectionComponent.CapsuleHalfHeightMeters * 100.0,
                    false);
                Capsule->SetRelativeTransform(MakeLocalUnrealTransform(
                    ProjectionComponent.LocalTransform,
                    ProjectionComponent.LocalTransform.Scale));
                BodyActor.AddInstanceComponent(Capsule);
                Capsule->RegisterComponent();
                if (!AttachCapsuleRenderProxy(
                        BodyActor,
                        *Capsule,
                        ProjectionComponent,
                        OutError))
                {
                    Capsule->DestroyComponent();
                    return false;
                }
                return true;
            }

            UStaticMeshComponent* StaticMesh = NewObject<UStaticMeshComponent>(&BodyActor, ComponentName);
            if (StaticMesh == nullptr)
            {
                return Fail(OutError, ProjectionComponent.VisualId, TEXT("failed to create static mesh component"));
            }
            StaticMesh->SetupAttachment(BodyActor.GetRootComponent());
            if (!ConfigureStaticMeshComponent(*StaticMesh, ProjectionComponent, OutError))
            {
                StaticMesh->DestroyComponent();
                return false;
            }
            if (!ApplyProjectionMaterial(*StaticMesh, ProjectionComponent, OutError))
            {
                StaticMesh->DestroyComponent();
                return false;
            }

            const FVector ShapeScale =
                ProjectionComponent.GeometryKind == TEXT("mesh")
                    ? FVector::OneVector
                    : BasicShapeScaleFromDimensions(ProjectionComponent.DimensionsMeters);
            StaticMesh->SetRelativeTransform(MakeLocalUnrealTransform(
                ProjectionComponent.LocalTransform,
                ShapeScale * ProjectionComponent.LocalTransform.Scale));
            BodyActor.AddInstanceComponent(StaticMesh);
            StaticMesh->RegisterComponent();
            return true;
        }

        bool AttachWorldEntityVisual(
            ALingTuSimWorldEntityActor& Actor,
            const FWorldEntityProjection& Entity,
            FVisualMaterializationError& OutError)
        {
            const FName ComponentName(*Entity.Visual.VisualId);
            UStaticMeshComponent* StaticMesh = NewObject<UStaticMeshComponent>(
                &Actor,
                ComponentName);
            if (StaticMesh == nullptr)
            {
                return Fail(
                    OutError,
                    Entity.EntityId,
                    TEXT("failed to create world entity static mesh component"));
            }

            StaticMesh->SetupAttachment(Actor.GetRootComponent());
            if (!ConfigureStaticMeshComponent(*StaticMesh, Entity.Visual, OutError)
                || !ApplyProjectionMaterial(*StaticMesh, Entity.Visual, OutError))
            {
                StaticMesh->DestroyComponent();
                return false;
            }
            StaticMesh->SetRelativeTransform(FTransform(
                FQuat::Identity,
                FVector::ZeroVector,
                BasicShapeScaleFromDimensions(Entity.Visual.DimensionsMeters)));
            Actor.AddInstanceComponent(StaticMesh);
            StaticMesh->RegisterComponent();
            Actor.SetActorTransform(MakeLocalUnrealTransform(
                Entity.WorldTransform,
                FVector::OneVector));
            return true;
        }
    }

    bool FRobotVisualProjectionMaterializer::MaterializeBundle(
        UWorld& World,
        const FString& BundleDirectory,
        const FString& ArtifactRoot,
        const uint64 ModelGeneration,
        const uint64 ResetGeneration,
        FVisualMaterializationResult& OutResult,
        FVisualMaterializationError& OutError)
    {
        OutError.Reset();
        OutResult = FVisualMaterializationResult();

        FSessionBundleView Bundle;
        FRuntimeLoadError BundleError;
        if (!FSessionBundleLoader::LoadSessionBundle(BundleDirectory, Bundle, BundleError))
        {
            return Fail(OutError, BundleError.Source, BundleError.Message);
        }

        TArray<FVisualPlanInstance> Instances;
        FWorldProjectionPlan WorldProjectionPlan;
        if (!ParseVisualPlan(
                Bundle.VisualPlanPath,
                Bundle.SessionId,
                Instances,
                WorldProjectionPlan,
                OutError))
        {
            return false;
        }

        TArray<TObjectPtr<ALingTuSimBodyActor>> SpawnedActors;
        TArray<TObjectPtr<ALingTuSimWorldEntityActor>> SpawnedWorldActors;
        TSet<FString> GlobalStableBodyIds;
        TSet<FString> GlobalStableIds;
        bool bKeepSpawnedActors = false;
        ON_SCOPE_EXIT
        {
            if (!bKeepSpawnedActors)
            {
                for (ALingTuSimBodyActor* Actor : SpawnedActors)
                {
                    if (IsValid(Actor))
                    {
                        Actor->Destroy();
                    }
                }
                for (ALingTuSimWorldEntityActor* Actor : SpawnedWorldActors)
                {
                    if (IsValid(Actor))
                    {
                        Actor->Destroy();
                    }
                }
            }
        };

        if (WorldProjectionPlan.bPresent)
        {
            FString ProjectionPath;
            if (!ResolveContainedPath(
                    ArtifactRoot,
                    WorldProjectionPlan.ProjectionPath,
                    ProjectionPath,
                    OutError))
            {
                return false;
            }
            FWorldProjectionDocument Projection;
            if (!ParseWorldProjection(
                    ProjectionPath,
                    WorldProjectionPlan.ProjectionSchema,
                    Projection,
                    OutError))
            {
                return false;
            }
            if (Projection.Binding != WorldProjectionPlan.Binding
                || Projection.Level != WorldProjectionPlan.Level)
            {
                return Fail(
                    OutError,
                    ProjectionPath,
                    TEXT("world projection binding or level does not match visual plan"));
            }
            if (!PackageMatches(WorldProjectionPlan.Package, Projection.Package))
            {
                return Fail(
                    OutError,
                    ProjectionPath,
                    TEXT("world projection package identity does not match visual plan"));
            }

            for (const FWorldEntityProjection& Entity : Projection.Entities)
            {
                const FString StableId = TEXT("world/") + Entity.EntityId;
                if (GlobalStableIds.Contains(StableId))
                {
                    return Fail(
                        OutError,
                        ProjectionPath,
                        TEXT("duplicate world entity stable ID"));
                }

                FActorSpawnParameters SpawnParameters;
                SpawnParameters.Name = MakeUniqueObjectName(
                    &World,
                    ALingTuSimWorldEntityActor::StaticClass(),
                    FName(*StableId.Replace(TEXT("/"), TEXT("_"))));
                SpawnParameters.SpawnCollisionHandlingOverride =
                    ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
                ALingTuSimWorldEntityActor* Actor =
                    World.SpawnActor<ALingTuSimWorldEntityActor>(
                        ALingTuSimWorldEntityActor::StaticClass(),
                        FTransform::Identity,
                        SpawnParameters);
                if (Actor == nullptr)
                {
                    return Fail(
                        OutError,
                        StableId,
                        TEXT("failed to spawn world entity actor"));
                }
                SpawnedWorldActors.Add(Actor);
                if (!Actor->ConfigureIdentity(
                        StableId,
                        Entity.SemanticClass,
                        Entity.Authority)
                    || !AttachWorldEntityVisual(*Actor, Entity, OutError))
                {
                    return Fail(
                        OutError,
                        StableId,
                        OutError.Message.IsEmpty()
                            ? TEXT("failed to configure world entity actor")
                            : OutError.Message);
                }
                Actor->SetActorHiddenInGame(true);
#if WITH_EDITOR
                Actor->SetIsTemporarilyHiddenInEditor(true);
#endif
                GlobalStableIds.Add(StableId);
            }
        }

        for (const FVisualPlanInstance& Instance : Instances)
        {
            FString ProjectionPath;
            if (!ResolveContainedPath(ArtifactRoot, Instance.ProjectionPath, ProjectionPath, OutError))
            {
                return false;
            }
            FProjectionDocument Projection;
            const bool bProjectionParsed = Instance.bPayload
                ? ParsePayloadProjection(ProjectionPath, Projection, OutError)
                : ParseProjection(
                    ProjectionPath,
                    Instance.ProjectionSchema,
                    Projection,
                    OutError);
            if (!bProjectionParsed)
            {
                return false;
            }
            if (Projection.Binding != Instance.Binding)
            {
                return Fail(OutError, ProjectionPath, TEXT("projection binding does not match visual plan"));
            }
            if (!PackageMatches(Instance.Package, Projection.Package))
            {
                return Fail(OutError, ProjectionPath, TEXT("projection package identity does not match visual plan"));
            }

            TMap<FString, ALingTuSimBodyActor*> BodyActors;
            for (const FProjectionComponent& Component : Projection.Components)
            {
                const FString StableBodyId = Instance.InstanceId + TEXT("/") + Component.LocalBodyId;
                const FString StableVisualId = Instance.InstanceId + TEXT("/") + Component.VisualFrameId;
                if (GlobalStableIds.Contains(StableVisualId))
                {
                    return Fail(OutError, ProjectionPath, TEXT("prefixed body/visual IDs must be disjoint"));
                }
                ALingTuSimBodyActor* BodyActor = BodyActors.FindRef(Component.LocalBodyId);
                if (BodyActor == nullptr)
                {
                    if (GlobalStableIds.Contains(StableBodyId))
                    {
                        return Fail(OutError, ProjectionPath, TEXT("duplicate prefixed body ID"));
                    }
                    FActorSpawnParameters SpawnParameters;
                    SpawnParameters.Name = MakeUniqueObjectName(
                        &World,
                        ALingTuSimBodyActor::StaticClass(),
                        FName(*StableBodyId.Replace(TEXT("/"), TEXT("_"))));
                    SpawnParameters.SpawnCollisionHandlingOverride =
                        ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
                    BodyActor = World.SpawnActor<ALingTuSimBodyActor>(
                        ALingTuSimBodyActor::StaticClass(),
                        FTransform::Identity,
                        SpawnParameters);
                    if (BodyActor == nullptr || !BodyActor->SetBodyStableId(StableBodyId))
                    {
                        return Fail(OutError, StableBodyId, TEXT("failed to spawn body actor"));
                    }
                    BodyActor->SetActorHiddenInGame(true);
#if WITH_EDITOR
                    BodyActor->SetIsTemporarilyHiddenInEditor(true);
#endif
                    BodyActors.Add(Component.LocalBodyId, BodyActor);
                    GlobalStableBodyIds.Add(StableBodyId);
                    GlobalStableIds.Add(StableBodyId);
                    SpawnedActors.Add(BodyActor);
                }
                GlobalStableIds.Add(StableVisualId);

                if (!AttachVisualComponent(*BodyActor, Component, OutError))
                {
                    return false;
                }
            }
        }

        if (SpawnedActors.Num() <= 0)
        {
            return Fail(
                OutError,
                Bundle.VisualPlanPath,
                TEXT("visual plan materialized no body actors"));
        }

        OutResult.SessionId = Bundle.SessionId;
        OutResult.ModelGeneration = ModelGeneration;
        OutResult.ResetGeneration = ResetGeneration;
        OutResult.ExpectedBodyCount = SpawnedActors.Num();
        OutResult.ExpectedWorldEntityCount = SpawnedWorldActors.Num();
        OutResult.Actors = MoveTemp(SpawnedActors);
        OutResult.WorldActors = MoveTemp(SpawnedWorldActors);
        bKeepSpawnedActors = true;
        return true;
    }
}

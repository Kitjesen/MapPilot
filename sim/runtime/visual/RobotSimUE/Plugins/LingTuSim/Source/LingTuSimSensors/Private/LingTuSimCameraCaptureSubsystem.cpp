#include "LingTuSimCameraCaptureSubsystem.h"

#include "Async/Async.h"
#include "Engine/TextureRenderTarget2D.h"
#include "GameFramework/Actor.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "LingTuCameraShmWriter.h"
#include "LingTuSimBundleLoader.h"
#include "LingTuSimBodyBindingComponent.h"
#include "LingTuSimVisualWorldSubsystem.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/Parse.h"
#include "Modules/ModuleManager.h"
#include "RHIGPUReadback.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "ShowFlags.h"
#include "UObject/Package.h"
#include "UObject/UObjectIterator.h"
#include "Components/SceneCaptureComponent2D.h"
#include "RenderingThread.h"

DEFINE_LOG_CATEGORY_STATIC(LogLingTuSimCamera, Log, All);

namespace
{
    using namespace LingTuSim::Sensors::Camera;
    using LingTuSim::Sensors::CameraShm::FFrameMetadata;
    using LingTuSim::Sensors::CameraShm::FFrameWriter;
    using LingTuSim::Sensors::CameraShm::FWriterConfig;

    constexpr double CameraVisualBindingTimeoutSeconds = 30.0;
    constexpr double CameraFirstFramePipelineTimeoutSeconds = 30.0;
    constexpr double CameraCapturePipelineTimeoutSeconds = 1.0;
    constexpr int32 MaxPendingReadbacksPerStream = 3;

    bool ReadRequiredString(
        const TSharedPtr<FJsonObject>& Object,
        const TCHAR* Key,
        FString& Out,
        FString& OutError)
    {
        if (!Object.IsValid() || !Object->TryGetStringField(Key, Out) || Out.IsEmpty())
        {
            OutError = FString::Printf(TEXT("missing non-empty string field '%s'"), Key);
            return false;
        }
        return true;
    }

    bool ReadPositiveNumber(
        const TSharedPtr<FJsonObject>& Object,
        const TCHAR* Key,
        double& Out,
        FString& OutError)
    {
        if (!Object.IsValid() || !Object->TryGetNumberField(Key, Out)
            || !FMath::IsFinite(Out) || Out <= 0.0)
        {
            OutError = FString::Printf(TEXT("field '%s' must be finite and positive"), Key);
            return false;
        }
        return true;
    }

    bool ReadPositiveInt(
        const TSharedPtr<FJsonObject>& Object,
        const TCHAR* Key,
        int32& Out,
        FString& OutError)
    {
        double Value = 0.0;
        if (!ReadPositiveNumber(Object, Key, Value, OutError)
            || Value > static_cast<double>(MAX_int32)
            || FMath::FloorToInt(Value) != Value)
        {
            OutError = FString::Printf(TEXT("field '%s' must be a positive integer"), Key);
            return false;
        }
        Out = static_cast<int32>(Value);
        return true;
    }

    bool ParseJsonObject(
        const FString& Json,
        TSharedPtr<FJsonObject>& OutObject,
        FString& OutError)
    {
        TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Json);
        if (!FJsonSerializer::Deserialize(Reader, OutObject) || !OutObject.IsValid())
        {
            OutError = TEXT("invalid JSON object");
            return false;
        }
        return true;
    }

    FString SafeField(const TSharedPtr<FJsonObject>& Object, const TCHAR* Key)
    {
        FString Value;
        Object->TryGetStringField(Key, Value);
        return Value;
    }


    FString EscapeCameraReadinessJsonString(const FString& Value)
    {
        FString Escaped;
        Escaped.Reserve(Value.Len() + 8);
        for (const TCHAR Character : Value)
        {
            switch (Character)
            {
            case TEXT('"'):
                Escaped += TEXT("\\\"");
                break;
            case TEXT('\\'):
                Escaped += TEXT("\\\\");
                break;
            case TEXT('\b'):
                Escaped += TEXT("\\b");
                break;
            case TEXT('\f'):
                Escaped += TEXT("\\f");
                break;
            case TEXT('\n'):
                Escaped += TEXT("\\n");
                break;
            case TEXT('\r'):
                Escaped += TEXT("\\r");
                break;
            case TEXT('\t'):
                Escaped += TEXT("\\t");
                break;
            default:
                if (Character < 0x20)
                {
                    Escaped += FString::Printf(TEXT("\\u%04x"), static_cast<uint32>(Character));
                }
                else
                {
                    Escaped.AppendChar(Character);
                }
                break;
            }
        }
        return Escaped;
    }

    bool ParseStream(
        const TSharedPtr<FJsonObject>& Object,
        const FString& KindName,
        FStreamPlan& Out,
        FString& OutError)
    {
        if (!ReadRequiredString(Object, TEXT("sensor_id"), Out.SensorId, OutError)
            || !ReadRequiredString(Object, TEXT("frame_id"), Out.FrameId, OutError)
            || !ReadRequiredString(Object, TEXT("parent_frame_id"), Out.ParentFrameId, OutError)
            || !ReadRequiredString(Object, TEXT("source"), Out.Source, OutError)
            || !ReadRequiredString(Object, TEXT("transport"), Out.Transport, OutError)
            || !ReadPositiveNumber(Object, TEXT("rate_hz"), Out.RateHz, OutError))
        {
            return false;
        }
        if (Out.Source != TEXT("unreal_camera") || Out.Transport != TEXT("camera_shm"))
        {
            OutError = FString::Printf(
                TEXT("camera stream '%s' must route through source=unreal_camera and transport=camera_shm"),
                *Out.SensorId);
            return false;
        }

        if (KindName == TEXT("rgb"))
        {
            Out.Kind = EStreamKind::Color;
            if (!ReadRequiredString(Object, TEXT("encoding"), Out.Encoding, OutError)
                || (Out.Encoding != TEXT("rgb8") && Out.Encoding != TEXT("rgba8")))
            {
                OutError = FString::Printf(
                    TEXT("RGB stream '%s' must use rgb8 or rgba8"), *Out.SensorId);
                return false;
            }
        }
        else if (KindName == TEXT("depth"))
        {
            Out.Kind = EStreamKind::Depth;
            if (!ReadRequiredString(Object, TEXT("encoding"), Out.Encoding, OutError)
                || (Out.Encoding != TEXT("16UC1") && Out.Encoding != TEXT("32FC1")))
            {
                OutError = FString::Printf(
                    TEXT("depth stream '%s' must declare 16UC1 or 32FC1 input encoding"),
                    *Out.SensorId);
                return false;
            }
            FString Unit = SafeField(Object, TEXT("unit"));
            if (!Unit.IsEmpty() && Unit != TEXT("m"))
            {
                OutError = FString::Printf(
                    TEXT("depth stream '%s' must declare unit=m"), *Out.SensorId);
                return false;
            }
            Out.Encoding = TEXT("16UC1");
            Out.DepthScale = 0.001;
        }
        else
        {
            OutError = FString::Printf(TEXT("unsupported camera stream kind '%s'"), *KindName);
            return false;
        }

        if (!ReadPositiveInt(Object, TEXT("width"), Out.Width, OutError)
            || !ReadPositiveInt(Object, TEXT("height"), Out.Height, OutError)
            || Out.Width > 8192 || Out.Height > 8192)
        {
            OutError = FString::Printf(
                TEXT("camera stream '%s' has invalid resolution"), *Out.SensorId);
            return false;
        }

        const TSharedPtr<FJsonObject>* Extrinsic = nullptr;
        if (!Object->TryGetObjectField(TEXT("extrinsic"), Extrinsic)
            || Extrinsic == nullptr || !Extrinsic->IsValid())
        {
            OutError = FString::Printf(
                TEXT("camera stream '%s' requires explicit SensorRig extrinsic"),
                *Out.SensorId);
            return false;
        }
        {
            const TArray<TSharedPtr<FJsonValue>>* Position = nullptr;
            const TArray<TSharedPtr<FJsonValue>>* Quaternion = nullptr;
            if (!(*Extrinsic)->TryGetArrayField(TEXT("position_m"), Position)
                || !(*Extrinsic)->TryGetArrayField(TEXT("quaternion_wxyz"), Quaternion)
                || Position == nullptr || Quaternion == nullptr
                || Position->Num() != 3 || Quaternion->Num() != 4)
            {
                OutError = FString::Printf(
                    TEXT("camera stream '%s' has invalid extrinsic; expected position_m[3] and quaternion_wxyz[4]"),
                    *Out.SensorId);
                return false;
            }
            TArray<double> PositionValues;
            TArray<double> QuaternionValues;
            for (const TSharedPtr<FJsonValue>& Value : *Position)
            {
                double Number = 0.0;
                if (!Value.IsValid() || !Value->TryGetNumber(Number) || !FMath::IsFinite(Number))
                {
                    OutError = TEXT("camera extrinsic position contains non-finite data");
                    return false;
                }
                PositionValues.Add(Number);
            }
            for (const TSharedPtr<FJsonValue>& Value : *Quaternion)
            {
                double Number = 0.0;
                if (!Value.IsValid() || !Value->TryGetNumber(Number) || !FMath::IsFinite(Number))
                {
                    OutError = TEXT("camera extrinsic quaternion contains non-finite data");
                    return false;
                }
                QuaternionValues.Add(Number);
            }
            Out.ExtrinsicTranslationMeters = FVector(
                PositionValues[0], PositionValues[1], PositionValues[2]);
            Out.ExtrinsicRotation = FQuat(
                QuaternionValues[1],
                QuaternionValues[2],
                QuaternionValues[3],
                QuaternionValues[0]);
            if (Out.ExtrinsicRotation.SizeSquared() <= UE_SMALL_NUMBER)
            {
                OutError = FString::Printf(
                    TEXT("camera stream '%s' extrinsic quaternion is degenerate"), *Out.SensorId);
                return false;
            }
            Out.ExtrinsicRotation.Normalize();
            Out.bHasExtrinsic = true;
        }
        return true;
    }

    bool ParseGeneration(
        const TCHAR* ArgumentName,
        const bool bInitialBind,
        uint64& OutGeneration,
        FString& OutError)
    {
        OutGeneration = 0;
        if (FParse::Value(FCommandLine::Get(), ArgumentName, OutGeneration))
        {
            return true;
        }
        if (bInitialBind)
        {
            return true;
        }
        OutError = FString::Printf(TEXT("%s is required for a non-initial bind"), ArgumentName);
        return false;
    }

    bool ParseCommaVector(
        const FString& Text,
        FVector& Out)
    {
        TArray<FString> Parts;
        Text.ParseIntoArray(Parts, TEXT(","), true);
        if (Parts.Num() != 3)
        {
            return false;
        }
        Out.X = FCString::Atod(*Parts[0]);
        Out.Y = FCString::Atod(*Parts[1]);
        Out.Z = FCString::Atod(*Parts[2]);
        return Out.ContainsNaN() == false;
    }

    bool ReadCommandLineToken(
        const TCHAR* Prefix,
        FString& Out)
    {
        const FString CommandLine = FCommandLine::Get();
        const int32 PrefixIndex = CommandLine.Find(
            Prefix, ESearchCase::IgnoreCase, ESearchDir::FromStart);
        if (PrefixIndex == INDEX_NONE)
        {
            return false;
        }
        const int32 ValueStart = PrefixIndex + FCString::Strlen(Prefix);
        int32 ValueEnd = CommandLine.Find(
            TEXT(" "), ESearchCase::IgnoreCase, ESearchDir::FromStart, ValueStart);
        if (ValueEnd == INDEX_NONE)
        {
            ValueEnd = CommandLine.Len();
        }
        Out = CommandLine.Mid(ValueStart, ValueEnd - ValueStart);
        Out.TrimStartAndEndInline();
        return !Out.IsEmpty();
    }

    bool ReplaceReadinessFileWithRetry(
        const FString& TargetPath,
        const FString& TempPath)
    {
#if PLATFORM_WINDOWS
        constexpr int32 MaxAttempts = 5;
#else
        constexpr int32 MaxAttempts = 1;
#endif
        for (int32 Attempt = 0; Attempt < MaxAttempts; ++Attempt)
        {
            if (IFileManager::Get().Move(
                    *TargetPath,
                    *TempPath,
                    true,
                    true,
                    false,
                    true))
            {
                return true;
            }
#if PLATFORM_WINDOWS
            if (Attempt + 1 < MaxAttempts)
            {
                FPlatformProcess::SleepNoStats(0.005F * static_cast<float>(Attempt + 1));
            }
#endif
        }
        return false;
    }
}

namespace LingTuSim::Sensors::Camera
{
    EWorldBindingAction EvaluateWorldBindingAction(
        const EWorldType::Type WorldType,
        const bool bHasBegunPlay,
        const bool bWorldInitialized,
        const bool bWorldTearingDown,
        TFunctionRef<int32()> VisualBindingCountProvider)
    {
        switch (WorldType)
        {
        case EWorldType::Game:
        case EWorldType::PIE:
        case EWorldType::GamePreview:
            break;
        default:
            return EWorldBindingAction::Skip;
        }
        if (bWorldTearingDown)
        {
            return EWorldBindingAction::Skip;
        }
        if (!bWorldInitialized || !bHasBegunPlay)
        {
            return EWorldBindingAction::Wait;
        }

        const int32 VisualBindingCount = VisualBindingCountProvider();
        if (VisualBindingCount <= 0)
        {
            return EWorldBindingAction::Wait;
        }
        return EWorldBindingAction::Bind;
    }

    bool HasWorldBindingWaitExpired(
        const bool bHasBegunPlay,
        const double ElapsedSeconds,
        const double TimeoutSeconds)
    {
        return bHasBegunPlay
            && FMath::IsFinite(ElapsedSeconds)
            && FMath::IsFinite(TimeoutSeconds)
            && TimeoutSeconds > 0.0
            && ElapsedSeconds >= TimeoutSeconds;
    }

    double SelectCapturePipelineTimeoutSeconds(
        const uint64 PublishedFrames,
        const double StartupTimeoutSeconds,
        const double SteadyTimeoutSeconds)
    {
        if (!FMath::IsFinite(StartupTimeoutSeconds)
            || StartupTimeoutSeconds <= 0.0
            || !FMath::IsFinite(SteadyTimeoutSeconds)
            || SteadyTimeoutSeconds <= 0.0)
        {
            return 0.0;
        }
        return PublishedFrames == 0 ? StartupTimeoutSeconds : SteadyTimeoutSeconds;
    }

    EReadbackDeadlineAction EvaluateReadbackStageDeadline(
        const EReadbackDeadlineStage Stage,
        const double StageStartedAtSeconds,
        const double NowSeconds,
        const double TimeoutSeconds,
        FString& OutError)
    {
        OutError.Reset();
        if (!FMath::IsFinite(TimeoutSeconds) || TimeoutSeconds <= 0.0)
        {
            OutError = TEXT("camera readback pipeline deadline is invalid");
            return EReadbackDeadlineAction::Fail;
        }
        if (Stage == EReadbackDeadlineStage::CpuReady)
        {
            return EReadbackDeadlineAction::Complete;
        }
        if (Stage != EReadbackDeadlineStage::EnqueuePending
            && Stage != EReadbackDeadlineStage::GpuPending
            && Stage != EReadbackDeadlineStage::CpuDecodePending)
        {
            OutError = TEXT("camera readback deadline stage is invalid");
            return EReadbackDeadlineAction::Fail;
        }
        if (!FMath::IsFinite(StageStartedAtSeconds) || StageStartedAtSeconds < 0.0
            || !FMath::IsFinite(NowSeconds))
        {
            OutError = Stage == EReadbackDeadlineStage::EnqueuePending
                ? TEXT("camera render-command queue wall-clock age is invalid")
                : Stage == EReadbackDeadlineStage::GpuPending
                    ? TEXT("asynchronous GPU readback wall-clock age is invalid")
                    : TEXT("asynchronous camera CPU decode wall-clock age is invalid");
            return EReadbackDeadlineAction::Fail;
        }

        const double AgeSeconds = NowSeconds - StageStartedAtSeconds;
        if (!FMath::IsFinite(AgeSeconds))
        {
            OutError = Stage == EReadbackDeadlineStage::EnqueuePending
                ? TEXT("camera render-command queue wall-clock age is invalid")
                : Stage == EReadbackDeadlineStage::GpuPending
                    ? TEXT("asynchronous GPU readback wall-clock age is invalid")
                    : TEXT("asynchronous camera CPU decode wall-clock age is invalid");
            return EReadbackDeadlineAction::Fail;
        }
        if (AgeSeconds < 0.0)
        {
            // A game-thread timestamp may precede a render-thread stage
            // transition observed later in the same pump.
            return EReadbackDeadlineAction::Wait;
        }
        if (AgeSeconds < TimeoutSeconds)
        {
            return EReadbackDeadlineAction::Wait;
        }

        if (Stage == EReadbackDeadlineStage::EnqueuePending)
        {
            OutError = FString::Printf(
                TEXT("camera render command exceeded its %.3f second pipeline deadline (stage=EnqueuePending age=%.3f deadline=%.3f)"),
                TimeoutSeconds,
                AgeSeconds,
                TimeoutSeconds);
        }
        else if (Stage == EReadbackDeadlineStage::GpuPending)
        {
            OutError = FString::Printf(
                TEXT("asynchronous GPU readback exceeded its %.3f second pipeline deadline (stage=GpuPending age=%.3f deadline=%.3f)"),
                TimeoutSeconds,
                AgeSeconds,
                TimeoutSeconds);
        }
        else
        {
            OutError = FString::Printf(
                TEXT("asynchronous camera CPU decode exceeded its %.3f second pipeline deadline (stage=CpuDecodePending age=%.3f deadline=%.3f)"),
                TimeoutSeconds,
                AgeSeconds,
                TimeoutSeconds);
        }
        return EReadbackDeadlineAction::Fail;
    }

    EDeferredCaptureAction EvaluateDeferredCapturePromotion(
        const FTruthSampleStamp& DeferredTruth,
        const FString& ExpectedSessionId,
        const uint64 ExpectedModelGeneration,
        const uint64 ExpectedResetGeneration,
        const uint64 SubmittedFrame,
        const uint64 CurrentFrame,
        const double SubmittedAtSeconds,
        const double NowSeconds,
        const int32 PendingReadbackCount,
        const int32 MaxPendingReadbackCount,
        const double TimeoutSeconds,
        FString& OutError)
    {
        OutError.Reset();
        if (DeferredTruth.SessionId.IsEmpty()
            || DeferredTruth.SimTimeNs < 0
            || !FMath::IsFinite(SubmittedAtSeconds)
            || !FMath::IsFinite(NowSeconds)
            || !FMath::IsFinite(TimeoutSeconds)
            || TimeoutSeconds <= 0.0
            || PendingReadbackCount < 0
            || MaxPendingReadbackCount <= 0)
        {
            OutError = TEXT("deferred capture promotion inputs are invalid");
            return EDeferredCaptureAction::Fail;
        }
        if (DeferredTruth.SessionId != ExpectedSessionId
            || DeferredTruth.ModelGeneration != ExpectedModelGeneration
            || DeferredTruth.ResetGeneration != ExpectedResetGeneration)
        {
            return EDeferredCaptureAction::Discard;
        }
        if (CurrentFrame < SubmittedFrame)
        {
            OutError = TEXT("deferred capture current frame regressed before readback promotion");
            return EDeferredCaptureAction::Fail;
        }
        const double AgeSeconds = NowSeconds - SubmittedAtSeconds;
        if (!FMath::IsFinite(AgeSeconds) || AgeSeconds < 0.0)
        {
            OutError = TEXT("deferred capture wall-clock age is invalid");
            return EDeferredCaptureAction::Fail;
        }
        if (AgeSeconds >= TimeoutSeconds)
        {
            OutError = FString::Printf(
                TEXT("deferred capture exceeded its %.3f second pipeline deadline (stage=DeferredCapture age=%.3f deadline=%.3f)"),
                TimeoutSeconds,
                AgeSeconds,
                TimeoutSeconds);
            return EDeferredCaptureAction::Fail;
        }
        if (CurrentFrame == SubmittedFrame
            || PendingReadbackCount >= MaxPendingReadbackCount)
        {
            return EDeferredCaptureAction::Wait;
        }
        return EDeferredCaptureAction::Promote;
    }

    bool ShouldRenderCameraCaptureInMainRenderer(
        const EStreamKind StreamKind,
        const ESceneCaptureSource CaptureSource,
        const bool bDepthOptionEnabled)
    {
        return bDepthOptionEnabled
            && StreamKind == EStreamKind::Depth
            && CaptureSource == ESceneCaptureSource::SCS_SceneDepth;
    }

    void ConfigureCameraCaptureShowFlags(
        const EStreamKind StreamKind,
        FEngineShowFlags& ShowFlags)
    {
        if (StreamKind != EStreamKind::Depth)
        {
            return;
        }

        // SceneDepth depends on visible geometry, not Lumen, shadows, atmosphere,
        // or post processing. UE provides these helpers for temporary/unlit
        // scene captures and leaves geometry show flags enabled.
        ShowFlags.DisableAdvancedFeatures();
        ShowFlags.DisableFeaturesForUnlit(false);
    }

    bool ParseCameraPlanJson(
        const FString& Json,
        FCameraPlan& OutPlan,
        FString& OutError)
    {
        OutPlan = FCameraPlan{};
        OutError.Reset();
        TSharedPtr<FJsonObject> Root;
        if (!ParseJsonObject(Json, Root, OutError))
        {
            return false;
        }
        if (SafeField(Root, TEXT("schema")) != TEXT("lingtu.sim.sensor-plan.v1"))
        {
            OutError = TEXT("sensor plan schema mismatch");
            return false;
        }
        if (!ReadRequiredString(Root, TEXT("session_id"), OutPlan.SessionId, OutError))
        {
            return false;
        }
        const TSharedPtr<FJsonObject>* StreamsObject = nullptr;
        if (!Root->TryGetObjectField(TEXT("streams"), StreamsObject) || !StreamsObject->IsValid())
        {
            OutError = TEXT("sensor plan has no streams object");
            return false;
        }

        for (const TPair<FString, TSharedPtr<FJsonValue>>& Pair : (*StreamsObject)->Values)
        {
            const TArray<TSharedPtr<FJsonValue>>* Values = nullptr;
            if (Pair.Value.IsValid() && Pair.Value->TryGetArray(Values) && Values != nullptr)
            {
                OutPlan.TotalStreamCount += Values->Num();
            }
        }

        for (const TCHAR* KindName : {TEXT("rgb"), TEXT("depth")})
        {
            const TArray<TSharedPtr<FJsonValue>>* Values = nullptr;
            if (!(*StreamsObject)->TryGetArrayField(KindName, Values))
            {
                continue;
            }
            for (const TSharedPtr<FJsonValue>& Value : *Values)
            {
                const TSharedPtr<FJsonObject>* StreamObject = nullptr;
                if (!Value.IsValid() || !Value->TryGetObject(StreamObject) || !StreamObject->IsValid())
                {
                    OutError = FString::Printf(TEXT("stream '%s' is not an object"), KindName);
                    return false;
                }
                FStreamPlan Stream;
                Stream.SessionId = OutPlan.SessionId;
                if (!ParseStream(*StreamObject, KindName, Stream, OutError))
                {
                    return false;
                }
                if (OutPlan.Streams.ContainsByPredicate([&Stream](const FStreamPlan& Existing)
                    { return Existing.SensorId == Stream.SensorId; }))
                {
                    OutError = FString::Printf(TEXT("duplicate camera sensor_id '%s'"), *Stream.SensorId);
                    return false;
                }
                OutPlan.Streams.Add(MoveTemp(Stream));
            }
        }
        if (OutPlan.Streams.Num() == 0 || OutPlan.TotalStreamCount == 0)
        {
            OutError = TEXT("sensor plan contains no Unreal camera streams");
            return false;
        }
        return true;
    }

    bool ParseRunAllocationJson(
        const FString& Json,
        FRunAllocation& OutAllocation,
        FString& OutError)
    {
        OutAllocation = FRunAllocation{};
        OutError.Reset();
        TSharedPtr<FJsonObject> Root;
        if (!ParseJsonObject(Json, Root, OutError))
        {
            return false;
        }
        if (SafeField(Root, TEXT("schema")) != TEXT("lingtu.sim.run-allocation.v1"))
        {
            OutError = TEXT("run allocation schema mismatch");
            return false;
        }
        if (!ReadRequiredString(Root, TEXT("run_id"), OutAllocation.RunId, OutError)
            || !ReadRequiredString(Root, TEXT("session_id"), OutAllocation.SessionId, OutError))
        {
            return false;
        }
        OutAllocation.EvidenceDirectory = SafeField(Root, TEXT("log_dir"));
        const TSharedPtr<FJsonObject>* Shm = nullptr;
        if (!Root->TryGetObjectField(TEXT("shm"), Shm) || !Shm->IsValid())
        {
            OutError = TEXT("run allocation has no shm object");
            return false;
        }
        for (const TPair<FString, TSharedPtr<FJsonValue>>& Pair : (*Shm)->Values)
        {
            FString Mapping;
            if (!Pair.Value.IsValid() || !Pair.Value->TryGetString(Mapping) || Mapping.IsEmpty())
            {
                OutError = FString::Printf(TEXT("shm mapping for '%s' is invalid"), *Pair.Key);
                return false;
            }
            if (OutAllocation.SharedMemoryBySensor.Contains(Pair.Key))
            {
                OutError = FString::Printf(TEXT("duplicate shm mapping for '%s'"), *Pair.Key);
                return false;
            }
            OutAllocation.SharedMemoryBySensor.Add(Pair.Key, Mapping);
        }
        return true;
    }

    bool ConvertColorPixels(
        const TArray<FColor>& Pixels,
        const int32 Width,
        const int32 Height,
        const FString& Encoding,
        TArray<uint8>& OutPayload,
        FString& OutError)
    {
        OutPayload.Reset();
        OutError.Reset();
        if (Width <= 0 || Height <= 0 || Pixels.Num() != Width * Height)
        {
            OutError = TEXT("RGB readback dimensions do not match the plan");
            return false;
        }
        const int32 Channels = Encoding == TEXT("rgb8") ? 3 : Encoding == TEXT("rgba8") ? 4 : 0;
        if (Channels == 0)
        {
            OutError = TEXT("RGB output encoding must be rgb8 or rgba8");
            return false;
        }
        OutPayload.SetNumUninitialized(Pixels.Num() * Channels);
        uint8* Destination = OutPayload.GetData();
        for (const FColor& Pixel : Pixels)
        {
            *Destination++ = Pixel.R;
            *Destination++ = Pixel.G;
            *Destination++ = Pixel.B;
            if (Channels == 4)
            {
                *Destination++ = Pixel.A;
            }
        }
        return true;
    }

    bool ConvertDepthPixels(
        const TArray<FLinearColor>& Pixels,
        const int32 Width,
        const int32 Height,
        const double DepthScale,
        TArray<uint8>& OutPayload,
        FString& OutError)
    {
        if (Width <= 0 || Height <= 0 || Pixels.Num() != Width * Height)
        {
            OutPayload.Reset();
            OutError.Reset();
            OutError = TEXT("depth readback dimensions do not match the plan");
            return false;
        }
        TArray<float> DepthValues;
        DepthValues.SetNumUninitialized(Pixels.Num());
        for (int32 Index = 0; Index < Pixels.Num(); ++Index)
        {
            DepthValues[Index] = Pixels[Index].R;
        }
        return ConvertDepthValues(
            DepthValues,
            Width,
            Height,
            DepthScale,
            OutPayload,
            OutError);
    }

    bool ConvertDepthValues(
        const TArray<float>& DepthValues,
        const int32 Width,
        const int32 Height,
        const double DepthScale,
        TArray<uint8>& OutPayload,
        FString& OutError)
    {
        OutPayload.Reset();
        OutError.Reset();
        if (Width <= 0 || Height <= 0 || DepthValues.Num() != Width * Height)
        {
            OutError = TEXT("depth readback dimensions do not match the plan");
            return false;
        }
        if (!FMath::IsFinite(DepthScale) || DepthScale <= 0.0)
        {
            OutError = TEXT("depth_scale must be finite and positive");
            return false;
        }
        OutPayload.SetNumUninitialized(DepthValues.Num() * sizeof(uint16));
        uint16* Destination = reinterpret_cast<uint16*>(OutPayload.GetData());
        for (const float DepthValue : DepthValues)
        {
            const double DepthMeters = static_cast<double>(DepthValue);
            if (!FMath::IsFinite(DepthMeters) || DepthMeters < 0.0)
            {
                OutError = TEXT("depth readback contains a non-finite or negative value");
                OutPayload.Reset();
                return false;
            }
            const double Millimeters = DepthMeters / DepthScale;
            *Destination++ = Millimeters >= 65535.0
                ? 0
                : static_cast<uint16>(FMath::RoundToInt(Millimeters));
        }
        return true;
    }

    namespace
    {
        bool SharedCaptureGeometryMatches(
            const FStreamPlan& Color,
            const FStreamPlan& Depth)
        {
            if (Color.Kind != EStreamKind::Color || Depth.Kind != EStreamKind::Depth
                || Color.SessionId != Depth.SessionId
                || Color.ParentFrameId != Depth.ParentFrameId
                || Color.Source != TEXT("unreal_camera")
                || Depth.Source != TEXT("unreal_camera")
                || Color.Transport != TEXT("camera_shm")
                || Depth.Transport != TEXT("camera_shm")
                || (Color.Encoding != TEXT("rgb8") && Color.Encoding != TEXT("rgba8"))
                || Depth.Encoding != TEXT("16UC1")
                || Color.Width != Depth.Width
                || Color.Height != Depth.Height
                || !FMath::IsNearlyEqual(Color.RateHz, Depth.RateHz, 1e-9)
                || Color.bHasExtrinsic != Depth.bHasExtrinsic)
            {
                return false;
            }
            if (!Color.bHasExtrinsic)
            {
                return true;
            }
            const FQuat ColorRotation = Color.ExtrinsicRotation.GetNormalized();
            const FQuat DepthRotation = Depth.ExtrinsicRotation.GetNormalized();
            return Color.ExtrinsicTranslationMeters.Equals(
                       Depth.ExtrinsicTranslationMeters, 1e-6)
                && FMath::Abs(ColorRotation | DepthRotation) >= 1.0 - 1e-6;
        }
    }

    TArray<FSharedColorDepthPair> ResolveSharedColorDepthCapturePairs(
        const TArray<FStreamPlan>& Streams)
    {
        TArray<FSharedColorDepthPair> Result;
        TArray<TArray<int32>> DepthCandidatesByColor;
        DepthCandidatesByColor.SetNum(Streams.Num());
        TArray<int32> ColorCandidateCountByDepth;
        ColorCandidateCountByDepth.Init(0, Streams.Num());

        for (int32 ColorIndex = 0; ColorIndex < Streams.Num(); ++ColorIndex)
        {
            if (Streams[ColorIndex].Kind != EStreamKind::Color)
            {
                continue;
            }
            for (int32 DepthIndex = 0; DepthIndex < Streams.Num(); ++DepthIndex)
            {
                if (!SharedCaptureGeometryMatches(
                        Streams[ColorIndex], Streams[DepthIndex]))
                {
                    continue;
                }
                DepthCandidatesByColor[ColorIndex].Add(DepthIndex);
                ++ColorCandidateCountByDepth[DepthIndex];
            }
        }

        for (int32 ColorIndex = 0; ColorIndex < Streams.Num(); ++ColorIndex)
        {
            const TArray<int32>& Candidates = DepthCandidatesByColor[ColorIndex];
            if (Candidates.Num() != 1)
            {
                continue;
            }
            const int32 DepthIndex = Candidates[0];
            if (ColorCandidateCountByDepth[DepthIndex] != 1)
            {
                continue;
            }
            Result.Add(FSharedColorDepthPair{ColorIndex, DepthIndex});
        }
        return Result;
    }

    bool DecodeSharedColorDepthPixels(
        const TArray<FLinearColor>& Pixels,
        const int32 Width,
        const int32 Height,
        TArray<FColor>& OutColorPixels,
        TArray<float>& OutDepthMeters,
        FString& OutError)
    {
        OutColorPixels.Reset();
        OutDepthMeters.Reset();
        OutError.Reset();
        if (Width <= 0 || Height <= 0 || Pixels.Num() != Width * Height)
        {
            OutError = TEXT("shared RGB/depth readback dimensions do not match the plan");
            return false;
        }
        OutColorPixels.Reserve(Pixels.Num());
        OutDepthMeters.Reserve(Pixels.Num());
        for (const FLinearColor& Pixel : Pixels)
        {
            if (!FMath::IsFinite(Pixel.R)
                || !FMath::IsFinite(Pixel.G)
                || !FMath::IsFinite(Pixel.B)
                || !FMath::IsFinite(Pixel.A)
                || Pixel.A < 0.0f)
            {
                OutColorPixels.Reset();
                OutDepthMeters.Reset();
                OutError = TEXT("shared RGB/depth readback contains invalid color or depth");
                return false;
            }
            OutColorPixels.Add(
                FLinearColor(Pixel.R, Pixel.G, Pixel.B, 1.0f)
                    .GetClamped()
                    .ToFColorSRGB());
            // RobotSimUE uses UE's default centimetre world scale. The shared
            // alpha channel is linear SceneDepth in those world units.
            OutDepthMeters.Add(Pixel.A * 0.01f);
        }
        return true;
    }

    bool EncodeSharedColorDepthHalfPixels(
        const TArray<FFloat16Color>& Pixels,
        const int32 Width,
        const int32 Height,
        const FString& ColorEncoding,
        const double DepthScale,
        TArray<uint8>& OutColorPayload,
        TArray<uint8>& OutDepthPayload,
        FString& OutError)
    {
        OutColorPayload.Reset();
        OutDepthPayload.Reset();
        OutError.Reset();
        if (Width <= 0 || Height <= 0 || Pixels.Num() != Width * Height)
        {
            OutError = TEXT("shared RGBA16F readback dimensions do not match the plan");
            return false;
        }
        const int32 Channels = ColorEncoding == TEXT("rgb8")
            ? 3
            : ColorEncoding == TEXT("rgba8") ? 4 : 0;
        if (Channels == 0)
        {
            OutError = TEXT("shared RGB output encoding must be rgb8 or rgba8");
            return false;
        }
        if (!FMath::IsFinite(DepthScale) || DepthScale <= 0.0)
        {
            OutError = TEXT("shared depth_scale must be finite and positive");
            return false;
        }

        OutColorPayload.SetNumUninitialized(Pixels.Num() * Channels);
        OutDepthPayload.SetNumUninitialized(Pixels.Num() * sizeof(uint16));
        uint8* ColorDestination = OutColorPayload.GetData();
        uint16* DepthDestination = reinterpret_cast<uint16*>(OutDepthPayload.GetData());
        for (const FFloat16Color& HalfPixel : Pixels)
        {
            const FLinearColor Pixel(HalfPixel);
            if (!FMath::IsFinite(Pixel.R)
                || !FMath::IsFinite(Pixel.G)
                || !FMath::IsFinite(Pixel.B)
                || !FMath::IsFinite(Pixel.A)
                || Pixel.A < 0.0f)
            {
                OutColorPayload.Reset();
                OutDepthPayload.Reset();
                OutError = TEXT("shared RGBA16F readback contains invalid color or depth");
                return false;
            }

            const FColor Color = FLinearColor(Pixel.R, Pixel.G, Pixel.B, 1.0f)
                .GetClamped()
                .ToFColorSRGB();
            *ColorDestination++ = Color.R;
            *ColorDestination++ = Color.G;
            *ColorDestination++ = Color.B;
            if (Channels == 4)
            {
                *ColorDestination++ = Color.A;
            }

            const double DepthMeters = static_cast<double>(Pixel.A) * 0.01;
            const double DepthUnits = DepthMeters / DepthScale;
            *DepthDestination++ = DepthUnits >= 65535.0
                ? 0
                : static_cast<uint16>(FMath::RoundToInt(DepthUnits));
        }
        return true;
    }

    bool AdvanceSchedule(
        const float DeltaSeconds,
        const double RateHz,
        double& InOutAccumulatorSeconds)
    {
        if (!FMath::IsFinite(DeltaSeconds) || DeltaSeconds < 0.0
            || !FMath::IsFinite(RateHz) || RateHz <= 0.0)
        {
            return false;
        }
        InOutAccumulatorSeconds += static_cast<double>(DeltaSeconds);
        const double Period = 1.0 / RateHz;
        if (InOutAccumulatorSeconds + 1e-9 < Period)
        {
            return false;
        }
        InOutAccumulatorSeconds = FMath::Fmod(InOutAccumulatorSeconds, Period);
        return true;
    }

    FReadinessEvidenceCadence::FReadinessEvidenceCadence(
        const double InHeartbeatIntervalSeconds,
        const double InRetryIntervalSeconds)
        : HeartbeatIntervalSeconds(FMath::Max(InHeartbeatIntervalSeconds, UE_SMALL_NUMBER))
        , RetryIntervalSeconds(FMath::Max(InRetryIntervalSeconds, UE_SMALL_NUMBER))
    {
    }

    void FReadinessEvidenceCadence::MarkDirty()
    {
        if (!bDirty)
        {
            NextAttemptSeconds = 0.0;
        }
        bDirty = true;
    }

    bool FReadinessEvidenceCadence::ShouldAttempt(const double NowSeconds) const
    {
        if (!FMath::IsFinite(NowSeconds))
        {
            return false;
        }
        if (bDirty)
        {
            return NowSeconds >= NextAttemptSeconds;
        }
        return LastSuccessfulWriteSeconds < 0.0
            || NowSeconds - LastSuccessfulWriteSeconds >= HeartbeatIntervalSeconds;
    }

    void FReadinessEvidenceCadence::RecordAttempt(
        const double NowSeconds,
        const bool bSucceeded)
    {
        if (!FMath::IsFinite(NowSeconds))
        {
            return;
        }
        if (bSucceeded)
        {
            bDirty = false;
            LastSuccessfulWriteSeconds = NowSeconds;
            NextAttemptSeconds = NowSeconds + HeartbeatIntervalSeconds;
            return;
        }
        bDirty = true;
        NextAttemptSeconds = NowSeconds + RetryIntervalSeconds;
    }

    void FReadinessEvidenceCadence::Reset()
    {
        LastSuccessfulWriteSeconds = -1.0;
        NextAttemptSeconds = 0.0;
        bDirty = true;
    }

    bool ResolveAppliedTruthSample(
        const LingTuSim::FSnapshotEnvelope& Snapshot,
        const FString& ExpectedSessionId,
        const uint64 ExpectedModelGeneration,
        const uint64 ExpectedResetGeneration,
        FTruthSampleStamp& OutStamp,
        FString& OutError)
    {
        OutStamp = FTruthSampleStamp{};
        OutError.Reset();
        if (ExpectedSessionId.IsEmpty())
        {
            OutError = TEXT("camera truth binding has an empty session_id");
            return false;
        }
        if (Snapshot.SessionId != ExpectedSessionId)
        {
            OutError = TEXT("latest applied Visual snapshot session_id mismatch");
            return false;
        }
        if (Snapshot.ModelGeneration != ExpectedModelGeneration)
        {
            OutError = FString::Printf(
                TEXT("latest applied Visual snapshot model_generation mismatch: expected=%llu actual=%llu"),
                static_cast<unsigned long long>(ExpectedModelGeneration),
                static_cast<unsigned long long>(Snapshot.ModelGeneration));
            return false;
        }
        if (Snapshot.ResetGeneration != ExpectedResetGeneration)
        {
            OutError = FString::Printf(
                TEXT("latest applied Visual snapshot reset_generation mismatch: expected=%llu actual=%llu"),
                static_cast<unsigned long long>(ExpectedResetGeneration),
                static_cast<unsigned long long>(Snapshot.ResetGeneration));
            return false;
        }
        if (Snapshot.SimTimeNs < 0)
        {
            OutError = TEXT("latest applied Visual snapshot sim_time_ns must be non-negative");
            return false;
        }
        OutStamp.SessionId = Snapshot.SessionId;
        OutStamp.ModelGeneration = Snapshot.ModelGeneration;
        OutStamp.ResetGeneration = Snapshot.ResetGeneration;
        OutStamp.TruthSequence = Snapshot.Sequence;
        OutStamp.SimTimeNs = Snapshot.SimTimeNs;
        return true;
    }

    bool ResolveMountTransform(
        const FStreamPlan& Stream,
        const bool bPreviewOverride,
        const FVector& PreviewLocationCm,
        const FQuat& PreviewRotation,
        FVector& OutRelativeLocationCm,
        FQuat& OutRelativeRotation,
        bool& OutUsedPlanExtrinsic,
        FString& OutError)
    {
        OutError.Reset();
        OutUsedPlanExtrinsic = false;
        if (bPreviewOverride)
        {
            if (!PreviewRotation.IsNormalized() || PreviewLocationCm.ContainsNaN())
            {
                OutError = TEXT("preview camera transform is invalid");
                return false;
            }
            OutRelativeLocationCm = PreviewLocationCm;
            OutRelativeRotation = PreviewRotation;
            return true;
        }
        if (!Stream.bHasExtrinsic)
        {
            OutError = FString::Printf(
                TEXT("camera stream '%s' has no SensorRig extrinsic or default pose"),
                *Stream.SensorId);
            return false;
        }
        // Sensor-plan parent frames use RH Z-up metres. Visual bindings use
        // the same canonical handedness conversion as the rest of RobotSimUE.
        OutRelativeLocationCm = FVector(
            100.0 * Stream.ExtrinsicTranslationMeters.X,
            -100.0 * Stream.ExtrinsicTranslationMeters.Y,
            100.0 * Stream.ExtrinsicTranslationMeters.Z);
        const FQuat ConvertedOpticalRotation(
            -Stream.ExtrinsicRotation.X,
            Stream.ExtrinsicRotation.Y,
            -Stream.ExtrinsicRotation.Z,
            Stream.ExtrinsicRotation.W);

        // ROS optical is +X right, +Y down, +Z forward; SceneCapture is
        // +X forward, +Y right, +Z up. This local basis maps capture axes
        // (+X,+Y,+Z) onto optical axes (+Z,+X,-Y) after RH-to-LH conversion.
        const FQuat OpticalToSceneCaptureBasis(-0.5, -0.5, -0.5, 0.5);
        OutRelativeRotation = ConvertedOpticalRotation * OpticalToSceneCaptureBasis;
        if (OutRelativeLocationCm.ContainsNaN()
            || !OutRelativeRotation.IsNormalized())
        {
            OutError = FString::Printf(
                TEXT("camera stream '%s' compiled extrinsic is invalid"), *Stream.SensorId);
            return false;
        }
        OutUsedPlanExtrinsic = true;
        return true;
    }

    FString BuildReadinessEvidenceJson(
        const FString& InSessionId,
        const uint64 InModelGeneration,
        const uint64 InResetGeneration,
        const TArray<FStreamReadiness>& Streams)
    {
        const bool bHasCameraStreams = Streams.Num() > 0;
        bool bAllCameraStreamsActive = bHasCameraStreams;
        bool bAnyCameraStreamFailed = false;
        FString StreamEvidence;
        for (int32 Index = 0; Index < Streams.Num(); ++Index)
        {
            const FStreamReadiness& Stream = Streams[Index];
            bAllCameraStreamsActive &= Stream.State == TEXT("ACTIVE");
            bAnyCameraStreamFailed |= Stream.State == TEXT("FAILED");
            if (Index > 0)
            {
                StreamEvidence += TEXT(",");
            }
            StreamEvidence += FString::Printf(
                TEXT("{\"sensor_id\":\"%s\",\"state\":\"%s\",\"published_frames\":%llu,"
                     "\"last_sample_truth_sequence\":%llu,\"last_sample_sim_time_ns\":%lld"),
                *EscapeCameraReadinessJsonString(Stream.SensorId),
                *EscapeCameraReadinessJsonString(Stream.State),
                static_cast<unsigned long long>(Stream.PublishedFrames),
                static_cast<unsigned long long>(Stream.LastSampleTruthSequence),
                static_cast<long long>(Stream.LastSampleSimTimeNs));
            if (Stream.State == TEXT("FAILED") && !Stream.Reason.IsEmpty())
            {
                StreamEvidence += FString::Printf(
                    TEXT(",\"reason\":\"%s\""),
                    *EscapeCameraReadinessJsonString(Stream.Reason));
            }
            StreamEvidence += TEXT("}");
        }
        const TCHAR* CameraState = bAnyCameraStreamFailed
            ? TEXT("FAILED")
            : (bAllCameraStreamsActive ? TEXT("ACTIVE") : TEXT("PREPARING"));
        return FString::Printf(
            TEXT("{\n"
                 "  \"schema\": \"lingtu.sim.sensor-readiness-evidence.v1\",\n"
                 "  \"session_id\": \"%s\",\n"
                 "  \"model_generation\": %llu,\n"
                 "  \"reset_generation\": %llu,\n"
                 "  \"source_id\": \"robotsimue-camera\",\n"
                 "  \"basis\": \"real_rendered_frame_to_camera_shm\",\n"
                 "  \"visual\": {\"state\": \"PREPARED\"},\n"
                 "  \"sensors\": {\"camera_streams\": \"%s\", \"overall\": \"%s\"},\n"
                 "  \"streams\": [%s]\n"
                 "}\n"),
            *EscapeCameraReadinessJsonString(InSessionId),
            static_cast<unsigned long long>(InModelGeneration),
            static_cast<unsigned long long>(InResetGeneration),
            CameraState,
            CameraState,
            *StreamEvidence);
    }
}

enum class ELingTuCameraReadbackStage : uint8
{
    EnqueuePending,
    GpuPending,
    CpuDecodePending,
    CpuReady,
    Failed,
    Cancelled,
};

struct ULingTuSimCameraCaptureSubsystem::FAsyncReadbackJob
{
    TUniquePtr<FRHIGPUTextureReadback> Readback;
    LingTuSim::Sensors::Camera::FTruthSampleStamp TruthSample;
    TArray<FColor> ColorPixels;
    TArray<float> DepthValues;
    TArray<uint8> SharedColorPayload;
    TArray<uint8> SharedDepthPayload;
    FString FailureReason;
    double QueuedAtSeconds = 0.0;
    double PipelineTimeoutSeconds = 0.0;
    TAtomic<double> GpuSubmittedAtSeconds{-1.0};
    TAtomic<double> CpuDecodeStartedAtSeconds{-1.0};
    TAtomic<ELingTuCameraReadbackStage> Stage{ELingTuCameraReadbackStage::EnqueuePending};
    TAtomic<bool> bPollCommandQueued{false};
    bool bHasSharedPayloads = false;
    bool bDiscard = false;
};

struct ULingTuSimCameraCaptureSubsystem::FDeferredCaptureJob
{
    LingTuSim::Sensors::Camera::FTruthSampleStamp TruthSample;
    uint64 SubmittedFrame = 0;
    double SubmittedAtSeconds = 0.0;
    double PipelineTimeoutSeconds = 0.0;
};

struct ULingTuSimCameraCaptureSubsystem::FCaptureState
{
    LingTuSim::Sensors::Camera::FStreamPlan Plan;
    ECaptureLayout Layout = ECaptureLayout::Separate;
    TUniquePtr<FFrameWriter> Writer;
    TObjectPtr<AActor> Owner = nullptr;
    TObjectPtr<USceneCaptureComponent2D> Capture = nullptr;
    TObjectPtr<UTextureRenderTarget2D> Target = nullptr;
    double AccumulatorSeconds = 1.0;
    uint64 PublishedFrames = 0;
    LingTuSim::Sensors::Camera::FTruthSampleStamp LastPublishedTruth;
    LingTuSim::Sensors::Camera::FTruthSampleStamp LastSubmittedTruth;
    TOptional<FDeferredCaptureJob> DeferredCapture;
    TArray<TSharedPtr<FAsyncReadbackJob, ESPMode::ThreadSafe>> PendingReadbacks;
    FCaptureState* SharedColorLeader = nullptr;
    FCaptureState* SharedDepthFollower = nullptr;
    FString FailureReason;
    bool bActive = false;
};

void ULingTuSimCameraCaptureSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
    Super::Initialize(Collection);
    FString BundleDirectory;
    bCommandLineBindingRequested = FParse::Value(
        FCommandLine::Get(), TEXT("LingTuBundle="), BundleDirectory);
    bDepthCaptureInMainRenderer = FParse::Param(
        FCommandLine::Get(), TEXT("LingTuDepthCaptureInMainRenderer"));
    bSharedColorDepthCapture = FParse::Param(
        FCommandLine::Get(), TEXT("LingTuSharedColorDepthCapture"));
}

void ULingTuSimCameraCaptureSubsystem::OnWorldBeginPlay(UWorld& InWorld)
{
    Super::OnWorldBeginPlay(InWorld);
    bWorldBindingLifecycleActive =
        InWorld.WorldType == EWorldType::Game
        || InWorld.WorldType == EWorldType::PIE
        || InWorld.WorldType == EWorldType::GamePreview;
    VisualBindingWaitStartSeconds = -1.0;
}

void ULingTuSimCameraCaptureSubsystem::OnWorldEndPlay(UWorld& InWorld)
{
    bWorldBindingLifecycleActive = false;
    VisualBindingWaitStartSeconds = -1.0;
    Super::OnWorldEndPlay(InWorld);
}

void ULingTuSimCameraCaptureSubsystem::PreDeinitialize()
{
    bWorldBindingLifecycleActive = false;
    VisualBindingWaitStartSeconds = -1.0;
    Super::PreDeinitialize();
}

ULingTuSimCameraCaptureSubsystem::~ULingTuSimCameraCaptureSubsystem()
{
    ReleaseCaptures();
}

void ULingTuSimCameraCaptureSubsystem::Deinitialize()
{
    bWorldBindingLifecycleActive = false;
    VisualBindingWaitStartSeconds = -1.0;
    ReleaseCaptures();
    Super::Deinitialize();
}

void ULingTuSimCameraCaptureSubsystem::Tick(const float DeltaTime)
{
    UWorld* World = GetWorld();
    if (World == nullptr
        || !bWorldBindingLifecycleActive
        || !World->IsInitialized()
        || World->bIsTearingDown
        || !World->HasBegunPlay())
    {
        return;
    }
    if (bCommandLineBindingRequested && !bBound)
    {
        const LingTuSim::Sensors::Camera::EWorldBindingAction Action =
            LingTuSim::Sensors::Camera::EvaluateWorldBindingAction(
                World->WorldType,
                World->HasBegunPlay(),
                World->IsInitialized(),
                World->bIsTearingDown,
                [World]()
                {
                    const ULingTuSimVisualWorldSubsystem* VisualSubsystem =
                        World->GetSubsystem<ULingTuSimVisualWorldSubsystem>();
                    return VisualSubsystem != nullptr
                        ? VisualSubsystem->GetRegisteredBindingCount()
                        : 0;
                });
        if (Action == LingTuSim::Sensors::Camera::EWorldBindingAction::Skip)
        {
            bCommandLineBindingRequested = false;
            VisualBindingWaitStartSeconds = -1.0;
            return;
        }
        if (Action == LingTuSim::Sensors::Camera::EWorldBindingAction::Wait)
        {
            if (World->HasBegunPlay())
            {
                const double NowSeconds = FPlatformTime::Seconds();
                if (VisualBindingWaitStartSeconds < 0.0)
                {
                    VisualBindingWaitStartSeconds = NowSeconds;
                }
                const double ElapsedSeconds = NowSeconds - VisualBindingWaitStartSeconds;
                if (LingTuSim::Sensors::Camera::HasWorldBindingWaitExpired(
                        true,
                        ElapsedSeconds,
                        CameraVisualBindingTimeoutSeconds))
                {
                    bCommandLineBindingRequested = false;
                    FailClosed(FString::Printf(
                        TEXT("timed out after %.1f seconds waiting for Visual Runtime body bindings in world '%s'"),
                        CameraVisualBindingTimeoutSeconds,
                        *World->GetName()));
                }
            }
            return;
        }

        bCommandLineBindingRequested = false;
        VisualBindingWaitStartSeconds = -1.0;
        FString Error;
        if (!BindFromCommandLine(Error))
        {
            FailClosed(Error);
            return;
        }
    }
    if (!bBound)
    {
        return;
    }
    const double ReadinessNowSeconds = FPlatformTime::Seconds();
    TArray<FCaptureState*> DueCaptures;
    DueCaptures.Reserve(Captures.Num());
    bool bHasCapturePipelineWork = false;
    for (FCaptureState* State : Captures)
    {
        if (State == nullptr)
        {
            continue;
        }
        bHasCapturePipelineWork |= State->DeferredCapture.IsSet()
            || !State->PendingReadbacks.IsEmpty();
        if (State->Layout == ECaptureLayout::SharedDepthFollower)
        {
            // The color leader owns the one render schedule and readback ring.
            continue;
        }
        if (LingTuSim::Sensors::Camera::AdvanceSchedule(
                DeltaTime, State->Plan.RateHz, State->AccumulatorSeconds))
        {
            DueCaptures.Add(State);
        }
    }
    if (DueCaptures.IsEmpty() && !bHasCapturePipelineWork)
    {
        FlushReadinessEvidenceIfDue(ReadinessNowSeconds);
        return;
    }

    // Capture all streams due in this game-thread boundary against one copied
    // frame which Visual has already applied completely to this same world.
    const ULingTuSimVisualWorldSubsystem* VisualSubsystem =
        World->GetSubsystem<ULingTuSimVisualWorldSubsystem>();
    LingTuSim::FSnapshotEnvelope AppliedSnapshot;
    if (VisualSubsystem == nullptr
        || !VisualSubsystem->GetLatestAppliedSnapshot(AppliedSnapshot))
    {
        // Initial bind and reset legitimately have a window with no complete
        // Visual frame. Retract any old ACTIVE projection without advancing or
        // clearing its last proven sample; the next reset frame performs the
        // generation-stamped clear below.
        bool bReadinessChanged = bHasActiveFrame;
        bHasActiveFrame = false;
        for (FCaptureState* State : Captures)
        {
            if (State != nullptr)
            {
                DiscardDeferredCapture(*State);
                DiscardPendingReadbacks(*State);
                bool bPumpChanged = false;
                FString ReadbackError;
                if (!PumpCaptureReadbacks(
                        *State,
                        false,
                        ReadinessNowSeconds,
                        bPumpChanged,
                        ReadbackError))
                {
                    FailClosed(FString::Printf(
                        TEXT("sensor_id=%s: %s"), *State->Plan.SensorId, *ReadbackError));
                    return;
                }
                bReadinessChanged |= State->bActive;
                State->bActive = false;
                State->LastSubmittedTruth = State->LastPublishedTruth;
            }
        }
        if (bReadinessChanged)
        {
            ReadinessEvidenceCadence.MarkDirty();
            FString EvidenceError;
            const bool bSucceeded = WriteReadinessEvidence(EvidenceError);
            ReadinessEvidenceCadence.RecordAttempt(ReadinessNowSeconds, bSucceeded);
            if (!bSucceeded)
            {
                UE_LOG(
                    LogLingTuSimCamera,
                    Warning,
                    TEXT("LINGTU_CAMERA_READINESS_WRITE_RETRY reason=%s"),
                    *EvidenceError);
            }
        }
        FlushReadinessEvidenceIfDue(ReadinessNowSeconds);
        return;
    }
    if (AppliedSnapshot.SessionId == SessionId
        && AppliedSnapshot.ModelGeneration == ModelGeneration
        && AppliedSnapshot.ResetGeneration > ResetGeneration
        && AppliedSnapshot.ResetGeneration - ResetGeneration == 1)
    {
        // Reset is an ordered generation boundary, not a camera sample. Clear
        // every per-stream claim and publish PREPARING for the new generation;
        // the next scheduled attempt may capture against this applied frame.
        ResetGeneration = AppliedSnapshot.ResetGeneration;
        bHasActiveFrame = false;
        for (FCaptureState* State : Captures)
        {
            if (State != nullptr)
            {
                DiscardDeferredCapture(*State);
                DiscardPendingReadbacks(*State);
                bool bPumpChanged = false;
                FString ReadbackError;
                if (!PumpCaptureReadbacks(
                        *State,
                        false,
                        ReadinessNowSeconds,
                        bPumpChanged,
                        ReadbackError))
                {
                    FailClosed(FString::Printf(
                        TEXT("sensor_id=%s: %s"), *State->Plan.SensorId, *ReadbackError));
                    return;
                }
                State->AccumulatorSeconds = 1.0;
                State->PublishedFrames = 0;
                State->LastPublishedTruth =
                    LingTuSim::Sensors::Camera::FTruthSampleStamp{};
                State->LastSubmittedTruth =
                    LingTuSim::Sensors::Camera::FTruthSampleStamp{};
                State->FailureReason.Reset();
                State->bActive = false;
            }
        }
        ReadinessEvidenceCadence.MarkDirty();
        FString EvidenceError;
        const bool bSucceeded = WriteReadinessEvidence(EvidenceError);
        ReadinessEvidenceCadence.RecordAttempt(ReadinessNowSeconds, bSucceeded);
        if (!bSucceeded)
        {
            UE_LOG(
                LogLingTuSimCamera,
                Warning,
                TEXT("LINGTU_CAMERA_READINESS_WRITE_RETRY reason=%s"),
                *EvidenceError);
        }
        FlushReadinessEvidenceIfDue(ReadinessNowSeconds);
        UE_LOG(
            LogLingTuSimCamera,
            Display,
            TEXT("LINGTU_CAMERA_RESET_PREPARING session_id=%s model_generation=%llu reset_generation=%llu"),
            *SessionId,
            static_cast<unsigned long long>(ModelGeneration),
            static_cast<unsigned long long>(ResetGeneration));
        return;
    }
    LingTuSim::Sensors::Camera::FTruthSampleStamp TruthSample;
    FString TruthError;
    if (!LingTuSim::Sensors::Camera::ResolveAppliedTruthSample(
            AppliedSnapshot,
            SessionId,
            ModelGeneration,
            ResetGeneration,
            TruthSample,
            TruthError))
    {
        FailClosed(TruthError);
        return;
    }

    bool bReadinessChanged = false;
    const uint64 CurrentFrame = GFrameCounter;
    // Promote every capture rendered at the preceding end-of-frame before any
    // new deferred request below. Render-command FIFO order is therefore
    // capture(N), readback-copy(N), capture(N+1), so one target cannot be
    // overwritten before its exact truth-stamped pixels enter the staging ring.
    for (FCaptureState* State : Captures)
    {
        if (State == nullptr)
        {
            continue;
        }
        FString Error;
        if (!PromoteDeferredCaptureToReadback(
                *State,
                CurrentFrame,
                ReadinessNowSeconds,
                Error))
        {
            FailClosed(FString::Printf(TEXT("sensor_id=%s: %s"), *State->Plan.SensorId, *Error));
            return;
        }
    }
    for (FCaptureState* State : Captures)
    {
        if (State == nullptr)
        {
            continue;
        }
        FString Error;
        if (!PumpCaptureReadbacks(
                *State,
                true,
                ReadinessNowSeconds,
                bReadinessChanged,
                Error))
        {
            FailClosed(FString::Printf(TEXT("sensor_id=%s: %s"), *State->Plan.SensorId, *Error));
            return;
        }
    }

    bool bBatchTruthEligible = !DueCaptures.IsEmpty();
    for (FCaptureState* State : DueCaptures)
    {
        const LingTuSim::Sensors::Camera::FTruthSampleStamp& LastSubmitted =
            State->LastSubmittedTruth.SessionId.IsEmpty()
                ? State->LastPublishedTruth
                : State->LastSubmittedTruth;
        if (!LastSubmitted.SessionId.IsEmpty())
        {
            if (TruthSample.TruthSequence == LastSubmitted.TruthSequence
                && TruthSample.SimTimeNs == LastSubmitted.SimTimeNs)
            {
                // Camera cadence can be faster than applied Visual truth. Do
                // not manufacture a second sample stamp for the same frame.
                bBatchTruthEligible = false;
                break;
            }
            if (TruthSample.TruthSequence <= LastSubmitted.TruthSequence
                || TruthSample.SimTimeNs <= LastSubmitted.SimTimeNs)
            {
                FailClosed(TEXT("latest applied Visual snapshot did not advance monotonically"));
                return;
            }
        }
    }
    if (bBatchTruthEligible)
    {
        bool bCanQueueBatch = false;
        FString Error;
        if (!CanQueueDeferredCaptureBatch(DueCaptures, bCanQueueBatch, Error))
        {
            FailClosed(Error);
            return;
        }
        if (bCanQueueBatch)
        {
            for (FCaptureState* State : DueCaptures)
            {
                if (!QueueDeferredCapture(
                        *State,
                        TruthSample,
                        CurrentFrame,
                        ReadinessNowSeconds,
                        Error))
                {
                    FailClosed(FString::Printf(
                        TEXT("sensor_id=%s: %s"), *State->Plan.SensorId, *Error));
                    return;
                }
            }
        }
    }
    if (bReadinessChanged)
    {
        ReadinessEvidenceCadence.MarkDirty();
    }
    FlushReadinessEvidenceIfDue(ReadinessNowSeconds);
}

TStatId ULingTuSimCameraCaptureSubsystem::GetStatId() const
{
    RETURN_QUICK_DECLARE_CYCLE_STAT(ULingTuSimCameraCaptureSubsystem, STATGROUP_Tickables);
}

bool ULingTuSimCameraCaptureSubsystem::IsTickable() const
{
    if (HasAnyFlags(RF_ClassDefaultObject) || !bWorldBindingLifecycleActive)
    {
        return false;
    }
    const UWorld* World = GetWorld();
    return World != nullptr
        && World->IsInitialized()
        && !World->bIsTearingDown
        && World->HasBegunPlay();
}

bool ULingTuSimCameraCaptureSubsystem::BindPlan(
    const LingTuSim::Sensors::Camera::FCameraPlan& Plan,
    const LingTuSim::Sensors::Camera::FRunAllocation& Allocation,
    const uint64 InModelGeneration,
    const uint64 InResetGeneration,
    FString& OutError)
{
    OutError.Reset();
    ReleaseCaptures();
    bBound = false;
    bHasActiveFrame = false;
    LastFailure.Reset();
    if (!GetWorld() || Plan.Streams.Num() == 0 || Plan.SessionId.IsEmpty()
        || Allocation.SessionId != Plan.SessionId)
    {
        OutError = TEXT("camera plan/allocation/world binding is invalid");
        return false;
    }

    TArray<FCaptureState*> Prepared;
    TArray<ECaptureLayout> Layouts;
    Layouts.Init(ECaptureLayout::Separate, Plan.Streams.Num());
    TArray<LingTuSim::Sensors::Camera::FSharedColorDepthPair> SharedPairs;
    if (bSharedColorDepthCapture)
    {
        SharedPairs = LingTuSim::Sensors::Camera::ResolveSharedColorDepthCapturePairs(
            Plan.Streams);
        for (const LingTuSim::Sensors::Camera::FSharedColorDepthPair& Pair : SharedPairs)
        {
            Layouts[Pair.ColorStreamIndex] = ECaptureLayout::SharedColorLeader;
            Layouts[Pair.DepthStreamIndex] = ECaptureLayout::SharedDepthFollower;
        }
    }
    const auto ReleasePreparedCaptures = [this, &Prepared]()
    {
        for (FCaptureState* PreparedState : Prepared)
        {
            ReleaseCaptureState(*PreparedState);
            delete PreparedState;
        }
        Prepared.Reset();
    };
    for (int32 StreamIndex = 0; StreamIndex < Plan.Streams.Num(); ++StreamIndex)
    {
        const LingTuSim::Sensors::Camera::FStreamPlan& Stream = Plan.Streams[StreamIndex];
        const FString* Mapping = Allocation.SharedMemoryBySensor.Find(Stream.SensorId);
        if (Mapping == nullptr || Mapping->IsEmpty())
        {
            OutError = FString::Printf(TEXT("no SHM allocation for camera stream '%s'"), *Stream.SensorId);
            ReleasePreparedCaptures();
            return false;
        }
        FCaptureState* State = nullptr;
        if (!CreateCaptureState(
                Stream,
                *Mapping,
                InModelGeneration,
                Layouts[StreamIndex],
                State,
                OutError))
        {
            ReleasePreparedCaptures();
            return false;
        }
        Prepared.Add(State);
    }
    for (const LingTuSim::Sensors::Camera::FSharedColorDepthPair& Pair : SharedPairs)
    {
        FCaptureState* Color = Prepared[Pair.ColorStreamIndex];
        FCaptureState* Depth = Prepared[Pair.DepthStreamIndex];
        Color->SharedDepthFollower = Depth;
        Depth->SharedColorLeader = Color;
    }
    SessionId = Plan.SessionId;
    EvidenceDirectory = Allocation.EvidenceDirectory;
    ModelGeneration = InModelGeneration;
    ResetGeneration = InResetGeneration;
    TotalPlanStreamCount = Plan.TotalStreamCount;
    Captures = MoveTemp(Prepared);
    ReadinessEvidenceCadence.Reset();
    bHasBoundOnce = true;
    bBound = true;
    if (!WriteReadinessEvidence(OutError))
    {
        ReleaseCaptures();
        bBound = false;
        return false;
    }
    ReadinessEvidenceCadence.RecordAttempt(FPlatformTime::Seconds(), true);
    UE_LOG(
        LogLingTuSimCamera,
        Display,
        TEXT("LINGTU_CAMERA_PREPARING session_id=%s model_generation=%llu streams=%d capture=deferred readback=async_gpu max_in_flight=%d startup_deadline_s=%.3f steady_deadline_s=%.3f depth_main_renderer=%d shared_color_depth=%d shared_pairs=%d"),
        *SessionId,
        static_cast<unsigned long long>(ModelGeneration),
        Captures.Num(),
        MaxPendingReadbacksPerStream,
        CameraFirstFramePipelineTimeoutSeconds,
        CameraCapturePipelineTimeoutSeconds,
        bDepthCaptureInMainRenderer ? 1 : 0,
        bSharedColorDepthCapture ? 1 : 0,
        SharedPairs.Num());
    return true;
}

bool ULingTuSimCameraCaptureSubsystem::BindFromCommandLine(FString& OutError)
{
    FString BundleDirectory;
    if (!FParse::Value(FCommandLine::Get(), TEXT("LingTuBundle="), BundleDirectory)
        || BundleDirectory.IsEmpty())
    {
        OutError = TEXT("-LingTuBundle is required");
        return false;
    }
    FString AllocationPath;
    if (!FParse::Value(FCommandLine::Get(), TEXT("LingTuRunAllocation="), AllocationPath)
        || AllocationPath.IsEmpty())
    {
        AllocationPath = FPaths::Combine(BundleDirectory, TEXT("run-allocation.json"));
    }
    const FString SensorPath = FPaths::Combine(BundleDirectory, TEXT("sensor.plan.json"));
    FString SensorJson;
    FString AllocationJson;
    if (!FFileHelper::LoadFileToString(SensorJson, *SensorPath)
        || !FFileHelper::LoadFileToString(AllocationJson, *AllocationPath))
    {
        OutError = FString::Printf(TEXT("camera plan or RunAllocation cannot be read: %s / %s"), *SensorPath, *AllocationPath);
        return false;
    }
    LingTuSim::FRuntimeLoadError LoadError;
    LingTuSim::FSessionBundleView Bundle;
    if (!LingTuSim::FSessionBundleLoader::LoadSessionBundle(BundleDirectory, Bundle, LoadError))
    {
        OutError = FString::Printf(TEXT("SessionBundle rejected: %s"), *LoadError.Message);
        return false;
    }
    LingTuSim::Sensors::Camera::FCameraPlan Plan;
    LingTuSim::Sensors::Camera::FRunAllocation Allocation;
    if (!LingTuSim::Sensors::Camera::ParseCameraPlanJson(SensorJson, Plan, OutError)
        || !LingTuSim::Sensors::Camera::ParseRunAllocationJson(AllocationJson, Allocation, OutError))
    {
        return false;
    }
    if (Bundle.SessionId != Plan.SessionId
        || Bundle.SessionId != Allocation.SessionId)
    {
        OutError = TEXT("camera plan, RunAllocation, and SessionBundle digest mismatch");
        return false;
    }
    if (Allocation.EvidenceDirectory.IsEmpty())
    {
        OutError = TEXT("RunAllocation.log_dir is required for owned readiness evidence");
        return false;
    }
    uint64 InModelGeneration = 0;
    uint64 InResetGeneration = 0;
    const bool bInitialBind = !bHasBoundOnce;
    if (!ParseGeneration(TEXT("LingTuModelGeneration="), bInitialBind, InModelGeneration, OutError)
        || !ParseGeneration(TEXT("LingTuResetGeneration="), bInitialBind, InResetGeneration, OutError))
    {
        return false;
    }
    return BindPlan(Plan, Allocation, InModelGeneration, InResetGeneration, OutError);
}

bool ULingTuSimCameraCaptureSubsystem::CreateCaptureState(
    const LingTuSim::Sensors::Camera::FStreamPlan& Stream,
    const FString& MappingName,
    const uint64 InModelGeneration,
    const ECaptureLayout Layout,
    FCaptureState*& OutState,
    FString& OutError)
{
    UWorld* World = GetWorld();
    if (World == nullptr)
    {
        OutError = TEXT("world is unavailable");
        return false;
    }

    // FParse::Param is sensitive to the exact command-line token shape on
    // packaged/editor invocations. This is an explicit preview-only escape
    // hatch, so recognize the complete switch token and never use it for a
    // production mount when a compiled extrinsic is present.
    const FString CommandLine = FCommandLine::Get();
    const bool bPreviewOverride = FParse::Param(
        *CommandLine, TEXT("LingTuCameraPreviewOverride"))
        || CommandLine.Contains(TEXT("-LingTuCameraPreviewOverride"), ESearchCase::IgnoreCase);
    FVector PreviewLocationCm(0.0, 0.0, 100.0);
    FVector PreviewRotationDegrees(-5.0, 0.0, 0.0);
    FString LocationText;
    FString RotationText;
    if (ReadCommandLineToken(TEXT("-LingTuCameraLocationCm="), LocationText))
    {
        if (!bPreviewOverride || !ParseCommaVector(LocationText, PreviewLocationCm))
        {
            OutError = TEXT("camera CLI transform requires -LingTuCameraPreviewOverride and three finite location values");
            return false;
        }
    }
    if (ReadCommandLineToken(TEXT("-LingTuCameraRotationDeg="), RotationText))
    {
        if (!bPreviewOverride || !ParseCommaVector(RotationText, PreviewRotationDegrees))
        {
            OutError = TEXT("camera CLI transform requires -LingTuCameraPreviewOverride and three finite rotation values");
            return false;
        }
    }
    const FQuat PreviewRotation = FRotator::MakeFromEuler(PreviewRotationDegrees).Quaternion();
    FVector RelativeLocationCm;
    FQuat RelativeRotation;
    bool bUsedPlanExtrinsic = false;
    if (!LingTuSim::Sensors::Camera::ResolveMountTransform(
            Stream,
            bPreviewOverride,
            PreviewLocationCm,
            PreviewRotation,
            RelativeLocationCm,
            RelativeRotation,
            bUsedPlanExtrinsic,
            OutError))
    {
        return false;
    }

    ULingTuSimBodyBindingComponent* ParentFrame = nullptr;
    if (bUsedPlanExtrinsic)
    {
        for (TObjectIterator<ULingTuSimBodyBindingComponent> It; It; ++It)
        {
            if (It->GetWorld() == World && It->StableId == Stream.ParentFrameId)
            {
                ParentFrame = *It;
                break;
            }
        }
        if (ParentFrame == nullptr)
        {
            OutError = FString::Printf(
                TEXT("compiled parent_frame '%s' for camera '%s' is not bound in RobotSimUE"),
                *Stream.ParentFrameId,
                *Stream.SensorId);
            return false;
        }
    }

    TUniquePtr<FCaptureState> State = MakeUnique<FCaptureState>();
    State->Plan = Stream;
    State->Layout = Layout;
    State->Writer = MakeUnique<FFrameWriter>();
    FWriterConfig Config;
    Config.MappingName = MappingName;
    Config.Generation = InModelGeneration;
    if (!State->Writer->TryOpen(Config, OutError))
    {
        return false;
    }
    if (Layout == ECaptureLayout::SharedDepthFollower)
    {
        // The follower owns only its SHM writer and readiness state. Its paired
        // color leader owns the one SceneCapture component and GPU readback.
        OutState = State.Release();
        return true;
    }

    State->Owner = World->SpawnActor<AActor>();
    if (State->Owner == nullptr)
    {
        OutError = TEXT("failed to spawn transient camera owner");
        return false;
    }
    struct FTransientOwnerCleanup final
    {
        AActor* Owner = nullptr;
        bool bReleased = false;

        ~FTransientOwnerCleanup()
        {
            if (!bReleased && IsValid(Owner) && !Owner->IsActorBeingDestroyed())
            {
                Owner->Destroy();
            }
        }

        void Release()
        {
            bReleased = true;
        }
    } OwnerCleanup{State->Owner};

    State->Owner->SetActorHiddenInGame(true);
    State->Capture = NewObject<USceneCaptureComponent2D>(State->Owner, NAME_None, RF_Transient);
    if (State->Capture == nullptr)
    {
        OutError = TEXT("failed to create USceneCaptureComponent2D");
        return false;
    }
    State->Owner->SetRootComponent(State->Capture);
    State->Capture->RegisterComponent();
    State->Capture->bCaptureEveryFrame = false;
    State->Capture->bCaptureOnMovement = false;
    State->Capture->ProjectionType = ECameraProjectionMode::Perspective;
    State->Capture->FOVAngle = 90.0f;
    State->Capture->CaptureSource = Layout == ECaptureLayout::SharedColorLeader
        ? ESceneCaptureSource::SCS_SceneColorSceneDepth
        : Stream.Kind == LingTuSim::Sensors::Camera::EStreamKind::Color
            ? ESceneCaptureSource::SCS_FinalColorLDR
            : ESceneCaptureSource::SCS_SceneDepth;
    LingTuSim::Sensors::Camera::ConfigureCameraCaptureShowFlags(
        Stream.Kind,
        State->Capture->ShowFlags);
    State->Capture->bRenderInMainRenderer =
        LingTuSim::Sensors::Camera::ShouldRenderCameraCaptureInMainRenderer(
            Stream.Kind,
            State->Capture->CaptureSource,
            bDepthCaptureInMainRenderer);
    if (bUsedPlanExtrinsic)
    {
        State->Capture->AttachToComponent(
            ParentFrame,
            FAttachmentTransformRules::KeepRelativeTransform);
        State->Capture->SetRelativeLocationAndRotation(RelativeLocationCm, RelativeRotation);
        UE_LOG(
            LogLingTuSimCamera,
            Display,
            TEXT("LINGTU_CAMERA_MOUNT_BOUND sensor_id=%s parent_frame=%s source=compiled_sensor_plan"),
            *Stream.SensorId,
            *Stream.ParentFrameId);
    }
    else
    {
        State->Owner->SetActorLocationAndRotation(RelativeLocationCm, RelativeRotation);
        UE_LOG(
            LogLingTuSimCamera,
            Warning,
            TEXT("LINGTU_CAMERA_PREVIEW_MOUNT sensor_id=%s source=explicit_preview_override"),
            *Stream.SensorId);
    }

    State->Target = NewObject<UTextureRenderTarget2D>(State->Owner, NAME_None, RF_Transient);
    if (State->Target == nullptr)
    {
        OutError = TEXT("failed to create UTextureRenderTarget2D");
        return false;
    }
    State->Target->ClearColor = FLinearColor::Black;
    State->Target->bAutoGenerateMips = false;
    const EPixelFormat TargetFormat = Layout == ECaptureLayout::SharedColorLeader
        ? PF_FloatRGBA
        : Stream.Kind == EStreamKind::Color ? PF_B8G8R8A8 : PF_R32_FLOAT;
    State->Target->InitCustomFormat(
        Stream.Width,
        Stream.Height,
        TargetFormat,
        true);
    State->Target->UpdateResourceImmediate(true);
    State->Capture->TextureTarget = State->Target;
    OwnerCleanup.Release();
    OutState = State.Release();
    return true;
}

bool ULingTuSimCameraCaptureSubsystem::CanQueueDeferredCaptureBatch(
    const TArray<FCaptureState*>& DueCaptures,
    bool& OutCanQueue,
    FString& OutError) const
{
    OutCanQueue = false;
    OutError.Reset();
    if (DueCaptures.IsEmpty())
    {
        return true;
    }
    for (const FCaptureState* State : DueCaptures)
    {
        if (State == nullptr
            || State->Capture == nullptr
            || State->Target == nullptr
            || !State->Writer.IsValid()
            || !State->Capture->IsRegistered()
            || !State->Capture->IsVisible())
        {
            OutError = TEXT("deferred capture batch contains an incomplete stream");
            return false;
        }
        if (State->Layout == ECaptureLayout::SharedColorLeader
            && (State->SharedDepthFollower == nullptr
                || !State->SharedDepthFollower->Writer.IsValid()))
        {
            OutError = TEXT("shared RGB/depth capture has no bound depth follower");
            return false;
        }
        if (State->DeferredCapture.IsSet()
            || State->PendingReadbacks.Num() >= MaxPendingReadbacksPerStream)
        {
            // RGB and depth are a single truth-stamped batch. If either stream
            // has no capacity, submit neither and preserve their sequence join.
            return true;
        }
    }
    OutCanQueue = true;
    return true;
}

bool ULingTuSimCameraCaptureSubsystem::QueueDeferredCapture(
    FCaptureState& State,
    const LingTuSim::Sensors::Camera::FTruthSampleStamp& TruthSample,
    const uint64 SubmittedFrame,
    const double SubmittedAtSeconds,
    FString& OutError)
{
    OutError.Reset();
    if (State.Capture == nullptr
        || State.Target == nullptr
        || !State.Writer.IsValid()
        || !State.Capture->IsRegistered()
        || !State.Capture->IsVisible())
    {
        OutError = TEXT("capture state is incomplete");
        return false;
    }
    if (State.DeferredCapture.IsSet()
        || State.PendingReadbacks.Num() >= MaxPendingReadbacksPerStream)
    {
        OutError = TEXT("paired deferred capture capacity changed after batch preflight");
        return false;
    }
    if (TruthSample.SessionId != SessionId
        || TruthSample.ModelGeneration != ModelGeneration
        || TruthSample.ResetGeneration != ResetGeneration
        || TruthSample.SimTimeNs < 0
        || !FMath::IsFinite(SubmittedAtSeconds))
    {
        OutError = TEXT("deferred capture truth stamp does not match the bound runtime identity");
        return false;
    }

    FDeferredCaptureJob Deferred;
    Deferred.TruthSample = TruthSample;
    Deferred.SubmittedFrame = SubmittedFrame;
    Deferred.SubmittedAtSeconds = SubmittedAtSeconds;
    Deferred.PipelineTimeoutSeconds =
        LingTuSim::Sensors::Camera::SelectCapturePipelineTimeoutSeconds(
            State.PublishedFrames,
            CameraFirstFramePipelineTimeoutSeconds,
            CameraCapturePipelineTimeoutSeconds);
    State.Capture->CaptureSceneDeferred();
    State.DeferredCapture = MoveTemp(Deferred);
    State.LastSubmittedTruth = TruthSample;
    if (State.SharedDepthFollower != nullptr)
    {
        State.SharedDepthFollower->LastSubmittedTruth = TruthSample;
    }
    return true;
}

bool ULingTuSimCameraCaptureSubsystem::PromoteDeferredCaptureToReadback(
    FCaptureState& State,
    const uint64 CurrentFrame,
    const double NowSeconds,
    FString& OutError)
{
    OutError.Reset();
    if (!State.DeferredCapture.IsSet())
    {
        return true;
    }
    const FDeferredCaptureJob& Deferred = State.DeferredCapture.GetValue();
    const LingTuSim::Sensors::Camera::EDeferredCaptureAction Action =
        LingTuSim::Sensors::Camera::EvaluateDeferredCapturePromotion(
            Deferred.TruthSample,
            SessionId,
            ModelGeneration,
            ResetGeneration,
            Deferred.SubmittedFrame,
            CurrentFrame,
            Deferred.SubmittedAtSeconds,
            NowSeconds,
            State.PendingReadbacks.Num(),
            MaxPendingReadbacksPerStream,
            Deferred.PipelineTimeoutSeconds,
            OutError);
    if (Action == LingTuSim::Sensors::Camera::EDeferredCaptureAction::Wait)
    {
        return true;
    }
    if (Action == LingTuSim::Sensors::Camera::EDeferredCaptureAction::Discard)
    {
        State.DeferredCapture.Reset();
        State.LastSubmittedTruth = State.LastPublishedTruth;
        if (State.SharedDepthFollower != nullptr)
        {
            State.SharedDepthFollower->LastSubmittedTruth =
                State.SharedDepthFollower->LastPublishedTruth;
        }
        return true;
    }
    if (Action == LingTuSim::Sensors::Camera::EDeferredCaptureAction::Fail)
    {
        return false;
    }
    if (State.Capture == nullptr || State.Target == nullptr || !State.Writer.IsValid())
    {
        OutError = TEXT("capture state is incomplete during deferred readback promotion");
        return false;
    }

    FTextureRenderTargetResource* RenderTarget = State.Target->GameThread_GetRenderTargetResource();
    if (RenderTarget == nullptr)
    {
        OutError = TEXT("render target resource is unavailable");
        return false;
    }

    TSharedPtr<FAsyncReadbackJob, ESPMode::ThreadSafe> Readback =
        MakeShared<FAsyncReadbackJob, ESPMode::ThreadSafe>();
    Readback->TruthSample = Deferred.TruthSample;
    Readback->QueuedAtSeconds = NowSeconds;
    Readback->PipelineTimeoutSeconds = Deferred.PipelineTimeoutSeconds;
    Readback->Readback = MakeUnique<FRHIGPUTextureReadback>(
        FName(*FString::Printf(TEXT("LingTuCamera_%s"), *State.Plan.SensorId)));
    const int32 Width = State.Plan.Width;
    const int32 Height = State.Plan.Height;
    ENQUEUE_RENDER_COMMAND(LingTuCameraEnqueueReadback)(
        [Readback, RenderTarget, Width, Height](FRHICommandListImmediate& RHICmdList)
        {
            FRHITexture* SourceTexture = RenderTarget->GetRenderTargetTexture();
            if (SourceTexture == nullptr || !Readback->Readback.IsValid())
            {
                Readback->FailureReason = TEXT("render target texture is unavailable");
                Readback->Stage.Store(ELingTuCameraReadbackStage::Failed);
                return;
            }
            RHICmdList.Transition(FRHITransitionInfo(
                SourceTexture, ERHIAccess::SRVMask, ERHIAccess::CopySrc));
            Readback->GpuSubmittedAtSeconds.Store(FPlatformTime::Seconds());
            Readback->Readback->EnqueueCopy(
                RHICmdList,
                SourceTexture,
                FResolveRect(0, 0, Width, Height));
            RHICmdList.Transition(FRHITransitionInfo(
                SourceTexture, ERHIAccess::CopySrc, ERHIAccess::SRVMask));
            Readback->Stage.Store(ELingTuCameraReadbackStage::GpuPending);
        });

    State.PendingReadbacks.Add(Readback);
    State.DeferredCapture.Reset();
    return true;
}

bool ULingTuSimCameraCaptureSubsystem::PublishCompletedReadback(
    FCaptureState& State,
    FAsyncReadbackJob& Readback,
    FString& OutError)
{
    OutError.Reset();
    if (!State.Writer.IsValid())
    {
        OutError = TEXT("camera SHM writer is unavailable");
        return false;
    }
    if (Readback.TruthSample.SessionId != SessionId
        || Readback.TruthSample.ModelGeneration != ModelGeneration
        || Readback.TruthSample.ResetGeneration != ResetGeneration)
    {
        OutError = TEXT("camera readback truth identity changed before SHM publication");
        return false;
    }
    if (!State.LastPublishedTruth.SessionId.IsEmpty()
        && (Readback.TruthSample.TruthSequence <= State.LastPublishedTruth.TruthSequence
            || Readback.TruthSample.SimTimeNs <= State.LastPublishedTruth.SimTimeNs))
    {
        OutError = TEXT("camera readback truth did not advance monotonically before SHM publication");
        return false;
    }

    TArray<uint8> ConvertedPayload;
    const TArray<uint8>* Payload = nullptr;
    FFrameMetadata Metadata;
    Metadata.StreamKind = State.Plan.Kind == LingTuSim::Sensors::Camera::EStreamKind::Color
        ? LingTuSim::Sensors::CameraShm::EStreamKind::Color
        : LingTuSim::Sensors::CameraShm::EStreamKind::Depth;
    Metadata.Generation = ModelGeneration;
    // Camera SHM keeps its unix-realtime transport freshness clock. The
    // independently latched TruthSample below is the authoritative simulation
    // clock join; mixing these two clock domains would break SHM stale checks.
    Metadata.TimestampNs = LingTuSim::Sensors::CameraShm::UnixTimeNs();
    Metadata.Width = State.Plan.Width;
    Metadata.Height = State.Plan.Height;
    Metadata.FrameId = State.Plan.FrameId;
    Metadata.Encoding = State.Plan.Encoding;
    Metadata.DepthScale = 0.001;
    const double Focal = static_cast<double>(State.Plan.Width)
        / (2.0 * FMath::Tan(FMath::DegreesToRadians(45.0)));
    Metadata.Fx = Focal;
    Metadata.Fy = Focal;
    Metadata.Cx = (static_cast<double>(State.Plan.Width) - 1.0) * 0.5;
    Metadata.Cy = (static_cast<double>(State.Plan.Height) - 1.0) * 0.5;

    if (State.Plan.Kind == LingTuSim::Sensors::Camera::EStreamKind::Color)
    {
        if (Readback.bHasSharedPayloads)
        {
            Payload = &Readback.SharedColorPayload;
        }
        else if (!LingTuSim::Sensors::Camera::ConvertColorPixels(
                     Readback.ColorPixels,
                     State.Plan.Width,
                     State.Plan.Height,
                     State.Plan.Encoding,
                     ConvertedPayload,
                     OutError))
        {
            if (OutError.IsEmpty())
            {
                OutError = TEXT("asynchronous RGB readback conversion failed");
            }
            return false;
        }
        else
        {
            Payload = &ConvertedPayload;
        }
        Metadata.Stride = State.Plan.Width * (State.Plan.Encoding == TEXT("rgb8") ? 3 : 4);
    }
    else
    {
        if (Readback.bHasSharedPayloads)
        {
            Payload = &Readback.SharedDepthPayload;
        }
        else if (!LingTuSim::Sensors::Camera::ConvertDepthValues(
                     Readback.DepthValues,
                     State.Plan.Width,
                     State.Plan.Height,
                     0.001,
                     ConvertedPayload,
                     OutError))
        {
            if (OutError.IsEmpty())
            {
                OutError = TEXT("asynchronous depth readback conversion failed");
            }
            return false;
        }
        else
        {
            Payload = &ConvertedPayload;
        }
        Metadata.Stride = State.Plan.Width * sizeof(uint16);
    }
    const int32 ExpectedPayloadBytes = Metadata.Stride * Metadata.Height;
    if (Payload == nullptr || Payload->Num() != ExpectedPayloadBytes)
    {
        OutError = TEXT("camera payload size does not match the compiled stream contract");
        return false;
    }
    if (!State.Writer->TryPublish(
            Metadata,
            Payload->GetData(),
            Payload->Num(),
            OutError))
    {
        return false;
    }

    // Publication is the commit boundary. Failed render/readback/SHM attempts
    // never change readiness, counters, or the latched simulation-clock truth.
    const bool bWasActive = State.bActive;
    State.LastPublishedTruth = Readback.TruthSample;
    ++State.PublishedFrames;
    State.bActive = true;
    State.FailureReason.Reset();
    bHasActiveFrame = true;
    if (!bWasActive)
    {
        UE_LOG(
            LogLingTuSimCamera,
            Display,
            TEXT("LINGTU_SENSOR_STREAM_ACTIVE sensor_id=%s source=unreal_camera first_real_render_frame=1 encoding=%s width=%d height=%d mapping=%s truth_sequence=%llu sim_time_ns=%lld"),
            *State.Plan.SensorId,
            *Metadata.Encoding,
            Metadata.Width,
            Metadata.Height,
            *State.Writer->Name(),
            static_cast<unsigned long long>(Readback.TruthSample.TruthSequence),
            static_cast<long long>(Readback.TruthSample.SimTimeNs));
    }
    return true;
}

bool ULingTuSimCameraCaptureSubsystem::PumpCaptureReadbacks(
    FCaptureState& State,
    const bool bAllowPublish,
    const double NowSeconds,
    bool& OutReadinessChanged,
    FString& OutError)
{
    OutError.Reset();
    if (!FMath::IsFinite(NowSeconds))
    {
        OutError = TEXT("camera readback wall clock is invalid");
        return false;
    }
    for (const TSharedPtr<FAsyncReadbackJob, ESPMode::ThreadSafe>& Readback
         : State.PendingReadbacks)
    {
        if (!Readback.IsValid())
        {
            continue;
        }
        const ELingTuCameraReadbackStage Stage = Readback->Stage.Load();
        TOptional<LingTuSim::Sensors::Camera::EReadbackDeadlineStage> DeadlineStage;
        double StageStartedAtSeconds = 0.0;
        if (Stage == ELingTuCameraReadbackStage::EnqueuePending)
        {
            DeadlineStage = LingTuSim::Sensors::Camera::EReadbackDeadlineStage::EnqueuePending;
            StageStartedAtSeconds = Readback->QueuedAtSeconds;
        }
        else if (Stage == ELingTuCameraReadbackStage::GpuPending)
        {
            DeadlineStage = LingTuSim::Sensors::Camera::EReadbackDeadlineStage::GpuPending;
            StageStartedAtSeconds = Readback->GpuSubmittedAtSeconds.Load();
        }
        else if (Stage == ELingTuCameraReadbackStage::CpuDecodePending)
        {
            DeadlineStage = LingTuSim::Sensors::Camera::EReadbackDeadlineStage::CpuDecodePending;
            StageStartedAtSeconds = Readback->CpuDecodeStartedAtSeconds.Load();
        }
        else if (Stage == ELingTuCameraReadbackStage::CpuReady)
        {
            DeadlineStage = LingTuSim::Sensors::Camera::EReadbackDeadlineStage::CpuReady;
        }
        if (DeadlineStage.IsSet())
        {
            FString DeadlineError;
            const LingTuSim::Sensors::Camera::EReadbackDeadlineAction DeadlineAction =
                LingTuSim::Sensors::Camera::EvaluateReadbackStageDeadline(
                    DeadlineStage.GetValue(),
                    StageStartedAtSeconds,
                    NowSeconds,
                    Readback->PipelineTimeoutSeconds,
                    DeadlineError);
            if (DeadlineAction == LingTuSim::Sensors::Camera::EReadbackDeadlineAction::Fail
                && Readback->Stage.Load() == Stage)
            {
                OutError = MoveTemp(DeadlineError);
                return false;
            }
        }
        if (Stage != ELingTuCameraReadbackStage::GpuPending
            || Readback->bPollCommandQueued.Exchange(true))
        {
            continue;
        }
        const bool bColor = State.Plan.Kind
            == LingTuSim::Sensors::Camera::EStreamKind::Color;
        const bool bSharedColorDepth =
            State.Layout == ECaptureLayout::SharedColorLeader;
        const int32 Width = State.Plan.Width;
        const int32 Height = State.Plan.Height;
        const FString ColorEncoding = State.Plan.Encoding;
        ENQUEUE_RENDER_COMMAND(LingTuCameraPollReadback)(
            [Readback, bColor, bSharedColorDepth, Width, Height, ColorEncoding](FRHICommandListImmediate& RHICmdList)
            {
                (void)RHICmdList;
                if (Readback->Stage.Load() != ELingTuCameraReadbackStage::GpuPending
                    || !Readback->Readback.IsValid())
                {
                    Readback->bPollCommandQueued.Store(false);
                    return;
                }
                if (!Readback->Readback->IsReady())
                {
                    Readback->bPollCommandQueued.Store(false);
                    return;
                }

                int32 RowPitchInPixels = 0;
                int32 BufferHeight = 0;
                void* SourceData = Readback->Readback->Lock(
                    RowPitchInPixels, &BufferHeight);
                if (SourceData == nullptr
                    || RowPitchInPixels < Width
                    || BufferHeight < Height)
                {
                    Readback->FailureReason = TEXT("asynchronous GPU readback returned invalid dimensions");
                    if (SourceData != nullptr)
                    {
                        Readback->Readback->Unlock();
                    }
                    Readback->Readback.Reset();
                    Readback->Stage.Store(ELingTuCameraReadbackStage::Failed);
                    Readback->bPollCommandQueued.Store(false);
                    return;
                }

                if (bSharedColorDepth)
                {
                    TArray<FFloat16Color> SharedPixels;
                    SharedPixels.SetNumUninitialized(Width * Height);
                    const FFloat16Color* Source =
                        static_cast<const FFloat16Color*>(SourceData);
                    FFloat16Color* Destination = SharedPixels.GetData();
                    for (int32 Row = 0; Row < Height; ++Row)
                    {
                        const FFloat16Color* SourceRow = Source + Row * RowPitchInPixels;
                        FMemory::Memcpy(
                            Destination + Row * Width,
                            SourceRow,
                            Width * sizeof(FFloat16Color));
                    }
                    Readback->Readback->Unlock();
                    Readback->Readback.Reset();
                    Readback->CpuDecodeStartedAtSeconds.Store(FPlatformTime::Seconds());
                    Readback->Stage.Store(ELingTuCameraReadbackStage::CpuDecodePending);
                    Readback->bPollCommandQueued.Store(false);
                    AsyncTask(
                        ENamedThreads::AnyBackgroundThreadNormalTask,
                        [Readback, SharedPixels = MoveTemp(SharedPixels), Width, Height, ColorEncoding]() mutable
                        {
                            TArray<uint8> ColorPayload;
                            TArray<uint8> DepthPayload;
                            FString DecodeError;
                            const bool bDecoded =
                                LingTuSim::Sensors::Camera::EncodeSharedColorDepthHalfPixels(
                                    SharedPixels,
                                    Width,
                                    Height,
                                    ColorEncoding,
                                    0.001,
                                    ColorPayload,
                                    DepthPayload,
                                    DecodeError);
                            if (!bDecoded)
                            {
                                Readback->FailureReason = MoveTemp(DecodeError);
                                ELingTuCameraReadbackStage Expected =
                                    ELingTuCameraReadbackStage::CpuDecodePending;
                                Readback->Stage.CompareExchange(
                                    Expected,
                                    ELingTuCameraReadbackStage::Failed);
                                return;
                            }
                            Readback->SharedColorPayload = MoveTemp(ColorPayload);
                            Readback->SharedDepthPayload = MoveTemp(DepthPayload);
                            Readback->bHasSharedPayloads = true;
                            ELingTuCameraReadbackStage Expected =
                                ELingTuCameraReadbackStage::CpuDecodePending;
                            Readback->Stage.CompareExchange(
                                Expected,
                                ELingTuCameraReadbackStage::CpuReady);
                        });
                    return;
                }
                else if (bColor)
                {
                    Readback->ColorPixels.SetNumUninitialized(Width * Height);
                    const FColor* Source = static_cast<const FColor*>(SourceData);
                    FColor* Destination = Readback->ColorPixels.GetData();
                    for (int32 Row = 0; Row < Height; ++Row)
                    {
                        FMemory::Memcpy(
                            Destination + Row * Width,
                            Source + Row * RowPitchInPixels,
                            Width * sizeof(FColor));
                    }
                }
                else
                {
                    Readback->DepthValues.SetNumUninitialized(Width * Height);
                    const float* Source = static_cast<const float*>(SourceData);
                    float* Destination = Readback->DepthValues.GetData();
                    for (int32 Row = 0; Row < Height; ++Row)
                    {
                        FMemory::Memcpy(
                            Destination + Row * Width,
                            Source + Row * RowPitchInPixels,
                            Width * sizeof(float));
                    }
                }
                Readback->Readback->Unlock();
                Readback->Readback.Reset();
                Readback->Stage.Store(ELingTuCameraReadbackStage::CpuReady);
                Readback->bPollCommandQueued.Store(false);
            });
    }

    while (!State.PendingReadbacks.IsEmpty())
    {
        const TSharedPtr<FAsyncReadbackJob, ESPMode::ThreadSafe>& Readback =
            State.PendingReadbacks[0];
        if (!Readback.IsValid())
        {
            State.PendingReadbacks.RemoveAt(0);
            continue;
        }
        const ELingTuCameraReadbackStage Stage = Readback->Stage.Load();
        if (Stage == ELingTuCameraReadbackStage::EnqueuePending
            || Stage == ELingTuCameraReadbackStage::GpuPending
            || Stage == ELingTuCameraReadbackStage::CpuDecodePending)
        {
            break;
        }
        if (Stage == ELingTuCameraReadbackStage::Failed)
        {
            OutError = Readback->FailureReason.IsEmpty()
                ? TEXT("asynchronous GPU readback failed")
                : Readback->FailureReason;
            return false;
        }
        if (Stage == ELingTuCameraReadbackStage::CpuReady
            && bAllowPublish
            && !Readback->bDiscard)
        {
            const bool bWasActive = State.bActive;
            if (!PublishCompletedReadback(State, *Readback, OutError))
            {
                return false;
            }
            OutReadinessChanged |= bWasActive != State.bActive;
            if (State.Layout == ECaptureLayout::SharedColorLeader)
            {
                FCaptureState* DepthFollower = State.SharedDepthFollower;
                if (DepthFollower == nullptr
                    || DepthFollower->Layout != ECaptureLayout::SharedDepthFollower)
                {
                    OutError = TEXT("shared RGB/depth readback has no valid depth follower");
                    return false;
                }
                const bool bDepthWasActive = DepthFollower->bActive;
                if (!PublishCompletedReadback(*DepthFollower, *Readback, OutError))
                {
                    return false;
                }
                OutReadinessChanged |= bDepthWasActive != DepthFollower->bActive;
            }
        }
        State.PendingReadbacks.RemoveAt(0);
    }
    return true;
}

void ULingTuSimCameraCaptureSubsystem::DiscardDeferredCapture(FCaptureState& State)
{
    State.DeferredCapture.Reset();
    State.LastSubmittedTruth = State.LastPublishedTruth;
    if (State.SharedDepthFollower != nullptr)
    {
        State.SharedDepthFollower->LastSubmittedTruth =
            State.SharedDepthFollower->LastPublishedTruth;
    }
}

void ULingTuSimCameraCaptureSubsystem::DiscardPendingReadbacks(FCaptureState& State)
{
    for (const TSharedPtr<FAsyncReadbackJob, ESPMode::ThreadSafe>& Readback
         : State.PendingReadbacks)
    {
        if (Readback.IsValid())
        {
            Readback->bDiscard = true;
        }
    }
}

void ULingTuSimCameraCaptureSubsystem::DrainReadbacksForRelease(FCaptureState& State)
{
    if (State.PendingReadbacks.IsEmpty())
    {
        return;
    }
    DiscardPendingReadbacks(State);
    for (const TSharedPtr<FAsyncReadbackJob, ESPMode::ThreadSafe>& Readback
         : State.PendingReadbacks)
    {
        if (!Readback.IsValid())
        {
            continue;
        }
        ENQUEUE_RENDER_COMMAND(LingTuCameraDrainReadback)(
            [Readback](FRHICommandListImmediate& RHICmdList)
            {
                (void)RHICmdList;
                const ELingTuCameraReadbackStage Stage = Readback->Stage.Load();
                if (Readback->Readback.IsValid()
                    && Stage == ELingTuCameraReadbackStage::GpuPending)
                {
                    int32 RowPitchInPixels = 0;
                    int32 BufferHeight = 0;
                    Readback->Readback->Lock(RowPitchInPixels, &BufferHeight);
                    Readback->Readback->Unlock();
                }
                Readback->Readback.Reset();
                Readback->Stage.Store(ELingTuCameraReadbackStage::Cancelled);
                Readback->bPollCommandQueued.Store(false);
            });
    }
    // Teardown is the only blocking boundary. Per-frame capture and publish
    // never flush the render thread.
    FlushRenderingCommands();
    State.PendingReadbacks.Reset();
}

void ULingTuSimCameraCaptureSubsystem::FlushReadinessEvidenceIfDue(
    const double NowSeconds)
{
    if (!ReadinessEvidenceCadence.ShouldAttempt(NowSeconds))
    {
        return;
    }
    FString EvidenceError;
    const bool bSucceeded = WriteReadinessEvidence(EvidenceError);
    ReadinessEvidenceCadence.RecordAttempt(NowSeconds, bSucceeded);
    if (!bSucceeded)
    {
        UE_LOG(
            LogLingTuSimCamera,
            Warning,
            TEXT("LINGTU_CAMERA_READINESS_WRITE_RETRY reason=%s"),
            *EvidenceError);
    }
}

bool ULingTuSimCameraCaptureSubsystem::WriteReadinessEvidence(FString& OutError) const
{
    OutError.Reset();
    if (EvidenceDirectory.IsEmpty())
    {
        OutError = TEXT("owned readiness evidence directory is not configured");
        return false;
    }
    TArray<LingTuSim::Sensors::Camera::FStreamReadiness> Streams;
    Streams.Reserve(Captures.Num());
    for (const FCaptureState* State : Captures)
    {
        if (State == nullptr)
        {
            continue;
        }
        LingTuSim::Sensors::Camera::FStreamReadiness Stream;
        Stream.SensorId = State->Plan.SensorId;
        Stream.State = !State->FailureReason.IsEmpty()
            ? TEXT("FAILED")
            : (State->bActive ? TEXT("ACTIVE") : TEXT("PREPARING"));
        Stream.PublishedFrames = State->PublishedFrames;
        Stream.LastSampleTruthSequence = State->LastPublishedTruth.TruthSequence;
        Stream.LastSampleSimTimeNs = State->LastPublishedTruth.SimTimeNs;
        Stream.Reason = State->FailureReason;
        Streams.Add(MoveTemp(Stream));
    }
    IFileManager::Get().MakeDirectory(*EvidenceDirectory, true);
    // Keep the evidence file in the same directory and replace it only after
    // the complete document has been flushed. The coordinator must never see
    // a partially written readiness document.
    const FString EvidencePath = FPaths::Combine(EvidenceDirectory, TEXT("sensor-readiness.json"));
    const FString TempEvidencePath = FString::Printf(
        TEXT("%s.tmp.%u"),
        *EvidencePath,
        FPlatformProcess::GetCurrentProcessId());
    const FString Json = LingTuSim::Sensors::Camera::BuildReadinessEvidenceJson(
        SessionId,
        ModelGeneration,
        ResetGeneration,
        Streams);
    if (!FFileHelper::SaveStringToFile(Json, *TempEvidencePath, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM)
        || !ReplaceReadinessFileWithRetry(EvidencePath, TempEvidencePath))
    {
        IFileManager::Get().Delete(*TempEvidencePath, false, true, true);
        OutError = FString::Printf(TEXT("failed to write owned readiness evidence '%s'"), *EvidencePath);
        return false;
    }
    return true;
}

void ULingTuSimCameraCaptureSubsystem::FailClosed(const FString& Error)
{
    LastFailure = Error;
    bBound = false;
    bHasActiveFrame = false;
    for (FCaptureState* State : Captures)
    {
        if (State != nullptr)
        {
            State->bActive = false;
            State->FailureReason = Error;
        }
    }
    FString EvidenceError;
    if (!EvidenceDirectory.IsEmpty()
        && Captures.Num() > 0
        && !WriteReadinessEvidence(EvidenceError))
    {
        UE_LOG(
            LogLingTuSimCamera,
            Error,
            TEXT("LINGTU_CAMERA_FAILED_EVIDENCE_WRITE_FAILED reason=%s"),
            *EvidenceError);
    }
    UE_LOG(LogLingTuSimCamera, Error, TEXT("LINGTU_CAMERA_FAILED_CLOSED reason=%s"), *Error);
    ReleaseCaptures();
    EvidenceDirectory.Reset();
}

void ULingTuSimCameraCaptureSubsystem::ReleaseCaptureState(FCaptureState& State)
{
    const bool bHadDeferredCapture = State.DeferredCapture.IsSet();
    const bool bHadPendingReadbacks = !State.PendingReadbacks.IsEmpty();
    DiscardDeferredCapture(State);
    if (State.Capture != nullptr && State.Capture->IsRegistered())
    {
        // OnUnregister removes this component from UE's deferred scene-capture
        // map before any owned render target or actor can be destroyed.
        State.Capture->UnregisterComponent();
    }
    DrainReadbacksForRelease(State);
    if (bHadDeferredCapture && !bHadPendingReadbacks)
    {
        // A queued CaptureSceneDeferred has no readback job to provide the
        // teardown flush. This remains a teardown-only blocking boundary.
        FlushRenderingCommands();
    }
    State.Capture = nullptr;
    if (State.Owner != nullptr)
    {
        State.Owner->Destroy();
        State.Owner = nullptr;
    }
    State.Target = nullptr;
    State.Writer.Reset();
}

void ULingTuSimCameraCaptureSubsystem::ReleaseCaptures()
{
    for (FCaptureState* State : Captures)
    {
        if (State == nullptr)
        {
            continue;
        }
        ReleaseCaptureState(*State);
        delete State;
    }
    Captures.Reset();
}

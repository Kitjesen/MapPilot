#pragma once

#include "CoreMinimal.h"
#include "LingTuSimRuntimeUIModel.h"

namespace LingTuSim::UI
{
    struct FRuntimeUIStatusSnapshot;

    enum class EHudScreenshotValidationPhase : uint8
    {
        BeforeRequest,
        ScreenshotWritten,
        EvidenceCommitted,
    };

    struct LINGTUSIMUI_API FHudScreenshotTarget final
    {
        FString CommandLineName;
        FString ScreenshotPath;
        FString EvidencePath;
        FString ModeName;
        FString ExpectedFilename;
        ERuntimeUIMode Mode = ERuntimeUIMode::Drive;
    };

    /** Strict, run-owned contract for the three playable HUD screenshots. */
    class LINGTUSIMUI_API FHudScreenshotContract final
    {
    public:
        static bool ParseCommandLine(
            const FString& CommandLine,
            TArray<FHudScreenshotTarget>& OutTargets,
            bool& bOutConfigured,
            FString& OutError);

        static bool ValidateHudScreenshotTarget(
            const FHudScreenshotTarget& Target,
            const FString& ControlLogDirectory,
            EHudScreenshotValidationPhase Phase,
            FString& OutError);

        static bool ReadPngDimensions(
            const FString& ScreenshotPath,
            int32& OutWidth,
            int32& OutHeight,
            FString& OutError);

        static bool ValidateEvidenceTempTarget(
            const FHudScreenshotTarget& Target,
            bool bMustExist,
            FString& OutError);

        /** Exclusively creates the temp sidecar without following a target link. */
        static bool WriteEvidenceTempFileNoFollow(
            const FHudScreenshotTarget& Target,
            const FString& EvidenceJson,
            FString& OutError);

        /** Only the exact Drive -> Tactical -> Menu-recording target may advance. */
        static bool IsNextCaptureMode(
            const FHudScreenshotTarget& Target,
            int32 NextCaptureIndex,
            ERuntimeUIMode ActualMode);

        static bool SerializeEvidenceJson(
            const FHudScreenshotTarget& Target,
            const FRuntimeUIStatusSnapshot& Status,
            uint64 CapturedMonotonicNs,
            int64 ScreenshotBytes,
            int32 Width,
            int32 Height,
            FString& OutJson,
            FString& OutError);

    private:
        FHudScreenshotContract() = delete;
    };
}

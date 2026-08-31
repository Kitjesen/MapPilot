#pragma once

#include "CoreMinimal.h"
#include "LingTuSimRuntimeTypes.h"

namespace LingTuSim
{
    enum class ERuntimeLoadErrorCode : uint8
    {
        None,
        InvalidArgument,
        MissingArtifact,
        ReadFailed,
        InvalidJson,
        SchemaMismatch,
        InvalidField,
    };

    struct LINGTUSIMRUNTIME_API FRuntimeLoadError
    {
        ERuntimeLoadErrorCode Code = ERuntimeLoadErrorCode::None;
        FString Source;
        FString Message;

        bool IsSet() const
        {
            return Code != ERuntimeLoadErrorCode::None;
        }

        void Reset()
        {
            Code = ERuntimeLoadErrorCode::None;
            Source.Reset();
            Message.Reset();
        }
    };

    /**
     * Loads compiler-produced SessionBundle JSON and immutable truth snapshots.
     * Output values are replaced only after the complete input has passed validation.
     */
    class LINGTUSIMRUNTIME_API FSessionBundleLoader final
    {
    public:
        static bool LoadSessionBundle(
            const FString& BundleDirectory,
            FSessionBundleView& OutBundle,
            FRuntimeLoadError& OutError);

        static bool LoadSnapshotFile(
            const FString& SnapshotPath,
            const FString& ExpectedSessionId,
            FSnapshotEnvelope& OutSnapshot,
            FRuntimeLoadError& OutError);

        static bool ParseSnapshotJson(
            const FString& SnapshotJson,
            const FString& ExpectedSessionId,
            FSnapshotEnvelope& OutSnapshot,
            FRuntimeLoadError& OutError);

        /**
         * Parses and validates a snapshot envelope without binding it to the
         * currently active session. Session ingress uses this overload so a
         * well-formed next-generation snapshot can trigger an atomic rebind.
         */
        static bool ParseSnapshotJson(
            const FString& SnapshotJson,
            FSnapshotEnvelope& OutSnapshot,
            FRuntimeLoadError& OutError);
    };
}

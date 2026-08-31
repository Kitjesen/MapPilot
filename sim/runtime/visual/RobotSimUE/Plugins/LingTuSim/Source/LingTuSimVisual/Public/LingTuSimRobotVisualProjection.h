#pragma once

#include "CoreMinimal.h"

class ALingTuSimBodyActor;
class ALingTuSimWorldEntityActor;
class UWorld;

namespace LingTuSim::Visual
{
    struct LINGTUSIMVISUAL_API FVisualMaterializationError
    {
        FString Source;
        FString Message;

        void Reset()
        {
            Source.Reset();
            Message.Reset();
        }
    };

    struct LINGTUSIMVISUAL_API FVisualMaterializationResult
    {
        FString SessionId;
        uint64 ModelGeneration = 0;
        uint64 ResetGeneration = 0;
        int32 ExpectedBodyCount = 0;
        int32 ExpectedWorldEntityCount = 0;
        TArray<TObjectPtr<ALingTuSimBodyActor>> Actors;
        TArray<TObjectPtr<ALingTuSimWorldEntityActor>> WorldActors;
    };

    class LINGTUSIMVISUAL_API FRobotVisualProjectionMaterializer final
    {
    public:
        static bool MaterializeBundle(
            UWorld& World,
            const FString& BundleDirectory,
            const FString& ArtifactRoot,
            uint64 ModelGeneration,
            uint64 ResetGeneration,
            FVisualMaterializationResult& OutResult,
            FVisualMaterializationError& OutError);
    };
}

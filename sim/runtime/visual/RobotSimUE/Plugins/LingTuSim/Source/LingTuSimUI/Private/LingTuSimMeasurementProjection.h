#pragma once

#include "CoreMinimal.h"

class FJsonObject;

namespace LingTuSim::UI
{
struct FMeasurementBinding
{
    FString ProductSessionId;
    FString SessionId;
    uint64 ModelGeneration = 0;
    uint64 ResetGeneration = 0;

    bool IsConfigured() const;
};

struct FMeasurementStatus
{
    FString SensorId;
    FString Modality;
    FString State;
    FMeasurementBinding Binding;
    uint64 Sequence = 0;
    uint64 SimTimeNs = 0;
    bool bHasIdentity = false;
    bool bFresh = false;
    double ReceiptAgeSeconds = 0;
    double StaleTimeoutSeconds = 0;
    TSharedPtr<FJsonObject> Settings;
};

struct FMeasurementPreview
{
    FMeasurementStatus Status;
    FString Kind;
    int32 Width = 0;
    int32 Height = 0;
    int32 SourceWidth = 0;
    int32 SourceHeight = 0;
    int32 Channels = 0;
    int32 SampleRateHz = 0;
    int32 SourceSampleCount = 0;
    TArray<int32> RowIndices;
    TArray<int32> ColumnIndices;
    TArray<int32> BinStarts;
    TArray<int32> BinEnds;
    // Row-major Kelvin or class IDs; semantic instance IDs are Secondary.
    TArray<TOptional<double>> Primary;
    TArray<TOptional<double>> Secondary;
    TArray<bool> ValidMask;
    // Channel-major acoustic envelopes, retaining explicit source-bin bounds.
    TArray<TOptional<double>> MinPa;
    TArray<TOptional<double>> MaxPa;
    TArray<double> ValidFractions;
};

// Parses Gateway evidence only. The caller owns binding changes and RTT/local age.
class FMeasurementProjectionParser
{
public:
    static bool ParseList(const FString& Json, const FMeasurementBinding& Expected,
        TArray<FMeasurementStatus>& Out, FString& Error);
    static bool ParsePreview(const FString& Json, const FMeasurementBinding& Expected,
        const FString& ExpectedSensorId, FMeasurementPreview& Out, FString& Error);
    static bool ParseAck(const FString& Json, const FMeasurementBinding& Expected,
        const FString& RequestId, const FString& SensorId, bool& Applied,
        FString& Reason, TSharedPtr<FJsonObject>& Settings, FString& Error);
};
}

#include "LingTuSimMeasurementProjection.h"

#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

namespace LingTuSim::UI
{
namespace
{
using FObject = TSharedPtr<FJsonObject>;
using FValue = TSharedPtr<FJsonValue>;
using FValues = TArray<FValue>;

bool Fail(FString& Error, const TCHAR* Message)
{
    Error = Message;
    return false;
}

FValue Field(const FObject& Object, const TCHAR* Name)
{
    return Object.IsValid() ? Object->TryGetField(Name) : nullptr;
}

FObject ObjectValue(const FValue& Value)
{
    return Value.IsValid() && Value->Type == EJson::Object ? Value->AsObject() : nullptr;
}

const FValues* ArrayValue(const FValue& Value)
{
    return Value.IsValid() && Value->Type == EJson::Array ? &Value->AsArray() : nullptr;
}

bool Text(const FObject& Object, const TCHAR* Name, FString& Out)
{
    const FValue Value = Field(Object, Name);
    return Value.IsValid() && Value->Type == EJson::String && Value->TryGetString(Out);
}

bool Boolean(const FObject& Object, const TCHAR* Name, bool& Out)
{
    const FValue Value = Field(Object, Name);
    return Value.IsValid() && Value->Type == EJson::Boolean && Value->TryGetBool(Out);
}

bool Number(const FValue& Value, double& Out)
{
    return Value.IsValid() && Value->Type == EJson::Number && Value->TryGetNumber(Out) && FMath::IsFinite(Out);
}

// The serializer retains numeric tokens, so uint64 identities never round via double.
bool Unsigned(const FValue& Value, uint64& Out)
{
    FString Token;
    if (!Value.IsValid() || Value->Type != EJson::Number || !Value->TryGetString(Token) || Token.IsEmpty())
    {
        return false;
    }
    Out = 0;
    for (const TCHAR Digit : Token)
    {
        if (Digit < TEXT('0') || Digit > TEXT('9'))
        {
            return false;
        }
        const uint64 Part = static_cast<uint64>(Digit - TEXT('0'));
        if (Out > (MAX_uint64 - Part) / 10)
        {
            return false;
        }
        Out = Out * 10 + Part;
    }
    return true;
}

bool Integer(const FValue& Value, int32& Out, int32 Min = 0, int32 Max = MAX_int32)
{
    uint64 Parsed = 0;
    if (!Unsigned(Value, Parsed) || Parsed < static_cast<uint64>(Min) || Parsed > static_cast<uint64>(Max))
    {
        return false;
    }
    Out = static_cast<int32>(Parsed);
    return true;
}

bool ReadRoot(const FString& Json, const FMeasurementBinding& Expected, FObject& Root, FString& Error)
{
    if (!Expected.IsConfigured())
    {
        return Fail(Error, TEXT("Expected measurement binding is not configured"));
    }
    const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Json);
    if (!FJsonSerializer::Deserialize(Reader, Root, FJsonSerializer::EFlags::StoreNumbersAsStrings) || !Root.IsValid())
    {
        return Fail(Error, TEXT("Invalid measurement JSON object"));
    }
    return true;
}

bool Binding(const FObject& Object, const FMeasurementBinding& Expected, FMeasurementBinding& Out)
{
    return Text(Object, TEXT("product_session_id"), Out.ProductSessionId)
        && Text(Object, TEXT("session_id"), Out.SessionId)
        && Unsigned(Field(Object, TEXT("model_generation")), Out.ModelGeneration)
        && Unsigned(Field(Object, TEXT("reset_generation")), Out.ResetGeneration)
        && Out.ProductSessionId == Expected.ProductSessionId && Out.SessionId == Expected.SessionId
        && Out.ModelGeneration == Expected.ModelGeneration && Out.ResetGeneration == Expected.ResetGeneration;
}

bool ReadStatus(const FObject& Object, const FMeasurementBinding& Expected, FMeasurementStatus& Out, FString& Error)
{
    bool bFresh = false;
    if (!Text(Object, TEXT("sensor_id"), Out.SensorId) || Out.SensorId.IsEmpty()
        || !Text(Object, TEXT("status"), Out.State) || !Boolean(Object, TEXT("fresh"), bFresh)
        || !Number(Field(Object, TEXT("stale_timeout_s")), Out.StaleTimeoutSeconds) || Out.StaleTimeoutSeconds <= 0)
    {
        return Fail(Error, TEXT("Invalid measurement status"));
    }
    const FValue Controls = Field(Object, TEXT("controls"));
    if (Controls.IsValid() && Controls->Type != EJson::Null)
    {
        Out.Settings = ObjectValue(Controls);
        if (!Out.Settings.IsValid())
        {
            return Fail(Error, TEXT("Invalid measurement controls"));
        }
    }
    const FValue Identity = Field(Object, TEXT("source_identity"));
    if (!Identity.IsValid() || Identity->Type == EJson::Null)
    {
        // Declared sensors can have no sample yet. They cannot establish a binding.
        if (bFresh || Out.State == TEXT("live"))
        {
            return Fail(Error, TEXT("Live measurement has no source identity"));
        }
        return true;
    }
    const FObject Source = ObjectValue(Identity);
    FString SourceSensor;
    if (!Binding(Source, Expected, Out.Binding) || !Text(Source, TEXT("sensor_id"), SourceSensor)
        || SourceSensor != Out.SensorId || !Unsigned(Field(Source, TEXT("sequence")), Out.Sequence)
        || !Unsigned(Field(Source, TEXT("sim_time_ns")), Out.SimTimeNs))
    {
        return Fail(Error, TEXT("Measurement source identity mismatch"));
    }
    Out.bHasIdentity = true;
    if (!Text(Object, TEXT("modality"), Out.Modality)
        || (Out.Modality != TEXT("thermal") && Out.Modality != TEXT("semantic") && Out.Modality != TEXT("acoustic"))
        || !Number(Field(Object, TEXT("receipt_age_s")), Out.ReceiptAgeSeconds) || Out.ReceiptAgeSeconds < 0)
    {
        return Fail(Error, TEXT("Invalid measurement modality or receipt age"));
    }
    bool bStale = false;
    if (!Boolean(Object, TEXT("stale"), bStale)
        || (bFresh && (Out.State != TEXT("live") || bStale || Out.ReceiptAgeSeconds > Out.StaleTimeoutSeconds)))
    {
        return Fail(Error, TEXT("Inconsistent measurement freshness"));
    }
    Out.bFresh = bFresh;
    return true;
}

bool Literal(const FObject& Object, const TCHAR* Name, const TCHAR* Expected)
{
    FString Value;
    return Text(Object, Name, Value) && Value == Expected;
}

bool TrueField(const FObject& Object, const TCHAR* Name)
{
    bool Value = false;
    return Boolean(Object, Name, Value) && Value;
}

bool Shape(const FObject& Status, const TCHAR* Name, int32 Rows, int32 Columns)
{
    const FObject Arrays = ObjectValue(Field(Status, TEXT("arrays")));
    const FObject Descriptor = ObjectValue(Field(Arrays, Name));
    const FValues* Values = ArrayValue(Field(Descriptor, TEXT("shape")));
    int32 ActualRows = 0;
    int32 ActualColumns = 0;
    return Values && Values->Num() == 2 && Integer((*Values)[0], ActualRows)
        && Integer((*Values)[1], ActualColumns) && ActualRows == Rows && ActualColumns == Columns;
}

bool Indices(const FObject& Display, const TCHAR* Name, int32 Count, int32 SourceCount, TArray<int32>& Out)
{
    const FValues* Values = ArrayValue(Field(Display, Name));
    if (!Values || Values->Num() != Count)
    {
        return false;
    }
    for (const FValue& Value : *Values)
    {
        int32 Index = 0;
        if (!Integer(Value, Index, 0, SourceCount - 1) || (!Out.IsEmpty() && Index <= Out.Last()))
        {
            return false;
        }
        Out.Add(Index);
    }
    return true;
}

bool Matrix(const FObject& Display, const TCHAR* Name, int32 Rows, int32 Columns,
    bool bLabels, TArray<TOptional<double>>& Out)
{
    const FValues* Values = ArrayValue(Field(Display, Name));
    if (!Values || Values->Num() != Rows)
    {
        return false;
    }
    Out.Reserve(Rows * Columns);
    for (const FValue& Row : *Values)
    {
        const FValues* Cells = ArrayValue(Row);
        if (!Cells || Cells->Num() != Columns)
        {
            return false;
        }
        for (const FValue& Cell : *Cells)
        {
            if (!Cell.IsValid() || (Cell->Type != EJson::Null && Cell->Type != EJson::Number))
            {
                return false;
            }
            double Value = 0;
            const bool bFinite = Number(Cell, Value);
            uint64 Label = 0;
            if (bFinite && bLabels && (!Unsigned(Cell, Label) || Label > MAX_uint32))
            {
                return false;
            }
            Out.Add(bFinite ? TOptional<double>(Value) : TOptional<double>());
        }
    }
    return true;
}

bool Image(const FObject& Status, const FObject& Display, FMeasurementPreview& Out, FString& Error)
{
    if (!Integer(Field(Display, TEXT("width")), Out.Width, 1, 192)
        || !Integer(Field(Display, TEXT("height")), Out.Height, 1, 144)
        || !Integer(Field(Display, TEXT("source_width")), Out.SourceWidth, 1)
        || !Integer(Field(Display, TEXT("source_height")), Out.SourceHeight, 1)
        || Out.Width != FMath::Min(Out.SourceWidth, 192) || Out.Height != FMath::Min(Out.SourceHeight, 144)
        || !Literal(Display, TEXT("sampling"), TEXT("nearest_source_pixel"))
        || !Indices(Display, TEXT("row_indices"), Out.Height, Out.SourceHeight, Out.RowIndices)
        || !Indices(Display, TEXT("column_indices"), Out.Width, Out.SourceWidth, Out.ColumnIndices)
        || !Shape(Status, TEXT("valid_mask"), Out.SourceHeight, Out.SourceWidth))
    {
        return Fail(Error, TEXT("Invalid image dimensions or source indices"));
    }
    const bool bSemantic = Out.Kind == TEXT("semantic");
    const TCHAR* PrimaryName = bSemantic ? TEXT("class_ids") : TEXT("temperature_k");
    if (!Shape(Status, PrimaryName, Out.SourceHeight, Out.SourceWidth)
        || !Matrix(Display, PrimaryName, Out.Height, Out.Width, bSemantic, Out.Primary)
        || (bSemantic && (!TrueField(Display, TEXT("ground_truth"))
            || !TrueField(ObjectValue(Field(Status, TEXT("metadata"))), TEXT("ground_truth"))
            || !Shape(Status, TEXT("instance_ids"), Out.SourceHeight, Out.SourceWidth)
            || !Matrix(Display, TEXT("instance_ids"), Out.Height, Out.Width, true, Out.Secondary))))
    {
        return Fail(Error, TEXT("Invalid image values, labels, or semantic ground truth"));
    }
    const FValues* Mask = ArrayValue(Field(Display, TEXT("valid_mask")));
    if (!Mask || Mask->Num() != Out.Height)
    {
        return Fail(Error, TEXT("Invalid image validity dimensions"));
    }
    for (const FValue& Row : *Mask)
    {
        const FValues* Cells = ArrayValue(Row);
        if (!Cells || Cells->Num() != Out.Width)
        {
            return Fail(Error, TEXT("Invalid image validity row"));
        }
        for (const FValue& Cell : *Cells)
        {
            bool bValid = false;
            if (!Cell.IsValid() || Cell->Type != EJson::Boolean || !Cell->TryGetBool(bValid))
            {
                return Fail(Error, TEXT("Invalid image validity value"));
            }
            const int32 Index = Out.ValidMask.Num();
            bValid = bValid && Out.Primary[Index].IsSet() && (!bSemantic || Out.Secondary[Index].IsSet());
            Out.ValidMask.Add(bValid);
            if (!bValid)
            {
                Out.Primary[Index].Reset();
                if (bSemantic)
                {
                    Out.Secondary[Index].Reset();
                }
            }
        }
    }
    return true;
}

bool Acoustic(const FObject& Status, const FObject& Display, FMeasurementPreview& Out, FString& Error)
{
    const FValues* Starts = ArrayValue(Field(Display, TEXT("bin_start_samples")));
    const FValues* Ends = ArrayValue(Field(Display, TEXT("bin_end_samples")));
    const FValues* Minima = ArrayValue(Field(Display, TEXT("pressure_min_pa")));
    int32 StatusRate = 0;
    if (!Literal(Display, TEXT("sampling"), TEXT("min_max_envelope"))
        || !Integer(Field(Display, TEXT("sample_rate_hz")), Out.SampleRateHz, 1)
        || !Integer(Field(Status, TEXT("sample_rate_hz")), StatusRate, 1) || StatusRate != Out.SampleRateHz
        || !Integer(Field(Display, TEXT("source_sample_count")), Out.SourceSampleCount, 1)
        || !Starts || !Ends || Starts->Num() != FMath::Min(Out.SourceSampleCount, 256) || Ends->Num() != Starts->Num()
        || !Minima || Minima->IsEmpty() || Minima->Num() > 64)
    {
        return Fail(Error, TEXT("Invalid acoustic sampling or channel bounds"));
    }
    Out.Channels = Minima->Num();
    const int32 Bins = Starts->Num();
    for (int32 Index = 0; Index < Bins; ++Index)
    {
        int32 Start = 0;
        int32 End = 0;
        if (!Integer((*Starts)[Index], Start, 0, Out.SourceSampleCount - 1)
            || !Integer((*Ends)[Index], End, 1, Out.SourceSampleCount) || Start >= End
            || Start != (Index == 0 ? 0 : Out.BinEnds.Last()))
        {
            return Fail(Error, TEXT("Invalid acoustic source bin bounds"));
        }
        Out.BinStarts.Add(Start);
        Out.BinEnds.Add(End);
    }
    TArray<TOptional<double>> Fractions;
    if (Out.BinEnds.Last() != Out.SourceSampleCount
        || !Shape(Status, TEXT("pressure_pa"), Out.Channels, Out.SourceSampleCount)
        || !Shape(Status, TEXT("valid_mask"), Out.Channels, Out.SourceSampleCount)
        || !Matrix(Display, TEXT("pressure_min_pa"), Out.Channels, Bins, false, Out.MinPa)
        || !Matrix(Display, TEXT("pressure_max_pa"), Out.Channels, Bins, false, Out.MaxPa)
        || !Matrix(Display, TEXT("valid_fraction"), Out.Channels, Bins, false, Fractions))
    {
        return Fail(Error, TEXT("Invalid acoustic channel or envelope dimensions"));
    }
    for (int32 Index = 0; Index < Fractions.Num(); ++Index)
    {
        if (!Fractions[Index].IsSet() || Fractions[Index].GetValue() < 0 || Fractions[Index].GetValue() > 1)
        {
            return Fail(Error, TEXT("Invalid acoustic valid fraction"));
        }
        const double Fraction = Fractions[Index].GetValue();
        Out.ValidFractions.Add(Fraction);
        const bool bValid = Fraction > 0 && Out.MinPa[Index].IsSet() && Out.MaxPa[Index].IsSet();
        if (bValid && Out.MinPa[Index].GetValue() > Out.MaxPa[Index].GetValue())
        {
            return Fail(Error, TEXT("Acoustic minimum exceeds maximum"));
        }
        Out.ValidMask.Add(bValid);
        if (!bValid)
        {
            Out.MinPa[Index].Reset();
            Out.MaxPa[Index].Reset();
        }
    }
    return true;
}
}

bool FMeasurementBinding::IsConfigured() const
{
    return !ProductSessionId.IsEmpty() && !SessionId.IsEmpty();
}

bool FMeasurementProjectionParser::ParseList(const FString& Json, const FMeasurementBinding& Expected,
    TArray<FMeasurementStatus>& Out, FString& Error)
{
    Out.Reset();
    Error.Reset();
    FObject Root;
    if (!ReadRoot(Json, Expected, Root, Error))
    {
        return false;
    }
    const FValues* Sensors = ArrayValue(Field(Root, TEXT("sensors")));
    if (!Sensors)
    {
        return Fail(Error, TEXT("Missing measurement sensor list"));
    }
    TArray<FMeasurementStatus> Parsed;
    for (const FValue& Sensor : *Sensors)
    {
        FMeasurementStatus Status;
        if (!ReadStatus(ObjectValue(Sensor), Expected, Status, Error))
        {
            return false;
        }
        Parsed.Add(MoveTemp(Status));
    }
    Out = MoveTemp(Parsed);
    return true;
}

bool FMeasurementProjectionParser::ParsePreview(const FString& Json, const FMeasurementBinding& Expected,
    const FString& ExpectedSensorId, FMeasurementPreview& Out, FString& Error)
{
    Out = FMeasurementPreview();
    Error.Reset();
    FObject Root;
    if (!ReadRoot(Json, Expected, Root, Error))
    {
        return false;
    }
    const FObject Status = ObjectValue(Field(Root, TEXT("measurement")));
    const FObject Display = ObjectValue(Field(Root, TEXT("display")));
    FMeasurementPreview Parsed;
    if (!ReadStatus(Status, Expected, Parsed.Status, Error))
    {
        return false;
    }
    if (!Parsed.Status.bHasIdentity || !Parsed.Status.bFresh || Parsed.Status.SensorId != ExpectedSensorId
        || !Text(Display, TEXT("kind"), Parsed.Kind) || Parsed.Kind != Parsed.Status.Modality
        || !TrueField(Display, TEXT("display_only")))
    {
        return Fail(Error, TEXT("Preview is not fresh, bound, display-only measurement evidence"));
    }
    const FObject Units = ObjectValue(Field(Status, TEXT("units")));
    const bool bUnitsMatch = Parsed.Kind == TEXT("thermal") ? Literal(Units, TEXT("temperature_k"), TEXT("K"))
        : Parsed.Kind == TEXT("acoustic") ? Literal(Units, TEXT("pressure_pa"), TEXT("Pa"))
        : Literal(Units, TEXT("class_ids"), TEXT("label")) && Literal(Units, TEXT("instance_ids"), TEXT("label"));
    if (!bUnitsMatch)
    {
        return Fail(Error, TEXT("Measurement units do not match the modality"));
    }
    if (!(Parsed.Kind == TEXT("acoustic") ? Acoustic(Status, Display, Parsed, Error) : Image(Status, Display, Parsed, Error)))
    {
        return false;
    }
    Out = MoveTemp(Parsed);
    return true;
}

bool FMeasurementProjectionParser::ParseAck(const FString& Json, const FMeasurementBinding& Expected,
    const FString& RequestId, const FString& SensorId, bool& Applied, FString& Reason,
    TSharedPtr<FJsonObject>& Settings, FString& Error)
{
    Applied = false;
    Reason.Reset();
    Settings.Reset();
    Error.Reset();
    FObject Root;
    if (!ReadRoot(Json, Expected, Root, Error))
    {
        return false;
    }
    FMeasurementBinding Actual;
    FString ActualRequest;
    FString ActualSensor;
    FString ParsedReason;
    bool bParsedApplied = false;
    const FObject ParsedSettings = ObjectValue(Field(Root, TEXT("settings")));
    if (!Binding(Root, Expected, Actual) || !Text(Root, TEXT("request_id"), ActualRequest)
        || ActualRequest != RequestId || RequestId.IsEmpty() || !Text(Root, TEXT("sensor_id"), ActualSensor)
        || ActualSensor != SensorId || SensorId.IsEmpty() || !Boolean(Root, TEXT("applied"), bParsedApplied)
        || !Text(Root, TEXT("reason"), ParsedReason) || !ParsedSettings.IsValid())
    {
        return Fail(Error, TEXT("Invalid or mismatched sensor control ACK"));
    }
    Applied = bParsedApplied;
    Reason = MoveTemp(ParsedReason);
    Settings = ParsedSettings;
    return true;
}
}

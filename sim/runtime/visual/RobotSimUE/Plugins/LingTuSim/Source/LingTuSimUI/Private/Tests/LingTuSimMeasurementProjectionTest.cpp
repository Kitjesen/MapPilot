#if WITH_DEV_AUTOMATION_TESTS

#include "LingTuSimMeasurementProjection.h"

#include "Dom/JsonObject.h"
#include "Misc/AutomationTest.h"

namespace LingTuSim::UI::MeasurementProjectionTests
{
FMeasurementBinding Expected()
{
    return {TEXT("product"), TEXT("session"), 3, 4};
}

FString Status(const FString& Kind)
{
    const FString Units = Kind == TEXT("thermal") ? TEXT(R"({"temperature_k":"K","valid_mask":"1"})")
        : Kind == TEXT("semantic") ? TEXT(R"({"class_ids":"label","instance_ids":"label","valid_mask":"1"})")
        : TEXT(R"({"pressure_pa":"Pa","valid_mask":"1"})");
    const FString Arrays = Kind == TEXT("thermal")
        ? TEXT(R"({"temperature_k":{"shape":[2,2],"dtype":"<f4"},"valid_mask":{"shape":[2,2],"dtype":"|b1"}})")
        : Kind == TEXT("semantic")
        ? TEXT(R"({"class_ids":{"shape":[2,2],"dtype":"<u4"},"instance_ids":{"shape":[2,2],"dtype":"<u4"},"valid_mask":{"shape":[2,2],"dtype":"|b1"}})")
        : TEXT(R"({"pressure_pa":{"shape":[2,4],"dtype":"<f4"},"valid_mask":{"shape":[2,4],"dtype":"|b1"}})");
    return FString::Printf(TEXT(R"({"sensor_id":"sensor","status":"live","fresh":true,"stale":false,
        "receipt_age_s":0.1,"stale_timeout_s":1,"modality":"%s","controls":{"enabled":true},
        "source_identity":{"product_session_id":"product","session_id":"session","sensor_id":"sensor",
        "model_generation":3,"reset_generation":4,"sequence":1,"sim_time_ns":25,"frame_id":"frame","instance_id":"rig"},
        "units":%s,"metadata":{"ground_truth":true,"units":%s},"arrays":%s,"sample_rate_hz":8000})"),
        *Kind, *Units, *Units, *Arrays);
}

FString Preview(const FString& Kind)
{
    const FString Display = Kind == TEXT("thermal")
        ? TEXT(R"({"kind":"thermal","display_only":true,"sampling":"nearest_source_pixel",
            "source_width":2,"source_height":2,"width":2,"height":2,"row_indices":[0,1],"column_indices":[0,1],
            "temperature_k":[[300,null],[302,303]],"valid_mask":[[true,false],[true,true]]})")
        : Kind == TEXT("semantic")
        ? TEXT(R"({"kind":"semantic","display_only":true,"sampling":"nearest_source_pixel","ground_truth":true,
            "source_width":2,"source_height":2,"width":2,"height":2,"row_indices":[0,1],"column_indices":[0,1],
            "class_ids":[[7,null],[9,10]],"instance_ids":[[42,null],[4000000,4294967295]],
            "valid_mask":[[true,false],[true,true]]})")
        : TEXT(R"({"kind":"acoustic","display_only":true,"sampling":"min_max_envelope",
            "sample_rate_hz":8000,"source_sample_count":4,"bin_start_samples":[0,1,2,3],"bin_end_samples":[1,2,3,4],
            "pressure_min_pa":[[-2,null,0,1],[2,3,4,5]],"pressure_max_pa":[[-2,null,0,1],[2,3,4,5]],
            "valid_fraction":[[1,0,1,1],[1,1,1,1]]})");
    return TEXT("{\"measurement\":") + Status(Kind) + TEXT(",\"display\":") + Display + TEXT("}");
}

FString List(const FString& Entry)
{
    return TEXT("{\"sensors\":[") + Entry + TEXT("],\"status\":\"configured\",\"read_only\":true}");
}

FString Ack()
{
    return TEXT(R"({"request_id":"request","product_session_id":"product","session_id":"session",
        "sensor_id":"sensor","model_generation":3,"reset_generation":4,"applied_sim_time_ns":25,
        "applied":false,"reason":"unsupported_rate","settings":{"enabled":true,"rate_hz":10}})");
}

FString AcousticSize(int32 Channels, int32 Bins)
{
    TArray<FString> Starts;
    TArray<FString> Ends;
    TArray<FString> Values;
    for (int32 Index = 0; Index < Bins; ++Index)
    {
        Starts.Add(FString::FromInt(Index));
        Ends.Add(FString::FromInt(Index + 1));
        Values.Add(TEXT("1"));
    }
    const FString Row = TEXT("[") + FString::Join(Values, TEXT(",")) + TEXT("]");
    TArray<FString> Rows;
    for (int32 Index = 0; Index < Channels; ++Index)
    {
        Rows.Add(Row);
    }
    const FString Matrix = TEXT("[") + FString::Join(Rows, TEXT(",")) + TEXT("]");
    FString Measurement = Status(TEXT("acoustic"));
    Measurement.ReplaceInline(TEXT("[2,4]"), *FString::Printf(TEXT("[%d,%d]"), Channels, Bins));
    return FString::Printf(TEXT(R"({"measurement":%s,"display":{"kind":"acoustic","display_only":true,
        "sampling":"min_max_envelope","sample_rate_hz":8000,"source_sample_count":%d,
        "bin_start_samples":[%s],"bin_end_samples":[%s],"pressure_min_pa":%s,"pressure_max_pa":%s,"valid_fraction":%s}})"),
        *Measurement, Bins, *FString::Join(Starts, TEXT(",")), *FString::Join(Ends, TEXT(",")), *Matrix, *Matrix, *Matrix);
}
}

using namespace LingTuSim::UI;
using namespace LingTuSim::UI::MeasurementProjectionTests;

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuSimMeasurementModalitiesTest,
    "LingTuSim.UI.Measurements.Projection.ValidModalities",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FLingTuSimMeasurementModalitiesTest::RunTest(const FString& Parameters)
{
    (void)Parameters;
    FString Error;
    FMeasurementPreview Out;
    if (!TestTrue(TEXT("thermal parses"), FMeasurementProjectionParser::ParsePreview(
        Preview(TEXT("thermal")), Expected(), TEXT("sensor"), Out, Error)))
    {
        AddError(Error);
        return false;
    }
    TestEqual(TEXT("thermal remains Kelvin"), Out.Primary[0].GetValue(), 300.0);
    TestEqual(TEXT("thermal row-major second row"), Out.Primary[2].GetValue(), 302.0);
    TestEqual(TEXT("image width"), Out.Width, 2);
    TestEqual(TEXT("source height"), Out.SourceHeight, 2);
    TestEqual(TEXT("explicit source row"), Out.RowIndices[1], 1);
    TestTrue(TEXT("identity present"), Out.Status.bHasIdentity);
    TestTrue(TEXT("source sequence retained"), Out.Status.Sequence == 1);
    TestTrue(TEXT("source simulation time retained"), Out.Status.SimTimeNs == 25);
    TestTrue(TEXT("server evidence is fresh"), Out.Status.bFresh);
    TestEqual(TEXT("receipt age is not inflated by parser"), Out.Status.ReceiptAgeSeconds, 0.1);
    TestTrue(TEXT("settings retained"), Out.Status.Settings.IsValid());
    TestFalse(TEXT("null thermal is unknown, not zero"), Out.Primary[1].IsSet());
    TestFalse(TEXT("null thermal invalid"), Out.ValidMask[1]);

    if (!TestTrue(TEXT("semantic parses"), FMeasurementProjectionParser::ParsePreview(
        Preview(TEXT("semantic")), Expected(), TEXT("sensor"), Out, Error)))
    {
        AddError(Error);
        return false;
    }
    TestEqual(TEXT("class label retained"), Out.Primary[0].GetValue(), 7.0);
    TestEqual(TEXT("instance label retained"), Out.Secondary[0].GetValue(), 42.0);
    TestEqual(TEXT("instance labels are not narrowed"), Out.Secondary[3].GetValue(), 4294967295.0);
    TestFalse(TEXT("invalid semantic class unknown"), Out.Primary[1].IsSet());
    TestFalse(TEXT("invalid semantic instance unknown"), Out.Secondary[1].IsSet());

    if (!TestTrue(TEXT("acoustic parses"), FMeasurementProjectionParser::ParsePreview(
        Preview(TEXT("acoustic")), Expected(), TEXT("sensor"), Out, Error)))
    {
        AddError(Error);
        return false;
    }
    TestEqual(TEXT("all channels retained"), Out.Channels, 2);
    TestEqual(TEXT("all bins retained"), Out.MinPa.Num(), 8);
    TestEqual(TEXT("channel-major second channel"), Out.MinPa[4].GetValue(), 2.0);
    TestEqual(TEXT("signed Pa retained"), Out.MinPa[0].GetValue(), -2.0);
    TestEqual(TEXT("sample rate retained"), Out.SampleRateHz, 8000);
    TestEqual(TEXT("source sample count retained"), Out.SourceSampleCount, 4);
    TestEqual(TEXT("explicit bin start"), Out.BinStarts[2], 2);
    TestEqual(TEXT("explicit bin end"), Out.BinEnds[3], 4);
    TestFalse(TEXT("empty acoustic bin stays unknown"), Out.MinPa[1].IsSet());
    TestFalse(TEXT("empty acoustic maximum stays unknown"), Out.MaxPa[1].IsSet());
    TestEqual(TEXT("invalid fraction retained"), Out.ValidFractions[1], 0.0);
    TestTrue(TEXT("success clears previous errors"), Error.IsEmpty());
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuSimMeasurementIdentityTest,
    "LingTuSim.UI.Measurements.Projection.IdentityAndGenerations",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FLingTuSimMeasurementIdentityTest::RunTest(const FString& Parameters)
{
    (void)Parameters;
    FString Error;
    TArray<FMeasurementStatus> Out;
    TestTrue(TEXT("declared sensor without sample is allowed"), FMeasurementProjectionParser::ParseList(
        List(TEXT(R"({"sensor_id":"waiting","status":"no_sample","fresh":false,"stale_timeout_s":1,
            "source_identity":null,"modality":null,"receipt_age_s":null,"controls":null})")), Expected(), Out, Error));
    if (TestEqual(TEXT("one declared sensor"), Out.Num(), 1))
    {
        TestFalse(TEXT("no identity inferred"), Out[0].bHasIdentity);
        TestFalse(TEXT("no freshness inferred"), Out[0].bFresh);
        TestFalse(TEXT("no Product auto-discovery"), Out[0].Binding.IsConfigured());
    }
    const FString Valid = Status(TEXT("thermal"));
    const TPair<FString, FString> Changes[] = {
        {TEXT("\"product_session_id\":\"product\""), TEXT("\"product_session_id\":\"other\"")},
        {TEXT("\"session_id\":\"session\""), TEXT("\"session_id\":\"other\"")},
        {TEXT("\"model_generation\":3"), TEXT("\"model_generation\":2")},
        {TEXT("\"reset_generation\":4"), TEXT("\"reset_generation\":5")},
        {TEXT("\"model_generation\":3"), TEXT("\"model_generation\":3.5")},
        {TEXT("\"model_generation\":3"), TEXT("\"model_generation\":\"3\"")},
        {TEXT("\"reset_generation\":4"), TEXT("\"reset_generation\":-1")},
        {TEXT("\"reset_generation\":4"), TEXT("\"reset_generation\":18446744073709551616")},
        {TEXT("\"sequence\":1"), TEXT("\"sequence\":-1")},
        {TEXT("\"sim_time_ns\":25"), TEXT("\"sim_time_ns\":-1")},
    };
    for (const auto& Change : Changes)
    {
        const FString Bad = Valid.Replace(*Change.Key, *Change.Value, ESearchCase::CaseSensitive);
        TestFalse(TEXT("mixed list fails atomically"), FMeasurementProjectionParser::ParseList(
            List(Valid + TEXT(",") + Bad), Expected(), Out, Error));
        TestTrue(TEXT("failed list clears old statuses"), Out.IsEmpty());
        TestFalse(TEXT("failure has reason"), Error.IsEmpty());
    }
    FMeasurementBinding Large = Expected();
    Large.ModelGeneration = 9007199254740993ULL;
    Large.ResetGeneration = MAX_uint64;
    const FString LargeStatus = Valid.Replace(TEXT("\"model_generation\":3"), TEXT("\"model_generation\":9007199254740993"))
        .Replace(TEXT("\"reset_generation\":4"), TEXT("\"reset_generation\":18446744073709551615"));
    TestTrue(TEXT("full uint64 binding is exact"), FMeasurementProjectionParser::ParseList(List(LargeStatus), Large, Out, Error));
    Large.ModelGeneration -= 1;
    TestFalse(TEXT("adjacent generations above double precision differ"),
        FMeasurementProjectionParser::ParseList(List(LargeStatus), Large, Out, Error));
    TestFalse(TEXT("missing expected Product is not discovered"),
        FMeasurementProjectionParser::ParseList(List(Valid), FMeasurementBinding(), Out, Error));
    const FString BadSensor = Valid.Replace(TEXT("\"sensor_id\":\"sensor\",\"status\""),
        TEXT("\"sensor_id\":\"other\",\"status\""));
    TestFalse(TEXT("identity sensor must agree with status"),
        FMeasurementProjectionParser::ParseList(List(BadSensor), Expected(), Out, Error));
    FMeasurementPreview Frame;
    TestFalse(TEXT("preview sensor must match request"), FMeasurementProjectionParser::ParsePreview(
        Preview(TEXT("thermal")), Expected(), TEXT("other"), Frame, Error));
    const FString Zero = Valid.Replace(TEXT("\"sequence\":1"), TEXT("\"sequence\":0"))
        .Replace(TEXT("\"sim_time_ns\":25"), TEXT("\"sim_time_ns\":0"));
    TestTrue(TEXT("zero source sequence and sim time are allowed"),
        FMeasurementProjectionParser::ParseList(List(Zero), Expected(), Out, Error));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuSimMeasurementMalformedTest,
    "LingTuSim.UI.Measurements.Projection.DimensionsLabelsAndFreshness",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FLingTuSimMeasurementMalformedTest::RunTest(const FString& Parameters)
{
    (void)Parameters;
    struct FMutation
    {
        const TCHAR* Kind;
        const TCHAR* From;
        const TCHAR* To;
    };
    const FMutation Mutations[] = {
        {TEXT("thermal"), TEXT("\"width\":2"), TEXT("\"width\":193")},
        {TEXT("thermal"), TEXT("\"height\":2"), TEXT("\"height\":145")},
        {TEXT("thermal"), TEXT("\"width\":2"), TEXT("\"width\":2.5")},
        {TEXT("thermal"), TEXT("[300,null]"), TEXT("[300]")},
        {TEXT("thermal"), TEXT("[true,false]"), TEXT("[true]")},
        {TEXT("thermal"), TEXT("\"shape\":[2,2]"), TEXT("\"shape\":[2,3]")},
        {TEXT("thermal"), TEXT("\"column_indices\":[0,1]"), TEXT("\"column_indices\":[0,2]")},
        {TEXT("thermal"), TEXT("\"temperature_k\":\"K\""), TEXT("\"temperature_k\":\"C\"")},
        {TEXT("thermal"), TEXT("\"fresh\":true"), TEXT("\"fresh\":false")},
        {TEXT("thermal"), TEXT("\"stale\":false"), TEXT("\"stale\":true")},
        {TEXT("thermal"), TEXT("\"receipt_age_s\":0.1"), TEXT("\"receipt_age_s\":2")},
        {TEXT("thermal"), TEXT("\"display_only\":true"), TEXT("\"display_only\":false")},
        {TEXT("semantic"), TEXT("[7,null]"), TEXT("[7.5,null]")},
        {TEXT("semantic"), TEXT("[7,null]"), TEXT("[7.0,null]")},
        {TEXT("semantic"), TEXT("[7,null]"), TEXT("[-1,null]")},
        {TEXT("semantic"), TEXT("[42,null]"), TEXT("[4294967296,null]")},
        {TEXT("semantic"), TEXT("\"ground_truth\":true"), TEXT("\"ground_truth\":false")},
        {TEXT("semantic"), TEXT("\"metadata\":{\"ground_truth\":true"), TEXT("\"metadata\":{\"ground_truth\":false")},
        {TEXT("semantic"), TEXT("\"sampling\":\"nearest_source_pixel\",\"ground_truth\":true"),
            TEXT("\"sampling\":\"nearest_source_pixel\",\"ground_truth\":false")},
        {TEXT("semantic"), TEXT("\"class_ids\":\"label\""), TEXT("\"class_ids\":\"K\"")},
        {TEXT("acoustic"), TEXT("[2,3,4,5]"), TEXT("[2,3,4]")},
        {TEXT("acoustic"), TEXT("\"bin_end_samples\":[1,2,3,4]"), TEXT("\"bin_end_samples\":[1,2,3,3]")},
        {TEXT("acoustic"), TEXT("\"bin_start_samples\":[0,1,2,3]"), TEXT("\"bin_start_samples\":[0,0,2,3]")},
        {TEXT("acoustic"), TEXT("\"source_sample_count\":4"), TEXT("\"source_sample_count\":5")},
        {TEXT("acoustic"), TEXT("\"pressure_pa\":\"Pa\""), TEXT("\"pressure_pa\":\"V\"")},
        {TEXT("acoustic"), TEXT("[1,0,1,1]"), TEXT("[1,1.1,1,1]")},
    };
    FString Error;
    FMeasurementPreview Out;
    for (const FMutation& Mutation : Mutations)
    {
        const FString Json = Preview(Mutation.Kind).Replace(Mutation.From, Mutation.To, ESearchCase::CaseSensitive);
        Out.Kind = TEXT("old_frame");
        Out.Primary.Add(999.0);
        Out.Status.bFresh = true;
        TestFalse(FString::Printf(TEXT("reject %s -> %s"), Mutation.From, Mutation.To),
            FMeasurementProjectionParser::ParsePreview(Json, Expected(), TEXT("sensor"), Out, Error));
        TestTrue(TEXT("failure removes old frame and values"), Out.Kind.IsEmpty() && Out.Primary.IsEmpty());
        TestFalse(TEXT("failure cannot leave old freshness"), Out.Status.bFresh);
        TestFalse(TEXT("failure has diagnostic"), Error.IsEmpty());
    }
    TestFalse(TEXT("malformed JSON rejected"), FMeasurementProjectionParser::ParsePreview(
        TEXT("not json"), Expected(), TEXT("sensor"), Out, Error));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuSimMeasurementBoundsTest,
    "LingTuSim.UI.Measurements.Projection.AcousticBoundsAndChannels",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FLingTuSimMeasurementBoundsTest::RunTest(const FString& Parameters)
{
    (void)Parameters;
    FString Error;
    FMeasurementPreview Out;
    if (TestTrue(TEXT("64 complete channels with 256 bins supported"), FMeasurementProjectionParser::ParsePreview(
        AcousticSize(64, 256), Expected(), TEXT("sensor"), Out, Error)))
    {
        TestEqual(TEXT("no channels silently dropped"), Out.Channels, 64);
        TestEqual(TEXT("all channel bins present"), Out.MinPa.Num(), 64 * 256);
        TestEqual(TEXT("last explicit end bound"), Out.BinEnds.Last(), 256);
        TestEqual(TEXT("last channel bin valid"), Out.MaxPa.Last().GetValue(), 1.0);
    }
    TestFalse(TEXT("65 channels rejected"), FMeasurementProjectionParser::ParsePreview(
        AcousticSize(65, 2), Expected(), TEXT("sensor"), Out, Error));
    TestFalse(TEXT("257 bins rejected"), FMeasurementProjectionParser::ParsePreview(
        AcousticSize(2, 257), Expected(), TEXT("sensor"), Out, Error));
    const FString Incomplete = AcousticSize(2, 4).Replace(TEXT("\"shape\":[2,4]"), TEXT("\"shape\":[3,4]"));
    TestFalse(TEXT("source channels must all be present"), FMeasurementProjectionParser::ParsePreview(
        Incomplete, Expected(), TEXT("sensor"), Out, Error));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuSimMeasurementInvalidValuesTest,
    "LingTuSim.UI.Measurements.Projection.InvalidValuesStayUnknown",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FLingTuSimMeasurementInvalidValuesTest::RunTest(const FString& Parameters)
{
    (void)Parameters;
    FString Error;
    FMeasurementPreview Out;
    // Gateway converts source NaN/nonfinite samples to JSON null, never a zero.
    const FString Thermal = Preview(TEXT("thermal")).Replace(TEXT("[300,null]"), TEXT("[null,null]"));
    if (TestTrue(TEXT("null overrides optimistic valid mask"), FMeasurementProjectionParser::ParsePreview(
        Thermal, Expected(), TEXT("sensor"), Out, Error)))
    {
        TestFalse(TEXT("NaN projection remains unknown"), Out.Primary[0].IsSet());
        TestFalse(TEXT("NaN projection is invalid"), Out.ValidMask[0]);
    }
    const FString Masked = Preview(TEXT("thermal")).Replace(TEXT("[300,null]"), TEXT("[300,301]"));
    if (TestTrue(TEXT("explicit invalid mask retained"), FMeasurementProjectionParser::ParsePreview(
        Masked, Expected(), TEXT("sensor"), Out, Error)))
    {
        TestFalse(TEXT("masked numeric temperature is not exposed"), Out.Primary[1].IsSet());
    }
    const FString Semantic = Preview(TEXT("semantic")).Replace(TEXT("[42,null]"), TEXT("[null,null]"));
    if (TestTrue(TEXT("missing instance invalidates semantic pair"), FMeasurementProjectionParser::ParsePreview(
        Semantic, Expected(), TEXT("sensor"), Out, Error)))
    {
        TestFalse(TEXT("class is not exposed without instance"), Out.Primary[0].IsSet());
        TestFalse(TEXT("semantic mask reflects missing instance"), Out.ValidMask[0]);
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLingTuSimMeasurementAckTest,
    "LingTuSim.UI.Measurements.Projection.AckRejectionAndIdentity",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FLingTuSimMeasurementAckTest::RunTest(const FString& Parameters)
{
    (void)Parameters;
    bool bApplied = true;
    FString Reason;
    FString Error;
    TSharedPtr<FJsonObject> Settings;
    TestTrue(TEXT("owner rejection is a parsed ACK, not parser failure"), FMeasurementProjectionParser::ParseAck(
        Ack(), Expected(), TEXT("request"), TEXT("sensor"), bApplied, Reason, Settings, Error));
    TestFalse(TEXT("owner rejection never becomes applied"), bApplied);
    TestEqual(TEXT("owner rejection reason retained"), Reason, FString(TEXT("unsupported_rate")));
    TestTrue(TEXT("owner rejection settings retained"), Settings.IsValid());
    const FString AppliedAck = Ack().Replace(TEXT("\"applied\":false"), TEXT("\"applied\":true"));
    TestTrue(TEXT("owner acceptance parses"), FMeasurementProjectionParser::ParseAck(
        AppliedAck, Expected(), TEXT("request"), TEXT("sensor"), bApplied, Reason, Settings, Error));
    TestTrue(TEXT("owner acceptance retained"), bApplied);
    const TPair<FString, FString> Changes[] = {
        {TEXT("\"request_id\":\"request\""), TEXT("\"request_id\":\"other\"")},
        {TEXT("\"sensor_id\":\"sensor\""), TEXT("\"sensor_id\":\"other\"")},
        {TEXT("\"product_session_id\":\"product\""), TEXT("\"product_session_id\":\"other\"")},
        {TEXT("\"session_id\":\"session\""), TEXT("\"session_id\":\"other\"")},
        {TEXT("\"model_generation\":3"), TEXT("\"model_generation\":4")},
        {TEXT("\"reset_generation\":4"), TEXT("\"reset_generation\":5")},
        {TEXT("\"applied\":false"), TEXT("\"applied\":\"false\"")},
    };
    for (const auto& Change : Changes)
    {
        bApplied = true;
        Reason = TEXT("old_reason");
        Settings = MakeShared<FJsonObject>();
        TestFalse(TEXT("unconfirmed ACK fails"), FMeasurementProjectionParser::ParseAck(
            Ack().Replace(*Change.Key, *Change.Value), Expected(), TEXT("request"), TEXT("sensor"),
            bApplied, Reason, Settings, Error));
        TestFalse(TEXT("failed ACK clears old applied"), bApplied);
        TestTrue(TEXT("failed ACK clears old reason"), Reason.IsEmpty());
        TestFalse(TEXT("failed ACK clears old settings"), Settings.IsValid());
    }
    TestFalse(TEXT("HTTP conflict error is not owner ACK"), FMeasurementProjectionParser::ParseAck(
        TEXT(R"({"error":"product_session_mismatch","request_id":"request","sensor_id":"sensor"})"),
        Expected(), TEXT("request"), TEXT("sensor"), bApplied, Reason, Settings, Error));
    return true;
}

#endif

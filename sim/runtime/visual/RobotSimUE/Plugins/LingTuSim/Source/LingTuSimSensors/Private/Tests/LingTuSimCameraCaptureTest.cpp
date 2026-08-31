#include "LingTuSimCameraCaptureSubsystem.h"

#include "Misc/AutomationTest.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "ShowFlags.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuCameraWorldBindingLifecycleTest,
    "LingTuSim.Sensors.CameraCapture.WorldBindingLifecycle",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuCameraWorldBindingLifecycleTest::RunTest(const FString& Parameters)
{
    (void)Parameters;
    using LingTuSim::Sensors::Camera::EWorldBindingAction;
    using LingTuSim::Sensors::Camera::EvaluateWorldBindingAction;

    int32 VisualQueryCount = 0;
    int32 VisualBindingCount = 0;
    const auto QueryVisualBindings = [&VisualQueryCount, &VisualBindingCount]()
    {
        ++VisualQueryCount;
        return VisualBindingCount;
    };

    TestTrue(
        TEXT("Untitled bootstrap world waits before BeginPlay"),
        EvaluateWorldBindingAction(
            EWorldType::Game, false, true, false, QueryVisualBindings)
            == EWorldBindingAction::Wait);
    TestEqual(
        TEXT("bootstrap world does not inspect the Visual subsystem"),
        VisualQueryCount,
        0);

    VisualQueryCount = 0;
    TestTrue(
        TEXT("uninitialized runtime world waits for its lifecycle"),
        EvaluateWorldBindingAction(
            EWorldType::Game, false, false, false, QueryVisualBindings)
            == EWorldBindingAction::Wait);
    TestEqual(
        TEXT("uninitialized world does not inspect the Visual subsystem"),
        VisualQueryCount,
        0);

    VisualQueryCount = 0;
    TestTrue(
        TEXT("runtime world waits while Visual Runtime has no body bindings"),
        EvaluateWorldBindingAction(
            EWorldType::Game, true, true, false, QueryVisualBindings)
            == EWorldBindingAction::Wait);
    TestEqual(
        TEXT("active runtime world inspects Visual bindings once"),
        VisualQueryCount,
        1);

    VisualQueryCount = 0;
    VisualBindingCount = 1;
    TestTrue(
        TEXT("runtime world enters strict camera binding after Visual binding begins"),
        EvaluateWorldBindingAction(
            EWorldType::Game, true, true, false, QueryVisualBindings)
            == EWorldBindingAction::Bind);
    TestEqual(
        TEXT("ready runtime world inspects Visual bindings once"),
        VisualQueryCount,
        1);

    VisualQueryCount = 0;
    TestTrue(
        TEXT("editor-only worlds never own command-line camera binding"),
        EvaluateWorldBindingAction(
            EWorldType::Editor, true, true, false, QueryVisualBindings)
            == EWorldBindingAction::Skip);
    TestEqual(
        TEXT("editor world does not inspect the Visual subsystem"),
        VisualQueryCount,
        0);

    VisualQueryCount = 0;
    TestTrue(
        TEXT("tearing-down runtime world skips camera binding"),
        EvaluateWorldBindingAction(
            EWorldType::Game, true, true, true, QueryVisualBindings)
            == EWorldBindingAction::Skip);
    TestEqual(
        TEXT("tearing-down world does not inspect the Visual subsystem"),
        VisualQueryCount,
        0);
    TestFalse(
        TEXT("bootstrap world cannot time out before BeginPlay"),
        LingTuSim::Sensors::Camera::HasWorldBindingWaitExpired(false, 60.0, 30.0));
    TestFalse(
        TEXT("formal runtime world may wait below the binding timeout"),
        LingTuSim::Sensors::Camera::HasWorldBindingWaitExpired(true, 29.0, 30.0));
    TestTrue(
        TEXT("formal runtime world fails closed when Visual binding wait expires"),
        LingTuSim::Sensors::Camera::HasWorldBindingWaitExpired(true, 30.0, 30.0));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuCameraPlanParsingTest,
    "LingTuSim.Sensors.CameraCapture.PlanParsing",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuCameraPlanParsingTest::RunTest(const FString& Parameters)
{
    const FString Json = TEXT(R"json(
{
  "schema":"lingtu.sim.sensor-plan.v1",
  "session_id":"digest-a",
  "streams":{
    "rgb":[{"sensor_id":"robot.front_rgb","frame_id":"robot/front_camera","parent_frame_id":"robot/base_link","source":"unreal_camera","transport":"camera_shm","rate_hz":30,"width":1280,"height":720,"encoding":"rgb8","extrinsic":{"position_m":[0.2,0.0,0.3],"quaternion_wxyz":[1.0,0.0,0.0,0.0]}}],
    "depth":[{"sensor_id":"robot.front_depth","frame_id":"robot/front_camera","parent_frame_id":"robot/base_link","source":"unreal_camera","transport":"camera_shm","rate_hz":30,"width":640,"height":480,"encoding":"32FC1","unit":"m","extrinsic":{"position_m":[0.2,0.0,0.3],"quaternion_wxyz":[1.0,0.0,0.0,0.0]}}]
  }
})json");
    LingTuSim::Sensors::Camera::FCameraPlan Plan;
    FString Error;
    TestTrue(TEXT("plan parses"), LingTuSim::Sensors::Camera::ParseCameraPlanJson(Json, Plan, Error));
    TestEqual(TEXT("two camera streams"), Plan.Streams.Num(), 2);
    TestEqual(TEXT("depth is output as 16UC1"), Plan.Streams[1].Encoding, FString(TEXT("16UC1")));
    TestEqual(TEXT("parent_frame_id comes from the compiled plan"), Plan.Streams[0].ParentFrameId, FString(TEXT("robot/base_link")));
    TestTrue(TEXT("SensorRig extrinsic is parsed"), Plan.Streams[0].bHasExtrinsic);

    FVector RelativeLocationCm;
    FQuat RelativeRotation;
    bool bUsedPlanExtrinsic = false;
    TestTrue(TEXT("plan mount resolves without preview override"), LingTuSim::Sensors::Camera::ResolveMountTransform(
        Plan.Streams[0],
        false,
        FVector(999.0, 999.0, 999.0),
        FQuat::Identity,
        RelativeLocationCm,
        RelativeRotation,
        bUsedPlanExtrinsic,
        Error));
    TestTrue(TEXT("compiled plan wins when no preview override is requested"), bUsedPlanExtrinsic);
    TestEqual(TEXT("compiled X extrinsic is used"), RelativeLocationCm.X, 20.0);
    TestTrue(TEXT("explicit preview override bypasses the compiled SensorRig pose"), LingTuSim::Sensors::Camera::ResolveMountTransform(
        Plan.Streams[0],
        true,
        FVector(999.0, 999.0, 999.0),
        FQuat::Identity,
        RelativeLocationCm,
        RelativeRotation,
        bUsedPlanExtrinsic,
        Error));
    TestFalse(TEXT("preview override does not claim plan extrinsic"), bUsedPlanExtrinsic);
    TestEqual(TEXT("preview override location is used only when explicit"), RelativeLocationCm.X, 999.0);

    const FString MissingParentJson = TEXT(R"json(
{
  "schema":"lingtu.sim.sensor-plan.v1",
  "session_id":"digest-default",
  "streams":{
    "rgb":[{"sensor_id":"thunder_01.front_rgb","frame_id":"thunder_01/front_camera","source":"unreal_camera","transport":"camera_shm","rate_hz":30,"width":64,"height":48,"encoding":"rgb8","extrinsic":{"position_m":[0.0,0.0,0.0],"quaternion_wxyz":[1.0,0.0,0.0,0.0]}}],
    "imu":[{"sensor_id":"thunder_01.imu","source":"mujoco_imu","transport":"typed_dds"}]
  }
})json");
    LingTuSim::Sensors::Camera::FCameraPlan InvalidMountPlan;
    TestFalse(TEXT("old camera plan without parent_frame_id fails closed"), LingTuSim::Sensors::Camera::ParseCameraPlanJson(
        MissingParentJson, InvalidMountPlan, Error));
    TestTrue(TEXT("failure names parent_frame_id"), Error.Contains(TEXT("parent_frame_id")));

    const FString MissingExtrinsicJson = TEXT(R"json(
{
  "schema":"lingtu.sim.sensor-plan.v1",
  "session_id":"digest-default",
  "streams":{
    "rgb":[{"sensor_id":"thunder_01.front_rgb","frame_id":"thunder_01/front_camera","parent_frame_id":"thunder_01/base_link","source":"unreal_camera","transport":"camera_shm","rate_hz":30,"width":64,"height":48,"encoding":"rgb8"}],
    "imu":[{"sensor_id":"thunder_01.imu","source":"mujoco_imu","transport":"typed_dds"}]
  }
})json");
    TestFalse(TEXT("camera plan without explicit SensorRig extrinsic fails closed"), LingTuSim::Sensors::Camera::ParseCameraPlanJson(
        MissingExtrinsicJson, InvalidMountPlan, Error));
    TestTrue(TEXT("failure names extrinsic"), Error.Contains(TEXT("extrinsic")));

    const FString NonCameraJson = TEXT(R"json(
{
  "schema":"lingtu.sim.sensor-plan.v1",
  "session_id":"digest-default",
  "streams":{
    "rgb":[{"sensor_id":"thunder_01.front_rgb","frame_id":"thunder_01/front_camera","parent_frame_id":"thunder_01/base_link","source":"unreal_camera","transport":"camera_shm","rate_hz":30,"width":64,"height":48,"encoding":"rgb8","extrinsic":{"position_m":[0.0,0.0,0.0],"quaternion_wxyz":[1.0,0.0,0.0,0.0]}}],
    "imu":[{"sensor_id":"thunder_01.imu","source":"mujoco_imu","transport":"typed_dds"}]
  }
})json");
    LingTuSim::Sensors::Camera::FCameraPlan NonCameraPlan;
    TestTrue(TEXT("valid camera plan with non-camera streams parses"), LingTuSim::Sensors::Camera::ParseCameraPlanJson(
        NonCameraJson, NonCameraPlan, Error));
    TestEqual(TEXT("non-camera stream is counted in total plan streams only"), NonCameraPlan.TotalStreamCount, 2);
    TestEqual(TEXT("only camera streams bind to camera producer"), NonCameraPlan.Streams.Num(), 1);

    const FString AllocationJson = TEXT(R"json(
{
  "schema":"lingtu.sim.run-allocation.v1",
  "run_id":"run-a",
  "session_id":"digest-a",
  "log_dir":"D:\\run-evidence",
  "shm":{"robot.front_rgb":"Local\\rgb","robot.front_depth":"Local\\depth"}
})json");
    LingTuSim::Sensors::Camera::FRunAllocation Allocation;
    TestTrue(TEXT("allocation parses"), LingTuSim::Sensors::Camera::ParseRunAllocationJson(AllocationJson, Allocation, Error));
    TestEqual(TEXT("RGB mapping is explicit"), Allocation.SharedMemoryBySensor.FindRef(TEXT("robot.front_rgb")), FString(TEXT("Local\\rgb")));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuCameraOpticalMountTest,
    "LingTuSim.Sensors.CameraCapture.OpticalMount",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuCameraOpticalMountTest::RunTest(const FString& Parameters)
{
    LingTuSim::Sensors::Camera::FStreamPlan Stream;
    Stream.SensorId = TEXT("thunder_01.front_rgb");
    Stream.ExtrinsicTranslationMeters = FVector(0.35, 0.0, 0.15);
    // ROS optical frame in an RH x-forward/y-left/z-up parent:
    // optical +X is right, +Y is down, and +Z is forward.
    Stream.ExtrinsicRotation = FQuat(-0.5, 0.5, -0.5, 0.5);
    Stream.bHasExtrinsic = true;

    FVector RelativeLocationCm;
    FQuat RelativeRotation;
    bool bUsedPlanExtrinsic = false;
    FString Error;
    TestTrue(TEXT("front optical mount resolves"), LingTuSim::Sensors::Camera::ResolveMountTransform(
        Stream,
        false,
        FVector::ZeroVector,
        FQuat::Identity,
        RelativeLocationCm,
        RelativeRotation,
        bUsedPlanExtrinsic,
        Error));
    TestTrue(TEXT("compiled optical extrinsic is used"), bUsedPlanExtrinsic);
    TestTrue(TEXT("camera is mounted in front and above base"), RelativeLocationCm.Equals(
        FVector(35.0, 0.0, 15.0), UE_KINDA_SMALL_NUMBER));

    const FVector CaptureForward = RelativeRotation.RotateVector(FVector::ForwardVector);
    const FVector CaptureRight = RelativeRotation.RotateVector(FVector::RightVector);
    const FVector CaptureUp = RelativeRotation.RotateVector(FVector::UpVector);
    TestTrue(TEXT("SceneCapture +X faces robot forward"), CaptureForward.Equals(
        FVector::ForwardVector, UE_KINDA_SMALL_NUMBER));
    TestTrue(TEXT("SceneCapture +Y faces robot right"), CaptureRight.Equals(
        FVector::RightVector, UE_KINDA_SMALL_NUMBER));
    TestTrue(TEXT("SceneCapture +Z faces robot up"), CaptureUp.Equals(
        FVector::UpVector, UE_KINDA_SMALL_NUMBER));

    const FVector DirectionBackToBase = -RelativeLocationCm.GetSafeNormal();
    TestTrue(TEXT("camera does not look back toward the robot body"),
        FVector::DotProduct(CaptureForward, DirectionBackToBase) < 0.0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuCameraPixelConversionTest,
    "LingTuSim.Sensors.CameraCapture.PixelConversion",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuCameraPixelConversionTest::RunTest(const FString& Parameters)
{
    TArray<FColor> ColorPixels;
    ColorPixels.Add(FColor(1, 2, 3, 4));
    TArray<uint8> Payload;
    FString Error;
    TestTrue(TEXT("RGB conversion succeeds"), LingTuSim::Sensors::Camera::ConvertColorPixels(
        ColorPixels, 1, 1, TEXT("rgba8"), Payload, Error));
    TestEqual(TEXT("RGBA payload has four bytes"), Payload.Num(), 4);
    TestEqual(TEXT("R channel is preserved"), Payload[0], static_cast<uint8>(1));
    TestEqual(TEXT("G channel is preserved"), Payload[1], static_cast<uint8>(2));
    TestEqual(TEXT("B channel is preserved"), Payload[2], static_cast<uint8>(3));
    TestEqual(TEXT("A channel is preserved"), Payload[3], static_cast<uint8>(4));

    TArray<FLinearColor> DepthPixels;
    DepthPixels.Add(FLinearColor(1.234f, 0.0f, 0.0f, 1.0f));
    TestTrue(TEXT("depth conversion succeeds"), LingTuSim::Sensors::Camera::ConvertDepthPixels(
        DepthPixels, 1, 1, 0.001, Payload, Error));
    TestEqual(TEXT("depth output is one uint16 pixel"), Payload.Num(), 2);
    const uint16 DepthMm = *reinterpret_cast<const uint16*>(Payload.GetData());
    TestEqual(TEXT("meters convert to millimeters"), DepthMm, static_cast<uint16>(1234));

    TArray<float> RawDepthValues;
    RawDepthValues.Add(2.5f);
    TestTrue(TEXT("raw R32F depth conversion succeeds"),
        LingTuSim::Sensors::Camera::ConvertDepthValues(
            RawDepthValues, 1, 1, 0.001, Payload, Error));
    const uint16 RawDepthMm = *reinterpret_cast<const uint16*>(Payload.GetData());
    TestEqual(TEXT("raw R32F meters convert to millimeters"),
        RawDepthMm, static_cast<uint16>(2500));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuCameraSharedColorDepthPairingTest,
    "LingTuSim.Sensors.CameraCapture.SharedColorDepthPairing",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuCameraSharedColorDepthPairingTest::RunTest(const FString& Parameters)
{
    (void)Parameters;
    using LingTuSim::Sensors::Camera::EStreamKind;
    using LingTuSim::Sensors::Camera::FStreamPlan;

    FStreamPlan Color;
    Color.SensorId = TEXT("thunder_01.front_rgb");
    Color.FrameId = TEXT("front_rgb_optical");
    Color.ParentFrameId = TEXT("thunder_01/base_link");
    Color.SessionId = TEXT("digest-a");
    Color.Encoding = TEXT("rgb8");
    Color.Source = TEXT("unreal_camera");
    Color.Transport = TEXT("camera_shm");
    Color.Kind = EStreamKind::Color;
    Color.Width = 640;
    Color.Height = 480;
    Color.RateHz = 30.0;
    Color.bHasExtrinsic = true;
    Color.ExtrinsicTranslationMeters = FVector(0.30, 0.0, 0.10);
    Color.ExtrinsicRotation = FQuat::Identity;

    FStreamPlan Depth = Color;
    Depth.SensorId = TEXT("thunder_01.front_depth");
    Depth.FrameId = TEXT("front_depth_optical");
    Depth.Encoding = TEXT("16UC1");
    Depth.Kind = EStreamKind::Depth;

    TArray<FStreamPlan> Streams{Color, Depth};
    const TArray<LingTuSim::Sensors::Camera::FSharedColorDepthPair> Pairs =
        LingTuSim::Sensors::Camera::ResolveSharedColorDepthCapturePairs(Streams);
    TestEqual(TEXT("one compatible camera pair shares one scene render"), Pairs.Num(), 1);
    TestEqual(TEXT("the RGB stream is the pair leader"), Pairs[0].ColorStreamIndex, 0);
    TestEqual(TEXT("the depth stream is the pair follower"), Pairs[0].DepthStreamIndex, 1);

    Streams[1].Width = 320;
    TestEqual(
        TEXT("different render geometry falls back to independent captures"),
        LingTuSim::Sensors::Camera::ResolveSharedColorDepthCapturePairs(Streams).Num(),
        0);

    Streams[1] = Depth;
    FStreamPlan AmbiguousDepth = Depth;
    AmbiguousDepth.SensorId = TEXT("thunder_01.front_depth_duplicate");
    Streams.Add(AmbiguousDepth);
    TestEqual(
        TEXT("ambiguous matching never guesses a shared capture"),
        LingTuSim::Sensors::Camera::ResolveSharedColorDepthCapturePairs(Streams).Num(),
        0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuCameraSharedColorDepthDecodeTest,
    "LingTuSim.Sensors.CameraCapture.SharedColorDepthDecode",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuCameraSharedColorDepthDecodeTest::RunTest(const FString& Parameters)
{
    (void)Parameters;
    TArray<FLinearColor> SharedPixels;
    // SceneCapture stores linear SceneColor in RGB and SceneDepth in Unreal
    // world units (centimetres) in alpha.
    SharedPixels.Add(FLinearColor(1.0f, 0.0f, 0.0f, 123.4f));
    TArray<FColor> ColorPixels;
    TArray<float> DepthMeters;
    FString Error;
    TestTrue(
        TEXT("one shared pixel decodes into RGB and metric depth"),
        LingTuSim::Sensors::Camera::DecodeSharedColorDepthPixels(
            SharedPixels, 1, 1, ColorPixels, DepthMeters, Error));
    TestEqual(TEXT("shared RGB decode emits one pixel"), ColorPixels.Num(), 1);
    TestEqual(TEXT("shared depth decode emits one value"), DepthMeters.Num(), 1);
    TestEqual(TEXT("linear red becomes red rgb8"), ColorPixels[0].R, static_cast<uint8>(255));
    TestEqual(TEXT("zero green remains zero"), ColorPixels[0].G, static_cast<uint8>(0));
    TestTrue(TEXT("centimetres become metres"), FMath::IsNearlyEqual(DepthMeters[0], 1.234f, 1e-4f));

    TArray<uint8> DepthPayload;
    TestTrue(
        TEXT("decoded metric depth preserves the existing 16UC1 contract"),
        LingTuSim::Sensors::Camera::ConvertDepthValues(
            DepthMeters, 1, 1, 0.001, DepthPayload, Error));
    TestEqual(
        TEXT("shared depth becomes 1234 millimetres"),
        *reinterpret_cast<const uint16*>(DepthPayload.GetData()),
        static_cast<uint16>(1234));

    TArray<FFloat16Color> HalfPixels;
    HalfPixels.Add(FFloat16Color(FLinearColor(1.0f, 0.0f, 0.0f, 123.4f)));
    TArray<uint8> ColorPayload;
    DepthPayload.Reset();
    TestTrue(
        TEXT("packed RGBA16F is encoded directly into both transport payloads"),
        LingTuSim::Sensors::Camera::EncodeSharedColorDepthHalfPixels(
            HalfPixels,
            1,
            1,
            TEXT("rgb8"),
            0.001,
            ColorPayload,
            DepthPayload,
            Error));
    TestEqual(TEXT("direct RGB payload has three channels"), ColorPayload.Num(), 3);
    TestEqual(TEXT("direct RGB payload preserves red"), ColorPayload[0], static_cast<uint8>(255));
    TestEqual(TEXT("direct RGB payload preserves green"), ColorPayload[1], static_cast<uint8>(0));
    TestEqual(TEXT("direct depth payload has one uint16"), DepthPayload.Num(), 2);
    TestEqual(
        TEXT("direct shared depth becomes 1234 millimetres"),
        *reinterpret_cast<const uint16*>(DepthPayload.GetData()),
        static_cast<uint16>(1234));

    SharedPixels[0].A = -1.0f;
    TestFalse(
        TEXT("negative shared SceneDepth fails closed"),
        LingTuSim::Sensors::Camera::DecodeSharedColorDepthPixels(
            SharedPixels, 1, 1, ColorPixels, DepthMeters, Error));
    TestTrue(TEXT("failed shared decode clears both outputs"),
        ColorPixels.IsEmpty() && DepthMeters.IsEmpty());
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuCameraCaptureScheduleTest,
    "LingTuSim.Sensors.CameraCapture.Schedule",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuCameraCaptureScheduleTest::RunTest(const FString& Parameters)
{
    double Accumulator = 0.0;
    TestFalse(TEXT("30Hz does not capture before period"), LingTuSim::Sensors::Camera::AdvanceSchedule(
        0.02f, 30.0, Accumulator));
    TestTrue(TEXT("30Hz captures at period"), LingTuSim::Sensors::Camera::AdvanceSchedule(
        0.014f, 30.0, Accumulator));
    TestFalse(TEXT("invalid rate fails closed"), LingTuSim::Sensors::Camera::AdvanceSchedule(
        0.1f, 0.0, Accumulator));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuCameraDeferredCapturePromotionTest,
    "LingTuSim.Sensors.CameraCapture.DeferredPromotion",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuCameraDeferredCapturePromotionTest::RunTest(const FString& Parameters)
{
    (void)Parameters;
    using LingTuSim::Sensors::Camera::EDeferredCaptureAction;
    using LingTuSim::Sensors::Camera::EvaluateDeferredCapturePromotion;

    LingTuSim::Sensors::Camera::FTruthSampleStamp Stamp;
    Stamp.SessionId = TEXT("digest-a");
    Stamp.ModelGeneration = 7;
    Stamp.ResetGeneration = 3;
    Stamp.TruthSequence = 41;
    Stamp.SimTimeNs = 1'250'000'000;
    FString Error;

    TestTrue(
        TEXT("same-frame deferred capture waits for end-of-frame rendering"),
        EvaluateDeferredCapturePromotion(
            Stamp, TEXT("digest-a"), 7, 3, 100, 100, 10.0, 10.01, 0, 3, 0.5, Error)
            == EDeferredCaptureAction::Wait);
    TestTrue(
        TEXT("next-frame deferred capture may promote to asynchronous readback"),
        EvaluateDeferredCapturePromotion(
            Stamp, TEXT("digest-a"), 7, 3, 100, 101, 10.0, 10.02, 0, 3, 0.5, Error)
            == EDeferredCaptureAction::Promote);
    TestTrue(
        TEXT("a reset-generation capture is discarded instead of published"),
        EvaluateDeferredCapturePromotion(
            Stamp, TEXT("digest-a"), 7, 4, 100, 101, 10.0, 10.02, 0, 3, 0.5, Error)
            == EDeferredCaptureAction::Discard);
    TestTrue(
        TEXT("a session change discards the old deferred capture"),
        EvaluateDeferredCapturePromotion(
            Stamp, TEXT("digest-b"), 7, 3, 100, 101, 10.0, 10.02, 0, 3, 0.5, Error)
            == EDeferredCaptureAction::Discard);
    TestTrue(
        TEXT("a model-generation change discards the old deferred capture"),
        EvaluateDeferredCapturePromotion(
            Stamp, TEXT("digest-a"), 8, 3, 100, 101, 10.0, 10.02, 0, 3, 0.5, Error)
            == EDeferredCaptureAction::Discard);
    TestTrue(
        TEXT("a full readback ring waits while inside the bounded deadline"),
        EvaluateDeferredCapturePromotion(
            Stamp, TEXT("digest-a"), 7, 3, 100, 101, 10.0, 10.20, 3, 3, 0.5, Error)
            == EDeferredCaptureAction::Wait);
    TestTrue(
        TEXT("a first-frame startup budget may outlive the steady-state deadline"),
        EvaluateDeferredCapturePromotion(
            Stamp, TEXT("digest-a"), 7, 3, 100, 101, 10.0, 11.50, 0, 3, 30.0, Error)
            == EDeferredCaptureAction::Promote);
    TestTrue(
        TEXT("the first-frame startup budget remains bounded"),
        EvaluateDeferredCapturePromotion(
            Stamp, TEXT("digest-a"), 7, 3, 100, 101, 10.0, 40.0, 0, 3, 30.0, Error)
            == EDeferredCaptureAction::Fail);
    TestTrue(TEXT("startup timeout reports its exact stage and age"),
        Error.Contains(TEXT("stage=DeferredCapture"))
            && Error.Contains(TEXT("age=30.000")));
    TestTrue(
        TEXT("a deferred capture that exceeds its deadline fails closed"),
        EvaluateDeferredCapturePromotion(
            Stamp, TEXT("digest-a"), 7, 3, 100, 101, 10.0, 10.50, 3, 3, 0.5, Error)
            == EDeferredCaptureAction::Fail);
    TestTrue(TEXT("timeout failure is explicit"), Error.Contains(TEXT("deadline")));
    TestTrue(
        TEXT("game-frame regression fails closed"),
        EvaluateDeferredCapturePromotion(
            Stamp, TEXT("digest-a"), 7, 3, 100, 99, 10.0, 10.01, 0, 3, 0.5, Error)
            == EDeferredCaptureAction::Fail);
    TestTrue(TEXT("frame regression failure is explicit"), Error.Contains(TEXT("regressed")));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuCameraReadbackDeadlinePolicyTest,
    "LingTuSim.Sensors.CameraCapture.ReadbackDeadlinePolicy",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuCameraReadbackDeadlinePolicyTest::RunTest(const FString& Parameters)
{
    (void)Parameters;
    using LingTuSim::Sensors::Camera::EReadbackDeadlineAction;
    using LingTuSim::Sensors::Camera::EReadbackDeadlineStage;
    using LingTuSim::Sensors::Camera::EvaluateReadbackStageDeadline;
    using LingTuSim::Sensors::Camera::SelectCapturePipelineTimeoutSeconds;

    constexpr double StartupTimeoutSeconds = 30.0;
    constexpr double SteadyTimeoutSeconds = 1.0;
    const double FirstJobTimeout = SelectCapturePipelineTimeoutSeconds(
        0, StartupTimeoutSeconds, SteadyTimeoutSeconds);
    const double SteadyJobTimeout = SelectCapturePipelineTimeoutSeconds(
        1, StartupTimeoutSeconds, SteadyTimeoutSeconds);
    TestEqual(TEXT("a pre-ACTIVE job freezes the startup budget"),
        FirstJobTimeout, StartupTimeoutSeconds);
    TestEqual(TEXT("an ACTIVE stream freezes the one-second steady budget"),
        SteadyJobTimeout, SteadyTimeoutSeconds);
    TestEqual(TEXT("a reset generation receives the startup budget again"),
        SelectCapturePipelineTimeoutSeconds(0, StartupTimeoutSeconds, SteadyTimeoutSeconds),
        StartupTimeoutSeconds);

    FString Error;
    TestTrue(TEXT("EnqueuePending waits below its startup deadline"),
        EvaluateReadbackStageDeadline(
            EReadbackDeadlineStage::EnqueuePending,
            10.0,
            39.999,
            FirstJobTimeout,
            Error) == EReadbackDeadlineAction::Wait);
    TestTrue(TEXT("EnqueuePending fails exactly at its startup deadline"),
        EvaluateReadbackStageDeadline(
            EReadbackDeadlineStage::EnqueuePending,
            10.0,
            40.0,
            FirstJobTimeout,
            Error) == EReadbackDeadlineAction::Fail);
    TestTrue(TEXT("EnqueuePending reports its stage and exact age"),
        Error.Contains(TEXT("stage=EnqueuePending"))
            && Error.Contains(TEXT("age=30.000")));
    TestTrue(TEXT("EnqueuePending waits below its steady deadline"),
        EvaluateReadbackStageDeadline(
            EReadbackDeadlineStage::EnqueuePending,
            50.0,
            50.999,
            SteadyJobTimeout,
            Error) == EReadbackDeadlineAction::Wait);
    TestTrue(TEXT("EnqueuePending fails exactly at its steady deadline"),
        EvaluateReadbackStageDeadline(
            EReadbackDeadlineStage::EnqueuePending,
            50.0,
            51.0,
            SteadyJobTimeout,
            Error) == EReadbackDeadlineAction::Fail);

    TestTrue(TEXT("GpuPending waits below its startup deadline"),
        EvaluateReadbackStageDeadline(
            EReadbackDeadlineStage::GpuPending,
            20.0,
            49.999,
            FirstJobTimeout,
            Error) == EReadbackDeadlineAction::Wait);
    TestTrue(TEXT("GpuPending fails exactly at its startup deadline"),
        EvaluateReadbackStageDeadline(
            EReadbackDeadlineStage::GpuPending,
            20.0,
            50.0,
            FirstJobTimeout,
            Error) == EReadbackDeadlineAction::Fail);
    TestTrue(TEXT("GpuPending uses its independent steady submission clock"),
        EvaluateReadbackStageDeadline(
            EReadbackDeadlineStage::GpuPending,
            20.0,
            20.999,
            SteadyJobTimeout,
            Error) == EReadbackDeadlineAction::Wait);
    TestTrue(TEXT("GpuPending fails exactly at its steady deadline"),
        EvaluateReadbackStageDeadline(
            EReadbackDeadlineStage::GpuPending,
            20.0,
            21.0,
            SteadyJobTimeout,
            Error) == EReadbackDeadlineAction::Fail);
    TestTrue(TEXT("GpuPending reports its stage and exact age"),
        Error.Contains(TEXT("stage=GpuPending"))
            && Error.Contains(TEXT("age=1.000")));

    TestTrue(TEXT("CpuDecodePending waits below its steady deadline"),
        EvaluateReadbackStageDeadline(
            EReadbackDeadlineStage::CpuDecodePending,
            30.0,
            30.999,
            SteadyJobTimeout,
            Error) == EReadbackDeadlineAction::Wait);
    TestTrue(TEXT("CpuDecodePending fails exactly at its steady deadline"),
        EvaluateReadbackStageDeadline(
            EReadbackDeadlineStage::CpuDecodePending,
            30.0,
            31.0,
            SteadyJobTimeout,
            Error) == EReadbackDeadlineAction::Fail);
    TestTrue(TEXT("CpuDecodePending reports its own stage"),
        Error.Contains(TEXT("stage=CpuDecodePending"))
            && Error.Contains(TEXT("age=1.000")));

    TestTrue(TEXT("CpuReady completes exactly at the steady deadline"),
        EvaluateReadbackStageDeadline(
            EReadbackDeadlineStage::CpuReady,
            100.0,
            101.0,
            SteadyJobTimeout,
            Error) == EReadbackDeadlineAction::Complete);
    TestTrue(TEXT("CpuReady never becomes a GPU timeout"),
        EvaluateReadbackStageDeadline(
            EReadbackDeadlineStage::CpuReady,
            0.0,
            1000.0,
            SteadyJobTimeout,
            Error) == EReadbackDeadlineAction::Complete);
    TestTrue(TEXT("CpuReady completion has no timeout error"), Error.IsEmpty());
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuCameraMainRendererOptionTest,
    "LingTuSim.Sensors.CameraCapture.MainRendererOption",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuCameraMainRendererOptionTest::RunTest(const FString& Parameters)
{
    (void)Parameters;
    using LingTuSim::Sensors::Camera::EStreamKind;
    using LingTuSim::Sensors::Camera::ShouldRenderCameraCaptureInMainRenderer;

    TestFalse(
        TEXT("depth main-renderer integration defaults off"),
        ShouldRenderCameraCaptureInMainRenderer(
            EStreamKind::Depth, ESceneCaptureSource::SCS_SceneDepth, false));
    TestTrue(
        TEXT("explicit depth option accepts only SceneDepth"),
        ShouldRenderCameraCaptureInMainRenderer(
            EStreamKind::Depth, ESceneCaptureSource::SCS_SceneDepth, true));
    TestFalse(
        TEXT("color FinalColorLDR never enters the main-renderer depth path"),
        ShouldRenderCameraCaptureInMainRenderer(
            EStreamKind::Color, ESceneCaptureSource::SCS_FinalColorLDR, true));
    TestFalse(
        TEXT("a depth stream with an incompatible source fails the option gate"),
        ShouldRenderCameraCaptureInMainRenderer(
            EStreamKind::Depth, ESceneCaptureSource::SCS_FinalColorLDR, true));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuCameraDepthShowFlagsTest,
    "LingTuSim.Sensors.CameraCapture.DepthShowFlags",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuCameraDepthShowFlagsTest::RunTest(const FString& Parameters)
{
    (void)Parameters;
    using LingTuSim::Sensors::Camera::ConfigureCameraCaptureShowFlags;
    using LingTuSim::Sensors::Camera::EStreamKind;

    FEngineShowFlags ColorFlags(EShowFlagInitMode::ESFIM_Game);
    ColorFlags.SetLumenGlobalIllumination(true);
    ColorFlags.SetLumenReflections(true);
    ColorFlags.SetDynamicShadows(true);
    ColorFlags.SetLighting(true);
    ColorFlags.SetPostProcessing(true);
    ConfigureCameraCaptureShowFlags(EStreamKind::Color, ColorFlags);
    TestTrue(TEXT("RGB retains Lumen GI"), ColorFlags.LumenGlobalIllumination);
    TestTrue(TEXT("RGB retains Lumen reflections"), ColorFlags.LumenReflections);
    TestTrue(TEXT("RGB retains dynamic shadows"), ColorFlags.DynamicShadows);
    TestTrue(TEXT("RGB retains lighting"), ColorFlags.Lighting);
    TestTrue(TEXT("RGB retains post processing"), ColorFlags.PostProcessing);

    FEngineShowFlags DepthFlags(EShowFlagInitMode::ESFIM_Game);
    DepthFlags.SetLumenGlobalIllumination(true);
    DepthFlags.SetLumenReflections(true);
    DepthFlags.SetDynamicShadows(true);
    DepthFlags.SetLighting(true);
    DepthFlags.SetPostProcessing(true);
    DepthFlags.SetAtmosphere(true);
    ConfigureCameraCaptureShowFlags(EStreamKind::Depth, DepthFlags);
    TestFalse(TEXT("depth disables Lumen GI"), DepthFlags.LumenGlobalIllumination);
    TestFalse(TEXT("depth disables Lumen reflections"), DepthFlags.LumenReflections);
    TestFalse(TEXT("depth disables dynamic shadows"), DepthFlags.DynamicShadows);
    TestFalse(TEXT("depth disables lighting"), DepthFlags.Lighting);
    TestFalse(TEXT("depth disables post processing"), DepthFlags.PostProcessing);
    TestFalse(TEXT("depth disables atmosphere"), DepthFlags.Atmosphere);
    TestTrue(TEXT("depth still renders static geometry"), DepthFlags.StaticMeshes);
    TestTrue(TEXT("depth still renders instanced static geometry"), DepthFlags.InstancedStaticMeshes);
    TestTrue(TEXT("depth still renders skeletal geometry"), DepthFlags.SkeletalMeshes);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuCameraReadinessEvidenceCadenceTest,
    "LingTuSim.Sensors.CameraCapture.ReadinessEvidenceCadence",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuCameraReadinessEvidenceCadenceTest::RunTest(const FString& Parameters)
{
    (void)Parameters;
    LingTuSim::Sensors::Camera::FReadinessEvidenceCadence Cadence(0.5, 0.1);

    TestTrue(TEXT("initial readiness evidence is due"), Cadence.ShouldAttempt(10.0));
    Cadence.RecordAttempt(10.0, true);
    TestFalse(TEXT("ordinary frames do not rewrite evidence before heartbeat"), Cadence.ShouldAttempt(10.49));
    TestTrue(TEXT("heartbeat refreshes counters at the configured cadence"), Cadence.ShouldAttempt(10.5));
    Cadence.RecordAttempt(10.5, true);

    Cadence.MarkDirty();
    TestTrue(TEXT("a readiness state transition is published immediately"), Cadence.ShouldAttempt(10.51));
    Cadence.RecordAttempt(10.51, false);
    TestFalse(TEXT("a transient sharing failure is rate limited"), Cadence.ShouldAttempt(10.59));
    TestTrue(TEXT("dirty evidence retries without failing the camera stream"), Cadence.ShouldAttempt(10.61));
    Cadence.RecordAttempt(10.61, true);
    TestFalse(TEXT("a successful retry clears the dirty state"), Cadence.ShouldAttempt(10.7));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuCameraAppliedTruthSampleTest,
    "LingTuSim.Sensors.CameraCapture.AppliedTruthSample",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuCameraAppliedTruthSampleTest::RunTest(const FString& Parameters)
{
    (void)Parameters;
    LingTuSim::FSnapshotEnvelope Snapshot;
    Snapshot.SessionId = TEXT("digest-a");
    Snapshot.ModelGeneration = 7;
    Snapshot.ResetGeneration = 3;
    Snapshot.Sequence = 41;
    Snapshot.SimTimeNs = 1'250'000'000;

    LingTuSim::Sensors::Camera::FTruthSampleStamp Stamp;
    FString Error;
    TestTrue(
        TEXT("same-session latest-applied Visual truth is accepted"),
        LingTuSim::Sensors::Camera::ResolveAppliedTruthSample(
            Snapshot, TEXT("digest-a"), 7, 3, Stamp, Error));
    TestEqual(TEXT("session identity is latched"), Stamp.SessionId, FString(TEXT("digest-a")));
    TestEqual(TEXT("model generation is latched"), Stamp.ModelGeneration, 7ULL);
    TestEqual(TEXT("reset generation is latched"), Stamp.ResetGeneration, 3ULL);
    TestEqual(TEXT("truth sequence is latched"), Stamp.TruthSequence, 41ULL);
    TestEqual(TEXT("simulation time is latched"), Stamp.SimTimeNs, 1'250'000'000LL);

    TestFalse(
        TEXT("session mismatch is rejected"),
        LingTuSim::Sensors::Camera::ResolveAppliedTruthSample(
            Snapshot, TEXT("digest-b"), 7, 3, Stamp, Error));
    TestTrue(TEXT("rejection clears stale session truth"), Stamp.SessionId.IsEmpty());
    TestEqual(TEXT("rejection clears stale truth sequence"), Stamp.TruthSequence, 0ULL);
    TestEqual(TEXT("rejection clears stale simulation time"), Stamp.SimTimeNs, 0LL);

    TestFalse(
        TEXT("model generation mismatch is rejected"),
        LingTuSim::Sensors::Camera::ResolveAppliedTruthSample(
            Snapshot, TEXT("digest-a"), 8, 3, Stamp, Error));
    TestFalse(
        TEXT("reset generation mismatch is rejected"),
        LingTuSim::Sensors::Camera::ResolveAppliedTruthSample(
            Snapshot, TEXT("digest-a"), 7, 4, Stamp, Error));

    Snapshot.SimTimeNs = -1;
    TestFalse(
        TEXT("negative simulation time is rejected instead of replaced by wall clock"),
        LingTuSim::Sensors::Camera::ResolveAppliedTruthSample(
            Snapshot, TEXT("digest-a"), 7, 3, Stamp, Error));
    Snapshot.SimTimeNs = 0;
    Snapshot.Sequence = 0;
    TestTrue(
        TEXT("zero is a legal first simulation timestamp and truth sequence"),
        LingTuSim::Sensors::Camera::ResolveAppliedTruthSample(
            Snapshot, TEXT("digest-a"), 7, 3, Stamp, Error));
    TestEqual(TEXT("zero simulation time remains exact"), Stamp.SimTimeNs, 0LL);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuCameraCaptureValidationTest,
    "LingTuSim.Sensors.CameraCapture.Validation",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuCameraCaptureValidationTest::RunTest(const FString& Parameters)
{
    LingTuSim::Sensors::Camera::FCameraPlan Plan;
    FString Error;
    const FString InvalidJson = TEXT(R"json(
{"schema":"lingtu.sim.sensor-plan.v1","session_id":"d","streams":{"rgb":[{"sensor_id":"r","frame_id":"r/c","parent_frame_id":"r/base_link","source":"fake","transport":"camera_shm","rate_hz":30,"width":2,"height":2,"encoding":"rgb8","extrinsic":{"position_m":[0,0,0],"quaternion_wxyz":[1,0,0,0]}}]}}
)json");
    TestFalse(TEXT("non-Unreal source fails closed"), LingTuSim::Sensors::Camera::ParseCameraPlanJson(
        InvalidJson, Plan, Error));
    TestTrue(TEXT("failure explains source mismatch"), Error.Contains(TEXT("unreal_camera")));

    TArray<LingTuSim::Sensors::Camera::FStreamReadiness> Streams;
    LingTuSim::Sensors::Camera::FStreamReadiness Rgb;
    Rgb.SensorId = TEXT("thunder_01.front_rgb");
    Rgb.State = TEXT("ACTIVE");
    Rgb.PublishedFrames = 1;
    Rgb.LastSampleTruthSequence = 19;
    Rgb.LastSampleSimTimeNs = 750'000'000;
    Streams.Add(Rgb);
    const FString EvidenceJson = LingTuSim::Sensors::Camera::BuildReadinessEvidenceJson(
        TEXT("digest-a"), 0, 42, Streams);
    TSharedPtr<FJsonObject> Root;
    TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(EvidenceJson);
    TestTrue(TEXT("readiness evidence is valid JSON"), FJsonSerializer::Deserialize(Reader, Root));
    TestTrue(TEXT("readiness root is an object"), Root.IsValid());
    if (!Root.IsValid())
    {
        return false;
    }
    TestEqual(TEXT("root has only the ExternalRuntimeEvidence fields"), Root->Values.Num(), 9);
    TestEqual(TEXT("generation zero is serialized as legal"), static_cast<uint64>(Root->GetIntegerField(TEXT("model_generation"))), 0ULL);
    TestEqual(TEXT("reset_generation is preserved"), static_cast<uint64>(Root->GetIntegerField(TEXT("reset_generation"))), 42ULL);
    TestEqual(TEXT("source_id is robotsimue-camera"), Root->GetStringField(TEXT("source_id")), FString(TEXT("robotsimue-camera")));
    TestEqual(TEXT("basis is camera SHM rendered frame"), Root->GetStringField(TEXT("basis")), FString(TEXT("real_rendered_frame_to_camera_shm")));
    const TSharedPtr<FJsonObject>* Visual = nullptr;
    TestTrue(TEXT("visual is an object"), Root->TryGetObjectField(TEXT("visual"), Visual));
    TestTrue(TEXT("visual object exists"), Visual != nullptr && Visual->IsValid());
    if (Visual != nullptr && Visual->IsValid())
    {
        TestEqual(TEXT("visual has state only"), (*Visual)->Values.Num(), 1);
        TestEqual(TEXT("camera producer never marks Visual ACTIVE"), (*Visual)->GetStringField(TEXT("state")), FString(TEXT("PREPARED")));
    }
    const TSharedPtr<FJsonObject>* Sensors = nullptr;
    TestTrue(TEXT("sensors is an object"), Root->TryGetObjectField(TEXT("sensors"), Sensors));
    TestTrue(TEXT("sensors object exists"), Sensors != nullptr && Sensors->IsValid());
    if (Sensors != nullptr && Sensors->IsValid())
    {
        TestEqual(TEXT("sensors summary has camera_streams and overall only"), (*Sensors)->Values.Num(), 2);
        TestEqual(TEXT("camera streams can become active"), (*Sensors)->GetStringField(TEXT("camera_streams")), FString(TEXT("ACTIVE")));
        TestEqual(TEXT("overall is based only on camera streams"), (*Sensors)->GetStringField(TEXT("overall")), FString(TEXT("ACTIVE")));
    }
    const TArray<TSharedPtr<FJsonValue>>* Declarations = nullptr;
    TestTrue(TEXT("streams is a list"), Root->TryGetArrayField(TEXT("streams"), Declarations));
    TestTrue(TEXT("one camera stream is emitted"), Declarations != nullptr && Declarations->Num() == 1);
    if (Declarations != nullptr && Declarations->Num() == 1)
    {
        const TSharedPtr<FJsonObject>* Stream = nullptr;
        TestTrue(TEXT("stream entry is an object"), (*Declarations)[0]->TryGetObject(Stream));
        TestTrue(TEXT("stream object exists"), Stream != nullptr && Stream->IsValid());
        if (Stream != nullptr && Stream->IsValid())
        {
            TestEqual(TEXT("ACTIVE stream has exact truth evidence fields and omits reason"), (*Stream)->Values.Num(), 5);
            TestEqual(TEXT("published frames are retained"), static_cast<uint64>((*Stream)->GetIntegerField(TEXT("published_frames"))), 1ULL);
            TestEqual(TEXT("last truth sequence is retained"), static_cast<uint64>((*Stream)->GetIntegerField(TEXT("last_sample_truth_sequence"))), 19ULL);
            int64 LastSampleSimTimeNs = 0;
            TestTrue(
                TEXT("last simulation time is an exact 64-bit integer"),
                (*Stream)->TryGetNumberField(TEXT("last_sample_sim_time_ns"), LastSampleSimTimeNs));
            TestEqual(
                TEXT("last simulation time is retained"),
                LastSampleSimTimeNs,
                int64{750'000'000});
        }
    }

    TArray<LingTuSim::Sensors::Camera::FStreamReadiness> ResetStreams;
    LingTuSim::Sensors::Camera::FStreamReadiness Reset = Rgb;
    Reset.State = TEXT("PREPARING");
    Reset.PublishedFrames = 0;
    Reset.LastSampleTruthSequence = 0;
    Reset.LastSampleSimTimeNs = 0;
    ResetStreams.Add(Reset);
    const FString ResetJson = LingTuSim::Sensors::Camera::BuildReadinessEvidenceJson(
        TEXT("digest-a"), 0, 43, ResetStreams);
    TSharedPtr<FJsonObject> ResetRoot;
    TSharedRef<TJsonReader<>> ResetReader = TJsonReaderFactory<>::Create(ResetJson);
    TestTrue(TEXT("reset PREPARING evidence is valid JSON"), FJsonSerializer::Deserialize(ResetReader, ResetRoot));
    if (ResetRoot.IsValid())
    {
        const TArray<TSharedPtr<FJsonValue>>* ResetDeclarations = nullptr;
        TestTrue(TEXT("reset streams are present"), ResetRoot->TryGetArrayField(TEXT("streams"), ResetDeclarations));
        if (ResetDeclarations != nullptr && ResetDeclarations->Num() == 1)
        {
            const TSharedPtr<FJsonObject>* ResetStream = nullptr;
            TestTrue(TEXT("reset stream is an object"), (*ResetDeclarations)[0]->TryGetObject(ResetStream));
            if (ResetStream != nullptr && ResetStream->IsValid())
            {
                int64 PublishedFrames = -1;
                int64 LastSampleTruthSequence = -1;
                int64 LastSampleSimTimeNs = -1;
                TestTrue(
                    TEXT("reset published frames are an exact 64-bit integer"),
                    (*ResetStream)->TryGetNumberField(TEXT("published_frames"), PublishedFrames));
                TestTrue(
                    TEXT("reset truth sequence is an exact 64-bit integer"),
                    (*ResetStream)->TryGetNumberField(TEXT("last_sample_truth_sequence"), LastSampleTruthSequence));
                TestTrue(
                    TEXT("reset simulation time is an exact 64-bit integer"),
                    (*ResetStream)->TryGetNumberField(TEXT("last_sample_sim_time_ns"), LastSampleSimTimeNs));
                TestEqual(TEXT("reset clears published frames"), PublishedFrames, int64{0});
                TestEqual(TEXT("reset clears truth sequence"), LastSampleTruthSequence, int64{0});
                TestEqual(TEXT("reset clears simulation time"), LastSampleSimTimeNs, int64{0});
            }
        }
    }

    TArray<LingTuSim::Sensors::Camera::FStreamReadiness> FailedStreams;
    LingTuSim::Sensors::Camera::FStreamReadiness Failed = Rgb;
    Failed.State = TEXT("FAILED");
    Failed.Reason = TEXT("SHM publish failed");
    FailedStreams.Add(Failed);
    const FString FailedJson = LingTuSim::Sensors::Camera::BuildReadinessEvidenceJson(
        TEXT("digest-a"), 0, 42, FailedStreams);
    TSharedPtr<FJsonObject> FailedRoot;
    TSharedRef<TJsonReader<>> FailedReader = TJsonReaderFactory<>::Create(FailedJson);
    TestTrue(TEXT("FAILED readiness evidence is valid JSON"), FJsonSerializer::Deserialize(FailedReader, FailedRoot));
    if (FailedRoot.IsValid())
    {
        const TSharedPtr<FJsonObject>* FailedSensors = nullptr;
        TestTrue(TEXT("FAILED sensors summary is an object"), FailedRoot->TryGetObjectField(TEXT("sensors"), FailedSensors));
        if (FailedSensors != nullptr && FailedSensors->IsValid())
        {
            TestEqual(TEXT("a producer failure cannot leave camera ACTIVE"), (*FailedSensors)->GetStringField(TEXT("camera_streams")), FString(TEXT("FAILED")));
            TestEqual(TEXT("overall follows the producer failure"), (*FailedSensors)->GetStringField(TEXT("overall")), FString(TEXT("FAILED")));
        }
    }
    return true;
}

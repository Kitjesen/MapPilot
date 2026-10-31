#if WITH_DEV_AUTOMATION_TESTS

#include "LingTuCameraShmWriter.h"

#include "Misc/AutomationTest.h"
#include "Misc/Guid.h"

#if PLATFORM_WINDOWS
#include "Windows/AllowWindowsPlatformTypes.h"
#include <Windows.h>
#include "Windows/HideWindowsPlatformTypes.h"
#endif

namespace
{
    constexpr uint64 TestGeneration = 7;

    FString MakeMappingName()
    {
        return FString::Printf(
            TEXT("Local\\LingTuCameraShmWriterTest-%s"),
            *FGuid::NewGuid().ToString(EGuidFormats::Digits));
    }

    LingTuSim::Sensors::CameraShm::FWriterConfig MakeConfig(const FString& Name)
    {
        LingTuSim::Sensors::CameraShm::FWriterConfig Config;
        Config.MappingName = Name;
        Config.Generation = TestGeneration;
        Config.SlotCount = LingTuSim::Sensors::CameraShm::DefaultSlotCount;
        Config.SlotCapacity = 32;
        return Config;
    }

    LingTuSim::Sensors::CameraShm::FFrameMetadata MakeMetadata(
        const LingTuSim::Sensors::CameraShm::EStreamKind Kind =
            LingTuSim::Sensors::CameraShm::EStreamKind::Color,
        const uint64 TimestampNs = 1000000000ULL)
    {
        LingTuSim::Sensors::CameraShm::FFrameMetadata Metadata;
        Metadata.StreamKind = Kind;
        Metadata.Generation = TestGeneration;
        Metadata.TimestampNs = TimestampNs;
        Metadata.Width = 2;
        Metadata.Height = 2;
        Metadata.Stride = Kind == LingTuSim::Sensors::CameraShm::EStreamKind::Depth ? 4 : 6;
        Metadata.Encoding = Kind == LingTuSim::Sensors::CameraShm::EStreamKind::Depth
            ? TEXT("16UC1")
            : TEXT("rgb8");
        Metadata.FrameId = TEXT("camera_color");
        Metadata.Fx = 10.0;
        Metadata.Fy = 11.0;
        Metadata.Cx = 1.0;
        Metadata.Cy = 1.0;
        return Metadata;
    }

    const uint8* SlotPayload(const uint8* Base, const uint16 SlotIndex, const uint32 SlotCapacity)
    {
        return Base
            + sizeof(LingTuSim::Sensors::CameraShm::FSharedHeader)
            + static_cast<uint64>(SlotIndex)
                * (sizeof(LingTuSim::Sensors::CameraShm::FSlotHeader) + SlotCapacity)
            + sizeof(LingTuSim::Sensors::CameraShm::FSlotHeader);
    }

    const LingTuSim::Sensors::CameraShm::FSlotHeader* SlotHeader(
        const uint8* Base,
        const uint16 SlotIndex,
        const uint32 SlotCapacity)
    {
        return reinterpret_cast<const LingTuSim::Sensors::CameraShm::FSlotHeader*>(
            Base
            + sizeof(LingTuSim::Sensors::CameraShm::FSharedHeader)
            + static_cast<uint64>(SlotIndex)
                * (sizeof(LingTuSim::Sensors::CameraShm::FSlotHeader) + SlotCapacity));
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuCameraShmAbiConstantsTest,
    "LingTuSim.Sensors.CameraShm.AbiConstants",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuCameraShmAbiConstantsTest::RunTest(const FString& Parameters)
{
    (void)Parameters;
    using namespace LingTuSim::Sensors::CameraShm;

    TestEqual(TEXT("Schema name matches the canonical adapter"), FString(UTF8_TO_TCHAR(Schema)), FString(TEXT("lingtu.camera.shm_frame.v1")));
    TestTrue(TEXT("Magic is LTCSHM01"), FMemory::Memcmp(Magic, "LTCSHM01", 8) == 0);
    TestEqual(TEXT("Superblock is 64 bytes"), static_cast<int32>(sizeof(FSharedHeader)), 64);
    TestEqual(TEXT("Slot header is 256 bytes"), static_cast<int32>(sizeof(FSlotHeader)), 256);
    TestEqual(TEXT("Published sequence offset is 32"), static_cast<int32>(offsetof(FSharedHeader, published_sequence)), 32);
    TestEqual(TEXT("Model generation offset is 56"), static_cast<int32>(offsetof(FSharedHeader, reserved1)), 56);
    TestEqual(TEXT("Slot flags offset is 48"), static_cast<int32>(offsetof(FSlotHeader, flags)), 48);
    TestEqual(TEXT("Slot encoding offset is 64"), static_cast<int32>(offsetof(FSlotHeader, encoding)), 64);
    TestEqual(TEXT("Slot frame_id offset is 80"), static_cast<int32>(offsetof(FSlotHeader, frame_id)), 80);
    TestEqual(TEXT("Truth extension offset is 224"), static_cast<int32>(offsetof(FSlotHeader, reserved1)), 224);
    TestEqual(TEXT("Truth flag is bit zero"), HasSimulationTruth, uint32{1});
    TestEqual(TEXT("Slot guard_end offset is 248"), static_cast<int32>(offsetof(FSlotHeader, guard_end)), 248);
    TestEqual(TEXT("Mapping size follows the canonical ring formula"), MappingSize(2, 32), uint64{640});
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuCameraShmWriteReadValidationTest,
    "LingTuSim.Sensors.CameraShm.WriteReadValidation",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuCameraShmWriteReadValidationTest::RunTest(const FString& Parameters)
{
    (void)Parameters;
    using namespace LingTuSim::Sensors::CameraShm;

    FFrameWriter Writer;
    FString Error;
    const FWriterConfig Config = MakeConfig(MakeMappingName());
    TestTrue(TEXT("Explicit RunAllocation mapping opens"), Writer.TryOpen(Config, Error));

    const ANSICHAR Payload[] = "abcdefghijkl";
    TestEqual(
        TEXT("Publishing caller-supplied color bytes returns sequence 1"),
        Writer.TryPublish(MakeMetadata(), Payload, 12, Error),
        true);
    TestEqual(TEXT("Last sequence updates"), Writer.LastSequence(), 1ULL);

    const FSharedHeader* Header = reinterpret_cast<const FSharedHeader*>(Writer.MappedBytes());
    TestEqual(TEXT("Superblock magic byte 0 is little-endian layout compatible"), Header->magic[0], ANSICHAR('L'));
    TestEqual(TEXT("Schema version is v1"), Header->schema_version, uint16{1});
    TestEqual(TEXT("Header size is canonical"), Header->header_size, uint16{64});
    TestEqual(TEXT("Slot header size is canonical"), Header->slot_header_size, uint16{256});
    TestEqual(TEXT("Ring has two slots"), Header->slot_count, uint16{2});
    TestEqual(TEXT("Slot capacity is exact allocation value"), Header->slot_capacity, uint32{32});
    TestEqual(TEXT("First active slot is zero"), Header->active_slot, uint32{0});
    TestEqual(TEXT("Published sequence is one"), Header->published_sequence, 1ULL);
    TestTrue(TEXT("Created timestamp is present"), Header->created_timestamp_ns > 0);
    TestTrue(TEXT("Heartbeat timestamp is present"), Header->writer_heartbeat_ns > 0);
    TestEqual(TEXT("Mapping records the model generation"), Header->reserved1, TestGeneration);

    const FSlotHeader* Slot = SlotHeader(Writer.MappedBytes(), 0, Config.SlotCapacity);
    const uint8 ZeroExtension[24] = {};
    TestEqual(TEXT("Legacy metadata defaults to no truth flag"), Slot->flags, uint32{0});
    TestTrue(TEXT("Legacy truth extension is all zero"),
        FMemory::Memcmp(Slot->reserved1, ZeroExtension, sizeof(ZeroExtension)) == 0);
    TestEqual(TEXT("Seqlock begin token committed"), Slot->guard_begin, 2ULL);
    TestEqual(TEXT("Seqlock end token committed"), Slot->guard_end, 2ULL);
    TestEqual(TEXT("Slot sequence is one"), Slot->sequence, 1ULL);
    TestEqual(TEXT("Frame timestamp is exact"), Slot->timestamp_ns, 1000000000ULL);
    TestEqual(TEXT("Width is exact"), Slot->width, uint32{2});
    TestEqual(TEXT("Height is exact"), Slot->height, uint32{2});
    TestEqual(TEXT("Stride is exact"), Slot->stride, uint32{6});
    TestEqual(TEXT("Payload size is exact"), Slot->payload_size, uint32{12});
    TestEqual(TEXT("Payload capacity is exact"), Slot->payload_capacity, uint32{32});
    TestEqual(TEXT("CRC32 mirrors zlib/native value"), Slot->payload_crc32, uint32{0xF6781B24});
    TestEqual(TEXT("Stream kind is COLOR"), Slot->stream_kind, uint16{2});
    TestEqual(TEXT("Encoding size is exact"), Slot->encoding_size, uint16{4});
    TestEqual(TEXT("Slot schema version is v1"), Slot->schema_version, uint16{1});
    TestEqual(TEXT("Slot header size is canonical"), Slot->header_size, uint16{256});
    TestTrue(TEXT("Encoding bytes are rgb8"), FMemory::Memcmp(Slot->encoding, "rgb8", 4) == 0);
    TestTrue(TEXT("Frame id bytes are exact"), FMemory::Memcmp(Slot->frame_id, "camera_color", 12) == 0);
    TestEqual(TEXT("Fx calibration is exact"), Slot->fx, 10.0);
    TestEqual(TEXT("Fy calibration is exact"), Slot->fy, 11.0);
    TestTrue(TEXT("Payload bytes are copied byte-for-byte"), FMemory::Memcmp(SlotPayload(Writer.MappedBytes(), 0, Config.SlotCapacity), Payload, 12) == 0);

    const uint16 DepthPayload[] = {1, 2, 3, 4};
    TestTrue(
        TEXT("Publishing caller-supplied depth bytes returns sequence 2"),
        Writer.TryPublish(MakeMetadata(LingTuSim::Sensors::CameraShm::EStreamKind::Depth, 1000000001ULL), DepthPayload, 8, Error));
    const FSlotHeader* DepthSlot = SlotHeader(Writer.MappedBytes(), 1, Config.SlotCapacity);
    TestEqual(TEXT("Depth stream kind is DEPTH"), DepthSlot->stream_kind, uint16{3});
    TestEqual(TEXT("Depth width is exact"), DepthSlot->width, uint32{2});
    TestEqual(TEXT("Depth height is exact"), DepthSlot->height, uint32{2});
    TestEqual(TEXT("Depth stride is exact"), DepthSlot->stride, uint32{4});
    TestEqual(TEXT("Depth payload size is exact"), DepthSlot->payload_size, uint32{8});
    TestEqual(TEXT("Depth encoding size is exact"), DepthSlot->encoding_size, uint16{5});
    TestTrue(TEXT("Depth encoding bytes are 16UC1"), FMemory::Memcmp(DepthSlot->encoding, "16UC1", 5) == 0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuCameraShmCrcAndTwoSlotSequencingTest,
    "LingTuSim.Sensors.CameraShm.CrcAndTwoSlotSequencing",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuCameraShmCrcAndTwoSlotSequencingTest::RunTest(const FString& Parameters)
{
    (void)Parameters;
    using namespace LingTuSim::Sensors::CameraShm;

    FFrameWriter Writer;
    FString Error;
    const FWriterConfig Config = MakeConfig(MakeMappingName());
    TestTrue(TEXT("Explicit RunAllocation mapping opens"), Writer.TryOpen(Config, Error));

    const ANSICHAR First[] = "abcdefghijkl";
    const ANSICHAR Second[] = "mnopqrstuvwx";
    const ANSICHAR Third[] = "ABCDEFGHIJKL";
    TestTrue(TEXT("Sequence 1 publishes"), Writer.TryPublish(MakeMetadata(LingTuSim::Sensors::CameraShm::EStreamKind::Color, 1000000000ULL), First, 12, Error));
    TestTrue(TEXT("Sequence 2 publishes"), Writer.TryPublish(MakeMetadata(LingTuSim::Sensors::CameraShm::EStreamKind::Color, 1000000001ULL), Second, 12, Error));
    TestTrue(TEXT("Sequence 3 wraps to slot 0"), Writer.TryPublish(MakeMetadata(LingTuSim::Sensors::CameraShm::EStreamKind::Color, 1000000002ULL), Third, 12, Error));

    const FSharedHeader* Header = reinterpret_cast<const FSharedHeader*>(Writer.MappedBytes());
    TestEqual(TEXT("Published sequence reaches three"), Header->published_sequence, 3ULL);
    TestEqual(TEXT("Active slot wraps to zero"), Header->active_slot, uint32{0});

    const FSlotHeader* WrappedSlot = SlotHeader(Writer.MappedBytes(), 0, Config.SlotCapacity);
    const FSlotHeader* PreviousSlot = SlotHeader(Writer.MappedBytes(), 1, Config.SlotCapacity);
    TestEqual(TEXT("Wrapped slot carries sequence 3"), WrappedSlot->sequence, 3ULL);
    TestEqual(TEXT("Wrapped slot seqlock token is 6"), WrappedSlot->guard_begin, 6ULL);
    TestEqual(TEXT("Previous slot still carries sequence 2"), PreviousSlot->sequence, 2ULL);
    TestEqual(TEXT("Previous slot seqlock token is 4"), PreviousSlot->guard_begin, 4ULL);
    TestTrue(TEXT("Wrapped payload is the third frame"), FMemory::Memcmp(SlotPayload(Writer.MappedBytes(), 0, Config.SlotCapacity), Third, 12) == 0);
    TestEqual(TEXT("CRC32 helper matches canonical vector"), Crc32(First, 12), uint32{0xF6781B24});
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuCameraShmSimulationTruthTest,
    "LingTuSim.Sensors.CameraShm.SimulationTruth",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuCameraShmSimulationTruthTest::RunTest(const FString& Parameters)
{
    (void)Parameters;
    using namespace LingTuSim::Sensors::CameraShm;

    FFrameWriter Writer;
    FString Error;
    const FWriterConfig Config = MakeConfig(MakeMappingName());
    if (!TestTrue(TEXT("Truth mapping opens"), Writer.TryOpen(Config, Error)))
    {
        return false;
    }
    const ANSICHAR Payload[] = "abcdefghijkl";
    FFrameMetadata Metadata = MakeMetadata();
    TestFalse(TEXT("Truth metadata is opt-in"), Metadata.bHasSimulationTruth);
    TestEqual(TEXT("Default truth sequence is zero"), Metadata.TruthSequence, uint64{0});
    TestEqual(TEXT("Default reset generation is zero"), Metadata.ResetGeneration, uint64{0});
    TestEqual(TEXT("Default simulation time is zero"), Metadata.SimTimeNs, int64{0});
    Metadata.bHasSimulationTruth = true;
    Metadata.TruthSequence = 0xFEDCBA9876543210ULL;
    Metadata.ResetGeneration = 0x8877665544332211ULL;
    Metadata.SimTimeNs = 0x0123456789ABCDEFLL;
    if (!TestTrue(TEXT("Stamped frame publishes"), Writer.TryPublish(Metadata, Payload, 12, Error)))
    {
        return false;
    }

    // Inspect fixed wire offsets independently of the writer's serialization helper.
    const uint8* RawSlot = Writer.MappedBytes() + 64;
    const uint8 ExpectedFlags[] = {1, 0, 0, 0};
    const uint8 ExpectedExtension[] = {
        0x10, 0x32, 0x54, 0x76, 0x98, 0xBA, 0xDC, 0xFE,
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
        0xEF, 0xCD, 0xAB, 0x89, 0x67, 0x45, 0x23, 0x01};
    TestTrue(TEXT("Raw slot flags contain only HasSimulationTruth"),
        FMemory::Memcmp(RawSlot + 48, ExpectedFlags, sizeof(ExpectedFlags)) == 0);
    TestTrue(TEXT("Raw truth extension is exactly little-endian <QQq>"),
        FMemory::Memcmp(RawSlot + 224, ExpectedExtension, sizeof(ExpectedExtension)) == 0);
    const FSlotHeader* StampedSlot = SlotHeader(Writer.MappedBytes(), 0, Config.SlotCapacity);
    TestEqual(TEXT("Transport timestamp stays Unix realtime, not simulation time"),
        StampedSlot->timestamp_ns, Metadata.TimestampNs);
    TestEqual(TEXT("Truth frame retains the committed seqlock"), StampedSlot->guard_end, uint64{2});
    TestEqual(TEXT("Truth metadata leaves payload CRC unchanged"), StampedSlot->payload_crc32, Crc32(Payload, 12));

    Metadata.TruthSequence = 0;
    Metadata.ResetGeneration = 0;
    Metadata.SimTimeNs = 0;
    TestTrue(TEXT("Zero truth/reset/time are valid stamped values"), Writer.TryPublish(Metadata, Payload, 12, Error));
    const FSlotHeader* ZeroSlot = SlotHeader(Writer.MappedBytes(), 1, Config.SlotCapacity);
    const uint8 ZeroExtension[24] = {};
    TestEqual(TEXT("Zero truth still carries the explicit flag"), ZeroSlot->flags, uint32{1});
    TestTrue(TEXT("Zero truth is encoded exactly"),
        FMemory::Memcmp(ZeroSlot->reserved1, ZeroExtension, sizeof(ZeroExtension)) == 0);

    Metadata.bHasSimulationTruth = false;
    Metadata.TruthSequence = 99;
    Metadata.ResetGeneration = 88;
    Metadata.SimTimeNs = 77;
    TestTrue(TEXT("Legacy publication reuses the previously stamped slot"), Writer.TryPublish(Metadata, Payload, 12, Error));
    TestEqual(TEXT("Slot reuse clears the old truth flag"), StampedSlot->flags, uint32{0});
    TestTrue(TEXT("Slot reuse zeroes all 24 bytes even with nonzero unflagged metadata"),
        FMemory::Memcmp(StampedSlot->reserved1, ZeroExtension, sizeof(ZeroExtension)) == 0);

    const uint64 SequenceBeforeRejection = Writer.LastSequence();
    TArray<uint8> BeforeRejection;
    BeforeRejection.Append(Writer.MappedBytes(), static_cast<int32>(Writer.MappedSize()));
    Metadata.bHasSimulationTruth = true;
    Metadata.SimTimeNs = -1;
    TestFalse(TEXT("Negative simulation time is rejected"), Writer.TryPublish(Metadata, Payload, 12, Error));
    TestTrue(TEXT("Negative time rejection explains the error"), Error.Contains(TEXT("non-negative")));
    Metadata.bHasSimulationTruth = false;
    TestFalse(TEXT("Negative simulation time is not hidden by an unset truth flag"), Writer.TryPublish(Metadata, Payload, 12, Error));
    TestEqual(TEXT("Rejected frames do not advance sequence"), Writer.LastSequence(), SequenceBeforeRejection);
    TestTrue(TEXT("Rejected frames do not mutate the mapping"),
        FMemory::Memcmp(Writer.MappedBytes(), BeforeRejection.GetData(), BeforeRejection.Num()) == 0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuCameraShmMappingGenerationTest,
    "LingTuSim.Sensors.CameraShm.MappingGeneration",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuCameraShmMappingGenerationTest::RunTest(const FString& Parameters)
{
    (void)Parameters;
    using namespace LingTuSim::Sensors::CameraShm;

    FString Error;
    FFrameWriter Writer;
    FWriterConfig Config = MakeConfig(MakeMappingName());
    Config.Generation = 0x8877665544332211ULL;
    if (!TestTrue(TEXT("Model generation mapping opens"), Writer.TryOpen(Config, Error)))
    {
        return false;
    }
    const uint8 ExpectedGeneration[] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
    TestTrue(TEXT("Header model generation uses fixed little-endian wire bytes"),
        FMemory::Memcmp(Writer.MappedBytes() + 56, ExpectedGeneration, sizeof(ExpectedGeneration)) == 0);
    FFrameMetadata Metadata = MakeMetadata();
    Metadata.Generation = Config.Generation;
    const ANSICHAR Payload[] = "abcdefghijkl";
    TestTrue(TEXT("Original writer publishes"), Writer.TryPublish(Metadata, Payload, 12, Error));

    FFrameWriter SameGenerationWriter;
    TestTrue(TEXT("Same-generation existing mapping opens"), SameGenerationWriter.TryOpen(Config, Error));
    TestEqual(TEXT("Reopen preserves published sequence"), SameGenerationWriter.LastSequence(), uint64{1});
    TestTrue(TEXT("Reopen preserves the committed payload"),
        FMemory::Memcmp(SlotPayload(Writer.MappedBytes(), 0, Config.SlotCapacity), Payload, 12) == 0);

    TArray<uint8> BeforeRejection;
    BeforeRejection.Append(Writer.MappedBytes(), static_cast<int32>(Writer.MappedSize()));
    FFrameWriter WrongGenerationWriter;
    FWriterConfig WrongConfig = Config;
    WrongConfig.Generation = 0;
    TestFalse(TEXT("Existing mapping rejects a different generation"), WrongGenerationWriter.TryOpen(WrongConfig, Error));
    TestTrue(TEXT("Mapping mismatch explains the model generation"), Error.Contains(TEXT("model generation")));
    TestNull(TEXT("Rejected writer releases its view"), WrongGenerationWriter.MappedBytes());
    TestTrue(TEXT("Wrong-generation open does not overwrite header, heartbeat or frames"),
        FMemory::Memcmp(Writer.MappedBytes(), BeforeRejection.GetData(), BeforeRejection.Num()) == 0);

    FFrameWriter ZeroWriter;
    FWriterConfig ZeroConfig = MakeConfig(MakeMappingName());
    ZeroConfig.Generation = 0;
    if (!TestTrue(TEXT("Zero model generation mapping opens"), ZeroWriter.TryOpen(ZeroConfig, Error)))
    {
        return false;
    }
    Metadata.Generation = 0;
    Metadata.bHasSimulationTruth = true;
    TestTrue(TEXT("Zero model/reset/truth/time publishes with truth flag"), ZeroWriter.TryPublish(Metadata, Payload, 12, Error));
    const FSharedHeader* ZeroHeader = reinterpret_cast<const FSharedHeader*>(ZeroWriter.MappedBytes());
    TestEqual(TEXT("Header preserves legal generation zero"), ZeroHeader->reserved1, uint64{0});
    TestEqual(TEXT("Zero-generation truth frame is flagged"),
        SlotHeader(ZeroWriter.MappedBytes(), 0, ZeroConfig.SlotCapacity)->flags, uint32{1});
    FFrameWriter ZeroReopener;
    TestTrue(TEXT("Existing generation-zero mapping reopens"), ZeroReopener.TryOpen(ZeroConfig, Error));
    BeforeRejection.Reset();
    BeforeRejection.Append(ZeroWriter.MappedBytes(), static_cast<int32>(ZeroWriter.MappedSize()));
    WrongConfig = ZeroConfig;
    WrongConfig.Generation = TestGeneration;
    TestFalse(TEXT("Existing zero generation is not a wildcard"), WrongGenerationWriter.TryOpen(WrongConfig, Error));
    TestTrue(TEXT("Generation-zero mapping is not adopted or overwritten"),
        FMemory::Memcmp(ZeroWriter.MappedBytes(), BeforeRejection.GetData(), BeforeRejection.Num()) == 0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLingTuCameraShmNamedMappingAndFailClosedTest,
    "LingTuSim.Sensors.CameraShm.NamedMappingAndFailClosed",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLingTuCameraShmNamedMappingAndFailClosedTest::RunTest(const FString& Parameters)
{
    (void)Parameters;
    using namespace LingTuSim::Sensors::CameraShm;

    FString Error;
    FFrameWriter ZeroGenerationWriter;
    FWriterConfig ZeroGenerationConfig = MakeConfig(MakeMappingName());
    ZeroGenerationConfig.Generation = 0;
    TestTrue(TEXT("Zero generation is a legal initial RunAllocation generation"), ZeroGenerationWriter.TryOpen(ZeroGenerationConfig, Error));
    FFrameMetadata ZeroGenerationMetadata = MakeMetadata();
    ZeroGenerationMetadata.Generation = 0;
    const ANSICHAR ZeroGenerationPayload[] = "abcdefghijkl";
    TestTrue(TEXT("Zero generation publishes when frame metadata matches"), ZeroGenerationWriter.TryPublish(
        ZeroGenerationMetadata, ZeroGenerationPayload, 12, Error));

    FFrameWriter Writer;
    const FWriterConfig Config = MakeConfig(MakeMappingName());
    TestTrue(TEXT("Explicit RunAllocation mapping opens"), Writer.TryOpen(Config, Error));

    FFrameMetadata Metadata = MakeMetadata();
    const ANSICHAR Payload[] = "abcdefghijkl";
    Metadata.Generation = TestGeneration + 1;
    TestFalse(TEXT("Wrong frame generation is rejected"), Writer.TryPublish(Metadata, Payload, 12, Error));
    Metadata = MakeMetadata();
    Metadata.Width = 0;
    TestFalse(TEXT("Invalid dimensions are rejected"), Writer.TryPublish(Metadata, Payload, 12, Error));
    Metadata = MakeMetadata();
    TestFalse(TEXT("Invalid payload size is rejected"), Writer.TryPublish(Metadata, Payload, 11, Error));
    Metadata = MakeMetadata();
    Metadata.StreamKind = LingTuSim::Sensors::CameraShm::EStreamKind::Info;
    TestFalse(TEXT("INFO kind is not produced by the writer"), Writer.TryPublish(Metadata, nullptr, 0, Error));

#if PLATFORM_WINDOWS
    HANDLE ReadHandle = ::OpenFileMappingW(FILE_MAP_READ, 0, *Config.MappingName);
    TestTrue(TEXT("OpenFileMapping finds the explicit RunAllocation mapping name"), ReadHandle != nullptr);
    if (ReadHandle != nullptr)
    {
        const uint64 Size = MappingSize(Config.SlotCount, Config.SlotCapacity);
        const uint8* ReadBytes = static_cast<const uint8*>(::MapViewOfFile(ReadHandle, FILE_MAP_READ, 0, 0, static_cast<SIZE_T>(Size)));
        TestTrue(TEXT("Read-only named mapping view opens"), ReadBytes != nullptr);
        if (ReadBytes != nullptr)
        {
            const FSharedHeader* Header = reinterpret_cast<const FSharedHeader*>(ReadBytes);
            TestEqual(TEXT("Named mapping has no fake committed frames"), Header->published_sequence, 0ULL);
            ::UnmapViewOfFile(ReadBytes);
        }
        ::CloseHandle(ReadHandle);
    }
#else
    AddInfo(TEXT("Named mapping check skipped because this platform is not Windows."));
#endif
    return true;
}

#endif

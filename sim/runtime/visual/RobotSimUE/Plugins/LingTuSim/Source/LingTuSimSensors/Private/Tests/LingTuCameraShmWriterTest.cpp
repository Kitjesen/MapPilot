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
    TestEqual(TEXT("Slot encoding offset is 64"), static_cast<int32>(offsetof(FSlotHeader, encoding)), 64);
    TestEqual(TEXT("Slot frame_id offset is 80"), static_cast<int32>(offsetof(FSlotHeader, frame_id)), 80);
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

    const FSlotHeader* Slot = SlotHeader(Writer.MappedBytes(), 0, Config.SlotCapacity);
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

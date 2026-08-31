#pragma once

#include "CoreMinimal.h"

namespace LingTuSim::Sensors::CameraShm
{
    inline constexpr ANSICHAR Schema[] = "lingtu.camera.shm_frame.v1";
    inline constexpr ANSICHAR Magic[8] = {'L', 'T', 'C', 'S', 'H', 'M', '0', '1'};
    inline constexpr uint16 SchemaVersion = 1;
    inline constexpr uint16 SuperblockSize = 64;
    inline constexpr uint16 SlotHeaderSize = 256;
    inline constexpr uint16 DefaultSlotCount = 2;
    inline constexpr uint32 DefaultSlotCapacity = 8U * 1024U * 1024U;

    enum class EStreamKind : uint16
    {
        Info = 1,
        Color = 2,
        Depth = 3,
    };

    struct alignas(64) FSharedHeader
    {
        ANSICHAR magic[8];
        uint16 schema_version;
        uint16 header_size;
        uint16 slot_header_size;
        uint16 slot_count;
        uint32 slot_capacity;
        uint32 active_slot;
        uint32 flags;
        uint32 reserved0;
        uint64 published_sequence;
        uint64 created_timestamp_ns;
        uint64 writer_heartbeat_ns;
        uint64 reserved1;
    };

    struct alignas(64) FSlotHeader
    {
        uint64 guard_begin;
        uint64 sequence;
        uint64 timestamp_ns;
        uint32 width;
        uint32 height;
        uint32 stride;
        uint32 payload_size;
        uint32 payload_capacity;
        uint32 payload_crc32;
        uint32 flags;
        uint32 reserved0;
        uint16 stream_kind;
        uint16 encoding_size;
        uint16 schema_version;
        uint16 header_size;
        ANSICHAR encoding[16];
        ANSICHAR frame_id[64];
        double fx;
        double fy;
        double cx;
        double cy;
        double depth_scale;
        double dist_k1;
        double dist_k2;
        double dist_p1;
        double dist_p2;
        double dist_k3;
        uint8 reserved1[24];
        uint64 guard_end;
    };

    static_assert(sizeof(FSharedHeader) == SuperblockSize, "camera SHM superblock must be 64 bytes");
    static_assert(sizeof(FSlotHeader) == SlotHeaderSize, "camera SHM slot header must be 256 bytes");
    static_assert(offsetof(FSharedHeader, published_sequence) == 32, "camera SHM sequence offset drifted");
    static_assert(offsetof(FSlotHeader, encoding) == 64, "camera SHM encoding offset drifted");
    static_assert(offsetof(FSlotHeader, frame_id) == 80, "camera SHM frame_id offset drifted");
    static_assert(offsetof(FSlotHeader, guard_end) == 248, "camera SHM guard_end offset drifted");

    struct LINGTUSIMSENSORS_API FWriterConfig
    {
        FString MappingName;
        uint64 Generation = 0;
        uint16 SlotCount = DefaultSlotCount;
        uint32 SlotCapacity = DefaultSlotCapacity;
    };

    struct LINGTUSIMSENSORS_API FFrameMetadata
    {
        EStreamKind StreamKind = EStreamKind::Color;
        uint64 Generation = 0;
        uint64 TimestampNs = 0;
        uint32 Width = 0;
        uint32 Height = 0;
        uint32 Stride = 0;
        FString Encoding;
        FString FrameId;
        double Fx = 0.0;
        double Fy = 0.0;
        double Cx = 0.0;
        double Cy = 0.0;
        double DepthScale = 0.001;
        double DistK1 = 0.0;
        double DistK2 = 0.0;
        double DistP1 = 0.0;
        double DistP2 = 0.0;
        double DistK3 = 0.0;
    };

    LINGTUSIMSENSORS_API uint32 Crc32(const void* Data, uint64 Size);
    LINGTUSIMSENSORS_API uint64 UnixTimeNs();
    LINGTUSIMSENSORS_API uint64 MappingSize(uint16 SlotCount, uint32 SlotCapacity);

    class LINGTUSIMSENSORS_API FFrameWriter
    {
    public:
        FFrameWriter() = default;
        ~FFrameWriter();

        FFrameWriter(const FFrameWriter&) = delete;
        FFrameWriter& operator=(const FFrameWriter&) = delete;

        FFrameWriter(FFrameWriter&& Other) noexcept;
        FFrameWriter& operator=(FFrameWriter&& Other) noexcept;

        bool TryOpen(const FWriterConfig& Config, FString& OutError);
        bool TryPublish(const FFrameMetadata& Metadata, const void* Payload, uint64 PayloadSize, FString& OutError);
        void Close();

        uint64 LastSequence() const { return LastSequenceValue; }
        uint64 Generation() const { return Config.Generation; }
        const FString& Name() const { return Config.MappingName; }
        const uint8* MappedBytes() const { return Mapping; }
        uint64 MappedSize() const { return MappingSizeBytes; }

    private:
        bool ValidateConfig(const FWriterConfig& Candidate, FString& OutError) const;
        bool ValidateFrame(
            const FFrameMetadata& Metadata,
            const void* Payload,
            uint64 PayloadSize,
            FString& OutError) const;
        bool InitializeOrValidateHeader(FString& OutError);
        FSlotHeader* SlotHeader(uint16 Index) const;
        uint8* SlotPayload(uint16 Index) const;

        FWriterConfig Config;
        void* MappingHandle = nullptr;
        uint8* Mapping = nullptr;
        uint64 MappingSizeBytes = 0;
        FSharedHeader* Header = nullptr;
        uint64 LastSequenceValue = 0;
    };
}

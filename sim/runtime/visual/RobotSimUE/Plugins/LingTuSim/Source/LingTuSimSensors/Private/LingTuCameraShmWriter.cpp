#include "LingTuCameraShmWriter.h"

#include <type_traits>

static_assert(
    std::is_same_v<decltype(LingTuSim::Sensors::CameraShm::FWriterConfig::Generation), uint64>,
    "camera SHM model generation must remain unsigned so generation zero is valid");

#include "HAL/PlatformTime.h"
#include "Misc/DateTime.h"

#if PLATFORM_WINDOWS
#include "Windows/AllowWindowsPlatformTypes.h"
#include <Windows.h>
#include "Windows/HideWindowsPlatformTypes.h"
#endif

#include <atomic>
#include <limits>

namespace LingTuSim::Sensors::CameraShm
{
    namespace
    {
        uint32 BytesPerPixel(const FString& Encoding)
        {
            if (Encoding == TEXT("rgb8") || Encoding == TEXT("bgr8"))
            {
                return 3;
            }
            if (Encoding == TEXT("rgba8") || Encoding == TEXT("32FC1"))
            {
                return 4;
            }
            if (Encoding == TEXT("16UC1"))
            {
                return 2;
            }
            if (Encoding == TEXT("mono8") || Encoding == TEXT("8UC1"))
            {
                return 1;
            }
            return 0;
        }

        bool IsAscii(const FString& Value)
        {
            for (const TCHAR Character : Value)
            {
                if (Character > 0x7F)
                {
                    return false;
                }
            }
            return true;
        }

        void CopyAscii(ANSICHAR* Destination, const uint64 Capacity, const FString& Value)
        {
            FMemory::Memzero(Destination, Capacity);
            FTCHARToUTF8 Converted(*Value);
            if (Converted.Length() > 0)
            {
                FMemory::Memcpy(Destination, Converted.Get(), Converted.Length());
            }
        }

        void StoreRelease64(uint64* Address, const uint64 Value)
        {
            std::atomic_thread_fence(std::memory_order_release);
            *Address = Value;
            std::atomic_thread_fence(std::memory_order_release);
        }

        void StoreRelease32(uint32* Address, const uint32 Value)
        {
            std::atomic_thread_fence(std::memory_order_release);
            *Address = Value;
            std::atomic_thread_fence(std::memory_order_release);
        }

        uint64 LoadAcquire64(const uint64* Address)
        {
            std::atomic_thread_fence(std::memory_order_acquire);
            const uint64 Value = *Address;
            std::atomic_thread_fence(std::memory_order_acquire);
            return Value;
        }

        bool IsAllZero(const uint8* Data, const uint64 Size)
        {
            for (uint64 Index = 0; Index < Size; ++Index)
            {
                if (Data[Index] != 0)
                {
                    return false;
                }
            }
            return true;
        }
    }

    uint32 Crc32(const void* Data, const uint64 Size)
    {
        uint32 Crc = 0xFFFFFFFFU;
        const uint8* Bytes = static_cast<const uint8*>(Data);
        for (uint64 Index = 0; Index < Size; ++Index)
        {
            Crc ^= Bytes[Index];
            for (int32 Bit = 0; Bit < 8; ++Bit)
            {
                const uint32 Mask = 0U - (Crc & 1U);
                Crc = (Crc >> 1U) ^ (0xEDB88320U & Mask);
            }
        }
        return Crc ^ 0xFFFFFFFFU;
    }

    uint64 UnixTimeNs()
    {
        const FDateTime Now = FDateTime::UtcNow();
        return static_cast<uint64>(Now.ToUnixTimestamp()) * 1000000000ULL
            + static_cast<uint64>((Now.GetTicks() % ETimespan::TicksPerSecond) * 100ULL);
    }

    uint64 MappingSize(const uint16 SlotCount, const uint32 SlotCapacity)
    {
        return static_cast<uint64>(sizeof(FSharedHeader))
            + static_cast<uint64>(SlotCount) * (static_cast<uint64>(sizeof(FSlotHeader)) + SlotCapacity);
    }

    FFrameWriter::~FFrameWriter()
    {
        Close();
    }

    FFrameWriter::FFrameWriter(FFrameWriter&& Other) noexcept
    {
        *this = MoveTemp(Other);
    }

    FFrameWriter& FFrameWriter::operator=(FFrameWriter&& Other) noexcept
    {
        if (this != &Other)
        {
            Close();
            Config = MoveTemp(Other.Config);
            MappingHandle = Other.MappingHandle;
            Mapping = Other.Mapping;
            MappingSizeBytes = Other.MappingSizeBytes;
            Header = Other.Header;
            LastSequenceValue = Other.LastSequenceValue;
            Other.MappingHandle = nullptr;
            Other.Mapping = nullptr;
            Other.MappingSizeBytes = 0;
            Other.Header = nullptr;
            Other.LastSequenceValue = 0;
        }
        return *this;
    }

    bool FFrameWriter::TryOpen(const FWriterConfig& Candidate, FString& OutError)
    {
        Close();
        if (!ValidateConfig(Candidate, OutError))
        {
            return false;
        }

        Config = Candidate;
        MappingSizeBytes = MappingSize(Config.SlotCount, Config.SlotCapacity);

#if PLATFORM_WINDOWS
        if (MappingSizeBytes > static_cast<uint64>(std::numeric_limits<int64>::max()))
        {
            OutError = TEXT("camera SHM mapping size exceeds Windows mapping limits");
            Close();
            return false;
        }
        const DWORD SizeHigh = static_cast<DWORD>(MappingSizeBytes >> 32U);
        const DWORD SizeLow = static_cast<DWORD>(MappingSizeBytes & 0xFFFFFFFFULL);
        MappingHandle = ::CreateFileMappingW(
            INVALID_HANDLE_VALUE,
            nullptr,
            PAGE_READWRITE,
            SizeHigh,
            SizeLow,
            *Config.MappingName);
        if (MappingHandle == nullptr)
        {
            OutError = FString::Printf(
                TEXT("CreateFileMapping failed for camera SHM '%s' with error %lu"),
                *Config.MappingName,
                ::GetLastError());
            Close();
            return false;
        }
        Mapping = static_cast<uint8*>(::MapViewOfFile(
            MappingHandle,
            FILE_MAP_ALL_ACCESS,
            0,
            0,
            static_cast<SIZE_T>(MappingSizeBytes)));
        if (Mapping == nullptr)
        {
            OutError = FString::Printf(
                TEXT("MapViewOfFile failed for camera SHM '%s' with error %lu"),
                *Config.MappingName,
                ::GetLastError());
            Close();
            return false;
        }
#else
        OutError = TEXT("camera SHM Windows named mappings require PLATFORM_WINDOWS");
        Close();
        return false;
#endif

        Header = reinterpret_cast<FSharedHeader*>(Mapping);
        if (!InitializeOrValidateHeader(OutError))
        {
            Close();
            return false;
        }
        StoreRelease64(&Header->writer_heartbeat_ns, UnixTimeNs());
        LastSequenceValue = LoadAcquire64(&Header->published_sequence);
        return true;
    }

    bool FFrameWriter::TryPublish(
        const FFrameMetadata& Metadata,
        const void* Payload,
        const uint64 PayloadSize,
        FString& OutError)
    {
        if (Mapping == nullptr || Header == nullptr)
        {
            OutError = TEXT("camera SHM writer is not open");
            return false;
        }
        if (!ValidateFrame(Metadata, Payload, PayloadSize, OutError))
        {
            return false;
        }
        if (LastSequenceValue >= (std::numeric_limits<uint64>::max() - 1ULL) / 2ULL)
        {
            OutError = TEXT("camera SHM sequence exhausted");
            return false;
        }

        const uint64 Sequence = LastSequenceValue + 1ULL;
        const uint16 SlotIndex = static_cast<uint16>((Sequence - 1ULL) % Config.SlotCount);
        FSlotHeader* Slot = SlotHeader(SlotIndex);
        uint8* Destination = SlotPayload(SlotIndex);
        const uint64 DirtyToken = Sequence * 2ULL + 1ULL;
        const uint64 CommittedToken = Sequence * 2ULL;

        StoreRelease64(&Slot->guard_begin, DirtyToken);
        StoreRelease64(&Slot->guard_end, 0ULL);
        Slot->sequence = Sequence;
        Slot->timestamp_ns = Metadata.TimestampNs == 0 ? UnixTimeNs() : Metadata.TimestampNs;
        Slot->width = Metadata.Width;
        Slot->height = Metadata.Height;
        Slot->stride = Metadata.Stride;
        Slot->payload_size = static_cast<uint32>(PayloadSize);
        Slot->payload_capacity = Config.SlotCapacity;
        Slot->payload_crc32 = PayloadSize == 0 ? 0U : Crc32(Payload, PayloadSize);
        Slot->flags = 0;
        Slot->reserved0 = 0;
        Slot->stream_kind = static_cast<uint16>(Metadata.StreamKind);
        Slot->encoding_size = static_cast<uint16>(FTCHARToUTF8(*Metadata.Encoding).Length());
        Slot->schema_version = SchemaVersion;
        Slot->header_size = sizeof(FSlotHeader);
        CopyAscii(Slot->encoding, sizeof(Slot->encoding), Metadata.Encoding);
        CopyAscii(Slot->frame_id, sizeof(Slot->frame_id), Metadata.FrameId);
        Slot->fx = Metadata.Fx;
        Slot->fy = Metadata.Fy;
        Slot->cx = Metadata.Cx;
        Slot->cy = Metadata.Cy;
        Slot->depth_scale = Metadata.DepthScale;
        Slot->dist_k1 = Metadata.DistK1;
        Slot->dist_k2 = Metadata.DistK2;
        Slot->dist_p1 = Metadata.DistP1;
        Slot->dist_p2 = Metadata.DistP2;
        Slot->dist_k3 = Metadata.DistK3;
        FMemory::Memzero(Slot->reserved1, sizeof(Slot->reserved1));
        if (PayloadSize > 0)
        {
            FMemory::Memcpy(Destination, Payload, PayloadSize);
        }

        std::atomic_thread_fence(std::memory_order_release);
        StoreRelease64(&Slot->guard_end, CommittedToken);
        StoreRelease64(&Slot->guard_begin, CommittedToken);
        StoreRelease64(&Header->writer_heartbeat_ns, UnixTimeNs());
        StoreRelease32(&Header->active_slot, SlotIndex);
        StoreRelease64(&Header->published_sequence, Sequence);
        LastSequenceValue = Sequence;
        return true;
    }

    void FFrameWriter::Close()
    {
#if PLATFORM_WINDOWS
        if (Mapping != nullptr)
        {
            ::UnmapViewOfFile(Mapping);
        }
        if (MappingHandle != nullptr)
        {
            ::CloseHandle(MappingHandle);
        }
#endif
        MappingHandle = nullptr;
        Mapping = nullptr;
        MappingSizeBytes = 0;
        Header = nullptr;
        LastSequenceValue = 0;
        Config = FWriterConfig{};
    }

    bool FFrameWriter::ValidateConfig(const FWriterConfig& Candidate, FString& OutError) const
    {
        bool bHasNul = false;
        for (int32 Index = 0; Index < Candidate.MappingName.Len(); ++Index)
        {
            if (Candidate.MappingName[Index] == TCHAR('\0'))
            {
                bHasNul = true;
                break;
            }
        }
        if (Candidate.MappingName.IsEmpty() || bHasNul)
        {
            OutError = TEXT("camera SHM mapping name must be non-empty and NUL-free");
            return false;
        }

        if (Candidate.SlotCount != DefaultSlotCount)
        {
            OutError = TEXT("camera SHM ABI v1 requires exactly two slots");
            return false;
        }
        if (Candidate.SlotCapacity == 0)
        {
            OutError = TEXT("camera SHM slot capacity must be positive");
            return false;
        }
        return true;
    }

    bool FFrameWriter::ValidateFrame(
        const FFrameMetadata& Metadata,
        const void* Payload,
        const uint64 PayloadSize,
        FString& OutError) const
    {
        if (Metadata.Generation != Config.Generation)
        {
            OutError = TEXT("camera SHM frame generation does not match the RunAllocation generation");
            return false;
        }
        if (Metadata.StreamKind != EStreamKind::Color && Metadata.StreamKind != EStreamKind::Depth)
        {
            OutError = TEXT("camera SHM writer only publishes COLOR or DEPTH frames");
            return false;
        }
        if (!IsAscii(Metadata.Encoding))
        {
            OutError = TEXT("camera SHM encoding must be ASCII");
            return false;
        }
        const FTCHARToUTF8 EncodingUtf8(*Metadata.Encoding);
        const FTCHARToUTF8 FrameIdUtf8(*Metadata.FrameId);
        if (EncodingUtf8.Length() > 15 || FrameIdUtf8.Length() > 63)
        {
            OutError = TEXT("camera SHM encoding or frame_id is too long");
            return false;
        }
        if (PayloadSize > Config.SlotCapacity || PayloadSize > std::numeric_limits<uint32>::max())
        {
            OutError = TEXT("camera SHM payload exceeds slot capacity");
            return false;
        }
        if (PayloadSize > 0 && Payload == nullptr)
        {
            OutError = TEXT("camera SHM payload pointer is null");
            return false;
        }
        if (Metadata.Width == 0 || Metadata.Height == 0 || Metadata.Stride == 0)
        {
            OutError = TEXT("camera image SHM dimensions are invalid");
            return false;
        }
        if (Metadata.Height != 0 && Metadata.Stride > std::numeric_limits<uint32>::max() / Metadata.Height)
        {
            OutError = TEXT("camera image SHM payload size overflows stride * height");
            return false;
        }
        if (PayloadSize != static_cast<uint64>(Metadata.Stride) * Metadata.Height)
        {
            OutError = TEXT("camera image SHM payload must equal stride * height");
            return false;
        }
        const uint32 Bpp = BytesPerPixel(Metadata.Encoding);
        if (Bpp == 0 || Metadata.Width > std::numeric_limits<uint32>::max() / Bpp || Metadata.Stride < Metadata.Width * Bpp)
        {
            OutError = TEXT("camera image SHM encoding or stride is invalid");
            return false;
        }
        return true;
    }

    bool FFrameWriter::InitializeOrValidateHeader(FString& OutError)
    {
        if (IsAllZero(Mapping, sizeof(FSharedHeader)))
        {
            FMemory::Memzero(Mapping, MappingSizeBytes);
            FMemory::Memcpy(Header->magic, Magic, sizeof(Magic));
            Header->schema_version = SchemaVersion;
            Header->header_size = sizeof(FSharedHeader);
            Header->slot_header_size = sizeof(FSlotHeader);
            Header->slot_count = Config.SlotCount;
            Header->slot_capacity = Config.SlotCapacity;
            Header->active_slot = 0;
            Header->flags = 0;
            Header->reserved0 = 0;
            Header->published_sequence = 0;
            Header->created_timestamp_ns = UnixTimeNs();
            Header->writer_heartbeat_ns = Header->created_timestamp_ns;
            Header->reserved1 = 0;
            return true;
        }

        if (FMemory::Memcmp(Header->magic, Magic, sizeof(Magic)) != 0
            || Header->schema_version != SchemaVersion
            || Header->header_size != sizeof(FSharedHeader)
            || Header->slot_header_size != sizeof(FSlotHeader)
            || Header->slot_count != Config.SlotCount
            || Header->slot_capacity != Config.SlotCapacity
            || Header->created_timestamp_ns == 0)
        {
            OutError = TEXT("existing camera SHM mapping is incompatible with lingtu.camera.shm_frame.v1");
            return false;
        }
        return true;
    }

    FSlotHeader* FFrameWriter::SlotHeader(const uint16 Index) const
    {
        uint8* Base = Mapping + sizeof(FSharedHeader);
        return reinterpret_cast<FSlotHeader*>(
            Base + static_cast<uint64>(Index) * (sizeof(FSlotHeader) + Config.SlotCapacity));
    }

    uint8* FFrameWriter::SlotPayload(const uint16 Index) const
    {
        return reinterpret_cast<uint8*>(SlotHeader(Index)) + sizeof(FSlotHeader);
    }
}

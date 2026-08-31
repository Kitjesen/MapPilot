#include "LingTuSimControlTransportRuntime.h"

#include "Common/UdpSocketBuilder.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformFileManager.h"
#include "HAL/PlatformTime.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"
#include "Interfaces/IPv4/IPv4Address.h"
#include "Interfaces/IPv4/IPv4Endpoint.h"
#include "LingTuSimControlTransport.h"
#include "LingTuSimSessionService.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
#include "Misc/ScopeLock.h"
#include "Serialization/ArrayReader.h"
#include "SocketSubsystem.h"
#include "Sockets.h"

#include <atomic>

DEFINE_LOG_CATEGORY_STATIC(LogLingTuSimControlTransport, Log, All);

namespace LingTuSim
{
    namespace
    {
        constexpr uint32 MaxControlDatagramBytes = 4'096;
        using FControlDatagram = TSharedPtr<FArrayReader, ESPMode::ThreadSafe>;

        int32 CountAssignment(const FString& CommandLine, const TCHAR* Name)
        {
            const FString Needle = FString::Printf(TEXT("-%s="), Name);
            int32 Count = 0;
            int32 Offset = 0;
            while (Offset < CommandLine.Len())
            {
                const int32 Found = CommandLine.Find(
                    Needle,
                    ESearchCase::CaseSensitive,
                    ESearchDir::FromStart,
                    Offset);
                if (Found == INDEX_NONE)
                {
                    break;
                }
                ++Count;
                Offset = Found + Needle.Len();
            }
            return Count;
        }

        bool ReadRequiredStringArgument(
            const FString& CommandLine,
            const TCHAR* Name,
            FString& OutValue,
            FString& OutError)
        {
            if (CountAssignment(CommandLine, Name) != 1
                || !FParse::Value(*CommandLine, *FString::Printf(TEXT("%s="), Name), OutValue)
                || OutValue.IsEmpty())
            {
                OutError = FString::Printf(
                    TEXT("playable control requires exactly one -%s argument"),
                    Name);
                return false;
            }
            return true;
        }

        bool ReadRequiredUint64Argument(
            const FString& CommandLine,
            const TCHAR* Name,
            uint64& OutValue,
            FString& OutError)
        {
            if (CountAssignment(CommandLine, Name) != 1
                || !FParse::Value(*CommandLine, *FString::Printf(TEXT("%s="), Name), OutValue))
            {
                OutError = FString::Printf(
                    TEXT("playable control requires exactly one -%s argument"),
                    Name);
                return false;
            }
            return true;
        }

        bool ReadRequiredPortArgument(
            const FString& CommandLine,
            const TCHAR* Name,
            uint16& OutPort,
            FString& OutError)
        {
            uint32 Value = 0;
            if (CountAssignment(CommandLine, Name) != 1
                || !FParse::Value(*CommandLine, *FString::Printf(TEXT("%s="), Name), Value)
                || Value == 0
                || Value > MAX_uint16)
            {
                OutError = FString::Printf(
                    TEXT("playable control requires exactly one valid -%s port"),
                    Name);
                return false;
            }
            OutPort = static_cast<uint16>(Value);
            return true;
        }

        bool CanonicalizeAllocationPath(
            const FString& RawPath,
            FString& OutAllocationPath,
            FString& OutLogDirectory,
            FString& OutError)
        {
            if (RawPath.IsEmpty() || FPaths::IsRelative(RawPath))
            {
                OutError = TEXT("LingTuRunAllocation must be an absolute path");
                return false;
            }
            FString Normalized = RawPath;
            FPaths::NormalizeFilename(Normalized);
            FString Collapsed = Normalized;
            if (!FPaths::CollapseRelativeDirectories(Collapsed)
                || Normalized != Collapsed
                || FPaths::GetCleanFilename(Collapsed) != TEXT("run-allocation.json"))
            {
                OutError = TEXT("LingTuRunAllocation must be a canonical run-allocation.json path");
                return false;
            }
            OutAllocationPath = FPaths::ConvertRelativePathToFull(Collapsed);
            FPaths::NormalizeFilename(OutAllocationPath);
            if (!FPaths::IsSamePath(OutAllocationPath, Collapsed))
            {
                OutError = TEXT("LingTuRunAllocation path is not canonical");
                return false;
            }

            IFileManager& FileManager = IFileManager::Get();
            const FString RunDirectory = FPaths::GetPath(OutAllocationPath);
            OutLogDirectory = FPaths::Combine(RunDirectory, TEXT("logs"));
            FPaths::NormalizeDirectoryName(OutLogDirectory);
            if (!FileManager.FileExists(*OutAllocationPath)
                || !FileManager.DirectoryExists(*RunDirectory)
                || !FileManager.DirectoryExists(*OutLogDirectory))
            {
                OutError = TEXT("RunAllocation file/run/log directories must already exist");
                return false;
            }
            if (FileManager.IsSymlink(*OutAllocationPath)
                || FileManager.IsSymlink(*RunDirectory)
                || FileManager.IsSymlink(*OutLogDirectory))
            {
                OutError = TEXT("RunAllocation file/run/log paths cannot be symlinks or reparse points");
                return false;
            }
            return true;
        }

        bool IsExactUtf8(const FControlDatagram& Data, FString& OutJson)
        {
            if (!Data.IsValid() || Data->IsEmpty())
            {
                return false;
            }
            const FUTF8ToTCHAR Converted(
                reinterpret_cast<const ANSICHAR*>(Data->GetData()),
                Data->Num());
            OutJson = FString(Converted.Length(), Converted.Get());
            const FTCHARToUTF8 RoundTrip(*OutJson);
            return RoundTrip.Length() == Data->Num()
                && FMemory::Memcmp(
                    RoundTrip.Get(),
                    Data->GetData(),
                    Data->Num()) == 0;
        }

        uint64 CurrentMonotonicNs()
        {
            return static_cast<uint64>(FPlatformTime::Seconds() * 1'000'000'000.0);
        }

        class FControlSendState final
        {
        public:
            FControlSendState(
                FSocket& InSocket,
                const FIPv4Endpoint& InDestination,
                FString InEvidencePath)
                : Socket(&InSocket)
                , Destination(InDestination)
                , EvidencePath(MoveTemp(InEvidencePath))
            {
            }

            bool Send(const TArray<uint8>& Datagram, FString& OutError)
            {
                FScopeLock Lock(&CriticalSection);
                if (bStopping || Socket == nullptr)
                {
                    OutError = TEXT("control intent socket is stopping");
                    return false;
                }
                if (Datagram.IsEmpty()
                    || Datagram.Num() > static_cast<int32>(MaxControlDatagramBytes))
                {
                    OutError = TEXT("control intent datagram is empty or oversized");
                    return false;
                }
                const TSharedRef<FInternetAddr> Address = Destination.ToInternetAddr();
                int32 BytesSent = 0;
                if (!Socket->SendTo(
                        Datagram.GetData(),
                        Datagram.Num(),
                        BytesSent,
                        *Address)
                    || BytesSent != Datagram.Num())
                {
                    OutError = FString::Printf(
                        TEXT("control intent send was incomplete: %d/%d bytes"),
                        BytesSent,
                        Datagram.Num());
                    return false;
                }
                return true;
            }

            bool AppendEvidence(const FString& EvidenceJson, FString& OutError)
            {
                FScopeLock Lock(&CriticalSection);
                if (bStopping || EvidenceJson.IsEmpty())
                {
                    OutError = TEXT("control origin evidence writer is stopping or empty");
                    return false;
                }
                IFileManager& FileManager = IFileManager::Get();
                if (FileManager.IsSymlink(*EvidencePath))
                {
                    OutError = TEXT("UE control origin evidence path is a symlink/reparse point");
                    return false;
                }
                const FString Line = EvidenceJson + LINE_TERMINATOR;
                if (!FFileHelper::SaveStringToFile(
                        Line,
                        *EvidencePath,
                        FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM,
                        &FileManager,
                        FILEWRITE_Append))
                {
                    OutError = TEXT("cannot append UE control origin evidence");
                    return false;
                }
                return true;
            }

            void Stop()
            {
                FScopeLock Lock(&CriticalSection);
                bStopping = true;
                Socket = nullptr;
            }

        private:
            FCriticalSection CriticalSection;
            FSocket* Socket = nullptr;
            FIPv4Endpoint Destination;
            FString EvidencePath;
            bool bStopping = false;
        };

        class FLingTuSimControlStatusReceiver final : public FRunnable
        {
        public:
            explicit FLingTuSimControlStatusReceiver(FSocket& InSocket)
                : Socket(InSocket)
                , SocketSubsystem(ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM))
            {
            }

            virtual ~FLingTuSimControlStatusReceiver() override
            {
                StopAndWait();
            }

            bool StartReceiver()
            {
                if (Thread != nullptr)
                {
                    return true;
                }
                bStopping.store(false, std::memory_order_release);
                Thread = FRunnableThread::Create(
                    this,
                    TEXT("LingTuSimControlStatusReceiver"),
                    128 * 1024,
                    TPri_Normal,
                    FPlatformAffinity::GetPoolThreadMask());
                return Thread != nullptr;
            }

            void StopAndWait()
            {
                Stop();
                if (Thread != nullptr)
                {
                    Thread->WaitForCompletion();
                    delete Thread;
                    Thread = nullptr;
                }
            }

            virtual uint32 Run() override
            {
                while (!bStopping.load(std::memory_order_acquire))
                {
                    if (!Socket.Wait(
                            ESocketWaitConditions::WaitForRead,
                            FTimespan::FromMilliseconds(5)))
                    {
                        continue;
                    }
                    uint32 PendingBytes = 0;
                    while (!bStopping.load(std::memory_order_acquire)
                        && Socket.HasPendingData(PendingBytes))
                    {
                        const bool bOversized = PendingBytes > MaxControlDatagramBytes;
                        FControlDatagram Data = MakeShared<FArrayReader, ESPMode::ThreadSafe>(true);
                        Data->SetNumUninitialized(static_cast<int32>(FMath::Min(
                            PendingBytes,
                            MaxControlDatagramBytes)));
                        TSharedRef<FInternetAddr> Sender = SocketSubsystem->CreateInternetAddr();
                        int32 BytesRead = 0;
                        if (!Socket.RecvFrom(
                                Data->GetData(),
                                Data->Num(),
                                BytesRead,
                                *Sender))
                        {
                            continue;
                        }
                        if (bOversized)
                        {
                            UE_LOG(
                                LogLingTuSimControlTransport,
                                Warning,
                                TEXT("Rejected oversized control ACK datagram: %u bytes"),
                                PendingBytes);
                            continue;
                        }
                        Data->RemoveAt(
                            BytesRead,
                            Data->Num() - BytesRead,
                            EAllowShrinking::No);
                        const FIPv4Endpoint Endpoint(Sender);
                        if (Endpoint.Address != FIPv4Address::InternalLoopback)
                        {
                            UE_LOG(
                                LogLingTuSimControlTransport,
                                Warning,
                                TEXT("Rejected non-loopback control ACK datagram"));
                            continue;
                        }
                        FString AckJson;
                        if (!IsExactUtf8(Data, AckJson))
                        {
                            UE_LOG(
                                LogLingTuSimControlTransport,
                                Warning,
                                TEXT("Rejected invalid UTF-8 control ACK datagram"));
                            continue;
                        }
                        FString Error;
                        if (!FSessionService::PublishControlResponseJson(AckJson, Error))
                        {
                            UE_LOG(
                                LogLingTuSimControlTransport,
                                Warning,
                                TEXT("Rejected control ACK datagram: %s"),
                                *Error);
                        }
                    }
                }
                return 0;
            }

            virtual void Stop() override
            {
                bStopping.store(true, std::memory_order_release);
            }

        private:
            FSocket& Socket;
            ISocketSubsystem* SocketSubsystem = nullptr;
            FRunnableThread* Thread = nullptr;
            std::atomic<bool> bStopping{false};
        };
    }

    bool FControlTransportProtocol::ValidateControlCommandLineShape(
        const FString& CommandLine,
        bool& bOutControlRequested,
        FString& OutError)
    {
        bOutControlRequested = false;
        OutError.Reset();
        const int32 IntentAssignments = CountAssignment(
            CommandLine,
            TEXT("LingTuControlIntentPort"));
        const int32 StatusAssignments = CountAssignment(
            CommandLine,
            TEXT("LingTuControlStatusPort"));
        const int32 SourceAssignments = CountAssignment(
            CommandLine,
            TEXT("LingTuControlSourceId"));
        const int32 LegacyHudAssignments = CountAssignment(
            CommandLine,
            TEXT("LingTuHudScreenshot"));
        const int32 DriveHudAssignments = CountAssignment(
            CommandLine,
            TEXT("LingTuHudDriveScreenshot"));
        const int32 TacticalHudAssignments = CountAssignment(
            CommandLine,
            TEXT("LingTuHudTacticalScreenshot"));
        const int32 MenuRecordingHudAssignments = CountAssignment(
            CommandLine,
            TEXT("LingTuHudMenuRecordingScreenshot"));
        const int32 ControlOnlyAssignments =
            IntentAssignments
            + StatusAssignments
            + SourceAssignments
            + LegacyHudAssignments
            + DriveHudAssignments
            + TacticalHudAssignments
            + MenuRecordingHudAssignments;
        if (ControlOnlyAssignments == 0)
        {
            return true;
        }
        if (LegacyHudAssignments != 0)
        {
            OutError = TEXT("legacy singular HUD screenshot argument is unsupported");
            return false;
        }
        if (IntentAssignments != 1 || StatusAssignments != 1)
        {
            OutError = TEXT(
                "playable control intent/status ports must appear exactly once together");
            return false;
        }
        if (SourceAssignments != 1
            || DriveHudAssignments != 1
            || TacticalHudAssignments != 1
            || MenuRecordingHudAssignments != 1)
        {
            OutError = TEXT(
                "playable control source and exact three HUD screenshot arguments must appear once");
            return false;
        }
        bOutControlRequested = true;
        return true;
    }

    class FLingTuSimControlTransportRuntime::FImpl final
    {
    public:
        ~FImpl()
        {
            Stop();
        }

        bool Start(
            const FSessionBundleView& Bundle,
            FString& OutError,
            bool& bOutEnabled)
        {
            bOutEnabled = false;
            const FString CommandLine(FCommandLine::Get());
            bool bControlRequested = false;
            if (!FControlTransportProtocol::ValidateControlCommandLineShape(
                    CommandLine,
                    bControlRequested,
                    OutError))
            {
                return false;
            }
            if (!bControlRequested)
            {
                return true;
            }

            uint16 IntentPort = 0;
            uint16 StatusPort = 0;
            uint64 ModelGeneration = 0;
            uint64 ResetGeneration = 0;
            FString RawAllocationPath;
            FString RunId;
            FString SourceId;
            if (!ReadRequiredPortArgument(
                    CommandLine,
                    TEXT("LingTuControlIntentPort"),
                    IntentPort,
                    OutError)
                || !ReadRequiredPortArgument(
                    CommandLine,
                    TEXT("LingTuControlStatusPort"),
                    StatusPort,
                    OutError)
                || !ReadRequiredStringArgument(
                    CommandLine,
                    TEXT("LingTuRunAllocation"),
                    RawAllocationPath,
                    OutError)
                || !ReadRequiredStringArgument(
                    CommandLine,
                    TEXT("LingTuRunId"),
                    RunId,
                    OutError)
                || !ReadRequiredStringArgument(
                    CommandLine,
                    TEXT("LingTuControlSourceId"),
                    SourceId,
                    OutError)
                || !ReadRequiredUint64Argument(
                    CommandLine,
                    TEXT("LingTuModelGeneration"),
                    ModelGeneration,
                    OutError)
                || !ReadRequiredUint64Argument(
                    CommandLine,
                    TEXT("LingTuResetGeneration"),
                    ResetGeneration,
                    OutError))
            {
                return false;
            }
            if (IntentPort == StatusPort)
            {
                OutError = TEXT("control intent and status ports must be distinct");
                return false;
            }

            FString AllocationPath;
            FString LogDirectory;
            if (!CanonicalizeAllocationPath(
                    RawAllocationPath,
                    AllocationPath,
                    LogDirectory,
                    OutError))
            {
                return false;
            }
            FString AllocationJson;
            if (!FFileHelper::LoadFileToString(AllocationJson, *AllocationPath))
            {
                OutError = TEXT("cannot read the control RunAllocation");
                return false;
            }
            FControlTransportBinding Binding;
            if (!FControlTransportProtocol::ParseRunAllocationJson(
                    AllocationJson,
                    RunId,
                    Bundle.SessionId,
                    LogDirectory,
                    IntentPort,
                    StatusPort,
                    ModelGeneration,
                    ResetGeneration,
                    SourceId,
                    Binding,
                    OutError))
            {
                return false;
            }
            if (IFileManager::Get().IsSymlink(*Binding.OriginEvidencePath))
            {
                OutError = TEXT("UE control origin evidence cannot target a symlink/reparse point");
                return false;
            }

            StatusSocket = FUdpSocketBuilder(TEXT("LingTuSimControlStatusIngress"))
                .AsNonBlocking()
                .BoundToAddress(FIPv4Address::InternalLoopback)
                .BoundToPort(StatusPort)
                .WithReceiveBufferSize(64 * 1024);
            if (StatusSocket == nullptr)
            {
                OutError = FString::Printf(
                    TEXT("cannot bind control ACK receiver to 127.0.0.1:%u"),
                    StatusPort);
                Stop();
                return false;
            }

            IntentSocket = FUdpSocketBuilder(TEXT("LingTuSimControlIntentEgress"))
                .AsNonBlocking();
            if (IntentSocket == nullptr)
            {
                OutError = TEXT("cannot create control intent loopback socket");
                Stop();
                return false;
            }

            SendState = MakeShared<FControlSendState, ESPMode::ThreadSafe>(
                *IntentSocket,
                FIPv4Endpoint(FIPv4Address::InternalLoopback, IntentPort),
                Binding.OriginEvidencePath);
            Sender = MakeShared<FLingTuSimOperatorIntentSender, ESPMode::ThreadSafe>(
                Binding,
                [State = SendState](const TArray<uint8>& Datagram, FString& Error)
                {
                    return State->Send(Datagram, Error);
                },
                [State = SendState](const FString& EvidenceJson, FString& Error)
                {
                    return State->AppendEvidence(EvidenceJson, Error);
                });
            if (!FSessionService::BindControlTransport(Sender.ToSharedRef()))
            {
                OutError = TEXT("cannot bind control sender to the active SessionBundle");
                Stop();
                return false;
            }

            StatusReceiver = MakeUnique<FLingTuSimControlStatusReceiver>(*StatusSocket);
            if (!StatusReceiver->StartReceiver())
            {
                OutError = TEXT("cannot start control ACK receiver thread");
                Stop();
                return false;
            }
            bOutEnabled = true;
            UE_LOG(
                LogLingTuSimControlTransport,
                Display,
                TEXT("LINGTU_PLAYABLE_CONTROL_READY run_id=%s intent_port=%u status_port=%u"),
                *Binding.RunId,
                IntentPort,
                StatusPort);
            return true;
        }

        void Stop()
        {
            if (Sender.IsValid())
            {
                FOperatorIntentSample Zero;
                Zero.InputMode = TEXT("drive");
                Zero.InputDevice = TEXT("keyboard");
                Zero.bViewportFocused = false;
                Zero.bDeadman = false;
                Zero.SourceMonotonicNs = CurrentMonotonicNs();
                FString EventId;
                FString Error;
                Sender->PublishOperatorIntent(Zero, EventId, Error);

                FOperatorRuntimeRequest Release;
                Release.Request = EOperatorRuntimeRequestType::ControlRelease;
                Release.SourceMonotonicNs = CurrentMonotonicNs();
                Sender->PublishRuntimeRequest(Release, EventId, Error);
            }

            FSessionService::UnbindControlTransport();
            if (StatusReceiver)
            {
                StatusReceiver->StopAndWait();
                StatusReceiver.Reset();
            }
            if (SendState.IsValid())
            {
                SendState->Stop();
                SendState.Reset();
            }
            Sender.Reset();

            ISocketSubsystem* SocketSubsystem =
                ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
            if (StatusSocket != nullptr)
            {
                SocketSubsystem->DestroySocket(StatusSocket);
                StatusSocket = nullptr;
            }
            if (IntentSocket != nullptr)
            {
                SocketSubsystem->DestroySocket(IntentSocket);
                IntentSocket = nullptr;
            }
        }

    private:
        FSocket* IntentSocket = nullptr;
        FSocket* StatusSocket = nullptr;
        TSharedPtr<FControlSendState, ESPMode::ThreadSafe> SendState;
        TSharedPtr<FLingTuSimOperatorIntentSender, ESPMode::ThreadSafe> Sender;
        TUniquePtr<FLingTuSimControlStatusReceiver> StatusReceiver;
    };

    FLingTuSimControlTransportRuntime::FLingTuSimControlTransportRuntime() = default;

    FLingTuSimControlTransportRuntime::~FLingTuSimControlTransportRuntime()
    {
        Stop();
    }

    bool FLingTuSimControlTransportRuntime::StartFromCommandLine(
        const FSessionBundleView& Bundle,
        FString& OutError)
    {
        Stop();
        Impl = MakeUnique<FImpl>();
        if (!Impl->Start(Bundle, OutError, bEnabled))
        {
            Impl.Reset();
            bEnabled = false;
            return false;
        }
        return true;
    }

    void FLingTuSimControlTransportRuntime::Stop()
    {
        bEnabled = false;
        if (Impl)
        {
            Impl->Stop();
            Impl.Reset();
        }
    }
}

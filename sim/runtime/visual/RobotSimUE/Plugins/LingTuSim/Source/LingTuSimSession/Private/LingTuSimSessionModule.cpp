#include "Modules/ModuleManager.h"

#include "Common/UdpSocketBuilder.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"
#include "Interfaces/IPv4/IPv4Address.h"
#include "Interfaces/IPv4/IPv4Endpoint.h"
#include "LingTuSimBundleLoader.h"
#include "LingTuSimControlTransportRuntime.h"
#include "LingTuSimSessionService.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Serialization/ArrayReader.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "SocketSubsystem.h"
#include "Sockets.h"

#include <atomic>

DEFINE_LOG_CATEGORY_STATIC(LogLingTuSimSession, Log, All);

namespace
{
    constexpr uint32 MaxSnapshotDatagramBytes = 60'000;
    using FSnapshotDatagram = TSharedPtr<FArrayReader, ESPMode::ThreadSafe>;

    class FLingTuSnapshotReceiver final : public FRunnable
    {
    public:
        using FDatagramHandler =
            TFunction<void(const FSnapshotDatagram&, const FIPv4Endpoint&)>;

        FLingTuSnapshotReceiver(
            FSocket& InSocket,
            FDatagramHandler InHandler)
            : Socket(InSocket)
            , Handler(MoveTemp(InHandler))
            , SocketSubsystem(ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM))
        {
        }

        virtual ~FLingTuSnapshotReceiver() override
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
                TEXT("LingTuSimSnapshotReceiver"),
                128 * 1024,
                TPri_AboveNormal,
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
                    const bool bOversized = PendingBytes > MaxSnapshotDatagramBytes;
                    FSnapshotDatagram Data =
                        MakeShared<FArrayReader, ESPMode::ThreadSafe>(true);
                    Data->SetNumUninitialized(static_cast<int32>(FMath::Min(
                        PendingBytes,
                        MaxSnapshotDatagramBytes)));

                    TSharedRef<FInternetAddr> Sender =
                        SocketSubsystem->CreateInternetAddr();
                    int32 BytesRead = 0;
                    if (Socket.RecvFrom(
                            Data->GetData(),
                            Data->Num(),
                            BytesRead,
                            *Sender))
                    {
                        if (bOversized)
                        {
                            UE_LOG(
                                LogLingTuSimSession,
                                Warning,
                                TEXT("Rejected oversized snapshot datagram: %u bytes"),
                                PendingBytes);
                            continue;
                        }
                        Data->RemoveAt(
                            BytesRead,
                            Data->Num() - BytesRead,
                            EAllowShrinking::No);
                        Handler(Data, FIPv4Endpoint(Sender));
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
        FDatagramHandler Handler;
        ISocketSubsystem* SocketSubsystem = nullptr;
        FRunnableThread* Thread = nullptr;
        std::atomic<bool> bStopping{false};
    };

    class FLingTuSimSessionModule final : public IModuleInterface
    {
    public:
        virtual void StartupModule() override
        {
            LingTuSim::FSessionService::UnbindSession();

            FString BundleDirectory;
            uint32 Port = 0;
            uint64 ModelGeneration = 0;
            if (!FParse::Value(
                    FCommandLine::Get(),
                    TEXT("LingTuBundle="),
                    BundleDirectory)
                || BundleDirectory.IsEmpty()
                || !FParse::Value(
                    FCommandLine::Get(),
                    TEXT("LingTuSnapshotPort="),
                    Port))
            {
                return;
            }
            FParse::Value(
                FCommandLine::Get(),
                TEXT("LingTuModelGeneration="),
                ModelGeneration);
            if (Port == 0 || Port > MAX_uint16)
            {
                UE_LOG(
                    LogLingTuSimSession,
                    Error,
                    TEXT("LingTuSnapshotPort must be between 1 and 65535."));
                return;
            }

            LingTuSim::FRuntimeLoadError LoadError;
            if (!LingTuSim::FSessionBundleLoader::LoadSessionBundle(
                    BundleDirectory,
                    Bundle,
                    LoadError))
            {
                UE_LOG(
                    LogLingTuSimSession,
                    Error,
                    TEXT("Cannot bind SessionBundle '%s': %s"),
                    *BundleDirectory,
                    *LoadError.Message);
                return;
            }

            Socket = FUdpSocketBuilder(TEXT("LingTuSimSnapshotIngress"))
                .AsNonBlocking()
                .AsReusable()
                .BoundToAddress(FIPv4Address::InternalLoopback)
                .BoundToPort(static_cast<uint16>(Port))
                .WithReceiveBufferSize(1 << 20);
            if (Socket == nullptr)
            {
                UE_LOG(
                    LogLingTuSimSession,
                    Error,
                    TEXT("Cannot bind snapshot UDP receiver to 127.0.0.1:%u."),
                    Port);
                return;
            }

            Receiver = MakeUnique<FLingTuSnapshotReceiver>(
                *Socket,
                [this](
                    const FSnapshotDatagram& Data,
                    const FIPv4Endpoint& Endpoint)
                {
                    HandleDatagram(Data, Endpoint);
                });
            if (!Receiver->StartReceiver())
            {
                UE_LOG(
                    LogLingTuSimSession,
                    Error,
                    TEXT("Cannot start snapshot UDP receiver thread."));
                StopTransport();
                return;
            }
            if (!LingTuSim::FSessionService::RebindSession(
                    Bundle.SessionId,
                    ModelGeneration))
            {
                UE_LOG(
                    LogLingTuSimSession,
                    Error,
                    TEXT("Cannot commit truth-snapshot session binding."));
                StopTransport();
                LingTuSim::FSessionService::UnbindSession();
                return;
            }
            ControlTransport = MakeUnique<LingTuSim::FLingTuSimControlTransportRuntime>();
            FString ControlError;
            if (!ControlTransport->StartFromCommandLine(Bundle, ControlError))
            {
                UE_LOG(
                    LogLingTuSimSession,
                    Error,
                    TEXT("Cannot bind playable control transport: %s"),
                    *ControlError);
                StopTransport();
                LingTuSim::FSessionService::UnbindSession();
                return;
            }
            UE_LOG(
                LogLingTuSimSession,
                Display,
                TEXT("LINGTU_LIVE_SNAPSHOT_READY digest=%s model_generation=%llu port=%u playable_control=%s"),
                *Bundle.SessionId,
                static_cast<unsigned long long>(ModelGeneration),
                Port,
                ControlTransport->IsEnabled() ? TEXT("true") : TEXT("false"));
        }

        virtual void ShutdownModule() override
        {
            StopTransport();
            LingTuSim::FSessionService::UnbindSession();
        }

    private:
        void StopTransport()
        {
            if (ControlTransport)
            {
                ControlTransport->Stop();
                ControlTransport.Reset();
            }
            if (Receiver)
            {
                Receiver->StopAndWait();
                Receiver.Reset();
            }
            if (Socket != nullptr)
            {
                ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(Socket);
                Socket = nullptr;
            }
        }

        void HandleDatagram(
            const FSnapshotDatagram& Data,
            const FIPv4Endpoint& Endpoint)
        {
            if (!Data.IsValid()
                || Data->IsEmpty()
                || Data->Num() > static_cast<int32>(MaxSnapshotDatagramBytes)
                || Endpoint.Address != FIPv4Address::InternalLoopback)
            {
                return;
            }

            const FUTF8ToTCHAR Converted(
                reinterpret_cast<const ANSICHAR*>(Data->GetData()),
                Data->Num());
            const FString SnapshotJson(Converted.Length(), Converted.Get());
            TSharedPtr<FJsonObject> RootObject;
            const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(SnapshotJson);
            if (!FJsonSerializer::Deserialize(Reader, RootObject) || !RootObject.IsValid())
            {
                UE_LOG(
                    LogLingTuSimSession,
                    Warning,
                    TEXT("Rejected snapshot datagram: invalid JSON object"));
                return;
            }
            FString Schema;
            if (!RootObject->TryGetStringField(TEXT("schema"), Schema))
            {
                UE_LOG(
                    LogLingTuSimSession,
                    Warning,
                    TEXT("Rejected snapshot datagram: missing schema"));
                return;
            }

            if (Schema == TEXT("lingtu.sim.scenario-snapshot.v1"))
            {
                LingTuSim::ESnapshotPublishResult PublishResult =
                    LingTuSim::ESnapshotPublishResult::Stale;
                FString PublishError;
                if (!LingTuSim::FSessionService::PublishScenarioSnapshotJson(
                        SnapshotJson,
                        PublishResult,
                        PublishError))
                {
                    UE_LOG(
                        LogLingTuSimSession,
                        Warning,
                        TEXT("Rejected scenario-snapshot datagram: %s"),
                        *PublishError);
                    return;
                }
                if (!bLoggedFirstScenarioSnapshot
                    && (PublishResult == LingTuSim::ESnapshotPublishResult::Accepted
                        || PublishResult == LingTuSim::ESnapshotPublishResult::Replaced))
                {
                    bLoggedFirstScenarioSnapshot = true;
                    UE_LOG(
                        LogLingTuSimSession,
                        Display,
                        TEXT("LINGTU_SCENARIO_SNAPSHOT_FIRST"));
                }
                return;
            }
            if (Schema != TEXT("lingtu.sim.truth-snapshot.v1"))
            {
                UE_LOG(
                    LogLingTuSimSession,
                    Warning,
                    TEXT("Rejected snapshot datagram: unsupported schema '%s'"),
                    *Schema);
                return;
            }

            LingTuSim::FSnapshotEnvelope Snapshot;
            LingTuSim::FRuntimeLoadError LoadError;
            LingTuSim::ESnapshotPublishResult PublishResult =
                LingTuSim::ESnapshotPublishResult::Stale;
            if (!LingTuSim::FSessionService::PublishSnapshotJson(
                    SnapshotJson,
                    Snapshot,
                    PublishResult,
                    LoadError))
            {
                UE_LOG(
                    LogLingTuSimSession,
                    Warning,
                    TEXT("Rejected truth-snapshot datagram: %s"),
                    *LoadError.Message);
                return;
            }
            if (!bLoggedFirstSnapshot
                && (PublishResult == LingTuSim::ESnapshotPublishResult::Accepted
                    || PublishResult == LingTuSim::ESnapshotPublishResult::Replaced))
            {
                bLoggedFirstSnapshot = true;
                UE_LOG(
                    LogLingTuSimSession,
                    Display,
                    TEXT("LINGTU_LIVE_SNAPSHOT_FIRST reset_generation=%llu sequence=%llu entities=%d"),
                    static_cast<unsigned long long>(Snapshot.ResetGeneration),
                    static_cast<unsigned long long>(Snapshot.Sequence),
                    Snapshot.Entities.Num());
            }
        }

        LingTuSim::FSessionBundleView Bundle;
        FSocket* Socket = nullptr;
        TUniquePtr<FLingTuSnapshotReceiver> Receiver;
        TUniquePtr<LingTuSim::FLingTuSimControlTransportRuntime> ControlTransport;
        bool bLoggedFirstSnapshot = false;
        bool bLoggedFirstScenarioSnapshot = false;
    };
}

IMPLEMENT_MODULE(FLingTuSimSessionModule, LingTuSimSession);

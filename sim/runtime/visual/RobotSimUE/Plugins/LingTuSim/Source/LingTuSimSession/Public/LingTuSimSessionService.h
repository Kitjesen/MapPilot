#pragma once

#include "LingTuSimBundleLoader.h"
#include "LingTuSimControlTransport.h"
#include "LingTuSimSnapshotMailbox.h"

namespace LingTuSim
{
    /** Process-local access to services owned by the LingTuSimSession module. */
    class LINGTUSIMSESSION_API FSessionService final
    {
    public:
        /** Atomically replaces the process binding and clears all ingress state. */
        static bool RebindSession(FString SessionId, uint64 ModelGeneration);

        /** Atomically removes the process binding and clears all ingress state. */
        static void UnbindSession();

        /** Returns the current process-local session binding, if one exists. */
        static bool GetBoundSession(FString& OutSessionId, uint64& OutModelGeneration);

        /** Publishes one immutable snapshot without exposing mailbox storage. */
        static ESnapshotPublishResult PublishSnapshot(const FSnapshotEnvelope& Snapshot);

        /**
         * Parses a transport payload before classifying it against the active
         * binding. A valid next-generation payload is retained as rebind
         * evidence even when its session id has changed.
         */
        static bool PublishSnapshotJson(
            const FString& SnapshotJson,
            FSnapshotEnvelope& OutSnapshot,
            ESnapshotPublishResult& OutPublishResult,
            FRuntimeLoadError& OutError);

        /**
         * Validates and publishes one canonical ScenarioSnapshot JSON value.
         *
         * The original JSON is retained in a capacity-one mailbox so the
         * visual runtime can perform complete entity validation on the game
         * thread without introducing a second scenario payload type.
         */
        static bool PublishScenarioSnapshotJson(
            const FString& SnapshotJson,
            ESnapshotPublishResult& OutPublishResult,
            FString& OutError);

        /** Copies and consumes the current latest snapshot. */
        static bool TryTakeLatestSnapshot(FSnapshotEnvelope& OutSnapshot);

        /** Copies and consumes evidence that ingress observed a future model. */
        static bool TryTakeLatestFutureSnapshot(FSnapshotEnvelope& OutSnapshot);

        /** Copies and consumes the latest canonical ScenarioSnapshot JSON. */
        static bool TryTakeLatestScenarioSnapshotJson(FString& OutSnapshotJson);

        /** Drops pending snapshots and future-generation evidence. */
        static void ClearSnapshots();

        /** Installs the one allocation-bound control sender owned by this process. */
        static bool BindControlTransport(
            const TSharedRef<FLingTuSimOperatorIntentSender, ESPMode::ThreadSafe>& Sender);

        /** Removes the control sender and clears correlated ACK state. */
        static void UnbindControlTransport();

        /** Publishes one UI motion sample through the allocation-bound sender. */
        static bool PublishOperatorIntent(
            const FOperatorIntentSample& Sample,
            FString& OutEventId,
            FString& OutError);

        /** Publishes one non-droppable runtime request through the same source ordering. */
        static bool PublishRuntimeRequest(
            const FOperatorRuntimeRequest& Request,
            FString& OutEventId,
            FString& OutError);

        /** Validates and stores one minimal correlated ACK from the loopback receiver. */
        static bool PublishControlAckJson(
            const FString& AckJson,
            FString& OutError);

        /** Strict schema switch for ACK v1 and full status v1 on one UDP port. */
        static bool PublishControlResponseJson(
            const FString& ResponseJson,
            FString& OutError);

        /** Validates, correlates, timestamps, and stores one full runtime status. */
        static bool PublishControlStatusJson(
            const FString& StatusJson,
            FString& OutError);

        /** Copies, without consuming, the latest allocation/current-generation ACK. */
        static bool GetLatestControlAck(FControlAckEnvelope& OutAck);

        /** Copies, without consuming, the latest fresh-checkable full status. */
        static bool GetLatestControlStatus(FControlStatusEnvelope& OutStatus);

        /** Copies the current allocation-bound transport identity. */
        static bool GetControlTransportBinding(FControlTransportBinding& OutBinding);

    private:
        static FSnapshotMailbox& GetSnapshotMailbox();
        FSessionService() = delete;
    };
}

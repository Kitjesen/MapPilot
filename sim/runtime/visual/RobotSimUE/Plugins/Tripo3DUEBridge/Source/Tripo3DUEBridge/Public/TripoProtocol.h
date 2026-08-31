// Copyright Tripo3D. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "TripoProtocol.generated.h"

/**
 * Protocol constants for Tripo3D WebSocket communication
 */
namespace TripoProtocol
{
	// WebSocket connection settings
	inline const FString SERVER_HOST = TEXT("127.0.0.1");
	inline constexpr int32 SERVER_PORT = 60620;
	inline const FString PROTOCOL_VERSION = TEXT("1.0.0");

	// Incoming message type enum for O(1) routing
	enum class EMessageType : uint8
	{
		Handshake,
		Ping,
		FileTransfer,
		Unknown
	};

	inline EMessageType ParseMessageType(const FString& TypeString)
	{
		// Ordered by frequency: file_transfer is the hot path
		if (TypeString == TEXT("file_transfer")) return EMessageType::FileTransfer;
		if (TypeString == TEXT("ping")) return EMessageType::Ping;
		if (TypeString == TEXT("handshake")) return EMessageType::Handshake;
		return EMessageType::Unknown;
	}

	// Message type strings (used for JSON serialization in outgoing messages)
	namespace MessageType
	{
		inline const FString MSG_HANDSHAKE = TEXT("handshake");
		inline const FString MSG_HANDSHAKE_ACK = TEXT("handshake_ack");
		inline const FString MSG_PING = TEXT("ping");
		inline const FString MSG_PONG = TEXT("pong");
		inline const FString MSG_FILE_TRANSFER = TEXT("file_transfer");
		inline const FString MSG_FILE_TRANSFER_ACK = TEXT("file_transfer_ack");
		inline const FString MSG_FILE_TRANSFER_COMPLETE = TEXT("file_transfer_complete");
		inline const FString MSG_IMPORT_COMPLETE = TEXT("import_complete");
	}

	// Error codes
	namespace ErrorCode
	{
		inline constexpr int32 SUCCESS = 0;
		inline constexpr int32 ERROR_INVALID_JSON = 1001;
		inline constexpr int32 ERROR_PROCESSING = 1002;
	}

	// File transfer settings
	inline constexpr int32 CHUNK_SIZE = 5 * 1024 * 1024; // 5MB
	inline constexpr int32 MAX_RETRIES = 5;
	inline constexpr float RETRY_DELAY = 0.1f; // 100ms

	// Asset paths
	inline const FString ASSET_IMPORT_ROOT = TEXT("/Game/TripoModels");
	inline const FString TEMP_DIR_NAME = TEXT("Tripo3D");
}

/**
 * Message structures for JSON serialization
 */

// Handshake message from client
USTRUCT()
struct FTripoHandshakePayload
{
	GENERATED_BODY()

	UPROPERTY()
	FString clientName;

	UPROPERTY()
	FString protocolVersion;
};

// Handshake acknowledgment from server
USTRUCT()
struct FTripoHandshakeAckPayload
{
	GENERATED_BODY()

	UPROPERTY()
	bool success = false;

	UPROPERTY()
	FString clientName;

	UPROPERTY()
	FString dccVersion;

	UPROPERTY()
	FString pluginVersion;

	UPROPERTY()
	FString protocolVersion;
};

// File transfer message payload
USTRUCT()
struct FTripoFileTransferPayload
{
	GENERATED_BODY()

	UPROPERTY()
	FString fileId;

	UPROPERTY()
	FString fileName;

	UPROPERTY()
	FString fileType;

	UPROPERTY()
	int32 chunkIndex = 0;

	UPROPERTY()
	int32 chunkTotal = 0;

	UPROPERTY()
	int32 chunkSize = 0;
};

// File transfer acknowledgment payload
USTRUCT()
struct FTripoFileTransferAckPayload
{
	GENERATED_BODY()

	UPROPERTY()
	bool success = false;

	UPROPERTY()
	FString fileId;

	UPROPERTY()
	int32 fileIndex = 0;

	UPROPERTY()
	int32 code = 0;
};

// File transfer complete payload
USTRUCT()
struct FTripoFileTransferCompletePayload
{
	GENERATED_BODY()

	UPROPERTY()
	FString fileId;

	UPROPERTY()
	FString status;

	UPROPERTY()
	FString message;
};

// Import complete payload
USTRUCT()
struct FTripoImportCompletePayload
{
	GENERATED_BODY()

	UPROPERTY()
	FString fileId;

	UPROPERTY()
	bool success = false;

	UPROPERTY()
	FString message;
};

// Copyright Tripo3D. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "TripoProtocol.h"
#include "ITripoWebSocket.h"

// Forward declarations
class FTripoFileTransferManager;

DECLARE_MULTICAST_DELEGATE_TwoParams(FOnConnectionStatusChanged, bool /* bIsConnected */, const FString& /* ClientInfo */);
DECLARE_MULTICAST_DELEGATE_OneParam(FOnMessageReceived, const FString& /* Message */);

/**
 * WebSocket server for Tripo3D bridge communication
 * Singleton class managing WebSocket connections and message routing
 */
class TRIPO3DUEBRIDGE_API FTripoWebSocketServer
{
public:
	/** Get singleton instance */
	static FTripoWebSocketServer& Get();

	/** Start the WebSocket server */
	bool StartServer();

	/** Stop the WebSocket server */
	void StopServer();

	/** Check if server is running */
	bool IsServerRunning() const { return bIsServerRunning; }

	/** Check if a client is connected */
	bool IsClientConnected() const { return bIsClientConnected; }

	/** Get connected client information */
	FString GetClientInfo() const { return ConnectedClientInfo; }

	/** Send a message to the connected client */
	void SendMessage(const FString& JsonMessage);

	/** Send a binary message (JSON + binary data) */
	void SendBinaryMessage(const FString& JsonMessage, const TArray<uint8>& BinaryData);

	/** Delegates */
	FOnConnectionStatusChanged OnConnectionStatusChanged;
	FOnMessageReceived OnMessageReceived;

private:
	FTripoWebSocketServer();
	~FTripoWebSocketServer();

	// Prevent copying
	FTripoWebSocketServer(const FTripoWebSocketServer&) = delete;
	FTripoWebSocketServer& operator=(const FTripoWebSocketServer&) = delete;

	/** WebSocket 消息处理回调 */
	void OnWebSocketMessage(const FString& ClientId, TSharedPtr<ITripoWebSocketConnection> Connection, const FTripoWebSocketMessage& Message);

	/** Process incoming message (called on WebSocket thread) */
	void ProcessMessageOnWSThread(const TArray<uint8>& MessageData);

	/** Process incoming message */
	/** @note Called on WebSocket thread for data messages */
	void ProcessMessage(const TArray<uint8>& MessageData);

	/** Parse JSON header from binary message */
	int32 FindJsonEnd(const TArray<uint8>& Data);

	/** Handle different message types */
	void HandleHandshake(const TSharedPtr<FJsonObject>& JsonObject);
	void HandlePing();
	void HandleFileTransfer(const TSharedPtr<FJsonObject>& JsonObject, const TArray<uint8>& ChunkData);

	/** Send handshake acknowledgment */
	void SendHandshakeAck(bool bSuccess);

	/** Send file transfer acknowledgment */
	void SendFileTransferAck(const FString& FileId, int32 FileIndex, bool bSuccess, int32 ErrorCode = 0);

	/** Send file transfer complete message */
	void SendFileTransferComplete(const FString& FileId, const FString& Status, const FString& Message);

	/** Send import complete message */
	void SendImportComplete(const FString& FileId, bool bSuccess, const FString& Message);

private:
	/** Server running state */
	bool bIsServerRunning;

	/** Client connected state */
	bool bIsClientConnected;

	/** Connected client information string */
	FString ConnectedClientInfo;

	/** File transfer manager */
	TSharedPtr<FTripoFileTransferManager> FileTransferManager;

	/** WebSocket server interface (isolated by abstract interface) */
	TSharedPtr<ITripoWebSocketServer> WebSocketServer;

	/** Current client connection (isolated by abstract interface) */
	TSharedPtr<ITripoWebSocketConnection> ClientConnection;

	/** Current client ID */
	FString CurrentClientId;

	friend class FTripoFileTransferManager;
};

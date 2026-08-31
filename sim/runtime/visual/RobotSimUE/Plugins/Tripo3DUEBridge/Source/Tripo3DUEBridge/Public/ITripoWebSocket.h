// Copyright Tripo3D. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

/**
 * WebSocket message type
 */
enum class ETripoWebSocketMessageType : uint8
{
	Open,       // Connection established
	Close,      // Connection closed
	Message,    // Received a message (text or binary)
	Error       // An error occurred
};

/**
 * WebSocket close information
 */
struct FTripoWebSocketCloseInfo
{
	int32 Code = 0;
	FString Reason;
};

/**
 * WebSocket message payload
 */
struct FTripoWebSocketMessage
{
	ETripoWebSocketMessageType Type;
	TArray<uint8> Data;           // Message payload (text or binary)
	bool bIsBinary = false;       // Whether the message is binary
	FTripoWebSocketCloseInfo CloseInfo;  // Close event details
	FString ErrorMessage;         // Error details
};

/**
 * WebSocket connection interface
 * Abstracts a single client connection
 */
class TRIPO3DUEBRIDGE_API ITripoWebSocketConnection
{
public:
	virtual ~ITripoWebSocketConnection() = default;

	/** Send a text message */
	virtual void SendText(const FString& Message) = 0;

	/** Send a binary message */
	virtual void SendBinary(const TArray<uint8>& Data) = 0;

	/** Send a combined message (JSON + binary data) */
	virtual void SendBinary(const FString& JsonMessage, const TArray<uint8>& BinaryData) = 0;

	/** Get the remote IP address */
	virtual FString GetRemoteIp() const = 0;

	/** Close the connection */
	virtual void Close() = 0;
};

/**
 * WebSocket server callback delegate
 */
DECLARE_DELEGATE_ThreeParams(FOnTripoWebSocketMessage,
	const FString& /* ClientId */,
	TSharedPtr<ITripoWebSocketConnection> /* Connection */,
	const FTripoWebSocketMessage& /* Message */);

/**
 * WebSocket server interface
 * Abstracts the WebSocket server implementation so the backend can be swapped
 */
class TRIPO3DUEBRIDGE_API ITripoWebSocketServer
{
public:
	virtual ~ITripoWebSocketServer() = default;

	/** Start listening */
	virtual bool Start(int32 Port, const FString& Host = TEXT("127.0.0.1")) = 0;

	/** Stop the server */
	virtual void Stop() = 0;

	/** Check whether the server is running */
	virtual bool IsRunning() const = 0;

	/** Set the message callback */
	virtual void SetOnMessage(FOnTripoWebSocketMessage Callback) = 0;

	/** Get the listening port */
	virtual int32 GetPort() const = 0;

	/** Get the listening host */
	virtual FString GetHost() const = 0;
};


class TRIPO3DUEBRIDGE_API FTripoWebSocketFactory
{
public:
	static TSharedPtr<ITripoWebSocketServer> CreateServer();
};

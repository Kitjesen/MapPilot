#pragma once

#include "CoreMinimal.h"
#include "ITripoWebSocket.h"

// IXWebSocket includes - need full definition for callback signatures
#include "IXWebSocketServer.h"
#include "IXWebSocket.h"
#include "IXConnectionState.h"

/**
 * IXWebSocket
 */
class FTripoWebSocketConnectionImpl : public ITripoWebSocketConnection, public TSharedFromThis<FTripoWebSocketConnectionImpl>
{
public:
	FTripoWebSocketConnectionImpl(ix::WebSocket* InWebSocket, const FString& InRemoteIp);
	virtual ~FTripoWebSocketConnectionImpl() override;

	// ITripoWebSocketConnection interface
	virtual void SendText(const FString& Message) override;
	virtual void SendBinary(const TArray<uint8>& Data) override;
	virtual void SendBinary(const FString& JsonMessage, const TArray<uint8>& BinaryData) override;
	virtual FString GetRemoteIp() const override;
	virtual void Close() override;

	/** Check if the connection is valid */
	bool IsValid() const { return WebSocket != nullptr; }

	/** Invalidate the connection (when the connection is closed) */
	void Invalidate() { WebSocket = nullptr; }

private:
	ix::WebSocket* WebSocket;
	FString RemoteIp;
};

/**
 * IXWebSocket server implementation
 */
class FTripoWebSocketServerImpl : public ITripoWebSocketServer
{
public:
	FTripoWebSocketServerImpl();
	virtual ~FTripoWebSocketServerImpl() override;

	// ITripoWebSocketServer interface
	virtual bool Start(int32 Port, const FString& Host = TEXT("127.0.0.1")) override;
	virtual void Stop() override;
	virtual bool IsRunning() const override;
	virtual void SetOnMessage(FOnTripoWebSocketMessage Callback) override;
	virtual int32 GetPort() const override { return ListenPort; }
	virtual FString GetHost() const override { return ListenHost; }

private:
	/** Internal message processing callback */
	void OnClientMessage(
		std::shared_ptr<ix::ConnectionState> ConnectionState,
		ix::WebSocket& WebSocket,
		const ix::WebSocketMessagePtr& Msg);

	/** Find or create a connection wrapper */
	TSharedPtr<FTripoWebSocketConnectionImpl> GetOrCreateConnection(ix::WebSocket* WebSocket, const FString& RemoteIp);

	/** Remove a connection wrapper */
	void RemoveConnection(ix::WebSocket* WebSocket);

	/** Generate a client ID */
	FString GenerateClientId(const FString& RemoteIp);

private:
	ix::WebSocketServer* Server;
	int32 ListenPort;
	FString ListenHost;
	bool bIsRunning;

	/** Message callback */
	FOnTripoWebSocketMessage OnMessageCallback;

	/** Active connection mapping (WebSocket pointer -> connection wrapper) */
	TMap<ix::WebSocket*, TSharedPtr<FTripoWebSocketConnectionImpl>> Connections;

	/** Client ID mapping */
	TMap<ix::WebSocket*, FString> ClientIds;

	/** Thread-safe lock */
	FCriticalSection ConnectionsLock;
};

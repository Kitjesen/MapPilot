#include "TripoWebSocketImpl.h"

#ifdef SendMessage
#undef SendMessage
#endif

// ————— FTripoWebSocketConnectionImpl —————

FTripoWebSocketConnectionImpl::FTripoWebSocketConnectionImpl(ix::WebSocket* InWebSocket, const FString& InRemoteIp)
	: WebSocket(InWebSocket)
	, RemoteIp(InRemoteIp)
{
}

FTripoWebSocketConnectionImpl::~FTripoWebSocketConnectionImpl()
{
	WebSocket = nullptr;
}

void FTripoWebSocketConnectionImpl::SendText(const FString& Message)
{
	if (!WebSocket)
	{
		UE_LOG(LogTemp, Warning, TEXT("[Tripo3D WebSocket] Cannot send text: connection is invalid"));
		return;
	}

	FTCHARToUTF8 Converter(*Message);
	std::string Utf8Message(Converter.Get(), Converter.Length());
	WebSocket->send(Utf8Message);
}

void FTripoWebSocketConnectionImpl::SendBinary(const TArray<uint8>& Data)
{
	if (!WebSocket)
	{
		UE_LOG(LogTemp, Warning, TEXT("[Tripo3D WebSocket] Cannot send binary: connection is invalid"));
		return;
	}

	std::string BinaryData(reinterpret_cast<const char*>(Data.GetData()), Data.Num());
	WebSocket->sendBinary(BinaryData);
}

void FTripoWebSocketConnectionImpl::SendBinary(const FString& JsonMessage, const TArray<uint8>& BinaryData)
{
	if (!WebSocket)
	{
		UE_LOG(LogTemp, Warning, TEXT("[Tripo3D WebSocket] Cannot send binary: connection is invalid"));
		return;
	}

	// Combine JSON string + binary data
	FTCHARToUTF8 JsonConverter(*JsonMessage);

	std::string Message;
	Message.append(JsonConverter.Get(), JsonConverter.Length());
	Message.append(reinterpret_cast<const char*>(BinaryData.GetData()), BinaryData.Num());

	WebSocket->sendBinary(Message);
}

FString FTripoWebSocketConnectionImpl::GetRemoteIp() const
{
	return RemoteIp;
}

void FTripoWebSocketConnectionImpl::Close()
{
	if (WebSocket)
	{
		WebSocket->close();
	}
}

// ————— FTripoWebSocketServerImpl —————

FTripoWebSocketServerImpl::FTripoWebSocketServerImpl()
	: Server(nullptr)
	, ListenPort(0)
	, bIsRunning(false)
{
}

FTripoWebSocketServerImpl::~FTripoWebSocketServerImpl()
{
	Stop();
}

bool FTripoWebSocketServerImpl::Start(int32 Port, const FString& Host)
{
	if (bIsRunning)
	{
		UE_LOG(LogTemp, Warning, TEXT("[Tripo3D WebSocket] Server is already running"));
		return false;
	}

	ListenPort = Port;
	ListenHost = Host;

	// Create IXWebSocket server
	Server = new ix::WebSocketServer(Port, TCHAR_TO_UTF8(*Host));

	// Set callback using lambda that captures 'this'
	Server->setOnClientMessageCallback(
		[this](std::shared_ptr<ix::ConnectionState> ConnectionState,
			   ix::WebSocket& WebSocket,
			   const ix::WebSocketMessagePtr& Msg)
		{
			OnClientMessage(ConnectionState, WebSocket, Msg);
		}
	);

	// Start listening
	auto Result = Server->listen();
	if (!Result.first)
	{
		UE_LOG(LogTemp, Error, TEXT("[Tripo3D WebSocket] Failed to start server: %s"),
			UTF8_TO_TCHAR(Result.second.c_str()));
		delete Server;
		Server = nullptr;
		return false;
	}

	Server->start();
	bIsRunning = true;

	UE_LOG(LogTemp, Log, TEXT("[Tripo3D WebSocket] Server started on %s:%d"), *Host, Port);
	return true;
}

void FTripoWebSocketServerImpl::Stop()
{
	if (!bIsRunning || !Server)
	{
		return;
	}

	UE_LOG(LogTemp, Log, TEXT("[Tripo3D WebSocket] Stopping server..."));

	// Stop server
	Server->stop();
	delete Server;
	Server = nullptr;

	// Clear connections
	{
		FScopeLock Lock(&ConnectionsLock);
		for (auto& Pair : Connections)
		{
			if (Pair.Value.IsValid())
			{
				Pair.Value->Invalidate();
			}
		}
		Connections.Empty();
		ClientIds.Empty();
	}

	bIsRunning = false;

	UE_LOG(LogTemp, Log, TEXT("[Tripo3D WebSocket] Server stopped"));
}

bool FTripoWebSocketServerImpl::IsRunning() const
{
	return bIsRunning;
}

void FTripoWebSocketServerImpl::SetOnMessage(FOnTripoWebSocketMessage Callback)
{
	OnMessageCallback = Callback;
}

void FTripoWebSocketServerImpl::OnClientMessage(
	std::shared_ptr<ix::ConnectionState> ConnectionState,
	ix::WebSocket& WebSocket,
	const ix::WebSocketMessagePtr& Msg)
{
	// Get remote IP
	FString RemoteIp = UTF8_TO_TCHAR(ConnectionState->getRemoteIp().c_str());

	// Get or create connection wrapper
	TSharedPtr<FTripoWebSocketConnectionImpl> Connection = GetOrCreateConnection(&WebSocket, RemoteIp);
	FString ClientId;

	{
		FScopeLock Lock(&ConnectionsLock);
		if (FString* FoundId = ClientIds.Find(&WebSocket))
		{
			ClientId = *FoundId;
		}
	}

	// Create message struct
	FTripoWebSocketMessage Message;

	if (Msg->type == ix::WebSocketMessageType::Open)
	{
		Message.Type = ETripoWebSocketMessageType::Open;
		UE_LOG(LogTemp, Log, TEXT("[Tripo3D WebSocket] Client connected: %s from %s"), *ClientId, *RemoteIp);
	}
	else if (Msg->type == ix::WebSocketMessageType::Close)
	{
		Message.Type = ETripoWebSocketMessageType::Close;
		Message.CloseInfo.Code = Msg->closeInfo.code;
		Message.CloseInfo.Reason = UTF8_TO_TCHAR(Msg->closeInfo.reason.c_str());

		UE_LOG(LogTemp, Log, TEXT("[Tripo3D WebSocket] Client disconnected: %s (Code: %d, Reason: %s)"),
			*ClientId, Message.CloseInfo.Code, *Message.CloseInfo.Reason);

		// Invalidate and remove connection
		if (Connection.IsValid())
		{
			Connection->Invalidate();
		}
		RemoveConnection(&WebSocket);
	}
	else if (Msg->type == ix::WebSocketMessageType::Message)
	{
		Message.Type = ETripoWebSocketMessageType::Message;
		Message.bIsBinary = Msg->binary;
		Message.Data.Append((uint8*)Msg->str.data(), Msg->str.size());
	}
	else if (Msg->type == ix::WebSocketMessageType::Error)
	{
		Message.Type = ETripoWebSocketMessageType::Error;
		Message.ErrorMessage = UTF8_TO_TCHAR(Msg->errorInfo.reason.c_str());

		UE_LOG(LogTemp, Error, TEXT("[Tripo3D WebSocket] Error for client %s: %s"),
			*ClientId, *Message.ErrorMessage);
	}
	else
	{
		// Ignore other message types (Ping, Pong, Fragment)
		return;
	}

	// Invoke callback if set
	if (OnMessageCallback.IsBound())
	{
		OnMessageCallback.Execute(ClientId, Connection, Message);
	}
}

TSharedPtr<FTripoWebSocketConnectionImpl> FTripoWebSocketServerImpl::GetOrCreateConnection(
	ix::WebSocket* WebSocket, const FString& RemoteIp)
{
	FScopeLock Lock(&ConnectionsLock);

	if (TSharedPtr<FTripoWebSocketConnectionImpl>* Found = Connections.Find(WebSocket))
	{
		return *Found;
	}

	// Create new connection wrapper
	TSharedPtr<FTripoWebSocketConnectionImpl> NewConnection = MakeShared<FTripoWebSocketConnectionImpl>(WebSocket, RemoteIp);
	Connections.Add(WebSocket, NewConnection);

	// Generate client ID
	FString ClientId = GenerateClientId(RemoteIp);
	ClientIds.Add(WebSocket, ClientId);

	return NewConnection;
}

void FTripoWebSocketServerImpl::RemoveConnection(ix::WebSocket* WebSocket)
{
	FScopeLock Lock(&ConnectionsLock);
	Connections.Remove(WebSocket);
	ClientIds.Remove(WebSocket);
}

FString FTripoWebSocketServerImpl::GenerateClientId(const FString& RemoteIp)
{
	static int32 ClientCounter = 0;
	return FString::Printf(TEXT("%s_%d"), *RemoteIp, ++ClientCounter);
}

// ————— FTripoWebSocketFactory —————

TSharedPtr<ITripoWebSocketServer> FTripoWebSocketFactory::CreateServer()
{
	return MakeShared<FTripoWebSocketServerImpl>();
}

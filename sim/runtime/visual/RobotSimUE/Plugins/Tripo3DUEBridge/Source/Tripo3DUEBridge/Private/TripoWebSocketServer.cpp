#include "TripoWebSocketServer.h"
#include "TripoFileTransferManager.h"
#include "Json.h"
#include "JsonUtilities.h"
#include "Misc/EngineVersion.h"
#include "Async/Async.h"

#ifdef SendMessage
#undef SendMessage
#endif

// ————— Lifecycle —————

FTripoWebSocketServer& FTripoWebSocketServer::Get()
{
	static FTripoWebSocketServer Instance;
	return Instance;
}

FTripoWebSocketServer::FTripoWebSocketServer()
	: bIsServerRunning(false)
	, bIsClientConnected(false)
{
	FileTransferManager = MakeShared<FTripoFileTransferManager>();
}

FTripoWebSocketServer::~FTripoWebSocketServer()
{
	StopServer();
}

// ————— Public API —————

bool FTripoWebSocketServer::StartServer()
{
	if (bIsServerRunning)
	{
		UE_LOG(LogTemp, Warning, TEXT("Tripo3D WebSocket server is already running"));
		return false;
	}

	UE_LOG(LogTemp, Log, TEXT("Starting Tripo3D WebSocket server on %s:%d"),
		*TripoProtocol::SERVER_HOST, TripoProtocol::SERVER_PORT);

	// Use factory to create WebSocket server instance
	WebSocketServer = FTripoWebSocketFactory::CreateServer();
	if (!WebSocketServer.IsValid())
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to create WebSocket server instance"));
		return false;
	}

	// Set message callback
	WebSocketServer->SetOnMessage(FOnTripoWebSocketMessage::CreateRaw(
		this, &FTripoWebSocketServer::OnWebSocketMessage));

	// Start server
	if (!WebSocketServer->Start(TripoProtocol::SERVER_PORT, TripoProtocol::SERVER_HOST))
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to start WebSocket server"));
		WebSocketServer.Reset();
		return false;
	}

	bIsServerRunning = true;

	// Broadcast waiting for connection status
	OnConnectionStatusChanged.Broadcast(false, TEXT("Waiting for connection to Tripo Studio"));

	UE_LOG(LogTemp, Log, TEXT("Tripo3D WebSocket server started successfully"));
	return true;
}

void FTripoWebSocketServer::StopServer()
{
	if (!bIsServerRunning)
	{
		return;
	}

	UE_LOG(LogTemp, Log, TEXT("Stopping Tripo3D WebSocket server"));

	// Stop and release WebSocket server
	if (WebSocketServer.IsValid())
	{
		WebSocketServer->Stop();
		WebSocketServer.Reset();
	}

	// Clear client connection
	ClientConnection.Reset();
	CurrentClientId.Empty();

	bIsServerRunning = false;
	bIsClientConnected = false;
	ConnectedClientInfo.Empty();

	// Clear file transfer sessions
	if (FileTransferManager.IsValid())
	{
		FileTransferManager->ClearAllSessions();
	}

	OnConnectionStatusChanged.Broadcast(false, TEXT(""));

	UE_LOG(LogTemp, Log, TEXT("Tripo3D WebSocket server stopped"));
}

// ————— Internal —————

void FTripoWebSocketServer::OnWebSocketMessage(
	const FString& ClientId,
	TSharedPtr<ITripoWebSocketConnection> Connection,
	const FTripoWebSocketMessage& Message)
{
	switch (Message.Type)
	{
	case ETripoWebSocketMessageType::Open:
		// Client connected
		ClientConnection = Connection;
		CurrentClientId = ClientId;
		bIsClientConnected = true;

		UE_LOG(LogTemp, Log, TEXT("[Tripo3D Server] Client connected: %s from %s"),
			*ClientId, *Connection->GetRemoteIp());

		AsyncTask(ENamedThreads::GameThread, [this]()
		{
			OnConnectionStatusChanged.Broadcast(true, TEXT("Client Connected"));
		});
		break;

	case ETripoWebSocketMessageType::Close:
		{
			FString OldClientInfo = ConnectedClientInfo;

			// Clear connection
			if (CurrentClientId == ClientId)
			{
				ClientConnection.Reset();
				CurrentClientId.Empty();
				bIsClientConnected = false;
				ConnectedClientInfo.Empty();
			}

			UE_LOG(LogTemp, Log, TEXT("[Tripo3D Server] Client disconnected: %s (Code: %d, Reason: %s)"),
				*OldClientInfo, Message.CloseInfo.Code, *Message.CloseInfo.Reason);

			AsyncTask(ENamedThreads::GameThread, [this]()
			{
				OnConnectionStatusChanged.Broadcast(false, TEXT(""));
			});
		}
		break;

	case ETripoWebSocketMessageType::Message:
		// Process message directly on WebSocket thread (bypasses game thread throttling)
		ProcessMessageOnWSThread(Message.Data);
		break;

	case ETripoWebSocketMessageType::Error:
		UE_LOG(LogTemp, Error, TEXT("[Tripo3D Server] WebSocket error: %s"), *Message.ErrorMessage);
		break;
	}
}

void FTripoWebSocketServer::ProcessMessageOnWSThread(const TArray<uint8>& MessageData)
{
	ProcessMessage(MessageData);
}

void FTripoWebSocketServer::ProcessMessage(const TArray<uint8>& MessageData)
{
	// Find JSON end using brace counting algorithm
	int32 JsonEnd = FindJsonEnd(MessageData);
	if (JsonEnd <= 0)
	{
		UE_LOG(LogTemp, Error, TEXT("[Tripo3D Server] Failed to parse JSON header from message (%d bytes)"),
			MessageData.Num());
		return;
	}

	UE_LOG(LogTemp, Verbose, TEXT("[Tripo3D Server] JSON header found, length: %d bytes"), JsonEnd);

	// Extract JSON string
	FString JsonString;
	FFileHelper::BufferToString(JsonString, MessageData.GetData(), JsonEnd);

	// Parse JSON
	TSharedPtr<FJsonObject> JsonObject;
	TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(JsonString);
	if (!FJsonSerializer::Deserialize(Reader, JsonObject) || !JsonObject.IsValid())
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to deserialize JSON: %s"), *JsonString);
		return;
	}

	// Get message type
	FString MessageTypeStr;
	if (!JsonObject->TryGetStringField(TEXT("type"), MessageTypeStr))
	{
		UE_LOG(LogTemp, Error, TEXT("[Tripo3D Server] Message missing 'type' field"));
		return;
	}


#if UE_BUILD_DEBUG
	UE_LOG(LogTemp, Log, TEXT("[Tripo3D Server] Received %s message: %d bytes, type: %s"),
		MessageData.Num() > 0 ? TEXT("binary") : TEXT("text"), MessageData.Num(), *MessageTypeStr);
#endif

	// Route message based on type
	switch (TripoProtocol::ParseMessageType(MessageTypeStr))
	{
	case TripoProtocol::EMessageType::Handshake:
		HandleHandshake(JsonObject);
		break;

	case TripoProtocol::EMessageType::Ping:
		HandlePing();
		break;

	case TripoProtocol::EMessageType::FileTransfer:
		{
			TArray<uint8> ChunkData;
			if (JsonEnd < MessageData.Num())
			{
				ChunkData.Append(MessageData.GetData() + JsonEnd, MessageData.Num() - JsonEnd);
			}
			HandleFileTransfer(JsonObject, ChunkData);
		}
		break;

	default:
		UE_LOG(LogTemp, Warning, TEXT("Unknown message type: %s"), *MessageTypeStr);
		break;
	}
}

int32 FTripoWebSocketServer::FindJsonEnd(const TArray<uint8>& Data)
{
	int32 BraceCount = 0;
	bool bInString = false;
	bool bEscape = false;

	for (int32 i = 0; i < Data.Num(); ++i)
	{
		// Skip non-ASCII characters
		if (Data[i] >= 128)
		{
			continue;
		}

		char c = static_cast<char>(Data[i]);

		if (bEscape)
		{
			bEscape = false;
			continue;
		}

		if (c == '\\')
		{
			bEscape = true;
			continue;
		}

		if (c == '"')
		{
			bInString = !bInString;
			continue;
		}

		if (!bInString)
		{
			if (c == '{')
			{
				BraceCount++;
			}
			else if (c == '}')
			{
				BraceCount--;
				if (BraceCount == 0)
				{
					return i + 1; // Return position after closing brace
				}
			}
		}
	}

	return -1; // JSON not complete
}

void FTripoWebSocketServer::HandleHandshake(const TSharedPtr<FJsonObject>& JsonObject)
{
	const TSharedPtr<FJsonObject>* PayloadObject;
	if (!JsonObject->TryGetObjectField(TEXT("payload"), PayloadObject))
	{
		UE_LOG(LogTemp, Error, TEXT("Handshake message missing payload"));
		SendHandshakeAck(false);
		return;
	}

	FString ClientName;
	FString ProtocolVersion;
	(*PayloadObject)->TryGetStringField(TEXT("clientName"), ClientName);
	(*PayloadObject)->TryGetStringField(TEXT("protocolVersion"), ProtocolVersion);

	// Validate protocol version
	if (ProtocolVersion != TripoProtocol::PROTOCOL_VERSION)
	{
		UE_LOG(LogTemp, Warning, TEXT("Protocol version mismatch. Client: %s, Server: %s"),
			*ProtocolVersion, *TripoProtocol::PROTOCOL_VERSION);
	}

	bIsClientConnected = true;
	ConnectedClientInfo = ClientName;

	UE_LOG(LogTemp, Log, TEXT("Client connected: %s (Protocol: %s)"), *ConnectedClientInfo, *ProtocolVersion);

	SendHandshakeAck(true);

	// Broadcast on game thread (Slate UI requires game thread)
	FString ClientInfoCopy = ConnectedClientInfo;
	AsyncTask(ENamedThreads::GameThread, [this, ClientInfoCopy]()
	{
		OnConnectionStatusChanged.Broadcast(true, ClientInfoCopy);
	});
}

void FTripoWebSocketServer::HandlePing()
{
	// UE_LOG(LogTemp, Log, TEXT("[Tripo3D Server] Received ping, sending pong..."));

	// Send pong response
	TSharedPtr<FJsonObject> JsonObject = MakeShared<FJsonObject>();
	JsonObject->SetStringField(TEXT("type"), TripoProtocol::MessageType::MSG_PONG);

	FString JsonString;
	TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&JsonString);
	FJsonSerializer::Serialize(JsonObject.ToSharedRef(), Writer);

	SendMessage(JsonString);

	// UE_LOG(LogTemp, Log, TEXT("[Tripo3D Server] Pong sent successfully"));
}

void FTripoWebSocketServer::HandleFileTransfer(const TSharedPtr<FJsonObject>& JsonObject, const TArray<uint8>& ChunkData)
{
	const TSharedPtr<FJsonObject>* PayloadObject;
	if (!JsonObject->TryGetObjectField(TEXT("payload"), PayloadObject))
	{
		UE_LOG(LogTemp, Error, TEXT("File transfer message missing payload"));
		return;
	}

	FString FileId, FileName, FileType;
	int32 ChunkIndex = 0, ChunkTotal = 0, ChunkSize = 0;

	(*PayloadObject)->TryGetStringField(TEXT("fileId"), FileId);
	(*PayloadObject)->TryGetStringField(TEXT("fileName"), FileName);
	(*PayloadObject)->TryGetStringField(TEXT("fileType"), FileType);
	(*PayloadObject)->TryGetNumberField(TEXT("chunkIndex"), ChunkIndex);
	(*PayloadObject)->TryGetNumberField(TEXT("chunkTotal"), ChunkTotal);
	(*PayloadObject)->TryGetNumberField(TEXT("chunkSize"), ChunkSize);

	UE_LOG(LogTemp, Log, TEXT("Receiving chunk %d/%d for file %s (%.2f MB)"),
		ChunkIndex + 1, ChunkTotal, *FileName, ChunkData.Num() / 1024.0f / 1024.0f);

	// Forward to file transfer manager
	if (FileTransferManager.IsValid())
	{
		bool bSuccess = FileTransferManager->ProcessChunk(
			FileId, FileName, FileType, ChunkIndex, ChunkTotal, ChunkData
		);

		SendFileTransferAck(FileId, ChunkIndex, bSuccess,
			bSuccess ? TripoProtocol::ErrorCode::SUCCESS : TripoProtocol::ErrorCode::ERROR_PROCESSING);
	}
}

void FTripoWebSocketServer::SendHandshakeAck(bool bSuccess)
{
	TSharedPtr<FJsonObject> JsonObject = MakeShared<FJsonObject>();
	JsonObject->SetStringField(TEXT("type"), TripoProtocol::MessageType::MSG_HANDSHAKE_ACK);

	TSharedPtr<FJsonObject> Payload = MakeShared<FJsonObject>();
	Payload->SetBoolField(TEXT("success"), bSuccess);
	Payload->SetStringField(TEXT("clientName"), TEXT("Unreal Engine"));
	Payload->SetStringField(TEXT("dccVersion"), FEngineVersion::Current().ToString());
	Payload->SetStringField(TEXT("pluginVersion"), TripoProtocol::PROTOCOL_VERSION);
	Payload->SetStringField(TEXT("protocolVersion"), TripoProtocol::PROTOCOL_VERSION);

	JsonObject->SetObjectField(TEXT("payload"), Payload);

	FString JsonString;
	TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&JsonString);
	FJsonSerializer::Serialize(JsonObject.ToSharedRef(), Writer);

	SendMessage(JsonString);
}

void FTripoWebSocketServer::SendFileTransferAck(const FString& FileId, int32 FileIndex, bool bSuccess, int32 ErrorCode)
{
	TSharedPtr<FJsonObject> JsonObject = MakeShared<FJsonObject>();
	JsonObject->SetStringField(TEXT("type"), TripoProtocol::MessageType::MSG_FILE_TRANSFER_ACK);

	TSharedPtr<FJsonObject> Payload = MakeShared<FJsonObject>();
	Payload->SetBoolField(TEXT("success"), bSuccess);
	Payload->SetStringField(TEXT("fileId"), FileId);
	Payload->SetNumberField(TEXT("fileIndex"), FileIndex);
	Payload->SetNumberField(TEXT("code"), ErrorCode);

	JsonObject->SetObjectField(TEXT("payload"), Payload);

	FString JsonString;
	TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&JsonString);
	FJsonSerializer::Serialize(JsonObject.ToSharedRef(), Writer);

	SendMessage(JsonString);
}

void FTripoWebSocketServer::SendFileTransferComplete(const FString& FileId, const FString& Status, const FString& Message)
{
	TSharedPtr<FJsonObject> JsonObject = MakeShared<FJsonObject>();
	JsonObject->SetStringField(TEXT("type"), TripoProtocol::MessageType::MSG_FILE_TRANSFER_COMPLETE);

	TSharedPtr<FJsonObject> Payload = MakeShared<FJsonObject>();
	Payload->SetStringField(TEXT("fileId"), FileId);
	Payload->SetStringField(TEXT("status"), Status);
	Payload->SetStringField(TEXT("message"), Message);

	JsonObject->SetObjectField(TEXT("payload"), Payload);

	FString JsonString;
	TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&JsonString);
	FJsonSerializer::Serialize(JsonObject.ToSharedRef(), Writer);

	SendMessage(JsonString);
}

void FTripoWebSocketServer::SendImportComplete(const FString& FileId, bool bSuccess, const FString& Message)
{
	TSharedPtr<FJsonObject> JsonObject = MakeShared<FJsonObject>();
	JsonObject->SetStringField(TEXT("type"), TripoProtocol::MessageType::MSG_IMPORT_COMPLETE);

	TSharedPtr<FJsonObject> Payload = MakeShared<FJsonObject>();
	Payload->SetStringField(TEXT("fileId"), FileId);
	Payload->SetBoolField(TEXT("success"), bSuccess);
	Payload->SetStringField(TEXT("message"), Message);

	JsonObject->SetObjectField(TEXT("payload"), Payload);

	FString JsonString;
	TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&JsonString);
	FJsonSerializer::Serialize(JsonObject.ToSharedRef(), Writer);

	SendMessage(JsonString);
}

void FTripoWebSocketServer::SendMessage(const FString& JsonMessage)
{
	if (!bIsServerRunning || !bIsClientConnected || !ClientConnection.IsValid())
	{
		UE_LOG(LogTemp, Warning, TEXT("[Tripo3D Server] Cannot send message - not connected"));
		return;
	}

	ClientConnection->SendText(JsonMessage);
	UE_LOG(LogTemp, Verbose, TEXT("[Tripo3D Server] Sent message: %s"), *JsonMessage);
}

void FTripoWebSocketServer::SendBinaryMessage(const FString& JsonMessage, const TArray<uint8>& BinaryData)
{
	if (!bIsServerRunning || !bIsClientConnected || !ClientConnection.IsValid())
	{
		return;
	}

	ClientConnection->SendBinary(JsonMessage, BinaryData);
	UE_LOG(LogTemp, Verbose, TEXT("Sent binary message: %s + %d bytes"),
		*JsonMessage, BinaryData.Num());
}

#include "TripoFileTransferManager.h"
#include "TripoWebSocketServer.h"
#include "TripoModelImporter.h"
#include "Async/Async.h"
#include "Editor.h"

// ————— Lifecycle —————

FTripoFileTransferManager::FTripoFileTransferManager()
{
}

FTripoFileTransferManager::~FTripoFileTransferManager()
{
	ClearAllSessions();
}

// ————— Public API —————

bool FTripoFileTransferManager::ProcessChunk(
	const FString& FileId,
	const FString& FileName,
	const FString& FileType,
	int32 ChunkIndex,
	int32 ChunkTotal,
	const TArray<uint8>& ChunkData)
{
	bool bIsComplete = false;
	FFileTransferSession CompletedSession;

	// Narrow lock scope: only protect ActiveSessions read/write
	{
		FScopeLock Lock(&TransferLock);

		// Get or create session
		FFileTransferSession* Session = ActiveSessions.Find(FileId);
		if (!Session)
		{
			FFileTransferSession NewSession;
			NewSession.FileId = FileId;
			NewSession.FileName = FileName;
			NewSession.FileType = FileType;
			NewSession.TotalChunks = ChunkTotal;

			Session = &ActiveSessions.Add(FileId, NewSession);

			UE_LOG(LogTemp, Log, TEXT("Started new file transfer session: %s (%d chunks)"),
				*FileName, ChunkTotal);
		}

		// Validate chunk index
		if (ChunkIndex < 0 || ChunkIndex >= ChunkTotal)
		{
			UE_LOG(LogTemp, Error, TEXT("Invalid chunk index %d for file %s (total: %d)"),
				ChunkIndex, *FileId, ChunkTotal);
			return false;
		}

		// Check if chunk already received
		if (Session->HasChunk(ChunkIndex))
		{
			UE_LOG(LogTemp, Warning, TEXT("Chunk %d already received for file %s"),
				ChunkIndex, *FileId);
			return true;
		}

		// Store chunk data
		Session->Chunks.Add(ChunkIndex, ChunkData);

		float Progress = (Session->Chunks.Num() * 100.0f) / ChunkTotal;
		UE_LOG(LogTemp, Log, TEXT("[Tripo3D Server] Received chunk %d/%d for %s (%.2f MB) - Progress: %.1f%%"),
			ChunkIndex + 1, ChunkTotal, *FileName, ChunkData.Num() / 1024.0f / 1024.0f, Progress);

		// If complete, move session data out of the map to process outside the lock
		if (Session->IsComplete())
		{
			bIsComplete = true;
			CompletedSession = MoveTemp(*Session);
			ActiveSessions.Remove(FileId);
		}
	}

	// Process completed file outside the lock so other chunks aren't blocked
	if (bIsComplete)
	{
		UE_LOG(LogTemp, Log, TEXT("[Tripo3D Server] ✓ All %d chunks received for %s, assembling file..."), ChunkTotal, *FileName);

		FTripoWebSocketServer::Get().SendFileTransferComplete(
			FileId,
			TEXT("importing"),
			FString::Printf(TEXT("File transfer complete, importing %s..."), *FileName)
		);

		TArray<uint8> FileData;
		if (AssembleFile(CompletedSession, FileData))
		{
			TSharedPtr<TArray<uint8>, ESPMode::ThreadSafe> FileDataPtr = MakeShared<TArray<uint8>, ESPMode::ThreadSafe>(MoveTemp(FileData));

			TriggerImport(FileId, FileName, FileType, FileDataPtr);
		}
		else
		{
			UE_LOG(LogTemp, Error, TEXT("Failed to assemble file %s"), *FileId);

			FTripoWebSocketServer::Get().SendImportComplete(
				FileId,
				false,
				TEXT("Failed to assemble file chunks")
			);

			return false;
		}
	}

	return true;
}

// ————— Internal —————

bool FTripoFileTransferManager::AssembleFile(FFileTransferSession& Session, TArray<uint8>& OutFileData)
{
	// Calculate total size
	int32 TotalSize = 0;
	for (const auto& ChunkPair : Session.Chunks)
	{
		TotalSize += ChunkPair.Value.Num();
	}

	UE_LOG(LogTemp, Log, TEXT("Assembling file %s: %d chunks, %.2f MB total"),
		*Session.FileName, Session.TotalChunks, TotalSize / 1024.0f / 1024.0f);

	// Reserve space
	OutFileData.Reset(TotalSize);
	OutFileData.AddUninitialized(TotalSize);

	// Copy chunks in order, releasing each chunk immediately to avoid 2x memory peak
	int32 Offset = 0;
	for (int32 i = 0; i < Session.TotalChunks; ++i)
	{
		TArray<uint8> ChunkData;
		if (!Session.Chunks.RemoveAndCopyValue(i, ChunkData))
		{
			UE_LOG(LogTemp, Error, TEXT("Missing chunk %d/%d for file %s"),
				i, Session.TotalChunks, *Session.FileName);
			return false;
		}

		FMemory::Memcpy(OutFileData.GetData() + Offset, ChunkData.GetData(), ChunkData.Num());
		Offset += ChunkData.Num();
	}

	UE_LOG(LogTemp, Log, TEXT("Successfully assembled file %s"), *Session.FileName);
	return true;
}

void FTripoFileTransferManager::TriggerImport(
	const FString& FileId,
	const FString& FileName,
	const FString& FileType,
	const TSharedPtr<TArray<uint8>, ESPMode::ThreadSafe>& FileData)
{
	if (!FileData.IsValid())
	{
		UE_LOG(LogTemp, Error, TEXT("Import aborted for %s: file data is no longer available"), *FileName);
		return;
	}

	// Notify UI: starting extraction
	AsyncTask(ENamedThreads::GameThread, [FileName]()
	{
		FTripoWebSocketServer::Get().OnMessageReceived.Broadcast(
			FString::Printf(TEXT("Extracting %s..."), *FileName)
		);
	});

	// Phase 1: Extract/prepare file on background thread (avoids blocking UI)
	AsyncTask(ENamedThreads::AnyBackgroundThreadNormalTask, [FileId, FileName, FileType, FileData]()
	{
		if (!FileData.IsValid())
		{
			UE_LOG(LogTemp, Error, TEXT("Import aborted for %s: file data is no longer available"), *FileName);
			return;
		}

		FTripoImportContext Context;
		bool bPrepareSuccess = FTripoModelImporter::PrepareModelFile(
			FileId, FileName, FileType, *FileData, Context);

		// Phase 2: Import to engine on GameThread
		AsyncTask(ENamedThreads::GameThread, [FileId, FileName, Context, bPrepareSuccess]()
		{
			if (!bPrepareSuccess)
			{
				FString ErrorMsg = Context.ErrorMessage.IsEmpty()
					? FString::Printf(TEXT("Failed to prepare file: %s"), *FileName)
					: Context.ErrorMessage;

				UE_LOG(LogTemp, Error, TEXT("%s"), *ErrorMsg);
				FTripoWebSocketServer::Get().OnMessageReceived.Broadcast(ErrorMsg);
				FTripoWebSocketServer::Get().SendImportComplete(FileId, false, ErrorMsg);

				if (!Context.TempDir.IsEmpty())
				{
					FTripoModelImporter::DeleteDirectoryWithRetry(Context.TempDir);
				}
				return;
			}

			FTripoWebSocketServer::Get().OnMessageReceived.Broadcast(
				FString::Printf(TEXT("Extraction complete, importing %s..."), *FileName)
			);

			GEditor->GetTimerManager()->SetTimerForNextTick([FileId, FileName, Context]()
			{
				UE_LOG(LogTemp, Log, TEXT("Starting engine import for %s on game thread"), *FileName);

				bool bSuccess = FTripoModelImporter::ImportModelToEngine(Context);

				FString ResultMsg = bSuccess
					? FString::Printf(TEXT("Import complete: %s"), *FileName)
					: FString::Printf(TEXT("Import failed: %s"), *FileName);

				FTripoWebSocketServer::Get().OnMessageReceived.Broadcast(ResultMsg);
				FTripoWebSocketServer::Get().SendImportComplete(FileId, bSuccess, ResultMsg);
			});
		});
	});
}

void FTripoFileTransferManager::ClearAllSessions()
{
	FScopeLock Lock(&TransferLock);
	ActiveSessions.Empty();
	UE_LOG(LogTemp, Log, TEXT("Cleared all file transfer sessions"));
}

void FTripoFileTransferManager::ClearSession(const FString& FileId)
{
	FScopeLock Lock(&TransferLock);

	if (ActiveSessions.Remove(FileId) > 0)
	{
		UE_LOG(LogTemp, Log, TEXT("Cleared file transfer session: %s"), *FileId);
	}
}

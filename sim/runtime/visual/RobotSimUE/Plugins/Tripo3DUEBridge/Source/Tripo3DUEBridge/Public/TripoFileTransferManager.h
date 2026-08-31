// Copyright Tripo3D. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "TripoProtocol.h"

/**
 * File transfer session data
 */
struct FFileTransferSession
{
	FString FileId;
	FString FileName;
	FString FileType;
	int32 TotalChunks;
	TMap<int32, TArray<uint8>> Chunks;

	FFileTransferSession()
		: TotalChunks(0)
	{
	}

	bool IsComplete() const
	{
		return Chunks.Num() == TotalChunks && TotalChunks > 0;
	}

	bool HasChunk(int32 ChunkIndex) const
	{
		return Chunks.Contains(ChunkIndex);
	}
};

/**
 * Manages file transfer sessions and chunk reassembly
 */
class TRIPO3DUEBRIDGE_API FTripoFileTransferManager
{
public:
	FTripoFileTransferManager();
	~FTripoFileTransferManager();

	/**
	 * Process an incoming file chunk
	 * @param FileId Unique identifier for the file
	 * @param FileName Name of the file
	 * @param FileType Type of file (zip, fbx, obj)
	 * @param ChunkIndex Index of this chunk
	 * @param ChunkTotal Total number of chunks
	 * @param ChunkData Binary data for this chunk
	 * @return True if chunk was processed successfully
	 */
	bool ProcessChunk(
		const FString& FileId,
		const FString& FileName,
		const FString& FileType,
		int32 ChunkIndex,
		int32 ChunkTotal,
		const TArray<uint8>& ChunkData
	);

	/**
	 * Clear all active transfer sessions
	 */
	void ClearAllSessions();

	/**
	 * Clear a specific transfer session
	 */
	void ClearSession(const FString& FileId);

private:
	/**
	 * Assemble all chunks into a complete file
	 */
	bool AssembleFile(FFileTransferSession& Session, TArray<uint8>& OutFileData);

	/**
	 * Trigger model import on game thread
	 */
	void TriggerImport(const FString& FileId, const FString& FileName, const FString& FileType, const TSharedPtr<TArray<uint8>, ESPMode::ThreadSafe>& FileData);

private:
	/** Active transfer sessions mapped by FileId */
	TMap<FString, FFileTransferSession> ActiveSessions;

	/** Critical section for thread safety */
	FCriticalSection TransferLock;
};

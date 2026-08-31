// Copyright Tripo3D. All Rights Reserved.
// ZIP extraction utilities for TripoModelImporter

#include "TripoModelImporter.h"
#include "TripoProtocol.h"
#include "HAL/PlatformFileManager.h"
#include "GenericPlatform/GenericPlatformFile.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"

// Include minizip for ZIP extraction
THIRD_PARTY_INCLUDES_START
#include "zlib.h"
#include "minizip/unzip.h"
THIRD_PARTY_INCLUDES_END

// ————— Minizip Memory Stream —————

struct FMinizipMemoryStream
{
	const uint8* Data;
	int64 Size;
	int64 Position;
};

static voidpf ZCALLBACK MemOpen(voidpf Opaque, const char* /*Filename*/, int /*Mode*/)
{
	return Opaque;
}

static uLong ZCALLBACK MemRead(voidpf Opaque, voidpf /*Stream*/, void* Buf, uLong Size)
{
	FMinizipMemoryStream* Mem = static_cast<FMinizipMemoryStream*>(Opaque);
	int64 Remaining = Mem->Size - Mem->Position;
	uLong ToRead = static_cast<uLong>(FMath::Min(static_cast<int64>(Size), Remaining));
	if (ToRead > 0)
	{
		FMemory::Memcpy(Buf, Mem->Data + Mem->Position, ToRead);
		Mem->Position += ToRead;
	}
	return ToRead;
}

static uLong ZCALLBACK MemWrite(voidpf /*Opaque*/, voidpf /*Stream*/, const void* /*Buf*/, uLong /*Size*/)
{
	return 0;
}

static long ZCALLBACK MemTell(voidpf Opaque, voidpf /*Stream*/)
{
	FMinizipMemoryStream* Mem = static_cast<FMinizipMemoryStream*>(Opaque);
	return static_cast<long>(Mem->Position);
}

static long ZCALLBACK MemSeek(voidpf Opaque, voidpf /*Stream*/, uLong Offset, int Origin)
{
	FMinizipMemoryStream* Mem = static_cast<FMinizipMemoryStream*>(Opaque);
	int64 NewPos = 0;
	switch (Origin)
	{
	case ZLIB_FILEFUNC_SEEK_SET:
		NewPos = static_cast<int64>(Offset);
		break;
	case ZLIB_FILEFUNC_SEEK_CUR:
		NewPos = Mem->Position + static_cast<int64>(Offset);
		break;
	case ZLIB_FILEFUNC_SEEK_END:
		NewPos = Mem->Size + static_cast<int64>(Offset);
		break;
	default:
		return -1;
	}
	if (NewPos < 0 || NewPos > Mem->Size)
	{
		return -1;
	}
	Mem->Position = NewPos;
	return 0;
}

static int ZCALLBACK MemClose(voidpf /*Opaque*/, voidpf /*Stream*/)
{
	return 0;
}

static int ZCALLBACK MemError(voidpf /*Opaque*/, voidpf /*Stream*/)
{
	return 0;
}

// ————— FTripoModelImporter ZIP Extraction Implementation —————

bool FTripoModelImporter::ExtractZip(const TArray<uint8>& ZipData, const FString& ExtractPath)
{
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	FString NormalizedExtractRoot = FPaths::ConvertRelativePathToFull(ExtractPath);
	constexpr int64 MaxZipEntrySizeBytes = 500ll * 1024ll * 1024ll;
	FPaths::NormalizeFilename(NormalizedExtractRoot);
	FPaths::CollapseRelativeDirectories(NormalizedExtractRoot);
	if (!NormalizedExtractRoot.EndsWith(TEXT("/")))
	{
		NormalizedExtractRoot += TEXT("/");
	}

	// Create extract directory
	if (!PlatformFile.CreateDirectoryTree(*ExtractPath))
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to create extraction directory: %s"), *ExtractPath);
		return false;
	}

	// Setup memory stream for minizip (no temporary file needed)
	FMinizipMemoryStream MemStream;
	MemStream.Data = ZipData.GetData();
	MemStream.Size = ZipData.Num();
	MemStream.Position = 0;

	zlib_filefunc_def FileFuncs;
	FileFuncs.zopen_file = MemOpen;
	FileFuncs.zread_file = MemRead;
	FileFuncs.zwrite_file = MemWrite;
	FileFuncs.ztell_file = MemTell;
	FileFuncs.zseek_file = MemSeek;
	FileFuncs.zclose_file = MemClose;
	FileFuncs.zerror_file = MemError;
	FileFuncs.opaque = &MemStream;

	// Open ZIP from memory
	unzFile ZipFile = unzOpen2(nullptr, &FileFuncs);
	if (!ZipFile)
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to open ZIP from memory"));
		return false;
	}

	// Get global info
	unz_global_info GlobalInfo;
	if (unzGetGlobalInfo(ZipFile, &GlobalInfo) != UNZ_OK)
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to get ZIP global info"));
		unzClose(ZipFile);
		return false;
	}

	// Extract all files
	for (uLong i = 0; i < GlobalInfo.number_entry; ++i)
	{
		char Filename[512];
		unz_file_info FileInfo;

		if (unzGetCurrentFileInfo(ZipFile, &FileInfo, Filename, sizeof(Filename), nullptr, 0, nullptr, 0) != UNZ_OK)
		{
			UE_LOG(LogTemp, Error, TEXT("Failed to get file info for entry %d"), i);
			continue;
		}

		FString EntryName = UTF8_TO_TCHAR(Filename);
		const bool bIsDirectory = EntryName.EndsWith(TEXT("/")) || EntryName.EndsWith(TEXT("\\"));
		FPaths::NormalizeFilename(EntryName);

		if (EntryName.IsEmpty() || EntryName.StartsWith(TEXT("/")) || EntryName.Contains(TEXT(":")))
		{
			UE_LOG(LogTemp, Error, TEXT("Skipping suspicious ZIP entry: %s"), *EntryName);
			if ((i + 1) < GlobalInfo.number_entry)
			{
				unzGoToNextFile(ZipFile);
			}
			continue;
		}

		FString FullPath = FPaths::ConvertRelativePathToFull(FPaths::Combine(NormalizedExtractRoot, EntryName));
		FPaths::NormalizeFilename(FullPath);
		FPaths::CollapseRelativeDirectories(FullPath);

		if (!FullPath.StartsWith(NormalizedExtractRoot, ESearchCase::IgnoreCase))
		{
			UE_LOG(LogTemp, Error, TEXT("Skipping suspicious ZIP entry: %s"), *EntryName);
			if ((i + 1) < GlobalInfo.number_entry)
			{
				unzGoToNextFile(ZipFile);
			}
			continue;
		}

		// Check if it's a directory
		if (bIsDirectory)
		{
			PlatformFile.CreateDirectoryTree(*FullPath);
		}
		else
		{
			// Create directory for file
			FString DirectoryPath = FPaths::GetPath(FullPath);
			PlatformFile.CreateDirectoryTree(*DirectoryPath);

			// Open file in ZIP
			if (unzOpenCurrentFile(ZipFile) != UNZ_OK)
			{
				UE_LOG(LogTemp, Error, TEXT("Failed to open file in ZIP: %s"), *EntryName);
				unzGoToNextFile(ZipFile);
				continue;
			}

			// Read and extract file
			if (static_cast<int64>(FileInfo.uncompressed_size) > MaxZipEntrySizeBytes)
			{
				UE_LOG(LogTemp, Error, TEXT("Skipping oversized ZIP entry (%lld bytes): %s"),
					static_cast<long long>(FileInfo.uncompressed_size), *EntryName);
				unzCloseCurrentFile(ZipFile);
				unzGoToNextFile(ZipFile);
				continue;
			}

			TArray<uint8> FileData;
			FileData.SetNumUninitialized(static_cast<int32>(FileInfo.uncompressed_size));

			int BytesRead = unzReadCurrentFile(ZipFile, FileData.GetData(), FileData.Num());
			unzCloseCurrentFile(ZipFile);

			if (BytesRead < 0)
			{
				UE_LOG(LogTemp, Error, TEXT("Failed to read file from ZIP: %s"), *EntryName);
				unzGoToNextFile(ZipFile);
				continue;
			}

			// Write to disk
			if (!FFileHelper::SaveArrayToFile(FileData, *FullPath))
			{
				UE_LOG(LogTemp, Error, TEXT("Failed to write extracted file: %s"), *FullPath);
				unzGoToNextFile(ZipFile);
				continue;
			}
		}

		// Move to next file
		if ((i + 1) < GlobalInfo.number_entry)
		{
			if (unzGoToNextFile(ZipFile) != UNZ_OK)
			{
				UE_LOG(LogTemp, Error, TEXT("Failed to move to next file in ZIP"));
				break;
			}
		}
	}

	unzClose(ZipFile);

	UE_LOG(LogTemp, Log, TEXT("Extracted %d files from ZIP"), GlobalInfo.number_entry);
	return true;
}

bool FTripoModelImporter::FindModelFile(const FString& DirectoryPath, FString& OutModelPath)
{
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();

	TArray<FString> FoundFiles;
	PlatformFile.FindFilesRecursively(FoundFiles, *DirectoryPath, nullptr);

	int32 BestPriority = TNumericLimits<int32>::Max();
	FString BestMatch;

	for (const FString& FilePath : FoundFiles)
	{
		FString Extension = FPaths::GetExtension(FilePath).ToLower();
		int32 CurrentPriority = TNumericLimits<int32>::Max();

		if (Extension == TEXT("fbx"))
		{
			CurrentPriority = 0;
		}
		else if (Extension == TEXT("glb"))
		{
			CurrentPriority = 1;
		}
		else if (Extension == TEXT("gltf"))
		{
			CurrentPriority = 2;
		}
		else if (Extension == TEXT("obj"))
		{
			CurrentPriority = 3;
		}

		if (CurrentPriority < BestPriority)
		{
			BestPriority = CurrentPriority;
			BestMatch = FilePath;

			if (BestPriority == 0)
			{
				break;
			}
		}
	}

	if (BestPriority != TNumericLimits<int32>::Max())
	{
		OutModelPath = BestMatch;
		return true;
	}

	return false;
}

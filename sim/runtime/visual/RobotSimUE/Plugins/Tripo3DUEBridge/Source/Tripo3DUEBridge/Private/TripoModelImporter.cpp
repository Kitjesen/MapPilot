// Copyright Tripo3D. All Rights Reserved.
// Core import orchestration: PrepareModelFile, ImportModelToEngine, ImportFBX, OrganizeImportedAssets
// ZIP extraction → TripoZipExtractor.cpp
// PBR material & texture import → TripoPBRMaterialBuilder.cpp
// Scene spawning → TripoSceneSpawner.cpp

#include "TripoModelImporter.h"
#include "TripoVersionCompat.h"
#include "TripoProtocol.h"
#include "AssetRegistry/AssetRegistryModule.h"
#include "AssetToolsModule.h"
#include "IAssetTools.h"
#include "AssetImportTask.h"
#include "Factories/FbxFactory.h"
#include "Factories/FbxImportUI.h"
#include "Factories/FbxStaticMeshImportData.h"
#include "Factories/FbxSkeletalMeshImportData.h"
#include "Factories/FbxAnimSequenceImportData.h"
#include "Factories/FbxTextureImportData.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/PackageName.h"
#include "HAL/PlatformFileManager.h"
#include "GenericPlatform/GenericPlatformFile.h"
#include "Engine/StaticMesh.h"
#include "Engine/SkeletalMesh.h"
#include "Engine/SkinnedAssetCommon.h"
#include "Engine/Texture2D.h"
#include "Materials/MaterialInterface.h"
#include "Animation/AnimSequence.h"
#include "Animation/Skeleton.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "InterchangeGenericAnimationPipeline.h"
#include "InterchangeGenericAssetsPipeline.h"
#include "InterchangeManager.h"

// ————— Static Helpers —————

static FString SanitizeAssetName(const FString& RawName, const FString& FileId)
{
	FString BaseName = FPaths::GetBaseFilename(RawName);

	BaseName.ReplaceInline(TEXT(" "), TEXT("_"));

	const TCHAR IllegalChars[] = TEXT("/\\:*?\"<>|");
	for (int32 i = 0; IllegalChars[i] != '\0'; ++i)
	{
		BaseName.ReplaceCharInline(IllegalChars[i], TEXT('_'));
	}

	BaseName.TrimStartAndEndInline();

	if (BaseName.IsEmpty())
	{
		BaseName = FString::Printf(TEXT("Model_%s"), *FileId.Left(8));
	}

	return BaseName;
}

static FString StripNormalizedPrefix(const FString& PartPrefix, const FString& NormalizedModel)
{
	if (NormalizedModel.IsEmpty()) return PartPrefix;

	int32 i = 0;
	int32 j = 0;
	while (i < PartPrefix.Len() && j < NormalizedModel.Len())
	{
		const TCHAR Ch = PartPrefix[i];
		if (Ch == TEXT('_')) { ++i; continue; }
		if (FChar::ToLower(Ch) != FChar::ToLower(NormalizedModel[j]))
			return PartPrefix;
		++i; ++j;
	}
	if (j < NormalizedModel.Len()) return PartPrefix;

	while (i < PartPrefix.Len() && PartPrefix[i] == TEXT('_')) ++i;

	const FString Remainder = PartPrefix.Mid(i);
	return Remainder.IsEmpty() ? PartPrefix : Remainder;
}

static FString MakeUniqueAssetPath(const FString& BasePath)
{
	FAssetRegistryModule& Reg = FModuleManager::LoadModuleChecked<FAssetRegistryModule>("AssetRegistry");

	FString FinalPath = BasePath;
	int32 Suffix = 1;
	while (Reg.Get().HasAssets(FName(*FinalPath), true))
	{
		FinalPath = FString::Printf(TEXT("%s_%d"), *BasePath, Suffix++);
	}
	return FinalPath;
}

static FString BuildAnimationAssetName(const FString& BasePackagePath, const FString& FbxBaseName, const FString& SourceName)
{
	const FString UniqueSafeName = FPaths::GetCleanFilename(BasePackagePath);

	FString AnimName = SourceName;
	if (AnimName.StartsWith(FbxBaseName))
	{
		AnimName = AnimName.Mid(FbxBaseName.Len());
	}

	AnimName.TrimStartInline();
	while (AnimName.StartsWith(TEXT("_")) || AnimName.StartsWith(TEXT("-")) || AnimName.StartsWith(TEXT(".")))
	{
		AnimName.RightChopInline(1, EAllowShrinking::No);
	}

	AnimName.ReplaceInline(TEXT(" "), TEXT("_"));
	const TCHAR IllegalChars[] = TEXT("/\\:*?\"<>|");
	for (int32 i = 0; IllegalChars[i] != '\0'; ++i)
	{
		AnimName.ReplaceCharInline(IllegalChars[i], TEXT('_'));
	}

	return FString::Printf(TEXT("%s_%s"), *UniqueSafeName, *AnimName);
}

static void AppendRootPackageAnimations(const FString& BasePackagePath, TArray<UObject*>& AnimationObjects)
{
	TSet<FString> SeenPaths;
	for (UObject* AnimationObject : AnimationObjects)
	{
		if (AnimationObject)
		{
			SeenPaths.Add(AnimationObject->GetPathName());
		}
	}

	FAssetRegistryModule& AssetRegistryModule = FModuleManager::LoadModuleChecked<FAssetRegistryModule>("AssetRegistry");
	TArray<FAssetData> RootAssets;
	AssetRegistryModule.Get().GetAssetsByPath(FName(*BasePackagePath), RootAssets, false);

	int32 AddedCount = 0;
	for (const FAssetData& AssetData : RootAssets)
	{
		if (AssetData.AssetClassPath != UAnimSequence::StaticClass()->GetClassPathName())
		{
			continue;
		}

		UObject* AnimationAsset = AssetData.GetAsset();
		if (!AnimationAsset)
		{
			continue;
		}

		const FString ObjectPath = AnimationAsset->GetPathName();
		if (SeenPaths.Contains(ObjectPath))
		{
			continue;
		}

		AnimationObjects.Add(AnimationAsset);
		SeenPaths.Add(ObjectPath);
		AddedCount++;
	}

	if (AddedCount > 0)
	{
		UE_LOG(LogTemp, Log, TEXT("[Tripo] Recovered %d animation asset(s) from root package scan"), AddedCount);
	}
}

// ————— Public API —————

bool FTripoModelImporter::PrepareModelFile(
	const FString& FileId,
	const FString& FileName,
	const FString& FileType,
	const TArray<uint8>& FileData,
	FTripoImportContext& OutContext)
{
	OutContext.FileId = FileId;
	OutContext.FileName = FileName;
	OutContext.FileType = FileType;

	UE_LOG(LogTemp, Log, TEXT("Preparing model file: %s (Type: %s, Size: %.2f MB)"),
		*FileName, *FileType, FileData.Num() / 1024.0f / 1024.0f);

	OutContext.TempDir = FPaths::Combine(
		FPaths::ProjectIntermediateDir(),
		TripoProtocol::TEMP_DIR_NAME,
		FileId
	);

	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();

	if (!PlatformFile.CreateDirectoryTree(*OutContext.TempDir))
	{
		OutContext.ErrorMessage = FString::Printf(TEXT("Failed to create temp directory: %s"), *OutContext.TempDir);
		UE_LOG(LogTemp, Error, TEXT("%s"), *OutContext.ErrorMessage);
		return false;
	}

	UE_LOG(LogTemp, Log, TEXT("Created temp directory: %s"), *OutContext.TempDir);

	if (FileType.Equals(TEXT("zip"), ESearchCase::IgnoreCase))
	{
		UE_LOG(LogTemp, Log, TEXT("Extracting ZIP file: %s"), *FileName);

		if (!ExtractZip(FileData, OutContext.TempDir))
		{
			OutContext.ErrorMessage = TEXT("Failed to extract ZIP file");
			UE_LOG(LogTemp, Error, TEXT("%s"), *OutContext.ErrorMessage);
			return false;
		}

		if (!FindModelFile(OutContext.TempDir, OutContext.ModelFilePath))
		{
			OutContext.ErrorMessage = TEXT("No FBX, GLB, GLTF, or OBJ file found in ZIP archive");
			UE_LOG(LogTemp, Error, TEXT("%s"), *OutContext.ErrorMessage);
			return false;
		}
		UE_LOG(LogTemp, Log, TEXT("Found model file: %s"), *FPaths::GetCleanFilename(OutContext.ModelFilePath));
	}
	else if (FileType.Equals(TEXT("fbx"), ESearchCase::IgnoreCase) ||
	         FileType.Equals(TEXT("obj"), ESearchCase::IgnoreCase) ||
	         FileType.Equals(TEXT("glb"), ESearchCase::IgnoreCase) ||
	         FileType.Equals(TEXT("gltf"), ESearchCase::IgnoreCase))
	{
		OutContext.ModelFilePath = FPaths::Combine(OutContext.TempDir, FileName);
		if (!FFileHelper::SaveArrayToFile(FileData, *OutContext.ModelFilePath))
		{
			OutContext.ErrorMessage = FString::Printf(TEXT("Failed to save model file to: %s"), *OutContext.ModelFilePath);
			UE_LOG(LogTemp, Error, TEXT("%s"), *OutContext.ErrorMessage);
			return false;
		}
	}
	else
	{
		OutContext.ErrorMessage = FString::Printf(TEXT("Unsupported file type: %s"), *FileType);
		UE_LOG(LogTemp, Error, TEXT("%s"), *OutContext.ErrorMessage);
		return false;
	}

	UE_LOG(LogTemp, Log, TEXT("Model file prepared successfully: %s"), *OutContext.ModelFilePath);
	return true;
}

bool FTripoModelImporter::ImportModelToEngine(const FTripoImportContext& Context)
{
	check(IsInGameThread());

	if (!Context.IsValid())
	{
		UE_LOG(LogTemp, Error, TEXT("Invalid import context"));
		return false;
	}

	UE_LOG(LogTemp, Log, TEXT("Importing model to engine: %s"), *Context.FileName);

	FString SafeName = SanitizeAssetName(Context.FileName, Context.FileId);
	FString AssetPackagePath = MakeUniqueAssetPath(
		FPaths::Combine(TripoProtocol::ASSET_IMPORT_ROOT, SafeName));

	FString UniqueSafeName = FPaths::GetCleanFilename(AssetPackagePath);

	UObject* ImportedAsset = nullptr;
	if (!ImportFBX(Context.ModelFilePath, AssetPackagePath, ImportedAsset))
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to import model: %s"), *Context.ModelFilePath);
		DeleteDirectoryWithRetry(Context.TempDir);
		return false;
	}

	const FString FbxBaseName = FPaths::GetBaseFilename(Context.ModelFilePath);

#if TRIPO_HAS_FBX_IMPORT_UI
	{
		// UE 5.4 legacy FBX import can leave supplemental PBR textures outside the imported asset set,
		// so we import them explicitly before the shared rename/organization/PBR pass runs.
		const FString TexturesPath = FPaths::Combine(AssetPackagePath, TEXT("Textures"));
		const int32 ExtraTexCount = ImportTextures(Context.TempDir, TexturesPath);
		UE_LOG(LogTemp, Log, TEXT("[Tripo][PBR] Imported %d additional PBR texture(s) from TempDir"), ExtraTexCount);
	}
#endif

	// Rename imported assets to use the display name
	{
		FAssetRegistryModule& AssetRegistryModule = FModuleManager::LoadModuleChecked<FAssetRegistryModule>("AssetRegistry");
		TArray<FAssetData> AllAssets;
		AssetRegistryModule.Get().GetAssetsByPath(FName(*AssetPackagePath), AllAssets, true);

		IAssetTools& AssetTools = FModuleManager::LoadModuleChecked<FAssetToolsModule>("AssetTools").Get();
		TArray<FAssetRenameData> RenameList;

		TArray<FString> SortedMeshNames;
		for (const FAssetData& AD : AllAssets)
		{
			if (AD.AssetClassPath == UStaticMesh::StaticClass()->GetClassPathName() ||
				AD.AssetClassPath == USkeletalMesh::StaticClass()->GetClassPathName())
			{
				SortedMeshNames.Add(AD.AssetName.ToString());
			}
		}
		SortedMeshNames.Sort();
		const int32 TotalMeshCount = SortedMeshNames.Num();

		TArray<FString> TextureSuffixes = {
			TEXT("_basecolor"), TEXT("_base_color"), TEXT("_albedo"),
			TEXT("_normal"), TEXT("_normalmap"), TEXT("_norm"),
			TEXT("_roughness"), TEXT("_rough"),
			TEXT("_metallic"), TEXT("_metalness"),
			TEXT("_ao"), TEXT("_ambientocclusion"),
			TEXT("_emissive"), TEXT("_emission"),
			TEXT("_opacity"), TEXT("_alpha"),
			TEXT("_height"), TEXT("_displacement")
		};

		const FString LowerFbxBaseName = FbxBaseName.ToLower();
		const FString LowerUniqueSafeName = UniqueSafeName.ToLower();

		int32 TexIndex = 0;
		for (const FAssetData& AssetData : AllAssets)
		{
			UObject* Asset = AssetData.GetAsset();
			if (!Asset) continue;

			FString NewName;
			FString OldName = Asset->GetName();

			if (Asset->IsA<UStaticMesh>() || Asset->IsA<USkeletalMesh>())
			{
				if (TotalMeshCount <= 1)
				{
					NewName = UniqueSafeName;
				}
				else
				{
					const int32 PartIdx = SortedMeshNames.IndexOfByKey(OldName);
					NewName = FString::Printf(TEXT("%s_%d"), *UniqueSafeName, PartIdx >= 0 ? PartIdx : 0);
				}
			}
			else if (Asset->IsA<UTexture2D>())
			{
				const FString LowerOldName = OldName.ToLower();
				FString PreservedSuffix;

				for (const FString& Suffix : TextureSuffixes)
				{
					if (LowerOldName.EndsWith(Suffix))
					{
						PreservedSuffix = Suffix;
						break;
					}
				}

				if (!PreservedSuffix.IsEmpty())
				{
					const int32 SuffixStart = OldName.Len() - PreservedSuffix.Len();
					const FString PartPrefix = (SuffixStart > 0) ? OldName.Left(SuffixStart) : FString();
					const FString LowerPartPrefix = PartPrefix.ToLower();
					const FString NormalizedPartPrefix  = LowerPartPrefix.Replace(TEXT("_"), TEXT(""));
					const FString NormalizedFbxBase     = LowerFbxBaseName.Replace(TEXT("_"), TEXT(""));
					const FString NormalizedSafeName    = LowerUniqueSafeName.Replace(TEXT("_"), TEXT(""));
					const bool bIsMainTex = PartPrefix.IsEmpty()
						|| LowerPartPrefix    == LowerFbxBaseName
						|| LowerPartPrefix    == LowerUniqueSafeName
						|| NormalizedPartPrefix == NormalizedFbxBase
						|| NormalizedPartPrefix == NormalizedSafeName;

					if (bIsMainTex)
					{
						NewName = FString::Printf(TEXT("%s%s"), *UniqueSafeName, *PreservedSuffix);
					}
					else
					{
						FString ActualPartName = PartPrefix;
						if (NormalizedPartPrefix.StartsWith(NormalizedSafeName))
							ActualPartName = StripNormalizedPrefix(PartPrefix, NormalizedSafeName);
						else if (NormalizedPartPrefix.StartsWith(NormalizedFbxBase) && !NormalizedFbxBase.IsEmpty())
							ActualPartName = StripNormalizedPrefix(PartPrefix, NormalizedFbxBase);

						NewName = FString::Printf(TEXT("%s_%s%s"), *UniqueSafeName, *ActualPartName, *PreservedSuffix);
					}
				}
				else
				{
					NewName = (TexIndex == 0)
						? FString::Printf(TEXT("%s_Tex"), *UniqueSafeName)
						: FString::Printf(TEXT("%s_Tex_%d"), *UniqueSafeName, TexIndex);
					TexIndex++;
				}
			}
			else if (Asset->IsA<USkeleton>())
			{
				NewName = FString::Printf(TEXT("%s_Skeleton"), *UniqueSafeName);
			}
			else if (Asset->IsA<UPhysicsAsset>())
			{
				NewName = FString::Printf(TEXT("%s_PhysicsAsset"), *UniqueSafeName);
			}

			if (!NewName.IsEmpty() && OldName != NewName)
			{
				FAssetRenameData RenameData;
				RenameData.Asset = Asset;
				RenameData.NewPackagePath = AssetData.PackagePath.ToString();
				RenameData.NewName = NewName;
				RenameList.Add(RenameData);
			}
		}

		if (RenameList.Num() > 0)
		{
			AssetTools.RenameAssets(RenameList);
			UE_LOG(LogTemp, Log, TEXT("[Tripo] Renamed %d assets to use display name: %s"), RenameList.Num(), *UniqueSafeName);

			if (ImportedAsset)
			{
				const FString NewMeshBaseName = (TotalMeshCount > 1)
					? FString::Printf(TEXT("%s_0"), *UniqueSafeName)
					: UniqueSafeName;
				FString MeshPath = FString::Printf(TEXT("%s/%s.%s"),
					*AssetPackagePath, *NewMeshBaseName, *NewMeshBaseName);
				if (UObject* Reloaded = LoadObject<UObject>(nullptr, *MeshPath))
				{
					ImportedAsset = Reloaded;
				}
			}
		}
	}

	// Collect all mesh assets for PBR and spawning
	TArray<UObject*> AllPartMeshes;

	if (ImportedAsset)
	{
		// PBR post-processing is shared across supported UE versions.
		// UE 5.4 feeds this from ImportTextures above; UE 5.5+ relies on importer-created textures
		// that are renamed and moved under Textures/ by OrganizeImportedAssets.
		const FString TexturesPath = FPaths::Combine(AssetPackagePath, TEXT("Textures"));
		TSet<UMaterialInterface*> MaterialsBeforePBR;
		bool bAppliedAnyPBR = false;

		AllPartMeshes.Add(ImportedAsset);
		{
			FAssetRegistryModule& AssetReg = FModuleManager::LoadModuleChecked<FAssetRegistryModule>("AssetRegistry");
			TArray<FAssetData> RootAssets;
			AssetReg.Get().GetAssetsByPath(FName(*AssetPackagePath), RootAssets, false);

			for (const FAssetData& AD : RootAssets)
			{
				const bool bIsMesh =
					AD.AssetClassPath == UStaticMesh::StaticClass()->GetClassPathName() ||
					AD.AssetClassPath == USkeletalMesh::StaticClass()->GetClassPathName();
				if (!bIsMesh) continue;
				UObject* PartMesh = AD.GetAsset();
				if (!PartMesh || PartMesh == ImportedAsset) continue;
				AllPartMeshes.Add(PartMesh);
			}
			AllPartMeshes.Sort([](const UObject& A, const UObject& B)
			{
				return A.GetName() < B.GetName();
			});
		}

		CollectMeshMaterialInterfaces(AllPartMeshes, MaterialsBeforePBR);

		for (UObject* Mesh : AllPartMeshes)
		{
			UE_LOG(LogTemp, Log, TEXT("[Tripo][PBR] Applying PBR to mesh: %s"), *Mesh->GetName());
			if (ApplyPBRMaterialPerSlot(Mesh, TexturesPath, AssetPackagePath, UniqueSafeName))
			{
				bAppliedAnyPBR = true;
			}
			else
			{
				UE_LOG(LogTemp, Warning, TEXT("[Tripo][PBR] ApplyPBRMaterialPerSlot failed for: %s"), *Mesh->GetName());
			}
		}

		if (bAppliedAnyPBR)
		{
			RemoveUnreferencedMaterialAssets(AllPartMeshes, MaterialsBeforePBR, AssetPackagePath);
		}
		else
		{
			UE_LOG(LogTemp, Log, TEXT("[Tripo][PBR] No material slots were reassigned; skipping imported material cleanup"));
		}
	}

	FString ActorName = Context.FileName;
	if (ActorName.EndsWith(TEXT(".zip"), ESearchCase::IgnoreCase))
	{
		ActorName = FPaths::GetBaseFilename(ActorName);
	}

	if (AllPartMeshes.Num() > 1)
	{
		if (!SpawnMultiPartInLevel(AllPartMeshes, ActorName))
		{
			UE_LOG(LogTemp, Warning, TEXT("[Tripo] SpawnMultiPartInLevel failed"));
		}
	}
	else if (ImportedAsset && !SpawnInLevel(ImportedAsset, ActorName))
	{
		UE_LOG(LogTemp, Warning, TEXT("Failed to spawn model in level (import succeeded)"));
	}

	DeleteDirectoryWithRetry(Context.TempDir);

	UE_LOG(LogTemp, Log, TEXT("Successfully imported %s"), *Context.FileName);
	return true;
}

bool FTripoModelImporter::ImportModel(
	const FString& FileId,
	const FString& FileName,
	const FString& FileType,
	const TArray<uint8>& FileData)
{
	FTripoImportContext Context;

	if (!PrepareModelFile(FileId, FileName, FileType, FileData, Context))
	{
		if (!Context.TempDir.IsEmpty())
		{
			DeleteDirectoryWithRetry(Context.TempDir);
		}
		return false;
	}

	return ImportModelToEngine(Context);
}

// ————— FBX Import —————

bool FTripoModelImporter::ImportFBX(const FString& SourcePath, const FString& DestPackagePath, UObject*& OutImportedAsset)
{
	if (!FPaths::FileExists(SourcePath))
	{
		UE_LOG(LogTemp, Error, TEXT("FBX file not found: %s"), *SourcePath);
		return false;
	}

	UAssetImportTask* ImportTask = NewObject<UAssetImportTask>();
	if (!ImportTask)
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to create AssetImportTask"));
		return false;
	}

	ImportTask->Filename = SourcePath;
	ImportTask->DestinationPath = DestPackagePath;
	ImportTask->bAutomated = true;
	ImportTask->bReplaceExisting = true;
	ImportTask->bSave = false;

	bool bUsingDefaultImporter = true;

#if TRIPO_HAS_FBX_IMPORT_UI
	UFbxImportUI* ImportUI = NewObject<UFbxImportUI>();
	if (!ImportUI)
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to create UFbxImportUI for legacy FBX validation branch"));
		return false;
	}

	ImportUI->bImportMesh = true;
	ImportUI->bImportAnimations = true;
	ImportUI->bImportAsSkeletal = false;
	ImportUI->bAutomatedImportShouldDetectType = true;
	ImportUI->MeshTypeToImport = FBXIT_StaticMesh;
	ImportUI->OriginalImportType = FBXIT_StaticMesh;
	ImportUI->bImportMaterials = true;
	ImportUI->bImportTextures = true;

	UFbxStaticMeshImportData* StaticMeshData = ImportUI->StaticMeshImportData;
	if (StaticMeshData)
	{
		StaticMeshData->ImportUniformScale = TRIPO_FBX_SCALE_FACTOR;
		StaticMeshData->bCombineMeshes = false;
		StaticMeshData->bGenerateLightmapUVs = true;
		StaticMeshData->bAutoGenerateCollision = true;

		UE_LOG(LogTemp, Log, TEXT("[Tripo] Legacy FBX validation branch scale factor: %.2f"),
			StaticMeshData->ImportUniformScale);
	}

#if TRIPO_NEEDS_SKELETAL_MESH_SCALE_FIX
	UFbxSkeletalMeshImportData* SkeletalMeshData = ImportUI->SkeletalMeshImportData;
	if (SkeletalMeshData)
	{
		SkeletalMeshData->ImportUniformScale = TRIPO_FBX_SCALE_FACTOR;
		UE_LOG(LogTemp, Log, TEXT("[Tripo][UE5.4] Skeletal mesh scale factor: %.2f"), SkeletalMeshData->ImportUniformScale);
	}
#endif

	UFbxAnimSequenceImportData* AnimData = ImportUI->AnimSequenceImportData;
	if (AnimData)
	{
		AnimData->bSnapToClosestFrameBoundary = true;
	}

	UFbxTextureImportData* TextureData = ImportUI->TextureImportData;
	if (TextureData)
	{
		TextureData->MaterialSearchLocation = TRIPO_MATERIAL_SEARCH_LOCATION;
	}

	ImportTask->Options = ImportUI;
	bUsingDefaultImporter = false;

	UE_LOG(LogTemp, Log, TEXT("[Tripo] Legacy FBX validation branch: using UFbxImportUI"));
	UE_LOG(LogTemp, Log, TEXT("[Tripo] Legacy FBX import config: detectType=%s, importAsSkeletal=%s, importAnimations=%s"),
		ImportUI->bAutomatedImportShouldDetectType ? TEXT("true") : TEXT("false"),
		ImportUI->bImportAsSkeletal ? TEXT("true") : TEXT("false"),
		ImportUI->bImportAnimations ? TEXT("true") : TEXT("false"));
	UE_LOG(LogTemp, Log, TEXT("[Tripo] bImportMaterials: %s, bImportTextures: %s"),
		ImportUI->bImportMaterials ? TEXT("true") : TEXT("false"),
		ImportUI->bImportTextures ? TEXT("true") : TEXT("false"));
#else
	UInterchangePipelineStackOverride* PipelineOverride = NewObject<UInterchangePipelineStackOverride>();
	if (!PipelineOverride)
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to create UInterchangePipelineStackOverride"));
		return false;
	}

	UInterchangeGenericAssetsPipeline* GenericAssetsPipeline = NewObject<UInterchangeGenericAssetsPipeline>(PipelineOverride);
	if (!GenericAssetsPipeline || !GenericAssetsPipeline->AnimationPipeline)
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to create Interchange generic assets pipeline override"));
		return false;
	}

#if TRIPO_INTERCHANGE_HAS_FRAME_ALIGNMENT
	GenericAssetsPipeline->AnimationPipeline->FrameAlignment = EInterchangeFrameAlignment::SnapToClosest;
#else
	GenericAssetsPipeline->AnimationPipeline->bSnapToClosestFrameBoundary = true;
#endif

	PipelineOverride->AddPipeline(GenericAssetsPipeline);
	ImportTask->Options = PipelineOverride;

	UE_LOG(LogTemp, Log, TEXT("[Tripo] Interchange override: snap animation to closest frame boundary"));
	UE_LOG(LogTemp, Log, TEXT("[Tripo] Validation branch: using bare UAssetImportTask with default importer auto-detection (UE 5.5+)"));
#endif

	UE_LOG(LogTemp, Log, TEXT("Starting FBX import: %s -> %s"), *SourcePath, *DestPackagePath);
	UE_LOG(LogTemp, Log, TEXT("[Tripo] ImportTask->Options assigned: %s"),
		bUsingDefaultImporter ? TEXT("false") : TEXT("true"));

	FAssetToolsModule& AssetToolsModule = FModuleManager::LoadModuleChecked<FAssetToolsModule>("AssetTools");
	AssetToolsModule.Get().ImportAssetTasks({ImportTask});

	if (ImportTask->ImportedObjectPaths.Num() == 0)
	{
		UE_LOG(LogTemp, Error, TEXT("FBX import failed - no objects imported from: %s"), *SourcePath);
		return false;
	}

	UE_LOG(LogTemp, Log, TEXT("FBX import succeeded, imported %d objects"), ImportTask->ImportedObjectPaths.Num());

	int32 ImportedAnimationCount = 0;
	int32 ImportedSkeletonCount = 0;
	for (const FString& ImportedPath : ImportTask->ImportedObjectPaths)
	{
		UObject* ImportedObject = LoadObject<UObject>(nullptr, *ImportedPath);
		if (!ImportedObject)
		{
			continue;
		}

		UE_LOG(LogTemp, Log, TEXT("Imported object: %s (%s)"), *ImportedObject->GetName(), *ImportedObject->GetClass()->GetName());

		if (ImportedObject->IsA<UAnimSequence>())
		{
			ImportedAnimationCount++;
		}
		else if (ImportedObject->GetClass()->GetName() == TEXT("Skeleton"))
		{
			ImportedSkeletonCount++;
		}

		if (ImportedObject->IsA<UStaticMesh>() || ImportedObject->IsA<USkeletalMesh>())
		{
			OutImportedAsset = ImportedObject;
			UE_LOG(LogTemp, Log, TEXT("Found main mesh asset: %s"), *OutImportedAsset->GetName());
			break;
		}
	}

	UE_LOG(LogTemp, Log, TEXT("[Tripo] Imported animation assets: %d, skeleton assets: %d"),
		ImportedAnimationCount, ImportedSkeletonCount);

	FString FbxBaseName = FPaths::GetBaseFilename(SourcePath);
	OrganizeImportedAssets(ImportTask->ImportedObjectPaths, DestPackagePath, FbxBaseName);

	if (OutImportedAsset)
	{
		UE_LOG(LogTemp, Log, TEXT("Successfully imported FBX: %s (%s)"),
			*OutImportedAsset->GetName(), *OutImportedAsset->GetClass()->GetName());
		return true;
	}

	if (ImportTask->ImportedObjectPaths.Num() > 0)
	{
		FAssetRegistryModule& AssetRegistryModule = FModuleManager::LoadModuleChecked<FAssetRegistryModule>("AssetRegistry");
		TArray<FAssetData> AssetDataList;
		AssetRegistryModule.Get().GetAssetsByPath(FName(*DestPackagePath), AssetDataList, true);

		for (const FAssetData& AssetData : AssetDataList)
		{
			if (AssetData.AssetClassPath == UStaticMesh::StaticClass()->GetClassPathName() ||
			    AssetData.AssetClassPath == USkeletalMesh::StaticClass()->GetClassPathName())
			{
				OutImportedAsset = AssetData.GetAsset();
				if (OutImportedAsset)
				{
					UE_LOG(LogTemp, Warning, TEXT("Found mesh after organizing: %s (%s)"),
						*OutImportedAsset->GetName(), *OutImportedAsset->GetClass()->GetName());
					return true;
				}
			}
		}
	}

	UE_LOG(LogTemp, Error, TEXT("No suitable asset found in import results"));
	return false;
}

// ————— Asset Organization —————

void FTripoModelImporter::OrganizeImportedAssets(const TArray<FString>& ImportedObjectPaths, const FString& BasePackagePath, const FString& FbxBaseName)
{
	FAssetToolsModule& AssetToolsModule = FModuleManager::LoadModuleChecked<FAssetToolsModule>("AssetTools");
	IAssetTools& AssetTools = AssetToolsModule.Get();
	FAssetRegistryModule& AssetRegistryModule = FModuleManager::LoadModuleChecked<FAssetRegistryModule>("AssetRegistry");

	TArray<UObject*> AnimationObjects;
	for (const FString& ImportedPath : ImportedObjectPaths)
	{
		UObject* ImportedObject = LoadObject<UObject>(nullptr, *ImportedPath);
		if (ImportedObject && ImportedObject->GetClass()->GetName() == TEXT("AnimSequence"))
			AnimationObjects.Add(ImportedObject);
	}

	if (AnimationObjects.Num() == 0)
	{
		AppendRootPackageAnimations(BasePackagePath, AnimationObjects);
	}

	// Move textures to Textures/ subfolder
	{
		const FString TexturesFolder = FPaths::Combine(BasePackagePath, TEXT("Textures"));

		TArray<FAssetData> RootAssets;
		AssetRegistryModule.Get().GetAssetsByPath(FName(*BasePackagePath), RootAssets, false);

		TArray<FAssetRenameData> RenameData;
		for (const FAssetData& AD : RootAssets)
		{
			if (AD.AssetClassPath != UTexture2D::StaticClass()->GetClassPathName())
				continue;

			UObject* Tex = AD.GetAsset();
			if (!Tex) continue;

			FAssetRenameData Entry;
			Entry.Asset          = Tex;
			Entry.NewPackagePath = TexturesFolder;
			Entry.NewName        = Tex->GetName();
			RenameData.Add(Entry);
		}

		if (RenameData.Num() > 0)
		{
			AssetTools.RenameAssets(RenameData);
			UE_LOG(LogTemp, Log, TEXT("[Tripo] Moved %d root texture(s) to Textures/ subfolder"), RenameData.Num());
		}
	}

	// Move animations to Animations/ subfolder
	if (AnimationObjects.Num() > 0)
	{
		FString AnimationsFolder = FPaths::Combine(BasePackagePath, TEXT("Animations"));

		TArray<FAssetRenameData> RenameData;
		for (UObject* AnimObject : AnimationObjects)
		{
			FAssetRenameData RenameEntry;
			RenameEntry.Asset = AnimObject;
			RenameEntry.NewPackagePath = AnimationsFolder;
			RenameEntry.NewName = BuildAnimationAssetName(BasePackagePath, FbxBaseName, AnimObject->GetName());

			RenameData.Add(RenameEntry);
		}

		AssetTools.RenameAssets(RenameData);
		UE_LOG(LogTemp, Log, TEXT("Moved %d animation(s) to Animations folder"), RenameData.Num());
	}
}

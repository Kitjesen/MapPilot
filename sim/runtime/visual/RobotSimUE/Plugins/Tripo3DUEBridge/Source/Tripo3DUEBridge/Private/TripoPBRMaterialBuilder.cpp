#include "TripoModelImporter.h"
#include "TripoVersionCompat.h"
#include "TripoProtocol.h"
#include "AssetRegistry/AssetRegistryModule.h"
#include "AssetToolsModule.h"
#include "IAssetTools.h"
#include "Factories/TextureFactory.h"
#include "Engine/Texture2D.h"
#include "Engine/StaticMesh.h"
#include "Engine/SkeletalMesh.h"
#include "Engine/SkinnedAssetCommon.h"
#include "Materials/Material.h"
#include "Materials/MaterialInterface.h"
#include "Materials/MaterialInstanceConstant.h"
#include "Materials/MaterialExpressionTextureSampleParameter2D.h"
#include "Materials/MaterialExpressionComponentMask.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/PackageName.h"
#include "HAL/PlatformFileManager.h"
#include "GenericPlatform/GenericPlatformFile.h"
#include "ObjectTools.h"
#include "UObject/SavePackage.h"
#include "Interfaces/IPluginManager.h"

// ————— Static Helpers —————

static const FString& GetPluginContentRoot()
{
	static FString CachedRoot;
	if (CachedRoot.IsEmpty())
	{
		TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("Tripo3DUEBridge"));
		if (Plugin.IsValid())
		{
			CachedRoot = Plugin->GetMountedAssetPath();
		}
		else
		{
			CachedRoot = TEXT("/Tripo3DUEBridge/");
			UE_LOG(LogTemp, Warning, TEXT("[Tripo] Could not find plugin 'Tripo3DUEBridge' via IPluginManager, using fallback path"));
		}
	}
	return CachedRoot;
}

static const FString& GetProjectMaterialRoot()
{
	static const FString CachedRoot = FPaths::Combine(TripoProtocol::ASSET_IMPORT_ROOT, TEXT("Materials"));
	return CachedRoot;
}

template <typename TObjectType>
static TObjectType* LoadAssetFromPackagePath(const FString& PkgPath, const FString& AssetName)
{
	return LoadObject<TObjectType>(nullptr, *(PkgPath + TEXT(".") + AssetName));
}

static bool SaveAssetPackage(UPackage* Package, UObject* Asset, const FString& PackagePath)
{
	if (!Package || !Asset)
	{
		return false;
	}

	FString PackageFilename;
	if (!FPackageName::TryConvertLongPackageNameToFilename(
		PackagePath, PackageFilename, FPackageName::GetAssetPackageExtension()))
	{
		return false;
	}

	FSavePackageArgs SaveArgs;
	SaveArgs.TopLevelFlags = RF_Public | RF_Standalone;
	return UPackage::SavePackage(Package, Asset, *PackageFilename, SaveArgs);
}

static void CollectMaterialInterfacesFromMesh(UObject* MeshAsset, TSet<UMaterialInterface*>& OutMaterials)
{
	if (UStaticMesh* StaticMesh = Cast<UStaticMesh>(MeshAsset))
	{
		for (const FStaticMaterial& StaticMaterial : StaticMesh->GetStaticMaterials())
		{
			if (StaticMaterial.MaterialInterface)
			{
				OutMaterials.Add(StaticMaterial.MaterialInterface);
			}
		}
		return;
	}

	if (USkeletalMesh* SkeletalMesh = Cast<USkeletalMesh>(MeshAsset))
	{
		for (const FSkeletalMaterial& SkeletalMaterial : SkeletalMesh->GetMaterials())
		{
			if (SkeletalMaterial.MaterialInterface)
			{
				OutMaterials.Add(SkeletalMaterial.MaterialInterface);
			}
		}
	}
}

static bool IsPackageUnderPath(const UObject* Asset, const FString& BasePackagePath)
{
	if (!Asset || !Asset->GetOutermost())
	{
		return false;
	}

	FString NormalizedBasePath = BasePackagePath;
	NormalizedBasePath.RemoveFromEnd(TEXT("/"));

	const FString PackageName = Asset->GetOutermost()->GetName();
	return PackageName == NormalizedBasePath || PackageName.StartsWith(NormalizedBasePath + TEXT("/"));
}

static UTexture2D* DuplicateTextureToProject(const FString& TextureName)
{
	const FString ProjectPkgPath = FPaths::Combine(GetProjectMaterialRoot(), TextureName);
	if (UTexture2D* Existing = LoadAssetFromPackagePath<UTexture2D>(ProjectPkgPath, TextureName))
	{
		return Existing;
	}

	const FString PluginPkgPath = GetPluginContentRoot() + TEXT("Materials/") + TextureName;
	UTexture2D* PluginTexture = LoadAssetFromPackagePath<UTexture2D>(PluginPkgPath, TextureName);
	if (!PluginTexture)
	{
		return nullptr;
	}

	IAssetTools& AssetTools = FModuleManager::LoadModuleChecked<FAssetToolsModule>("AssetTools").Get();
	UTexture2D* DuplicatedTexture = Cast<UTexture2D>(AssetTools.DuplicateAsset(
		TextureName,
		GetProjectMaterialRoot(),
		PluginTexture));

	if (!DuplicatedTexture)
	{
		UE_LOG(LogTemp, Warning, TEXT("[Tripo][PBR] Failed to duplicate plugin texture to project: %s"), *TextureName);
		return nullptr;
	}

	DuplicatedTexture->MarkPackageDirty();
	SaveAssetPackage(DuplicatedTexture->GetOutermost(), DuplicatedTexture, ProjectPkgPath);
	return DuplicatedTexture;
}

static UTexture2D* LoadOrCreateProjectTexture(
	const FString& TextureName,
	FColor FallbackColor,
	TextureCompressionSettings FallbackCompression,
	bool bFallbackSRGB)
{
	const FString PkgPath = FPaths::Combine(GetProjectMaterialRoot(), TextureName);
	if (UTexture2D* Existing = LoadAssetFromPackagePath<UTexture2D>(PkgPath, TextureName))
	{
		return Existing;
	}

	if (UTexture2D* DuplicatedTexture = DuplicateTextureToProject(TextureName))
	{
		return DuplicatedTexture;
	}

	UE_LOG(LogTemp, Warning, TEXT("[Tripo][PBR] Project texture not found, creating fallback: %s"), *PkgPath);

	UPackage* Pkg = CreatePackage(*PkgPath);
	if (!Pkg) return nullptr;
	Pkg->FullyLoad();

	UTexture2D* Tex = NewObject<UTexture2D>(Pkg, *TextureName, RF_Public | RF_Standalone);
	Tex->Source.Init(1, 1, 1, 1, TSF_BGRA8);
	uint8* Pixels = Tex->Source.LockMip(0);
	Pixels[0] = FallbackColor.B;
	Pixels[1] = FallbackColor.G;
	Pixels[2] = FallbackColor.R;
	Pixels[3] = FallbackColor.A;
	Tex->Source.UnlockMip(0);
	Tex->SRGB                = bFallbackSRGB;
	Tex->CompressionSettings = FallbackCompression;
	Tex->PostEditChange();
	Tex->UpdateResource();
	FAssetRegistryModule::AssetCreated(Tex);
	Pkg->MarkPackageDirty();
	SaveAssetPackage(Pkg, Tex, PkgPath);
	return Tex;
}

void FTripoModelImporter::CollectMeshMaterialInterfaces(const TArray<UObject*>& MeshAssets, TSet<UMaterialInterface*>& OutMaterials)
{
	for (UObject* MeshAsset : MeshAssets)
	{
		CollectMaterialInterfacesFromMesh(MeshAsset, OutMaterials);
	}
}

void FTripoModelImporter::RemoveUnreferencedMaterialAssets(
	const TArray<UObject*>& MeshAssets,
	const TSet<UMaterialInterface*>& CandidateMaterials,
	const FString& BasePackagePath)
{
	// This cleanup is intentionally scoped to materials under the import destination that are no longer
	// referenced after Tripo PBR instances have been assigned. That includes importer-generated materials
	// from the default Interchange path when they were replaced by Tripo material instances.
	if (CandidateMaterials.IsEmpty())
	{
		return;
	}

	TSet<UMaterialInterface*> CurrentMaterials;
	CollectMeshMaterialInterfaces(MeshAssets, CurrentMaterials);

	TSet<UMaterialInterface*> MaterialsToConsider = CandidateMaterials;
	{
		FAssetRegistryModule& AssetRegistry = FModuleManager::LoadModuleChecked<FAssetRegistryModule>("AssetRegistry");
		TArray<FAssetData> ImportedAssets;
		AssetRegistry.Get().GetAssetsByPath(FName(*BasePackagePath), ImportedAssets, true);

		for (const FAssetData& AssetData : ImportedAssets)
		{
			if (UMaterialInterface* MaterialAsset = Cast<UMaterialInterface>(AssetData.GetAsset()))
			{
				MaterialsToConsider.Add(MaterialAsset);
			}
		}
	}

	TArray<UObject*> MaterialsToDelete;
	for (UMaterialInterface* Candidate : MaterialsToConsider)
	{
		if (!Candidate || CurrentMaterials.Contains(Candidate))
		{
			continue;
		}

		if (!IsPackageUnderPath(Candidate, BasePackagePath))
		{
			continue;
		}

		MaterialsToDelete.AddUnique(Candidate);
	}

	if (MaterialsToDelete.Num() > 0)
	{
		ObjectTools::ForceDeleteObjects(MaterialsToDelete, false);
		UE_LOG(LogTemp, Log, TEXT("[Tripo][PBR] Removed %d unreferenced imported material asset(s) after Tripo PBR reassignment"), MaterialsToDelete.Num());
	}
}

static void UpdateMasterMaterialTextureDefaults(
	UMaterial* Material,
	UTexture2D* BaseColorTexture,
	UTexture2D* NormalTexture,
	UTexture2D* MetallicTexture,
	UTexture2D* RoughnessTexture)
{
	if (!Material)
	{
		return;
	}

	bool bChanged = false;
	for (UMaterialExpression* Expression : Material->GetExpressionCollection().Expressions)
	{
		UMaterialExpressionTextureSampleParameter2D* TextureParam = Cast<UMaterialExpressionTextureSampleParameter2D>(Expression);
		if (!TextureParam)
		{
			continue;
		}

		UTexture2D* DesiredTexture = nullptr;
		if (TextureParam->ParameterName == TEXT("BaseColorTex"))
		{
			DesiredTexture = BaseColorTexture;
		}
		else if (TextureParam->ParameterName == TEXT("NormalTex"))
		{
			DesiredTexture = NormalTexture;
		}
		else if (TextureParam->ParameterName == TEXT("MetallicTex"))
		{
			DesiredTexture = MetallicTexture;
		}
		else if (TextureParam->ParameterName == TEXT("RoughnessTex"))
		{
			DesiredTexture = RoughnessTexture;
		}

		if (DesiredTexture && TextureParam->Texture != DesiredTexture)
		{
			TextureParam->Texture = DesiredTexture;
			bChanged = true;
		}
	}

	if (!bChanged)
	{
		return;
	}

	Material->PreEditChange(nullptr);
	Material->PostEditChange();
	Material->MarkPackageDirty();
	SaveAssetPackage(Material->GetOutermost(), Material, FPaths::Combine(GetProjectMaterialRoot(), TEXT("M_Tripo_PBR_Master")));
}

// ————— Texture Import —————

bool FTripoModelImporter::IsNormalMapTexture(const FString& Filename)
{
	FString LowerFilename = Filename.ToLower();

	return LowerFilename.Contains(TEXT("normal")) ||
	       LowerFilename.Contains(TEXT("_n.")) ||
	       LowerFilename.Contains(TEXT("_nrm.")) ||
	       LowerFilename.Contains(TEXT("_norm.")) ||
	       LowerFilename.EndsWith(TEXT("_n.png")) ||
	       LowerFilename.EndsWith(TEXT("_n.jpg")) ||
	       LowerFilename.EndsWith(TEXT("_n.tga")) ||
	       LowerFilename.EndsWith(TEXT("_n.tif"));
}

int32 FTripoModelImporter::ImportTextures(const FString& SourcePath, const FString& DestPackagePath)
{
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	IAssetTools& AssetTools = FModuleManager::LoadModuleChecked<FAssetToolsModule>("AssetTools").Get();

	TArray<FString> TextureExtensions = { TEXT("png"), TEXT("jpg"), TEXT("jpeg"), TEXT("tga"), TEXT("bmp"), TEXT("tif"), TEXT("tiff") };
	TArray<FString> FoundFiles;
	PlatformFile.FindFilesRecursively(FoundFiles, *SourcePath, nullptr);

	int32 ImportCount = 0;
	UTextureFactory* TextureFactory = NewObject<UTextureFactory>();
	TextureFactory->AddToRoot();

	for (const FString& FilePath : FoundFiles)
	{
		FString Extension = FPaths::GetExtension(FilePath).ToLower();
		if (!TextureExtensions.Contains(Extension))
		{
			continue;
		}

		FString Filename = FPaths::GetBaseFilename(FilePath);
		FString PackageName = FPaths::Combine(DestPackagePath, Filename);

		const FString LowerFilename = Filename.ToLower();
		const bool bIsNormalMap = IsNormalMapTexture(Filename);
		const bool bIsLinearGrayscale =
			LowerFilename.Contains(TEXT("metallic"))  ||
			LowerFilename.Contains(TEXT("metalness")) ||
			LowerFilename.Contains(TEXT("roughness")) ||
			LowerFilename.Contains(TEXT("rough"));

		if (bIsNormalMap)
		{
			UE_LOG(LogTemp, Log, TEXT("[Tripo] Detected normal map: %s"), *Filename);
			TextureFactory->CompressionSettings = TC_Normalmap;
			TextureFactory->LODGroup = TEXTUREGROUP_WorldNormalMap;
			TextureFactory->bFlipNormalMapGreenChannel = false;
		}
		else if (bIsLinearGrayscale)
		{
			UE_LOG(LogTemp, Log, TEXT("[Tripo] Detected linear grayscale map: %s"), *Filename);
			TextureFactory->CompressionSettings = TC_Grayscale;
			TextureFactory->LODGroup = TEXTUREGROUP_World;
		}
		else
		{
			TextureFactory->CompressionSettings = TC_Default;
			TextureFactory->LODGroup = TEXTUREGROUP_World;
		}

		{
			const FString ObjPath = PackageName + TEXT(".") + Filename;
			if (FPackageName::DoesPackageExist(PackageName) || FindObject<UObject>(nullptr, *ObjPath) != nullptr)
			{
				UE_LOG(LogTemp, Log, TEXT("[Tripo] Texture already imported, skipping: %s"), *Filename);
				continue;
			}
		}

		UPackage* Package = CreatePackage(*PackageName);
		Package->FullyLoad();

		bool bOutCanceled = false;
		UObject* ImportedTexture = TextureFactory->ImportObject(
			UTexture2D::StaticClass(),
			Package,
			*Filename,
			RF_Public | RF_Standalone,
			FilePath,
			nullptr,
			bOutCanceled
		);

		if (ImportedTexture)
		{
			if (UTexture2D* Texture = Cast<UTexture2D>(ImportedTexture))
			{
				if (bIsNormalMap)
				{
					Texture->SRGB = false;
					Texture->CompressionSettings = TC_Normalmap;
					Texture->LODGroup = TEXTUREGROUP_WorldNormalMap;
					Texture->UpdateResource();
				}
				else if (bIsLinearGrayscale)
				{
					Texture->SRGB = false;
					Texture->CompressionSettings = TC_Grayscale;
					Texture->UpdateResource();
				}
			}

			FAssetRegistryModule::AssetCreated(ImportedTexture);
			ImportedTexture->MarkPackageDirty();
			ImportCount++;

			UE_LOG(LogTemp, Log, TEXT("[Tripo] Imported texture: %s%s"),
				*Filename,
				bIsNormalMap      ? TEXT(" [Normal]")           :
				bIsLinearGrayscale? TEXT(" [LinearGrayscale]")  : TEXT(""));
		}
	}

	TextureFactory->RemoveFromRoot();
	return ImportCount;
}

// ————— Master Material —————

UMaterial* FTripoModelImporter::EnsureMasterMaterial()
{
	const FString MatPkgPath = FPaths::Combine(GetProjectMaterialRoot(), TEXT("M_Tripo_PBR_Master"));
	const FString PluginMatPkgPath = GetPluginContentRoot() + TEXT("Materials/M_Tripo_PBR_Master");

	if (UMaterial* Existing = LoadAssetFromPackagePath<UMaterial>(MatPkgPath, TEXT("M_Tripo_PBR_Master")))
	{
		UE_LOG(LogTemp, Log, TEXT("[Tripo][PBR] Loaded master material from disk"));
		return Existing;
	}

	UTexture2D* DefBaseColor = LoadOrCreateProjectTexture(
		TEXT("T_Tripo_Default_White"),
		FColor(255, 255, 255, 255), TC_Default, true);

	UTexture2D* DefNormal = LoadOrCreateProjectTexture(
		TEXT("T_Tripo_Default_Normal"),
		FColor(128, 128, 255, 255), TC_Normalmap, false);

	UTexture2D* DefMetallic = LoadOrCreateProjectTexture(
		TEXT("T_Tripo_Default_Black_Linear"),
		FColor(0, 0, 0, 255), TC_Grayscale, false);

	UTexture2D* DefRoughness = LoadOrCreateProjectTexture(
		TEXT("T_Tripo_Default_White_Linear"),
		FColor(255, 255, 255, 255), TC_Grayscale, false);

	if (UMaterial* PluginMaterial = LoadAssetFromPackagePath<UMaterial>(PluginMatPkgPath, TEXT("M_Tripo_PBR_Master")))
	{
		IAssetTools& AssetTools = FModuleManager::LoadModuleChecked<FAssetToolsModule>("AssetTools").Get();
		UMaterial* DuplicatedMaterial = Cast<UMaterial>(AssetTools.DuplicateAsset(
			TEXT("M_Tripo_PBR_Master"),
			GetProjectMaterialRoot(),
			PluginMaterial));

		if (DuplicatedMaterial)
		{
			UpdateMasterMaterialTextureDefaults(DuplicatedMaterial, DefBaseColor, DefNormal, DefMetallic, DefRoughness);
			DuplicatedMaterial->MarkPackageDirty();
			SaveAssetPackage(DuplicatedMaterial->GetOutermost(), DuplicatedMaterial, MatPkgPath);
			UE_LOG(LogTemp, Log, TEXT("[Tripo][PBR] Duplicated master material into project: %s"), *MatPkgPath);
			return DuplicatedMaterial;
		}

		UE_LOG(LogTemp, Warning, TEXT("[Tripo][PBR] Failed to duplicate plugin master material, creating fallback"));
	}

	UE_LOG(LogTemp, Warning, TEXT("[Tripo][PBR] Master material not found at %s, creating fallback"), *MatPkgPath);

	UPackage* Package = CreatePackage(*MatPkgPath);
	if (!Package)
	{
		UE_LOG(LogTemp, Error, TEXT("[Tripo][PBR] Failed to create package for master material"));
		return nullptr;
	}
	Package->FullyLoad();

	UMaterial* NewMat = NewObject<UMaterial>(Package, TEXT("M_Tripo_PBR_Master"),
		RF_Public | RF_Standalone | RF_Transactional);
	if (!NewMat)
	{
		UE_LOG(LogTemp, Error, TEXT("[Tripo][PBR] Failed to create UMaterial object"));
		return nullptr;
	}

	auto MakeTexParam = [&](const TCHAR* ParamName, UTexture2D* DefaultTex,
		EMaterialSamplerType Sampler, int32 EdX, int32 EdY)
		-> UMaterialExpressionTextureSampleParameter2D*
	{
		auto* Node = NewObject<UMaterialExpressionTextureSampleParameter2D>(NewMat);
		Node->ParameterName             = FName(ParamName);
		Node->Texture                   = DefaultTex;
		Node->SamplerType               = Sampler;
		Node->MaterialExpressionEditorX = EdX;
		Node->MaterialExpressionEditorY = EdY;
		NewMat->GetExpressionCollection().AddExpression(Node);
		return Node;
	};

	auto MakeMaskR = [&](UMaterialExpression* Source, int32 EdX, int32 EdY)
		-> UMaterialExpressionComponentMask*
	{
		auto* Mask = NewObject<UMaterialExpressionComponentMask>(NewMat);
		Mask->R = true; Mask->G = false; Mask->B = false; Mask->A = false;
		Mask->MaterialExpressionEditorX = EdX;
		Mask->MaterialExpressionEditorY = EdY;
		Mask->Input.Expression = Source;
		NewMat->GetExpressionCollection().AddExpression(Mask);
		return Mask;
	};

	auto* BaseColorNode = MakeTexParam(TEXT("BaseColorTex"), DefBaseColor,
		SAMPLERTYPE_Color,           -700, -200);
	auto* NormalNode    = MakeTexParam(TEXT("NormalTex"),    DefNormal,
		SAMPLERTYPE_Normal,          -700,    0);
	auto* MetallicNode  = MakeTexParam(TEXT("MetallicTex"),  DefMetallic,
		SAMPLERTYPE_LinearGrayscale, -700,  200);
	auto* RoughnessNode = MakeTexParam(TEXT("RoughnessTex"), DefRoughness,
		SAMPLERTYPE_LinearGrayscale, -700,  400);

	auto* MetallicMask  = MakeMaskR(MetallicNode,  -350, 200);
	auto* RoughnessMask = MakeMaskR(RoughnessNode, -350, 400);

	UMaterialEditorOnlyData* EdData = NewMat->GetEditorOnlyData();
	EdData->BaseColor.Expression = BaseColorNode;
	EdData->Normal.Expression    = NormalNode;
	EdData->Metallic.Expression  = MetallicMask;
	EdData->Roughness.Expression = RoughnessMask;

	NewMat->PreEditChange(nullptr);
	NewMat->PostEditChange();
	FAssetRegistryModule::AssetCreated(NewMat);
	Package->MarkPackageDirty();
	SaveAssetPackage(Package, NewMat, MatPkgPath);

	UE_LOG(LogTemp, Log, TEXT("[Tripo][PBR] Fallback master material saved: %s"), *MatPkgPath);
	return NewMat;
}

// ————— PBR Material Application —————

bool FTripoModelImporter::ApplyPBRMaterialPerSlot(
	UObject* MeshAsset,
	const FString& TexturesPackagePath,
	const FString& BasePackagePath,
	const FString& AssetName)
{
	if (!MeshAsset) return false;

	UStaticMesh*   StaticMesh   = Cast<UStaticMesh>(MeshAsset);
	USkeletalMesh* SkeletalMesh = Cast<USkeletalMesh>(MeshAsset);
	if (!StaticMesh && !SkeletalMesh) return false;

	const int32 NumSlots = StaticMesh
		? StaticMesh->GetStaticMaterials().Num()
		: SkeletalMesh->GetMaterials().Num();

	if (NumSlots == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("[Tripo][PBR] Mesh has no material slots"));
		return false;
	}

	FAssetRegistryModule& AssetRegistry = FModuleManager::LoadModuleChecked<FAssetRegistryModule>("AssetRegistry");
	TArray<FAssetData> TexAssets;
	AssetRegistry.Get().GetAssetsByPath(FName(*TexturesPackagePath), TexAssets, true);

	struct FPBRChannel { FString Suffix; FName ParamName; };
	const TArray<FPBRChannel> Channels = {
		{ TEXT("_basecolor"),  TEXT("BaseColorTex") },
		{ TEXT("_base_color"), TEXT("BaseColorTex") },
		{ TEXT("_albedo"),     TEXT("BaseColorTex") },
		{ TEXT("_normal"),     TEXT("NormalTex")    },
		{ TEXT("_normalmap"),  TEXT("NormalTex")    },
		{ TEXT("_norm"),       TEXT("NormalTex")    },
		{ TEXT("_metallic"),   TEXT("MetallicTex")  },
		{ TEXT("_metalness"),  TEXT("MetallicTex")  },
		{ TEXT("_roughness"),  TEXT("RoughnessTex") },
		{ TEXT("_rough"),      TEXT("RoughnessTex") },
	};

	const FString AssetNamePrefixLower = (AssetName + TEXT("_")).ToLower();

	// partPrefix (lower) → (ParamName → UTexture2D*)
	TMap<FString, TMap<FName, UTexture2D*>> PrefixTexMap;

	for (const FAssetData& AD : TexAssets)
	{
		if (AD.AssetClassPath != UTexture2D::StaticClass()->GetClassPathName()) continue;
		UTexture2D* Tex = Cast<UTexture2D>(AD.GetAsset());
		if (!Tex) continue;

		const FString LowerTexName = Tex->GetName().ToLower();

		if (!LowerTexName.StartsWith(AssetNamePrefixLower)) continue;

		for (const FPBRChannel& Ch : Channels)
		{
			if (LowerTexName.EndsWith(Ch.Suffix))
			{
				const int32 SuffixStart  = LowerTexName.Len() - Ch.Suffix.Len();
				const int32 PrefixEndPos = AssetNamePrefixLower.Len();
				FString PartPrefix = (SuffixStart > PrefixEndPos)
					? LowerTexName.Mid(PrefixEndPos, SuffixStart - PrefixEndPos)
					: TEXT("");
				if (PartPrefix.EndsWith(TEXT("_")))
					PartPrefix = PartPrefix.LeftChop(1);

				if (!PrefixTexMap.Contains(PartPrefix))
					PrefixTexMap.Add(PartPrefix, {});
				if (!PrefixTexMap[PartPrefix].Contains(Ch.ParamName))
					PrefixTexMap[PartPrefix].Add(Ch.ParamName, Tex);
				break;
			}
		}
	}

	// Fallback: fill missing PBR channels using texture compression settings.
	// Handles Interchange-imported textures with numeric suffixes (e.g. _0_0, _0_1)
	// that don't match the name-based suffix table above (UE 5.8+ naming change).
	{
		TSet<UTexture2D*> MatchedTextures;
		for (const auto& PrefixEntry : PrefixTexMap)
			for (const auto& ChEntry : PrefixEntry.Value)
				MatchedTextures.Add(ChEntry.Value);

		TMap<FName, UTexture2D*> InferredChannels;
		for (const FAssetData& AD : TexAssets)
		{
			if (AD.AssetClassPath != UTexture2D::StaticClass()->GetClassPathName()) continue;
			UTexture2D* Tex = Cast<UTexture2D>(AD.GetAsset());
			if (!Tex || MatchedTextures.Contains(Tex)) continue;

			FName ParamName = NAME_None;
			if (Tex->CompressionSettings == TC_Normalmap)
				ParamName = TEXT("NormalTex");
			else if (Tex->CompressionSettings == TC_Default && Tex->SRGB)
				ParamName = TEXT("BaseColorTex");

			if (ParamName != NAME_None && !InferredChannels.Contains(ParamName))
				InferredChannels.Add(ParamName, Tex);
		}

		if (InferredChannels.Num() > 0)
		{
			if (PrefixTexMap.IsEmpty())
			{
				PrefixTexMap.Add(TEXT(""), InferredChannels);
			}
			else
			{
				for (auto& PrefixEntry : PrefixTexMap)
					for (const auto& Inferred : InferredChannels)
						if (!PrefixEntry.Value.Contains(Inferred.Key))
							PrefixEntry.Value.Add(Inferred.Key, Inferred.Value);
			}

			UE_LOG(LogTemp, Log, TEXT("[Tripo][PBR] Inferred %d texture channel(s) from compression settings"), InferredChannels.Num());
		}
	}

	if (PrefixTexMap.IsEmpty())
	{
		UE_LOG(LogTemp, Warning, TEXT("[Tripo][PBR] No renamed PBR textures found under %s"), *TexturesPackagePath);
		return false;
	}

	UMaterial* MasterMat = EnsureMasterMaterial();
	if (!MasterMat)
	{
		UE_LOG(LogTemp, Error, TEXT("[Tripo][PBR] Master material unavailable, aborting"));
		return false;
	}

	TArray<FString> SortedPrefixes;
	PrefixTexMap.GetKeys(SortedPrefixes);
	SortedPrefixes.Sort();

	auto SetSlotMat = [&](int32 Idx, UMaterialInstanceConstant* MatInst)
	{
		if (StaticMesh)
			StaticMesh->SetMaterial(Idx, MatInst);
		else
			SkeletalMesh->GetMaterials()[Idx].MaterialInterface = MatInst;
	};

	struct FSlotAssignment
	{
		int32  SlotIdx;
		FString InstName;
		FString MatchedPrefix;
	};

	int32 MeshPartIdx = 0;
	{
		const FString MeshName = MeshAsset->GetName();
		int32 LastUnderscore = INDEX_NONE;
		MeshName.FindLastChar(TEXT('_'), LastUnderscore);
		if (LastUnderscore != INDEX_NONE)
		{
			const FString IdxStr = MeshName.Mid(LastUnderscore + 1);
			if (IdxStr.IsNumeric())
				MeshPartIdx = FCString::Atoi(*IdxStr);
		}
	}

	TArray<FSlotAssignment> Assignments;

	for (int32 SlotIdx = 0; SlotIdx < NumSlots; SlotIdx++)
	{
		FSlotAssignment Assignment;
		Assignment.SlotIdx = SlotIdx;

		if (SortedPrefixes.Num() == 1)
		{
			Assignment.MatchedPrefix = SortedPrefixes[0];
		}
		else
		{
			const int32 EffectiveIdx = (MeshPartIdx + SlotIdx) % SortedPrefixes.Num();
			Assignment.MatchedPrefix = SortedPrefixes[EffectiveIdx];
		}

		const FString MeshName = MeshAsset->GetName();
		Assignment.InstName = (SlotIdx == 0)
			? FString::Printf(TEXT("%s_Mat"), *MeshName)
			: FString::Printf(TEXT("%s_Mat_%d"), *MeshName, SlotIdx);

		Assignments.Add(Assignment);
	}

	for (const FSlotAssignment& Assignment : Assignments)
	{
		const TMap<FName, UTexture2D*>* TexMap = PrefixTexMap.Find(Assignment.MatchedPrefix);
		if (!TexMap || TexMap->IsEmpty()) continue;

		const FString InstPkgPath = FPaths::Combine(BasePackagePath, Assignment.InstName);
		UPackage* InstPackage = CreatePackage(*InstPkgPath);
		InstPackage->FullyLoad();

		UMaterialInstanceConstant* MatInst = NewObject<UMaterialInstanceConstant>(
			InstPackage, *Assignment.InstName, RF_Public | RF_Standalone | RF_Transactional);

		if (!MatInst)
		{
			UE_LOG(LogTemp, Error, TEXT("[Tripo][PBR] Failed to create material instance for slot %d"), Assignment.SlotIdx);
			continue;
		}

		MatInst->SetParentEditorOnly(MasterMat);

		for (const auto& KV : *TexMap)
		{
			MatInst->SetTextureParameterValueEditorOnly(KV.Key, KV.Value);
		}

		MatInst->PostEditChange();
		FAssetRegistryModule::AssetCreated(MatInst);
		InstPackage->MarkPackageDirty();

		SetSlotMat(Assignment.SlotIdx, MatInst);

		UE_LOG(LogTemp, Log, TEXT("[Tripo][PBR] Slot %d -> %s (prefix='%s', %d textures)"),
			Assignment.SlotIdx, *Assignment.InstName, *Assignment.MatchedPrefix, TexMap->Num());
	}

	if (StaticMesh)  StaticMesh->MarkPackageDirty();
	else             SkeletalMesh->MarkPackageDirty();

	return true;
}

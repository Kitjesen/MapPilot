// Copyright Tripo3D. All Rights Reserved.
// Scene spawning and cleanup utilities extracted from TripoModelImporter.cpp

#include "TripoModelImporter.h"
#include "Engine/StaticMesh.h"
#include "Engine/SkeletalMesh.h"
#include "Components/StaticMeshComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Materials/MaterialInterface.h"
#include "Editor.h"
#include "LevelEditorViewport.h"
#include "HAL/PlatformFileManager.h"
#include "GenericPlatform/GenericPlatformFile.h"
#include "Async/Async.h"

// ————— Scene Spawning —————

bool FTripoModelImporter::SpawnMultiPartInLevel(const TArray<UObject*>& MeshAssets, const FString& ModelName)
{
	if (MeshAssets.IsEmpty()) return false;

	UWorld* World = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr;
	if (!World) return false;

	FActorSpawnParameters SpawnParams;
	SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

	AActor* SpawnedActor = World->SpawnActor<AActor>(
		AActor::StaticClass(), FVector::ZeroVector, FRotator::ZeroRotator, SpawnParams);
	if (!SpawnedActor) return false;

	USceneComponent* RootComp = NewObject<USceneComponent>(SpawnedActor, TEXT("Root"));
	RootComp->RegisterComponent();
	SpawnedActor->SetRootComponent(RootComp);

	int32 SpawnedParts = 0;
	for (UObject* MeshAsset : MeshAssets)
	{
		if (!MeshAsset) continue;

		if (UStaticMesh* SM = Cast<UStaticMesh>(MeshAsset))
		{
			UStaticMeshComponent* Comp = NewObject<UStaticMeshComponent>(SpawnedActor, FName(*SM->GetName()));
			Comp->SetStaticMesh(SM);
			Comp->AttachToComponent(RootComp, FAttachmentTransformRules::KeepRelativeTransform);
			Comp->RegisterComponent();
			++SpawnedParts;
		}
		else if (USkeletalMesh* SK = Cast<USkeletalMesh>(MeshAsset))
		{
			USkeletalMeshComponent* Comp = NewObject<USkeletalMeshComponent>(SpawnedActor, FName(*SK->GetName()));
			Comp->SetSkeletalMesh(SK);
			Comp->AttachToComponent(RootComp, FAttachmentTransformRules::KeepRelativeTransform);
			Comp->RegisterComponent();
			++SpawnedParts;
		}
	}

	SpawnedActor->SetActorLabel(ModelName);
	GEditor->SelectNone(false, true);
	GEditor->SelectActor(SpawnedActor, true, true);

	UE_LOG(LogTemp, Log, TEXT("[Tripo] Spawned multi-part actor '%s' with %d part(s)"), *ModelName, SpawnedParts);
	return SpawnedParts > 0;
}

bool FTripoModelImporter::SpawnInLevel(UObject* Asset, const FString& ModelName)
{
	if (!Asset)
	{
		return false;
	}

	UWorld* World = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr;
	if (!World)
	{
		UE_LOG(LogTemp, Error, TEXT("No valid world found for spawning actor"));
		return false;
	}

	FActorSpawnParameters SpawnParams;
	SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

	AActor* SpawnedActor = nullptr;

	if (UStaticMesh* StaticMesh = Cast<UStaticMesh>(Asset))
	{
		SpawnedActor = World->SpawnActor<AActor>(AActor::StaticClass(), FVector::ZeroVector, FRotator::ZeroRotator, SpawnParams);
		if (SpawnedActor)
		{
			UStaticMeshComponent* MeshComponent = NewObject<UStaticMeshComponent>(SpawnedActor);
			MeshComponent->SetStaticMesh(StaticMesh);
			MeshComponent->RegisterComponent();
			SpawnedActor->SetRootComponent(MeshComponent);

			if (MeshComponent && MeshComponent->GetStaticMesh())
			{
				int32 NumMaterials = MeshComponent->GetNumMaterials();
				UE_LOG(LogTemp, Log, TEXT("[Tripo] Spawned static mesh has %d material slot(s)"), NumMaterials);

				for (int32 i = 0; i < NumMaterials; i++)
				{
					UMaterialInterface* Mat = MeshComponent->GetMaterial(i);
					if (!Mat)
					{
						UE_LOG(LogTemp, Warning, TEXT("[Tripo] Material slot %d is null"), i);
					}
					else
					{
						UE_LOG(LogTemp, Log, TEXT("[Tripo] Material slot %d: %s"), i, *Mat->GetName());
					}
				}
			}
		}
	}
	else if (USkeletalMesh* SkeletalMesh = Cast<USkeletalMesh>(Asset))
	{
		SpawnedActor = World->SpawnActor<AActor>(AActor::StaticClass(), FVector::ZeroVector, FRotator::ZeroRotator, SpawnParams);
		if (SpawnedActor)
		{
			USkeletalMeshComponent* MeshComponent = NewObject<USkeletalMeshComponent>(SpawnedActor);
			MeshComponent->SetSkeletalMesh(SkeletalMesh);
			MeshComponent->RegisterComponent();
			SpawnedActor->SetRootComponent(MeshComponent);
		}
	}

	if (SpawnedActor)
	{
		SpawnedActor->SetActorLabel(ModelName);

		GEditor->SelectNone(false, true);
		GEditor->SelectActor(SpawnedActor, true, true);

		UE_LOG(LogTemp, Log, TEXT("Spawned actor: %s"), *SpawnedActor->GetActorLabel());
		return true;
	}

	return false;
}

// ————— Cleanup Utility —————

void FTripoModelImporter::DeleteDirectoryWithRetry(const FString& DirectoryPath, int32 MaxRetries, float RetryDelay)
{
	AsyncTask(ENamedThreads::AnyBackgroundThreadNormalTask, [DirectoryPath, MaxRetries, RetryDelay]()
	{
		IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();

		for (int32 i = 0; i < MaxRetries; ++i)
		{
			if (PlatformFile.DeleteDirectoryRecursively(*DirectoryPath))
			{
				UE_LOG(LogTemp, Log, TEXT("Deleted temp directory: %s"), *DirectoryPath);
				return;
			}

			if (i < MaxRetries - 1)
			{
				UE_LOG(LogTemp, Warning, TEXT("Failed to delete directory (attempt %d/%d), retrying..."),
					i + 1, MaxRetries);
				FPlatformProcess::Sleep(RetryDelay);
			}
		}

		UE_LOG(LogTemp, Error, TEXT("Failed to delete directory after %d attempts: %s"),
			MaxRetries, *DirectoryPath);
	});
}

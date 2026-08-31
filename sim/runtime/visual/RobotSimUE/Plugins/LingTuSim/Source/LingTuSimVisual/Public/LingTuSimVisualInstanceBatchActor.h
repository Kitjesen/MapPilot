#pragma once

#include "GameFramework/Actor.h"

#include "LingTuSimVisualInstanceBatchActor.generated.h"

class UHierarchicalInstancedStaticMeshComponent;
class UMaterialInterface;
class USceneComponent;
class UStaticMesh;

/** Serializable identity and rendering metadata for one visual-only HISM group. */
USTRUCT(BlueprintType)
struct LINGTUSIMVISUAL_API FLingTuSimVisualInstanceGroup
{
    GENERATED_BODY()

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="LingTuSim|FactoryPark")
    FString GroupId;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="LingTuSim|FactoryPark")
    TObjectPtr<UStaticMesh> Mesh;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="LingTuSim|FactoryPark")
    TObjectPtr<UMaterialInterface> Material;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="LingTuSim|FactoryPark")
    TArray<FString> StableIds;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="LingTuSim|FactoryPark")
    TArray<FTransform> InstanceTransforms;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="LingTuSim|FactoryPark")
    bool bCastShadow = false;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="LingTuSim|FactoryPark")
    TObjectPtr<UHierarchicalInstancedStaticMeshComponent> InstanceComponent;
};

/** Editor-callable, serializable visual-only batching actor for FactoryPark assets. */
UCLASS()
class LINGTUSIMVISUAL_API ALingTuSimVisualInstanceBatchActor final : public AActor
{
    GENERATED_BODY()

public:
    ALingTuSimVisualInstanceBatchActor();

    /** Adds one validated HISM group atomically. Transforms are relative to IdentityRoot. */
    UFUNCTION(BlueprintCallable, CallInEditor, Category="LingTuSim|FactoryPark")
    bool AddVisualGroup(const FString& GroupId, UStaticMesh* Mesh, UMaterialInterface* Material,
                        const TArray<FTransform>& Transforms, const TArray<FString>& StableIds,
                        bool bCastShadow);

    /** Strictly compare serialized identity and live HISM state after save/reload. */
    UFUNCTION(BlueprintCallable, Category="LingTuSim|FactoryPark")
    bool ValidateVisualGroup(const FString& GroupId, UStaticMesh* ExpectedMesh,
                             UMaterialInterface* ExpectedMaterial,
                             const TArray<FTransform>& ExpectedTransforms,
                             const TArray<FString>& ExpectedStableIds,
                             bool bExpectedCastShadow) const;

    /** Removes every visual group and its owned HISM component. */
    UFUNCTION(BlueprintCallable, CallInEditor, Category="LingTuSim|FactoryPark")
    void ResetGroups();

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="LingTuSim|FactoryPark")
    TObjectPtr<USceneComponent> IdentityRoot;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="LingTuSim|FactoryPark")
    TArray<FLingTuSimVisualInstanceGroup> VisualGroups;
};

#pragma once

#include "Components/SceneComponent.h"

#include "LingTuSimBodyBindingComponent.generated.h"

UCLASS(ClassGroup=(LingTuSim), meta=(BlueprintSpawnableComponent))
class LINGTUSIMVISUAL_API ULingTuSimBodyBindingComponent : public USceneComponent
{
    GENERATED_BODY()

public:
    ULingTuSimBodyBindingComponent();

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category="LingTuSim|Visual")
    FString StableId;

protected:
    virtual void OnRegister() override;
    virtual void OnUnregister() override;

private:
    FString RegisteredStableId;
};

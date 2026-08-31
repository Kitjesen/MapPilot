#pragma once

#include "GameFramework/Actor.h"

#include "LingTuSimBodyActor.generated.h"

class ULingTuSimBodyBindingComponent;

/** Runtime-neutral visual root for one stable physics body. */
UCLASS()
class LINGTUSIMVISUAL_API ALingTuSimBodyActor : public AActor
{
    GENERATED_BODY()

public:
    ALingTuSimBodyActor();

    /** Configures the stable body identity and refreshes subsystem registration. */
    UFUNCTION(BlueprintCallable, CallInEditor, Category="LingTuSim|Visual")
    bool SetBodyStableId(const FString& StableId);

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="LingTuSim|Visual")
    TObjectPtr<ULingTuSimBodyBindingComponent> BodyBinding;
};

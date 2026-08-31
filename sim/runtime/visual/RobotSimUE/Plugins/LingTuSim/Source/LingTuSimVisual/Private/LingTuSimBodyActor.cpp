#include "LingTuSimBodyActor.h"

#include "LingTuSimBodyBindingComponent.h"

ALingTuSimBodyActor::ALingTuSimBodyActor()
{
    PrimaryActorTick.bCanEverTick = false;
    SetActorEnableCollision(false);
    BodyBinding = CreateDefaultSubobject<ULingTuSimBodyBindingComponent>(
        TEXT("BodyBinding"));
    RootComponent = BodyBinding;
}

bool ALingTuSimBodyActor::SetBodyStableId(const FString& StableId)
{
    if (StableId.IsEmpty() || BodyBinding == nullptr)
    {
        return false;
    }

    const bool bWasRegistered = BodyBinding->IsRegistered();
    if (bWasRegistered)
    {
        BodyBinding->UnregisterComponent();
    }
    BodyBinding->StableId = StableId;
    if (bWasRegistered)
    {
        BodyBinding->RegisterComponent();
    }
    return true;
}

#include "LingTuSimBodyBindingComponent.h"

#include "Engine/World.h"
#include "LingTuSimVisualWorldSubsystem.h"

ULingTuSimBodyBindingComponent::ULingTuSimBodyBindingComponent()
{
    PrimaryComponentTick.bCanEverTick = false;
    SetMobility(EComponentMobility::Movable);
}

void ULingTuSimBodyBindingComponent::OnRegister()
{
    Super::OnRegister();
    RegisteredStableId.Reset();

    if (UWorld* World = GetWorld())
    {
        if (ULingTuSimVisualWorldSubsystem* Subsystem =
                World->GetSubsystem<ULingTuSimVisualWorldSubsystem>())
        {
            if (Subsystem->RegisterBinding(this))
            {
                RegisteredStableId = StableId;
            }
        }
    }
}

void ULingTuSimBodyBindingComponent::OnUnregister()
{
    if (!RegisteredStableId.IsEmpty())
    {
        if (UWorld* World = GetWorld())
        {
            if (ULingTuSimVisualWorldSubsystem* Subsystem =
                    World->GetSubsystem<ULingTuSimVisualWorldSubsystem>())
            {
                Subsystem->UnregisterBinding(RegisteredStableId, this);
            }
        }
        RegisteredStableId.Reset();
    }

    Super::OnUnregister();
}

#include "LingTuSimPresentationPolicy.h"

#include "Components/PrimitiveComponent.h"
#include "Engine/CollisionProfile.h"

namespace LingTuSim::Visual
{
    void ApplyPresentationPolicy(UPrimitiveComponent& Component)
    {
        Component.SetSimulatePhysics(false);
        Component.SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);
        Component.SetCollisionEnabled(ECollisionEnabled::NoCollision);
        Component.SetGenerateOverlapEvents(false);
        Component.SetCanEverAffectNavigation(false);
    }

    bool HasPresentationPolicy(const UPrimitiveComponent& Component)
    {
        return !Component.IsSimulatingPhysics()
            && Component.GetCollisionEnabled() == ECollisionEnabled::NoCollision
            && Component.GetCollisionProfileName() == UCollisionProfile::NoCollision_ProfileName
            && !Component.GetGenerateOverlapEvents()
            && !Component.CanEverAffectNavigation();
    }
}

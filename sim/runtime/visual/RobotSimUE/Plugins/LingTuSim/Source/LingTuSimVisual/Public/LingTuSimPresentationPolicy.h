#pragma once

class UPrimitiveComponent;

namespace LingTuSim::Visual
{
    /** Keep one Unreal primitive strictly presentation-only; MuJoCo owns physics. */
    LINGTUSIMVISUAL_API void ApplyPresentationPolicy(UPrimitiveComponent& Component);

    /** Read back the complete presentation-only collision contract. */
    LINGTUSIMVISUAL_API bool HasPresentationPolicy(const UPrimitiveComponent& Component);
}

#pragma once

#include <chrono>

namespace lingtu::nav::endpoint {

using SteadyClock = std::chrono::steady_clock;

// Collision clearance is expanded once by LocalPlanner footprint checks.
inline constexpr double kLayerInflationM = 0.0;
inline constexpr double kMapOdomJumpTranslationM = 0.50;
inline constexpr double kMapOdomJumpYawRad = 0.25;
inline constexpr double kSourceClockRebaseThresholdS = 0.35;

inline double elapsedMs(SteadyClock::time_point start,
                        SteadyClock::time_point end = SteadyClock::now()) {
  return std::chrono::duration<double, std::milli>(end - start).count();
}

inline double nowSeconds() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

inline double steadySeconds() {
  const auto now = SteadyClock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

}  // namespace lingtu::nav::endpoint

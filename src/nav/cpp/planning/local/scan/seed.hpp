// Adapted from SCAN-Planner polynomial initialization at upstream commit 348e8a5.
// Modified for LingTu: ROS/Eigen-free state and deterministic retry sampling.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <vector>

#include "planning/local/planner.hpp"
#include "trajectory/spline.hpp"

namespace nav_kernel::local::scan {

enum class SeedMode {
  Previous,
  Guide,
  Polynomial,
  RandomPolynomial,
};

struct SeedHistory {
  const UniformSpline *trajectory{nullptr};
  double startedS{-1.0};
};

struct SeedResult {
  std::vector<Vec3> samples;
  std::array<Vec3, 4> boundary{};
  SeedMode mode{SeedMode::Polynomial};
  double interval{0.0};

  [[nodiscard]] bool valid() const noexcept {
    return samples.size() >= 7U && interval > 0.0;
  }
};

const char *seedModeName(SeedMode mode) noexcept;

SeedResult buildSeed(const std::vector<Vec3> &guide, const LocalPlanRequest &input,
                     const LocalPlannerParams &params, SeedMode mode,
                     const SeedHistory &history = {}, int failures = 0);

}  // namespace nav_kernel::local::scan

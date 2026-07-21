#pragma once

#include <algorithm>
#include <cstddef>
#include <vector>

namespace lingtu::nav::endpoint::detail {

template <typename DistanceSquared>
std::vector<std::size_t> sampleDebugIndices(
    const std::vector<std::size_t>& candidates,
    std::size_t limit,
    bool prioritize_origin,
    DistanceSquared distance_squared) {
  if (limit == 0 || candidates.empty()) {
    return {};
  }
  if (candidates.size() <= limit) {
    return candidates;
  }
  std::vector<std::size_t> sampled = candidates;
  if (prioritize_origin) {
    const auto compare = [&](std::size_t lhs, std::size_t rhs) {
      const double lhs_distance = distance_squared(lhs);
      const double rhs_distance = distance_squared(rhs);
      if (lhs_distance != rhs_distance) {
        return lhs_distance < rhs_distance;
      }
      return lhs < rhs;
    };
    std::partial_sort(
        sampled.begin(),
        sampled.begin() + static_cast<std::ptrdiff_t>(limit),
        sampled.end(),
        compare);
    sampled.resize(limit);
    return sampled;
  }
  std::vector<std::size_t> uniform;
  uniform.reserve(limit);
  for (std::size_t index = 0; index < limit; ++index) {
    const std::size_t source = limit <= 1
        ? 0
        : index * (candidates.size() - 1) / (limit - 1);
    uniform.push_back(candidates[source]);
  }
  return uniform;
}

}  // namespace lingtu::nav::endpoint::detail

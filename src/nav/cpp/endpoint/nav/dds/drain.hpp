#pragma once

#include <cstddef>
#include <utility>

namespace lingtu::nav::endpoint {

inline constexpr std::size_t kDdsReaderBatchSize = 16;

enum class DdsDrainProfile {
  kDefault,
  kTransform,
};

struct DdsDrainBudget {
  std::size_t batch_size{kDdsReaderBatchSize};
  std::size_t max_batches{1};

  [[nodiscard]] constexpr std::size_t max_samples() const noexcept {
    return batch_size * max_batches;
  }
};

[[nodiscard]] constexpr DdsDrainBudget drainBudget(DdsDrainProfile profile) noexcept {
  return profile == DdsDrainProfile::kTransform ? DdsDrainBudget{kDdsReaderBatchSize, 8}
                                                : DdsDrainBudget{kDdsReaderBatchSize, 1};
}

template <typename TakeBatch>
std::size_t drainBatches(const DdsDrainBudget &budget, TakeBatch &&take_batch) {
  if (budget.batch_size == 0 || budget.max_batches == 0) {
    return 0;
  }

  std::size_t drained = 0;
  for (std::size_t batch = 0; batch < budget.max_batches; ++batch) {
    const std::ptrdiff_t count = take_batch(budget.batch_size);
    if (count <= 0) {
      break;
    }
    const auto sample_count = static_cast<std::size_t>(count);
    drained += sample_count;
    if (sample_count < budget.batch_size) {
      break;
    }
  }
  return drained;
}

}  // namespace lingtu::nav::endpoint

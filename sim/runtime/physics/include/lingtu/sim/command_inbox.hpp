#pragma once

#include <array>
#include <cstddef>
#include <mutex>
#include <type_traits>

#include "lingtu/sim/runtime_contracts.hpp"

namespace lingtu::sim {

enum class CommandPushResult {
  accepted,
  rejected_stale_model_generation,
  rejected_stale_reset_generation,
  rejected_future_model_generation,
  rejected_future_reset_generation,
  rejected_queue_full,
};

// Bounded mutex-protected MPMC FIFO for commands targeted at one active
// model/reset generation. push(), pop(), set_generation(), clear(), and the
// accessors are linearizable and cannot race with one another. The mutex can
// block, so this queue is not a hard-real-time primitive and must not be used
// from a 500 Hz physics critical section. A dedicated SPSC/MPMC non-blocking
// command transport should be selected at that boundary.
template <std::size_t Capacity = 64>
class CommandInbox final {
 public:
  static_assert(Capacity > 0, "CommandInbox capacity must be positive");
  static_assert(std::is_nothrow_copy_assignable_v<CommandEnvelope>,
                "command copies must not allocate or throw");
  static_assert(std::is_trivially_copyable_v<CommandEnvelope>,
                "command envelope must remain a fixed-capacity in-process DTO");

  explicit CommandInbox(GenerationStamp generation = {}) noexcept : generation_(generation) {}
  CommandInbox(const CommandInbox &) = delete;
  CommandInbox &operator=(const CommandInbox &) = delete;

  CommandPushResult push(const CommandEnvelope &command) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    const CommandPushResult generation_result = validate_generation(command.generation);
    if (generation_result != CommandPushResult::accepted) {
      return generation_result;
    }
    if (size_ == Capacity) {
      return CommandPushResult::rejected_queue_full;
    }
    queue_[head_] = command;
    head_ = increment(head_);
    ++size_;
    return CommandPushResult::accepted;
  }

  bool pop(CommandEnvelope &out) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    if (size_ == 0) {
      return false;
    }
    out = queue_[tail_];
    tail_ = increment(tail_);
    --size_;
    return true;
  }

  void set_generation(GenerationStamp generation) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    generation_ = generation;
    head_ = 0;
    tail_ = 0;
    size_ = 0;
  }

  void clear() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    head_ = 0;
    tail_ = 0;
    size_ = 0;
  }

  [[nodiscard]] GenerationStamp generation() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return generation_;
  }
  [[nodiscard]] std::size_t size() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return size_;
  }
  [[nodiscard]] constexpr std::size_t capacity() const noexcept { return Capacity; }

 private:
  [[nodiscard]] CommandPushResult validate_generation(GenerationStamp command) const noexcept {
    if (command.model_generation < generation_.model_generation) {
      return CommandPushResult::rejected_stale_model_generation;
    }
    if (command.model_generation > generation_.model_generation) {
      return CommandPushResult::rejected_future_model_generation;
    }
    if (command.reset_generation < generation_.reset_generation) {
      return CommandPushResult::rejected_stale_reset_generation;
    }
    if (command.reset_generation > generation_.reset_generation) {
      return CommandPushResult::rejected_future_reset_generation;
    }
    return CommandPushResult::accepted;
  }

  [[nodiscard]] static constexpr std::size_t increment(std::size_t index) noexcept {
    return index + 1 == Capacity ? 0 : index + 1;
  }

  mutable std::mutex mutex_;
  std::array<CommandEnvelope, Capacity> queue_{};
  GenerationStamp generation_{};
  std::size_t head_{0};
  std::size_t tail_{0};
  std::size_t size_{0};
};

}  // namespace lingtu::sim

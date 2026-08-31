#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <utility>

namespace lingtu::driver {

class OutputAckEvidence final {
 public:
  OutputAckEvidence() = default;

  [[nodiscard]] bool accepted() const noexcept { return accepted_; }

  [[nodiscard]] const std::string& producerBootId() const noexcept {
    return producer_boot_id_;
  }

  [[nodiscard]] std::uint64_t outputSequence() const noexcept {
    return output_sequence_;
  }

 private:
  friend class OutputAckState;

  OutputAckEvidence(
      bool accepted,
      std::string producer_boot_id,
      std::uint64_t output_sequence)
      : accepted_(accepted),
        producer_boot_id_(std::move(producer_boot_id)),
        output_sequence_(output_sequence) {}

  bool accepted_{false};
  std::string producer_boot_id_;
  std::uint64_t output_sequence_{0};
};

class OutputAckState final {
 public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  inline static constexpr std::chrono::milliseconds
      kRejectedEvidenceLifetime{250};

  void record(
      std::string producer_boot_id,
      std::uint64_t output_sequence,
      bool accepted,
      TimePoint now) {
    invalidate();
    if (producer_boot_id.empty() || output_sequence == 0) {
      return;
    }
    accepted_ = accepted;
    producer_boot_id_ = std::move(producer_boot_id);
    output_sequence_ = output_sequence;
    rejected_evidence_expires_at_ =
        accepted ? TimePoint{} : now + kRejectedEvidenceLifetime;
    valid_ = true;
  }

  void invalidate() noexcept {
    valid_ = false;
    accepted_ = false;
    producer_boot_id_.clear();
    output_sequence_ = 0;
    rejected_evidence_expires_at_ = {};
  }

  void expire(TimePoint now) noexcept {
    if (valid_ && !accepted_ && now >= rejected_evidence_expires_at_) {
      invalidate();
    }
  }

  [[nodiscard]] OutputAckEvidence current(TimePoint now) const {
    if (!valid_ || producer_boot_id_.empty() || output_sequence_ == 0 ||
        (!accepted_ && now >= rejected_evidence_expires_at_)) {
      return {};
    }
    return {accepted_, producer_boot_id_, output_sequence_};
  }

 private:
  bool valid_{false};
  bool accepted_{false};
  std::string producer_boot_id_;
  std::uint64_t output_sequence_{0};
  TimePoint rejected_evidence_expires_at_{};
};

}  // namespace lingtu::driver

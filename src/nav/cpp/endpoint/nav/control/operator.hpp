#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>

#include "nav_kernel/types.hpp"

namespace lingtu::nav::endpoint {

struct OperatorMotionAuthorityOptions {
  double max_sample_age_s{0.35};
  double future_tolerance_s{0.05};
  double max_lease_ttl_s{2.0};
  std::size_t max_source_id_bytes{128};
  std::size_t max_retained_source_identities{1024};
};

struct OperatorMotionClaim {
  std::string source_id;
  std::uint64_t epoch{0};
  std::uint64_t sequence{0};
  double now_monotonic_s{0.0};
  double lease_ttl_s{0.0};
};

struct OperatorMotionSample {
  std::string source_id;
  std::uint64_t epoch{0};
  std::uint64_t sequence{0};
  double source_stamp_wall_s{0.0};
  double deadline_wall_s{0.0};
  bool deadman{false};
  bool manual_mode{false};
  nav_kernel::Twist command{};
};

struct OperatorMotionRelease {
  std::string source_id;
  std::uint64_t epoch{0};
  std::uint64_t sequence{0};
};

struct OperatorMotionReceipt {
  bool accepted{false};
  std::string reason{"none"};
  bool authority_changed{false};
  bool sample_changed{false};
};

struct OperatorMotionSnapshot {
  bool has_active_authority{false};
  std::string active_source_id;
  std::uint64_t active_epoch{0};
  double lease_until_monotonic_s{0.0};
  bool has_active_sample{false};
  bool holding{false};
  bool zero_required{false};
  bool zero_barrier_pending{false};
  std::uint64_t last_sequence{0};
  std::uint64_t last_sample_sequence{0};
  bool active_sample_manual_mode{false};
  nav_kernel::Twist active_sample{};
  std::uint64_t accepted{0};
  std::uint64_t rejected{0};
  std::string last_reason{"none"};
};

// Owns the lease and zero-barrier protocol for external operator sources.
class OperatorMotionAuthority {
 public:
  explicit OperatorMotionAuthority(OperatorMotionAuthorityOptions options = {});

  OperatorMotionReceipt claim(const OperatorMotionClaim &claim);
  OperatorMotionReceipt submit(const OperatorMotionSample &sample, double receive_monotonic_s,
                               double receive_wall_s);
  OperatorMotionReceipt release(const OperatorMotionRelease &release);
  OperatorMotionReceipt hold(const OperatorMotionRelease &hold);
  OperatorMotionReceipt tick(double now_monotonic_s);
  OperatorMotionReceipt completeZeroBarrier(bool zero_published);

  OperatorMotionSnapshot snapshot() const;

 private:
  struct SourceEpochHighWater {
    std::uint64_t epoch{0};
    std::uint64_t retained_order{0};
  };

  enum class ZeroBarrierDisposition {
    None,
    Hold,
    Release,
    Expiry,
    EpochTransition,
  };

  OperatorMotionReceipt accept(std::string reason, bool authority_changed, bool sample_changed);
  OperatorMotionReceipt reject(std::string reason);
  bool commandIsFinite(const nav_kernel::Twist &command) const;
  bool hasLiveAuthority(double now_monotonic_s) const;
  void enterZeroBarrier(ZeroBarrierDisposition disposition);
  OperatorMotionReceipt requireZeroBarrier(std::string reason, ZeroBarrierDisposition disposition);
  void refreshLease(double receive_monotonic_s);
  std::uint64_t sourceHighWater(const std::string &source_id) const;
  bool recordSourceHighWater(const std::string &source_id, std::uint64_t epoch);
  bool evictOldestInactiveSourceIdentity();
  void clearAuthority();
  void clearSample();
  OperatorMotionReceipt validateActiveControlSequence(const OperatorMotionRelease &control);

  OperatorMotionAuthorityOptions options_;
  OperatorMotionSnapshot snapshot_;
  double active_lease_ttl_s_{0.0};
  ZeroBarrierDisposition zero_barrier_disposition_{ZeroBarrierDisposition::None};
  std::unordered_map<std::string, SourceEpochHighWater> source_epoch_high_water_;
  std::uint64_t source_identity_retained_order_{0};
};

[[nodiscard]] std::uint64_t updateOperatorMotionOutputEvidence(
    const OperatorMotionReceipt &completion, const OperatorMotionSnapshot &snapshot,
    std::uint64_t zero_output_sequence, const nav_kernel::Twist &final_command,
    std::uint64_t &admitted_sequence, std::uint64_t &final_output_sequence) noexcept;

}  // namespace lingtu::nav::endpoint

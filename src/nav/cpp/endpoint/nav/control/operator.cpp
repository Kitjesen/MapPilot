#include "control/operator.hpp"

#include <cmath>
#include <utility>

namespace lingtu::nav::endpoint {

OperatorMotionAuthority::OperatorMotionAuthority(OperatorMotionAuthorityOptions options)
    : options_(options) {}

OperatorMotionReceipt OperatorMotionAuthority::claim(const OperatorMotionClaim &claim) {
  if (claim.source_id.empty()) {
    return reject("source_id_empty");
  }
  if (claim.source_id.size() > options_.max_source_id_bytes) {
    return reject("source_id_too_long");
  }
  if (claim.epoch == 0) {
    return reject("epoch_zero");
  }
  if (claim.sequence == 0) {
    return reject("sequence_zero");
  }
  if (!(claim.now_monotonic_s > 0.0)) {
    return reject("claim_time_invalid");
  }
  if (!(claim.lease_ttl_s > 0.0)) {
    return reject("lease_ttl_invalid");
  }
  if (claim.lease_ttl_s > options_.max_lease_ttl_s) {
    return reject("lease_ttl_too_long");
  }

  if (snapshot_.zero_barrier_pending) {
    return reject("zero_barrier_pending");
  }
  if (snapshot_.has_active_authority && !hasLiveAuthority(claim.now_monotonic_s)) {
    enterZeroBarrier(ZeroBarrierDisposition::Expiry);
    return reject("authority_lease_expired");
  }
  if (!snapshot_.has_active_authority && claim.epoch <= sourceHighWater(claim.source_id)) {
    return reject("stale_epoch");
  }
  if (hasLiveAuthority(claim.now_monotonic_s)) {
    if (snapshot_.active_source_id != claim.source_id) {
      return reject("authority_busy");
    }
    if (claim.epoch < snapshot_.active_epoch) {
      return reject("stale_epoch");
    }
    if (claim.epoch > snapshot_.active_epoch) {
      enterZeroBarrier(ZeroBarrierDisposition::EpochTransition);
      return reject("authority_change_requires_zero_barrier");
    }
    if (claim.sequence <= snapshot_.last_sequence) {
      return reject("non_monotonic_sequence");
    }
  }

  if (!recordSourceHighWater(claim.source_id, claim.epoch)) {
    return reject("source_identity_capacity_exhausted");
  }
  const bool authority_changed = !snapshot_.has_active_authority ||
                                 snapshot_.active_source_id != claim.source_id ||
                                 snapshot_.active_epoch != claim.epoch;
  if (authority_changed) {
    clearSample();
    snapshot_.last_sequence = 0;
    snapshot_.last_sample_sequence = 0;
  }
  snapshot_.has_active_authority = true;
  snapshot_.active_source_id = claim.source_id;
  snapshot_.active_epoch = claim.epoch;
  active_lease_ttl_s_ = claim.lease_ttl_s;
  snapshot_.lease_until_monotonic_s = claim.now_monotonic_s + claim.lease_ttl_s;
  snapshot_.last_sequence = claim.sequence;
  return accept("claimed", authority_changed, false);
}

OperatorMotionReceipt OperatorMotionAuthority::submit(const OperatorMotionSample &sample,
                                                      double receive_monotonic_s,
                                                      double receive_wall_s) {
  if (sample.source_id.size() > options_.max_source_id_bytes) {
    return reject("source_id_too_long");
  }
  if (!snapshot_.has_active_authority) {
    return reject("no_active_authority");
  }
  if (snapshot_.zero_barrier_pending) {
    return reject("zero_barrier_pending");
  }
  if (!hasLiveAuthority(receive_monotonic_s)) {
    return requireZeroBarrier("authority_lease_expired", ZeroBarrierDisposition::Expiry);
  }
  if (sample.source_id != snapshot_.active_source_id) {
    return reject("not_active_source");
  }
  if (sample.epoch != snapshot_.active_epoch) {
    return reject("not_active_epoch");
  }
  if (sample.sequence <= snapshot_.last_sequence) {
    return reject("non_monotonic_sequence");
  }
  if (!(sample.source_stamp_wall_s > 0.0)) {
    return reject("sample_stamp_invalid");
  }
  if (sample.source_stamp_wall_s > receive_wall_s + options_.future_tolerance_s) {
    return reject("sample_stamp_future");
  }
  if (receive_wall_s - sample.source_stamp_wall_s > options_.max_sample_age_s) {
    return reject("sample_stamp_stale");
  }
  if (sample.deadline_wall_s > 0.0 && sample.deadline_wall_s < receive_wall_s) {
    return reject("sample_deadline_expired");
  }
  if (!commandIsFinite(sample.command)) {
    return reject("command_non_finite");
  }

  snapshot_.last_sequence = sample.sequence;
  snapshot_.last_sample_sequence = sample.sequence;
  refreshLease(receive_monotonic_s);
  if (!sample.deadman) {
    return requireZeroBarrier("deadman_released_hold", ZeroBarrierDisposition::Hold);
  }

  snapshot_.has_active_sample = true;
  snapshot_.holding = false;
  snapshot_.active_sample_manual_mode = sample.manual_mode;
  snapshot_.active_sample = sample.command;
  return accept("sample_accepted", false, true);
}

OperatorMotionReceipt OperatorMotionAuthority::release(const OperatorMotionRelease &release) {
  const auto validation = validateActiveControlSequence(release);
  if (!validation.accepted) {
    return validation;
  }
  snapshot_.last_sequence = release.sequence;
  return requireZeroBarrier("released_zero_required", ZeroBarrierDisposition::Release);
}

OperatorMotionReceipt OperatorMotionAuthority::hold(const OperatorMotionRelease &hold) {
  const auto validation = validateActiveControlSequence(hold);
  if (!validation.accepted) {
    return validation;
  }
  snapshot_.last_sequence = hold.sequence;
  return requireZeroBarrier("hold_zero_required", ZeroBarrierDisposition::Hold);
}

OperatorMotionReceipt OperatorMotionAuthority::tick(double now_monotonic_s) {
  if (!snapshot_.has_active_authority) {
    return {true, "idle", false, false};
  }
  if (!hasLiveAuthority(now_monotonic_s)) {
    return requireZeroBarrier("authority_lease_expired", ZeroBarrierDisposition::Expiry);
  }
  return {true, "authority_live", false, false};
}

OperatorMotionReceipt OperatorMotionAuthority::completeZeroBarrier(bool zero_published) {
  if (!snapshot_.zero_barrier_pending) {
    return reject("zero_barrier_not_pending");
  }
  if (!zero_published) {
    snapshot_.zero_required = true;
    snapshot_.zero_barrier_pending = true;
    snapshot_.holding = true;
    return reject("zero_barrier_failed");
  }
  if (zero_barrier_disposition_ == ZeroBarrierDisposition::Hold) {
    snapshot_.has_active_sample = false;
    snapshot_.active_sample_manual_mode = false;
    snapshot_.active_sample = {};
    snapshot_.holding = true;
    snapshot_.zero_required = false;
    snapshot_.zero_barrier_pending = false;
    zero_barrier_disposition_ = ZeroBarrierDisposition::None;
    return accept("zero_barrier_complete", false, true);
  }
  if (!recordSourceHighWater(snapshot_.active_source_id, snapshot_.active_epoch)) {
    return reject("source_identity_capacity_exhausted");
  }
  clearAuthority();
  return accept("zero_barrier_complete", true, true);
}

OperatorMotionSnapshot OperatorMotionAuthority::snapshot() const { return snapshot_; }

OperatorMotionReceipt OperatorMotionAuthority::accept(std::string reason, bool authority_changed,
                                                      bool sample_changed) {
  ++snapshot_.accepted;
  snapshot_.last_reason = reason;
  return {true, std::move(reason), authority_changed, sample_changed};
}

OperatorMotionReceipt OperatorMotionAuthority::reject(std::string reason) {
  ++snapshot_.rejected;
  snapshot_.last_reason = reason;
  return {false, std::move(reason), false, false};
}

bool OperatorMotionAuthority::commandIsFinite(const nav_kernel::Twist &command) const {
  return std::isfinite(command.vx) && std::isfinite(command.vy) && std::isfinite(command.wz);
}

bool OperatorMotionAuthority::hasLiveAuthority(double now_monotonic_s) const {
  return snapshot_.has_active_authority && now_monotonic_s <= snapshot_.lease_until_monotonic_s;
}

void OperatorMotionAuthority::enterZeroBarrier(ZeroBarrierDisposition disposition) {
  clearSample();
  snapshot_.holding = true;
  snapshot_.zero_required = true;
  snapshot_.zero_barrier_pending = true;
  zero_barrier_disposition_ = disposition;
}

OperatorMotionReceipt
OperatorMotionAuthority::requireZeroBarrier(std::string reason,
                                            ZeroBarrierDisposition disposition) {
  enterZeroBarrier(disposition);
  return accept(std::move(reason), false, true);
}

void OperatorMotionAuthority::refreshLease(double receive_monotonic_s) {
  if (active_lease_ttl_s_ > 0.0) {
    snapshot_.lease_until_monotonic_s = receive_monotonic_s + active_lease_ttl_s_;
  }
}

std::uint64_t OperatorMotionAuthority::sourceHighWater(const std::string &source_id) const {
  const auto it = source_epoch_high_water_.find(source_id);
  return it == source_epoch_high_water_.end() ? 0 : it->second.epoch;
}

bool OperatorMotionAuthority::recordSourceHighWater(const std::string &source_id,
                                                    std::uint64_t epoch) {
  if (source_id.empty()) {
    return false;
  }
  auto it = source_epoch_high_water_.find(source_id);
  if (it == source_epoch_high_water_.end()) {
    if (source_epoch_high_water_.size() >= options_.max_retained_source_identities) {
      if (!evictOldestInactiveSourceIdentity()) {
        return false;
      }
    }
    it = source_epoch_high_water_.emplace(source_id, SourceEpochHighWater{}).first;
  }
  if (epoch > it->second.epoch) {
    it->second.epoch = epoch;
  }
  it->second.retained_order = ++source_identity_retained_order_;
  return true;
}

bool OperatorMotionAuthority::evictOldestInactiveSourceIdentity() {
  auto oldest = source_epoch_high_water_.end();
  for (auto it = source_epoch_high_water_.begin(); it != source_epoch_high_water_.end(); ++it) {
    if (snapshot_.has_active_authority && it->first == snapshot_.active_source_id) {
      continue;
    }
    if (oldest == source_epoch_high_water_.end() ||
        it->second.retained_order < oldest->second.retained_order) {
      oldest = it;
    }
  }
  if (oldest == source_epoch_high_water_.end()) {
    return false;
  }
  source_epoch_high_water_.erase(oldest);
  return true;
}

void OperatorMotionAuthority::clearAuthority() {
  snapshot_.has_active_authority = false;
  snapshot_.active_source_id.clear();
  snapshot_.active_epoch = 0;
  snapshot_.lease_until_monotonic_s = 0.0;
  active_lease_ttl_s_ = 0.0;
  clearSample();
  snapshot_.last_sequence = 0;
  snapshot_.last_sample_sequence = 0;
  zero_barrier_disposition_ = ZeroBarrierDisposition::None;
}

void OperatorMotionAuthority::clearSample() {
  snapshot_.has_active_sample = false;
  snapshot_.holding = false;
  snapshot_.zero_required = false;
  snapshot_.zero_barrier_pending = false;
  snapshot_.active_sample_manual_mode = false;
  snapshot_.active_sample = {};
}

OperatorMotionReceipt
OperatorMotionAuthority::validateActiveControlSequence(const OperatorMotionRelease &control) {
  if (control.source_id.size() > options_.max_source_id_bytes) {
    return reject("source_id_too_long");
  }
  if (control.sequence == 0) {
    return reject("sequence_zero");
  }
  if (!snapshot_.has_active_authority) {
    return reject("no_active_authority");
  }
  if (snapshot_.zero_barrier_pending) {
    return reject("zero_barrier_pending");
  }
  if (control.source_id != snapshot_.active_source_id) {
    return reject("not_active_source");
  }
  if (control.epoch != snapshot_.active_epoch) {
    return reject("not_active_epoch");
  }
  if (control.sequence <= snapshot_.last_sequence) {
    return reject("non_monotonic_sequence");
  }
  return {true, "control_sequence_valid", false, false};
}

std::uint64_t updateOperatorMotionOutputEvidence(
    const OperatorMotionReceipt &completion, const OperatorMotionSnapshot &snapshot,
    std::uint64_t zero_output_sequence, const nav_kernel::Twist &final_command,
    std::uint64_t &admitted_sequence, std::uint64_t &final_output_sequence) noexcept {
  const bool final_is_zero =
      final_command.vx == 0.0 && final_command.vy == 0.0 && final_command.wz == 0.0;
  if (!completion.accepted || zero_output_sequence == 0U || !final_is_zero) {
    return 0U;
  }

  if (snapshot.has_active_authority) {
    final_output_sequence = zero_output_sequence;
  } else {
    admitted_sequence = 0U;
    final_output_sequence = 0U;
  }
  return zero_output_sequence;
}

}  // namespace lingtu::nav::endpoint

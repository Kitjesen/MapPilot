#pragma once

#include <cstdint>

#include "motion/operator_motion_authority.hpp"

namespace lingtu::nav::endpoint {

/// Synchronize the ACK sequence and durable status evidence after a confirmed
/// operator-motion zero barrier.
///
/// A hold retains authority, so its sequenced zero becomes the latest durable
/// output. Release and expiry clear authority and therefore clear the durable
/// per-authority evidence. Failed or internally inconsistent completions leave
/// the prior evidence untouched.
inline std::uint64_t updateOperatorMotionOutputEvidence(
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

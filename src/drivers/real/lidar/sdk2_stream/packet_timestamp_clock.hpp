#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <mutex>
#include <string_view>

namespace lingtu::drivers::lidar {

enum class PacketTimestampStream : std::uint8_t {
  Lidar = 0,
  Imu = 1,
};

enum class PacketTimestampSource : std::uint8_t {
  Fallback = 0,
  Ptp = 1,
  Gps = 2,
  Invalid = 3,
};

enum class PacketTimestampAction : std::uint8_t {
  Publish = 0,
  DropStale = 1,
  Fatal = 2,
};

struct PacketTimestampResult {
  PacketTimestampAction action{PacketTimestampAction::Fatal};
  std::uint64_t stamp_ns{0};
  std::string_view reason{"timestamp_not_mapped"};

  bool publish() const noexcept {
    return action == PacketTimestampAction::Publish;
  }
};

// Maps the fallback host clock and the synchronized device clock onto one
// continuous output timeline. One instance must be shared by LiDAR and IMU
// from the same physical device so their relative timing stays intact.
//
// An isolated outlier is dropped. A persistent clock rollback/jump is fatal so
// the publisher can restart cleanly instead of feeding an unsafe dt to SLAM.
class PacketTimestampClock final {
 public:
  static constexpr std::uint64_t kDefaultMaxSourceSkewNs = 250000000ULL;
  static constexpr std::uint64_t kSevereRollbackNs = 1000000000ULL;

  explicit PacketTimestampClock(
      std::uint64_t max_source_skew_ns = kDefaultMaxSourceSkewNs) noexcept
      : max_source_skew_ns_(max_source_skew_ns) {}

  PacketTimestampResult map(
      PacketTimestampStream stream,
      PacketTimestampSource source,
      std::uint64_t device_ns,
      std::uint64_t fallback_ns) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    if (faulted_) {
      return {PacketTimestampAction::Fatal, 0U, fault_reason_};
    }
    if (source == PacketTimestampSource::Invalid) {
      return fault("timestamp_source_invalid");
    }

    fallback_ns = commit_fallback(fallback_ns);
    StreamState& state = streams_[stream_index(stream)];
    if (source == PacketTimestampSource::Fallback) {
      if (state.has_output && fallback_ns < state.last_output_ns) {
        return {PacketTimestampAction::DropStale, 0U,
                "fallback_packet_older_than_stream"};
      }
      state.has_output = true;
      state.last_output_ns = fallback_ns;
      return {PacketTimestampAction::Publish, fallback_ns, "fallback"};
    }

    if (device_ns == 0U) {
      return fault("device_timestamp_zero");
    }
    if (has_sync_source_ && source != sync_source_) {
      return fault("device_timestamp_source_changed");
    }
    if (!has_sync_source_) {
      sync_source_ = source;
      has_sync_source_ = true;
    }
    if (!has_device_anchor_) {
      device_anchor_ns_ = device_ns;
      output_anchor_ns_ = fallback_ns;
      has_device_anchor_ = true;
    }

    if (state.has_device && device_ns < state.last_device_ns) {
      const std::uint64_t rollback_ns = state.last_device_ns - device_ns;
      return suspect(
          state,
          device_ns,
          "device_timestamp_rollback",
          rollback_ns > kSevereRollbackNs ? 2U : 3U);
    }

    std::uint64_t candidate_ns = 0U;
    if (!translate_from_device(device_ns, candidate_ns)) {
      return fault("device_timestamp_arithmetic_overflow");
    }
    if (absolute_difference(candidate_ns, fallback_ns) > max_source_skew_ns_) {
      const bool forward = candidate_ns > fallback_ns;
      return suspect(
          state,
          device_ns,
          forward ? "device_timestamp_forward_jump"
                  : "device_timestamp_rollback",
          3U);
    }
    if (state.has_output && candidate_ns < state.last_output_ns) {
      return suspect(
          state,
          device_ns,
          "device_timestamp_stream_reorder",
          3U);
    }

    clear_suspect(state);
    state.has_device = true;
    state.last_device_ns = device_ns;
    state.has_output = true;
    state.last_output_ns = candidate_ns;
    return {PacketTimestampAction::Publish, candidate_ns, "synchronized"};
  }

  bool faulted() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return faulted_;
  }

  std::string_view fault_reason() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return fault_reason_;
  }

 private:
  struct StreamState {
    bool has_device{false};
    bool has_output{false};
    std::uint64_t last_device_ns{0};
    std::uint64_t last_output_ns{0};
    std::uint64_t suspect_device_ns{0};
    std::string_view suspect_reason{};
    std::uint32_t suspect_count{0};
  };

  static constexpr std::size_t stream_index(PacketTimestampStream stream) {
    return static_cast<std::size_t>(stream);
  }

  PacketTimestampResult suspect(
      StreamState& state,
      std::uint64_t device_ns,
      std::string_view reason,
      std::uint32_t fatal_count) noexcept {
    const bool continuing = state.suspect_count > 0U &&
        state.suspect_reason == reason &&
        device_ns >= state.suspect_device_ns;
    state.suspect_count = continuing ? state.suspect_count + 1U : 1U;
    state.suspect_device_ns = device_ns;
    state.suspect_reason = reason;
    if (state.suspect_count >= fatal_count) {
      return fault(reason);
    }
    return {PacketTimestampAction::DropStale, 0U, reason};
  }

  static void clear_suspect(StreamState& state) noexcept {
    state.suspect_count = 0U;
    state.suspect_device_ns = 0U;
    state.suspect_reason = {};
  }

  PacketTimestampResult fault(std::string_view reason) noexcept {
    faulted_ = true;
    fault_reason_ = reason;
    return {PacketTimestampAction::Fatal, 0U, fault_reason_};
  }

  bool translate_from_device(
      std::uint64_t device_ns,
      std::uint64_t& translated_ns) const noexcept {
    if (device_ns >= device_anchor_ns_) {
      const std::uint64_t elapsed_ns = device_ns - device_anchor_ns_;
      if (elapsed_ns > std::numeric_limits<std::uint64_t>::max() -
                           output_anchor_ns_) {
        return false;
      }
      translated_ns = output_anchor_ns_ + elapsed_ns;
      return true;
    }

    const std::uint64_t rollback_ns = device_anchor_ns_ - device_ns;
    if (rollback_ns > output_anchor_ns_) {
      return false;
    }
    translated_ns = output_anchor_ns_ - rollback_ns;
    return true;
  }

  std::uint64_t commit_fallback(std::uint64_t fallback_ns) noexcept {
    if (!has_fallback_ || fallback_ns > latest_fallback_ns_) {
      latest_fallback_ns_ = fallback_ns;
    }
    has_fallback_ = true;
    return latest_fallback_ns_;
  }

  static std::uint64_t absolute_difference(
      std::uint64_t lhs,
      std::uint64_t rhs) noexcept {
    return lhs >= rhs ? lhs - rhs : rhs - lhs;
  }

  mutable std::mutex mutex_;
  const std::uint64_t max_source_skew_ns_;
  bool has_fallback_{false};
  bool has_device_anchor_{false};
  bool has_sync_source_{false};
  bool faulted_{false};
  PacketTimestampSource sync_source_{PacketTimestampSource::Invalid};
  std::uint64_t device_anchor_ns_{0};
  std::uint64_t output_anchor_ns_{0};
  std::uint64_t latest_fallback_ns_{0};
  std::string_view fault_reason_{};
  std::array<StreamState, 2> streams_{};
};

}  // namespace lingtu::drivers::lidar

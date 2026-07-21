#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <string>
#include <string_view>

namespace lingtu::nav::endpoint {

enum class SourceStampDecision {
  kAccept,
  kClockRebase,
  kReject,
};

// Header timestamps describe the producer's clock domain. They are suitable
// for source ordering and transform association, but not for receiver-side
// liveness because NTP/PTP steps and replay pacing can move producer and
// consumer wall clocks independently. Freshness is evaluated separately from
// local steady-clock receipt timestamps in InputGate.
inline SourceStampDecision classifySourceOrder(
    double previous_stamp_s,
    double incoming_stamp_s,
    double clock_rebase_threshold_s) {
  if (!std::isfinite(incoming_stamp_s) || incoming_stamp_s <= 0.0) {
    return SourceStampDecision::kReject;
  }
  if (previous_stamp_s <= 0.0 || incoming_stamp_s + 1e-9 >= previous_stamp_s) {
    return SourceStampDecision::kAccept;
  }
  const double rollback_s = previous_stamp_s - incoming_stamp_s;
  return rollback_s > std::max(0.0, clock_rebase_threshold_s)
      ? SourceStampDecision::kClockRebase
      : SourceStampDecision::kReject;
}

inline SourceStampDecision classifySourceStamp(
    double previous_stamp_s,
    double incoming_stamp_s,
    double receive_now_s,
    double future_tolerance_s,
    double max_age_s) {
  if (!std::isfinite(incoming_stamp_s) || incoming_stamp_s <= 0.0 ||
      !std::isfinite(receive_now_s) || receive_now_s <= 0.0) {
    return SourceStampDecision::kReject;
  }
  const double bounded_future_tolerance_s = std::max(0.0, future_tolerance_s);
  const double bounded_max_age_s = std::max(0.0, max_age_s);
  const double incoming_age_s = receive_now_s - incoming_stamp_s;
  if (incoming_age_s < -bounded_future_tolerance_s ||
      (bounded_max_age_s > 0.0 && incoming_age_s > bounded_max_age_s)) {
    return SourceStampDecision::kReject;
  }
  if (previous_stamp_s <= 0.0 || incoming_stamp_s + 1e-9 >= previous_stamp_s) {
    return SourceStampDecision::kAccept;
  }

  const double previous_age_s = receive_now_s - previous_stamp_s;
  const bool previous_is_future =
      previous_age_s < -bounded_future_tolerance_s;
  const bool incoming_matches_current_epoch =
      incoming_age_s >= -bounded_future_tolerance_s &&
      (bounded_max_age_s <= 0.0 || incoming_age_s <= bounded_max_age_s);
  return previous_is_future && incoming_matches_current_epoch
      ? SourceStampDecision::kClockRebase
      : SourceStampDecision::kReject;
}

struct InputGateConfig {
  double odom_max_age_s{0.25};
  double tf_max_age_s{0.25};
  double cloud_max_age_s{0.35};
  std::uint32_t recovery_frames{3};
  double future_tolerance_s{0.05};
  bool require_odom{true};
  bool require_cloud{true};
  double traversability_max_age_s{1.5};
  double localization_health_max_age_s{0.5};
  double driver_control_max_age_s{0.35};
  bool require_traversability{false};
  bool require_localization_health{false};
  bool require_driver_control{false};
  double odom_max_speed_mps{3.0};
};

// Source timestamps, local steady-clock receipt timestamps, and accepted-sample
// generations are part of the safety contract. Source timestamps retain the
// producer clock for ordering and diagnostics. Freshness uses the corresponding
// *_receive_s value so producer/consumer wall-clock adjustments cannot create a
// false stale/future stop. Legacy callers may omit receipt timestamps and fall
// back to source timestamps while they migrate.
struct InputSnapshot {
  double now_s{0.0};
  double odom_stamp_s{0.0};
  double odom_receive_s{0.0};
  std::uint64_t odom_generation{0};
  double odom_linear_speed_mps{0.0};
  double tf_stamp_s{0.0};
  double tf_receive_s{0.0};
  std::uint64_t tf_generation{0};
  double cloud_stamp_s{0.0};
  double cloud_receive_s{0.0};
  std::uint64_t cloud_generation{0};
  double traversability_stamp_s{0.0};
  double traversability_receive_s{0.0};
  std::uint64_t traversability_generation{0};
  double localization_health_stamp_s{0.0};
  double localization_health_receive_s{0.0};
  std::uint64_t localization_health_generation{0};
  double driver_control_stamp_s{0.0};
  double driver_control_receive_s{0.0};
  std::uint64_t driver_control_generation{0};
  bool odom_requires_tf{true};
  bool localization_healthy{false};
  std::string localization_state;
  std::string localization_reason;
  bool driver_control_ready{false};
  std::string driver_control_reason;
};

struct LocalizationHealthSample {
  bool valid{false};
  bool healthy{false};
  double stamp_s{0.0};
  std::string state;
  std::string reason;
  std::string error;
};

bool isHealthyLocalizationState(std::string_view state);
bool isCatastrophicLocalizationReason(std::string_view reason);
LocalizationHealthSample decodeLocalizationHealth(std::string_view json);

class OdometrySpeedMonitor {
 public:
  double observe(
      double stamp_s,
      std::string_view frame_id,
      double x,
      double y,
      double z,
      double vx,
      double vy,
      double vz) {
    if (!std::isfinite(stamp_s) || stamp_s <= 0.0 ||
        !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        !std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(vz)) {
      return std::numeric_limits<double>::infinity();
    }

    double speed_mps = std::hypot(vx, vy, vz);
    if (previous_frame_ != frame_id) {
      resetPoseHistory(frame_id);
    } else if (
        sample_count_ > 0 &&
        stamp_s - samples_[sample_count_ - 1].stamp_s <= 1e-6) {
      return std::numeric_limits<double>::infinity();
    }

    appendSample({stamp_s, {x, y, z}});
    if (filtered_samples_.empty()) {
      filtered_samples_.push_back({stamp_s, {x, y, z}});
    }

    // DDS 3-D twist failures remain immediate. Geometry is a navigation-frame
    // consistency fallback, so derive planar speed from a causal five-sample
    // median instead of adjacent 3-D poses. This ignores quadruped body heave,
    // rejects one- or two-tick contact/SLAM outliers, then measures a 50 ms
    // filtered-pose chord so gait oscillation is not amplified by a 5-10 ms
    // derivative. Persistent XY teleport or sustained overspeed still closes
    // the gate on the bounded fallback window.
    if (sample_count_ == samples_.size()) {
      const double filtered_stamp_s = samples_[samples_.size() / 2].stamp_s;
      const auto filtered_position = medianPosition();
      if (
          !filtered_samples_.empty() &&
          filtered_stamp_s - filtered_samples_.back().stamp_s <= 1e-6) {
        return std::numeric_limits<double>::infinity();
      }
      filtered_samples_.push_back({filtered_stamp_s, filtered_position});

      constexpr double kPoseChordMinSpanS = 0.05;
      const PoseSample* baseline = nullptr;
      for (const auto& sample : filtered_samples_) {
        if (filtered_stamp_s - sample.stamp_s + 1e-9 < kPoseChordMinSpanS) {
          break;
        }
        baseline = &sample;
      }
      if (baseline != nullptr) {
        const double dt_s = filtered_stamp_s - baseline->stamp_s;
        speed_mps = std::max(
            speed_mps,
            planarDistance(filtered_position, baseline->position) / dt_s);
      }

      constexpr double kPoseHistoryS = 0.10;
      while (
          filtered_samples_.size() > 2 &&
          filtered_stamp_s - filtered_samples_[1].stamp_s > kPoseHistoryS) {
        filtered_samples_.pop_front();
      }
    }
    return speed_mps;
  }

  void reset() {
    previous_frame_.clear();
    sample_count_ = 0;
    filtered_samples_.clear();
  }

 private:
  struct PoseSample {
    double stamp_s{0.0};
    std::array<double, 3> position{};
  };

  static double planarDistance(
      const std::array<double, 3>& lhs,
      const std::array<double, 3>& rhs) {
    return std::hypot(lhs[0] - rhs[0], lhs[1] - rhs[1]);
  }

  std::array<double, 3> medianPosition() const {
    std::array<double, 3> result{};
    for (std::size_t axis = 0; axis < result.size(); ++axis) {
      std::array<double, 5> values{};
      for (std::size_t index = 0; index < samples_.size(); ++index) {
        values[index] = samples_[index].position[axis];
      }
      std::sort(values.begin(), values.end());
      result[axis] = values[values.size() / 2];
    }
    return result;
  }

  void resetPoseHistory(std::string_view frame_id) {
    previous_frame_.assign(frame_id);
    sample_count_ = 0;
    filtered_samples_.clear();
  }

  void appendSample(const PoseSample& sample) {
    if (sample_count_ < samples_.size()) {
      samples_[sample_count_++] = sample;
      return;
    }
    std::move(samples_.begin() + 1, samples_.end(), samples_.begin());
    samples_.back() = sample;
  }

  std::string previous_frame_;
  std::array<PoseSample, 5> samples_{};
  std::size_t sample_count_{0};
  std::deque<PoseSample> filtered_samples_;
};

struct InputGateState {
  bool ready{false};
  bool recovering{false};
  std::string reason{"not_evaluated"};
  double odom_age_s{-1.0};
  double tf_age_s{-1.0};
  double cloud_age_s{-1.0};
  double traversability_age_s{-1.0};
  double localization_health_age_s{-1.0};
  double driver_control_age_s{-1.0};
  bool localization_healthy{false};
  std::string localization_state;
  std::string localization_reason;
  bool driver_control_ready{false};
  std::string driver_control_reason;
  double odom_linear_speed_mps{0.0};
  std::uint32_t fresh_frames{0};
};

class InputGate {
 public:
  explicit InputGate(InputGateConfig config = {}) : config_(config) {
    config_.odom_max_age_s = std::max(0.0, config_.odom_max_age_s);
    config_.tf_max_age_s = std::max(0.0, config_.tf_max_age_s);
    config_.cloud_max_age_s = std::max(0.0, config_.cloud_max_age_s);
    config_.traversability_max_age_s =
        std::max(0.0, config_.traversability_max_age_s);
    config_.localization_health_max_age_s =
        std::max(0.0, config_.localization_health_max_age_s);
    config_.driver_control_max_age_s =
        std::max(0.0, config_.driver_control_max_age_s);
    config_.odom_max_speed_mps = std::max(0.0, config_.odom_max_speed_mps);
    config_.recovery_frames = std::max<std::uint32_t>(1, config_.recovery_frames);
    config_.future_tolerance_s = std::max(0.0, config_.future_tolerance_s);
  }

  InputGateState evaluate(const InputSnapshot& inputs) {
    InputGateState state;
    state.odom_age_s =
        config_.require_odom
        ? age(
              inputs.now_s,
              receiveOrSource(inputs.odom_receive_s, inputs.odom_stamp_s))
        : -1.0;
    state.tf_age_s = config_.require_odom && inputs.odom_requires_tf
        ? age(
              inputs.now_s,
              receiveOrSource(inputs.tf_receive_s, inputs.tf_stamp_s))
        : -1.0;
    state.cloud_age_s =
        config_.require_cloud
        ? age(
              inputs.now_s,
              receiveOrSource(inputs.cloud_receive_s, inputs.cloud_stamp_s))
        : -1.0;
    state.traversability_age_s = config_.require_traversability
        ? age(
              inputs.now_s,
              receiveOrSource(
                  inputs.traversability_receive_s,
                  inputs.traversability_stamp_s))
        : -1.0;
    state.localization_health_age_s = config_.require_localization_health
        ? age(
              inputs.now_s,
              receiveOrSource(
                  inputs.localization_health_receive_s,
                  inputs.localization_health_stamp_s))
        : -1.0;
    state.driver_control_age_s = config_.require_driver_control
        ? age(
              inputs.now_s,
              receiveOrSource(
                  inputs.driver_control_receive_s,
                  inputs.driver_control_stamp_s))
        : -1.0;
    state.localization_healthy = inputs.localization_healthy;
    state.localization_state = inputs.localization_state;
    state.localization_reason = inputs.localization_reason;
    state.driver_control_ready = inputs.driver_control_ready;
    state.driver_control_reason = inputs.driver_control_reason;
    state.odom_linear_speed_mps = inputs.odom_linear_speed_mps;

    const char* stop_reason = nullptr;
    if (!std::isfinite(inputs.now_s) || inputs.now_s <= 0.0) {
      stop_reason = "input_clock_invalid";
    } else if (
        config_.require_driver_control && inputs.driver_control_stamp_s <= 0.0) {
      stop_reason = "driver_control_missing";
    } else if (
        config_.require_driver_control &&
        state.driver_control_age_s < -config_.future_tolerance_s) {
      stop_reason = "driver_control_future";
    } else if (
        config_.require_driver_control && config_.driver_control_max_age_s > 0.0 &&
        state.driver_control_age_s > config_.driver_control_max_age_s) {
      stop_reason = "driver_control_stale";
    } else if (
        config_.require_driver_control && !inputs.driver_control_ready) {
      stop_reason = inputs.driver_control_reason.empty()
          ? "driver_control_not_ready"
          : "driver_control_rejected";
    } else if (config_.require_odom && inputs.odom_stamp_s <= 0.0) {
      stop_reason = "odom_missing";
    } else if (config_.require_odom && state.odom_age_s < -config_.future_tolerance_s) {
      stop_reason = "odom_future";
    } else if (
        config_.require_odom && config_.odom_max_age_s > 0.0 &&
        state.odom_age_s > config_.odom_max_age_s) {
      stop_reason = "odom_stale";
    } else if (
        config_.require_odom && !std::isfinite(inputs.odom_linear_speed_mps)) {
      stop_reason = "odom_velocity_nonfinite";
    } else if (
        config_.require_odom && config_.odom_max_speed_mps > 0.0 &&
        inputs.odom_linear_speed_mps > config_.odom_max_speed_mps) {
      stop_reason = "odom_velocity_out_of_bounds";
    } else if (
        config_.require_odom && inputs.odom_requires_tf &&
        inputs.tf_stamp_s <= 0.0) {
      stop_reason = "tf_missing";
    } else if (
        config_.require_odom && inputs.odom_requires_tf &&
        state.tf_age_s < -config_.future_tolerance_s) {
      stop_reason = "tf_future";
    } else if (
        config_.require_odom && inputs.odom_requires_tf &&
        config_.tf_max_age_s > 0.0 &&
        state.tf_age_s > config_.tf_max_age_s) {
      stop_reason = "tf_stale";
    } else if (config_.require_cloud && inputs.cloud_stamp_s <= 0.0) {
      stop_reason = "cloud_missing";
    } else if (config_.require_cloud && state.cloud_age_s < -config_.future_tolerance_s) {
      stop_reason = "cloud_future";
    } else if (
        config_.require_cloud && config_.cloud_max_age_s > 0.0 &&
        state.cloud_age_s > config_.cloud_max_age_s) {
      stop_reason = "cloud_stale";
    } else if (
        config_.require_traversability && inputs.traversability_stamp_s <= 0.0) {
      stop_reason = "traversability_missing";
    } else if (
        config_.require_traversability &&
        state.traversability_age_s < -config_.future_tolerance_s) {
      stop_reason = "traversability_future";
    } else if (
        config_.require_traversability && config_.traversability_max_age_s > 0.0 &&
        state.traversability_age_s > config_.traversability_max_age_s) {
      stop_reason = "traversability_stale";
    } else if (
        config_.require_localization_health &&
        inputs.localization_health_stamp_s <= 0.0) {
      stop_reason = "localization_health_missing";
    } else if (
        config_.require_localization_health &&
        state.localization_health_age_s < -config_.future_tolerance_s) {
      stop_reason = "localization_health_future";
    } else if (
        config_.require_localization_health &&
        config_.localization_health_max_age_s > 0.0 &&
        state.localization_health_age_s > config_.localization_health_max_age_s) {
      stop_reason = "localization_health_stale";
    } else if (
        config_.require_localization_health &&
        isCatastrophicLocalizationReason(inputs.localization_reason)) {
      stop_reason = "localization_catastrophic";
    } else if (
        config_.require_localization_health &&
        !inputs.localization_state.empty() &&
        !isHealthyLocalizationState(inputs.localization_state)) {
      stop_reason = "localization_not_tracking";
    } else if (
        config_.require_localization_health && !inputs.localization_healthy) {
      stop_reason = "localization_unhealthy";
    }

    if (stop_reason != nullptr) {
      fresh_frames_ = 0;
      recovery_generations_ = generations(inputs);
      state.reason = stop_reason;
      return state;
    }

    if (fresh_frames_ < config_.recovery_frames &&
        allRequiredInputsAdvanced(inputs)) {
      fresh_frames_ = std::min<std::uint32_t>(
          config_.recovery_frames, fresh_frames_ + 1);
      recovery_generations_ = generations(inputs);
    }
    state.fresh_frames = fresh_frames_;
    state.ready = fresh_frames_ >= config_.recovery_frames;
    state.recovering = !state.ready;
    state.reason = state.ready ? "ready" : "recovering";
    return state;
  }

  // Compatibility seam for legacy callers without DDS sample generations.
  // Production integrations should construct InputSnapshot explicitly.
  InputGateState evaluate(
      double now_s,
      double last_odom_s,
      double last_tf_s,
      double last_cloud_s,
      bool require_tf) {
    InputSnapshot inputs;
    inputs.now_s = now_s;
    inputs.odom_stamp_s = last_odom_s;
    inputs.tf_stamp_s = last_tf_s;
    inputs.cloud_stamp_s = last_cloud_s;
    inputs.odom_requires_tf = require_tf;
    const std::uint64_t generation = ++compat_generation_;
    inputs.odom_generation = generation;
    inputs.tf_generation = generation;
    inputs.cloud_generation = generation;
    inputs.driver_control_stamp_s = now_s;
    inputs.driver_control_generation = generation;
    inputs.driver_control_ready = true;
    return evaluate(inputs);
  }

  void reset() {
    fresh_frames_ = 0;
    recovery_generations_ = {};
    compat_generation_ = 0;
  }

  // Close an already-open gate at a coordinate-epoch boundary. Recovery may
  // advance only after every required DDS input has produced a newer accepted
  // generation than the supplied boundary snapshot.
  void beginRecoveryFrom(const InputSnapshot& inputs) {
    fresh_frames_ = 0;
    recovery_generations_ = generations(inputs);
  }

 private:
  struct InputGenerations {
    std::uint64_t odom{0};
    std::uint64_t tf{0};
    std::uint64_t cloud{0};
    std::uint64_t traversability{0};
    std::uint64_t localization_health{0};
    std::uint64_t driver_control{0};
  };

  static InputGenerations generations(const InputSnapshot& inputs) {
    return {
        inputs.odom_generation,
        inputs.tf_generation,
        inputs.cloud_generation,
        inputs.traversability_generation,
        inputs.localization_health_generation,
        inputs.driver_control_generation,
    };
  }

  static bool advanced(std::uint64_t current, std::uint64_t previous) {
    return current > previous;
  }

  bool allRequiredInputsAdvanced(const InputSnapshot& inputs) const {
    if (config_.require_odom &&
        !advanced(inputs.odom_generation, recovery_generations_.odom)) {
      return false;
    }
    if (config_.require_odom && inputs.odom_requires_tf &&
        !advanced(inputs.tf_generation, recovery_generations_.tf)) {
      return false;
    }
    if (config_.require_cloud &&
        !advanced(inputs.cloud_generation, recovery_generations_.cloud)) {
      return false;
    }
    if (config_.require_traversability &&
        !advanced(
            inputs.traversability_generation,
            recovery_generations_.traversability)) {
      return false;
    }
    if (config_.require_localization_health &&
        !advanced(
            inputs.localization_health_generation,
            recovery_generations_.localization_health)) {
      return false;
    }
    if (config_.require_driver_control &&
        !advanced(
            inputs.driver_control_generation,
            recovery_generations_.driver_control)) {
      return false;
    }
    return true;
  }

  static double age(double now_s, double last_s) {
    if (last_s <= 0.0) {
      return std::numeric_limits<double>::infinity();
    }
    return now_s - last_s;
  }

  static double receiveOrSource(double receive_s, double source_s) {
    return std::isfinite(receive_s) && receive_s > 0.0
        ? receive_s
        : source_s;
  }

  InputGateConfig config_{};
  std::uint32_t fresh_frames_{0};
  InputGenerations recovery_generations_{};
  std::uint64_t compat_generation_{0};
};

}  // namespace lingtu::nav::endpoint

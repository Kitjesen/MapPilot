#pragma once

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

namespace lingtu::nav::endpoint {

struct StopConfirmationConfig {
  std::chrono::milliseconds timeout{4000};
  double linear_speed_threshold_mps{0.03};
  double angular_speed_threshold_radps{0.08};
  std::size_t quiet_odometry_samples{8};
};

enum class StopConfirmationState {
  Pending,
  Confirmed,
  DriverRejected,
  TimedOut,
};

struct StopConfirmationDiagnostics {
  std::uint64_t zero_published_source_wall_ns{0U};
  std::uint64_t driver_ack_source_stamp_ns{0U};
  std::uint64_t last_odometry_source_stamp_ns{0U};
  std::size_t odometry_samples_observed{0U};
  std::size_t post_ack_odometry_samples{0U};
  std::size_t stale_odometry_samples{0U};
  std::size_t moving_odometry_samples{0U};
  std::size_t quiet_odometry_samples{0U};
  std::size_t required_quiet_odometry_samples{0U};
  double last_linear_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double last_angular_speed_radps{std::numeric_limits<double>::quiet_NaN()};
  bool driver_ack_observed{false};
  bool driver_accepted{false};
};

class StopConfirmation {
 public:
  using Clock = std::chrono::steady_clock;

  StopConfirmation(std::string producer_boot_id, std::uint64_t output_sequence,
                   std::uint64_t zero_published_source_wall_ns, Clock::time_point started_at,
                   StopConfirmationConfig config = {})
      : producer_boot_id_(std::move(producer_boot_id)),
        output_sequence_(output_sequence),
        zero_published_source_wall_ns_(zero_published_source_wall_ns),
        started_at_(started_at),
        config_(config) {
    if (producer_boot_id_.empty() || output_sequence_ == 0U ||
        zero_published_source_wall_ns_ == 0U) {
      throw std::invalid_argument(
          "stop confirmation requires a sequenced producer identity and "
          "zero-publication source wall stamp");
    }
    if (config_.timeout <= std::chrono::milliseconds::zero() ||
        !std::isfinite(config_.linear_speed_threshold_mps) ||
        config_.linear_speed_threshold_mps < 0.0 ||
        !std::isfinite(config_.angular_speed_threshold_radps) ||
        config_.angular_speed_threshold_radps < 0.0 || config_.quiet_odometry_samples == 0U) {
      throw std::invalid_argument("invalid stop confirmation configuration");
    }
  }

  void observeDriverAck(const std::string &producer_boot_id, std::uint64_t output_sequence,
                        bool accepted, std::uint64_t source_stamp_ns) noexcept {
    if (producer_boot_id != producer_boot_id_ || output_sequence != output_sequence_ ||
        source_stamp_ns <= zero_published_source_wall_ns_) {
      return;
    }
    if (!driver_ack_observed_ || driver_accepted_ != accepted) {
      // Only odometry observed after the matching Brainstem result can prove
      // that the sequenced zero was applied.
      quiet_odometry_samples_ = 0U;
      last_qualified_odometry_source_stamp_ns_ = 0U;
      driver_ack_source_stamp_ns_ = source_stamp_ns;
    }
    driver_ack_observed_ = true;
    driver_accepted_ = accepted;
  }

  void observeQuietOdometry(double source_stamp_s, double linear_speed_mps,
                            double angular_speed_radps) noexcept {
    ++odometry_samples_observed_;
    last_observed_odometry_source_stamp_ns_ = sourceStampNanoseconds(source_stamp_s);
    last_linear_speed_mps_ = linear_speed_mps;
    last_angular_speed_radps_ = angular_speed_radps;
    if (!driver_ack_observed_ || !driver_accepted_) {
      return;
    }
    ++post_ack_odometry_samples_;
    const std::uint64_t source_stamp_ns = last_observed_odometry_source_stamp_ns_;
    const std::uint64_t evidence_not_before_source_stamp_ns =
        std::max(zero_published_source_wall_ns_, driver_ack_source_stamp_ns_);
    if (source_stamp_ns <= evidence_not_before_source_stamp_ns ||
        source_stamp_ns <= last_qualified_odometry_source_stamp_ns_) {
      ++stale_odometry_samples_;
      return;
    }
    last_qualified_odometry_source_stamp_ns_ = source_stamp_ns;
    if (!std::isfinite(linear_speed_mps) || !std::isfinite(angular_speed_radps) ||
        linear_speed_mps > config_.linear_speed_threshold_mps ||
        angular_speed_radps > config_.angular_speed_threshold_radps) {
      ++moving_odometry_samples_;
      quiet_odometry_samples_ = 0U;
      return;
    }
    if (quiet_odometry_samples_ < config_.quiet_odometry_samples) {
      ++quiet_odometry_samples_;
    }
  }

  StopConfirmationState state(Clock::time_point now = Clock::now()) const noexcept {
    if (driver_ack_observed_ && !driver_accepted_) {
      return StopConfirmationState::DriverRejected;
    }
    if (now - started_at_ >= config_.timeout) {
      return StopConfirmationState::TimedOut;
    }
    if (driver_ack_observed_ && driver_accepted_ &&
        quiet_odometry_samples_ >= config_.quiet_odometry_samples) {
      return StopConfirmationState::Confirmed;
    }
    return StopConfirmationState::Pending;
  }

  StopConfirmationDiagnostics diagnostics() const noexcept {
    return {
        zero_published_source_wall_ns_,
        driver_ack_source_stamp_ns_,
        last_observed_odometry_source_stamp_ns_,
        odometry_samples_observed_,
        post_ack_odometry_samples_,
        stale_odometry_samples_,
        moving_odometry_samples_,
        quiet_odometry_samples_,
        config_.quiet_odometry_samples,
        last_linear_speed_mps_,
        last_angular_speed_radps_,
        driver_ack_observed_,
        driver_accepted_,
    };
  }

 private:
  static std::uint64_t sourceStampNanoseconds(double source_stamp_s) noexcept {
    if (!std::isfinite(source_stamp_s) || source_stamp_s <= 0.0) {
      return 0U;
    }
    const long double ns = static_cast<long double>(source_stamp_s) * 1000000000.0L;
    if (ns >= static_cast<long double>(std::numeric_limits<std::uint64_t>::max())) {
      return std::numeric_limits<std::uint64_t>::max();
    }
    return static_cast<std::uint64_t>(ns + 0.5L);
  }

  std::string producer_boot_id_;
  std::uint64_t output_sequence_{0};
  std::uint64_t zero_published_source_wall_ns_{0U};
  Clock::time_point started_at_{};
  StopConfirmationConfig config_;
  std::uint64_t driver_ack_source_stamp_ns_{0U};
  std::uint64_t last_observed_odometry_source_stamp_ns_{0U};
  std::uint64_t last_qualified_odometry_source_stamp_ns_{0U};
  std::size_t odometry_samples_observed_{0U};
  std::size_t post_ack_odometry_samples_{0U};
  std::size_t stale_odometry_samples_{0U};
  std::size_t moving_odometry_samples_{0U};
  std::size_t quiet_odometry_samples_{0};
  double last_linear_speed_mps_{std::numeric_limits<double>::quiet_NaN()};
  double last_angular_speed_radps_{std::numeric_limits<double>::quiet_NaN()};
  bool driver_ack_observed_{false};
  bool driver_accepted_{false};
};

}  // namespace lingtu::nav::endpoint

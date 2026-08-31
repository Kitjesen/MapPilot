#include "input/gate.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <limits>

namespace lingtu::nav::endpoint {
namespace {

std::string_view jsonValue(std::string_view json, std::string_view key) {
  const auto object_start = json.find_first_not_of(" \t\r\n");
  if (object_start == std::string_view::npos || json[object_start] != '{') {
    return {};
  }

  int depth = 0;
  for (std::size_t i = object_start; i < json.size(); ++i) {
    const char ch = json[i];
    if (ch == '{' || ch == '[') {
      ++depth;
      continue;
    }
    if (ch == '}' || ch == ']') {
      --depth;
      continue;
    }
    if (ch != '"') {
      continue;
    }

    const std::size_t text_start = i + 1;
    bool escaped = false;
    for (++i; i < json.size(); ++i) {
      if (escaped) {
        escaped = false;
      } else if (json[i] == '\\') {
        escaped = true;
      } else if (json[i] == '"') {
        break;
      }
    }
    if (i >= json.size() || depth != 1 || json.substr(text_start, i - text_start) != key) {
      continue;
    }
    const auto colon_pos = json.find_first_not_of(" \t\r\n", i + 1);
    if (colon_pos == std::string_view::npos || json[colon_pos] != ':') {
      continue;
    }
    const auto value_pos = json.find_first_not_of(" \t\r\n", colon_pos + 1);
    if (value_pos != std::string_view::npos) {
      return json.substr(value_pos);
    }
  }
  return {};
}

bool parseString(std::string_view json, std::string_view key, std::string &out) {
  const std::string_view value = jsonValue(json, key);
  if (value.size() < 2 || value.front() != '"') {
    return false;
  }
  const auto end = value.find('"', 1);
  if (end == std::string_view::npos) {
    return false;
  }
  out.assign(value.substr(1, end - 1));
  return true;
}

bool parsePositiveFiniteDouble(std::string_view json, std::string_view key, double &out) {
  const std::string_view value = jsonValue(json, key);
  if (value.empty()) {
    return false;
  }
  const std::string owned(value);
  char *end = nullptr;
  errno = 0;
  const double parsed = std::strtod(owned.c_str(), &end);
  if (end == owned.c_str() || errno == ERANGE || !std::isfinite(parsed) || parsed <= 0.0) {
    return false;
  }
  out = parsed;
  return true;
}

}  // namespace

SourceStampDecision classifySourceOrder(double previous_stamp_s, double incoming_stamp_s,
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

bool isHealthyLocalizationState(std::string_view state) {
  std::string normalized(state);
  std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                 [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  return normalized == "TRACKING" || normalized == "LOCKED" || normalized == "RECOVERED" ||
         normalized == "OK";
}

bool isCatastrophicLocalizationReason(std::string_view reason) {
  std::string normalized(reason);
  std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return normalized == "fastlio_state_nonfinite" ||
         normalized == "fastlio_velocity_out_of_bounds" ||
         normalized.find("catastrophic") != std::string::npos;
}

LocalizationHealthSample decodeLocalizationHealth(std::string_view json) {
  LocalizationHealthSample out;
  if (!parseString(json, "state", out.state)) {
    out.error = "localization_health_state_missing";
    return out;
  }
  if (!parsePositiveFiniteDouble(json, "ts", out.stamp_s)) {
    out.error = "localization_health_timestamp_missing";
    return out;
  }
  (void)parseString(json, "reason", out.reason);
  out.valid = true;
  out.healthy =
      isHealthyLocalizationState(out.state) && !isCatastrophicLocalizationReason(out.reason);
  return out;
}

OdometrySpeedMonitor::OdometrySpeedMonitor(OdometrySpeedEvidence evidence) : evidence_(evidence) {}

double OdometrySpeedMonitor::observe(double stamp_s, std::string_view frame_id, double x, double y,
                                     double z, double vx, double vy, double vz) {
  if (!std::isfinite(stamp_s) || stamp_s <= 0.0 || !std::isfinite(x) || !std::isfinite(y) ||
      !std::isfinite(z) || !std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(vz)) {
    return std::numeric_limits<double>::infinity();
  }

  double speed_mps = evidence_ == OdometrySpeedEvidence::ReportedTwistAndPose
                         ? std::hypot(vx, vy, vz)
                         : std::numeric_limits<double>::infinity();
  if (previous_frame_ != frame_id) {
    resetPoseHistory(frame_id);
  } else if (sample_count_ > 0 &&
             stamp_s - samples_[sample_count_ - 1].stamp_s <= kOdometryMinimumStampAdvanceS) {
    return std::numeric_limits<double>::infinity();
  }

  appendSample({stamp_s, {x, y, z}});
  if (filtered_samples_.empty()) {
    filtered_samples_.push_back({stamp_s, {x, y, z}});
  }

  if (sample_count_ == samples_.size()) {
    const double filtered_stamp_s = samples_[samples_.size() / 2].stamp_s;
    const auto filtered_position = medianPosition();
    if (!filtered_samples_.empty() &&
        filtered_stamp_s - filtered_samples_.back().stamp_s <= 1e-6) {
      return std::numeric_limits<double>::infinity();
    }
    filtered_samples_.push_back({filtered_stamp_s, filtered_position});

    constexpr double kPoseChordMinSpanS = 0.05;
    const PoseSample *baseline = nullptr;
    for (const auto &sample : filtered_samples_) {
      if (filtered_stamp_s - sample.stamp_s + 1e-9 < kPoseChordMinSpanS) {
        break;
      }
      baseline = &sample;
    }
    if (baseline != nullptr) {
      const double dt_s = filtered_stamp_s - baseline->stamp_s;
      const double pose_speed_mps = planarDistance(filtered_position, baseline->position) / dt_s;
      speed_mps = evidence_ == OdometrySpeedEvidence::ReportedTwistAndPose
                      ? std::max(speed_mps, pose_speed_mps)
                      : pose_speed_mps;
    }

    constexpr double kPoseHistoryS = 0.10;
    while (filtered_samples_.size() > 2 &&
           filtered_stamp_s - filtered_samples_[1].stamp_s > kPoseHistoryS) {
      filtered_samples_.pop_front();
    }
  }
  return speed_mps;
}

void OdometrySpeedMonitor::reset() {
  previous_frame_.clear();
  sample_count_ = 0;
  filtered_samples_.clear();
}

double OdometrySpeedMonitor::planarDistance(const std::array<double, 3> &lhs,
                                            const std::array<double, 3> &rhs) {
  return std::hypot(lhs[0] - rhs[0], lhs[1] - rhs[1]);
}

std::array<double, 3> OdometrySpeedMonitor::medianPosition() const {
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

void OdometrySpeedMonitor::resetPoseHistory(std::string_view frame_id) {
  previous_frame_.assign(frame_id);
  sample_count_ = 0;
  filtered_samples_.clear();
}

void OdometrySpeedMonitor::appendSample(const PoseSample &sample) {
  if (sample_count_ < samples_.size()) {
    samples_[sample_count_++] = sample;
    return;
  }
  std::move(samples_.begin() + 1, samples_.end(), samples_.begin());
  samples_.back() = sample;
}

InputGate::InputGate(InputGateConfig config) : config_(config) {
  config_.odom_max_age_s = std::max(0.0, config_.odom_max_age_s);
  config_.tf_max_age_s = std::max(0.0, config_.tf_max_age_s);
  config_.cloud_max_age_s = std::max(0.0, config_.cloud_max_age_s);
  config_.traversability_max_age_s = std::max(0.0, config_.traversability_max_age_s);
  config_.local_collision_max_age_s = std::max(0.0, config_.local_collision_max_age_s);
  config_.localization_health_max_age_s = std::max(0.0, config_.localization_health_max_age_s);
  config_.driver_control_max_age_s = std::max(0.0, config_.driver_control_max_age_s);
  config_.odom_max_speed_mps = std::max(0.0, config_.odom_max_speed_mps);
  config_.recovery_frames = std::max<std::uint32_t>(1, config_.recovery_frames);
  config_.future_tolerance_s = std::max(0.0, config_.future_tolerance_s);
}

InputGateState InputGate::evaluate(const InputSnapshot &inputs) {
  InputGateState state;
  state.odom_age_s = config_.require_odom
                         ? age(inputs.now_s, receiveOrSource(inputs.odom_receive_s,
                                                             inputs.odom_stamp_s))
                         : -1.0;
  state.tf_age_s = config_.require_odom && inputs.odom_requires_tf
                       ? age(inputs.now_s,
                             receiveOrSource(inputs.tf_receive_s, inputs.tf_stamp_s))
                       : -1.0;
  state.cloud_age_s = config_.require_cloud
                          ? age(inputs.now_s, receiveOrSource(inputs.cloud_receive_s,
                                                              inputs.cloud_stamp_s))
                          : -1.0;
  state.traversability_age_s =
      config_.require_traversability
          ? age(inputs.now_s, receiveOrSource(inputs.traversability_receive_s,
                                              inputs.traversability_stamp_s))
          : -1.0;
  state.local_collision_age_s =
      config_.require_local_collision
          ? age(inputs.now_s, receiveOrSource(inputs.local_collision_receive_s,
                                              inputs.local_collision_stamp_s))
          : -1.0;
  state.localization_health_age_s =
      config_.require_localization_health
          ? age(inputs.now_s, receiveOrSource(inputs.localization_health_receive_s,
                                              inputs.localization_health_stamp_s))
          : -1.0;
  state.driver_control_age_s =
      config_.require_driver_control
          ? age(inputs.now_s, receiveOrSource(inputs.driver_control_receive_s,
                                              inputs.driver_control_stamp_s))
          : -1.0;
  state.localization_healthy = inputs.localization_healthy;
  state.localization_state = inputs.localization_state;
  state.localization_reason = inputs.localization_reason;
  state.driver_control_ready = inputs.driver_control_ready;
  state.driver_control_reason = inputs.driver_control_reason;
  state.odom_linear_speed_mps = inputs.odom_linear_speed_mps;

  const char *stop_reason = nullptr;
  if (!std::isfinite(inputs.now_s) || inputs.now_s <= 0.0) {
    stop_reason = "input_clock_invalid";
  } else if (config_.require_driver_control && inputs.driver_control_stamp_s <= 0.0) {
    stop_reason = "driver_control_missing";
  } else if (config_.require_driver_control &&
             state.driver_control_age_s < -config_.future_tolerance_s) {
    stop_reason = "driver_control_future";
  } else if (config_.require_driver_control && config_.driver_control_max_age_s > 0.0 &&
             state.driver_control_age_s > config_.driver_control_max_age_s) {
    stop_reason = "driver_control_stale";
  } else if (config_.require_driver_control && !inputs.driver_control_ready) {
    stop_reason = inputs.driver_control_reason.empty() ? "driver_control_not_ready"
                                                       : "driver_control_rejected";
  } else if (config_.require_odom && inputs.odom_stamp_s <= 0.0) {
    stop_reason = "odom_missing";
  } else if (config_.require_odom && state.odom_age_s < -config_.future_tolerance_s) {
    stop_reason = "odom_future";
  } else if (config_.require_odom && config_.odom_max_age_s > 0.0 &&
             state.odom_age_s > config_.odom_max_age_s) {
    stop_reason = "odom_stale";
  } else if (config_.require_odom && !std::isfinite(inputs.odom_linear_speed_mps)) {
    stop_reason = "odom_velocity_nonfinite";
  } else if (config_.require_odom && config_.odom_max_speed_mps > 0.0 &&
             inputs.odom_linear_speed_mps > config_.odom_max_speed_mps) {
    stop_reason = "odom_velocity_out_of_bounds";
  } else if (config_.require_odom && inputs.odom_requires_tf && inputs.tf_stamp_s <= 0.0) {
    stop_reason = "tf_missing";
  } else if (config_.require_odom && inputs.odom_requires_tf &&
             state.tf_age_s < -config_.future_tolerance_s) {
    stop_reason = "tf_future";
  } else if (config_.require_odom && inputs.odom_requires_tf && config_.tf_max_age_s > 0.0 &&
             state.tf_age_s > config_.tf_max_age_s) {
    stop_reason = "tf_stale";
  } else if (config_.require_cloud && inputs.cloud_stamp_s <= 0.0) {
    stop_reason = "cloud_missing";
  } else if (config_.require_cloud && state.cloud_age_s < -config_.future_tolerance_s) {
    stop_reason = "cloud_future";
  } else if (config_.require_cloud && config_.cloud_max_age_s > 0.0 &&
             state.cloud_age_s > config_.cloud_max_age_s) {
    stop_reason = "cloud_stale";
  } else if (config_.require_local_collision && inputs.local_collision_stamp_s <= 0.0) {
    stop_reason = "local_collision_missing";
  } else if (config_.require_local_collision &&
             state.local_collision_age_s < -config_.future_tolerance_s) {
    stop_reason = "local_collision_future";
  } else if (config_.require_local_collision && config_.local_collision_max_age_s > 0.0 &&
             state.local_collision_age_s > config_.local_collision_max_age_s) {
    stop_reason = "local_collision_stale";
  } else if (config_.require_local_collision && !inputs.local_collision_complete) {
    stop_reason = "local_collision_incomplete";
  } else if (config_.require_traversability && inputs.traversability_stamp_s <= 0.0) {
    stop_reason = "traversability_missing";
  } else if (config_.require_traversability &&
             state.traversability_age_s < -config_.future_tolerance_s) {
    stop_reason = "traversability_future";
  } else if (config_.require_traversability && config_.traversability_max_age_s > 0.0 &&
             state.traversability_age_s > config_.traversability_max_age_s) {
    stop_reason = "traversability_stale";
  } else if (config_.require_localization_health && inputs.localization_health_stamp_s <= 0.0) {
    stop_reason = "localization_health_missing";
  } else if (config_.require_localization_health &&
             state.localization_health_age_s < -config_.future_tolerance_s) {
    stop_reason = "localization_health_future";
  } else if (config_.require_localization_health && config_.localization_health_max_age_s > 0.0 &&
             state.localization_health_age_s > config_.localization_health_max_age_s) {
    stop_reason = "localization_health_stale";
  } else if (config_.require_localization_health &&
             isCatastrophicLocalizationReason(inputs.localization_reason)) {
    stop_reason = "localization_catastrophic";
  } else if (config_.require_localization_health && !inputs.localization_state.empty() &&
             !isHealthyLocalizationState(inputs.localization_state)) {
    stop_reason = "localization_not_tracking";
  } else if (config_.require_localization_health && !inputs.localization_healthy) {
    stop_reason = "localization_unhealthy";
  }

  if (stop_reason != nullptr) {
    fresh_frames_ = 0;
    recovery_generations_ = generations(inputs);
    state.reason = stop_reason;
    return state;
  }

  if (fresh_frames_ < config_.recovery_frames && allRequiredInputsAdvanced(inputs)) {
    fresh_frames_ = std::min<std::uint32_t>(config_.recovery_frames, fresh_frames_ + 1);
    recovery_generations_ = generations(inputs);
  }
  state.fresh_frames = fresh_frames_;
  state.ready = fresh_frames_ >= config_.recovery_frames;
  state.recovering = !state.ready;
  state.reason = state.ready ? "ready" : "recovering";
  return state;
}

void InputGate::reset() {
  fresh_frames_ = 0;
  recovery_generations_ = {};
}

void InputGate::beginRecoveryFrom(const InputSnapshot &inputs) {
  fresh_frames_ = 0;
  recovery_generations_ = generations(inputs);
}

InputGate::InputGenerations InputGate::generations(const InputSnapshot &inputs) {
  return {
      inputs.odom_generation,
      inputs.tf_generation,
      inputs.cloud_generation,
      inputs.traversability_generation,
      inputs.local_collision_generation,
      inputs.localization_health_generation,
      inputs.driver_control_generation,
  };
}

bool InputGate::allRequiredInputsAdvanced(const InputSnapshot &inputs) const {
  if (config_.require_odom && inputs.odom_generation <= recovery_generations_.odom) {
    return false;
  }
  if (config_.require_odom && inputs.odom_requires_tf &&
      inputs.tf_generation <= recovery_generations_.tf) {
    return false;
  }
  if (config_.require_cloud && inputs.cloud_generation <= recovery_generations_.cloud) {
    return false;
  }
  if (config_.require_traversability &&
      inputs.traversability_generation <= recovery_generations_.traversability) {
    return false;
  }
  if (config_.require_local_collision &&
      inputs.local_collision_generation <= recovery_generations_.local_collision) {
    return false;
  }
  if (config_.require_localization_health &&
      inputs.localization_health_generation <= recovery_generations_.localization_health) {
    return false;
  }
  if (config_.require_driver_control &&
      inputs.driver_control_generation <= recovery_generations_.driver_control) {
    return false;
  }
  return true;
}

double InputGate::age(double now_s, double last_s) {
  return last_s <= 0.0 ? std::numeric_limits<double>::infinity() : now_s - last_s;
}

double InputGate::receiveOrSource(double receive_s, double source_s) {
  return std::isfinite(receive_s) && receive_s > 0.0 ? receive_s : source_s;
}

}  // namespace lingtu::nav::endpoint

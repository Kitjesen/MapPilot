#include "lingtu/recording/recording_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <string_view>

namespace lingtu::recording {

BoundedMessageQueue::BoundedMessageQueue(std::size_t capacity_bytes)
    : capacity_bytes_(capacity_bytes) {
  if (capacity_bytes_ == 0) {
    throw std::invalid_argument("recording queue capacity must be positive");
  }
}

bool BoundedMessageQueue::try_push(RecordedMessage message) {
  const std::size_t size = message.payload.size();
  std::lock_guard<std::mutex> lock(mutex_);
  if (closed_ || size > capacity_bytes_ || queued_bytes_ > capacity_bytes_ - size) {
    ++dropped_messages_;
    return false;
  }
  queued_bytes_ += size;
  high_watermark_bytes_ = std::max(high_watermark_bytes_, queued_bytes_);
  messages_.push_back(std::move(message));
  ready_.notify_one();
  return true;
}

bool BoundedMessageQueue::pop(RecordedMessage &message) {
  std::unique_lock<std::mutex> lock(mutex_);
  ready_.wait(lock, [this] { return closed_ || !messages_.empty(); });
  if (messages_.empty()) {
    return false;
  }
  message = std::move(messages_.front());
  messages_.pop_front();
  queued_bytes_ -= message.payload.size();
  return true;
}

void BoundedMessageQueue::close() {
  std::lock_guard<std::mutex> lock(mutex_);
  closed_ = true;
  ready_.notify_all();
}

std::uint64_t BoundedMessageQueue::dropped_messages() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return dropped_messages_;
}

std::size_t BoundedMessageQueue::high_watermark_bytes() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return high_watermark_bytes_;
}

std::size_t BoundedMessageQueue::queued_bytes() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return queued_bytes_;
}

namespace {

std::string canonical_topic(std::string topic) {
  if (topic.rfind("rt/", 0) == 0) {
    return "/" + topic.substr(3);
  }
  if (topic.empty() || topic.front() != '/') {
    return "/" + topic;
  }
  return topic;
}

}  // namespace

std::filesystem::path recording_executable_path(const std::filesystem::path &invocation) {
  std::error_code error;
  auto executable = std::filesystem::canonical("/proc/self/exe", error);
  if (!error) {
    return executable;
  }

  error.clear();
  executable = std::filesystem::absolute(invocation, error);
  if (error) {
    throw std::runtime_error("failed to resolve recording executable path: " + error.message());
  }
  auto canonical = std::filesystem::weakly_canonical(executable, error);
  return error ? executable.lexically_normal() : canonical;
}

std::filesystem::path resolve_recording_idl(
    const std::filesystem::path &executable_path,
    const std::filesystem::path &compile_time_fallback) {
  std::vector<std::filesystem::path> candidates;
  if (const char *idl = std::getenv("LINGTU_RECORDING_IDL"); idl != nullptr && *idl != '\0') {
    candidates.emplace_back(idl);
  }
  if (const char *repository = std::getenv("LINGTU_REPO");
      repository != nullptr && *repository != '\0') {
    candidates.emplace_back(std::filesystem::path(repository) / "src" / "message" / "idl" /
                            "lingtu_slam.idl");
  }
  candidates.emplace_back(executable_path.parent_path() / ".." / ".." / "src" / "message" /
                          "idl" / "lingtu_slam.idl");
  candidates.push_back(compile_time_fallback);

  std::string checked;
  for (const auto &candidate : candidates) {
    if (candidate.empty()) {
      continue;
    }
    std::error_code error;
    auto absolute = candidate.is_absolute() ? candidate : std::filesystem::absolute(candidate, error);
    if (error) {
      continue;
    }
    absolute = absolute.lexically_normal();
    if (!checked.empty()) {
      checked += ", ";
    }
    checked += absolute.string();
    if (!std::filesystem::is_regular_file(absolute, error) || error) {
      continue;
    }
    auto canonical = std::filesystem::weakly_canonical(absolute, error);
    return error ? absolute : canonical;
  }
  throw std::runtime_error("could not resolve LingTu recording IDL; checked: " + checked);
}
DdsRecordingPlan dds_recording_plan(std::string_view preset,
                                    const std::vector<std::string> &explicit_topics) {
  auto plan = dds_recording_plan(preset);
  if (explicit_topics.empty()) {
    return plan;
  }
  plan.selected_topics = explicit_topics;
  plan.required_topics = explicit_topics;
  return plan;
}
DdsRecordingPlan dds_recording_plan(std::string_view preset) {
  if (preset == "inspection-evidence-v1") {
    return {
        {
            "/imu/raw",
            "/driver/odometry",
            "/driver/control_state",
            "/slam/odometry",
            "/slam/registered_cloud",
            "/nav/goal/status",
            "/nav/state",
            "/nav/operator_motion/status",
            "/nav/global_path",
            "/nav/local_path",
            "/nav/cmd_vel",
            "/nav/inspection/task/event",
            "/nav/inspection/evidence/result",
        },
        {
            "/slam/odometry",
            "/driver/control_state",
            "/nav/state",
            "/nav/inspection/task/event",
        },
    };
  }
  if (preset == "generic-sensors-v1") {
    return {
        {
            "/imu/raw",
            "/lidar/raw_frame",
            "/slam/odometry",
            "/slam/registered_cloud",
        },
        {
            "/imu/raw",
            "/lidar/raw_frame",
        },
    };
  }
  throw std::invalid_argument("unknown native DDS recording preset: " + std::string(preset));
}

bool is_sensor_replay_topic(const std::string &topic) {
  static constexpr std::string_view kAllowed[] = {
      "/tf",                    "/tf_static",       "/lidar/raw_frame",
      "/imu/raw",               "/driver/odometry", "/slam/odom_prior",
      "/slam/odometry",         "/slam/state_at_scan", "/slam/registered_cloud",
      "/slam/map_observation",  "/slam/map_cloud",  "/gnss/fix", "/gnss/odom",
  };
  const std::string normalized = canonical_topic(topic);
  return std::find(std::begin(kAllowed), std::end(kAllowed), normalized) != std::end(kAllowed);
}

std::string validate_replay_domain(int domain_id, bool allow_live_domain) {
  if (domain_id < 0 || domain_id > 232) {
    return "DDS replay domain must be between 0 and 232";
  }
  if (domain_id == 0 && !allow_live_domain) {
    return "DDS domain 0 is a live field domain; pass --allow-live-domain explicitly";
  }
  return {};
}

std::uint64_t replay_offset_ns(std::uint64_t base_log_time_ns, std::uint64_t message_log_time_ns,
                               double rate) {
  if (!std::isfinite(rate) || rate <= 0.0) {
    throw std::invalid_argument("replay rate must be finite and positive");
  }
  if (message_log_time_ns <= base_log_time_ns) {
    return 0;
  }
  const long double scaled = static_cast<long double>(message_log_time_ns - base_log_time_ns) /
                             static_cast<long double>(rate);
  if (scaled >= static_cast<long double>(std::numeric_limits<std::uint64_t>::max())) {
    return std::numeric_limits<std::uint64_t>::max();
  }
  return static_cast<std::uint64_t>(scaled);
}

}  // namespace lingtu::recording

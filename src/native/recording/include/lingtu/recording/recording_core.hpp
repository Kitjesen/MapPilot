#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <deque>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

namespace lingtu::recording {

struct RecordedMessage {
  std::string wire_topic;
  std::uint64_t log_time_ns{0};
  std::uint64_t publish_time_ns{0};
  std::uint32_t sequence{0};
  std::vector<std::byte> payload;
};

class BoundedMessageQueue {
 public:
  explicit BoundedMessageQueue(std::size_t capacity_bytes);

  BoundedMessageQueue(const BoundedMessageQueue &) = delete;
  BoundedMessageQueue &operator=(const BoundedMessageQueue &) = delete;

  bool try_push(RecordedMessage message);
  bool pop(RecordedMessage &message);
  void close();

  std::uint64_t dropped_messages() const;
  std::size_t high_watermark_bytes() const;
  std::size_t queued_bytes() const;

 private:
  const std::size_t capacity_bytes_;
  mutable std::mutex mutex_;
  std::condition_variable ready_;
  std::deque<RecordedMessage> messages_;
  std::size_t queued_bytes_{0};
  std::size_t high_watermark_bytes_{0};
  std::uint64_t dropped_messages_{0};
  bool closed_{false};
};

struct DdsRecordingPlan {
  std::vector<std::string> selected_topics;
  std::vector<std::string> required_topics;
};

DdsRecordingPlan dds_recording_plan(std::string_view preset);
DdsRecordingPlan dds_recording_plan(std::string_view preset,
                                    const std::vector<std::string> &explicit_topics);
std::filesystem::path recording_executable_path(const std::filesystem::path &invocation);
std::filesystem::path resolve_recording_idl(
    const std::filesystem::path &executable_path,
    const std::filesystem::path &compile_time_fallback);
bool is_sensor_replay_topic(const std::string &topic);
std::string validate_replay_domain(int domain_id, bool allow_live_domain);
std::uint64_t replay_offset_ns(std::uint64_t base_log_time_ns, std::uint64_t message_log_time_ns,
                               double rate);

}  // namespace lingtu::recording

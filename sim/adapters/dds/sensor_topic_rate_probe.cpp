#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "dds/dds.h"
#include "messages.h"
#include "message/cpp/qos.hpp"
#include "message/cpp/topics.hpp"

namespace {

using Clock = std::chrono::steady_clock;
constexpr auto kObserverPrewarmDuration = std::chrono::milliseconds(500);

dds_entity_t checked(dds_entity_t value, const char *operation) {
  if (value < 0) {
    throw std::runtime_error(std::string(operation) + ": " + dds_strretcode(-value));
  }
  return value;
}

dds_duration_t remaining(Clock::time_point deadline) {
  const auto now = Clock::now();
  if (now >= deadline)
    return 0;
  return static_cast<dds_duration_t>(std::max<std::int64_t>(
      1, std::chrono::duration_cast<std::chrono::nanoseconds>(deadline - now).count()));
}

enum class StreamKind { Imu, Lidar, Image };

struct Stream {
  const char *name;
  StreamKind kind;
  const lingtu::message::TopicContract *contract;
  const dds_topic_descriptor_t *descriptor;
  double expected_hz;
  std::uint32_t expected_width{0};
  std::uint32_t expected_height{0};
  const char *expected_encoding{nullptr};
  std::uint32_t expected_step{0};
  std::uint32_t expected_data_size{0};
  dds_entity_t observer_reader{0};
  dds_entity_t observer_condition{0};
  dds_entity_t production_reader{0};
  dds_entity_t production_condition{0};
  std::vector<Clock::time_point> arrivals;
  std::vector<std::uint64_t> source_timestamps_ns;
  std::vector<std::uint64_t> dds_write_timestamps_ns;
  std::vector<std::uint64_t> production_source_timestamps_ns;
  std::uint32_t observer_lost_baseline{0};
  std::uint32_t production_lost_baseline{0};
  std::uint32_t production_rejected_baseline{0};
};

dds_entity_t make_reader(dds_entity_t participant, const lingtu::message::TopicContract &contract,
                         const dds_topic_descriptor_t *descriptor, int history_depth = 0) {
  const auto topic = checked(
      dds_create_topic(participant, descriptor, contract.dds_topic.data(), nullptr, nullptr),
      "dds_create_topic");
  auto qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(contract.dds_topic));
  if (history_depth > 0) {
    // A rate observer must retain short publisher catch-up bursts. Reusing the
    // camera's latest-only consumer QoS here would make the probe itself infer
    // a source gap whenever Windows wakes the replay process a little late.
    dds_qset_history(qos.get(), DDS_HISTORY_KEEP_LAST, history_depth);
    dds_qset_resource_limits(qos.get(), history_depth, 1, history_depth);
  }
  return checked(dds_create_reader(participant, topic, qos.get(), nullptr), "dds_create_reader");
}

void wait_for_writers(dds_entity_t participant, const std::vector<Stream> &streams) {
  const auto waitset = checked(dds_create_waitset(participant), "dds_create_waitset(discovery)");
  for (std::size_t index = 0; index < streams.size(); ++index) {
    checked(dds_set_status_mask(streams[index].observer_reader, DDS_SUBSCRIPTION_MATCHED_STATUS),
            "dds_set_status_mask(subscription matched)");
    checked(dds_set_status_mask(streams[index].production_reader, DDS_SUBSCRIPTION_MATCHED_STATUS),
            "dds_set_status_mask(production subscription matched)");
    checked(dds_waitset_attach(waitset, streams[index].observer_reader,
                               static_cast<dds_attach_t>(index + 1)),
            "dds_waitset_attach(subscription matched)");
    checked(dds_waitset_attach(waitset, streams[index].production_reader,
                               static_cast<dds_attach_t>(streams.size() + index + 1)),
            "dds_waitset_attach(production subscription matched)");
  }
  const auto deadline = Clock::now() + std::chrono::seconds(5);
  while (Clock::now() < deadline) {
    bool all_matched = true;
    for (const auto &stream : streams) {
      dds_subscription_matched_status_t observer_status{};
      dds_subscription_matched_status_t production_status{};
      checked(dds_get_subscription_matched_status(stream.observer_reader, &observer_status),
              "dds_get_subscription_matched_status");
      checked(dds_get_subscription_matched_status(stream.production_reader, &production_status),
              "dds_get_subscription_matched_status(production)");
      all_matched =
          all_matched && observer_status.current_count > 0 && production_status.current_count > 0;
    }
    if (all_matched) {
      checked(dds_delete(waitset), "dds_delete(discovery waitset)");
      return;
    }
    std::vector<dds_attach_t> triggered(streams.size() * 2);
    const auto count =
        dds_waitset_wait(waitset, triggered.data(), triggered.size(), remaining(deadline));
    if (count < 0)
      throw std::runtime_error(dds_strretcode(-count));
    if (count == 0)
      break;
  }
  checked(dds_delete(waitset), "dds_delete(discovery waitset)");
  throw std::runtime_error("sensor DDS writer discovery timed out");
}

const lingtu_dds_Header &header_for(StreamKind kind, const void *sample) {
  switch (kind) {
    case StreamKind::Imu:
      return static_cast<const lingtu_dds_Imu *>(sample)->header;
    case StreamKind::Lidar:
      return static_cast<const lingtu_dds_LivoxFrame *>(sample)->header;
    case StreamKind::Image:
      return static_cast<const lingtu_dds_Image *>(sample)->header;
  }
  throw std::runtime_error("unsupported sensor stream kind");
}

std::uint64_t timestamp_ns(const lingtu_dds_Header &header) {
  if (header.stamp.sec < 0 || header.stamp.nanosec >= 1000000000U) {
    throw std::runtime_error("sensor sample has an invalid source timestamp");
  }
  return static_cast<std::uint64_t>(header.stamp.sec) * 1000000000ULL + header.stamp.nanosec;
}

void validate_payload(const Stream &stream, const void *sample) {
  if (stream.expected_encoding == nullptr)
    return;
  const auto &image = *static_cast<const lingtu_dds_Image *>(sample);
  if (image.width != stream.expected_width || image.height != stream.expected_height ||
      image.encoding == nullptr || std::strcmp(image.encoding, stream.expected_encoding) != 0 ||
      image.step != stream.expected_step || image.data._length != stream.expected_data_size) {
    throw std::runtime_error(std::string(stream.name) + " image payload contract mismatch");
  }
}

void drain_observer(Stream &stream) {
  while (true) {
    void *samples[64]{};
    dds_sample_info_t infos[64]{};
    const auto count = dds_take(stream.observer_reader, samples, infos, 64, 64);
    if (count < 0)
      throw std::runtime_error(dds_strretcode(-count));
    if (count == 0)
      return;
    for (dds_return_t index = 0; index < count; ++index) {
      if (infos[index].valid_data && samples[index] != nullptr) {
        if (infos[index].source_timestamp <= 0) {
          throw std::runtime_error(std::string(stream.name) +
                                   " sample has an invalid DDS write timestamp");
        }
        stream.arrivals.push_back(Clock::now());
        validate_payload(stream, samples[index]);
        stream.source_timestamps_ns.push_back(
            timestamp_ns(header_for(stream.kind, samples[index])));
        stream.dds_write_timestamps_ns.push_back(
            static_cast<std::uint64_t>(infos[index].source_timestamp));
      }
    }
    checked(dds_return_loan(stream.observer_reader, samples, count), "dds_return_loan");
  }
}

void drain_production(Stream &stream) {
  while (true) {
    void *samples[64]{};
    dds_sample_info_t infos[64]{};
    const auto count = dds_take(stream.production_reader, samples, infos, 64, 64);
    if (count < 0)
      throw std::runtime_error(dds_strretcode(-count));
    if (count == 0)
      return;
    for (dds_return_t index = 0; index < count; ++index) {
      if (infos[index].valid_data && samples[index] != nullptr) {
        validate_payload(stream, samples[index]);
        stream.production_source_timestamps_ns.push_back(
            timestamp_ns(header_for(stream.kind, samples[index])));
      }
    }
    checked(dds_return_loan(stream.production_reader, samples, count),
            "dds_return_loan(production)");
  }
}

void clear_metrics(Stream &stream) {
  stream.arrivals.clear();
  stream.source_timestamps_ns.clear();
  stream.dds_write_timestamps_ns.clear();
  stream.production_source_timestamps_ns.clear();
  dds_sample_lost_status_t observer_lost{};
  checked(dds_get_sample_lost_status(stream.observer_reader, &observer_lost),
          "dds_get_sample_lost_status(observer baseline)");
  stream.observer_lost_baseline = observer_lost.total_count;
  dds_sample_lost_status_t production_lost{};
  checked(dds_get_sample_lost_status(stream.production_reader, &production_lost),
          "dds_get_sample_lost_status(production baseline)");
  stream.production_lost_baseline = production_lost.total_count;
  dds_sample_rejected_status_t production_rejected{};
  checked(dds_get_sample_rejected_status(stream.production_reader, &production_rejected),
          "dds_get_sample_rejected_status(production baseline)");
  stream.production_rejected_baseline = production_rejected.total_count;
}

bool take_camera_info(dds_entity_t reader) {
  void *sample[1]{};
  dds_sample_info_t info[1]{};
  const auto taken = dds_take(reader, sample, info, 1, 1);
  if (taken < 0)
    throw std::runtime_error(dds_strretcode(-taken));
  const bool received = taken == 1 && info[0].valid_data && sample[0] != nullptr;
  if (taken > 0)
    checked(dds_return_loan(reader, sample, taken), "info loan");
  return received;
}

bool warm_up(dds_entity_t participant, dds_entity_t waitset, std::vector<Stream> &streams,
             dds_entity_t &info_reader) {
  const auto readiness_deadline = Clock::now() + std::chrono::seconds(5);
  std::optional<Clock::time_point> prewarm_deadline;
  bool info_reader_created = false;
  bool info_received = false;
  while (Clock::now() < (prewarm_deadline ? *prewarm_deadline : readiness_deadline)) {
    const auto deadline = prewarm_deadline ? *prewarm_deadline : readiness_deadline;
    std::vector<dds_attach_t> triggered(streams.size() * 2 + 1);
    const auto count =
        dds_waitset_wait(waitset, triggered.data(), triggered.size(), remaining(deadline));
    if (count < 0)
      throw std::runtime_error(dds_strretcode(-count));
    for (dds_return_t index = 0; index < count; ++index) {
      const auto tag = static_cast<std::size_t>(triggered[index]);
      if (tag >= 1 && tag <= streams.size()) {
        drain_observer(streams[tag - 1]);
      } else if (tag > streams.size() && tag <= streams.size() * 2) {
        drain_production(streams[tag - streams.size() - 1]);
      } else if (tag == streams.size() * 2 + 1 && info_reader > 0 &&
                 take_camera_info(info_reader)) {
        info_received = true;
      }
    }
    if (!info_reader_created &&
        std::all_of(streams.begin(), streams.end(), [](const Stream &stream) {
          return !stream.arrivals.empty() && !stream.production_source_timestamps_ns.empty();
        })) {
      info_reader =
          make_reader(participant, lingtu::message::kCameraInfo, &lingtu_dds_CameraInfo_desc);
      const auto info_condition = checked(dds_create_readcondition(info_reader, DDS_ANY_STATE),
                                          "camera info read condition");
      checked(dds_waitset_attach(waitset, info_condition,
                                 static_cast<dds_attach_t>(streams.size() * 2 + 1)),
              "camera info waitset attach");
      info_reader_created = true;
    }
    if (!prewarm_deadline && info_received &&
        std::all_of(streams.begin(), streams.end(), [](const Stream &stream) {
          return !stream.arrivals.empty() && !stream.production_source_timestamps_ns.empty();
        })) {
      prewarm_deadline = Clock::now() + kObserverPrewarmDuration;
      std::fprintf(stderr, "sensor topic observer prewarm: %lld ms\n",
                   static_cast<long long>(kObserverPrewarmDuration.count()));
    }
  }
  if (prewarm_deadline)
    return info_received;
  throw std::runtime_error("sensor stream and CameraInfo warm-up timed out");
}

double percentile95(std::vector<double> values) {
  if (values.empty())
    return 0.0;
  std::sort(values.begin(), values.end());
  const auto index =
      static_cast<std::size_t>(std::ceil(0.95 * static_cast<double>(values.size())) - 1.0);
  return values[std::min(index, values.size() - 1)];
}

std::vector<double> arrival_intervals(const Stream &stream) {
  std::vector<double> intervals;
  intervals.reserve(stream.arrivals.size());
  for (std::size_t index = 1; index < stream.arrivals.size(); ++index) {
    intervals.push_back(
        std::chrono::duration<double>(stream.arrivals[index] - stream.arrivals[index - 1]).count());
  }
  return intervals;
}

std::vector<double> timestamp_intervals(const std::vector<std::uint64_t> &timestamps,
                                        const char *label) {
  std::vector<double> intervals;
  intervals.reserve(timestamps.size());
  for (std::size_t index = 1; index < timestamps.size(); ++index) {
    if (timestamps[index] <= timestamps[index - 1]) {
      throw std::runtime_error(std::string(label) + " timestamps are not strictly increasing at " +
                               std::to_string(index) +
                               ": previous=" + std::to_string(timestamps[index - 1]) +
                               " current=" + std::to_string(timestamps[index]));
    }
    intervals.push_back(static_cast<double>(timestamps[index] - timestamps[index - 1]) /
                        1000000000.0);
  }
  return intervals;
}

double max_value(const std::vector<double> &values) {
  return values.empty() ? 0.0 : *std::max_element(values.begin(), values.end());
}

std::vector<double> jitter(const std::vector<double> &intervals, double expected_period) {
  std::vector<double> values;
  values.reserve(intervals.size());
  for (const double interval : intervals) {
    values.push_back(std::abs(interval - expected_period));
  }
  return values;
}

void print_metrics(const Stream &stream, std::uint64_t window_start_ns,
                   std::uint64_t window_end_ns) {
  const auto source = timestamp_intervals(stream.source_timestamps_ns,
                                          (std::string(stream.name) + " source").c_str());
  const auto dds_write = timestamp_intervals(stream.dds_write_timestamps_ns,
                                             (std::string(stream.name) + " DDS write").c_str());
  const auto take = arrival_intervals(stream);
  const double expected_period = 1.0 / stream.expected_hz;
  const double source_elapsed = stream.source_timestamps_ns.size() > 1
                                    ? static_cast<double>(stream.source_timestamps_ns.back() -
                                                          stream.source_timestamps_ns.front()) /
                                          1000000000.0
                                    : 0.0;
  const double source_hz =
      source_elapsed > 0.0
          ? static_cast<double>(stream.source_timestamps_ns.size() - 1) / source_elapsed
          : 0.0;
  const double dds_write_elapsed =
      stream.dds_write_timestamps_ns.size() > 1
          ? static_cast<double>(stream.dds_write_timestamps_ns.back() -
                                stream.dds_write_timestamps_ns.front()) /
                1000000000.0
          : 0.0;
  const double dds_write_actual_hz =
      dds_write_elapsed > 0.0
          ? static_cast<double>(stream.dds_write_timestamps_ns.size() - 1) / dds_write_elapsed
          : 0.0;
  const double expected_samples =
      source_elapsed > 0.0 ? std::round(source_elapsed * stream.expected_hz) + 1.0 : 1.0;
  const double inferred_drop_rate =
      expected_samples > 0.0
          ? std::max(0.0,
                     expected_samples - static_cast<double>(stream.source_timestamps_ns.size())) /
                expected_samples
          : 0.0;
  dds_sample_lost_status_t lost{};
  checked(dds_get_sample_lost_status(stream.observer_reader, &lost), "dds_get_sample_lost_status");
  dds_sample_lost_status_t production_lost{};
  checked(dds_get_sample_lost_status(stream.production_reader, &production_lost),
          "dds_get_sample_lost_status(production)");
  dds_sample_rejected_status_t production_rejected{};
  checked(dds_get_sample_rejected_status(stream.production_reader, &production_rejected),
          "dds_get_sample_rejected_status(production)");
  const auto observer_lost_in_window = lost.total_count - stream.observer_lost_baseline;
  const auto production_lost_in_window =
      production_lost.total_count - stream.production_lost_baseline;
  const auto production_rejected_in_window =
      production_rejected.total_count - stream.production_rejected_baseline;
  const double production_elapsed =
      stream.production_source_timestamps_ns.size() > 1
          ? static_cast<double>(stream.production_source_timestamps_ns.back() -
                                stream.production_source_timestamps_ns.front()) /
                1000000000.0
          : 0.0;
  const double expected_production_samples =
      production_elapsed > 0.0 ? std::round(production_elapsed * stream.expected_hz) + 1.0 : 1.0;
  const double production_inferred_drop_rate =
      expected_production_samples > 0.0
          ? std::max(0.0, expected_production_samples -
                              static_cast<double>(stream.production_source_timestamps_ns.size())) /
                expected_production_samples
          : 0.0;
  const double window_s = static_cast<double>(window_end_ns - window_start_ns) / 1000000000.0;
  const double first_sample_delay_s =
      stream.dds_write_timestamps_ns.empty()
          ? window_s
          : static_cast<double>(stream.dds_write_timestamps_ns.front() > window_start_ns
                                    ? stream.dds_write_timestamps_ns.front() - window_start_ns
                                    : 0ULL) /
                1000000000.0;
  const double tail_silence_s =
      stream.dds_write_timestamps_ns.empty()
          ? window_s
          : static_cast<double>(window_end_ns > stream.dds_write_timestamps_ns.back()
                                    ? window_end_ns - stream.dds_write_timestamps_ns.back()
                                    : 0ULL) /
                1000000000.0;
  const double expected_window_samples = window_s * stream.expected_hz;
  const double window_delivery_ratio =
      expected_window_samples > 0.0
          ? std::min(1.0, static_cast<double>(stream.dds_write_timestamps_ns.size()) /
                              expected_window_samples)
          : 0.0;
  const double production_window_delivery_ratio =
      expected_window_samples > 0.0
          ? std::min(1.0, static_cast<double>(stream.production_source_timestamps_ns.size()) /
                              expected_window_samples)
          : 0.0;
  std::printf(
      "{\"topic\":\"%s\",\"samples\":%zu,\"source_hz\":%.9f,"
      "\"source_max_interval_s\":%.12f,\"source_p95_jitter_s\":%.12f,"
      "\"source_inferred_drop_rate\":%.9f,\"dds_write_actual_hz\":%.9f,"
      "\"dds_write_max_interval_s\":%.9f,\"dds_write_p95_jitter_s\":%.9f,"
      "\"take_max_interval_s\":%.9f,\"take_p95_jitter_s\":%.9f,"
      "\"first_sample_delay_s\":%.9f,\"tail_silence_s\":%.9f,"
      "\"window_delivery_ratio\":%.9f,\"dds_sample_lost\":%u,"
      "\"production_samples\":%zu,\"production_source_inferred_drop_rate\":%.9f,"
      "\"production_window_delivery_ratio\":%.9f,\"production_dds_sample_lost\":%u,"
      "\"production_sample_rejected\":%u}\n",
      stream.contract->dds_topic.data(), stream.source_timestamps_ns.size(), source_hz,
      max_value(source), percentile95(jitter(source, expected_period)), inferred_drop_rate,
      dds_write_actual_hz, max_value(dds_write), percentile95(jitter(dds_write, expected_period)),
      max_value(take), percentile95(jitter(take, expected_period)), first_sample_delay_s,
      tail_silence_s, window_delivery_ratio, observer_lost_in_window,
      stream.production_source_timestamps_ns.size(), production_inferred_drop_rate,
      production_window_delivery_ratio, production_lost_in_window, production_rejected_in_window);
}

}  // namespace

int main(int argc, const char *const argv[]) {
  if (argc != 3 && argc != 4) {
    std::fputs("usage: sensor_topic_rate_probe DOMAIN_ID DURATION_SECONDS [camera]\n", stderr);
    return 2;
  }
  dds_entity_t participant = 0;
  try {
    const int domain_id = std::stoi(argv[1]);
    const double duration_s = std::stod(argv[2]);
    if (domain_id < 0 || duration_s <= 0.0 || !std::isfinite(duration_s)) {
      throw std::runtime_error("domain and duration must be positive");
    }
    const bool camera_only = argc == 4 && std::string(argv[3]) == "camera";
    if (argc == 4 && !camera_only) {
      throw std::runtime_error("probe mode must be camera");
    }
    participant =
        checked(dds_create_participant(static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
                "dds_create_participant");
    std::vector<Stream> streams;
    if (camera_only) {
      streams = {
          {"color", StreamKind::Image, &lingtu::message::kCameraColor, &lingtu_dds_Image_desc, 30.0,
           640, 480, "rgb8", 1920, 921600},
          {"depth", StreamKind::Image, &lingtu::message::kCameraDepth, &lingtu_dds_Image_desc, 30.0,
           640, 480, "16UC1", 1280, 614400},
      };
    } else {
      streams = {
          {"imu", StreamKind::Imu, &lingtu::message::kImuRaw, &lingtu_dds_Imu_desc, 200.0},
          {"lidar", StreamKind::Lidar, &lingtu::message::kLidarRawFrame,
           &lingtu_dds_LivoxFrame_desc, 10.0},
          {"color", StreamKind::Image, &lingtu::message::kCameraColor, &lingtu_dds_Image_desc,
           30.0},
          {"depth", StreamKind::Image, &lingtu::message::kCameraDepth, &lingtu_dds_Image_desc,
           30.0},
      };
    }
    const auto waitset = checked(dds_create_waitset(participant), "dds_create_waitset");
    for (std::size_t index = 0; index < streams.size(); ++index) {
      auto &stream = streams[index];
      stream.observer_reader =
          make_reader(participant, *stream.contract, stream.descriptor, camera_only ? 16 : 256);
      stream.observer_condition =
          checked(dds_create_readcondition(stream.observer_reader, DDS_ANY_STATE),
                  "dds_create_readcondition");
      stream.production_reader = make_reader(participant, *stream.contract, stream.descriptor);
      stream.production_condition =
          checked(dds_create_readcondition(stream.production_reader, DDS_ANY_STATE),
                  "dds_create_readcondition(production)");
      checked(dds_waitset_attach(waitset, stream.observer_condition,
                                 static_cast<dds_attach_t>(index + 1)),
              "dds_waitset_attach");
      checked(dds_waitset_attach(waitset, stream.production_condition,
                                 static_cast<dds_attach_t>(streams.size() + index + 1)),
              "dds_waitset_attach(production)");
    }
    const auto raw_packet_reader = camera_only
                                       ? 0
                                       : make_reader(participant, lingtu::message::kLidarRawPacket,
                                                     &lingtu_dds_LivoxFrame_desc);

    wait_for_writers(participant, streams);
    std::puts("READY");
    std::fflush(stdout);
    dds_entity_t info_reader = 0;
    const bool late_joiner_info = warm_up(participant, waitset, streams, info_reader);
    for (auto &stream : streams) {
      drain_observer(stream);
      drain_production(stream);
      clear_metrics(stream);
    }
    const auto window_start_ns = static_cast<std::uint64_t>(dds_time());
    const auto deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                             std::chrono::duration<double>(duration_s));
    while (Clock::now() < deadline) {
      std::vector<dds_attach_t> triggered(streams.size() * 2 + 1);
      const auto count =
          dds_waitset_wait(waitset, triggered.data(), triggered.size(), remaining(deadline));
      if (count < 0)
        throw std::runtime_error(dds_strretcode(-count));
      for (dds_return_t index = 0; index < count; ++index) {
        const auto tag = static_cast<std::size_t>(triggered[index]);
        if (tag >= 1 && tag <= streams.size()) {
          drain_observer(streams[tag - 1]);
        } else if (tag > streams.size() && tag <= streams.size() * 2) {
          drain_production(streams[tag - streams.size() - 1]);
        }
      }
    }
    for (auto &stream : streams) {
      drain_observer(stream);
      drain_production(stream);
    }
    const auto window_end_ns = static_cast<std::uint64_t>(dds_time());
    for (const auto &stream : streams)
      print_metrics(stream, window_start_ns, window_end_ns);
    if (!camera_only) {
      dds_subscription_matched_status_t raw_packet_match{};
      checked(dds_get_subscription_matched_status(raw_packet_reader, &raw_packet_match),
              "raw packet matched status");
      std::printf("{\"topic\":\"%s\",\"writer_count\":%u}\n",
                  lingtu::message::kLidarRawPacket.dds_topic.data(), raw_packet_match.total_count);
    }
    std::printf("{\"topic\":\"%s\",\"late_joiner_received\":%s}\n",
                lingtu::message::kCameraInfo.dds_topic.data(), late_joiner_info ? "true" : "false");
    dds_delete(participant);
    return 0;
  } catch (const std::exception &error) {
    if (participant > 0)
      dds_delete(participant);
    std::fprintf(stderr, "sensor topic rate probe failed: %s\n", error.what());
    return 1;
  }
}

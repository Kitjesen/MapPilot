#include "lingtu/recording/live_observer.hpp"

#include "lingtu/recording/topic_catalog.hpp"
#include "lingtu/recording/message_preview.hpp"

#include "dds/ddsi/ddsi_serdata.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string_view>

#include "messages.h"

namespace lingtu::recording {
namespace {
using Clock = std::chrono::steady_clock;

dds_entity_t checked(dds_return_t result, const char *operation) {
  if (result < 0)
    throw std::runtime_error(std::string(operation) + ": " + dds_strretcode(-result));
  return result;
}
}  // namespace

class LiveObserver::Impl {
 public:
  struct Stream {
    const TopicBinding *binding{};
    dds_entity_t reader{};
    ddsi_serdata *latest{};
    std::uint64_t count{}, bytes{}, previous_count{}, previous_bytes{};
    dds_time_t source_time{};
    Clock::time_point received{};
  };
  explicit Impl(int selected_domain, const std::vector<std::string> &names)
      : domain(selected_domain) {
    if (domain < 0 || domain > 232)
      throw std::invalid_argument("DDS domain must be between 0 and 232");
    participant =
        checked(dds_create_participant(static_cast<dds_domainid_t>(domain), nullptr, nullptr),
                "dds_create_participant");
    try {
      std::vector<const TopicBinding *> selected;
      if (names.empty())
        for (const auto &binding : recording_topic_catalog())
          selected.push_back(&binding);
      else
        for (const auto &name : names) {
          const auto *binding = find_recording_topic(name);
          if (!binding)
            throw std::invalid_argument("unknown observation topic: " + name);
          if (std::find(selected.begin(), selected.end(), binding) == selected.end())
            selected.push_back(binding);
        }
      for (const auto *binding : selected) {
        const std::string wire(binding->contract->dds_topic);
        const auto topic = checked(
            dds_create_topic(participant, binding->descriptor, wire.c_str(), nullptr, nullptr),
            "dds_create_topic");
        auto qos = lingtu::dds::make_qos(binding->qos_profile);
        streams.push_back(
            {binding, checked(dds_create_reader(participant, topic, qos.get(), nullptr),
                              "dds_create_reader")});
      }
    } catch (...) {
      dds_delete(participant);
      participant = 0;
      throw;
    }
  }
  ~Impl() {
    for (auto &stream : streams)
      if (stream.latest)
        ddsi_serdata_unref(stream.latest);
    if (participant > 0)
      dds_delete(participant);
  }
  void poll() {
    for (auto &stream : streams) {
      std::array<ddsi_serdata *, 64> data{};
      std::array<dds_sample_info_t, 64> infos{};
      const auto count = checked(
          dds_takecdr(stream.reader, data.data(), 64, infos.data(), DDS_ANY_STATE), "dds_takecdr");
      for (int i = 0; i < count; ++i) {
        if (!data[i])
          continue;
        if (infos[i].valid_data) {
          ++stream.count;
          stream.bytes += ddsi_serdata_size(data[i]);
          stream.received = Clock::now();
          stream.source_time = infos[i].source_timestamp;
          if (stream.latest)
            ddsi_serdata_unref(stream.latest);
          stream.latest = data[i];
        } else
          ddsi_serdata_unref(data[i]);
      }
    }
  }
  std::string snapshot() {
    const auto now = Clock::now();
    const auto seconds = std::chrono::duration<double>(now - previous_snapshot).count();
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << "{\"version\":1,\"ok\":true,\"domain\":" << domain << ",\"observed_at_unix_ns\":"
        << std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch())
               .count()
        << ",\"window_s\":" << json_number(seconds) << ",\"topics\":[";
    bool first = true;
    for (auto &stream : streams) {
      if (!first)
        out << ',';
      first = false;
      out << "{\"topic\":" << json_quote(stream.binding->contract->topic)
          << ",\"type_name\":" << json_quote(stream.binding->contract->idl_type)
          << ",\"message_count\":" << stream.count << ",\"bytes_received\":" << stream.bytes
          << ",\"rate_hz\":"
          << json_number(seconds > 0 ? (stream.count - stream.previous_count) / seconds : 0)
          << ",\"bytes_per_second\":"
          << json_number(seconds > 0 ? (stream.bytes - stream.previous_bytes) / seconds : 0)
          << ",\"last_sample_age_ms\":";
      if (stream.latest)
        out << json_number(std::chrono::duration<double, std::milli>(now - stream.received).count());
      else
        out << "null";
      out << ",\"source_time_ns\":";
      if (stream.latest && stream.source_time > 0)
        out << stream.source_time;
      else
        out << "null";
      out << ",\"preview\":" << (stream.latest ? message_preview(*stream.binding, stream.latest) : "null")
          << '}';
      stream.previous_count = stream.count;
      stream.previous_bytes = stream.bytes;
    }
    previous_snapshot = now;
    return out.str() + "]}";
  }
  int domain;
  dds_entity_t participant{};
  std::vector<Stream> streams;
  Clock::time_point previous_snapshot{Clock::now()};
};

LiveObserver::LiveObserver(int domain, const std::vector<std::string> &topics)
    : impl_(std::make_unique<Impl>(domain, topics)) {}
LiveObserver::~LiveObserver() = default;
void LiveObserver::poll() {
  impl_->poll();
}
std::string LiveObserver::snapshot() {
  return impl_->snapshot();
}
}  // namespace lingtu::recording

#include "client.hpp"
#include "clock_sync.hpp"

#include "message/cpp/dds_qos_profiles.hpp"
#include "message/cpp/dds_topics.hpp"
#include "message/cpp/exploration_command.hpp"
#include "message/cpp/inspection_command.hpp"
#include "message/cpp/navigation_command.hpp"

#include "dds/dds.h"
#include "lingtu_slam.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

#include <unistd.h>

namespace lingtu::nav::commands {
namespace {

using CommandKind = lingtu::message::NavigationCommandKind;
using ExplorationKind = lingtu::message::ExplorationCommandKind;
using lingtu::message::explorationCommandKindName;
using lingtu::message::navigationCommandKindName;

using SteadyClock = std::chrono::steady_clock;

// ACK.header.stamp is the endpoint's ACK publication time.  The client may
// only use it as a lower-bound clock-offset sample when the request/ACK round
// trip is small enough that the unknown return-path delay cannot consume the
// command freshness budget.  100 ms leaves at least 150 ms of the field
// profile's 250 ms teleop budget for command delivery and endpoint scheduling.
constexpr double kMaximumClockSampleRoundTripS = 0.10;

double nowSeconds() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

void fillStamp(lingtu_dds_Header& header, double stamp_s) {
  header.stamp.sec = static_cast<std::int32_t>(stamp_s);
  header.stamp.nanosec = static_cast<std::uint32_t>(
      (stamp_s - static_cast<double>(header.stamp.sec)) * 1e9);
}

void fillHeader(lingtu_dds_Header& header, double stamp_s, const char* frame_id) {
  fillStamp(header, stamp_s);
  header.frame_id = const_cast<char*>(frame_id);
}

double stampSeconds(const lingtu_dds_Time& stamp) {
  return static_cast<double>(stamp.sec) +
      static_cast<double>(stamp.nanosec) * 1e-9;
}

lingtu_dds_Quaternion quaternionFromYaw(double yaw) {
  lingtu_dds_Quaternion q{};
  const double half = yaw * 0.5;
  q.z = std::sin(half);
  q.w = std::cos(half);
  return q;
}

dds_entity_t checked(dds_return_t value, const char* operation) {
  if (value < 0) {
    throw std::runtime_error(
        std::string(operation) + ": " + dds_strretcode(-value));
  }
  return static_cast<dds_entity_t>(value);
}

void requireFinite(double value, const char* label) {
  if (!std::isfinite(value)) {
    throw std::invalid_argument(std::string(label) + " must be finite");
  }
}

std::string text(const char* value) {
  return value == nullptr ? std::string{} : std::string(value);
}

bool diagnosticsEnabled() {
  const char* raw = std::getenv("LINGTU_NAV_CLIENT_DIAGNOSTICS");
  if (raw == nullptr) {
    return false;
  }
  const std::string value(raw);
  return !value.empty() && value != "0" && value != "false" && value != "off";
}

std::string makeRequestId(CommandKind kind) {
  static std::atomic<std::uint64_t> sequence{0};
  const auto ticks = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  return "nav-" + std::to_string(static_cast<std::int32_t>(kind)) + "-" +
      std::to_string(static_cast<long long>(getpid())) + "-" +
      std::to_string(ticks) + "-" + std::to_string(++sequence);
}

std::string makeExplorationRequestId(ExplorationKind kind) {
  static std::atomic<std::uint64_t> sequence{0};
  const auto ticks = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  return "explore-" + std::to_string(static_cast<std::int32_t>(kind)) + "-" +
      std::to_string(static_cast<long long>(getpid())) + "-" +
      std::to_string(ticks) + "-" + std::to_string(++sequence);
}

std::string makeInspectionRequestId(
    lingtu::message::InspectionCommandKind kind) {
  static std::atomic<std::uint64_t> sequence{0};
  const auto ticks = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  return "inspection-" + std::to_string(static_cast<std::int32_t>(kind)) + "-" +
      std::to_string(static_cast<long long>(getpid())) + "-" +
      std::to_string(ticks) + "-" + std::to_string(++sequence);
}

bool isRecoverableClockRejection(CommandKind kind, const std::string& reason) {
  if (kind == CommandKind::Teleop) {
    return reason == "teleop_source_stamp_stale" ||
        reason == "teleop_source_stamp_future";
  }
  if (kind == CommandKind::ClearEstop) {
    return reason == "clear_estop_source_stamp_stale" ||
        reason == "clear_estop_source_stamp_future";
  }
  if (kind == CommandKind::ResumeAutonomy) {
    return reason == "resume_autonomy_source_stamp_stale" ||
        reason == "resume_autonomy_source_stamp_future";
  }
  return false;
}

}  // namespace

struct Client::Impl {
  struct AckObservation {
    bool accepted{false};
    std::string reason;
    double endpoint_stamp_s{0.0};
    double local_receive_wall_s{0.0};
    SteadyClock::time_point received_steady{};
    double round_trip_s{0.0};
  };

  struct PendingNavigationAck {
    CommandKind kind;
    SteadyClock::time_point sent_steady;
    std::optional<AckObservation> observation;
  };

  struct PendingExplorationAck {
    ExplorationKind kind;
    SteadyClock::time_point sent_steady;
    std::optional<AckObservation> observation;
  };

  struct PendingInspectionAck {
    lingtu::message::InspectionCommandKind kind;
    SteadyClock::time_point sent_steady;
    std::optional<AckObservation> observation;
  };

  explicit Impl(int domain_id) {
    participant = checked(
        dds_create_participant(
            static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
        "dds_create_participant(nav_client)");
    try {
      publisher = checked(
          dds_create_publisher(participant, nullptr, nullptr),
          "dds_create_publisher(nav_client)");
      subscriber = checked(
          dds_create_subscriber(participant, nullptr, nullptr),
          "dds_create_subscriber(nav_client)");
      command_writer = createWriter(
          lingtu::message::kNavCommandRequest,
          &lingtu_dds_NavigationCommandRequest_desc,
          "nav_command_request");
      ack_reader = createReader(
          lingtu::message::kNavCommandAck,
          &lingtu_dds_NavigationCommandAck_desc,
          "nav_command_ack");
      exploration_writer = createWriter(
          lingtu::message::kNavExplorationCommand,
          &lingtu_dds_ExplorationCommandRequest_desc,
          "exploration_command");
      exploration_ack_reader = createReader(
          lingtu::message::kNavExplorationAck,
          &lingtu_dds_ExplorationCommandAck_desc,
          "exploration_ack");
      inspection_writer = createWriter(
          lingtu::message::kNavInspectionCommand,
          &lingtu_dds_InspectionCommandRequest_desc,
          "inspection_command");
      inspection_ack_reader = createReader(
          lingtu::message::kNavInspectionAck,
          &lingtu_dds_InspectionCommandAck_desc,
          "inspection_ack");
      ack_receiver = std::thread([this]() { ackReceiverLoop(); });
    } catch (...) {
      dds_delete(participant);
      participant = 0;
      throw;
    }
  }

  ~Impl() {
    ack_receiver_running.store(false, std::memory_order_release);
    if (ack_receiver.joinable()) {
      ack_receiver.join();
    }
    if (participant > 0) {
      dds_delete(participant);
    }
  }

  dds_entity_t createWriter(
      const lingtu::message::TopicContract& contract,
      const dds_topic_descriptor_t* descriptor,
      const char* label) {
    const dds_entity_t topic = checked(
        dds_create_topic(
            participant,
            descriptor,
            contract.dds_topic.data(),
            nullptr,
            nullptr),
        (std::string("dds_create_topic(") + label + ")").c_str());
    auto qos = lingtu::dds::make_qos(
        lingtu::dds::qos_for_topic(contract.dds_topic));
    return checked(
        dds_create_writer(publisher, topic, qos.get(), nullptr),
        (std::string("dds_create_writer(") + label + ")").c_str());
  }

  dds_entity_t createReader(
      const lingtu::message::TopicContract& contract,
      const dds_topic_descriptor_t* descriptor,
      const char* label) {
    const dds_entity_t topic = checked(
        dds_create_topic(
            participant,
            descriptor,
            contract.dds_topic.data(),
            nullptr,
            nullptr),
        (std::string("dds_create_topic(") + label + ")").c_str());
    auto qos = lingtu::dds::make_qos(
        lingtu::dds::qos_for_topic(contract.dds_topic));
    return checked(
        dds_create_reader(subscriber, topic, qos.get(), nullptr),
        (std::string("dds_create_reader(") + label + ")").c_str());
  }

  void waitForReader(
      dds_entity_t writer,
      const char* label,
      int timeout_ms) const {
    const auto deadline = SteadyClock::now() +
        std::chrono::milliseconds(std::max(0, timeout_ms));
    do {
      const dds_return_t count =
          dds_get_matched_subscriptions(writer, nullptr, 0);
      if (count > 0) {
        return;
      }
      if (count < 0) {
        throw std::runtime_error(
            std::string("dds_get_matched_subscriptions(") + label + "): " +
            dds_strretcode(-count));
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } while (SteadyClock::now() < deadline);
    throw std::runtime_error(std::string("no matched DDS reader for ") + label);
  }

  void waitForWriter(
      dds_entity_t reader,
      const char* label,
      int timeout_ms) const {
    const auto deadline = SteadyClock::now() +
        std::chrono::milliseconds(std::max(0, timeout_ms));
    do {
      const dds_return_t count = dds_get_matched_publications(reader, nullptr, 0);
      if (count > 0) {
        return;
      }
      if (count < 0) {
        throw std::runtime_error(
            std::string("dds_get_matched_publications(") + label + "): " +
            dds_strretcode(-count));
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } while (SteadyClock::now() < deadline);
    throw std::runtime_error(std::string("no matched DDS writer for ") + label);
  }

  bool updateEndpointClock(const AckObservation& observation) const {
    if (!std::isfinite(observation.endpoint_stamp_s) ||
        observation.endpoint_stamp_s <= 0.0 ||
        !std::isfinite(observation.local_receive_wall_s) ||
        observation.local_receive_wall_s <= 0.0 ||
        !std::isfinite(observation.round_trip_s) ||
        observation.round_trip_s < 0.0 ||
        observation.round_trip_s > kMaximumClockSampleRoundTripS) {
      return false;
    }
    endpoint_clock_offset_s.store(
        endpointClockOffset(
            observation.endpoint_stamp_s,
            observation.local_receive_wall_s),
        std::memory_order_relaxed);
    return true;
  }

  double endpointClockOffsetOrZero() const {
    const double offset = endpoint_clock_offset_s.load(std::memory_order_relaxed);
    return std::isfinite(offset) ? offset : 0.0;
  }

  std::string rejectionMessage(
      const std::string& request_id,
      CommandKind kind,
      const AckObservation& observation) const {
    std::ostringstream out;
    out << std::fixed << std::setprecision(9)
        << "navigation command rejected: "
        << (observation.reason.empty() ? "unspecified" : observation.reason)
        << " [request_id=" << request_id
        << " kind=" << navigationCommandKindName(kind)
        << " sync_rtt_ms="
        << last_sync_rtt_s.load(std::memory_order_relaxed) * 1000.0
        << " endpoint_stamp_s="
        << last_sync_endpoint_stamp_s.load(std::memory_order_relaxed)
        << " local_wall_receive_s="
        << last_sync_local_receive_wall_s.load(std::memory_order_relaxed)
        << " clock_offset_s="
        << last_send_clock_offset_s.load(std::memory_order_relaxed)
        << " send_source_stamp_s="
        << last_send_source_stamp_s.load(std::memory_order_relaxed)
        << " post_ack_clock_offset_s=" << endpointClockOffsetOrZero()
        << " ack_rtt_ms=" << observation.round_trip_s * 1000.0
        << " ack_endpoint_stamp_s=" << observation.endpoint_stamp_s
        << " ack_local_wall_receive_s=" << observation.local_receive_wall_s
        << ']';
    return out.str();
  }

  void logClockEvent(
      const char* event,
      const std::string& request_id,
      CommandKind kind,
      const AckObservation* observation,
      double send_source_stamp_s) const {
    if (!diagnostics_enabled) {
      return;
    }
    std::fprintf(
        stderr,
        "nav_client_clock: event=%s request_id=%s kind=%s sync_rtt_ms=%.3f endpoint_stamp_s=%.9f local_wall_receive_s=%.9f clock_offset_s=%.9f send_source_stamp_s=%.9f ack_rtt_ms=%.3f ack_endpoint_stamp_s=%.9f ack_local_wall_receive_s=%.9f\n",
        event,
        request_id.c_str(),
        navigationCommandKindName(kind),
        last_sync_rtt_s.load(std::memory_order_relaxed) * 1000.0,
        last_sync_endpoint_stamp_s.load(std::memory_order_relaxed),
        last_sync_local_receive_wall_s.load(std::memory_order_relaxed),
        send_source_stamp_s > 0.0
            ? last_send_clock_offset_s.load(std::memory_order_relaxed)
            : endpointClockOffsetOrZero(),
        send_source_stamp_s,
        observation == nullptr ? -1.0 : observation->round_trip_s * 1000.0,
        observation == nullptr ? 0.0 : observation->endpoint_stamp_s,
        observation == nullptr ? 0.0 : observation->local_receive_wall_s);
  }

  double sourceNowSeconds() const {
    const double local_wall_s = nowSeconds();
    const double offset = endpoint_clock_offset_s.load(std::memory_order_relaxed);
    return std::isfinite(offset)
        ? endpointSourceTime(local_wall_s, offset)
        : local_wall_s;
  }

  using NavigationPending = std::shared_ptr<PendingNavigationAck>;
  using ExplorationPending = std::shared_ptr<PendingExplorationAck>;
  using InspectionPending = std::shared_ptr<PendingInspectionAck>;

  NavigationPending registerNavigationAck(
      const std::string& request_id,
      CommandKind kind,
      SteadyClock::time_point sent_steady) const {
    auto pending = std::make_shared<PendingNavigationAck>(
        PendingNavigationAck{kind, sent_steady, std::nullopt});
    std::lock_guard<std::mutex> lock(ack_mutex);
    if (!ack_receiver_error.empty()) {
      throw std::runtime_error(ack_receiver_error);
    }
    if (!pending_navigation_acks.emplace(request_id, pending).second) {
      throw std::runtime_error(
          "duplicate in-flight navigation request id: " + request_id);
    }
    return pending;
  }

  ExplorationPending registerExplorationAck(
      const std::string& request_id,
      ExplorationKind kind,
      SteadyClock::time_point sent_steady) const {
    auto pending = std::make_shared<PendingExplorationAck>(
        PendingExplorationAck{kind, sent_steady, std::nullopt});
    std::lock_guard<std::mutex> lock(ack_mutex);
    if (!ack_receiver_error.empty()) {
      throw std::runtime_error(ack_receiver_error);
    }
    if (!pending_exploration_acks.emplace(request_id, pending).second) {
      throw std::runtime_error(
          "duplicate in-flight exploration request id: " + request_id);
    }
    return pending;
  }

  InspectionPending registerInspectionAck(
      const std::string& request_id,
      lingtu::message::InspectionCommandKind kind,
      SteadyClock::time_point sent_steady) const {
    auto pending = std::make_shared<PendingInspectionAck>(
        PendingInspectionAck{kind, sent_steady, std::nullopt});
    std::lock_guard<std::mutex> lock(ack_mutex);
    if (!ack_receiver_error.empty()) {
      throw std::runtime_error(ack_receiver_error);
    }
    if (!pending_inspection_acks.emplace(request_id, pending).second) {
      throw std::runtime_error(
          "duplicate in-flight inspection request id: " + request_id);
    }
    return pending;
  }

  void unregisterNavigationAck(
      const std::string& request_id,
      const NavigationPending& pending) const {
    std::lock_guard<std::mutex> lock(ack_mutex);
    const auto it = pending_navigation_acks.find(request_id);
    if (it != pending_navigation_acks.end() && it->second == pending) {
      pending_navigation_acks.erase(it);
    }
  }

  void unregisterExplorationAck(
      const std::string& request_id,
      const ExplorationPending& pending) const {
    std::lock_guard<std::mutex> lock(ack_mutex);
    const auto it = pending_exploration_acks.find(request_id);
    if (it != pending_exploration_acks.end() && it->second == pending) {
      pending_exploration_acks.erase(it);
    }
  }

  void unregisterInspectionAck(
      const std::string& request_id,
      const InspectionPending& pending) const {
    std::lock_guard<std::mutex> lock(ack_mutex);
    const auto it = pending_inspection_acks.find(request_id);
    if (it != pending_inspection_acks.end() && it->second == pending) {
      pending_inspection_acks.erase(it);
    }
  }

  AckObservation waitForAck(
      const std::string& request_id,
      const NavigationPending& pending,
      int timeout_ms) const {
    const auto deadline = SteadyClock::now() +
        std::chrono::milliseconds(std::max(1, timeout_ms));
    std::unique_lock<std::mutex> lock(ack_mutex);
    const bool ready = ack_cv.wait_until(lock, deadline, [&]() {
      return pending->observation.has_value() || !ack_receiver_error.empty();
    });
    const auto it = pending_navigation_acks.find(request_id);
    if (it != pending_navigation_acks.end() && it->second == pending) {
      pending_navigation_acks.erase(it);
    }
    if (!ready) {
      throw std::runtime_error(
          "timed out waiting for navigation command ACK: " + request_id);
    }
    if (!pending->observation) {
      throw std::runtime_error(ack_receiver_error);
    }
    return *pending->observation;
  }

  AckObservation waitForExplorationAck(
      const std::string& request_id,
      const ExplorationPending& pending,
      int timeout_ms) const {
    const auto deadline = SteadyClock::now() +
        std::chrono::milliseconds(std::max(1, timeout_ms));
    std::unique_lock<std::mutex> lock(ack_mutex);
    const bool ready = ack_cv.wait_until(lock, deadline, [&]() {
      return pending->observation.has_value() || !ack_receiver_error.empty();
    });
    const auto it = pending_exploration_acks.find(request_id);
    if (it != pending_exploration_acks.end() && it->second == pending) {
      pending_exploration_acks.erase(it);
    }
    if (!ready) {
      throw std::runtime_error(
          "timed out waiting for exploration command ACK: " + request_id);
    }
    if (!pending->observation) {
      throw std::runtime_error(ack_receiver_error);
    }
    return *pending->observation;
  }

  AckObservation waitForInspectionAck(
      const std::string& request_id,
      const InspectionPending& pending,
      int timeout_ms) const {
    const auto deadline = SteadyClock::now() +
        std::chrono::milliseconds(std::max(1, timeout_ms));
    std::unique_lock<std::mutex> lock(ack_mutex);
    const bool ready = ack_cv.wait_until(lock, deadline, [&]() {
      return pending->observation.has_value() || !ack_receiver_error.empty();
    });
    const auto it = pending_inspection_acks.find(request_id);
    if (it != pending_inspection_acks.end() && it->second == pending) {
      pending_inspection_acks.erase(it);
    }
    if (!ready) {
      throw std::runtime_error(
          "timed out waiting for inspection command ACK: " + request_id);
    }
    if (!pending->observation) {
      throw std::runtime_error(ack_receiver_error);
    }
    return *pending->observation;
  }

  bool takeNavigationAcks() {
    constexpr std::size_t kMaxSamples = 16;
    void* samples[kMaxSamples]{};
    dds_sample_info_t infos[kMaxSamples]{};
    const dds_return_t count =
        dds_take(ack_reader, samples, infos, kMaxSamples, kMaxSamples);
    if (count < 0) {
      throw std::runtime_error(
          std::string("dds_take(nav_command_ack): ") + dds_strretcode(-count));
    }
    bool matched = false;
    const auto received_steady = SteadyClock::now();
    const double local_receive_wall_s = nowSeconds();
    {
      std::lock_guard<std::mutex> lock(ack_mutex);
      for (dds_return_t i = 0; i < count; ++i) {
        if (!infos[i].valid_data) {
          continue;
        }
        const auto& ack =
            *static_cast<lingtu_dds_NavigationCommandAck*>(samples[i]);
        const auto it = pending_navigation_acks.find(text(ack.request_id));
        if (it == pending_navigation_acks.end() || it->second->observation) {
          continue;
        }
        auto& pending = *it->second;
        AckObservation observation{
            ack.accepted,
            text(ack.reason),
            stampSeconds(ack.header.stamp),
            local_receive_wall_s,
            received_steady,
            std::chrono::duration<double>(
                received_steady - pending.sent_steady).count(),
        };
        if (ack.kind != static_cast<std::int32_t>(pending.kind)) {
          observation.accepted = false;
          observation.reason = "command_ack_kind_mismatch";
        }
        pending.observation = std::move(observation);
        matched = true;
      }
    }
    if (count > 0) {
      checked(
          dds_return_loan(ack_reader, samples, count),
          "dds_return_loan(nav_command_ack)");
    }
    if (matched) {
      ack_cv.notify_all();
    }
    return count > 0;
  }

  bool takeExplorationAcks() {
    constexpr std::size_t kMaxSamples = 16;
    void* samples[kMaxSamples]{};
    dds_sample_info_t infos[kMaxSamples]{};
    const dds_return_t count = dds_take(
        exploration_ack_reader, samples, infos, kMaxSamples, kMaxSamples);
    if (count < 0) {
      throw std::runtime_error(
          std::string("dds_take(exploration_ack): ") + dds_strretcode(-count));
    }
    bool matched = false;
    const auto received_steady = SteadyClock::now();
    const double local_receive_wall_s = nowSeconds();
    {
      std::lock_guard<std::mutex> lock(ack_mutex);
      for (dds_return_t i = 0; i < count; ++i) {
        if (!infos[i].valid_data) {
          continue;
        }
        const auto& ack =
            *static_cast<lingtu_dds_ExplorationCommandAck*>(samples[i]);
        const auto it = pending_exploration_acks.find(text(ack.request_id));
        if (it == pending_exploration_acks.end() || it->second->observation) {
          continue;
        }
        auto& pending = *it->second;
        AckObservation observation{
            ack.accepted,
            text(ack.reason),
            stampSeconds(ack.header.stamp),
            local_receive_wall_s,
            received_steady,
            std::chrono::duration<double>(
                received_steady - pending.sent_steady).count(),
        };
        if (ack.kind != static_cast<std::int32_t>(pending.kind)) {
          observation.accepted = false;
          observation.reason = "exploration_ack_kind_mismatch";
        }
        pending.observation = std::move(observation);
        matched = true;
      }
    }
    if (count > 0) {
      checked(
          dds_return_loan(exploration_ack_reader, samples, count),
          "dds_return_loan(exploration_ack)");
    }
    if (matched) {
      ack_cv.notify_all();
    }
    return count > 0;
  }

  bool takeInspectionAcks() {
    constexpr std::size_t kMaxSamples = 16;
    void* samples[kMaxSamples]{};
    dds_sample_info_t infos[kMaxSamples]{};
    const dds_return_t count = dds_take(
        inspection_ack_reader, samples, infos, kMaxSamples, kMaxSamples);
    if (count < 0) {
      throw std::runtime_error(
          std::string("dds_take(inspection_ack): ") + dds_strretcode(-count));
    }
    bool matched = false;
    const auto received_steady = SteadyClock::now();
    const double local_receive_wall_s = nowSeconds();
    {
      std::lock_guard<std::mutex> lock(ack_mutex);
      for (dds_return_t i = 0; i < count; ++i) {
        if (!infos[i].valid_data) {
          continue;
        }
        const auto& ack =
            *static_cast<lingtu_dds_InspectionCommandAck*>(samples[i]);
        const auto it = pending_inspection_acks.find(text(ack.request_id));
        if (it == pending_inspection_acks.end() || it->second->observation) {
          continue;
        }
        auto& pending = *it->second;
        AckObservation observation{
            ack.accepted,
            text(ack.reason),
            stampSeconds(ack.header.stamp),
            local_receive_wall_s,
            received_steady,
            std::chrono::duration<double>(
                received_steady - pending.sent_steady).count(),
        };
        if (ack.kind != static_cast<std::int32_t>(pending.kind)) {
          observation.accepted = false;
          observation.reason = "inspection_ack_kind_mismatch";
        }
        pending.observation = std::move(observation);
        matched = true;
      }
    }
    if (count > 0) {
      checked(
          dds_return_loan(inspection_ack_reader, samples, count),
          "dds_return_loan(inspection_ack)");
    }
    if (matched) {
      ack_cv.notify_all();
    }
    return count > 0;
  }

  void failAckReceiver(const std::string& reason) noexcept {
    {
      std::lock_guard<std::mutex> lock(ack_mutex);
      if (ack_receiver_error.empty()) {
        ack_receiver_error = "navigation ACK receiver failed: " + reason;
      }
    }
    ack_cv.notify_all();
  }

  void ackReceiverLoop() noexcept {
    while (ack_receiver_running.load(std::memory_order_acquire)) {
      try {
        const bool received =
            takeNavigationAcks() | takeExplorationAcks() | takeInspectionAcks();
        if (!received) {
          std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
      } catch (const std::exception& exc) {
        failAckReceiver(exc.what());
        return;
      } catch (...) {
        failAckReceiver("unknown error");
        return;
      }
    }
  }

  void writeExplorationCommand(
      ExplorationKind kind,
      const std::string& session_id,
      const std::string& requested_reason,
      int timeout_ms,
      const std::string& requested_id) const {
    const std::string request_id = requested_id.empty()
        ? makeExplorationRequestId(kind)
        : requested_id;
    const std::string reason = requested_reason.empty()
        ? explorationCommandKindName(kind)
        : requested_reason;
    waitForReader(exploration_writer, "exploration_command", timeout_ms);
    waitForWriter(exploration_ack_reader, "exploration_ack", timeout_ms);
    lingtu_dds_ExplorationCommandRequest message{};
    fillHeader(message.header, nowSeconds(), "map");
    message.request_id = const_cast<char*>(request_id.c_str());
    message.kind = static_cast<std::int32_t>(kind);
    message.session_id = const_cast<char*>(session_id.c_str());
    message.reason = const_cast<char*>(reason.c_str());
    const auto sent = SteadyClock::now();
    const auto pending = registerExplorationAck(request_id, kind, sent);
    try {
      checked(
          dds_write(exploration_writer, static_cast<const void*>(&message)),
          "dds_write(exploration_command)");
    } catch (...) {
      unregisterExplorationAck(request_id, pending);
      throw;
    }
    const auto observation = waitForExplorationAck(
        request_id, pending, timeout_ms);
    if (!observation.accepted) {
      throw std::runtime_error(
          "exploration command rejected: " +
          (observation.reason.empty() ? std::string("unspecified") : observation.reason) +
          " [request_id=" + request_id +
          " kind=" + explorationCommandKindName(kind) + "]");
    }
  }

  void writeInspectionCommand(
      lingtu::message::InspectionCommandKind kind,
      const std::string& route_id,
      std::uint64_t route_revision,
      const std::string& reason,
      int timeout_ms,
      const std::string& requested_id) const {
    const std::string request_id = requested_id.empty()
        ? makeInspectionRequestId(kind)
        : requested_id;
    waitForReader(inspection_writer, "inspection_command", timeout_ms);
    waitForWriter(inspection_ack_reader, "inspection_ack", timeout_ms);
    lingtu_dds_InspectionCommandRequest message{};
    fillHeader(message.header, nowSeconds(), "map");
    message.request_id = const_cast<char*>(request_id.c_str());
    message.kind = static_cast<std::int32_t>(kind);
    message.route_id = const_cast<char*>(route_id.c_str());
    message.route_revision = route_revision;
    message.reason = const_cast<char*>(reason.c_str());
    const auto sent = SteadyClock::now();
    const auto pending = registerInspectionAck(request_id, kind, sent);
    try {
      checked(
          dds_write(inspection_writer, static_cast<const void*>(&message)),
          "dds_write(inspection_command)");
    } catch (...) {
      unregisterInspectionAck(request_id, pending);
      throw;
    }
    const auto observation = waitForInspectionAck(request_id, pending, timeout_ms);
    if (!observation.accepted) {
      throw std::runtime_error(
          "inspection command rejected: " +
          (observation.reason.empty() ? std::string("unspecified") : observation.reason) +
          " [request_id=" + request_id +
          " kind=" + lingtu::message::inspectionCommandKindName(kind) + "]");
    }
  }

  static bool requiresEndpointClock(CommandKind kind) {
    return kind == CommandKind::Teleop || kind == CommandKind::ClearEstop ||
        kind == CommandKind::ResumeAutonomy;
  }

  void synchronizeEndpointClock(int timeout_ms) const {
    const auto deadline = SteadyClock::now() +
        std::chrono::milliseconds(std::max(1, timeout_ms));
    double last_round_trip_s = 0.0;
    while (SteadyClock::now() < deadline) {
      const std::string request_id = makeRequestId(CommandKind::Stop);
      lingtu_dds_NavigationCommandRequest sync{};
      fillHeader(sync.header, nowSeconds(), "body");
      sync.request_id = const_cast<char*>(request_id.c_str());
      sync.kind = static_cast<std::int32_t>(CommandKind::Stop);
      sync.reason = const_cast<char*>("client_clock_sync");
      const auto sent_steady = SteadyClock::now();
      const auto pending = registerNavigationAck(
          request_id, CommandKind::Stop, sent_steady);
      try {
        checked(
            dds_write(command_writer, static_cast<const void*>(&sync)),
            "dds_write(nav_command_clock_sync)");
      } catch (...) {
        unregisterNavigationAck(request_id, pending);
        throw;
      }
      const auto remaining_ms = std::max<long long>(
          1,
          std::chrono::duration_cast<std::chrono::milliseconds>(
              deadline - SteadyClock::now())
              .count());
      const auto observation = waitForAck(
          request_id, pending, static_cast<int>(remaining_ms));
      if (!observation.accepted) {
        logClockEvent(
            "sync_rejected", request_id, CommandKind::Stop, &observation, 0.0);
        throw std::runtime_error(
            rejectionMessage(request_id, CommandKind::Stop, observation));
      }
      last_round_trip_s = observation.round_trip_s;
      const bool usable = updateEndpointClock(observation);
      if (usable) {
        last_sync_rtt_s.store(
            observation.round_trip_s, std::memory_order_relaxed);
        last_sync_endpoint_stamp_s.store(
            observation.endpoint_stamp_s, std::memory_order_relaxed);
        last_sync_local_receive_wall_s.store(
            observation.local_receive_wall_s, std::memory_order_relaxed);
      }
      logClockEvent(
          usable ? "sync_accepted" : "sync_resample",
          request_id,
          CommandKind::Stop,
          &observation,
          0.0);
      if (usable) {
        return;
      }
    }
    throw std::runtime_error(
        "navigation endpoint clock sync uncertainty too high: last_rtt_ms=" +
        std::to_string(last_round_trip_s * 1000.0));
  }

  void writeCommand(
      lingtu_dds_NavigationCommandRequest message,
      const std::string& request_id,
      CommandKind kind,
      int timeout_ms) const {
    std::unique_lock<std::mutex> clock_lane_lock;
    if (requiresEndpointClock(kind)) {
      clock_lane_lock = std::unique_lock<std::mutex>(clock_command_mutex);
    }
    waitForReader(command_writer, "nav_command_request", timeout_ms);
    waitForWriter(ack_reader, "nav_command_ack", timeout_ms);
    if (requiresEndpointClock(kind) &&
        !std::isfinite(
            endpoint_clock_offset_s.load(std::memory_order_relaxed))) {
      // Header timestamps are evaluated in the endpoint's wall-clock domain.
      // WSL2 may slew CLOCK_REALTIME while a new interop process starts, and
      // remote operators are not guaranteed to share the robot's wall clock.
      // A safe Stop/ACK handshake establishes the endpoint clock without
      // weakening the endpoint's future/stale safety gates.
      synchronizeEndpointClock(timeout_ms);
    }
    std::string active_request_id = request_id;
    for (int attempt = 0; attempt < 2; ++attempt) {
      if (attempt > 0) {
        active_request_id = request_id + "-clock-retry-1";
      }
      message.request_id = const_cast<char*>(active_request_id.c_str());
      // DDS discovery can consume most of the command freshness budget on the
      // first request from a new client. Source time describes the actual
      // publication, so refresh it only after both endpoints are matched.
      const double send_source_stamp_s = sourceNowSeconds();
      last_send_source_stamp_s.store(
          send_source_stamp_s, std::memory_order_relaxed);
      last_send_clock_offset_s.store(
          endpointClockOffsetOrZero(), std::memory_order_relaxed);
      fillStamp(message.header, send_source_stamp_s);
      logClockEvent(
          "command_send",
          active_request_id,
          kind,
          nullptr,
          send_source_stamp_s);
      const auto sent_steady = SteadyClock::now();
      const auto pending = registerNavigationAck(
          active_request_id, kind, sent_steady);
      try {
        checked(
            dds_write(command_writer, static_cast<const void*>(&message)),
            "dds_write(nav_command_request)");
      } catch (...) {
        unregisterNavigationAck(active_request_id, pending);
        throw;
      }
      // The typed application ACK is stronger evidence than a DDS protocol ACK:
      // it proves that the endpoint received, decoded, and accepted the exact
      // request id/kind. Waiting for the protocol ACK first can turn a transient
      // middleware timeout into a false command failure even when the endpoint's
      // NavigationCommandAck is already available.
      const auto observation = waitForAck(
          active_request_id, pending, timeout_ms);
      (void)updateEndpointClock(observation);
      if (observation.accepted) {
        return;
      }
      logClockEvent(
          "command_rejected",
          active_request_id,
          kind,
          &observation,
          send_source_stamp_s);
      if (attempt == 0 && isRecoverableClockRejection(kind, observation.reason)) {
        logClockEvent(
            "clock_recovery",
            active_request_id,
            kind,
            &observation,
            send_source_stamp_s);
        // The rejected motion was never applied.  Establish a new endpoint
        // clock sample through a safe Stop before issuing one fresh request id.
        endpoint_clock_offset_s.store(
            std::numeric_limits<double>::quiet_NaN(),
            std::memory_order_relaxed);
        synchronizeEndpointClock(timeout_ms);
        continue;
      }
      throw std::runtime_error(
          rejectionMessage(active_request_id, kind, observation));
    }
  }

  void writeReasonCommand(
      CommandKind kind,
      const std::string& requested_reason,
      const char* default_reason,
      int timeout_ms,
      const std::string& requested_id) const {
    const std::string reason =
        requested_reason.empty() ? std::string(default_reason) : requested_reason;
    const std::string request_id =
        requested_id.empty() ? makeRequestId(kind) : requested_id;
    lingtu_dds_NavigationCommandRequest message{};
    fillHeader(message.header, nowSeconds(), "body");
    message.request_id = const_cast<char*>(request_id.c_str());
    message.kind = static_cast<std::int32_t>(kind);
    message.reason = const_cast<char*>(reason.c_str());
    writeCommand(message, request_id, kind, timeout_ms);
  }

  mutable std::mutex ack_mutex;
  mutable std::condition_variable ack_cv;
  mutable std::unordered_map<std::string, NavigationPending>
      pending_navigation_acks;
  mutable std::unordered_map<std::string, ExplorationPending>
      pending_exploration_acks;
  mutable std::unordered_map<std::string, InspectionPending>
      pending_inspection_acks;
  mutable std::string ack_receiver_error;
  std::atomic<bool> ack_receiver_running{true};
  std::thread ack_receiver;
  mutable std::mutex clock_command_mutex;
  dds_entity_t participant{0};
  dds_entity_t publisher{0};
  dds_entity_t subscriber{0};
  dds_entity_t command_writer{0};
  dds_entity_t ack_reader{0};
  dds_entity_t exploration_writer{0};
  dds_entity_t exploration_ack_reader{0};
  dds_entity_t inspection_writer{0};
  dds_entity_t inspection_ack_reader{0};
  bool diagnostics_enabled{diagnosticsEnabled()};
  mutable std::atomic<double> endpoint_clock_offset_s{
      std::numeric_limits<double>::quiet_NaN()};
  mutable std::atomic<double> last_sync_rtt_s{-0.001};
  mutable std::atomic<double> last_sync_endpoint_stamp_s{0.0};
  mutable std::atomic<double> last_sync_local_receive_wall_s{0.0};
  mutable std::atomic<double> last_send_source_stamp_s{0.0};
  mutable std::atomic<double> last_send_clock_offset_s{0.0};
};

Client::Client(int domain_id)
    : impl_(std::make_unique<Impl>(domain_id)),
      navigation_(*this),
      exploration_(*this),
      inspection_(*this) {}

Client::~Client() = default;

void Client::NavigationCommands::sendGoal(
    double x,
    double y,
    double z,
    double yaw,
    int timeout_ms,
    const std::string& requested_id) {
  requireFinite(x, "goal x");
  requireFinite(y, "goal y");
  requireFinite(z, "goal z");
  requireFinite(yaw, "goal yaw");
  const std::string request_id =
      requested_id.empty() ? makeRequestId(CommandKind::Goal) : requested_id;
  lingtu_dds_NavigationCommandRequest message{};
  fillHeader(message.header, nowSeconds(), "map");
  message.request_id = const_cast<char*>(request_id.c_str());
  message.kind = static_cast<std::int32_t>(CommandKind::Goal);
  message.goal.position.x = x;
  message.goal.position.y = y;
  message.goal.position.z = z;
  message.goal.orientation = quaternionFromYaw(yaw);
  message.reason = const_cast<char*>("");
  owner_.impl_->writeCommand(message, request_id, CommandKind::Goal, timeout_ms);
}

void Client::NavigationCommands::cancel(
    const std::string& reason,
    int timeout_ms,
    const std::string& requested_id) {
  const std::string text = reason.empty() ? "cancel" : reason;
  const std::string request_id =
      requested_id.empty() ? makeRequestId(CommandKind::Cancel) : requested_id;
  lingtu_dds_NavigationCommandRequest message{};
  fillHeader(message.header, nowSeconds(), "map");
  message.request_id = const_cast<char*>(request_id.c_str());
  message.kind = static_cast<std::int32_t>(CommandKind::Cancel);
  message.reason = const_cast<char*>(text.c_str());
  owner_.impl_->writeCommand(message, request_id, CommandKind::Cancel, timeout_ms);
}

void Client::NavigationCommands::sendTeleop(
    double vx,
    double vy,
    double wz,
    int timeout_ms,
    const std::string& requested_id) {
  requireFinite(vx, "teleop vx");
  requireFinite(vy, "teleop vy");
  requireFinite(wz, "teleop wz");
  const std::string request_id =
      requested_id.empty() ? makeRequestId(CommandKind::Teleop) : requested_id;
  lingtu_dds_NavigationCommandRequest message{};
  fillHeader(message.header, nowSeconds(), "body");
  message.request_id = const_cast<char*>(request_id.c_str());
  message.kind = static_cast<std::int32_t>(CommandKind::Teleop);
  message.velocity.linear.x = vx;
  message.velocity.linear.y = vy;
  message.velocity.angular.z = wz;
  message.reason = const_cast<char*>("");
  owner_.impl_->writeCommand(message, request_id, CommandKind::Teleop, timeout_ms);
}

void Client::NavigationCommands::stop(
    const std::string& reason,
    int timeout_ms,
    const std::string& requested_id) {
  owner_.impl_->writeReasonCommand(
      CommandKind::Stop, reason, "stop", timeout_ms, requested_id);
}

void Client::NavigationCommands::estop(
    const std::string& reason,
    int timeout_ms,
    const std::string& requested_id) {
  owner_.impl_->writeReasonCommand(
      CommandKind::Estop, reason, "estop", timeout_ms, requested_id);
}

void Client::NavigationCommands::clearEstop(
    const std::string& reason,
    int timeout_ms,
    const std::string& requested_id) {
  owner_.impl_->writeReasonCommand(
      CommandKind::ClearEstop,
      reason,
      "clear_estop",
      timeout_ms,
      requested_id);
}

void Client::NavigationCommands::resumeAutonomy(
    const std::string& reason,
    int timeout_ms,
    const std::string& requested_id) {
  owner_.impl_->writeReasonCommand(
      CommandKind::ResumeAutonomy,
      reason,
      "resume_autonomy",
      timeout_ms,
      requested_id);
}

void Client::ExplorationCommands::start(
    const std::string& session_id,
    const std::string& reason,
    int timeout_ms,
    const std::string& request_id) {
  owner_.impl_->writeExplorationCommand(
      ExplorationKind::kStart,
      session_id,
      reason.empty() ? "operator_start" : reason,
      timeout_ms,
      request_id);
}

void Client::ExplorationCommands::pause(
    const std::string& reason,
    int timeout_ms,
    const std::string& request_id) {
  owner_.impl_->writeExplorationCommand(
      ExplorationKind::kPause,
      "",
      reason.empty() ? "operator_pause" : reason,
      timeout_ms,
      request_id);
}

void Client::ExplorationCommands::resume(
    const std::string& reason,
    int timeout_ms,
    const std::string& request_id) {
  owner_.impl_->writeExplorationCommand(
      ExplorationKind::kResume,
      "",
      reason.empty() ? "operator_resume" : reason,
      timeout_ms,
      request_id);
}

void Client::ExplorationCommands::stop(
    const std::string& reason,
    int timeout_ms,
    const std::string& request_id) {
  owner_.impl_->writeExplorationCommand(
      ExplorationKind::kStop,
      "",
      reason.empty() ? "operator_stop" : reason,
      timeout_ms,
      request_id);
}

void Client::InspectionCommands::start(
    const std::string& route_id,
    std::uint64_t route_revision,
    int timeout_ms,
    const std::string& request_id) {
  if (route_id.empty()) throw std::invalid_argument("inspection route id is required");
  owner_.impl_->writeInspectionCommand(
      lingtu::message::InspectionCommandKind::kStart,
      route_id,
      route_revision,
      "",
      timeout_ms,
      request_id);
}

void Client::InspectionCommands::pause(
    const std::string& reason,
    int timeout_ms,
    const std::string& request_id) {
  owner_.impl_->writeInspectionCommand(
      lingtu::message::InspectionCommandKind::kPause,
      "",
      0U,
      reason.empty() ? "operator_pause" : reason,
      timeout_ms,
      request_id);
}

void Client::InspectionCommands::resume(
    const std::string& reason,
    int timeout_ms,
    const std::string& request_id) {
  owner_.impl_->writeInspectionCommand(
      lingtu::message::InspectionCommandKind::kResume,
      "",
      0U,
      reason.empty() ? "operator_resume" : reason,
      timeout_ms,
      request_id);
}

void Client::InspectionCommands::cancel(
    const std::string& reason,
    int timeout_ms,
    const std::string& request_id) {
  owner_.impl_->writeInspectionCommand(
      lingtu::message::InspectionCommandKind::kCancel,
      "",
      0U,
      reason.empty() ? "operator_cancel" : reason,
      timeout_ms,
      request_id);
}

}  // namespace lingtu::nav::commands

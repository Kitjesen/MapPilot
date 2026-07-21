#pragma once
// TARE DDS transport: native CycloneDDS (dds/dds.h) reader/writer for the
// five exploration topics.  Conditionally compiled with LINGTU_EXPLORE_HAS_DDS;
// when the macro is not defined the class is unavailable and the nanobind
// bindings raise RuntimeError.
//
// Style mirrors src/nav/cpp/endpoint/explore_dds.cpp.

#ifdef LINGTU_EXPLORE_HAS_DDS

#include "message/cpp/dds_qos_profiles.hpp"
#include "message/cpp/dds_topics.hpp"

#include "dds/dds.h"

// idlc-generated type descriptors.  The build system generates these from
// explore_types.idl (self-contained: includes all types needed by TARE).
#include "explore_types.h"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace lingtu::explore {

// ── Received-message value types exposed to Python via nanobind ─────────

struct DdsWayPoint {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  std::string frame_id;
  bool valid = false;
};

struct DdsPathPoint {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct DdsPath {
  std::string frame_id;
  std::vector<DdsPathPoint> poses;
  bool valid = false;
};

struct DdsRuntime {
  float data = 0.0f;
  bool valid = false;
};

struct DdsFinish {
  bool data = false;
  bool valid = false;
};

// ── DDS transport ────────────────────────────────────────────────────────

/// Owns a DDS participant and the readers/writer for the TARE topic set.
/// All methods are non-blocking; designed to be called from a Python spin
/// loop driven by the framework module lifecycle.
class TareDdsTransport {
 public:
  /// Create participant, subscriber, publisher, readers, and writer.
  /// @param domain_id  CycloneDDS domain (typically from ROS_DOMAIN_ID).
  explicit TareDdsTransport(int domain_id) {
    participant_ = checked(
        dds_create_participant(
            static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
        "dds_create_participant");
    subscriber_ = checked(
        dds_create_subscriber(participant_, nullptr, nullptr),
        "dds_create_subscriber");
    publisher_ = checked(
        dds_create_publisher(participant_, nullptr, nullptr),
        "dds_create_publisher");

    // Readers (subscribed topics)
    way_point_reader_ = make_reader(
        message::kExplorationWayPoint, &lingtu_dds_PointStamped_desc,
        "exploration_way_point");
    local_path_reader_ = make_reader(
        message::kExplorationLocalPath, &lingtu_dds_Path_desc,
        "exploration_local_path");
    runtime_reader_ = make_reader(
        message::kExplorationRuntime, &lingtu_dds_Float32_desc,
        "exploration_runtime");
    finish_reader_ = make_reader(
        message::kExplorationFinish, &lingtu_dds_Bool_desc,
        "exploration_finish");

    // Writer (published topic)
    start_writer_ = make_writer(
        message::kExplorationStart, &lingtu_dds_Bool_desc,
        "exploration_start");

    domain_id_ = domain_id;
  }

  ~TareDdsTransport() {
    if (participant_ > 0) {
      dds_delete(participant_);
    }
  }

  TareDdsTransport(const TareDdsTransport&) = delete;
  TareDdsTransport& operator=(const TareDdsTransport&) = delete;

  /// Non-blocking take from all readers.  Returns the latest valid sample
  /// for each topic (or .valid == false if nothing new).
  /// Thread-safe: acquires data_mutex_ before updating cached state.
  void spin_once() {
    auto wp  = take_way_point();
    auto pth = take_path();
    auto rt  = take_runtime();
    auto fin = take_finish();
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_way_point_ = std::move(wp);
    last_path_      = std::move(pth);
    last_runtime_   = std::move(rt);
    last_finish_    = std::move(fin);
  }

  /// Publish a start/stop signal to the TARE planner.
  void publish_start(bool start) {
    lingtu_dds_Bool msg{};
    msg.data = start;
    auto rc = dds_write(start_writer_, &msg);
    if (rc < 0) {
      std::fprintf(stderr, "dds_write(exploration_start): %s\n",
                   dds_strretcode(-rc));
    }
  }

  /// Tear down DDS entities.  Safe to call multiple times.
  void cleanup() {
    if (participant_ > 0) {
      dds_delete(participant_);
      participant_ = 0;
      subscriber_ = 0;
      publisher_ = 0;
      way_point_reader_ = 0;
      local_path_reader_ = 0;
      runtime_reader_ = 0;
      finish_reader_ = 0;
      start_writer_ = 0;
    }
  }

  // Thread-safe accessors for the latest received data.
  // Return by value so the caller owns a consistent snapshot.
  DdsWayPoint last_way_point() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return last_way_point_;
  }
  DdsPath last_path() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return last_path_;
  }
  DdsRuntime last_runtime() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return last_runtime_;
  }
  DdsFinish last_finish() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return last_finish_;
  }
  int domain_id() const { return domain_id_; }

 private:
  // ── helpers ──────────────────────────────────────────────────────────────

  static dds_entity_t checked(dds_return_t value, const char* what) {
    if (value < 0) {
      throw std::runtime_error(
          std::string(what) + ": " + dds_strretcode(-value));
    }
    return static_cast<dds_entity_t>(value);
  }

  static void log_error(dds_return_t value, const char* what) {
    if (value < 0) {
      std::fprintf(stderr, "%s: %s\n", what, dds_strretcode(-value));
    }
  }

  dds_entity_t make_reader(
      const message::TopicContract& contract,
      const dds_topic_descriptor_t* desc,
      const char* label) {
    // Create QoS from the central profile registry.
    auto qos = dds::make_qos(dds::qos_for_topic(contract.dds_topic));

    const dds_entity_t topic = checked(
        dds_create_topic(participant_, desc,
                         contract.dds_topic.data(), nullptr, nullptr),
        (std::string("dds_create_topic(") + label + ")").c_str());
    const dds_entity_t reader = checked(
        dds_create_reader(subscriber_, topic,
                          qos ? qos.get() : nullptr, nullptr),
        (std::string("dds_create_reader(") + label + ")").c_str());
    return reader;
  }

  dds_entity_t make_writer(
      const message::TopicContract& contract,
      const dds_topic_descriptor_t* desc,
      const char* label) {
    auto qos = dds::make_qos(dds::qos_for_topic(contract.dds_topic));

    const dds_entity_t topic = checked(
        dds_create_topic(participant_, desc,
                         contract.dds_topic.data(), nullptr, nullptr),
        (std::string("dds_create_topic(") + label + ")").c_str());
    const dds_entity_t writer = checked(
        dds_create_writer(publisher_, topic,
                          qos ? qos.get() : nullptr, nullptr),
        (std::string("dds_create_writer(") + label + ")").c_str());
    return writer;
  }

  // ── per-topic take helpers ─────────────────────────────────────────────

  DdsWayPoint take_way_point() {
    DdsWayPoint result;
    constexpr std::size_t kMax = 4;
    void* samples[kMax];
    dds_sample_info_t infos[kMax];
    for (auto& s : samples) {
      s = dds_alloc(sizeof(lingtu_dds_PointStamped));
      std::memset(s, 0, sizeof(lingtu_dds_PointStamped));
    }
    const dds_return_t n = dds_take(
        way_point_reader_, samples, infos, kMax, kMax);
    if (n > 0) {
      // Use the last valid sample.
      for (dds_return_t i = n - 1; i >= 0; --i) {
        if (infos[i].valid_data) {
          const auto* msg = static_cast<lingtu_dds_PointStamped*>(samples[i]);
          result.x = msg->point.x;
          result.y = msg->point.y;
          result.z = msg->point.z;
          if (msg->header.frame_id) {
            result.frame_id = msg->header.frame_id;
          }
          result.valid = true;
          break;
        }
      }
    }
    for (auto& s : samples) {
      dds_sample_free(s, &lingtu_dds_PointStamped_desc, DDS_FREE_ALL);
    }
    return result;
  }

  DdsPath take_path() {
    DdsPath result;
    constexpr std::size_t kMax = 4;
    void* samples[kMax];
    dds_sample_info_t infos[kMax];
    for (auto& s : samples) {
      s = dds_alloc(sizeof(lingtu_dds_Path));
      std::memset(s, 0, sizeof(lingtu_dds_Path));
    }
    const dds_return_t n = dds_take(
        local_path_reader_, samples, infos, kMax, kMax);
    if (n > 0) {
      for (dds_return_t i = n - 1; i >= 0; --i) {
        if (infos[i].valid_data) {
          const auto* msg = static_cast<lingtu_dds_Path*>(samples[i]);
          if (msg->header.frame_id) {
            result.frame_id = msg->header.frame_id;
          }
          result.poses.reserve(msg->poses._length);
          for (std::uint32_t j = 0; j < msg->poses._length; ++j) {
            DdsPathPoint pt;
            pt.x = msg->poses._buffer[j].pose.position.x;
            pt.y = msg->poses._buffer[j].pose.position.y;
            pt.z = msg->poses._buffer[j].pose.position.z;
            result.poses.push_back(pt);
          }
          result.valid = true;
          break;
        }
      }
    }
    for (auto& s : samples) {
      dds_sample_free(s, &lingtu_dds_Path_desc, DDS_FREE_ALL);
    }
    return result;
  }

  DdsRuntime take_runtime() {
    DdsRuntime result;
    constexpr std::size_t kMax = 4;
    void* samples[kMax];
    dds_sample_info_t infos[kMax];
    for (auto& s : samples) {
      s = dds_alloc(sizeof(lingtu_dds_Float32));
      std::memset(s, 0, sizeof(lingtu_dds_Float32));
    }
    const dds_return_t n = dds_take(
        runtime_reader_, samples, infos, kMax, kMax);
    if (n > 0) {
      for (dds_return_t i = n - 1; i >= 0; --i) {
        if (infos[i].valid_data) {
          const auto* msg = static_cast<lingtu_dds_Float32*>(samples[i]);
          result.data = msg->data;
          result.valid = true;
          break;
        }
      }
    }
    for (auto& s : samples) {
      dds_sample_free(s, &lingtu_dds_Float32_desc, DDS_FREE_ALL);
    }
    return result;
  }

  DdsFinish take_finish() {
    DdsFinish result;
    constexpr std::size_t kMax = 4;
    void* samples[kMax];
    dds_sample_info_t infos[kMax];
    for (auto& s : samples) {
      s = dds_alloc(sizeof(lingtu_dds_Bool));
      std::memset(s, 0, sizeof(lingtu_dds_Bool));
    }
    const dds_return_t n = dds_take(
        finish_reader_, samples, infos, kMax, kMax);
    if (n > 0) {
      for (dds_return_t i = n - 1; i >= 0; --i) {
        if (infos[i].valid_data) {
          const auto* msg = static_cast<lingtu_dds_Bool*>(samples[i]);
          result.data = msg->data;
          result.valid = true;
          break;
        }
      }
    }
    for (auto& s : samples) {
      dds_sample_free(s, &lingtu_dds_Bool_desc, DDS_FREE_ALL);
    }
    return result;
  }

  // ── state ──────────────────────────────────────────────────────────────

  int domain_id_ = 0;
  dds_entity_t participant_{0};
  dds_entity_t subscriber_{0};
  dds_entity_t publisher_{0};
  dds_entity_t way_point_reader_{0};
  dds_entity_t local_path_reader_{0};
  dds_entity_t runtime_reader_{0};
  dds_entity_t finish_reader_{0};
  dds_entity_t start_writer_{0};

  // Mutex protecting last_way_point_, last_path_, last_runtime_, last_finish_.
  mutable std::mutex data_mutex_;

  DdsWayPoint last_way_point_;
  DdsPath     last_path_;
  DdsRuntime  last_runtime_;
  DdsFinish   last_finish_;
};

}  // namespace lingtu::explore

#endif  // LINGTU_EXPLORE_HAS_DDS

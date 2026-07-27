#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>

#include "dds/dds.h"
#include "lingtu_slam.h"
#include "message/cpp/dds_qos_profiles.hpp"
#include "message/cpp/dds_topics.hpp"
#include "nav_dds_runtime.hpp"

namespace {

using namespace std::chrono_literals;

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

dds_entity_t checked(dds_return_t value, const char *operation) {
  if (value < 0) {
    throw std::runtime_error(std::string(operation) + ": " + dds_strretcode(-value));
  }
  return static_cast<dds_entity_t>(value);
}

template <typename Wire>
bool takeOne(dds_entity_t reader, Wire *output, const dds_topic_descriptor_t &descriptor) {
  void *samples[1]{};
  dds_sample_info_t infos[1]{};
  samples[0] = dds_alloc(sizeof(Wire));
  std::memset(samples[0], 0, sizeof(Wire));
  const dds_return_t count = dds_take(reader, samples, infos, 1, 1);
  checked(count, "dds_take(nav_goal_status)");
  if (count == 0) {
    dds_sample_free(samples[0], &descriptor, DDS_FREE_ALL);
    return false;
  }
  const bool valid = infos[0].valid_data;
  if (valid) {
    *output = *static_cast<Wire *>(samples[0]);
  }
  dds_free(samples[0]);
  return valid;
}

class GoalStatusPeer {
 public:
  explicit GoalStatusPeer(int domain_id) {
    participant_ =
        checked(dds_create_participant(static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
                "dds_create_participant(goal_status_peer)");
    reader_ = makeReader(lingtu::message::kNavGoalStatus.dds_topic.data(),
                         &lingtu_dds_NavigationGoalStatus_desc);
  }

  ~GoalStatusPeer() {
    if (participant_ > 0) {
      dds_delete(participant_);
    }
  }

  bool take(lingtu_dds_NavigationGoalStatus *output) {
    return takeOne(reader_, output, lingtu_dds_NavigationGoalStatus_desc);
  }

 private:
  dds_entity_t makeReader(const char *topic_name, const dds_topic_descriptor_t *descriptor) {
    const dds_entity_t topic =
        checked(dds_create_topic(participant_, descriptor, topic_name, nullptr, nullptr),
                "dds_create_topic(nav_goal_status)");
    auto qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(topic_name));
    return checked(dds_create_reader(participant_, topic, qos.get(), nullptr),
                   "dds_create_reader(nav_goal_status)");
  }

  dds_entity_t participant_{0};
  dds_entity_t reader_{0};
};

void testGoalStatusQosIsReliableAndRetained() {
  const auto topic = lingtu::message::kNavGoalStatus.dds_topic.data();
  require(lingtu::dds::qos_for_topic(topic) == lingtu::dds::QosProfile::CommandAck,
          "goal lifecycle must use the command-ack QoS profile");
  auto qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(topic));
  dds_reliability_kind_t reliability{};
  dds_duration_t max_blocking_time{};
  require(dds_qget_reliability(qos.get(), &reliability, &max_blocking_time),
          "goal lifecycle QoS must define reliability");
  require(reliability == DDS_RELIABILITY_RELIABLE, "goal lifecycle must be reliable");
  dds_history_kind_t history{};
  int32_t depth = 0;
  require(dds_qget_history(qos.get(), &history, &depth), "goal lifecycle QoS must define history");
  require(history == DDS_HISTORY_KEEP_LAST && depth == 64,
          "goal lifecycle must retain the bounded lifecycle history");
}

void testRuntimePublishesCorrelatedGoalLifecycle() {
  constexpr int kDomain = 121;
  lingtu::nav::endpoint::DdsRuntime runtime(kDomain);
  GoalStatusPeer peer(kDomain);

  lingtu_dds_NavigationGoalStatus observed{};
  for (int attempt = 0; attempt < 100; ++attempt) {
    runtime.writeNavigationGoalStatus("navigation-task-1", "goal-attempt-1",
                                      lingtu::message::NavigationGoalState::Failed, 42U,
                                      "goal_outside_map");
    std::this_thread::sleep_for(10ms);
    if (peer.take(&observed)) {
      break;
    }
  }

  require(observed.task_id != nullptr, "goal lifecycle sample must arrive");
  require(std::string(observed.task_id) == "navigation-task-1",
          "goal lifecycle task id must be preserved");
  require(observed.request_id != nullptr && std::string(observed.request_id) == "goal-attempt-1",
          "goal lifecycle request id must be preserved");
  require(observed.boot_id != nullptr && std::strlen(observed.boot_id) > 0U,
          "goal lifecycle producer boot id must be present");
  require(observed.event_sequence > 0U, "goal lifecycle sequence must be positive");
  require(observed.state == static_cast<std::int32_t>(lingtu::message::NavigationGoalState::Failed),
          "goal lifecycle state must be preserved");
  require(observed.goal_epoch == 42U, "goal lifecycle epoch must be preserved");
  require(observed.reason != nullptr && std::string(observed.reason) == "goal_outside_map",
          "goal lifecycle reason must be preserved");
  require(observed.header.frame_id != nullptr && std::string(observed.header.frame_id) == "map",
          "goal lifecycle header must be map framed");
  dds_free(observed.header.frame_id);
  dds_free(observed.boot_id);
  dds_free(observed.task_id);
  dds_free(observed.request_id);
  dds_free(observed.reason);
}

}  // namespace

int main() {
  try {
    testGoalStatusQosIsReliableAndRetained();
    testRuntimePublishesCorrelatedGoalLifecycle();
    return 0;
  } catch (const std::exception &exc) {
    std::fprintf(stderr, "test_navigation_goal_status_dds: FAIL: %s\n", exc.what());
    return 1;
  }
}

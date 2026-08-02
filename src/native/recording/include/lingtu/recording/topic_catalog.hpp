#pragma once

#include <string>
#include <vector>

#include "dds/dds.h"
#include "lingtu/recording/mcap_session.hpp"
#include "message/cpp/dds_qos_profiles.hpp"
#include "message/cpp/dds_topics.hpp"

namespace lingtu::recording {

enum class ReplayPolicy { Replayable, RecordOnly };

struct TopicBinding {
  const lingtu::message::TopicContract *contract;
  const dds_topic_descriptor_t *descriptor;
  lingtu::dds::QosProfile qos_profile;
  const char *qos_name;
  void *(*allocate_sample)();
  void (*free_sample)(void *);
  ReplayPolicy replay_policy{ReplayPolicy::RecordOnly};
};

const std::vector<TopicBinding> &sensor_topic_catalog();
const std::vector<TopicBinding> &recording_topic_catalog();
const TopicBinding *find_sensor_topic(const std::string &topic);
const TopicBinding *find_recording_topic(const std::string &topic);
std::vector<const TopicBinding *> default_recording_topics();
ChannelDefinition channel_definition(const TopicBinding &binding);
const char *replay_policy_name(ReplayPolicy policy) noexcept;

}  // namespace lingtu::recording

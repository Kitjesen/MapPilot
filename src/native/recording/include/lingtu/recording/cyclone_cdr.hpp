#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "dds/dds.h"
#include "lingtu/recording/recording_core.hpp"
#include "lingtu/recording/topic_catalog.hpp"

namespace lingtu::recording {

std::vector<RecordedMessage> take_cdr_messages(dds_entity_t reader, std::string_view wire_topic,
                                               std::uint32_t &next_sequence,
                                               std::size_t max_samples = 64);

std::string validate_cdr_payload(const TopicBinding &binding, const std::byte *payload,
                                 std::size_t payload_size);

std::string validate_cdr_payload(const TopicBinding &binding,
                                 const std::vector<std::byte> &payload);

std::string forward_cdr_message(dds_entity_t writer, const TopicBinding &binding,
                                const std::byte *payload, std::size_t payload_size,
                                dds_time_t source_timestamp);

std::string forward_cdr_message(dds_entity_t writer, const TopicBinding &binding,
                                const std::vector<std::byte> &payload, dds_time_t source_timestamp);

}  // namespace lingtu::recording

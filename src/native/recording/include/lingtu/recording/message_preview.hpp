#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <memory>
#include <string>
#include <string_view>
#include <vector>
#include "lingtu/recording/topic_catalog.hpp"

struct ddsi_serdata;

namespace lingtu::recording {
std::string json_quote(std::string_view value);
std::string json_number(double value);
using RecordedSample = std::unique_ptr<void, void (*)(void *)>;
RecordedSample decode_recorded_sample(const TopicBinding &binding, const std::byte *data, std::size_t size);
std::string message_preview(const TopicBinding &binding, const ddsi_serdata *data);
std::string message_preview(const TopicBinding &binding, const std::byte *data, std::size_t size);

struct NumericPreviewValue {
  std::string path;
  std::optional<double> value;
};
struct NumericPreview {
  std::vector<NumericPreviewValue> values;
  std::string frame_id;
};
// Walk only our bounded typed preview, never a full cloud/image JSON document.
NumericPreview numeric_preview(const std::string &preview);
std::string numeric_field_unit(std::string_view path);
std::uint32_t recorded_path_pose_count(const std::byte *data, std::size_t size);
}  // namespace lingtu::recording

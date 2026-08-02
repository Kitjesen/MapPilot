#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "lingtu/recording/recording_core.hpp"

namespace lingtu::recording {

inline constexpr const char *kMcapProfile = "lingtu.dds.v1";

struct ChannelDefinition {
  std::string canonical_topic;
  std::string wire_topic;
  std::string idl_type;
  std::string qos_profile;
};

class McapSessionWriter {
 public:
  McapSessionWriter(std::filesystem::path final_path, std::string idl_schema,
                    std::vector<ChannelDefinition> channels,
                    std::uint64_t chunk_size_bytes = 4U * 1024U * 1024U);
  ~McapSessionWriter();

  McapSessionWriter(const McapSessionWriter &) = delete;
  McapSessionWriter &operator=(const McapSessionWriter &) = delete;

  void write(const RecordedMessage &message);
  void commit();

  const std::filesystem::path &final_path() const noexcept;
  const std::filesystem::path &temporary_path() const noexcept;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

std::string read_text_file(const std::filesystem::path &path);

}  // namespace lingtu::recording

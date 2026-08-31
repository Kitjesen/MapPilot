#pragma once

#include <charconv>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>

namespace lingtu::sim::dds_adapter {

constexpr int kMinSupportedDdsDomainId = 0;
constexpr int kMaxSupportedDdsDomainId = 232;

inline int parse_supported_dds_domain_id(std::string_view value) {
  int domain_id = 0;
  const auto result = std::from_chars(value.data(), value.data() + value.size(), domain_id);
  if (value.empty() || result.ec != std::errc{} || result.ptr != value.data() + value.size()) {
    throw std::runtime_error("DDS domain id must be an integer between 0 and 232");
  }
  if (domain_id < kMinSupportedDdsDomainId || domain_id > kMaxSupportedDdsDomainId) {
    throw std::runtime_error("DDS domain id must be between 0 and 232");
  }
  return domain_id;
}

}  // namespace lingtu::sim::dds_adapter

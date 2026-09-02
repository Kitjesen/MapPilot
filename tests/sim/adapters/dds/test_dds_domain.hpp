#pragma once

#include <charconv>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>

#ifndef LINGTU_TEST_DEFAULT_DDS_DOMAIN_ID
#error "LINGTU_TEST_DEFAULT_DDS_DOMAIN_ID must be assigned by the test build"
#endif

namespace lingtu::sim::dds_adapter::test {

constexpr int kDefaultDdsDomainId = LINGTU_TEST_DEFAULT_DDS_DOMAIN_ID;
constexpr int kCycloneDefaultPortBase = 7400;
constexpr int kCycloneDefaultDomainGain = 250;
constexpr int kCycloneMaximumFixedPortOffset = 11;
constexpr int kWindowsDynamicPortStart = 49152;
constexpr int kReservedWindowsProductDdsDomainId = 17;
constexpr int kFirstNavigationTestDdsDomainId = 20;

static_assert(kDefaultDdsDomainId >= 0 &&
                  kDefaultDdsDomainId < kFirstNavigationTestDdsDomainId,
              "adapter CTest DDS domain must stay in the low-domain slot range");
static_assert(kDefaultDdsDomainId != kReservedWindowsProductDdsDomainId,
              "adapter CTest must not use the Windows Product runtime DDS domain");
static_assert(
    kCycloneDefaultPortBase + kCycloneDefaultDomainGain * kDefaultDdsDomainId +
            kCycloneMaximumFixedPortOffset <
        kWindowsDynamicPortStart,
    "test DDS domain maps Cyclone discovery into Windows dynamic ports");

inline int validate_domain_id(int domain_id) {
  if (domain_id == kReservedWindowsProductDdsDomainId) {
    throw std::runtime_error(
        "LINGTU_TEST_DDS_DOMAIN_ID uses the Windows Product runtime DDS domain");
  }
  const std::int64_t highest_fixed_port =
      static_cast<std::int64_t>(kCycloneDefaultPortBase) +
      static_cast<std::int64_t>(kCycloneDefaultDomainGain) * domain_id +
      kCycloneMaximumFixedPortOffset;
  if (highest_fixed_port >= kWindowsDynamicPortStart) {
    throw std::runtime_error(
        "LINGTU_TEST_DDS_DOMAIN_ID maps CycloneDDS fixed ports into Windows dynamic ports");
  }
  if (domain_id < 0 || domain_id >= kFirstNavigationTestDdsDomainId) {
    throw std::runtime_error(
        "LINGTU_TEST_DDS_DOMAIN_ID is outside the adapter low-domain test slots");
  }
  return domain_id;
}

inline int parse_domain_id(std::string_view configured) {
  int domain_id = 0;
  const auto result =
      std::from_chars(configured.data(), configured.data() + configured.size(), domain_id);
  if (result.ec != std::errc{} || result.ptr != configured.data() + configured.size()) {
    throw std::runtime_error("LINGTU_TEST_DDS_DOMAIN_ID must be an integer");
  }
  return validate_domain_id(domain_id);
}

inline std::optional<std::string> configured_domain_id() {
#ifdef _WIN32
  char *value = nullptr;
  std::size_t size = 0;
  if (_dupenv_s(&value, &size, "LINGTU_TEST_DDS_DOMAIN_ID") != 0) {
    throw std::runtime_error("failed to read LINGTU_TEST_DDS_DOMAIN_ID");
  }
  if (value == nullptr) {
    return std::nullopt;
  }
  std::string configured(value, size > 0 ? size - 1 : 0);
  std::free(value);
  return configured;
#else
  const char *value = std::getenv("LINGTU_TEST_DDS_DOMAIN_ID");
  return value == nullptr ? std::nullopt : std::optional<std::string>{value};
#endif
}

inline int domain_id_from_environment() {
  const auto configured = configured_domain_id();
  return configured.has_value() ? parse_domain_id(*configured)
                                : validate_domain_id(kDefaultDdsDomainId);
}

}  // namespace lingtu::sim::dds_adapter::test

#pragma once

#include <cstddef>
#include <cstdint>

namespace lingtu::maps::mapd::query {

constexpr char kRequestMagic[4] = {'L', 'T', 'M', 'P'};
constexpr char kResponseMagic[4] = {'L', 'T', 'M', 'R'};
constexpr std::uint8_t kVersion = 1U;
constexpr std::size_t kRequestHeaderSize = 16U;
constexpr std::size_t kResponseHeaderSize = 12U;
constexpr std::size_t kMaxMapIdBytes = 128U;
constexpr std::size_t kMaxCapabilityBytes = 64U;
constexpr std::size_t kDefaultMaxJsonBytes = 1024U * 1024U;
constexpr const char* kDefaultSocketPath = "/run/lingtu-mapd/mapd.sock";

enum class Opcode : std::uint8_t {
  kPing = 1U,
  kOpenArtifact = 5U,
};

enum class Status : std::uint8_t {
  kOk = 0U,
  kError = 1U,
};

constexpr std::uint16_t kResponseFlagHasFd = 1U;

}  // namespace lingtu::maps::mapd::query

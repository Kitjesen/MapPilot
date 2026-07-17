#include "lingtu/maps/hash.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace lingtu::maps {
namespace {

class Sha256 {
 public:
  Sha256() {
    state_ = {
        0x6a09e667U,
        0xbb67ae85U,
        0x3c6ef372U,
        0xa54ff53aU,
        0x510e527fU,
        0x9b05688cU,
        0x1f83d9abU,
        0x5be0cd19U,
    };
  }

  void Update(const unsigned char* data, std::size_t size) {
    total_bits_ += static_cast<std::uint64_t>(size) * 8U;
    std::size_t offset = 0;
    if (buffer_size_ > 0U) {
      const std::size_t count = std::min<std::size_t>(64U - buffer_size_, size);
      std::memcpy(buffer_.data() + buffer_size_, data, count);
      buffer_size_ += count;
      offset += count;
      if (buffer_size_ == 64U) {
        Transform(buffer_.data());
        buffer_size_ = 0;
      }
    }
    while (offset + 64U <= size) {
      Transform(data + offset);
      offset += 64U;
    }
    if (offset < size) {
      buffer_size_ = size - offset;
      std::memcpy(buffer_.data(), data + offset, buffer_size_);
    }
  }

  std::string FinalHex() {
    const std::uint64_t bit_count = total_bits_;
    buffer_[buffer_size_++] = 0x80U;
    if (buffer_size_ > 56U) {
      while (buffer_size_ < 64U) {
        buffer_[buffer_size_++] = 0U;
      }
      Transform(buffer_.data());
      buffer_size_ = 0;
    }
    while (buffer_size_ < 56U) {
      buffer_[buffer_size_++] = 0U;
    }
    for (int shift = 56; shift >= 0; shift -= 8) {
      buffer_[buffer_size_++] = static_cast<unsigned char>((bit_count >> shift) & 0xffU);
    }
    Transform(buffer_.data());

    std::ostringstream out;
    out << std::hex << std::setfill('0');
    for (const auto value : state_) {
      for (int shift = 24; shift >= 0; shift -= 8) {
        out << std::setw(2) << ((value >> shift) & 0xffU);
      }
    }
    return out.str();
  }

 private:
  static std::uint32_t RotateRight(std::uint32_t value, std::uint32_t count) {
    return (value >> count) | (value << (32U - count));
  }

  static std::uint32_t LoadBigEndian32(const unsigned char* data) {
    return (static_cast<std::uint32_t>(data[0]) << 24U) |
        (static_cast<std::uint32_t>(data[1]) << 16U) |
        (static_cast<std::uint32_t>(data[2]) << 8U) |
        static_cast<std::uint32_t>(data[3]);
  }

  void Transform(const unsigned char* block) {
    static constexpr std::array<std::uint32_t, 64> kRoundConstants = {
        0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U,
        0x3956c25bU, 0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U,
        0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U,
        0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U,
        0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU,
        0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
        0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U,
        0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U,
        0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U,
        0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
        0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U,
        0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
        0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U,
        0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U,
        0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
        0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U,
    };

    std::array<std::uint32_t, 64> w{};
    for (std::size_t i = 0; i < 16U; ++i) {
      w[i] = LoadBigEndian32(block + i * 4U);
    }
    for (std::size_t i = 16U; i < 64U; ++i) {
      const std::uint32_t s0 =
          RotateRight(w[i - 15U], 7U) ^ RotateRight(w[i - 15U], 18U) ^ (w[i - 15U] >> 3U);
      const std::uint32_t s1 =
          RotateRight(w[i - 2U], 17U) ^ RotateRight(w[i - 2U], 19U) ^ (w[i - 2U] >> 10U);
      w[i] = w[i - 16U] + s0 + w[i - 7U] + s1;
    }

    std::uint32_t a = state_[0];
    std::uint32_t b = state_[1];
    std::uint32_t c = state_[2];
    std::uint32_t d = state_[3];
    std::uint32_t e = state_[4];
    std::uint32_t f = state_[5];
    std::uint32_t g = state_[6];
    std::uint32_t h = state_[7];

    for (std::size_t i = 0; i < 64U; ++i) {
      const std::uint32_t s1 =
          RotateRight(e, 6U) ^ RotateRight(e, 11U) ^ RotateRight(e, 25U);
      const std::uint32_t ch = (e & f) ^ ((~e) & g);
      const std::uint32_t temp1 = h + s1 + ch + kRoundConstants[i] + w[i];
      const std::uint32_t s0 =
          RotateRight(a, 2U) ^ RotateRight(a, 13U) ^ RotateRight(a, 22U);
      const std::uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
      const std::uint32_t temp2 = s0 + maj;
      h = g;
      g = f;
      f = e;
      e = d + temp1;
      d = c;
      c = b;
      b = a;
      a = temp1 + temp2;
    }

    state_[0] += a;
    state_[1] += b;
    state_[2] += c;
    state_[3] += d;
    state_[4] += e;
    state_[5] += f;
    state_[6] += g;
    state_[7] += h;
  }

  std::array<std::uint32_t, 8> state_{};
  std::array<unsigned char, 64> buffer_{};
  std::size_t buffer_size_{0};
  std::uint64_t total_bits_{0};
};

}  // namespace

std::string Sha256File(const std::filesystem::path& path) {
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    throw std::runtime_error("failed to hash: " + path.string());
  }
  Sha256 hasher;
  std::array<unsigned char, 64 * 1024> buffer{};
  while (file) {
    file.read(reinterpret_cast<char*>(buffer.data()), static_cast<std::streamsize>(buffer.size()));
    const auto count = file.gcount();
    if (count > 0) {
      hasher.Update(buffer.data(), static_cast<std::size_t>(count));
    }
  }
  return hasher.FinalHex();
}

std::string Sha256Text(const std::string& value) {
  Sha256 hasher;
  hasher.Update(
      reinterpret_cast<const unsigned char*>(value.data()),
      value.size());
  return hasher.FinalHex();
}

}  // namespace lingtu::maps

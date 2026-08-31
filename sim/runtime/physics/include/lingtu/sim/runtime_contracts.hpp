#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string_view>
#include <type_traits>

namespace lingtu::sim {

inline constexpr std::size_t kMaxStableIdBytes = 127;
inline constexpr std::size_t kMaxSessionIdBytes = 64;
inline constexpr std::size_t kMaxCommandTypeBytes = 63;
inline constexpr std::size_t kMaxCommandPayloadBytes = 2048;
inline constexpr std::size_t kMaxSnapshotEntities = 64;
inline constexpr std::size_t kMaxJointValuesPerEntity = 128;
inline constexpr std::size_t kMaxActuatorValues = 128;

template <std::size_t Capacity>
class FixedString final {
 public:
  static_assert(Capacity > 0, "FixedString capacity must be positive");
  FixedString() noexcept = default;
  FixedString(const char *value) noexcept {
    assign(value == nullptr ? std::string_view{} : std::string_view(value));
  }
  FixedString(std::string_view value) noexcept { assign(value); }

  bool assign(std::string_view value) noexcept {
    if (value.size() > Capacity) {
      return false;
    }
    if (!value.empty()) {
      std::memcpy(data_.data(), value.data(), value.size());
    }
    size_ = value.size();
    data_[size_] = '\0';
    return true;
  }

  [[nodiscard]] std::size_t size() const noexcept { return size_; }
  [[nodiscard]] bool empty() const noexcept { return size_ == 0; }
  [[nodiscard]] const char *c_str() const noexcept { return data_.data(); }
  [[nodiscard]] std::string_view view() const noexcept { return {data_.data(), size_}; }

  friend bool operator==(const FixedString &lhs, const FixedString &rhs) noexcept {
    return lhs.view() == rhs.view();
  }
  friend bool operator!=(const FixedString &lhs, const FixedString &rhs) noexcept {
    return !(lhs == rhs);
  }

 private:
  std::array<char, Capacity + 1> data_{};
  std::size_t size_{0};
};

using StableId = FixedString<kMaxStableIdBytes>;
// Session ids come from the session document. This is a process-local
// fixed-capacity field; CDR serialization maps it explicitly.
using SessionId = FixedString<kMaxSessionIdBytes>;
using CommandType = FixedString<kMaxCommandTypeBytes>;

template <typename T, std::size_t Capacity>
class FixedVector final {
 public:
  static_assert(Capacity > 0, "FixedVector capacity must be positive");
  using iterator = typename std::array<T, Capacity>::iterator;
  using const_iterator = typename std::array<T, Capacity>::const_iterator;

  [[nodiscard]] std::size_t size() const noexcept { return size_; }
  [[nodiscard]] constexpr std::size_t capacity() const noexcept { return Capacity; }
  [[nodiscard]] bool empty() const noexcept { return size_ == 0; }
  [[nodiscard]] bool full() const noexcept { return size_ == Capacity; }

  bool push_back(const T &value) noexcept(std::is_nothrow_copy_assignable_v<T>) {
    if (full()) {
      return false;
    }
    values_[size_++] = value;
    return true;
  }

  bool push_back(T &&value) noexcept(std::is_nothrow_move_assignable_v<T>) {
    if (full()) {
      return false;
    }
    values_[size_++] = static_cast<T &&>(value);
    return true;
  }

  void clear() noexcept { size_ = 0; }
  T &operator[](std::size_t index) noexcept { return values_[index]; }
  const T &operator[](std::size_t index) const noexcept { return values_[index]; }
  T &front() noexcept { return values_[0]; }
  const T &front() const noexcept { return values_[0]; }
  T &back() noexcept { return values_[size_ - 1]; }
  const T &back() const noexcept { return values_[size_ - 1]; }
  iterator begin() noexcept { return values_.begin(); }
  iterator end() noexcept { return values_.begin() + static_cast<std::ptrdiff_t>(size_); }
  const_iterator begin() const noexcept { return values_.begin(); }
  const_iterator end() const noexcept { return values_.begin() + static_cast<std::ptrdiff_t>(size_); }

 private:
  std::array<T, Capacity> values_{};
  std::size_t size_{0};
};

struct GenerationStamp final {
  std::uint64_t model_generation{0};
  std::uint64_t reset_generation{0};
  friend bool operator==(const GenerationStamp &lhs, const GenerationStamp &rhs) noexcept {
    return lhs.model_generation == rhs.model_generation &&
           lhs.reset_generation == rhs.reset_generation;
  }
  friend bool operator!=(const GenerationStamp &lhs, const GenerationStamp &rhs) noexcept {
    return !(lhs == rhs);
  }
};

struct EntityState final {
  StableId instance_id;
  StableId frame_id;
  StableId stable_id;
  std::array<double, 3> position_m{};
  std::array<double, 4> quaternion_wxyz{1.0, 0.0, 0.0, 0.0};
  std::array<double, 3> linear_velocity_mps{};
  std::array<double, 3> angular_velocity_rps{};
  FixedVector<double, kMaxJointValuesPerEntity> joint_position_rad;
  FixedVector<double, kMaxJointValuesPerEntity> joint_velocity_rps;
};

struct TruthSnapshotEnvelope final {
  SessionId session_id;
  GenerationStamp generation;
  std::uint64_t sequence{0};
  std::uint64_t sim_time_ns{0};
  FixedVector<EntityState, kMaxSnapshotEntities> entities;
};

struct ActuatorCommandPayload final {
  std::array<double, kMaxActuatorValues> values{};
  std::uint32_t value_count{0};
  std::uint8_t safe_stop{0};
};

struct CommandEnvelope final {
  SessionId session_id;
  StableId source_id;
  StableId instance_id;
  GenerationStamp generation;
  std::uint64_t sequence{0};
  std::uint64_t apply_time_ns{0};
  CommandType type;
  std::array<std::byte, kMaxCommandPayloadBytes> payload{};
  std::uint32_t payload_size{0};

  template <typename T>
  bool set_payload(const T &value) noexcept {
    static_assert(std::is_trivially_copyable_v<T>, "command payload must be trivially copyable");
    if (sizeof(T) > payload.size()) {
      return false;
    }
    std::memcpy(payload.data(), &value, sizeof(T));
    payload_size = static_cast<std::uint32_t>(sizeof(T));
    return true;
  }
};

// These DTOs are deliberately fixed-capacity and trivially copyable for
// allocation-free in-process handoff. Their native layout, padding, and
// sizeof() are not a cross-process ABI. DDS/CDR, shared-memory descriptors,
// and future network transports must serialize each field explicitly.
static_assert(std::is_trivially_copyable_v<GenerationStamp>);
static_assert(std::is_trivially_copyable_v<EntityState>);
static_assert(std::is_trivially_copyable_v<TruthSnapshotEnvelope>);
static_assert(std::is_trivially_copyable_v<ActuatorCommandPayload>);
static_assert(std::is_trivially_copyable_v<CommandEnvelope>);
static_assert(sizeof(ActuatorCommandPayload) <= kMaxCommandPayloadBytes);
static_assert(std::is_nothrow_copy_assignable_v<TruthSnapshotEnvelope>);
static_assert(std::is_nothrow_copy_assignable_v<CommandEnvelope>);

}  // namespace lingtu::sim

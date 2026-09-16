#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace fake_unitree {
inline std::vector<std::string> calls;
inline bool enabled = true;
inline bool apply_setting = true;
inline int stop_code = 0;
inline int set_code = 0;
inline int failed_query = 0;
inline int queries = 0;
inline std::function<void(const void *)> state_callback;
inline std::function<void(const void *)> low_state_callback;
inline void reset() {
  calls.clear();
  enabled = apply_setting = true;
  stop_code = set_code = failed_query = queries = 0;
  state_callback = {};
  low_state_callback = {};
}
}  // namespace fake_unitree

namespace unitree_go::msg::dds_ {
class MotorState_ {
 public:
  float q() const { return q_; }
  void q(float value) { q_ = value; }
  float dq() const { return dq_; }
  void dq(float value) { dq_ = value; }
  float tau_est() const { return tau_est_; }
  void tau_est(float value) { tau_est_ = value; }
 private:
  float q_{0.0F};
  float dq_{0.0F};
  float tau_est_{0.0F};
};
class LowState_ {
 public:
  std::uint32_t tick() const { return tick_; }
  void tick(std::uint32_t value) { tick_ = value; }
  const std::array<MotorState_, 20>& motor_state() const { return motors_; }
  std::array<MotorState_, 20>& motor_state() { return motors_; }
 private:
  std::uint32_t tick_{0};
  std::array<MotorState_, 20> motors_;
};
class SportModeState_ {
 public:
  std::uint32_t error_code() const { return 0; }
  std::uint8_t mode() const { return 1; }
  std::array<float, 3> position() const { return {}; }
  std::array<float, 3> velocity() const { return {}; }
  float yaw_speed() const { return 0; }
  float body_height() const { return 0.3F; }
};
}  // namespace unitree_go::msg::dds_

namespace unitree::robot {
class ChannelFactory {
 public:
  static ChannelFactory *Instance() { static ChannelFactory instance; return &instance; }
  void Init(int, const std::string &) {}
  void Release() {}
};
template <typename T> class ChannelSubscriber {
 public:
  explicit ChannelSubscriber(const std::string &topic) : topic_(topic) {}
  void InitChannel(std::function<void(const void *)> callback, int) {
    if (topic_ == "rt/lowstate") {
      fake_unitree::low_state_callback = callback;
    } else {
      fake_unitree::state_callback = callback;
      const T state;
      callback(&state);
    }
  }
 private:
  std::string topic_;
};
namespace go2 {
class SportClient {
 public:
  void SetTimeout(float) {}
  void Init() {}
  int StopMove() { fake_unitree::calls.push_back("stop"); return fake_unitree::stop_code; }
  int Move(float, float, float) { fake_unitree::calls.push_back("move"); return 0; }
  int StandUp() { return 0; }
  int Sit() { return 0; }
  int RecoveryStand() { return 0; }
  int Damp() { return 0; }
};
class ObstaclesAvoidClient {
 public:
  void SetTimeout(float) {}
  void Init() {}
  int SwitchGet(bool &enabled) {
    fake_unitree::calls.push_back("get");
    if (++fake_unitree::queries == fake_unitree::failed_query) return 42;
    enabled = fake_unitree::enabled;
    return 0;
  }
  int SwitchSet(bool enabled) {
    fake_unitree::calls.push_back(enabled ? "enable" : "disable");
    if (fake_unitree::set_code != 0) return fake_unitree::set_code;
    if (fake_unitree::apply_setting) fake_unitree::enabled = enabled;
    return 0;
  }
};
}  // namespace go2
}  // namespace unitree::robot

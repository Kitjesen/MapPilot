#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/obstacles_avoid/obstacles_avoid_client.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

#include "body.hpp"
#include "config.hpp"

namespace lingtu::driver {
namespace {

using State = unitree_go::msg::dds_::SportModeState_;
using StateSubscriber = unitree::robot::ChannelSubscriber<State>;
using LowState = unitree_go::msg::dds_::LowState_;
using LowStateSubscriber = unitree::robot::ChannelSubscriber<LowState>;

constexpr char kSportStateTopic[] = "rt/sportmodestate";
constexpr char kLowStateTopic[] = "rt/lowstate";
constexpr auto kStateMaxAge = std::chrono::milliseconds(500);
constexpr std::array<const char *, 12> kJointNames{
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
};

bool readyMode(std::uint8_t mode) noexcept {
  return mode == 0 || mode == 1 || mode == 2 || mode == 3;
}

std::string modeName(std::uint8_t mode) {
  switch (mode) {
    case 0:
      return "standing";
    case 1:
    case 2:
      return "standing";
    case 3:
      return "walking";
    case 5:
      return "lying";
    case 7:
      return "damping";
    case 8:
      return "recovery";
    case 10:
      return "sitting";
    default:
      return "mode_" + std::to_string(mode);
  }
}

Posture postureForMode(std::uint8_t mode) noexcept {
  switch (mode) {
    case 0:
    case 1:
    case 2:
    case 3:
      return Posture::Standing;
    case 5:
      return Posture::Lying;
    case 7:
      return Posture::Damping;
    case 8:
      return Posture::Recovering;
    case 10:
      return Posture::Sitting;
    default:
      return Posture::Unknown;
  }
}

class Go2 final : public Body {
 public:
  explicit Go2(const Config &config)
      : target_("dds://" + config.network_interface + "/rt/api/sport/request") {
    unitree::robot::ChannelFactory::Instance()->Init(0, config.network_interface);
    client_ = std::make_unique<unitree::robot::go2::SportClient>();
    client_->SetTimeout(static_cast<float>(config.rpc_timeout.count()) / 1000.0F);
    client_->Init();
    avoidance_ = std::make_unique<unitree::robot::go2::ObstaclesAvoidClient>();
    avoidance_->SetTimeout(static_cast<float>(config.rpc_timeout.count()) / 1000.0F);
    avoidance_->Init();
    subscriber_ = std::make_unique<StateSubscriber>(kSportStateTopic);
    subscriber_->InitChannel(std::bind(&Go2::onState, this, std::placeholders::_1), 1);
    low_subscriber_ = std::make_unique<LowStateSubscriber>(kLowStateTopic);
    low_subscriber_->InitChannel(std::bind(&Go2::onLowState, this, std::placeholders::_1), 1);
  }

  ~Go2() override {
    low_subscriber_.reset();
    subscriber_.reset();
    avoidance_.reset();
    client_.reset();
    unitree::robot::ChannelFactory::Instance()->Release();
  }

  Result refresh() override {
    if (!initial_zero_acknowledged_) {
      const int result = client_->StopMove();
      initial_zero_acknowledged_ = result == 0;
      if (!initial_zero_acknowledged_) {
        return resultForCode(result, false, "initial_stop_rejected");
      }
    }
    if (!factory_avoidance_disabled_) {
      // Configure only after the acquisition stop, never in the motion hot path.
      bool enabled = true;
      int result = avoidance_->SwitchGet(enabled);
      if (result != 0) {
        return resultForCode(result, false, "factory_avoidance_query_failed");
      }
      if (enabled) {
        result = avoidance_->SwitchSet(false);
        if (result != 0) {
          return resultForCode(result, false, "factory_avoidance_disable_failed");
        }
        result = avoidance_->SwitchGet(enabled);
        if (result != 0) {
          return resultForCode(result, false, "factory_avoidance_readback_failed");
        }
      }
      if (enabled) {
        return resultForCode(0, false, "factory_avoidance_still_enabled");
      }
      factory_avoidance_disabled_ = true;
      std::fputs("lingtu_driver: Go2 factory avoidance disabled (confirmed)\n", stderr);
    }
    return resultForCode(0, true, "ready");
  }

  Result move(const Velocity &velocity) override {
    const bool zero = std::abs(velocity.vx_mps) <= 1e-9 && std::abs(velocity.vy_mps) <= 1e-9 &&
                      std::abs(velocity.yaw_rps) <= 1e-9;
    if (!zero && !factory_avoidance_disabled_) {
      return resultForCode(0, false, "factory_avoidance_not_configured");
    }
    const int result = zero ? client_->StopMove()
                            : client_->Move(static_cast<float>(velocity.vx_mps),
                                            static_cast<float>(velocity.vy_mps),
                                            static_cast<float>(velocity.yaw_rps));
    return resultForCode(result, result == 0,
                         result == 0 ? "command_accepted" : "command_rejected");
  }

  Result stop() noexcept override {
    int result = -1;
    try {
      result = client_->StopMove();
    } catch (...) {}
    std::uint8_t state_mode = 255;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      state_mode = state_mode_;
    }
    const bool stopped = result == 0;
    initial_zero_acknowledged_ = false;
    factory_avoidance_disabled_ = false;
    Result response;
    response.ok = stopped;
    response.transport_ok = stopped;
    response.accepted = stopped;
    response.state.connected = stopped;
    response.state.ready = false;
    response.state.motors_enabled = readyMode(state_mode);
    response.state.critical_fault = false;
    response.state.control_assured = false;
    response.state.lease_valid = false;
    response.state.initial_zero_acknowledged = false;
    response.state.fsm = modeName(state_mode);
    response.state.reason =
        stopped ? "stop_confirmed" : "stop_rejected:code=" + std::to_string(result);
    response.error = stopped ? "" : response.state.reason;
    response.stop_confirmed = stopped;
    return response;
  }

  Result act(BodyAction action) override {
    int result = -1;
    switch (action) {
      case BodyAction::Stand:
        result = client_->StandUp();
        break;
      case BodyAction::Sit:
        result = client_->Sit();
        break;
      case BodyAction::Recover:
        result = client_->RecoveryStand();
        break;
      case BodyAction::Damp:
        result = client_->Damp();
        break;
    }
    Result response =
        resultForCode(result, result == 0,
                      result == 0 ? std::string("body_action_accepted:") + bodyActionName(action)
                                  : std::string("body_action_rejected:") + bodyActionName(action));
    if (result == 0 && response.transport_ok && !response.state.critical_fault) {
      response.ok = true;
      response.accepted = true;
      response.error.clear();
      response.state.reason = std::string("body_action_accepted:") + bodyActionName(action);
    }
    response.state.ready = false;
    return response;
  }

  Capabilities capabilities() const noexcept override { return {true, true, true, true}; }

  BodyState state() const override {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const bool fresh =
        state_received_ && std::chrono::steady_clock::now() - state_received_at_ <= kStateMaxAge;
    BodyState state;
    state.fresh = fresh;
    state.posture = fresh ? postureForMode(state_mode_) : Posture::Unknown;
    if (fresh) {
      state.velocity = {
          state_velocity_[0],
          state_velocity_[1],
          state_yaw_speed_,
      };
      state.velocity_available = true;
      state.odometry_position_m = {
          state_position_[0],
          state_position_[1],
          state_position_[2],
      };
      state.odometry_position_available = true;
      state.height_m = state_body_height_;
      state.height_available = true;
    }
    return state;
  }

  HealthState health() const override {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const bool fresh =
        state_received_ && std::chrono::steady_clock::now() - state_received_at_ <= kStateMaxAge;
    HealthState state;
    state.fresh = fresh;
    state.healthy = fresh;
    state.reason = fresh ? "sport_state_available" : "sport_state_stale";
    return state;
  }

  std::optional<JointState> jointState() const override {
    std::lock_guard<std::mutex> lock(joint_mutex_);
    if (!joint_state_ || std::chrono::steady_clock::now() - joint_received_at_ > kStateMaxAge) {
      return std::nullopt;
    }
    return joint_state_;
  }

  AdapterDiagnostics diagnostics() const override {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return {
        "go2",
        "unitree_sdk2",
        target_,
        "none",
        "",
        state_received_,
        state_error_code_,
    };
  }

 private:
  void onLowState(const void *message) {
    if (message == nullptr) return;
    const auto &state = *static_cast<const LowState *>(message);
    std::lock_guard<std::mutex> lock(joint_mutex_);
    if (joint_state_ && state.tick() == joint_tick_) return;
    JointState sample;
    sample.robot_model = "go2";
    sample.names.assign(kJointNames.begin(), kJointNames.end());
    sample.position.reserve(kJointNames.size());
    sample.velocity.reserve(kJointNames.size());
    sample.effort.reserve(kJointNames.size());
    for (std::size_t index = 0; index < kJointNames.size(); ++index) {
      const auto &motor = state.motor_state()[index];
      if (!std::isfinite(motor.q()) || !std::isfinite(motor.dq()) || !std::isfinite(motor.tau_est())) return;
      sample.position.push_back(motor.q());
      sample.velocity.push_back(motor.dq());
      sample.effort.push_back(motor.tau_est());
    }
    sample.stamp_s = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    joint_tick_ = state.tick();
    joint_received_at_ = std::chrono::steady_clock::now();
    joint_state_ = std::move(sample);
  }

  void onState(const void *message) {
    if (message == nullptr) {
      return;
    }
    const auto &state = *static_cast<const State *>(message);
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_received_ = true;
    state_error_code_ = state.error_code();
    state_mode_ = state.mode();
    state_position_ = state.position();
    state_velocity_ = state.velocity();
    state_yaw_speed_ = state.yaw_speed();
    state_body_height_ = state.body_height();
    state_received_at_ = std::chrono::steady_clock::now();
  }

  Result resultForCode(int result, bool accepted, const std::string &reason) {
    bool state_received = false;
    std::uint8_t state_mode = 255;
    std::chrono::steady_clock::time_point state_received_at;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      state_received = state_received_;
      state_mode = state_mode_;
      state_received_at = state_received_at_;
    }
    const bool state_fresh =
        state_received && std::chrono::steady_clock::now() - state_received_at <= kStateMaxAge;
    const bool transport_ok = result == 0 && state_fresh;
    const bool mode_ready = readyMode(state_mode);

    Result response;
    response.ok = accepted && transport_ok && mode_ready && initial_zero_acknowledged_ &&
                  factory_avoidance_disabled_;
    response.transport_ok = transport_ok;
    response.accepted = accepted && response.ok;
    response.state.connected = state_fresh;
    response.state.ready = response.ok;
    response.state.motors_enabled = mode_ready;
    response.state.critical_fault = false;
    response.state.control_assured = response.ok;
    response.state.lease_valid = false;
    response.state.initial_zero_acknowledged = initial_zero_acknowledged_;
    response.state.fsm = modeName(state_mode);
    response.state.reason =
        response.ok ? reason : failureReason(result, state_fresh, mode_ready, reason);
    response.error = response.ok ? "" : response.state.reason;
    return response;
  }

  static std::string failureReason(int result, bool state_fresh, bool mode_ready,
                                   const std::string &fallback) {
    if (result != 0) {
      return fallback + ":code=" + std::to_string(result);
    }
    if (!state_fresh) {
      return "sport_state_stale";
    }
    if (!mode_ready) {
      return "sport_mode_not_motion_ready";
    }
    return fallback;
  }

  std::string target_;
  std::unique_ptr<unitree::robot::go2::SportClient> client_;
  std::unique_ptr<unitree::robot::go2::ObstaclesAvoidClient> avoidance_;
  std::unique_ptr<StateSubscriber> subscriber_;
  std::unique_ptr<LowStateSubscriber> low_subscriber_;
  mutable std::mutex joint_mutex_;
  std::optional<JointState> joint_state_;
  std::uint32_t joint_tick_{0};
  std::chrono::steady_clock::time_point joint_received_at_{};
  mutable std::mutex state_mutex_;
  bool state_received_{false};
  std::uint32_t state_error_code_{0};
  std::uint8_t state_mode_{255};
  std::array<float, 3> state_position_{};
  std::array<float, 3> state_velocity_{};
  float state_yaw_speed_{0.0F};
  float state_body_height_{0.0F};
  std::chrono::steady_clock::time_point state_received_at_{};
  bool initial_zero_acknowledged_{false};
  bool factory_avoidance_disabled_{false};
};

}  // namespace

std::unique_ptr<Body> makeBody(const Config &config) {
  if (config.robot != "go2") {
    throw std::runtime_error("Go2 driver binary cannot run robot " + config.robot);
  }
  return std::make_unique<Go2>(config);
}

}  // namespace lingtu::driver

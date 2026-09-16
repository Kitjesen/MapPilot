#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>

#include "body.hpp"
#include "config.hpp"
#include "fake_sdk.hpp"

namespace {
void require(bool value, const char *message) {
  if (!value) throw std::runtime_error(message);
}
double wallSeconds() {
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}
}  // namespace

int main() {
  try {
    fake_unitree::reset();
    lingtu::driver::Config config;
    config.robot = "go2";
    config.network_interface = "eth0";
    auto body = lingtu::driver::makeBody(config);
    require(!body->jointState(), "joint state fabricated before LowState receipt");
    require(static_cast<bool>(fake_unitree::low_state_callback), "rt/lowstate was not subscribed");
    require(fake_unitree::calls.empty(), "telemetry startup issued motion calls");

    unitree_go::msg::dds_::LowState_ state;
    state.tick(42);
    for (std::size_t index = 0; index < 20; ++index) {
      state.motor_state()[index].q(static_cast<float>(index) * 0.125F - 1.0F);
      state.motor_state()[index].dq(static_cast<float>(index) * -0.25F);
      state.motor_state()[index].tau_est(static_cast<float>(index) * 0.5F);
    }
    const double before = wallSeconds();
    fake_unitree::low_state_callback(&state);
    const auto first = body->jointState();
    require(first.has_value(), "fresh LowState not exposed");
    require(first->robot_model == "go2", "robot identity missing");
    require(first->names == std::vector<std::string>{
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"}, "motor order does not match URDF names");
    require(first->position.size() == 12 && first->velocity.size() == 12 && first->effort.size() == 12,
            "non-leg motors included or joint arrays truncated");
    for (std::size_t index = 0; index < 12; ++index) {
      require(first->position[index] == state.motor_state()[index].q(), "joint q changed");
      require(first->velocity[index] == state.motor_state()[index].dq(), "joint dq changed");
      require(first->effort[index] == state.motor_state()[index].tau_est(), "joint effort changed");
    }
    require(first->stamp_s >= before && first->stamp_s <= wallSeconds(), "sample timestamp is not receive wall time");

    state.motor_state()[0].q(0.375F);
    fake_unitree::low_state_callback(&state);
    require(body->jointState()->stamp_s == first->stamp_s, "duplicate tick refreshed timestamp");
    require(body->jointState()->position[0] == first->position[0], "duplicate tick changed sample");
    state.tick(43);
    fake_unitree::low_state_callback(&state);
    const auto second = body->jointState();
    require(second->position[0] == 0.375, "new sample was not captured");
    state.tick(44);
    state.motor_state()[1].q(std::numeric_limits<float>::quiet_NaN());
    fake_unitree::low_state_callback(&state);
    require(body->jointState()->stamp_s == second->stamp_s, "invalid sample refreshed timestamp");
    std::this_thread::sleep_for(std::chrono::milliseconds(550));
    require(!body->jointState(), "stale joint state exposed");
    state.tick(43);
    state.motor_state()[1].q(0.5F);
    fake_unitree::low_state_callback(&state);
    require(!body->jointState(), "duplicate old tick revived stale state");
    state.tick(45);
    fake_unitree::low_state_callback(&state);
    require(body->jointState().has_value(), "new sample did not recover telemetry");
    require(fake_unitree::calls.empty(), "joint telemetry issued motion calls");
    std::cout << "Go2 measured joints, motor order, sample timestamps and expiry passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}

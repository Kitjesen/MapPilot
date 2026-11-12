#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace lingtu::message {

struct ExternalJointCommand {
  double q{0};
  double dq{0};
  double kp{0};
  double kd{0};
  double tau{0};
};

struct ExternalFrame {
  std::string session_id;
  std::uint64_t sequence{0};
  std::uint64_t input_window{0};
  std::array<ExternalJointCommand, 16> joints{};
};

struct ExternalState {
  std::string session_id;
  bool selected{false};
  bool active{false};
  bool motors_enabled{false};
  std::uint64_t last_sequence{0};
  std::string reason{"external_unavailable"};
  std::uint64_t input_window{0};
  bool fresh{false};
  std::uint32_t joint_count{0};
  bool motor_disable_confirmed{false};
};

}  // namespace lingtu::message

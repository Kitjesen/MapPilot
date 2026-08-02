#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <string>

#include "nav_kernel/types.hpp"

namespace lingtu::nav::endpoint {

struct TeleopAdmissionRequest {
  nav_kernel::Twist command{};
  double source_stamp_s{0.0};
  std::string decode_error;
};

struct TeleopAdmissionContext {
  double receive_s{0.0};
  bool motion_allowed{false};
  bool autonomy_mode{true};
  bool allow_takeover{false};
  bool operator_takeover_latched{false};
  bool has_active_teleop{false};
  double max_age_s{0.0};
  double future_tolerance_s{0.0};
  double min_motion{0.0};
  bool publish_cmd_vel{false};
};

struct TeleopAdmissionActions {
  std::function<void()> hold_operator_takeover;
  std::function<void()> stop_control;
  std::function<bool(const nav_kernel::Twist &, double)> begin_operator_takeover;
  std::function<void()> mark_operator_resume_required;
  std::function<bool(const std::string &)> clear_motion;
  std::function<void(const std::string &)> pause_inspection;
  std::function<bool(const nav_kernel::Twist &, double)> accept_teleop;
  std::function<bool()> publish_zero;
};

struct TeleopAdmissionDelta {
  std::uint64_t command_count{1};
  std::uint64_t frame_rejected{0};
  std::uint64_t output_count{0};
  std::uint64_t stop_count{0};
  std::uint64_t limited_count{0};
  std::optional<std::string> frame_error;
};

struct TeleopAdmissionObservation {
  std::optional<bool> seen;
  std::optional<bool> fresh;
  std::optional<bool> published;
  std::optional<bool> stopped;
  std::optional<bool> limited;
  std::optional<std::string> reason;
  std::optional<nav_kernel::Twist> request;
  std::optional<nav_kernel::Twist> output;
  std::optional<double> age_s;
};

struct TeleopAdmissionResult {
  bool accepted{false};
  std::string reason;
  bool update_receive_timestamp{false};
  TeleopAdmissionDelta delta;
  TeleopAdmissionObservation observation;
};

class TeleopAdmissionController {
 public:
  explicit TeleopAdmissionController(TeleopAdmissionActions actions);

  TeleopAdmissionResult admit(const TeleopAdmissionRequest &request,
                              const TeleopAdmissionContext &context) const;

 private:
  TeleopAdmissionActions actions_;
};

}  // namespace lingtu::nav::endpoint

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

TeleopAdmissionResult admitTeleop(const TeleopAdmissionRequest &request);

}  // namespace lingtu::nav::endpoint

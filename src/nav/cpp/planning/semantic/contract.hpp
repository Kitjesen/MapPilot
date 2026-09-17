#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "planning/global/contract.hpp"

namespace lingtu::nav::semantic {

struct CameraView {
  double x{0.0}, y{0.0}, yaw{0.0};
  double range_m{0.0}, horizontal_fov_rad{0.0};
};

struct ViewQuery {
  std::string request_id;
  std::string boot_id;
  plan::MapIdentity map;
  // Zero is allowed only with an empty history on the first query.
  std::uint64_t frame_epoch{0U};
  double reference_z{0.0};
  // Effective camera model for the next observation, supplied by the caller.
  double camera_range_m{0.0};
  double camera_horizontal_fov_rad{0.0};
  std::vector<CameraView> views;
};

struct ViewCandidate {
  plan::GlobalPlanPoint position;
  double yaw{0.0}, score{0.0}, route_cost_m{0.0};
  std::uint32_t visible_cells{0U};
};

struct ViewResult {
  std::string request_id;
  std::string boot_id;
  plan::MapIdentity map;
  std::uint64_t frame_epoch{0U};
  double timestamp_s{0.0};
  double reference_z{0.0};
  bool available{false};
  bool geometry_exhausted{false};
  std::string reason;
  std::vector<ViewCandidate> candidates;
};

}  // namespace lingtu::nav::semantic

#pragma once

#include "slam.hpp"
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace lingtu::slam {

// Shared by the native status publisher and the diagnostics contract test.
inline std::string ltlioGeometryJson(const SlamOutputs& out) {
  std::ostringstream json;
  json << std::setprecision(std::numeric_limits<double>::max_digits10);
  const auto number = [&](double value) {
    if (std::isfinite(value)) json << value;
    else json << "null";
  };
  const auto values = [&](const std::array<double, 6>& data) {
    json << '[';
    for (std::size_t axis = 0; axis < data.size(); ++axis) {
      if (axis) json << ',';
      number(data[axis]);
    }
    json << ']';
  };
  json << "{\"available\":" << (out.ltlio_geometry_available ? "true" : "false")
       << ",\"detected\":" << (out.ltlio_degeneracy_detected ? "true" : "false")
       << ",\"degenerate_dof_count\":" << out.ltlio_degenerate_dof_count
       << ",\"direction_frame\":\"imu\",\"condition_number\":";
  if (out.ltlio_geometry_available) number(out.ltlio_condition_number);
  else json << "null";
  json << ",\"min_eigenvalue\":";
  if (out.ltlio_geometry_available) number(out.ltlio_min_eigenvalue);
  else json << "null";
  json << ",\"max_eigenvalue\":";
  if (out.ltlio_geometry_available) number(out.ltlio_max_eigenvalue);
  else json << "null";
  json << ",\"rotation_scale_m\":";
  if (out.ltlio_geometry_available) number(out.ltlio_rotation_scale_m);
  else json << "null";
  json << ",\"eigenvalues\":";
  if (out.ltlio_geometry_available) values(out.ltlio_geometry_eigenvalues);
  else json << "null";
  json << ",\"principal_directions\":";
  if (out.ltlio_geometry_available) {
    json << '[';
    for (std::size_t mode = 0; mode < out.ltlio_principal_directions.size(); ++mode) {
      if (mode) json << ',';
      values(out.ltlio_principal_directions[mode]);
    }
    json << ']';
  } else json << "null";
  json << ",\"categories\":";
  if (out.ltlio_geometry_available) {
    json << '[';
    for (std::size_t mode = 0; mode < out.ltlio_geometry_categories.size(); ++mode) {
      if (mode) json << ',';
      json << std::quoted(out.ltlio_geometry_categories[mode]);
    }
    json << ']';
  } else json << "null";
  json << ",\"support_ratios\":";
  if (out.ltlio_geometry_available) values(out.ltlio_geometry_support_ratios);
  else json << "null";
  json << ",\"update_applied\":" << (out.ltlio_geometry_update_applied ? "true" : "false")
       << ",\"guidance\":" << std::quoted(out.ltlio_geometry_guidance)
       << ",\"effective_ratio\":";
  number(out.ltlio_effective_ratio);
  json << ",\"pos_cov_trace\":";
  number(out.ltlio_pos_cov_trace);
  json << ",\"iter_num\":" << out.ltlio_iter_num
       << ",\"converged\":" << (out.ltlio_converged ? "true" : "false") << '}';
  return json.str();
}

}  // namespace lingtu::slam

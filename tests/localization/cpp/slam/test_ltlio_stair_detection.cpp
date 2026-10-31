#include "ltlio/detail/ElevatorProcess.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
namespace lio = lingtu::ltlio;
namespace detail = lingtu::ltlio::detail;
using Vec = Eigen::Vector3d;
constexpr double kPi = 3.14159265358979323846;

// Ideal instantaneous range snapshots of a fixed world. This exercises the
// detector, not the full estimator, a real LiDAR scan pattern or robot physics.
struct Surface {
  Vec normal;
  double offset;
  Vec lower;
  Vec upper;
};

void plane(std::vector<Surface>& surfaces, const Vec& normal, double offset,
           const Vec& lower, const Vec& upper) {
  surfaces.push_back({normal, offset, lower, upper});
}

std::vector<Surface> room(double half_length, double half_width, double height) {
  std::vector<Surface> surfaces;
  const Vec lower(-half_length, -half_width, 0.0);
  const Vec upper(half_length, half_width, height);
  for (const double x : {-half_length, half_length})
    plane(surfaces, Vec::UnitX(), x, lower, upper);
  for (const double y : {-half_width, half_width})
    plane(surfaces, Vec::UnitY(), y, lower, upper);
  for (const double z : {0.0, height})
    plane(surfaces, Vec::UnitZ(), z, lower, upper);
  return surfaces;
}

std::vector<Surface> stairwell() {
  std::vector<Surface> surfaces;
  const Vec lower(-1.0, -0.7, 0.0);
  const Vec upper(3.2, 0.7, 4.0);
  for (const double y : {-0.7, 0.7})
    plane(surfaces, Vec::UnitY(), y, lower, upper);
  plane(surfaces, Vec::UnitX(), -1.0, lower, upper);
  plane(surfaces, Vec::UnitX(), 3.2, lower, upper);
  // The underside of the next parallel flight keeps 2.4 m vertical clearance.
  plane(surfaces, Vec(-0.5, 0.0, 1.0), 2.4, lower, upper);
  plane(surfaces, Vec::UnitZ(), 0.0, lower, Vec(0.0, 0.7, 0.0));
  for (int i = 0; i < 8; ++i) {
    const double x = i * 0.3;
    const double z = (i + 1) * 0.15;
    plane(surfaces, Vec::UnitZ(), z, Vec(x, -0.7, z), Vec(x + 0.3, 0.7, z));
    plane(surfaces, Vec::UnitX(), x,
          Vec(x, -0.7, z - 0.15), Vec(x, 0.7, z));
  }
  plane(surfaces, Vec::UnitZ(), 1.2, Vec(2.4, -0.7, 1.2), Vec(3.2, 0.7, 1.2));
  return surfaces;
}

detail::PointCloudXYZI scan(const std::vector<Surface>& surfaces, const Vec& origin) {
  detail::PointCloudXYZI cloud;
  cloud.reserve(360 * 21);
  for (int elevation = -50; elevation <= 50; elevation += 5) {
    const double pitch = elevation * kPi / 180.0;
    for (int azimuth = 0; azimuth < 360; ++azimuth) {
      const double yaw = azimuth * kPi / 180.0;
      const Vec direction(std::cos(pitch) * std::cos(yaw),
                          std::cos(pitch) * std::sin(yaw), std::sin(pitch));
      double nearest = std::numeric_limits<double>::infinity();
      for (const auto& surface : surfaces) {
        const double denominator = surface.normal.dot(direction);
        if (std::abs(denominator) < 1e-12) continue;
        const double distance = (surface.offset - surface.normal.dot(origin)) / denominator;
        if (distance <= 0.0 || distance >= nearest) continue;
        const Vec point = origin + distance * direction;
        if ((point.array() >= surface.lower.array() - 1e-9).all() &&
            (point.array() <= surface.upper.array() + 1e-9).all()) nearest = distance;
      }
      if (!std::isfinite(nearest) || nearest < 0.8) continue;
      pcl::PointXYZINormal point;
      point.x = static_cast<float>(nearest * direction.x());
      point.y = static_cast<float>(nearest * direction.y());
      point.z = static_cast<float>(nearest * direction.z());
      point.intensity = 1.0f;
      cloud.push_back(point);
    }
  }
  return cloud;
}

struct Result {
  bool triggered = false;
  double first_trigger_s = -1.0;
  double distance_at_trigger_m = 0.0;
  double rise_at_trigger_m = 0.0;
  double quantile_min_m = std::numeric_limits<double>::infinity();
  double quantile_max_m = 0.0;
  bool request_preserved = true;
};

struct Scenario {
  std::string name;
  bool enabled = true;
  bool automatic = true;
  bool stairs = false;
  bool active_transport = false;
  bool explicit_request = false;
  bool unordered_excursion = false;
  double duration_s = 4.0;
  Vec initial_position = Vec(0.0, 0.0, 0.7);
  std::function<Vec(double)> displacement = [](double) { return Vec::Zero(); };
};

Result run(const Scenario& scenario) {
  lio::Config config;
  config.elevator_enabled = scenario.enabled;
  config.automatic_door = scenario.automatic;
  config.elevator_entry_range_m = 3.0;
  config.elevator_entry_hold_s = 2.0;
  config.elevator_entry_motion_m = 0.15;
  auto context = std::make_unique<detail::Context>(config);
  detail::ElevatorProcess detector(*context);
  const auto surfaces = scenario.stairs ? stairwell() : room(1.0, 0.7, 2.4);
  const auto corridor = room(3.0, 0.7, 2.4);
  auto state = detail::State{};
  state.q = Eigen::Quaterniond::Identity();
  state.p = scenario.initial_position;
  state.v.setZero();
  state.bw.setZero();
  state.ba.setZero();
  state.g = Vec(0.0, 0.0, -9.80665);
  state.P = Eigen::MatrixXd::Identity(detail::STATE_TOTAL, detail::STATE_TOTAL) * 0.01;
  state.z = state.vz = state.az = 0.0;
  state.in_elevator = scenario.active_transport;
  context->ELEVATOR_TRIGGER = scenario.explicit_request || scenario.active_transport;
  Result result;
  const bool initially_requested = context->ELEVATOR_TRIGGER;
  for (int frame = 0; frame <= static_cast<int>(std::lround(scenario.duration_s * 10.0)); ++frame) {
    const double elapsed = frame * 0.1;
    state.time = 10.0 + elapsed;
    state.p = scenario.initial_position + scenario.displacement(elapsed);
    // In a co-moving cabin the measured scene stays fixed in sensor space.
    // An entry-motion limit must not cancel an already active transport.
    const Vec scan_origin = scenario.active_transport ? scenario.initial_position : state.p;
    const auto cloud = scan(scenario.name == "active_corridor" ||
        scenario.name == "walk_then_stop" ? corridor : surfaces, scan_origin);
    if (cloud.size() < 1000) throw std::runtime_error("insufficient ideal range returns");
    detector.processDoorDetector(cloud, state.time, state, true);
    if (scenario.unordered_excursion && frame == 5) {
      // Rejected timestamps must not restart the accepted hold or its origin.
      auto late_state = state;
      late_state.p.x() += 1.0;
      detector.processDoorDetector(cloud, state.time, late_state, true);
      detector.processDoorDetector(cloud, state.time - 0.25, late_state, true);
    }
    const double quantile = detector.doorLastFilteredMaxDistance();
    result.quantile_min_m = std::min(result.quantile_min_m, quantile);
    result.quantile_max_m = std::max(result.quantile_max_m, quantile);
    if (initially_requested && !context->ELEVATOR_TRIGGER) result.request_preserved = false;
    if (context->ELEVATOR_TRIGGER && !result.triggered) {
      result.triggered = true;
      result.first_trigger_s = elapsed;
      result.distance_at_trigger_m = (state.p - scenario.initial_position).norm();
      result.rise_at_trigger_m = state.p.z() - scenario.initial_position.z();
    }
  }
  std::cout << std::setprecision(12) << "{\"case\":\"" << scenario.name
            << "\",\"enabled\":" << scenario.enabled << ",\"automatic\":" << scenario.automatic
            << ",\"triggered\":" << result.triggered
            << ",\"first_trigger_s\":" << result.first_trigger_s
            << ",\"distance_at_trigger_m\":" << result.distance_at_trigger_m
            << ",\"rise_at_trigger_m\":" << result.rise_at_trigger_m
            << ",\"quantile_min_m\":" << result.quantile_min_m
            << ",\"quantile_max_m\":" << result.quantile_max_m
            << ",\"request_preserved\":" << result.request_preserved << "}\n";
  return result;
}
} // namespace

int main(int argc, char** argv) {
  try {
    const bool observe_only = argc == 2 && std::string(argv[1]) == "--observe";
    Scenario scenario;
    scenario.name = "closed_cabin";
    const auto cabin = run(scenario);
    scenario.name = "active_stairs_up";
    scenario.stairs = true;
    scenario.initial_position = Vec(0.1, 0.0, 0.7);
    scenario.displacement = [](double t) { return Vec(0.4 * t, 0.0, 0.2 * t); };
    const auto stairs_up = run(scenario);
    scenario.name = "active_stairs_down";
    scenario.initial_position = Vec(1.9, 0.0, 1.6);
    scenario.displacement = [](double t) { return Vec(-0.4 * t, 0.0, -0.2 * t); };
    const auto stairs_down = run(scenario);
    scenario = Scenario{};
    scenario.name = "active_corridor";
    scenario.displacement = [](double t) { return Vec(0.2 * t, 0.0, 0.0); };
    const auto corridor = run(scenario);
    scenario.name = "vertical_motion";
    scenario.displacement = [](double t) { return Vec(0.0, 0.0, 0.2 * t); };
    const auto vertical = run(scenario);
    scenario.name = "walk_then_stop";
    scenario.duration_s = 6.0;
    scenario.displacement = [](double t) { return Vec(0.2 * std::min(t, 2.0), 0.0, 0.0); };
    const auto stopped = run(scenario);
    scenario = Scenario{};
    scenario.name = "standing_jitter";
    scenario.displacement = [](double t) {
      return Vec(0.02 * std::sin(7.0 * t), 0.02 * (1.0 - std::cos(5.0 * t)),
                 0.01 * std::sin(9.0 * t));
    };
    const auto jitter = run(scenario);
    scenario.name = "out_and_back";
    // A 24 cm excursion finishes before the original 2 s decision. Looking
    // only at the endpoint displacement would incorrectly accept this window.
    scenario.displacement = [](double t) {
      const double x = t <= 0.4 ? 0.6 * t : std::max(0.0, 0.6 * (0.8 - t));
      return Vec(x, 0.0, 0.0);
    };
    const auto returned = run(scenario);
    scenario = Scenario{};
    scenario.name = "motion_at_limit";
    scenario.displacement = [](double t) { return Vec(t > 0.0 ? 0.15 : 0.0, 0.0, 0.0); };
    const auto at_limit = run(scenario);
    scenario.name = "motion_above_limit";
    scenario.displacement = [](double t) { return Vec(t > 0.0 ? 0.1501 : 0.0, 0.0, 0.0); };
    const auto above_limit = run(scenario);
    scenario = Scenario{};
    scenario.name = "unordered_excursion";
    scenario.unordered_excursion = true;
    const auto unordered = run(scenario);
    scenario = Scenario{};
    scenario.name = "disabled";
    scenario.enabled = false;
    scenario.automatic = false;
    const auto disabled = run(scenario);
    scenario.name = "manual_without_request";
    scenario.enabled = true;
    const auto manual = run(scenario);
    scenario.name = "manual_explicit_request";
    scenario.explicit_request = true;
    scenario.displacement = [](double t) { return Vec(0.2 * t, 0.0, 0.0); };
    const auto requested = run(scenario);
    scenario.name = "active_transport";
    scenario.automatic = true;
    scenario.active_transport = true;
    scenario.displacement = [](double t) { return Vec(0.0, 0.0, 0.5 * t); };
    const auto active = run(scenario);
    if (!cabin.triggered || !jitter.triggered || disabled.triggered || manual.triggered ||
        !requested.triggered || !requested.request_preserved ||
        !active.triggered || !active.request_preserved)
      throw std::runtime_error("detector mode/positive-control fixture failed");
    if (!observe_only && (stairs_up.triggered || stairs_down.triggered ||
                          corridor.triggered || vertical.triggered))
      throw std::runtime_error("active motion through a fixed world was misclassified as elevator entry");
    if (!observe_only && (!stopped.triggered || stopped.first_trigger_s < 3.2))
      throw std::runtime_error("movement did not restart the entry hold before a later stop");
    if (!observe_only && (!returned.triggered || returned.first_trigger_s < 2.7))
      throw std::runtime_error("returning to the initial point incorrectly reused the old entry hold");
    if (!at_limit.triggered || std::abs(at_limit.first_trigger_s - 2.0) > 1e-9 ||
        !above_limit.triggered || above_limit.first_trigger_s < 2.05)
      throw std::runtime_error("entry excursion boundary did not distinguish equal from above limit");
    if (!unordered.triggered || std::abs(unordered.first_trigger_s - 2.0) > 1e-9)
      throw std::runtime_error("duplicate or out-of-order movement changed the entry hold");
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}

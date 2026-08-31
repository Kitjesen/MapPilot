#include "status/nav_status_writer.hpp"

#include <cmath>
#include <iomanip>
#include <numeric>
#include <ostream>
#include <sstream>
#include <utility>

#include "planning/local/planner.hpp"
#include "safety/command.hpp"
#include "status/nav_status_debug_sampling.hpp"

namespace lingtu::nav::endpoint {
namespace {

constexpr std::size_t kStatusPathPointLimit = 512;

std::string jsonEscape(const std::string &input) {
  std::string out;
  out.reserve(input.size() + 8);
  for (const char c : input) {
    switch (c) {
      case '\\':
        out += "\\\\";
        break;
      case '"':
        out += "\\\"";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        out += c;
        break;
    }
  }
  return out;
}

void writeVec3Json(std::ostream &out, const nav_kernel::Vec3 &point) {
  out << "[" << point.x << ", " << point.y << ", " << point.z << "]";
}

void writeTwistJson(std::ostream &out, const nav_kernel::Twist &twist) {
  out << "{\"vx\": " << twist.vx << ", "
      << "\"vy\": " << twist.vy << ", "
      << "\"wz\": " << twist.wz << "}";
}

void writeOperatorMotionDiagnosticsJson(std::ostream &out, const StatusWriterConfig &cfg,
                                        bool interface_enabled,
                                        const OperatorMotionTransportDiagnostics &transport) {
  const auto &ack = transport.last_ack;
  const auto &status = transport.status;
  out << "{"
      << "\"schema_version\": 1, "
      << "\"interface_enabled\": " << (interface_enabled ? "true" : "false") << ", "
      << "\"authority_owner\": \"native_endpoint\", "
      << "\"control_mode\": \"" << jsonEscape(cfg.control_mode) << "\", "
      << "\"allow_teleop_takeover\": " << (cfg.allow_teleop_takeover ? "true" : "false") << ", "
      << "\"control_ack_scope\": \"claim_hold_release\", "
      << "\"sample_evidence\": \"status_sequences\", "
      << "\"ack_sent\": " << transport.ack_sent << ", "
      << "\"ack_publish_failed\": " << transport.ack_publish_failed << ", "
      << "\"status_sent\": " << transport.status_sent << ", "
      << "\"status_publish_failed\": " << transport.status_publish_failed << ", "
      << "\"last_ack\": {"
      << "\"observed\": " << (ack.observed ? "true" : "false") << ", "
      << "\"published\": " << (ack.published ? "true" : "false") << ", "
      << "\"source_id\": \"" << jsonEscape(ack.source_id) << "\", "
      << "\"source_epoch\": " << ack.source_epoch << ", "
      << "\"source_sequence\": " << ack.source_sequence << ", "
      << "\"request_id\": \"" << jsonEscape(ack.request_id) << "\", "
      << "\"action\": " << ack.action << ", "
      << "\"accepted\": " << (ack.accepted ? "true" : "false") << ", "
      << "\"reason\": \"" << jsonEscape(ack.reason) << "\", "
      << "\"accepted_sequence\": " << ack.accepted_sequence << ", "
      << "\"final_output_sequence\": " << ack.final_output_sequence << "}, "
      << "\"status\": {"
      << "\"observed\": " << (status.observed ? "true" : "false") << ", "
      << "\"published\": " << (status.published ? "true" : "false") << ", "
      << "\"active_source_id\": \"" << jsonEscape(status.active_source_id) << "\", "
      << "\"active_source_epoch\": " << status.active_source_epoch << ", "
      << "\"has_active_authority\": " << (status.has_active_authority ? "true" : "false") << ", "
      << "\"holding\": " << (status.holding ? "true" : "false") << ", "
      << "\"has_active_sample\": " << (status.has_active_sample ? "true" : "false") << ", "
      << "\"last_sample_sequence\": " << status.last_sample_sequence << ", "
      << "\"admitted_sequence\": " << status.admitted_sequence << ", "
      << "\"final_output_sequence\": " << status.final_output_sequence << ", "
      << "\"authority_reason\": \"" << jsonEscape(status.authority_reason) << "\", "
      << "\"input_gate_reason\": \"" << jsonEscape(status.input_gate_reason) << "\", "
      << "\"teleop_output\": ";
  writeTwistJson(out, status.teleop_output);
  out << ", \"final_cmd_vel\": ";
  writeTwistJson(out, status.final_cmd_vel);
  out << "}}";
}

void writePathJson(std::ostream &out, const std::vector<nav_kernel::Vec3> &path) {
  out << "[";
  const std::size_t count = std::min(path.size(), kStatusPathPointLimit);
  for (std::size_t i = 0; i < count; ++i) {
    if (i > 0) {
      out << ", ";
    }
    writeVec3Json(out, path[i]);
  }
  out << "]";
}

void writeDynamicClustersJson(std::ostream &out, const std::vector<DynamicCluster> &clusters) {
  out << "[";
  for (std::size_t i = 0; i < clusters.size(); ++i) {
    const auto &cluster = clusters[i];
    if (i > 0) {
      out << ", ";
    }
    out << "{"
        << "\"id\": " << cluster.id << ", "
        << "\"centroid\": [" << cluster.x << ", " << cluster.y << ", " << cluster.z << "], "
        << "\"velocity\": [" << cluster.vx << ", " << cluster.vy << ", " << cluster.vz << "], "
        << "\"age_s\": " << cluster.age_s << ", "
        << "\"confidence\": " << cluster.confidence << ", "
        << "\"cells\": " << cluster.cells << "}";
  }
  out << "]";
}

void writeLocalCandidatesJson(std::ostream &out,
                              const nav_kernel::LocalPlannerDebugSnapshot &snapshot) {
  out << "{";
  out << "\"valid\": " << (snapshot.valid ? "true" : "false") << ", ";
  out << "\"frame_id\": \"map\", ";
  out << "\"representation\": \"sampled_best_group_per_valid_rotation\", ";
  out << "\"state_semantics\": \"first_zero_survivor_gate_or_soft_cost\", ";
  out << "\"timestamp_s\": " << snapshot.timestampS << ", ";
  out << "\"path_scale\": " << snapshot.pathScale << ", ";
  out << "\"path_range_m\": " << snapshot.pathRange << ", ";
  out << "\"relative_goal_distance_m\": " << snapshot.relativeGoalDistanceM << ", ";
  out << "\"traversability_soft_cost\": " << snapshot.traversabilitySoftCost << ", ";
  out << "\"traversability_hard_cost\": " << snapshot.traversabilityHardCost << ", ";
  out << "\"valid_rotation_count\": " << snapshot.validRotationCount << ", ";
  out << "\"returned_count\": " << snapshot.candidates.size() << ", ";
  out << "\"truncated\": "
      << (snapshot.candidates.size() <
                  static_cast<std::size_t>(std::max(0, snapshot.validRotationCount))
              ? "true"
              : "false")
      << ", ";
  out << "\"selected_flat_group_id\": " << snapshot.selectedGroupId << ", ";
  out << "\"selected_rotation_index\": "
      << (snapshot.selectedGroupId >= 0 ? snapshot.selectedGroupId / nav_kernel::kGroupNum : -1)
      << ", ";
  out << "\"selected_group_id\": "
      << (snapshot.selectedGroupId >= 0 ? snapshot.selectedGroupId % nav_kernel::kGroupNum : -1)
      << ", ";
  out << "\"candidates\": [";
  for (std::size_t index = 0; index < snapshot.candidates.size(); ++index) {
    const auto &candidate = snapshot.candidates[index];
    if (index > 0) {
      out << ", ";
    }
    out << "{";
    out << "\"rotation_index\": " << candidate.rotationIndex << ", ";
    out << "\"rotation_deg\": " << candidate.rotationDeg << ", ";
    out << "\"group_id\": " << candidate.groupId << ", ";
    out << "\"state\": \"" << nav_kernel::localCandidateStateName(candidate.state) << "\", ";
    out << "\"dominant_state\": \"" << nav_kernel::localCandidateStateName(candidate.state)
        << "\", ";
    out << "\"rotation_allowed\": " << (candidate.rotationAllowed ? "true" : "false") << ", ";
    out << "\"selected\": " << (candidate.selected ? "true" : "false") << ", ";
    out << "\"score\": " << candidate.aggregateScore << ", ";
    out << "\"terrain_risk\": " << candidate.terrainRisk << ", ";
    out << "\"total_path_count\": " << candidate.totalPathCount << ", ";
    out << "\"collision_free_path_count\": " << candidate.collisionFreePathCount << ", ";
    out << "\"direction_allowed_path_count\": " << candidate.directionAllowedPathCount << ", ";
    out << "\"terrain_allowed_path_count\": " << candidate.terrainAllowedPathCount << ", ";
    out << "\"direction_scored_path_count\": " << candidate.directionScoredPathCount << ", ";
    out << "\"height_cost_allowed_path_count\": " << candidate.heightCostAllowedPathCount << ", ";
    out << "\"terrain_soft_penalized_path_count\": " << candidate.terrainSoftPenalizedPathCount
        << ", ";
    out << "\"contributing_path_count\": " << candidate.contributingPathCount << ", ";
    out << "\"path\": ";
    writePathJson(out, candidate.path);
    out << "}";
  }
  out << "]}";
}

std::vector<float> copyCollisionDebugPoints(std::size_t point_limit,
                                            const LocalCollisionStatusView &collision,
                                            const SensorOrigin &sensor_origin) {
  if (point_limit == 0U || collision.occupied_xyz == nullptr || collision.occupied_count == 0U) {
    return {};
  }
  std::vector<std::size_t> indices(collision.occupied_count);
  std::iota(indices.begin(), indices.end(), 0U);
  const bool has_debug_origin =
      sensor_origin.valid && std::isfinite(sensor_origin.x) && std::isfinite(sensor_origin.y);
  const auto sampled = detail::sampleDebugIndices(
      indices, point_limit, has_debug_origin, [&](std::size_t index) {
        const std::size_t offset = index * 3U;
        const double dx = static_cast<double>(collision.occupied_xyz[offset]) - sensor_origin.x;
        const double dy = static_cast<double>(collision.occupied_xyz[offset + 1U]) - sensor_origin.y;
        return dx * dx + dy * dy;
      });
  std::vector<float> points;
  points.reserve(sampled.size() * 3U);
  for (const std::size_t index : sampled) {
    const float *point = collision.occupied_xyz + index * 3U;
    points.insert(points.end(), point, point + 3U);
  }
  return points;
}

void writeCollisionMapJson(std::ostream &out, std::size_t point_limit,
                           const LocalCollisionStatusView &collision,
                           std::size_t total_points, const std::vector<float> &occupied_xyz,
                           const SensorOrigin &sensor_origin) {
  const bool enabled = point_limit > 0;
  const std::size_t returned_points = occupied_xyz.size() / 3U;
  const bool has_debug_origin =
      sensor_origin.valid && std::isfinite(sensor_origin.x) && std::isfinite(sensor_origin.y);
  const bool truncated = returned_points < total_points;
  out << "{";
  out << "\"enabled\": " << (enabled ? "true" : "false") << ", ";
  out << "\"frame_id\": \"map\", ";
  out << "\"live\": " << (collision.live ? "true" : "false") << ", ";
  out << "\"complete\": " << (collision.complete ? "true" : "false") << ", ";
  out << "\"resolution_m\": " << collision.resolution << ", ";
  out << "\"generation\": " << collision.generation << ", ";
  out << "\"observation_sequence\": " << collision.observation_sequence << ", ";
  out << "\"reset_epoch\": " << collision.reset_epoch << ", ";
  out << "\"stamp_s\": " << collision.stamp_s << ", ";
  out << "\"aabb_min\": ";
  writeVec3Json(out, collision.aabb_min);
  out << ", \"aabb_max\": ";
  writeVec3Json(out, collision.aabb_max);
  out << ", \"occupied_points_total\": " << total_points << ", ";
  out << "\"occupied_points_returned\": " << returned_points << ", ";
  out << "\"occupied_points_truncated\": " << (truncated ? "true" : "false") << ", ";
  out << "\"sampling\": \""
      << (truncated && has_debug_origin ? "nearest_sensor_origin"
          : truncated                   ? "uniform_source_order"
                                        : "complete")
      << "\", ";
  out << "\"occupied_points\": [";
  for (std::size_t index = 0; index < returned_points; ++index) {
    if (index > 0) {
      out << ", ";
    }
    const std::size_t offset = index * 3U;
    out << "[" << occupied_xyz[offset] << ", " << occupied_xyz[offset + 1U] << ", "
        << occupied_xyz[offset + 2U] << "]";
  }
  out << "]}";
}

void writeLocalMapJson(std::ostream &out, const StatusWriterConfig &cfg,
                       const std::vector<float> &obstacle_xyzh,
                       const TraversabilityGrid &traversability,
                       const LocalCollisionStatusView &collision,
                       std::size_t collision_total_points,
                       const std::vector<float> &collision_occupied_xyz,
                       const SensorOrigin &sensor_origin, bool obstacle_points_fresh,
                       bool traversability_fresh) {
  const std::size_t point_limit = cfg.local_map_debug_point_limit;
  const bool enabled = point_limit > 0;
  const std::size_t source_points = obstacle_xyzh.size() / 4;
  const bool cmu_height_filter = cfg.local_planner == "cmu";
  std::vector<std::size_t> obstacle_indices;
  obstacle_indices.reserve(source_points);
  for (std::size_t index = 0; index < source_points; ++index) {
    const float height = obstacle_xyzh[index * 4U + 3U];
    if (cmu_height_filter &&
        (!std::isfinite(height) || height < cfg.planner_obstacle_height_min_m ||
         height > cfg.planner_obstacle_height_max_m)) {
      continue;
    }
    obstacle_indices.push_back(index);
  }
  const std::size_t total_points = obstacle_indices.size();
  std::vector<std::size_t> risk_indices;
  if (enabled) {
    risk_indices.reserve(std::min(point_limit, traversability.values.size()));
    for (std::size_t index = 0; index < traversability.values.size(); ++index) {
      if (std::abs(traversability.values[index]) > 0.5f) {
        risk_indices.push_back(index);
      }
    }
  }
  const bool has_debug_origin =
      sensor_origin.valid && std::isfinite(sensor_origin.x) && std::isfinite(sensor_origin.y);
  const auto sampled_obstacles =
      enabled
          ? detail::sampleDebugIndices(
                obstacle_indices, point_limit, has_debug_origin,
                [&](std::size_t index) {
                  const std::size_t offset = index * 4;
                  const double dx = static_cast<double>(obstacle_xyzh[offset]) - sensor_origin.x;
                  const double dy =
                      static_cast<double>(obstacle_xyzh[offset + 1]) - sensor_origin.y;
                  return dx * dx + dy * dy;
                })
          : std::vector<std::size_t>{};
  const auto sampled_risk =
      enabled
          ? detail::sampleDebugIndices(
                risk_indices, point_limit, has_debug_origin,
                [&](std::size_t flat) {
                  const std::size_t cols =
                      traversability.cols > 0 ? static_cast<std::size_t>(traversability.cols) : 1;
                  const std::size_t row = flat / cols;
                  const std::size_t col = flat % cols;
                  const double x = traversability.origin_x +
                                   (static_cast<double>(col) + 0.5) * traversability.resolution;
                  const double y = traversability.origin_y +
                                   (static_cast<double>(row) + 0.5) * traversability.resolution;
                  const double dx = x - sensor_origin.x;
                  const double dy = y - sensor_origin.y;
                  return dx * dx + dy * dy;
                })
          : std::vector<std::size_t>{};
  const std::size_t point_count = sampled_obstacles.size();
  const std::size_t risk_count = sampled_risk.size();
  const bool obstacle_points_truncated = point_count < total_points;
  const bool traversability_complete = enabled && risk_count == risk_indices.size();
  out << "{";
  out << "\"enabled\": " << (enabled ? "true" : "false") << ", ";
  out << "\"frame_id\": \"map\", ";
  out << "\"source_obstacle_points_total\": " << source_points << ", ";
  out << "\"obstacle_filter\": \""
      << (cmu_height_filter ? "cmu_height_envelope" : "none") << "\", ";
  out << "\"obstacle_height_range_m\": [" << cfg.planner_obstacle_height_min_m << ", "
      << cfg.planner_obstacle_height_max_m << "], ";
  out << "\"obstacle_points_fresh\": " << (obstacle_points_fresh ? "true" : "false") << ", ";
  out << "\"obstacle_points_total\": " << total_points << ", ";
  out << "\"obstacle_points_returned\": " << point_count << ", ";
  out << "\"obstacle_points_truncated\": " << (obstacle_points_truncated ? "true" : "false")
      << ", ";
  out << "\"obstacle_sampling\": \""
      << (obstacle_points_truncated && has_debug_origin ? "nearest_sensor_origin"
          : obstacle_points_truncated                   ? "uniform_source_order"
                                                        : "complete")
      << "\", ";
  out << "\"sampling_origin_xy\": [" << sensor_origin.x << ", " << sensor_origin.y << "], ";
  out << "\"obstacle_points\": [";
  for (std::size_t index = 0; index < point_count; ++index) {
    if (index > 0) {
      out << ", ";
    }
    const std::size_t source = sampled_obstacles[index];
    const std::size_t offset = source * 4;
    out << "[" << obstacle_xyzh[offset] << ", " << obstacle_xyzh[offset + 1] << ", "
        << obstacle_xyzh[offset + 2] << ", " << obstacle_xyzh[offset + 3] << "]";
  }
  out << "], ";
  out << "\"traversability\": {";
  out << "\"rows\": " << traversability.rows << ", ";
  out << "\"cols\": " << traversability.cols << ", ";
  out << "\"resolution_m\": " << traversability.resolution << ", ";
  out << "\"origin_xy\": [" << traversability.origin_x << ", " << traversability.origin_y << "], ";
  out << "\"fresh\": " << (traversability_fresh ? "true" : "false") << ", ";
  out << "\"cells_total\": " << traversability.values.size() << ", ";
  out << "\"risk_cells_total\": " << risk_indices.size() << ", ";
  out << "\"risk_cells_returned\": " << risk_count << ", ";
  out << "\"complete\": " << (traversability_complete ? "true" : "false") << ", ";
  out << "\"truncated\": " << (risk_count < risk_indices.size() ? "true" : "false") << ", ";
  out << "\"sampling\": \""
      << (risk_count < risk_indices.size() && has_debug_origin ? "nearest_sensor_origin"
          : risk_count < risk_indices.size()                   ? "uniform_source_order"
                                                               : "complete")
      << "\", ";
  out << "\"default_cost\": " << (traversability_complete ? "0" : "null") << ", ";
  out << "\"unreported_cells\": \"" << (traversability_complete ? "zero_cost" : "not_serialized")
      << "\", ";
  out << "\"risk_cells\": [";
  for (std::size_t index = 0; index < risk_count; ++index) {
    if (index > 0) {
      out << ", ";
    }
    const std::size_t flat = sampled_risk[index];
    const std::size_t cols =
        traversability.cols > 0 ? static_cast<std::size_t>(traversability.cols) : 1;
    out << "[" << flat / cols << ", " << flat % cols << ", " << traversability.values[flat] << "]";
  }
  out << "]}, \"collision\": ";
  writeCollisionMapJson(out, point_limit, collision, collision_total_points,
                        collision_occupied_xyz, sensor_origin);
  out << "}";
}

void writeMetricDistributionJson(std::ostream &out, const MetricDistribution &distribution) {
  out << "{\"mean\": " << distribution.mean << ", "
      << "\"p50\": " << distribution.p50 << ", "
      << "\"p95\": " << distribution.p95 << ", "
      << "\"p99\": " << distribution.p99 << ", "
      << "\"max\": " << distribution.max << "}";
}

void writeMotionOutputJson(std::ostream &out, const nav_kernel::Twist &final_cmd_vel,
                           const FinalOutputDiagnostics &final_output,
                           const DriverControlDiagnostics &driver_control,
                           const InputGateState &input_gate, const StatusWriterConfig &cfg) {
  const bool published =
      final_output.output_sequence != 0U && !final_output.producer_boot_id.empty();
  const bool driver_control_fresh =
      driver_control.received && driver_control.ready && input_gate.driver_control_ready &&
      std::isfinite(input_gate.driver_control_age_s) && input_gate.driver_control_age_s >= 0.0 &&
      cfg.driver_control_max_age_s > 0.0 &&
      input_gate.driver_control_age_s <= cfg.driver_control_max_age_s &&
      input_gate.reason != "driver_control_stale";
  const bool driver_delivery_accepted =
      published && driver_control_fresh && driver_control.last_command_accepted &&
      driver_control.accepted_producer_boot_id == final_output.producer_boot_id &&
      driver_control.accepted_output_sequence == final_output.output_sequence;

  out << "  \"final_cmd_vel\": ";
  writeTwistJson(out, final_cmd_vel);
  out << ",\n"
      << "  \"final_output\": {\"published\": " << (published ? "true" : "false")
      << ", \"producer_boot_id\": \"" << jsonEscape(final_output.producer_boot_id)
      << "\", \"output_sequence\": " << final_output.output_sequence
      << ", \"driver_delivery_accepted\": " << (driver_delivery_accepted ? "true" : "false")
      << "},\n"
      << "  \"driver_control\": {\"received\": " << (driver_control.received ? "true" : "false")
      << ", \"ready\": " << (driver_control.ready ? "true" : "false")
      << ", \"fresh\": " << (driver_control_fresh ? "true" : "false") << ", \"age_s\": "
      << (std::isfinite(input_gate.driver_control_age_s) ? input_gate.driver_control_age_s : -1.0)
      << ", \"reason\": \"" << jsonEscape(driver_control.reason)
      << "\", \"last_command_accepted\": "
      << (driver_control.last_command_accepted ? "true" : "false")
      << ", \"accepted_producer_boot_id\": \""
      << jsonEscape(driver_control.accepted_producer_boot_id)
      << "\", \"accepted_output_sequence\": " << driver_control.accepted_output_sequence << "},\n";
}

}  // namespace

void writeStatusSnapshot(
    StatusSnapshotFileWriter &snapshot_writer, const StatusWriterConfig &cfg, double stamp_s,
    bool has_odom, bool has_map_odom_tf, bool has_path, bool estop_latched,
    const std::string &estop_reason, const nav_kernel::Twist &final_cmd_vel,
    const FinalOutputDiagnostics &final_output, const DriverControlDiagnostics &driver_control,
    const FarInputStatus &far_input, bool has_traversability, bool has_terrain_map,
    bool has_terrain_map_ext, const InputGateState &input_gate,
    const CloudSyncDiagnostics &cloud_sync, const FrameDiagnostics &frames,
    const CommandDiagnostics &commands,
    const OperatorMotionTransportDiagnostics &operator_motion_transport, std::uint64_t odom_count,
    std::uint64_t tf_count, std::uint64_t goal_count, std::uint64_t cancel_count,
    std::uint64_t map_clearing_count, std::uint64_t cloud_clearing_count, std::uint64_t cloud_count,
    std::uint64_t terrain_map_count, std::uint64_t terrain_map_ext_count,
    std::uint64_t traversability_count, std::uint64_t teleop_cmd_count,
    std::uint64_t teleop_output_count, std::uint64_t teleop_stop_count,
    std::uint64_t teleop_slow_count, std::uint64_t teleop_limited_count, std::uint64_t path_count,
    std::uint64_t plan_fail_count, std::uint64_t output_count, std::uint64_t cmd_vel_count,
    std::size_t live_obstacle_cells, const MotionLayerStats &motion_layer,
    const std::vector<DynamicCluster> &dynamic_clusters, const SensorOrigin &last_sensor_origin,
    std::size_t obstacle_points, const PlanDiagnostics &plan, const LocalDiagnostics &local,
    const TeleopDiagnostics &teleop, const TimingDiagnostics &timing,
    const ControlLoopHealthSnapshot &control_loop_health,
    const std::vector<nav_kernel::Vec3> &global_path,
    const std::vector<nav_kernel::Vec3> &local_path,
    const nav_kernel::LocalPlannerDebugSnapshot &local_planner_debug,
    const std::vector<float> &local_map_obstacle_xyzh,
    const TraversabilityGrid &local_map_traversability,
    LocalCollisionStatusView local_collision_map) {
  if (cfg.status_file.empty()) {
    return;
  }
  const std::size_t local_collision_total_points = local_collision_map.occupied_count;
  auto local_collision_occupied_xyz = copyCollisionDebugPoints(
      cfg.local_map_debug_point_limit, local_collision_map, last_sensor_origin);
  snapshot_writer.submitFactory(
      [=, local_collision_occupied_xyz = std::move(local_collision_occupied_xyz),
       snapshot_diagnostics = snapshot_writer.diagnostics()]() {
    const bool operator_motion_enabled =
        cfg.control_mode == "teleop" || cfg.control_mode == "teleop_avoid" ||
        (cfg.control_mode == "autonomy" && cfg.allow_teleop_takeover);
    const bool navigation_ready =
        !cfg.product_session_id.empty() && has_odom && has_map_odom_tf && input_gate.ready &&
        control_loop_health.ready && control_loop_health.healthy && !estop_latched &&
        !cfg.operator_takeover_latched && !cfg.resume_required && cfg.publish_cmd_vel &&
        (!far_input.required || far_input.ready);
    std::ostringstream out;
    out << std::fixed << std::setprecision(6);
    out << "{\n"
        << "  \"schema_version\": \"lingtu.nav.endpoint.status.v1\",\n"
        << "  \"endpoint\": \"navd\",\n"
        << "  \"control_mode\": \"" << jsonEscape(cfg.control_mode) << "\",\n"
        << "  \"native_product\": {\"product\": \"" << jsonEscape(cfg.product)
        << "\", \"product_session_id\": \"" << jsonEscape(cfg.product_session_id)
        << "\"},\n"
        << "  \"operator_motion\": ";
    writeOperatorMotionDiagnosticsJson(out, cfg, operator_motion_enabled,
                                       operator_motion_transport);
    out << ",\n"
        << "  \"stamp_s\": " << stamp_s << ",\n"
        << "  \"domain_id\": " << cfg.domain_id << ",\n"
        << "  \"tick_hz\": " << cfg.tick_hz << ",\n"
        << "  \"path_follower\": {\"max_speed_mps\": " << cfg.max_speed_mps
        << ", \"min_speed_mps\": " << cfg.min_speed_mps
        << ", \"max_accel_mps2\": " << cfg.max_accel_mps2
        << ", \"lookahead_m\": " << cfg.follower_lookahead_m
        << ", \"goal_tolerance_m\": " << cfg.follower_goal_tolerance_m
        << ", \"nominal_dt_s\": " << cfg.nominal_dt_s << "},\n"
        << "  \"nav_loop\": {\"waypoint_reached_m\": " << cfg.waypoint_reached_m
        << ", \"goal_reached_m\": " << cfg.goal_reached_m
        << ", \"corridor_lookahead_m\": " << cfg.corridor_lookahead_m << "},\n"
        << "  \"publish_cmd_vel\": " << (cfg.publish_cmd_vel ? "true" : "false") << ",\n"
        << "  \"check_obstacle\": " << (cfg.check_obstacle ? "true" : "false") << ",\n"
        << "  \"use_traversability_cost\": " << (cfg.use_traversability_cost ? "true" : "false")
        << ",\n"
        << "  \"allow_teleop_takeover\": " << (cfg.allow_teleop_takeover ? "true" : "false")
        << ",\n"
        << "  \"teleop_local_planner\": " << (cfg.teleop_local_planner ? "true" : "false") << ",\n"
        << "  \"teleop_planner_horizon_m\": " << cfg.teleop_planner_horizon_m << ",\n"
        << "  \"teleop_planner_max_deviation_deg\": " << cfg.teleop_planner_max_deviation_deg
        << ",\n"
        << "  \"local_planner_threads\": " << cfg.local_planner_threads << ",\n"
        << "  \"stop_confirmation_timeout_s\": " << cfg.stop_confirmation_timeout_s << ",\n"
        << "  \"stop_confirmation_evidence\": \""
        << jsonEscape(cfg.stop_confirmation_evidence) << "\",\n"
        << "  \"active_cmd_source\": \"" << jsonEscape(cfg.active_cmd_source) << "\",\n"
        << "  \"navigation_compute_owner\": \"navd\",\n"
        << "  \"local_path_role\": \"dds_telemetry_and_preview\",\n"
        << "  \"path_follower_role\": \"embedded_before_cmd_vel_gate\",\n"
        << "  \"cmd_vel_role\": \"final_navigation_command_output_when_enabled\",\n"
        << "  \"control_authority\": {\"owner\": \"native_endpoint\", "
        << "\"estop_latched\": " << (estop_latched ? "true" : "false") << ", "
        << "\"estop_reason\": \"" << jsonEscape(estop_reason) << "\", "
        << "\"operator_takeover_latched\": " << (cfg.operator_takeover_latched ? "true" : "false")
        << ", "
        << "\"resume_required\": " << (cfg.resume_required ? "true" : "false") << "},\n";
    writeMotionOutputJson(out, final_cmd_vel, final_output, driver_control, input_gate, cfg);
    out << "  \"global_planner\": \"" << jsonEscape(cfg.global_planner) << "\",\n"
        << "  \"local_planner\": \"" << jsonEscape(cfg.local_planner) << "\",\n"
        << "  \"planner_map\": \"" << jsonEscape(cfg.map_path) << "\",\n"
        << "  \"active_octomap\": \""
        << (cfg.global_planner == "octoplanner3d" ? jsonEscape(cfg.map_path) : "") << "\",\n"
        << "  \"active_occupancy\": \""
        << (cfg.global_planner == "far" ? jsonEscape(cfg.map_path) : "") << "\",\n"
        << "  \"far_input\": {\"required\": " << (far_input.required ? "true" : "false")
        << ", \"ready\": " << (far_input.ready ? "true" : "false")
        << ", \"reason\": \"" << jsonEscape(far_input.reason)
        << "\", \"map_id\": \"" << jsonEscape(far_input.map_id)
        << "\", \"content_epoch\": " << far_input.content_epoch << "},\n"
        << "  \"navigation_ready\": " << (navigation_ready ? "true" : "false") << ",\n"
        << "  \"octoplanner3d\": {"
        << "\"robot_radius_m\": " << cfg.octoplanner_options.robot_radius << ", "
        << "\"body_clearance_below_m\": " << cfg.octoplanner_options.body_clearance_below_m << ", "
        << "\"body_clearance_above_m\": " << cfg.octoplanner_options.body_clearance_above_m << ", "
        << "\"max_iterations\": " << cfg.octoplanner_options.max_iterations << ", "
        << "\"snap_radius_cells\": " << cfg.octoplanner_options.snap_search_radius_cells << ", "
        << "\"require_ground_support\": "
        << (cfg.octoplanner_options.require_ground_support ? "true" : "false") << ", "
        << "\"strict_ground_support\": "
        << (cfg.octoplanner_options.strict_direct_ground_support ? "true" : "false") << ", "
        << "\"ground_support_xy_radius_cells\": "
        << cfg.octoplanner_options.ground_support_xy_radius_cells << ", "
        << "\"ground_support_depth_cells\": " << cfg.octoplanner_options.ground_support_depth_cells
        << ", "
        << "\"support_height_m\": " << cfg.octoplanner_options.support_height_m << ", "
        << "\"support_height_tolerance_m\": " << cfg.octoplanner_options.support_height_tolerance_m
        << ", "
        << "\"support_patch_radius_cells\": " << cfg.octoplanner_options.support_patch_radius_cells
        << ", "
        << "\"support_patch_min_samples\": " << cfg.octoplanner_options.support_patch_min_samples
        << ", "
        << "\"preblocked_costmap\": "
        << (cfg.octoplanner_options.enable_preblocked_costmap ? "true" : "false") << ", "
        << "\"lowest_traversable_only\": "
        << (cfg.octoplanner_options.lowest_traversable_only ? "true" : "false") << ", "
        << "\"floor_change_penalty\": " << cfg.octoplanner_options.floor_change_penalty << ", "
        << "\"max_step_height_m\": " << cfg.octoplanner_options.max_step_height << ", "
        << "\"max_slope\": " << cfg.octoplanner_options.max_slope << ", "
        << "\"obstacle_clearance_radius_cells\": "
        << cfg.octoplanner_options.obstacle_clearance_radius_cells << ", "
        << "\"obstacle_clearance_weight\": " << cfg.octoplanner_options.obstacle_clearance_weight
        << ", "
        << "\"terminal_goal_tolerance_m\": " << cfg.octoplanner_options.terminal_goal_tolerance_m
        << ", "
        << "\"terminal_goal_xy_tolerance_m\": "
        << cfg.octoplanner_options.terminal_goal_xy_tolerance_m << ", "
        << "\"terminal_goal_z_tolerance_m\": "
        << cfg.octoplanner_options.terminal_goal_z_tolerance_m << "},\n"
        << "  \"far\": {"
        << "\"robot_radius_m\": " << cfg.far_options.robot_radius_m << ", "
        << "\"obstacle_clearance_m\": " << cfg.far_options.obstacle_clearance_m << ", "
        << "\"max_visibility_distance_m\": " << cfg.far_options.max_visibility_distance_m << ", "
        << "\"unknown_cost_multiplier\": " << cfg.far_options.unknown_cost_multiplier << ", "
        << "\"corner_separation_cells\": " << cfg.far_options.corner_separation_cells << ", "
        << "\"snap_radius_cells\": " << cfg.far_options.snap_search_radius_cells << ", "
        << "\"max_graph_nodes\": " << cfg.far_options.max_graph_nodes << ", "
        << "\"max_visibility_pairs\": " << cfg.far_options.max_visibility_pairs << ", "
        << "\"max_search_expansions\": " << cfg.far_options.max_search_expansions << ", "
        << "\"allow_unknown_fallback\": "
        << (cfg.far_options.allow_unknown_fallback ? "true" : "false") << ", "
        << "\"simplify_path\": " << (cfg.far_options.simplify_path ? "true" : "false") << "},\n"
        << "  \"path_library\": \"" << jsonEscape(cfg.path_library_dir) << "\",\n"
        << "  \"max_obstacle_points\": " << cfg.max_obstacle_points << ",\n"
        << "  \"obstacle_voxel_size_m\": " << cfg.obstacle_voxel_size_m << ",\n"
        << "  \"live_obstacle_decay_s\": " << cfg.live_obstacle_decay_s << ",\n"
        << "  \"live_obstacle_inflation_radius_m\": " << cfg.live_obstacle_inflation_radius_m
        << ",\n"
        << "  \"obstacle_inflation\": {\"owner\": \"active_path_blockage_overlay\", "
        << "\"overlay_extra_radius_m\": " << cfg.live_obstacle_inflation_radius_m << ", "
        << "\"layer_radius_m\": " << cfg.live_obstacle_layer_inflation_radius_m << "},\n"
        << "  \"live_obstacle_ray_clearing\": "
        << (cfg.live_obstacle_ray_clearing ? "true" : "false") << ",\n"
        << "  \"live_obstacle_ray_clear_max_range_m\": " << cfg.live_obstacle_ray_clear_max_range_m
        << ",\n"
        << "  \"live_obstacle_ray_clearing_interval_s\": "
        << cfg.live_obstacle_ray_clearing_interval_s << ",\n"
        << "  \"live_obstacle_max_clearing_rays\": " << cfg.live_obstacle_max_clearing_rays << ",\n"
        << "  \"live_obstacle_min_hits\": " << cfg.live_obstacle_min_hits << ",\n"
        << "  \"cloud_pose_max_gap_s\": " << cfg.cloud_pose_max_gap_s << ",\n"
        << "  \"planner_obstacle_source\": \""
        << (cfg.local_planner == "cmu"
                ? "fresh_terrain_map_else_registered_scan"
                : "registered_scan")
        << "\",\n"
        << "  \"terrain_map_ext_role\": \"diagnostics\",\n"
        << "  \"sensor_origin_model\": \"body_pose_plus_configured_lidar_extrinsics\",\n"
        << "  \"sensor_offset_m\": [" << cfg.sensor_offset_x_m << ", " << cfg.sensor_offset_y_m
        << ", " << cfg.sensor_offset_z_m << "],\n"
        << "  \"last_sensor_origin\": {"
        << "\"valid\": " << (last_sensor_origin.valid ? "true" : "false") << ", "
        << "\"xyz\": [" << last_sensor_origin.x << ", " << last_sensor_origin.y << ", "
        << last_sensor_origin.z << "]},\n"
        << "  \"has_odom\": " << (has_odom ? "true" : "false") << ",\n"
        << "  \"has_map_odom_tf\": " << (has_map_odom_tf ? "true" : "false") << ",\n"
        << "  \"planning_frame_id\": \"map\",\n"
        << "  \"odom_frame_id\": \"odom\",\n"
        << "  \"active_path\": " << (has_path ? "true" : "false") << ",\n"
        << "  \"has_traversability\": " << (has_traversability ? "true" : "false") << ",\n"
        << "  \"has_terrain_map\": " << (has_terrain_map ? "true" : "false") << ",\n"
        << "  \"has_terrain_map_ext\": " << (has_terrain_map_ext ? "true" : "false") << ",\n"
        << "  \"input_gate\": {"
        << "\"ready\": " << (input_gate.ready ? "true" : "false") << ", "
        << "\"recovering\": " << (input_gate.recovering ? "true" : "false") << ", "
        << "\"reason\": \"" << jsonEscape(input_gate.reason) << "\", "
        << "\"fresh_frames\": " << input_gate.fresh_frames << ", "
        << "\"required_frames\": " << cfg.input_recovery_frames << ", "
        << "\"require_odom\": " << (cfg.input_require_odom ? "true" : "false") << ", "
        << "\"require_cloud\": " << (cfg.input_require_cloud ? "true" : "false") << ", "
        << "\"require_traversability\": " << (cfg.input_require_traversability ? "true" : "false")
        << ", "
        << "\"require_localization_health\": "
        << (cfg.input_require_localization_health ? "true" : "false") << ", "
        << "\"require_driver_control\": " << (cfg.input_require_driver_control ? "true" : "false")
        << ", "
        << "\"odom_age_s\": "
        << (std::isfinite(input_gate.odom_age_s) ? input_gate.odom_age_s : -1.0) << ", "
        << "\"tf_age_s\": " << (std::isfinite(input_gate.tf_age_s) ? input_gate.tf_age_s : -1.0)
        << ", "
        << "\"cloud_age_s\": "
        << (std::isfinite(input_gate.cloud_age_s) ? input_gate.cloud_age_s : -1.0) << ", "
        << "\"traversability_age_s\": "
        << (std::isfinite(input_gate.traversability_age_s) ? input_gate.traversability_age_s : -1.0)
        << ", "
        << "\"localization_health_age_s\": "
        << (std::isfinite(input_gate.localization_health_age_s)
                ? input_gate.localization_health_age_s
                : -1.0)
        << ", "
        << "\"driver_control_age_s\": "
        << (std::isfinite(input_gate.driver_control_age_s) ? input_gate.driver_control_age_s : -1.0)
        << ", "
        << "\"driver_control_ready\": " << (input_gate.driver_control_ready ? "true" : "false")
        << ", "
        << "\"driver_control_reason\": \"" << jsonEscape(input_gate.driver_control_reason) << "\", "
        << "\"localization_healthy\": " << (input_gate.localization_healthy ? "true" : "false")
        << ", "
        << "\"localization_state\": \"" << jsonEscape(input_gate.localization_state) << "\", "
        << "\"odom_max_age_s\": " << cfg.odom_max_age_s << ", "
        << "\"tf_max_age_s\": " << cfg.tf_max_age_s << ", "
        << "\"cloud_max_age_s\": " << cfg.cloud_max_age_s << ", "
        << "\"traversability_max_age_s\": " << cfg.traversability_max_age_s << ", "
        << "\"localization_health_max_age_s\": " << cfg.localization_health_max_age_s << ", "
        << "\"driver_control_max_age_s\": " << cfg.driver_control_max_age_s << ", "
        << "\"future_tolerance_s\": " << cfg.input_future_tolerance_s << "},\n"
        << "  \"cloud_sync\": {\"last_stamp_age_s\": " << cloud_sync.last_stamp_age_s
        << ", \"last_pose_gap_s\": " << cloud_sync.last_pose_gap_s
        << ", \"stamp_rejected\": " << cloud_sync.stamp_rejected
        << ", \"pose_rejected\": " << cloud_sync.pose_rejected << "},\n"
        << "  \"frame_gate\": {"
        << "\"odom_rejected\": " << frames.odom_rejected << ", "
        << "\"clock_rebases\": " << frames.clock_rebases << ", "
        << "\"goal_rejected\": " << frames.goal_rejected << ", "
        << "\"path_rejected\": " << frames.path_rejected << ", "
        << "\"grid_rejected\": " << frames.grid_rejected << ", "
        << "\"teleop_rejected\": " << frames.teleop_rejected << ", "
        << "\"last_error\": \"" << jsonEscape(frames.last_error) << "\"},\n"
        << "  \"command_boundary\": {"
        << "\"transport\": \"typed_dds_request_ack\", "
        << "\"received\": " << commands.received << ", "
        << "\"ack_sent\": " << commands.ack_sent << ", "
        << "\"rejected\": " << commands.rejected << ", "
        << "\"ack_publish_failed\": " << commands.ack_publish_failed << ", "
        << "\"replayed\": " << commands.replayed << ", "
        << "\"last_request_id\": \"" << jsonEscape(commands.last_request_id) << "\", "
        << "\"last_kind\": \"" << jsonEscape(commands.last_kind) << "\", "
        << "\"last_accepted\": " << (commands.last_accepted ? "true" : "false") << ", "
        << "\"last_reason\": \"" << jsonEscape(commands.last_reason) << "\"},\n"
        << "  \"live_obstacle_cells\": " << live_obstacle_cells << ",\n"
        << "  \"motion_layer\": {"
        << "\"unknown_model\": \"sparse_absent_cell\", "
        << "\"unknown_query_state\": \"Unknown\", "
        << "\"unknown_count_available\": false, "
        << "\"cells\": " << motion_layer.cells << ", "
        << "\"free\": " << motion_layer.free_cells << ", "
        << "\"occupied\": " << motion_layer.occupied_cells << ", "
        << "\"static\": " << motion_layer.static_cells << ", "
        << "\"dynamic\": " << motion_layer.dynamic_cells << ", "
        << "\"cleared\": " << motion_layer.cleared_cells << ", "
        << "\"obstacle\": " << motion_layer.obstacle_cells << ", "
        << "\"raycast_rays\": " << motion_layer.raycast_rays << ", "
        << "\"raycast_voxels\": " << motion_layer.raycast_voxels << ", "
        << "\"ray_cleared\": " << motion_layer.ray_cleared_cells << ", "
        << "\"prune_passes\": " << motion_layer.prune_passes << ", "
        << "\"predicted_points\": " << motion_layer.predicted_points << ", "
        << "\"prediction_clusters\": " << motion_layer.prediction_clusters << ", "
        << "\"prediction_horizon_s\": " << motion_layer.prediction_horizon_s << "},\n"
        << "  \"dynamic_objects\": ";
    writeDynamicClustersJson(out, dynamic_clusters);
    out << ",\n"
        << "  \"obstacle_points\": " << obstacle_points << ",\n"
        << "  \"global_path_points\": " << global_path.size() << ",\n"
        << "  \"local_path_points\": " << local_path.size() << ",\n"
        << "  \"global_path\": ";
    writePathJson(out, global_path);
    out << ",\n"
        << "  \"local_path\": ";
    writePathJson(out, local_path);
    out << ",\n"
        << "  \"local_candidates\": ";
    writeLocalCandidatesJson(out, local_planner_debug);
    out << ",\n"
        << "  \"local_planner_debug\": {\"backend\": \""
        << nav_kernel::localPlannerBackendName(local_planner_debug.backend)
        << "\", \"planning_ms\": " << local_planner_debug.planningTimeMs
        << ", \"reuse_ms\": " << local_planner_debug.reuseTimeMs
        << ", \"grid_ms\": " << local_planner_debug.gridTimeMs
        << ", \"search_ms\": " << local_planner_debug.searchTimeMs
        << ", \"spline_ms\": " << local_planner_debug.splineTimeMs
        << ", \"search_reason\": \"" << jsonEscape(local_planner_debug.searchReason) << "\""
        << ", \"expanded_nodes\": " << local_planner_debug.expandedNodes
        << ", \"occupied_cells\": " << local_planner_debug.occupiedCellCount
        << ", \"collision_points_used\": " << local_planner_debug.collisionPointCount
        << ", \"trajectory_points\": " << local_planner_debug.trajectoryPointCount
        << ", \"rebound_restarts\": " << local_planner_debug.reboundRestarts
        << ", \"optimizer_evaluations\": " << local_planner_debug.optimizerEvaluations
        << ", \"collision_segments\": " << local_planner_debug.collisionSegments
        << ", \"anchor_searches\": " << local_planner_debug.anchorSearches
        << ", \"continuity_reused\": "
        << (local_planner_debug.continuityReused ? "true" : "false")
        << ", \"spline_fallback\": "
        << (local_planner_debug.splineFallback ? "true" : "false") << "},\n"
        << "  \"local_map\": ";
    writeLocalMapJson(out, cfg, local_map_obstacle_xyzh,
                      local_map_traversability, local_collision_map, local_collision_total_points,
                      local_collision_occupied_xyz, last_sensor_origin, input_gate.ready,
                      has_traversability);
    out << ",\n"
        << "  \"last_plan\": {\n"
        << "    \"seen\": " << (plan.seen ? "true" : "false") << ",\n"
        << "    \"accepted\": " << (plan.accepted ? "true" : "false") << ",\n"
        << "    \"reason\": \"" << jsonEscape(plan.reason) << "\",\n"
        << "    \"reached_goal\": " << (plan.reached_goal ? "true" : "false") << ",\n"
        << "    \"waypoints\": " << plan.waypoints << ",\n"
        << "    \"goal_error_m\": " << plan.goal_error_m << ",\n"
        << "    \"elapsed_ms\": " << plan.elapsed_ms << ",\n"
        << "    \"start\": ";
    writeVec3Json(out, plan.start);
    out << ",\n"
        << "    \"goal\": ";
    writeVec3Json(out, plan.goal);
    out << "\n"
        << "  },\n"
        << "  \"last_local\": {\n"
        << "    \"seen\": " << (local.seen ? "true" : "false") << ",\n"
        << "    \"active\": " << (local.active ? "true" : "false") << ",\n"
        << "    \"goal_reached\": " << (local.goal_reached ? "true" : "false") << ",\n"
        << "    \"path_found\": " << (local.path_found ? "true" : "false") << ",\n"
        << "    \"near_field_stop\": " << (local.near_field_stop ? "true" : "false") << ",\n"
        << "    \"reason\": \"" << jsonEscape(local.reason) << "\",\n"
        << "    \"slow_down\": " << local.slow_down << ",\n"
        << "    \"recovery_state\": " << local.recovery_state << ",\n"
        << "    \"recovery_action\": " << local.recovery_action << ",\n"
        << "    \"recovery_attempt\": " << local.recovery_attempt << ",\n"
        << "    \"recovery_candidate_count\": " << local.recovery_candidate_count << ",\n"
        << "    \"recovery_rotation_target_rad\": "
        << local.recovery_rotation_target_rad << ",\n"
        << "    \"recovery_verified\": " << (local.recovery_verified ? "true" : "false") << ",\n"
        << "    \"recovery_progress\": " << local.recovery_progress << ",\n"
        << "    \"recovery_trigger\": \"" << jsonEscape(local.recovery_trigger) << "\",\n"
        << "    \"recovery_reason\": \"" << jsonEscape(local.recovery_reason) << "\",\n"
        << "    \"recovery_exhausted\": " << (local.recovery_exhausted ? "true" : "false") << ",\n"
        << "    \"target_index\": " << local.target_index << ",\n"
        << "    \"target_distance_m\": " << local.target_distance_m << ",\n"
        << "    \"local_path_points\": " << local.local_path_points << ",\n"
        << "    \"final_safety\": {"
        << "\"applied\": " << (local.final_safety_applied ? "true" : "false") << ", "
        << "\"stopped\": " << (local.final_safety_stopped ? "true" : "false") << ", "
        << "\"slowed\": " << (local.final_safety_slowed ? "true" : "false") << ", "
        << "\"limited\": " << (local.final_safety_limited ? "true" : "false") << ", "
        << "\"reason\": \"" << jsonEscape(local.final_safety_reason) << "\", "
        << "\"obstacle_distance_m\": " << local.final_safety_obstacle_distance_m << ", "
        << "\"traversability_cost\": " << local.final_safety_traversability_cost << "},\n"
        << "    \"target\": ";
    writeVec3Json(out, local.target);
    out << ",\n"
        << "    \"path_follower_cmd_vel\": ";
    writeTwistJson(out, local.path_follower_cmd_vel);
    out << ",\n"
        << "    \"cmd_vel\": {"
        << "\"vx\": " << local.cmd_vel.vx << ", "
        << "\"vy\": " << local.cmd_vel.vy << ", "
        << "\"wz\": " << local.cmd_vel.wz << "}\n"
        << "  },\n"
        << "  \"teleop\": {\n"
        << "    \"seen\": " << (teleop.seen ? "true" : "false") << ",\n"
        << "    \"fresh\": " << (teleop.fresh ? "true" : "false") << ",\n"
        << "    \"manual_mode\": " << (teleop.manual_mode ? "true" : "false") << ",\n"
        << "    \"published\": " << (teleop.published ? "true" : "false") << ",\n"
        << "    \"stopped\": " << (teleop.stopped ? "true" : "false") << ",\n"
        << "    \"slowed\": " << (teleop.slowed ? "true" : "false") << ",\n"
        << "    \"limited\": " << (teleop.limited ? "true" : "false") << ",\n"
        << "    \"reason\": \"" << jsonEscape(teleop.reason) << "\",\n"
        << "    \"age_s\": " << teleop.age_s << ",\n"
        << "    \"obstacle_distance_m\": " << teleop.obstacle_distance_m << ",\n"
        << "    \"traversability_cost\": " << teleop.traversability_cost << ",\n"
        << "    \"request\": ";
    writeTwistJson(out, teleop.request);
    out << ",\n"
        << "    \"output\": ";
    writeTwistJson(out, teleop.output);
    out << "\n"
        << "  },\n"
        << "  \"status_snapshot_writer\": {"
        << "\"submitted\": " << snapshot_diagnostics.submitted << ", "
        << "\"written\": " << snapshot_diagnostics.written << ", "
        << "\"dropped\": " << snapshot_diagnostics.dropped << ", "
        << "\"failures\": " << snapshot_diagnostics.failures << ", "
        << "\"writing\": " << (snapshot_diagnostics.writing ? "true" : "false") << ", "
        << "\"pending\": " << (snapshot_diagnostics.pending ? "true" : "false") << "},\n"
        << "  \"timing_ms\": {\n"
        << "    \"loop\": " << timing.loop_ms << ",\n"
        << "    \"sensor_drain\": " << timing.sensor_drain_ms << ",\n"
        << "    \"operator_drain\": " << timing.operator_drain_ms << ",\n"
        << "    \"command_drain\": " << timing.command_drain_ms << ",\n"
        << "    \"runtime\": " << timing.runtime_ms << ",\n"
        << "    \"publish\": " << timing.publish_ms << ",\n"
        << "    \"input_callbacks\": " << timing.input_callbacks_ms << ",\n"
        << "    \"global_plan\": " << timing.global_plan_ms << ",\n"
        << "    \"cloud_convert\": " << timing.cloud_convert_ms << ",\n"
        << "    \"motion_update_last\": " << timing.motion_update_last_ms << ",\n"
        << "    \"obstacle_snapshot_last\": " << timing.obstacle_snapshot_last_ms << ",\n"
        << "    \"teleop_gate\": " << timing.teleop_gate_ms << ",\n"
        << "    \"obstacle_merge\": " << timing.obstacle_merge_ms << ",\n"
        << "    \"nav_tick\": " << timing.nav_tick_ms << ",\n"
        << "    \"dds_write\": " << timing.dds_write_ms << ",\n"
        << "    \"status_log\": " << timing.status_log_ms << ",\n"
        << "    \"status_snapshot\": " << timing.status_snapshot_ms << ",\n"
        << "    \"sleep\": " << timing.sleep_ms << ",\n"
        << "    \"overrun\": " << timing.overrun_ms << "\n"
        << "  },\n";
    out << "  \"control_loop_health\": {\n"
        << "    \"ready\": " << (control_loop_health.ready ? "true" : "false") << ",\n"
        << "    \"healthy\": " << (control_loop_health.healthy ? "true" : "false") << ",\n"
        << "    \"reason\": \"" << jsonEscape(control_loop_health.reason) << "\",\n"
        << "    \"period_ms\": " << control_loop_health.period_ms << ",\n"
        << "    \"window_samples\": " << control_loop_health.window_samples << ",\n"
        << "    \"total_samples\": " << control_loop_health.total_samples << ",\n"
        << "    \"loop_ms\": ";
    writeMetricDistributionJson(out, control_loop_health.loop_ms);
    out << ",\n"
        << "    \"work_ms\": ";
    writeMetricDistributionJson(out, control_loop_health.work_ms);
    out << ",\n"
        << "    \"overrun_ms\": ";
    writeMetricDistributionJson(out, control_loop_health.overrun_ms);
    out << ",\n"
        << "    \"deadline_misses\": " << control_loop_health.deadline_misses << ",\n"
        << "    \"deadline_miss_ratio\": " << control_loop_health.deadline_miss_ratio << ",\n"
        << "    \"current_miss_streak\": " << control_loop_health.current_miss_streak << ",\n"
        << "    \"max_miss_streak\": " << control_loop_health.max_miss_streak << ",\n"
        << "    \"p95_utilization\": " << control_loop_health.p95_utilization << ",\n"
        << "    \"max_utilization\": " << control_loop_health.max_utilization << "\n"
        << "  },\n"
        << "  \"counters\": {\n"
        << "    \"odom\": " << odom_count << ",\n"
        << "    \"tf\": " << tf_count << ",\n"
        << "    \"goals\": " << goal_count << ",\n"
        << "    \"cancels\": " << cancel_count << ",\n"
        << "    \"map_clearing\": " << map_clearing_count << ",\n"
        << "    \"cloud_clearing\": " << cloud_clearing_count << ",\n"
        << "    \"registered_clouds\": " << cloud_count << ",\n"
        << "    \"cloud_stamp_rejected\": " << cloud_sync.stamp_rejected << ",\n"
        << "    \"cloud_pose_rejected\": " << cloud_sync.pose_rejected << ",\n"
        << "    \"terrain_maps\": " << terrain_map_count << ",\n"
        << "    \"terrain_map_exts\": " << terrain_map_ext_count << ",\n"
        << "    \"traversability\": " << traversability_count << ",\n"
        << "    \"teleop_cmd\": " << teleop_cmd_count << ",\n"
        << "    \"teleop_outputs\": " << teleop_output_count << ",\n"
        << "    \"teleop_stops\": " << teleop_stop_count << ",\n"
        << "    \"teleop_slows\": " << teleop_slow_count << ",\n"
        << "    \"teleop_limited\": " << teleop_limited_count << ",\n"
        << "    \"command_requests\": " << commands.received << ",\n"
        << "    \"command_acks\": " << commands.ack_sent << ",\n"
        << "    \"command_rejected\": " << commands.rejected << ",\n"
        << "    \"command_ack_publish_failed\": " << commands.ack_publish_failed << ",\n"
        << "    \"command_replayed\": " << commands.replayed << ",\n"
        << "    \"operator_motion_acks\": " << operator_motion_transport.ack_sent << ",\n"
        << "    \"operator_motion_ack_publish_failed\": "
        << operator_motion_transport.ack_publish_failed << ",\n"
        << "    \"operator_motion_status\": " << operator_motion_transport.status_sent << ",\n"
        << "    \"operator_motion_status_publish_failed\": "
        << operator_motion_transport.status_publish_failed << ",\n"
        << "    \"paths\": " << path_count << ",\n"
        << "    \"plan_fail\": " << plan_fail_count << ",\n"
        << "    \"outputs\": " << output_count << ",\n"
        << "    \"cmd_vel_published\": " << cmd_vel_count << "\n"
        << "  }\n"
        << "}\n";
    return out.str();
  });
}

}  // namespace lingtu::nav::endpoint

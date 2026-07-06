#include "nav_status_writer.hpp"

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <ostream>

namespace lingtu::nav::endpoint {
namespace {

constexpr std::size_t kStatusPathPointLimit = 512;

std::string jsonEscape(const std::string& input) {
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

void writeVec3Json(std::ostream& out, const nav_kernel::Vec3& point) {
  out << "[" << point.x << ", " << point.y << ", " << point.z << "]";
}

void writePathJson(std::ostream& out, const std::vector<nav_kernel::Vec3>& path) {
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

}  // namespace

void writeStatusSnapshot(
    const StatusWriterConfig& cfg,
    double stamp_s,
    bool has_odom,
    bool has_map_odom_tf,
    bool has_path,
    bool has_traversability,
    bool has_terrain_map,
    bool has_terrain_map_ext,
    std::uint64_t odom_count,
    std::uint64_t tf_count,
    std::uint64_t goal_count,
    std::uint64_t cancel_count,
    std::uint64_t map_clearing_count,
    std::uint64_t cloud_clearing_count,
    std::uint64_t instruction_count,
    std::uint64_t cloud_count,
    std::uint64_t terrain_map_count,
    std::uint64_t terrain_map_ext_count,
    std::uint64_t traversability_count,
    std::uint64_t path_count,
    std::uint64_t plan_fail_count,
    std::uint64_t output_count,
    std::uint64_t cmd_vel_count,
    std::size_t obstacle_points,
    const PlanDiagnostics& plan,
    const LocalDiagnostics& local,
    const std::vector<nav_kernel::Vec3>& global_path,
    const std::vector<nav_kernel::Vec3>& local_path,
    const std::string& last_instruction) {
  if (cfg.status_file.empty()) {
    return;
  }
  const std::filesystem::path path(cfg.status_file);
  const auto parent = path.parent_path();
  if (!parent.empty()) {
    std::error_code ec;
    std::filesystem::create_directories(parent, ec);
  }
  const std::filesystem::path tmp = path.string() + ".tmp";
  std::ofstream out(tmp, std::ios::trunc);
  if (!out) {
    std::fprintf(stderr, "nav_native: failed to open status snapshot %s\n", tmp.string().c_str());
    return;
  }
  out << "{\n"
      << "  \"schema_version\": \"lingtu.nav.endpoint.status.v1\",\n"
      << "  \"endpoint\": \"lingtu_nav_native_endpoint\",\n"
      << "  \"stamp_s\": " << stamp_s << ",\n"
      << "  \"domain_id\": " << cfg.domain_id << ",\n"
      << "  \"tick_hz\": " << cfg.tick_hz << ",\n"
      << "  \"publish_cmd_vel\": " << (cfg.publish_cmd_vel ? "true" : "false") << ",\n"
      << "  \"check_obstacle\": " << (cfg.check_obstacle ? "true" : "false") << ",\n"
      << "  \"use_traversability_cost\": "
      << (cfg.use_traversability_cost ? "true" : "false") << ",\n"
      << "  \"semantic_instruction_supported\": false,\n"
      << "  \"last_semantic_instruction\": \"" << jsonEscape(last_instruction) << "\",\n"
      << "  \"active_octomap\": \"" << jsonEscape(cfg.map_path) << "\",\n"
      << "  \"path_library\": \"" << jsonEscape(cfg.path_library_dir) << "\",\n"
      << "  \"has_odom\": " << (has_odom ? "true" : "false") << ",\n"
      << "  \"has_map_odom_tf\": " << (has_map_odom_tf ? "true" : "false") << ",\n"
      << "  \"planning_frame_id\": \"map\",\n"
      << "  \"odom_frame_id\": \"odom\",\n"
      << "  \"active_path\": " << (has_path ? "true" : "false") << ",\n"
      << "  \"has_traversability\": " << (has_traversability ? "true" : "false") << ",\n"
      << "  \"has_terrain_map\": " << (has_terrain_map ? "true" : "false") << ",\n"
      << "  \"has_terrain_map_ext\": " << (has_terrain_map_ext ? "true" : "false") << ",\n"
      << "  \"obstacle_points\": " << obstacle_points << ",\n"
      << "  \"global_path_points\": " << global_path.size() << ",\n"
      << "  \"local_path_points\": " << local_path.size() << ",\n"
      << "  \"global_path\": ";
  writePathJson(out, global_path);
  out << ",\n"
      << "  \"local_path\": ";
  writePathJson(out, local_path);
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
      << "    \"target_index\": " << local.target_index << ",\n"
      << "    \"target_distance_m\": " << local.target_distance_m << ",\n"
      << "    \"local_path_points\": " << local.local_path_points << ",\n"
      << "    \"target\": ";
  writeVec3Json(out, local.target);
  out << ",\n"
      << "    \"cmd_vel\": {"
      << "\"vx\": " << local.cmd_vel.vx << ", "
      << "\"vy\": " << local.cmd_vel.vy << ", "
      << "\"wz\": " << local.cmd_vel.wz << "}\n"
      << "  },\n"
      << "  \"counters\": {\n"
      << "    \"odom\": " << odom_count << ",\n"
      << "    \"tf\": " << tf_count << ",\n"
      << "    \"goals\": " << goal_count << ",\n"
      << "    \"cancels\": " << cancel_count << ",\n"
      << "    \"map_clearing\": " << map_clearing_count << ",\n"
      << "    \"cloud_clearing\": " << cloud_clearing_count << ",\n"
      << "    \"semantic_instructions\": " << instruction_count << ",\n"
      << "    \"registered_clouds\": " << cloud_count << ",\n"
      << "    \"terrain_maps\": " << terrain_map_count << ",\n"
      << "    \"terrain_map_exts\": " << terrain_map_ext_count << ",\n"
      << "    \"traversability\": " << traversability_count << ",\n"
      << "    \"paths\": " << path_count << ",\n"
      << "    \"plan_fail\": " << plan_fail_count << ",\n"
      << "    \"outputs\": " << output_count << ",\n"
      << "    \"cmd_vel_published\": " << cmd_vel_count << "\n"
      << "  }\n"
      << "}\n";
  out.close();
  std::error_code ec;
  std::filesystem::rename(tmp, path, ec);
  if (ec) {
    std::fprintf(stderr, "nav_native: failed to replace status snapshot %s: %s\n",
                 path.string().c_str(), ec.message().c_str());
  }
}

}  // namespace lingtu::nav::endpoint

#include "localization/opt/graph.hpp"
#include "localization/opt/gravity_graph.hpp"
#include "localization/opt/cloud.hpp"
#include "localization/opt/constraints.hpp"
#include "localization/opt/pose_math.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <unordered_set>

namespace lingtu::localization::opt {
namespace {

bool is_finite(double value) {
  return std::isfinite(value);
}

double sqr(double value) {
  return value * value;
}

Pose normalized(Pose pose) {
  const double norm = std::sqrt(
      sqr(pose.qw) + sqr(pose.qx) + sqr(pose.qy) + sqr(pose.qz));
  if (!is_finite(norm) || norm <= std::numeric_limits<double>::epsilon()) {
    pose.qw = 1.0;
    pose.qx = 0.0;
    pose.qy = 0.0;
    pose.qz = 0.0;
    return pose;
  }
  pose.qw /= norm;
  pose.qx /= norm;
  pose.qy /= norm;
  pose.qz /= norm;
  return pose;
}

std::array<double, 3> rotate_point(const Pose& pose, double x, double y, double z) {
  return {
      (1.0 - 2.0 * (pose.qy * pose.qy + pose.qz * pose.qz)) * x +
          2.0 * (pose.qx * pose.qy - pose.qz * pose.qw) * y +
          2.0 * (pose.qx * pose.qz + pose.qy * pose.qw) * z,
      2.0 * (pose.qx * pose.qy + pose.qz * pose.qw) * x +
          (1.0 - 2.0 * (pose.qx * pose.qx + pose.qz * pose.qz)) * y +
          2.0 * (pose.qy * pose.qz - pose.qx * pose.qw) * z,
      2.0 * (pose.qx * pose.qz - pose.qy * pose.qw) * x +
          2.0 * (pose.qy * pose.qz + pose.qx * pose.qw) * y +
          (1.0 - 2.0 * (pose.qx * pose.qx + pose.qy * pose.qy)) * z};
}

lt_pose_graph_opt_pose3 to_kernel_pose(const Pose& pose) {
  lt_pose_graph_opt_pose3 out{};
  out.t_xyz[0] = pose.x;
  out.t_xyz[1] = pose.y;
  out.t_xyz[2] = pose.z;
  out.q_wxyz[0] = pose.qw;
  out.q_wxyz[1] = pose.qx;
  out.q_wxyz[2] = pose.qy;
  out.q_wxyz[3] = pose.qz;
  return out;
}

Pose from_kernel_pose(const lt_pose_graph_opt_pose3& pose) {
  return normalized(Pose{
      pose.t_xyz[0],
      pose.t_xyz[1],
      pose.t_xyz[2],
      pose.q_wxyz[0],
      pose.q_wxyz[1],
      pose.q_wxyz[2],
      pose.q_wxyz[3],
  });
}

std::vector<std::string> split_ws(const std::string& line) {
  std::stringstream ss(line);
  std::vector<std::string> tokens;
  std::string token;
  while (ss >> token) {
    tokens.push_back(token);
  }
  return tokens;
}

void write_pcd(const std::filesystem::path& path, const std::vector<Point>& points) {
  std::ofstream out(path, std::ios::binary);
  if (!out.is_open()) {
    throw std::runtime_error("failed to write pcd: " + path.string());
  }
  out << "# .PCD v0.7 - Point Cloud Data file format\n";
  out << "VERSION 0.7\n";
  out << "FIELDS x y z intensity\n";
  out << "SIZE 4 4 4 4\n";
  out << "TYPE F F F F\n";
  out << "COUNT 1 1 1 1\n";
  out << "WIDTH " << points.size() << "\n";
  out << "HEIGHT 1\n";
  out << "VIEWPOINT 0 0 0 1 0 0 0\n";
  out << "POINTS " << points.size() << "\n";
  out << "DATA binary\n";
  for (const Point& point : points) {
    const float row[4] = {point.x, point.y, point.z, point.intensity};
    out.write(reinterpret_cast<const char*>(row), sizeof(row));
  }
}

std::vector<Point> transform_points(const std::vector<Point>& points, const Pose& pose) {
  std::vector<Point> out;
  out.reserve(points.size());
  for (const Point& point : points) {
    const auto rotated = rotate_point(pose, point.x, point.y, point.z);
    out.push_back(Point{
        static_cast<float>(rotated[0] + pose.x),
        static_cast<float>(rotated[1] + pose.y),
        static_cast<float>(rotated[2] + pose.z),
        point.intensity,
    });
  }
  return out;
}

void append_points(std::vector<Point>& dst, std::vector<Point>&& src) {
  dst.reserve(dst.size() + src.size());
  std::move(src.begin(), src.end(), std::back_inserter(dst));
}

void write_poses(const std::filesystem::path& path, const std::vector<Keyframe>& poses) {
  std::ofstream out(path);
  if (!out.is_open()) {
    throw std::runtime_error("failed to write poses: " + path.string());
  }
  out << std::setprecision(17);
  for (const Keyframe& keyframe : poses) {
    if (!keyframe.patch_name.empty()) {
      out << keyframe.patch_name << " ";
    }
    out << keyframe.pose.x << " " << keyframe.pose.y << " " << keyframe.pose.z << " "
        << keyframe.pose.qw << " " << keyframe.pose.qx << " " << keyframe.pose.qy << " "
        << keyframe.pose.qz << "\n";
  }
}

std::string json_escape(const std::string& value) {
  std::string out;
  out.reserve(value.size());
  for (char c : value) {
    if (c == '\\' || c == '"') {
      out.push_back('\\');
    }
    out.push_back(c);
  }
  return out;
}

void write_report(
    const std::filesystem::path& path,
    const OptimizeOptions& options,
    const Result& result,
    const lt_pose_graph_opt_report& report) {
  std::ofstream out(path);
  if (!out.is_open()) {
    throw std::runtime_error("failed to write optimization report");
  }
  out << std::setprecision(17);
  out << "{\n";
  out << "  \"schema\": \"lingtu.map_optimization.v1\",\n";
  out << "  \"strategy\": \"" << json_escape(options.strategy) << "\",\n";
  out << "  \"backend\": \"pose_graph_opt\",\n";
  out << "  \"success\": " << (result.ok ? "true" : "false") << ",\n";
  out << "  \"performed\": " << (result.ok && result.changed ? "true" : "false") << ",\n";
  out << "  \"code\": \"" << json_escape(result.code) << "\",\n";
  out << "  \"message\": \"" << json_escape(result.message) << "\",\n";
  out << "  \"pose_count\": " << result.pose_count << ",\n";
  out << "  \"patch_count\": " << result.patch_count << ",\n";
  out << "  \"factor_count\": " << result.factor_count << ",\n";
  out << "  \"iterations\": " << report.iterations << ",\n";
  out << "  \"accepted_steps\": " << report.accepted_steps << ",\n";
  out << "  \"rejected_steps\": " << report.rejected_steps << ",\n";
  out << "  \"converged\": " << (report.converged ? "true" : "false") << ",\n";
  out << "  \"initial_cost\": " << report.initial_cost << ",\n";
  out << "  \"final_cost\": " << report.final_cost << "\n";
  out << "}\n";
  out.flush();
  if (!out) {
    throw std::runtime_error("failed to write optimization report");
  }
}

struct PatchBundleManifest {
  bool complete = false;
  std::uint64_t dropped_count = 0;
  std::uint64_t first_sequence = 0;
  std::uint64_t last_sequence = 0;
  std::size_t patch_count = 0;
};

bool parse_uint64(const std::string& token, std::uint64_t& value) {
  if (token.empty() || token.front() == '-') {
    return false;
  }
  char* end = nullptr;
  const unsigned long long parsed = std::strtoull(token.c_str(), &end, 10);
  if (end == token.c_str() || end == nullptr || *end != '\0') {
    return false;
  }
  value = static_cast<std::uint64_t>(parsed);
  return true;
}

bool read_patch_bundle_manifest(
    const std::filesystem::path& path,
    PatchBundleManifest& manifest,
    std::string& reason) {
  std::ifstream in(path);
  if (!in.is_open()) {
    reason = "patch_bundle_manifest_missing";
    return false;
  }
  std::string line;
  if (!std::getline(in, line) || line != "LINGTU_PATCH_BUNDLE_V1") {
    reason = "patch_bundle_manifest_malformed";
    return false;
  }
  const std::array<const char*, 5> keys = {
      "complete", "dropped_count", "first_sequence", "last_sequence", "patch_count"};
  std::array<std::uint64_t, 5> values{};
  for (std::size_t i = 0; i < keys.size(); ++i) {
    if (!std::getline(in, line)) {
      reason = "patch_bundle_manifest_malformed";
      return false;
    }
    const auto tokens = split_ws(line);
    if (tokens.size() != 2 || tokens[0] != keys[i] || !parse_uint64(tokens[1], values[i])) {
      reason = "patch_bundle_manifest_malformed";
      return false;
    }
  }
  while (std::getline(in, line)) {
    if (!split_ws(line).empty()) {
      reason = "patch_bundle_manifest_malformed";
      return false;
    }
  }
  if (values[0] > 1 || values[4] > std::numeric_limits<std::size_t>::max()) {
    reason = "patch_bundle_manifest_malformed";
    return false;
  }
  manifest.complete = values[0] == 1;
  manifest.dropped_count = values[1];
  manifest.first_sequence = values[2];
  manifest.last_sequence = values[3];
  manifest.patch_count = static_cast<std::size_t>(values[4]);
  return true;
}

Result fail_from_exception(const Result& base, const std::exception& exc, std::string code) {
  Result result = base;
  result.ok = false;
  result.code = std::move(code);
  result.message = exc.what();
  return result;
}

}  // namespace

std::vector<std::size_t> connected_pose_indices(
    std::size_t pose_count, const std::vector<GeometricConstraint>& constraints,
    std::size_t root) {
  std::vector<std::vector<std::size_t>> neighbors(pose_count);
  for (const auto& edge : constraints) {
    if (edge.from_index >= pose_count || edge.to_index >= pose_count)
      throw std::invalid_argument("graph edge index exceeds pose count");
    neighbors[edge.from_index].push_back(edge.to_index);
    neighbors[edge.to_index].push_back(edge.from_index);
  }
  if (pose_count == 0) return {};
  if (root >= pose_count) throw std::invalid_argument("graph root exceeds pose count");
  std::vector<bool> visited(pose_count, false);
  std::vector<std::size_t> indices{root};
  visited[root] = true;
  for (std::size_t head = 0; head < indices.size(); ++head)
    for (auto next : neighbors[indices[head]])
      if (!visited[next]) {
        visited[next] = true;
        indices.push_back(next);
      }
  std::sort(indices.begin(), indices.end());
  return indices;
}

GraphSolution optimize_graph(const std::vector<Keyframe>& keyframes,
                             const OptimizeOptions& options) {
  GraphSolution result;
  if (keyframes.empty() || keyframes.size() > std::numeric_limits<uint32_t>::max() ||
      options.max_iterations == 0 ||
      options.max_iterations > std::numeric_limits<uint32_t>::max()) {
    result.code = "invalid_graph_input";
    result.message = "graph requires poses and a representable positive iteration bound";
    return result;
  }
  if (options.geometric_constraints.empty()) {
    result.code = "no_independent_constraints";
    result.message = "pose estimates alone do not define graph measurements";
    return result;
  }
  const auto valid_pose = [](const Pose& pose) {
    const double norm = std::sqrt(sqr(pose.qw) + sqr(pose.qx) + sqr(pose.qy) + sqr(pose.qz));
    return is_finite(pose.x) && is_finite(pose.y) && is_finite(pose.z) &&
           is_finite(norm) && norm >= 0.9 && norm <= 1.1;
  };
  for (const auto& keyframe : keyframes) {
    if (!valid_pose(keyframe.pose)) {
      result.code = "invalid_graph_pose";
      result.message = "graph poses must be finite with unit quaternions";
      return result;
    }
  }
  for (const auto& constraint : options.geometric_constraints) {
    if (!valid_pose(constraint.pose_from_to)) {
      result.code = "geometric_constraint_invalid";
      result.message = "constraint transform must be finite with a unit quaternion";
      return result;
    }
  }
  std::vector<lt_pose_graph_opt_pose3> poses;
  poses.reserve(keyframes.size());
  for (const Keyframe& keyframe : keyframes) {
    poses.push_back(to_kernel_pose(keyframe.pose));
  }

  std::vector<lt_pose_graph_opt_between3> betweens;
  betweens.reserve(options.geometric_constraints.size());
  for (const GeometricConstraint& constraint : options.geometric_constraints) {
    if (constraint.from_index >= keyframes.size() ||
        constraint.to_index >= keyframes.size() ||
        constraint.from_index == constraint.to_index ||
        !valid_information_upper(constraint.information_upper)) {
      result.code = "geometric_constraint_invalid";
      result.message =
          "independent geometric constraint has invalid indices or information matrix";
      return result;
    }
    lt_pose_graph_opt_between3 edge{};
    edge.from_index = static_cast<uint32_t>(constraint.from_index);
    edge.to_index = static_cast<uint32_t>(constraint.to_index);
    edge.pose_from_to = to_kernel_pose(constraint.pose_from_to);
    std::copy(constraint.information_upper.begin(), constraint.information_upper.end(),
        std::begin(edge.information_upper));
    betweens.push_back(edge);
  }

  if (connected_pose_indices(keyframes.size(), options.geometric_constraints).size() !=
      keyframes.size()) {
    result.code = "graph_disconnected";
    result.message = "measured edges do not connect every pose to the fixed first pose";
    return result;
  }

  if (!options.gravity_reference.empty()) {
    if (options.gravity_reference.size() != keyframes.size() ||
        !std::isfinite(options.max_gravity_error_rad) || options.max_gravity_error_rad <= 0) {
      result.code = "invalid_gravity_reference";
      return result;
    }
    for (std::size_t i = 0; i < keyframes.size(); ++i) {
      const auto& reference = options.gravity_reference[i];
      if (!valid_pose(reference.pose) || reference.patch_name != keyframes[i].patch_name) {
        result.code = "invalid_gravity_reference";
        return result;
      }
    }
    return optimize_gravity_graph(keyframes, options);
  }

  lt_pose_graph_opt_config config{};
  config.struct_size = sizeof(lt_pose_graph_opt_config);
  config.version = LT_POSE_GRAPH_OPT_CONFIG_VERSION;
  config.max_iterations = static_cast<uint32_t>(options.max_iterations);
  config.method = 1;
  config.fixed_pose_index = 0;
  config.auto_anchor = 1;
  config.initial_lambda = 1e-3;
  config.tolerance = 1e-9;
  config.numeric_epsilon = 1e-6;

  lt_pose_graph_opt_handle* handle = lt_pose_graph_opt_create(&config);
  if (handle == nullptr) {
    result.ok = false;
    result.code = "optimizer_create_failed";
    result.message = "pose graph optimizer could not be created";
    return result;
  }

  lt_pose_graph_opt_report report{};
  const lt_pose_graph_opt_result status = lt_pose_graph_opt_process_se3(
    handle,
    poses.data(),
    static_cast<uint64_t>(poses.size()),
    nullptr,
    0,
    betweens.empty() ? nullptr : betweens.data(),
    static_cast<uint64_t>(betweens.size()),
    &report);
  if (status != LT_POSE_GRAPH_OPT_OK) {
    lt_pose_graph_opt_destroy(handle);
    result.ok = false;
    result.code = "optimizer_failed";
    result.message = "pose graph optimizer failed: " + std::to_string(status);
    return result;
  }

  uint64_t written = 0;
  const lt_pose_graph_opt_result copy_status = lt_pose_graph_opt_copy_result_poses(
    handle,
    poses.data(),
    static_cast<uint64_t>(poses.size()),
    &written);
  lt_pose_graph_opt_destroy(handle);
  if (copy_status != LT_POSE_GRAPH_OPT_OK || written != poses.size()) {
    result.ok = false;
    result.code = "optimizer_copy_failed";
    result.message = "pose graph optimizer did not return all poses";
    return result;
  }

  if (!report.converged || !is_finite(report.initial_cost) ||
      !is_finite(report.final_cost) || report.final_cost > report.initial_cost) {
    result.code = "optimizer_quality_failed";
    result.message = "pose graph optimizer did not converge to a finite non-increasing cost";
    result.report = report;
    return result;
  }
  result.keyframes = keyframes;
  for (std::size_t i = 0; i < keyframes.size(); ++i) {
    result.keyframes[i].pose = from_kernel_pose(poses[i]);
  }

  result.report = report;
  result.ok = true;
  result.code = "optimized";
  result.message = "in-memory pose graph optimization completed";
  return result;

}

Result optimize_map(const Map& map, const OptimizeOptions& options) {
  Result result = check(map);
  if (!result.ok) {
    return result;
  }

  std::filesystem::path staging;
  try {
    std::vector<Keyframe> keyframes = read_poses(map.poses_txt);
    const auto patches = sorted_point_cloud_files(map.patches_dir);

    if (options.geometric_constraints.empty()) {
      result.ok = true;
      result.code = "skipped_no_independent_constraints";
      result.message = options.strategy + " skipped: no independent factors were provided";
      result.patch_count = patches.size();
      result.pose_count = keyframes.size();
      result.factor_count = 0;
      result.iterations = 0;
      result.changed = false;
      const auto existing_report = map.map_dir / "map_optimization.json";
      if (std::filesystem::is_regular_file(existing_report)) {
        result.report_path = existing_report;
      }
      return result;
    }

    PatchBundleManifest manifest;
    std::string manifest_reason;
    if (!read_patch_bundle_manifest(
            map.patch_bundle_manifest, manifest, manifest_reason)) {
      result.ok = false;
      result.code = manifest_reason;
      result.message = "PGO requires a valid patch_bundle.manifest";
      return result;
    }
    if (!manifest.complete || manifest.dropped_count != 0) {
      result.ok = false;
      result.code = "patch_bundle_incomplete";
      result.message = "PGO refuses a patch bundle whose in-memory history was truncated";
      return result;
    }
    if (manifest.patch_count != patches.size() || manifest.patch_count != keyframes.size() ||
        manifest.patch_count == 0 || manifest.first_sequence != 0 ||
        manifest.last_sequence != manifest.patch_count - 1) {
      result.ok = false;
      result.code = "patch_bundle_manifest_mismatch";
      result.message = "patch bundle manifest does not match its poses and patches";
      return result;
    }

    std::unordered_set<std::string> pose_patch_names;
    pose_patch_names.reserve(keyframes.size());
    for (const Keyframe& keyframe : keyframes) {
      if (!pose_patch_names.insert(keyframe.patch_name).second) {
        result.ok = false;
        result.code = "duplicate_patch_name";
        result.message = "poses.txt contains a duplicate patch name";
        return result;
      }
    }
    std::unordered_set<std::string> disk_patch_names;
    disk_patch_names.reserve(patches.size());
    for (const auto& patch : patches) {
      disk_patch_names.insert(patch.filename().string());
    }
    if (disk_patch_names != pose_patch_names) {
      result.ok = false;
      result.code = "patch_pose_mismatch";
      result.message = "patch files and poses.txt patch names are not identical";
      return result;
    }

    std::error_code path_error;
    const auto source = std::filesystem::weakly_canonical(map.map_dir, path_error);
    if (path_error) {
      throw std::runtime_error("failed to normalize source map directory");
    }
    const auto output = std::filesystem::weakly_canonical(map.output_dir, path_error);
    if (path_error) {
      throw std::runtime_error("failed to normalize output directory");
    }
    auto output_it = output.begin();
    bool output_within_source = true;
    for (auto source_it = source.begin(); source_it != source.end(); ++source_it, ++output_it) {
      if (output_it == output.end() || *output_it != *source_it) {
        output_within_source = false;
        break;
      }
    }
    if (output_within_source) {
      result.ok = false;
      result.code = "output_inside_input";
      result.message = "optimization output must be outside the source map directory";
      return result;
    }
    if (std::filesystem::exists(map.output_dir)) {
      result.ok = false;
      result.code = "output_exists";
      result.message = "optimization output directory already exists";
      return result;
    }

    auto measured_options = options;
    if (measured_options.gravity_reference.empty()) {
      const auto raw_poses = map.map_dir / "poses.raw.txt";
      measured_options.gravity_reference = std::filesystem::is_regular_file(raw_poses)
          ? read_poses(raw_poses) : keyframes;
    }
    const auto solution = optimize_graph(keyframes, measured_options);
    if (!solution.ok) {
      result.ok = false;
      result.code = solution.code;
      result.message = solution.message;
      result.pose_count = keyframes.size();
      result.factor_count = options.geometric_constraints.size();
      result.iterations = solution.report.iterations;
      return result;
    }
    keyframes = solution.keyframes;
    const auto& report = solution.report;

    std::vector<Point> map_points;
    for (const Keyframe& keyframe : keyframes) {
      const auto patch_path = map.patches_dir / keyframe.patch_name;
      if (!std::filesystem::is_regular_file(patch_path)) {
        result.ok = false;
        result.code = "patch_missing";
        result.message = "patch file missing: " + patch_path.string();
        return result;
      }
      append_points(map_points, transform_points(read_point_cloud(patch_path), keyframe.pose));
    }

    staging = map.output_dir;
    staging += ".tmp";
    if (std::filesystem::exists(staging)) {
      result.ok = false;
      result.code = "output_staging_exists";
      result.message = "optimization staging directory already exists";
      return result;
    }
    std::filesystem::create_directories(staging / "patches");
    const std::filesystem::path out_map = staging / "map.pcd";
    const std::filesystem::path out_poses = staging / "poses.txt";
    write_pcd(out_map, map_points);
    write_poses(out_poses, keyframes);
    for (const auto& patch : patches) {
      std::filesystem::copy_file(
          patch, staging / "patches" / patch.filename(),
          std::filesystem::copy_options::none);
    }
    std::filesystem::copy_file(
        map.patch_bundle_manifest,
        staging / "patch_bundle.manifest",
        std::filesystem::copy_options::none);
    const auto scan_origin = map.map_dir / "scan_origin.txt";
    if (std::filesystem::is_regular_file(scan_origin))
      std::filesystem::copy_file(scan_origin, staging / "scan_origin.txt");
    for (const auto* filename : {"poses.raw.txt", "trajectory.raw.txt"}) {
      const auto source = map.map_dir / filename;
      if (std::filesystem::is_regular_file(source))
        std::filesystem::copy_file(source, staging / filename);
    }

    result.ok = true;
    result.code = "optimized";
    result.message = options.strategy + " optimization completed";
    result.patch_count = keyframes.size();
    result.pose_count = keyframes.size();
    result.factor_count = options.geometric_constraints.size();
    result.iterations = report.iterations;
    result.changed = true;
    result.report_path = map.output_dir / "map_optimization.json";
    write_report(staging / "map_optimization.json", options, result, report);
    std::filesystem::rename(staging, map.output_dir);
    staging.clear();
    return result;
  } catch (const std::exception& exc) {
    if (!staging.empty()) {
      std::error_code cleanup_error;
      std::filesystem::remove_all(staging, cleanup_error);
    }
    return fail_from_exception(result, exc, "optimizer_io_failed");
  }
}

}  // namespace lingtu::localization::opt

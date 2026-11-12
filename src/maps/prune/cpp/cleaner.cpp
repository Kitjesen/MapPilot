#include "cleaner.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <sstream>
#include <unordered_map>
#include <utility>
#include <vector>

#include "core/io.hpp"
#include "core/save.hpp"
#include "core/text.hpp"
#include "core/dufo.hpp"

namespace fs = std::filesystem;

namespace lingtu::map_cleaning {
namespace {

StaticCleanerResult fail(std::string reason, std::string message) {
  StaticCleanerResult result;
  result.success = false;
  result.reason_code = std::move(reason);
  result.message = std::move(message);
  return result;
}

}  // namespace

StaticCleanerResult cleanStaticMap(const StaticCleanerOptions &requested) {
  try {
    auto options = requested;
    if (options.map_dir.empty()) {
      return fail("missing_map_dir", "missing --map-dir");
    }
    if (!std::isfinite(options.voxel_size_m) || options.voxel_size_m <= 0.0F) {
      return fail("bad_voxel_size", "voxel size must be positive");
    }
    if (options.dry_run && options.apply_to_map) {
      return fail("conflicting_mode", "--dry-run cannot be combined with --apply");
    }

    const fs::path map_dir = fs::absolute(options.map_dir);
    const fs::path map_pcd = map_dir / "map.pcd";
    const fs::path patches_dir = map_dir / "patches";
    const fs::path poses_path = map_dir / "poses.txt";
    fs::path clean_path =
        options.output_clean_pcd.empty() ? (map_dir / "map.clean.pcd") : options.output_clean_pcd;
    fs::path removed_path = options.output_removed_pcd.empty() ? (map_dir / "map.removed.pcd")
                                                               : options.output_removed_pcd;
    const fs::path canonical_backup = map_dir / "map.before_clean.pcd";
    const fs::path legacy_backup = map_dir / "map.pcd.preclean";
    const fs::path backup_path =
        fs::exists(canonical_backup) || !fs::exists(legacy_backup)
            ? canonical_backup : legacy_backup;
    const fs::path tmp_map_path = fs::absolute(map_dir / "map.pcd.tmpclean");
    clean_path = fs::absolute(clean_path).lexically_normal();
    removed_path = fs::absolute(removed_path).lexically_normal();

    if (!fs::is_regular_file(map_pcd)) {
      return fail("missing_map_pcd", "map.pcd not found: " + map_pcd.string());
    }
    if (!fs::is_directory(patches_dir)) {
      return fail("missing_patches", "patches directory not found: " + patches_dir.string());
    }
    if (!fs::is_regular_file(poses_path)) {
      return fail("missing_poses", "poses.txt not found: " + poses_path.string());
    }
    if (!options.sensor_origin)
      options.sensor_origin = readSensorOrigin(map_dir / "scan_origin.txt");
    if (!options.sensor_origin)
      return fail("missing_sensor_origin", "provide calibrated --sensor-origin X Y Z or scan_origin.txt");
    if (!std::all_of(options.sensor_origin->begin(), options.sensor_origin->end(),
        [](float value) { return std::isfinite(value); }))
      return fail("bad_sensor_origin", "sensor origin must be finite");
    const auto same_file = [](const fs::path& left, const fs::path& right) {
      return left.lexically_normal() == right.lexically_normal()
          || (fs::exists(left) && fs::exists(right) && fs::equivalent(left, right));
    };
    if (same_file(clean_path, removed_path)) {
      return fail("bad_output_path", "clean and removed outputs must differ");
    }
    for (const auto& input : {map_pcd, poses_path, canonical_backup, legacy_backup,
                             tmp_map_path, map_dir / "scan_origin.txt"}) {
      if (same_file(clean_path, input) || same_file(removed_path, input)) {
        return fail("bad_output_path", "outputs cannot overwrite source, poses, backup or staging files");
      }
    }
    if (!options.dry_run && !options.overwrite &&
        (fs::exists(clean_path) || fs::exists(removed_path) ||
         (options.apply_to_map && (fs::exists(backup_path) || fs::exists(tmp_map_path))))) {
      return fail("output_exists", "output exists; pass --overwrite");
    }
    std::vector<fs::path> patches;
    for (const fs::directory_entry &entry : fs::directory_iterator(patches_dir)) {
      if (entry.is_regular_file() && entry.path().extension() == ".pcd") {
        patches.push_back(entry.path());
      }
    }
    std::sort(patches.begin(), patches.end());
    if (patches.empty()) {
      return fail("no_patches", "no patch PCD files found in " + patches_dir.string());
    }
    for (const auto& patch : patches) {
      if (same_file(clean_path, patch) || same_file(removed_path, patch)) {
        return fail("bad_output_path", "outputs cannot overwrite scan patches");
      }
    }

    std::vector<std::string> patch_order;
    const auto poses = readLingtuPoses(poses_path, &patch_order);
    if (poses.empty()) {
      return fail("no_poses", "no usable LingTu poses found in " + poses_path.string());
    }

    std::unordered_map<std::string, std::size_t> order;
    for (std::size_t i = 0; i < patch_order.size(); ++i) order.emplace(patch_order[i], i);
    const auto rank = [&](const fs::path& patch) {
      const auto it = order.find(patch.filename().string());
      return it == order.end() ? order.size() : it->second;
    };
    std::sort(patches.begin(), patches.end(), [&](const auto& a, const auto& b) { return rank(a) < rank(b); });
    const std::vector<PointXYZI> source_map = readPcd(map_pcd);
    if (source_map.empty()) return fail("empty_source_map", "map.pcd contains no points");
    const auto finite_point = [](const PointXYZI& p) {
      return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
    };
    if (!std::all_of(source_map.begin(), source_map.end(), finite_point))
      return fail("invalid_map_point", "map.pcd contains non-finite coordinates");
    DufoMap evidence(options.voxel_size_m);
    std::uint64_t matched_patches = 0;
    bool has_scan_points = false;
    for (const fs::path &patch : patches) {
      const std::string patch_name = patch.filename().string();
      auto pose_it = poses.find(patch_name);
      if (pose_it == poses.end()) {
        continue;
      }
      const std::vector<PointXYZI> points = readPcd(patch);
      if (!std::all_of(points.begin(), points.end(), finite_point))
        return fail("invalid_scan_point", "scan contains non-finite coordinates: " + patch_name);
      const auto& pose = pose_it->second;
      const double norm2 = pose.qw * pose.qw + pose.qx * pose.qx + pose.qy * pose.qy + pose.qz * pose.qz;
      if (!std::isfinite(pose.tx) || !std::isfinite(pose.ty) || !std::isfinite(pose.tz)
          || !std::isfinite(norm2) || norm2 < 1e-12)
        return fail("invalid_scan_pose", "scan pose is not a finite rigid transform: " + patch_name);
      evidence.observe(points, pose, *options.sensor_origin);
      has_scan_points = has_scan_points || !points.empty();
      ++matched_patches;
    }

    if (matched_patches == 0 || !has_scan_points) {
      return fail("no_matched_patch_evidence", "no patch PCD files matched poses.txt");
    }

    const auto remove_mask = evidence.removeMask(source_map);
    StaticCleanerResult result;
    result.removed_points = std::count(remove_mask.begin(), remove_mask.end(), std::uint8_t{1});
    result.kept_points = source_map.size() - result.removed_points;

    result.success = true;
    result.reason_code = options.dry_run ? "analyzed" : "cleaned";
    result.message = options.dry_run ? "analysis completed; no files written"
        : "DUFOMap saved-map cleanup completed";
    result.dry_run = options.dry_run;
    result.unmatched_patch_count = patches.size() - matched_patches;
    result.patch_count = matched_patches;
    result.pose_count = poses.size();
    result.source_points = source_map.size();
    if (!options.dry_run) {
      fs::create_directories(clean_path.parent_path());
      fs::create_directories(removed_path.parent_path());
      const SaveOptions save_options{
          map_pcd, clean_path, removed_path, backup_path,
          tmp_map_path, options.overwrite, options.apply_to_map,
      };
      const auto saved = writeCleanedMap(save_options, remove_mask);
      result.success = saved.success;
      if (!saved.success) {
        result.reason_code = saved.reason_code;
        result.message = saved.message;
        return result;
      }
      result.clean_pcd = clean_path;
      result.removed_pcd = removed_path;
      result.backup_pcd = saved.backup_pcd;
      result.applied = options.apply_to_map;
    }
    return result;
  } catch (const std::exception &exc) {
    return fail("exception", exc.what());
  }
}

std::string toJson(const StaticCleanerResult &result) {
  std::ostringstream out;
  out << "{\n";
  out << "  \"success\": " << (result.success ? "true" : "false") << ",\n";
  out << "  \"dry_run\": " << (result.dry_run ? "true" : "false") << ",\n";
  out << "  \"applied\": " << (result.applied ? "true" : "false") << ",\n";
  out << "  \"reason_code\": \"" << jsonEscape(result.reason_code) << "\",\n";
  out << "  \"message\": \"" << jsonEscape(result.message) << "\",\n";
  out << "  \"clean_pcd\": \"" << jsonEscape(genericString(result.clean_pcd)) << "\",\n";
  out << "  \"removed_pcd\": \"" << jsonEscape(genericString(result.removed_pcd)) << "\",\n";
  out << "  \"backup_pcd\": \"" << jsonEscape(genericString(result.backup_pcd)) << "\",\n";
  out << "  \"patch_count\": " << result.patch_count << ",\n";
  out << "  \"unmatched_patch_count\": " << result.unmatched_patch_count << ",\n";
  out << "  \"pose_count\": " << result.pose_count << ",\n";
  out << "  \"source_points\": " << result.source_points << ",\n";
  out << "  \"kept_points\": " << result.kept_points << ",\n";
  out << "  \"removed_points\": " << result.removed_points << ",\n";
  out << "  \"removed_fraction\": " << (result.source_points
      ? static_cast<double>(result.removed_points) / result.source_points : 0.0) << ",\n";
  out << "  \"removal_semantics\": \"dufomap_seen_free_not_semantic_motion\",\n";
  out << "  \"algorithm\": \"dufomap.9e239ddd_ufomap.fcb20f44\",\n";
  out << "  \"license_boundary\": \"DUFOMap and UFOMap BSD-3-Clause notices accompany the binary\"\n";
  out << "}\n";
  return out.str();
}

}  // namespace lingtu::map_cleaning

#include "cleaner.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <sstream>
#include <unordered_map>
#include <utility>
#include <vector>

#include "core/evidence.hpp"
#include "core/flow.hpp"
#include "core/io.hpp"
#include "core/save.hpp"
#include "core/score.hpp"
#include "core/text.hpp"
#include "core/visibility.hpp"

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
    if (!std::isfinite(options.ground_z_threshold) || !std::isfinite(options.instance_grid_m)
        || options.instance_grid_m <= 0 || !std::isfinite(options.moving_score_threshold)
        || options.moving_score_threshold < 0 || options.moving_score_threshold > 1) {
      return fail("bad_threshold", "ground and score thresholds must be finite and in range");
    }
    if (options.dry_run && options.apply_to_map) {
      return fail("conflicting_mode", "--dry-run cannot be combined with --apply");
    }
    if (options.min_frame_support == 0 || options.min_hit_support == 0) {
      return fail("bad_support_threshold", "support thresholds must be positive");
    }

    if (options.min_free_frames < 2 || !std::isfinite(options.ray_tolerance_m)
        || options.ray_tolerance_m <= 0 || !std::isfinite(options.endpoint_margin_m)
        || options.endpoint_margin_m < options.ray_tolerance_m
        || !std::isfinite(options.max_ray_length_m) || options.max_ray_length_m <= 0) {
      return fail("bad_visibility_threshold", "visibility requires at least two frames and finite positive distances");
    }
    const fs::path map_dir = fs::absolute(options.map_dir);
    const fs::path map_pcd = map_dir / "map.pcd";
    const fs::path patches_dir = map_dir / "patches";
    const fs::path poses_path = map_dir / "poses.txt";
    fs::path clean_path =
        options.output_clean_pcd.empty() ? (map_dir / "map.clean.pcd") : options.output_clean_pcd;
    fs::path removed_path = options.output_removed_pcd.empty() ? (map_dir / "map.removed.pcd")
                                                               : options.output_removed_pcd;
    const fs::path backup_path = fs::absolute(map_dir / "map.pcd.preclean");
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
    for (const auto& input : {map_pcd, poses_path, backup_path, tmp_map_path, map_dir / "scan_origin.txt"}) {
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
    VisibilityEvidence visibility(source_map, options);
    std::unordered_map<VoxelKey, VoxelEvidence, VoxelKeyHash> evidence;
    std::uint64_t matched_patches = 0;
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
      visibility.observe(points, pose);
      const std::size_t frame_idx = static_cast<std::size_t>(matched_patches);
      for (const PointXYZI &local_pt : points) {
        PointXYZI map_pt = transformPoint(local_pt, pose_it->second);
        const VoxelKey key = voxelKey(map_pt, options.voxel_size_m);
        VoxelEvidence &item = evidence[key];
        ++item.hits;
        if (local_pt.z <= options.ground_z_threshold) {
          ++item.ground_hits;
        }
        if (item.last_frame != frame_idx) {
          item.last_frame = frame_idx;
          ++item.frame_count;
        }
      }
      ++matched_patches;
    }

    if (matched_patches == 0 || evidence.empty()) {
      return fail("no_matched_patch_evidence", "no patch PCD files matched poses.txt");
    }

    std::vector<PointXYZI> kept;
    std::vector<PointXYZI> removed;
    kept.reserve(source_map.size());

    const MovingScoreSummary score_summary = scoreMovingInstances(source_map, evidence, options);

    std::uint64_t dynamic_voxels = 0;
    for (const auto &entry : evidence) {
      if (!isProtected(entry.second, options)) {
        ++dynamic_voxels;
      }
    }

    StaticCleanerResult result;
    for (std::size_t index = 0; index < source_map.size(); ++index) {
      const auto& pt = source_map[index];
      if (visibility.contradicted(index)) ++result.free_space_candidate_points;
      const VoxelKey key = voxelKey(pt, options.voxel_size_m);
      auto found = evidence.find(key);
      if (found == evidence.end()) {
        ++result.kept_without_evidence_points;
        kept.push_back(pt);
      } else if (found->second.ground_hits > 0) {
        ++result.kept_ground_points;
        kept.push_back(pt);
      } else if (visibility.contradicted(index)) {
        if (visibility.onSupportedSurface(index)) {
          ++result.kept_supported_surface_points;
          kept.push_back(pt);
        } else {
          removed.push_back(pt);
        }
      } else if (found->second.frame_count >= options.min_frame_support) {
        ++result.kept_multi_frame_points;
        kept.push_back(pt);
      } else if (found->second.hits >= options.min_hit_support) {
        ++result.kept_hit_support_points;
        kept.push_back(pt);
      } else {
        ++result.kept_unconfirmed_points;
        kept.push_back(pt);
      }
    }

    result.success = true;
    result.reason_code = options.dry_run ? "analyzed" : "cleaned";
    result.message = options.dry_run ? "analysis completed; no files written"
        : "prune completed with multi-frame observed free-space contradictions";
    result.preset = options.preset;
    result.dry_run = options.dry_run;
    result.unmatched_patch_count = patches.size() - matched_patches;
    result.patch_count = matched_patches;
    result.pose_count = poses.size();
    result.source_points = source_map.size();
    result.kept_points = kept.size();
    result.removed_points = removed.size();
    result.evidence_voxels = evidence.size();
    result.dynamic_candidate_voxels = dynamic_voxels;
    result.scored_instances = score_summary.scored_instances;
    result.moving_instances = score_summary.moving_instances;
    result.score_candidate_points = score_summary.candidate_points;
    result.max_moving_score = score_summary.max_candidate_ratio;
    if (!options.dry_run) {
      fs::create_directories(clean_path.parent_path());
      fs::create_directories(removed_path.parent_path());
      const SaveOptions save_options{
          map_pcd, clean_path, removed_path, backup_path,
          tmp_map_path, options.overwrite, options.apply_to_map,
      };
      const auto saved = writeCleanedMap(save_options, kept, removed);
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
  out << "  \"preset\": \"" << jsonEscape(result.preset) << "\",\n";
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
  out << "  \"decision_counts\": {\n";
  out << "    \"kept_ground_threshold\": " << result.kept_ground_points << ",\n";
  out << "    \"kept_multi_frame\": " << result.kept_multi_frame_points << ",\n";
  out << "    \"kept_hit_support\": " << result.kept_hit_support_points << ",\n";
  out << "    \"kept_without_evidence\": " << result.kept_without_evidence_points << ",\n";
  out << "    \"kept_unconfirmed\": " << result.kept_unconfirmed_points << ",\n";
  out << "    \"kept_supported_surface\": " << result.kept_supported_surface_points << ",\n";
  out << "    \"removed_free_space_contradiction\": " << result.removed_points << "\n  },\n";
  out << "  \"free_space_candidate_points\": " << result.free_space_candidate_points << ",\n";
  out << "  \"removal_semantics\": \"multi_frame_observed_free_space_not_semantic_motion\",\n";
  out << "  \"evidence_voxels\": " << result.evidence_voxels << ",\n";
  out << "  \"dynamic_candidate_voxels\": " << result.dynamic_candidate_voxels << ",\n";
  out << "  \"scored_instances\": " << result.scored_instances << ",\n";
  out << "  \"moving_instances\": " << result.moving_instances << ",\n";
  out << "  \"score_candidate_points\": " << result.score_candidate_points << ",\n";
  out << "  \"max_moving_score\": " << result.max_moving_score << ",\n";
  out << "  \"algorithm\": \"prune.visibility_v2\",\n";
  out << "  \"flow\": " << flowJson() << ",\n";
  out << "  \"license_boundary\": \"LingTu-owned clean-room implementation; does not include or "
         "link ERASOR2 code\"\n";
  out << "}\n";
  return out.str();
}

}  // namespace lingtu::map_cleaning

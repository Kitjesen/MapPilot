#include "cleaner.hpp"

#include <algorithm>
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

StaticCleanerResult cleanStaticMap(const StaticCleanerOptions &options) {
  try {
    if (options.map_dir.empty()) {
      return fail("missing_map_dir", "missing --map-dir");
    }
    if (options.voxel_size_m <= 0.0F) {
      return fail("bad_voxel_size", "voxel size must be positive");
    }
    if (options.min_frame_support == 0 || options.min_hit_support == 0) {
      return fail("bad_support_threshold", "support thresholds must be positive");
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
    clean_path = fs::absolute(clean_path);
    removed_path = fs::absolute(removed_path);

    if (!fs::is_regular_file(map_pcd)) {
      return fail("missing_map_pcd", "map.pcd not found: " + map_pcd.string());
    }
    if (!fs::is_directory(patches_dir)) {
      return fail("missing_patches", "patches directory not found: " + patches_dir.string());
    }
    if (!fs::is_regular_file(poses_path)) {
      return fail("missing_poses", "poses.txt not found: " + poses_path.string());
    }
    if (options.apply_to_map && clean_path == fs::absolute(map_pcd)) {
      return fail("bad_output_path", "--apply cannot use map.pcd as --out-clean");
    }
    if (removed_path == fs::absolute(map_pcd)) {
      return fail("bad_output_path", "--out-removed cannot be map.pcd");
    }
    if (!options.overwrite &&
        (fs::exists(clean_path) || fs::exists(removed_path) ||
         (options.apply_to_map && (fs::exists(backup_path) || fs::exists(tmp_map_path))))) {
      return fail("output_exists", "output exists; pass --overwrite");
    }
    fs::create_directories(clean_path.parent_path());
    fs::create_directories(removed_path.parent_path());

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

    const auto poses = readLingtuPoses(poses_path);
    if (poses.empty()) {
      return fail("no_poses", "no usable LingTu poses found in " + poses_path.string());
    }

    std::unordered_map<VoxelKey, VoxelEvidence, VoxelKeyHash> evidence;
    std::uint64_t matched_patches = 0;
    for (const fs::path &patch : patches) {
      const std::string patch_name = patch.filename().string();
      auto pose_it = poses.find(patch_name);
      if (pose_it == poses.end()) {
        continue;
      }
      const std::vector<PointXYZI> points = readPcd(patch);
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

    const std::vector<PointXYZI> source_map = readPcd(map_pcd);
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

    for (const PointXYZI &pt : source_map) {
      const VoxelKey key = voxelKey(pt, options.voxel_size_m);
      auto found = evidence.find(key);
      if (found == evidence.end() || isProtected(found->second, options)) {
        kept.push_back(pt);
      } else {
        removed.push_back(pt);
      }
    }

    const SaveOptions save_options{
        map_pcd,      clean_path,        removed_path,         backup_path,
        tmp_map_path, options.overwrite, options.apply_to_map,
    };
    const SaveResult save_result = writeCleanedMap(save_options, kept, removed);
    if (!save_result.success) {
      return fail(save_result.reason_code, save_result.message);
    }

    StaticCleanerResult result;
    result.success = true;
    result.reason_code = "cleaned";
    result.message = "prune completed with LingTu field temporal occupancy voting";
    result.preset = options.preset;
    result.clean_pcd = clean_path;
    result.removed_pcd = removed_path;
    result.backup_pcd = save_result.backup_pcd;
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
    return result;
  } catch (const std::exception &exc) {
    return fail("exception", exc.what());
  }
}

std::string toJson(const StaticCleanerResult &result) {
  std::ostringstream out;
  out << "{\n";
  out << "  \"success\": " << (result.success ? "true" : "false") << ",\n";
  out << "  \"reason_code\": \"" << jsonEscape(result.reason_code) << "\",\n";
  out << "  \"message\": \"" << jsonEscape(result.message) << "\",\n";
  out << "  \"preset\": \"" << jsonEscape(result.preset) << "\",\n";
  out << "  \"clean_pcd\": \"" << jsonEscape(genericString(result.clean_pcd)) << "\",\n";
  out << "  \"removed_pcd\": \"" << jsonEscape(genericString(result.removed_pcd)) << "\",\n";
  out << "  \"backup_pcd\": \"" << jsonEscape(genericString(result.backup_pcd)) << "\",\n";
  out << "  \"patch_count\": " << result.patch_count << ",\n";
  out << "  \"pose_count\": " << result.pose_count << ",\n";
  out << "  \"source_points\": " << result.source_points << ",\n";
  out << "  \"kept_points\": " << result.kept_points << ",\n";
  out << "  \"removed_points\": " << result.removed_points << ",\n";
  out << "  \"evidence_voxels\": " << result.evidence_voxels << ",\n";
  out << "  \"dynamic_candidate_voxels\": " << result.dynamic_candidate_voxels << ",\n";
  out << "  \"scored_instances\": " << result.scored_instances << ",\n";
  out << "  \"moving_instances\": " << result.moving_instances << ",\n";
  out << "  \"score_candidate_points\": " << result.score_candidate_points << ",\n";
  out << "  \"max_moving_score\": " << result.max_moving_score << ",\n";
  out << "  \"algorithm\": \"prune.temporal_occupancy_v1\",\n";
  out << "  \"flow\": " << flowJson() << ",\n";
  out << "  \"license_boundary\": \"LingTu-owned clean-room implementation; does not include or "
         "link ERASOR2 code\"\n";
  out << "}\n";
  return out.str();
}

}  // namespace lingtu::map_cleaning

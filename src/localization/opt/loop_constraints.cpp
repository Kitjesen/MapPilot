#include "localization/opt/loop_constraints.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "localization/opt/cloud.hpp"
#include "localization/opt/pose_math.hpp"

namespace lingtu::localization::opt {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr std::uint64_t kFnvOffset = 14695981039346656037ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

struct VoxelKey {
  int x = 0;
  int y = 0;
  int z = 0;

  bool operator==(const VoxelKey &other) const {
    return x == other.x && y == other.y && z == other.z;
  }

  bool operator<(const VoxelKey &other) const {
    return std::tie(x, y, z) < std::tie(other.x, other.y, other.z);
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey &key) const {
    std::size_t seed = static_cast<std::size_t>(key.x) * 73856093U;
    seed ^= static_cast<std::size_t>(key.y) * 19349663U;
    seed ^= static_cast<std::size_t>(key.z) * 83492791U;
    return seed;
  }
};

struct Accumulator {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double intensity = 0.0;
  std::size_t count = 0;
};

struct Descriptor {
  std::size_t angular_bins = 0;
  std::size_t features_per_angle = 0;
  std::vector<double> values;
};

struct YawHypothesis {
  double score = 0.0;
  double yaw_rad = 0.0;
  std::size_t shift = 0;
};

struct Candidate {
  std::size_t from_index = 0;
  std::size_t to_index = 0;
  double descriptor_score = 0.0;
  double descriptor_margin = 0.0;
  double descriptor_yaw_margin = 0.0;
  std::vector<YawHypothesis> yaw_hypotheses;
};

struct Correspondence {
  std::array<double, 3> source_transformed{};
  std::array<double, 3> target{};
  std::size_t target_index = 0;
  double distance = 0.0;
};

struct RegistrationResult {
  bool ok = false;
  std::string reason;
  Pose transform;
  std::size_t iterations = 0;
  std::size_t source_points = 0;
  std::size_t target_points = 0;
  std::size_t inliers = 0;
  double inlier_ratio = 0.0;
  double rmse_m = -1.0;
  double p50_m = -1.0;
  double p95_m = -1.0;
  double xy_span_m = 0.0;
  double z_span_m = 0.0;
  double hessian_condition = -1.0;
  std::array<double, 4> hessian_diagonal{};
  std::size_t planar_inliers = 0;
  double planar_inlier_ratio = 0.0;
  std::array<double, 4> point_to_plane_eigenvalues{};
  std::array<double, 4> point_to_plane_weak_mode{};
  double point_to_plane_condition = -1.0;
};

struct VerifiedCandidate {
  GeometricConstraint constraint;
  std::size_t diagnostic_index = 0;
  Pose correction;
};

bool finite_point(const Point &point) {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

std::optional<std::string> validate_loop_options(const LoopConstraintOptions &options) {
  const auto finite_positive = [](double value) { return std::isfinite(value) && value > 0.0; };
  const auto finite_nonnegative = [](double value) { return std::isfinite(value) && value >= 0.0; };
  const auto unit_interval = [](double value) {
    return std::isfinite(value) && value >= 0.0 && value <= 1.0;
  };
  if (options.min_index_separation == 0 || options.min_index_separation > 1000000) {
    return "min_index_separation must be positive";
  }
  if (!finite_nonnegative(options.min_path_separation_m) ||
      options.min_path_separation_m > 100000.0 || !finite_positive(options.candidate_xy_radius_m) ||
      options.candidate_xy_radius_m > 100.0 || !finite_nonnegative(options.candidate_max_abs_z_m) ||
      options.candidate_max_abs_z_m > 50.0) {
    return "candidate path/XY/Z gates must be finite and nonnegative";
  }
  if (options.submap_half_window > 100 || !finite_positive(options.voxel_size_m) ||
      options.voxel_size_m < 1.0e-4 || options.voxel_size_m > 5.0 ||
      options.max_points_per_submap == 0 || options.max_points_per_submap > 100000) {
    return "submap voxel size and point budget must be positive";
  }
  if (options.angular_bins < 4 || options.angular_bins > 720 || options.radial_bins < 2 ||
      options.radial_bins > 256 || options.height_bins < 2 || options.height_bins > 256 ||
      !finite_positive(options.descriptor_max_radius_m) ||
      options.descriptor_max_radius_m > 200.0 || !std::isfinite(options.descriptor_min_z_m) ||
      !std::isfinite(options.descriptor_max_z_m) || std::abs(options.descriptor_min_z_m) > 100.0 ||
      std::abs(options.descriptor_max_z_m) > 100.0 ||
      options.descriptor_min_z_m >= options.descriptor_max_z_m ||
      !unit_interval(options.descriptor_min_similarity) ||
      !unit_interval(options.descriptor_min_margin) ||
      !unit_interval(options.descriptor_min_yaw_margin)) {
    return "descriptor dimensions, bounds, similarity, and margins are invalid";
  }
  if (options.yaw_hypotheses == 0 || options.yaw_hypotheses > 32 ||
      options.max_spatial_candidates_per_anchor == 0 ||
      options.max_spatial_candidates_per_anchor > 1024 || options.max_candidates_per_anchor == 0 ||
      options.max_candidates_per_anchor > 32 || options.max_total_candidates == 0 ||
      options.max_total_candidates > 4096) {
    return "descriptor hypothesis and candidate budgets must be positive";
  }
  if (!finite_positive(options.max_correspondence_distance_m) ||
      options.max_correspondence_distance_m > 10.0 || options.max_icp_iterations == 0 ||
      options.max_icp_iterations > 100 || !finite_positive(options.trim_fraction) ||
      options.trim_fraction > 1.0 || options.min_inliers == 0 ||
      !unit_interval(options.min_inlier_ratio)) {
    return "ICP distance, iteration, trim, and inlier gates are invalid";
  }
  if (!finite_positive(options.max_rmse_m) || options.max_rmse_m > 10.0 ||
      !finite_positive(options.max_p95_m) || options.max_p95_m > 10.0 ||
      !finite_nonnegative(options.min_xy_span_m) || !finite_nonnegative(options.min_z_span_m) ||
      !std::isfinite(options.max_hessian_condition) || options.max_hessian_condition < 1.0) {
    return "registration residual, span, and Hessian gates are invalid";
  }
  if (!finite_positive(options.normal_radius_m) || options.normal_radius_m > 5.0 ||
      options.normal_min_neighbors < 3 || options.normal_min_neighbors > 256 ||
      !unit_interval(options.normal_max_small_to_middle_ratio) ||
      !unit_interval(options.normal_min_middle_to_large_ratio) ||
      !finite_nonnegative(options.normal_min_middle_variance_m2) ||
      options.normal_min_middle_variance_m2 > 25.0 || options.min_planar_inliers == 0 ||
      options.min_planar_inliers > options.max_points_per_submap ||
      !unit_interval(options.min_planar_inlier_ratio) ||
      !finite_positive(options.min_point_to_plane_eigenvalue) ||
      options.min_point_to_plane_eigenvalue > 1.0 ||
      !std::isfinite(options.max_point_to_plane_condition) ||
      options.max_point_to_plane_condition < 1.0 || options.max_point_to_plane_condition > 1.0e12) {
    return "normal estimation and point-to-plane observability gates are invalid";
  }
  if (!finite_nonnegative(options.max_correction_xy_m) ||
      !finite_nonnegative(options.max_correction_z_m) ||
      !finite_nonnegative(options.max_correction_yaw_rad) ||
      !finite_nonnegative(options.max_inverse_translation_m) ||
      !finite_nonnegative(options.max_inverse_yaw_rad)) {
    return "correction and inverse-consistency gates must be finite and nonnegative";
  }
  if (options.min_consistent_matches == 0 ||
      (options.require_consensus && options.min_consistent_matches < 2) ||
      options.consensus_anchor_tolerance == 0 ||
      options.consensus_step_tolerance > options.consensus_anchor_tolerance ||
      !finite_nonnegative(options.consensus_translation_m) ||
      !finite_nonnegative(options.consensus_yaw_rad)) {
    return "consensus support, step, translation, and yaw gates are invalid";
  }
  return std::nullopt;
}

std::optional<std::string> validate_keyframes(const std::vector<Keyframe> &keyframes) {
  for (std::size_t index = 0; index < keyframes.size(); ++index) {
    const Pose &pose = keyframes[index].pose;
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.z) ||
        !std::isfinite(pose.qw) || !std::isfinite(pose.qx) || !std::isfinite(pose.qy) ||
        !std::isfinite(pose.qz)) {
      return "keyframe " + std::to_string(index) + " has a non-finite pose";
    }
    const double norm =
        std::sqrt(square(pose.qw) + square(pose.qx) + square(pose.qy) + square(pose.qz));
    if (norm < 0.9 || norm > 1.1) {
      return "keyframe " + std::to_string(index) + " has an invalid quaternion norm";
    }
  }
  return std::nullopt;
}

VoxelKey voxel_key(double x, double y, double z, double resolution) {
  const double safe_resolution = std::max(1.0e-6, resolution);
  return {
      static_cast<int>(std::floor(x / safe_resolution)),
      static_cast<int>(std::floor(y / safe_resolution)),
      static_cast<int>(std::floor(z / safe_resolution)),
  };
}

Point transform_point(const Point &point, const Pose &transform) {
  const auto rotated = rotate_vector(transform, point.x, point.y, point.z);
  return Point{
      static_cast<float>(rotated[0] + transform.x),
      static_cast<float>(rotated[1] + transform.y),
      static_cast<float>(rotated[2] + transform.z),
      point.intensity,
  };
}

std::vector<Point> voxel_downsample(const std::vector<Point> &input, double resolution,
                                    std::size_t maximum_points) {
  std::unordered_map<VoxelKey, Accumulator, VoxelKeyHash> voxels;
  voxels.reserve(input.size());
  for (const Point &point : input) {
    if (!finite_point(point)) {
      continue;
    }
    Accumulator &accumulator = voxels[voxel_key(point.x, point.y, point.z, resolution)];
    accumulator.x += point.x;
    accumulator.y += point.y;
    accumulator.z += point.z;
    accumulator.intensity += point.intensity;
    ++accumulator.count;
  }

  std::vector<std::pair<VoxelKey, Accumulator>> ordered;
  ordered.reserve(voxels.size());
  for (const auto &entry : voxels) {
    ordered.push_back(entry);
  }
  std::sort(ordered.begin(), ordered.end(),
            [](const auto &lhs, const auto &rhs) { return lhs.first < rhs.first; });

  const std::size_t limit = std::max<std::size_t>(1, maximum_points);
  const std::size_t stride =
      ordered.size() <= limit
          ? 1
          : static_cast<std::size_t>(
                std::ceil(static_cast<double>(ordered.size()) / static_cast<double>(limit)));
  std::vector<Point> output;
  output.reserve(std::min(limit, ordered.size()));
  for (std::size_t index = 0; index < ordered.size(); index += stride) {
    const Accumulator &accumulator = ordered[index].second;
    if (accumulator.count == 0) {
      continue;
    }
    const double scale = 1.0 / static_cast<double>(accumulator.count);
    output.push_back(Point{
        static_cast<float>(accumulator.x * scale),
        static_cast<float>(accumulator.y * scale),
        static_cast<float>(accumulator.z * scale),
        static_cast<float>(accumulator.intensity * scale),
    });
    if (output.size() >= limit) {
      break;
    }
  }
  return output;
}

std::filesystem::path patch_path(const Map &map,
                                 const std::vector<std::filesystem::path> &sorted_patches,
                                 const Keyframe &keyframe, std::size_t index) {
  if (!keyframe.patch_name.empty()) {
    const std::filesystem::path requested(keyframe.patch_name);
    if (requested.is_absolute() || requested != requested.filename() ||
        requested.extension() != ".pcd") {
      return {};
    }
    const auto named = map.patches_dir / requested;
    if (std::filesystem::is_regular_file(named)) {
      return named;
    }
    // A named patch is part of the pose/cloud contract. Falling back to the
    // same ordinal position can silently attach a different cloud to a pose.
    return {};
  }
  if (index < sorted_patches.size()) {
    return sorted_patches[index];
  }
  return {};
}

std::vector<Point> build_submap(const Map &map, const std::vector<Keyframe> &keyframes,
                                const std::vector<std::filesystem::path> &sorted_patches,
                                std::size_t anchor, const LoopConstraintOptions &options) {
  const std::size_t begin =
      anchor > options.submap_half_window ? anchor - options.submap_half_window : 0;
  const std::size_t end = std::min(keyframes.size() - 1, anchor + options.submap_half_window);
  std::vector<Point> accumulated;
  for (std::size_t index = begin; index <= end; ++index) {
    const auto path = patch_path(map, sorted_patches, keyframes[index], index);
    if (path.empty()) {
      throw std::runtime_error("patch missing for keyframe " + std::to_string(index));
    }
    const Pose neighbor_to_anchor = between_poses(keyframes[anchor].pose, keyframes[index].pose);
    const auto cloud = read_point_cloud(path);
    accumulated.reserve(accumulated.size() + cloud.size());
    for (const Point &point : cloud) {
      if (finite_point(point)) {
        accumulated.push_back(transform_point(point, neighbor_to_anchor));
      }
    }
  }
  return voxel_downsample(accumulated, options.voxel_size_m, options.max_points_per_submap);
}

Pose map_gravity_pose(const Pose &map_body) {
  const auto rpy = pose_rpy(map_body);
  return pose_with_rpy(map_body, 0.0, 0.0, rpy[2]);
}

Pose gravity_body_pose(const Pose &map_body) {
  return between_poses(map_gravity_pose(map_body), map_body);
}

std::vector<Point> gravity_aligned_cloud(const std::vector<Point> &body_cloud,
                                         const Pose &map_body) {
  const Pose gravity_body = gravity_body_pose(map_body);
  std::vector<Point> aligned;
  aligned.reserve(body_cloud.size());
  for (const Point &point : body_cloud) {
    aligned.push_back(transform_point(point, gravity_body));
  }
  return aligned;
}

Pose gravity_measurement_to_body(const Pose &gravity_from_to, const Pose &map_body_from,
                                 const Pose &map_body_to) {
  return compose_pose(inverse_pose(gravity_body_pose(map_body_from)),
                      compose_pose(gravity_from_to, gravity_body_pose(map_body_to)));
}

bool submaps_overlap(std::size_t first, std::size_t second, const LoopConstraintOptions &options) {
  const std::size_t lower = std::min(first, second);
  const std::size_t upper = std::max(first, second);
  return upper - lower <= 2 * options.submap_half_window;
}

Descriptor make_descriptor(const std::vector<Point> &cloud, const LoopConstraintOptions &options) {
  Descriptor descriptor;
  descriptor.angular_bins = std::max<std::size_t>(4, options.angular_bins);
  const std::size_t radial_bins = std::max<std::size_t>(2, options.radial_bins);
  const std::size_t height_bins = std::max<std::size_t>(2, options.height_bins);
  descriptor.features_per_angle = radial_bins + height_bins;
  descriptor.values.assign(descriptor.angular_bins * descriptor.features_per_angle, 0.0);

  const double radius_limit = std::max(0.1, options.descriptor_max_radius_m);
  const double height_span = std::max(0.1, options.descriptor_max_z_m - options.descriptor_min_z_m);
  for (const Point &point : cloud) {
    const double radius = std::hypot(point.x, point.y);
    if (!finite_point(point) || radius > radius_limit || point.z < options.descriptor_min_z_m ||
        point.z > options.descriptor_max_z_m) {
      continue;
    }
    double angle = std::atan2(point.y, point.x);
    if (angle < 0.0) {
      angle += kTwoPi;
    }
    const std::size_t angular =
        std::min(descriptor.angular_bins - 1,
                 static_cast<std::size_t>(angle / kTwoPi * descriptor.angular_bins));
    const std::size_t radial =
        std::min(radial_bins - 1, static_cast<std::size_t>(radius / radius_limit * radial_bins));
    const std::size_t height =
        std::min(height_bins - 1, static_cast<std::size_t>((point.z - options.descriptor_min_z_m) /
                                                           height_span * height_bins));
    const std::size_t base = angular * descriptor.features_per_angle;
    descriptor.values[base + radial] += 1.0;
    descriptor.values[base + radial_bins + height] += 1.0;
  }

  const double norm = std::sqrt(std::inner_product(
      descriptor.values.begin(), descriptor.values.end(), descriptor.values.begin(), 0.0));
  if (norm > std::numeric_limits<double>::epsilon()) {
    for (double &value : descriptor.values) {
      value /= norm;
    }
  }
  return descriptor;
}

double shifted_similarity(const Descriptor &target, const Descriptor &source, std::size_t shift) {
  if (target.angular_bins != source.angular_bins ||
      target.features_per_angle != source.features_per_angle ||
      target.values.size() != source.values.size()) {
    return 0.0;
  }
  double score = 0.0;
  for (std::size_t target_angle = 0; target_angle < target.angular_bins; ++target_angle) {
    const std::size_t source_angle =
        (target_angle + target.angular_bins - shift % target.angular_bins) % target.angular_bins;
    const std::size_t target_base = target_angle * target.features_per_angle;
    const std::size_t source_base = source_angle * source.features_per_angle;
    for (std::size_t feature = 0; feature < target.features_per_angle; ++feature) {
      score += target.values[target_base + feature] * source.values[source_base + feature];
    }
  }
  return score;
}

std::vector<YawHypothesis> descriptor_yaw_hypotheses(const Descriptor &target,
                                                     const Descriptor &source,
                                                     std::size_t requested, double *margin) {
  std::vector<YawHypothesis> scores;
  scores.reserve(target.angular_bins);
  for (std::size_t shift = 0; shift < target.angular_bins; ++shift) {
    scores.push_back(YawHypothesis{
        shifted_similarity(target, source, shift),
        wrap_angle(kTwoPi * static_cast<double>(shift) / static_cast<double>(target.angular_bins)),
        shift,
    });
  }
  std::sort(scores.begin(), scores.end(), [](const auto &lhs, const auto &rhs) {
    if (lhs.score != rhs.score) {
      return lhs.score > rhs.score;
    }
    return lhs.shift < rhs.shift;
  });
  if (scores.empty()) {
    if (margin != nullptr) {
      *margin = 0.0;
    }
    return {};
  }

  double second = 0.0;
  for (std::size_t index = 1; index < scores.size(); ++index) {
    const std::size_t distance = std::min(
        (scores[index].shift + target.angular_bins - scores[0].shift) % target.angular_bins,
        (scores[0].shift + target.angular_bins - scores[index].shift) % target.angular_bins);
    if (distance > 1) {
      second = scores[index].score;
      break;
    }
  }
  if (margin != nullptr) {
    *margin = scores[0].score - second;
  }

  std::vector<YawHypothesis> selected;
  const std::size_t limit = std::max<std::size_t>(1, requested);
  for (const auto &hypothesis : scores) {
    bool separated = true;
    for (const auto &existing : selected) {
      const std::size_t distance =
          std::min((hypothesis.shift + target.angular_bins - existing.shift) % target.angular_bins,
                   (existing.shift + target.angular_bins - hypothesis.shift) % target.angular_bins);
      if (distance <= 1) {
        separated = false;
        break;
      }
    }
    if (separated) {
      selected.push_back(hypothesis);
      if (selected.size() >= limit) {
        break;
      }
    }
  }
  return selected;
}

std::vector<double> cumulative_path(const std::vector<Keyframe> &keyframes) {
  std::vector<double> result(keyframes.size(), 0.0);
  for (std::size_t index = 1; index < keyframes.size(); ++index) {
    result[index] =
        result[index - 1] + pose_xy_distance(keyframes[index - 1].pose, keyframes[index].pose);
  }
  return result;
}

std::vector<Candidate>
generate_candidates(const std::vector<Keyframe> &keyframes, const LoopConstraintOptions &options,
                    const std::function<const Descriptor &(std::size_t)> &descriptor_at) {
  std::vector<Candidate> candidates;
  if (keyframes.size() <= options.min_index_separation) {
    return candidates;
  }
  const auto path = cumulative_path(keyframes);
  const double cell_size = std::max(0.1, options.candidate_xy_radius_m);
  std::unordered_map<VoxelKey, std::vector<std::size_t>, VoxelKeyHash> spatial;
  std::size_t next_history = 0;

  for (std::size_t to_index = 0; to_index < keyframes.size(); ++to_index) {
    while (next_history + options.min_index_separation <= to_index) {
      const Pose &pose = keyframes[next_history].pose;
      spatial[voxel_key(pose.x, pose.y, 0.0, cell_size)].push_back(next_history);
      ++next_history;
    }
    if (spatial.empty()) {
      continue;
    }

    const Pose &current_pose = keyframes[to_index].pose;
    const VoxelKey center = voxel_key(current_pose.x, current_pose.y, 0.0, cell_size);
    std::vector<Candidate> current;
    std::vector<const std::vector<std::size_t> *> history_buckets;
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        const auto found = spatial.find(VoxelKey{center.x + dx, center.y + dy, 0});
        if (found != spatial.end() && !found->second.empty()) {
          history_buckets.push_back(&found->second);
        }
      }
    }
    std::size_t remaining_budget = options.max_spatial_candidates_per_anchor;
    for (std::size_t bucket_index = 0;
         bucket_index < history_buckets.size() && remaining_budget > 0; ++bucket_index) {
      const auto &bucket = *history_buckets[bucket_index];
      const std::size_t buckets_left = history_buckets.size() - bucket_index;
      const std::size_t bucket_budget =
          std::min(bucket.size(), (remaining_budget + buckets_left - 1) / buckets_left);
      for (std::size_t sample = 0; sample < bucket_budget; ++sample) {
        // Uniform deterministic sampling keeps old and recent revisits visible
        // without enumerating an unbounded dense history bucket.
        const std::size_t sampled_offset =
            bucket_budget == bucket.size()
                ? sample
                : ((2 * sample + 1) * bucket.size()) / (2 * bucket_budget);
        const std::size_t from_index = bucket[std::min(sampled_offset, bucket.size() - 1)];
        if (submaps_overlap(from_index, to_index, options) ||
            path[to_index] - path[from_index] < options.min_path_separation_m ||
            pose_xy_distance(keyframes[from_index].pose, current_pose) >
                options.candidate_xy_radius_m ||
            std::abs(keyframes[from_index].pose.z - current_pose.z) >
                options.candidate_max_abs_z_m) {
          continue;
        }
        double yaw_margin = 0.0;
        auto hypotheses =
            descriptor_yaw_hypotheses(descriptor_at(from_index), descriptor_at(to_index),
                                      options.yaw_hypotheses, &yaw_margin);
        if (hypotheses.empty() || hypotheses.front().score < options.descriptor_min_similarity ||
            yaw_margin < options.descriptor_min_yaw_margin) {
          continue;
        }
        current.push_back(Candidate{
            from_index,
            to_index,
            hypotheses.front().score,
            0.0,
            yaw_margin,
            std::move(hypotheses),
        });
      }
      remaining_budget -= bucket_budget;
    }
    std::sort(current.begin(), current.end(), [](const Candidate &lhs, const Candidate &rhs) {
      if (lhs.descriptor_score != rhs.descriptor_score) {
        return lhs.descriptor_score > rhs.descriptor_score;
      }
      return lhs.from_index < rhs.from_index;
    });
    if (current.empty()) {
      continue;
    }
    const std::size_t best_from_index = current.front().from_index;
    const std::size_t place_exclusion =
        std::max(2 * options.submap_half_window, options.consensus_anchor_tolerance);
    double second_place_score = 0.0;
    bool has_second_place = false;
    for (std::size_t index = 1; index < current.size(); ++index) {
      const std::size_t gap = current[index].from_index > best_from_index
                                  ? current[index].from_index - best_from_index
                                  : best_from_index - current[index].from_index;
      if (gap > place_exclusion) {
        second_place_score = current[index].descriptor_score;
        has_second_place = true;
        break;
      }
    }
    const double place_margin =
        current.front().descriptor_score - (has_second_place ? second_place_score : 0.0);
    if (place_margin < options.descriptor_min_margin) {
      continue;
    }

    std::size_t kept = 0;
    for (Candidate &candidate : current) {
      const std::size_t gap = candidate.from_index > best_from_index
                                  ? candidate.from_index - best_from_index
                                  : best_from_index - candidate.from_index;
      if (gap > place_exclusion) {
        continue;
      }
      candidate.descriptor_margin = place_margin;
      candidates.push_back(std::move(candidate));
      ++kept;
      if (kept >= options.max_candidates_per_anchor) {
        break;
      }
    }
  }
  std::sort(candidates.begin(), candidates.end(), [](const Candidate &lhs, const Candidate &rhs) {
    if (lhs.descriptor_score != rhs.descriptor_score) {
      return lhs.descriptor_score > rhs.descriptor_score;
    }
    if (lhs.descriptor_margin != rhs.descriptor_margin) {
      return lhs.descriptor_margin > rhs.descriptor_margin;
    }
    if (lhs.to_index != rhs.to_index) {
      return lhs.to_index < rhs.to_index;
    }
    return lhs.from_index < rhs.from_index;
  });
  if (candidates.size() > options.max_total_candidates) {
    candidates.resize(options.max_total_candidates);
  }
  std::sort(candidates.begin(), candidates.end(), [](const Candidate &lhs, const Candidate &rhs) {
    if (lhs.to_index != rhs.to_index) {
      return lhs.to_index < rhs.to_index;
    }
    return lhs.from_index < rhs.from_index;
  });
  return candidates;
}

using VoxelIndex = std::unordered_map<VoxelKey, std::vector<std::size_t>, VoxelKeyHash>;

VoxelIndex build_voxel_index(const std::vector<Point> &cloud, double cell_size) {
  VoxelIndex index;
  index.reserve(cloud.size());
  for (std::size_t point_index = 0; point_index < cloud.size(); ++point_index) {
    const Point &point = cloud[point_index];
    index[voxel_key(point.x, point.y, point.z, cell_size)].push_back(point_index);
  }
  return index;
}

std::vector<Correspondence> find_correspondences(const std::vector<Point> &source,
                                                 const std::vector<Point> &target,
                                                 const VoxelIndex &target_index,
                                                 const Pose &source_to_target,
                                                 double maximum_distance) {
  std::vector<Correspondence> correspondences;
  correspondences.reserve(source.size());
  const double maximum_distance_sq = maximum_distance * maximum_distance;
  for (const Point &point : source) {
    const Point transformed = transform_point(point, source_to_target);
    const VoxelKey center =
        voxel_key(transformed.x, transformed.y, transformed.z, maximum_distance);
    double best_distance_sq = maximum_distance_sq;
    const Point *best = nullptr;
    std::size_t best_index = 0;
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          const auto found =
              target_index.find(VoxelKey{center.x + dx, center.y + dy, center.z + dz});
          if (found == target_index.end()) {
            continue;
          }
          for (std::size_t target_point_index : found->second) {
            const Point &candidate = target[target_point_index];
            const double distance_sq = square(static_cast<double>(candidate.x) - transformed.x) +
                                       square(static_cast<double>(candidate.y) - transformed.y) +
                                       square(static_cast<double>(candidate.z) - transformed.z);
            if (distance_sq < best_distance_sq) {
              best_distance_sq = distance_sq;
              best = &candidate;
              best_index = target_point_index;
            }
          }
        }
      }
    }
    if (best != nullptr) {
      correspondences.push_back(Correspondence{
          {transformed.x, transformed.y, transformed.z},
          {best->x, best->y, best->z},
          best_index,
          std::sqrt(best_distance_sq),
      });
    }
  }
  std::sort(correspondences.begin(), correspondences.end(),
            [](const auto &lhs, const auto &rhs) { return lhs.distance < rhs.distance; });
  return correspondences;
}

void trim_correspondences(std::vector<Correspondence> &correspondences,
                          const LoopConstraintOptions &options) {
  const double fraction = std::clamp(options.trim_fraction, 0.1, 1.0);
  const std::size_t keep = std::max(
      std::min(options.min_inliers, correspondences.size()),
      static_cast<std::size_t>(std::floor(fraction * static_cast<double>(correspondences.size()))));
  if (correspondences.size() > keep) {
    correspondences.resize(keep);
  }
}

template <std::size_t Size>
struct SymmetricEigenResult {
  std::array<double, Size> values{};
  // Eigenvectors are stored as columns and sorted with the eigenvalues.
  std::array<std::array<double, Size>, Size> vectors{};
};

template <std::size_t Size>
SymmetricEigenResult<Size>
symmetric_eigen_decomposition(std::array<std::array<double, Size>, Size> matrix) {
  SymmetricEigenResult<Size> result;
  for (std::size_t index = 0; index < Size; ++index) {
    result.vectors[index][index] = 1.0;
  }
  for (int sweep = 0; sweep < 48; ++sweep) {
    std::size_t p = 0;
    std::size_t q = 1;
    double largest = 0.0;
    for (std::size_t row = 0; row < Size; ++row) {
      for (std::size_t col = row + 1; col < Size; ++col) {
        if (std::abs(matrix[row][col]) > largest) {
          largest = std::abs(matrix[row][col]);
          p = row;
          q = col;
        }
      }
    }
    if (largest < 1.0e-10) {
      break;
    }
    const double angle = 0.5 * std::atan2(2.0 * matrix[p][q], matrix[q][q] - matrix[p][p]);
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    for (std::size_t index = 0; index < Size; ++index) {
      if (index == p || index == q) {
        continue;
      }
      const double mip = matrix[index][p];
      const double miq = matrix[index][q];
      matrix[index][p] = matrix[p][index] = cosine * mip - sine * miq;
      matrix[index][q] = matrix[q][index] = sine * mip + cosine * miq;
    }
    const double mpp = matrix[p][p];
    const double mqq = matrix[q][q];
    const double mpq = matrix[p][q];
    matrix[p][p] = cosine * cosine * mpp - 2.0 * sine * cosine * mpq + sine * sine * mqq;
    matrix[q][q] = sine * sine * mpp + 2.0 * sine * cosine * mpq + cosine * cosine * mqq;
    matrix[p][q] = matrix[q][p] = 0.0;
    for (std::size_t row = 0; row < Size; ++row) {
      const double vip = result.vectors[row][p];
      const double viq = result.vectors[row][q];
      result.vectors[row][p] = cosine * vip - sine * viq;
      result.vectors[row][q] = sine * vip + cosine * viq;
    }
  }
  std::array<std::size_t, Size> order{};
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(),
            [&](std::size_t lhs, std::size_t rhs) { return matrix[lhs][lhs] < matrix[rhs][rhs]; });
  const auto unsorted_vectors = result.vectors;
  for (std::size_t sorted = 0; sorted < Size; ++sorted) {
    result.values[sorted] = matrix[order[sorted]][order[sorted]];
    for (std::size_t row = 0; row < Size; ++row) {
      result.vectors[row][sorted] = unsorted_vectors[row][order[sorted]];
    }
  }
  return result;
}

struct NormalEstimate {
  std::array<double, 3> unit{};
  bool valid = false;
};

struct PreparedCloud {
  const std::vector<Point> *points = nullptr;
  VoxelIndex match_index;
  std::vector<NormalEstimate> normals;
};

PreparedCloud prepare_cloud(const std::vector<Point> &cloud, const LoopConstraintOptions &options) {
  PreparedCloud prepared;
  prepared.points = &cloud;
  prepared.match_index = build_voxel_index(cloud, options.max_correspondence_distance_m);
  prepared.normals.resize(cloud.size());
  const VoxelIndex normal_index = build_voxel_index(cloud, options.normal_radius_m);
  const double radius_squared = square(options.normal_radius_m);

  for (std::size_t point_index = 0; point_index < cloud.size(); ++point_index) {
    const Point &center_point = cloud[point_index];
    const VoxelKey center =
        voxel_key(center_point.x, center_point.y, center_point.z, options.normal_radius_m);
    std::vector<std::size_t> neighbors;
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          const auto found =
              normal_index.find(VoxelKey{center.x + dx, center.y + dy, center.z + dz});
          if (found == normal_index.end()) {
            continue;
          }
          for (std::size_t neighbor_index : found->second) {
            const Point &neighbor = cloud[neighbor_index];
            const double distance_squared =
                square(static_cast<double>(neighbor.x) - center_point.x) +
                square(static_cast<double>(neighbor.y) - center_point.y) +
                square(static_cast<double>(neighbor.z) - center_point.z);
            if (distance_squared <= radius_squared) {
              neighbors.push_back(neighbor_index);
            }
          }
        }
      }
    }
    if (neighbors.size() < options.normal_min_neighbors) {
      continue;
    }

    std::array<double, 3> centroid{};
    for (std::size_t neighbor_index : neighbors) {
      const Point &neighbor = cloud[neighbor_index];
      centroid[0] += neighbor.x;
      centroid[1] += neighbor.y;
      centroid[2] += neighbor.z;
    }
    const double inverse_count = 1.0 / static_cast<double>(neighbors.size());
    for (double &value : centroid) {
      value *= inverse_count;
    }
    std::array<std::array<double, 3>, 3> covariance{};
    for (std::size_t neighbor_index : neighbors) {
      const Point &neighbor = cloud[neighbor_index];
      const std::array<double, 3> delta{
          static_cast<double>(neighbor.x) - centroid[0],
          static_cast<double>(neighbor.y) - centroid[1],
          static_cast<double>(neighbor.z) - centroid[2],
      };
      for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t col = row; col < 3; ++col) {
          covariance[row][col] += delta[row] * delta[col] * inverse_count;
          covariance[col][row] = covariance[row][col];
        }
      }
    }
    const auto eigen = symmetric_eigen_decomposition<3>(covariance);
    const double middle = eigen.values[1];
    const double largest = eigen.values[2];
    if (middle < options.normal_min_middle_variance_m2 || largest <= 1.0e-12 ||
        eigen.values[0] / middle > options.normal_max_small_to_middle_ratio ||
        middle / largest < options.normal_min_middle_to_large_ratio) {
      continue;
    }
    NormalEstimate &normal = prepared.normals[point_index];
    normal.unit = {
        eigen.vectors[0][0],
        eigen.vectors[1][0],
        eigen.vectors[2][0],
    };
    normal.valid = true;
  }
  return prepared;
}

RegistrationResult register_4dof(const PreparedCloud &prepared_source,
                                 const PreparedCloud &prepared_target, const Pose &initial,
                                 const LoopConstraintOptions &options) {
  const std::vector<Point> &source = *prepared_source.points;
  const std::vector<Point> &target = *prepared_target.points;
  RegistrationResult result;
  result.transform = initial;
  result.source_points = source.size();
  result.target_points = target.size();
  if (source.size() < options.min_inliers || target.size() < options.min_inliers) {
    result.reason = "insufficient_points";
    return result;
  }

  const VoxelIndex &target_index = prepared_target.match_index;
  const auto seed_rpy = pose_rpy(initial);
  Pose current = initial;
  for (std::size_t iteration = 0; iteration < options.max_icp_iterations; ++iteration) {
    auto correspondences = find_correspondences(source, target, target_index, current,
                                                options.max_correspondence_distance_m);
    if (correspondences.size() < options.min_inliers) {
      result.reason = "insufficient_correspondences";
      return result;
    }
    trim_correspondences(correspondences, options);
    if (correspondences.size() < options.min_inliers) {
      result.reason = "insufficient_trimmed_correspondences";
      return result;
    }

    std::array<double, 3> source_centroid{};
    std::array<double, 3> target_centroid{};
    for (const Correspondence &correspondence : correspondences) {
      for (int axis = 0; axis < 3; ++axis) {
        source_centroid[axis] += correspondence.source_transformed[axis];
        target_centroid[axis] += correspondence.target[axis];
      }
    }
    const double inverse_count = 1.0 / static_cast<double>(correspondences.size());
    for (int axis = 0; axis < 3; ++axis) {
      source_centroid[axis] *= inverse_count;
      target_centroid[axis] *= inverse_count;
    }

    double cross = 0.0;
    double dot = 0.0;
    for (const Correspondence &correspondence : correspondences) {
      const double sx = correspondence.source_transformed[0] - source_centroid[0];
      const double sy = correspondence.source_transformed[1] - source_centroid[1];
      const double tx = correspondence.target[0] - target_centroid[0];
      const double ty = correspondence.target[1] - target_centroid[1];
      cross += sx * ty - sy * tx;
      dot += sx * tx + sy * ty;
    }
    const double delta_yaw = std::atan2(cross, dot);
    const double cosine = std::cos(delta_yaw);
    const double sine = std::sin(delta_yaw);
    Pose delta;
    delta = pose_with_rpy(delta, 0.0, 0.0, delta_yaw);
    delta.x = target_centroid[0] - (cosine * source_centroid[0] - sine * source_centroid[1]);
    delta.y = target_centroid[1] - (sine * source_centroid[0] + cosine * source_centroid[1]);
    delta.z = target_centroid[2] - source_centroid[2];

    Pose updated = compose_pose(delta, current);
    const double updated_yaw = pose_rpy(updated)[2];
    updated = pose_with_rpy(updated, seed_rpy[0], seed_rpy[1], updated_yaw);
    current = updated;
    result.iterations = iteration + 1;
    if (std::hypot(delta.x, delta.y) < 1.0e-4 && std::abs(delta.z) < 1.0e-4 &&
        std::abs(delta_yaw) < 1.0e-5) {
      break;
    }
  }

  auto all_correspondences = find_correspondences(source, target, target_index, current,
                                                  options.max_correspondence_distance_m);
  const std::size_t untrimmed_inliers = all_correspondences.size();
  result.inlier_ratio =
      source.empty() ? 0.0
                     : static_cast<double>(untrimmed_inliers) / static_cast<double>(source.size());
  trim_correspondences(all_correspondences, options);
  result.inliers = all_correspondences.size();
  if (all_correspondences.size() < options.min_inliers) {
    result.reason = "insufficient_final_inliers";
    return result;
  }

  double squared_error = 0.0;
  std::vector<double> residuals;
  residuals.reserve(all_correspondences.size());
  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  double min_z = std::numeric_limits<double>::infinity();
  double max_z = -std::numeric_limits<double>::infinity();
  std::array<std::array<double, 4>, 4> hessian{};
  for (const Correspondence &correspondence : all_correspondences) {
    squared_error += correspondence.distance * correspondence.distance;
    residuals.push_back(correspondence.distance);
    const double x = correspondence.source_transformed[0];
    const double y = correspondence.source_transformed[1];
    min_x = std::min(min_x, correspondence.target[0]);
    max_x = std::max(max_x, correspondence.target[0]);
    min_y = std::min(min_y, correspondence.target[1]);
    max_y = std::max(max_y, correspondence.target[1]);
    min_z = std::min(min_z, correspondence.target[2]);
    max_z = std::max(max_z, correspondence.target[2]);
    const std::array<std::array<double, 4>, 3> jacobian{{
        {{-y, 1.0, 0.0, 0.0}},
        {{x, 0.0, 1.0, 0.0}},
        {{0.0, 0.0, 0.0, 1.0}},
    }};
    for (int row = 0; row < 4; ++row) {
      for (int col = row; col < 4; ++col) {
        for (int residual_axis = 0; residual_axis < 3; ++residual_axis) {
          hessian[row][col] += jacobian[residual_axis][row] * jacobian[residual_axis][col];
        }
        hessian[col][row] = hessian[row][col];
      }
    }
  }
  std::sort(residuals.begin(), residuals.end());
  auto percentile = [&](double fraction) {
    if (residuals.empty()) {
      return -1.0;
    }
    const std::size_t index = std::min(
        residuals.size() - 1,
        static_cast<std::size_t>(std::floor(fraction * static_cast<double>(residuals.size() - 1))));
    return residuals[index];
  };
  result.rmse_m = std::sqrt(squared_error / static_cast<double>(residuals.size()));
  result.p50_m = percentile(0.50);
  result.p95_m = percentile(0.95);
  result.xy_span_m = std::hypot(max_x - min_x, max_y - min_y);
  result.z_span_m = max_z - min_z;
  for (int axis = 0; axis < 4; ++axis) {
    result.hessian_diagonal[static_cast<std::size_t>(axis)] = hessian[axis][axis];
  }
  const auto point_eigen = symmetric_eigen_decomposition<4>(hessian);
  const double minimum = point_eigen.values.front();
  const double maximum = point_eigen.values.back();
  result.hessian_condition =
      minimum > 1.0e-9 ? maximum / minimum : std::numeric_limits<double>::infinity();

  // Fixed-correspondence point-to-point Hessians make every translation
  // direction appear observable. Build a target-normal point-to-plane Fisher
  // matrix instead, with each target point contributing at most once.
  std::vector<const Correspondence *> planar_correspondences;
  planar_correspondences.reserve(all_correspondences.size());
  std::vector<unsigned char> used_target(target.size(), 0);
  std::array<double, 2> planar_centroid{};
  for (const Correspondence &correspondence : all_correspondences) {
    if (correspondence.target_index >= prepared_target.normals.size() ||
        used_target[correspondence.target_index] != 0 ||
        !prepared_target.normals[correspondence.target_index].valid) {
      continue;
    }
    used_target[correspondence.target_index] = 1;
    planar_correspondences.push_back(&correspondence);
    planar_centroid[0] += correspondence.source_transformed[0];
    planar_centroid[1] += correspondence.source_transformed[1];
  }
  result.planar_inliers = planar_correspondences.size();
  result.planar_inlier_ratio = all_correspondences.empty()
                                   ? 0.0
                                   : static_cast<double>(result.planar_inliers) /
                                         static_cast<double>(all_correspondences.size());
  if (!planar_correspondences.empty()) {
    const double inverse_planar_count = 1.0 / static_cast<double>(planar_correspondences.size());
    planar_centroid[0] *= inverse_planar_count;
    planar_centroid[1] *= inverse_planar_count;
    double lever_squared = 0.0;
    for (const Correspondence *correspondence : planar_correspondences) {
      lever_squared += square(correspondence->source_transformed[0] - planar_centroid[0]) +
                       square(correspondence->source_transformed[1] - planar_centroid[1]);
    }
    const double lever = std::max(0.5, std::sqrt(lever_squared * inverse_planar_count));
    std::array<std::array<double, 4>, 4> plane_hessian{};
    for (const Correspondence *correspondence : planar_correspondences) {
      const auto &normal = prepared_target.normals[correspondence->target_index].unit;
      const double centered_x = correspondence->source_transformed[0] - planar_centroid[0];
      const double centered_y = correspondence->source_transformed[1] - planar_centroid[1];
      const std::array<double, 4> jacobian{
          (-normal[0] * centered_y + normal[1] * centered_x) / lever,
          normal[0],
          normal[1],
          normal[2],
      };
      for (std::size_t row = 0; row < 4; ++row) {
        for (std::size_t col = row; col < 4; ++col) {
          plane_hessian[row][col] += jacobian[row] * jacobian[col] * inverse_planar_count;
          plane_hessian[col][row] = plane_hessian[row][col];
        }
      }
    }
    const auto plane_eigen = symmetric_eigen_decomposition<4>(plane_hessian);
    result.point_to_plane_eigenvalues = plane_eigen.values;
    for (std::size_t axis = 0; axis < 4; ++axis) {
      result.point_to_plane_weak_mode[axis] = plane_eigen.vectors[axis][0];
    }
    const double plane_minimum = plane_eigen.values.front();
    const double plane_maximum = plane_eigen.values.back();
    result.point_to_plane_condition = plane_minimum > 1.0e-12
                                          ? plane_maximum / plane_minimum
                                          : std::numeric_limits<double>::infinity();
  } else {
    result.point_to_plane_condition = std::numeric_limits<double>::infinity();
  }
  result.transform = current;
  result.ok = true;
  result.reason = "verified";
  return result;
}

bool registration_passes(const RegistrationResult &registration,
                         const LoopConstraintOptions &options, std::string *reason) {
  auto reject = [&](const std::string &value) {
    if (reason != nullptr) {
      *reason = value;
    }
    return false;
  };
  if (!registration.ok) {
    return reject(registration.reason);
  }
  if (registration.inliers < options.min_inliers ||
      registration.inlier_ratio < options.min_inlier_ratio) {
    return reject("overlap_gate");
  }
  if (!std::isfinite(registration.rmse_m) || registration.rmse_m > options.max_rmse_m ||
      !std::isfinite(registration.p95_m) || registration.p95_m > options.max_p95_m) {
    return reject("residual_gate");
  }
  if (registration.xy_span_m < options.min_xy_span_m ||
      registration.z_span_m < options.min_z_span_m) {
    return reject("geometry_span_gate");
  }
  if (registration.planar_inliers < options.min_planar_inliers ||
      registration.planar_inlier_ratio < options.min_planar_inlier_ratio) {
    return reject("insufficient_planar_correspondences");
  }
  if (!std::isfinite(registration.point_to_plane_eigenvalues.front()) ||
      registration.point_to_plane_eigenvalues.front() < options.min_point_to_plane_eigenvalue) {
    return reject("point_to_plane_rank_gate");
  }
  if (!std::isfinite(registration.point_to_plane_condition) ||
      registration.point_to_plane_condition > options.max_point_to_plane_condition) {
    return reject("point_to_plane_condition_gate");
  }
  if (!std::isfinite(registration.hessian_condition) ||
      registration.hessian_condition > options.max_hessian_condition) {
    return reject("degeneracy_gate");
  }
  return true;
}

bool registration_is_better(const RegistrationResult &candidate,
                            const RegistrationResult &incumbent) {
  // A low-overlap alignment can have an attractive RMSE because it explains
  // only a repeated fragment. Rank valid hypotheses by coverage first, then
  // by the tail and aggregate residuals.
  if (candidate.inlier_ratio != incumbent.inlier_ratio) {
    return candidate.inlier_ratio > incumbent.inlier_ratio;
  }
  if (candidate.inliers != incumbent.inliers) {
    return candidate.inliers > incumbent.inliers;
  }
  if (candidate.p95_m != incumbent.p95_m) {
    return candidate.p95_m < incumbent.p95_m;
  }
  if (candidate.rmse_m != incumbent.rmse_m) {
    return candidate.rmse_m < incumbent.rmse_m;
  }
  if (candidate.point_to_plane_eigenvalues.front() !=
      incumbent.point_to_plane_eigenvalues.front()) {
    return candidate.point_to_plane_eigenvalues.front() >
           incumbent.point_to_plane_eigenvalues.front();
  }
  if (candidate.point_to_plane_condition != incumbent.point_to_plane_condition) {
    return candidate.point_to_plane_condition < incumbent.point_to_plane_condition;
  }
  if (candidate.hessian_condition != incumbent.hessian_condition) {
    return candidate.hessian_condition < incumbent.hessian_condition;
  }
  return candidate.iterations < incumbent.iterations;
}

void copy_registration_diagnostics(const RegistrationResult &registration,
                                   LoopCandidateDiagnostic &diagnostic) {
  diagnostic.source_points = registration.source_points;
  diagnostic.target_points = registration.target_points;
  diagnostic.inliers = registration.inliers;
  diagnostic.inlier_ratio = registration.inlier_ratio;
  diagnostic.rmse_m = registration.rmse_m;
  diagnostic.p50_m = registration.p50_m;
  diagnostic.p95_m = registration.p95_m;
  diagnostic.hessian_condition = registration.hessian_condition;
  diagnostic.planar_inliers = registration.planar_inliers;
  diagnostic.planar_inlier_ratio = registration.planar_inlier_ratio;
  diagnostic.point_to_plane_eigenvalues = registration.point_to_plane_eigenvalues;
  diagnostic.point_to_plane_weak_mode = registration.point_to_plane_weak_mode;
  diagnostic.point_to_plane_condition = registration.point_to_plane_condition;
  diagnostic.iterations = registration.iterations;
}

std::optional<VerifiedCandidate>
verify_candidate(const Candidate &candidate, const std::vector<Keyframe> &keyframes,
                 const std::vector<Point> &target, const std::vector<Point> &source,
                 const LoopConstraintOptions &options, LoopCandidateDiagnostic &diagnostic) {
  diagnostic.from_index = candidate.from_index;
  diagnostic.to_index = candidate.to_index;
  diagnostic.descriptor_score = candidate.descriptor_score;
  diagnostic.descriptor_margin = candidate.descriptor_margin;
  diagnostic.descriptor_yaw_margin = candidate.descriptor_yaw_margin;
  const Pose &map_body_from = keyframes[candidate.from_index].pose;
  const Pose &map_body_to = keyframes[candidate.to_index].pose;
  const Pose gravity_seed =
      between_poses(map_gravity_pose(map_body_from), map_gravity_pose(map_body_to));
  const double seed_gravity_yaw = pose_rpy(gravity_seed)[2];
  const PreparedCloud prepared_target = prepare_cloud(target, options);
  const PreparedCloud prepared_source = prepare_cloud(source, options);

  std::vector<YawHypothesis> hypotheses = candidate.yaw_hypotheses;
  hypotheses.push_back(YawHypothesis{candidate.descriptor_score, seed_gravity_yaw, 0});
  std::sort(hypotheses.begin(), hypotheses.end(), [](const auto &lhs, const auto &rhs) {
    if (lhs.score != rhs.score) {
      return lhs.score > rhs.score;
    }
    return lhs.yaw_rad < rhs.yaw_rad;
  });

  RegistrationResult best;
  RegistrationResult best_rejected;
  bool has_best = false;
  bool has_rejected = false;
  double best_yaw = seed_gravity_yaw;
  double best_rejected_yaw = seed_gravity_yaw;
  std::string best_rejected_reason;
  std::set<int> used_yaw_bins;
  for (const YawHypothesis &hypothesis : hypotheses) {
    const int yaw_key = static_cast<int>(std::llround(hypothesis.yaw_rad * 1000.0));
    if (!used_yaw_bins.insert(yaw_key).second) {
      continue;
    }
    Pose initial = pose_with_rpy(gravity_seed, 0.0, 0.0, hypothesis.yaw_rad);
    const RegistrationResult registration =
        register_4dof(prepared_source, prepared_target, initial, options);
    if (!registration.ok) {
      continue;
    }
    std::string rejection_reason;
    if (!registration_passes(registration, options, &rejection_reason)) {
      if (!has_rejected || registration_is_better(registration, best_rejected)) {
        best_rejected = registration;
        best_rejected_yaw = hypothesis.yaw_rad;
        best_rejected_reason = rejection_reason;
        has_rejected = true;
      }
      continue;
    }
    if (!has_best || registration_is_better(registration, best)) {
      best = registration;
      best_yaw = hypothesis.yaw_rad;
      has_best = true;
    }
  }
  if (!has_best) {
    if (has_rejected) {
      diagnostic.yaw_hypothesis_rad = best_rejected_yaw;
      copy_registration_diagnostics(best_rejected, diagnostic);
      diagnostic.reason = best_rejected_reason;
    } else {
      diagnostic.reason = "registration_failed";
    }
    return std::nullopt;
  }
  diagnostic.yaw_hypothesis_rad = best_yaw;
  copy_registration_diagnostics(best, diagnostic);
  const Pose body_measurement =
      gravity_measurement_to_body(best.transform, map_body_from, map_body_to);
  const Pose corrected_to_pose = compose_pose(map_body_from, body_measurement);

  // Apply correction limits in the map/gravity frame. A tilted body frame
  // mixes horizontal and vertical components even though full 3D norm is
  // invariant under T_map_from.
  const double correction_xy =
      std::hypot(corrected_to_pose.x - map_body_to.x, corrected_to_pose.y - map_body_to.y);
  const double correction_z = std::abs(corrected_to_pose.z - map_body_to.z);
  const double correction_yaw = pose_yaw_difference(corrected_to_pose, map_body_to);
  diagnostic.correction_xy_m = correction_xy;
  diagnostic.correction_z_m = correction_z;
  diagnostic.correction_yaw_rad = correction_yaw;
  if (correction_xy > options.max_correction_xy_m || correction_z > options.max_correction_z_m ||
      correction_yaw > options.max_correction_yaw_rad) {
    diagnostic.reason = "correction_gate";
    return std::nullopt;
  }

  RegistrationResult reverse;
  RegistrationResult reverse_rejected;
  bool has_reverse = false;
  bool has_reverse_rejected = false;
  std::string reverse_rejected_reason;
  std::set<int> used_reverse_yaw_bins;
  for (const YawHypothesis &hypothesis : hypotheses) {
    const Pose forward_initial = pose_with_rpy(gravity_seed, 0.0, 0.0, hypothesis.yaw_rad);
    const Pose reverse_initial = inverse_pose(forward_initial);
    const double reverse_yaw = pose_rpy(reverse_initial)[2];
    const int yaw_key = static_cast<int>(std::llround(reverse_yaw * 1000.0));
    if (!used_reverse_yaw_bins.insert(yaw_key).second) {
      continue;
    }
    const RegistrationResult registration =
        register_4dof(prepared_target, prepared_source, reverse_initial, options);
    if (!registration.ok) {
      continue;
    }
    std::string rejection_reason;
    if (!registration_passes(registration, options, &rejection_reason)) {
      if (!has_reverse_rejected || registration_is_better(registration, reverse_rejected)) {
        reverse_rejected = registration;
        reverse_rejected_reason = rejection_reason;
        has_reverse_rejected = true;
      }
      continue;
    }
    if (!has_reverse || registration_is_better(registration, reverse)) {
      reverse = registration;
      has_reverse = true;
    }
  }
  if (!has_reverse) {
    diagnostic.reason =
        has_reverse_rejected ? "reverse_" + reverse_rejected_reason : "reverse_registration_failed";
    return std::nullopt;
  }
  const Pose round_trip = compose_pose(best.transform, reverse.transform);
  const Pose identity;
  diagnostic.inverse_translation_error_m = pose_translation_distance(round_trip, identity);
  diagnostic.inverse_yaw_error_rad = pose_yaw_difference(round_trip, identity);
  if (diagnostic.inverse_translation_error_m > options.max_inverse_translation_m ||
      diagnostic.inverse_yaw_error_rad > options.max_inverse_yaw_rad) {
    diagnostic.reason = "inverse_consistency_gate";
    return std::nullopt;
  }

  GeometricConstraint constraint;
  constraint.from_index = candidate.from_index;
  constraint.to_index = candidate.to_index;
  // ICP solves T_Gfrom_Gto in gravity-aligned frames. Convert that result to
  // the graph's body-frame T_from_to measurement before publishing it.
  constraint.pose_from_to = body_measurement;
  // This milestone is deliberately shadow-only. The 4DoF observability
  // diagnostic is expressed in a gravity-frame left tangent, while the graph
  // residual uses a body-frame right tangent. Keep the graph information zero
  // so Graph::optimize rejects accidental consumption until a full 6x6
  // adjoint/Jacobian conversion and non-diagonal ABI are implemented.
  constraint.information_diagonal = {};
  VerifiedCandidate verified;
  verified.constraint = constraint;
  // Express every loop correction in the shared map frame so neighboring
  // candidate pairs can be compared without mixing local anchor frames.
  verified.correction = compose_pose(corrected_to_pose, inverse_pose(map_body_to));
  return verified;
}

bool correction_consistent(const VerifiedCandidate &lhs, const VerifiedCandidate &rhs,
                           const LoopConstraintOptions &options) {
  const auto lhs_rpy = pose_rpy(lhs.correction);
  const auto rhs_rpy = pose_rpy(rhs.correction);
  return pose_translation_distance(lhs.correction, rhs.correction) <=
             options.consensus_translation_m &&
         std::abs(wrap_angle(lhs_rpy[2] - rhs_rpy[2])) <= options.consensus_yaw_rad;
}

void apply_consensus(std::vector<VerifiedCandidate> &verified, LoopConstraintResult &result,
                     const LoopConstraintOptions &options) {
  if (verified.empty()) {
    return;
  }
  std::vector<bool> assigned(verified.size(), false);
  for (std::size_t index = 0; index < verified.size(); ++index) {
    if (assigned[index]) {
      continue;
    }
    std::vector<std::size_t> cluster{index};
    for (std::size_t other = index + 1; other < verified.size(); ++other) {
      if (assigned[other]) {
        continue;
      }
      const auto &lhs = verified[index].constraint;
      const auto &rhs = verified[other].constraint;
      const std::size_t from_gap = lhs.from_index > rhs.from_index
                                       ? lhs.from_index - rhs.from_index
                                       : rhs.from_index - lhs.from_index;
      const std::size_t to_gap =
          lhs.to_index > rhs.to_index ? lhs.to_index - rhs.to_index : rhs.to_index - lhs.to_index;
      const std::int64_t from_step =
          static_cast<std::int64_t>(rhs.from_index) - static_cast<std::int64_t>(lhs.from_index);
      const std::int64_t to_step =
          static_cast<std::int64_t>(rhs.to_index) - static_cast<std::int64_t>(lhs.to_index);
      const std::size_t step_gap = from_gap > to_gap ? from_gap - to_gap : to_gap - from_gap;
      if (from_gap > 0 && to_gap > 0 &&
          ((from_step > 0 && to_step > 0) || (from_step < 0 && to_step < 0)) &&
          from_gap <= options.consensus_anchor_tolerance &&
          to_gap <= options.consensus_anchor_tolerance &&
          step_gap <= options.consensus_step_tolerance &&
          correction_consistent(verified[index], verified[other], options)) {
        cluster.push_back(other);
      }
    }
    const bool accepted =
        !options.require_consensus || cluster.size() >= options.min_consistent_matches;
    if (!accepted) {
      for (std::size_t member : cluster) {
        assigned[member] = true;
        auto &diagnostic = result.report.candidates[verified[member].diagnostic_index];
        diagnostic.accepted = false;
        diagnostic.reason = "consensus_gate";
      }
      continue;
    }

    std::size_t best_member = cluster.front();
    for (std::size_t member : cluster) {
      assigned[member] = true;
      const auto &candidate = result.report.candidates[verified[member].diagnostic_index];
      const auto &best = result.report.candidates[verified[best_member].diagnostic_index];
      if (candidate.rmse_m < best.rmse_m) {
        best_member = member;
      }
    }
    auto &best_diagnostic = result.report.candidates[verified[best_member].diagnostic_index];
    best_diagnostic.accepted = true;
    best_diagnostic.reason = "accepted";
    result.constraints.push_back(verified[best_member].constraint);
    for (std::size_t member : cluster) {
      if (member == best_member) {
        continue;
      }
      auto &support = result.report.candidates[verified[member].diagnostic_index];
      support.accepted = false;
      support.reason = "consensus_support";
    }
  }
}

std::uint64_t hash_bytes(std::uint64_t hash, const char *data, std::size_t size) {
  for (std::size_t index = 0; index < size; ++index) {
    hash ^= static_cast<unsigned char>(data[index]);
    hash *= kFnvPrime;
  }
  return hash;
}

std::uint64_t hash_file(std::uint64_t hash, const std::filesystem::path &path) {
  const std::string name = path.filename().generic_string();
  hash = hash_bytes(hash, name.data(), name.size());
  std::ifstream input(path, std::ios::binary);
  if (!input.is_open()) {
    return hash_bytes(hash, "missing", 7);
  }
  std::array<char, 64 * 1024> buffer{};
  while (input) {
    input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const auto count = input.gcount();
    if (count > 0) {
      hash = hash_bytes(hash, buffer.data(), static_cast<std::size_t>(count));
    }
  }
  return hash;
}

std::string hash_hex(std::uint64_t value) {
  std::ostringstream output;
  output << "fnv1a64:" << std::hex << std::setw(16) << std::setfill('0') << value;
  return output.str();
}

std::string loop_options_fingerprint(const LoopConstraintOptions &options) {
  std::ostringstream value;
  value << std::setprecision(17) << options.min_index_separation << '|'
        << options.min_path_separation_m << '|' << options.candidate_xy_radius_m << '|'
        << options.candidate_max_abs_z_m << '|' << options.submap_half_window << '|'
        << options.voxel_size_m << '|' << options.max_points_per_submap << '|'
        << options.angular_bins << '|' << options.radial_bins << '|' << options.height_bins << '|'
        << options.descriptor_max_radius_m << '|' << options.descriptor_min_z_m << '|'
        << options.descriptor_max_z_m << '|' << options.descriptor_min_similarity << '|'
        << options.descriptor_min_margin << '|' << options.descriptor_min_yaw_margin << '|'
        << options.yaw_hypotheses << '|' << options.max_spatial_candidates_per_anchor << '|'
        << options.max_candidates_per_anchor << '|' << options.max_total_candidates << '|'
        << options.max_correspondence_distance_m << '|' << options.max_icp_iterations << '|'
        << options.trim_fraction << '|' << options.min_inliers << '|' << options.min_inlier_ratio
        << '|' << options.max_rmse_m << '|' << options.max_p95_m << '|' << options.min_xy_span_m
        << '|' << options.min_z_span_m << '|' << options.normal_radius_m << '|'
        << options.normal_min_neighbors << '|' << options.normal_max_small_to_middle_ratio << '|'
        << options.normal_min_middle_to_large_ratio << '|' << options.normal_min_middle_variance_m2
        << '|' << options.min_planar_inliers << '|' << options.min_planar_inlier_ratio << '|'
        << options.min_point_to_plane_eigenvalue << '|' << options.max_point_to_plane_condition
        << '|' << options.max_hessian_condition << '|' << options.max_correction_xy_m << '|'
        << options.max_correction_z_m << '|' << options.max_correction_yaw_rad << '|'
        << options.max_inverse_translation_m << '|' << options.max_inverse_yaw_rad << '|'
        << options.require_consensus << '|' << options.min_consistent_matches << '|'
        << options.consensus_anchor_tolerance << '|' << options.consensus_step_tolerance << '|'
        << options.consensus_translation_m << '|' << options.consensus_yaw_rad;
  const std::string serialized = value.str();
  return hash_hex(hash_bytes(kFnvOffset, serialized.data(), serialized.size()));
}

std::string json_escape(const std::string &value) {
  std::ostringstream output;
  for (char character : value) {
    switch (character) {
      case '\\':
        output << "\\\\";
        break;
      case '"':
        output << "\\\"";
        break;
      case '\n':
        output << "\\n";
        break;
      case '\r':
        output << "\\r";
        break;
      case '\t':
        output << "\\t";
        break;
      default:
        output << character;
        break;
    }
  }
  return output.str();
}

void write_json_number(std::ostream &output, double value) {
  if (std::isfinite(value)) {
    output << value;
  } else {
    output << "null";
  }
}

void write_pose_json(std::ostream &output, const Pose &pose) {
  output << "{\"x\":";
  write_json_number(output, pose.x);
  output << ",\"y\":";
  write_json_number(output, pose.y);
  output << ",\"z\":";
  write_json_number(output, pose.z);
  output << ",\"qw\":";
  write_json_number(output, pose.qw);
  output << ",\"qx\":";
  write_json_number(output, pose.qx);
  output << ",\"qy\":";
  write_json_number(output, pose.qy);
  output << ",\"qz\":";
  write_json_number(output, pose.qz);
  output << "}";
}

void write_loop_options_json(std::ostream &output, const LoopConstraintOptions &options) {
  output << "{\"min_index_separation\":" << options.min_index_separation
         << ",\"min_path_separation_m\":";
  write_json_number(output, options.min_path_separation_m);
  output << ",\"candidate_xy_radius_m\":";
  write_json_number(output, options.candidate_xy_radius_m);
  output << ",\"candidate_max_abs_z_m\":";
  write_json_number(output, options.candidate_max_abs_z_m);
  output << ",\"submap_half_window\":" << options.submap_half_window << ",\"voxel_size_m\":";
  write_json_number(output, options.voxel_size_m);
  output << ",\"max_points_per_submap\":" << options.max_points_per_submap
         << ",\"angular_bins\":" << options.angular_bins
         << ",\"radial_bins\":" << options.radial_bins << ",\"height_bins\":" << options.height_bins
         << ",\"descriptor_max_radius_m\":";
  write_json_number(output, options.descriptor_max_radius_m);
  output << ",\"descriptor_min_z_m\":";
  write_json_number(output, options.descriptor_min_z_m);
  output << ",\"descriptor_max_z_m\":";
  write_json_number(output, options.descriptor_max_z_m);
  output << ",\"descriptor_min_similarity\":";
  write_json_number(output, options.descriptor_min_similarity);
  output << ",\"descriptor_min_margin\":";
  write_json_number(output, options.descriptor_min_margin);
  output << ",\"descriptor_min_yaw_margin\":";
  write_json_number(output, options.descriptor_min_yaw_margin);
  output << ",\"yaw_hypotheses\":" << options.yaw_hypotheses
         << ",\"max_spatial_candidates_per_anchor\":" << options.max_spatial_candidates_per_anchor
         << ",\"max_candidates_per_anchor\":" << options.max_candidates_per_anchor
         << ",\"max_total_candidates\":" << options.max_total_candidates
         << ",\"max_correspondence_distance_m\":";
  write_json_number(output, options.max_correspondence_distance_m);
  output << ",\"max_icp_iterations\":" << options.max_icp_iterations << ",\"trim_fraction\":";
  write_json_number(output, options.trim_fraction);
  output << ",\"min_inliers\":" << options.min_inliers << ",\"min_inlier_ratio\":";
  write_json_number(output, options.min_inlier_ratio);
  output << ",\"max_rmse_m\":";
  write_json_number(output, options.max_rmse_m);
  output << ",\"max_p95_m\":";
  write_json_number(output, options.max_p95_m);
  output << ",\"min_xy_span_m\":";
  write_json_number(output, options.min_xy_span_m);
  output << ",\"min_z_span_m\":";
  write_json_number(output, options.min_z_span_m);
  output << ",\"normal_radius_m\":";
  write_json_number(output, options.normal_radius_m);
  output << ",\"normal_min_neighbors\":" << options.normal_min_neighbors
         << ",\"normal_max_small_to_middle_ratio\":";
  write_json_number(output, options.normal_max_small_to_middle_ratio);
  output << ",\"normal_min_middle_to_large_ratio\":";
  write_json_number(output, options.normal_min_middle_to_large_ratio);
  output << ",\"normal_min_middle_variance_m2\":";
  write_json_number(output, options.normal_min_middle_variance_m2);
  output << ",\"min_planar_inliers\":" << options.min_planar_inliers
         << ",\"min_planar_inlier_ratio\":";
  write_json_number(output, options.min_planar_inlier_ratio);
  output << ",\"min_point_to_plane_eigenvalue\":";
  write_json_number(output, options.min_point_to_plane_eigenvalue);
  output << ",\"max_point_to_plane_condition\":";
  write_json_number(output, options.max_point_to_plane_condition);
  output << ",\"max_hessian_condition\":";
  write_json_number(output, options.max_hessian_condition);
  output << ",\"max_correction_xy_m\":";
  write_json_number(output, options.max_correction_xy_m);
  output << ",\"max_correction_z_m\":";
  write_json_number(output, options.max_correction_z_m);
  output << ",\"max_correction_yaw_rad\":";
  write_json_number(output, options.max_correction_yaw_rad);
  output << ",\"max_inverse_translation_m\":";
  write_json_number(output, options.max_inverse_translation_m);
  output << ",\"max_inverse_yaw_rad\":";
  write_json_number(output, options.max_inverse_yaw_rad);
  output << ",\"require_consensus\":" << (options.require_consensus ? "true" : "false")
         << ",\"min_consistent_matches\":" << options.min_consistent_matches
         << ",\"consensus_anchor_tolerance\":" << options.consensus_anchor_tolerance
         << ",\"consensus_step_tolerance\":" << options.consensus_step_tolerance
         << ",\"consensus_translation_m\":";
  write_json_number(output, options.consensus_translation_m);
  output << ",\"consensus_yaw_rad\":";
  write_json_number(output, options.consensus_yaw_rad);
  output << "}";
}

}  // namespace

LoopConstraintResult generate_loop_constraints(const Map &map,
                                               const std::vector<Keyframe> &keyframes,
                                               const LoopConstraintOptions &options) {
  const auto started = std::chrono::steady_clock::now();
  LoopConstraintResult result;
  result.report.options = options;
  result.report.options_fingerprint = loop_options_fingerprint(options);
  result.report.pose_count = keyframes.size();
  auto record_elapsed = [&]() {
    result.report.elapsed_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started)
            .count();
  };
  if (const auto option_error = validate_loop_options(options)) {
    result.code = "invalid_loop_options";
    result.message = *option_error;
    record_elapsed();
    return result;
  }
  if (const auto keyframe_error = validate_keyframes(keyframes)) {
    result.code = "invalid_loop_keyframes";
    result.message = *keyframe_error;
    record_elapsed();
    return result;
  }
  const Result map_check = check(map);
  if (!map_check.ok) {
    result.code = map_check.code;
    result.message = map_check.message;
    record_elapsed();
    return result;
  }
  if (keyframes.empty()) {
    result.code = "poses_empty";
    result.message = "no keyframes are available for loop verification";
    record_elapsed();
    return result;
  }

  try {
    const auto sorted_patches = sorted_point_cloud_files(map.patches_dir);
    result.report.patch_count = sorted_patches.size();
    if (sorted_patches.size() != keyframes.size()) {
      result.code = "patch_pose_mismatch";
      result.message = "patch count must exactly match keyframe count";
      record_elapsed();
      return result;
    }
    std::vector<std::filesystem::path> referenced_patches;
    referenced_patches.reserve(keyframes.size());
    std::set<std::string> referenced_names;
    for (std::size_t index = 0; index < keyframes.size(); ++index) {
      if (keyframes[index].patch_name.empty()) {
        throw std::runtime_error("patch provenance requires a basename in every poses.txt row");
      }
      const auto referenced = patch_path(map, sorted_patches, keyframes[index], index);
      if (referenced.empty()) {
        throw std::runtime_error("invalid or missing named patch for keyframe " +
                                 std::to_string(index));
      }
      const std::string name = referenced.filename().generic_string();
      if (!referenced_names.insert(name).second) {
        throw std::runtime_error("duplicate named patch in poses.txt: " + name);
      }
      referenced_patches.push_back(referenced);
    }
    result.report.poses_fingerprint = hash_hex(hash_file(kFnvOffset, map.poses_txt));
    std::uint64_t patch_hash = kFnvOffset;
    for (const auto &patch : referenced_patches) {
      patch_hash = hash_file(patch_hash, patch);
    }
    result.report.patches_fingerprint = hash_hex(patch_hash);

    std::map<std::size_t, std::vector<Point>> gravity_submaps;
    std::map<std::size_t, Descriptor> descriptors;
    auto gravity_submap_at = [&](std::size_t index) -> const std::vector<Point> & {
      auto found = gravity_submaps.find(index);
      if (found == gravity_submaps.end()) {
        found =
            gravity_submaps
                .emplace(index, gravity_aligned_cloud(
                                    build_submap(map, keyframes, sorted_patches, index, options),
                                    keyframes[index].pose))
                .first;
      }
      return found->second;
    };
    auto descriptor_at = [&](std::size_t index) -> const Descriptor & {
      auto found = descriptors.find(index);
      if (found == descriptors.end()) {
        found =
            descriptors.emplace(index, make_descriptor(gravity_submap_at(index), options)).first;
      }
      return found->second;
    };

    const auto candidates = generate_candidates(keyframes, options, descriptor_at);
    result.report.candidate_count = candidates.size();
    std::vector<VerifiedCandidate> verified;
    verified.reserve(candidates.size());
    for (const Candidate &candidate : candidates) {
      LoopCandidateDiagnostic diagnostic;
      const std::size_t diagnostic_index = result.report.candidates.size();
      auto candidate_result =
          verify_candidate(candidate, keyframes, gravity_submap_at(candidate.from_index),
                           gravity_submap_at(candidate.to_index), options, diagnostic);
      result.report.candidates.push_back(diagnostic);
      if (candidate_result.has_value()) {
        candidate_result->diagnostic_index = diagnostic_index;
        verified.push_back(*candidate_result);
      }
    }
    result.report.geometrically_verified_count = verified.size();
    const std::string poses_after = hash_hex(hash_file(kFnvOffset, map.poses_txt));
    std::uint64_t patch_hash_after = kFnvOffset;
    for (const auto &patch : referenced_patches) {
      patch_hash_after = hash_file(patch_hash_after, patch);
    }
    const std::string patches_after = hash_hex(patch_hash_after);
    if (poses_after != result.report.poses_fingerprint ||
        patches_after != result.report.patches_fingerprint) {
      result.ok = false;
      result.code = "map_changed_during_verification";
      result.message = "poses.txt or a referenced patch changed during shadow verification";
      result.constraints.clear();
      result.report.accepted_constraint_count = 0;
      result.report.rejected_count = result.report.candidate_count;
      record_elapsed();
      return result;
    }
    apply_consensus(verified, result, options);
    result.report.accepted_constraint_count = result.constraints.size();
    result.report.consensus_support_count = static_cast<std::size_t>(
        std::count_if(result.report.candidates.begin(), result.report.candidates.end(),
                      [](const LoopCandidateDiagnostic &diagnostic) {
                        return diagnostic.reason == "consensus_support";
                      }));
    const std::size_t non_rejected =
        result.report.accepted_constraint_count + result.report.consensus_support_count;
    result.report.rejected_count = result.report.candidate_count > non_rejected
                                       ? result.report.candidate_count - non_rejected
                                       : 0;
    result.ok = true;
    result.code = result.constraints.empty() ? "shadow_no_verified_loops" : "shadow_verified_loops";
    result.message = result.constraints.empty()
                         ? "shadow loop verification completed without an accepted loop"
                         : "shadow loop verification produced accepted independent constraints";
  } catch (const std::exception &exception) {
    result.ok = false;
    result.code = "loop_verification_failed";
    result.message = exception.what();
  }
  record_elapsed();
  return result;
}

bool write_loop_constraints_report(const std::filesystem::path &path,
                                   const LoopConstraintResult &result, std::string *error) {
  std::filesystem::path temp_path;
  try {
    if (path.empty()) {
      throw std::runtime_error("loop report path is empty");
    }
    if (!path.parent_path().empty()) {
      std::filesystem::create_directories(path.parent_path());
    }
    if (std::filesystem::exists(path)) {
      throw std::runtime_error("refusing to overwrite an existing loop report: " + path.string());
    }
    const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
    for (std::size_t attempt = 0; attempt < 100; ++attempt) {
      temp_path = path;
      temp_path += ".tmp-" + std::to_string(nonce) + "-" + std::to_string(attempt);
      if (!std::filesystem::exists(temp_path)) {
        break;
      }
      temp_path.clear();
    }
    if (temp_path.empty()) {
      throw std::runtime_error("failed to allocate a temporary loop report path");
    }
    std::ofstream output(temp_path, std::ios::binary | std::ios::trunc);
    if (!output.is_open()) {
      throw std::runtime_error("failed to open temporary loop report: " + temp_path.string());
    }
    output << std::setprecision(17);
    output << "{\n"
           << "  \"schema_version\":\"" << json_escape(result.report.schema_version) << "\",\n"
           << "  \"algorithm_version\":\"" << json_escape(result.report.algorithm_version)
           << "\",\n"
           << "  \"threshold_version\":\"" << json_escape(result.report.threshold_version)
           << "\",\n"
           << "  \"mode\":\"shadow\",\n"
           << "  \"ok\":" << (result.ok ? "true" : "false") << ",\n"
           << "  \"code\":\"" << json_escape(result.code) << "\",\n"
           << "  \"message\":\"" << json_escape(result.message) << "\",\n"
           << "  \"frame_convention\":\"" << json_escape(result.report.frame_convention) << "\",\n"
           << "  \"information_convention\":\"" << json_escape(result.report.information_convention)
           << "\",\n"
           << "  \"options_fingerprint\":\"" << json_escape(result.report.options_fingerprint)
           << "\",\n"
           << "  \"options\":";
    write_loop_options_json(output, result.report.options);
    output << ",\n"
           << "  \"poses_fingerprint\":\"" << json_escape(result.report.poses_fingerprint)
           << "\",\n"
           << "  \"patches_fingerprint\":\"" << json_escape(result.report.patches_fingerprint)
           << "\",\n"
           << "  \"pose_count\":" << result.report.pose_count << ",\n"
           << "  \"patch_count\":" << result.report.patch_count << ",\n"
           << "  \"candidate_count\":" << result.report.candidate_count << ",\n"
           << "  \"geometrically_verified_count\":" << result.report.geometrically_verified_count
           << ",\n"
           << "  \"accepted_constraint_count\":" << result.report.accepted_constraint_count << ",\n"
           << "  \"consensus_support_count\":" << result.report.consensus_support_count << ",\n"
           << "  \"rejected_count\":" << result.report.rejected_count << ",\n"
           << "  \"elapsed_ms\":";
    write_json_number(output, result.report.elapsed_ms);
    output << ",\n  \"constraints\":[";
    for (std::size_t index = 0; index < result.constraints.size(); ++index) {
      if (index > 0) {
        output << ",";
      }
      const auto &constraint = result.constraints[index];
      output << "{\"from_index\":" << constraint.from_index
             << ",\"to_index\":" << constraint.to_index << ",\"pose_from_to\":";
      write_pose_json(output, constraint.pose_from_to);
      output << ",\"information_diagonal\":[";
      for (std::size_t axis = 0; axis < constraint.information_diagonal.size(); ++axis) {
        if (axis > 0) {
          output << ",";
        }
        write_json_number(output, constraint.information_diagonal[axis]);
      }
      output << "]}";
    }
    output << "],\n  \"candidates\":[";
    for (std::size_t index = 0; index < result.report.candidates.size(); ++index) {
      if (index > 0) {
        output << ",";
      }
      const auto &candidate = result.report.candidates[index];
      output << "{\"from_index\":" << candidate.from_index << ",\"to_index\":" << candidate.to_index
             << ",\"accepted\":" << (candidate.accepted ? "true" : "false") << ",\"reason\":\""
             << json_escape(candidate.reason) << "\""
             << ",\"descriptor_score\":";
      write_json_number(output, candidate.descriptor_score);
      output << ",\"descriptor_margin\":";
      write_json_number(output, candidate.descriptor_margin);
      output << ",\"descriptor_yaw_margin\":";
      write_json_number(output, candidate.descriptor_yaw_margin);
      output << ",\"yaw_hypothesis_rad\":";
      write_json_number(output, candidate.yaw_hypothesis_rad);
      output << ",\"source_points\":" << candidate.source_points
             << ",\"target_points\":" << candidate.target_points
             << ",\"inliers\":" << candidate.inliers << ",\"inlier_ratio\":";
      write_json_number(output, candidate.inlier_ratio);
      output << ",\"rmse_m\":";
      write_json_number(output, candidate.rmse_m);
      output << ",\"p50_m\":";
      write_json_number(output, candidate.p50_m);
      output << ",\"p95_m\":";
      write_json_number(output, candidate.p95_m);
      output << ",\"hessian_condition\":";
      write_json_number(output, candidate.hessian_condition);
      output << ",\"planar_inliers\":" << candidate.planar_inliers << ",\"planar_inlier_ratio\":";
      write_json_number(output, candidate.planar_inlier_ratio);
      output << ",\"point_to_plane_eigenvalues\":[";
      for (std::size_t axis = 0; axis < candidate.point_to_plane_eigenvalues.size(); ++axis) {
        if (axis > 0) {
          output << ",";
        }
        write_json_number(output, candidate.point_to_plane_eigenvalues[axis]);
      }
      output << "],\"point_to_plane_weak_mode\":[";
      for (std::size_t axis = 0; axis < candidate.point_to_plane_weak_mode.size(); ++axis) {
        if (axis > 0) {
          output << ",";
        }
        write_json_number(output, candidate.point_to_plane_weak_mode[axis]);
      }
      output << "],\"point_to_plane_condition\":";
      write_json_number(output, candidate.point_to_plane_condition);
      output << ",\"correction_xy_m\":";
      write_json_number(output, candidate.correction_xy_m);
      output << ",\"correction_z_m\":";
      write_json_number(output, candidate.correction_z_m);
      output << ",\"correction_yaw_rad\":";
      write_json_number(output, candidate.correction_yaw_rad);
      output << ",\"inverse_translation_error_m\":";
      write_json_number(output, candidate.inverse_translation_error_m);
      output << ",\"inverse_yaw_error_rad\":";
      write_json_number(output, candidate.inverse_yaw_error_rad);
      output << ",\"iterations\":" << candidate.iterations << "}";
    }
    output << "]\n}\n";
    if (!output.good()) {
      throw std::runtime_error("failed while writing loop report: " + path.string());
    }
    output.close();
    if (!output.good()) {
      throw std::runtime_error("failed while closing loop report: " + temp_path.string());
    }
    std::error_code rename_error;
    std::filesystem::rename(temp_path, path, rename_error);
    if (rename_error) {
      throw std::runtime_error("failed to publish loop report atomically: " +
                               rename_error.message());
    }
    temp_path.clear();
    return true;
  } catch (const std::exception &exception) {
    if (!temp_path.empty()) {
      std::error_code cleanup_error;
      std::filesystem::remove(temp_path, cleanup_error);
    }
    if (error != nullptr) {
      *error = exception.what();
    }
    return false;
  }
}

}  // namespace lingtu::localization::opt

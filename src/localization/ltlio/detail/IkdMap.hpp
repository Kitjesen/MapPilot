#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <pcl/point_cloud.h>

#include "ikd_Tree.h"
#include "ProbabilityPlane.hpp"
#include "HybridCorrespondence.hpp"
namespace lingtu::ltlio::detail {
/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Project-level ikd-tree map wrapper for elevator-lio.
 * The local map update workflow follows the FAST-LIO / ikd-Tree design.
 */



template <typename PointType>
struct IKD_MAP_TEST_ACCESS;

template <typename PointType>
class IkdMap final {
public:
    using PointCloudConstPtr = typename pcl::PointCloud<PointType>::ConstPtr;
    using KDTreeType = KD_TREE<PointType>;
    using KDPointVector = typename KDTreeType::PointVector;

    struct NeighborCacheEntry {
        KDPointVector points;
        std::vector<float> distance_sq;
    };

    struct PlaneBatch {
        std::vector<ProbabilityPlane> planes;
        std::vector<std::vector<std::size_t>> candidate_indices;
        std::vector<HybridPointBucket> point_buckets;
        std::vector<std::vector<std::size_t>> point_bucket_indices;
        std::vector<std::size_t> candidate_roots;
    };

    struct Config {
        int knn = 5;
        double default_map_resolution = 0.5;
        double voxel_half_extent_ratio = 0.5;
        double local_map_cube_len = 1500.0;
        double local_map_detect_range = 200.0;
        double local_map_move_threshold = 1.5;
        double local_map_move_scale = 0.9;
        double cache_quantization_scale = 100.0;
        int cache_key_axis_bits = 21;
        std::size_t max_map_points = 0;
    };

    explicit IkdMap(const Config& config = Config(), KDTreeType* external_tree = nullptr)
            : config_(config),
              owned_tree_(external_tree == nullptr ? std::make_unique<KDTreeType>() : nullptr),
              tree_ptr_(external_tree != nullptr ? external_tree : owned_tree_.get()),
              map_resolution_(config_.default_map_resolution),
              cube_len_(config_.local_map_cube_len),
              det_range_(config_.local_map_detect_range),
              mov_threshold_(config_.local_map_move_threshold) {}

    ~IkdMap() = default;

    void init(double map_resolution) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (map_resolution > 0.0) {
            map_resolution_ = map_resolution;
            tree_ptr_->set_downsample_param(static_cast<float>(map_resolution));
        }
        initialized_ = true;
    }

    void add(const PointCloudConstPtr& cloud) {
        // Geometry-only callers (native fixtures) declare exact target points.
        // Runtime insertions always supply the propagated physical covariance.
        add(cloud, std::vector<Eigen::Matrix3d>(cloud ? cloud->size() : 0,
                                               Eigen::Matrix3d::Zero()));
    }

    void add(const PointCloudConstPtr& cloud,
             const std::vector<Eigen::Matrix3d>& point_covariances) {
        if (!cloud || cloud->points.empty()) {
            return;
        }
        if (point_covariances.size() != cloud->size())
            throw std::invalid_argument("map covariance count must equal point count");

        std::lock_guard<std::mutex> lock(mutex_);
        if (!initialized_) {
            return;
        }

        KDPointVector points;
        points.reserve(cloud->points.size());
        for (const auto& p : cloud->points) {
            points.push_back(p);
        }

        // 首次插入需要 Build，后续帧才使用 Add_Points 增量更新
        if (tree_ptr_->Root_Node == nullptr) {
            for (std::size_t i = 0; i < points.size(); ++i)
                point_covariances_.emplace(pointKey(points[i]), point_covariances[i]);
            tree_ptr_->Build(points);
            enforcePointLimit();
            compactCovarianceStorage();
            return;
        }

        KDPointVector points_need_downsample;
        KDPointVector points_no_need_downsample;
        std::vector<std::size_t> submitted_indices;
        points_need_downsample.reserve(points.size());
        points_no_need_downsample.reserve(points.size());

        const int kNumMatchPoints = std::max(1, config_.knn);
        const double filter_size_map_min =
                (map_resolution_ > 0.0) ? map_resolution_ : config_.default_map_resolution;
        const double half_filter = config_.voxel_half_extent_ratio * filter_size_map_min;
        constexpr float kMinInsertDistanceSq = 0.02f * 0.02f; // 2cm

        for (std::size_t point_index = 0; point_index < points.size(); ++point_index) {
            const auto& world_point = points[point_index];
            const NeighborCacheEntry& near = nearest(world_point, kNumMatchPoints);
            const auto& points_near = near.points;
            const auto& distance_sq = near.distance_sq;

            if (!points_near.empty()) {
                // 如果当前点离最近邻过近（<=2cm），则不再添加该点
                if (!distance_sq.empty() && distance_sq[0] <= kMinInsertDistanceSq) {
                    continue;
                }

                bool need_add = true;
                const PointType mid_point = voxelCenter(world_point, filter_size_map_min, half_filter);
                const float dist = dist2(world_point, mid_point);
                if (std::fabs(points_near[0].x - mid_point.x) > half_filter &&
                    std::fabs(points_near[0].y - mid_point.y) > half_filter &&
                    std::fabs(points_near[0].z - mid_point.z) > half_filter) {
                    points_no_need_downsample.push_back(world_point);
                    submitted_indices.push_back(point_index);
                    continue;
                }

                // 判断当前点的 NUM_MATCH_POINTS 个邻近点与包围盒中心的关系
                for (int idx = 0; idx < kNumMatchPoints; ++idx) {
                    if (static_cast<int>(points_near.size()) < kNumMatchPoints) {
                        break; // 邻近点不足时直接保留 need_add=true
                    }
                    if (dist2(points_near[static_cast<size_t>(idx)], mid_point) < dist) {
                        need_add = false;
                        break;
                    }
                }
                if (need_add) {
                    points_need_downsample.push_back(world_point);
                    submitted_indices.push_back(point_index);
                }
            } else {
                // 如果周围没有点，则加入到需要降采样集合
                points_need_downsample.push_back(world_point);
                submitted_indices.push_back(point_index);
            }
        }

        tree_ptr_->Add_Points(points_need_downsample, true);
        tree_ptr_->Add_Points(points_no_need_downsample, false);
        // Existing identical points are excluded by the insertion distance
        // gate. Assign only submitted points, so a previously deleted coordinate
        // reused by a new measurement receives that measurement's covariance.
        for (const std::size_t i : submitted_indices)
            point_covariances_.insert_or_assign(pointKey(points[i]), point_covariances[i]);
        for (const auto& point : points_need_downsample) dirtyDownsampleRoots(point);
        for (const auto& point : points_no_need_downsample) dirtyRoot(point);
        nearest_cache_.clear(); // 地图已更新，旧近邻结果失效
        enforcePointLimit();
        compactCovarianceStorage();
    }

private:
    template <typename>
    friend struct IKD_MAP_TEST_ACCESS;

    bool pointCovariance(const PointType& point, Eigen::Matrix3d& covariance) const {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto found = point_covariances_.find(pointKey(point));
        if (found == point_covariances_.end()) return false;
        covariance = found->second;
        return true;
    }

    std::size_t covarianceCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return point_covariances_.size();
    }

    bool fitPlane(const KDPointVector& neighbors, double max_rmse,
                  ProbabilityPlane& plane) const {
        std::vector<Eigen::Vector3d> points;
        std::vector<Eigen::Matrix3d> covariances;
        points.reserve(neighbors.size());
        covariances.reserve(neighbors.size());
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (const auto& point : neighbors) {
                const auto found = point_covariances_.find(pointKey(point));
                if (found == point_covariances_.end()) return false;
                points.emplace_back(point.x, point.y, point.z);
                covariances.push_back(found->second);
            }
        }
        return fitProbabilityPlane(points, covariances, max_rmse, plane);
    }

    std::size_t planeCacheRootCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return plane_roots_.size();
    }

    void planeCandidates(const PointType& query, const KDPointVector& nearest_points,
                         double max_rmse, std::vector<ProbabilityPlane>& planes) {
        std::lock_guard<std::mutex> lock(mutex_);
        planes.clear();
        std::vector<RootKey> roots;
        roots.push_back(rootKey(query));
        for (const auto& point : nearest_points) {
            const auto key = rootKey(point);
            if (std::find(roots.begin(), roots.end(), key) == roots.end()) roots.push_back(key);
        }
        for (const auto& key : roots) appendRootPlanes(key, max_rmse, planes);
    }

public:
    PlaneBatch planeCandidatesBatch(const KDPointVector& queries,
            const std::vector<KDPointVector>& nearest_points,
            const std::vector<std::uint8_t>& eligible, double max_rmse,
            bool include_point_candidates = false) {
        if (queries.size() != nearest_points.size() || queries.size() != eligible.size())
            throw std::invalid_argument("plane batch query counts must match");
        std::lock_guard<std::mutex> lock(mutex_);
        PlaneBatch batch;
        batch.candidate_indices.resize(queries.size());
        if (include_point_candidates) {
            batch.point_bucket_indices.resize(queries.size());
            batch.candidate_roots.assign(queries.size(), 0);
        }
        struct RootWork {
            RootKey key;
            CachedRoot* cached;
            bool empty;
            std::vector<ProbabilityPlane> planes;
            std::vector<HybridPointBucket> point_buckets;
        };
        std::vector<RootWork> work;
        std::unordered_map<RootKey, std::size_t, RootKeyHash> prepared;
        prepared.reserve(queries.size());
        std::vector<std::vector<std::size_t>> point_roots(queries.size());
        std::vector<RootKey> roots;
        for (std::size_t i = 0; i < queries.size(); ++i) {
            if (!eligible[i]) continue;
            roots.clear();
            roots.push_back(rootKey(queries[i]));
            for (const auto& point : nearest_points[i]) {
                const auto key = rootKey(point);
                if (std::find(roots.begin(), roots.end(), key) == roots.end()) roots.push_back(key);
            }
            for (const auto& key : roots) {
                const auto entry = prepared.emplace(key, work.size());
                if (entry.second) work.push_back(RootWork{key, nullptr, false, {}, {}});
                point_roots[i].push_back(entry.first->second);
            }
        }

        // Finish all hash-table mutation before independent workers start.
        plane_roots_.reserve(plane_roots_.size() + work.size());
        for (auto& item : work) item.cached = &plane_roots_.try_emplace(item.key).first->second;
        const auto work_count = static_cast<std::ptrdiff_t>(work.size());
#ifdef MP_EN
        #pragma omp parallel for schedule(guided, 1) num_threads(MP_PROC_NUM)
#endif
        for (std::ptrdiff_t i = 0; i < work_count; ++i) {
            auto& item = work[i];
            auto& cached = *item.cached;
            if (cached.dirty || cached.max_rmse != max_rmse ||
                (include_point_candidates && !cached.point_candidates_ready)) {
                KDPointVector retained;
                tree_ptr_->Box_Search(rootBox(item.key), retained);
                if (retained.empty()) {
                    item.empty = true;
                    continue;
                }
                refreshPlaneNode(cached.node, retained, rootBox(item.key), 0, max_rmse,
                                 include_point_candidates);
                cached.dirty = false;
                cached.max_rmse = max_rmse;
                cached.point_candidates_ready = include_point_candidates;
            }
            collectPlanes(cached.node, item.planes);
            if (include_point_candidates) collectPointBuckets(cached.node, item.point_buckets);
        }

        std::size_t plane_count = 0;
        for (const auto& item : work) plane_count += item.planes.size();
        batch.planes.reserve(plane_count);
        std::vector<std::pair<std::size_t, std::size_t>> ranges(work.size());
        std::vector<std::pair<std::size_t, std::size_t>> point_ranges(work.size());
        for (std::size_t i = 0; i < work.size(); ++i) {
            const auto& item = work[i];
            if (item.empty) plane_roots_.erase(item.key);
            const std::size_t first = batch.planes.size();
            batch.planes.insert(batch.planes.end(), item.planes.begin(), item.planes.end());
            ranges[i] = std::make_pair(first, batch.planes.size());
            if (include_point_candidates) {
                const std::size_t first_point = batch.point_buckets.size();
                batch.point_buckets.insert(batch.point_buckets.end(),
                    item.point_buckets.begin(), item.point_buckets.end());
                point_ranges[i] = std::make_pair(first_point, batch.point_buckets.size());
            }
        }
        // Snapshot values own their lifetime; original candidate order sets ties.
        for (std::size_t i = 0; i < queries.size(); ++i) {
            for (const std::size_t root : point_roots[i]) {
                for (std::size_t index = ranges[root].first; index < ranges[root].second; ++index)
                    batch.candidate_indices[i].push_back(index);
                if (include_point_candidates) {
                    if (!work[root].empty) ++batch.candidate_roots[i];
                    for (std::size_t index = point_ranges[root].first;
                         index < point_ranges[root].second; ++index)
                        batch.point_bucket_indices[i].push_back(index);
                }
            }
        }
        return batch;
    }

    void reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        KDPointVector empty_points;
        tree_ptr_->Build(empty_points);
        point_covariances_.clear();
        plane_roots_.clear();
        nearest_cache_.clear();
        localmap_initialized_ = false;
        local_map_points_ = BoxPointType{};
    }

    void removeFar(const Eigen::Vector3d& current_pos) {
        std::lock_guard<std::mutex> lock(mutex_);
        current_position_ = current_pos;
        if (!initialized_) {
            return;
        }

        if (!localmap_initialized_) {
            // 以 world 系当前位置为中心，初始化局部地图包围盒（长宽高均为 cube_len_）
            for (int i = 0; i < 3; ++i) {
                local_map_points_.vertex_min[i] = static_cast<float>(current_pos(i) - cube_len_ * 0.5);
                local_map_points_.vertex_max[i] = static_cast<float>(current_pos(i) + cube_len_ * 0.5);
            }
            localmap_initialized_ = true;
            return;
        }

        double dist_to_map_edge[3][2];
        bool need_move = false;
        for (int i = 0; i < 3; ++i) {
            dist_to_map_edge[i][0] = std::fabs(current_pos(i) - local_map_points_.vertex_min[i]);
            dist_to_map_edge[i][1] = std::fabs(current_pos(i) - local_map_points_.vertex_max[i]);
            if (dist_to_map_edge[i][0] <= mov_threshold_ * det_range_ ||
                dist_to_map_edge[i][1] <= mov_threshold_ * det_range_) {
                need_move = true;
            }
        }
        if (!need_move) {
            return;
        }

        BoxPointType new_local_map_points = local_map_points_;
        std::vector<BoxPointType> boxes_to_remove;
        boxes_to_remove.reserve(3);

        const double mov_dist = std::max(
                config_.local_map_move_scale * (0.5 * cube_len_ - mov_threshold_ * det_range_),
                det_range_ * (mov_threshold_ - 1.0));

        for (int i = 0; i < 3; ++i) {
            BoxPointType tmp_box = local_map_points_;
            if (dist_to_map_edge[i][0] <= mov_threshold_ * det_range_) {
                // 如果与坐标较小的那个面较近，包围框整体向负方向移动
                new_local_map_points.vertex_min[i] -= static_cast<float>(mov_dist);
                new_local_map_points.vertex_max[i] -= static_cast<float>(mov_dist);
                tmp_box.vertex_min[i] = local_map_points_.vertex_max[i] - static_cast<float>(mov_dist);
                boxes_to_remove.push_back(tmp_box);
            } else if (dist_to_map_edge[i][1] <= mov_threshold_ * det_range_) {
                // 如果与坐标较大的那个面较近，包围框整体向正方向移动
                new_local_map_points.vertex_min[i] += static_cast<float>(mov_dist);
                new_local_map_points.vertex_max[i] += static_cast<float>(mov_dist);
                tmp_box.vertex_max[i] = local_map_points_.vertex_min[i] + static_cast<float>(mov_dist);
                boxes_to_remove.push_back(tmp_box);
            }
        }

        local_map_points_ = new_local_map_points;
        if (!boxes_to_remove.empty()) {
            tree_ptr_->Delete_Point_Boxes(boxes_to_remove);
            for (const auto& box : boxes_to_remove) dirtyBox(box);
            for (auto it = point_covariances_.begin(); it != point_covariances_.end();) {
                bool removed = false;
                for (const auto& box : boxes_to_remove) {
                    removed = true;
                    for (int axis = 0; axis < 3; ++axis)
                        removed = removed && it->first[axis] >= box.vertex_min[axis] &&
                            it->first[axis] < box.vertex_max[axis];
                    if (removed) break;
                }
                if (removed) it = point_covariances_.erase(it);
                else ++it;
            }
            purgeEmptyPlaneRoots();
            nearest_cache_.clear(); // 地图已更新，旧近邻结果失效
        }
    }

private:
    using PointKey = std::array<float, 3>;
    struct PointKeyHash {
        std::size_t operator()(const PointKey& point) const {
            std::size_t value = std::hash<float>{}(point[0]);
            for (int i = 1; i < 3; ++i)
                value ^= std::hash<float>{}(point[i]) + 0x9e3779b9U + (value << 6) + (value >> 2);
            return value;
        }
    };
    static PointKey pointKey(const PointType& point) { return {point.x, point.y, point.z}; }
    using CovarianceStorage = std::unordered_map<PointKey, Eigen::Matrix3d, PointKeyHash>;

    using RootKey = std::array<std::int64_t, 3>;
    struct RootKeyHash {
        std::size_t operator()(const RootKey& key) const {
            std::size_t value = std::hash<std::int64_t>{}(key[0]);
            for (int axis = 1; axis < 3; ++axis)
                value ^= std::hash<std::int64_t>{}(key[axis]) + 0x9e3779b9U + (value << 6) + (value >> 2);
            return value;
        }
    };
    struct PlaneNode {
        ProbabilityPlane plane;
        bool planar = false;
        std::vector<PointKey> support;
        std::unique_ptr<HybridPointBucket> point_bucket;
        std::array<std::unique_ptr<PlaneNode>, 8> children;
    };
    struct CachedRoot {
        PlaneNode node;
        bool dirty = true;
        double max_rmse = 0.0;
        bool point_candidates_ready = false;
    };
    // A derived index of retained geometry, not an independently updated map.
    std::unordered_map<RootKey, CachedRoot, RootKeyHash> plane_roots_;

    static RootKey rootKey(const PointType& point) {
        return {static_cast<std::int64_t>(std::floor(point.x)),
                static_cast<std::int64_t>(std::floor(point.y)),
                static_cast<std::int64_t>(std::floor(point.z))};
    }

    static BoxPointType rootBox(const RootKey& key) {
        BoxPointType box;
        for (int axis = 0; axis < 3; ++axis) {
            box.vertex_min[axis] = static_cast<float>(key[axis]);
            box.vertex_max[axis] = static_cast<float>(key[axis] + 1);
        }
        return box;
    }

    void dirtyRoot(const PointType& point) {
        const auto found = plane_roots_.find(rootKey(point));
        if (found != plane_roots_.end()) found->second.dirty = true;
    }

    void dirtyDownsampleRoots(const PointType& point) {
        // A map-resolution voxel can straddle a 1 m plane-root boundary.
        RootKey lower, upper;
        const float coordinates[3] = {point.x, point.y, point.z};
        const float downsample_size = static_cast<float>(map_resolution_);
        for (int axis = 0; axis < 3; ++axis) {
            // Match ikd Add_Points' float division and box arithmetic exactly.
            const float minimum = static_cast<float>(std::floor(coordinates[axis] / downsample_size) * downsample_size);
            const float maximum = minimum + downsample_size;
            lower[axis] = static_cast<std::int64_t>(std::floor(minimum));
            upper[axis] = static_cast<std::int64_t>(std::ceil(maximum)) - 1;
        }
        for (std::int64_t x = lower[0]; x <= upper[0]; ++x)
            for (std::int64_t y = lower[1]; y <= upper[1]; ++y)
                for (std::int64_t z = lower[2]; z <= upper[2]; ++z) {
                    const auto found = plane_roots_.find(RootKey{x, y, z});
                    if (found != plane_roots_.end()) found->second.dirty = true;
                }
    }

    void dirtyBox(const BoxPointType& box) {
        for (auto& entry : plane_roots_) {
            const auto cell = rootBox(entry.first);
            bool intersects = true;
            for (int axis = 0; axis < 3; ++axis)
                intersects = intersects && cell.vertex_min[axis] < box.vertex_max[axis] &&
                    cell.vertex_max[axis] > box.vertex_min[axis];
            if (intersects) entry.second.dirty = true;
        }
    }

    void purgeEmptyPlaneRoots() {
        KDPointVector retained;
        for (auto it = plane_roots_.begin(); it != plane_roots_.end();) {
            if (!it->second.dirty) { ++it; continue; }
            tree_ptr_->Box_Search(rootBox(it->first), retained);
            if (retained.empty()) it = plane_roots_.erase(it);
            else ++it;
        }
    }

    void appendRootPlanes(const RootKey& key, double max_rmse,
                          std::vector<ProbabilityPlane>& planes) {
        auto found = plane_roots_.find(key);
        if (found == plane_roots_.end() || found->second.dirty || found->second.max_rmse != max_rmse) {
            KDPointVector retained;
            tree_ptr_->Box_Search(rootBox(key), retained);
            if (retained.empty()) {
                if (found != plane_roots_.end()) plane_roots_.erase(found);
                return;
            }
            auto& cached = plane_roots_[key];
            refreshPlaneNode(cached.node, retained, rootBox(key), 0, max_rmse);
            cached.dirty = false;
            cached.max_rmse = max_rmse;
            cached.point_candidates_ready = false;
            found = plane_roots_.find(key);
        }
        collectPlanes(found->second.node, planes);
    }

    static void collectPlanes(const PlaneNode& node, std::vector<ProbabilityPlane>& planes) {
        if (node.planar) { planes.push_back(node.plane); return; }
        for (const auto& child : node.children)
            if (child) collectPlanes(*child, planes);
    }

    static void collectPointBuckets(const PlaneNode& node,
                                   std::vector<HybridPointBucket>& buckets) {
        // An accepted planar leaf contributes no second, correlated point term.
        if (node.planar) return;
        if (node.point_bucket && !node.point_bucket->points.empty()) {
            buckets.push_back(*node.point_bucket);
            return;
        }
        for (const auto& child : node.children)
            if (child) collectPointBuckets(*child, buckets);
    }

    void refreshPlaneNode(PlaneNode& node, const KDPointVector& retained,
                          const BoxPointType& box, int level, double max_rmse,
                          bool include_point_candidates = false) {
        constexpr std::size_t max_support = 100;
        constexpr int max_layer = 3;
        struct RetainedReference {
            std::size_t index;
            const Eigen::Matrix3d* covariance;
            bool covariance_checked;
        };
        std::vector<RetainedReference> references;
        references.reserve(retained.size());
        for (std::size_t i = 0; i < retained.size(); ++i)
            references.push_back(RetainedReference{i, nullptr, false});
        std::sort(references.begin(), references.end(), [&](const auto& left, const auto& right) {
            const auto left_key = pointKey(retained[left.index]);
            const auto right_key = pointKey(retained[right.index]);
            return left_key == right_key ? left.index < right.index : left_key < right_key;
        });
        std::vector<std::size_t> root_indices;
        root_indices.reserve(references.size());
        for (std::size_t i = 0; i < references.size(); ++i) root_indices.push_back(i);
        const auto keyAt = [&](std::size_t index) { return pointKey(retained[references[index].index]); };

        const auto refresh = [&](auto&& self, PlaneNode& current,
                const std::vector<std::size_t>& indices,
                const BoxPointType& current_box, int depth) -> void {
            std::vector<std::size_t> selected;
            std::vector<PointKey> support;
            selected.reserve(std::min(max_support, indices.size()));
            support.reserve(std::min(max_support, indices.size()));
            std::vector<std::uint8_t> used(indices.size(), 0);
            // Preserve surviving support before adding new exact-XYZ groups.
            for (const auto& key : current.support) {
                const auto found = std::lower_bound(indices.begin(), indices.end(), key,
                    [&](std::size_t index, const PointKey& value) { return keyAt(index) < value; });
                if (found == indices.end() || keyAt(*found) != key) continue;
                const std::size_t position = static_cast<std::size_t>(found - indices.begin());
                if (used[position]) continue;
                used[position] = 1;
                selected.push_back(*found);
                support.push_back(key);
            }
            for (std::size_t position = 0; position < indices.size();) {
                if (selected.size() == max_support) break;
                const auto key = keyAt(indices[position]);
                if (!used[position]) {
                    selected.push_back(indices[position]);
                    support.push_back(key);
                }
                // The first original retained occurrence represents duplicates.
                do { ++position; } while (position < indices.size() && keyAt(indices[position]) == key);
            }
            current.support = std::move(support);
            std::vector<Eigen::Vector3d> points;
            std::vector<Eigen::Matrix3d> covariances;
            points.reserve(selected.size());
            covariances.reserve(selected.size());
            for (const auto index : selected) {
                auto& reference = references[index];
                const auto& point = retained[reference.index];
                if (!reference.covariance_checked) {
                    const auto found = point_covariances_.find(pointKey(point));
                    reference.covariance = found == point_covariances_.end() ? nullptr : &found->second;
                    reference.covariance_checked = true;
                }
                if (!reference.covariance) continue;
                points.emplace_back(point.x, point.y, point.z);
                covariances.push_back(*reference.covariance);
            }
            current.planar = points.size() >= 4 && fitProbabilityPlane(points, covariances, max_rmse, current.plane);
            if (current.point_bucket) current.point_bucket->points.clear();
            if (current.planar || depth == max_layer || indices.size() < 4) {
                for (auto& child : current.children) child.reset();
                if (include_point_candidates && !current.planar && !points.empty()) {
                    if (!current.point_bucket) current.point_bucket = std::make_unique<HybridPointBucket>();
                    current.point_bucket->minimum = points.front();
                    current.point_bucket->maximum = points.front();
                    current.point_bucket->points.reserve(points.size());
                    for (std::size_t i = 0; i < points.size(); ++i) {
                        current.point_bucket->minimum = current.point_bucket->minimum.cwiseMin(points[i]);
                        current.point_bucket->maximum = current.point_bucket->maximum.cwiseMax(points[i]);
                        current.point_bucket->points.push_back(HybridMapPoint{points[i], covariances[i]});
                    }
                } else current.point_bucket.reset();
                return;
            }
            current.point_bucket.reset();
            std::array<std::vector<std::size_t>, 8> partition;
            float middle[3];
            for (int axis = 0; axis < 3; ++axis)
                middle[axis] = 0.5f * (current_box.vertex_min[axis] + current_box.vertex_max[axis]);
            // Stable filtering retains sorted order and full point multiplicity.
            for (const auto index : indices) {
                const auto& point = retained[references[index].index];
                const int child = (point.x >= middle[0] ? 4 : 0) +
                    (point.y >= middle[1] ? 2 : 0) + (point.z >= middle[2] ? 1 : 0);
                partition[child].push_back(index);
            }
            for (int child = 0; child < 8; ++child) {
                if (partition[child].empty()) { current.children[child].reset(); continue; }
                if (!current.children[child]) current.children[child] = std::make_unique<PlaneNode>();
                BoxPointType child_box = current_box;
                for (int axis = 0; axis < 3; ++axis) {
                    if (child & (4 >> axis)) child_box.vertex_min[axis] = middle[axis];
                    else child_box.vertex_max[axis] = middle[axis];
                }
                self(self, *current.children[child], partition[child], child_box, depth + 1);
            }
        };
        refresh(refresh, node, root_indices, box, level);
    }

    void compactCovarianceStorage() {
        // Voxel replacements leave inactive metadata until this bounded sweep.
        // The ikd tree remains the only geometry owner and authoritative set.
        const std::size_t active = static_cast<std::size_t>(std::max(0, tree_ptr_->validnum()));
        if (point_covariances_.size() <= 2 * active) return;
        KDPointVector retained;
        tree_ptr_->Snapshot(retained);
        CovarianceStorage current;
        current.reserve(retained.size());
        for (const auto& point : retained) {
            const auto found = point_covariances_.find(pointKey(point));
            if (found != point_covariances_.end()) current.emplace(found->first, found->second);
        }
        point_covariances_.swap(current);
        purgeEmptyPlaneRoots();
    }

    void enforcePointLimit() {
        if (config_.max_map_points == 0) return;
        const int valid = tree_ptr_->validnum();
        if (valid >= 0 && static_cast<std::size_t>(valid) <= config_.max_map_points) return;
        // Only a budget overflow needs a temporary pruning workspace. Keep
        // the nearest active geometry in the same tree; normal ikd rebuilds
        // reclaim deleted nodes without replacing the map owner.
        KDPointVector points;
        tree_ptr_->Snapshot(points);
        if (points.size() <= config_.max_map_points) return;
        const auto distance = [this](const PointType& point) {
            return (Eigen::Vector3d(point.x, point.y, point.z) - current_position_).squaredNorm();
        };
        const auto retained_end = points.begin() + config_.max_map_points;
        std::nth_element(points.begin(), retained_end, points.end(),
            [&](const PointType& lhs, const PointType& rhs) { return distance(lhs) < distance(rhs); });
        points.erase(points.begin(), retained_end);
        tree_ptr_->Delete_Points(points);
        for (const auto& point : points) {
            point_covariances_.erase(pointKey(point));
            dirtyRoot(point);
        }
        purgeEmptyPlaneRoots();
        nearest_cache_.clear();
    }

    const NeighborCacheEntry& nearest(const PointType& point, int k) const {
        const std::uint64_t cache_key = key(point);
        const auto it = nearest_cache_.find(cache_key);
        if (it != nearest_cache_.end() && static_cast<int>(it->second.points.size()) >= k) {
            return it->second;
        }

        auto& out = nearest_cache_[cache_key];
        tree_ptr_->Nearest_Search(point, k, out.points, out.distance_sq);
        return out;
    }

    static PointType voxelCenter(const PointType& point, double size, double half) {
        PointType center;
        center.x = static_cast<float>(std::floor(point.x / size) * size + half);
        center.y = static_cast<float>(std::floor(point.y / size) * size + half);
        center.z = static_cast<float>(std::floor(point.z / size) * size + half);
        return center;
    }

    std::uint64_t key(const PointType& p) const {
        const int axis_bits = std::max(1, std::min(21, config_.cache_key_axis_bits));
        const std::uint64_t axis_mask = (1ULL << axis_bits) - 1ULL;
        const int shift_y = axis_bits;
        const int shift_x = axis_bits * 2;

        const double q = (config_.cache_quantization_scale > 0.0) ? config_.cache_quantization_scale : 100.0;
        const int64_t xi = static_cast<int64_t>(std::llround(static_cast<double>(p.x) * q));
        const int64_t yi = static_cast<int64_t>(std::llround(static_cast<double>(p.y) * q));
        const int64_t zi = static_cast<int64_t>(std::llround(static_cast<double>(p.z) * q));
        const std::uint64_t ux = static_cast<std::uint64_t>(xi) & axis_mask;
        const std::uint64_t uy = static_cast<std::uint64_t>(yi) & axis_mask;
        const std::uint64_t uz = static_cast<std::uint64_t>(zi) & axis_mask;
        return (ux << shift_x) | (uy << shift_y) | uz;
    }

    static float dist2(const PointType& p1, const PointType& p2) {
        return (p1.x - p2.x) * (p1.x - p2.x) +
               (p1.y - p2.y) * (p1.y - p2.y) +
               (p1.z - p2.z) * (p1.z - p2.z);
    }

    mutable std::mutex mutex_;
    Config config_;
    std::unique_ptr<KDTreeType> owned_tree_;
    KDTreeType* tree_ptr_ = nullptr;
    bool initialized_ = false;
    double map_resolution_;
    bool localmap_initialized_ = false;
    BoxPointType local_map_points_{};
    double cube_len_;
    double det_range_;
    double mov_threshold_;
    Eigen::Vector3d current_position_ = Eigen::Vector3d::Zero();
    mutable std::unordered_map<std::uint64_t, NeighborCacheEntry> nearest_cache_;
    CovarianceStorage point_covariances_;
};

} // namespace lingtu::ltlio::detail

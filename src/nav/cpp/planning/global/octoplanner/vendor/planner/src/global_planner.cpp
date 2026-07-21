#include "global_planner.h"

#include <array>
#include <cmath>
#include <limits>
#include <queue>

namespace global_planner
{

    OctoPlanner3D::OctoPlanner3D()
    {
        printf("OctoPlanner3D constructed\n");
    }

    OctoPlanner3D::~OctoPlanner3D()
    {
        printf("OctoPlanner3D destroyed\n");
    }

    void OctoPlanner3D::setConfig(const PlannerConfig & config)
    {
        robot_radius_ = config.robot_radius;
        body_clearance_below_m_ = config.body_clearance_below_m;
        body_clearance_above_m_ = config.body_clearance_above_m;
        max_iterations_ = config.max_iterations;
        snap_search_radius_cells_ = config.snap_search_radius_cells;
        require_ground_support_ = config.require_ground_support;
        strict_direct_ground_support_ = config.strict_direct_ground_support;
        ground_support_xy_radius_cells_ = config.ground_support_xy_radius_cells;
        ground_support_depth_cells_ = config.ground_support_depth_cells;
        support_height_m_ = config.support_height_m;
        support_height_tolerance_m_ = config.support_height_tolerance_m;
        support_patch_radius_cells_ = config.support_patch_radius_cells;
        support_patch_min_samples_ = config.support_patch_min_samples;
        enable_preblocked_costmap_ = config.enable_preblocked_costmap;
        preblocked_costmap_radius_cells_ = config.preblocked_costmap_radius_cells;
        preblocked_costmap_weight_ = config.preblocked_costmap_weight;
        lowest_traversable_only_ = config.lowest_traversable_only;
        floor_change_penalty_ = config.floor_change_penalty;
        max_step_height_ = config.max_step_height;
        max_slope_ = config.max_slope;
        same_floor_preference_ = config.same_floor_preference;
        same_floor_z_tolerance_ = config.same_floor_z_tolerance;
        obstacle_clearance_radius_cells_ = config.obstacle_clearance_radius_cells;
        obstacle_clearance_weight_ = config.obstacle_clearance_weight;
    }

    void OctoPlanner3D::setCancelCheck(std::function<bool()> cancel_check)
    {
        cancel_check_ = std::move(cancel_check);
    }

    void OctoPlanner3D::setOctomap(std::shared_ptr<octomap::OcTree> map)
    {
        if (!map)
        {
            printf("Octomap is Null!!! return.\n");
            return;
        }

        if (octree_ == map)
        {
            printf("Octomap is No Update!!! return.\n");
            return;
        }

        octree_ = map;
        map_ready_ = true;
        rebuildPreblockedCells();
        rebuildDerivedLayers();
        rebuildPreblockedCostmap();
        rebuildObstacleClearanceCostmap();
    }

    void OctoPlanner3D::makePlan(const PointPose start,const PointPose goal)
    {
        start_point_ = start;
        has_start_ = true;

        goal_point_ = goal;
        has_goal_ = true;

        printf("start = (%f,%f,%f),goal = (%f,%f,%f) \n",start_point_.x,start_point_.y,start_point_.z,goal_point_.x,goal_point_.y,goal_point_.z);

        tryPlan();

    }

    void OctoPlanner3D::tryPlan()
    {
        printf("OctoPlanner3D::tryPlan planning...\n");
        if (!map_ready_ || !has_start_ || !has_goal_ || planning_in_progress_)
        {
            printf("OctoPlanner3D::tryPlan prerequisites not ready\n");
            return;
        }
        planning_in_progress_ = true;
        const bool ok = startPlan();
        planning_in_progress_ = false;
        if (!ok)
        {
            printf("OctoPlanner3D::tryPlan A* planning failed\n");
        }
    }

    void OctoPlanner3D::getPlannerResults(std::vector<PointPose>& plannerResults)
    {
       plannerResults = planner_results_;
    }

    bool OctoPlanner3D::resolvePlanEndpoints(GridIndex & start, GridIndex & goal) const
    {
        const GridIndex start_raw = worldToGrid(
            start_point_.x, start_point_.y, start_point_.z);
        const GridIndex goal_raw = worldToGrid(
            goal_point_.x, goal_point_.y, goal_point_.z);
        start = start_raw;
        goal = goal_raw;

        if (!findNearestFreeCell(
                start_raw,
                robot_radius_,
                snap_search_radius_cells_,
                require_ground_support_,
                strict_direct_ground_support_,
                ground_support_xy_radius_cells_,
                ground_support_depth_cells_,
                start)) {
            printf("OctoPlanner3D::startPlan start is occupied/out of map and no nearby free cell\n");
            return false;
        }
        if (!findNearestFreeCell(
                goal_raw,
                robot_radius_,
                snap_search_radius_cells_,
                require_ground_support_,
                strict_direct_ground_support_,
                ground_support_xy_radius_cells_,
                ground_support_depth_cells_,
                goal)) {
            printf("OctoPlanner3D::startPlan goal is occupied/out of map and no nearby free cell\n");
            return false;
        }

        if (!(start == start_raw)) {
            const auto point = gridToWorld(start);
            printf(
                "OctoPlanner3D::startPlan start snapped to free cell: [%.2f, %.2f, %.2f]\n",
                point.x(), point.y(), point.z());
        }
        if (!(goal == goal_raw)) {
            const auto point = gridToWorld(goal);
            printf(
                "OctoPlanner3D::startPlan goal snapped to free cell: [%.2f, %.2f, %.2f]\n",
                point.x(), point.y(), point.z());
        }
        return true;
    }

    bool OctoPlanner3D::startPlan()
    {
        planner_results_.clear();
        const double robot_radius = robot_radius_;
        const int max_iterations = max_iterations_;
        const bool require_ground_support = require_ground_support_;
        const bool strict_direct_ground_support = strict_direct_ground_support_;
        const int support_xy_radius_cells =  ground_support_xy_radius_cells_;
        const int support_depth_cells = ground_support_depth_cells_;
        const bool enable_preblocked_costmap =  enable_preblocked_costmap_;
        const double preblocked_costmap_weight = preblocked_costmap_weight_;
        const double floor_change_penalty = floor_change_penalty_;
        const bool same_floor_preference = same_floor_preference_;
        const double same_floor_z_tolerance = same_floor_z_tolerance_;
        const int obstacle_clearance_radius_cells = obstacle_clearance_radius_cells_;
        const double obstacle_clearance_weight = obstacle_clearance_weight_;
        const double resolution = octree_->getResolution();

        GridIndex start{};
        GridIndex goal{};
        if (!resolvePlanEndpoints(start, goal)) {
            return false;
        }
        const bool same_floor_request =
            same_floor_preference &&
            std::abs(static_cast<double>(goal.z - start.z) * resolution) <= same_floor_z_tolerance;

        std::priority_queue<QueueNode, std::vector<QueueNode>, QueueNodeCompare> open_set;
        std::unordered_map<GridIndex, double, GridIndexHash> g_score;
        std::unordered_map<GridIndex, GridIndex, GridIndexHash> came_from;
        std::unordered_set<GridIndex, GridIndexHash> closed_set;
        std::unordered_map<GridIndex, TraversabilityFailure, GridIndexHash>
            planning_cell_cache;
        std::unordered_map<GridIndex, TraversabilityFailure, GridIndexHash>
            motion_cell_cache;
        planning_cell_cache.reserve(65536);
        motion_cell_cache.reserve(32768);

        g_score[start] = 0.0;
        open_set.push(QueueNode{start, euclidean(start, goal), 0.0});

        const std::vector<GridIndex> directions = make26Directions();
        int iters = 0;
        int rejected_closed = 0;
        int rejected_outside = 0;
        int rejected_ground = 0;
        int rejected_preblocked_below = 0;
        int rejected_preblocked_body = 0;
        int rejected_occupied_body = 0;
        int rejected_motion = 0;
        int planning_cache_hits = 0;
        int motion_cache_hits = 0;

        const auto planningCellTraversable = [&](const GridIndex & idx, TraversabilityFailure & failure) {
            const auto cached = planning_cell_cache.find(idx);
            if (cached != planning_cell_cache.end()) {
                ++planning_cache_hits;
                failure = cached->second;
                return failure == TraversabilityFailure::None;
            }
            TraversabilityFailure evaluated = TraversabilityFailure::None;
            const bool traversable = isPlanningCellTraversableDetailed(
                idx, robot_radius, require_ground_support, strict_direct_ground_support,
                support_xy_radius_cells, support_depth_cells, &evaluated);
            planning_cell_cache.emplace(idx, evaluated);
            failure = evaluated;
            return traversable;
        };
        const auto motionCellTraversable = [&](const GridIndex & idx) {
            const auto cached = motion_cell_cache.find(idx);
            if (cached != motion_cell_cache.end()) {
                ++motion_cache_hits;
                return cached->second == TraversabilityFailure::None;
            }
            TraversabilityFailure evaluated = TraversabilityFailure::None;
            const bool traversable = isCellTraversableDetailed(
                idx, robot_radius, false, strict_direct_ground_support,
                support_xy_radius_cells, support_depth_cells, &evaluated);
            motion_cell_cache.emplace(idx, evaluated);
            return traversable;
        };

        while (!open_set.empty() && iters < max_iterations)
        {
            if (cancel_check_ && cancel_check_()) {
                planner_results_.clear();
                return false;
            }
            const QueueNode current = open_set.top();
            open_set.pop();
            const auto best_g = g_score.find(current.idx);
            if (best_g == g_score.end() || current.g > best_g->second + 1e-12) {
                continue;
            }
            ++iters;

            if (closed_set.find(current.idx) != closed_set.end()) {
                continue;
            }
            closed_set.insert(current.idx);

            if (current.idx == goal) {
                const auto cells = reconstructPath(came_from, current.idx);
                printf("OctoPlanner3D::startPlan A* path found in %d iterations, waypoints=%zu\n", iters, cells.size());
                planner_results_.clear();
                for (std::size_t i = 0; i < cells.size(); ++i)
                {
                    const auto & c = cells[i];
                    const auto p = gridToWorld(c);
                    PointPose temp;
                    temp.x = p.x();
                    temp.y = p.y();
                    temp.z = p.z();
                    planner_results_.push_back(temp);
                }

                printf(
                    "OctoPlanner3D::startPlan cache diagnostics: planning_cells=%zu planning_hits=%d "
                    "motion_cells=%zu motion_hits=%d\n",
                    planning_cell_cache.size(),
                    planning_cache_hits,
                    motion_cell_cache.size(),
                    motion_cache_hits);

                return true;
            }

            for (const auto & d : directions)
            {
                GridIndex nbr{current.idx.x + d.x, current.idx.y + d.y, current.idx.z + d.z};
                if (closed_set.find(nbr) != closed_set.end()) {
                ++rejected_closed;
                continue;
                }
                TraversabilityFailure failure = TraversabilityFailure::None;
                if (!planningCellTraversable(nbr, failure))
                {
                switch (failure) {
                    case TraversabilityFailure::OutsideBounds:
                    ++rejected_outside;
                    break;
                    case TraversabilityFailure::GroundSupport:
                    ++rejected_ground;
                    break;
                    case TraversabilityFailure::ExternalPreblockedBelow:
                    ++rejected_preblocked_below;
                    break;
                    case TraversabilityFailure::ExternalPreblockedBody:
                    ++rejected_preblocked_body;
                    break;
                    case TraversabilityFailure::OccupiedBody:
                    ++rejected_occupied_body;
                    break;
                    case TraversabilityFailure::None:
                    default:
                    break;
                }
                continue;
                }
                if (!isMotionAllowed(current.idx, nbr)) {
                ++rejected_motion;
                continue;
                }
                const int motion_steps = std::max({std::abs(d.x), std::abs(d.y), std::abs(d.z)});
                bool blocked_intermediate = false;
                for (int step = 1; step < motion_steps; ++step) {
                    GridIndex mid{
                        current.idx.x + (d.x * step) / motion_steps,
                        current.idx.y + (d.y * step) / motion_steps,
                        current.idx.z + (d.z * step) / motion_steps};
                    if ((mid.x == current.idx.x && mid.y == current.idx.y && mid.z == current.idx.z) ||
                        (mid.x == nbr.x && mid.y == nbr.y && mid.z == nbr.z)) {
                    continue;
                    }
                    if (!motionCellTraversable(mid)) {
                    blocked_intermediate = true;
                    break;
                    }
                }
                if (blocked_intermediate) {
                ++rejected_motion;
                continue;
                }
                const double step_cost = euclidean(current.idx, nbr);
                double tentative_g = current.g + step_cost;
                if (enable_preblocked_costmap) {
                tentative_g += preblocked_costmap_weight * getPreblockedCost(nbr);
                }
                const double dz_m = std::abs(static_cast<double>(nbr.z - current.idx.z)) * resolution;
                tentative_g += floor_change_penalty * dz_m;
                if (same_floor_request) {
                tentative_g += floor_change_penalty * 0.5 *
                    std::abs(static_cast<double>(nbr.z - start.z)) * resolution;
                }
                if (obstacle_clearance_weight > 0.0 && obstacle_clearance_radius_cells > 0) {
                tentative_g += obstacle_clearance_weight * getObstacleClearanceCost(nbr);
                }

                auto g_it = g_score.find(nbr);
                if (g_it == g_score.end() || tentative_g < g_it->second) {
                came_from[nbr] = current.idx;
                g_score[nbr] = tentative_g;
                const double f = tentative_g + euclidean(nbr, goal);
                open_set.push(QueueNode{nbr, f, tentative_g});
                }
            }
        }

        printf(
            "OctoPlanner3D::startPlan A* failed diagnostics: iterations=%d closed=%zu open=%zu "
            "reject_closed=%d reject_outside=%d reject_ground=%d reject_external_preblocked_below=%d "
            "reject_external_preblocked_body=%d reject_occupied_body=%d reject_motion=%d\n",
            iters,
            closed_set.size(),
            open_set.size(),
            rejected_closed,
            rejected_outside,
            rejected_ground,
            rejected_preblocked_below,
            rejected_preblocked_body,
            rejected_occupied_body,
            rejected_motion);
        printf(
            "OctoPlanner3D::startPlan cache diagnostics: planning_cells=%zu planning_hits=%d "
            "motion_cells=%zu motion_hits=%d\n",
            planning_cell_cache.size(),
            planning_cache_hits,
            motion_cell_cache.size(),
            motion_cache_hits);
        return false;
    }

    std::vector<GridIndex> OctoPlanner3D::reconstructPath(const std::unordered_map<GridIndex, GridIndex, GridIndexHash> & came_from,GridIndex current) const
    {
        std::vector<GridIndex> path;
        path.push_back(current);
        while (came_from.find(current) != came_from.end())
        {
            current = came_from.at(current);
            path.push_back(current);
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    bool OctoPlanner3D::findNearestFreeCell(const GridIndex & seed, double robot_radius, int radius_cells, bool require_ground_support,bool strict_direct_ground_support, int support_xy_radius_cells, int support_depth_cells,GridIndex & out) const
    {
        if (isPlanningCellTraversableDetailed(
            seed, robot_radius, require_ground_support, strict_direct_ground_support,
            support_xy_radius_cells, support_depth_cells, nullptr))
        {
        out = seed;
        return true;
        }

        bool found = false;
        GridIndex best = seed;
        double best_score = std::numeric_limits<double>::infinity();
        double best_xy_score = std::numeric_limits<double>::infinity();
        double best_z_score = std::numeric_limits<double>::infinity();

        for (int dz = -radius_cells; dz <= radius_cells; ++dz) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
            if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) > radius_cells) {
            continue;
            }
            GridIndex candidate{seed.x + dx, seed.y + dy, seed.z + dz};
            if (!isPlanningCellTraversableDetailed(
                candidate, robot_radius, require_ground_support, strict_direct_ground_support,
                support_xy_radius_cells, support_depth_cells, nullptr))
            {
            continue;
            }
            const double xy_score =
                static_cast<double>(dx * dx + dy * dy);
            const double z_score = static_cast<double>(dz * dz);
            const double score = xy_score + z_score;
            if (!found ||
                score < best_score ||
                (score == best_score && xy_score < best_xy_score) ||
                (score == best_score && xy_score == best_xy_score && z_score < best_z_score))
            {
            found = true;
            best = candidate;
            best_score = score;
            best_xy_score = xy_score;
            best_z_score = z_score;
            }
        }
        }
        }
        if (found) {
        out = best;
        return true;
        }
        return false;
    }

    double OctoPlanner3D::getPreblockedCost(const GridIndex & idx) const
    {
        const auto it = preblocked_costmap_.find(idx);
        if (it == preblocked_costmap_.end()) {
        return 0.0;
        }
        return it->second;
    }

    double OctoPlanner3D::getObstacleClearanceCost(const GridIndex & idx) const
    {
        const auto it = obstacle_clearance_costmap_.find(idx);
        if (it == obstacle_clearance_costmap_.end()) {
        return 0.0;
        }
        return it->second;
    }

    bool OctoPlanner3D::isMotionAllowed(const GridIndex & from, const GridIndex & to) const
    {
        const double r = octree_->getResolution();
        const double dx = static_cast<double>(to.x - from.x) * r;
        const double dy = static_cast<double>(to.y - from.y) * r;
        const double dz = std::abs(static_cast<double>(to.z - from.z) * r);
        if (max_step_height_ > 0.0 && dz > max_step_height_) {
        return false;
        }
        const double dxy = std::sqrt(dx * dx + dy * dy);
        if (dz > 0.0 && dxy <= 1e-9) {
        return false;
        }
        if (max_slope_ <= 0.0 || dz <= 0.0) {
        return true;
        }
        return (dz / dxy) <= max_slope_;
    }

    std::vector<GridIndex> OctoPlanner3D::make26Directions() const
    {
        std::vector<GridIndex> dirs;
        dirs.reserve(26);
        const double r = octree_ ? octree_->getResolution() : 0.2;
        const int max_step_cells = std::max(
            1,
            static_cast<int>(std::floor(std::max(0.0, max_step_height_) / std::max(1e-6, r) + 1e-9)));
        const int max_stair_run_cells = std::max(
            2,
            static_cast<int>(std::ceil(static_cast<double>(max_step_cells) / std::max(0.1, max_slope_))) + 1);
        for (int dx = -1; dx <= 1; ++dx)
        {
            for (int dy = -1; dy <= 1; ++dy)
            {
                for (int dz = -1; dz <= 1; ++dz)
                {
                    if (dx == 0 && dy == 0 && dz == 0)
                    {
                        continue;
                    }
                    dirs.push_back(GridIndex{dx, dy, dz});
                }
            }
        }
        for (int dx = -max_stair_run_cells; dx <= max_stair_run_cells; ++dx)
        {
            for (int dy = -max_stair_run_cells; dy <= max_stair_run_cells; ++dy)
            {
                const int run_cells = std::max(std::abs(dx), std::abs(dy));
                if (run_cells < 2)
                {
                    continue;
                }
                for (int dz = 1; dz <= max_step_cells; ++dz)
                {
                    const int min_run_for_slope = max_slope_ > 0.0
                        ? static_cast<int>(std::ceil(static_cast<double>(dz) / max_slope_))
                        : 2;
                    if (run_cells < min_run_for_slope || run_cells > min_run_for_slope + 1)
                    {
                        continue;
                    }
                    dirs.push_back(GridIndex{dx, dy, -dz});
                    dirs.push_back(GridIndex{dx, dy, dz});
                }
            }
        }
        return dirs;
    }

    bool OctoPlanner3D::isCellTraversable(const GridIndex & idx, double robot_radius, bool require_ground_support,bool strict_direct_ground_support,int support_xy_radius_cells, int support_depth_cells) const
    {
        return isCellTraversableDetailed(
            idx,
            robot_radius,
            require_ground_support,
            strict_direct_ground_support,
            support_xy_radius_cells,
            support_depth_cells,
            nullptr);
    }

    bool OctoPlanner3D::isCellTraversableDetailed(const GridIndex & idx, double robot_radius, bool require_ground_support,bool strict_direct_ground_support,int support_xy_radius_cells, int support_depth_cells, TraversabilityFailure * failure) const
    {
        if (!isInsideMetricBounds(idx)) {
        if (failure) {
            *failure = TraversabilityFailure::OutsideBounds;
        }
        return false;
        }

        if (require_ground_support &&
        !hasGroundSupport(
            idx, strict_direct_ground_support, support_xy_radius_cells, support_depth_cells))
        {
        if (failure) {
            *failure = TraversabilityFailure::GroundSupport;
        }
        return false;
        }
        if (require_ground_support && strict_direct_ground_support &&
            !hasFootprintGroundSupport(idx, robot_radius, support_depth_cells))
        {
        if (failure) {
            *failure = TraversabilityFailure::GroundSupport;
        }
        return false;
        }

        for (int z = idx.z - 1; z >= 0; --z) {
        const GridIndex below_idx{idx.x, idx.y, z};
        if (isOccupiedCell(below_idx)) {
            break;
        }
        if (external_preblocked_cells_.find(below_idx) != external_preblocked_cells_.end()) {
            if (failure) {
            *failure = TraversabilityFailure::ExternalPreblockedBelow;
            }
            return false;
        }
        }

        const octomap::point3d center = gridToWorld(idx);
        const double r = octree_->getResolution();
        const int n = std::max(1, static_cast<int>(std::ceil(robot_radius / r)));
        const double radius_sq = robot_radius * robot_radius;
        const bool use_body_cylinder =
            body_clearance_below_m_ > 0.0 || body_clearance_above_m_ > 0.0;
        const int minimum_dz = use_body_cylinder
            ? -std::max(0, static_cast<int>(std::floor(body_clearance_below_m_ / r + 1e-9)))
            : 0;
        const int maximum_dz = use_body_cylinder
            ? std::max(0, static_cast<int>(std::ceil(body_clearance_above_m_ / r - 1e-9)))
            : n;

        // Collision check for vehicle body volume (same height and above),
        // while allowing occupied support cells below. Apply the same footprint
        // rule to preblocked cells so a cell is rejected if the vehicle radius
        // overlaps any preblocked voxel.
        for (int dx = -n; dx <= n; ++dx) {
        for (int dy = -n; dy <= n; ++dy) {
            for (int dz = minimum_dz; dz <= maximum_dz; ++dz) {
            const double dist_x = static_cast<double>(dx) * r;
            const double dist_y = static_cast<double>(dy) * r;
            const double dist_z = static_cast<double>(dz) * r;
            const double dist_sq = dist_x * dist_x + dist_y * dist_y +
                (use_body_cylinder ? 0.0 : dist_z * dist_z);
            if (dist_sq > radius_sq) {
                continue;
            }
            const octomap::point3d p(
                center.x() + static_cast<float>(dx * r),
                center.y() + static_cast<float>(dy * r),
                center.z() + static_cast<float>(dz * r));
            const GridIndex nearby_idx = worldToGrid(p.x(), p.y(), p.z());
            // Map-derived preblocked cells are a soft risk layer and are scored
            // through preblocked_costmap_. Only explicit external preblocked
            // cells are hard forbidden; otherwise noisy LiDAR walls can close
            // valid doors and corridors.
            if (external_preblocked_cells_.find(nearby_idx) != external_preblocked_cells_.end()) {
                if (failure) {
                *failure = TraversabilityFailure::ExternalPreblockedBody;
                }
                return false;
            }
            const octomap::OcTreeNode * node = octree_->search(p);
            if (node && octree_->isNodeOccupied(node)) {
                if (failure) {
                *failure = TraversabilityFailure::OccupiedBody;
                }
                return false;
            }
            }
        }
        }
        if (failure) {
        *failure = TraversabilityFailure::None;
        }
        return true;
    }

    bool OctoPlanner3D::isPlanningCellTraversableDetailed(
        const GridIndex & idx,
        double robot_radius,
        bool require_ground_support,
        bool strict_direct_ground_support,
        int support_xy_radius_cells,
        int support_depth_cells,
        TraversabilityFailure * failure) const
    {
        if (lowest_traversable_only_) {
            const bool traversable = traversable_cells_.find(idx) != traversable_cells_.end();
            if (failure) {
                *failure = traversable
                    ? TraversabilityFailure::None
                    : TraversabilityFailure::GroundSupport;
            }
            return traversable;
        }
        return isCellTraversableDetailed(
            idx,
            robot_radius,
            require_ground_support,
            strict_direct_ground_support,
            support_xy_radius_cells,
            support_depth_cells,
            failure);
    }

    std::pair<int, int> OctoPlanner3D::supportDepthRange(int support_depth_cells) const
    {
        const int fallback_max = std::max(1, support_depth_cells);
        if (!octree_ || support_height_m_ <= 0.0) {
            return {1, fallback_max};
        }

        const double resolution = std::max(1e-6, octree_->getResolution());
        const double tolerance = std::max(0.0, support_height_tolerance_m_);
        const double minimum_height = std::max(0.0, support_height_m_ - tolerance);
        const double maximum_height = support_height_m_ + tolerance;
        int minimum_depth = std::max(
            1,
            static_cast<int>(std::ceil(minimum_height / resolution - 1e-9)));
        int maximum_depth = std::max(
            1,
            static_cast<int>(std::floor(maximum_height / resolution + 1e-9)));
        if (maximum_depth < minimum_depth) {
            const int nearest_depth = std::max(
                1,
                static_cast<int>(std::lround(support_height_m_ / resolution)));
            minimum_depth = nearest_depth;
            maximum_depth = nearest_depth;
        }
        return {minimum_depth, maximum_depth};
    }

    bool OctoPlanner3D::hasOccupiedNearZ(
        const GridIndex & index,
        int z_tolerance_cells) const
    {
        const int tolerance = std::max(0, z_tolerance_cells);
        for (int dz = -tolerance; dz <= tolerance; ++dz) {
            const GridIndex candidate{index.x, index.y, index.z + dz};
            if (isInsideMetricBounds(candidate) && isOccupiedCell(candidate)) {
                return true;
            }
        }
        return false;
    }

    bool OctoPlanner3D::isSupportPatch(const GridIndex & support) const
    {
        const int radius = std::max(0, support_patch_radius_cells_);
        const int required = std::max(0, support_patch_min_samples_);
        if (radius == 0 || required <= 1) {
            return true;
        }

        const std::array<GridIndex, 5> samples{{
            support,
            GridIndex{support.x + radius, support.y, support.z},
            GridIndex{support.x - radius, support.y, support.z},
            GridIndex{support.x, support.y + radius, support.z},
            GridIndex{support.x, support.y - radius, support.z},
        }};
        int occupied_samples = 0;
        for (const auto & sample : samples) {
            if (hasOccupiedNearZ(sample, 1)) {
                ++occupied_samples;
            }
        }
        return occupied_samples >= std::min<int>(required, samples.size());
    }

    bool OctoPlanner3D::isSupportCell(const GridIndex & support) const
    {
        if (!isInsideMetricBounds(support)) {
            return false;
        }
        const auto point = gridToWorld(support);
        const octomap::OcTreeNode * node = octree_->search(point);
        if (!node || !octree_->isNodeOccupied(node)) {
            return false;
        }
        return isSupportPatch(support);
    }

    bool OctoPlanner3D::hasGroundSupport(const GridIndex & idx, bool strict_direct_ground_support, int support_xy_radius_cells,int support_depth_cells) const
    {
        const auto [minimum_depth, maximum_depth] = supportDepthRange(support_depth_cells);
        if (strict_direct_ground_support) {
        for (int dz = minimum_depth; dz <= maximum_depth; ++dz) {
            GridIndex below{idx.x, idx.y, idx.z - dz};
            if (!isInsideMetricBounds(below)) {
                continue;
            }
            if (isSupportCell(below)) {
                return true;
            }
        }
        return false;
        }

        for (int dz = minimum_depth; dz <= maximum_depth; ++dz) {
        for (int dx = -support_xy_radius_cells; dx <= support_xy_radius_cells; ++dx) {
            for (int dy = -support_xy_radius_cells; dy <= support_xy_radius_cells; ++dy) {
            GridIndex below{idx.x + dx, idx.y + dy, idx.z - dz};
            if (!isInsideMetricBounds(below)) {
                continue;
            }
            if (isSupportCell(below)) {
                return true;
            }
            }
        }
        }
        return false;
    }

    bool OctoPlanner3D::hasFootprintGroundSupport(
        const GridIndex & idx,
        double robot_radius,
        int support_depth_cells) const
    {
        if (!octree_) {
        return false;
        }
        const double r = octree_->getResolution();
        const auto [minimum_depth, maximum_depth] = supportDepthRange(support_depth_cells);
        const int n = std::max(1, static_cast<int>(std::ceil(robot_radius / std::max(1e-6, r))));
        const std::array<GridIndex, 5> samples{{
            GridIndex{idx.x, idx.y, idx.z},
            GridIndex{idx.x + n, idx.y, idx.z},
            GridIndex{idx.x - n, idx.y, idx.z},
            GridIndex{idx.x, idx.y + n, idx.z},
            GridIndex{idx.x, idx.y - n, idx.z},
        }};
        for (const auto & sample : samples) {
        bool supported = false;
        for (int dz = minimum_depth; dz <= maximum_depth; ++dz) {
            const GridIndex below{sample.x, sample.y, sample.z - dz};
            if (!isInsideMetricBounds(below)) {
            continue;
            }
            if (isSupportCell(below)) {
            supported = true;
            break;
            }
        }
        if (!supported) {
            return false;
        }
        }
        return true;
    }

    void OctoPlanner3D::rebuildPreblockedCells()
    {
        preblocked_cells_.clear();
        if (!octree_) {
        return;
        }

        std::unordered_set<GridIndex, GridIndexHash> candidates;
        for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
        if (!octree_->isNodeOccupied(*it)) {
            continue;
        }
        const GridIndex occ = worldToGrid(it.getX(), it.getY(), it.getZ());
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) {
                continue;
            }
            candidates.insert(GridIndex{occ.x + dx, occ.y + dy, occ.z});
            }
        }
        }

        for (const auto & c : candidates) {
        if (!isInsideMetricBounds(c)) {
            continue;
        }
        if (isOccupiedCell(c)) {
            continue;
        }
        const GridIndex below0{c.x, c.y, c.z - 1};
        const bool below0_occ = isInsideMetricBounds(below0) && isOccupiedCell(below0);
        if (below0_occ && hasSameLevelNeighborWithOccupiedAbove(c)) {
            preblocked_cells_.insert(c);
            continue;
        }
        const GridIndex above1{c.x, c.y, c.z + 1};
        const bool above1_occ = isInsideMetricBounds(above1) && isOccupiedCell(above1);
        if (!hasNonOccupiedNeighborSameLevel(c)) {
            continue;
        }
        if (above1_occ) {
            continue;
        }
        const GridIndex below1{c.x, c.y, c.z - 1};
        if (!isInsideMetricBounds(below1)) {
            continue;
        }
        const bool below1_non_occupied = !isOccupiedCell(below1);
        if (below1_non_occupied) {
            preblocked_cells_.insert(c);
        }
        }

        for (const auto & c : external_preblocked_cells_) {
        if (isInsideMetricBounds(c) && !isOccupiedCell(c)) {
            preblocked_cells_.insert(c);
        }
        }
        printf("Preprocess mask rebuilt. preblocked_cells=%zu external=%zu \n",preblocked_cells_.size(), external_preblocked_cells_.size());
    }

    bool OctoPlanner3D::hasSameLevelNeighborWithOccupiedAbove(const GridIndex & idx) const
    {
        for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) {
            continue;
            }
            const GridIndex n{idx.x + dx, idx.y + dy, idx.z};
            if (!isInsideMetricBounds(n)) {
            continue;
            }
            const GridIndex n_above1{n.x, n.y, n.z + 1};
            if (!isInsideMetricBounds(n_above1)) {
            continue;
            }
            if (isOccupiedCell(n_above1)) {
            return true;
            }
        }
        }
        return false;
    }

    bool OctoPlanner3D::hasNonOccupiedNeighborSameLevel(const GridIndex & idx) const
    {
        for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) {
            continue;
            }
            const GridIndex n{idx.x + dx, idx.y + dy, idx.z};
            if (!isInsideMetricBounds(n)) {
            continue;
            }
            if (!isOccupiedCell(n)) {
            return true;
            }
        }
        }
        return false;
    }

    void OctoPlanner3D::rebuildDerivedLayers()
    {
        traversable_cells_.clear();
        if (!octree_) {
        return;
        }

        const bool require_ground_support = require_ground_support_;
        const bool strict_direct_ground_support = strict_direct_ground_support_;
        const int support_xy_radius_cells = ground_support_xy_radius_cells_;
        const int support_depth_cells = ground_support_depth_cells_;
        const double robot_radius = robot_radius_;
        const bool lowest_traversable_only = lowest_traversable_only_;

        double min_x, min_y, min_z, max_x, max_y, max_z;
        octree_->getMetricMin(min_x, min_y, min_z);
        octree_->getMetricMax(max_x, max_y, max_z);
        const GridIndex min_idx = worldToGrid(min_x, min_y, min_z);
        const GridIndex max_idx = worldToGrid(max_x, max_y, max_z);

        for (int x = min_idx.x; x <= max_idx.x; ++x) {
        for (int y = min_idx.y; y <= max_idx.y; ++y) {
            for (int z = min_idx.z; z <= max_idx.z; ++z) {
            const GridIndex idx{x, y, z};
            if (!isInsideMetricBounds(idx) || isOccupiedCell(idx)) {
                continue;
            }
            if (isCellTraversable(
                idx, robot_radius, require_ground_support, strict_direct_ground_support,
                support_xy_radius_cells, support_depth_cells))
            {
                traversable_cells_.insert(idx);
                if (lowest_traversable_only) {
                break;
                }
            }
            }
        }
        }

        // publishCellSetMarker(
        // traversable_cells_, traversable_marker_pub_, "traversable_cells", 0.20F, 0.95F, 0.55F,
        // 0.55F);
    }

    bool OctoPlanner3D::isInsideMetricBounds(const GridIndex & idx) const
    {
        double min_x, min_y, min_z, max_x, max_y, max_z;
        octree_->getMetricMin(min_x, min_y, min_z);
        octree_->getMetricMax(max_x, max_y, max_z);
        const auto p = gridToWorld(idx);
        return p.x() >= static_cast<float>(min_x) && p.x() <= static_cast<float>(max_x) &&
            p.y() >= static_cast<float>(min_y) && p.y() <= static_cast<float>(max_y) &&
            p.z() >= static_cast<float>(min_z) && p.z() <= static_cast<float>(max_z);
    }

    bool OctoPlanner3D::isOccupiedCell(const GridIndex & idx) const
    {
        if (!isInsideMetricBounds(idx)) {
        return false;
        }
        const auto p = gridToWorld(idx);
        const octomap::OcTreeNode * node = octree_->search(p);
        return node && octree_->isNodeOccupied(node);
    }

    GridIndex OctoPlanner3D::worldToGrid(double x, double y, double z) const
    {
        const double r = octree_->getResolution();
        return GridIndex{
        static_cast<int>(std::floor(x / r)),
        static_cast<int>(std::floor(y / r)),
        static_cast<int>(std::floor(z / r))};
    }

    octomap::point3d OctoPlanner3D::gridToWorld(const GridIndex & idx) const
    {
        const double r = octree_->getResolution();
        return octomap::point3d(
        static_cast<float>((static_cast<double>(idx.x) + 0.5) * r),
        static_cast<float>((static_cast<double>(idx.y) + 0.5) * r),
        static_cast<float>((static_cast<double>(idx.z) + 0.5) * r));
    }

    void OctoPlanner3D::rebuildPreblockedCostmap()
    {
        preblocked_costmap_.clear();
        if (!octree_) {
        return;
        }
        const bool enable = enable_preblocked_costmap_;
        if (!enable) {
        return;
        }

        const int radius_cells = std::max(
        1, static_cast<int>(preblocked_costmap_radius_cells_));
        const double denom = static_cast<double>(radius_cells) + 1.0;

        for (const auto & c : preblocked_cells_) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
            for (int dz = -radius_cells; dz <= radius_cells; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) {
                continue;
                }
                const GridIndex n{c.x + dx, c.y + dy, c.z + dz};
                if (!isInsideMetricBounds(n)) {
                continue;
                }
                if (traversable_cells_.find(n) == traversable_cells_.end()) {
                continue;
                }
                if (preblocked_cells_.find(n) != preblocked_cells_.end()) {
                continue;
                }
                const double d = std::sqrt(
                static_cast<double>(dx * dx + dy * dy + dz * dz));
                if (d > static_cast<double>(radius_cells)) {
                continue;
                }
                const double cst = std::max(0.0, (denom - d) / denom);
                auto it = preblocked_costmap_.find(n);
                if (it == preblocked_costmap_.end() || cst > it->second) {
                preblocked_costmap_[n] = cst;
                }
            }
            }
        }
        }

        // RCLCPP_INFO(
        // get_logger(),
        // "Preblocked costmap rebuilt. cells=%zu radius=%d",
        // preblocked_costmap_.size(), radius_cells);
        printf("Preblocked costmap rebuilt. cells=%zu radius=%d \n",preblocked_costmap_.size(),radius_cells);
    }

    void OctoPlanner3D::rebuildObstacleClearanceCostmap()
    {
        obstacle_clearance_costmap_.clear();
        if (!octree_ || obstacle_clearance_radius_cells_ <= 0) {
        return;
        }

        const int radius_cells = std::max(1, obstacle_clearance_radius_cells_);
        const double denom = static_cast<double>(radius_cells) + 1.0;

        for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
        if (!octree_->isNodeOccupied(*it)) {
            continue;
        }
        const GridIndex occ = worldToGrid(it.getX(), it.getY(), it.getZ());
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
            for (int dz = -radius_cells; dz <= radius_cells; ++dz) {
                const GridIndex n{occ.x + dx, occ.y + dy, occ.z + dz};
                if (!isInsideMetricBounds(n)) {
                continue;
                }
                if (traversable_cells_.find(n) == traversable_cells_.end()) {
                continue;
                }
                if (isOccupiedCell(n)) {
                continue;
                }
                const double d = std::sqrt(
                static_cast<double>(dx * dx + dy * dy + dz * dz));
                if (d > static_cast<double>(radius_cells)) {
                continue;
                }
                const double cost = std::max(0.0, (denom - d) / denom);
                auto cost_it = obstacle_clearance_costmap_.find(n);
                if (cost_it == obstacle_clearance_costmap_.end() || cost > cost_it->second) {
                obstacle_clearance_costmap_[n] = cost;
                }
            }
            }
        }
        }

        printf("Obstacle clearance costmap rebuilt. cells=%zu radius=%d \n",obstacle_clearance_costmap_.size(),radius_cells);
    }







}

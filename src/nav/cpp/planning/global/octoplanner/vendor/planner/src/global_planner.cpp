#include "global_planner.h"
#include "surface_graph.hpp"
#include "route_smoothing.hpp"
#include "lingtu/maps/io/binary.hpp"
#include <chrono>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>

namespace global_planner
{
    namespace
    {
        // OctoMap exposes point coordinates as floats; preserve inclusive
        // metric bounds when those coordinates are compared with doubles.
        constexpr double kMetricToleranceEpsilonM = 1e-6;
    }

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
        planning_resolution_m_ = 0.0;
        const bool surface_changed = surface_closing_radius_m_ != config.surface_closing_radius_m ||
            graph_config_.surface_max_hole_area_m2 != config.surface_max_hole_area_m2 ||
            graph_config_.surface_max_slope != config.surface_max_slope ||
            support_height_m_ != config.support_height_m || body_clearance_above_m_ != config.body_clearance_above_m ||
            ground_support_ != config.ground_support || direct_support_only_ != config.direct_support_only;
        graph_config_ = config;
        surface_closing_radius_m_ = config.surface_closing_radius_m;
        search_resolution_m_ = config.search_resolution_m;
        robot_radius_ = config.robot_radius;
        body_clearance_below_m_ = config.body_clearance_below_m;
        body_clearance_above_m_ = config.body_clearance_above_m;
        ground_support_ = config.ground_support;
        direct_support_only_ = config.direct_support_only;
        support_depth_cells_ = config.support_depth_cells;
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
        obstacle_clearance_radius_cells_ = config.obstacle_clearance_radius_cells;
        obstacle_clearance_weight_ = config.obstacle_clearance_weight;
        setQuery(config);
        rebuildQueryOffsets();
        if (octree_) {
            external_preblocked_cells_.clear();
            if (surface_changed) rebuildSurfaceIndex();
            clearMapQueryCaches();
            rebuildSurfaceGraph();
            rebuildExternalPreblockedCells();
            rebuildDerivedLayers();
        }
    }

    void OctoPlanner3D::setCancelCheck(std::function<bool()> cancel_check)
    {
        cancel_check_ = std::move(cancel_check);
    }

    void OctoPlanner3D::setQuery(const PlannerConfig &config)
    {
        max_iterations_=config.max_iterations;
        snap_search_radius_cells_=config.snap_search_radius_cells;
        terminal_goal_tolerance_m_=config.terminal_goal_tolerance_m;
        terminal_goal_xy_tolerance_m_=config.terminal_goal_xy_tolerance_m;
        terminal_goal_z_tolerance_m_=config.terminal_goal_z_tolerance_m;
        same_floor_preference_=config.same_floor_preference;
        same_floor_z_tolerance_=config.same_floor_z_tolerance;
        if (surface_graph_) surface_graph_->setQuery(max_iterations_,snap_search_radius_cells_);
    }

    void OctoPlanner3D::setExternalPreblockedRegions(
        std::vector<ExternalBlockedRegion> regions)
    {
        external_preblocked_regions_ = std::move(regions);
        if (!octree_)
        {
            return;
        }
        rebuildExternalPreblockedCells();
        clearMapQueryCaches();
        rebuildDerivedLayers();
    }

    void OctoPlanner3D::setNoGoRegions(
        std::shared_ptr<const std::vector<lingtu::maps::NoGoRegion>> regions)
    {
        if (no_go_regions_ == regions) return;
        no_go_regions_ = std::move(regions);
        clearMapQueryCaches();
    }

    bool OctoPlanner3D::noGoIntersects(const octomap::point3d &from,
                                      const octomap::point3d &to, double radius) const
    {
        if (!no_go_regions_ || no_go_regions_->empty()) return false;
        const bool explicit_body = body_clearance_below_m_ > 0.0 || body_clearance_above_m_ > 0.0;
        const double below = explicit_body ? std::max(0.0, body_clearance_below_m_) : octree_->getResolution();
        const double above = explicit_body ? std::max(0.0, body_clearance_above_m_) : octree_->getResolution();
        for (const auto &region : *no_go_regions_)
            if (lingtu::maps::SweptCylinderIntersectsNoGo(region,
                    {from.x(), from.y(), from.z()}, {to.x(), to.y(), to.z()},
                    radius, below, above)) return true;
        return false;
    }

    void OctoPlanner3D::setOctomap(std::shared_ptr<octomap::OcTree> map,
        std::shared_ptr<const SurfaceIndex> surface,std::shared_ptr<SurfaceGraph> graph)
    {
        if (!map)
        {
            printf("Octomap is Null!!! return.\n");
            return;
        }

        octree_ = map;
        map_ready_ = true;
        external_preblocked_cells_.clear();
        if(surface) surface_index_=std::move(surface);
        else rebuildSurfaceIndex();
        rebuildQueryOffsets();
        clearMapQueryCaches();
        if(graph) surface_graph_=std::move(graph);
        else rebuildSurfaceGraph();
        rebuildExternalPreblockedCells();
        rebuildDerivedLayers();
    }

    void OctoPlanner3D::writePrepared(lingtu::maps::BinaryWriter& out) const
    {
        surface_index_->write(out);
        out.value<std::uint8_t>(surface_graph_ ? 1 : 0);
        if(surface_graph_)surface_graph_->write(out);
    }

    void OctoPlanner3D::makePlan(const PointPose start,const PointPose goal)
    {
        occupied_blocks_.clear();
        last_occupied_block_ = nullptr;
        endpoint_resolution_ = {};
        search_info_ = {};
        start_point_ = start;
        has_start_ = true;

        goal_point_ = goal;
        has_goal_ = true;
        anchorPlanningLattice(start);

        bool has_occupied_map = false;
        if (ground_support_ && support_height_m_ > 0.0) {
            for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
                if (octree_->isNodeOccupied(*it)) {
                    has_occupied_map = true;
                    break;
                }
            }
        }
        if (ground_support_ && support_height_m_ > 0.0 && !has_occupied_map) {
            endpoint_resolution_.failure =
                EndpointResolutionInfo::Failure::StartSnapExhausted;
            endpoint_resolution_.start_failure = TraversabilityFailure::None;
            return;
        }

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

    OctoPlanner3D::EndpointResolutionInfo OctoPlanner3D::endpointResolution() const noexcept
    {
        return endpoint_resolution_;
    }

    OctoPlanner3D::SearchInfo OctoPlanner3D::searchInfo() const noexcept
    {
        return search_info_;
    }

    OctoPlanner3D::PoseQueryInfo OctoPlanner3D::preparePoseQueries(const PointPose &anchor)
    {
        PoseQueryInfo info;
        if (!octree_ || !map_ready_) return info;
        occupied_blocks_.clear();
        last_occupied_block_ = nullptr;
        anchorPlanningLattice(anchor);
        info.ready = true;
        info.xy_resolution = planningXYResolution();
        info.z_resolution = planningZResolution();
        octree_->getMetricMin(info.minimum.x, info.minimum.y, info.minimum.z);
        octree_->getMetricMax(info.maximum.x, info.maximum.y, info.maximum.z);
        info.max_iterations = max_iterations_;
        info.snap_search_radius_cells = snap_search_radius_cells_;
        info.goal_tolerance_m = terminal_goal_tolerance_m_;
        info.goal_xy_tolerance_m = terminal_goal_xy_tolerance_m_;
        info.goal_z_tolerance_m = terminal_goal_z_tolerance_m_;
        return info;
    }

    bool OctoPlanner3D::poseSupportFree(const PointPose &point) const
    {
        if (!octree_ || !map_ready_) return false;
        return queryWorld(octomap::point3d(point.x, point.y, point.z), 0.0,
                          ground_support_, false) == TraversabilityFailure::None;
    }

    bool OctoPlanner3D::supportConnectionFree(
        const PointPose &from, const PointPose &to) const
    {
        if (!metricConnectionWithinLimits(from, to)) return false;
        return sweptRouteFree(octomap::point3d(from.x, from.y, from.z),
                              octomap::point3d(to.x, to.y, to.z), nullptr, nullptr, false);
    }

    bool OctoPlanner3D::metricConnectionWithinLimits(
        const PointPose &from, const PointPose &to) const
    {
        if (!octree_ || !map_ready_) return false;
        const double dx=to.x-from.x,dy=to.y-from.y,dz=to.z-from.z;
        if (isStepWithinLimits(dx,dy,dz)) return true;
        if (!ground_support_ || support_height_m_<=0.0 ||
            std::hypot(dx,dy)<=kMetricToleranceEpsilonM ||
            (max_step_height_>0.0 && std::abs(dz)>max_step_height_+kMetricToleranceEpsilonM)) return false;
        // Different valid body heights over the same observed floor are not a terrain slope.
        const auto floor=[&](const PointPose& point)->std::optional<int> {
            const auto cell=worldToGrid(point.x,point.y,point.z);
            const double offset=(cell.z+.5)*octree_->getResolution()-point.z;
            const auto [low,high]=supportDepthRange(support_depth_cells_,offset);
            const auto z=surface_index_->below(cell.x,cell.y,cell.z);
            return z && *z>=cell.z-high && *z<=cell.z-low ? z : std::nullopt;
        };
        const auto a=floor(from),b=floor(to);
        return a && b && *a==*b;
    }

    bool OctoPlanner3D::resolveGoal()
    {
        // Resolve at map spacing once; graph and grid share this physical terminal.
        anchorPlanningLattice(start_point_, octree_->getResolution());
        const octomap::point3d requested(goal_point_.x, goal_point_.y, goal_point_.z);
        endpoint_resolution_.goal_raw_outside_bounds = !isInsideMetricBounds(
            worldToGrid(requested.x(), requested.y(), requested.z()));
        const bool physical_pose = support_height_m_ > 0.0 ||
            body_clearance_below_m_ > 0.0 || body_clearance_above_m_ > 0.0;
        if (!physical_pose || queryWorld(requested, robot_radius_, ground_support_) !=
                TraversabilityFailure::None) {
            const auto seed = worldToPlanningGrid(requested.x(), requested.y(), requested.z());
            GridIndex goal{};
            if (!findNearestFreeCell(seed, robot_radius_, snap_search_radius_cells_,
                    ground_support_, direct_support_only_, support_depth_cells_, goal, &goal_point_)) {
                endpoint_resolution_.failure = EndpointResolutionInfo::Failure::GoalSnapExhausted;
                return false;
            }
            const auto point = planningPoint(goal);
            endpoint_resolution_.goal_snapped = (point-requested).norm() > kMetricToleranceEpsilonM;
            goal_point_ = {point.x(), point.y(), point.z(), goal_point_.yaw};
        }
        anchorPlanningLattice(start_point_);
        return true;
    }

    bool OctoPlanner3D::resolvePlanEndpoints(GridIndex & start, GridIndex & goal)
    {
        const GridIndex start_raw = worldToPlanningGrid(
            start_point_.x, start_point_.y, start_point_.z);
        const GridIndex goal_raw = worldToPlanningGrid(
            goal_point_.x, goal_point_.y, goal_point_.z);
        start = start_raw;
        goal = goal_raw;
        const auto inside = [&](const GridIndex & index) {
            const auto point = planningPoint(index);
            return isInsideMetricBounds(worldToGrid(point.x(), point.y(), point.z()));
        };
        endpoint_resolution_.start_raw_outside_bounds = !inside(start_raw);
        endpoint_resolution_.failure = EndpointResolutionInfo::Failure::None;
        endpoint_resolution_.start_failure = TraversabilityFailure::None;

        if (ground_support_ && support_height_m_ > 0.0 &&
            surface_index_->observedSurfaces() == 0U) {
            endpoint_resolution_.failure =
                EndpointResolutionInfo::Failure::StartSnapExhausted;
            return false;
        }

        if (!findNearestFreeCell(
                start_raw,
                robot_radius_,
                snap_search_radius_cells_,
                ground_support_,
                direct_support_only_, support_depth_cells_,
                start, nullptr, &start_point_, &endpoint_resolution_.start_failure)) {
            endpoint_resolution_.failure =
                EndpointResolutionInfo::Failure::StartSnapExhausted;
            printf("OctoPlanner3D::startPlan no traversable start with an executable entry\n");
            return false;
        }
        endpoint_resolution_.start_snapped = !(start == start_raw);
        if (!findNearestFreeCell(
                goal_raw,
                robot_radius_,
                snap_search_radius_cells_,
                ground_support_,
                direct_support_only_, support_depth_cells_,
                goal, &goal_point_, &goal_point_)) {
            endpoint_resolution_.failure =
                EndpointResolutionInfo::Failure::GoalSnapExhausted;
            search_info_.outcome = SearchInfo::Outcome::Exhausted;
            printf("OctoPlanner3D::startPlan terminal cannot attach to this search lattice\n");
            return false;
        }

        if (!(start == start_raw)) {
            const auto point = planningPoint(start);
            printf(
                "OctoPlanner3D::startPlan start snapped to free cell: [%.2f, %.2f, %.2f]\n",
                point.x(), point.y(), point.z());
        }
        if (!(goal == goal_raw)) {
            const auto point = planningPoint(goal);
            printf(
                "OctoPlanner3D::startPlan goal snapped to free cell: [%.2f, %.2f, %.2f]\n",
                point.x(), point.y(), point.z());
        }
        return true;
    }

    bool OctoPlanner3D::routeConnectionFree(
        const PointPose &from, const PointPose &to, bool endpoints_validated) const
    {
        if (!metricConnectionWithinLimits(from,to)) return false;
        const octomap::point3d a(from.x,from.y,from.z), b(to.x,to.y,to.z);
        const bool start=a==octomap::point3d(start_point_.x,start_point_.y,start_point_.z);
        if (start) return isStartConnectionAllowed(a,b);
        return (endpoints_validated ||
            (queryWorld(a,robot_radius_,ground_support_)==TraversabilityFailure::None &&
             queryWorld(b,robot_radius_,ground_support_)==TraversabilityFailure::None)) &&
            sweptRouteFree(a,b,nullptr,nullptr,true,true);
    }

    bool OctoPlanner3D::startPlan()
    {
        planner_results_.clear();
        if (ground_support_ && support_height_m_ > 0.0 &&
            surface_index_->observedSurfaces() == 0U) {
            endpoint_resolution_.failure =
                EndpointResolutionInfo::Failure::StartSnapExhausted;
            endpoint_resolution_.start_failure = TraversabilityFailure::None;
            return false;
        }
        if (!resolveGoal()) return false;
        if (surface_graph_) {
            const auto valid = [&](const PointPose &a,const PointPose &b) {
                return routeConnectionFree(a,b);
            };
            const bool static_only=external_preblocked_cells_.empty() &&
                (!no_go_regions_ || no_go_regions_->empty());
            const auto prepared_valid=[&](const PointPose &a,const PointPose &b) {
                return static_only || routeConnectionFree(a,b);
            };
            const auto graph_started=std::chrono::steady_clock::now();
            auto candidate=surface_graph_->plan(start_point_,goal_point_,valid,cancel_check_,prepared_valid);
            const auto graph_finished=std::chrono::steady_clock::now();
            if (!candidate.empty()) {
                const auto height = [&](const PointPose &point) -> std::optional<double> {
                    auto ground=surface_index_->height(
                        {point.x,point.y,point.z-support_height_m_},support_height_tolerance_m_);
                    return ground ? std::optional<double>(*ground+support_height_m_) : std::nullopt;
                };
                candidate=smoothRoute(std::move(candidate),valid,height,cancel_check_);
                if (cancel_check_ && cancel_check_()) {
                    search_info_.outcome=SearchInfo::Outcome::Cancelled;
                    return false;
                }
                if (!candidate.empty()) {
                    planner_results_=std::move(candidate);
                    search_info_.outcome=SearchInfo::Outcome::Found;
                    search_info_.used_surface_graph=true;
                    printf("OctoPlanner3D surface route: nodes=%zu edges=%zu waypoints=%zu graph_ms=%.3f smooth_ms=%.3f\n",
                        surface_graph_->nodeCount(),surface_graph_->edgeCount(),planner_results_.size(),
                        std::chrono::duration<double,std::milli>(graph_finished-graph_started).count(),
                        std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-graph_finished).count());
                    return true;
                }
            }
            if (cancel_check_ && cancel_check_()) {
                search_info_.outcome=SearchInfo::Outcome::Cancelled;
                return false;
            }
            printf("OctoPlanner3D surface connection unavailable; continuing XYZ search\n");
        }
        if (searchGrid(max_iterations_)) return true;
        const double map_resolution = octree_->getResolution();
        if (search_info_.outcome != SearchInfo::Outcome::Exhausted ||
            planningXYResolution() <= map_resolution + kMetricToleranceEpsilonM ||
            search_info_.iterations >= max_iterations_) return false;
        // A coarse lattice can miss a valid corridor between its samples.
        // Refine the query only; reuse the map, geometry rules and remaining budget.
        printf("OctoPlanner3D coarse lattice exhausted; refining XY %.3f -> %.3f\n",
               planningXYResolution(), map_resolution);
        anchorPlanningLattice(start_point_, map_resolution);
        return searchGrid(max_iterations_ - search_info_.iterations);
    }

    bool OctoPlanner3D::searchGrid(int max_iterations)
    {
        const double robot_radius = robot_radius_;
        const bool ground_support = ground_support_;
        const bool direct_support_only = direct_support_only_;
        const int support_depth_cells = support_depth_cells_;
        const bool enable_preblocked_costmap =  enable_preblocked_costmap_;
        const double preblocked_costmap_weight = preblocked_costmap_weight_;
        const double floor_change_penalty = floor_change_penalty_;
        const bool same_floor_preference = same_floor_preference_;
        const double same_floor_z_tolerance = same_floor_z_tolerance_;
        const int obstacle_clearance_radius_cells = obstacle_clearance_radius_cells_;
        const double obstacle_clearance_weight = obstacle_clearance_weight_;
        const double z_resolution = planningZResolution();

        GridIndex start{};
        GridIndex goal{};
        if (!resolvePlanEndpoints(start, goal)) {
            return false;
        }
        const bool same_floor_request =
            same_floor_preference &&
            std::abs(static_cast<double>(goal.z - start.z) * z_resolution) <= same_floor_z_tolerance;

        std::unordered_map<GridIndex, TraversabilityFailure, GridIndexHash>
            planning_cell_cache;
        planning_cell_cache.reserve(65536);

        int iters = 0;
        int rejected_closed = 0;
        int rejected_outside = 0;
        int rejected_ground = 0;
        int rejected_preblocked_below = 0;
        int rejected_preblocked_body = 0;
        int rejected_occupied_body = 0;
        int rejected_sweep = 0;
        int planning_cache_hits = 0;

        const auto planningCellTraversable = [&](const GridIndex & idx, TraversabilityFailure & failure) {
            const auto cached = planning_cell_cache.find(idx);
            if (cached != planning_cell_cache.end()) {
                ++planning_cache_hits;
                failure = cached->second;
                return failure == TraversabilityFailure::None;
            }
            TraversabilityFailure evaluated = TraversabilityFailure::None;
            const bool traversable = isPlanningCellTraversableDetailed(
                idx, robot_radius, ground_support, direct_support_only, support_depth_cells, &evaluated);
            planning_cell_cache.emplace(idx, evaluated);
            failure = evaluated;
            return traversable;
        };
        // Keep 26 adjacent offsets. A finer body-height lattice permits
        // slope-bounded transitions without changing measured occupancy.
        const auto directions = make26Directions();
        printf("OctoPlanner3D::startPlan directions=%zu max_iterations=%d\n",
               directions.size(), max_iterations);
        std::priority_queue<QueueNode, std::vector<QueueNode>, QueueNodeCompare> open_set;
        std::unordered_map<GridIndex, double, GridIndexHash> g_score;
        std::unordered_map<GridIndex, GridIndex, GridIndexHash> came_from;
        std::unordered_set<GridIndex, GridIndexHash> closed_set;
        const auto poseAt=[&](const GridIndex &cell) {
            const auto point=planningPoint(cell);
            return PointPose{point.x(),point.y(),point.z()};
        };
        g_score[start] = 0.0;
        open_set.push({start, euclidean(start, goal), 0.0});

        while (!open_set.empty() && iters < max_iterations)
        {
            if (cancel_check_ && cancel_check_()) {
                planner_results_.clear();
                search_info_.outcome = SearchInfo::Outcome::Cancelled;
                return false;
            }
            const QueueNode current = open_set.top();
            open_set.pop();
            const auto best_g = g_score.find(current.idx);
            if (best_g == g_score.end() || current.g > best_g->second + 1e-12) {
                continue;
            }
            ++iters;
            ++search_info_.iterations;

            if (closed_set.find(current.idx) != closed_set.end()) {
                continue;
            }
            closed_set.insert(current.idx);

            const auto current_pose = poseAt(current.idx);
            if (current.idx == goal) {
                const auto terminal = goal_point_;
                std::vector<PointPose> path;
                for (const auto &cell : reconstructPath(came_from, current.idx))
                    path.push_back(poseAt(cell));
                if (std::hypot(path.front().x-start_point_.x,path.front().y-start_point_.y,
                               path.front().z-start_point_.z)>1e-6)
                    path.insert(path.begin(),start_point_);
                else path.front().yaw=start_point_.yaw;
                if (std::hypot(path.back().x-terminal.x,path.back().y-terminal.y,
                               path.back().z-terminal.z)>1e-6)
                    path.push_back(terminal);
                path.back().yaw=goal_point_.yaw;
                planner_results_=std::move(path);
                search_info_.outcome = SearchInfo::Outcome::Found;
                printf("OctoPlanner3D::startPlan found: iterations=%d waypoints=%zu\n",
                       iters,planner_results_.size());
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
                const double step_cost = euclidean(current.idx, nbr);
                double tentative_g = current.g + step_cost;
                if (enable_preblocked_costmap) {
                const auto point = planningPoint(nbr);
                tentative_g += preblocked_costmap_weight * getPreblockedCost(
                    worldToGrid(point.x(), point.y(), point.z()));
                }
                const double dz_m = std::abs(static_cast<double>(nbr.z - current.idx.z)) * z_resolution;
                tentative_g += floor_change_penalty * dz_m;
                if (same_floor_request) {
                tentative_g += floor_change_penalty * 0.5 *
                    std::abs(static_cast<double>(nbr.z - start.z)) * z_resolution;
                }
                if (obstacle_clearance_weight > 0.0 && obstacle_clearance_radius_cells > 0) {
                const auto point = planningPoint(nbr);
                tentative_g += obstacle_clearance_weight * getObstacleClearanceCost(
                    worldToGrid(point.x(), point.y(), point.z()));
                }

                auto g_it = g_score.find(nbr);
                if (g_it == g_score.end() || tentative_g < g_it->second) {
                if ((support_height_m_>0.0 && !lowest_traversable_only_) &&
                    !routeConnectionFree(current_pose,poseAt(nbr),!(current.idx==start))) {
                    ++rejected_sweep;
                    continue;
                }
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
            "reject_external_preblocked_body=%d reject_occupied_body=%d reject_sweep=%d\n",
            iters,
            closed_set.size(),
            open_set.size(),
            rejected_closed,
            rejected_outside,
            rejected_ground,
            rejected_preblocked_below,
            rejected_preblocked_body,
            rejected_occupied_body,
            rejected_sweep);
        printf(
            "OctoPlanner3D::startPlan cache diagnostics: planning_cells=%zu planning_hits=%d\n",
            planning_cell_cache.size(),
            planning_cache_hits);
        if (cancel_check_ && cancel_check_()) {
            search_info_.outcome = SearchInfo::Outcome::Cancelled;
            return false;
        }
        if (!open_set.empty() && iters >= max_iterations) {
            search_info_.outcome = SearchInfo::Outcome::IterationLimit;
            return false;
        }
        search_info_.outcome = SearchInfo::Outcome::Exhausted;
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

    bool OctoPlanner3D::findNearestFreeCell(const GridIndex & seed, double robot_radius, int radius_cells, bool ground_support,bool direct_support_only,  int support_depth_cells,GridIndex & out, const PointPose * reference, const PointPose * connection_start, TraversabilityFailure *start_failure) const
    {
        const octomap::point3d entry_origin = connection_start
            ? octomap::point3d(connection_start->x, connection_start->y, connection_start->z)
            : octomap::point3d();
        const auto entry_allowed = [&](const octomap::point3d & point) {
            if (!connection_start) return true;
            return isStartConnectionAllowed(entry_origin, point);
        };
        if (connection_start &&
            !isStartConnectionAllowed(entry_origin,entry_origin,start_failure))
            return false;
        const auto distance_to_reference = [&](const octomap::point3d &point) {
            return std::hypot(point.x() - reference->x, point.y() - reference->y,
                              point.z() - reference->z);
        };
        const bool exact_goal = reference && connection_start;
        // Grid quantization limits the connector search, not the final goal accuracy.
        const double reference_radius = exact_goal
            ? .5 * std::hypot(std::sqrt(2.0) * planningXYResolution(), planningZResolution())
            : terminal_goal_tolerance_m_;
        const auto within_goal_tolerance = [&](const octomap::point3d &point) {
            return !reference ||
                (distance_to_reference(point) <= reference_radius + kMetricToleranceEpsilonM &&
                 (exact_goal || (std::hypot(point.x() - reference->x, point.y() - reference->y) <=
                   terminal_goal_xy_tolerance_m_ + kMetricToleranceEpsilonM &&
                 std::abs(point.z() - reference->z) <=
                   terminal_goal_z_tolerance_m_ + kMetricToleranceEpsilonM)));
        };
        const auto seed_point = planningPoint(seed);
        if (within_goal_tolerance(seed_point) && isPlanningCellTraversableDetailed(
            seed, robot_radius, ground_support, direct_support_only, support_depth_cells, nullptr) && entry_allowed(seed_point))
        {
        out = seed;
        return true;
        }

        // Visit nearest candidates first. Previously even a two-cell height
        // correction checked the complete 49^3 cube with footprint queries.
        if (snap_offsets_.empty()) {
            radius_cells = static_cast<int>(std::floor(radius_cells * octree_->getResolution() / planningXYResolution()));
            const double z_scale = planningZResolution() / planningXYResolution();
            const int vertical_radius = static_cast<int>(std::ceil(radius_cells / z_scale));
            for (int dz = -vertical_radius; dz <= vertical_radius; ++dz)
                for (int dx = -radius_cells; dx <= radius_cells; ++dx)
                    for (int dy = -radius_cells; dy <= radius_cells; ++dy)
                        if (dx != 0 || dy != 0 || dz != 0) snap_offsets_.push_back({dx, dy, dz});
            std::sort(snap_offsets_.begin(), snap_offsets_.end(), [z_scale](const auto &a, const auto &b) {
                const double axy = a.x * a.x + a.y * a.y;
                const double bxy = b.x * b.x + b.y * b.y;
                return std::array<double, 5>{axy + a.z * a.z * z_scale * z_scale, axy,
                        static_cast<double>(a.z), static_cast<double>(a.x), static_cast<double>(a.y)} <
                       std::array<double, 5>{bxy + b.z * b.z * z_scale * z_scale, bxy,
                        static_cast<double>(b.z), static_cast<double>(b.x), static_cast<double>(b.y)};
            });
        }
        const double seed_error = reference ? distance_to_reference(seed_point) : 0.0;
        double best_distance = reference ? reference_radius : std::numeric_limits<double>::infinity();
        bool found = false;
        for (const auto &offset : snap_offsets_) {
            if (cancel_check_ && cancel_check_()) return false;
            // Offsets are ordered by seed distance. The triangle inequality
            // bounds all remaining candidates without scanning the full cube.
            const double seed_distance = std::hypot(
                planningXYResolution() * std::hypot(offset.x, offset.y),
                planningZResolution() * offset.z);
            if (seed_distance - seed_error > best_distance + 1e-7) break;
            const GridIndex candidate{seed.x + offset.x, seed.y + offset.y, seed.z + offset.z};
            if (!within_goal_tolerance(planningPoint(candidate))) continue;
            if (isPlanningCellTraversableDetailed(
                    candidate, robot_radius, ground_support, direct_support_only, support_depth_cells, nullptr) &&
                entry_allowed(planningPoint(candidate))) {
                // Use the route's existing height-change preference for start
                // snapping too. A slightly nearer lower cell can otherwise
                // turn a level route into an unnecessary stair search.
                // Goal snapping still minimizes error to the requested goal.
                const double distance = reference ? distance_to_reference(planningPoint(candidate)) :
                    seed_distance + (same_floor_preference_ ? std::max(0.0, floor_change_penalty_) *
                        planningZResolution() * std::abs(offset.z) : 0.0);
                if (!found || distance < best_distance) {
                    out = candidate;
                    best_distance = distance;
                    found = true;
                }
            }
        }
        return found;
    }

    double OctoPlanner3D::getPreblockedCost(const GridIndex & idx) const
    {
        if (!enable_preblocked_costmap_ || isPreblockedCell(idx)) return 0.0;
        const auto cached = preblocked_costmap_.find(idx);
        if (cached != preblocked_costmap_.end()) return cached->second;
        const int radius = std::max(1, preblocked_costmap_radius_cells_);
        double cost = 0.0;
        for (int dx = -radius; dx <= radius; ++dx) {
            for (int dy = -radius; dy <= radius; ++dy) {
                for (int dz = -radius; dz <= radius; ++dz) {
                    const int distance2 = dx * dx + dy * dy + dz * dz;
                    if (distance2 == 0 || distance2 > radius * radius) continue;
                    if (isPreblockedCell({idx.x + dx, idx.y + dy, idx.z + dz}))
                        cost = std::max(cost, 1.0 - std::sqrt(static_cast<double>(distance2)) / (radius + 1.0));
                }
            }
        }
        preblocked_costmap_.emplace(idx, cost);
        return cost;
    }

    double OctoPlanner3D::getObstacleClearanceCost(const GridIndex & idx) const
    {
        if (obstacle_clearance_radius_cells_ <= 0) return 0.0;
        const auto cached = obstacle_clearance_costmap_.find(idx);
        if (cached != obstacle_clearance_costmap_.end()) return cached->second;
        const int radius = obstacle_clearance_radius_cells_;
        double cost = 0.0;
        for (int dx = -radius; dx <= radius; ++dx) {
            for (int dy = -radius; dy <= radius; ++dy) {
                for (int dz = -radius; dz <= radius; ++dz) {
                    const int distance2 = dx * dx + dy * dy + dz * dz;
                    if (distance2 > radius * radius) continue;
                    if (isOccupiedLeafCell({idx.x + dx, idx.y + dy, idx.z + dz}))
                        cost = std::max(cost, 1.0 - std::sqrt(static_cast<double>(distance2)) / (radius + 1.0));
                }
            }
        }
        obstacle_clearance_costmap_.emplace(idx, cost);
        return cost;
    }

    bool OctoPlanner3D::isStepWithinLimits(const GridIndex & step) const
    {
        const double r = planningXYResolution();
        return isStepWithinLimits(step.x * r, step.y * r,
                                  step.z * planningZResolution());
    }

    bool OctoPlanner3D::isStepWithinLimits(double dx, double dy, double dz) const
    {
        const double rise = std::abs(dz);
        const double horizontal = std::hypot(dx, dy);
        if (max_step_height_ > 0.0 && rise > max_step_height_ + kMetricToleranceEpsilonM)
            return false;
        if (rise <= kMetricToleranceEpsilonM) return true;
        if (horizontal <= kMetricToleranceEpsilonM) return false;
        return max_slope_ <= 0.0 ||
            rise <= max_slope_ * horizontal + kMetricToleranceEpsilonM;
    }

    bool OctoPlanner3D::isStartConnectionAllowed(
        const octomap::point3d & from, const octomap::point3d & to,
        TraversabilityFailure *failure) const
    {
        if (!metricConnectionWithinLimits({from.x(),from.y(),from.z()},{to.x(),to.y(),to.z()}))
            return false;
        if (support_height_m_ > 0.0 || body_clearance_below_m_ > 0.0 ||
            body_clearance_above_m_ > 0.0) {
            const bool actual_start=has_start_&&from==octomap::point3d(start_point_.x,start_point_.y,start_point_.z);
            // The actual start may contain the robot or incomplete ground observations.
            const auto origin=queryWorld(from,robot_radius_,ground_support_&&!actual_start,!actual_start);
            if(origin!=TraversabilityFailure::None){if(failure)*failure=origin;return false;}
            if (actual_start && ground_support_) {
                if (support_height_m_ > 0.0 && surface_index_->observedSurfaces() == 0U)
                    return false;
                const auto cell = worldToGrid(from.x(), from.y(), from.z());
                const auto [minimum_depth, maximum_depth] =
                    supportDepthRange(support_depth_cells_);
                const bool robot_evidence = hasOccupiedNearZ(cell, 2);
                bool support_found = false;
                for (auto floor = surface_index_->below(cell.x, cell.y, cell.z);
                     floor; floor = surface_index_->below(cell.x, cell.y, *floor)) {
                    if (*floor >= cell.z - maximum_depth &&
                        *floor <= cell.z - minimum_depth &&
                        (robot_evidence || surface_index_->exposed(
                            cell.x, cell.y, *floor,
                            static_cast<int>(std::ceil(
                                (support_height_m_ + std::max(
                                    octree_->getResolution(), body_clearance_above_m_)) /
                                octree_->getResolution())),
                            !direct_support_only_))) {
                        support_found = true;
                        break;
                    }
                }
                if (!support_found && surface_index_->below(cell.x, cell.y, cell.z)) {
                    if (failure) *failure = TraversabilityFailure::GroundSupport;
                    return false;
                }
            }
            if(from==to)return true;
            const auto target=queryWorld(to,robot_radius_,ground_support_);
            if(target!=TraversabilityFailure::None){if(failure)*failure=target;return false;}
            return sweptRouteFree(from,to,failure,nullptr,true,true,!actual_start);
        }
        // The legacy surface graph roots its upper body hemisphere at a
        // voxel, rather than declaring a body-centre clearance. Keep that
        // coordinate meaning while checking every part of its entry.
        const int count = std::max(1, static_cast<int>(std::ceil(
            2.0 * (to-from).norm() / octree_->getResolution())));
        const bool actual_start=has_start_&&from==octomap::point3d(start_point_.x,start_point_.y,start_point_.z);
        for (int i = 0; i <= count; ++i) {
            const double t = static_cast<double>(i) / count;
            const auto cell = worldToPlanningGrid(
                from.x()+t*(to.x()-from.x()), from.y()+t*(to.y()-from.y()),
                from.z()+t*(to.z()-from.z()));
            if(actual_start&&(i==0||from==to)) {
                const auto origin=queryWorld(from,robot_radius_,false,false);
                if(origin!=TraversabilityFailure::None){if(failure)*failure=origin;return false;}
                if (ground_support_) {
                    const auto support = worldToGrid(from.x(), from.y(), from.z());
                    if (surface_index_->below(support.x, support.y, support.z) &&
                        (!hasGroundSupport(
                            support, direct_support_only_, support_depth_cells_) ||
                         (direct_support_only_ &&
                          !hasFootprintGroundSupport(
                              support, robot_radius_, support_depth_cells_)))) {
                        return false;
                    }
                }
                continue;
            }
            if (!isPlanningCellTraversableDetailed(
                    cell, robot_radius_, ground_support_ && !actual_start, direct_support_only_, support_depth_cells_, failure))
                return false;
        }
        return true;
    }

    bool OctoPlanner3D::isMotionAllowed(const GridIndex & from, const GridIndex & to) const
    {
        if (support_height_m_ > 0.0 && !lowest_traversable_only_) {
            const auto a = planningPoint(from), b = planningPoint(to);
            return metricConnectionWithinLimits({a.x(),a.y(),a.z()},{b.x(),b.y(),b.z()}) &&
                   sweptRouteFree(a, b);
        }
        return isStepWithinLimits({to.x - from.x, to.y - from.y, to.z - from.z});
    }

    std::vector<GridIndex> OctoPlanner3D::make26Directions() const
    {
        std::vector<GridIndex> dirs;
        dirs.reserve(26);
        for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy)
                for (int dz = -1; dz <= 1; ++dz) {
                    if (dx != 0 || dy != 0 || dz != 0)
                        dirs.push_back(GridIndex{dx, dy, dz});
                }
        return dirs;
    }

    OctoPlanner3D::PlanningSlice OctoPlanner3D::projectPlanningSlice(
        double reference_z, std::size_t max_cells) const
    {
        PlanningSlice result;
        result.reference_z = reference_z;
        if (!octree_ || octree_->size() == 0U || !std::isfinite(reference_z)) {
            result.reason = "planning_map_not_ready";
            return result;
        }
        result.resolution = octree_->getResolution();
        double min_x, min_y, min_z, max_x, max_y, max_z;
        octree_->getMetricMin(min_x, min_y, min_z);
        octree_->getMetricMax(max_x, max_y, max_z);
        const auto minimum = worldToGrid(min_x, min_y, reference_z);
        const auto maximum = worldToGrid(
            std::nextafter(max_x, min_x), std::nextafter(max_y, min_y), reference_z);
        const auto cols = static_cast<std::int64_t>(maximum.x) - minimum.x + 1;
        const auto rows = static_cast<std::int64_t>(maximum.y) - minimum.y + 1;
        if (cols <= 0 || rows <= 0 || static_cast<std::uint64_t>(cols) > max_cells ||
            static_cast<std::uint64_t>(rows) > max_cells / static_cast<std::uint64_t>(cols)) {
            result.reason = "planning_map_grid_limit";
            return result;
        }
        const auto first = gridToWorld(minimum);
        result.origin = {first.x() - result.resolution * 0.5,
                         first.y() - result.resolution * 0.5, first.z()};
        result.rows = static_cast<int>(rows);
        result.cols = static_cast<int>(cols);
        result.cells.resize(static_cast<std::size_t>(rows * cols), 0U);
        for (int row = 0; row < result.rows; ++row) {
            if (cancel_check_ && cancel_check_()) {
                result.rows = result.cols = 0;
                result.cells.clear();
                result.reason = "planning_map_cancelled";
                return result;
            }
            for (int col = 0; col < result.cols; ++col) {
                const GridIndex index{minimum.x + col, minimum.y + row, minimum.z};
                TraversabilityFailure failure = TraversabilityFailure::None;
                bool allowed;
                if (support_height_m_ > 0.0 && !lowest_traversable_only_) {
                    auto point = gridToWorld(index);
                    point.z() = static_cast<float>(reference_z);
                    failure = queryWorld(point, robot_radius_, ground_support_);
                    allowed = failure == TraversabilityFailure::None;
                } else {
                    allowed = isPlanningCellTraversableDetailed(
                        index, robot_radius_, ground_support_, direct_support_only_, support_depth_cells_, &failure);
                }
                const bool blocked = failure == TraversabilityFailure::OccupiedBody ||
                    failure == TraversabilityFailure::ExternalPreblockedBody ||
                    failure == TraversabilityFailure::ExternalPreblockedBelow ||
                    isOccupiedCell(index);
                result.cells[static_cast<std::size_t>(row) * result.cols + col] =
                    allowed ? 1U : (blocked ? 2U : 0U);
            }
        }
        result.available = true;
        return result;
    }

    bool OctoPlanner3D::isCellTraversable(const GridIndex & idx, double robot_radius, bool ground_support,bool direct_support_only, int support_depth_cells) const
    {
        return isCellTraversableDetailed(
            idx,
            robot_radius,
            ground_support,
            direct_support_only, support_depth_cells,
            nullptr);
    }

    bool OctoPlanner3D::isCellTraversableDetailed(const GridIndex & idx, double robot_radius, bool ground_support,bool direct_support_only, int support_depth_cells, TraversabilityFailure * failure) const
    {
        const double resolution = octree_->getResolution();
        const auto policy_point = support_height_m_ > 0.0 && !lowest_traversable_only_
            ? planningPoint(idx) : octomap::point3d((idx.x + .5) * resolution,
                (idx.y + .5) * resolution, (idx.z + .5) * resolution);
        if (noGoIntersects(policy_point, policy_point, robot_radius)) {
            if (failure) *failure = TraversabilityFailure::ExternalPreblockedBody;
            return false;
        }
        if (support_height_m_ > 0.0 && !lowest_traversable_only_) {
            const auto point = planningPoint(idx);
            if (!ground_support &&
                cellState(worldToGrid(point.x(), point.y(), point.z())) == CellState::Unknown) {
                if (failure) *failure = TraversabilityFailure::UnknownSpace;
                return false;
            }
            auto cached = body_cache_.find(idx);
            // Vertices and edges use route clearance, independent of SCAN.
            const auto result = ground_support && cached != body_cache_.end()
                ? cached->second : queryWorld(point, robot_radius, ground_support);
            if (ground_support) body_cache_.emplace(idx, result);
            if (failure) *failure = result;
            return result == TraversabilityFailure::None;
        }
        if (!ground_support && cellState(idx) == CellState::Unknown) {
            if (failure) *failure = TraversabilityFailure::UnknownSpace;
            return false;
        }
        if (!isInsideMetricBounds(idx)) {
        if (failure) {
            *failure = TraversabilityFailure::OutsideBounds;
        }
        return false;
        }

        if (ground_support &&
        !hasGroundSupport(
            idx, direct_support_only, support_depth_cells))
        {
        if (failure) {
            *failure = TraversabilityFailure::GroundSupport;
        }
        return false;
        }
        bool uneven_support = false;
        const bool footprint_supported = ground_support_ && direct_support_only &&
            hasFootprintGroundSupport(idx, robot_radius, support_depth_cells, &uneven_support);
        if (ground_support && direct_support_only && !footprint_supported)
        {
        if (failure) {
            *failure = TraversabilityFailure::GroundSupport;
        }
        return false;
        }

        // Planning vertices, stair intermediates and path simplification share
        // the same body envelope. Ground support is checked separately above.
        const auto cached = body_cache_.find(idx);
        if (cached != body_cache_.end()) {
            if (failure) *failure = cached->second;
            return cached->second == TraversabilityFailure::None;
        }
        const auto evaluate_body = [&]() {
            if (!external_preblocked_cells_.empty()) {
                for (int z = idx.z - 1; z >= 0; --z) {
                    const GridIndex below_idx{idx.x, idx.y, z};
                    if (isOccupiedCell(below_idx)) break;
                    if (external_preblocked_cells_.count(below_idx) != 0)
                        return TraversabilityFailure::ExternalPreblockedBelow;
                }
            }
            for (const auto & offset : body_offsets_) {
                const GridIndex nearby_idx{idx.x + offset.x, idx.y + offset.y, idx.z + offset.z};
                // Only explicit external regions are hard forbidden. The
                // map-derived preblocked layer retains its existing soft cost.
                if (!external_preblocked_cells_.empty() &&
                    external_preblocked_cells_.count(nearby_idx) != 0)
                    return TraversabilityFailure::ExternalPreblockedBody;
                if (isOccupiedCell(nearby_idx)) {
                    const double r=octree_->getResolution();
                    if (start_exemption_ && start_exemption_(
                        {(nearby_idx.x+.5)*r,(nearby_idx.y+.5)*r,(nearby_idx.z+.5)*r},r)) continue;
                    // The default envelope is rooted at a support surface.
                    // A reachable tread under the legs is terrain; explicit
                    // body-centre cylinders still reject every occupied cell.
                    const bool surface_envelope = body_clearance_below_m_ <= 0.0 &&
                        body_clearance_above_m_ <= 0.0 && uneven_support;
                    if (!surface_envelope ||
                        !isTraversableTerrainBelow(nearby_idx, idx))
                        return TraversabilityFailure::OccupiedBody;
                }
            }
            return TraversabilityFailure::None;
        };
        const auto evaluated = evaluate_body();
        body_cache_.emplace(idx, evaluated);
        if (failure) *failure = evaluated;
        return evaluated == TraversabilityFailure::None;
    }

    bool OctoPlanner3D::isPlanningCellTraversableDetailed(
        const GridIndex & idx,
        double robot_radius,
        bool ground_support,
        bool direct_support_only,
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
            ground_support,
            direct_support_only, support_depth_cells,
            failure);
    }

    std::pair<int, int> OctoPlanner3D::supportDepthRange(int support_depth_cells, double centre_offset) const
    {
        const int fallback_max = std::max(1, support_depth_cells);
        if (!octree_ || support_height_m_ <= 0.0) {
            return {1, fallback_max};
        }

        const double resolution = std::max(1e-6, octree_->getResolution());
        const double tolerance = std::max(0.0, support_height_tolerance_m_);
        // Compare the cell's extent with the stance band, not its centre as a measured surface.
        const double minimum_height = std::max(0.0, support_height_m_ - tolerance - 0.5 * resolution);
        const double maximum_height = support_height_m_ + tolerance + 0.5 * resolution;
        int minimum_depth = std::max(
            1,
            static_cast<int>(std::ceil((minimum_height + centre_offset) / resolution - 1e-6)));
        int maximum_depth = std::max(
            1,
            static_cast<int>(std::floor((maximum_height + centre_offset) / resolution + 1e-6)));
        if (maximum_depth < minimum_depth) {
            const int nearest_depth = std::max(
                1,
                static_cast<int>(std::lround((support_height_m_ + centre_offset) / resolution)));
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
        if (!isOccupiedCell(support)) {
            return false;
        }
        // A support surface must be exposed above the occupied voxel. This
        // rejects vertical walls and slab interiors when relaxed neighbouring
        // support is enabled.
        if (isOccupiedCell({support.x, support.y, support.z + 1})) {
            return false;
        }
        return isSupportPatch(support);
    }

    bool OctoPlanner3D::hasGroundSupport(const GridIndex & idx, bool direct_support_only,
                                        int support_depth_cells, double centre_offset) const
    {
        const auto [minimum_depth, maximum_depth] = supportDepthRange(support_depth_cells, centre_offset);
        if (!direct_support_only && surface_index_->inferred(idx.x, idx.y,
                idx.z - maximum_depth, idx.z - minimum_depth)) return true;
        const auto floor = surface_index_->below(idx.x,idx.y,idx.z);
        if (!floor || *floor < idx.z-maximum_depth || *floor > idx.z-minimum_depth)
            return false;
        const double resolution = octree_->getResolution();
        const double headroom = support_height_m_ +
            std::max(resolution, body_clearance_above_m_);
        const int clearance = static_cast<int>(std::ceil(headroom / resolution));
        if (!surface_index_->exposed(idx.x, idx.y, *floor, clearance,
                                     !direct_support_only))
            return false;
        return (*floor+1 != idx.z || !isOccupiedCell(idx)) &&
               isSupportPatch({idx.x,idx.y,*floor});
    }

    bool OctoPlanner3D::hasFootprintGroundSupport(
        const GridIndex & idx,
        double robot_radius,
        int support_depth_cells,
        bool * uneven) const
    {
        if (!octree_) {
        return false;
        }
        const double r = octree_->getResolution();
        const auto [minimum_depth, maximum_depth] = supportDepthRange(support_depth_cells);
        // Samples start at a voxel centre. A footprint ending exactly on a
        // cell boundary also needs the next support cell to avoid edge snaps.
        const double cells = robot_radius / std::max(1e-6, r);
        int n = std::max(0, static_cast<int>(std::floor(cells + 0.5)));
        if (n == 1 && std::abs(cells - 0.5) <= 1e-9) ++n;
        const std::array<GridIndex, 5> samples{{
            GridIndex{idx.x, idx.y, idx.z},
            GridIndex{idx.x + n, idx.y, idx.z},
            GridIndex{idx.x - n, idx.y, idx.z},
            GridIndex{idx.x, idx.y + n, idx.z},
            GridIndex{idx.x, idx.y - n, idx.z},
        }};
        int center_z = idx.z;
        for (int dz = 1; dz <= maximum_depth; ++dz) {
            const GridIndex below{idx.x, idx.y, idx.z - dz};
            if (isOccupiedCell(below)) {
                if (dz < minimum_depth || !isSupportCell(below)) return false;
                center_z = idx.z - dz;
                break;
            }
        }
        if (center_z == idx.z) return false;
        // Feet on a stair occupy different heights. Keep the centre's support
        // depth contract, and bound the four surrounding surfaces by the
        // configured step limit. A wall interior is not a support surface.
        const int step_cells = std::max(0, static_cast<int>(std::ceil(max_step_height_ / r - 1e-9)));
        std::array<int, 5> support_z{{center_z, center_z, center_z, center_z, center_z}};
        for (std::size_t i = 1; i < samples.size(); ++i) {
            bool supported = false;
            for (int distance = 0; distance <= step_cells && !supported; ++distance) {
                for (const int sign : {1, -1}) {
                    if (distance == 0 && sign == -1) continue;
                    const int z = center_z + sign * distance;
                    const GridIndex below{samples[i].x, samples[i].y, z};
                    if (isSupportCell(below) && !isOccupiedCell({below.x, below.y, z + 1})) {
                        support_z[i] = z;
                        supported = true;
                        break;
                    }
                }
            }
            if (!supported) return false;
        }
        for (const int axis : {1, 3}) {
            // A tread may lead to a higher or lower surface on one side. A
            // narrow ridge with both opposing feet below its top is not a
            // stair stance. Keep one voxel for surface quantization.
            if (support_z[axis] < center_z - 1 && support_z[axis + 1] < center_z - 1)
                return false;
        }
        if (uneven) {
            *uneven = std::any_of(support_z.begin(), support_z.end(),
                [center_z](int z) { return z != center_z; });
        }
        return true;
    }

    bool OctoPlanner3D::isTraversableTerrainBelow(
        const GridIndex & point, const GridIndex & body) const
    {
        if (point.x == body.x && point.y == body.y) return false;
        const int steps = std::max(0, static_cast<int>(std::ceil(
            max_step_height_ / octree_->getResolution() - 1e-9)));
        for (int z = point.z; z <= body.z - 1 + steps; ++z) {
            if (!isOccupiedCell({point.x, point.y, z})) return false;
            if (!isOccupiedCell({point.x, point.y, z + 1})) {
                return isSupportCell({point.x, point.y, z});
            }
        }
        return false;
    }

    void OctoPlanner3D::rebuildExternalPreblockedCells()
    {
        external_preblocked_cells_.clear();
        if (!octree_)
        {
            return;
        }

        constexpr std::uint64_t kMaxExternalPreblockedCells = 500000U;
        struct RasterRegion
        {
            const ExternalBlockedRegion * region;
            GridIndex minimum;
            GridIndex maximum;
        };

        double map_min_x = 0.0;
        double map_min_y = 0.0;
        double map_min_z = 0.0;
        double map_max_x = 0.0;
        double map_max_y = 0.0;
        double map_max_z = 0.0;
        octree_->getMetricMin(map_min_x, map_min_y, map_min_z);
        octree_->getMetricMax(map_max_x, map_max_y, map_max_z);

        std::uint64_t raster_cell_count = 0U;
        std::vector<RasterRegion> raster_regions;
        raster_regions.reserve(external_preblocked_regions_.size());
        for (const auto & region : external_preblocked_regions_)
        {
            if (!std::isfinite(region.center.x) || !std::isfinite(region.center.y) ||
                !std::isfinite(region.center.z) || !std::isfinite(region.radius_xy_m) ||
                !std::isfinite(region.min_z) || !std::isfinite(region.max_z) ||
                region.radius_xy_m <= 0.0 || region.min_z > region.max_z)
            {
                continue;
            }

            const double min_x = std::max(
                map_min_x, region.center.x - region.radius_xy_m);
            const double max_x = std::min(
                map_max_x, region.center.x + region.radius_xy_m);
            const double min_y = std::max(
                map_min_y, region.center.y - region.radius_xy_m);
            const double max_y = std::min(
                map_max_y, region.center.y + region.radius_xy_m);
            const double min_z = std::max(map_min_z, region.min_z);
            const double max_z = std::min(map_max_z, region.max_z);
            if (min_x > max_x || min_y > max_y || min_z > max_z)
            {
                continue;
            }

            const GridIndex minimum = worldToGrid(
                min_x, min_y, min_z);
            const GridIndex maximum = worldToGrid(
                max_x, max_y, max_z);
            const auto x_cells = static_cast<std::uint64_t>(
                static_cast<std::int64_t>(maximum.x) - minimum.x + 1);
            const auto y_cells = static_cast<std::uint64_t>(
                static_cast<std::int64_t>(maximum.y) - minimum.y + 1);
            const auto z_cells = static_cast<std::uint64_t>(
                static_cast<std::int64_t>(maximum.z) - minimum.z + 1);
            if (x_cells == 0U || y_cells == 0U || z_cells == 0U ||
                x_cells > kMaxExternalPreblockedCells / y_cells ||
                x_cells * y_cells > kMaxExternalPreblockedCells / z_cells)
            {
                throw std::length_error("temporary_overlay_raster_budget_exceeded");
            }
            const std::uint64_t region_cell_count = x_cells * y_cells * z_cells;
            if (region_cell_count > kMaxExternalPreblockedCells - raster_cell_count)
            {
                throw std::length_error("temporary_overlay_raster_budget_exceeded");
            }
            raster_cell_count += region_cell_count;
            raster_regions.push_back(RasterRegion{&region, minimum, maximum});
        }

        external_preblocked_cells_.reserve(
            external_preblocked_cells_.size() +
            static_cast<std::size_t>(raster_cell_count));
        for (const auto & raster : raster_regions)
        {
            const auto & region = *raster.region;
            const double radius_squared = region.radius_xy_m * region.radius_xy_m;
            for (int x = raster.minimum.x; x <= raster.maximum.x; ++x)
            {
                for (int y = raster.minimum.y; y <= raster.maximum.y; ++y)
                {
                    for (int z = raster.minimum.z; z <= raster.maximum.z; ++z)
                    {
                        const GridIndex index{x, y, z};
                        if (!isInsideMetricBounds(index))
                        {
                            continue;
                        }
                        const auto point = gridToWorld(index);
                        const double dx = static_cast<double>(point.x()) - region.center.x;
                        const double dy = static_cast<double>(point.y()) - region.center.y;
                        if (dx * dx + dy * dy <= radius_squared)
                        {
                            external_preblocked_cells_.insert(index);
                        }
                    }
                }
            }
        }
    }

    void OctoPlanner3D::clearMapQueryCaches()
    {
        occupied_blocks_.clear();
        last_occupied_block_ = nullptr;
        body_cache_.clear();
        preblocked_cache_.clear();
        preblocked_costmap_.clear();
        obstacle_clearance_costmap_.clear();
    }

    void OctoPlanner3D::rebuildQueryOffsets()
    {
        snap_offsets_.clear();
        body_offsets_.clear();
        body_cache_.clear();
        if (!octree_) return;
        const double r = octree_->getResolution();
        const int n = std::max(1, static_cast<int>(std::ceil(robot_radius_ / r)));
        const double radius_sq = robot_radius_ * robot_radius_;
        const bool cylinder = body_clearance_below_m_ > 0.0 || body_clearance_above_m_ > 0.0;
        // The body is centred in this voxel. Include neighbouring voxel
        // volumes only when they overlap the configured vertical interval;
        // ceil(clearance / r) adds a whole layer even at a shared boundary.
        const int minimum_dz = cylinder
            ? -std::max(0, static_cast<int>(std::ceil(body_clearance_below_m_ / r - 0.5 - 1e-9))) : 0;
        const int maximum_dz = cylinder
            ? std::max(0, static_cast<int>(std::ceil(body_clearance_above_m_ / r - 0.5 - 1e-9))) : n;
        // Preserve the sampled envelope and its visitation order. Cell-centre
        // neighbours differ by integer indices; no world/grid round trip is needed.
        for (int dx = -n; dx <= n; ++dx) {
            for (int dy = -n; dy <= n; ++dy) {
                for (int dz = minimum_dz; dz <= maximum_dz; ++dz) {
                    const double x = dx * r, y = dy * r, z = dz * r;
                    if (x * x + y * y + (cylinder ? 0.0 : z * z) <= radius_sq)
                        body_offsets_.push_back({dx, dy, dz});
                }
            }
        }
    }

    bool OctoPlanner3D::isPreblockedCell(const GridIndex & c) const
    {
        const auto cached = preblocked_cache_.find(c);
        if (cached != preblocked_cache_.end()) return cached->second;
        bool blocked = false;
        if (isInsideMetricBounds(c) && !isOccupiedCell(c)) {
            blocked = external_preblocked_cells_.count(c) != 0;
            bool candidate = false;
            for (int dx = -1; dx <= 1 && !candidate; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    if ((dx != 0 || dy != 0) &&
                        isOccupiedLeafCell({c.x + dx, c.y + dy, c.z})) {
                        candidate = true;
                        break;
                    }
                }
            }
            if (!blocked && candidate) {
                const GridIndex below{c.x, c.y, c.z - 1};
                const bool belowOccupied = isInsideMetricBounds(below) && isOccupiedCell(below);
                blocked = (belowOccupied && hasSameLevelNeighborWithOccupiedAbove(c)) ||
                    (hasNonOccupiedNeighborSameLevel(c) && !isOccupiedCell({c.x, c.y, c.z + 1}) &&
                     isInsideMetricBounds(below) && !belowOccupied);
            }
        }
        preblocked_cache_.emplace(c, blocked);
        return blocked;
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

    void OctoPlanner3D::rebuildSurfaceIndex()
    {
        const auto started = std::chrono::steady_clock::now();
        const double headroom = support_height_m_ + std::max(octree_->getResolution(), body_clearance_above_m_);
        const double closing = ground_support_ && !direct_support_only_ ? surface_closing_radius_m_ : 0.0;
        surface_index_ = std::make_shared<const SurfaceIndex>(*octree_, headroom, closing,
            graph_config_.surface_max_hole_area_m2, graph_config_.surface_max_slope);
        printf("OctoPlanner3D surface index: observed=%zu inferred=%zu radius=%.3f prepare_ms=%.3f\n",
            surface_index_->observedSurfaces(), surface_index_->inferredSurfaces(), closing,
            std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count());
    }

    void OctoPlanner3D::rebuildSurfaceGraph()
    {
        surface_graph_.reset();
        if (!ground_support_ || support_height_m_<=0 || lowest_traversable_only_) return;
        const auto started=std::chrono::steady_clock::now();
        const double r=octree_->getResolution();
        const auto cells=surface_index_->surfaces(r);
        std::vector<PointPose> points;
        points.reserve(cells.size());
        const int levels=static_cast<int>(std::ceil((support_height_tolerance_m_+.5*r)/(.5*r)));
        for (const auto &cell : cells) {
            if (cancel_check_ && cancel_check_()) throw SurfaceGraph::Cancelled{};
            const PointPose nominal{(cell.x+.5)*r,(cell.y+.5)*r,(cell.z+.5)*r+support_height_m_};
            std::optional<PointPose> selected;
            // Use the existing stance band instead of fixing every node to one height.
            for (int level=0;level<=levels && !selected;++level) {
                for (int sign : {1,-1}) {
                    if (level==0 && sign<0) continue;
                    auto point=nominal;point.z+=sign*level*.5*r;
                    if (queryWorld({float(point.x),float(point.y),float(point.z)},robot_radius_,true)==TraversabilityFailure::None) {
                        selected=point;break;
                    }
                }
            }
            if (selected) points.push_back(*selected);
        }
        surface_graph_=std::make_shared<SurfaceGraph>(std::move(points),r,graph_config_,
            [](const PointPose &) { return true; }, // Points already passed full static geometry.
            [&](const PointPose &a,const PointPose &b) {
                return metricConnectionWithinLimits(a,b) &&
                    sweptRouteFree({float(a.x),float(a.y),float(a.z)},
                                   {float(b.x),float(b.y),float(b.z)},nullptr,nullptr,true,true);
            },cancel_check_);
        printf("OctoPlanner3D surface graph: nodes=%zu edges=%zu prepare_ms=%.3f\n",
            surface_graph_->nodeCount(),surface_graph_->edgeCount(),
            std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count());
    }

    void OctoPlanner3D::rebuildDerivedLayers()
    {
        traversable_cells_.clear();
        if (!octree_ || !lowest_traversable_only_) {
        return;
        }

        const bool ground_support = ground_support_;
        const bool direct_support_only = direct_support_only_;
        const int support_depth_cells = support_depth_cells_;
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
                idx, robot_radius, ground_support, direct_support_only, support_depth_cells))
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
        return surface_index_->occupied(idx.x, idx.y, idx.z, idx.z);
    }

    bool OctoPlanner3D::isOccupiedLeafCell(const GridIndex & idx) const
    {
        return isOccupiedCell(idx) && cellState(idx) == CellState::OccupiedLeafCenter;
    }

    OctoPlanner3D::CellState OctoPlanner3D::cellState(const GridIndex & idx) const
    {
        const auto block_axis = [](int value) {
            return value >= 0 ? value / 16 : (value + 1) / 16 - 1;
        };
        const GridIndex block_index{block_axis(idx.x), block_axis(idx.y), block_axis(idx.z)};
        if (!last_occupied_block_ || !(last_occupied_block_index_ == block_index)) {
            last_occupied_block_ = &occupied_blocks_.try_emplace(block_index).first->second;
            last_occupied_block_index_ = block_index;
        }
        const auto offset = static_cast<std::size_t>(
            ((idx.x - block_index.x * 16) * 16 + idx.y - block_index.y * 16) * 16 +
            idx.z - block_index.z * 16);
        auto & cached = (*last_occupied_block_)[offset];
        if (cached != CellState::Unqueried) return cached;
        cached = CellState::Unknown;
        if (!isInsideMetricBounds(idx)) {
            return cached;
        }
        const auto p = gridToWorld(idx);
        octomap::OcTreeKey key;
        if (!octree_->coordToKeyChecked(p, key)) return cached;
        const auto * node = octree_->getRoot();
        if (!node) return cached;
        const auto tree_depth = octree_->getTreeDepth();
        unsigned depth = 0;
        // Keep the containing leaf's depth during the occupancy lookup. This
        // gives the exact upstream leaf-centre risk source without a full-map
        // leaf scan and an additional hash entry for every occupied leaf.
        while (depth < tree_depth) {
            const auto child = octomap::computeChildIdx(key, tree_depth - depth - 1);
            if (!octree_->nodeChildExists(node, child)) {
                if (octree_->nodeHasChildren(node)) return cached;
                break;
            }
            node = octree_->getNodeChild(node, child);
            ++depth;
        }
        if (octree_->isNodeOccupied(node)) {
            const auto center = worldToGrid(
                octree_->keyToCoord(key[0], depth),
                octree_->keyToCoord(key[1], depth),
                octree_->keyToCoord(key[2], depth));
            cached = center == idx ? CellState::OccupiedLeafCenter : CellState::Occupied;
        } else {
            cached = CellState::Free;
        }
        return cached;
    }

    double OctoPlanner3D::planningXYResolution() const
    {
        return support_height_m_ > 0.0 && !lowest_traversable_only_
            ? std::max(octree_->getResolution(), planning_resolution_m_ > 0.0
                ? planning_resolution_m_ : search_resolution_m_) : octree_->getResolution();
    }

    double OctoPlanner3D::planningZResolution() const
    {
        const double r = planningXYResolution();
        if (support_height_m_ <= 0.0 || lowest_traversable_only_ || max_slope_ <= 0.0)
            return r;
        // An isotropic 26-neighbour grid can only rise at slopes >= 1/sqrt(2).
        // Subdivide height so one XY step has a slope-admissible rising option.
        return r / std::max(1.0, std::ceil(1.0 / max_slope_));
    }

    GridIndex OctoPlanner3D::worldToPlanningGrid(double x, double y, double z) const
    {
        const double r = planningXYResolution();
        return {static_cast<int>(std::floor((x - planning_offset_.x) / r)),
                static_cast<int>(std::floor((y - planning_offset_.y) / r)),
                static_cast<int>(std::floor((z - planning_offset_.z) / planningZResolution()))};
    }

    void OctoPlanner3D::anchorPlanningLattice(const PointPose &point, double resolution)
    {
        // Grid keys and snap ranges change with the query lattice.
        body_cache_.clear();
        snap_offsets_.clear();
        planning_resolution_m_ = resolution > 0.0 ? resolution : search_resolution_m_;
        planning_offset_ = {};
        if (octree_ && support_height_m_ > 0.0 && !lowest_traversable_only_) {
            const auto center = planningPoint(worldToPlanningGrid(point.x, point.y, point.z));
            planning_offset_ = {point.x - center.x(), point.y - center.y(), point.z - center.z()};
        }
    }

    octomap::point3d OctoPlanner3D::planningPoint(const GridIndex &idx) const
    {
        const double r = planningXYResolution();
        return octomap::point3d((idx.x + .5) * r + planning_offset_.x,
                               (idx.y + .5) * r + planning_offset_.y,
                               (idx.z + .5) * planningZResolution() + planning_offset_.z);
    }

    OctoPlanner3D::TraversabilityFailure OctoPlanner3D::queryWorld(
        const octomap::point3d &p, double radius, bool require_support, bool check_body) const
    {
        if (noGoIntersects(p, p, radius)) return TraversabilityFailure::ExternalPreblockedBody;
        double min_x, min_y, min_z, max_x, max_y, max_z;
        octree_->getMetricMin(min_x, min_y, min_z);
        octree_->getMetricMax(max_x, max_y, max_z);
        if (p.x() < min_x || p.x() >= max_x || p.y() < min_y || p.y() >= max_y ||
            p.z() < min_z || p.z() >= max_z) return TraversabilityFailure::OutsideBounds;
        // Ground evidence is below the body. Missing saved rays through the
        // body-centre air do not invalidate an observed supporting surface.
        const double r = octree_->getResolution();
        if (!external_preblocked_cells_.empty()) {
            const auto column = worldToGrid(p.x(), p.y(), p.z());
            const int bottom = worldToGrid(p.x(), p.y(), min_z).z;
            for (int z = column.z - 1; z >= bottom; --z) {
                const GridIndex voxel{column.x, column.y, z};
                if (isOccupiedCell(voxel)) break;
                if (external_preblocked_cells_.count(voxel))
                    return TraversabilityFailure::ExternalPreblockedBelow;
            }
        }
        const auto supportFailure = [&]() {
            if (!require_support) return TraversabilityFailure::None;
            const GridIndex support_seed = worldToGrid(p.x(), p.y(), p.z());
            const double centre_offset = (support_seed.z + .5) * r - p.z();
            return hasGroundSupport(
                       support_seed, direct_support_only_, support_depth_cells_, centre_offset)
                ? TraversabilityFailure::None
                : TraversabilityFailure::GroundSupport;
        };
        if (!check_body) return supportFailure();
        const bool explicit_body = body_clearance_below_m_ > 0.0 || body_clearance_above_m_ > 0.0;
        const double below = explicit_body ? std::max(0.0, body_clearance_below_m_) : r;
        const double above = explicit_body ? std::max(0.0, body_clearance_above_m_) : r;
        const auto low = worldToGrid(p.x() - radius, p.y() - radius, p.z() - below);
        const auto high = worldToGrid(p.x() + radius, p.y() + radius, p.z() + above);
        if (external_preblocked_cells_.empty()) {
            const int bottom=static_cast<int>(std::floor((p.z()-below+1e-7)/r));
            const int top=static_cast<int>(std::ceil((p.z()+above-1e-7)/r))-1;
            if (!surface_index_->regionOccupied(low.x,low.y,high.x,high.y,bottom,top))
                return supportFailure();
            for (int x=low.x; x<=high.x; ++x) {
                const double dx=std::max({x*r-p.x(),0.0,p.x()-(x+1)*r});
                for (int y=low.y; y<=high.y; ++y) {
                    const double dy=std::max({y*r-p.y(),0.0,p.y()-(y+1)*r});
                    if (dx*dx+dy*dy>radius*radius+1e-9) continue;
                    auto z=surface_index_->below(x,y,top+1);
                    if (!z || *z<bottom) continue;
                    if (!start_exemption_) return TraversabilityFailure::OccupiedBody;
                    for (; z && *z>=bottom; z=surface_index_->below(x,y,*z))
                        if (!start_exemption_({(x+.5)*r,(y+.5)*r,(*z+.5)*r},r))
                            return TraversabilityFailure::OccupiedBody;
                }
            }
            return TraversabilityFailure::None;
        }
        for (int x = low.x; x <= high.x; ++x) {
            const double dx = std::max({x * r - p.x(), 0.0, p.x() - (x + 1) * r});
            for (int y = low.y; y <= high.y; ++y) {
                const double dy = std::max({y * r - p.y(), 0.0, p.y() - (y + 1) * r});
                if (dx * dx + dy * dy > radius * radius + 1e-9) continue;
                for (int z = low.z; z <= high.z; ++z) {
                    if ((z + 1) * r <= p.z() - below + 1e-7 || z * r >= p.z() + above - 1e-7)
                        continue;
                    const GridIndex voxel{x, y, z};
                    if (external_preblocked_cells_.count(voxel))
                        return TraversabilityFailure::ExternalPreblockedBody;
                    if (isOccupiedCell(voxel) && (!start_exemption_ ||
                        !start_exemption_({(x+.5)*r,(y+.5)*r,(z+.5)*r},r)))
                        return TraversabilityFailure::OccupiedBody;
                }
            }
        }
        return supportFailure();
    }

    bool OctoPlanner3D::sweptRouteFree(
        const octomap::point3d &from, const octomap::point3d &to,
        TraversabilityFailure *failure_out, octomap::point3d *failed_point, bool check_body,
        bool endpoints_validated, bool require_support) const
    {
        if (failure_out) *failure_out = TraversabilityFailure::None;
        if (noGoIntersects(from, to, robot_radius_)) {
            if (failure_out) *failure_out = TraversabilityFailure::ExternalPreblockedBody;
            if (failed_point) *failed_point = (from + to) * .5f;
            return false;
        }
        const double length = (to - from).norm();
        const int count = std::max(1, static_cast<int>(std::ceil(
            2.0 * length / octree_->getResolution())));
        // Callers can reuse endpoints already checked against the same observations.
        for (int i = endpoints_validated ? 1 : 0; i <= count-(endpoints_validated ? 1 : 0); ++i) {
            const double t = static_cast<double>(i) / count;
            const octomap::point3d point(
                from.x() + t * (to.x() - from.x()),
                from.y() + t * (to.y() - from.y()),
                from.z() + t * (to.z() - from.z()));
            const auto failure = queryWorld(point, robot_radius_, ground_support_ && require_support, check_body);
            if (failure == TraversabilityFailure::None) continue;
            // A step connects two supported stances. Its body moves continuously,
            // while the observed tread height changes discontinuously. Keep the
            // actual body sweep, but do not require its interpolated height to
            // remain the nominal standing height above either tread.
            if (failure != TraversabilityFailure::GroundSupport ||
                std::abs(to.z() - from.z()) < 1e-7 || i == 0 || i == count ||
                queryWorld(point, robot_radius_, false, check_body) != TraversabilityFailure::None) {
                if (failure_out) *failure_out = failure;
                if (failed_point) *failed_point = point;
                return false;
            }
            bool supported = false;
            const double lower_z = std::min(from.z(), to.z());
            const double upper_z = std::max(from.z(), to.z());
            const int levels = std::max(1, static_cast<int>(std::ceil(
                (upper_z - lower_z) / octree_->getResolution())));
            for (int level = 0; level <= levels && !supported; ++level) {
                auto stance = point;
                stance.z() = static_cast<float>(lower_z +
                    (upper_z - lower_z) * static_cast<double>(level) / levels);
                // Reuse the same first-surface, height and neighbouring
                // support rules at this XY, bounded by the endpoint stances.
                supported = queryWorld(stance, 0.0, true, check_body) == TraversabilityFailure::None;
            }
            if (!supported) {
                if (failure_out) *failure_out = failure;
                if (failed_point) *failed_point = point;
                return false;
            }
        }
        return true;
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

}  // namespace global_planner

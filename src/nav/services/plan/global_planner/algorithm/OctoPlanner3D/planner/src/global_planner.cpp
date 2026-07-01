#include "global_planner.h"

namespace global_planner
{

    GlobalPlanner::GlobalPlanner():map_ready_(false),has_start_(false),has_goal_(false),planning_in_progress_(false),plan_seq_(0),last_success_seq_(0)
    {
        printf("GlobalPlanner Constructure!!! \n");
    }

    GlobalPlanner::~GlobalPlanner()
    {
        printf("GlobalPlanner Destructure!!! \n");
    }

    void GlobalPlanner::setOctomap(std::shared_ptr<octomap::OcTree> map)
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
    }

    void GlobalPlanner::makePlan(const PointPose start,const PointPose goal)
    {
        start_point_ = start;
        has_start_ = true;

        goal_point_ = goal;
        has_goal_ = true;

        printf("start = (%f,%f,%f),goal = (%f,%f,%f) \n",start_point_.x,start_point_.y,start_point_.z,goal_point_.x,goal_point_.y,goal_point_.z);
         
        tryPlan();

    }

    void GlobalPlanner::tryPlan()
    {
        printf("GlobalPlanner::tryPlan planning...\n");
        if (!map_ready_ || !has_start_ || !has_goal_ || planning_in_progress_) 
        {
            printf("GlobalPlanner::tryPlan 异常退出规划器\n");
            return;
        }
        planning_in_progress_ = true;
        ++plan_seq_;
        const bool ok = startPlan();
        planning_in_progress_ = false;
        if (!ok) 
        {
            printf("GlobalPlanner::tryPlan() A* planning failed. \n");
        } 
        else 
        {
            last_success_seq_ = plan_seq_;
        }
    }

    void GlobalPlanner::getPlannerResults(std::vector<PointPose>& plannerResults)
    {
       plannerResults = planner_results_;
    }

    bool GlobalPlanner::startPlan()
    {
        const double robot_radius = robot_radius_;
        const int max_iterations = max_iterations_;
        const int snap_radius = snap_search_radius_cells_;
        const bool require_ground_support = require_ground_support_; 
        const bool strict_direct_ground_support = strict_direct_ground_support_;
        const int support_xy_radius_cells =  ground_support_xy_radius_cells_;
        const int support_depth_cells = ground_support_depth_cells_;
        const bool enable_preblocked_costmap =  enable_preblocked_costmap_;
        const double preblocked_costmap_weight = preblocked_costmap_weight_;

        const GridIndex start_raw = worldToGrid(
        start_point_.x, start_point_.y, start_point_.z);
        const GridIndex goal_raw = worldToGrid(
        goal_point_.x, goal_point_.y, goal_point_.z);

        GridIndex start = start_raw;
        GridIndex goal = goal_raw;
        const bool start_ok = findNearestFreeCell(
        start_raw, robot_radius, snap_radius, require_ground_support, strict_direct_ground_support,
        support_xy_radius_cells, support_depth_cells, start);
        const bool goal_ok = findNearestFreeCell(
        goal_raw, robot_radius, snap_radius, require_ground_support, strict_direct_ground_support,
        support_xy_radius_cells, support_depth_cells, goal);

        if (!start_ok) 
        {
            printf("GlobalPlanner::startPlan() Start is occupied/out of map and no nearby free cell.\n");
            return false;
        }

        if (!goal_ok) 
        {
            printf("GlobalPlanner::startPlan() Goal is occupied/out of map and no nearby free cell.\n");
            return false;
        }

        if (!(start == start_raw)) 
        {
            const auto p = gridToWorld(start);
            printf("GlobalPlanner::startPlan() Start snapped to free cell: [%.2f, %.2f, %.2f] \n",p.x(), p.y(), p.z());
        }

        if (!(goal == goal_raw))
        {
            const auto p = gridToWorld(goal);
            printf("GlobalPlanner::startPlan() Goal snapped to free cell: [%.2f, %.2f, %.2f] \n",p.x(), p.y(), p.z());
        }

        std::priority_queue<QueueNode, std::vector<QueueNode>, QueueNodeCompare> open_set;
        std::unordered_map<GridIndex, double, GridIndexHash> g_score;
        std::unordered_map<GridIndex, GridIndex, GridIndexHash> came_from;
        std::unordered_set<GridIndex, GridIndexHash> closed_set;

        g_score[start] = 0.0;
        open_set.push(QueueNode{start, euclidean(start, goal), 0.0});

        const std::vector<GridIndex> directions = make26Directions();
        int iters = 0;

        while (!open_set.empty() && iters < max_iterations) 
        {
            const QueueNode current = open_set.top();
            open_set.pop();
            ++iters;

            if (closed_set.find(current.idx) != closed_set.end()) {
                continue;
            }
            closed_set.insert(current.idx);

            if (current.idx == goal) {
                const auto cells = reconstructPath(came_from, current.idx);
                printf("GlobalPlanner::startPlan() A* path found in %d iterations. waypoints=%zu \n", iters, cells.size());
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

                return true;
            }

            for (const auto & d : directions) 
            {
                GridIndex nbr{current.idx.x + d.x, current.idx.y + d.y, current.idx.z + d.z};
                if (closed_set.find(nbr) != closed_set.end()) {
                continue;
                }
                if (!isCellTraversable(
                    nbr, robot_radius, require_ground_support, strict_direct_ground_support,
                    support_xy_radius_cells, support_depth_cells))
                {
                continue;
                }
                const double step_cost = euclidean(current.idx, nbr);
                double tentative_g = current.g + step_cost;
                if (enable_preblocked_costmap) {
                tentative_g += preblocked_costmap_weight * getPreblockedCost(nbr);
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

        return false;
    }

    std::vector<GridIndex> GlobalPlanner::reconstructPath(const std::unordered_map<GridIndex, GridIndex, GridIndexHash> & came_from,GridIndex current) const
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

    bool GlobalPlanner::findNearestFreeCell(const GridIndex & seed, double robot_radius, int radius_cells, bool require_ground_support,bool strict_direct_ground_support, int support_xy_radius_cells, int support_depth_cells,GridIndex & out) const
    {
        if (isCellTraversable(
            seed, robot_radius, require_ground_support, strict_direct_ground_support,
            support_xy_radius_cells, support_depth_cells))
        {
        out = seed;
        return true;
        }

        for (int r = 1; r <= radius_cells; ++r) {
        for (int dz = 0; dz <= r; ++dz) {
            for (int dx = -r; dx <= r; ++dx) {
            for (int dy = -r; dy <= r; ++dy) {
                if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != r) {
                continue;
                }

                GridIndex c1{seed.x + dx, seed.y + dy, seed.z + dz};
                if (isCellTraversable(
                    c1, robot_radius, require_ground_support, strict_direct_ground_support,
                    support_xy_radius_cells, support_depth_cells))
                {
                out = c1;
                return true;
                }

                if (dz > 0) {
                GridIndex c2{seed.x + dx, seed.y + dy, seed.z - dz};
                if (isCellTraversable(
                    c2, robot_radius, require_ground_support, strict_direct_ground_support,
                    support_xy_radius_cells, support_depth_cells))
                {
                    out = c2;
                    return true;
                }
                }
            }
            }
        }
        }
        return false;
    }

    double GlobalPlanner::getPreblockedCost(const GridIndex & idx) const
    {
        const auto it = preblocked_costmap_.find(idx);
        if (it == preblocked_costmap_.end()) {
        return 0.0;
        }
        return it->second;
    }

    std::vector<GridIndex> GlobalPlanner::make26Directions() const
    {
        std::vector<GridIndex> dirs;
        dirs.reserve(26);
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
        return dirs;
    }

    bool GlobalPlanner::isCellTraversable(const GridIndex & idx, double robot_radius, bool require_ground_support,bool strict_direct_ground_support,int support_xy_radius_cells, int support_depth_cells) const
    {
        if (!isInsideMetricBounds(idx)) {
        return false;
        }

        if (require_ground_support &&
        !hasGroundSupport(
            idx, strict_direct_ground_support, support_xy_radius_cells, support_depth_cells))
        {
        return false;
        }

        for (int z = idx.z - 1; z >= 0; --z) {
        const GridIndex below_idx{idx.x, idx.y, z};
        if (isOccupiedCell(below_idx)) {
            break;
        }
        if (preblocked_cells_.find(below_idx) != preblocked_cells_.end()) {
            return false;
        }
        }

        const octomap::point3d center = gridToWorld(idx);
        const double r = octree_->getResolution();
        const int n = std::max(1, static_cast<int>(std::ceil(robot_radius / r)));
        const double radius_sq = robot_radius * robot_radius;

        // Collision check for vehicle body volume (same height and above),
        // while allowing occupied support cells below. Apply the same footprint
        // rule to preblocked cells so a cell is rejected if the vehicle radius
        // overlaps any preblocked voxel.
        for (int dx = -n; dx <= n; ++dx) {
        for (int dy = -n; dy <= n; ++dy) {
            for (int dz = 0; dz <= n; ++dz) {
            const double dist_x = static_cast<double>(dx) * r;
            const double dist_y = static_cast<double>(dy) * r;
            const double dist_z = static_cast<double>(dz) * r;
            const double dist_sq = dist_x * dist_x + dist_y * dist_y + dist_z * dist_z;
            if (dist_sq > radius_sq) {
                continue;
            }
            const octomap::point3d p(
                center.x() + static_cast<float>(dx * r),
                center.y() + static_cast<float>(dy * r),
                center.z() + static_cast<float>(dz * r));
            const GridIndex nearby_idx = worldToGrid(p.x(), p.y(), p.z());
            if (preblocked_cells_.find(nearby_idx) != preblocked_cells_.end()) {
                return false;
            }
            const octomap::OcTreeNode * node = octree_->search(p);
            if (node && octree_->isNodeOccupied(node)) {
                return false;
            }
            }
        }
        }
        return true;
    }

    bool GlobalPlanner::hasGroundSupport(const GridIndex & idx, bool strict_direct_ground_support, int support_xy_radius_cells,int support_depth_cells) const
    {
        if (strict_direct_ground_support) {
        GridIndex below{idx.x, idx.y, idx.z - 1};
        if (!isInsideMetricBounds(below)) {
            return false;
        }
        const auto p = gridToWorld(below);
        const octomap::OcTreeNode * node = octree_->search(p);
        return node && octree_->isNodeOccupied(node);
        }

        for (int dz = 1; dz <= std::max(1, support_depth_cells); ++dz) {
        for (int dx = -support_xy_radius_cells; dx <= support_xy_radius_cells; ++dx) {
            for (int dy = -support_xy_radius_cells; dy <= support_xy_radius_cells; ++dy) {
            GridIndex below{idx.x + dx, idx.y + dy, idx.z - dz};
            if (!isInsideMetricBounds(below)) {
                continue;
            }
            const auto p = gridToWorld(below);
            const octomap::OcTreeNode * node = octree_->search(p);
            if (node && octree_->isNodeOccupied(node)) {
                return true;
            }
            }
        }
        }
        return false;
    }

    void GlobalPlanner::rebuildPreblockedCells()
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
        // publishPreblockedCellsMarker();
    }

    bool GlobalPlanner::hasSameLevelNeighborWithOccupiedAbove(const GridIndex & idx) const
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

    bool GlobalPlanner::hasNonOccupiedNeighborSameLevel(const GridIndex & idx) const
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

    void GlobalPlanner::rebuildDerivedLayers()
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

    bool GlobalPlanner::isInsideMetricBounds(const GridIndex & idx) const
    {
        double min_x, min_y, min_z, max_x, max_y, max_z;
        octree_->getMetricMin(min_x, min_y, min_z);
        octree_->getMetricMax(max_x, max_y, max_z);
        const auto p = gridToWorld(idx);
        return p.x() >= static_cast<float>(min_x) && p.x() <= static_cast<float>(max_x) &&
            p.y() >= static_cast<float>(min_y) && p.y() <= static_cast<float>(max_y) &&
            p.z() >= static_cast<float>(min_z) && p.z() <= static_cast<float>(max_z);
    }

    bool GlobalPlanner::isOccupiedCell(const GridIndex & idx) const
    {
        if (!isInsideMetricBounds(idx)) {
        return false;
        }
        const auto p = gridToWorld(idx);
        const octomap::OcTreeNode * node = octree_->search(p);
        return node && octree_->isNodeOccupied(node);
    }

    GridIndex GlobalPlanner::worldToGrid(double x, double y, double z) const
    {
        const double r = octree_->getResolution();
        return GridIndex{
        static_cast<int>(std::floor(x / r)),
        static_cast<int>(std::floor(y / r)),
        static_cast<int>(std::floor(z / r))};
    }

    octomap::point3d GlobalPlanner::gridToWorld(const GridIndex & idx) const
    {
        const double r = octree_->getResolution();
        return octomap::point3d(
        static_cast<float>((static_cast<double>(idx.x) + 0.5) * r),
        static_cast<float>((static_cast<double>(idx.y) + 0.5) * r),
        static_cast<float>((static_cast<double>(idx.z) + 0.5) * r));
    }

    void GlobalPlanner::rebuildPreblockedCostmap()
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
        // publishRiskCostCloud();
    }







}
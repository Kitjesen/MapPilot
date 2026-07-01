/**
 * @file      main.cpp
 * @brief     3d A star Planner
 * @author    juchunyu <juchunyu@qq.com>
 * @date      2026-05-31 21:00:01
 * @copyright Copyright (c) 2025-2026 Institute of Robotics Planning and Control (IRPC).
 *            All rights reserved.
 */

#include "pcd2octomap_converter.h"
#include "global_planner.h"

#include <fstream>

void savePathAsPcd(
    const std::vector<global_planner::PointPose>& path,
    const std::string& filename)
{
    std::ofstream file(filename);

    file << "# .PCD v0.7 - Point Cloud Data file format\n";
    file << "VERSION 0.7\n";
    file << "FIELDS x y z\n";
    file << "SIZE 4 4 4\n";
    file << "TYPE F F F\n";
    file << "COUNT 1 1 1\n";
    file << "WIDTH " << path.size() << "\n";
    file << "HEIGHT 1\n";
    file << "VIEWPOINT 0 0 0 1 0 0 0\n";
    file << "POINTS " << path.size() << "\n";
    file << "DATA ascii\n";

    for (const auto& p : path)
    {
        file << p.x << " " << p.y << " " << p.z << "\n";
    }
}


int main()
{
    std::shared_ptr<pcd2octomap::Pcd2OctomapConverter> octomap_obj;
    octomap_obj = std::make_shared<pcd2octomap::Pcd2OctomapConverter>();

    octomap_obj->convert();

    // octomap_obj->visualizeWithOctovis();

    std::shared_ptr<global_planner::GlobalPlanner> global_obj;
    global_obj = std::make_shared<global_planner::GlobalPlanner>();

    global_obj->setOctomap(octomap_obj->getOctomap());

    global_planner::PointPose start;
    global_planner::PointPose goal;
    //test 1
    // start.x = 7.900;
    // start.y = -2.700;
    // start.z = 0.3;

    // goal.x = 2.50;
    // goal.y = -0.3;
    // goal.z = 13.7;
    
    //test2
    start.x = 9.30;
    start.y =  0.5;
    start.z = 0.3;

    goal.x = -6.1;
    goal.y = -1.5;
    goal.z = 0.3;

//     rt set to [9.300, 0.500, 0.300]
// [octomap_to_occupied_markers_node-3] [INFO] [1780235263.382200406] [octomap_to_occupied_markers]: Published occupied marker from OctoMap: 55585 voxels
// [jie_path_node-2] [INFO] [1780235264.088126056] [jie_path_node]: Goal set to [-6.100, -1.500, 0.300]


    global_obj->makePlan(start,goal);

    std::vector< global_planner::PointPose> path;
    global_obj->getPlannerResults(path);

    std::cout << "path size = " << path.size() << std::endl;

    savePathAsPcd(path, "planned_path.pcd");



}

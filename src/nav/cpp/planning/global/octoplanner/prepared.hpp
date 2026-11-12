#pragma once

#include "global_planner.h"
#include "planning/global/contract.hpp"
#include <filesystem>

namespace octoplanner3d::runtime {

bool restorePrepared(const std::filesystem::path& directory,
    const std::filesystem::path& source,const lingtu::nav::plan::MapIdentity& identity,const global_planner::PlannerConfig& config,
    const std::shared_ptr<octomap::OcTree>& map,global_planner::OctoPlanner3D& planner);
void savePrepared(const std::filesystem::path& directory,
    const std::filesystem::path& source,const lingtu::nav::plan::MapIdentity& identity,const global_planner::PlannerConfig& config,
    const global_planner::OctoPlanner3D& planner);

} // namespace octoplanner3d::runtime

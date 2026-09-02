#pragma once

#include <cstddef>
#include <optional>
#include <string>

#include "explore_contract.hpp"
#include "dds/frame.hpp"
#include "messages.h"

namespace lingtu::nav::endpoint {

struct ExploreInputGateConfig {
  double odometry_max_age_s{0.50};
  double snapshot_max_age_s{2.00};
  double transform_max_age_s{1.00};
  double future_tolerance_s{0.50};
  std::size_t max_grid_cells{1'000'000U};
};

struct ExplorationSnapshot {
  lingtu::explore::Grid2D grid;
  lingtu::explore::ExploreMapIdentity identity;
  double stamp_s{0.0};
};

struct TimedMapPose {
  lingtu::explore::Pose2D pose;
  double stamp_s{0.0};
};

[[nodiscard]] bool sourceStampFresh(double source_stamp_s, double now_s, double maximum_age_s,
                                    double future_tolerance_s);

[[nodiscard]] std::optional<ExplorationSnapshot>
parseExplorationSnapshot(const lingtu_dds_ExplorationGrid &message,
                         const ExploreInputGateConfig &config,
                         std::string *rejection_reason = nullptr);

[[nodiscard]] std::optional<TimedMapPose>
mapPoseFromOdometry(const lingtu_dds_Odometry &message,
                    const std::optional<RigidTransform> &map_odom, double map_odom_receive_s,
                    double now_s, const ExploreInputGateConfig &config,
                    std::string *rejection_reason = nullptr);

[[nodiscard]] bool snapshotFresh(const ExplorationSnapshot &snapshot, double now_s,
                                 const ExploreInputGateConfig &config,
                                 std::string *rejection_reason = nullptr);

}  // namespace lingtu::nav::endpoint

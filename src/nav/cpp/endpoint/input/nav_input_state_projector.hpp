#pragma once

#include <chrono>
#include <cstddef>
#include <functional>
#include <string>

#include "endpoint_state.hpp"
#include "endpoint_time.hpp"
#include "lingtu_slam.h"
#include "nav_kernel/types.hpp"
#include "plan/input_gate.hpp"
#include "plan/live_obstacle_layer.hpp"
#include "status/nav_status_writer.hpp"
#include "traversability/transform_buffer.hpp"

namespace lingtu::nav::endpoint {

enum class PlannerInputClearSource {
  kMap,
  kCloud,
};

struct NavInputStateProjectorConfig {
  double source_transform_max_gap_s{0.25};
  double cloud_pose_max_gap_s{0.12};
  double driver_control_max_age_s{0.35};
  nav_kernel::Vec3 sensor_offset{};
  bool check_obstacle{true};
  std::size_t max_obstacle_points{20000};
};

struct NavInputStateProjectorActions {
  std::function<void(double, const std::string &, bool)> on_epoch_reset;
  std::function<void(const std::string &)> on_rolling_snapshot_invalidated;
};

class NavInputStateProjector {
 public:
  NavInputStateProjector(EndpointState &state, InputGate &input_gate, TransformBuffer &pose_buffer,
                         TransformBuffer &map_odom_buffer, LiveObstacleLayer &live_obstacles,
                         NavInputStateProjectorConfig config,
                         NavInputStateProjectorActions actions);

  void projectTf(const lingtu_dds_TFMessage &message, double receive_steady_s);
  void projectOdometry(const lingtu_dds_Odometry &message, double receive_steady_s);
  void projectDriverControl(const lingtu_dds_DriverControlState &message,
                            SteadyClock::time_point receive_time);
  void projectCloud(const lingtu_dds_PointCloud2 &message, double receive_steady_s,
                    double receive_wall_s, TimingDiagnostics &timing);
  void projectTerrainMap(const lingtu_dds_PointCloud2 &message, double receive_steady_s,
                         TimingDiagnostics &timing);
  void projectTerrainMapExt(const lingtu_dds_PointCloud2 &message, double receive_steady_s,
                            TimingDiagnostics &timing);
  void clearPlannerInputs(const lingtu_dds_Bool &message, PlannerInputClearSource source);
  void projectTraversability(const lingtu_dds_OccupancyGrid &message, double receive_steady_s);
  void projectLocalizationHealth(const lingtu_dds_Text &message, double receive_steady_s);

  InputGateState evaluateInputGate(double now_steady_s, SteadyClock::time_point now_time);
  double driverControlReceiveAge(SteadyClock::time_point now_time) const;
  std::string driverControlBlocker(SteadyClock::time_point now_time) const;
  bool materializeLiveObstacleSnapshot(TimingDiagnostics &timing);

 private:
  void resetInputEpoch(double epoch_start_s, const std::string &reason, bool clear_motion);
  void clearPlannerInputState();

  EndpointState &state_;
  InputGate &input_gate_;
  TransformBuffer &pose_buffer_;
  TransformBuffer &map_odom_buffer_;
  LiveObstacleLayer &live_obstacles_;
  NavInputStateProjectorConfig config_;
  NavInputStateProjectorActions actions_;
};

}  // namespace lingtu::nav::endpoint

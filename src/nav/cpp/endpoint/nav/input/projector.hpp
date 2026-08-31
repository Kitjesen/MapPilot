#pragma once

#include <chrono>
#include <cstddef>
#include <functional>
#include <string>

#include "input/samples.hpp"
#include "runtime/state.hpp"
#include "runtime/time.hpp"
#include "nav_kernel/types.hpp"
#include "input/gate.hpp"
#include "input/obstacle.hpp"
#include "status/nav_status_writer.hpp"
#include "traversability/transform_buffer.hpp"

namespace lingtu::nav::endpoint {

struct InputConfig {
  double source_transform_max_gap_s{0.25};
  double cloud_pose_max_gap_s{0.12};
  double driver_control_max_age_s{0.35};
  nav_kernel::Vec3 sensor_offset{};
  bool check_obstacle{true};
  std::size_t max_obstacle_points{20000};
};

struct InputActions {
  std::function<void(double, const std::string &, bool)> on_epoch_reset;
  std::function<void(const std::string &)> on_rolling_snapshot_invalidated;
  std::function<void(RollingSegmentExecutionGrid)> on_execution_grid;
};

// Projects native DDS inputs into the endpoint's current input state.
class InputProjector {
 public:
  InputProjector(EndpointState &state, InputGate &gate, TransformBuffer &pose_buffer,
                 TransformBuffer &map_odom_buffer, MotionLayer &obstacles, InputConfig config,
                 InputActions actions);

  void apply(SensorBatch batch, TimingDiagnostics &timing);

  // Pose and frame inputs.
  void projectTf(const InputSample<TransformSample> &sample, double receive_steady_s);
  void projectOdometry(const InputSample<OdometrySample> &sample, double receive_steady_s);

  // Environment inputs.
  void projectCloud(InputSample<PointCloudSample> sample, double receive_steady_s,
                    double receive_wall_s, TimingDiagnostics &timing);
  void projectTerrainMap(InputSample<PointCloudSample> sample, double receive_steady_s,
                         TimingDiagnostics &timing);
  void projectTerrainMapExt(InputSample<PointCloudSample> sample, double receive_steady_s,
                            TimingDiagnostics &timing);
  void clearPlannerInputs(const PlannerClearSample &sample);
  void projectTraversability(InputSample<GridSample> sample, double receive_steady_s);
  void projectLocalTraversability(InputSample<GridSample> sample,
                                  double receive_steady_s);
  void projectLocalCollision(InputSample<LocalCollisionMap> sample,
                             double receive_steady_s);
  bool materializeObstacles(TimingDiagnostics &timing);

  // Health and motion-readiness inputs.
  void projectDriverControl(const DriverControlSample &sample, double receive_steady_s);
  void projectLocalizationHealth(std::string payload, double receive_steady_s);
  InputGateState evaluateGate(double now_steady_s, SteadyClock::time_point now_time);
  double driverAge(SteadyClock::time_point now_time) const;
  std::string driverBlocker(SteadyClock::time_point now_time) const;

 private:
  void projectTerrain(InputSample<PointCloudSample> sample, double receive_steady_s,
                      TimingDiagnostics &timing, bool extended);
  void resetEpoch(double epoch_start_s, const std::string &reason, bool clear_motion);
  void clearPlanState();

  EndpointState &state_;
  InputGate &gate_;
  TransformBuffer &pose_buffer_;
  TransformBuffer &map_odom_buffer_;
  MotionLayer &obstacles_;
  InputConfig config_;
  InputActions actions_;
};

}  // namespace lingtu::nav::endpoint

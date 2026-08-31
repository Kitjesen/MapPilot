#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "status/nav_status_writer.hpp"

namespace lingtu::nav::endpoint {

struct StatusCounters {
  std::uint64_t odom{0};
  std::uint64_t tf{0};
  std::uint64_t goals{0};
  std::uint64_t cancels{0};
  std::uint64_t map_clearing{0};
  std::uint64_t cloud_clearing{0};
  std::uint64_t clouds{0};
  std::uint64_t terrain_maps{0};
  std::uint64_t terrain_map_exts{0};
  std::uint64_t traversability{0};
  std::uint64_t teleop_commands{0};
  std::uint64_t teleop_outputs{0};
  std::uint64_t teleop_stops{0};
  std::uint64_t teleop_slows{0};
  std::uint64_t teleop_limited{0};
  std::uint64_t paths{0};
  std::uint64_t plan_failures{0};
  std::uint64_t outputs{0};
  std::uint64_t cmd_vel{0};
};

// A transport-free view of the endpoint state needed by periodic status.
// Large diagnostic and debug values remain caller-owned.
struct StatusRuntimeState {
  bool has_odom{false};
  bool has_map_odom_tf{false};
  bool path_active{false};
  bool estop_latched{false};
  std::string estop_reason{"clear"};
  bool operator_takeover_latched{false};
  bool teleop_request_active{false};
  bool operator_resume_required{false};
  FinalOutputDiagnostics final_output{};
  DriverControlDiagnostics driver_control{};
  StatusCounters counters{};
  const InputGateState *input_gate{nullptr};
  const CloudSyncDiagnostics *cloud_sync{nullptr};
  const FrameDiagnostics *frames{nullptr};
  const OperatorMotionTransportDiagnostics *operator_motion_transport{nullptr};
  const SensorOrigin *last_sensor_origin{nullptr};
  const PlanDiagnostics *plan{nullptr};
  const LocalDiagnostics *local{nullptr};
  const TeleopDiagnostics *teleop{nullptr};
  const std::vector<nav_kernel::Vec3> *global_path{nullptr};
  const std::vector<nav_kernel::Vec3> *local_path{nullptr};
  const nav_kernel::LocalPlannerDebugSnapshot *local_planner_debug{nullptr};
  const TraversabilityGrid *local_map_traversability{nullptr};
  LocalCollisionStatusView local_collision_map{};
};

struct StatusPlannerSample {
  bool traversability_fresh{false};
  bool terrain_map_fresh{false};
  bool terrain_ext_fresh{false};
  std::size_t obstacle_points{0};
  const std::vector<float> *local_map_obstacle_xyzh{nullptr};
};

struct StatusMotionLayerSample {
  std::size_t live_obstacle_cells{0};
  MotionLayerStats stats{};
  const std::vector<DynamicCluster> *dynamic_clusters{nullptr};
};

using StatusFarInputSample = FarInputStatus;

struct NavStatusPublisherActions {
  std::function<double()> steady_now_s;
  std::function<double()> wall_now_s;
  std::function<StatusPlannerSample(double, TimingDiagnostics &)> sample_planner;
  std::function<StatusMotionLayerSample(double)> sample_motion_layer;
  std::function<StatusFarInputSample()> sample_far_input;
  std::function<ControlLoopHealthSnapshot()> sample_loop_health;
  std::function<void(const std::string &)> log;
};

class NavStatusPublisher {
 public:
  NavStatusPublisher(StatusWriterConfig config, double interval_s,
                     NavStatusPublisherActions actions);
  NavStatusPublisher(StatusWriterConfig config, double interval_s,
                     NavStatusPublisherActions actions, StatusSnapshotFileWriter::Sink sink);

  NavStatusPublisher(const NavStatusPublisher &) = delete;
  NavStatusPublisher &operator=(const NavStatusPublisher &) = delete;

  bool publishIfDue(const StatusRuntimeState &state, const CommandDiagnostics &commands,
                    const TimingDiagnostics &previous_timing, TimingDiagnostics &current_timing);
  void requestImmediate();
  void flush();

 private:
  void validateRuntimeState(const StatusRuntimeState &state) const;

  StatusWriterConfig config_;
  double interval_s_{0.0};
  double next_due_s_{0.0};
  NavStatusPublisherActions actions_;
  StatusSnapshotFileWriter snapshot_writer_;
};

}  // namespace lingtu::nav::endpoint

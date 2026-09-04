#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "input/frame.hpp"
#include "nav_kernel/types.hpp"
#include "runtime/rolling/contract.hpp"
#include "safety/command.hpp"

namespace lingtu::nav::endpoint {

struct SampleHeader {
  double stamp_s{0.0};
  std::string frame_id;
};

template <typename T>
struct InputSample {
  T value{};
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return error.empty(); }
};

struct TransformSample {
  RigidTransform map_odom{};
};

struct OdometrySample {
  SampleHeader header;
  std::string child_frame_id;
  RigidTransform body{};
  nav_kernel::Pose pose{};
  nav_kernel::Vec3 linear_velocity{};
  nav_kernel::Vec3 angular_velocity{};
};

// Source-frame points stored as x/y/z/height tuples. A non-finite height means
// the navigation input layer must derive height after applying the frame transform.
struct PointCloudSample {
  SampleHeader header;
  std::vector<float> xyzh;
};

struct GridSample {
  SampleHeader header;
  TraversabilityGrid grid;
};

struct LocalCollisionMap {
  std::shared_ptr<const std::vector<std::uint8_t>> inflated_occupied_bits;
  int size_x{0};
  int size_y{0};
  int size_z{0};
  double resolution{0.0};
  nav_kernel::Vec3 aabb_min{};
  nav_kernel::Vec3 aabb_max{};
  std::uint64_t reset_epoch{0U};
  std::uint64_t observation_sequence{0U};
  std::uint64_t generation{0U};
  double stamp_s{0.0};
  double receive_stamp_s{0.0};
  bool complete{false};
  bool live{false};

  [[nodiscard]] nav_kernel::LocalCollisionMapView view() const noexcept;
};

struct DriverControlSample {
  double stamp_s{0.0};
  std::uint64_t stamp_ns{0U};
  bool connected{false};
  bool ready{false};
  bool motors_enabled{false};
  bool critical_fault{false};
  bool control_assured{false};
  std::string reason;
  std::string accepted_producer_boot_id;
  std::uint64_t accepted_output_sequence{0U};
  bool last_command_accepted{false};
};

enum class ClearSource {
  Map,
  Cloud,
};

struct PlannerClearSample {
  ClearSource source{ClearSource::Map};
  bool requested{false};
};

struct SensorBatch {
  double receive_steady_s{0.0};
  double receive_wall_s{0.0};
  std::vector<InputSample<TransformSample>> transforms;
  std::vector<InputSample<OdometrySample>> odometry;
  std::optional<DriverControlSample> driver_control;
  std::optional<RollingSegmentExecutionGrid> exploration_grid;
  std::optional<InputSample<PointCloudSample>> obstacles;
  std::optional<InputSample<PointCloudSample>> terrain;
  std::optional<InputSample<PointCloudSample>> terrain_extended;
  std::vector<PlannerClearSample> clears;
  std::optional<InputSample<GridSample>> traversability;
  std::optional<InputSample<GridSample>> local_traversability;
  std::optional<InputSample<LocalCollisionMap>> local_collision;
  std::optional<std::string> localization_health;
};

}  // namespace lingtu::nav::endpoint

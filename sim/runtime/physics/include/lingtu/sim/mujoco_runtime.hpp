#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "lingtu/sim/physics_scene_composer.hpp"
#include "lingtu/sim/runtime_contracts.hpp"

namespace lingtu::sim {

struct BodyPose {
  std::int32_t body_id{0};
  std::string name;
  std::string stable_id;
  std::string instance_id;
  std::string frame_id;
  std::array<double, 3> position_m{};
  std::array<double, 4> quaternion_wxyz{1.0, 0.0, 0.0, 0.0};
  std::array<double, 3> linear_velocity_mps{};
  std::array<double, 3> angular_velocity_rps{};
};

struct JointState {
  std::int32_t joint_id{0};
  std::string name;
  std::string stable_id;
  std::string instance_id;
  std::string frame_id;
  std::vector<double> position_rad;
  std::vector<double> velocity_rps;
};

struct ActuatorState {
  std::int32_t actuator_id{0};
  std::string name;
  std::string stable_id;
  std::string instance_id;
  std::string channel_id;
  double control{0.0};
};

struct SensorState {
  std::int32_t sensor_id{0};
  std::string name;
  std::string stable_id;
  std::string instance_id;
  std::string frame_id;
  std::string sensor_type;
  std::string source_stable_id;
  std::vector<double> values;
};

struct MujocoRayDirection {
  std::array<double, 3> direction_sensor{};
  std::uint32_t offset_time_ns{0};
};

struct MujocoRayHit {
  std::array<float, 3> xyz_sensor{};
  std::array<double, 3> origin_world_m{};
  std::array<double, 3> direction_world{};
  std::array<double, 3> position_world_m{};
  double distance_m{0.0};
  std::string body_stable_id;
  std::string entity_id;
  std::uint32_t offset_time_ns{0};
  std::uint8_t reflectivity{15};
  std::uint8_t tag{0};
  std::uint8_t line{0};
};

struct MujocoRaycastRequest {
  std::string sensor_frame_id;
  std::string session_id;
  std::uint64_t model_generation{0};
  std::uint32_t reset_generation{0};
  std::uint64_t sequence{0};
  std::uint64_t sim_time_ns{0};
  std::vector<MujocoRayDirection> rays;
  double range_min_m{0.1};
  double range_max_m{40.0};
  std::uint8_t reflectivity_proxy{15};
  std::uint8_t unknown_line{0};
};

struct MujocoRaycastFrame {
  std::string session_id;
  std::uint64_t sim_time_ns{0};
  std::uint64_t model_generation{0};
  std::uint32_t reset_generation{0};
  std::uint64_t sequence{0};
  std::vector<MujocoRayHit> hits;
};

struct SimulationSnapshot {
  std::string session_id;
  std::uint64_t sequence{0};
  std::uint64_t physics_step{0};
  std::uint64_t sim_time_ns{0};
  std::uint64_t model_generation{0};
  std::uint32_t reset_generation{0};
  std::vector<BodyPose> bodies;
  std::vector<JointState> joints;
  std::vector<ActuatorState> actuators;
  std::vector<SensorState> sensors;
};

struct KinematicEntityPose {
  std::string entity_id;
  std::array<double, 3> position_m{};
  std::array<double, 4> quaternion_wxyz{1.0, 0.0, 0.0, 0.0};
  std::string body_stable_id;
};

struct KinematicPoseBatch {
  std::string session_id;
  std::uint64_t model_generation{0};
  std::uint32_t reset_generation{0};
  std::uint64_t sequence{0};
  std::uint64_t sim_time_ns{0};
  std::vector<KinematicEntityPose> entities;
};

enum class KinematicPoseResult {
  applied,
  rejected_unavailable,
  rejected_session,
  rejected_stale_model_generation,
  rejected_future_model_generation,
  rejected_stale_reset_generation,
  rejected_future_reset_generation,
  rejected_sequence,
  rejected_time,
  rejected_entity_set,
  rejected_non_finite,
  rejected_invalid_quaternion,
};

[[nodiscard]] std::string_view kinematic_pose_result_name(
    KinematicPoseResult result) noexcept;

struct ActuatorBindingSpec {
  std::string source_id;
  std::string instance_id;
  std::string command_type;
  std::uint64_t stale_timeout_ns{0};
  std::vector<std::string> channels;
};

enum class ActuatorCommandResult {
  applied,
  rejected_unknown_source,
  rejected_session,
  rejected_paused,
  rejected_instance,
  rejected_command_type,
  rejected_stale_model_generation,
  rejected_future_model_generation,
  rejected_stale_reset_generation,
  rejected_future_reset_generation,
  rejected_out_of_order,
  rejected_future_apply_time,
  rejected_stale,
  rejected_invalid_payload,
  rejected_non_finite,
  rejected_out_of_range,
};

[[nodiscard]] std::string_view actuator_command_result_name(
    ActuatorCommandResult result) noexcept;

struct MujocoRuntimeConfig {
  std::filesystem::path model_path;
  std::string initial_keyframe;
};

// Single-owner MuJoCo runtime. All calls must be made from one owning thread.
// Snapshot storage is allocated while loading the model and reused thereafter.
class MujocoRuntime final {
 public:
  explicit MujocoRuntime(MujocoRuntimeConfig config);
  static MujocoRuntime from_plan(PhysicsScenePlan plan);
  ~MujocoRuntime();

  MujocoRuntime(MujocoRuntime &&) noexcept;
  MujocoRuntime &operator=(MujocoRuntime &&) noexcept;

  MujocoRuntime(const MujocoRuntime &) = delete;
  MujocoRuntime &operator=(const MujocoRuntime &) = delete;

  const SimulationSnapshot &advance(std::uint32_t steps = 1);
  const SimulationSnapshot &reset();
  void bind_actuators(ActuatorBindingSpec binding);
  [[nodiscard]] ActuatorCommandResult apply_command(
      const CommandEnvelope &command) noexcept;
  [[nodiscard]] KinematicPoseResult apply_kinematic_poses(
      const KinematicPoseBatch &batch) noexcept;
  void set_paused(bool paused) noexcept;

  [[nodiscard]] bool paused() const noexcept;
  [[nodiscard]] double timestep_seconds() const noexcept;
  [[nodiscard]] const SimulationSnapshot &snapshot() const noexcept;
  [[nodiscard]] MujocoRaycastFrame raycast(const MujocoRaycastRequest &request) const;

 private:
  struct Implementation;
  explicit MujocoRuntime(std::unique_ptr<Implementation> implementation);
  std::unique_ptr<Implementation> implementation_;
};

}  // namespace lingtu::sim

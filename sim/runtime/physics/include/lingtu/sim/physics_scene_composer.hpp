#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

namespace lingtu::sim {

struct PhysicsTransform {
  std::array<double, 3> position_m{};
  std::array<double, 4> quaternion_wxyz{1.0, 0.0, 0.0, 0.0};
};

struct PhysicsRobotSpec {
  std::string instance_id;
  std::filesystem::path model_path;
  std::string attach_root;
  PhysicsTransform spawn;
  std::string initial_keyframe;
};

struct PhysicsPackageIdentity {
  std::string id;
  std::string version;
  std::string kind;
  std::string manifest;
};

struct PhysicsPayloadModelSpec {
  std::filesystem::path mjcf_path;
  std::string attach_root;
};

struct PhysicsPayloadFrameSpec {
  std::string name;
  std::string role;
  std::string parent_frame;
};

struct PhysicsPayloadSpec {
  std::string instance_id;
  std::string namespace_name;
  std::string robot_instance_id;
  PhysicsPackageIdentity package;
  std::string parent_frame;
  std::string parent_body;
  PhysicsTransform mount_transform;
  PhysicsPayloadModelSpec model;
  std::string authority;
  std::string collision_representation;
  std::vector<PhysicsPayloadFrameSpec> frames;
};

struct PhysicsKinematicEntitySpec {
  std::string entity_id;
  std::filesystem::path model_path;
  std::string attach_root;
  PhysicsTransform initial_transform;
};

struct PhysicsGlobalPolicy {
  double timestep_seconds{0.002};
  int integrator{mjINT_EULER};
  int solver{mjSOL_NEWTON};
  int iterations{100};
  std::array<double, 3> gravity_mps2{0.0, 0.0, -9.81};
};

struct PhysicsScenePlan {
  std::string session_id;
  std::uint64_t model_generation{0};
  std::filesystem::path world_model_path;
  PhysicsGlobalPolicy global_policy;
  std::vector<PhysicsRobotSpec> robots;
  std::vector<PhysicsPayloadSpec> payloads;
  std::vector<PhysicsKinematicEntitySpec> kinematic_entities;
};

struct ElementDescriptor {
  std::string stable_id;
  std::string local_name;
  std::int32_t dense_index{-1};
  std::int32_t address{-1};
  std::int32_t width{1};
};

using BodyDescriptor = ElementDescriptor;
using JointDescriptor = ElementDescriptor;
using DofDescriptor = ElementDescriptor;
using ActuatorDescriptor = ElementDescriptor;
using SiteDescriptor = ElementDescriptor;

struct SensorDescriptor : ElementDescriptor {
  // MuJoCo's mjtSensor and mjtObj values are kept as integers at the
  // contract boundary so descriptors remain trivially serializable.
  std::int32_t sensor_type{-1};
  std::int32_t object_type{-1};
  std::int32_t object_index{-1};
  std::string source_stable_id;
};

struct RobotInstanceDescriptor {
  std::string instance_id;
  std::string namespace_name;
  std::int32_t root_body_index{-1};
};

struct KinematicEntityDescriptor {
  std::string entity_id;
  std::string namespace_name;
  std::string body_stable_id;
  std::int32_t root_body_index{-1};
  std::int32_t mocap_index{-1};
};

struct PayloadInstanceDescriptor {
  std::string instance_id;
  std::string namespace_name;
  std::string robot_instance_id;
  std::string root_body_stable_id;
  std::int32_t root_body_index{-1};
};

struct ModelDescriptor {
  std::string session_id;
  std::uint64_t model_generation{0};
  std::vector<BodyDescriptor> bodies;
  std::vector<JointDescriptor> joints;
  std::vector<DofDescriptor> dofs;
  std::vector<ActuatorDescriptor> actuators;
  std::vector<SiteDescriptor> sites;
  std::vector<SensorDescriptor> sensors;
  std::vector<RobotInstanceDescriptor> instances;
  std::vector<PayloadInstanceDescriptor> payloads;
  std::vector<KinematicEntityDescriptor> kinematic_entities;
};

// Composes one session-scoped mjModel. It never parses SessionSpec/YAML; its
// caller must provide the already-resolved PhysicsScenePlan.
class PhysicsSceneComposer final {
 public:
  explicit PhysicsSceneComposer(PhysicsScenePlan plan);
  ~PhysicsSceneComposer();

  PhysicsSceneComposer(PhysicsSceneComposer &&) noexcept;
  PhysicsSceneComposer &operator=(PhysicsSceneComposer &&) noexcept;

  PhysicsSceneComposer(const PhysicsSceneComposer &) = delete;
  PhysicsSceneComposer &operator=(const PhysicsSceneComposer &) = delete;

  [[nodiscard]] const mjModel *model() const noexcept;
  [[nodiscard]] const ModelDescriptor &descriptor() const noexcept;
  void reset_data(mjData *data) const;

 private:
  struct Implementation;
  std::unique_ptr<Implementation> implementation_;
};

}  // namespace lingtu::sim

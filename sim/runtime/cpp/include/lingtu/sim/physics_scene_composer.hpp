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
};

struct PhysicsScenePlan {
  std::string session_digest;
  std::uint64_t model_generation{0};
  std::filesystem::path world_model_path;
  std::vector<PhysicsRobotSpec> robots;
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
using SensorDescriptor = ElementDescriptor;

struct RobotInstanceDescriptor {
  std::string instance_id;
  std::string namespace_name;
  std::int32_t root_body_index{-1};
};

struct ModelDescriptor {
  std::string session_digest;
  std::uint64_t model_generation{0};
  std::vector<BodyDescriptor> bodies;
  std::vector<JointDescriptor> joints;
  std::vector<DofDescriptor> dofs;
  std::vector<ActuatorDescriptor> actuators;
  std::vector<SiteDescriptor> sites;
  std::vector<SensorDescriptor> sensors;
  std::vector<RobotInstanceDescriptor> instances;
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

 private:
  struct Implementation;
  std::unique_ptr<Implementation> implementation_;
};

}  // namespace lingtu::sim

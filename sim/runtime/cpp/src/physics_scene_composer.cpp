#include "lingtu/sim/physics_scene_composer.hpp"

#include "mujoco_spec_compat.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <mujoco/mujoco.h>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

namespace lingtu::sim {
namespace {

using SpecPtr = std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)>;
using ModelPtr = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;

std::runtime_error scene_error(const std::string &message) {
  return std::runtime_error("failed to compose MuJoCo physics scene: " + message);
}

SpecPtr parse_spec(const std::filesystem::path &path) {
  if (path.empty() || !std::filesystem::is_regular_file(path)) {
    throw scene_error("model file does not exist: " + path.string());
  }

  char error[2048]{};
  const std::string filename = path.string();
  SpecPtr spec(mj_parseXML(filename.c_str(), nullptr, error, sizeof(error)), mj_deleteSpec);
  if (!spec) {
    std::string detail = error;
    if (detail.empty()) {
      detail = "MuJoCo returned no parser detail";
    }
    throw scene_error(path.string() + ": " + detail);
  }
  return spec;
}

void validate_transform(const PhysicsTransform &transform, const std::string &context) {
  for (const double value : transform.position_m) {
    if (!std::isfinite(value)) {
      throw scene_error(context + " has a non-finite position");
    }
  }
  double norm_squared = 0.0;
  for (const double value : transform.quaternion_wxyz) {
    if (!std::isfinite(value)) {
      throw scene_error(context + " has a non-finite quaternion");
    }
    norm_squared += value * value;
  }
  if (norm_squared < 1e-24) {
    throw scene_error(context + " has a zero quaternion");
  }
}

std::string local_name_for(const mjModel *model, mjtObj object_type, int dense_index) {
  const char *name = mj_id2name(model, object_type, dense_index);
  if (name != nullptr && name[0] != '\0') {
    return name;
  }
  if (object_type == mjOBJ_BODY && dense_index == 0) {
    return "world";
  }
  const char *type_name = mju_type2Str(object_type);
  return std::string(type_name != nullptr ? type_name : "element") + "_" + std::to_string(dense_index);
}

std::string stable_id_for(const std::string &compiled_name, const PhysicsScenePlan &plan) {
  for (const auto &robot : plan.robots) {
    const std::string prefix = robot.instance_id + "__";
    if (compiled_name.rfind(prefix, 0) == 0) {
      return robot.instance_id + "/" + compiled_name.substr(prefix.size());
    }
  }
  return compiled_name;
}

ElementDescriptor element_descriptor(const mjModel *model,
                                    mjtObj object_type,
                                    int dense_index,
                                    int address,
                                    int width,
                                    const PhysicsScenePlan &plan) {
  const std::string local_name = local_name_for(model, object_type, dense_index);
  return ElementDescriptor{
      stable_id_for(local_name, plan),
      local_name,
      dense_index,
      address,
      width,
  };
}

}  // namespace

struct PhysicsSceneComposer::Implementation {
  explicit Implementation(PhysicsScenePlan input) : plan(std::move(input)) {
    if (plan.session_digest.empty()) {
      throw scene_error("session digest must not be empty");
    }
    if (plan.robots.empty()) {
      throw scene_error("physics scene must contain at least one robot instance");
    }
    if (mj_version() != mjVERSION_HEADER) {
      throw scene_error("MuJoCo header and runtime library versions do not match");
    }

    std::set<std::string> instance_ids;
    for (const auto &robot : plan.robots) {
      if (robot.instance_id.empty() || robot.attach_root.empty()) {
        throw scene_error("robot instance_id and attach_root must not be empty");
      }
      if (robot.instance_id.find("__") != std::string::npos) {
        throw scene_error("robot instance_id must not contain the namespace separator: " + robot.instance_id);
      }
      if (!instance_ids.insert(robot.instance_id).second) {
        throw scene_error("duplicate robot instance_id: " + robot.instance_id);
      }
      validate_transform(robot.spawn, "robot " + robot.instance_id + " spawn");
    }

    SpecPtr parent = parse_spec(plan.world_model_path);
    if (mjs_setDeepCopy(parent.get(), 1) != 0) {
      throw scene_error("MuJoCo refused deep-copy attachment mode");
    }
    mujoco_compat::set_attach_conflict_error(parent.get());
    if (mjs_findBody(parent.get(), "world") == nullptr) {
      throw scene_error("world model has no world body");
    }

    for (const auto &robot : plan.robots) {
      mjsBody *world = mjs_findBody(parent.get(), "world");
      if (world == nullptr) {
        throw scene_error("world body disappeared during attachment");
      }
      SpecPtr child = parse_spec(robot.model_path);
      mjsBody *root = mjs_findBody(child.get(), robot.attach_root.c_str());
      if (root == nullptr) {
        throw scene_error("robot " + robot.instance_id + " has no attach root " + robot.attach_root);
      }
      const std::string prefix = robot.instance_id + "__";
      mjsFrame *mount = mjs_addFrame(world, nullptr);
      if (mount == nullptr) {
        throw scene_error("could not create a mount frame for robot " + robot.instance_id);
      }
      std::copy(robot.spawn.position_m.begin(), robot.spawn.position_m.end(), mount->pos);
      std::copy(robot.spawn.quaternion_wxyz.begin(), robot.spawn.quaternion_wxyz.end(), mount->quat);
      if (mjs_attach(mount->element, root->element, prefix.c_str(), "") == nullptr) {
        const char *detail = mjs_getError(parent.get());
        throw scene_error("could not attach robot " + robot.instance_id +
                          (detail != nullptr ? ": " + std::string(detail) : std::string{}));
      }
      mjsBody *attached = mjs_findBody(parent.get(), (prefix + robot.attach_root).c_str());
      if (attached == nullptr) {
        throw scene_error("attached robot root was not addressable: " + prefix + robot.attach_root);
      }
    }

    model.reset(mj_compile(parent.get(), nullptr));
    if (!model) {
      const char *detail = mjs_getError(parent.get());
      throw scene_error(detail != nullptr ? detail : "MuJoCo compilation failed");
    }
    build_descriptor();
  }

  void build_descriptor() {
    descriptor.session_digest = plan.session_digest;
    descriptor.model_generation = plan.model_generation;

    descriptor.bodies.reserve(static_cast<std::size_t>(model->nbody));
    for (int index = 0; index < model->nbody; ++index) {
      descriptor.bodies.push_back(element_descriptor(model.get(), mjOBJ_BODY, index, -1, 1, plan));
    }

    descriptor.joints.reserve(static_cast<std::size_t>(model->njnt));
    for (int index = 0; index < model->njnt; ++index) {
      const int qpos_width = model->jnt_type[index] == mjJNT_FREE ? 7 :
                             (model->jnt_type[index] == mjJNT_BALL ? 4 : 1);
      descriptor.joints.push_back(element_descriptor(
          model.get(), mjOBJ_JOINT, index, model->jnt_qposadr[index], qpos_width, plan));
    }

    descriptor.dofs.reserve(static_cast<std::size_t>(model->nv));
    for (int joint_index = 0; joint_index < model->njnt; ++joint_index) {
      const std::string joint_local_name = local_name_for(model.get(), mjOBJ_JOINT, joint_index);
      const int first_dof = model->jnt_dofadr[joint_index];
      const int dof_count = model->jnt_type[joint_index] == mjJNT_FREE ? 6 :
                            (model->jnt_type[joint_index] == mjJNT_BALL ? 3 : 1);
      for (int local_index = 0; local_index < dof_count; ++local_index) {
        const int dense_index = first_dof + local_index;
        const std::string compiled_name = joint_local_name + "/dof_" + std::to_string(local_index);
        descriptor.dofs.push_back(ElementDescriptor{
            stable_id_for(compiled_name, plan),
            compiled_name,
            dense_index,
            dense_index,
            1,
        });
      }
    }

    descriptor.actuators.reserve(static_cast<std::size_t>(model->nu));
    for (int index = 0; index < model->nu; ++index) {
      descriptor.actuators.push_back(element_descriptor(model.get(), mjOBJ_ACTUATOR, index, index, 1, plan));
    }

    descriptor.sites.reserve(static_cast<std::size_t>(model->nsite));
    for (int index = 0; index < model->nsite; ++index) {
      descriptor.sites.push_back(element_descriptor(model.get(), mjOBJ_SITE, index, index, 1, plan));
    }

    descriptor.sensors.reserve(static_cast<std::size_t>(model->nsensor));
    for (int index = 0; index < model->nsensor; ++index) {
      descriptor.sensors.push_back(element_descriptor(
          model.get(), mjOBJ_SENSOR, index, model->sensor_adr[index], model->sensor_dim[index], plan));
    }

    descriptor.instances.reserve(plan.robots.size());
    for (const auto &robot : plan.robots) {
      const std::string compiled_root = robot.instance_id + "__" + robot.attach_root;
      const int root_index = mj_name2id(model.get(), mjOBJ_BODY, compiled_root.c_str());
      if (root_index < 0) {
        throw scene_error("descriptor cannot find attached root body: " + compiled_root);
      }
      descriptor.instances.push_back(RobotInstanceDescriptor{
          robot.instance_id,
          robot.instance_id,
          root_index,
      });
    }
  }

  PhysicsScenePlan plan;
  ModelPtr model{nullptr, mj_deleteModel};
  ModelDescriptor descriptor;
};

PhysicsSceneComposer::PhysicsSceneComposer(PhysicsScenePlan plan)
    : implementation_(std::make_unique<Implementation>(std::move(plan))) {}

PhysicsSceneComposer::~PhysicsSceneComposer() = default;
PhysicsSceneComposer::PhysicsSceneComposer(PhysicsSceneComposer &&) noexcept = default;
PhysicsSceneComposer &PhysicsSceneComposer::operator=(PhysicsSceneComposer &&) noexcept = default;

const mjModel *PhysicsSceneComposer::model() const noexcept {
  return implementation_->model.get();
}

const ModelDescriptor &PhysicsSceneComposer::descriptor() const noexcept {
  return implementation_->descriptor;
}

}  // namespace lingtu::sim

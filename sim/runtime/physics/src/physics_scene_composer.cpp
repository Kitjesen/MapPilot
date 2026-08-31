#include "lingtu/sim/physics_scene_composer.hpp"

#include "mujoco_spec_compat.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <mujoco/mujoco.h>
#include <sstream>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

namespace lingtu::sim {
namespace {

using SpecPtr = std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)>;
using ModelPtr = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using DataPtr = std::unique_ptr<mjData, decltype(&mj_deleteData)>;

std::runtime_error scene_error(const std::string &message) {
  return std::runtime_error("failed to compose MuJoCo physics scene: " + message);
}

void validate_global_policy(const PhysicsGlobalPolicy &policy) {
  if (!std::isfinite(policy.timestep_seconds) || policy.timestep_seconds <= 0.0) {
    throw scene_error("global policy timestep must be positive and finite");
  }
  if (policy.integrator != mjINT_EULER && policy.integrator != mjINT_RK4 &&
      policy.integrator != mjINT_IMPLICIT && policy.integrator != mjINT_IMPLICITFAST) {
    throw scene_error("global policy integrator is unsupported");
  }
  if (policy.solver != mjSOL_PGS && policy.solver != mjSOL_CG &&
      policy.solver != mjSOL_NEWTON) {
    throw scene_error("global policy solver is unsupported");
  }
  if (policy.iterations <= 0) {
    throw scene_error("global policy iterations must be positive");
  }
  if (!std::all_of(policy.gravity_mps2.begin(), policy.gravity_mps2.end(),
                   [](double value) { return std::isfinite(value); })) {
    throw scene_error("global policy gravity must be finite");
  }
}

void apply_global_policy(mjSpec *spec, const PhysicsGlobalPolicy &policy) {
  spec->option.timestep = policy.timestep_seconds;
  spec->option.integrator = policy.integrator;
  spec->option.solver = policy.solver;
  spec->option.iterations = policy.iterations;
  std::copy(policy.gravity_mps2.begin(), policy.gravity_mps2.end(), spec->option.gravity);
}

void assert_global_policy(const mjModel *model, const PhysicsGlobalPolicy &policy) {
  if (model->opt.timestep != policy.timestep_seconds ||
      model->opt.integrator != policy.integrator ||
      model->opt.solver != policy.solver ||
      model->opt.iterations != policy.iterations ||
      !std::equal(policy.gravity_mps2.begin(), policy.gravity_mps2.end(), model->opt.gravity)) {
    throw scene_error("compiled mjModel.opt does not match the resolved global policy");
  }
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
  for (const auto &payload : plan.payloads) {
    const std::string prefix = payload.namespace_name + "__";
    if (compiled_name.rfind(prefix, 0) == 0) {
      return payload.instance_id + "/" + compiled_name.substr(prefix.size());
    }
  }
  for (const auto &robot : plan.robots) {
    const std::string prefix = robot.instance_id + "__";
    if (compiled_name.rfind(prefix, 0) == 0) {
      return robot.instance_id + "/" + compiled_name.substr(prefix.size());
    }
  }
  for (const auto &entity : plan.kinematic_entities) {
    const std::string prefix = entity.entity_id + "__";
    if (compiled_name.rfind(prefix, 0) == 0) {
      return entity.entity_id + "/" + compiled_name.substr(prefix.size());
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

SensorDescriptor sensor_descriptor(const mjModel *model,
                                   int sensor_index,
                                   const PhysicsScenePlan &plan) {
  const int object_type = model->sensor_objtype[sensor_index];
  const int object_index = model->sensor_objid[sensor_index];
  const std::string local_name =
      local_name_for(model, mjOBJ_SENSOR, sensor_index);
  const std::string stable_id = stable_id_for(local_name, plan);

  std::string source_stable_id;
  if (object_type == mjOBJ_UNKNOWN && object_index < 0) {
    // Clock and user sensors can be model-scoped rather than attached to a
    // body/site. Their own namespaced sensor identity is the stable source.
    source_stable_id = stable_id;
  } else if (object_type >= 0 && object_type < mjNOBJECT && object_index >= 0) {
    const auto source_type = static_cast<mjtObj>(object_type);
    const std::string source_name =
        local_name_for(model, source_type, object_index);
    source_stable_id = stable_id_for(source_name, plan);
  } else {
    throw scene_error("sensor " + local_name +
                      " has inconsistent MuJoCo source metadata");
  }
  if (source_stable_id.empty()) {
    throw scene_error("sensor " + local_name +
                      " has no stable source object identity");
  }

  SensorDescriptor descriptor;
  descriptor.stable_id = stable_id;
  descriptor.local_name = local_name;
  descriptor.dense_index = sensor_index;
  descriptor.address = model->sensor_adr[sensor_index];
  descriptor.width = model->sensor_dim[sensor_index];
  descriptor.sensor_type = model->sensor_type[sensor_index];
  descriptor.object_type = object_type;
  descriptor.object_index = object_index;
  descriptor.source_stable_id = source_stable_id;
  return descriptor;
}

}  // namespace

struct PhysicsSceneComposer::Implementation {
  explicit Implementation(PhysicsScenePlan input) : plan(std::move(input)) {
    if (plan.session_id.empty()) {
      throw scene_error("session id must not be empty");
    }
    if (plan.robots.empty()) {
      throw scene_error("physics scene must contain at least one robot instance");
    }
    validate_global_policy(plan.global_policy);
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
    std::set<std::string> payload_ids;
    for (const auto &payload : plan.payloads) {
      if (payload.instance_id.empty() || payload.namespace_name.empty() ||
          payload.robot_instance_id.empty() || payload.parent_frame.empty() ||
          payload.parent_body.empty() || payload.model.mjcf_path.empty() ||
          payload.model.attach_root.empty()) {
        throw scene_error(
            "payload identity, parent, and model fields must not be empty");
      }
      if (payload.instance_id.find("__") != std::string::npos ||
          payload.namespace_name.find("__") != std::string::npos) {
        throw scene_error(
            "payload instance_id and namespace must not contain the namespace separator: " +
            payload.instance_id);
      }
      if (!payload_ids.insert(payload.instance_id).second) {
        throw scene_error("duplicate payload instance_id: " + payload.instance_id);
      }
      if (!instance_ids.insert(payload.namespace_name).second) {
        throw scene_error("duplicate attachment namespace: " +
                          payload.namespace_name);
      }
      if (std::none_of(plan.robots.begin(), plan.robots.end(),
                       [&payload](const PhysicsRobotSpec &robot) {
                         return robot.instance_id == payload.robot_instance_id;
                       })) {
        throw scene_error("payload " + payload.instance_id +
                          " references unknown robot instance " +
                          payload.robot_instance_id);
      }
      if (payload.package.id.empty() || payload.package.version.empty() ||
          payload.package.kind != "payload" || payload.package.manifest.empty() ||
          payload.authority != "mujoco" ||
          payload.collision_representation.empty() || payload.frames.empty()) {
        throw scene_error("payload " + payload.instance_id +
                          " has incomplete or unsupported resolved metadata");
      }
      validate_transform(payload.mount_transform,
                         "payload " + payload.instance_id + " mount transform");
    }
    for (const auto &entity : plan.kinematic_entities) {
      if (entity.entity_id.empty() || entity.attach_root.empty()) {
        throw scene_error(
            "kinematic entity_id and attach_root must not be empty");
      }
      if (entity.entity_id.find("__") != std::string::npos) {
        throw scene_error(
            "kinematic entity_id must not contain the namespace separator: " +
            entity.entity_id);
      }
      if (!instance_ids.insert(entity.entity_id).second) {
        throw scene_error("duplicate attachment namespace: " + entity.entity_id);
      }
      validate_transform(entity.initial_transform,
                         "kinematic entity " + entity.entity_id +
                             " initial transform");
    }

    SpecPtr parent = parse_spec(plan.world_model_path);
    apply_global_policy(parent.get(), plan.global_policy);
    if (mjs_setDeepCopy(parent.get(), 1) != 0) {
      throw scene_error("MuJoCo refused deep-copy attachment mode");
    }
    if (mjs_findBody(parent.get(), "world") == nullptr) {
      throw scene_error("world model has no world body");
    }

    for (const auto &robot : plan.robots) {
      mjsBody *world = mjs_findBody(parent.get(), "world");
      if (world == nullptr) {
        throw scene_error("world body disappeared during attachment");
      }
      SpecPtr child = parse_spec(robot.model_path);
      const auto authored_options =
          mujoco_compat::authored_option_fields(child.get());
      if (!authored_options.empty()) {
        std::ostringstream fields;
        for (std::size_t index = 0; index < authored_options.size(); ++index) {
          if (index != 0) {
            fields << ", ";
          }
          fields << authored_options[index];
        }
        throw scene_error("robot " + robot.instance_id +
                          " authors session-global MuJoCo option fields: " +
                          fields.str());
      }
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

    for (const auto &payload : plan.payloads) {
      const auto robot = std::find_if(
          plan.robots.begin(), plan.robots.end(),
          [&payload](const PhysicsRobotSpec &candidate) {
            return candidate.instance_id == payload.robot_instance_id;
          });
      if (robot == plan.robots.end()) {
        throw scene_error("payload robot disappeared during attachment: " +
                          payload.robot_instance_id);
      }
      const std::string parent_name =
          robot->instance_id + "__" + payload.parent_body;
      mjsBody *parent_body = mjs_findBody(parent.get(), parent_name.c_str());
      if (parent_body == nullptr) {
        throw scene_error("payload " + payload.instance_id +
                          " parent body was not addressable: " + parent_name);
      }
      SpecPtr child = parse_spec(payload.model.mjcf_path);
      const auto authored_options =
          mujoco_compat::authored_option_fields(child.get());
      if (!authored_options.empty()) {
        std::ostringstream fields;
        for (std::size_t index = 0; index < authored_options.size(); ++index) {
          if (index != 0) {
            fields << ", ";
          }
          fields << authored_options[index];
        }
        throw scene_error("payload " + payload.instance_id +
                          " authors session-global MuJoCo option fields: " +
                          fields.str());
      }
      mjsBody *root =
          mjs_findBody(child.get(), payload.model.attach_root.c_str());
      if (root == nullptr) {
        throw scene_error("payload " + payload.instance_id +
                          " has no attach root " + payload.model.attach_root);
      }
      mjsFrame *mount = mjs_addFrame(parent_body, nullptr);
      if (mount == nullptr) {
        throw scene_error("could not create a mount frame for payload " +
                          payload.instance_id);
      }
      std::copy(payload.mount_transform.position_m.begin(),
                payload.mount_transform.position_m.end(), mount->pos);
      std::copy(payload.mount_transform.quaternion_wxyz.begin(),
                payload.mount_transform.quaternion_wxyz.end(), mount->quat);
      const std::string prefix = payload.namespace_name + "__";
      if (mjs_attach(mount->element, root->element, prefix.c_str(), "") ==
          nullptr) {
        const char *detail = mjs_getError(parent.get());
        throw scene_error(
            "could not attach payload " + payload.instance_id +
            (detail != nullptr ? ": " + std::string(detail) : std::string{}));
      }
      if (mjs_findBody(parent.get(),
                       (prefix + payload.model.attach_root).c_str()) == nullptr) {
        throw scene_error("attached payload root was not addressable: " +
                          prefix + payload.model.attach_root);
      }
    }

    for (const auto &entity : plan.kinematic_entities) {
      mjsBody *world = mjs_findBody(parent.get(), "world");
      if (world == nullptr) {
        throw scene_error("world body disappeared during kinematic attachment");
      }
      SpecPtr child = parse_spec(entity.model_path);
      const auto authored_options =
          mujoco_compat::authored_option_fields(child.get());
      if (!authored_options.empty()) {
        std::ostringstream fields;
        for (std::size_t index = 0; index < authored_options.size(); ++index) {
          if (index != 0) {
            fields << ", ";
          }
          fields << authored_options[index];
        }
        throw scene_error("kinematic entity " + entity.entity_id +
                          " authors session-global MuJoCo option fields: " +
                          fields.str());
      }
      mjsBody *root = mjs_findBody(child.get(), entity.attach_root.c_str());
      if (root == nullptr) {
        throw scene_error("kinematic entity " + entity.entity_id +
                          " has no attach root " + entity.attach_root);
      }
      const std::string prefix = entity.entity_id + "__";
      mjsFrame *mount = mjs_addFrame(world, nullptr);
      if (mount == nullptr) {
        throw scene_error("could not create a mount frame for kinematic entity " +
                          entity.entity_id);
      }
      std::copy(entity.initial_transform.position_m.begin(),
                entity.initial_transform.position_m.end(), mount->pos);
      std::copy(entity.initial_transform.quaternion_wxyz.begin(),
                entity.initial_transform.quaternion_wxyz.end(), mount->quat);
      if (mjs_attach(mount->element, root->element, prefix.c_str(), "") ==
          nullptr) {
        const char *detail = mjs_getError(parent.get());
        throw scene_error(
            "could not attach kinematic entity " + entity.entity_id +
            (detail != nullptr ? ": " + std::string(detail) : std::string{}));
      }
    }

    model.reset(mj_compile(parent.get(), nullptr));
    if (!model) {
      const char *detail = mjs_getError(parent.get());
      throw scene_error(detail != nullptr ? detail : "MuJoCo compilation failed");
    }
    assert_global_policy(model.get(), plan.global_policy);
    build_descriptor();
    build_initial_state();
  }

  void build_descriptor() {
    descriptor.session_id = plan.session_id;
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
      descriptor.sensors.push_back(sensor_descriptor(model.get(), index, plan));
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

    descriptor.payloads.reserve(plan.payloads.size());
    for (const auto &payload : plan.payloads) {
      const std::string compiled_root =
          payload.namespace_name + "__" + payload.model.attach_root;
      const int root_index =
          mj_name2id(model.get(), mjOBJ_BODY, compiled_root.c_str());
      if (root_index < 0) {
        throw scene_error("descriptor cannot find attached payload root body: " +
                          compiled_root);
      }
      descriptor.payloads.push_back(PayloadInstanceDescriptor{
          payload.instance_id,
          payload.namespace_name,
          payload.robot_instance_id,
          stable_id_for(compiled_root, plan),
          root_index,
      });
    }

    descriptor.kinematic_entities.reserve(plan.kinematic_entities.size());
    for (const auto &entity : plan.kinematic_entities) {
      const std::string compiled_root =
          entity.entity_id + "__" + entity.attach_root;
      const int root_index =
          mj_name2id(model.get(), mjOBJ_BODY, compiled_root.c_str());
      if (root_index < 0) {
        throw scene_error(
            "descriptor cannot find attached kinematic root body: " +
            compiled_root);
      }
      const int mocap_index = model->body_mocapid[root_index];
      if (mocap_index < 0) {
        throw scene_error("kinematic entity root is not a MuJoCo mocap body: " +
                          compiled_root);
      }
      descriptor.kinematic_entities.push_back(KinematicEntityDescriptor{
          entity.entity_id,
          entity.entity_id,
          stable_id_for(compiled_root, plan),
          root_index,
          mocap_index,
      });
    }
  }

  bool belongs_to(const char *compiled_name, const PhysicsRobotSpec &robot) const {
    if (compiled_name == nullptr) {
      return false;
    }
    const std::string prefix = robot.instance_id + "__";
    return std::string(compiled_name).rfind(prefix, 0) == 0;
  }

  void compose_free_joint_pose(mjtNum *qpos, const PhysicsRobotSpec &robot) const {
    mjtNum spawn_quaternion[4]{};
    std::copy(robot.spawn.quaternion_wxyz.begin(),
              robot.spawn.quaternion_wxyz.end(), spawn_quaternion);
    mju_normalize4(spawn_quaternion);

    mjtNum rotated_position[3]{};
    mju_rotVecQuat(rotated_position, qpos, spawn_quaternion);
    for (int axis = 0; axis < 3; ++axis) {
      qpos[axis] = robot.spawn.position_m[static_cast<std::size_t>(axis)] +
                   rotated_position[axis];
    }
    mjtNum composed_quaternion[4]{};
    mju_mulQuat(composed_quaternion, spawn_quaternion, qpos + 3);
    mju_normalize4(composed_quaternion);
    std::copy(composed_quaternion, composed_quaternion + 4, qpos + 3);
  }

  void rotate_free_joint_velocity(mjtNum *qvel,
                                  const PhysicsRobotSpec &robot) const {
    mjtNum spawn_quaternion[4]{};
    std::copy(robot.spawn.quaternion_wxyz.begin(),
              robot.spawn.quaternion_wxyz.end(), spawn_quaternion);
    mju_normalize4(spawn_quaternion);
    mjtNum rotated_linear[3]{};
    mjtNum rotated_angular[3]{};
    mju_rotVecQuat(rotated_linear, qvel, spawn_quaternion);
    mju_rotVecQuat(rotated_angular, qvel + 3, spawn_quaternion);
    std::copy(rotated_linear, rotated_linear + 3, qvel);
    std::copy(rotated_angular, rotated_angular + 3, qvel + 3);
  }

  void build_initial_state() {
    DataPtr defaults(mj_makeData(model.get()), mj_deleteData);
    if (!defaults) {
      throw scene_error("could not allocate default data for initial state");
    }
    mj_resetData(model.get(), defaults.get());
    if (model->nq > 0) {
      initial_qpos.assign(defaults->qpos, defaults->qpos + model->nq);
    }
    if (model->nv > 0) {
      initial_qvel.assign(defaults->qvel, defaults->qvel + model->nv);
    }
    if (model->na > 0) {
      initial_act.assign(defaults->act, defaults->act + model->na);
    }
    if (model->nu > 0) {
      initial_ctrl.assign(defaults->ctrl, defaults->ctrl + model->nu);
    }
    if (model->nmocap > 0) {
      initial_mocap_pos.assign(defaults->mocap_pos,
                               defaults->mocap_pos + model->nmocap * 3);
      initial_mocap_quat.assign(defaults->mocap_quat,
                                defaults->mocap_quat + model->nmocap * 4);
    }

    for (const auto &robot : plan.robots) {
      if (robot.initial_keyframe.empty()) {
        continue;
      }
      const std::string compiled_keyframe =
          robot.instance_id + "__" + robot.initial_keyframe;
      const int key_id =
          mj_name2id(model.get(), mjOBJ_KEY, compiled_keyframe.c_str());
      if (key_id < 0) {
        throw scene_error("robot " + robot.instance_id +
                          " has no compiled initial keyframe " +
                          compiled_keyframe);
      }
      const mjtNum *key_qpos = model->nq > 0
                                   ? model->key_qpos + key_id * model->nq
                                   : nullptr;
      const mjtNum *key_qvel = model->nv > 0
                                   ? model->key_qvel + key_id * model->nv
                                   : nullptr;
      const mjtNum *key_act = model->na > 0
                                  ? model->key_act + key_id * model->na
                                  : nullptr;
      const mjtNum *key_ctrl = model->nu > 0
                                   ? model->key_ctrl + key_id * model->nu
                                   : nullptr;
      const mjtNum *key_mocap_pos = model->nmocap > 0
          ? model->key_mpos + key_id * model->nmocap * 3
          : nullptr;
      const mjtNum *key_mocap_quat = model->nmocap > 0
          ? model->key_mquat + key_id * model->nmocap * 4
          : nullptr;

      for (int joint_id = 0; joint_id < model->njnt; ++joint_id) {
        if (!belongs_to(mj_id2name(model.get(), mjOBJ_JOINT, joint_id), robot)) {
          continue;
        }
        const int qpos_address = model->jnt_qposadr[joint_id];
        const int dof_address = model->jnt_dofadr[joint_id];
        const int qpos_width = model->jnt_type[joint_id] == mjJNT_FREE ? 7 :
                               model->jnt_type[joint_id] == mjJNT_BALL ? 4 : 1;
        const int dof_width = model->jnt_type[joint_id] == mjJNT_FREE ? 6 :
                              model->jnt_type[joint_id] == mjJNT_BALL ? 3 : 1;
        std::copy(key_qpos + qpos_address,
                  key_qpos + qpos_address + qpos_width,
                  initial_qpos.begin() + qpos_address);
        std::copy(key_qvel + dof_address,
                  key_qvel + dof_address + dof_width,
                  initial_qvel.begin() + dof_address);
        if (model->jnt_type[joint_id] == mjJNT_FREE) {
          compose_free_joint_pose(initial_qpos.data() + qpos_address, robot);
          rotate_free_joint_velocity(initial_qvel.data() + dof_address, robot);
        }
      }

      for (int actuator_id = 0; actuator_id < model->nu; ++actuator_id) {
        if (!belongs_to(
                mj_id2name(model.get(), mjOBJ_ACTUATOR, actuator_id), robot)) {
          continue;
        }
        initial_ctrl[static_cast<std::size_t>(actuator_id)] =
            key_ctrl[actuator_id];
        const int act_address = model->actuator_actadr[actuator_id];
        const int act_count = model->actuator_actnum[actuator_id];
        if (act_address >= 0 && act_count > 0) {
          std::copy(key_act + act_address, key_act + act_address + act_count,
                    initial_act.begin() + act_address);
        }
      }

      for (int body_id = 1; body_id < model->nbody; ++body_id) {
        if (!belongs_to(mj_id2name(model.get(), mjOBJ_BODY, body_id), robot)) {
          continue;
        }
        const int mocap_id = model->body_mocapid[body_id];
        if (mocap_id < 0) {
          continue;
        }
        mjtNum *position = initial_mocap_pos.data() + mocap_id * 3;
        mjtNum *quaternion = initial_mocap_quat.data() + mocap_id * 4;
        std::copy(key_mocap_pos + mocap_id * 3,
                  key_mocap_pos + mocap_id * 3 + 3, position);
        std::copy(key_mocap_quat + mocap_id * 4,
                  key_mocap_quat + mocap_id * 4 + 4, quaternion);
        mjtNum spawn_quaternion[4]{};
        std::copy(robot.spawn.quaternion_wxyz.begin(),
                  robot.spawn.quaternion_wxyz.end(), spawn_quaternion);
        mju_normalize4(spawn_quaternion);
        mjtNum rotated_position[3]{};
        mju_rotVecQuat(rotated_position, position, spawn_quaternion);
        for (int axis = 0; axis < 3; ++axis) {
          position[axis] = robot.spawn.position_m[static_cast<std::size_t>(axis)] +
                           rotated_position[axis];
        }
        mjtNum composed_quaternion[4]{};
        mju_mulQuat(composed_quaternion, spawn_quaternion, quaternion);
        mju_normalize4(composed_quaternion);
        std::copy(composed_quaternion, composed_quaternion + 4, quaternion);
      }
    }
  }

  void reset_data(mjData *data) const {
    if (data == nullptr) {
      throw std::invalid_argument("MuJoCo data must not be null");
    }
    mj_resetData(model.get(), data);
    if (model->nq > 0) {
      std::copy(initial_qpos.begin(), initial_qpos.end(), data->qpos);
    }
    if (model->nv > 0) {
      std::copy(initial_qvel.begin(), initial_qvel.end(), data->qvel);
    }
    if (model->na > 0) {
      std::copy(initial_act.begin(), initial_act.end(), data->act);
    }
    if (model->nu > 0) {
      std::copy(initial_ctrl.begin(), initial_ctrl.end(), data->ctrl);
    }
    if (model->nmocap > 0) {
      std::copy(initial_mocap_pos.begin(), initial_mocap_pos.end(),
                data->mocap_pos);
      std::copy(initial_mocap_quat.begin(), initial_mocap_quat.end(),
                data->mocap_quat);
    }
    data->time = 0.0;
  }

  PhysicsScenePlan plan;
  ModelPtr model{nullptr, mj_deleteModel};
  ModelDescriptor descriptor;
  std::vector<mjtNum> initial_qpos;
  std::vector<mjtNum> initial_qvel;
  std::vector<mjtNum> initial_act;
  std::vector<mjtNum> initial_ctrl;
  std::vector<mjtNum> initial_mocap_pos;
  std::vector<mjtNum> initial_mocap_quat;
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

void PhysicsSceneComposer::reset_data(mjData *data) const {
  implementation_->reset_data(data);
}

}  // namespace lingtu::sim

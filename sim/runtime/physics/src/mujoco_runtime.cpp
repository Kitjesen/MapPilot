#include "lingtu/sim/mujoco_runtime.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mujoco/mujoco.h>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

namespace lingtu::sim {
namespace {

std::runtime_error load_error(const std::filesystem::path &path, const char *detail) {
  std::string message = "failed to load MuJoCo model '" + path.string() + "'";
  if (detail != nullptr && detail[0] != '\0') {
    message += ": ";
    message += detail;
  }
  return std::runtime_error(message);
}

void split_stable_id(const std::string &stable_id,
                     std::string &instance_id,
                     std::string &frame_id) {
  const std::size_t separator = stable_id.find('/');
  if (separator == std::string::npos) {
    instance_id.clear();
    frame_id = stable_id;
    return;
  }
  instance_id = stable_id.substr(0, separator);
  frame_id = stable_id.substr(separator + 1);
}

int top_level_body(const mjModel *model, int body_id) noexcept {
  if (body_id <= 0 || body_id >= model->nbody) {
    return -1;
  }
  while (model->body_parentid[body_id] > 0) {
    body_id = model->body_parentid[body_id];
  }
  return body_id;
}

bool body_belongs_to_subtree(const mjModel *model, int body_id,
                             int root_body_id) noexcept {
  if (body_id <= 0 || body_id >= model->nbody || root_body_id <= 0 ||
      root_body_id >= model->nbody) {
    return false;
  }
  while (body_id > 0) {
    if (body_id == root_body_id) {
      return true;
    }
    body_id = model->body_parentid[body_id];
  }
  return false;
}

const BodyDescriptor *body_descriptor_for_dense_index(
    const ModelDescriptor &descriptor, int body_id) {
  const auto match = std::find_if(
      descriptor.bodies.begin(), descriptor.bodies.end(),
      [body_id](const BodyDescriptor &candidate) {
        return candidate.dense_index == body_id;
      });
  return match == descriptor.bodies.end() ? nullptr : &*match;
}

int raycast_self_root(const ModelDescriptor &descriptor, const mjModel *model,
                      const std::string &stable_frame_id,
                      int frame_body_id) noexcept {
  std::string instance_id;
  std::string frame_id;
  split_stable_id(stable_frame_id, instance_id, frame_id);
  if (!instance_id.empty()) {
    const auto instance = std::find_if(
        descriptor.instances.begin(), descriptor.instances.end(),
        [&instance_id](const RobotInstanceDescriptor &candidate) {
          return candidate.instance_id == instance_id;
        });
    if (instance != descriptor.instances.end() &&
        instance->root_body_index > 0 &&
        instance->root_body_index < model->nbody) {
      return instance->root_body_index;
    }
  }
  return top_level_body(model, frame_body_id);
}

int joint_qpos_width(const mjModel *model, int joint_id) noexcept {
  return model->jnt_type[joint_id] == mjJNT_FREE
             ? 7
             : (model->jnt_type[joint_id] == mjJNT_BALL ? 4 : 1);
}

int joint_dof_width(const mjModel *model, int joint_id) noexcept {
  return model->jnt_type[joint_id] == mjJNT_FREE
             ? 6
             : (model->jnt_type[joint_id] == mjJNT_BALL ? 3 : 1);
}

std::string sensor_type_name(std::int32_t sensor_type) {
  switch (sensor_type) {
    case mjSENS_FRAMEQUAT:
      return "framequat";
    case mjSENS_GYRO:
      return "gyro";
    case mjSENS_ACCELEROMETER:
      return "accelerometer";
    case mjSENS_VELOCIMETER:
      return "velocimeter";
    case mjSENS_FRAMEPOS:
      return "framepos";
    case mjSENS_MAGNETOMETER:
      return "magnetometer";
    case mjSENS_TOUCH:
      return "touch";
    case mjSENS_FORCE:
      return "force";
    case mjSENS_TORQUE:
      return "torque";
    case mjSENS_RANGEFINDER:
      return "rangefinder";
    case mjSENS_JOINTPOS:
      return "jointpos";
    case mjSENS_JOINTVEL:
      return "jointvel";
    case mjSENS_FRAMEANGVEL:
      return "frameangvel";
    case mjSENS_FRAMELINVEL:
      return "framelinearvel";
    case mjSENS_FRAMELINACC:
      return "framelinearacc";
    case mjSENS_FRAMEANGACC:
      return "frameangularacc";
    default:
      return "sensor_" + std::to_string(sensor_type);
  }
}

std::string local_name_for(const mjModel *model, mjtObj object_type,
                           int dense_index) {
  const char *name = mj_id2name(model, object_type, dense_index);
  if (name != nullptr && name[0] != '\0') {
    return name;
  }
  return std::string(mju_type2Str(object_type) != nullptr
                         ? mju_type2Str(object_type)
                         : "element") +
         "_" + std::to_string(dense_index);
}

bool finite_vector3(const std::array<double, 3> &values) noexcept {
  return std::all_of(values.begin(), values.end(), [](double value) {
    return std::isfinite(value);
  });
}

double norm3(const std::array<double, 3> &values) noexcept {
  return std::sqrt(values[0] * values[0] + values[1] * values[1] +
                   values[2] * values[2]);
}

SensorDescriptor standalone_sensor_descriptor(const mjModel *model,
                                              int sensor_index) {
  SensorDescriptor descriptor;
  descriptor.local_name = local_name_for(model, mjOBJ_SENSOR, sensor_index);
  descriptor.stable_id = descriptor.local_name;
  descriptor.dense_index = sensor_index;
  descriptor.address = model->sensor_adr[sensor_index];
  descriptor.width = model->sensor_dim[sensor_index];
  descriptor.sensor_type = model->sensor_type[sensor_index];
  descriptor.object_type = model->sensor_objtype[sensor_index];
  descriptor.object_index = model->sensor_objid[sensor_index];
  if (descriptor.object_type >= 0 && descriptor.object_type < mjNOBJECT &&
      descriptor.object_index >= 0) {
    descriptor.source_stable_id = local_name_for(
        model, static_cast<mjtObj>(descriptor.object_type),
        descriptor.object_index);
  }
  if (descriptor.source_stable_id.empty()) {
    descriptor.source_stable_id = descriptor.local_name;
  }
  return descriptor;
}

}  // namespace

struct MujocoRuntime::Implementation {
  struct ActuatorBinding {
    std::string source_id;
    std::string instance_id;
    std::string command_type;
    std::uint64_t stale_timeout_ns{0};
    std::vector<int> actuator_ids;
    std::uint64_t last_sequence{0};
    std::uint64_t last_apply_time_ns{0};
    bool has_applied_command{false};
  };

  explicit Implementation(const MujocoRuntimeConfig &config) {
    if (config.model_path.empty()) {
      throw std::invalid_argument("MuJoCo model path must not be empty");
    }
    if (!std::filesystem::is_regular_file(config.model_path)) {
      throw load_error(config.model_path, "file does not exist");
    }

    if (mj_version() != mjVERSION_HEADER) {
      throw std::runtime_error("MuJoCo header and runtime library versions do not match");
    }

    char error[1024]{};
    const std::string model_path = config.model_path.string();
    owned_model.reset(mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error)));
    if (!owned_model) {
      throw load_error(config.model_path, error);
    }
    model = owned_model.get();

    initialize(config.initial_keyframe);
  }

  explicit Implementation(PhysicsScenePlan plan) {
    scene = std::make_unique<PhysicsSceneComposer>(std::move(plan));
    model = scene->model();
    snapshot.session_id = scene->descriptor().session_id;
    snapshot.model_generation = scene->descriptor().model_generation;
    initialize({});
  }

  void initialize(const std::string &initial_keyframe) {
    data.reset(mj_makeData(model));
    if (!data) {
      throw std::runtime_error("failed to allocate MuJoCo simulation data");
    }

    if (scene != nullptr) {
      scene->reset_data(data.get());
    } else if (!initial_keyframe.empty()) {
      initial_keyframe_id =
          mj_name2id(model, mjOBJ_KEY, initial_keyframe.c_str());
      if (initial_keyframe_id < 0) {
        throw std::invalid_argument(
            "MuJoCo model does not contain initial keyframe '" +
            initial_keyframe + "'");
      }
      mj_resetDataKeyframe(model, data.get(), initial_keyframe_id);
    }

    mj_forward(model, data.get());
    if (scene == nullptr) {
      standalone_descriptor.model_generation = 0;
      standalone_descriptor.bodies.reserve(
          static_cast<std::size_t>(model->nbody));
      for (int body_id = 0; body_id < model->nbody; ++body_id) {
        ElementDescriptor descriptor;
        descriptor.local_name = local_name_for(model, mjOBJ_BODY, body_id);
        descriptor.stable_id = descriptor.local_name;
        descriptor.dense_index = body_id;
        standalone_descriptor.bodies.push_back(std::move(descriptor));
      }
      standalone_descriptor.sites.reserve(
          static_cast<std::size_t>(model->nsite));
      for (int site_id = 0; site_id < model->nsite; ++site_id) {
        ElementDescriptor descriptor;
        descriptor.local_name = local_name_for(model, mjOBJ_SITE, site_id);
        descriptor.stable_id = descriptor.local_name;
        descriptor.dense_index = site_id;
        descriptor.address = site_id;
        standalone_descriptor.sites.push_back(std::move(descriptor));
      }
      standalone_descriptor.sensors.reserve(
          static_cast<std::size_t>(model->nsensor));
      for (int sensor_id = 0; sensor_id < model->nsensor; ++sensor_id) {
        standalone_descriptor.sensors.push_back(
            standalone_sensor_descriptor(model, sensor_id));
      }
    }
    allocate_snapshot();
    refresh_snapshot();
  }

  ~Implementation() = default;

  Implementation(const Implementation &) = delete;
  Implementation &operator=(const Implementation &) = delete;

  void allocate_snapshot() {
    if (snapshot_allocated) {
      throw std::logic_error("MuJoCo snapshot storage was allocated twice");
    }
    if (model->nbody > static_cast<mjtSize>(std::numeric_limits<int>::max())) {
      throw std::runtime_error("MuJoCo model has too many bodies for stable IDs");
    }
    const int body_count = static_cast<int>(model->nbody);
    snapshot.bodies.reserve(
        static_cast<std::size_t>(body_count > 0 ? body_count - 1 : 0));

    for (int body_id = 1; body_id < body_count; ++body_id) {
      const BodyDescriptor *descriptor = nullptr;
      if (scene != nullptr) {
        const ModelDescriptor &model_descriptor = scene->descriptor();
        descriptor = body_descriptor_for_dense_index(model_descriptor, body_id);
        if (descriptor == nullptr) {
          throw std::runtime_error(
              "composed model descriptor is missing body index " +
              std::to_string(body_id));
        }

        const bool robot_body = std::any_of(
            model_descriptor.instances.begin(), model_descriptor.instances.end(),
            [this, body_id](const RobotInstanceDescriptor &instance) {
              return body_belongs_to_subtree(model, body_id,
                                             instance.root_body_index);
            });
        const bool kinematic_body = std::any_of(
            model_descriptor.kinematic_entities.begin(),
            model_descriptor.kinematic_entities.end(),
            [this, body_id](const KinematicEntityDescriptor &entity) {
              return body_belongs_to_subtree(model, body_id,
                                             entity.root_body_index);
            });
        if (!robot_body && !kinematic_body) {
          continue;
        }
      }

      snapshot.bodies.emplace_back();
      BodyPose &pose = snapshot.bodies.back();
      pose.body_id = body_id;
      const char *name = mj_id2name(model, mjOBJ_BODY, body_id);
      pose.name = name != nullptr ? name : "body_" + std::to_string(body_id);
      pose.stable_id = pose.name;
      pose.frame_id = pose.name;

      if (descriptor != nullptr) {
        pose.stable_id = descriptor->stable_id;
        split_stable_id(pose.stable_id, pose.instance_id, pose.frame_id);
        if (pose.instance_id.empty() || pose.frame_id.empty()) {
          throw std::runtime_error(
              "dynamic composed body has no instance-scoped stable ID: " +
              pose.stable_id);
        }
      }
    }

    snapshot.joints.resize(static_cast<std::size_t>(model->njnt));
    for (int joint_id = 0; joint_id < model->njnt; ++joint_id) {
      JointState &state = snapshot.joints[static_cast<std::size_t>(joint_id)];
      state.joint_id = joint_id;
      const char *name = mj_id2name(model, mjOBJ_JOINT, joint_id);
      state.name = name != nullptr ? name : "joint_" + std::to_string(joint_id);
      state.stable_id = state.name;
      if (scene != nullptr) {
        const auto &joints = scene->descriptor().joints;
        const auto descriptor = std::find_if(
            joints.begin(), joints.end(),
            [joint_id](const JointDescriptor &joint) {
              return joint.dense_index == joint_id;
            });
        if (descriptor == joints.end()) {
          throw std::runtime_error(
              "composed model descriptor is missing joint index " +
              std::to_string(joint_id));
        }
        state.stable_id = descriptor->stable_id;
      }
      split_stable_id(state.stable_id, state.instance_id, state.frame_id);
      state.position_rad.resize(
          static_cast<std::size_t>(joint_qpos_width(model, joint_id)));
      state.velocity_rps.resize(
          static_cast<std::size_t>(joint_dof_width(model, joint_id)));
    }

    snapshot.actuators.resize(static_cast<std::size_t>(model->nu));
    actuator_owners.resize(static_cast<std::size_t>(model->nu));
    for (int actuator_id = 0; actuator_id < model->nu; ++actuator_id) {
      ActuatorState &state =
          snapshot.actuators[static_cast<std::size_t>(actuator_id)];
      state.actuator_id = actuator_id;
      const char *name = mj_id2name(model, mjOBJ_ACTUATOR, actuator_id);
      state.name =
          name != nullptr ? name : "actuator_" + std::to_string(actuator_id);
      state.stable_id = state.name;
      if (scene != nullptr) {
        const auto &actuators = scene->descriptor().actuators;
        const auto descriptor = std::find_if(
            actuators.begin(), actuators.end(),
            [actuator_id](const ActuatorDescriptor &actuator) {
              return actuator.dense_index == actuator_id;
            });
        if (descriptor == actuators.end()) {
          throw std::runtime_error(
              "composed model descriptor is missing actuator index " +
              std::to_string(actuator_id));
        }
        state.stable_id = descriptor->stable_id;
      }
      split_stable_id(state.stable_id, state.instance_id, state.channel_id);
    }

    const ModelDescriptor &model_descriptor = descriptor();
    if (model_descriptor.sensors.size() !=
        static_cast<std::size_t>(model->nsensor)) {
      throw std::runtime_error("model descriptor sensor count does not match MuJoCo model");
    }
    snapshot.sensors.resize(static_cast<std::size_t>(model->nsensor));
    for (int sensor_id = 0; sensor_id < model->nsensor; ++sensor_id) {
      const SensorDescriptor &descriptor =
          model_descriptor.sensors[static_cast<std::size_t>(sensor_id)];
      if (descriptor.dense_index != sensor_id || descriptor.address < 0 ||
          descriptor.width < 0 ||
          descriptor.address + descriptor.width > model->nsensordata) {
        throw std::runtime_error("invalid MuJoCo sensor descriptor for sensor " +
                                 std::to_string(sensor_id));
      }
      SensorState &state = snapshot.sensors[static_cast<std::size_t>(sensor_id)];
      state.sensor_id = sensor_id;
      state.name = descriptor.local_name;
      state.stable_id = descriptor.stable_id;
      state.source_stable_id = descriptor.source_stable_id;
      state.sensor_type = sensor_type_name(descriptor.sensor_type);
      split_stable_id(state.stable_id, state.instance_id, state.frame_id);
      std::string source_instance;
      std::string source_frame;
      split_stable_id(state.source_stable_id, source_instance, source_frame);
      if (!source_instance.empty()) {
        state.instance_id = source_instance;
      }
      state.frame_id = source_frame;
      state.values.resize(static_cast<std::size_t>(descriptor.width));
    }
    snapshot_allocated = true;
  }

  void refresh_snapshot() noexcept {
    snapshot.sim_time_ns = static_cast<std::uint64_t>(std::llround(data->time * 1'000'000'000.0));

    for (BodyPose &pose : snapshot.bodies) {
      const int body_id = pose.body_id;
      const mjtNum *position = data->xpos + 3 * body_id;
      const mjtNum *quaternion = data->xquat + 4 * body_id;
      mjtNum velocity[6]{};
      mj_objectVelocity(model, data.get(), mjOBJ_BODY, body_id, velocity, 0);

      for (std::size_t axis = 0; axis < pose.position_m.size(); ++axis) {
        pose.position_m[axis] = position[axis];
      }
      for (std::size_t axis = 0; axis < pose.quaternion_wxyz.size(); ++axis) {
        pose.quaternion_wxyz[axis] = quaternion[axis];
      }
      for (std::size_t axis = 0; axis < pose.linear_velocity_mps.size(); ++axis) {
        pose.angular_velocity_rps[axis] = velocity[axis];
        pose.linear_velocity_mps[axis] = velocity[axis + 3];
      }
    }

    for (int joint_id = 0; joint_id < model->njnt; ++joint_id) {
      JointState &state = snapshot.joints[static_cast<std::size_t>(joint_id)];
      const int qpos_address = model->jnt_qposadr[joint_id];
      const int dof_address = model->jnt_dofadr[joint_id];
      for (std::size_t index = 0; index < state.position_rad.size(); ++index) {
        state.position_rad[index] = data->qpos[qpos_address + static_cast<int>(index)];
      }
      for (std::size_t index = 0; index < state.velocity_rps.size(); ++index) {
        state.velocity_rps[index] = data->qvel[dof_address + static_cast<int>(index)];
      }
    }
    const auto &sensors = descriptor().sensors;
    for (std::size_t sensor_id = 0; sensor_id < sensors.size(); ++sensor_id) {
      const SensorDescriptor &descriptor = sensors[sensor_id];
      SensorState &state = snapshot.sensors[sensor_id];
      const mjtNum *source = data->sensordata + descriptor.address;
      for (std::size_t value_index = 0; value_index < state.values.size();
           ++value_index) {
        state.values[value_index] = source[value_index];
      }
    }
    refresh_actuator_state();
  }

  const ModelDescriptor &descriptor() const noexcept {
    return scene != nullptr ? scene->descriptor() : standalone_descriptor;
  }

  void refresh_actuator_state() noexcept {
    for (int actuator_id = 0; actuator_id < model->nu; ++actuator_id) {
      snapshot.actuators[static_cast<std::size_t>(actuator_id)].control =
          data->ctrl[actuator_id];
    }
  }

  void zero_binding(ActuatorBinding &binding) noexcept {
    for (const int actuator_id : binding.actuator_ids) {
      data->ctrl[actuator_id] = 0.0;
    }
    binding.has_applied_command = false;
    refresh_actuator_state();
  }

  ActuatorCommandResult reject(ActuatorBinding &binding,
                               ActuatorCommandResult result) noexcept {
    zero_binding(binding);
    return result;
  }

  void expire_stale_controls() noexcept {
    const std::uint64_t now = snapshot.sim_time_ns;
    for (auto &entry : bindings) {
      ActuatorBinding &binding = entry.second;
      if (binding.has_applied_command &&
          now >= binding.last_apply_time_ns &&
          now - binding.last_apply_time_ns >= binding.stale_timeout_ns) {
        zero_binding(binding);
      }
    }
  }

  void clear_command_history() noexcept {
    for (auto &entry : bindings) {
      ActuatorBinding &binding = entry.second;
      binding.last_sequence = 0;
      binding.last_apply_time_ns = 0;
      binding.has_applied_command = false;
    }
  }

  void zero_all_bindings() noexcept {
    for (auto &entry : bindings) {
      zero_binding(entry.second);
    }
  }

  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> owned_model{nullptr, mj_deleteModel};
  std::unique_ptr<PhysicsSceneComposer> scene;
  ModelDescriptor standalone_descriptor;
  const mjModel *model{nullptr};
  std::unique_ptr<mjData, decltype(&mj_deleteData)> data{nullptr, mj_deleteData};
  bool paused{false};
  int initial_keyframe_id{-1};
  SimulationSnapshot snapshot;
  std::unordered_map<std::string, ActuatorBinding> bindings;
  std::vector<std::string> actuator_owners;
  std::uint64_t last_kinematic_sequence{0};
  bool has_applied_kinematic_pose{false};
  bool snapshot_allocated{false};
};

MujocoRuntime::MujocoRuntime(MujocoRuntimeConfig config)
    : implementation_(std::make_unique<Implementation>(config)) {}

MujocoRuntime::MujocoRuntime(std::unique_ptr<Implementation> implementation)
    : implementation_(std::move(implementation)) {}

MujocoRuntime MujocoRuntime::from_plan(PhysicsScenePlan plan) {
  return MujocoRuntime(std::make_unique<Implementation>(std::move(plan)));
}

MujocoRuntime::~MujocoRuntime() = default;
MujocoRuntime::MujocoRuntime(MujocoRuntime &&) noexcept = default;
MujocoRuntime &MujocoRuntime::operator=(MujocoRuntime &&) noexcept = default;

const SimulationSnapshot &MujocoRuntime::advance(std::uint32_t steps) {
  if (steps == 0) {
    throw std::invalid_argument("MuJoCo advance requires at least one step");
  }
  if (implementation_->paused) {
    return implementation_->snapshot;
  }

  for (std::uint32_t step = 0; step < steps; ++step) {
    implementation_->expire_stale_controls();
    mj_step(implementation_->model, implementation_->data.get());
    implementation_->snapshot.sim_time_ns = static_cast<std::uint64_t>(
        std::llround(implementation_->data->time * 1'000'000'000.0));
  }

  implementation_->snapshot.physics_step += steps;
  ++implementation_->snapshot.sequence;
  implementation_->refresh_snapshot();
  return implementation_->snapshot;
}

const SimulationSnapshot &MujocoRuntime::reset() {
  if (implementation_->scene != nullptr) {
    implementation_->scene->reset_data(implementation_->data.get());
  } else if (implementation_->initial_keyframe_id >= 0) {
    mj_resetDataKeyframe(implementation_->model, implementation_->data.get(),
                         implementation_->initial_keyframe_id);
  } else {
    mj_resetData(implementation_->model, implementation_->data.get());
  }
  mj_forward(implementation_->model, implementation_->data.get());

  ++implementation_->snapshot.reset_generation;
  implementation_->snapshot.sequence = 0;
  implementation_->snapshot.physics_step = 0;
  implementation_->last_kinematic_sequence = 0;
  implementation_->has_applied_kinematic_pose = false;
  implementation_->clear_command_history();
  implementation_->zero_all_bindings();
  implementation_->refresh_snapshot();
  return implementation_->snapshot;
}

void MujocoRuntime::bind_actuators(ActuatorBindingSpec binding) {
  Implementation &runtime = *implementation_;
  if (runtime.scene == nullptr) {
    throw std::logic_error(
        "actuator bindings require a session-composed MuJoCo runtime");
  }
  if (runtime.snapshot.physics_step != 0 || runtime.snapshot.sequence != 0) {
    throw std::logic_error("actuator bindings must be declared before physics advances");
  }
  if (binding.source_id.empty() ||
      binding.source_id.size() > kMaxStableIdBytes ||
      binding.instance_id.empty() ||
      binding.instance_id.size() > kMaxStableIdBytes ||
      binding.command_type.empty() ||
      binding.command_type.size() > kMaxCommandTypeBytes ||
      binding.stale_timeout_ns == 0 || binding.channels.empty() ||
      binding.channels.size() > kMaxActuatorValues) {
    throw std::invalid_argument("invalid actuator binding metadata");
  }
  if (binding.command_type != "joint_torque") {
    throw std::invalid_argument(
        "first-slice actuator bindings support joint_torque only; other "
        "command types require an explicit sink-side fail-safe value contract");
  }
  if (runtime.bindings.find(binding.source_id) != runtime.bindings.end()) {
    throw std::invalid_argument("duplicate actuator binding source_id: " +
                                binding.source_id);
  }

  const auto &instances = runtime.scene->descriptor().instances;
  if (std::none_of(instances.begin(), instances.end(),
                   [&binding](const RobotInstanceDescriptor &instance) {
                     return instance.instance_id == binding.instance_id;
                   })) {
    throw std::invalid_argument("actuator binding references unknown instance: " +
                                binding.instance_id);
  }

  std::set<std::string> unique_channels;
  std::vector<int> actuator_ids;
  actuator_ids.reserve(binding.channels.size());
  const auto &descriptors = runtime.scene->descriptor().actuators;
  for (const std::string &channel : binding.channels) {
    if (channel.empty() || !unique_channels.insert(channel).second) {
      throw std::invalid_argument(
          "actuator binding channels must be non-empty and unique");
    }
    const std::string stable_id = binding.instance_id + "/" + channel;
    const auto descriptor = std::find_if(
        descriptors.begin(), descriptors.end(),
        [&stable_id](const ActuatorDescriptor &candidate) {
          return candidate.stable_id == stable_id;
        });
    if (descriptor == descriptors.end()) {
      throw std::invalid_argument("actuator binding cannot resolve " + stable_id);
    }
    const int actuator_id = descriptor->dense_index;
    const std::string &owner =
        runtime.actuator_owners[static_cast<std::size_t>(actuator_id)];
    if (!owner.empty()) {
      throw std::invalid_argument("actuator " + stable_id +
                                  " is already owned by " + owner);
    }
    actuator_ids.push_back(actuator_id);
  }

  Implementation::ActuatorBinding resolved;
  resolved.source_id = std::move(binding.source_id);
  resolved.instance_id = std::move(binding.instance_id);
  resolved.command_type = std::move(binding.command_type);
  resolved.stale_timeout_ns = binding.stale_timeout_ns;
  resolved.actuator_ids = std::move(actuator_ids);
  for (const int actuator_id : resolved.actuator_ids) {
    runtime.actuator_owners[static_cast<std::size_t>(actuator_id)] =
        resolved.source_id;
    runtime.data->ctrl[actuator_id] = 0.0;
  }
  const std::string source_id = resolved.source_id;
  runtime.bindings.emplace(source_id, std::move(resolved));
  runtime.refresh_actuator_state();
}

ActuatorCommandResult MujocoRuntime::apply_command(
    const CommandEnvelope &command) noexcept {
  Implementation &runtime = *implementation_;
  const auto found = runtime.bindings.find(std::string(command.source_id.view()));
  if (found == runtime.bindings.end()) {
    return ActuatorCommandResult::rejected_unknown_source;
  }
  Implementation::ActuatorBinding &binding = found->second;
  if (command.session_id.view() != runtime.snapshot.session_id) {
    return runtime.reject(binding, ActuatorCommandResult::rejected_session);
  }
  if (runtime.paused) {
    return runtime.reject(binding, ActuatorCommandResult::rejected_paused);
  }
  if (command.instance_id.view() != binding.instance_id) {
    return runtime.reject(binding, ActuatorCommandResult::rejected_instance);
  }
  if (command.type.view() != binding.command_type) {
    return runtime.reject(binding, ActuatorCommandResult::rejected_command_type);
  }
  const GenerationStamp active{runtime.snapshot.model_generation,
                               runtime.snapshot.reset_generation};
  if (command.generation.model_generation < active.model_generation) {
    return runtime.reject(
        binding, ActuatorCommandResult::rejected_stale_model_generation);
  }
  if (command.generation.model_generation > active.model_generation) {
    return runtime.reject(
        binding, ActuatorCommandResult::rejected_future_model_generation);
  }
  if (command.generation.reset_generation < active.reset_generation) {
    return runtime.reject(
        binding, ActuatorCommandResult::rejected_stale_reset_generation);
  }
  if (command.generation.reset_generation > active.reset_generation) {
    return runtime.reject(
        binding, ActuatorCommandResult::rejected_future_reset_generation);
  }
  if (command.sequence == 0 || command.sequence <= binding.last_sequence ||
      (binding.has_applied_command &&
       command.apply_time_ns < binding.last_apply_time_ns)) {
    return runtime.reject(binding,
                          ActuatorCommandResult::rejected_out_of_order);
  }
  if (command.apply_time_ns > runtime.snapshot.sim_time_ns) {
    return runtime.reject(
        binding, ActuatorCommandResult::rejected_future_apply_time);
  }
  if (runtime.snapshot.sim_time_ns - command.apply_time_ns >=
      binding.stale_timeout_ns) {
    return runtime.reject(binding, ActuatorCommandResult::rejected_stale);
  }
  if (command.payload_size != sizeof(ActuatorCommandPayload)) {
    return runtime.reject(binding,
                          ActuatorCommandResult::rejected_invalid_payload);
  }
  ActuatorCommandPayload payload;
  std::memcpy(&payload, command.payload.data(), sizeof(payload));
  if (payload.value_count != binding.actuator_ids.size() ||
      payload.safe_stop > 1) {
    return runtime.reject(binding,
                          ActuatorCommandResult::rejected_invalid_payload);
  }

  for (std::size_t index = 0; index < payload.value_count; ++index) {
    const double value = payload.values[index];
    if (!std::isfinite(value)) {
      return runtime.reject(binding,
                            ActuatorCommandResult::rejected_non_finite);
    }
    const int actuator_id = binding.actuator_ids[index];
    if (runtime.model->actuator_ctrllimited[actuator_id] != 0) {
      const double lower = runtime.model->actuator_ctrlrange[2 * actuator_id];
      const double upper = runtime.model->actuator_ctrlrange[2 * actuator_id + 1];
      if (value < lower || value > upper) {
        return runtime.reject(binding,
                              ActuatorCommandResult::rejected_out_of_range);
      }
    }
  }
  for (std::size_t index = 0; index < payload.value_count; ++index) {
    runtime.data->ctrl[binding.actuator_ids[index]] = payload.values[index];
  }
  binding.last_sequence = command.sequence;
  binding.last_apply_time_ns = command.apply_time_ns;
  binding.has_applied_command = true;
  runtime.refresh_actuator_state();
  return ActuatorCommandResult::applied;
}

KinematicPoseResult MujocoRuntime::apply_kinematic_poses(
    const KinematicPoseBatch &batch) noexcept {
  Implementation &runtime = *implementation_;
  if (runtime.scene == nullptr) {
    return KinematicPoseResult::rejected_unavailable;
  }
  const auto &entities = runtime.scene->descriptor().kinematic_entities;
  if (entities.empty()) {
    return KinematicPoseResult::rejected_unavailable;
  }
  if (batch.session_id != runtime.snapshot.session_id) {
    return KinematicPoseResult::rejected_session;
  }
  if (batch.model_generation < runtime.snapshot.model_generation) {
    return KinematicPoseResult::rejected_stale_model_generation;
  }
  if (batch.model_generation > runtime.snapshot.model_generation) {
    return KinematicPoseResult::rejected_future_model_generation;
  }
  if (batch.reset_generation < runtime.snapshot.reset_generation) {
    return KinematicPoseResult::rejected_stale_reset_generation;
  }
  if (batch.reset_generation > runtime.snapshot.reset_generation) {
    return KinematicPoseResult::rejected_future_reset_generation;
  }
  if (batch.sequence != runtime.snapshot.sequence) {
    return KinematicPoseResult::rejected_sequence;
  }
  if (runtime.has_applied_kinematic_pose &&
      batch.sequence <= runtime.last_kinematic_sequence) {
    return KinematicPoseResult::rejected_sequence;
  }
  if (batch.sim_time_ns != runtime.snapshot.sim_time_ns) {
    return KinematicPoseResult::rejected_time;
  }
  if (batch.entities.size() != entities.size()) {
    return KinematicPoseResult::rejected_entity_set;
  }

  struct StagedPose {
    int mocap_index{-1};
    std::array<double, 3> position_m{};
    std::array<double, 4> quaternion_wxyz{};
  };
  std::vector<StagedPose> staged;
  std::vector<bool> matched(entities.size(), false);
  staged.reserve(entities.size());
  for (const auto &pose : batch.entities) {
    const auto found = std::find_if(
        entities.begin(), entities.end(),
        [&pose](const KinematicEntityDescriptor &entity) {
          return entity.entity_id == pose.entity_id &&
                 entity.body_stable_id == pose.body_stable_id;
        });
    if (found == entities.end()) {
      return KinematicPoseResult::rejected_entity_set;
    }
    const std::size_t descriptor_index =
        static_cast<std::size_t>(std::distance(entities.begin(), found));
    if (matched[descriptor_index]) {
      return KinematicPoseResult::rejected_entity_set;
    }
    matched[descriptor_index] = true;
    if (!std::all_of(pose.position_m.begin(), pose.position_m.end(),
                     [](double value) { return std::isfinite(value); }) ||
        !std::all_of(pose.quaternion_wxyz.begin(),
                     pose.quaternion_wxyz.end(),
                     [](double value) { return std::isfinite(value); })) {
      return KinematicPoseResult::rejected_non_finite;
    }
    double norm_squared = 0.0;
    for (const double value : pose.quaternion_wxyz) {
      norm_squared += value * value;
    }
    if (norm_squared < 1e-24) {
      return KinematicPoseResult::rejected_invalid_quaternion;
    }
    StagedPose value;
    value.mocap_index = found->mocap_index;
    value.position_m = pose.position_m;
    const double inverse_norm = 1.0 / std::sqrt(norm_squared);
    for (std::size_t index = 0; index < value.quaternion_wxyz.size();
         ++index) {
      value.quaternion_wxyz[index] =
          pose.quaternion_wxyz[index] * inverse_norm;
    }
    staged.push_back(value);
  }

  for (const auto &pose : staged) {
    std::copy(pose.position_m.begin(), pose.position_m.end(),
              runtime.data->mocap_pos + 3 * pose.mocap_index);
    std::copy(pose.quaternion_wxyz.begin(), pose.quaternion_wxyz.end(),
              runtime.data->mocap_quat + 4 * pose.mocap_index);
  }
  mj_forward(runtime.model, runtime.data.get());
  runtime.refresh_snapshot();
  runtime.last_kinematic_sequence = batch.sequence;
  runtime.has_applied_kinematic_pose = true;
  return KinematicPoseResult::applied;
}

void MujocoRuntime::set_paused(bool paused) noexcept {
  if (paused && !implementation_->paused) {
    implementation_->zero_all_bindings();
  }
  implementation_->paused = paused;
}

bool MujocoRuntime::paused() const noexcept {
  return implementation_->paused;
}

double MujocoRuntime::timestep_seconds() const noexcept {
  return implementation_->model->opt.timestep;
}

const SimulationSnapshot &MujocoRuntime::snapshot() const noexcept {
  return implementation_->snapshot;
}

MujocoRaycastFrame MujocoRuntime::raycast(const MujocoRaycastRequest &request) const {
  const Implementation &runtime = *implementation_;
  if (request.sensor_frame_id.empty()) {
    throw std::invalid_argument("raycast sensor_frame_id must not be empty");
  }
  if (request.range_min_m < 0.0 || request.range_max_m <= request.range_min_m ||
      !std::isfinite(request.range_min_m) ||
      !std::isfinite(request.range_max_m)) {
    throw std::invalid_argument("raycast range must be finite and increasing");
  }
  if (request.reflectivity_proxy == 0) {
    throw std::invalid_argument("raycast reflectivity proxy must be non-zero");
  }

  const ModelDescriptor &descriptor = runtime.descriptor();
  if (request.session_id != runtime.snapshot.session_id) {
    throw std::invalid_argument(
        "raycast session_id does not match the active runtime");
  }
  if (request.model_generation != descriptor.model_generation) {
    throw std::invalid_argument(
        "raycast model_generation does not match the active ModelDescriptor");
  }
  if (request.reset_generation != runtime.snapshot.reset_generation) {
    throw std::invalid_argument(
        "raycast reset_generation does not match the active runtime");
  }
  if (request.sequence != runtime.snapshot.sequence) {
    throw std::invalid_argument(
        "raycast sequence does not match the active runtime");
  }
  if (request.sim_time_ns != runtime.snapshot.sim_time_ns) {
    throw std::invalid_argument(
        "raycast sim_time_ns does not match the active runtime");
  }
  if (!descriptor.kinematic_entities.empty() &&
      (!runtime.has_applied_kinematic_pose ||
       request.sequence != runtime.last_kinematic_sequence)) {
    throw std::invalid_argument(
        "raycast scenario stamp has not been applied to kinematic proxies");
  }
  const SiteDescriptor *site = nullptr;
  const BodyDescriptor *body = nullptr;
  std::size_t match_count = 0;
  for (const SiteDescriptor &candidate : descriptor.sites) {
    if (candidate.stable_id == request.sensor_frame_id) {
      site = &candidate;
      ++match_count;
    }
  }
  for (const BodyDescriptor &candidate : descriptor.bodies) {
    if (candidate.stable_id == request.sensor_frame_id) {
      body = &candidate;
      ++match_count;
    }
  }
  if (match_count == 0) {
    throw std::invalid_argument(
        "raycast sensor frame is not in the active ModelDescriptor: " +
        request.sensor_frame_id);
  }
  if (match_count != 1) {
    throw std::invalid_argument(
        "raycast sensor frame is ambiguous in the active ModelDescriptor: " +
        request.sensor_frame_id);
  }

  const mjtNum *origin = nullptr;
  const mjtNum *rotation = nullptr;
  int frame_body_id = -1;
  if (site != nullptr) {
    const int site_id = site->dense_index;
    if (site_id < 0 || site_id >= runtime.model->nsite) {
      throw std::runtime_error("raycast sensor site descriptor is out of range");
    }
    origin = runtime.data->site_xpos + 3 * site_id;
    rotation = runtime.data->site_xmat + 9 * site_id;
    frame_body_id = runtime.model->site_bodyid[site_id];
    if (frame_body_id < 0 || frame_body_id >= runtime.model->nbody) {
      throw std::runtime_error("raycast sensor site body is out of range");
    }
  } else {
    const int body_id = body->dense_index;
    if (body_id < 0 || body_id >= runtime.model->nbody) {
      throw std::runtime_error("raycast sensor body descriptor is out of range");
    }
    origin = runtime.data->xpos + 3 * body_id;
    rotation = runtime.data->xmat + 9 * body_id;
    frame_body_id = body_id;
  }
  const int self_root_body = raycast_self_root(
      descriptor, runtime.model, request.sensor_frame_id, frame_body_id);
  std::array<mjtByte, mjNGROUP> geom_groups{};
  geom_groups.fill(1);

  MujocoRaycastFrame frame;
  frame.session_id = runtime.snapshot.session_id;
  frame.sim_time_ns = runtime.snapshot.sim_time_ns;
  frame.model_generation = runtime.snapshot.model_generation;
  frame.reset_generation = runtime.snapshot.reset_generation;
  frame.sequence = runtime.snapshot.sequence;
  frame.hits.reserve(request.rays.size());

  for (const MujocoRayDirection &ray : request.rays) {
    if (!finite_vector3(ray.direction_sensor)) {
      throw std::invalid_argument("raycast direction contains non-finite data");
    }
    const double length = norm3(ray.direction_sensor);
    if (length <= 0.0) {
      throw std::invalid_argument("raycast direction must be non-zero");
    }
    mjtNum direction_world[3]{};
    for (int row = 0; row < 3; ++row) {
      direction_world[row] =
          (rotation[3 * row + 0] * ray.direction_sensor[0] +
           rotation[3 * row + 1] * ray.direction_sensor[1] +
           rotation[3 * row + 2] * ray.direction_sensor[2]) /
          length;
    }

    std::array<mjtNum, 3> ray_origin{origin[0], origin[1], origin[2]};
    double travelled_m = 0.0;
    double hit_distance_m = -1.0;
    int accepted_hit_body_id = -1;
    const mjtSize maximum_intersections =
        std::max<mjtSize>(16, 2 * runtime.model->ngeom + 8);
    for (mjtSize intersection = 0; intersection < maximum_intersections;
         ++intersection) {
      int hit_geom = -1;
      mjtNum normal[3]{};
      const mjtNum segment_distance =
          mj_ray(runtime.model, runtime.data.get(), ray_origin.data(),
                 direction_world, geom_groups.data(), 1, -1, &hit_geom,
                 normal);
      if (hit_geom < 0 || !std::isfinite(segment_distance) ||
          segment_distance < 0.0) {
        break;
      }

      const double total_distance = travelled_m + segment_distance;
      if (total_distance > request.range_max_m) {
        break;
      }
      const int hit_body_id = runtime.model->geom_bodyid[hit_geom];
      if (body_belongs_to_subtree(runtime.model, hit_body_id,
                                  self_root_body)) {
        constexpr double kSelfIntersectionAdvanceM = 1e-6;
        const double advance_m = segment_distance + kSelfIntersectionAdvanceM;
        travelled_m += advance_m;
        if (travelled_m > request.range_max_m) {
          break;
        }
        for (int axis = 0; axis < 3; ++axis) {
          ray_origin[static_cast<std::size_t>(axis)] +=
              direction_world[axis] * advance_m;
        }
        continue;
      }

      if (total_distance >= request.range_min_m) {
        hit_distance_m = total_distance;
        accepted_hit_body_id = hit_body_id;
      }
      break;
    }
    if (hit_distance_m < 0.0) {
      continue;
    }

    MujocoRayHit hit;
    hit.distance_m = hit_distance_m;
    for (int axis = 0; axis < 3; ++axis) {
      hit.origin_world_m[static_cast<std::size_t>(axis)] = origin[axis];
      hit.direction_world[static_cast<std::size_t>(axis)] = direction_world[axis];
      hit.position_world_m[static_cast<std::size_t>(axis)] =
          origin[axis] + direction_world[axis] * hit_distance_m;
    }
    if (const BodyDescriptor *body_descriptor =
            body_descriptor_for_dense_index(descriptor, accepted_hit_body_id);
        body_descriptor != nullptr) {
      hit.body_stable_id = body_descriptor->stable_id;
      const std::size_t separator = hit.body_stable_id.find('/');
      hit.entity_id = separator == std::string::npos
                          ? hit.body_stable_id
                          : hit.body_stable_id.substr(0, separator);
    } else if (accepted_hit_body_id >= 0 &&
               accepted_hit_body_id < runtime.model->nbody) {
      const char *body_name =
          mj_id2name(runtime.model, mjOBJ_BODY, accepted_hit_body_id);
      if (body_name != nullptr) {
        hit.body_stable_id = body_name;
        hit.entity_id = hit.body_stable_id;
      }
    }
    for (int axis = 0; axis < 3; ++axis) {
      const double value = ray.direction_sensor[static_cast<std::size_t>(axis)] *
                           (hit_distance_m / length);
      if (!std::isfinite(value)) {
        throw std::runtime_error("raycast produced non-finite point data");
      }
      hit.xyz_sensor[static_cast<std::size_t>(axis)] = static_cast<float>(value);
    }
    hit.offset_time_ns = ray.offset_time_ns;
    hit.reflectivity = request.reflectivity_proxy;
    hit.tag = 0;
    hit.line = request.unknown_line;
    frame.hits.push_back(hit);
  }
  return frame;
}

std::string_view actuator_command_result_name(
    ActuatorCommandResult result) noexcept {
  switch (result) {
    case ActuatorCommandResult::applied:
      return "applied";
    case ActuatorCommandResult::rejected_unknown_source:
      return "rejected_unknown_source";
    case ActuatorCommandResult::rejected_session:
      return "rejected_session";
    case ActuatorCommandResult::rejected_paused:
      return "rejected_paused";
    case ActuatorCommandResult::rejected_instance:
      return "rejected_instance";
    case ActuatorCommandResult::rejected_command_type:
      return "rejected_command_type";
    case ActuatorCommandResult::rejected_stale_model_generation:
      return "rejected_stale_model_generation";
    case ActuatorCommandResult::rejected_future_model_generation:
      return "rejected_future_model_generation";
    case ActuatorCommandResult::rejected_stale_reset_generation:
      return "rejected_stale_reset_generation";
    case ActuatorCommandResult::rejected_future_reset_generation:
      return "rejected_future_reset_generation";
    case ActuatorCommandResult::rejected_out_of_order:
      return "rejected_out_of_order";
    case ActuatorCommandResult::rejected_future_apply_time:
      return "rejected_future_apply_time";
    case ActuatorCommandResult::rejected_stale:
      return "rejected_stale";
    case ActuatorCommandResult::rejected_invalid_payload:
      return "rejected_invalid_payload";
    case ActuatorCommandResult::rejected_non_finite:
      return "rejected_non_finite";
    case ActuatorCommandResult::rejected_out_of_range:
      return "rejected_out_of_range";
  }
  return "unknown";
}

std::string_view kinematic_pose_result_name(
    KinematicPoseResult result) noexcept {
  switch (result) {
    case KinematicPoseResult::applied:
      return "applied";
    case KinematicPoseResult::rejected_unavailable:
      return "rejected_unavailable";
    case KinematicPoseResult::rejected_session:
      return "rejected_session";
    case KinematicPoseResult::rejected_stale_model_generation:
      return "rejected_stale_model_generation";
    case KinematicPoseResult::rejected_future_model_generation:
      return "rejected_future_model_generation";
    case KinematicPoseResult::rejected_stale_reset_generation:
      return "rejected_stale_reset_generation";
    case KinematicPoseResult::rejected_future_reset_generation:
      return "rejected_future_reset_generation";
    case KinematicPoseResult::rejected_sequence:
      return "rejected_sequence";
    case KinematicPoseResult::rejected_time:
      return "rejected_time";
    case KinematicPoseResult::rejected_entity_set:
      return "rejected_entity_set";
    case KinematicPoseResult::rejected_non_finite:
      return "rejected_non_finite";
    case KinematicPoseResult::rejected_invalid_quaternion:
      return "rejected_invalid_quaternion";
  }
  return "unknown";
}

}  // namespace lingtu::sim

#include "lingtu/sim/mujoco_runtime.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <mujoco/mujoco.h>
#include <stdexcept>
#include <string>
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

}  // namespace

struct MujocoRuntime::Implementation {
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
    model.reset(mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error)));
    if (!model) {
      throw load_error(config.model_path, error);
    }

    data.reset(mj_makeData(model.get()));
    if (!data) {
      throw std::runtime_error("failed to allocate MuJoCo simulation data");
    }

    mj_forward(model.get(), data.get());
    allocate_snapshot();
    refresh_snapshot();
  }

  ~Implementation() = default;

  Implementation(const Implementation &) = delete;
  Implementation &operator=(const Implementation &) = delete;

  void allocate_snapshot() {
    if (model->nbody > static_cast<mjtSize>(std::numeric_limits<int>::max())) {
      throw std::runtime_error("MuJoCo model has too many bodies for stable IDs");
    }
    const int body_count = static_cast<int>(model->nbody);
    const int exposed_body_count = body_count > 0 ? body_count - 1 : 0;
    snapshot.bodies.resize(static_cast<std::size_t>(exposed_body_count));

    for (int body_id = 1; body_id < body_count; ++body_id) {
      BodyPose &pose = snapshot.bodies[static_cast<std::size_t>(body_id - 1)];
      pose.body_id = body_id;
      const char *name = mj_id2name(model.get(), mjOBJ_BODY, body_id);
      pose.name = name != nullptr ? name : "body_" + std::to_string(body_id);
    }
  }

  void refresh_snapshot() noexcept {
    snapshot.sim_time_ns = static_cast<std::uint64_t>(std::llround(data->time * 1'000'000'000.0));

    const int body_count = static_cast<int>(model->nbody);
    for (int body_id = 1; body_id < body_count; ++body_id) {
      BodyPose &pose = snapshot.bodies[static_cast<std::size_t>(body_id - 1)];
      const mjtNum *position = data->xpos + 3 * body_id;
      const mjtNum *quaternion = data->xquat + 4 * body_id;

      for (std::size_t axis = 0; axis < pose.position_m.size(); ++axis) {
        pose.position_m[axis] = position[axis];
      }
      for (std::size_t axis = 0; axis < pose.quaternion_wxyz.size(); ++axis) {
        pose.quaternion_wxyz[axis] = quaternion[axis];
      }
    }
  }

  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model{nullptr, mj_deleteModel};
  std::unique_ptr<mjData, decltype(&mj_deleteData)> data{nullptr, mj_deleteData};
  bool paused{false};
  SimulationSnapshot snapshot;
};

MujocoRuntime::MujocoRuntime(MujocoRuntimeConfig config)
    : implementation_(std::make_unique<Implementation>(config)) {}

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
    mj_step(implementation_->model.get(), implementation_->data.get());
  }

  implementation_->snapshot.physics_step += steps;
  ++implementation_->snapshot.sequence;
  implementation_->refresh_snapshot();
  return implementation_->snapshot;
}

const SimulationSnapshot &MujocoRuntime::reset() {
  mj_resetData(implementation_->model.get(), implementation_->data.get());
  mj_forward(implementation_->model.get(), implementation_->data.get());

  ++implementation_->snapshot.reset_generation;
  implementation_->snapshot.sequence = 0;
  implementation_->snapshot.physics_step = 0;
  implementation_->refresh_snapshot();
  return implementation_->snapshot;
}

void MujocoRuntime::set_paused(bool paused) noexcept {
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

}  // namespace lingtu::sim

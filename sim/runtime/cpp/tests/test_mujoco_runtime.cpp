#include <cmath>
#include <cstdlib>
#include <iostream>
#include <set>
#include <stdexcept>
#include <string>

#include "lingtu/sim/mujoco_runtime.hpp"

namespace {

void require(bool condition, const std::string &message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void test_load_and_advance() {
  lingtu::sim::MujocoRuntime runtime({LINGTU_MUJOCO_TEST_MODEL});

  const auto &initial = runtime.snapshot();
  require(initial.sim_time_ns == 0, "initial simulation time must be zero");
  require(initial.bodies.size() == 1, "world body must not be exposed");
  require(initial.bodies.front().name == "test_body", "body name must be stable");
  const double initial_z = initial.bodies.front().position_m[2];

  const auto &advanced = runtime.advance(5);
  require(advanced.sequence == 1, "one published advance must increment sequence");
  require(advanced.physics_step == 5, "physics step count must include all substeps");
  require(advanced.sim_time_ns == 10'000'000, "five 2 ms steps must reach 10 ms");
  require(advanced.bodies.front().position_m[2] < initial_z,
          "gravity must change the physical body state");
}

void test_pause_preserves_the_published_state() {
  lingtu::sim::MujocoRuntime runtime({LINGTU_MUJOCO_TEST_MODEL});
  const auto &before_pause = runtime.advance(2);
  const auto sequence = before_pause.sequence;
  const auto physics_step = before_pause.physics_step;
  const auto sim_time_ns = before_pause.sim_time_ns;
  const auto position = before_pause.bodies.front().position_m;

  runtime.set_paused(true);
  require(runtime.paused(), "runtime must report its paused state");
  const auto &while_paused = runtime.advance(20);

  require(while_paused.sequence == sequence, "paused advance must not publish a new sequence");
  require(while_paused.physics_step == physics_step,
          "paused advance must not consume physics steps");
  require(while_paused.sim_time_ns == sim_time_ns,
          "paused advance must not change simulation time");
  require(while_paused.bodies.front().position_m == position,
          "paused advance must not change body state");

  runtime.set_paused(false);
  require(!runtime.paused(), "runtime must resume when pause is cleared");
  require(runtime.advance().physics_step == physics_step + 1,
          "resumed runtime must continue from the paused state");
}

void test_reset_starts_a_new_generation() {
  lingtu::sim::MujocoRuntime runtime({LINGTU_MUJOCO_TEST_MODEL});
  const double initial_z = runtime.snapshot().bodies.front().position_m[2];
  runtime.advance(10);

  const auto &reset = runtime.reset();
  require(reset.reset_generation == 1, "reset must increment the reset generation");
  require(reset.sequence == 0, "reset must restart the publication sequence");
  require(reset.physics_step == 0, "reset must restart the physics step count");
  require(reset.sim_time_ns == 0, "reset must restore simulation time to zero");
  require(std::abs(reset.bodies.front().position_m[2] - initial_z) < 1e-12,
          "reset must restore the model reference pose");
}

void test_thunder_model_snapshot_contract() {
  lingtu::sim::MujocoRuntime runtime({LINGTU_MUJOCO_THUNDER_MODEL});

  const auto &initial = runtime.snapshot();
  require(initial.bodies.size() == 21, "ThunderV4 must expose every non-world body");

  std::set<std::string> body_names;
  for (std::size_t index = 0; index < initial.bodies.size(); ++index) {
    const auto &pose = initial.bodies[index];
    require(pose.body_id == static_cast<std::int32_t>(index + 1),
            "ThunderV4 body IDs must preserve MuJoCo ordering");
    require(!pose.name.empty(), "every ThunderV4 body must have a stable name");
    require(body_names.insert(pose.name).second, "ThunderV4 body names must be unique");

    double quaternion_norm_squared = 0.0;
    for (const double value : pose.position_m) {
      require(std::isfinite(value), "every ThunderV4 body position must be finite");
    }
    for (const double value : pose.quaternion_wxyz) {
      require(std::isfinite(value), "every ThunderV4 body quaternion must be finite");
      quaternion_norm_squared += value * value;
    }
    require(std::abs(quaternion_norm_squared - 1.0) < 1e-9,
            "every ThunderV4 body quaternion must be normalized");
  }

  const auto &advanced = runtime.advance(10);
  require(advanced.physics_step == 10, "ThunderV4 must advance with the fixed MuJoCo step");
  const auto sim_time_ns = advanced.sim_time_ns;

  runtime.set_paused(true);
  require(runtime.advance(10).sim_time_ns == sim_time_ns,
          "paused ThunderV4 must preserve simulation time");
  runtime.set_paused(false);
  require(runtime.advance().physics_step == 11, "ThunderV4 must resume after pause");

  const auto &reset = runtime.reset();
  require(reset.reset_generation == 1, "ThunderV4 reset must start a new generation");
  require(reset.physics_step == 0 && reset.sim_time_ns == 0,
          "ThunderV4 reset must restore the clock origin");
}

}  // namespace

int main() {
  try {
    test_load_and_advance();
    test_pause_preserves_the_published_state();
    test_reset_starts_a_new_generation();
    test_thunder_model_snapshot_contract();
  } catch (const std::exception &error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return EXIT_FAILURE;
  }

  std::cout << "PASS: load, advance, pause, and reset\n";
  return EXIT_SUCCESS;
}

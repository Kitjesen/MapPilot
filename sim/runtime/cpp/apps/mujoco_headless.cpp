#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <limits>
#include <string>

#include "lingtu/sim/mujoco_runtime.hpp"

namespace {

std::uint32_t parse_steps(const char *text) {
  const auto value = std::stoull(text);
  if (value == 0 || value > std::numeric_limits<std::uint32_t>::max()) {
    throw std::invalid_argument("steps must be in the range 1..4294967295");
  }
  return static_cast<std::uint32_t>(value);
}

}  // namespace

int main(int argc, char **argv) {
  if (argc < 2 || argc > 3) {
    std::cerr << "usage: lingtu_mujoco_headless MODEL.xml [STEPS]\n";
    return EXIT_FAILURE;
  }

  try {
    const std::uint32_t steps = argc == 3 ? parse_steps(argv[2]) : 1;
    lingtu::sim::MujocoRuntime runtime({argv[1]});
    const auto &snapshot = runtime.advance(steps);

    std::cout << "{\n"
              << "  \"physics_step\": " << snapshot.physics_step << ",\n"
              << "  \"sequence\": " << snapshot.sequence << ",\n"
              << "  \"sim_time_ns\": " << snapshot.sim_time_ns << ",\n"
              << "  \"reset_generation\": " << snapshot.reset_generation << ",\n"
              << "  \"timestep_seconds\": " << runtime.timestep_seconds() << ",\n"
              << "  \"body_count\": " << snapshot.bodies.size() << "\n"
              << "}\n";
  } catch (const std::exception &error) {
    std::cerr << "lingtu_mujoco_headless: " << error.what() << '\n';
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

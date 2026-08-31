#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>

#include "lingtu/sim/mujoco_runtime.hpp"

namespace {

std::string json_string(const std::string &value) {
  std::string escaped;
  escaped.reserve(value.size() + 2);
  escaped.push_back('"');
  for (const unsigned char character : value) {
    switch (character) {
      case '"':
        escaped += "\\\"";
        break;
      case '\\':
        escaped += "\\\\";
        break;
      case '\b':
        escaped += "\\b";
        break;
      case '\f':
        escaped += "\\f";
        break;
      case '\n':
        escaped += "\\n";
        break;
      case '\r':
        escaped += "\\r";
        break;
      case '\t':
        escaped += "\\t";
        break;
      default:
        if (character < 0x20) {
          constexpr char hex[] = "0123456789abcdef";
          escaped += "\\u00";
          escaped.push_back(hex[(character >> 4) & 0x0f]);
          escaped.push_back(hex[character & 0x0f]);
        } else {
          escaped.push_back(static_cast<char>(character));
        }
    }
  }
  escaped.push_back('"');
  return escaped;
}

template <std::size_t Size>
void write_array(const std::array<double, Size> &values) {
  std::cout << '[';
  for (std::size_t index = 0; index < Size; ++index) {
    if (index != 0) {
      std::cout << ',';
    }
    std::cout << values[index];
  }
  std::cout << ']';
}

}  // namespace

int main(int argc, char **argv) {
  if (argc < 2 || argc > 3) {
    std::cerr << "usage: lingtu_mujoco_snapshot MODEL.xml [KEYFRAME]\n";
    return EXIT_FAILURE;
  }

  try {
    const std::string keyframe = argc == 3 ? argv[2] : "";
    lingtu::sim::MujocoRuntime runtime({argv[1], keyframe});
    const auto &snapshot = runtime.snapshot();

    std::cout << std::setprecision(17)
              << "{\n"
              << "  \"schema\": \"lingtu.sim.mujoco-snapshot.v1\",\n"
              << "  \"model_generation\": " << snapshot.model_generation << ",\n"
              << "  \"reset_generation\": " << snapshot.reset_generation << ",\n"
              << "  \"sequence\": " << snapshot.sequence << ",\n"
              << "  \"sim_time_ns\": " << snapshot.sim_time_ns << ",\n"
              << "  \"bodies\": [\n";

    for (std::size_t index = 0; index < snapshot.bodies.size(); ++index) {
      const auto &body = snapshot.bodies[index];
      std::cout << "    {\"body_id\": " << body.body_id
                << ", \"name\": " << json_string(body.name)
                << ", \"position_m\": ";
      write_array(body.position_m);
      std::cout << ", \"quaternion_wxyz\": ";
      write_array(body.quaternion_wxyz);
      std::cout << ", \"linear_velocity_mps\": ";
      write_array(body.linear_velocity_mps);
      std::cout << ", \"angular_velocity_rps\": ";
      write_array(body.angular_velocity_rps);
      std::cout << '}';
      if (index + 1 != snapshot.bodies.size()) {
        std::cout << ',';
      }
      std::cout << '\n';
    }

    std::cout << "  ]\n}\n";
  } catch (const std::exception &error) {
    std::cerr << "lingtu_mujoco_snapshot: " << error.what() << '\n';
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

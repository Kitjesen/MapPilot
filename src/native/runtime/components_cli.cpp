#include "components.hpp"

#include <cstdio>

int main() {
  std::size_t count = 0;
  const auto* components = lingtu::native_runtime::components(count);

  std::printf("name|state|lane|target|how_to_use|ros2\n");
  for (std::size_t i = 0; i < count; ++i) {
    const auto& component = components[i];
    std::printf(
        "%s|%s|%s|%s|%s|%s\n",
        component.name,
        lingtu::native_runtime::to_string(component.state),
        lingtu::native_runtime::to_string(component.lane),
        component.target,
        component.use,
        component.requires_ros2 ? "yes" : "no");
  }

  return 0;
}

#pragma once

#include <cstddef>
#include <string_view>

namespace lingtu::native_runtime {

enum class RuntimeLane {
  SensorInput,
  Localization,
  Relocalization,
  MapOptimization,
  Navigation,
};

enum class State {
  Ready,
  Wip,
  Legacy,
  Exp,
};

struct Component {
  const char* name;
  RuntimeLane lane;
  State state;
  const char* source_dir;
  const char* target;
  const char* use;
  const char* role;
  bool product_default;
  bool requires_ros2;
};

const Component* components(std::size_t& count) noexcept;
const Component* find(std::string_view name) noexcept;
bool ready(std::string_view name) noexcept;
const char* to_string(RuntimeLane lane) noexcept;
const char* to_string(State state) noexcept;

}  // namespace lingtu::native_runtime

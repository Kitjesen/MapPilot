#pragma once

#include <mujoco/mujoco.h>
#include <mujoco/mjxmacro.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace lingtu::sim::mujoco_compat {

inline std::vector<std::string> authored_option_fields(
    const mjSpec *spec) {
  static constexpr const char *field_names[] = {
#define X(type, name, count) #name,
#define XVEC(type, name, count) #name,
      MJOPTION_FIELDS
#undef XVEC
#undef X
  };

  std::vector<std::string> fields;
  if (spec == nullptr) {
    return fields;
  }
  for (std::size_t index = 0;
       index < sizeof(field_names) / sizeof(field_names[0]); ++index) {
    if ((spec->authored.option & (std::uint64_t{1} << index)) != 0) {
      fields.emplace_back(field_names[index]);
    }
  }
  return fields;
}

}  // namespace lingtu::sim::mujoco_compat

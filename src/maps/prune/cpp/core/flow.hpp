#pragma once

#include <string>
#include <vector>

namespace lingtu::map_cleaning {

enum class StageState {
  Ready,
  Partial,
  Planned,
};

struct StageSpec {
  const char *id;
  const char *state;
  const char *input;
  const char *output;
  const char *owner;
  const char *note;
};

std::vector<StageSpec> productFlow();

std::string flowJson();

}  // namespace lingtu::map_cleaning

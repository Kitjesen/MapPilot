#pragma once

#include <ostream>
#include <stdexcept>
#include <string_view>

namespace lingtu::nav::endpoint {

// The status mirror and ExplorationRunEvent stream must expose the same
// endpoint incarnation so durable consumers can detect a restart boundary.
inline void writeExploreStatusIdentity(std::ostream &output, std::string_view boot_id) {
  if (boot_id.empty()) {
    throw std::invalid_argument("explore endpoint boot_id is required");
  }
  output << "  \"schema_version\": \"lingtu.explore.status.v2\",\n"
         << "  \"endpoint\": \"lingtu_explore_dds\",\n"
         << "  \"boot_id\": \"" << boot_id << "\",\n";
}

}  // namespace lingtu::nav::endpoint

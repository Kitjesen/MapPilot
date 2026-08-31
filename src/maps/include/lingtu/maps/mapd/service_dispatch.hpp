#pragma once

#include <string>

#include "lingtu/maps/service.hpp"

namespace lingtu::maps::mapd {
class SaveCoordinator;
}

namespace lingtu::maps::mapd::query {

struct DispatchResult {
  bool ok{false};
  std::string json;
};

DispatchResult DispatchServiceJson(MapsServiceCore &service,
                                   mapd::SaveCoordinator *save_coordinator,
                                   const std::string &request_json);

inline DispatchResult DispatchServiceJson(MapsServiceCore &service,
                                          const std::string &request_json) {
  return DispatchServiceJson(service, nullptr, request_json);
}

}  // namespace lingtu::maps::mapd::query

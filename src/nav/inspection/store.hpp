#pragma once

#include "inspection.hpp"

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace lingtu::nav::inspection {

struct StoreResult {
  bool ok{false};
  std::string reason;
};

struct RouteSummary {
  std::string id;
  std::string name;
  std::string map_id;
  std::int64_t map_version{0};
  std::uint64_t revision{0};
  std::size_t point_count{0};
};

class Store {
 public:
  explicit Store(std::filesystem::path map_root);

  StoreResult Put(const Route& route);
  std::optional<Route> Get(const std::string& map_id, const std::string& route_id) const;
  StoreResult Delete(const std::string& map_id, const std::string& route_id);
  std::vector<RouteSummary> List(const std::string& map_id) const;
  StoreResult PutStatus(const RunStatus& status);
  std::string StatusJson() const;

  std::filesystem::path RouteDirectory(const std::string& map_id) const;
  std::filesystem::path RoutePath(
      const std::string& map_id,
      const std::string& route_id) const;

 private:
  std::filesystem::path map_root_;
};

std::string RouteToJson(const Route& route);
std::string RouteListToJson(const std::vector<RouteSummary>& routes);
std::string RunStatusToJson(const RunStatus& status);

}  // namespace lingtu::nav::inspection

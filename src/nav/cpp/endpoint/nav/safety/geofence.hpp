#pragma once

#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "message/cpp/geofence.hpp"

namespace lingtu::nav::endpoint {

struct GeofencePoint {
  double x{0.0};
  double y{0.0};
};

struct GeofenceZone {
  std::string name;
  bool enabled{true};
  std::vector<GeofencePoint> polygon;
};

struct GeofenceSummary {
  std::string name;
  bool enabled{true};
  std::uint32_t vertex_count{0U};
};

struct GeofenceCommand {
  lingtu::message::GeofenceAction action{lingtu::message::GeofenceAction::kList};
  std::string name;
  std::vector<GeofencePoint> polygon;
};

struct GeofenceCommandResult {
  bool accepted{false};
  std::string reason;
  std::uint64_t revision{0U};
  std::vector<GeofenceSummary> zones;
};

// Owns restricted-zone geometry used by navd's final motion gate. All methods
// are called from the endpoint thread; transport and API concerns stay outside.
class GeofenceManager {
 public:
  explicit GeofenceManager(std::filesystem::path persistence_file = {});

  [[nodiscard]] GeofenceCommandResult apply(const GeofenceCommand &command);
  [[nodiscard]] std::optional<std::string> intrusion(double x, double y) const;
  [[nodiscard]] std::vector<GeofenceSummary> summaries() const;
  [[nodiscard]] std::uint64_t revision() const noexcept { return revision_; }
  [[nodiscard]] std::size_t size() const noexcept { return zones_.size(); }

  [[nodiscard]] static bool contains(const std::vector<GeofencePoint> &polygon,
                                     double x, double y) noexcept;

 private:
  using Zones = std::map<std::string, GeofenceZone>;

  [[nodiscard]] static std::optional<std::string> validate(const GeofenceZone &zone);
  [[nodiscard]] bool persist(const Zones &zones, std::uint64_t revision,
                             std::string *error) const;
  void load();

  std::filesystem::path persistence_file_;
  Zones zones_;
  std::uint64_t revision_{0U};
};

}  // namespace lingtu::nav::endpoint

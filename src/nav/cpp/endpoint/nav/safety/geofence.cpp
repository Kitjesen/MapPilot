#include "safety/geofence.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace lingtu::nav::endpoint {
namespace {

constexpr const char *kFormat = "LINGTU_GEOFENCE_V1";
constexpr std::size_t kMaximumZones = 256U;
constexpr std::size_t kMaximumVertices = 4096U;
constexpr std::size_t kMaximumNameBytes = 128U;
constexpr double kBoundaryTolerance = 1e-9;

bool pointOnSegment(const GeofencePoint &a, const GeofencePoint &b, double x,
                    double y) noexcept {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double cross = (x - a.x) * dy - (y - a.y) * dx;
  const double scale = std::max({1.0, std::abs(dx), std::abs(dy)});
  if (std::abs(cross) > kBoundaryTolerance * scale) {
    return false;
  }
  return x >= std::min(a.x, b.x) - kBoundaryTolerance &&
         x <= std::max(a.x, b.x) + kBoundaryTolerance &&
         y >= std::min(a.y, b.y) - kBoundaryTolerance &&
         y <= std::max(a.y, b.y) + kBoundaryTolerance;
}

}  // namespace

GeofenceManager::GeofenceManager(std::filesystem::path persistence_file)
    : persistence_file_(std::move(persistence_file)) {
  load();
}

std::optional<std::string> GeofenceManager::validate(const GeofenceZone &zone) {
  if (zone.name.empty()) {
    return "missing_zone_name";
  }
  if (zone.name.size() > kMaximumNameBytes ||
      std::any_of(zone.name.begin(), zone.name.end(), [](unsigned char value) {
        return value < 0x20U || value == 0x7fU;
      })) {
    return "invalid_zone_name";
  }
  if (zone.polygon.size() < 3U) {
    return "polygon_needs_at_least_three_vertices";
  }
  if (zone.polygon.size() > kMaximumVertices) {
    return "polygon_vertex_limit_exceeded";
  }
  for (const auto &point : zone.polygon) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      return "polygon_contains_nonfinite_vertex";
    }
  }
  return std::nullopt;
}

GeofenceCommandResult GeofenceManager::apply(const GeofenceCommand &command) {
  GeofenceCommandResult result;
  result.revision = revision_;

  if (command.action == lingtu::message::GeofenceAction::kList) {
    result.accepted = true;
    result.reason = "listed";
    result.zones = summaries();
    return result;
  }

  Zones candidate = zones_;
  bool changed = false;
  switch (command.action) {
    case lingtu::message::GeofenceAction::kAdd: {
      GeofenceZone zone{command.name, true, command.polygon};
      if (const auto error = validate(zone)) {
        result.reason = *error;
        result.zones = summaries();
        return result;
      }
      if (candidate.size() >= kMaximumZones && candidate.find(zone.name) == candidate.end()) {
        result.reason = "zone_limit_exceeded";
        result.zones = summaries();
        return result;
      }
      candidate[zone.name] = std::move(zone);
      changed = true;
      result.reason = "zone_added";
      break;
    }
    case lingtu::message::GeofenceAction::kRemove: {
      if (command.name.empty()) {
        result.reason = "missing_zone_name";
        result.zones = summaries();
        return result;
      }
      changed = candidate.erase(command.name) != 0U;
      if (!changed) {
        result.reason = "zone_not_found";
        result.zones = summaries();
        return result;
      }
      result.reason = "zone_removed";
      break;
    }
    case lingtu::message::GeofenceAction::kClear:
      changed = !candidate.empty();
      candidate.clear();
      result.reason = "zones_cleared";
      break;
    case lingtu::message::GeofenceAction::kEnable:
    case lingtu::message::GeofenceAction::kDisable: {
      const auto found = candidate.find(command.name);
      if (found == candidate.end()) {
        result.reason = command.name.empty() ? "missing_zone_name" : "zone_not_found";
        result.zones = summaries();
        return result;
      }
      const bool enabled = command.action == lingtu::message::GeofenceAction::kEnable;
      changed = found->second.enabled != enabled;
      found->second.enabled = enabled;
      result.reason = enabled ? "zone_enabled" : "zone_disabled";
      break;
    }
    case lingtu::message::GeofenceAction::kList:
      break;
  }

  const std::uint64_t next_revision =
      changed && revision_ != std::numeric_limits<std::uint64_t>::max() ? revision_ + 1U
                                                                       : revision_;
  std::string persistence_error;
  if (changed && !persist(candidate, next_revision, &persistence_error)) {
    result.reason = "persistence_failed:" + persistence_error;
    result.zones = summaries();
    return result;
  }
  zones_ = std::move(candidate);
  revision_ = next_revision;
  result.accepted = true;
  result.revision = revision_;
  result.zones = summaries();
  return result;
}

std::optional<std::string> GeofenceManager::intrusion(double x, double y) const {
  if (!std::isfinite(x) || !std::isfinite(y)) {
    return std::nullopt;
  }
  for (const auto &[name, zone] : zones_) {
    if (zone.enabled && contains(zone.polygon, x, y)) {
      return name;
    }
  }
  return std::nullopt;
}

std::vector<GeofenceSummary> GeofenceManager::summaries() const {
  std::vector<GeofenceSummary> result;
  result.reserve(zones_.size());
  for (const auto &[name, zone] : zones_) {
    result.push_back(
        {name, zone.enabled, static_cast<std::uint32_t>(zone.polygon.size())});
  }
  return result;
}

bool GeofenceManager::contains(const std::vector<GeofencePoint> &polygon, double x,
                               double y) noexcept {
  if (polygon.size() < 3U || !std::isfinite(x) || !std::isfinite(y)) {
    return false;
  }
  bool inside = false;
  std::size_t previous = polygon.size() - 1U;
  for (std::size_t current = 0U; current < polygon.size(); ++current) {
    const auto &a = polygon[previous];
    const auto &b = polygon[current];
    if (!std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(b.x) ||
        !std::isfinite(b.y)) {
      return false;
    }
    if (pointOnSegment(a, b, x, y)) {
      return true;
    }
    const bool crosses = (a.y > y) != (b.y > y);
    if (crosses) {
      const double intersection = (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x;
      if (x < intersection) {
        inside = !inside;
      }
    }
    previous = current;
  }
  return inside;
}

bool GeofenceManager::persist(const Zones &zones, std::uint64_t revision,
                              std::string *error) const {
  if (persistence_file_.empty()) {
    return true;
  }
  std::error_code ec;
  if (!persistence_file_.parent_path().empty()) {
    std::filesystem::create_directories(persistence_file_.parent_path(), ec);
    if (ec) {
      *error = "create_parent:" + ec.message();
      return false;
    }
  }
  const auto temporary = persistence_file_.string() + ".tmp";
  {
    std::ofstream output(temporary, std::ios::trunc);
    if (!output) {
      *error = "open_temporary";
      return false;
    }
    output << kFormat << ' ' << revision << ' ' << zones.size() << '\n';
    output << std::setprecision(17);
    for (const auto &[name, zone] : zones) {
      output << std::quoted(name) << ' ' << (zone.enabled ? 1 : 0) << ' '
             << zone.polygon.size();
      for (const auto &point : zone.polygon) {
        output << ' ' << point.x << ' ' << point.y;
      }
      output << '\n';
    }
    output.flush();
    if (!output) {
      *error = "write_temporary";
      return false;
    }
  }
  std::filesystem::rename(temporary, persistence_file_, ec);
  if (ec) {
    ec.clear();
    std::filesystem::copy_file(temporary, persistence_file_,
                               std::filesystem::copy_options::overwrite_existing, ec);
    std::error_code remove_error;
    std::filesystem::remove(temporary, remove_error);
  }
  if (ec) {
    *error = "replace_file:" + ec.message();
    return false;
  }
  return true;
}

void GeofenceManager::load() {
  if (persistence_file_.empty() || !std::filesystem::exists(persistence_file_)) {
    return;
  }
  std::ifstream input(persistence_file_);
  std::string format;
  std::uint64_t revision = 0U;
  std::size_t zone_count = 0U;
  if (!(input >> format >> revision >> zone_count) || format != kFormat ||
      zone_count > kMaximumZones) {
    throw std::runtime_error("invalid geofence persistence header: " +
                             persistence_file_.string());
  }
  Zones loaded;
  for (std::size_t index = 0U; index < zone_count; ++index) {
    GeofenceZone zone;
    int enabled = 0;
    std::size_t vertex_count = 0U;
    if (!(input >> std::quoted(zone.name) >> enabled >> vertex_count) ||
        vertex_count > kMaximumVertices) {
      throw std::runtime_error("invalid geofence persistence entry: " +
                               persistence_file_.string());
    }
    zone.enabled = enabled != 0;
    zone.polygon.resize(vertex_count);
    for (auto &point : zone.polygon) {
      if (!(input >> point.x >> point.y)) {
        throw std::runtime_error("truncated geofence persistence entry: " +
                                 persistence_file_.string());
      }
    }
    if (const auto error = validate(zone)) {
      throw std::runtime_error("invalid geofence persistence zone " + zone.name + ": " +
                               *error);
    }
    loaded[zone.name] = std::move(zone);
  }
  zones_ = std::move(loaded);
  revision_ = revision;
}

}  // namespace lingtu::nav::endpoint

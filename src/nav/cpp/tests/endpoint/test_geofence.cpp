#include <filesystem>
#include <limits>
#include <stdexcept>
#include <string>

#include "safety/geofence.hpp"

namespace {

using lingtu::message::GeofenceAction;
using lingtu::nav::endpoint::GeofenceCommand;
using lingtu::nav::endpoint::GeofenceManager;
using lingtu::nav::endpoint::GeofencePoint;

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

std::vector<GeofencePoint> square() {
  return {{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}};
}

void testCrudAndIntrusion() {
  GeofenceManager manager;
  auto result = manager.apply({GeofenceAction::kAdd, "loading-bay", square()});
  require(result.accepted && result.revision == 1U, "add must advance revision");
  require(manager.intrusion(1.0, 1.0) == std::optional<std::string>{"loading-bay"},
          "inside point must block motion");
  require(manager.intrusion(3.0, 1.0) == std::nullopt,
          "outside point must remain clear");
  require(manager.intrusion(0.0, 1.0) == std::optional<std::string>{"loading-bay"},
          "polygon boundary must be treated as restricted");

  result = manager.apply({GeofenceAction::kDisable, "loading-bay", {}});
  require(result.accepted && manager.intrusion(1.0, 1.0) == std::nullopt,
          "disabled zone must not block motion");
  result = manager.apply({GeofenceAction::kEnable, "loading-bay", {}});
  require(result.accepted && manager.intrusion(1.0, 1.0).has_value(),
          "enabled zone must block again");
  result = manager.apply({GeofenceAction::kRemove, "loading-bay", {}});
  require(result.accepted && manager.size() == 0U, "remove must delete the zone");
}

void testRejectsInvalidGeometry() {
  GeofenceManager manager;
  auto result = manager.apply(
      {GeofenceAction::kAdd, "short", {{0.0, 0.0}, {1.0, 1.0}}});
  require(!result.accepted, "polygon with fewer than three points must fail");
  result = manager.apply({GeofenceAction::kAdd,
                          "nan",
                          {{0.0, 0.0},
                           {1.0, 0.0},
                           {std::numeric_limits<double>::quiet_NaN(), 1.0}}});
  require(!result.accepted, "non-finite polygon point must fail");
}

void testPersistenceRoundTrip() {
  const auto path = std::filesystem::temp_directory_path() /
                    "lingtu_geofence_native_test.dat";
  std::error_code ec;
  std::filesystem::remove(path, ec);
  {
    GeofenceManager manager(path);
    const auto added = manager.apply({GeofenceAction::kAdd, "yard", square()});
    require(added.accepted, "persistent add must succeed");
    const auto disabled = manager.apply({GeofenceAction::kDisable, "yard", {}});
    require(disabled.accepted, "persistent disable must succeed");
  }
  {
    GeofenceManager manager(path);
    require(manager.size() == 1U && manager.revision() == 2U,
            "native store must restore zones and revision");
    require(manager.intrusion(1.0, 1.0) == std::nullopt,
            "native store must restore enabled state");
  }
  std::filesystem::remove(path, ec);
}

}  // namespace

int main() {
  testCrudAndIntrusion();
  testRejectsInvalidGeometry();
  testPersistenceRoundTrip();
  return 0;
}

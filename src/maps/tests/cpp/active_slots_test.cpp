#include "lingtu/maps/active_slots.hpp"

#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <unordered_set>

using lingtu::maps::ActiveSlots;

namespace {

std::filesystem::path TempRoot() {
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  return std::filesystem::temp_directory_path() /
      ("lingtu_active_slots_test_" + std::to_string(stamp));
}

std::string ReadText(const std::filesystem::path& path) {
  std::ifstream file(path, std::ios::binary);
  return std::string(
      std::istreambuf_iterator<char>(file),
      std::istreambuf_iterator<char>());
}

}  // namespace

int main() {
  const auto root = TempRoot();
  std::filesystem::create_directories(root);
  const auto path = root / "active_slots.lts";
  const std::unordered_set<std::string> valid_maps{"field_a", "field_b"};
  ActiveSlots slots(path, [&](const std::string& map_id) {
    return valid_maps.count(map_id) != 0U;
  });

  assert(ActiveSlots::IsValidSlotName("navigation"));
  assert(!ActiveSlots::IsValidSlotName("../bad"));
  assert(slots.List().size() == 3U);
  assert(slots.Get("navigation").empty());

  auto set_navigation = slots.Set("navigation", "field_a");
  assert(set_navigation.ok);
  assert(set_navigation.entry.has_value());
  assert(set_navigation.entry->slot == "navigation");
  assert(set_navigation.entry->map_id == "field_a");
  assert(slots.Get("navigation") == "field_a");

  auto set_reference = slots.Set("reference", "field_b");
  assert(set_reference.ok);
  ActiveSlots loaded(path, [&](const std::string& map_id) {
    return valid_maps.count(map_id) != 0U;
  });
  assert(loaded.Get("navigation") == "field_a");
  assert(loaded.Get("reference") == "field_b");

  auto replaced = slots.ReplaceMapId("field_a", "field_b");
  assert(replaced.ok);
  assert(slots.Get("navigation") == "field_b");

  auto cleared_map = slots.ClearMapId("field_b");
  assert(cleared_map.ok);
  assert(slots.Get("navigation").empty());
  assert(slots.Get("reference").empty());

  assert(slots.Set("reference", "field_b").ok);

  auto rejected = slots.Set("mapping", "missing_map");
  assert(!rejected.ok);
  assert(slots.Get("mapping").empty());

  auto cleared = slots.Clear("navigation");
  assert(cleared.ok);
  assert(slots.Get("navigation").empty());

  const auto text = ReadText(path);
  assert(text.find("LINGTU_ACTIVE_SLOTS 1\n") == 0U);
  assert(text.find("navigation=\n") != std::string::npos);
  assert(text.find("reference=field_b\n") != std::string::npos);
  assert(!std::filesystem::exists(path.string() + ".tmp"));

  std::filesystem::remove_all(root);
  return 0;
}

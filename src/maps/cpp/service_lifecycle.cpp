#include "lingtu/maps/service.hpp"
#include "lingtu/maps/json.hpp"

#include "lingtu/maps/lock.hpp"
#include "lingtu/maps/map_graph.hpp"

#include <algorithm>
#include <sstream>
#include <string>

namespace lingtu::maps {
namespace {

}  // namespace

std::string MapsServiceCore::CreateMapJson(const std::string& map_id) {
  try {
    auto result = store_.CreateMap(map_id);
    if (!result.ok) {
      const std::string reason = result.message.find("write in progress") != std::string::npos
          ? "map_write_in_progress"
          : "map_exists";
      return FailureJson("create", result.message, reason);
    }
    return "{"
        "\"action\":\"create\","
        "\"success\":true,"
        "\"map_id\":" + JsonString(MapStore::NormalizeMapId(map_id)) + ","
        "\"record\":" + RecordJson(*result.record) +
        "}";
  } catch (const std::exception& exc) {
    return FailureJson("create", exc.what(), "invalid_map_name");
  }
}

std::string MapsServiceCore::DeleteMapJson(const std::string& map_id) {
  try {
    const std::string id = MapStore::NormalizeMapId(map_id);
    const auto allowed = store_.CheckMapMutationAllowed(id);
    if (!allowed.ok) {
      return FailureJson(
          "delete",
          allowed.message,
          allowed.message.find("active map conflict") != std::string::npos
              ? "active_map_conflict"
              : "active_map_state_invalid");
    }
    auto graph_lock = MapLock::TryAcquire(store_.RootDir(), "__map_graph__", "delete-map");
    if (!graph_lock.has_value()) {
      return FailureJson("delete", "map graph write in progress", "map_graph_busy");
    }
    MapGraph graph([](const std::string& value) { return MapStore::IsValidMapId(value); });
    if (std::filesystem::is_regular_file(GraphPath())) {
      const auto loaded = graph.Load(GraphPath());
      if (!loaded.ok) {
        return FailureJson(
            "delete",
            loaded.message,
            loaded.corrupt ? "graph_corrupt" : "graph_store_read_failed");
      }
      if (graph.ReferencesMap(id)) {
        return FailureJson(
            "delete",
            "map is still referenced by map graph: " + id,
            "map_graph_reference_conflict");
      }
    }
    auto result = store_.DeleteMap(id);
    if (!result.ok) {
      const std::string reason =
          result.message.find("write in progress") != std::string::npos
          ? "map_write_in_progress"
          : result.message.find("active map conflict") != std::string::npos
          ? "active_map_conflict"
          : result.message.find("active map state") != std::string::npos
          ? "active_map_state_invalid"
          : "map_not_found";
      return FailureJson("delete", result.message, reason);
    }
    return "{"
        "\"action\":\"delete\","
        "\"success\":true,"
        "\"map_id\":" + JsonString(id) + ","
        "\"message\":" + JsonString("deleted: " + id) +
        "}";
  } catch (const std::exception& exc) {
    return FailureJson("delete", exc.what(), "invalid_map_name");
  }
}

std::string MapsServiceCore::RenameMapJson(const std::string& map_id, const std::string& new_map_id) {
  try {
    const std::string old_id = MapStore::NormalizeMapId(map_id);
    const std::string new_id = MapStore::NormalizeMapId(new_map_id);
    const auto allowed = store_.CheckMapMutationAllowed(old_id);
    if (!allowed.ok) {
      return FailureJson(
          "rename",
          allowed.message,
          allowed.message.find("active map conflict") != std::string::npos
              ? "active_map_conflict"
              : "active_map_state_invalid");
    }
    auto graph_lock = MapLock::TryAcquire(store_.RootDir(), "__map_graph__", "rename-map");
    if (!graph_lock.has_value()) {
      return FailureJson("rename", "map graph write in progress", "map_graph_busy");
    }
    MapGraph graph([](const std::string& value) { return MapStore::IsValidMapId(value); });
    if (std::filesystem::is_regular_file(GraphPath())) {
      const auto loaded = graph.Load(GraphPath());
      if (!loaded.ok) {
        return FailureJson(
            "rename",
            loaded.message,
            loaded.corrupt ? "graph_corrupt" : "graph_store_read_failed");
      }
      if (graph.ReferencesMap(old_id)) {
        return FailureJson(
            "rename",
            "map is still referenced by map graph: " + old_id,
            "map_graph_reference_conflict");
      }
    }
    auto result = store_.RenameMap(old_id, new_id);
    if (!result.ok) {
      const std::string reason = result.message.find("write in progress") != std::string::npos
          ? "map_write_in_progress"
          : result.message.find("active map conflict") != std::string::npos
          ? "active_map_conflict"
          : result.message.find("active map state") != std::string::npos
          ? "active_map_state_invalid"
          : (result.message.find("not found") != std::string::npos
                 ? "map_not_found"
                 : "target_exists");
      return FailureJson("rename", result.message, reason);
    }
    return "{"
        "\"action\":\"rename\","
        "\"success\":true,"
        "\"map_id\":" + JsonString(new_id) + ","
        "\"old_map_id\":" + JsonString(old_id) + ","
        "\"message\":" + JsonString(old_id + " -> " + new_id) + ","
        "\"record\":" + RecordJson(*result.record) +
        "}";
  } catch (const std::exception& exc) {
    return FailureJson("rename", exc.what(), "invalid_map_name");
  }
}

std::string MapsServiceCore::RetireMapJson(const std::string& map_id) {
  try {
    const std::string id = MapStore::NormalizeMapId(map_id);
    const auto allowed = store_.CheckMapMutationAllowed(id);
    if (!allowed.ok) {
      return FailureJson(
          "retire",
          allowed.message,
          allowed.message.find("active map conflict") != std::string::npos
              ? "active_map_conflict"
              : "active_map_state_invalid");
    }
    auto graph_lock = MapLock::TryAcquire(store_.RootDir(), "__map_graph__", "retire-map");
    if (!graph_lock.has_value()) {
      return FailureJson("retire", "map graph write in progress", "map_graph_busy");
    }
    MapGraph graph([](const std::string& value) { return MapStore::IsValidMapId(value); });
    if (std::filesystem::is_regular_file(GraphPath())) {
      const auto loaded = graph.Load(GraphPath());
      if (!loaded.ok) {
        return FailureJson(
            "retire",
            loaded.message,
            loaded.corrupt ? "graph_corrupt" : "graph_store_read_failed");
      }
      if (graph.ReferencesMap(id)) {
        return FailureJson(
            "retire",
            "map is still referenced by map graph: " + id,
            "map_graph_reference_conflict");
      }
    }
    auto result = store_.RetireMap(id);
    if (!result.ok) {
      const std::string reason =
          result.message.find("write in progress") != std::string::npos
          ? "map_write_in_progress"
          : result.message.find("active map conflict") != std::string::npos
          ? "active_map_conflict"
          : result.message.find("active map state") != std::string::npos
          ? "active_map_state_invalid"
          : "map_not_found";
      return FailureJson("retire", result.message, reason);
    }
    return "{"
        "\"action\":\"retire\","
        "\"success\":true,"
        "\"map_id\":" + JsonString(id) + ","
        "\"message\":" + JsonString("retired: " + id) + ","
        "\"record\":" + RecordJson(*result.record) +
        "}";
  } catch (const std::exception& exc) {
    return FailureJson("retire", exc.what(), "invalid_map_name");
  }
}

}  // namespace lingtu::maps

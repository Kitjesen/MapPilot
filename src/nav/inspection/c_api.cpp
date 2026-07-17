#include "c_api.h"

#include "store.hpp"

#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <memory>
#include <string>

namespace {

constexpr std::uint32_t kInspectionStoreAbiVersion = 1U;

struct Handle {
  explicit Handle(std::filesystem::path root) : store(std::move(root)) {}
  lingtu::nav::inspection::Store store;
  std::string error;
};

std::string Text(const char* value) {
  return value == nullptr ? std::string{} : std::string(value);
}

char* CopyString(const std::string& value) {
  auto* copy = static_cast<char*>(std::malloc(value.size() + 1U));
  if (copy == nullptr) return nullptr;
  std::memcpy(copy, value.c_str(), value.size() + 1U);
  return copy;
}

lingtu::nav::inspection::Route ToRoute(const lingtu_inspection_route& input) {
  using lingtu::nav::inspection::FailurePolicy;
  lingtu::nav::inspection::Route route;
  route.id = Text(input.id);
  route.name = Text(input.name);
  route.map_id = Text(input.map_id);
  route.map_version = input.map_version;
  route.revision = input.revision;
  route.loop_count = input.loop_count;
  route.failure_policy = input.failure_policy == 1
      ? FailurePolicy::kRetry
      : (input.failure_policy == 2 ? FailurePolicy::kSkip : FailurePolicy::kStop);
  route.max_retries = input.max_retries;
  route.points.reserve(static_cast<std::size_t>(input.point_count));
  for (std::uint64_t i = 0U; i < input.point_count; ++i) {
    const auto& source = input.points[i];
    lingtu::nav::inspection::Point point;
    point.id = Text(source.id);
    point.x_m = source.x_m;
    point.y_m = source.y_m;
    point.z_m = source.z_m;
    point.yaw_rad = source.yaw_rad;
    point.has_yaw = source.has_yaw != 0;
    point.position_tolerance_m = source.position_tolerance_m;
    point.yaw_tolerance_rad = source.yaw_tolerance_rad;
    point.dwell_s = source.dwell_s;
    point.action = Text(source.action);
    point.enabled = source.enabled != 0;
    route.points.push_back(std::move(point));
  }
  return route;
}

}  // namespace

extern "C" {

uint32_t lingtu_inspection_store_abi_version() {
  return kInspectionStoreAbiVersion;
}

lingtu_inspection_store_handle lingtu_inspection_store_create(const char* map_root) {
  if (map_root == nullptr || map_root[0] == '\0') return nullptr;
  try {
    return new Handle(map_root);
  } catch (...) {
    return nullptr;
  }
}

void lingtu_inspection_store_destroy(lingtu_inspection_store_handle handle) {
  delete static_cast<Handle*>(handle);
}

int32_t lingtu_inspection_store_put(
    lingtu_inspection_store_handle handle,
    const lingtu_inspection_route* route) {
  if (handle == nullptr || route == nullptr ||
      (route->point_count > 0U && route->points == nullptr)) {
    return -1;
  }
  auto* state = static_cast<Handle*>(handle);
  try {
    const auto result = state->store.Put(ToRoute(*route));
    state->error = result.ok ? std::string{} : result.reason;
    return result.ok ? 0 : -1;
  } catch (const std::exception& exc) {
    state->error = exc.what();
    return -1;
  }
}

int32_t lingtu_inspection_store_delete(
    lingtu_inspection_store_handle handle,
    const char* map_id,
    const char* route_id) {
  if (handle == nullptr) return -1;
  auto* state = static_cast<Handle*>(handle);
  const auto result = state->store.Delete(Text(map_id), Text(route_id));
  state->error = result.ok ? std::string{} : result.reason;
  return result.ok ? 0 : -1;
}

char* lingtu_inspection_store_get_json(
    lingtu_inspection_store_handle handle,
    const char* map_id,
    const char* route_id) {
  if (handle == nullptr) return nullptr;
  auto* state = static_cast<Handle*>(handle);
  const auto route = state->store.Get(Text(map_id), Text(route_id));
  if (!route) {
    state->error = "route_not_found";
    return nullptr;
  }
  state->error.clear();
  return CopyString(lingtu::nav::inspection::RouteToJson(*route));
}

char* lingtu_inspection_store_list_json(
    lingtu_inspection_store_handle handle,
    const char* map_id) {
  if (handle == nullptr) return nullptr;
  auto* state = static_cast<Handle*>(handle);
  state->error.clear();
  return CopyString(
      lingtu::nav::inspection::RouteListToJson(state->store.List(Text(map_id))));
}

char* lingtu_inspection_store_status_json(
    lingtu_inspection_store_handle handle) {
  if (handle == nullptr) return nullptr;
  auto* state = static_cast<Handle*>(handle);
  state->error.clear();
  return CopyString(state->store.StatusJson());
}

const char* lingtu_inspection_store_last_error(lingtu_inspection_store_handle handle) {
  if (handle == nullptr) return "invalid_inspection_store_handle";
  return static_cast<Handle*>(handle)->error.c_str();
}

void lingtu_inspection_string_free(char* value) {
  std::free(value);
}

}  // extern "C"

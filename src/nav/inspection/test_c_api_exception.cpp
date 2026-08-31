#include "c_api.h"

#include <cassert>
#include <filesystem>
#include <string>

namespace {

void RequireAllocationFailure(lingtu_inspection_store_handle handle, char* value) {
  assert(value == nullptr);
  const char* error = lingtu_inspection_store_last_error(handle);
  assert(error != nullptr);
  assert(std::string(error).find("alloc") != std::string::npos);
}

}  // namespace

int main() {
  const auto root =
      std::filesystem::temp_directory_path() / "lingtu_inspection_c_api_exception_test";
  std::error_code ec;
  std::filesystem::remove_all(root, ec);

  const std::string root_text = root.string();
  const auto handle = lingtu_inspection_store_create(root_text.c_str());
  assert(handle != nullptr);

  lingtu_inspection_point point{};
  point.id = "camera_a";
  point.enabled = 1;
  point.position_tolerance_m = 0.25;
  point.yaw_tolerance_rad = 0.25;

  lingtu_inspection_route route{};
  route.id = "north_loop";
  route.name = "North loop";
  route.map_id = "factory";
  route.map_content_epoch = 7;
  route.revision = 1U;
  route.loop_count = 1U;
  route.points = &point;
  route.point_count = 1U;
  assert(lingtu_inspection_store_put(handle, &route) == 0);

  RequireAllocationFailure(
      handle,
      lingtu_inspection_store_get_json(handle, route.map_id, route.id));
  RequireAllocationFailure(
      handle,
      lingtu_inspection_store_list_json(handle, route.map_id));
  RequireAllocationFailure(handle, lingtu_inspection_store_status_json(handle));

  assert(lingtu_inspection_store_delete(handle, route.map_id, route.id) == 0);
  lingtu_inspection_store_destroy(handle);
  std::filesystem::remove_all(root, ec);
  return 0;
}

#include "lingtu/maps/build/pcd.hpp"
#include "lingtu/maps/c_api/pcd.h"

#include <cassert>
#include <cmath>
#include <filesystem>
#include <limits>

int main() {
  const auto root = std::filesystem::temp_directory_path() /
                    "lingtu_maps_pcd_c_api_test";
  std::error_code ec;
  std::filesystem::remove_all(root, ec);
  const auto output = root / "capture" / "map.pcd";

  const float points[] = {
      0.0F, 1.0F, 2.0F, 10.0F,
      std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F, 11.0F,
      600.0F, 0.0F, 0.0F, 12.0F,
      -2.0F, -1.0F, 0.5F, 13.0F,
  };
  uint64_t written = 0U;
  assert(lingtu_maps_write_xyz_pcd(
             output.string().c_str(), points, 4U, 4U, 500.0F, &written) == 0);
  assert(written == 2U);

  const auto loaded = lingtu::maps::LoadPcdXyz(output);
  assert(loaded.ok);
  assert(loaded.points.size() == 2U);
  assert(std::abs(loaded.points[1].x + 2.0F) < 1.0e-6F);

  std::filesystem::remove_all(root, ec);
  return 0;
}

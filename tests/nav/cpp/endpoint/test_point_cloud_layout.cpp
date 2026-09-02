#include <cstdio>
#include <cstdlib>

#include "traversability/point_cloud_layout.hpp"

namespace {

void require(bool condition, const char *message) {
  if (!condition) {
    std::fprintf(stderr, "test_point_cloud_layout failed: %s\n", message);
    std::exit(1);
  }
}

}  // namespace

int main() {
  using lingtu::nav::endpoint::pointCloudStorageIsValid;
  using lingtu::nav::endpoint::pointFieldIsScalarFloat32;

  require(pointFieldIsScalarFloat32(7, 1, 8, 12), "valid z float32 field must pass");
  require(!pointFieldIsScalarFloat32(8, 1, 8, 12), "float64 field must be rejected");
  require(!pointFieldIsScalarFloat32(7, 2, 8, 16), "vector field must be rejected");
  require(!pointFieldIsScalarFloat32(7, 1, 10, 12), "field beyond point_step must be rejected");

  require(pointCloudStorageIsValid(2, 2, 12, 24, 48), "valid organized cloud must pass");
  require(pointCloudStorageIsValid(2, 0, 12, 0, 24),
          "height-zero unorganized cloud must use one row");
  require(!pointCloudStorageIsValid(2, 2, 12, 20, 48), "short row_step must be rejected");
  require(!pointCloudStorageIsValid(2, 2, 12, 24, 47), "short data buffer must be rejected");

  std::puts("test_point_cloud_layout passed");
  return 0;
}

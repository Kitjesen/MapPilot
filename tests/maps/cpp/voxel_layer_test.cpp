#include "lingtu/maps/layers/voxel.hpp"
#include "lingtu/maps/layers/grid.hpp"

#include <cassert>
#include <cmath>
#include <vector>

namespace {

using lingtu::maps::CloudLayout;
using lingtu::maps::MapCloudFrame;
using lingtu::maps::OwnedPointCloud;
using lingtu::maps::layers::VoxelLayerConfig;
using lingtu::maps::layers::VoxelLayerCore;

struct CloudFixture {
  OwnedPointCloud owned;
  MapCloudFrame frame;
};

CloudFixture InterleavedFrame(
    std::vector<float> xyz,
    float origin_x = 0.0F,
    float origin_y = 0.0F,
    float origin_z = 0.0F) {
  CloudFixture fixture;
  fixture.owned.frame_id = "map";
  fixture.owned.layout = CloudLayout::kXyzF32Interleaved;
  fixture.owned.point_count = xyz.size() / 3U;
  fixture.owned.interleaved = std::move(xyz);

  fixture.frame.cloud = fixture.owned.View();
  fixture.frame.sensor_origin_x_m = origin_x;
  fixture.frame.sensor_origin_y_m = origin_y;
  fixture.frame.sensor_origin_z_m = origin_z;
  return fixture;
}

CloudFixture SoAFrame(
    std::vector<float> x,
    std::vector<float> y,
    std::vector<float> z) {
  CloudFixture fixture;
  fixture.owned.frame_id = "map";
  fixture.owned.layout = CloudLayout::kXyzF32SoA;
  fixture.owned.point_count = x.size();
  fixture.owned.x = std::move(x);
  fixture.owned.y = std::move(y);
  fixture.owned.z = std::move(z);

  fixture.frame.cloud = fixture.owned.View();
  return fixture;
}

void TestFilterAndVoxelDedupe() {
  VoxelLayerConfig config;
  config.voxel_size_m = 0.10F;
  config.max_range_m = 1.0F;
  config.min_z_m = -0.1F;
  config.max_z_m = 0.5F;
  config.column_carving = false;

  VoxelLayerCore layer(config);
  auto frame = InterleavedFrame({
      0.01F, 0.01F, 0.01F,  // voxel 0,0,0
      0.09F, 0.09F, 0.09F,  // same voxel
      0.15F, 0.05F, 0.01F,  // voxel 1,0,0
      3.00F, 0.00F, 0.00F,  // range filtered
      0.20F, 0.00F, 2.00F,  // z filtered
  });

  layer.Update(frame.frame);
  assert(layer.VoxelCount() == 2U);
  assert(layer.Contains(0.01F, 0.01F, 0.01F));
  assert(layer.Contains(0.15F, 0.05F, 0.01F));

  const auto stats = layer.LastStats();
  assert(stats.input_points == 5U);
  assert(stats.accepted_points == 3U);
  assert(stats.input_voxels == 2U);
  assert(stats.total_voxels == 2U);
}

void TestColumnCarvingReplacesObservedColumns() {
  VoxelLayerConfig config;
  config.voxel_size_m = 1.0F;
  config.max_range_m = 0.0F;
  config.min_z_m = -10.0F;
  config.max_z_m = 10.0F;
  config.column_carving = true;

  VoxelLayerCore layer(config);
  auto first_frame = InterleavedFrame({
      0.2F, 0.2F, 0.2F,
      0.2F, 0.2F, 1.2F,
      2.2F, 0.2F, 0.2F,
  });
  layer.Update(first_frame.frame);
  assert(layer.VoxelCount() == 3U);

  auto second_frame = InterleavedFrame({
      0.3F, 0.3F, 0.3F,
  });
  layer.Update(second_frame.frame);
  assert(layer.VoxelCount() == 2U);
  assert(layer.Contains(0.3F, 0.3F, 0.3F));
  assert(!layer.Contains(0.2F, 0.2F, 1.2F));
  assert(layer.Contains(2.2F, 0.2F, 0.2F));

  const auto stats = layer.LastStats();
  assert(stats.carved_columns == 1U);
  assert(stats.carved_voxels == 2U);
  assert(stats.total_voxels == 2U);
}

void TestColumnCarvingPreservesOtherHeightBands() {
  VoxelLayerConfig config;
  config.voxel_size_m = 1.0F;
  config.max_range_m = 0.0F;
  config.min_z_m = -10.0F;
  config.max_z_m = 10.0F;
  config.column_carving = true;

  VoxelLayerCore layer(config);
  auto first_frame = InterleavedFrame({
      0.2F, 0.2F, 0.2F,
      0.2F, 0.2F, 4.2F,
  });
  layer.Update(first_frame.frame);

  auto second_frame = InterleavedFrame({0.3F, 0.3F, 0.3F});
  second_frame.frame.column_carving_z_range_enabled = true;
  second_frame.frame.column_carving_min_z_m = -1.0F;
  second_frame.frame.column_carving_max_z_m = 2.0F;
  layer.Update(second_frame.frame);
  assert(layer.Contains(0.3F, 0.3F, 0.3F));
  assert(layer.Contains(0.2F, 0.2F, 4.2F));
  assert(layer.VoxelCount() == 2U);
}

void TestSoAInputAndSnapshotCloud() {
  VoxelLayerConfig config;
  config.voxel_size_m = 0.5F;
  config.max_range_m = 0.0F;
  config.min_z_m = -1.0F;
  config.max_z_m = 1.0F;
  config.column_carving = false;

  VoxelLayerCore layer(config);
  auto frame = SoAFrame(
      {0.1F, -0.1F},
      {0.1F, -0.1F},
      {0.1F, -0.1F});
  layer.Update(frame.frame);
  assert(layer.VoxelCount() == 2U);

  const auto snapshot = layer.SnapshotCloud();
  assert(snapshot.layout == CloudLayout::kXyzF32SoA);
  assert(snapshot.point_count == 2U);
  assert(snapshot.x.size() == 2U);
  assert(snapshot.y.size() == 2U);
  assert(snapshot.z.size() == 2U);
}

void TestRollingWindowEvictsBeforeAdmission() {
  VoxelLayerConfig config;
  config.column_carving = false;
  config.max_range_m = 0;
  config.max_z_m = 100;
  config.min_z_m = -100;
  config.max_voxels = 1;
  VoxelLayerCore layer(config);
  lingtu::maps::layers::VoxelSnapshotRequest window;
  window.radius_m = 2;
  window.min_z_m = -1;
  window.max_z_m = 1;
  auto first = InterleavedFrame({1, 0, 0});
  layer.Update(first.frame, window);
  assert(layer.VoxelCount() == 1U);
  window.center_x_m = 10;
  auto next = InterleavedFrame({11, 0, 0, 1, 0, 0, 11, 0, 30});
  layer.Update(next.frame, window);
  assert(layer.VoxelCount() == 1U);
  assert(layer.Contains(11, 0, 0));
  assert(!layer.Contains(1, 0, 0));
  assert(layer.LastStats().capacity_rejected_voxels == 0U);
  window.min_z_m = 20;
  window.max_z_m = 40;
  auto upper = InterleavedFrame({11, 0, 30});
  layer.Update(upper.frame, window);
  assert(layer.Contains(11, 0, 30));
  assert(!layer.Contains(11, 0, 0));
  assert(layer.LastStats().capacity_rejected_voxels == 0U);
}

void TestModelSnapshotKeepsAllHeightsInsideGrid() {
  VoxelLayerConfig config;
  config.voxel_size_m = 0.1F;
  config.column_carving = false;
  VoxelLayerCore layer(config);
  auto frame = InterleavedFrame({
      -0.15F, 0.05F, -0.35F,
      -0.15F, 0.05F, 0.45F,
      0.05F, 0.15F, -0.35F,
      -0.25F, 0.05F, 0.0F,  // Outside the grid's left edge.
      0.25F, 0.05F, 0.0F,   // Outside the grid's right edge.
      0.05F, -0.15F, 0.0F,
      0.05F, 0.25F, 0.0F,
  });
  layer.Update(frame.frame);
  const auto window = lingtu::maps::layers::makeGrid2D(2, 4, 0.1, -0.2, 0.0);
  const auto xyz = layer.SnapshotXyz(window);
  assert(xyz.size() == 9U);
  int low = 0, high = 0;
  for (std::size_t i = 0; i < xyz.size(); i += 3U) {
    assert(xyz[i] >= -0.2F && xyz[i] < 0.2F);
    assert(xyz[i+1] >= 0.0F && xyz[i+1] < 0.2F);
    if (xyz[i+2] < 0.0F) ++low;
    else ++high;
  }
  assert(low == 2 && high == 1);
  assert(layer.VoxelCount() == 7U);
  assert(layer.SnapshotXyz({}).empty());
}

void TestDecayPrunesWeakVoxels() {
  VoxelLayerConfig config;
  config.voxel_size_m = 1.0F;
  config.max_range_m = 0.0F;
  config.decay_rate = 0.75F;
  config.prune_below_count = 1.0F;

  VoxelLayerCore layer(config);
  auto frame = InterleavedFrame({
      0.1F, 0.1F, 0.1F,
      0.1F, 0.1F, 0.1F,
      0.1F, 0.1F, 0.1F,
  });
  layer.Update(frame.frame);
  assert(layer.VoxelCount() == 1U);

  layer.Decay();
  assert(layer.VoxelCount() == 0U);
}

}  // namespace

int main() {
  TestFilterAndVoxelDedupe();
  TestColumnCarvingReplacesObservedColumns();
  TestColumnCarvingPreservesOtherHeightBands();
  TestSoAInputAndSnapshotCloud();
  TestDecayPrunesWeakVoxels();
  TestRollingWindowEvictsBeforeAdmission();
  TestModelSnapshotKeepsAllHeightsInsideGrid();
  return 0;
}

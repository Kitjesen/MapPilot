#include "lingtu/maps/build/pipeline.hpp"
#include "lingtu/maps/cloud.hpp"
#include "lingtu/maps/layers/semantic_occupancy.hpp"
#include "lingtu/maps/layers/voxel.hpp"
#include "lingtu/maps/model.hpp"
#include "lingtu/maps/store.hpp"

int main() {
  lingtu::maps::MapRecord record;
  record.map_id = "smoke";
  record.scope.frame_id = "map";

  lingtu::maps::PointCloudView cloud;
  cloud.frame_id = record.scope.frame_id;
  cloud.point_count = 0;

  lingtu::maps::MapCloudFrame frame;
  frame.cloud = cloud;

  lingtu::maps::layers::VoxelLayerCore voxel_layer;
  voxel_layer.Update(frame);

  lingtu::maps::layers::SemanticOccupancyLayerCore semantic_layer;
  lingtu::maps::layers::SemanticObservationFrame observation;
  observation.frame = frame;
  semantic_layer.Update(observation);

  return (record.map_id == "smoke" && voxel_layer.VoxelCount() == 0 &&
          semantic_layer.VoxelCount() == 0)
             ? 0
             : 1;
}

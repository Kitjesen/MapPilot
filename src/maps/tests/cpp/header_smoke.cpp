#include "lingtu/maps/api.hpp"
#include "lingtu/maps/build/artifacts.hpp"
#include "lingtu/maps/build/optimizer.hpp"
#include "lingtu/maps/build/pipeline.hpp"
#include "lingtu/maps/cloud.hpp"
#include "lingtu/maps/frame.hpp"
#include "lingtu/maps/layers/elevation.hpp"
#include "lingtu/maps/layers/esdf.hpp"
#include "lingtu/maps/layers/occupancy.hpp"
#include "lingtu/maps/layers/semantic_occupancy.hpp"
#include "lingtu/maps/layers/traversability.hpp"
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

  lingtu::maps::layers::OccupancyGridView occupancy;
  occupancy.frame_id = record.scope.frame_id;

  lingtu::maps::layers::ElevationGridView elevation;
  elevation.frame_id = record.scope.frame_id;

  lingtu::maps::layers::EsdfGridView esdf;
  esdf.frame_id = record.scope.frame_id;

  lingtu::maps::layers::TraversabilityGridView traversability;
  traversability.frame_id = record.scope.frame_id;

  lingtu::maps::MapBundle bundle;
  bundle.map_id = record.map_id;
  bundle.capability = lingtu::maps::MapCapability::kVisualization;

  lingtu::maps::layers::VoxelLayerCore voxel_layer;
  voxel_layer.Update(frame);

  lingtu::maps::layers::SemanticOccupancyLayerCore semantic_layer;
  lingtu::maps::layers::SemanticObservationFrame observation;
  observation.frame = frame;
  semantic_layer.Update(observation);

  return (bundle.map_id == "smoke" && voxel_layer.VoxelCount() == 0 &&
          semantic_layer.VoxelCount() == 0)
             ? 0
             : 1;
}

#include "lingtu/maps/io/pcd.hpp"
#include "lingtu/maps/octomap/rays.hpp"

#include <octomap/OcTree.h>

#include <cassert>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using lingtu::maps::PointXyz;

class TempDirectory {
 public:
  explicit TempDirectory(const std::string& label) {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    path_ = std::filesystem::temp_directory_path() /
            ("lingtu_maps_ray_test_" + label + "_" + std::to_string(stamp));
    std::filesystem::create_directories(path_ / "patches");
  }

  ~TempDirectory() { std::filesystem::remove_all(path_); }

  const std::filesystem::path& path() const { return path_; }

 private:
  std::filesystem::path path_;
};

void WriteText(const std::filesystem::path& path, const std::string& text) {
  std::ofstream stream(path, std::ios::binary | std::ios::trunc);
  stream << text;
  assert(stream.good());
}

void WriteBundle(const std::filesystem::path& directory,
                 const std::vector<PointXyz>& retained,
                 const std::vector<std::vector<PointXyz>>& scans) {
  std::string error;
  assert(lingtu::maps::WriteBinaryXyzPcd(directory / "map.pcd", retained, &error));
  std::string poses;
  for (std::size_t index = 0; index < scans.size(); ++index) {
    const auto name = std::to_string(index) + ".pcd";
    assert(lingtu::maps::WriteBinaryXyzPcd(
        directory / "patches" / name, scans[index], &error));
    poses += name + " 0 0 0 1 0 0 0\n";
  }
  WriteText(directory / "poses.txt", poses);
  WriteText(directory / "scan_origin.txt", "lidar_origin_in_patch 0.05 0.05 0.05\n");
}

const octomap::OcTreeNode* NodeAt(const octomap::OcTree& tree, const PointXyz& point) {
  return tree.search(point.x, point.y, point.z);
}

void TestDroppedEndpointKeepsMeasuredPrefix() {
  TempDirectory directory("dropped_prefix");
  const PointXyz dropped{2.05F, 0.05F, 0.05F};
  const PointXyz retained{0.05F, 2.05F, 0.05F};
  WriteBundle(directory.path(), {retained}, {{dropped, retained}});

  octomap::OcTree tree(0.1);
  const auto stats = lingtu::maps::PopulateSavedRayOctomap(tree, directory.path());
  assert(stats.valid_endpoints == 2U);
  assert(stats.retained_endpoints == 1U);
  assert(stats.dropped_endpoints == 1U);
  assert(stats.valid_endpoints == stats.retained_endpoints + stats.dropped_endpoints);
  assert(stats.inserted_points() == stats.retained_endpoints);
  const auto* prefix = tree.search(0.55, 0.05, 0.05);
  assert(prefix != nullptr && !tree.isNodeOccupied(prefix));
  assert(NodeAt(tree, dropped) == nullptr);
  assert(NodeAt(tree, retained) != nullptr && tree.isNodeOccupied(NodeAt(tree, retained)));
}

void TestNearbyRetainedPointKeepsMeasuredEndpoint() {
  TempDirectory directory("nearby_retained_endpoint");
  const PointXyz retained{0.087589838F, 0.103406139F, -0.328530699F};
  const PointXyz measured{0.099648373F, 0.097906159F, -0.310658816F};
  WriteBundle(directory.path(), {retained}, {{measured}});

  octomap::OcTree tree(0.05);
  const auto stats = lingtu::maps::PopulateSavedRayOctomap(tree, directory.path());
  assert(stats.valid_endpoints == 1U);
  assert(stats.retained_endpoints == 1U);
  assert(stats.dropped_endpoints == 0U);
  assert(NodeAt(tree, measured) != nullptr && tree.isNodeOccupied(NodeAt(tree, measured)));
  assert(NodeAt(tree, retained) == nullptr);
}

void TestRetainedPointBeyondFiveCentimetresDoesNotMatch() {
  TempDirectory directory("retained_endpoint_too_far");
  const PointXyz retained{1.101F, 0.05F, 0.05F};
  const PointXyz measured{1.05F, 0.05F, 0.05F};
  const PointXyz anchor{0.05F, 2.05F, 0.05F};
  WriteBundle(directory.path(), {retained, anchor}, {{measured, anchor}});

  octomap::OcTree tree(0.02);
  const auto stats = lingtu::maps::PopulateSavedRayOctomap(tree, directory.path());
  assert(stats.valid_endpoints == 2U);
  assert(stats.retained_endpoints == 1U);
  assert(stats.dropped_endpoints == 1U);
  const auto* measured_node = NodeAt(tree, measured);
  assert(measured_node == nullptr || !tree.isNodeOccupied(measured_node));
}

void TestRetainedEndpointMatchUsesThreeDimensionalDistance() {
  TempDirectory directory("retained_endpoint_3d_distance");
  const PointXyz retained{1.08F, 0.08F, 0.08F};
  const PointXyz measured{1.05F, 0.05F, 0.05F};
  const PointXyz anchor{0.05F, 2.05F, 0.05F};
  WriteBundle(directory.path(), {retained, anchor}, {{measured, anchor}});

  octomap::OcTree tree(0.02);
  const auto stats = lingtu::maps::PopulateSavedRayOctomap(tree, directory.path());
  assert(stats.retained_endpoints == 1U);
  assert(stats.dropped_endpoints == 1U);
  const auto* measured_node = NodeAt(tree, measured);
  assert(measured_node == nullptr || !tree.isNodeOccupied(measured_node));
}

void TestSameFrameHitWinsOverMiss() {
  TempDirectory directory("hit_wins");
  const PointXyz hit{1.05F, 0.05F, 0.05F};
  const PointXyz anchor{0.05F, 2.05F, 0.05F};
  const PointXyz crossing_endpoint{2.55F, 0.05F, 0.05F};
  WriteBundle(directory.path(), {hit, anchor}, {{hit, crossing_endpoint, anchor}});

  octomap::OcTree tree(0.1);
  const auto stats = lingtu::maps::PopulateSavedRayOctomap(tree, directory.path());
  const auto* hit_node = NodeAt(tree, hit);
  assert(hit_node != nullptr && tree.isNodeOccupied(hit_node));
  assert(std::abs(hit_node->getLogOdds() - tree.getProbHitLog()) < 1e-6F);
  assert(stats.hit_updates == 2U);
  assert(stats.dropped_endpoints == 1U);
}

void TestPerFrameDedupAndCrossFrameAccumulation() {
  TempDirectory duplicate_directory("dedup_duplicate");
  TempDirectory single_directory("dedup_single");
  TempDirectory cross_frame_directory("dedup_cross_frame");
  const PointXyz hit{1.05F, 0.05F, 0.05F};
  WriteBundle(duplicate_directory.path(), {hit}, {{hit, hit}});
  WriteBundle(single_directory.path(), {hit}, {{hit}});
  WriteBundle(cross_frame_directory.path(), {hit}, {{hit}, {hit}});

  octomap::OcTree duplicate_tree(0.1);
  octomap::OcTree single_tree(0.1);
  octomap::OcTree cross_frame_tree(0.1);
  const auto duplicate =
      lingtu::maps::PopulateSavedRayOctomap(duplicate_tree, duplicate_directory.path());
  const auto single =
      lingtu::maps::PopulateSavedRayOctomap(single_tree, single_directory.path());
  const auto cross_frame =
      lingtu::maps::PopulateSavedRayOctomap(cross_frame_tree, cross_frame_directory.path());
  assert(duplicate.valid_endpoints == 2U);
  assert(duplicate.retained_endpoints == 2U);
  assert(duplicate.hit_updates == single.hit_updates);
  assert(duplicate.free_updates == single.free_updates);
  assert(cross_frame.hit_updates == 2U * single.hit_updates);
  assert(cross_frame.free_updates == 2U * single.free_updates);
  const auto* hit_node = NodeAt(cross_frame_tree, hit);
  assert(hit_node != nullptr && cross_frame_tree.isNodeOccupied(hit_node));
  assert(std::abs(hit_node->getLogOdds() - 2.0F * cross_frame_tree.getProbHitLog()) < 1e-6F);
}

void TestDiagonalGuardUsesMetricDistance() {
  TempDirectory directory("diagonal_guard");
  const PointXyz endpoint{2.05F, 2.05F, 0.05F};
  const PointXyz protected_surface{1.45F, 1.45F, 0.05F};
  const PointXyz clearable_surface{1.25F, 1.25F, 0.05F};
  WriteBundle(directory.path(), {endpoint, protected_surface, clearable_surface}, {{endpoint}});

  octomap::OcTree tree(0.1);
  const auto stats = lingtu::maps::PopulateSavedRayOctomap(tree, directory.path());
  assert(NodeAt(tree, protected_surface) == nullptr);
  const auto* clearable = NodeAt(tree, clearable_surface);
  assert(clearable != nullptr && !tree.isNodeOccupied(clearable));
  assert(NodeAt(tree, endpoint) != nullptr && tree.isNodeOccupied(NodeAt(tree, endpoint)));
  assert(stats.guarded_miss_suppressions > 0U);
}

void TestNoRetainedEndpointFails() {
  TempDirectory directory("no_retained_endpoint");
  const PointXyz retained_but_unmeasured{3.05F, 3.05F, 0.05F};
  const PointXyz dropped{2.05F, 0.05F, 0.05F};
  WriteBundle(directory.path(), {retained_but_unmeasured}, {{dropped}});

  octomap::OcTree tree(0.1);
  bool threw = false;
  try {
    (void)lingtu::maps::PopulateSavedRayOctomap(tree, directory.path());
  } catch (const std::runtime_error&) {
    threw = true;
  }
  assert(threw);
}

void TestLongGrazingMissUsesMeasuredSurface() {
  TempDirectory directory("long_grazing_floor");
  const PointXyz floor{2.45F,-.75F,-.489F};
  const PointXyz end{6.05F,-.75F,-.489F};
  std::vector<PointXyz> retained{floor,end};
  // Reduced measured Corridor geometry: normal approximately
  // [0.002985, 0.007853, 0.999965], with the surface inside a 10 cm voxel.
  for (int x=-6;x<=6;++x) for (int y=-6;y<=6;++y) {
    if(x==3 && y==2)continue;
    const float px=floor.x+.05F*x,py=floor.y+.05F*y;
    retained.push_back({px,py,static_cast<float>(floor.z-.002985*(px-floor.x)-.007853*(py-floor.y))});
  }
  retained.push_back({floor.x+.15F,floor.y+.10F,floor.z-.12F});
  std::vector<std::vector<PointXyz>> scans{{floor}};
  for(int i=0;i<6;++i) scans.push_back({end});
  WriteBundle(directory.path(),retained,scans);
  WriteText(directory.path()/"scan_origin.txt","lidar_origin_in_patch -24 -0.75 0.05\n");
  octomap::OcTree tree(.1);
  const auto stats=lingtu::maps::PopulateSavedRayOctomap(tree,directory.path());
  const auto* node=NodeAt(tree,floor);
  assert(node!=nullptr && tree.isNodeOccupied(node));
  assert(std::abs(node->getLogOdds()-tree.getProbHitLog())<1e-6F);
  assert(stats.surface_guarded_miss_suppressions>=6U);
  // A retained PCD sample with no scan hit is never promoted to occupancy.
  const auto* unhit=tree.search(2.25,-.75,-.489);
  assert(unhit!=nullptr && !tree.isNodeOccupied(unhit));
  assert(tree.search(2.25,-.95,-.489)==nullptr);
}

void TestCrossingRayStillClearsStableSurface(bool other_level) {
  TempDirectory directory(other_level?"other_level_crossing":"stable_surface_crossing");
  const float height=other_level?-.289F:-.489F;
  const PointXyz hit{2.45F,-.75F,height};
  // Arrange a genuine plane crossing at the centre of the measured voxel.
  const float origin_z=height+.30F;
  const PointXyz end{6.05F,-.75F,origin_z+(height-origin_z)*30.05F/26.45F};
  std::vector<PointXyz> retained{hit,end};
  for(int x=-6;x<=6;++x) for(int y=-6;y<=6;++y) {
    retained.push_back({hit.x+.05F*x,hit.y+.05F*y,height});
    if(other_level)retained.push_back({hit.x+.05F*x,hit.y+.05F*y,height-.20F});
  }
  std::vector<std::vector<PointXyz>> scans{{hit}};
  for(int i=0;i<6;++i)scans.push_back({end});
  WriteBundle(directory.path(),retained,scans);
  WriteText(directory.path()/"scan_origin.txt",
    "lidar_origin_in_patch -24 -0.75 "+std::to_string(origin_z)+"\n");
  octomap::OcTree tree(.1);
  lingtu::maps::PopulateSavedRayOctomap(tree,directory.path());
  const auto* node=NodeAt(tree,hit);
  assert(node!=nullptr && !tree.isNodeOccupied(node));
}

void TestLineDoesNotEstablishProtectedSurface() {
  TempDirectory directory("line_is_not_surface");
  const PointXyz hit{2.45F,-.75F,-.489F},end{6.05F,-.75F,-.489F};
  std::vector<PointXyz> retained{hit,end};
  for(int x=-6;x<=6;++x)retained.push_back({hit.x+.05F*x,hit.y,hit.z});
  std::vector<std::vector<PointXyz>> scans{{hit}};
  for(int i=0;i<6;++i)scans.push_back({end});
  WriteBundle(directory.path(),retained,scans);
  WriteText(directory.path()/"scan_origin.txt","lidar_origin_in_patch -24 -0.75 0.05\n");
  octomap::OcTree tree(.1);
  const auto stats=lingtu::maps::PopulateSavedRayOctomap(tree,directory.path());
  const auto* node=NodeAt(tree,hit);
  assert(node!=nullptr && !tree.isNodeOccupied(node));
  assert(stats.surface_guarded_miss_suppressions==0U);
}

}  // namespace

int main() {
  TestDroppedEndpointKeepsMeasuredPrefix();
  TestNearbyRetainedPointKeepsMeasuredEndpoint();
  TestRetainedPointBeyondFiveCentimetresDoesNotMatch();
  TestRetainedEndpointMatchUsesThreeDimensionalDistance();
  TestSameFrameHitWinsOverMiss();
  TestPerFrameDedupAndCrossFrameAccumulation();
  TestDiagonalGuardUsesMetricDistance();
  TestNoRetainedEndpointFails();
  TestLongGrazingMissUsesMeasuredSurface();
  TestCrossingRayStillClearsStableSurface(false);
  TestCrossingRayStillClearsStableSurface(true);
  TestLineDoesNotEstablishProtectedSurface();
  return 0;
}

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

#include <octomap/OcTree.h>

#include "lingtu/maps/io/pcd.hpp"
#include "collision/terrain.hpp"
#include "input/active/octomap.hpp"
#include "planning/global/octoplanner/body_query.hpp"

namespace {

using lingtu::nav::endpoint::ActiveOctomapGate;
using lingtu::nav::endpoint::runWithActiveOctomap;

lingtu::nav::plan::MapIdentity mapIdentity(std::string map_id = "field") {
  return {std::move(map_id), 7, "map"};
}

[[noreturn]] void fail(const std::string &message) {
  throw std::runtime_error(message);
}

void require(bool condition, const std::string &message) {
  if (!condition) {
    fail(message);
  }
}

class TempMapRoot {
 public:
  TempMapRoot() {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    path_ = std::filesystem::temp_directory_path() /
            ("lingtu_nav_active_octomap_gate_" + std::to_string(stamp));
    std::filesystem::remove_all(path_);
    std::filesystem::create_directories(path_ / "field");
  }

  ~TempMapRoot() {
    std::error_code ec;
    std::filesystem::remove_all(path_, ec);
  }

  const std::filesystem::path &path() const { return path_; }

 private:
  std::filesystem::path path_;
};

void writeFile(const std::filesystem::path &path, const std::string &content) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  file << content;
  require(file.good(), "failed to write fixture: " + path.string());
}

std::string readFile(const std::filesystem::path &path) {
  std::ifstream file(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

std::filesystem::path writeValidMap(const std::filesystem::path &root) {
  const auto map_dir = root / "field";
  const auto map_pcd = map_dir / "map.pcd";
  const auto octomap = map_dir / "octomap.ot";
  const std::vector<lingtu::maps::PointXyz> points = {
      {0.0F, 0.0F, 0.0F},
      {1.0F, 1.0F, 0.5F},
  };
  std::string pcd_error;
  require(
      lingtu::maps::WriteBinaryXyzPcd(map_pcd, points, &pcd_error),
      "failed to write PCD fixture: " + pcd_error);
  octomap::OcTree tree(0.2);
  tree.updateNode(octomap::point3d(0.0F, 0.0F, 0.0F), true);
  tree.updateInnerOccupancy();
  require(tree.writeBinary(octomap.string()), "failed to write OctoMap fixture");
  writeFile(map_dir / "metadata.json",
            "{\"schema_version\":\"lingtu.saved_map_artifacts.v1\","
            "\"frame_id\":\"map\","
            "\"artifacts\":{"
            "\"map_pcd\":{\"path\":\"map.pcd\"},"
            "\"octomap\":{\"path\":\"octomap.ot\"}}}");
  return octomap;
}

void testValidActiveMapGetsPrivateSnapshot() {
  TempMapRoot root;
  const auto configured = writeValidMap(root.path());
  ActiveOctomapGate gate(mapIdentity());

  auto result = gate.prepare(configured);

  require(result.ok(), "valid active map was rejected: " + result.reason);
  require(result.artifact->mapId() == "field", "prepared map id is not active map");
  require(result.artifact->identity().content_epoch == 7,
          "prepared map content epoch is not runtime-bound");
  require(result.artifact->identity().frame_id == "map", "prepared frame is invalid");
  require(result.artifact->loadPath() != configured,
          "planner must load a private snapshot, not the mutable active path");
  require(readFile(result.artifact->loadPath()) == readFile(configured),
          "private snapshot bytes differ from validated artifact");
  const auto current = gate.currentIdentity(configured);
  require(current.ok(), "current active map identity is unavailable: " + current.reason);
  require(lingtu::nav::plan::sameMapIdentity(*current.identity, result.artifact->identity()),
          "prepare and current identity disagree");
}

void testConfiguredPathMustBelongToActiveMap() {
  TempMapRoot root;
  (void)writeValidMap(root.path());
  const auto other = root.path() / "other" / "octomap.ot";
  writeFile(other, "unrelated map\n");
  ActiveOctomapGate gate(mapIdentity());

  const auto result = gate.prepare(other);

  require(!result.ok(), "non-active octomap path unexpectedly passed gate");
}

void testPcdPreviewCannotEnterNavigationGate() {
  TempMapRoot root;
  (void)writeValidMap(root.path());
  ActiveOctomapGate gate(mapIdentity());

  const auto result = gate.currentIdentity(root.path() / "field" / "map.pcd");

  require(!result.ok(), "point-cloud preview unexpectedly passed the OctoMap gate");
  require(result.reason.find("PCD preview") != std::string::npos,
          "PCD rejection did not explain the navigation artifact requirement");
}

void testMissingArtifactNeverInvokesPlanner() {
  TempMapRoot root;
  const auto configured = writeValidMap(root.path());
  std::filesystem::remove(configured);
  ActiveOctomapGate gate(mapIdentity());
  lingtu::nav::plan::GlobalPlanRequest request;
  bool planner_called = false;

  bool rejected = false;
  try {
    (void)runWithActiveOctomap(gate, configured, request, {},
                               [&](const auto &, const auto &, const auto &, const auto &) {
                                 planner_called = true;
                                 return lingtu::nav::plan::GlobalPlanResult{};
                               });
  } catch (const std::runtime_error &) {
    rejected = true;
  }

  require(rejected, "guarded planner did not surface gate rejection");
  require(!planner_called, "planner was invoked after active map gate rejected input");
}

void testPlannerLoadsImmutableSnapshotAfterValidation() {
  TempMapRoot root;
  const auto configured = writeValidMap(root.path());
  const std::string accepted_content = readFile(configured);
  lingtu::nav::plan::GlobalPlanRequest request;
  std::filesystem::path planner_path;
  {
    ActiveOctomapGate gate(mapIdentity());
    const auto result = runWithActiveOctomap(
        gate, configured, request, {},
        [&](const auto &snapshot_path, const auto &, const auto &, const auto &) {
          planner_path = snapshot_path;
          writeFile(configured, "source replaced after validation\n");
          require(readFile(planner_path) == accepted_content,
                  "planner snapshot changed after mutable source was replaced");
          return lingtu::nav::plan::GlobalPlanResult{};
        });

    require(result.map_identity.valid(), "guarded planner result has no map identity");
    require(planner_path != configured, "planner still received mutable active artifact path");
    require(std::filesystem::exists(planner_path),
            "cached planner snapshot disappeared while its gate was alive");
  }
  require(!std::filesystem::exists(planner_path),
          "private planner snapshot was not removed with its gate");
}

void testRepeatedPrepareReusesValidatedSnapshot() {
  TempMapRoot root;
  const auto configured = writeValidMap(root.path());
  ActiveOctomapGate gate(mapIdentity());

  const auto first = gate.prepare(configured);
  const auto second = gate.prepare(configured);

  require(first.ok() && second.ok(), "repeated valid prepare failed");
  require(first.artifact.get() == second.artifact.get(),
          "same map revision created duplicate private snapshots");
}

std::filesystem::path writeTerrainMap(const std::filesystem::path &root) {
  const auto configured = writeValidMap(root);
  writeFile(root / "field" / ".content_epoch", "7\n");
  std::vector<lingtu::maps::PointXyz> points;
  for (int x=-15; x<=15; ++x) for (int y=-15; y<=15; ++y) {
    const float px=(x+.5F)*.01F, py=(y+.5F)*.01F;
    points.push_back({px,py,.13F+.08F*px});
  }
  std::string error;
  require(lingtu::maps::WriteBinaryXyzPcd(root / "field" / "map.pcd", points, &error),
          "failed to write original terrain samples: " + error);
  return configured;
}

std::shared_ptr<const lingtu::nav::collision::Body> contactBody() {
  namespace c = lingtu::nav::collision;
  auto body = std::make_shared<c::Body>();
  c::Part torso;
  torso.link = "torso";
  torso.half_size = {.1,.05,.05};
  body->parts.push_back(torso);
  c::Part foot;
  foot.link = "foot";
  foot.center = {0,0,-.3};
  foot.half_size = {.02,.02,.02};
  foot.shape = c::Shape::Sphere;
  foot.radius_m = .02;
  foot.support_contact = true;
  body->parts.push_back(foot);
  return body;
}

void testTerrainUsesDeclaredSourceAndKeepsItsOriginalCoordinates() {
  TempMapRoot root;
  const auto configured = writeTerrainMap(root.path());
  ActiveOctomapGate gate(mapIdentity());
  lingtu::nav::plan::GlobalPlanRequest request;
  request.body = contactBody();
  request.terrain = std::make_shared<const lingtu::nav::collision::Terrain>(
      std::vector<lingtu::nav::collision::Vec3>{{0,0,10}}, .05);
  std::shared_ptr<const lingtu::nav::collision::Terrain> used_terrain;
  (void)runWithActiveOctomap(gate, configured, request, {},
      [&](const auto &snapshot_path, const auto &, const auto &bound, const auto &) {
        require(snapshot_path.parent_path() != configured.parent_path(),
                "fixture did not exercise the private snapshot path");
        used_terrain = bound.terrain;
        require(used_terrain && used_terrain != request.terrain,
                "request did not receive its own map's declared original samples");
        require(std::abs(used_terrain->resolution()-.2) < 1e-12,
                "terrain sample index did not retain the original OctoMap resolution");
        const auto surface = used_terrain->surface({.025,.025,.132}, .06, .13);
        require(surface && std::abs(surface->heightAt(.025,.025)-.132) < 1e-7,
                "raw sample coordinates were replaced by voxel centres or top faces");
        return lingtu::nav::plan::GlobalPlanResult{};
      });
  writeFile(configured.parent_path() / "map.pcd", "source replaced after loading\n");
  const auto prepared = gate.prepare(configured);
  require(prepared.ok() && prepared.artifact->terrain() == used_terrain,
          "repeated preparation rebuilt immutable terrain");
  require(used_terrain->surface({.025,.025,.132}, .06, .13).has_value(),
          "frozen terrain depended on mutable PCD bytes");
}

void testMissingOrUnboundTerrainKeepsTheOrdinaryMapQuery() {
  for (int variant=0; variant<4; ++variant) {
    TempMapRoot root;
    const auto configured = writeTerrainMap(root.path());
    if (variant == 0) std::filesystem::remove(configured.parent_path() / "map.pcd");
    if (variant == 1) writeFile(configured.parent_path() / ".content_epoch", "8\n");
    if (variant == 2) writeFile(configured.parent_path() / "metadata.json",
        "{\"frame_id\":\"odom\",\"artifacts\":{\"map_pcd\":{\"path\":\"map.pcd\"},"
        "\"octomap\":{\"path\":\"octomap.ot\"}}}");
    if (variant == 3) {
      std::filesystem::copy_file(configured, configured.parent_path() / "other.ot");
      writeFile(configured.parent_path() / "metadata.json",
          "{\"frame_id\":\"map\",\"artifacts\":{\"map_pcd\":{\"path\":\"map.pcd\"},"
          "\"octomap\":{\"path\":\"other.ot\"}}}");
    }
    ActiveOctomapGate gate(mapIdentity());
    bool called = false;
    lingtu::nav::plan::GlobalPlanRequest request;
    request.body = contactBody();
    (void)runWithActiveOctomap(gate, configured, request, {},
        [&](const auto &, const auto &, const auto &bound, const auto &) {
          called = true;
          require(!bound.terrain, "unbound raw samples entered the native terrain query");
          return lingtu::nav::plan::GlobalPlanResult{};
        });
    require(called, "missing auxiliary raw samples blocked the existing map query");
  }
}

void testOrdinaryBodyDoesNotLoadUnusedTerrain() {
  for (const bool with_box : {false, true}) {
    TempMapRoot root;
    const auto configured = writeTerrainMap(root.path());
    ActiveOctomapGate gate(mapIdentity());
    writeFile(configured.parent_path() / "map.pcd", "not loaded by this request\n");
    lingtu::nav::plan::GlobalPlanRequest request;
    if (with_box) {
      auto box = std::make_shared<lingtu::nav::collision::Body>();
      lingtu::nav::collision::Part part;
      part.link = "body";
      part.half_size = {.4,.2,.15};
      box->parts.push_back(part);
      request.body = box;
    }
    (void)runWithActiveOctomap(gate, configured, request, {},
        [&](const auto &, const auto &, const auto &bound, const auto &) {
          require(!bound.terrain, "ordinary body unnecessarily built a raw terrain index");
          return lingtu::nav::plan::GlobalPlanResult{};
        });
    (void)writeTerrainMap(root.path());
    request.body = contactBody();
    (void)runWithActiveOctomap(gate, configured, request, {},
        [&](const auto &, const auto &, const auto &bound, const auto &) {
          require(bound.terrain != nullptr,
                  "ordinary query consumed or poisoned the lazy terrain initialization");
          return lingtu::nav::plan::GlobalPlanResult{};
        });
  }
}

void testTerrainIndexUsesFullAndBinaryOctomapResolution() {
  for (const bool binary : {false, true}) {
    TempMapRoot root;
    auto configured = writeTerrainMap(root.path());
    octomap::OcTree tree(.02);
    tree.updateNode(octomap::point3d(0,0,0), true);
    if (binary) {
      configured.replace_extension(".bt");
      require(tree.writeBinary(configured.string()), "failed to write binary terrain map");
      writeFile(configured.parent_path() / "metadata.json",
          "{\"frame_id\":\"map\",\"artifacts\":{\"map_pcd\":{\"path\":\"map.pcd\"},"
          "\"octomap\":{\"path\":\"octomap.bt\"}}}");
    } else require(tree.write(configured.string()), "failed to write full terrain map");
    ActiveOctomapGate gate(mapIdentity());
    const auto prepared = gate.prepare(configured);
    require(prepared.ok() && prepared.artifact->terrain(),
            "supported OctoMap format lost declared original terrain");
    require(std::abs(prepared.artifact->terrain()->resolution()-.02) < 1e-12,
            "terrain hard-coded a different map resolution");
  }
}

void testCompleteSavedHitsRefineWithoutAPlaneAndPreserveManualOccupancy() {
  namespace c = lingtu::nav::collision;
  for (int variant=0; variant<6; ++variant) {
    TempMapRoot root;
    const auto configured=writeValidMap(root.path());
    writeFile(configured.parent_path()/".content_epoch","7\n");
    std::vector<lingtu::maps::PointXyz> hits{{.1F,.1F,.002F}};
    if (variant==1) hits.push_back({.1F,.1F,.15F});
    std::string error;
    require(lingtu::maps::WriteBinaryXyzPcd(configured.parent_path()/"patches"/"0.pcd",hits,&error),
        "failed to save scan: "+error);
    writeFile(configured.parent_path()/"poses.txt","0.pcd 0 0 0 1 0 0 0\n");
    writeFile(configured.parent_path()/"scan_origin.txt","lidar_origin_in_patch 0 0 0\n");
    const std::string retained=std::to_string(hits.size()+(variant==3?1:0));
    writeFile(configured.parent_path()/"metadata.json",
        "{\"frame_id\":\"map\",\"artifacts\":{\"map_pcd\":{\"path\":\"map.pcd\"},"
        "\"octomap\":{\"path\":\"octomap.ot\",\"evidence_source\":\"saved_rays\","
        "\"build_mode\":\"native_octomap\",\"stats\":{\"retained_endpoints\":"+retained+
        ",\"dropped_endpoints\":"+(variant==4?"1":"0")+"}}}"
        +(variant==2?",\"manual_voxel_edits\":{\"count\":1}":"")+"}");
    if (variant==5) std::filesystem::remove(configured.parent_path()/"patches"/"0.pcd");
    ActiveOctomapGate gate(mapIdentity());
    const auto prepared=gate.prepare(configured);
    require(prepared.ok(),"saved-hit fixture map was rejected");
    const auto terrain=prepared.artifact->terrain();
    c::Body body;
    c::Part part;
    part.link="body";
    part.center={.1,.1,.15};
    part.half_size={.02,.02,.02};
    body.parts.push_back(part);
    octomap::OcTree tree(.2);
    tree.updateNode(.1,.1,.1,true);
    const lingtu::nav::plan::GlobalPlanTemporaryOverlay overlay;
    const bool free=octoplanner3d::runtime::BodyQuery(tree,body,overlay,{},terrain).pose({}).free;
    require(free==(variant==0),"saved-hit refinement lost obstacles or missing-evidence fallback");
    if (variant==0) require(terrain && terrain->hitBounds(),"complete original scans lost their hit index");
    if (variant==2) require(!terrain,"manual occupied voxels were exposed to stale scan refinement");
  }
}

void testManualOccupancyCannotBeClearedByAnOldSampledSurface() {
  namespace c=lingtu::nav::collision;
  TempMapRoot root;
  const auto configured=writeTerrainMap(root.path());
  octomap::OcTree tree(.2);
  tree.updateNode(.1,.1,.1,true);
  c::Body body;
  c::Part part;
  part.link="body";
  part.center={.025,.025,.18};
  part.half_size={.01,.01,.01};
  body.parts.push_back(part);
  const lingtu::nav::plan::GlobalPlanTemporaryOverlay overlay;
  ActiveOctomapGate original_gate(mapIdentity());
  const auto original=original_gate.prepare(configured);
  require(original.ok(),"original map was rejected");
  const auto sampled=original.artifact->terrain();
  require(sampled && octoplanner3d::runtime::BodyQuery(tree,body,overlay,{},sampled).pose({}).free,
      "fixture must reproduce the sampled plane clearing the occupied voxel");
  writeFile(configured.parent_path()/"metadata.json",
      "{\"frame_id\":\"map\",\"manual_voxel_edits\":{\"count\":1},"
      "\"artifacts\":{\"map_pcd\":{\"path\":\"map.pcd\"},"
      "\"octomap\":{\"path\":\"octomap.ot\"}}}");
  ActiveOctomapGate gate(mapIdentity());
  const auto prepared=gate.prepare(configured);
  require(prepared.ok(),"manual map lost its active snapshot");
  const auto terrain=prepared.artifact->terrain();
  require(!terrain,"manual occupied cell still uses stale sampled surfaces");
  require(!octoplanner3d::runtime::BodyQuery(tree,body,overlay,{},terrain).pose({}).free,
      "manually occupied voxel was cleared by an original ground sample");
}

}  // namespace

int main() {
  try {
    testValidActiveMapGetsPrivateSnapshot();
    testConfiguredPathMustBelongToActiveMap();
    testPcdPreviewCannotEnterNavigationGate();
    testMissingArtifactNeverInvokesPlanner();
    testPlannerLoadsImmutableSnapshotAfterValidation();
    testRepeatedPrepareReusesValidatedSnapshot();
    testTerrainUsesDeclaredSourceAndKeepsItsOriginalCoordinates();
    testMissingOrUnboundTerrainKeepsTheOrdinaryMapQuery();
    testOrdinaryBodyDoesNotLoadUnusedTerrain();
    testTerrainIndexUsesFullAndBinaryOctomapResolution();
    testCompleteSavedHitsRefineWithoutAPlaneAndPreserveManualOccupancy();
    testManualOccupancyCannotBeClearedByAnOldSampledSurface();
    std::cout << "test_active_octomap_gate: PASS\n";
    return 0;
  } catch (const std::exception &exc) {
    std::cerr << "test_active_octomap_gate: FAIL: " << exc.what() << "\n";
    return 1;
  }
}

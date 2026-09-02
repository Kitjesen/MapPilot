#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

#include <octomap/OcTree.h>

#include "lingtu/maps/build/pcd.hpp"
#include "input/active/octomap.hpp"

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

}  // namespace

int main() {
  try {
    testValidActiveMapGetsPrivateSnapshot();
    testConfiguredPathMustBelongToActiveMap();
    testMissingArtifactNeverInvokesPlanner();
    testPlannerLoadsImmutableSnapshotAfterValidation();
    testRepeatedPrepareReusesValidatedSnapshot();
    std::cout << "test_active_octomap_gate: PASS\n";
    return 0;
  } catch (const std::exception &exc) {
    std::cerr << "test_active_octomap_gate: FAIL: " << exc.what() << "\n";
    return 1;
  }
}

#include "active_occupancy_gate.hpp"

#include "lingtu/maps/build/grid_artifacts.hpp"
#include "lingtu/maps/build/occupancy_snapshot.hpp"
#include "lingtu/maps/hash.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

using lingtu::maps::BuildOccupancyProjectionSnapshot;
using lingtu::maps::LoadOccupancyArtifact;
using lingtu::maps::Sha256File;
using lingtu::nav::endpoint::ActiveOccupancyGate;
using lingtu::nav::endpoint::runWithActiveOccupancy;
using lingtu::nav::plan::GlobalPlanRequest;
using lingtu::nav::plan::far::FarPlanner;
using lingtu::nav::plan::far::FarPlannerConfig;

[[noreturn]] void fail(const std::string& message) {
  throw std::runtime_error(message);
}

void require(bool condition, const std::string& message) {
  if (!condition) {
    fail(message);
  }
}

class TempMapRoot {
 public:
  TempMapRoot() {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    path_ = std::filesystem::temp_directory_path() /
        ("lingtu_nav_active_occupancy_gate_" + std::to_string(stamp));
    std::filesystem::remove_all(path_);
    std::filesystem::create_directories(path_);
  }

  ~TempMapRoot() {
    std::error_code error;
    std::filesystem::remove_all(path_, error);
  }

  const std::filesystem::path& path() const { return path_; }

 private:
  std::filesystem::path path_;
};

void writeFile(const std::filesystem::path& path, const std::string& content) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  file << content;
  require(file.good(), "failed to write fixture: " + path.string());
}

void writeOpenRoomPcd(const std::filesystem::path& path) {
  writeFile(
      path,
      "# .PCD v0.7\n"
      "VERSION 0.7\n"
      "FIELDS x y z\n"
      "SIZE 4 4 4\n"
      "TYPE F F F\n"
      "COUNT 1 1 1\n"
      "WIDTH 4\n"
      "HEIGHT 1\n"
      "VIEWPOINT 0 0 0 1 0 0 0\n"
      "POINTS 4\n"
      "DATA ascii\n"
      "0 0 0\n"
      "5 0 0\n"
      "0 5 0\n"
      "5 5 0\n");
}

std::filesystem::path writeValidActiveMap(
    const std::filesystem::path& root,
    const std::string& map_id) {
  const auto map_dir = root / map_id;
  const auto map_pcd = map_dir / "map.pcd";
  writeOpenRoomPcd(map_pcd);
  const auto built = BuildOccupancyProjectionSnapshot(map_dir);
  require(built.ok, "failed to build occupancy fixture: " + built.message);
  const auto occupancy = map_dir / "occupancy.npz";
  const std::string map_sha = Sha256File(map_pcd);
  const std::string occupancy_sha = Sha256File(occupancy);
  writeFile(
      map_dir / "metadata.json",
      "{\"schema_version\":\"lingtu.saved_map_artifacts.v1\","
      "\"frame_id\":\"map\","
      "\"artifacts\":{"
      "\"map_pcd\":{\"path\":\"map.pcd\",\"sha256\":\"" + map_sha + "\"},"
      "\"occupancy_grid\":{\"path\":\"occupancy.npz\",\"sha256\":\"" +
          occupancy_sha + "\",\"source_map_sha256\":\"" + map_sha + "\"}}}");
  writeFile(root / "active_map.txt", map_id + "\n");
  return occupancy;
}

FarPlannerConfig plannerConfig() {
  FarPlannerConfig config;
  config.robot_radius_m = 0.0;
  config.obstacle_clearance_m = 0.0;
  config.max_visibility_distance_m = 100.0;
  config.snap_search_radius_cells = 2;
  return config;
}

GlobalPlanRequest roomRequest() {
  GlobalPlanRequest request;
  request.start = {0.5, 0.5, 0.0};
  request.goal = {4.5, 4.5, 0.0};
  return request;
}

void testValidActiveOccupancyLoadsThroughNativeMapsContract() {
  TempMapRoot root;
  const auto configured = writeValidActiveMap(root.path(), "field");
  ActiveOccupancyGate gate(root.path());

  const auto result = gate.prepare(configured);

  require(result.ok(), "valid active occupancy was rejected: " + result.reason);
  require(result.artifact->identity().map_id == "field", "wrong active map id");
  require(result.artifact->identity().version == 1, "wrong active map version");
  require(result.artifact->identity().frame_id == "map", "wrong planning frame");
  require(result.artifact->generation() == 1U, "first map generation must be one");
  require(result.artifact->map().width > 20, "occupancy geometry was not loaded");
  require(
      result.artifact->map().cells.size() == result.artifact->map().CellCount(),
      "occupancy payload size does not match geometry");

  const auto decoded = LoadOccupancyArtifact(configured);
  require(decoded.rows == result.artifact->map().height, "loader and FAR rows disagree");
  require(decoded.cols == result.artifact->map().width, "loader and FAR columns disagree");
}

void testTamperedOccupancyIsRejectedBeforePlanner() {
  TempMapRoot root;
  const auto configured = writeValidActiveMap(root.path(), "field");
  writeFile(configured, "tampered occupancy\n");
  ActiveOccupancyGate gate(root.path());
  FarPlanner planner(plannerConfig());
  bool planner_rejected = false;
  try {
    (void)runWithActiveOccupancy(gate, configured, planner, roomRequest());
  } catch (const std::runtime_error&) {
    planner_rejected = true;
  }
  require(planner_rejected, "tampered occupancy reached FAR");
  require(!planner.HasMap(), "FAR accepted a map after gate rejection");
}

void testConfiguredPathMustBeTheNativeActiveArtifact() {
  TempMapRoot root;
  (void)writeValidActiveMap(root.path(), "field");
  const auto other = writeValidActiveMap(root.path(), "other");
  writeFile(root.path() / "active_map.txt", "field\n");
  ActiveOccupancyGate gate(root.path());

  const auto result = gate.prepare(other);

  require(!result.ok(), "non-active occupancy unexpectedly passed the gate");
}

void testMapSwitchAdvancesGenerationAndInvalidatesCache() {
  TempMapRoot root;
  const auto first_path = writeValidActiveMap(root.path(), "field_a");
  ActiveOccupancyGate gate(root.path());
  const auto first = gate.prepare(first_path);
  require(first.ok(), "first active map failed: " + first.reason);
  const auto repeated = gate.prepare(first_path);
  require(repeated.ok(), "repeated active map failed: " + repeated.reason);
  require(
      first.artifact.get() == repeated.artifact.get(),
      "identical active revision was reparsed");

  const auto second_path = writeValidActiveMap(root.path(), "field_b");
  const auto second = gate.prepare(second_path);
  require(second.ok(), "second active map failed: " + second.reason);
  require(second.artifact->identity().map_id == "field_b", "map switch identity is stale");
  require(second.artifact->generation() == 2U, "map switch did not advance generation");
}

void testGuardedFarPlanCarriesValidatedIdentity() {
  TempMapRoot root;
  const auto configured = writeValidActiveMap(root.path(), "field");
  ActiveOccupancyGate gate(root.path());
  FarPlanner planner(plannerConfig());

  const auto result = runWithActiveOccupancy(
      gate, configured, planner, roomRequest());

  require(result.ok, "guarded FAR plan failed: " + result.failure_reason);
  require(result.reached_goal, "guarded FAR plan did not reach the goal");
  require(result.path.size() >= 2U, "guarded FAR plan returned no usable path");
  require(result.map_identity.valid(), "plan result has no validated map identity");
  require(result.map_identity.map_id == "field", "plan result identity is wrong");
  require(result.map_generation == 1U, "plan result generation is wrong");
}

}  // namespace

int main() {
  try {
    testValidActiveOccupancyLoadsThroughNativeMapsContract();
    testTamperedOccupancyIsRejectedBeforePlanner();
    testConfiguredPathMustBeTheNativeActiveArtifact();
    testMapSwitchAdvancesGenerationAndInvalidatesCache();
    testGuardedFarPlanCarriesValidatedIdentity();
    std::cout << "test_active_occupancy_gate: PASS\n";
    return 0;
  } catch (const std::exception& exc) {
    std::cerr << "test_active_occupancy_gate: FAIL: " << exc.what() << "\n";
    return 1;
  }
}

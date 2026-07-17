#include "active_octomap_gate.hpp"

#include "lingtu/maps/hash.hpp"

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

using lingtu::maps::Sha256File;
using lingtu::nav::endpoint::ActiveOctomapGate;
using lingtu::nav::endpoint::runWithActiveOctomap;

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
        ("lingtu_nav_active_octomap_gate_" + std::to_string(stamp));
    std::filesystem::remove_all(path_);
    std::filesystem::create_directories(path_ / "field");
  }

  ~TempMapRoot() {
    std::error_code ec;
    std::filesystem::remove_all(path_, ec);
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

std::filesystem::path writeValidActiveMap(const std::filesystem::path& root) {
  const auto map_dir = root / "field";
  const auto map_pcd = map_dir / "map.pcd";
  const auto octomap = map_dir / "octomap.ot";
  writeFile(map_pcd, "valid point cloud fixture\n");
  writeFile(octomap, "valid octomap fixture\n");
  const std::string map_sha = Sha256File(map_pcd);
  const std::string octomap_sha = Sha256File(octomap);
  writeFile(
      map_dir / "metadata.json",
      "{\"schema_version\":\"lingtu.saved_map_artifacts.v1\","
      "\"frame_id\":\"map\","
      "\"artifacts\":{"
      "\"map_pcd\":{\"path\":\"map.pcd\",\"sha256\":\"" + map_sha + "\"},"
      "\"octomap\":{\"path\":\"octomap.ot\",\"sha256\":\"" + octomap_sha +
          "\",\"source_map_sha256\":\"" + map_sha + "\"}}}");
  writeFile(root / "active_map.txt", "field\n");
  return octomap;
}

void testValidActiveMapGetsPrivateSnapshot() {
  TempMapRoot root;
  const auto configured = writeValidActiveMap(root.path());
  ActiveOctomapGate gate(root.path());

  auto result = gate.prepare(configured);

  require(result.ok(), "valid active map was rejected: " + result.reason);
  require(result.artifact->mapId() == "field", "prepared map id is not active map");
  require(
      result.artifact->loadPath() != configured,
      "planner must load a private snapshot, not the mutable active path");
  require(
      Sha256File(result.artifact->loadPath()) == Sha256File(configured),
      "private snapshot bytes differ from validated artifact");
}

void testTamperedOctomapIsRejected() {
  TempMapRoot root;
  const auto configured = writeValidActiveMap(root.path());
  writeFile(configured, "tampered octomap\n");
  ActiveOctomapGate gate(root.path());

  const auto result = gate.prepare(configured);

  require(!result.ok(), "tampered octomap unexpectedly passed native Maps gate");
}

void testTamperedSourceMapIsRejected() {
  TempMapRoot root;
  const auto configured = writeValidActiveMap(root.path());
  writeFile(root.path() / "field" / "map.pcd", "tampered source map\n");
  ActiveOctomapGate gate(root.path());

  const auto result = gate.prepare(configured);

  require(!result.ok(), "tampered map.pcd unexpectedly passed same-source gate");
}

void testConfiguredPathMustBelongToActiveMap() {
  TempMapRoot root;
  (void)writeValidActiveMap(root.path());
  const auto other = root.path() / "other" / "octomap.ot";
  writeFile(other, "unrelated map\n");
  ActiveOctomapGate gate(root.path());

  const auto result = gate.prepare(other);

  require(!result.ok(), "non-active octomap path unexpectedly passed gate");
}

void testLegacyActiveAliasMustResolveToNativeActiveMap() {
  TempMapRoot root;
  (void)writeValidActiveMap(root.path());
  std::filesystem::create_directory_symlink(root.path() / "field", root.path() / "active");
  ActiveOctomapGate gate(root.path());

  const auto result = gate.prepare(root.path() / "active" / "octomap.ot");

  require(result.ok(), "active alias pointing at native active map was rejected: " + result.reason);
}

void testRejectedGateNeverInvokesPlanner() {
  TempMapRoot root;
  const auto configured = writeValidActiveMap(root.path());
  writeFile(configured, "tampered before planner load\n");
  ActiveOctomapGate gate(root.path());
  lingtu::nav::plan::GlobalPlanRequest request;
  request.map_path = configured.string();
  bool planner_called = false;

  bool rejected = false;
  try {
    (void)runWithActiveOctomap(
        gate,
        request,
        {},
        [&](const auto&, const auto&) {
          planner_called = true;
          return lingtu::nav::plan::GlobalPlanResult{};
        });
  } catch (const std::runtime_error&) {
    rejected = true;
  }

  require(rejected, "guarded planner did not surface gate rejection");
  require(!planner_called, "planner was invoked after active map gate rejected input");
}

void testPlannerLoadsImmutableSnapshotAfterValidation() {
  TempMapRoot root;
  const auto configured = writeValidActiveMap(root.path());
  const std::string accepted_sha = Sha256File(configured);
  ActiveOctomapGate gate(root.path());
  lingtu::nav::plan::GlobalPlanRequest request;
  request.map_path = configured.string();
  std::filesystem::path planner_path;

  (void)runWithActiveOctomap(
      gate,
      request,
      {},
      [&](const auto& validated_request, const auto&) {
        planner_path = validated_request.map_path;
        writeFile(configured, "source replaced after validation\n");
        require(
            Sha256File(planner_path) == accepted_sha,
            "planner snapshot changed after mutable source was replaced");
        return lingtu::nav::plan::GlobalPlanResult{};
      });

  require(planner_path != configured, "planner still received mutable active artifact path");
  require(
      !std::filesystem::exists(planner_path),
      "private planner snapshot was not removed after planner returned");
}

}  // namespace

int main() {
  try {
    testValidActiveMapGetsPrivateSnapshot();
    testTamperedOctomapIsRejected();
    testTamperedSourceMapIsRejected();
    testConfiguredPathMustBelongToActiveMap();
    testLegacyActiveAliasMustResolveToNativeActiveMap();
    testRejectedGateNeverInvokesPlanner();
    testPlannerLoadsImmutableSnapshotAfterValidation();
    std::cout << "test_active_octomap_gate: PASS\n";
    return 0;
  } catch (const std::exception& exc) {
    std::cerr << "test_active_octomap_gate: FAIL: " << exc.what() << "\n";
    return 1;
  }
}

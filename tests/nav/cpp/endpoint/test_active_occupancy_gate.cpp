#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include "input/active/occupancy.hpp"
#include "lingtu/maps/build/occupancy_snapshot.hpp"

namespace {
using lingtu::nav::endpoint::ActiveOccupancyGate;
lingtu::nav::plan::MapIdentity mapIdentity(std::string map_id = "field", std::int64_t epoch = 7) {
  return {std::move(map_id), epoch, "map"};
}
[[noreturn]] void fail(const std::string &message) { throw std::runtime_error(message); }
void require(bool condition, const std::string &message) { if (!condition) fail(message); }
class TempMapRoot {
 public:
  TempMapRoot() {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    path_ = std::filesystem::temp_directory_path() / ("lingtu_nav_active_occupancy_gate_" + std::to_string(stamp));
    std::filesystem::create_directories(path_);
  }
  ~TempMapRoot() { std::error_code error; std::filesystem::remove_all(path_, error); }
  const std::filesystem::path &path() const { return path_; }
 private: std::filesystem::path path_;
};
void writeFile(const std::filesystem::path &path, const std::string &content) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  file << content;
  require(file.good(), "failed to write fixture: " + path.string());
}
std::filesystem::path writeValidMap(const std::filesystem::path &root, const std::string &map_id) {
  const auto map_dir = root / map_id;
  writeFile(map_dir / "map.pcd", "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 4\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 4\nDATA ascii\n0 0 0\n5 0 0\n0 5 0\n5 5 0\n");
  const auto built = lingtu::maps::BuildOccupancyProjectionSnapshot(map_dir);
  require(built.ok, "failed to build occupancy fixture: " + built.message);
  writeFile(map_dir / "metadata.json", "{\"schema_version\":\"lingtu.saved_map_artifacts.v1\",\"frame_id\":\"map\",\"artifacts\":{\"map_pcd\":{\"path\":\"map.pcd\"},\"occupancy_grid\":{\"path\":\"occupancy.npz\"}}}");
  return map_dir / "occupancy.npz";
}
void testValidActiveOccupancyLoads() {
  TempMapRoot root; const auto configured = writeValidMap(root.path(), "field"); ActiveOccupancyGate gate(mapIdentity());
  const auto result = gate.prepare(configured);
  require(result.ok(), "valid active occupancy was rejected: " + result.reason);
  require(result.artifact->identity().map_id == "field", "wrong active map id");
  require(result.artifact->identity().content_epoch == 7, "wrong map content epoch");
  const auto &data = result.artifact->data();
  require(data.rows > 20 && data.grid.size() == static_cast<std::size_t>(data.rows) * static_cast<std::size_t>(data.cols), "occupancy payload geometry mismatch");
}
void testTamperedOccupancyIsRejected() {
  TempMapRoot root; const auto configured = writeValidMap(root.path(), "field"); writeFile(configured, "tampered occupancy\n");
  require(!ActiveOccupancyGate(mapIdentity()).prepare(configured).ok(), "tampered occupancy was accepted");
}
void testConfiguredPathMustMatchIdentity() {
  TempMapRoot root; (void)writeValidMap(root.path(), "field"); const auto other = writeValidMap(root.path(), "other");
  require(!ActiveOccupancyGate(mapIdentity()).prepare(other).ok(), "a different map bypassed identity binding");
}
void testRepeatedPrepareUsesCachedArtifact() {
  TempMapRoot root; const auto configured = writeValidMap(root.path(), "field"); ActiveOccupancyGate gate(mapIdentity());
  const auto first = gate.prepare(configured); const auto repeated = gate.prepare(configured);
  require(first.ok() && repeated.ok(), "repeated active map load failed");
  require(first.artifact.get() == repeated.artifact.get(), "identical map was reparsed");
}
}  // namespace
int main() {
  try {
    testValidActiveOccupancyLoads(); testTamperedOccupancyIsRejected(); testConfiguredPathMustMatchIdentity(); testRepeatedPrepareUsesCachedArtifact();
    std::cout << "test_active_occupancy_gate: PASS\n"; return 0;
  } catch (const std::exception &error) { std::cerr << "test_active_occupancy_gate: FAIL: " << error.what() << "\n"; return 1; }
}

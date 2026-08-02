#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include "lingtu/maps/hash.hpp"
#include "lingtu/maps/store.hpp"
#include "plan/active_occupancy_gate.hpp"
#include "plan/active_octomap_gate.hpp"

namespace {

using lingtu::maps::ArtifactType;
using lingtu::maps::MapStore;
using lingtu::maps::MapStoreConfig;
using lingtu::maps::Sha256File;
using lingtu::nav::endpoint::ActiveOccupancyGate;
using lingtu::nav::endpoint::ActiveOctomapGate;

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
            ("lingtu_nav_declared_identity_" + std::to_string(stamp));
    std::filesystem::remove_all(path_);
    std::filesystem::create_directories(path_);
  }

  ~TempMapRoot() {
    std::error_code error;
    std::filesystem::remove_all(path_, error);
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

struct ActiveFixture {
  std::filesystem::path map_dir;
  std::filesystem::path octomap;
  std::filesystem::path occupancy;
  std::string octomap_sha;
  std::string occupancy_sha;
};

ActiveFixture writeActiveFixture(const std::filesystem::path &root,
                                 const std::string &map_id = "field",
                                 const std::string &frame_id = "map") {
  const auto map_dir = root / map_id;
  const auto map_pcd = map_dir / "map.pcd";
  const auto octomap = map_dir / "octomap.ot";
  const auto occupancy = map_dir / "occupancy.npz";
  writeFile(map_pcd, "point cloud fixture\n");
  writeFile(octomap, "octomap fixture\n");
  writeFile(occupancy, "occupancy fixture\n");
  const std::string map_sha = Sha256File(map_pcd);
  const std::string octomap_sha = Sha256File(octomap);
  const std::string occupancy_sha = Sha256File(occupancy);
  writeFile(map_dir / "metadata.json",
            "{\"schema_version\":\"lingtu.saved_map_artifacts.v1\","
            "\"frame_id\":\"" + frame_id + "\","
            "\"artifacts\":{"
            "\"map_pcd\":{\"path\":\"map.pcd\",\"sha256\":\"" +
                map_sha +
                "\"},"
                "\"octomap\":{\"path\":\"octomap.ot\",\"sha256\":\"" +
                octomap_sha + "\",\"source_map_sha256\":\"" + map_sha + "\"},"
                "\"occupancy_grid\":{\"path\":\"occupancy.npz\",\"sha256\":\"" +
                occupancy_sha + "\",\"source_map_sha256\":\"" + map_sha + "\"}}}");
  writeFile(root / "active_map.txt", map_id + "\n");
  return {map_dir, octomap, occupancy, octomap_sha, occupancy_sha};
}

void testStoreReadsDeclaredIdentityWithoutArtifactBytes() {
  TempMapRoot root;
  auto fixture = writeActiveFixture(root.path());
  MapStore store(MapStoreConfig{root.path()});

  auto declared = store.ReadDeclaredArtifactIdentity("field", ArtifactType::kOctomap3D, "map");
  require(declared.ok(), "declared octomap identity was unavailable: " + declared.reason);
  require(declared.identity->map_id == "field", "wrong map id");
  require(declared.identity->version == 1, "wrong version");
  require(declared.identity->frame_id == "map", "wrong frame");
  require(declared.identity->artifact_sha256 == fixture.octomap_sha,
          "declared sha was not read from metadata");

  writeFile(fixture.octomap, "tampered octomap bytes after metadata\n");
  auto after_tamper = store.ReadDeclaredArtifactIdentity("field", ArtifactType::kOctomap3D, "map");
  require(after_tamper.ok(), "cheap declared identity must not hash artifact bytes");
  require(after_tamper.identity->artifact_sha256 == fixture.octomap_sha,
          "cheap identity must report declared metadata hash");
}

void testStoreVersionedContentAndFrameMismatch() {
  TempMapRoot root;
  const auto versioned = root.path() / "field" / ".versions" / "7";
  std::filesystem::create_directories(versioned);
  auto fixture = writeActiveFixture(versioned.parent_path().parent_path(), "unused");
  std::filesystem::remove_all(root.path() / "unused");
  writeFile(versioned / "octomap.ot", "versioned octomap\n");
  writeFile(versioned / "map.pcd", "versioned pcd\n");
  const std::string map_sha = Sha256File(versioned / "map.pcd");
  const std::string octomap_sha = Sha256File(versioned / "octomap.ot");
  writeFile(versioned / "metadata.json",
            "{\"frame_id\":\"map\",\"artifacts\":{"
            "\"map_pcd\":{\"path\":\"map.pcd\",\"sha256\":\"" + map_sha + "\"},"
            "\"octomap\":{\"path\":\"octomap.ot\",\"sha256\":\"" + octomap_sha +
                "\",\"source_map_sha256\":\"" + map_sha + "\"}}}");
  writeFile(root.path() / "field" / "current_version.txt", "7\n");
  writeFile(root.path() / "active_map.txt", "field\n");

  MapStore store(MapStoreConfig{root.path()});
  auto declared = store.ReadDeclaredArtifactIdentity("field", ArtifactType::kOctomap3D, "map");
  require(declared.ok(), "versioned declared identity failed: " + declared.reason);
  require(declared.identity->version == 7, "versioned identity did not use current_version.txt");
  require(declared.identity->artifact_path == versioned / "octomap.ot",
          "versioned identity used wrong content directory");

  auto wrong_frame = store.ReadDeclaredArtifactIdentity("field", ArtifactType::kOctomap3D, "odom");
  require(!wrong_frame.ok() && wrong_frame.reason.find("frame mismatch") != std::string::npos,
          "frame mismatch was not rejected");
  (void)fixture;
}

void testGatesRequireActivePointerAndConfiguredPathMatch() {
  TempMapRoot root;
  auto fixture = writeActiveFixture(root.path());
  ActiveOctomapGate octomap_gate(root.path());
  ActiveOccupancyGate occupancy_gate(root.path());

  auto octomap_id = octomap_gate.currentDeclaredIdentity(fixture.octomap);
  require(octomap_id.ok(), "octomap declared identity failed: " + octomap_id.reason);
  require(octomap_id.identity->artifact_sha256 == fixture.octomap_sha,
          "octomap gate returned wrong declared hash");

  auto occupancy_id = occupancy_gate.currentDeclaredIdentity(fixture.occupancy);
  require(occupancy_id.ok(), "occupancy declared identity failed: " + occupancy_id.reason);
  require(occupancy_id.identity->artifact_sha256 == fixture.occupancy_sha,
          "occupancy gate returned wrong declared hash");

  const auto other = root.path() / "field" / "other.ot";
  writeFile(other, "other artifact\n");
  auto wrong_path = octomap_gate.currentDeclaredIdentity(other);
  require(!wrong_path.ok() && wrong_path.reason.find("not the declared active") != std::string::npos,
          "configured path mismatch was not rejected");

  writeFile(root.path() / "active_map.txt", "missing\n");
  auto missing_active = occupancy_gate.currentDeclaredIdentity(fixture.occupancy);
  require(!missing_active.ok(), "missing active map was not rejected");
}

void testDeclaredHashAndPathMetadataFailures() {
  TempMapRoot root;
  auto fixture = writeActiveFixture(root.path());
  MapStore store(MapStoreConfig{root.path()});

  writeFile(fixture.map_dir / "metadata.json",
            "{\"frame_id\":\"map\",\"artifacts\":{\"octomap\":{\"path\":\"octomap.ot\","
            "\"sha256\":\"too-short\"}}}");
  auto bad_hash = store.ReadDeclaredArtifactIdentity("field", ArtifactType::kOctomap3D, "map");
  require(!bad_hash.ok() && bad_hash.reason.find("sha256") != std::string::npos,
          "bad declared hash was not rejected");

  writeFile(fixture.map_dir / "metadata.json",
            "{\"frame_id\":\"map\",\"artifacts\":{\"octomap\":{\"path\":\"missing.ot\","
            "\"sha256\":\"" + fixture.octomap_sha + "\"}}}");
  auto missing_path = store.ReadDeclaredArtifactIdentity("field", ArtifactType::kOctomap3D, "map");
  require(!missing_path.ok() && missing_path.reason.find("artifact is missing") != std::string::npos,
          "missing declared artifact path was not rejected");
}


void testDeclaredArtifactPathsStayInsideContentDirectory() {
  TempMapRoot root;
  auto fixture = writeActiveFixture(root.path());
  MapStore store(MapStoreConfig{root.path()});

  const auto nested = fixture.map_dir / "nested" / "octomap.ot";
  writeFile(nested, "nested octomap fixture\n");
  const std::string nested_sha = Sha256File(nested);
  writeFile(fixture.map_dir / "metadata.json",
            "{\"frame_id\":\"map\",\"artifacts\":{\"octomap\":{\"path\":\"nested/octomap.ot\","
            "\"sha256\":\"" + nested_sha + "\"}}}");
  auto nested_result = store.ReadDeclaredArtifactIdentity("field", ArtifactType::kOctomap3D, "map");
  require(nested_result.ok(), "legal nested artifact path was rejected: " + nested_result.reason);
  require(nested_result.identity->artifact_path == nested,
          "legal nested artifact path resolved incorrectly");

  writeFile(fixture.map_dir / "metadata.json",
            "{\"frame_id\":\"map\",\"artifacts\":{\"octomap\":{\"path\":\"" +
                fixture.octomap.generic_string() + "\",\"sha256\":\"" + fixture.octomap_sha +
                "\"}}}");
  auto absolute = store.ReadDeclaredArtifactIdentity("field", ArtifactType::kOctomap3D, "map");
  require(!absolute.ok() && absolute.reason.find("escapes map content") != std::string::npos,
          "absolute declared artifact path was not rejected");

  const auto outside = root.path() / "outside.ot";
  writeFile(outside, "outside octomap fixture\n");
  const std::string outside_sha = Sha256File(outside);
  writeFile(fixture.map_dir / "metadata.json",
            "{\"frame_id\":\"map\",\"artifacts\":{\"octomap\":{\"path\":\"../outside.ot\","
            "\"sha256\":\"" + outside_sha + "\"}}}");
  auto parent_escape = store.ReadDeclaredArtifactIdentity("field", ArtifactType::kOctomap3D, "map");
  require(!parent_escape.ok() &&
              parent_escape.reason.find("escapes map content") != std::string::npos,
          "parent-directory declared artifact path was not rejected");
}
}  // namespace
int main() {
  try {
    testStoreReadsDeclaredIdentityWithoutArtifactBytes();
    testStoreVersionedContentAndFrameMismatch();
    testGatesRequireActivePointerAndConfiguredPathMatch();
    testDeclaredHashAndPathMetadataFailures();
    testDeclaredArtifactPathsStayInsideContentDirectory();
  } catch (const std::exception &error) {
    std::fprintf(stderr, "test_declared_map_identity: FAIL: %s\n", error.what());
    return 1;
  }
  return 0;
}

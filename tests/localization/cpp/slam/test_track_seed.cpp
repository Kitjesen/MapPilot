// Exercise the actual saved-seed/request boundary without starting DDS or SLAM.
#define main lingtu_slam_runtime_entry
#include "cyclone_runtime.cpp"
#undef main

namespace {

void require(bool condition, const char* reason) {
  if (!condition) throw std::runtime_error(reason);
}

void requirePose(const std::optional<Pose3d>& actual, const Pose3d& expected,
                 const char* reason) {
  require(actual.has_value(), reason);
  const double actual_values[] = {actual->x, actual->y, actual->z, actual->qx,
                                  actual->qy, actual->qz, actual->qw};
  const double expected_values[] = {expected.x, expected.y, expected.z, expected.qx,
                                    expected.qy, expected.qz, expected.qw};
  for (int i = 0; i < 7; ++i) {
    require(std::isfinite(actual_values[i]) &&
                std::abs(actual_values[i] - expected_values[i]) < 1e-6, reason);
  }
}

void testSeedSelection(const std::filesystem::path& file) {
  const std::string map_a = "/var/lib/lingtu/maps/a/map.pcd";
  const std::string map_b = "/var/lib/lingtu/maps/b/map.pcd";
  // Preserve a complete map-body pose, including roll/pitch. It is only an
  // estimate for the matcher; seed selection must not award alignment success.
  const Pose3d saved{-2.662586, 0.561693, -0.260770,
                     -0.846364, 0.092310, -0.042798, 0.522796};
  const Pose3d explicit_pose{2.0, 3.0, 0.1, 0.0, 0.0, 0.6, 0.8};
  saveTrackSeed(file.string(), map_a, saved);
  const auto startup_seed = loadTrackSeed(file.string(), map_a);
  requirePose(startup_seed, saved, "startup must load the full same-map seed");

  // This is the real ProductControl sequence: startup loads the seed, then
  // track-against-map arrives with neither a map path nor an initial pose.
  requirePose(trackSeedForRequest(std::nullopt, startup_seed, map_a, "", file.string()),
              saved, "Product start discarded the persisted full-pose seed");
  requirePose(trackSeedForRequest(std::nullopt, startup_seed, map_a, map_a, ""),
              saved, "same-map request must preserve an already loaded seed");
  requirePose(trackSeedForRequest(std::nullopt, std::nullopt, map_a, "", file.string()),
              saved, "same-map request must reload its persisted seed when needed");
  requirePose(trackSeedForRequest(explicit_pose, startup_seed, map_a, "", file.string()),
              explicit_pose, "explicit request pose must override the saved seed");
  requirePose(trackSeedForRequest(explicit_pose, startup_seed, map_a, map_b, file.string()),
              explicit_pose, "explicit pose belongs to the requested map");
  require(!trackSeedForRequest(std::nullopt, startup_seed, map_a, map_b, file.string()),
          "a map switch reused another map's pending or persisted seed");
  require(!trackSeedForRequest(std::nullopt, std::nullopt, map_a, "", ""),
          "a missing seed must remain unseeded, never identity");

  saveTrackSeed(file.string(), map_b, explicit_pose);
  requirePose(trackSeedForRequest(std::nullopt, startup_seed, map_a, map_b, file.string()),
              explicit_pose, "a map switch must load only the target map's saved seed");
  require(!loadTrackSeed(file.string(), map_a), "mismatched map tag accepted");
  require(!loadTrackSeed(file.string(), ""), "a seed without an active map was accepted");
}

void testIncompleteSeedRejected(const std::filesystem::path& file) {
  const std::string pose = "\"pose\":{\"x\":1,\"y\":2,\"z\":3,";
  const std::string map = "\"map_path\":\"/maps/a/map.pcd\",";
  const std::string invalid[] = {
      "{" + pose + "\"qx\":0,\"qy\":0,\"qz\":0,\"qw\":1}}",
      "{" + map + pose + "\"qx\":0,\"qy\":0,\"qz\":0}}",
      "{" + map + pose + "\"qx\":0,\"qy\":0,\"qz\":0,\"qw\":0}}",
      "{" + map + pose + "\"qx\":0,\"qy\":0,\"qz\":0,\"qw\":1e999}}",
  };
  for (const auto& json : invalid) {
    { std::ofstream out(file); out << json; }
    require(!loadTrackSeed(file.string(), "/maps/a/map.pcd"),
            "untagged or incomplete seed must not become an identity estimate");
  }
}

}  // namespace

int main() {
  const auto file = std::filesystem::temp_directory_path() / "lingtu_track_seed_regression.json";
  try {
    testSeedSelection(file);
    testIncompleteSeedRejected(file);
    std::filesystem::remove(file);
    std::puts("track seed: Product restart/full quaternion/map switch/explicit override PASS");
    return 0;
  } catch (const std::exception& error) {
    std::filesystem::remove(file);
    std::fprintf(stderr, "track seed regression failed: %s\n", error.what());
    return 1;
  }
}

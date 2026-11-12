#include "lingtu/maps/save.hpp"
#include "lingtu/maps/io/json.hpp"
#include "lingtu/maps/io/pcd.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace lingtu::maps;

namespace {
void Require(bool value, const std::string& message) {
  if (!value) throw std::runtime_error(message);
}

std::string Read(const fs::path& path) {
  std::ifstream input(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

// Synthetic closed trajectory from the optimizer's constraint-assembly fixture.
// This is a native save/reload test, not a SLAM or factory qualification run.
void WriteCloud(const fs::path& path, double body_x, float phase) {
  std::ofstream out(path);
  out << "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
         "WIDTH 1875\nHEIGHT 1\nPOINTS 1875\nDATA ascii\n";
  for (int a = -12; a <= 12; ++a) {
    for (int b = -12; b <= 12; ++b) {
      const float u = a * 0.20F, v = b * 0.20F;
      const float noise = 0.006F * std::sin(0.7F * u + 0.3F * v + phase);
      out << u - body_x << ' ' << v << ' ' << noise << '\n';
      out << u - body_x << ' ' << -2.2F + noise << ' ' << v << '\n';
      out << 2.4F + noise - body_x << ' ' << u << ' ' << v << '\n';
    }
  }
}

MapSnapshot WriteSnapshot(const fs::path& source) {
  fs::create_directories(source / "patches");
  std::vector<double> positions;
  for (int x = 0; x <= 12; ++x) positions.push_back(x);
  for (int x = 11; x >= 0; --x) positions.push_back(x);
  std::ofstream poses(source / "poses.txt");
  for (std::size_t i = 0; i < positions.size(); ++i) {
    const auto name = std::to_string(i) + ".pcd";
    poses << name << ' ' << positions[i] + 0.005 * i << " 0 0 1 0 0 0\n";
    WriteCloud(source / "patches" / name, positions[i], static_cast<float>(i) * 0.37F);
  }
  WriteCloud(source / "map.pcd", 0.0, 0.0F);
  std::ofstream(source / "patch_bundle.manifest")
      << "LINGTU_PATCH_BUNDLE_V1\ncomplete 1\ndropped_count 0\nfirst_sequence 0\n"
         "last_sequence 24\npatch_count 25\n";
  std::ofstream(source / "scan_origin.txt") << "lidar_origin_in_patch 0 0 0.12\n";
  MapSnapshot snapshot;
  snapshot.snapshot_id = "synthetic-closed-trajectory";
  snapshot.source_dir = source;
  snapshot.frame_id = "map";
  snapshot.captured_at_ns = 1;
  // The atomic observation receipt is distinct from the patch history range.
  snapshot.first_sequence = 25;
  snapshot.last_sequence = 25;
  snapshot.slam_boot_id = "native-test-not-slam";
  snapshot.product_session_id = "native-test-not-product";
  snapshot.reset_epoch = 1;
  snapshot.observation_sequence = 25;
  snapshot.source_point_count = 1875;
  return snapshot;
}
}  // namespace

int main(int argc, char** argv) {
  fs::path root;
  try {
    Require(argc == 2 || argc == 3, "usage: real_pgo_save_reload_test PGO_EXE [NEW_EVIDENCE_DIR]");
    const auto pgo = fs::absolute(argv[1]);
    Require(fs::is_regular_file(pgo), "current PGO executable is missing");
    root = argc == 3 ? fs::absolute(argv[2]) : fs::temp_directory_path() /
        ("lingtu-real-pgo-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    Require(!fs::exists(root), "evidence directory must be new");
    const auto snapshot = WriteSnapshot(root / "snapshot");
    const std::string online_report =
        R"({"schema":"lingtu.map_optimization.v1","backend":"lio_sam_isam2","success":true,"performed":true,"pose_count":25,"loop_count":1})";
    std::ofstream(snapshot.source_dir / "map_optimization.json") << online_report;
    const MapStoreConfig config{root / "maps"};
    SaveMapRequest request;
    request.request_id = "native-real-pgo-save";
    request.map_id = "synthetic_closed_loop";
    request.product_session_id = snapshot.product_session_id;
    request.pgo.executable = pgo.string();
    request.pgo.timeout_sec = 90.0;
    request.source.dynamic_filter_enabled = false;  // Static fixture; prune is a separate gate.
    request.octomap.resolution = 0.20;
    std::int64_t saved_epoch = 0;
    std::string saved_cloud;
    {
      MapStore store(config);
      SaveMapEngine engine(store);
      Require(engine.Begin(request).accepted, "save request rejected");
      const auto provided = engine.ProvideSnapshot(request.request_id, snapshot);
      Require(provided.accepted, "snapshot rejected: " + provided.reason_code + ": " + provided.status.message);
      const auto result = engine.Wait(request.request_id, std::chrono::seconds(120));
      Require(result.has_value(), "save result missing");
      std::ofstream(root / "save-result.json") << SaveMapStatusJson(*result);
      Require(result->state == SaveJobState::kSucceeded,
              "real PGO save failed: " + result->reason_code + ": " + result->message);
      Require(Read(result->map_dir / "map_optimization.json") == online_report,
              "normal online save did not preserve its optimization report");
      const auto initial_epoch = store.ContentEpoch(request.map_id);
      const auto started = engine.Reoptimize(request.request_id, "native-real-reoptimize");
      Require(started.accepted && started.status.source_job_id == request.request_id,
              "reoptimization request rejected");
      const auto optimized = engine.Wait(started.status.job_id, std::chrono::seconds(120));
      Require(optimized && optimized->state == SaveJobState::kSucceeded,
              "real reoptimization failed: " + (optimized ? optimized->message : "missing result"));
      std::ofstream(root / "reoptimize-result.json") << SaveMapStatusJson(*optimized);
      Require(Read(result->capture_dir / "map_optimization.json") == online_report &&
                  Read(optimized->capture_dir / "map_optimization.json") == online_report,
              "reoptimization changed the archived online evidence");
      Require(store.ContentEpoch(request.map_id) > initial_epoch,
              "reoptimization did not commit a new map identity");
      const auto report = Read(optimized->map_dir / "map_optimization.json");
      Require(JsonObjectBoolAtPath(report, {"performed"}) == true, "PGO was skipped");
      Require(JsonObjectStringAtPath(report, {"code"}) == "optimized", "PGO did not optimize");
      Require(store.CheckMapActivation(request.map_id).ok, "saved navigation artifacts invalid");
      Require(store.ActiveMapId().empty(), "saving unexpectedly activated a map");
      saved_epoch = store.ContentEpoch(request.map_id);
      saved_cloud = Read(result->map_dir / "map.pcd");
    }
    {
      MapStore reloaded(config);
      SaveMapEngine recovered(reloaded);
      const auto result = recovered.GetStatus(request.request_id);
      Require(result && result->state == SaveJobState::kSucceeded, "saved operation did not reload");
      Require(reloaded.ContentEpoch(request.map_id) == saved_epoch, "map identity changed on reload");
      Require(reloaded.CheckMapActivation(request.map_id).ok, "reloaded map payload validation failed");
      const auto cloud = LoadPcdXyz(reloaded.MapPath(request.map_id) / "map.pcd");
      Require(cloud.ok && !cloud.points.empty(), "reloaded cloud invalid or empty");
      Require(reloaded.SetActiveMap(request.map_id, true).ok, "strict test-store activation failed");
      Require(reloaded.ActiveMapId() == request.map_id, "wrong map activated");
      Require(reloaded.ClearActiveMap().ok, "test-store deactivation failed");
      const auto optimized = recovered.GetStatus("native-real-reoptimize");
      Require(optimized && optimized->state == SaveJobState::kSucceeded &&
                  optimized->source_job_id == request.request_id,
              "reoptimization task identity did not reload");
      // A real subprocess launch failure must not replace the last good map.
      fs::remove(snapshot.source_dir / "map_optimization.json");
      request.request_id = "native-real-pgo-failure";
      request.pgo.executable = (root / "missing-pgo.exe").string();
      Require(recovered.Begin(request).accepted, "failure test request rejected");
      Require(recovered.ProvideSnapshot(request.request_id, snapshot).accepted, "failure snapshot rejected");
      const auto failure = recovered.Wait(request.request_id, std::chrono::seconds(30));
      Require(failure && failure->state == SaveJobState::kFailed, "missing PGO was reported as success");
      std::ofstream(root / "failure-result.json") << SaveMapStatusJson(*failure);
      Require(reloaded.ContentEpoch(request.map_id) == saved_epoch, "failed save changed map identity");
      Require(Read(reloaded.MapPath(request.map_id) / "map.pcd") == saved_cloud,
              "failed save replaced the last good map");
    }
    std::ofstream(root / "verification.json")
        << "{\"scope\":\"synthetic_native_save_reload\",\"optimized\":true,\"reoptimized\":true,\"reloaded\":true,"
           "\"strict_activation\":true,\"failed_replacement_preserved\":true,\"product_run\":false}\n";
    std::cout << "real PGO save/reload passed; evidence: " << root << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << "; evidence: " << root << '\n';
    return 1;
  }
}

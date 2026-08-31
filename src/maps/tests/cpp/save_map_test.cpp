#include "lingtu/maps/save.hpp"
#include "lingtu/maps/json.hpp"

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

using lingtu::maps::MapSnapshot;
using lingtu::maps::MapState;
using lingtu::maps::MapStore;
using lingtu::maps::MapStoreConfig;
using lingtu::maps::SaveJobState;
using lingtu::maps::SaveMapEngine;
using lingtu::maps::SaveMapHooks;
using lingtu::maps::SaveMapRequest;
using lingtu::maps::SaveMapResult;
using lingtu::maps::SavePhase;

[[noreturn]] void Fail(const std::string& message) {
  std::cerr << message << "\n";
  std::exit(1);
}

void Require(bool condition, const std::string& message) {
  if (!condition) {
    Fail(message);
  }
}

std::filesystem::path TempRoot() {
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  auto root = std::filesystem::temp_directory_path() /
      ("lingtu_maps_save_map_test_" + std::to_string(stamp));
  std::filesystem::remove_all(root);
  std::filesystem::create_directories(root);
  return root;
}

void WriteAsciiPcd(const std::filesystem::path& path, double offset = 0.0) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream file(path, std::ios::binary);
  file
      << "VERSION 0.7\n"
      << "FIELDS x y z\n"
      << "SIZE 4 4 4\n"
      << "TYPE F F F\n"
      << "COUNT 1 1 1\n"
      << "WIDTH 4\n"
      << "HEIGHT 1\n"
      << "POINTS 4\n"
      << "DATA ascii\n"
      << offset << " 0 0\n"
      << offset + 0.5 << " 0 0\n"
      << offset << " 0.5 0\n"
      << offset + 0.5 << " 0.5 0.2\n";
}

std::string ReadFile(const std::filesystem::path& path) {
  std::ifstream file(path, std::ios::binary);
  return std::string(
      std::istreambuf_iterator<char>(file),
      std::istreambuf_iterator<char>());
}

void ReplaceStateValue(
    const std::filesystem::path& path,
    const std::string& key,
    const std::string& value) {
  std::istringstream input(ReadFile(path));
  std::ostringstream output;
  std::string line;
  const std::string prefix = key + "=";
  bool replaced = false;
  while (std::getline(input, line)) {
    if (line.rfind(prefix, 0) == 0U) {
      output << prefix << value << '\n';
      replaced = true;
    } else {
      output << line << '\n';
    }
  }
  Require(replaced, "missing persisted SaveMap field: " + key);
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  file << output.str();
}

std::string QuoteCommandPath(const std::filesystem::path& path) {
#if defined(_WIN32)
  return "\"" + path.string() + "\"";
#else
  std::string out = "'";
  for (const char ch : path.string()) {
    out += ch == '\'' ? "'\\''" : std::string(1, ch);
  }
  return out + "'";
#endif
}

std::string WriteFakeConverter(const std::filesystem::path& root, bool slow = false) {
#if defined(_WIN32)
  const auto script = root / (slow ? "slow_converter.cmd" : "converter.cmd");
  std::ofstream file(script, std::ios::binary);
    file << "@echo off\r\n";
  if (slow) {
    file << "ping -n 20 127.0.0.1 > nul\r\n";
  }
  file << "(\r\n"
       << "echo # Octomap OcTree binary file\r\n"
       << "echo id OcTree\r\n"
       << "echo size 1\r\n"
       << "echo res 0.1\r\n"
       << "echo data\r\n"
       << "echo x\r\n"
       << ") > \"%~2\"\r\n";
#else
  const auto script = root / (slow ? "slow_converter.sh" : "converter.sh");
  std::ofstream file(script, std::ios::binary);
  file << "#!/bin/sh\n";
  if (slow) {
    file << "sleep 20\n";
  }
  file << "printf '# Octomap OcTree binary file\\nid OcTree\\nsize 1\\nres 0.1\\ndata\\nx' > \"$2\"\n";
  file.close();
  std::filesystem::permissions(
      script,
      std::filesystem::perms::owner_exec |
          std::filesystem::perms::owner_read |
          std::filesystem::perms::owner_write,
      std::filesystem::perm_options::add);
#endif
  return QuoteCommandPath(script) + " {input} {output}";
}

void WriteCompletePatchBundle(
    const std::filesystem::path& root,
    double offset = 0.0,
    std::size_t patch_count = 1U) {
  WriteAsciiPcd(root / "map.pcd", offset);
  std::ofstream poses(root / "poses.txt", std::ios::binary);
  for (std::size_t index = 1U; index <= patch_count; ++index) {
    const std::string name = "00000" + std::to_string(index) + ".pcd";
    WriteAsciiPcd(root / "patches" / name, offset + static_cast<double>(index - 1U));
    poses << name << ' ' << (index - 1U) << " 0 0 1 0 0 0\n";
  }
  poses.close();
  std::ofstream(root / "patch_bundle.manifest", std::ios::binary)
      << "LINGTU_PATCH_BUNDLE_V1\n"
      << "complete 1\n"
      << "dropped_count 0\n"
      << "first_sequence 0\n"
      << "last_sequence " << (patch_count - 1U) << "\n"
      << "patch_count " << patch_count << "\n";
}

std::filesystem::path WriteFakePgo(const std::filesystem::path& root, bool fail = false) {
#if defined(_WIN32)
  const auto script = root / (fail ? "pgo_fail.cmd" : "pgo.cmd");
  std::ofstream file(script, std::ios::binary);
  file << "@echo off\r\n";
  if (fail) {
    file << "exit /b 7\r\n";
  } else {
    file << "if not \"%~5\"==\"--constraints\" if not \"%~5\"==\"--auto-constraints\" exit /b 18\r\n"
         << "if exist \"%~2\\force_skip\" (echo {\"ok\":true,\"performed\":false,\"code\":\"no_verified_loops\",\"message\":\"no verified loop constraints\",\"pose_count\":2,\"factor_count\":1,\"sequential_count\":1,\"loop_count\":0} & exit /b 0)\r\n"
         << "if exist \"%~2\\auto_skip\" (\r\n"
         << "  if not \"%~5\"==\"--auto-constraints\" exit /b 19\r\n"
         << "  echo {\"ok\":true,\"performed\":false,\"code\":\"no_verified_loops\",\"message\":\"no verified loop constraints\",\"pose_count\":2,\"factor_count\":1,\"sequential_count\":1,\"loop_count\":0}\r\n"
         << "  exit /b 0\r\n"
         << ")\r\n"
         << "if \"%~5\"==\"--constraints\" findstr /c:\"RIGHT_TANGENT omega_x omega_y omega_z upsilon_x upsilon_y upsilon_z\" \"%~6\" > nul || (echo {\"ok\":false,\"performed\":false,\"code\":\"pgo_constraints_invalid\",\"message\":\"invalid constraints\",\"pose_count\":0,\"factor_count\":0,\"sequential_count\":0,\"loop_count\":0} & exit /b 7)\r\n"
         << "mkdir \"%~4\"\r\n"
         << "mkdir \"%~4\\patches\"\r\n"
         << "copy /y \"%~2\\map.pcd\" \"%~4\\map.pcd\" > nul\r\n"
         << "copy /y \"%~2\\map.pcd\" \"%~4\\patches\\000001.pcd\" > nul\r\n"
         << "copy /y \"%~2\\map.pcd\" \"%~4\\patches\\000002.pcd\" > nul\r\n"
         << "(echo LINGTU_PATCH_BUNDLE_V1&echo complete 1&echo dropped_count 0&echo first_sequence 0&echo last_sequence 1&echo patch_count 2)>\"%~4\\patch_bundle.manifest\"\r\n"
         << "(echo 000001.pcd 0 0 0 1 0 0 0&echo 000002.pcd 1 0 0 1 0 0 0)>\"%~4\\poses.txt\"\r\n"
         << "echo {\"schema\":\"lingtu.map_optimization.v1\",\"success\":true,\"code\":\"optimized\",\"converged\":true,\"pose_count\":2,\"patch_count\":2,\"factor_count\":2}>\"%~4\\map_optimization.json\"\r\n"
         << "if exist \"%~2\\bad_report\" echo {\"schema\":\"lingtu.map_optimization.v1\",\"success\":false,\"code\":\"optimized\",\"converged\":true,\"pose_count\":2,\"patch_count\":2,\"factor_count\":2}>\"%~4\\map_optimization.json\"\r\n"
         << "if exist \"%~2\\bad_counts\" echo {\"schema\":\"lingtu.map_optimization.v1\",\"success\":true,\"code\":\"optimized\",\"converged\":true,\"pose_count\":2,\"patch_count\":2,\"factor_count\":1}>\"%~4\\map_optimization.json\"\r\n"
         << "if exist \"%~2\\bad_manifest\" (echo LINGTU_PATCH_BUNDLE_V1&echo complete 1&echo dropped_count 0&echo first_sequence 0&echo last_sequence 2&echo patch_count 3)>\"%~4\\patch_bundle.manifest\"\r\n"
         << "if exist \"%~2\\bad_pose_set\" echo other.pcd 0 0 0 1 0 0 0 >\"%~4\\poses.txt\"\r\n"
         << "if exist \"%~2\\performed_without_loop\" (echo {\"ok\":true,\"performed\":true,\"code\":\"optimized\",\"message\":\"optimized\",\"pose_count\":2,\"factor_count\":1,\"sequential_count\":1,\"loop_count\":0} & exit /b 0)\r\n"
         << "echo {\"ok\":true,\"performed\":true,\"code\":\"optimized\",\"message\":\"optimized\",\"pose_count\":2,\"factor_count\":2,\"sequential_count\":1,\"loop_count\":1}\r\n";
  }
#else
  const auto script = root / (fail ? "pgo_fail.sh" : "pgo.sh");
  std::ofstream file(script, std::ios::binary);
    file << "#!/bin/sh\n";
  if (fail) {
    file << "exit 7\n";
  } else {
    file << "[ \"$5\" = \"--constraints\" ] || [ \"$5\" = \"--auto-constraints\" ] || exit 18\n"
         << "if [ -f \"$2/force_skip\" ]; then\n"
         << "  printf '{\"ok\":true,\"performed\":false,\"code\":\"no_verified_loops\",\"message\":\"no verified loop constraints\",\"pose_count\":2,\"factor_count\":1,\"sequential_count\":1,\"loop_count\":0}\\n'\n"
         << "  exit 0\n"
         << "fi\n"
         << "if [ -f \"$2/auto_skip\" ]; then\n"
         << "  [ \"$5\" = \"--auto-constraints\" ] || exit 19\n"
         << "  printf '{\"ok\":true,\"performed\":false,\"code\":\"no_verified_loops\",\"message\":\"no verified loop constraints\",\"pose_count\":2,\"factor_count\":1,\"sequential_count\":1,\"loop_count\":0}\\n'\n"
         << "  exit 0\n"
         << "fi\n"
         << "if [ \"$5\" = \"--constraints\" ] && ! grep -q 'RIGHT_TANGENT omega_x omega_y omega_z upsilon_x upsilon_y upsilon_z' \"$6\"; then\n"
         << "  printf '{\"ok\":false,\"performed\":false,\"code\":\"pgo_constraints_invalid\",\"message\":\"invalid constraints\",\"pose_count\":0,\"factor_count\":0,\"sequential_count\":0,\"loop_count\":0}\\n'\n"
         << "  exit 7\n"
         << "fi\n"
         << "mkdir -p \"$4/patches\"\n"
         << "cp \"$2/map.pcd\" \"$4/map.pcd\"\n"
         << "cp \"$2/map.pcd\" \"$4/patches/000001.pcd\"\n"
         << "cp \"$2/map.pcd\" \"$4/patches/000002.pcd\"\n"
         << "printf 'LINGTU_PATCH_BUNDLE_V1\\ncomplete 1\\ndropped_count 0\\nfirst_sequence 0\\nlast_sequence 1\\npatch_count 2\\n' > \"$4/patch_bundle.manifest\"\n"
         << "printf '000001.pcd 0 0 0 1 0 0 0\\n000002.pcd 1 0 0 1 0 0 0\\n' > \"$4/poses.txt\"\n"
         << "printf '{\"schema\":\"lingtu.map_optimization.v1\",\"success\":true,\"code\":\"optimized\",\"converged\":true,\"pose_count\":2,\"patch_count\":2,\"factor_count\":2}\\n' > \"$4/map_optimization.json\"\n"
         << "[ ! -f \"$2/bad_report\" ] || printf '{\"schema\":\"lingtu.map_optimization.v1\",\"success\":false,\"code\":\"optimized\",\"converged\":true,\"pose_count\":2,\"patch_count\":2,\"factor_count\":2}\\n' > \"$4/map_optimization.json\"\n"
         << "[ ! -f \"$2/bad_counts\" ] || printf '{\"schema\":\"lingtu.map_optimization.v1\",\"success\":true,\"code\":\"optimized\",\"converged\":true,\"pose_count\":2,\"patch_count\":2,\"factor_count\":1}\\n' > \"$4/map_optimization.json\"\n"
         << "[ ! -f \"$2/bad_manifest\" ] || printf 'LINGTU_PATCH_BUNDLE_V1\\ncomplete 1\\ndropped_count 0\\nfirst_sequence 0\\nlast_sequence 2\\npatch_count 3\\n' > \"$4/patch_bundle.manifest\"\n"
         << "[ ! -f \"$2/bad_pose_set\" ] || printf 'other.pcd 0 0 0 1 0 0 0\\n' > \"$4/poses.txt\"\n"
         << "if [ -f \"$2/performed_without_loop\" ]; then\n"
         << "  printf '{\"ok\":true,\"performed\":true,\"code\":\"optimized\",\"message\":\"optimized\",\"pose_count\":2,\"factor_count\":1,\"sequential_count\":1,\"loop_count\":0}\\n'\n"
         << "  exit 0\n"
         << "fi\n"
         << "printf '{\"ok\":true,\"performed\":true,\"code\":\"optimized\",\"message\":\"optimized\",\"pose_count\":2,\"factor_count\":2,\"sequential_count\":1,\"loop_count\":1}\\n'\n";
  }
  file.close();
  std::filesystem::permissions(
      script,
      std::filesystem::perms::owner_exec |
          std::filesystem::perms::owner_read |
          std::filesystem::perms::owner_write,
      std::filesystem::perm_options::add);
#endif
  return script;
}

std::filesystem::path WriteSlowPgo(const std::filesystem::path& root) {
#if defined(_WIN32)
  const auto script = root / "pgo_slow.cmd";
  std::ofstream file(script, std::ios::binary);
  file << "@echo off\r\n"
       << "ping -n 2 127.0.0.1 > nul\r\n";
#else
  const auto script = root / "pgo_slow.sh";
  std::ofstream file(script, std::ios::binary);
  file << "#!/bin/sh\n"
       << "sleep 1\n";
  file.close();
  std::filesystem::permissions(
      script,
      std::filesystem::perms::owner_exec |
          std::filesystem::perms::owner_read |
          std::filesystem::perms::owner_write,
      std::filesystem::perm_options::add);
#endif
  return script;
}

void WriteValidConstraints(const std::filesystem::path& path) {
  std::ofstream file(path, std::ios::binary);
  file << "LINGTU_PGO_CONSTRAINTS_V1\n"
       << "T_from_to tx ty tz qw qx qy qz\n"
       << "RIGHT_TANGENT omega_x omega_y omega_z upsilon_x upsilon_y upsilon_z\n"
       << "UPPER_TRIANGLE row_major 21\n"
       << "0 1 1 0 0 1 0 0 0 "
       << "1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1\n";
}

SaveMapRequest Request(
    const std::string& request_id,
    const std::string& map_id,
    const std::string& converter) {
  SaveMapRequest request;
  request.request_id = request_id;
  request.map_id = map_id;
  request.source.dynamic_filter_enabled = false;
  request.octomap.converter_command = converter;
  request.octomap.build_mode = "external_pcl_converter";
  request.octomap.timeout_sec = 30.0;
  request.require.semantic = false;
  return request;
}

void UseValidActivationOctomap(SaveMapRequest& request) {
#if defined(LINGTU_MAPS_HAS_OCTOMAP)
  request.octomap.build_mode = "native_octomap";
  request.octomap.converter_command.clear();
#else
  (void)request;
#endif
}

MapSnapshot Snapshot(const std::string& id, const std::filesystem::path& source) {
  MapSnapshot snapshot;
  snapshot.snapshot_id = id;
  snapshot.source_dir = source;
  snapshot.frame_id = "map";
  snapshot.captured_at_ns = 1;
  snapshot.first_sequence = 1U;
  snapshot.last_sequence = 1U;
  snapshot.slam_boot_id = "slam-test-boot";
  snapshot.product_session_id = "save-map-test-session";
  snapshot.reset_epoch = 1U;
  snapshot.observation_sequence = 1U;
  snapshot.source_point_count = 4U;
  snapshot.slam_healthy = true;
  return snapshot;
}

std::filesystem::path WorkMapDir(
    const MapStore& store,
    const std::string& job_id,
    const std::string& map_id) {
#if defined(_WIN32)
  return std::filesystem::temp_directory_path() / "lt_maps" / job_id / "s" / map_id;
#else
  return store.RootDir() / ".save_jobs" / job_id / "work" / "s" / map_id;
#endif
}

lingtu::maps::SaveMapStatus WaitTerminal(SaveMapEngine& engine, const std::string& id) {
  const auto status = engine.Wait(id, std::chrono::seconds(30));
  Require(status.has_value(), "SaveMap job disappeared: " + id);
  Require(
      status->state == SaveJobState::kSucceeded ||
          status->state == SaveJobState::kFailed ||
          status->state == SaveJobState::kCancelled,
      "SaveMap job did not reach terminal state: " + id);
  return *status;
}

void WaitPhase(SaveMapEngine& engine, const std::string& id, SavePhase phase) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (std::chrono::steady_clock::now() < deadline) {
    const auto status = engine.GetStatus(id);
    if (status.has_value() && status->phase == phase) {
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  Fail("SaveMap job did not enter expected phase");
}

}  // namespace

int main() {
  const auto root = TempRoot();
  const auto converter = WriteFakeConverter(root);
  const auto source_v1 = root / "snapshots" / "v1";
  WriteAsciiPcd(source_v1 / "map.pcd", 0.0);
  const auto invalid_source = root / "snapshots" / "invalid";
  std::filesystem::create_directories(invalid_source);
  {
    std::ofstream file(invalid_source / "map.pcd", std::ios::binary);
    file << "not a PCD\n";
  }
  const auto empty_source = root / "snapshots" / "empty";
  std::filesystem::create_directories(empty_source);
  std::ofstream(empty_source / "map.pcd", std::ios::binary);

  MapStore store(MapStoreConfig{root / "maps"});
  {
    SaveMapEngine owner(store);
    bool rejected = false;
    try {
      SaveMapEngine duplicate(store);
    } catch (const std::exception&) {
      rejected = true;
    }
    Require(rejected, "map root accepted two concurrent SaveMap engines");
  }
  SaveMapRequest persisted_pgo = Request("persisted_pgo", "persisted_pgo_map", converter);
  persisted_pgo.pgo.executable = "custom-lt-pgo";
  persisted_pgo.pgo.constraints_file = "custom.constraints";
  persisted_pgo.pgo.timeout_sec = 12.5;
  {
    SaveMapEngine engine(store);
    Require(engine.Begin(persisted_pgo).accepted, "persisted PGO request was rejected");
  }
  {
    SaveMapEngine recovered(store);
    const auto replay = recovered.Begin(persisted_pgo);
    Require(replay.accepted && replay.replayed, "recovered PGO request did not replay");
    auto changed = persisted_pgo;
    changed.pgo.timeout_sec = 13.0;
    const auto conflict = recovered.Begin(changed);
    Require(
        !conflict.accepted && conflict.reason_code == "idempotency_conflict",
        "recovered PGO fields were not part of the request identity");
    Require(recovered.Cancel("persisted_pgo").accepted, "failed to cancel persisted PGO fixture");
  }
  ReplaceStateValue(
      store.RootDir() / ".save_jobs" / "persisted_pgo" / "job.state", "schema", "3");
  {
    SaveMapEngine recovered(store);
    const auto status = recovered.GetStatus("persisted_pgo");
    Require(status.has_value() && status->state == SaveJobState::kCancelled,
            "terminal v3 SaveMap history was not readable");
  }
  {
    SaveMapEngine engine(store);
    auto invalid_pgo_request = Request("invalid_pgo", "invalid_pgo", converter);
    invalid_pgo_request.pgo.constraints_file = "nested/pose_graph.constraints";
    bool invalid_pgo_rejected = false;
    try {
      (void)engine.Begin(invalid_pgo_request);
    } catch (const std::invalid_argument&) {
      invalid_pgo_rejected = true;
    }
    Require(invalid_pgo_rejected, "SaveMap accepted a non-basename PGO constraints file");

    auto unsafe_native_request = Request("unsafe_native", "unsafe_native_map", converter);
    unsafe_native_request.octomap.slam_source = "native_dds";
    unsafe_native_request.allow_unverified_snapshot = true;
    bool unsafe_native_rejected = false;
    try {
      (void)engine.Begin(unsafe_native_request);
    } catch (const std::invalid_argument&) {
      unsafe_native_rejected = true;
    }
    Require(
        unsafe_native_rejected,
        "native_dds SaveMap accepted the unverified snapshot compatibility mode");

    const auto strict_request = Request("missing_receipt", "receipt_gate", converter);
    Require(engine.Begin(strict_request).accepted, "strict SaveMap request was not accepted");
    auto unverified = Snapshot("missing_receipt_snapshot", source_v1);
    unverified.slam_boot_id.clear();
    unverified.product_session_id.clear();
    unverified.reset_epoch = 0U;
    unverified.observation_sequence = 0U;
    unverified.source_point_count = 0U;
    const auto rejected = engine.ProvideSnapshot("missing_receipt", unverified);
    Require(!rejected.accepted, "SaveMap accepted a snapshot without an atomic receipt");
    Require(
        rejected.reason_code == "snapshot_receipt_required",
        "missing snapshot receipt returned the wrong reason");

    auto minimum_request = Request("receipt_too_small", "receipt_too_small", converter);
    minimum_request.minimum_point_count = 5U;
    Require(engine.Begin(minimum_request).accepted, "minimum receipt request was not accepted");
    const auto minimum_rejected =
        engine.ProvideSnapshot("receipt_too_small", Snapshot("receipt_too_small", source_v1));
    Require(!minimum_rejected.accepted, "SaveMap ignored the receipt minimum point count");
    Require(
        minimum_rejected.reason_code == "snapshot_receipt_point_count_too_small",
        "small receipt returned the wrong reason");

    Require(
        engine.Begin(Request("empty_snapshot", "empty_snapshot", converter)).accepted,
        "empty snapshot request was not accepted");
    const auto empty_rejected =
        engine.ProvideSnapshot("empty_snapshot", Snapshot("empty_snapshot", empty_source));
    Require(!empty_rejected.accepted, "SaveMap accepted an empty map.pcd");
    Require(
        empty_rejected.reason_code == "snapshot_pcd_missing",
        "empty map.pcd returned the wrong reason");

    Require(
        engine.Begin(Request("invalid_snapshot", "invalid_snapshot", converter)).accepted,
        "invalid snapshot request was not accepted");
    const auto invalid_provided =
        engine.ProvideSnapshot("invalid_snapshot", Snapshot("invalid_snapshot", invalid_source));
    Require(invalid_provided.accepted, "ingress parsed a non-empty map.pcd");
    const auto invalid_status = WaitTerminal(engine, "invalid_snapshot");
    Require(invalid_status.state == SaveJobState::kFailed,
            "source processing accepted an invalid PCD");
    Require(invalid_status.reason_code == "source_pcd_unreadable",
            "invalid PCD did not fail in source processing");

    Require(
        engine.Begin(Request("receipt_count_differs", "receipt_count_differs", converter)).accepted,
        "receipt-count fixture was not accepted");
    auto differing_receipt = Snapshot("receipt_count_differs", source_v1);
    differing_receipt.source_point_count = 99U;
    Require(engine.ProvideSnapshot("receipt_count_differs", differing_receipt).accepted,
            "SaveMap rescanned map.pcd to compare receipt point count");
    Require(WaitTerminal(engine, "receipt_count_differs").state == SaveJobState::kSucceeded,
            "valid source with a sufficient receipt point count did not save");

    const auto request = Request("save_v1", "warehouse", converter);
    const auto begin = engine.Begin(request);
    Require(begin.accepted && !begin.replayed, "first SaveMap request was not accepted");
    Require(std::filesystem::is_directory(begin.status.capture_dir), "capture dir missing");
    const auto provided = engine.ProvideSnapshot("save_v1", Snapshot("snapshot_v1", source_v1));
    Require(provided.accepted, "snapshot was not accepted");
    const auto status = WaitTerminal(engine, "save_v1");
    Require(status.state == SaveJobState::kSucceeded, "SaveMap v1 failed: " + status.message);
    Require(!status.activation_requested, "non-activating SaveMap reported activation request");
    Require(!status.activation_succeeded, "non-activating SaveMap reported activation success");
    Require(status.map_dir == store.MapPath("warehouse"), "SaveMap status did not expose direct map dir");
    Require(std::filesystem::is_regular_file(status.map_dir / "map.pcd"), "map.pcd missing");
    const auto skip_report = ReadFile(status.map_dir / "map_optimization.json");
    Require(lingtu::maps::IsValidJsonObject(skip_report), "PGO skip report is not valid JSON");
    Require(
        lingtu::maps::JsonObjectStringAtPath(skip_report, {"schema"}) ==
                "lingtu.map_optimization.v1" &&
            lingtu::maps::JsonObjectBoolAtPath(skip_report, {"success"}) == true &&
            lingtu::maps::JsonObjectStringAtPath(skip_report, {"code"}) ==
                "patch_bundle_incomplete" &&
            lingtu::maps::JsonObjectBoolAtPath(skip_report, {"performed"}) == false &&
            lingtu::maps::JsonObjectNumberAtPath(skip_report, {"pose_count"}) == 0.0 &&
            lingtu::maps::JsonObjectNumberAtPath(skip_report, {"factor_count"}) == 0.0 &&
            lingtu::maps::JsonObjectNumberAtPath(skip_report, {"sequential_count"}) == 0.0 &&
            lingtu::maps::JsonObjectNumberAtPath(skip_report, {"loop_count"}) == 0.0,
        "SaveMap without a complete patch bundle did not publish the exact PGO skip contract");
    Require(std::filesystem::is_regular_file(status.map_dir / "octomap.ot"), "octomap missing");
    Require(std::filesystem::is_regular_file(status.map_dir / "occupancy.npz"), "occupancy missing");
    Require(std::filesystem::is_regular_file(status.map_dir / "esdf.npz"), "ESDF missing");
    Require(
        std::filesystem::is_regular_file(status.map_dir / "traversability.npz"),
        "traversability missing");
    Require(!std::filesystem::exists(status.map_dir / ".versions"),
            "SaveMap retained a version-history directory");
    Require(!std::filesystem::exists(status.map_dir / "current_version.txt"),
            "SaveMap retained a current-version pointer");
    Require(!std::filesystem::exists(status.map_dir / "save_manifest.json"),
            "SaveMap retained a redundant save manifest");
    Require(ReadFile(status.map_dir / "metadata.json").find("sha256") == std::string::npos,
            "saved map metadata still contains SHA fields");
    const auto journal = ReadFile(
        store.RootDir() / ".save_jobs" / "save_v1" / "events.jsonl");
    const auto state = ReadFile(
        store.RootDir() / ".save_jobs" / "save_v1" / "job.state");
    Require(state.find("fingerprint") == std::string::npos &&
                state.find("sha256") == std::string::npos,
            "SaveMap private state still persists hashes");
    Require(journal.find("MAP_COMMITTED") != std::string::npos,
            "SaveMap journal is missing MAP_COMMITTED");
    Require(journal.find("SUCCEEDED:DONE") != std::string::npos,
            "SaveMap journal is missing terminal success");
    Require(!std::filesystem::exists(status.capture_dir),
            "successful SaveMap retained its capture staging directory");

    const auto record = store.GetMapRecord("warehouse");
    Require(record.has_value(), "MapRecord missing after SaveMap");
    Require(record->content_epoch > 0, "existing map directory must expose a content epoch");
    Require(!record->artifacts.empty(), "MapRecord artifacts missing");
    for (const auto& artifact : record->artifacts) {
      Require(std::filesystem::path(artifact.uri).parent_path() == store.MapPath("warehouse"),
              "MapRecord artifact must live directly in the map directory");
      Require(std::filesystem::is_regular_file(artifact.uri),
              "MapRecord artifact must point to a committed file");
    }

    const auto replay = engine.Begin(request);
    Require(replay.accepted && replay.replayed, "identical request_id was not replayed");
    auto conflict_request = request;
    conflict_request.map_id = "different_map";
    const auto conflict = engine.Begin(conflict_request);
    Require(!conflict.accepted && conflict.reason_code == "idempotency_conflict",
            "request_id conflict was not rejected");

    auto session_request = Request("session_bound", "session_bound_map", converter);
    session_request.product_session_id = "product-11111111111111111111111111111111";
    const auto session_begin = engine.Begin(session_request);
    Require(session_begin.accepted && !session_begin.replayed,
            "session-bound SaveMap request was not accepted");
    Require(session_begin.status.product_session_id == session_request.product_session_id,
            "SaveMap status lost the request Product session");
    Require(
        engine.BeginJson(session_request).find(
            "\"product_session_id\":\"" + session_request.product_session_id + "\"") !=
            std::string::npos,
        "public SaveMap status did not expose the request Product session");
    auto foreign_session_request = session_request;
    foreign_session_request.product_session_id = "product-22222222222222222222222222222222";
    const auto session_conflict = engine.Begin(foreign_session_request);
    Require(
        !session_conflict.accepted &&
            session_conflict.reason_code == "idempotency_conflict",
        "request_id replay from another Product session was not rejected");
    auto foreign_snapshot = Snapshot("session_bound_snapshot", source_v1);
    foreign_snapshot.product_session_id = foreign_session_request.product_session_id;
    const auto foreign_snapshot_result =
        engine.ProvideSnapshot("session_bound", foreign_snapshot);
    Require(
        !foreign_snapshot_result.accepted &&
            foreign_snapshot_result.reason_code == "snapshot_product_session_mismatch",
        "SaveMap accepted a snapshot from another Product session");
    auto session_snapshot = Snapshot("session_bound_snapshot", source_v1);
    session_snapshot.product_session_id = session_request.product_session_id;
    Require(engine.ProvideSnapshot("session_bound", session_snapshot).accepted,
            "matching Product-session snapshot was rejected");
    Require(WaitTerminal(engine, "session_bound").state == SaveJobState::kSucceeded,
            "session-bound SaveMap did not complete");
  }

  {
    SaveMapEngine engine(store);
    auto request = Request("activate_success", "activated_map", converter);
    UseValidActivationOctomap(request);
    request.activate_on_success = true;
    Require(engine.Begin(request).accepted, "activation success request rejected");
    Require(
        engine.ProvideSnapshot(
            "activate_success", Snapshot("activate_success_snapshot", source_v1)).accepted,
        "activation success snapshot rejected");
    const auto status = WaitTerminal(engine, "activate_success");
    Require(
        status.state == SaveJobState::kSucceeded,
        "SaveMap activation under held map lock failed: " + status.message);
    Require(status.activation_requested, "activated SaveMap lost activation request evidence");
    Require(status.activation_succeeded, "activated SaveMap lost activation success evidence");
    Require(store.ActiveMapId() == "activated_map", "SaveMap did not activate committed map");
  }

  Require(
      store.SetActiveMap("warehouse", false).ok,
      "failed to switch away from activated map for recovery-history test");
  {
    SaveMapEngine recovered(store);
    const auto status = recovered.GetStatus("activate_success");
    Require(status.has_value(), "activated SaveMap status disappeared on restart");
    Require(status->state == SaveJobState::kSucceeded,
            "terminal activation success changed state on restart");
    Require(status->activation_succeeded,
            "restart rewrote historical activation success from current active-map state");
    Require(store.ActiveMapId() == "warehouse",
            "SaveMap recovery unexpectedly changed the current active map");
  }

  {
    SaveMapHooks artifact_hooks;
    artifact_hooks.before_phase = [&](const std::string& job_id, SavePhase phase) {
      if (phase != SavePhase::kVerify) return;
      if (job_id == "empty_required_artifact") {
        std::ofstream(WorkMapDir(store, job_id, "empty_required_map") / "occupancy.npz",
                      std::ios::binary | std::ios::trunc);
      } else if (job_id == "wrong_metadata_frame") {
        std::ofstream metadata(
            WorkMapDir(store, job_id, "wrong_metadata_map") / "metadata.json",
            std::ios::binary | std::ios::trunc);
        metadata << "{\"frame_id\":\"odom\"}\n";
      }
    };
    SaveMapEngine checked(store, artifact_hooks);
    for (const auto& fixture : std::vector<std::pair<std::string, std::string>>{
             {"empty_required_artifact", "empty_required_map"},
             {"wrong_metadata_frame", "wrong_metadata_map"}}) {
      const auto checked_request = Request(fixture.first, fixture.second, converter);
      Require(checked.Begin(checked_request).accepted, "artifact validation request rejected");
      Require(checked.ProvideSnapshot(
                  fixture.first, Snapshot(fixture.first + "_snapshot", source_v1)).accepted,
              "artifact validation snapshot rejected");
      const auto checked_status = WaitTerminal(checked, fixture.first);
      Require(checked_status.state == SaveJobState::kFailed,
              "SaveMap committed an empty artifact or invalid metadata frame");
      Require(!std::filesystem::exists(store.MapPath(fixture.second)),
              "failed artifact validation committed a map directory");
    }
  }

  {
    SaveMapEngine engine(store);
    Require(engine.Begin(Request(
                "new_map_interrupted", "new_map_interrupted", converter)).accepted,
            "new-map recovery fixture was rejected");
  }
  {
    const auto state =
        store.RootDir() / ".save_jobs" / "new_map_interrupted" / "job.state";
    ReplaceStateValue(state, "state", "RUNNING");
    ReplaceStateValue(state, "phase", "COMMIT");
    ReplaceStateValue(state, "completed_at_ns", "0");
    const auto stage = store.RootDir() / ".save-staging-new_map_interrupted";
    std::filesystem::create_directory(stage);
    std::ofstream(stage / "partial", std::ios::binary) << "partial";
    SaveMapEngine recovered(store);
    const auto status = recovered.GetStatus("new_map_interrupted");
    Require(status.has_value() && status->state == SaveJobState::kWaitingSnapshot,
            "interrupted new map did not return to snapshot wait");
    Require(!std::filesystem::exists(stage), "interrupted new-map staging was retained");
    Require(!std::filesystem::exists(store.MapPath("new_map_interrupted")),
            "interrupted new map recovery created an empty canonical map directory");
    Require(!std::filesystem::exists(
                store.RootDir() / ".save-backup-new_map_interrupted-new_map_interrupted"),
            "interrupted new map recovery created an empty backup marker");
  }

  {
    SaveMapEngine engine(store);
    auto request = Request("recover_activation_gap", "activation_gap_map", converter);
    UseValidActivationOctomap(request);
    request.activate_on_success = true;
    Require(engine.Begin(request).accepted, "activation gap request rejected");
    Require(
        engine.ProvideSnapshot(
            "recover_activation_gap",
            Snapshot("activation_gap_snapshot", source_v1)).accepted,
        "activation gap snapshot rejected");
    const auto status = WaitTerminal(engine, "recover_activation_gap");
    Require(status.state == SaveJobState::kSucceeded, "activation gap setup failed");
  }
  Require(
      store.SetActiveMap("warehouse", false).ok,
      "failed to simulate an activation gap after version commit");

  {
    const auto state =
        store.RootDir() / ".save_jobs" / "recover_activation_gap" / "job.state";
    ReplaceStateValue(state, "state", "RUNNING");
    ReplaceStateValue(state, "phase", "COMMIT");
    ReplaceStateValue(state, "completed_at_ns", "0");
    ReplaceStateValue(state, "activation_succeeded", "0");
    SaveMapEngine recovered(store);
    const auto status = WaitTerminal(recovered, "recover_activation_gap");
    Require(status.state == SaveJobState::kFailed,
            "recovery reported committed-but-not-activated request as success");
    Require(status.reason_code == "activation_failed_after_commit",
            "recovery lost committed activation failure reason");
    Require(status.activation_requested && !status.activation_succeeded,
            "recovery misreported activation evidence");
  }

  {
    const auto state = store.RootDir() / ".save_jobs" / "save_v1" / "job.state";
    ReplaceStateValue(state, "state", "RUNNING");
    ReplaceStateValue(state, "phase", "COMMIT");
    ReplaceStateValue(state, "completed_at_ns", "0");
    ReplaceStateValue(state, "map_committed", "0");
    const auto backup = store.RootDir() / ".save-backup-warehouse-save_v1";
    std::filesystem::create_directory(backup);
    SaveMapEngine recovered(store);
    const auto status = WaitTerminal(recovered, "save_v1");
    Require(status.state == SaveJobState::kSucceeded,
            "completed directory swap recovery did not restore success");
    Require(status.recovered, "directory swap recovery lacks recovery evidence");
    Require(!std::filesystem::exists(backup), "directory swap recovery retained backup");
  }

  const std::string v1_content = ReadFile(store.ContentPath("warehouse") / "map.pcd");
  const auto v1_epoch = store.ContentEpoch("warehouse");

  const auto pgo_source = root / "snapshots" / "pgo";
  WriteAsciiPcd(pgo_source / "map.pcd", 20.0);
  WriteValidConstraints(pgo_source / "pose_graph.constraints");
  const auto fake_pgo = WriteFakePgo(root);
  const auto auto_pgo_source = root / "snapshots" / "pgo_auto_skip";
  WriteCompletePatchBundle(auto_pgo_source, 19.0, 2U);
  std::ofstream(auto_pgo_source / "auto_skip", std::ios::binary) << "1\n";
  {
    SaveMapEngine engine(store);
    auto request = Request("save_with_auto_pgo_skip", "auto_pgo_map", converter);
    request.pgo.executable = fake_pgo.string();
    Require(engine.Begin(request).accepted, "automatic PGO SaveMap request was rejected");
    Require(
        engine.ProvideSnapshot(
            request.request_id, Snapshot("auto_pgo_snapshot", auto_pgo_source)).accepted,
        "automatic PGO snapshot was rejected");
    const auto status = WaitTerminal(engine, request.request_id);
    Require(
        status.state == SaveJobState::kSucceeded,
        "automatic PGO skip did not preserve SaveMap success: " + status.message);
    const auto report = ReadFile(store.MapPath("auto_pgo_map") / "map_optimization.json");
    Require(
        lingtu::maps::IsValidJsonObject(report) &&
            lingtu::maps::JsonObjectBoolAtPath(report, {"success"}) == true &&
            lingtu::maps::JsonObjectBoolAtPath(report, {"performed"}) == false &&
            lingtu::maps::JsonObjectStringAtPath(report, {"code"}) ==
                "no_verified_loops" &&
            lingtu::maps::JsonObjectNumberAtPath(report, {"pose_count"}) == 2.0 &&
            lingtu::maps::JsonObjectNumberAtPath(report, {"factor_count"}) == 1.0 &&
            lingtu::maps::JsonObjectNumberAtPath(report, {"sequential_count"}) == 1.0 &&
            lingtu::maps::JsonObjectNumberAtPath(report, {"loop_count"}) == 0.0,
        "automatic PGO skip report did not preserve the exact CLI result");
  }
  const auto auto_pgo_opt_source = root / "snapshots" / "pgo_auto_optimize";
  WriteCompletePatchBundle(auto_pgo_opt_source, 19.5, 2U);
  {
    SaveMapEngine engine(store);
    auto request = Request("save_with_auto_pgo", "auto_optimized_map", converter);
    request.pgo.executable = fake_pgo.string();
    Require(engine.Begin(request).accepted, "automatic PGO optimization request was rejected");
    Require(
        engine.ProvideSnapshot(
            request.request_id, Snapshot("auto_pgo_opt_snapshot", auto_pgo_opt_source)).accepted,
        "automatic PGO optimization snapshot was rejected");
    const auto status = WaitTerminal(engine, request.request_id);
    Require(
        status.state == SaveJobState::kSucceeded,
        "automatic PGO optimization failed: " + status.message);
    Require(
        lingtu::maps::JsonObjectStringAtPath(
            ReadFile(store.MapPath("auto_optimized_map") / "map_optimization.json"),
            {"code"}) == "optimized",
        "automatic PGO did not publish the optimized bundle");
  }
  const auto auto_without_loop_source =
      root / "snapshots" / "pgo_auto_performed_without_loop";
  WriteCompletePatchBundle(auto_without_loop_source, 19.75, 2U);
  std::ofstream(auto_without_loop_source / "performed_without_loop", std::ios::binary)
      << "1\n";
  {
    SaveMapEngine engine(store);
    auto request = Request(
        "save_auto_performed_without_loop",
        "auto_performed_without_loop",
        converter);
    request.pgo.executable = fake_pgo.string();
    Require(engine.Begin(request).accepted, "invalid automatic PGO request was rejected");
    Require(
        engine.ProvideSnapshot(
            request.request_id,
            Snapshot("auto_performed_without_loop_snapshot", auto_without_loop_source)).accepted,
        "invalid automatic PGO snapshot was rejected");
    const auto status = WaitTerminal(engine, request.request_id);
    Require(
        status.state == SaveJobState::kFailed &&
            status.reason_code == "pgo_output_invalid",
        "automatic PGO performed without a loop was accepted");
    Require(
        !std::filesystem::exists(store.MapPath("auto_performed_without_loop")),
        "invalid automatic PGO published a map");
  }

  const auto slow_pgo = WriteSlowPgo(root);
  {
    SaveMapEngine engine(store);
    auto request = Request("save_auto_pgo_timeout", "auto_timeout_map", converter);
    request.pgo.executable = slow_pgo.string();
    request.pgo.timeout_sec = 0.05;
    Require(engine.Begin(request).accepted, "automatic PGO timeout request was rejected");
    Require(
        engine.ProvideSnapshot(
            request.request_id, Snapshot("auto_timeout_snapshot", auto_pgo_opt_source)).accepted,
        "automatic PGO timeout snapshot was rejected");
    const auto status = WaitTerminal(engine, request.request_id);
    Require(
        status.state == SaveJobState::kSucceeded,
        "automatic PGO timeout should preserve the original save");
    const auto report = ReadFile(store.MapPath("auto_timeout_map") / "map_optimization.json");
    Require(
        lingtu::maps::JsonObjectStringAtPath(report, {"code"}) == "pgo_timeout" &&
            lingtu::maps::JsonObjectBoolAtPath(report, {"performed"}) == false,
        "automatic PGO timeout did not publish its exact skip report");
  }
  {
    SaveMapEngine engine(store);
    auto request = Request("save_explicit_pgo_timeout", "explicit_timeout_map", converter);
    request.pgo.executable = slow_pgo.string();
    request.pgo.timeout_sec = 0.05;
    Require(engine.Begin(request).accepted, "explicit PGO timeout request was rejected");
    Require(
        engine.ProvideSnapshot(
            request.request_id, Snapshot("explicit_timeout_snapshot", pgo_source)).accepted,
        "explicit PGO timeout snapshot was rejected");
    const auto status = WaitTerminal(engine, request.request_id);
    Require(
        status.state == SaveJobState::kFailed && status.reason_code == "pgo_timeout",
        "explicit PGO timeout did not fail strictly");
    Require(
        !std::filesystem::exists(store.MapPath("explicit_timeout_map")),
        "explicit PGO timeout published a partial map");
  }
  const auto explicit_skip_source = root / "snapshots" / "pgo_explicit_skip";
  WriteAsciiPcd(explicit_skip_source / "map.pcd", 20.25);
  WriteValidConstraints(explicit_skip_source / "pose_graph.constraints");
  std::ofstream(explicit_skip_source / "force_skip", std::ios::binary) << "1\n";
  {
    SaveMapEngine engine(store);
    auto request = Request("save_explicit_pgo_skip", "explicit_skip_map", converter);
    request.pgo.executable = fake_pgo.string();
    Require(engine.Begin(request).accepted, "explicit PGO skip request was rejected");
    Require(
        engine.ProvideSnapshot(
            request.request_id,
            Snapshot("explicit_pgo_skip_snapshot", explicit_skip_source)).accepted,
        "explicit PGO skip snapshot was rejected");
    const auto status = WaitTerminal(engine, request.request_id);
    Require(
        status.state == SaveJobState::kFailed &&
            status.reason_code == "pgo_output_invalid",
        "explicit constraints accepted a not-performed optimizer result");
    Require(
        !std::filesystem::exists(store.MapPath("explicit_skip_map")),
        "explicit PGO skip published a raw map");
  }
  std::atomic<bool> saw_optimize_phase{false};
  SaveMapHooks pgo_hooks;
  pgo_hooks.before_phase = [&](const std::string& job_id, SavePhase phase) {
    if (job_id == "save_with_pgo" && phase == SavePhase::kOptimizeSource) {
      saw_optimize_phase.store(true);
    }
  };
  {
    SaveMapEngine engine(store, pgo_hooks);
    auto request = Request("save_with_pgo", "optimized_map", converter);
    request.pgo.executable = fake_pgo.string();
    Require(engine.Begin(request).accepted, "PGO SaveMap request was rejected");
    Require(
        engine.ProvideSnapshot("save_with_pgo", Snapshot("pgo_snapshot", pgo_source)).accepted,
        "PGO snapshot was rejected");
    const auto status = WaitTerminal(engine, "save_with_pgo");
    Require(status.state == SaveJobState::kSucceeded, "PGO SaveMap failed: " + status.message);
    Require(saw_optimize_phase.load(), "SaveMap did not expose OPTIMIZE_SOURCE phase");
    Require(
        ReadFile(store.MapPath("optimized_map") / "poses.txt").find("000001.pcd") !=
            std::string::npos,
        "SaveMap did not consume the optimizer output bundle");
    Require(
        ReadFile(store.MapPath("optimized_map") / "map_optimization.json").find("optimized") !=
            std::string::npos,
        "SaveMap did not publish the optimizer report");
    Require(
        std::filesystem::is_regular_file(
            store.MapPath("optimized_map") / "patch_bundle.manifest") &&
            std::filesystem::is_regular_file(
                store.MapPath("optimized_map") / "patches" / "000001.pcd"),
        "SaveMap did not publish the optimizer patch bundle evidence");
    Require(
        !std::filesystem::exists(store.MapPath("optimized_map") / "pose_graph.constraints"),
        "SaveMap published private pose graph constraints");
    const auto persisted = ReadFile(
        store.RootDir() / ".save_jobs" / "save_with_pgo" / "job.state");
    Require(
        persisted.find("pgo_executable=") != std::string::npos &&
            persisted.find("pgo_constraints_file=pose_graph.constraints") != std::string::npos &&
            persisted.find("pgo_timeout_sec=300") != std::string::npos,
        "SaveMap did not persist its PGO request fields");
  }

  const auto invalid_constraints_source = root / "snapshots" / "pgo_invalid_constraints";
  WriteAsciiPcd(invalid_constraints_source / "map.pcd", 21.0);
  std::ofstream(invalid_constraints_source / "pose_graph.constraints", std::ios::binary)
      << "LINGTU_PGO_CONSTRAINTS_V1\n"
      << "T_from_to tx ty tz qw qx qy qz\n"
      << "RIGHT_TANGENT wx wy wz tx ty tz\n"
      << "UPPER_TRIANGLE row_major 21\n"
      << "factor\n";
  {
    SaveMapEngine engine(store);
    auto request = Request("save_pgo_bad_constraints", "warehouse", converter);
    request.pgo.executable = fake_pgo.string();
    Require(engine.Begin(request).accepted, "invalid-constraints PGO request was rejected");
    Require(
        engine.ProvideSnapshot(
            request.request_id,
            Snapshot("pgo_bad_constraints_snapshot", invalid_constraints_source)).accepted,
        "invalid-constraints snapshot was rejected");
    const auto status = WaitTerminal(engine, request.request_id);
    Require(status.state == SaveJobState::kFailed,
            "invalid PGO constraints did not fail SaveMap");
    Require(status.reason_code == "pgo_constraints_invalid",
            "invalid PGO constraints returned the wrong reason");
  }
  Require(store.ContentEpoch("warehouse") == v1_epoch,
          "invalid PGO constraints changed the canonical content epoch");
  Require(ReadFile(store.ContentPath("warehouse") / "map.pcd") == v1_content,
          "invalid PGO constraints changed the canonical map.pcd");

  for (const std::string invalid_output : {"bad_report", "bad_counts", "bad_manifest", "bad_pose_set"}) {
    const auto invalid_output_source = root / "snapshots" / invalid_output;
    WriteAsciiPcd(invalid_output_source / "map.pcd", 22.0);
    WriteValidConstraints(invalid_output_source / "pose_graph.constraints");
    std::ofstream(invalid_output_source / invalid_output, std::ios::binary) << "1\n";
    SaveMapEngine engine(store);
    const std::string job_id = "save_pgo_" + invalid_output;
    auto request = Request(job_id, "warehouse", converter);
    request.pgo.executable = fake_pgo.string();
    Require(engine.Begin(request).accepted, "invalid PGO output request was rejected");
    Require(
        engine.ProvideSnapshot(job_id, Snapshot(job_id + "_snapshot", invalid_output_source)).accepted,
        "invalid PGO output snapshot was rejected");
    const auto status = WaitTerminal(engine, job_id);
    Require(status.state == SaveJobState::kFailed,
            "invalid PGO output did not fail SaveMap: " + invalid_output);
    Require(status.reason_code == "pgo_output_invalid",
            "invalid PGO output returned the wrong reason: " + invalid_output);
    Require(store.ContentEpoch("warehouse") == v1_epoch,
            "invalid PGO output changed the canonical content epoch: " + invalid_output);
    Require(ReadFile(store.ContentPath("warehouse") / "map.pcd") == v1_content,
            "invalid PGO output changed the canonical map.pcd: " + invalid_output);
  }

  {
    std::atomic<bool> entered_optimize{false};
    std::atomic<bool> release_optimize{false};
    SaveMapHooks legacy_hooks;
    legacy_hooks.before_phase = [&](const std::string& job_id, SavePhase phase) {
      if (job_id == "legacy_v3_interrupted" && phase == SavePhase::kOptimizeSource) {
        entered_optimize.store(true);
        while (!release_optimize.load()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
      }
    };
    auto interrupted = std::make_unique<SaveMapEngine>(store, legacy_hooks);
    auto request = Request("legacy_v3_interrupted", "warehouse", converter);
    request.pgo.executable = fake_pgo.string();
    Require(interrupted->Begin(request).accepted, "legacy-v3 fixture request was rejected");
    Require(
        interrupted->ProvideSnapshot(
            request.request_id, Snapshot("legacy_v3_snapshot", pgo_source)).accepted,
        "legacy-v3 fixture snapshot was rejected");
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!entered_optimize.load() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    Require(entered_optimize.load(), "legacy-v3 fixture did not reach PGO phase");
    std::thread shutdown([engine = std::move(interrupted)]() mutable { engine.reset(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    release_optimize.store(true);
    shutdown.join();

    const auto job_dir = store.RootDir() / ".save_jobs" / request.request_id;
    const auto capture_sentinel = job_dir / "capture" / "legacy-sentinel";
    std::ofstream(capture_sentinel, std::ios::binary) << "preserve\n";
    ReplaceStateValue(job_dir / "job.state", "schema", "3");
    SaveMapEngine recovered(store);
    const auto status = recovered.GetStatus(request.request_id);
    Require(status.has_value() && status->state == SaveJobState::kFailed &&
                status->phase == SavePhase::kDone,
            "non-terminal v3 SaveMap job was resumed under the v4 behavior");
    Require(status->reason_code == "legacy_schema_requires_resubmit",
            "non-terminal v3 SaveMap job returned the wrong recovery reason");
    Require(std::filesystem::is_regular_file(capture_sentinel),
            "legacy-v3 recovery removed captured source evidence");
    const auto retry = recovered.Retry(request.request_id);
    Require(!retry.accepted && retry.reason_code == "legacy_schema_requires_resubmit",
            "legacy-v3 recovery was allowed to retry under the new behavior");
    Require(store.ContentEpoch("warehouse") == v1_epoch,
            "legacy-v3 recovery changed the canonical content epoch");
    Require(ReadFile(store.ContentPath("warehouse") / "map.pcd") == v1_content,
            "legacy-v3 recovery changed the canonical map.pcd");
  }

  const auto failing_pgo = WriteFakePgo(root, true);
  {
    SaveMapEngine engine(store);
    auto request = Request("save_pgo_fail", "warehouse", converter);
    request.pgo.executable = failing_pgo.string();
    Require(engine.Begin(request).accepted, "failing PGO request was rejected");
    Require(
        engine.ProvideSnapshot("save_pgo_fail", Snapshot("pgo_fail_snapshot", pgo_source)).accepted,
        "failing PGO snapshot was rejected");
    const auto status = WaitTerminal(engine, "save_pgo_fail");
    Require(status.state == SaveJobState::kFailed, "failing optimizer did not fail SaveMap");
    Require(status.reason_code == "pgo_failed", "optimizer failure returned the wrong reason");
  }
  Require(
      store.ContentEpoch("warehouse") == v1_epoch,
      "optimizer failure changed the canonical content epoch");
  Require(
      ReadFile(store.ContentPath("warehouse") / "map.pcd") == v1_content,
      "optimizer failure changed the canonical map.pcd");

  const auto source_v2 = root / "snapshots" / "v2";
  WriteAsciiPcd(source_v2 / "map.pcd", 10.0);
  SaveMapHooks fail_hooks;
  fail_hooks.forced_failure = [](const std::string& job_id, SavePhase phase)
      -> std::optional<std::string> {
    if (job_id == "save_v2_fail" && phase == SavePhase::kBuildArtifacts) {
      return "injected artifact failure";
    }
    return std::nullopt;
  };
  {
    SaveMapEngine engine(store, fail_hooks);
    const auto request = Request("save_v2_fail", "warehouse", converter);
    Require(engine.Begin(request).accepted, "v2 failure request rejected");
    Require(engine.ProvideSnapshot("save_v2_fail", Snapshot("snapshot_v2", source_v2)).accepted,
            "v2 failure snapshot rejected");
    const auto status = WaitTerminal(engine, "save_v2_fail");
    Require(status.state == SaveJobState::kFailed, "injected failure did not fail job");
  }
  Require(store.ContentEpoch("warehouse") > 0, "failed save removed current map");
  Require(ReadFile(store.ContentPath("warehouse") / "map.pcd") == v1_content,
          "failed save changed current map content");

  {
    // Reproduce the durable state left when commit replacement and its inline
    // previous-map restore both fail. Startup must restore the backup even
    // though the job journal is already terminal.
    ReplaceStateValue(
        store.RootDir() / ".save_jobs" / "save_v2_fail" / "job.state",
        "reason_code",
        "save_map_failed");
    const auto backup = store.RootDir() / ".save-backup-warehouse-save_v2_fail";
    std::filesystem::rename(store.MapPath("warehouse"), backup);
    SaveMapEngine recovered(store);
    const auto status = recovered.GetStatus("save_v2_fail");
    Require(status.has_value() && status->state == SaveJobState::kFailed,
            "restart rewrote the failed commit history");
    Require(std::filesystem::is_directory(store.MapPath("warehouse")),
            "restart did not restore the previous map after inline restore failure");
    Require(!std::filesystem::exists(backup),
            "restart retained the previous-map backup after recovery");
    Require(ReadFile(store.ContentPath("warehouse") / "map.pcd") == v1_content,
            "restart restored different map content after commit failure");
  }

  {
    SaveMapEngine engine(store);
    const auto request = Request("cancel_before_snapshot", "warehouse", converter);
    Require(engine.Begin(request).accepted, "cancel request setup failed");
    const auto cancelled = engine.Cancel("cancel_before_snapshot");
    Require(cancelled.accepted, "cancel request rejected");
    Require(cancelled.status.state == SaveJobState::kCancelled,
            "waiting job did not cancel immediately");
    const auto retried = engine.Retry("cancel_before_snapshot");
    Require(retried.accepted, "cancelled SaveMap job could not be retried");
    Require(retried.status.state == SaveJobState::kWaitingSnapshot,
            "retry without a captured snapshot must wait for a new snapshot");
    Require(
        engine.ProvideSnapshot(
            "cancel_before_snapshot",
            Snapshot("retry_snapshot", source_v2)).accepted,
        "retried SaveMap snapshot was rejected");
    const auto retry_status = WaitTerminal(engine, "cancel_before_snapshot");
    Require(retry_status.state == SaveJobState::kSucceeded,
            "retried SaveMap job did not succeed");
  }

  {
    std::atomic<bool> snapshot_copied{false};
    std::atomic<bool> release_snapshot{false};
    SaveMapHooks hooks;
    hooks.after_snapshot_copied = [&](const std::string& job_id) {
      if (job_id != "cancel_during_snapshot_copy") return;
      snapshot_copied.store(true);
      while (!release_snapshot.load()) {
        std::this_thread::yield();
      }
    };
    SaveMapEngine engine(store, hooks);
    const auto request = Request(
        "cancel_during_snapshot_copy", "cancel_during_snapshot_copy_map", converter);
    Require(engine.Begin(request).accepted, "snapshot-copy cancel request rejected");

    SaveMapResult provided;
    std::thread provider([&]() {
      provided = engine.ProvideSnapshot(
          request.request_id, Snapshot("cancel_during_copy_snapshot", source_v2));
    });
    const auto copied_deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!snapshot_copied.load() && std::chrono::steady_clock::now() < copied_deadline) {
      std::this_thread::yield();
    }
    Require(snapshot_copied.load(), "snapshot provider did not reach the post-copy boundary");
    const auto cancelled = engine.Cancel(request.request_id);
    Require(cancelled.accepted && cancelled.status.state == SaveJobState::kCancelled,
            "snapshot-copy overlap did not cancel the waiting SaveMap job");
    const auto early_retry = engine.Retry(request.request_id);
    Require(!early_retry.accepted &&
                early_retry.reason_code == "snapshot_capture_in_progress",
            "retry admitted a stale in-flight snapshot ACK after cancellation");
    release_snapshot.store(true);
    provider.join();

    Require(!provided.accepted,
            "snapshot ACK was accepted after its SaveMap job had been cancelled");
    const auto status = engine.GetStatus(request.request_id);
    Require(status.has_value() && status->state == SaveJobState::kCancelled,
            "snapshot ACK changed a cancelled SaveMap job back to queued");
    Require(!std::filesystem::exists(store.MapPath("cancel_during_snapshot_copy_map")),
            "cancelled snapshot-copy overlap published a map");
  }

  {
    std::atomic<bool> failure_ready{false};
    std::atomic<bool> release_failure{false};
    SaveMapHooks hooks;
    hooks.after_snapshot_copied = [&](const std::string& job_id) {
      if (job_id != "cancel_during_snapshot_failure") return;
      failure_ready.store(true);
      while (!release_failure.load()) {
        std::this_thread::yield();
      }
      throw std::runtime_error("snapshot identity mismatch");
    };
    SaveMapEngine engine(store, hooks);
    const auto request = Request(
        "cancel_during_snapshot_failure", "cancel_during_snapshot_failure_map", converter);
    Require(engine.Begin(request).accepted, "snapshot-failure cancel request rejected");

    SaveMapResult rejected;
    std::thread provider([&]() {
      rejected = engine.ProvideSnapshot(
          request.request_id, Snapshot("cancel_during_failure_snapshot", source_v2));
    });
    const auto failure_deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!failure_ready.load() && std::chrono::steady_clock::now() < failure_deadline) {
      std::this_thread::yield();
    }
    Require(failure_ready.load(), "snapshot provider did not reach the failure boundary");
    const auto cancelled = engine.Cancel(request.request_id);
    Require(cancelled.accepted && cancelled.status.state == SaveJobState::kCancelled,
            "snapshot-failure overlap did not cancel the waiting SaveMap job");
    release_failure.store(true);
    provider.join();

    Require(!rejected.accepted && rejected.replayed &&
                rejected.status.state == SaveJobState::kCancelled,
            "failure ACK overwrote a cancelled SaveMap job");
    const auto status = engine.GetStatus(request.request_id);
    Require(status.has_value() && status->state == SaveJobState::kCancelled,
            "failure ACK changed a cancelled SaveMap job to failed");
  }

  {
    SaveMapEngine engine(store);
    const auto request = Request("snapshot_race", "snapshot_race_map", converter);
    Require(engine.Begin(request).accepted, "snapshot race request rejected");
    SaveMapResult first_result;
    SaveMapResult second_result;
    std::atomic<bool> start{false};
    std::thread first([&]() {
      while (!start.load()) std::this_thread::yield();
      first_result = engine.ProvideSnapshot(
          "snapshot_race", Snapshot("race_snapshot_one", source_v1));
    });
    std::thread second([&]() {
      while (!start.load()) std::this_thread::yield();
      second_result = engine.ProvideSnapshot(
          "snapshot_race", Snapshot("race_snapshot_two", source_v2));
    });
    start.store(true);
    first.join();
    second.join();
    const int accepted_snapshots =
        static_cast<int>(first_result.accepted && !first_result.replayed) +
        static_cast<int>(second_result.accepted && !second_result.replayed);
    Require(accepted_snapshots == 1,
            "concurrent snapshot callers did not bind exactly one map snapshot");
    const auto& rejected = first_result.accepted ? second_result : first_result;
    Require(
        rejected.reason_code == "snapshot_capture_in_progress" ||
            rejected.reason_code == "snapshot_idempotency_conflict",
        "concurrent snapshot caller was not rejected with a stable reason");
    Require(WaitTerminal(engine, "snapshot_race").state == SaveJobState::kSucceeded,
            "snapshot race winner did not produce a valid map version");
  }

  {
    SaveMapEngine engine(store);
    const auto first = Request("concurrent_first", "concurrent_map", converter);
    const auto second = Request("concurrent_second", "concurrent_map", converter);
    Require(engine.Begin(first).accepted && engine.Begin(second).accepted,
            "concurrent SaveMap submissions were rejected");
    Require(engine.ProvideSnapshot(
                "concurrent_first",
                Snapshot("concurrent_snapshot_1", source_v1)).accepted,
            "first concurrent snapshot rejected");
    Require(engine.ProvideSnapshot(
                "concurrent_second",
                Snapshot("concurrent_snapshot_2", source_v2)).accepted,
            "second concurrent snapshot rejected");
    const auto first_status = WaitTerminal(engine, "concurrent_first");
    const auto second_status = WaitTerminal(engine, "concurrent_second");
    Require(first_status.state == SaveJobState::kSucceeded,
            "first concurrent SaveMap failed");
    Require(second_status.state == SaveJobState::kSucceeded,
            "second concurrent SaveMap failed");
    Require(store.ContentEpoch("concurrent_map") > 0,
            "direct map directory lost its content epoch");
    const auto final_points = lingtu::maps::LoadPcdXyz(
        store.ContentPath("concurrent_map") / "map.pcd");
    Require(final_points.ok && !final_points.points.empty() && final_points.points[0].x > 5.0F,
            "second serialized SaveMap did not replace direct map content");
    Require(!std::filesystem::exists(store.MapPath("concurrent_map") / ".versions"),
            "concurrent saves retained version history");
  }

  const auto slow_converter = WriteFakeConverter(root, true);
  {
    SaveMapEngine engine(store);
    auto request = Request("cancel_running", "cancel_map", slow_converter);
    request.octomap.timeout_sec = 60.0;
    Require(engine.Begin(request).accepted, "running cancel request rejected");
    Require(engine.ProvideSnapshot("cancel_running", Snapshot("snapshot_cancel", source_v2)).accepted,
            "running cancel snapshot rejected");
    WaitPhase(engine, "cancel_running", SavePhase::kBuildArtifacts);
    Require(!store.DeleteMap("cancel_map").ok,
            "map deletion bypassed the active SaveMap write lock");
    Require(!store.RenameMap("cancel_map", "cancel_map_renamed").ok,
            "map rename bypassed the active SaveMap write lock");
    Require(engine.Cancel("cancel_running").accepted, "running cancel call rejected");
    const auto status = WaitTerminal(engine, "cancel_running");
    Require(status.state == SaveJobState::kCancelled, "running process did not cancel");
    Require(!std::filesystem::exists(store.MapPath("cancel_map")),
            "cancelled save committed a map");
  }

  std::atomic<bool> entered_validate{false};
  std::atomic<bool> release_validate{false};
  SaveMapHooks recovery_hooks;
  recovery_hooks.before_phase = [&](const std::string& job_id, SavePhase phase) {
    if (job_id == "recover_job" && phase == SavePhase::kValidate) {
      entered_validate.store(true);
      while (!release_validate.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    }
  };
  auto interrupted = std::make_unique<SaveMapEngine>(store, recovery_hooks);
  const auto recover_request = Request("recover_job", "recovered_map", converter);
  Require(interrupted->Begin(recover_request).accepted, "recovery request rejected");
  Require(interrupted->ProvideSnapshot("recover_job", Snapshot("recover_snapshot", source_v2)).accepted,
          "recovery snapshot rejected");
  const auto enter_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!entered_validate.load() && std::chrono::steady_clock::now() < enter_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  Require(entered_validate.load(), "recovery job did not enter validation");
  std::thread shutdown([engine = std::move(interrupted)]() mutable { engine.reset(); });
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  release_validate.store(true);
  shutdown.join();

  {
    SaveMapEngine recovered(store);
    const auto status = WaitTerminal(recovered, "recover_job");
    Require(status.state == SaveJobState::kSucceeded, "recovered job did not succeed: " + status.message);
    Require(status.recovered, "recovered job status did not preserve recovery evidence");
    Require(store.ContentEpoch("recovered_map") > 0, "recovered job identity missing");
  }

  {
    const auto active_state = store.RootDir() / "active_map.txt";
    std::filesystem::remove_all(active_state);
    std::filesystem::create_directory(active_state);
    SaveMapEngine engine(store);
    auto request = Request("activation_failure", "activation_failure_map", converter);
    UseValidActivationOctomap(request);
    request.activate_on_success = true;
    Require(engine.Begin(request).accepted, "activation failure request rejected");
    Require(
        engine.ProvideSnapshot(
            "activation_failure",
            Snapshot("activation_failure_snapshot", source_v2)).accepted,
        "activation failure snapshot rejected");
    const auto status = WaitTerminal(engine, "activation_failure");
    Require(status.state == SaveJobState::kFailed,
            "failed requested activation was reported as SaveMap success");
    Require(status.reason_code == "activation_failed_after_commit",
            "activation failure did not expose a stable reason code");
    Require(status.activation_requested,
            "activation failure did not preserve activation request evidence");
    Require(!status.activation_succeeded,
            "activation failure incorrectly reported activation success");
    Require(store.ContentEpoch("activation_failure_map") > 0,
            "activation failure discarded the already committed version");
    Require(store.ActiveMapId() != "activation_failure_map",
            "activation failure incorrectly changed active-map state");
    std::filesystem::remove_all(active_state);
  }

  {
    SaveMapEngine engine(store);
    Require(engine.Begin(Request("corrupt_save_state", "corrupt_save_map", converter)).accepted,
            "failed to create SaveMap state-corruption fixture");
    Require(engine.Begin(Request("duplicate_save_key", "duplicate_save_map", converter)).accepted,
            "failed to create SaveMap duplicate-key fixture");
  }
  ReplaceStateValue(
      store.RootDir() / ".save_jobs" / "corrupt_save_state" / "job.state",
      "state",
      "UNKNOWN");
  {
    std::ofstream duplicate(
        store.RootDir() / ".save_jobs" / "duplicate_save_key" / "job.state",
        std::ios::binary | std::ios::app);
    duplicate << "state=FAILED\n";
  }
  {
    SaveMapEngine recovered(store);
    for (const std::string job_id : {"corrupt_save_state", "duplicate_save_key"}) {
      const auto status = recovered.GetStatus(job_id);
      Require(status.has_value(), "corrupt SaveMap journal disappeared: " + job_id);
      Require(status->state == SaveJobState::kFailed,
              "corrupt SaveMap journal did not fail closed: " + job_id);
      Require(status->reason_code == "journal_corrupt",
              "corrupt SaveMap journal lacks stable reason: " + job_id);
      const auto retry = recovered.Retry(job_id);
      Require(!retry.accepted && retry.reason_code == "journal_corrupt",
              "corrupt SaveMap journal was allowed to retry: " + job_id);
    }
    const auto replay = recovered.Begin(
        Request("corrupt_save_state", "corrupt_save_map", converter));
    Require(!replay.accepted && replay.reason_code == "journal_corrupt",
            "corrupt SaveMap request identity was allowed to execute again");
  }

  {
    const auto lock_root = TempRoot();
    MapStore lock_store(MapStoreConfig{lock_root});
    const auto lock_dir = lock_root / ".save_engine_lock";
    std::filesystem::create_directories(lock_dir);
    std::ofstream owner(lock_dir / "owner.state", std::ios::binary);
    owner << "pid=not-a-pid\n";
    owner.close();
    bool rejected_corrupt_owner = false;
    try {
      SaveMapEngine engine(lock_store);
    } catch (const std::exception&) {
      rejected_corrupt_owner = true;
    }
    Require(rejected_corrupt_owner,
            "SaveMap engine stole a lock with corrupt ownership evidence");
    std::filesystem::remove_all(lock_root);
  }

  const auto obsolete_pointer = store.MapPath("concurrent_map") / "current_version.txt";
  {
    std::ofstream file(obsolete_pointer, std::ios::binary | std::ios::trunc);
    file << "obsolete\n";
  }
  const auto direct_record = store.GetMapRecord("concurrent_map");
  Require(direct_record.has_value(), "obsolete pointer hid direct map record");
  Require(!direct_record->artifacts.empty(), "obsolete pointer hid direct map artifacts");
  {
    SaveMapEngine engine(store);
    const auto request = Request(
        "save_over_obsolete_pointer", "concurrent_map", converter);
    const auto begin = engine.Begin(request);
    Require(begin.accepted, "SaveMap did not accept direct-map replacement fixture");
    const auto provided = engine.ProvideSnapshot(
        request.request_id,
        Snapshot("snapshot_direct_replacement", source_v1));
    Require(provided.accepted, "SaveMap did not accept replacement snapshot");
    const auto status = WaitTerminal(engine, request.request_id);
    Require(status.state == SaveJobState::kSucceeded,
            "obsolete pointer blocked direct-map replacement");
    Require(!std::filesystem::exists(obsolete_pointer),
            "direct-map replacement retained obsolete pointer");
  }
  Require(store.ContentEpoch("concurrent_map") > 0,
          "direct map compatibility version changed");

  std::filesystem::remove_all(root);
  return 0;
}

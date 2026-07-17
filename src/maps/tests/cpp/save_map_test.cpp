#include "lingtu/maps/hash.hpp"
#include "lingtu/maps/save.hpp"

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <sstream>
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
using lingtu::maps::Sha256File;

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
  file << "echo FAKE_OCTOMAP > \"%~2\"\r\n"
       << "type \"%~1\" >> \"%~2\"\r\n";
#else
  const auto script = root / (slow ? "slow_converter.sh" : "converter.sh");
  std::ofstream file(script, std::ios::binary);
  file << "#!/bin/sh\n";
  if (slow) {
    file << "sleep 20\n";
  }
  file << "printf 'FAKE_OCTOMAP\\n' > \"$2\"\n"
       << "cat \"$1\" >> \"$2\"\n";
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

SaveMapRequest Request(
    const std::string& request_id,
    const std::string& map_id,
    const std::string& converter) {
  SaveMapRequest request;
  request.request_id = request_id;
  request.map_id = map_id;
  request.source.dynamic_filter_enabled = false;
  request.source.optimizer_strategy = "none";
  request.octomap.converter_command = converter;
  request.octomap.build_mode = "external_pcl_converter";
  request.octomap.timeout_sec = 30.0;
  request.require.semantic = false;
  return request;
}

MapSnapshot Snapshot(const std::string& id, const std::filesystem::path& source) {
  MapSnapshot snapshot;
  snapshot.snapshot_id = id;
  snapshot.source_dir = source;
  snapshot.frame_id = "map";
  snapshot.slam_healthy = true;
  return snapshot;
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
  {
    SaveMapEngine engine(store);
    const auto request = Request("save_v1", "warehouse", converter);
    const auto begin = engine.Begin(request);
    Require(begin.accepted && !begin.replayed, "first SaveMap request was not accepted");
    Require(std::filesystem::is_directory(begin.status.capture_dir), "capture dir missing");
    const auto provided = engine.ProvideSnapshot("save_v1", Snapshot("snapshot_v1", source_v1));
    Require(provided.accepted, "snapshot was not accepted");
    const auto status = WaitTerminal(engine, "save_v1");
    Require(status.state == SaveJobState::kSucceeded, "SaveMap v1 failed: " + status.message);
    Require(status.version == 1, "first committed version must be 1");
    Require(std::filesystem::is_regular_file(status.manifest_path), "version manifest missing");
    Require(std::filesystem::is_regular_file(status.version_dir / "map.pcd"), "version map.pcd missing");
    Require(std::filesystem::is_regular_file(status.version_dir / "octomap.ot"), "version octomap missing");
    Require(std::filesystem::is_regular_file(status.version_dir / "occupancy.npz"), "version occupancy missing");
    Require(std::filesystem::is_regular_file(status.version_dir / "esdf.npz"), "version ESDF missing");
    Require(
        std::filesystem::is_regular_file(status.version_dir / "traversability.npz"),
        "version traversability missing");
    const auto manifest = ReadFile(status.manifest_path);
    Require(!manifest.empty(), "cannot read version manifest");
    Require(
        ReadFile(status.version_dir / "save_manifest.sha256").find(
            Sha256File(status.manifest_path)) != std::string::npos,
        "manifest checksum sidecar does not match manifest");
    Require(
        manifest.find(Sha256File(status.version_dir / "map.pcd")) != std::string::npos,
        "manifest does not contain actual map.pcd hash");
    Require(
        manifest.find(Sha256File(status.version_dir / "octomap.ot")) != std::string::npos,
        "manifest does not contain actual octomap hash");
    Require(manifest.find("\"source_sha256\"") != std::string::npos,
            "manifest does not bind artifact dependency hashes");
    Require(manifest.find("\"octomap_converter_command_sha256\"") != std::string::npos,
            "manifest does not record builder command provenance");
    const auto journal = ReadFile(
        store.RootDir() / ".save_jobs" / "save_v1" / "events.jsonl");
    Require(journal.find("VERSION_PREPARED") != std::string::npos,
            "SaveMap journal is missing VERSION_PREPARED");
    Require(journal.find("VERSION_COMMITTED") != std::string::npos,
            "SaveMap journal is missing VERSION_COMMITTED");
    Require(journal.find("SUCCEEDED:DONE") != std::string::npos,
            "SaveMap journal is missing terminal success");
    Require(!std::filesystem::exists(status.capture_dir),
            "successful SaveMap retained its capture staging directory");
    Require(std::filesystem::is_regular_file(store.MapPath("warehouse") / "map.pcd"),
            "compatibility map view missing");

    const auto record = store.GetMapRecord("warehouse");
    Require(record.has_value(), "MapRecord missing after SaveMap");
    Require(record->version == 1, "MapRecord version not derived from current version");
    Require(!record->artifacts.empty(), "MapRecord artifacts missing");
    for (const auto& artifact : record->artifacts) {
      Require(
          artifact.uri.find(".versions") != std::string::npos,
          "MapRecord must expose immutable version artifacts");
      Require(artifact.sha256 == Sha256File(artifact.uri), "MapRecord artifact hash mismatch");
    }

    const auto replay = engine.Begin(request);
    Require(replay.accepted && replay.replayed, "identical request_id was not replayed");
    auto conflict_request = request;
    conflict_request.map_id = "different_map";
    const auto conflict = engine.Begin(conflict_request);
    Require(!conflict.accepted && conflict.reason_code == "idempotency_conflict",
            "request_id conflict was not rejected");
  }

  {
    SaveMapEngine engine(store);
    auto request = Request("activate_success", "activated_map", converter);
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
    Require(store.ActiveMapId() == "activated_map", "SaveMap did not activate committed map");
  }

  {
    const auto state = store.RootDir() / ".save_jobs" / "save_v1" / "job.state";
    ReplaceStateValue(state, "state", "RUNNING");
    ReplaceStateValue(state, "phase", "COMMIT");
    ReplaceStateValue(state, "completed_at_ns", "0");
    SaveMapEngine recovered(store);
    const auto status = WaitTerminal(recovered, "save_v1");
    Require(status.state == SaveJobState::kSucceeded,
            "committed pointer recovery did not restore success");
    Require(status.recovered, "committed pointer recovery lacks recovery evidence");
    Require(status.compatibility_ready,
            "committed pointer recovery did not restore the compatibility view");
  }

  {
    std::filesystem::remove(store.MapPath("warehouse") / "map.pcd");
    std::filesystem::remove(store.MapPath("warehouse") / "compatibility_version.txt");
    SaveMapEngine recovered(store);
    const auto status = recovered.GetStatus("save_v1");
    Require(status.has_value() && status->compatibility_ready,
            "terminal SaveMap job did not repair a stale compatibility view on restart");
    Require(std::filesystem::is_regular_file(store.MapPath("warehouse") / "map.pcd"),
            "compatibility recovery did not restore map.pcd");
  }

  const std::string v1_pointer = [&]() {
    std::ifstream file(store.MapPath("warehouse") / "current_version.txt");
    std::string value;
    std::getline(file, value);
    return value;
  }();
  const std::string v1_sha = Sha256File(store.ContentPath("warehouse") / "map.pcd");

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
  Require(store.CurrentVersion("warehouse") == 1, "failed save changed current version");
  Require(Sha256File(store.ContentPath("warehouse") / "map.pcd") == v1_sha,
          "failed save changed current map content");
  {
    std::ifstream file(store.MapPath("warehouse") / "current_version.txt");
    std::string pointer;
    std::getline(file, pointer);
    Require(pointer == v1_pointer, "failed save changed current version pointer");
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
            "concurrent snapshot callers did not bind exactly one immutable snapshot");
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
    Require(first_status.version == 1 && second_status.version == 2,
            "same-map SaveMap jobs were not serialized into distinct versions");
    Require(store.CurrentVersion("concurrent_map") == 2,
            "latest serialized SaveMap version is not current");
  }

  {
    const auto versions_root = store.MapPath("concurrent_map") / ".versions";
    const auto version_one = versions_root / "00000000000000000001";
    const auto version_two = versions_root / "00000000000000000002";
    const auto version_one_sha = Sha256File(version_one / "map.pcd");
    const auto version_two_sha = Sha256File(version_two / "map.pcd");
    Require(version_one_sha != version_two_sha,
            "version fixtures must contain different map content");
    std::filesystem::create_directories(versions_root / "not-a-version");

    SaveMapEngine engine(store);
    const auto listed = engine.ListVersionsJson("concurrent_map");
    const auto version_two_pos = listed.find("\"version\":2");
    const auto version_one_pos = listed.find("\"version\":1");
    Require(listed.find("\"count\":2") != std::string::npos,
            "version list exposed invalid or missing versions");
    Require(version_two_pos != std::string::npos && version_one_pos != std::string::npos &&
                version_two_pos < version_one_pos,
            "version list is not newest-first");
    Require(listed.find("\"version\":2,\"current\":true") != std::string::npos,
            "version list did not identify the current version");

    const auto invalid = engine.RollbackVersionJson("concurrent_map", 0);
    Require(invalid.find("invalid_version") != std::string::npos,
            "rollback accepted a non-positive version");
    const auto missing = engine.RollbackVersionJson("concurrent_map", 99);
    Require(missing.find("version_not_found_or_invalid") != std::string::npos,
            "rollback accepted a missing version");

    const auto rolled = engine.RollbackVersionJson("concurrent_map", 1);
    Require(rolled.find("\"success\":true") != std::string::npos,
            "rollback to a verified version failed");
    Require(store.CurrentVersion("concurrent_map") == 1,
            "rollback did not atomically change the current version");
    Require(Sha256File(store.ContentPath("concurrent_map") / "map.pcd") == version_one_sha,
            "rollback content does not match immutable version one");
    Require(Sha256File(store.MapPath("concurrent_map") / "map.pcd") == version_one_sha,
            "rollback did not synchronize the compatibility view");
    Require(ReadFile(store.MapPath("concurrent_map") / "version_events.jsonl").find(
                "VERSION_ROLLED_BACK") != std::string::npos,
            "rollback did not append a durable version event");
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
    Require(engine.ListVersionsJson("cancel_map").find("map_write_in_progress") !=
                std::string::npos,
            "version query ignored the active map write lock");
    Require(!store.DeleteMap("cancel_map").ok,
            "map deletion bypassed the active SaveMap write lock");
    Require(!store.RenameMap("cancel_map", "cancel_map_renamed").ok,
            "map rename bypassed the active SaveMap write lock");
    Require(engine.RollbackVersionJson("cancel_map", 1).find("map_save_in_progress") !=
                std::string::npos,
            "rollback bypassed the active SaveMap write lock");
    Require(engine.Cancel("cancel_running").accepted, "running cancel call rejected");
    const auto status = WaitTerminal(engine, "cancel_running");
    Require(status.state == SaveJobState::kCancelled, "running process did not cancel");
    Require(!std::filesystem::exists(store.MapPath("cancel_map") / "current_version.txt"),
            "cancelled save published a version");
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
    Require(store.CurrentVersion("recovered_map") == 1, "recovered job version missing");
  }

  {
    const auto active_state = store.RootDir() / "active_map.txt";
    std::filesystem::remove_all(active_state);
    std::filesystem::create_directory(active_state);
    SaveMapEngine engine(store);
    auto request = Request("activation_failure", "activation_failure_map", converter);
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
    Require(store.CurrentVersion("activation_failure_map") == 1,
            "activation failure discarded the already committed version");
    Require(store.ActiveMapId() != "activation_failure_map",
            "activation failure incorrectly changed active-map state");
    std::filesystem::remove_all(active_state);
  }

  const auto corrupt_dir = store.ContentPath("concurrent_map");
  {
    std::ofstream file(corrupt_dir / "map.pcd", std::ios::binary | std::ios::app);
    file << "corrupt";
  }
  const auto corrupt_record = store.GetMapRecord("concurrent_map");
  Require(corrupt_record.has_value(), "corrupt current map record disappeared");
  Require(corrupt_record->state == MapState::kFailed,
          "artifact corruption did not fail the current map record");
  Require(corrupt_record->artifacts.empty(),
          "artifact corruption exposed unverified map artifacts");
  Require(!store.SetActiveMap("concurrent_map", true).ok,
          "artifact corruption passed the strict active-map gate");

  std::filesystem::remove_all(root);
  return 0;
}

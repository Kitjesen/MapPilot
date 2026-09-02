#include <cassert>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

#include "lingtu/maps/build/occupancy_snapshot.hpp"
#include "lingtu/maps/build/pcd.hpp"
#include "lingtu/maps/build/pipeline.hpp"
#include "lingtu/maps/build/process.hpp"
#include "lingtu/maps/json.hpp"
#include "lingtu/maps/mapd/activation.hpp"

#if defined(_WIN32)
#define NOMINMAX
#include <windows.h>
#else
#include <unistd.h>
#endif

namespace {

using lingtu::maps::MapStore;
using lingtu::maps::MapStoreConfig;
using lingtu::maps::ProcessRunOptions;
using lingtu::maps::RunShellCommand;
using lingtu::maps::mapd::ActivationCoordinator;
using lingtu::maps::mapd::DecodeActivationToken;
using lingtu::maps::mapd::EncodeActivationToken;
using lingtu::maps::mapd::ActivationOperation;
using lingtu::maps::mapd::ActivationRequest;
using lingtu::maps::mapd::MapIdentity;

std::filesystem::path TempRoot() {
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto root = std::filesystem::temp_directory_path() /
                    ("lingtu_map_activation_test_" + std::to_string(stamp));
  std::filesystem::create_directories(root);
  return root;
}

void Write(const std::filesystem::path &path, const std::string &value) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  output << value;
  assert(output.good());
}

std::string Read(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  assert(input.good());
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

void SetEnvVar(const char *name, const char *value) {
#if defined(_WIN32)
  assert(_putenv_s(name, value) == 0);
#else
  assert(setenv(name, value, 1) == 0);
#endif
}

void UnsetEnvVar(const char *name) {
#if defined(_WIN32)
  assert(_putenv_s(name, "") == 0);
#else
  assert(unsetenv(name) == 0);
#endif
}

std::uint64_t CurrentProcessIdValue() {
#if defined(_WIN32)
  return static_cast<std::uint64_t>(GetCurrentProcessId());
#else
  return static_cast<std::uint64_t>(getpid());
#endif
}

void CreateReadyMap(const std::filesystem::path &root, const std::string &id) {
  const auto map = root / id;
  const std::vector<lingtu::maps::PointXyz> points = {
      {0.0F, 0.0F, 0.5F},
      {1.0F, 1.0F, 0.5F},
  };
  std::string error;
  assert(lingtu::maps::WriteBinaryXyzPcd(map / "map.pcd", points, &error));
  assert(lingtu::maps::BuildOccupancyProjectionSnapshot(map, true).ok);
  MapStore fixture_store(MapStoreConfig{root});
  lingtu::maps::MapPipelineCore pipeline(fixture_store);
  lingtu::maps::OctomapBuildOptions options;
  options.build_mode = "native_octomap";
  options.resolution = 0.1;
  const auto octomap_result = pipeline.BuildOctomapArtifactJson(id, options);
  if (lingtu::maps::JsonObjectBoolAtPath(octomap_result, {"success"}) != true) {
    // Builds without OctoMap keep the existing header-only fallback. A build
    // with OctoMap must exercise the real reader with a valid binary tree.
    Write(
        map / "octomap.ot",
        "# Octomap OcTree binary file\n"
        "id OcTree\n"
        "size 1\n"
        "res 0.1\n"
        "data\n"
        "x");
  }
  Write(
      map / "metadata.json",
      "{\"frame_id\":\"map\",\"artifacts\":{"
      "\"map_pcd\":{\"path\":\"map.pcd\"},"
      "\"octomap\":{\"path\":\"octomap.ot\"},"
      "\"occupancy_grid\":{\"path\":\"occupancy.npz\"}}}");
}

ActivationRequest Request(std::string id, ActivationOperation operation, MapIdentity target,
                          MapIdentity previous) {
  ActivationRequest request;
  request.request_id = std::move(id);
  request.operation = operation;
  request.target = std::move(target);
  request.previous = std::move(previous);
  request.caller = "map-activation-test";
  return request;
}

void RequireInvalidToken(const std::string &token, const std::string &expected_message) {
  try {
    (void)DecodeActivationToken(token);
    assert(false);
  } catch (const std::invalid_argument &error) {
    assert(error.what() == expected_message);
  }
}

void TestMetadataMtimeDoesNotChangeContentEpoch() {
  const auto root = TempRoot();
  CreateReadyMap(root, "map_a");
  MapStore store(MapStoreConfig{root});
  ActivationCoordinator activation(store);
  const MapIdentity first = activation.IdentityFor("map_a");
  const auto staged = activation.Execute(
      Request("stage-first", ActivationOperation::kStage, first, {}));
  assert(staged.accepted && staged.changed);

  const auto metadata_path = root / "map_a" / "metadata.json";
  const auto first_time = std::filesystem::last_write_time(metadata_path);
  Write(
      metadata_path,
      "{\"frame_id\":\"map\",\"created_at\":\"replacement\",\"artifacts\":{"
      "\"map_pcd\":{\"path\":\"map.pcd\"},"
      "\"octomap\":{\"path\":\"octomap.ot\"},"
      "\"occupancy_grid\":{\"path\":\"occupancy.npz\"}}}");
  std::filesystem::last_write_time(metadata_path, first_time + std::chrono::seconds(1));

  ActivationRequest replacement = activation.PrepareStage("map_a");
  replacement.request_id = "stage-replacement";
  assert(replacement.target.content_epoch == first.content_epoch);
  assert(replacement.previous == first);
  const auto restaged = activation.Execute(replacement);
  assert(restaged.accepted);
  assert(!restaged.changed);
  assert(restaged.message == "already_staged");
  assert(restaged.active == first);
  std::filesystem::remove_all(root);
}

void TestPublishingEpochRecoveryKeepsCommittedGeneration() {
  const auto root = TempRoot();
  CreateReadyMap(root, "map_a");
  MapStore store(MapStoreConfig{root});
  const auto map_dir = root / "map_a";
  const std::int64_t base_epoch = store.ContentEpoch("map_a");
  const std::int64_t next_epoch = store.AllocateContentEpoch();
  assert(next_epoch > base_epoch);

  const std::string build_id = "RECOVER_PUBLISHED_1";
  const auto transaction_dir = map_dir / ".builds" / (build_id + "_transaction");
  Write(transaction_dir / "backup" / "occupancy.npz", "old occupancy");
  Write(transaction_dir / "backup" / "map.pgm", "old pgm");
  Write(transaction_dir / "backup" / MapStore::ContentEpochFilename(),
        std::to_string(base_epoch) + "\n");
  Write(map_dir / "occupancy.npz", "new occupancy");
  Write(map_dir / "map.pgm", "new pgm");
  Write(map_dir / MapStore::ContentEpochFilename(), std::to_string(next_epoch) + "\n");
  Write(
      transaction_dir / "transaction.state",
      "LINGTU_MAP_TRANSACTION\t1\n"
      "map_id\tmap_a\n"
      "phase\tPUBLISHING\n"
      "base_epoch\t" + std::to_string(base_epoch) + "\n"
      "next_epoch\t" + std::to_string(next_epoch) + "\n"
      "artifact\toccupancy.npz\t1\n"
      "artifact\tmap.pgm\t1\n"
      "artifact\t.content_epoch\t1\n");
  Write(map_dir / ".build_lock" / "owner.state", "owner=stale-build\npid=2147483646\n");
  Write(map_dir / ".build_lock" / "metadata.txt", build_id + "\nOCCUPANCY_SNAPSHOT\n");

  lingtu::maps::MapPipelineCore recovery_pipeline(store);
  assert(Read(map_dir / "occupancy.npz") == "new occupancy");
  assert(Read(map_dir / "map.pgm") == "new pgm");
  assert(store.ContentEpoch("map_a") == next_epoch);
  assert(!std::filesystem::exists(map_dir / ".build_lock"));
  assert(!std::filesystem::exists(transaction_dir));
  std::filesystem::remove_all(root);
}

void TestCommittedManifestFailureDoesNotRollback() {
  const auto root = TempRoot();
  CreateReadyMap(root, "map_a");
  MapStore store(MapStoreConfig{root});
  lingtu::maps::MapPipelineCore pipeline(store);
  const std::int64_t before_epoch = store.ContentEpoch("map_a");
  Write(root / "map_a" / "occupancy.npz", "old occupancy");
  Write(root / "map_a" / "map.pgm", "old pgm");
  Write(root / "map_a" / "map.yaml", "old yaml");

  SetEnvVar("LINGTU_MAPS_INJECT_COMMITTED_MANIFEST_FAILURE", "1");
  const auto result = pipeline.BuildOccupancySnapshotJson("map_a");
  UnsetEnvVar("LINGTU_MAPS_INJECT_COMMITTED_MANIFEST_FAILURE");

  assert(lingtu::maps::JsonObjectBoolAtPath(result, {"success"}) == true);
  assert(store.ContentEpoch("map_a") > before_epoch);
  assert(std::filesystem::is_regular_file(root / "map_a" / "occupancy.npz"));
  assert(std::filesystem::is_regular_file(root / "map_a" / "map.pgm"));
  assert(std::filesystem::is_regular_file(root / "map_a" / "map.yaml"));
  assert(Read(root / "map_a" / "occupancy.npz") != "old occupancy");
  assert(Read(root / "map_a" / "map.pgm") != "old pgm");
  assert(Read(root / "map_a" / "map.yaml") != "old yaml");
  assert(!std::filesystem::exists(root / "map_a" / ".build_lock"));
  std::filesystem::remove_all(root);
}

void TestRollbackIncompleteRetainsTransaction() {
  const auto root = TempRoot();
  CreateReadyMap(root, "map_a");
  MapStore store(MapStoreConfig{root});
  lingtu::maps::MapPipelineCore pipeline(store);
  const std::int64_t before_epoch = store.ContentEpoch("map_a");

  SetEnvVar("LINGTU_MAPS_INJECT_PUBLISH_FAILURE_AFTER", "1");
  SetEnvVar("LINGTU_MAPS_INJECT_ROLLBACK_FAILURE", "1");
  const auto result = pipeline.BuildOccupancySnapshotJson("map_a");
  UnsetEnvVar("LINGTU_MAPS_INJECT_ROLLBACK_FAILURE");
  UnsetEnvVar("LINGTU_MAPS_INJECT_PUBLISH_FAILURE_AFTER");

  assert(lingtu::maps::JsonObjectBoolAtPath(result, {"success"}) == false);
  assert(lingtu::maps::JsonObjectBoolAtPath(result, {"rolled_back"}) == false);
  assert(store.ContentEpoch("map_a") == before_epoch);
  const auto lock_dir = root / "map_a" / ".build_lock";
  assert(std::filesystem::is_directory(lock_dir));
  const std::string lock_text = Read(lock_dir / "metadata.txt");
  const auto line_end = lock_text.find('\n');
  assert(line_end != std::string::npos);
  const std::string build_id = lock_text.substr(0U, line_end);
  assert(std::filesystem::is_directory(
      root / "map_a" / ".builds" / (build_id + "_transaction")));
  std::filesystem::remove_all(root);
}

std::string ShellQuote(const std::string &value) {
#if defined(_WIN32)
  assert(value.find('"') == std::string::npos);
  return '"' + value + '"';
#else
  std::string quoted{"'"};
  for (const char character : value) {
    quoted += character == '\'' ? "'\"'\"'" : std::string(1U, character);
  }
  return quoted + "'";
#endif
}

std::string TrimLineEnd(std::string value) {
  while (!value.empty() && (value.back() == '\n' || value.back() == '\r')) {
    value.pop_back();
  }
  return value;
}

void TestMapctlPublicContract(const std::filesystem::path &mapctl) {
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto root = std::filesystem::absolute(mapctl).parent_path() /
                    ("mapctl_public_contract_blackbox_" + std::to_string(stamp));
  std::filesystem::create_directories(root);
  CreateReadyMap(root, "map_a");
  CreateReadyMap(root, "map_b");
  MapStore store(MapStoreConfig{root});
  assert(store.SetActiveMap("map_a", true).ok);
  ActivationCoordinator activation(store);
  const ActivationRequest expected_prepared = activation.PrepareStage("map_b");
  ProcessRunOptions options;
  options.timeout_sec = 30.0;
  std::string uppercase_command =
      ShellQuote(std::filesystem::absolute(mapctl).string()) +
      " STAGE map_b --map-root " + ShellQuote(root.string());
#if defined(_WIN32)
  uppercase_command = '"' + uppercase_command + '"';
#endif
  const auto uppercase_result = RunShellCommand(uppercase_command, options);
  assert(!uppercase_result.launch_failed);
  assert(!uppercase_result.timed_out);
  assert(uppercase_result.exit_code != 0);
  const std::string uppercase_json = TrimLineEnd(uppercase_result.stdout_text);
  assert(lingtu::maps::IsValidJsonObject(uppercase_json));
  assert(lingtu::maps::JsonObjectStringAtPath(uppercase_json, {"message"}) ==
         "operation must be prepare, stage, restore, or verify");

  std::string removed_offline_command =
      ShellQuote(std::filesystem::absolute(mapctl).string()) +
      " stage map_b --offline --map-root " + ShellQuote(root.string()) +
      " --request-id removed-offline --caller map-activation-test";
#if defined(_WIN32)
  removed_offline_command = '"' + removed_offline_command + '"';
#endif
  const auto removed_offline = RunShellCommand(removed_offline_command, options);
  assert(!removed_offline.launch_failed && !removed_offline.timed_out);
  assert(removed_offline.exit_code != 0);
  const std::string removed_offline_json = TrimLineEnd(removed_offline.stdout_text);
  assert(lingtu::maps::IsValidJsonObject(removed_offline_json));
  assert(lingtu::maps::JsonObjectStringAtPath(removed_offline_json, {"message"}) ==
         "unknown option: --offline");

  std::string removed_activate_command =
      ShellQuote(std::filesystem::absolute(mapctl).string()) +
      " build map_b --map-root " + ShellQuote(root.string()) + " --activate";
#if defined(_WIN32)
  removed_activate_command = '"' + removed_activate_command + '"';
#endif
  const auto removed_activate = RunShellCommand(removed_activate_command, options);
  assert(!removed_activate.launch_failed && !removed_activate.timed_out);
  assert(removed_activate.exit_code != 0);
  const std::string removed_activate_json = TrimLineEnd(removed_activate.stdout_text);
  assert(lingtu::maps::IsValidJsonObject(removed_activate_json));
  assert(lingtu::maps::JsonObjectStringAtPath(removed_activate_json, {"message"}) ==
         "unknown build option: --activate");
  assert(store.ActiveMapId() == "map_a");

  std::string lowercase_command =
      ShellQuote(std::filesystem::absolute(mapctl).string()) +
      " stage missing_map --map-root " + ShellQuote(root.string()) +
      " --request-id parser-stage --caller map-activation-test";
#if defined(_WIN32)
  lowercase_command = '"' + lowercase_command + '"';
#endif
  const auto lowercase_result = RunShellCommand(lowercase_command, options);
  assert(!lowercase_result.launch_failed && !lowercase_result.timed_out);
  assert(lowercase_result.exit_code != 0);
  const std::string lowercase_json = TrimLineEnd(lowercase_result.stdout_text);
  assert(lingtu::maps::IsValidJsonObject(lowercase_json));
  assert(lingtu::maps::JsonObjectStringAtPath(lowercase_json, {"request_id"}) ==
         "parser-stage");
  assert(lingtu::maps::JsonObjectStringAtPath(lowercase_json, {"operation"}) == "stage");
  assert(lingtu::maps::JsonObjectStringAtPath(lowercase_json, {"message"}) ==
         "target_identity_invalid");
  assert(store.ActiveMapId() == "map_a");

  std::string prepare_command =
      ShellQuote(std::filesystem::absolute(mapctl).string()) +
      " prepare map_b --map-root " + ShellQuote(root.string()) +
      " --request-id prepare-map-b --caller map-activation-test";
#if defined(_WIN32)
  prepare_command = '"' + prepare_command + '"';
#endif
  const auto prepare_result = RunShellCommand(prepare_command, options);
  assert(!prepare_result.launch_failed && !prepare_result.timed_out);
  assert(prepare_result.exit_code == 0);
  const std::string prepare_json = TrimLineEnd(prepare_result.stdout_text);
  assert(lingtu::maps::IsValidJsonObject(prepare_json));
  assert(lingtu::maps::JsonObjectStringAtPath(prepare_json, {"request_id"}) ==
         "prepare-map-b");
  assert(lingtu::maps::JsonObjectStringAtPath(prepare_json, {"operation"}) == "prepare");
  assert(lingtu::maps::JsonObjectBoolAtPath(prepare_json, {"accepted"}) == true);
  assert(lingtu::maps::JsonObjectStringAtPath(prepare_json, {"message"}) == "prepared");
  assert(lingtu::maps::JsonObjectBoolAtPath(prepare_json, {"changed"}) == false);
  assert(lingtu::maps::JsonObjectStringAtPath(prepare_json, {"producer_boot_id"}) == "");
  assert(lingtu::maps::JsonObjectBoolAtPath(prepare_json, {"target", "present"}) == true);
  assert(lingtu::maps::JsonObjectStringAtPath(prepare_json, {"target", "map_id"}) == "map_b");
  assert(lingtu::maps::JsonObjectBoolAtPath(prepare_json, {"previous", "present"}) == true);
  assert(lingtu::maps::JsonObjectStringAtPath(prepare_json, {"previous", "map_id"}) == "map_a");
  assert(lingtu::maps::JsonObjectBoolAtPath(prepare_json, {"active", "present"}) == true);
  assert(lingtu::maps::JsonObjectStringAtPath(prepare_json, {"active", "map_id"}) == "map_a");
  const auto token = lingtu::maps::JsonObjectStringAtPath(prepare_json, {"activation_token"});
  assert(token.has_value() && !token->empty());
  const auto decoded = DecodeActivationToken(*token);
  assert(decoded.first == expected_prepared.target);
  assert(decoded.second == expected_prepared.previous);
  assert(store.ActiveMapId() == "map_a");
  std::filesystem::remove_all(root);
}

}  // namespace

int main(int argc, char **argv) {
  assert(argc == 1 || argc == 2);
  if (argc == 2) {
    TestMapctlPublicContract(argv[1]);
  }
  TestMetadataMtimeDoesNotChangeContentEpoch();
  TestPublishingEpochRecoveryKeepsCommittedGeneration();
  TestCommittedManifestFailureDoesNotRollback();
  TestRollbackIncompleteRetainsTransaction();
  const auto root = TempRoot();
  CreateReadyMap(root, "map_a");
  CreateReadyMap(root, "map_b");
  MapStore store(MapStoreConfig{root});
  ActivationCoordinator activation(store);

  const MapIdentity absent;
  const MapIdentity map_a = activation.IdentityFor("map_a");
  const MapIdentity map_b = activation.IdentityFor("map_b");
  assert(map_a.present);
  assert(map_a.content_epoch > 0);
  assert(map_a.content_epoch <= 9'007'199'254'740'991LL);
  assert(map_a.frame_id == "map");
  assert(map_a.artifacts.size() == 3U);

  const ActivationRequest prepared_a = activation.PrepareStage("map_a");
  assert(prepared_a.operation == ActivationOperation::kStage);
  assert(prepared_a.target == map_a);
  assert(prepared_a.previous == absent);
  assert(store.ActiveMapId().empty());

  const std::string activation_token = EncodeActivationToken(map_a, absent);
  const auto decoded = DecodeActivationToken(activation_token);
  assert(decoded.first == map_a);
  assert(decoded.second == absent);
  RequireInvalidToken(
      activation_token + ".trailing", "activation token payload is not base64url");
  RequireInvalidToken(
      "v2." + activation_token.substr(3U), "activation token version is unsupported");
  RequireInvalidToken(
      "v3.*", "activation token payload is not base64url");

  const auto stage_a =
      activation.Execute(Request("stage-a", ActivationOperation::kStage, map_a, absent));
  assert(stage_a.accepted);
  assert(stage_a.changed);
  assert(stage_a.active == map_a);
  assert(store.ActiveMapId() == "map_a");

  const ActivationRequest prepared_b = activation.PrepareStage("map_b");
  assert(prepared_b.target == map_b);
  assert(prepared_b.previous == map_a);
  assert(store.ActiveMapId() == "map_a");

  const auto verify_a =
      activation.Execute(Request("verify-a", ActivationOperation::kVerify, map_a, absent));
  assert(verify_a.accepted);
  assert(!verify_a.changed);

  const auto stage_b =
      activation.Execute(Request("stage-b", ActivationOperation::kStage, map_b, map_a));
  assert(stage_b.accepted);
  assert(stage_b.changed);
  assert(store.ActiveMapId() == "map_b");

  const auto stale_restore =
      activation.Execute(Request("stale-restore", ActivationOperation::kRestore, map_a, absent));
  assert(!stale_restore.accepted);
  assert(stale_restore.message == "stale_rollback");
  assert(store.ActiveMapId() == "map_b");

  const auto restore_a =
      activation.Execute(Request("restore-a", ActivationOperation::kRestore, map_b, map_a));
  assert(restore_a.accepted);
  assert(restore_a.changed);
  assert(restore_a.active == map_a);
  assert(store.ActiveMapId() == "map_a");

  const auto repeated_restore =
      activation.Execute(Request("restore-again", ActivationOperation::kRestore, map_b, map_a));
  assert(repeated_restore.accepted);
  assert(!repeated_restore.changed);
  assert(repeated_restore.message == "already_restored");
  assert(repeated_restore.active == map_a);

  const auto stale_other_restore = activation.Execute(
      Request("restore-stale-other", ActivationOperation::kRestore, map_b, absent));
  assert(!stale_other_restore.accepted);
  assert(stale_other_restore.message == "stale_rollback");

  const auto restore_absent =
      activation.Execute(Request("restore-absent", ActivationOperation::kRestore, map_a, absent));
  assert(restore_absent.accepted);
  assert(!restore_absent.active.present);
  assert(store.ActiveMapId().empty());
  const auto repeated_restore_absent = activation.Execute(
      Request("restore-absent-again", ActivationOperation::kRestore, map_a, absent));
  assert(repeated_restore_absent.accepted);
  assert(!repeated_restore_absent.changed);
  assert(repeated_restore_absent.message == "already_restored");

  Write(
      root / "map_b" / ".build_lock" / "owner.state",
      "owner=map-activation-test\npid=" + std::to_string(CurrentProcessIdValue()) + "\n");
  Write(root / "map_b" / ".build_lock" / "metadata.txt", "build-b\nOCCUPANCY_2D\n");
  const auto build_race_stage =
      activation.Execute(Request("build-race", ActivationOperation::kStage, map_b, absent));
  assert(!build_race_stage.accepted);
  assert(build_race_stage.message == "target_map_write_in_progress");
  assert(store.ActiveMapId().empty());
  std::filesystem::remove_all(root / "map_b" / ".build_lock");

  Write(
      root / "map_b" / ".build_lock" / "owner.state",
      "owner=stale-build\npid=2147483646\n");
  Write(root / "map_b" / ".build_lock" / "metadata.txt", "STALE_BUILD_1\nOCCUPANCY_2D\n");
  const auto blocked_stale_stage =
      activation.Execute(Request("blocked-stale-build", ActivationOperation::kStage, map_b, absent));
  assert(!blocked_stale_stage.accepted);
  assert(blocked_stale_stage.message == "target_map_write_in_progress");
  lingtu::maps::MapPipelineCore recovery_pipeline(store);
  assert(!std::filesystem::exists(root / "map_b" / ".build_lock"));
  const auto reclaimed_stage =
      activation.Execute(Request("reclaimed-build", ActivationOperation::kStage, map_b, absent));
  assert(reclaimed_stage.accepted);
  assert(store.ActiveMapId() == "map_b");
  assert(!std::filesystem::exists(root / "map_b" / ".build_lock"));
  const auto reclaimed_restore = activation.Execute(
      Request("reclaimed-build-restore", ActivationOperation::kRestore, map_b, absent));
  assert(reclaimed_restore.accepted);
  assert(store.ActiveMapId().empty());

  const MapIdentity before_mutation = activation.IdentityFor("map_b");
  const auto occupancy_rebuild = recovery_pipeline.BuildOccupancySnapshotJson("map_b");
  assert(lingtu::maps::JsonObjectBoolAtPath(occupancy_rebuild, {"success"}) == true);
  const MapIdentity after_mutation = activation.IdentityFor("map_b");
  assert(after_mutation.content_epoch > before_mutation.content_epoch);
  const auto mutated_stage = activation.Execute(
      Request("mutated-stage", ActivationOperation::kStage, after_mutation, absent));
  assert(mutated_stage.accepted);
  assert(mutated_stage.changed);
  assert(store.ActiveMapId() == "map_b");

  CreateReadyMap(root, "map_bad");
  const MapIdentity map_bad = activation.IdentityFor("map_bad");
  Write(root / "map_bad" / "map.pcd", "not a pcd\n");
  const auto bad_check = store.CheckMapActivation("map_bad");
  assert(!bad_check.ok);
  assert(!store.SetActiveMap("map_bad", true).ok);
  const auto bad_stage = activation.Execute(
      Request("bad-stage", ActivationOperation::kStage, map_bad, after_mutation));
  assert(!bad_stage.accepted);
  assert(bad_stage.message.find("artifact_gate_failed") == 0U);
  assert(store.ActiveMapId() == "map_b");

  std::filesystem::remove_all(root);
  return 0;
}

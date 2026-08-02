#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

#include "lingtu/maps/mapd/activation.hpp"

namespace {

using lingtu::maps::MapStore;
using lingtu::maps::MapStoreConfig;
using lingtu::maps::mapd::ActivationCoordinator;
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

void CreateReadyMap(const std::filesystem::path &root, const std::string &id) {
  constexpr const char *kXHash = "2d711642b726b04401627ca9fbac32f5c8530fb1903cc4db02258717921a4881";
  const auto map = root / id;
  Write(map / "map.pcd", "x");
  Write(map / "occupancy.npz", "x");
  Write(map / "metadata.json", std::string{"{\"frame_id\":\"map\",\"artifacts\":{"} +
                                   "\"map_pcd\":{\"path\":\"map.pcd\",\"sha256\":\"" + kXHash +
                                   "\"}," +
                                   "\"occupancy_grid\":{\"path\":\"occupancy.npz\",\"sha256\":\"" +
                                   kXHash + "\",\"source_map_sha256\":\"" + kXHash + "\"}}}");
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

}  // namespace

int main() {
  const auto root = TempRoot();
  CreateReadyMap(root, "map_a");
  CreateReadyMap(root, "map_b");
  MapStore store(MapStoreConfig{root});
  ActivationCoordinator activation(store);

  const MapIdentity absent;
  const MapIdentity map_a = activation.IdentityFor("map_a");
  const MapIdentity map_b = activation.IdentityFor("map_b");
  assert(map_a.present);
  assert(map_a.version_id == "map_a:v1");
  assert(map_a.frame_id == "map");
  assert(map_a.artifacts.size() == 2U);

  const auto stage_a =
      activation.Execute(Request("stage-a", ActivationOperation::kStage, map_a, absent));
  assert(stage_a.accepted);
  assert(stage_a.changed);
  assert(stage_a.active == map_a);
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
  assert(!repeated_restore.accepted);
  assert(repeated_restore.message == "stale_rollback");

  const auto restore_absent =
      activation.Execute(Request("restore-absent", ActivationOperation::kRestore, map_a, absent));
  assert(restore_absent.accepted);
  assert(!restore_absent.active.present);
  assert(store.ActiveMapId().empty());

  const MapIdentity before_mutation = activation.IdentityFor("map_b");
  Write(root / "map_b" / "map.pcd", "changed");
  const auto mutated_stage = activation.Execute(
      Request("mutated-stage", ActivationOperation::kStage, before_mutation, absent));
  assert(!mutated_stage.accepted);
  assert(mutated_stage.message == "target_identity_mismatch");
  assert(store.ActiveMapId().empty());

  std::filesystem::remove_all(root);
  return 0;
}

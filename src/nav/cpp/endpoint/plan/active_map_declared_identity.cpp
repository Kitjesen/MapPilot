#include "plan/active_octomap_gate.hpp"
#include "plan/active_occupancy_gate.hpp"

#include <system_error>
#include <utility>

#include "lingtu/maps/store.hpp"

namespace lingtu::nav::endpoint {
namespace {

std::filesystem::path canonicalPath(const std::filesystem::path &path, std::string *error) {
  std::error_code ec;
  const auto canonical = std::filesystem::canonical(path, ec);
  if (ec) {
    if (error != nullptr) {
      *error = "cannot resolve " + path.string() + ": " + ec.message();
    }
    return {};
  }
  return canonical;
}

std::filesystem::path inferMapRoot(const std::filesystem::path &configured_path,
                                   std::string *error) {
  std::error_code ec;
  auto current = std::filesystem::absolute(configured_path, ec).parent_path();
  if (ec) {
    if (error != nullptr) {
      *error = "cannot make configured map path absolute: " + ec.message();
    }
    return {};
  }
  while (!current.empty()) {
    if (std::filesystem::is_regular_file(current / "active_map.txt", ec) && !ec) {
      return current;
    }
    ec.clear();
    const auto parent = current.parent_path();
    if (parent == current) {
      break;
    }
    current = parent;
  }
  if (error != nullptr) {
    *error = "cannot infer map root containing active_map.txt from " + configured_path.string();
  }
  return {};
}

std::filesystem::path weaklyCanonical(const std::filesystem::path &path,
                                      const std::string &label,
                                      std::string *error) {
  std::error_code ec;
  const auto canonical = std::filesystem::weakly_canonical(path, ec);
  if (ec) {
    if (error != nullptr) {
      *error = "cannot resolve " + label + ": " + ec.message();
    }
    return {};
  }
  return canonical;
}

lingtu::nav::plan::MapIdentity toPlanIdentity(
    const lingtu::maps::DeclaredArtifactIdentity &declared) {
  lingtu::nav::plan::MapIdentity identity;
  identity.map_id = declared.map_id;
  identity.version = declared.version;
  identity.artifact_sha256 = declared.artifact_sha256;
  identity.frame_id = declared.frame_id;
  return identity;
}

}  // namespace

ActiveOctomapGate::ActiveOctomapGate(std::filesystem::path map_root, std::string expected_frame_id)
    : map_root_(std::move(map_root)), expected_frame_id_(std::move(expected_frame_id)) {}

ActiveOctomapGate::~ActiveOctomapGate() = default;

lingtu::maps::MapStore *ActiveOctomapGate::resolveStore(
    const std::filesystem::path &configured_octomap_path, std::string *error) const {
  const auto root = map_root_.empty() ? inferMapRoot(configured_octomap_path, error)
                                      : canonicalPath(map_root_, error);
  if (root.empty()) {
    return nullptr;
  }
  if (!map_store_ || resolved_map_root_ != root) {
    lingtu::maps::MapStoreConfig config;
    config.root_dir = root;
    map_store_ = std::make_unique<lingtu::maps::MapStore>(std::move(config));
    resolved_map_root_ = root;
    cached_artifact_.reset();
  }
  return map_store_.get();
}

ActiveOctomapIdentityResult ActiveOctomapGate::currentDeclaredIdentity(
    const std::filesystem::path &configured_octomap_path) const {
  if (configured_octomap_path.empty()) {
    return {std::nullopt, "configured octomap path is empty"};
  }
  std::lock_guard<std::mutex> lock(mutex_);
  std::string error;
  auto *store = resolveStore(configured_octomap_path, &error);
  if (store == nullptr) {
    return {std::nullopt, error};
  }
  const std::string active_map = store->ActiveMapId();
  if (active_map.empty()) {
    return {std::nullopt, "native Maps store has no active map"};
  }
  auto declared = store->ReadDeclaredArtifactIdentity(
      active_map, lingtu::maps::ArtifactType::kOctomap3D, expected_frame_id_);
  if (!declared.ok()) {
    return {std::nullopt, std::move(declared.reason)};
  }
  const auto configured = weaklyCanonical(configured_octomap_path, "configured octomap", &error);
  if (configured.empty()) {
    return {std::nullopt, error};
  }
  const auto active = weaklyCanonical(declared.identity->artifact_path, "active octomap", &error);
  if (active.empty()) {
    return {std::nullopt, error};
  }
  if (configured != active) {
    return {std::nullopt, "configured octomap is not the declared active map artifact"};
  }
  auto identity = toPlanIdentity(*declared.identity);
  if (!identity.valid()) {
    return {std::nullopt, "active map has an incomplete declared planning identity"};
  }
  return {identity, {}};
}

ActiveOccupancyGate::ActiveOccupancyGate(std::filesystem::path map_root,
                                         std::string expected_frame_id)
    : map_root_(std::move(map_root)), expected_frame_id_(std::move(expected_frame_id)) {}

ActiveOccupancyGate::~ActiveOccupancyGate() = default;

lingtu::maps::MapStore *ActiveOccupancyGate::resolveStore(
    const std::filesystem::path &configured_occupancy_path, std::string *error) const {
  const auto root = map_root_.empty() ? inferMapRoot(configured_occupancy_path, error)
                                      : canonicalPath(map_root_, error);
  if (root.empty()) {
    return nullptr;
  }
  if (!map_store_ || resolved_map_root_ != root) {
    map_store_ = std::make_unique<lingtu::maps::MapStore>(lingtu::maps::MapStoreConfig{root});
    resolved_map_root_ = root;
    cached_artifact_.reset();
    next_generation_ = 1U;
  }
  return map_store_.get();
}

ActiveOccupancyIdentityResult ActiveOccupancyGate::currentDeclaredIdentity(
    const std::filesystem::path &configured_occupancy_path) const {
  if (configured_occupancy_path.empty()) {
    return {std::nullopt, "configured occupancy path is empty"};
  }
  std::lock_guard<std::mutex> lock(mutex_);
  std::string error;
  auto *store = resolveStore(configured_occupancy_path, &error);
  if (store == nullptr) {
    return {std::nullopt, error};
  }
  const std::string active_map = store->ActiveMapId();
  if (active_map.empty()) {
    return {std::nullopt, "native Maps store has no active map"};
  }
  auto declared = store->ReadDeclaredArtifactIdentity(
      active_map, lingtu::maps::ArtifactType::kOccupancy2D, expected_frame_id_);
  if (!declared.ok()) {
    return {std::nullopt, std::move(declared.reason)};
  }
  const auto configured = weaklyCanonical(configured_occupancy_path, "configured occupancy", &error);
  if (configured.empty()) {
    return {std::nullopt, error};
  }
  const auto active = weaklyCanonical(declared.identity->artifact_path, "active occupancy", &error);
  if (active.empty()) {
    return {std::nullopt, error};
  }
  if (configured != active) {
    return {std::nullopt, "configured occupancy is not the declared active map artifact"};
  }
  auto identity = toPlanIdentity(*declared.identity);
  if (!identity.valid()) {
    return {std::nullopt, "active map has an incomplete declared occupancy identity"};
  }
  return {identity, {}};
}

}  // namespace lingtu::nav::endpoint

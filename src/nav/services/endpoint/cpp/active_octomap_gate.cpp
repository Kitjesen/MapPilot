#include "active_octomap_gate.hpp"

#include "octoplanner3d_core.hpp"

#include "lingtu/maps/hash.hpp"
#include "lingtu/maps/store.hpp"

#include <atomic>
#include <chrono>
#include <stdexcept>
#include <system_error>
#include <utility>

#if defined(__unix__) || defined(__APPLE__)
#include <unistd.h>
#endif

namespace lingtu::nav::endpoint {
namespace {

using lingtu::maps::ArtifactValidationOptions;
using lingtu::maps::ArtifactValidationResult;
using lingtu::maps::MapStore;
using lingtu::maps::MapStoreConfig;
using lingtu::maps::Sha256File;

std::filesystem::path canonicalPath(
    const std::filesystem::path& path,
    std::string* error) {
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

std::filesystem::path inferMapRoot(
    const std::filesystem::path& configured_path,
    std::string* error) {
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
    *error = "cannot infer map root containing active_map.txt from " +
        configured_path.string();
  }
  return {};
}

std::string validationFailure(const ArtifactValidationResult& validation) {
  std::string out = "native Maps validation rejected active octomap";
  bool first = true;
  for (const auto& blocker : validation.blockers) {
    out += first ? ": " : "; ";
    out += blocker;
    first = false;
  }
  return out;
}

std::filesystem::path copyPrivateSnapshot(
    const std::filesystem::path& source,
    std::string* error) {
  static std::atomic<std::uint64_t> sequence{0U};
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
#if defined(__unix__) || defined(__APPLE__)
  const auto process_id = static_cast<unsigned long long>(::getpid());
#else
  const auto process_id = 0ULL;
#endif
  for (int attempt = 0; attempt < 16; ++attempt) {
    const auto name = "lingtu-nav-octomap-" + std::to_string(process_id) + "-" +
        std::to_string(stamp) + "-" + std::to_string(sequence.fetch_add(1U)) + ".ot";
    const auto target = std::filesystem::temp_directory_path() / name;
    std::error_code ec;
    if (!std::filesystem::copy_file(
            source,
            target,
            std::filesystem::copy_options::none,
            ec)) {
      if (ec == std::errc::file_exists) {
        continue;
      }
      if (error != nullptr) {
        *error = "failed to snapshot validated octomap: " + ec.message();
      }
      return {};
    }
    std::filesystem::permissions(
        target,
        std::filesystem::perms::owner_read,
        std::filesystem::perm_options::replace,
        ec);
    if (ec) {
      std::filesystem::remove(target);
      if (error != nullptr) {
        *error = "failed to make octomap snapshot read-only: " + ec.message();
      }
      return {};
    }
    return target;
  }
  if (error != nullptr) {
    *error = "failed to allocate a unique private octomap snapshot";
  }
  return {};
}

}  // namespace

ValidatedOctomap::ValidatedOctomap(
    std::filesystem::path load_path,
    std::string map_id,
    std::string sha256)
    : load_path_(std::move(load_path)),
      map_id_(std::move(map_id)),
      sha256_(std::move(sha256)) {}

ValidatedOctomap::~ValidatedOctomap() {
  std::error_code ec;
  std::filesystem::remove(load_path_, ec);
}

ActiveOctomapGate::ActiveOctomapGate(
    std::filesystem::path map_root,
    std::string expected_frame_id)
    : map_root_(std::move(map_root)),
      expected_frame_id_(std::move(expected_frame_id)) {}

ActiveOctomapGate::~ActiveOctomapGate() = default;

ActiveOctomapGateResult ActiveOctomapGate::prepare(
    const std::filesystem::path& configured_octomap_path) const {
  if (configured_octomap_path.empty()) {
    return {nullptr, "configured octomap path is empty"};
  }
  std::string error;
  const auto root = map_root_.empty()
      ? inferMapRoot(configured_octomap_path, &error)
      : canonicalPath(map_root_, &error);
  if (root.empty()) {
    return {nullptr, error};
  }
  if (!map_store_ || resolved_map_root_ != root) {
    MapStoreConfig config;
    config.root_dir = root;
    map_store_ = std::make_unique<MapStore>(std::move(config));
    resolved_map_root_ = root;
  }

  auto& store = *map_store_;
  const std::string active_map = store.ActiveMapId();
  if (active_map.empty()) {
    return {nullptr, "native Maps store has no active map"};
  }
  ArtifactValidationOptions validation_options;
  validation_options.require_octomap = true;
  validation_options.require_runtime_planning_artifact = true;
  validation_options.expected_frame_id = expected_frame_id_;
  const auto first_validation = store.ValidateArtifacts(active_map, validation_options);
  if (!first_validation.ok) {
    return {nullptr, validationFailure(first_validation)};
  }
  const auto content = canonicalPath(first_validation.map_dir, &error);
  if (content.empty()) {
    return {nullptr, error};
  }
  const auto active_octomap = canonicalPath(first_validation.octomap.path, &error);
  if (active_octomap.empty()) {
    return {nullptr, error};
  }
  if (active_octomap.parent_path() != content) {
    return {nullptr, "active octomap resolves outside the active map content directory"};
  }
  const auto configured = canonicalPath(configured_octomap_path, &error);
  if (configured.empty()) {
    return {nullptr, error};
  }
  if (configured != active_octomap) {
    return {
        nullptr,
        "configured octomap is not the native active map artifact: configured=" +
            configured.string() + ", active=" + active_octomap.string(),
    };
  }

  auto snapshot = copyPrivateSnapshot(active_octomap, &error);
  if (snapshot.empty()) {
    return {nullptr, error};
  }
  const auto cleanup_snapshot = [&]() {
    std::error_code ec;
    std::filesystem::permissions(
        snapshot,
        std::filesystem::perms::owner_write,
        std::filesystem::perm_options::add,
        ec);
    ec.clear();
    std::filesystem::remove(snapshot, ec);
  };
  std::string snapshot_sha;
  try {
    snapshot_sha = Sha256File(snapshot);
  } catch (const std::exception& exc) {
    cleanup_snapshot();
    return {nullptr, "failed to hash private octomap snapshot: " + std::string(exc.what())};
  }

  const auto second_validation = store.ValidateArtifacts(active_map, validation_options);
  if (!second_validation.ok) {
    cleanup_snapshot();
    return {nullptr, validationFailure(second_validation)};
  }
  const auto second_content = canonicalPath(second_validation.map_dir, &error);
  const auto second_octomap = canonicalPath(second_validation.octomap.path, &error);
  const auto configured_after = canonicalPath(configured_octomap_path, &error);
  const bool stable = !second_content.empty() &&
      !second_octomap.empty() &&
      !configured_after.empty() &&
      store.ActiveMapId() == active_map &&
      configured_after == second_octomap &&
      content == second_content &&
      active_octomap == second_octomap &&
      first_validation.map_pcd.sha256 == second_validation.map_pcd.sha256 &&
      first_validation.octomap.sha256 == second_validation.octomap.sha256 &&
      snapshot_sha == second_validation.octomap.sha256;
  if (!stable) {
    cleanup_snapshot();
    return {
        nullptr,
        "active map or artifact bytes changed during validation; refusing TOCTOU-prone load",
    };
  }
  return {
      std::unique_ptr<ValidatedOctomap>(
          new ValidatedOctomap(
              snapshot,
              active_map,
              snapshot_sha)),
      ""};
}

lingtu::nav::plan::GlobalPlanResult runWithActiveOctomap(
    const ActiveOctomapGate& gate,
    const lingtu::nav::plan::GlobalPlanRequest& request,
    const lingtu::nav::plan::GlobalPlanCancelCheck& cancel_check,
    const OctomapPlanner& planner) {
  if (cancel_check && cancel_check()) {
    lingtu::nav::plan::GlobalPlanResult result;
    result.cancelled = true;
    result.options = request.options;
    return result;
  }
  auto prepared = gate.prepare(request.map_path);
  if (!prepared.ok()) {
    throw std::runtime_error("active_octomap_gate_rejected: " + prepared.reason);
  }
  auto validated_request = request;
  validated_request.map_path = prepared.artifact->loadPath().string();
  return planner(validated_request, cancel_check);
}

lingtu::nav::plan::GlobalPlanResult runWithActiveOctomap(
    const ActiveOctomapGate& gate,
    const lingtu::nav::plan::GlobalPlanRequest& request,
    const lingtu::nav::plan::GlobalPlanCancelCheck& cancel_check) {
  return runWithActiveOctomap(
      gate,
      request,
      cancel_check,
      octoplanner3d::runtime::runPlan);
}

}  // namespace lingtu::nav::endpoint

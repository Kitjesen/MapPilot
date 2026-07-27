#include "plan/active_octomap_gate.hpp"

#include <atomic>
#include <chrono>
#include <stdexcept>
#include <system_error>
#include <utility>

#include "lingtu/maps/hash.hpp"
#include "lingtu/maps/store.hpp"
#include "nav/cpp/planning/global/octoplanner/octoplanner3d_core.hpp"

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

std::string validationFailure(const ArtifactValidationResult &validation) {
  std::string out = "native Maps validation rejected active octomap";
  bool first = true;
  for (const auto &blocker : validation.blockers) {
    out += first ? ": " : "; ";
    out += blocker;
    first = false;
  }
  return out;
}

struct ActiveSource {
  std::filesystem::path content_path;
  std::filesystem::path artifact_path;
  lingtu::nav::plan::MapIdentity identity;
  std::string source_map_sha256;
};

struct ActiveSourceResult {
  std::optional<ActiveSource> source;
  std::string reason;

  bool ok() const { return source.has_value(); }
};

ActiveSourceResult validateActiveSource(MapStore &store,
                                        const std::filesystem::path &configured_octomap_path,
                                        const std::string &expected_frame_id) {
  const std::string active_map = store.ActiveMapId();
  if (active_map.empty()) {
    return {std::nullopt, "native Maps store has no active map"};
  }

  ArtifactValidationOptions validation_options;
  validation_options.require_octomap = true;
  validation_options.require_runtime_planning_artifact = true;
  validation_options.expected_frame_id = expected_frame_id;
  const auto validation = store.ValidateArtifacts(active_map, validation_options);
  if (!validation.ok) {
    return {std::nullopt, validationFailure(validation)};
  }

  std::string error;
  const auto content = canonicalPath(validation.map_dir, &error);
  if (content.empty()) {
    return {std::nullopt, error};
  }
  const auto active_octomap = canonicalPath(validation.octomap.path, &error);
  if (active_octomap.empty()) {
    return {std::nullopt, error};
  }
  if (active_octomap.parent_path() != content) {
    return {
        std::nullopt,
        "active octomap resolves outside the active map content directory",
    };
  }
  const auto configured = canonicalPath(configured_octomap_path, &error);
  if (configured.empty()) {
    return {std::nullopt, error};
  }
  if (configured != active_octomap) {
    return {
        std::nullopt,
        "configured octomap is not the native active map artifact: configured=" +
            configured.string() + ", active=" + active_octomap.string(),
    };
  }

  lingtu::nav::plan::MapIdentity identity;
  identity.map_id = active_map;
  identity.version = store.CurrentVersion(active_map);
  identity.artifact_sha256 = validation.octomap.sha256;
  identity.frame_id = validation.checked_frame_id;
  if (!identity.valid()) {
    return {std::nullopt, "active map has an incomplete planning identity"};
  }
  return {
      ActiveSource{
          content,
          active_octomap,
          std::move(identity),
          validation.map_pcd.sha256,
      },
      "",
  };
}

std::filesystem::path copyPrivateSnapshot(const std::filesystem::path &source, std::string *error) {
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
    if (!std::filesystem::copy_file(source, target, std::filesystem::copy_options::none, ec)) {
      if (ec == std::errc::file_exists) {
        continue;
      }
      if (error != nullptr) {
        *error = "failed to snapshot validated octomap: " + ec.message();
      }
      return {};
    }
    std::filesystem::permissions(target, std::filesystem::perms::owner_read,
                                 std::filesystem::perm_options::replace, ec);
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

ValidatedOctomap::ValidatedOctomap(std::filesystem::path load_path,
                                   lingtu::nav::plan::MapIdentity identity)
    : load_path_(std::move(load_path)), identity_(std::move(identity)) {}

ValidatedOctomap::~ValidatedOctomap() {
  std::error_code ec;
  std::filesystem::remove(load_path_, ec);
}

ActiveOctomapGate::ActiveOctomapGate(std::filesystem::path map_root, std::string expected_frame_id)
    : map_root_(std::move(map_root)), expected_frame_id_(std::move(expected_frame_id)) {}

ActiveOctomapGate::~ActiveOctomapGate() = default;

MapStore *ActiveOctomapGate::resolveStore(const std::filesystem::path &configured_octomap_path,
                                          std::string *error) const {
  const auto root = map_root_.empty() ? inferMapRoot(configured_octomap_path, error)
                                      : canonicalPath(map_root_, error);
  if (root.empty()) {
    return nullptr;
  }
  if (!map_store_ || resolved_map_root_ != root) {
    MapStoreConfig config;
    config.root_dir = root;
    map_store_ = std::make_unique<MapStore>(std::move(config));
    resolved_map_root_ = root;
    cached_artifact_.reset();
  }
  return map_store_.get();
}

ActiveOctomapIdentityResult
ActiveOctomapGate::currentIdentity(const std::filesystem::path &configured_octomap_path) const {
  if (configured_octomap_path.empty()) {
    return {std::nullopt, "configured octomap path is empty"};
  }
  std::lock_guard<std::mutex> lock(mutex_);
  std::string error;
  auto *store = resolveStore(configured_octomap_path, &error);
  if (store == nullptr) {
    return {std::nullopt, error};
  }
  auto source = validateActiveSource(*store, configured_octomap_path, expected_frame_id_);
  if (!source.ok()) {
    return {std::nullopt, std::move(source.reason)};
  }
  return {source.source->identity, ""};
}

ActiveOctomapGateResult
ActiveOctomapGate::prepare(const std::filesystem::path &configured_octomap_path) const {
  if (configured_octomap_path.empty()) {
    return {nullptr, "configured octomap path is empty"};
  }
  std::lock_guard<std::mutex> lock(mutex_);
  std::string error;
  auto *store = resolveStore(configured_octomap_path, &error);
  if (store == nullptr) {
    return {nullptr, error};
  }
  auto first = validateActiveSource(*store, configured_octomap_path, expected_frame_id_);
  if (!first.ok()) {
    return {nullptr, std::move(first.reason)};
  }
  if (cached_artifact_ &&
      lingtu::nav::plan::sameMapIdentity(cached_artifact_->identity(), first.source->identity) &&
      std::filesystem::is_regular_file(cached_artifact_->loadPath())) {
    return {cached_artifact_, ""};
  }

  auto snapshot = copyPrivateSnapshot(first.source->artifact_path, &error);
  if (snapshot.empty()) {
    return {nullptr, error};
  }
  const auto cleanup_snapshot = [&]() {
    std::error_code ec;
    std::filesystem::permissions(snapshot, std::filesystem::perms::owner_write,
                                 std::filesystem::perm_options::add, ec);
    ec.clear();
    std::filesystem::remove(snapshot, ec);
  };
  std::string snapshot_sha;
  try {
    snapshot_sha = Sha256File(snapshot);
  } catch (const std::exception &exc) {
    cleanup_snapshot();
    return {nullptr, "failed to hash private octomap snapshot: " + std::string(exc.what())};
  }

  auto second = validateActiveSource(*store, configured_octomap_path, expected_frame_id_);
  if (!second.ok()) {
    cleanup_snapshot();
    return {nullptr, std::move(second.reason)};
  }
  const bool stable =
      first.source->content_path == second.source->content_path &&
      first.source->artifact_path == second.source->artifact_path &&
      lingtu::nav::plan::sameMapIdentity(first.source->identity, second.source->identity) &&
      first.source->source_map_sha256 == second.source->source_map_sha256 &&
      snapshot_sha == second.source->identity.artifact_sha256;
  if (!stable) {
    cleanup_snapshot();
    return {
        nullptr,
        "active map or artifact bytes changed during validation; refusing TOCTOU-prone load",
    };
  }
  cached_artifact_ = std::shared_ptr<const ValidatedOctomap>(
      new ValidatedOctomap(snapshot, std::move(second.source->identity)));
  return {cached_artifact_, ""};
}

lingtu::nav::plan::GlobalPlanResult runWithActiveOctomap(
    const ActiveOctomapGate &gate, const std::filesystem::path &configured_octomap_path,
    const lingtu::nav::plan::GlobalPlanRequest &request,
    const lingtu::nav::plan::GlobalPlanCancelCheck &cancel_check, const OctomapPlanner &planner) {
  if (cancel_check && cancel_check()) {
    lingtu::nav::plan::GlobalPlanResult result;
    result.cancelled = true;
    result.options = request.options;
    return result;
  }
  auto prepared = gate.prepare(configured_octomap_path);
  if (!prepared.ok()) {
    throw std::runtime_error("active_octomap_gate_rejected: " + prepared.reason);
  }
  auto result =
      planner(prepared.artifact->loadPath(), prepared.artifact->identity(), request, cancel_check);
  result.map_identity = prepared.artifact->identity();
  return result;
}

lingtu::nav::plan::GlobalPlanResult
runWithActiveOctomap(const ActiveOctomapGate &gate,
                     const std::filesystem::path &configured_octomap_path,
                     const lingtu::nav::plan::GlobalPlanRequest &request,
                     const lingtu::nav::plan::GlobalPlanCancelCheck &cancel_check) {
  return runWithActiveOctomap(
      gate, configured_octomap_path, request, cancel_check,
      [](const auto &map_path, const auto &, const auto &plan_request, const auto &cancel) {
        return octoplanner3d::runtime::runPlan(map_path, plan_request, cancel);
      });
}

}  // namespace lingtu::nav::endpoint

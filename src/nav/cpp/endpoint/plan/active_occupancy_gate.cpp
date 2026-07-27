#include "plan/active_occupancy_gate.hpp"

#include <atomic>
#include <chrono>
#include <limits>
#include <stdexcept>
#include <system_error>
#include <utility>

#include "lingtu/maps/build/grid_artifacts.hpp"
#include "lingtu/maps/hash.hpp"
#include "lingtu/maps/store.hpp"

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
  std::string out = "native Maps validation rejected active occupancy";
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
  plan::MapIdentity identity;
  std::string source_map_sha256;
};

struct ActiveSourceResult {
  std::optional<ActiveSource> source;
  std::string reason;

  bool ok() const { return source.has_value(); }
};

ActiveSourceResult validateActiveSource(MapStore &store,
                                        const std::filesystem::path &configured_occupancy_path,
                                        const std::string &expected_frame_id) {
  const std::string active_map = store.ActiveMapId();
  if (active_map.empty()) {
    return {std::nullopt, "native Maps store has no active map"};
  }

  ArtifactValidationOptions options;
  options.require_occupancy = true;
  options.require_runtime_planning_artifact = true;
  options.expected_frame_id = expected_frame_id;
  const auto validation = store.ValidateArtifacts(active_map, options);
  if (!validation.ok) {
    return {std::nullopt, validationFailure(validation)};
  }

  std::string error;
  const auto content = canonicalPath(validation.map_dir, &error);
  if (content.empty()) {
    return {std::nullopt, error};
  }
  const auto active_occupancy = canonicalPath(validation.occupancy_grid.path, &error);
  if (active_occupancy.empty()) {
    return {std::nullopt, error};
  }
  if (active_occupancy.parent_path() != content) {
    return {std::nullopt, "active occupancy resolves outside active map content"};
  }
  const auto configured = canonicalPath(configured_occupancy_path, &error);
  if (configured.empty()) {
    return {std::nullopt, error};
  }
  if (configured != active_occupancy) {
    return {
        std::nullopt,
        "configured occupancy is not the native active map artifact: configured=" +
            configured.string() + ", active=" + active_occupancy.string(),
    };
  }

  plan::MapIdentity identity;
  identity.map_id = active_map;
  identity.version = store.CurrentVersion(active_map);
  identity.artifact_sha256 = validation.occupancy_grid.sha256;
  identity.frame_id = validation.checked_frame_id;
  if (!identity.valid()) {
    return {std::nullopt, "active map has an incomplete occupancy identity"};
  }
  return {
      ActiveSource{
          content,
          active_occupancy,
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
    const auto name = "lingtu-nav-occupancy-" + std::to_string(process_id) + "-" +
                      std::to_string(stamp) + "-" + std::to_string(sequence.fetch_add(1U)) + ".npz";
    const auto target = std::filesystem::temp_directory_path() / name;
    std::error_code ec;
    if (!std::filesystem::copy_file(source, target, std::filesystem::copy_options::none, ec)) {
      if (ec == std::errc::file_exists) {
        continue;
      }
      if (error != nullptr) {
        *error = "failed to snapshot validated occupancy: " + ec.message();
      }
      return {};
    }
    return target;
  }
  if (error != nullptr) {
    *error = "failed to allocate a unique private occupancy snapshot";
  }
  return {};
}

plan::far::FarGridMap toFarMap(const lingtu::maps::OccupancyArtifactData &occupancy,
                               plan::MapIdentity identity, std::uint64_t generation) {
  plan::far::FarGridMap out;
  out.width = occupancy.cols;
  out.height = occupancy.rows;
  out.resolution_m = occupancy.resolution;
  out.origin_x_m = occupancy.origin_x;
  out.origin_y_m = occupancy.origin_y;
  out.frame_id = identity.frame_id;
  out.generation = generation;
  out.identity = std::move(identity);
  out.cells.resize(occupancy.grid.size(), plan::far::kUnknownCell);
  for (std::size_t i = 0; i < occupancy.grid.size(); ++i) {
    const auto value = occupancy.grid[i];
    out.cells[i] = value < 0     ? plan::far::kUnknownCell
                   : value >= 50 ? plan::far::kOccupiedCell
                                 : plan::far::kFreeCell;
  }
  out.Validate();
  return out;
}

}  // namespace

ActiveOccupancyGate::ActiveOccupancyGate(std::filesystem::path map_root,
                                         std::string expected_frame_id)
    : map_root_(std::move(map_root)), expected_frame_id_(std::move(expected_frame_id)) {}

ActiveOccupancyGate::~ActiveOccupancyGate() = default;

MapStore *ActiveOccupancyGate::resolveStore(const std::filesystem::path &configured_occupancy_path,
                                            std::string *error) const {
  const auto root = map_root_.empty() ? inferMapRoot(configured_occupancy_path, error)
                                      : canonicalPath(map_root_, error);
  if (root.empty()) {
    return nullptr;
  }
  if (!map_store_ || resolved_map_root_ != root) {
    map_store_ = std::make_unique<MapStore>(MapStoreConfig{root});
    resolved_map_root_ = root;
    cached_artifact_.reset();
    next_generation_ = 1U;
  }
  return map_store_.get();
}

ActiveOccupancyIdentityResult
ActiveOccupancyGate::currentIdentity(const std::filesystem::path &configured_occupancy_path) const {
  if (configured_occupancy_path.empty()) {
    return {std::nullopt, "configured occupancy path is empty"};
  }
  std::lock_guard<std::mutex> lock(mutex_);
  std::string error;
  auto *store = resolveStore(configured_occupancy_path, &error);
  if (store == nullptr) {
    return {std::nullopt, error};
  }
  auto source = validateActiveSource(*store, configured_occupancy_path, expected_frame_id_);
  if (!source.ok()) {
    return {std::nullopt, std::move(source.reason)};
  }
  return {source.source->identity, ""};
}

ActiveOccupancyGateResult
ActiveOccupancyGate::prepare(const std::filesystem::path &configured_occupancy_path) const {
  if (configured_occupancy_path.empty()) {
    return {nullptr, "configured occupancy path is empty"};
  }
  std::lock_guard<std::mutex> lock(mutex_);
  std::string error;
  auto *store = resolveStore(configured_occupancy_path, &error);
  if (store == nullptr) {
    return {nullptr, error};
  }
  auto first = validateActiveSource(*store, configured_occupancy_path, expected_frame_id_);
  if (!first.ok()) {
    return {nullptr, std::move(first.reason)};
  }
  if (cached_artifact_ &&
      plan::sameMapIdentity(cached_artifact_->identity(), first.source->identity)) {
    return {cached_artifact_, ""};
  }

  const auto snapshot = copyPrivateSnapshot(first.source->artifact_path, &error);
  if (snapshot.empty()) {
    return {nullptr, error};
  }
  const auto cleanup = [&snapshot]() {
    std::error_code ec;
    std::filesystem::remove(snapshot, ec);
  };

  lingtu::maps::OccupancyArtifactData occupancy;
  std::string snapshot_sha;
  try {
    snapshot_sha = Sha256File(snapshot);
    occupancy = lingtu::maps::LoadOccupancyArtifact(snapshot);
  } catch (const std::exception &exc) {
    cleanup();
    return {nullptr, "failed to load private occupancy snapshot: " + std::string(exc.what())};
  }

  auto second = validateActiveSource(*store, configured_occupancy_path, expected_frame_id_);
  const bool stable = second.ok() && first.source->content_path == second.source->content_path &&
                      first.source->artifact_path == second.source->artifact_path &&
                      plan::sameMapIdentity(first.source->identity, second.source->identity) &&
                      first.source->source_map_sha256 == second.source->source_map_sha256 &&
                      snapshot_sha == second.source->identity.artifact_sha256;
  if (!stable) {
    cleanup();
    return {
        nullptr,
        second.ok() ? "active map or occupancy bytes changed during validation"
                    : std::move(second.reason),
    };
  }
  if (next_generation_ == std::numeric_limits<std::uint64_t>::max()) {
    cleanup();
    return {nullptr, "occupancy generation counter exhausted"};
  }
  auto far_map = toFarMap(occupancy, second.source->identity, next_generation_++);
  cleanup();
  cached_artifact_ =
      std::shared_ptr<const ValidatedFarMap>(new ValidatedFarMap(std::move(far_map)));
  return {cached_artifact_, ""};
}

plan::GlobalPlanResult
runWithActiveOccupancy(const ActiveOccupancyGate &gate,
                       const std::filesystem::path &configured_occupancy_path,
                       plan::far::FarPlanner &planner, const plan::GlobalPlanRequest &request,
                       const plan::GlobalPlanCancelCheck &cancel_check) {
  if (cancel_check && cancel_check()) {
    plan::GlobalPlanResult result;
    result.cancelled = true;
    result.options = request.options;
    return result;
  }
  auto prepared = gate.prepare(configured_occupancy_path);
  if (!prepared.ok()) {
    throw std::runtime_error("active_occupancy_gate_rejected: " + prepared.reason);
  }
  planner.UpdateMap(prepared.artifact->map());
  auto bound_request = request;
  bound_request.map_identity = prepared.artifact->identity();
  bound_request.map_generation = prepared.artifact->generation();
  auto result = planner.Plan(bound_request, cancel_check);
  result.map_identity = prepared.artifact->identity();
  result.map_generation = prepared.artifact->generation();
  return result;
}

}  // namespace lingtu::nav::endpoint

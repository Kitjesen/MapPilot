#include "input/active/octomap.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <system_error>
#include <utility>

#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>

#include "collision/terrain.hpp"
#include "lingtu/maps/io/json.hpp"
#include "lingtu/maps/io/pcd.hpp"
#include "lingtu/maps/io/scans.hpp"
#include "lingtu/maps/storage/store.hpp"
#include "nav/cpp/planning/global/octoplanner/octoplanner3d_core.hpp"
#include "nav/cpp/platform/runtime.hpp"

namespace lingtu::nav::endpoint {
namespace {

std::filesystem::path copyPrivateSnapshot(const std::filesystem::path &source,
                                          std::string *error) {
  static std::atomic<std::uint64_t> sequence{0U};
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto process_id = lingtu::nav::platform::processId();
  for (int attempt = 0; attempt < 16; ++attempt) {
    const auto target = std::filesystem::temp_directory_path() /
                        ("lingtu-nav-octomap-" + std::to_string(process_id) + "-" +
                         std::to_string(stamp) + "-" +
                         std::to_string(sequence.fetch_add(1U)) + ".ot");
    std::error_code ec;
    if (!std::filesystem::copy_file(source, target, std::filesystem::copy_options::none, ec)) {
      if (ec == std::errc::file_exists) {
        continue;
      }
      if (error != nullptr) {
        *error = "failed to snapshot octomap: " + ec.message();
      }
      return {};
    }
    return target;
  }
  if (error != nullptr) {
    *error = "failed to allocate a private octomap snapshot";
  }
  return {};
}

bool sameFileState(const std::filesystem::path &path,
                   std::uintmax_t size,
                   std::filesystem::file_time_type modified) {
  std::error_code error;
  return std::filesystem::file_size(path, error) == size && !error &&
         std::filesystem::last_write_time(path, error) == modified && !error;
}

std::shared_ptr<const collision::Terrain> loadDeclaredTerrain(
    const std::filesystem::path &source, const std::filesystem::path &snapshot,
    const plan::MapIdentity &identity) {
  maps::MapStore store({source.parent_path().parent_path()});
  const auto declared_octomap = store.ReadDeclaredArtifactIdentity(
      identity.map_id, maps::ArtifactType::kOctomap3D, identity.frame_id);
  const auto pointcloud = store.ReadDeclaredArtifactIdentity(
      identity.map_id, maps::ArtifactType::kPointCloud, identity.frame_id);
  if (!declared_octomap.ok() || !pointcloud.ok() ||
      declared_octomap.identity->content_epoch != identity.content_epoch ||
      pointcloud.identity->content_epoch != identity.content_epoch ||
      declared_octomap.identity->artifact_path != source) return {};
  std::ifstream metadata_file(source.parent_path() / "metadata.json");
  const std::string metadata{std::istreambuf_iterator<char>(metadata_file), {}};
  // shortcut: Edited maps retain whole voxels until modified cells carry separate hit provenance.
  if (maps::JsonObjectHasPath(metadata, {"manual_voxel_edits"}) ||
      maps::JsonObjectBoolAtPath(metadata,
          {"artifacts", "octomap", "manual_voxel_edit"}).value_or(false)) return {};
  std::ifstream map_file(snapshot, std::ios::binary);
  std::string header;
  std::getline(map_file, header);
  std::unique_ptr<octomap::AbstractOcTree> tree;
  if (header == "# Octomap OcTree binary file") {
    auto binary = std::make_unique<octomap::OcTree>(1.0);
    if (!binary->readBinary(snapshot.string())) return {};
    tree = std::move(binary);
  } else tree.reset(octomap::AbstractOcTree::read(snapshot.string()));
  if (!tree) return {};
  const auto *octree = dynamic_cast<octomap::OcTree *>(tree.get());
  if (!octree) return {};
  const auto retained = maps::JsonObjectNumberAtPath(metadata,
      {"artifacts", "octomap", "stats", "retained_endpoints"});
  if (maps::JsonObjectStringAtPath(metadata,
          {"artifacts", "octomap", "evidence_source"}) == "saved_rays" &&
      maps::JsonObjectStringAtPath(metadata,
          {"artifacts", "octomap", "build_mode"}) == "native_octomap" &&
      maps::JsonObjectNumberAtPath(metadata,
          {"artifacts", "octomap", "stats", "dropped_endpoints"}) == 0 &&
      retained && *retained > 0) {
    std::vector<collision::Aabb> hits;
    hits.reserve(static_cast<std::size_t>(*retained));
    try {
      maps::VisitSavedScans(source.parent_path(), [&](const maps::SavedScan &scan) {
        for (std::size_t i = 0; i + 2 < scan.xyz.size(); i += 3) {
          const octomap::point3d hit(static_cast<float>(scan.xyz[i]),
              static_cast<float>(scan.xyz[i+1]),static_cast<float>(scan.xyz[i+2]));
          octomap::OcTreeKey key;
          if (!octree->coordToKeyChecked(hit,key)) continue;
          hits.push_back({{hit.x(),hit.y(),hit.z()}, {}});
        }
      });
      if (hits.size() == *retained && store.ContentEpoch(identity.map_id) == identity.content_epoch)
        return std::make_shared<const collision::Terrain>(std::move(hits), tree->getResolution());
    } catch (const std::runtime_error &) {
      // Missing saved scans keep the existing sampled-surface/voxel query.
    }
  }
  auto loaded = maps::LoadPcdXyz(pointcloud.identity->artifact_path);
  if (!loaded.ok || loaded.points.empty()) return {};
  if (store.ContentEpoch(identity.map_id) != identity.content_epoch) return {};
  std::vector<collision::Vec3> points;
  points.reserve(loaded.points.size());
  for (const auto &point : loaded.points) points.push_back({point.x, point.y, point.z});
  // This is only a sample index: the original subvoxel positions stay intact.
  return std::make_shared<const collision::Terrain>(std::move(points), tree->getResolution());
}

}  // namespace

ValidatedOctomap::ValidatedOctomap(std::filesystem::path load_path,
                                   lingtu::nav::plan::MapIdentity identity,
                                   std::filesystem::path source_path)
    : load_path_(std::move(load_path)), identity_(std::move(identity)),
      source_path_(std::move(source_path)) {}

const std::shared_ptr<const collision::Terrain> &ValidatedOctomap::terrain() const {
  std::call_once(terrain_once_, [&] {
    terrain_ = loadDeclaredTerrain(source_path_, load_path_, identity_);
  });
  return terrain_;
}

ValidatedOctomap::~ValidatedOctomap() {
  std::error_code error;
  std::filesystem::remove(load_path_, error);
}

ActiveOctomapGateResult ActiveOctomapGate::prepare(
    const std::filesystem::path &configured_octomap_path) const {
  const auto bound = currentIdentity(configured_octomap_path);
  if (!bound.ok()) {
    return {nullptr, bound.reason};
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (cached_artifact_ && std::filesystem::is_regular_file(cached_artifact_->loadPath())) {
    return {cached_artifact_, {}};
  }

  std::error_code error;
  const auto source = std::filesystem::canonical(configured_octomap_path, error);
  const auto size = std::filesystem::file_size(source, error);
  if (error) {
    return {nullptr, "cannot read configured octomap: " + error.message()};
  }
  const auto modified = std::filesystem::last_write_time(source, error);
  if (error) {
    return {nullptr, "cannot inspect configured octomap: " + error.message()};
  }
  std::string copy_error;
  auto snapshot = copyPrivateSnapshot(source, &copy_error);
  if (snapshot.empty()) {
    return {nullptr, copy_error};
  }
  if (!sameFileState(source, size, modified)) {
    std::filesystem::remove(snapshot, error);
    return {nullptr, "octomap changed while it was being loaded"};
  }
  cached_artifact_ = std::shared_ptr<const ValidatedOctomap>(
      new ValidatedOctomap(std::move(snapshot), *bound.identity, source));
  return {cached_artifact_, {}};
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
  auto bound_request = request;
  const bool contact_body = request.body && std::any_of(
      request.body->parts.begin(), request.body->parts.end(),
      [](const auto &part) { return part.support_contact; });
  bound_request.terrain = contact_body ? prepared.artifact->terrain() : nullptr;
  auto result = planner(prepared.artifact->loadPath(), prepared.artifact->identity(),
                        bound_request, cancel_check);
  result.map_identity = prepared.artifact->identity();
  return result;
}

lingtu::nav::plan::GlobalPlanResult runWithActiveOctomap(
    const ActiveOctomapGate &gate, const std::filesystem::path &configured_octomap_path,
    const lingtu::nav::plan::GlobalPlanRequest &request,
    const lingtu::nav::plan::GlobalPlanCancelCheck &cancel_check) {
  return runWithActiveOctomap(
      gate, configured_octomap_path, request, cancel_check,
      [](const auto &map_path, const auto &, const auto &plan_request, const auto &cancel) {
        return octoplanner3d::runtime::runPlan(map_path, plan_request, cancel);
      });
}

}  // namespace lingtu::nav::endpoint

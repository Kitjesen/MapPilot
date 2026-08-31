#include "input/active/octomap.hpp"

#include <atomic>
#include <chrono>
#include <stdexcept>
#include <system_error>
#include <utility>

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

}  // namespace

ValidatedOctomap::ValidatedOctomap(std::filesystem::path load_path,
                                   lingtu::nav::plan::MapIdentity identity)
    : load_path_(std::move(load_path)), identity_(std::move(identity)) {}

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
      new ValidatedOctomap(std::move(snapshot), *bound.identity));
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
  auto result =
      planner(prepared.artifact->loadPath(), prepared.artifact->identity(), request, cancel_check);
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

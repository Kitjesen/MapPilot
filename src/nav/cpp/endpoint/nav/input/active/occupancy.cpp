#include "input/active/occupancy.hpp"

#include <atomic>
#include <chrono>
#include <limits>
#include <stdexcept>
#include <system_error>

#include "lingtu/maps/build/grid_artifacts.hpp"
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
                        ("lingtu-nav-occupancy-" + std::to_string(process_id) + "-" +
                         std::to_string(stamp) + "-" +
                         std::to_string(sequence.fetch_add(1U)) + ".npz");
    std::error_code ec;
    if (!std::filesystem::copy_file(source, target, std::filesystem::copy_options::none, ec)) {
      if (ec == std::errc::file_exists) {
        continue;
      }
      if (error != nullptr) {
        *error = "failed to snapshot occupancy: " + ec.message();
      }
      return {};
    }
    return target;
  }
  if (error != nullptr) {
    *error = "failed to allocate a private occupancy snapshot";
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

plan::far_planner::FarGridMap toFarMap(const lingtu::maps::OccupancyArtifactData &occupancy,
                                       plan::MapIdentity identity,
                                       std::uint64_t generation) {
  plan::far_planner::FarGridMap out;
  out.width = occupancy.cols;
  out.height = occupancy.rows;
  out.resolution_m = occupancy.resolution;
  out.origin_x_m = occupancy.origin_x;
  out.origin_y_m = occupancy.origin_y;
  out.frame_id = identity.frame_id;
  out.generation = generation;
  out.identity = std::move(identity);
  out.cells.resize(occupancy.grid.size(), plan::far_planner::kUnknownCell);
  for (std::size_t i = 0; i < occupancy.grid.size(); ++i) {
    const auto value = occupancy.grid[i];
    out.cells[i] = value < 0     ? plan::far_planner::kUnknownCell
                   : value >= 50 ? plan::far_planner::kOccupiedCell
                                 : plan::far_planner::kFreeCell;
  }
  out.Validate();
  return out;
}

}  // namespace

ActiveOccupancyGateResult ActiveOccupancyGate::prepare(
    const std::filesystem::path &configured_occupancy_path) const {
  const auto bound = currentIdentity(configured_occupancy_path);
  if (!bound.ok()) {
    return {nullptr, bound.reason};
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (cached_artifact_) {
    return {cached_artifact_, {}};
  }

  std::error_code error;
  const auto source = std::filesystem::canonical(configured_occupancy_path, error);
  const auto size = std::filesystem::file_size(source, error);
  if (error) {
    return {nullptr, "cannot read configured occupancy: " + error.message()};
  }
  const auto modified = std::filesystem::last_write_time(source, error);
  if (error) {
    return {nullptr, "cannot inspect configured occupancy: " + error.message()};
  }
  std::string copy_error;
  const auto snapshot = copyPrivateSnapshot(source, &copy_error);
  if (snapshot.empty()) {
    return {nullptr, copy_error};
  }
  const auto cleanup = [&]() {
    std::error_code ignored;
    std::filesystem::remove(snapshot, ignored);
  };

  lingtu::maps::OccupancyArtifactData occupancy;
  try {
    occupancy = lingtu::maps::LoadOccupancyArtifact(snapshot);
  } catch (const std::exception &exception) {
    cleanup();
    return {nullptr, "failed to load occupancy: " + std::string(exception.what())};
  }
  if (!sameFileState(source, size, modified)) {
    cleanup();
    return {nullptr, "occupancy changed while it was being loaded"};
  }
  if (next_generation_ == std::numeric_limits<std::uint64_t>::max()) {
    cleanup();
    return {nullptr, "occupancy generation counter exhausted"};
  }
  auto map = toFarMap(occupancy, *bound.identity, next_generation_++);
  cleanup();
  cached_artifact_ =
      std::shared_ptr<const ValidatedFarMap>(new ValidatedFarMap(std::move(map)));
  return {cached_artifact_, {}};
}

plan::GlobalPlanResult runWithActiveOccupancy(
    const ActiveOccupancyGate &gate,
    const std::filesystem::path &configured_occupancy_path,
    plan::far_planner::FarPlanner &planner,
    const plan::GlobalPlanRequest &request,
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

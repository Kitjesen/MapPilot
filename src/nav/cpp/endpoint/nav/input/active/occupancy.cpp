#include "input/active/occupancy.hpp"

#include <atomic>
#include <chrono>
#include <limits>
#include <stdexcept>
#include <system_error>

#include "lingtu/maps/build/grids.hpp"
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
  const auto generation = next_generation_++;
  cleanup();
  cached_artifact_ =
      std::shared_ptr<const ValidatedOccupancyMap>(
          new ValidatedOccupancyMap(std::move(occupancy), *bound.identity, generation));
  return {cached_artifact_, {}};
}

}  // namespace lingtu::nav::endpoint

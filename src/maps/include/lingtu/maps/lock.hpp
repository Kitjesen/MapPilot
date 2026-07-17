#pragma once

#include <chrono>
#include <filesystem>
#include <optional>
#include <string>

namespace lingtu::maps {

class MapLock {
 public:
  static std::optional<MapLock>
  TryAcquire(const std::filesystem::path &map_root, const std::string &map_id,
             const std::string &owner,
             std::chrono::seconds incomplete_owner_grace = std::chrono::seconds(5));

  MapLock(MapLock &&other) noexcept;
  MapLock &operator=(MapLock &&other) noexcept;
  ~MapLock();

  MapLock(const MapLock &) = delete;
  MapLock &operator=(const MapLock &) = delete;

  const std::filesystem::path &path() const noexcept { return path_; }

 private:
  explicit MapLock(std::filesystem::path path);
  void Release() noexcept;

  std::filesystem::path path_;
  bool owns_{false};
};

}  // namespace lingtu::maps

#pragma once
#include "ground/segmenter.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <unordered_map>
#include <vector>

namespace lingtu::maps::cleaning {
struct Point { float x{}, y{}, z{}, intensity{}; };
struct Config {
  float resolution_m{.10F};
  float ray_tolerance_m{.05F};
  float endpoint_margin_m{.15F};
  float confirm_radius_m{.05F};
  float free_neighborhood_m{.02F};
  float max_ray_length_m{40.F};
  std::uint32_t min_free_frames{3};
  std::size_t max_cells{500000};
};
// Offline representative-point experiment. Measured endpoints authorize rays;
// missing returns and elapsed time contribute no free evidence.
class IncrementalCleaner {
 public:
  explicit IncrementalCleaner(Config config = {});
  void Reset();
  void Observe(std::span<const Point> map_points, Point map_origin,
               std::span<const std::uint8_t> ground_mask = {});
  // Retained representatives for offline evaluation, not a complete map or an
  // arbitrary-point filter. Evidence capacity must not limit a production map.
  std::vector<Point> Snapshot() const;
  std::size_t Count() const;
  std::uint64_t RemovedCount() const { return removed_; }
  std::uint64_t CapacityRejections() const { return rejected_; }
 private:
  struct Key { int x{}, y{}, z{}; bool operator==(const Key&) const = default; };
  struct Hash { std::size_t operator()(Key k) const; };
  struct Record {
    Point point;
    std::uint64_t hit_stamp{}, free_stamp{};
    std::uint32_t hits{}, ground_hits{}, free_frames{};
    std::uint8_t coverage{};
    bool removed{};
  };
  Key ToKey(Point p) const;
  bool Supported(const Record& point) const;
  Config config_;
  std::unordered_map<Key, Record, Hash> cells_;
  ground::PatchworkGroundSegmenter ground_;
  std::uint64_t frame_{}, removed_{}, rejected_{};
};
}

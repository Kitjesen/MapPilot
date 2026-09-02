#include "lingtu/maps/build/grid_artifacts.hpp"
#include "lingtu/maps/build/occupancy_snapshot.hpp"
#include "lingtu/maps/build/pcd.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace {

std::filesystem::path TempRoot() {
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  auto root = std::filesystem::temp_directory_path() /
      ("lingtu_grid_artifact_io_test_" + std::to_string(stamp));
  std::filesystem::remove_all(root);
  std::filesystem::create_directories(root);
  return root;
}

std::vector<std::uint8_t> ReadBytes(const std::filesystem::path& path) {
  std::ifstream file(path, std::ios::binary);
  return std::vector<std::uint8_t>(
      std::istreambuf_iterator<char>(file),
      std::istreambuf_iterator<char>());
}

void WriteBytes(const std::filesystem::path& path, const std::vector<std::uint8_t>& bytes) {
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  file.write(
      reinterpret_cast<const char*>(bytes.data()),
      static_cast<std::streamsize>(bytes.size()));
  assert(file.good());
}

std::uint16_t ReadU16(const std::vector<std::uint8_t>& bytes, std::size_t offset) {
  assert(offset + 2U <= bytes.size());
  return static_cast<std::uint16_t>(bytes[offset]) |
      (static_cast<std::uint16_t>(bytes[offset + 1U]) << 8U);
}

void RequireRejected(const std::filesystem::path& path) {
  bool rejected = false;
  try {
    static_cast<void>(lingtu::maps::LoadOccupancyArtifact(path));
  } catch (const std::exception&) {
    rejected = true;
  }
  assert(rejected);
}

}  // namespace

int main() {
  const auto root = TempRoot();
  const std::vector<lingtu::maps::PointXyz> points = {
      {0.0F, 0.0F, 0.0F},
      {1.0F, 1.0F, 0.5F},
  };
  std::string write_error;
  assert(lingtu::maps::WriteBinaryXyzPcd(root / "map.pcd", points, &write_error));
  const auto built = lingtu::maps::BuildOccupancyProjectionSnapshot(root, true);
  assert(built.ok);
  const auto original = ReadBytes(root / "occupancy.npz");
  assert(!original.empty());
  const auto loaded = lingtu::maps::LoadOccupancyArtifact(root / "occupancy.npz");
  assert(loaded.rows > 0);
  assert(loaded.cols > 0);
  assert(built.occupied_count == 1);
  assert(built.free_count == 0);
  assert(built.unknown_count == built.rows * built.cols - 1);
  assert(std::count(loaded.grid.begin(), loaded.grid.end(), static_cast<std::int8_t>(100)) == 1);
  assert(std::count(loaded.grid.begin(), loaded.grid.end(), static_cast<std::int8_t>(0)) == 0);
  assert(std::count(loaded.grid.begin(), loaded.grid.end(), static_cast<std::int8_t>(-1)) ==
      built.rows * built.cols - 1);

  auto malformed_shape = original;
  const std::string shape_marker = "'shape': (";
  const auto marker = std::search(
      malformed_shape.begin(),
      malformed_shape.end(),
      shape_marker.begin(),
      shape_marker.end());
  assert(marker != malformed_shape.end());
  const auto marker_offset = static_cast<std::size_t>(marker - malformed_shape.begin());
  const auto comma = std::find(
      malformed_shape.begin() + static_cast<std::ptrdiff_t>(marker_offset + shape_marker.size()),
      malformed_shape.end(),
      static_cast<std::uint8_t>(','));
  assert(comma != malformed_shape.end());
  assert(comma + 1 != malformed_shape.end());
  assert(*(comma + 1) == static_cast<std::uint8_t>(' '));
  *comma = static_cast<std::uint8_t>('x');
  *(comma + 1) = static_cast<std::uint8_t>(',');
  const auto malformed_shape_path = root / "malformed_shape.npz";
  WriteBytes(malformed_shape_path, malformed_shape);
  RequireRejected(malformed_shape_path);

  auto bad_crc = original;
  assert(bad_crc.size() >= 30U);
  const auto name_length = static_cast<std::size_t>(ReadU16(bad_crc, 26U));
  const auto extra_length = static_cast<std::size_t>(ReadU16(bad_crc, 28U));
  const auto npy_offset = 30U + name_length + extra_length;
  assert(npy_offset + 10U <= bad_crc.size());
  const auto npy_header_length = static_cast<std::size_t>(ReadU16(bad_crc, npy_offset + 8U));
  const auto grid_payload = npy_offset + 10U + npy_header_length;
  assert(grid_payload < bad_crc.size());
  bad_crc[grid_payload] = bad_crc[grid_payload] == 0U ? 1U : 0U;
  const auto bad_crc_path = root / "bad_crc.npz";
  WriteBytes(bad_crc_path, bad_crc);
  RequireRejected(bad_crc_path);

  std::filesystem::remove_all(root);
  return 0;
}

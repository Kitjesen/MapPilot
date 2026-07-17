#include "lingtu/maps/build/occupancy_snapshot.hpp"

#include "lingtu/maps/build/pcd.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>

namespace lingtu::maps {
namespace {

constexpr double kResolution = 0.20;
constexpr double kZMinRel = 0.10;
constexpr double kZMaxRel = 2.00;

struct ZipEntry {
  std::string name;
  std::vector<std::uint8_t> data;
  std::uint32_t crc{0};
  std::uint32_t local_offset{0};
};

OccupancySnapshotResult Error(const std::string& message) {
  OccupancySnapshotResult result;
  result.ok = false;
  result.message = message;
  return result;
}

std::uint32_t Crc32(const std::vector<std::uint8_t>& data) {
  std::uint32_t crc = 0xffffffffU;
  for (const auto byte : data) {
    crc ^= byte;
    for (int bit = 0; bit < 8; ++bit) {
      const std::uint32_t mask = 0U - (crc & 1U);
      crc = (crc >> 1U) ^ (0xedb88320U & mask);
    }
  }
  return ~crc;
}

void PushU16(std::vector<std::uint8_t>* out, std::uint16_t value) {
  out->push_back(static_cast<std::uint8_t>(value & 0xffU));
  out->push_back(static_cast<std::uint8_t>((value >> 8U) & 0xffU));
}

void PushU32(std::vector<std::uint8_t>* out, std::uint32_t value) {
  out->push_back(static_cast<std::uint8_t>(value & 0xffU));
  out->push_back(static_cast<std::uint8_t>((value >> 8U) & 0xffU));
  out->push_back(static_cast<std::uint8_t>((value >> 16U) & 0xffU));
  out->push_back(static_cast<std::uint8_t>((value >> 24U) & 0xffU));
}

void PushBytes(std::vector<std::uint8_t>* out, const void* data, size_t size) {
  const auto* bytes = static_cast<const std::uint8_t*>(data);
  out->insert(out->end(), bytes, bytes + size);
}

void PushString(std::vector<std::uint8_t>* out, const std::string& value) {
  out->insert(out->end(), value.begin(), value.end());
}

std::string NpyHeader(
    const std::string& descr,
    const std::string& shape,
    bool fortran_order = false) {
  std::string dict = "{'descr': '" + descr + "', 'fortran_order': " +
      (fortran_order ? "True" : "False") + ", 'shape': " + shape + ", }";
  const size_t prefix = 10U;
  size_t padding = 16U - ((prefix + dict.size() + 1U) % 16U);
  if (padding == 16U) {
    padding = 0U;
  }
  dict.append(padding, ' ');
  dict.push_back('\n');
  return dict;
}

std::vector<std::uint8_t> MakeNpy(
    const std::string& descr,
    const std::string& shape,
    const void* payload,
    size_t payload_size) {
  const std::string header = NpyHeader(descr, shape);
  std::vector<std::uint8_t> out;
  out.reserve(10U + header.size() + payload_size);
  out.push_back(0x93U);
  PushString(&out, "NUMPY");
  out.push_back(1U);
  out.push_back(0U);
  PushU16(&out, static_cast<std::uint16_t>(header.size()));
  PushString(&out, header);
  PushBytes(&out, payload, payload_size);
  return out;
}

std::vector<std::uint8_t> MakeScalarF64Npy(double value) {
  return MakeNpy("<f8", "()", &value, sizeof(double));
}

std::vector<std::uint8_t> MakeOriginNpy(double origin_x, double origin_y) {
  const std::array<double, 2> origin{origin_x, origin_y};
  return MakeNpy("<f8", "(2,)", origin.data(), sizeof(double) * origin.size());
}

std::vector<std::uint8_t> MakeGridNpy(const std::vector<std::int8_t>& grid, int rows, int cols) {
  return MakeNpy("|i1", "(" + std::to_string(rows) + ", " + std::to_string(cols) + ")", grid.data(), grid.size());
}

bool WriteStoredZip(const std::filesystem::path& path, std::vector<ZipEntry> entries) {
  std::filesystem::create_directories(path.parent_path());
  std::vector<std::uint8_t> out;
  for (auto& entry : entries) {
    entry.crc = Crc32(entry.data);
    entry.local_offset = static_cast<std::uint32_t>(out.size());
    PushU32(&out, 0x04034b50U);
    PushU16(&out, 20U);
    PushU16(&out, 0U);
    PushU16(&out, 0U);
    PushU16(&out, 0U);
    PushU16(&out, 0U);
    PushU32(&out, entry.crc);
    PushU32(&out, static_cast<std::uint32_t>(entry.data.size()));
    PushU32(&out, static_cast<std::uint32_t>(entry.data.size()));
    PushU16(&out, static_cast<std::uint16_t>(entry.name.size()));
    PushU16(&out, 0U);
    PushString(&out, entry.name);
    PushBytes(&out, entry.data.data(), entry.data.size());
  }

  const std::uint32_t central_start = static_cast<std::uint32_t>(out.size());
  for (const auto& entry : entries) {
    PushU32(&out, 0x02014b50U);
    PushU16(&out, 20U);
    PushU16(&out, 20U);
    PushU16(&out, 0U);
    PushU16(&out, 0U);
    PushU16(&out, 0U);
    PushU16(&out, 0U);
    PushU32(&out, entry.crc);
    PushU32(&out, static_cast<std::uint32_t>(entry.data.size()));
    PushU32(&out, static_cast<std::uint32_t>(entry.data.size()));
    PushU16(&out, static_cast<std::uint16_t>(entry.name.size()));
    PushU16(&out, 0U);
    PushU16(&out, 0U);
    PushU16(&out, 0U);
    PushU16(&out, 0U);
    PushU32(&out, 0U);
    PushU32(&out, entry.local_offset);
    PushString(&out, entry.name);
  }
  const std::uint32_t central_size = static_cast<std::uint32_t>(out.size()) - central_start;
  PushU32(&out, 0x06054b50U);
  PushU16(&out, 0U);
  PushU16(&out, 0U);
  PushU16(&out, static_cast<std::uint16_t>(entries.size()));
  PushU16(&out, static_cast<std::uint16_t>(entries.size()));
  PushU32(&out, central_size);
  PushU32(&out, central_start);
  PushU16(&out, 0U);

  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  if (!file) {
    return false;
  }
  file.write(reinterpret_cast<const char*>(out.data()), static_cast<std::streamsize>(out.size()));
  return true;
}

bool WritePgm(const std::filesystem::path& path, const std::vector<std::int8_t>& grid, int rows, int cols) {
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  if (!file) {
    return false;
  }
  file << "P5\n" << cols << " " << rows << "\n255\n";
  for (int row = rows - 1; row >= 0; --row) {
    for (int col = 0; col < cols; ++col) {
      const auto value = grid[static_cast<size_t>(row * cols + col)];
      std::uint8_t out = 205U;
      if (value == 0) {
        out = 254U;
      } else if (value == 100) {
        out = 0U;
      }
      file.write(reinterpret_cast<const char*>(&out), sizeof(out));
    }
  }
  return true;
}

bool WriteYaml(
    const std::filesystem::path& path,
    double resolution,
    double origin_x,
    double origin_y) {
  std::ofstream file(path, std::ios::trunc);
  if (!file) {
    return false;
  }
  file
      << "image: map.pgm\n"
      << "resolution: " << resolution << "\n"
      << "origin: [" << origin_x << ", " << origin_y << ", 0.0]\n"
      << "negate: 0\n"
      << "occupied_thresh: 0.65\n"
      << "free_thresh: 0.196\n"
      << "mode: trinary\n";
  return true;
}

std::filesystem::path MakeStagingDir(const std::filesystem::path& map_dir) {
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch();
  const auto suffix =
      std::chrono::duration_cast<std::chrono::nanoseconds>(stamp).count();
  auto staging = map_dir / ".builds" /
      ("occupancy_snapshot_tmp_" + std::to_string(suffix));
  std::filesystem::create_directories(staging);
  return staging;
}

void CleanupStaging(const std::filesystem::path& staging) {
  std::error_code ec;
  std::filesystem::remove_all(staging, ec);
}

bool CommitStagedFile(
    const std::filesystem::path& staged,
    const std::filesystem::path& final_path,
    std::string* error) {
  std::error_code ec;
  std::filesystem::create_directories(final_path.parent_path(), ec);
  if (ec) {
    if (error != nullptr) {
      *error = "failed to create artifact directory: " + ec.message();
    }
    return false;
  }
  std::filesystem::remove(final_path, ec);
  ec.clear();
  std::filesystem::rename(staged, final_path, ec);
  if (ec) {
    if (error != nullptr) {
      *error = "failed to commit " + final_path.filename().string() + ": " +
          ec.message();
    }
    return false;
  }
  return true;
}

}  // namespace

OccupancySnapshotResult BuildOccupancyProjectionSnapshot(
    const std::filesystem::path& map_dir,
    bool output_is_staged) {
  const auto pcd_path = map_dir / "map.pcd";
  auto loaded = LoadPcdXyz(pcd_path);
  if (!loaded.ok || loaded.points.empty()) {
    return Error(loaded.message.empty() ? "PCD file empty or unparseable" : loaded.message);
  }

  std::vector<float> z_values;
  z_values.reserve(loaded.points.size());
  float min_x = std::numeric_limits<float>::infinity();
  float min_y = std::numeric_limits<float>::infinity();
  float max_x = -std::numeric_limits<float>::infinity();
  float max_y = -std::numeric_limits<float>::infinity();
  for (const auto& point : loaded.points) {
    z_values.push_back(point.z);
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
    max_x = std::max(max_x, point.x);
    max_y = std::max(max_y, point.y);
  }
  std::sort(z_values.begin(), z_values.end());
  const size_t z_index = static_cast<size_t>(
      std::floor(0.05 * static_cast<double>(z_values.size() - 1U)));
  const float ground_z = z_values[z_index];
  const double z_lo = static_cast<double>(ground_z) + kZMinRel;
  const double z_hi = static_cast<double>(ground_z) + kZMaxRel;

  const double border = kResolution;
  const double origin_x = static_cast<double>(min_x) - border;
  const double origin_y = static_cast<double>(min_y) - border;
  const int cols = static_cast<int>(std::ceil((static_cast<double>(max_x) + border - origin_x) / kResolution)) + 1;
  const int rows = static_cast<int>(std::ceil((static_cast<double>(max_y) + border - origin_y) / kResolution)) + 1;
  if (rows <= 0 || cols <= 0 || static_cast<int64_t>(rows) * static_cast<int64_t>(cols) > 25000000LL) {
    return Error("grid size out of range: " + std::to_string(rows) + "x" + std::to_string(cols));
  }

  std::vector<std::int8_t> grid(static_cast<size_t>(rows * cols), 0);
  for (const auto& point : loaded.points) {
    if (static_cast<double>(point.z) < z_lo || static_cast<double>(point.z) > z_hi) {
      continue;
    }
    const int col = static_cast<int>(std::floor((static_cast<double>(point.x) - origin_x) / kResolution));
    const int row = static_cast<int>(std::floor((static_cast<double>(point.y) - origin_y) / kResolution));
    if (row >= 0 && row < rows && col >= 0 && col < cols) {
      grid[static_cast<size_t>(row * cols + col)] = 100;
    }
  }

  OccupancySnapshotResult result;
  result.ok = true;
  result.occupancy_path = map_dir / "occupancy.npz";
  result.pgm_path = map_dir / "map.pgm";
  result.yaml_path = map_dir / "map.yaml";
  result.rows = rows;
  result.cols = cols;
  result.resolution = kResolution;
  result.origin_x = origin_x;
  result.origin_y = origin_y;
  for (const auto value : grid) {
    if (value < 0) {
      ++result.unknown_count;
    } else if (value >= 100) {
      ++result.occupied_count;
    } else {
      ++result.free_count;
    }
  }

  std::vector<ZipEntry> entries;
  entries.push_back({"grid.npy", MakeGridNpy(grid, rows, cols), 0U, 0U});
  entries.push_back({"resolution.npy", MakeScalarF64Npy(kResolution), 0U, 0U});
  entries.push_back({"origin.npy", MakeOriginNpy(origin_x, origin_y), 0U, 0U});
  const auto staging = output_is_staged ? map_dir : MakeStagingDir(map_dir);
  const auto cleanup_staging = [&]() {
    if (!output_is_staged) {
      CleanupStaging(staging);
    }
  };
  const auto staged_occupancy = staging / "occupancy.npz";
  const auto staged_pgm = staging / "map.pgm";
  const auto staged_yaml = staging / "map.yaml";
  if (!WriteStoredZip(staged_occupancy, std::move(entries))) {
    cleanup_staging();
    return Error("failed to write occupancy.npz");
  }
  if (!WritePgm(staged_pgm, grid, rows, cols)) {
    cleanup_staging();
    return Error("failed to write map.pgm");
  }
  if (!WriteYaml(staged_yaml, kResolution, origin_x, origin_y)) {
    cleanup_staging();
    return Error("failed to write map.yaml");
  }
  if (output_is_staged) {
    return result;
  }
  std::string commit_error;
  if (!CommitStagedFile(staged_occupancy, result.occupancy_path, &commit_error) ||
      !CommitStagedFile(staged_pgm, result.pgm_path, &commit_error) ||
      !CommitStagedFile(staged_yaml, result.yaml_path, &commit_error)) {
    cleanup_staging();
    return Error(commit_error);
  }
  cleanup_staging();
  return result;
}

}  // namespace lingtu::maps

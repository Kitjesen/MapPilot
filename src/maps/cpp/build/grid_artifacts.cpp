#include "lingtu/maps/build/grid_artifacts.hpp"

#include "lingtu/maps/layers/grid.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace lingtu::maps {
namespace {

struct ZipEntry {
  std::string name;
  std::vector<std::uint8_t> data;
  std::uint32_t crc{0};
  std::uint32_t local_offset{0};
};

struct NpyArray {
  std::string descr;
  std::vector<int> shape;
  std::vector<std::uint8_t> payload;
};

struct EsdfArray {
  std::vector<float> distance;
  std::vector<float> grad_x;
  std::vector<float> grad_y;
  int rows{0};
  int cols{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
};

GridArtifactResult Error(const std::string& message) {
  GridArtifactResult result;
  result.ok = false;
  result.message = message;
  return result;
}

std::uint16_t ReadU16(const std::vector<std::uint8_t>& data, size_t offset) {
  if (offset + 2U > data.size()) {
    throw std::runtime_error("unexpected EOF while reading u16");
  }
  return static_cast<std::uint16_t>(data[offset]) |
      (static_cast<std::uint16_t>(data[offset + 1U]) << 8U);
}

std::uint32_t ReadU32(const std::vector<std::uint8_t>& data, size_t offset) {
  if (offset + 4U > data.size()) {
    throw std::runtime_error("unexpected EOF while reading u32");
  }
  return static_cast<std::uint32_t>(data[offset]) |
      (static_cast<std::uint32_t>(data[offset + 1U]) << 8U) |
      (static_cast<std::uint32_t>(data[offset + 2U]) << 16U) |
      (static_cast<std::uint32_t>(data[offset + 3U]) << 24U);
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

std::vector<std::uint8_t> ReadFile(const std::filesystem::path& path) {
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    throw std::runtime_error("failed to open " + path.string());
  }
  const auto begin = std::istreambuf_iterator<char>(file);
  const auto end = std::istreambuf_iterator<char>();
  std::vector<std::uint8_t> bytes(begin, end);
  if (file.bad()) {
    throw std::runtime_error("failed to read " + path.string());
  }
  return bytes;
}

bool WriteFile(const std::filesystem::path& path, const std::vector<std::uint8_t>& data) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  if (!file) {
    return false;
  }
  file.write(reinterpret_cast<const char*>(data.data()), static_cast<std::streamsize>(data.size()));
  file.flush();
  return file.good();
}

std::map<std::string, std::vector<std::uint8_t>> ReadStoredNpz(
    const std::filesystem::path& path) {
  const auto bytes = ReadFile(path);
  std::map<std::string, std::vector<std::uint8_t>> out;
  size_t offset = 0U;
  while (offset + 4U <= bytes.size()) {
    const std::uint32_t sig = ReadU32(bytes, offset);
    if (sig == 0x02014b50U || sig == 0x06054b50U) {
      break;
    }
    if (sig != 0x04034b50U) {
      throw std::runtime_error("invalid npz local header in " + path.string());
    }
    const std::uint16_t flags = ReadU16(bytes, offset + 6U);
    const std::uint16_t method = ReadU16(bytes, offset + 8U);
    const std::uint32_t compressed = ReadU32(bytes, offset + 18U);
    const std::uint32_t uncompressed = ReadU32(bytes, offset + 22U);
    const std::uint16_t name_len = ReadU16(bytes, offset + 26U);
    const std::uint16_t extra_len = ReadU16(bytes, offset + 28U);
    if ((flags & 0x8U) != 0U) {
      throw std::runtime_error("npz data descriptors are not supported: " + path.string());
    }
    if (method != 0U) {
      throw std::runtime_error(
          "compressed npz is not supported by native maps builder; rebuild occupancy with native pipeline");
    }
    const size_t name_offset = offset + 30U;
    const size_t data_offset = name_offset + name_len + extra_len;
    if (data_offset > bytes.size() || compressed > bytes.size() - data_offset) {
      throw std::runtime_error("truncated npz entry in " + path.string());
    }
    if (compressed != uncompressed) {
      throw std::runtime_error("stored npz entry size mismatch in " + path.string());
    }
    std::string name(
        reinterpret_cast<const char*>(bytes.data() + name_offset),
        static_cast<size_t>(name_len));
    std::vector<std::uint8_t> entry(
        bytes.begin() + static_cast<std::ptrdiff_t>(data_offset),
        bytes.begin() + static_cast<std::ptrdiff_t>(data_offset + compressed));
    const std::uint32_t expected_crc = ReadU32(bytes, offset + 14U);
    if (Crc32(entry) != expected_crc) {
      throw std::runtime_error("npz entry CRC mismatch: " + name);
    }
    if (!out.emplace(name, std::move(entry)).second) {
      throw std::runtime_error("duplicate npz entry: " + name);
    }
    offset = data_offset + compressed;
  }
  return out;
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

std::vector<std::uint8_t> MakeGridF32Npy(const std::vector<float>& grid, int rows, int cols) {
  const std::string shape = "(" + std::to_string(rows) + ", " + std::to_string(cols) + ")";
  return MakeNpy("<f4", shape, grid.data(), sizeof(float) * grid.size());
}

bool WriteStoredZip(const std::filesystem::path& path, std::vector<ZipEntry> entries) {
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
  return WriteFile(path, out);
}

NpyArray ParseNpy(const std::vector<std::uint8_t>& bytes) {
  if (bytes.size() < 10U || bytes[0] != 0x93U ||
      std::memcmp(bytes.data() + 1U, "NUMPY", 5U) != 0) {
    throw std::runtime_error("invalid npy payload");
  }
  const std::uint8_t major = bytes[6U];
  size_t header_len = 0U;
  size_t header_offset = 0U;
  if (major == 1U) {
    header_len = ReadU16(bytes, 8U);
    header_offset = 10U;
  } else if (major == 2U || major == 3U) {
    header_len = ReadU32(bytes, 8U);
    header_offset = 12U;
  } else {
    throw std::runtime_error("unsupported npy version");
  }
  if (header_offset > bytes.size() || header_len > bytes.size() - header_offset) {
    throw std::runtime_error("truncated npy header");
  }
  const std::string header(
      reinterpret_cast<const char*>(bytes.data() + header_offset),
      header_len);
  auto findQuoted = [&](const std::string& key) -> std::string {
    const std::string marker = "'" + key + "': '";
    const auto start = header.find(marker);
    if (start == std::string::npos) {
      throw std::runtime_error("missing npy header field: " + key);
    }
    const auto value_start = start + marker.size();
    const auto value_end = header.find('\'', value_start);
    if (value_end == std::string::npos) {
      throw std::runtime_error("unterminated npy header field: " + key);
    }
    return header.substr(value_start, value_end - value_start);
  };
  const std::string descr = findQuoted("descr");
  const auto shape_key = header.find("'shape':");
  if (shape_key == std::string::npos) {
    throw std::runtime_error("missing npy shape");
  }
  const auto open = header.find('(', shape_key);
  const auto close = header.find(')', open);
  if (open == std::string::npos || close == std::string::npos) {
    throw std::runtime_error("invalid npy shape");
  }
  const std::string shape_text = header.substr(open + 1U, close - open - 1U);
  std::vector<int> shape;
  std::size_t cursor = 0U;
  while (cursor < shape_text.size()) {
    const auto comma = shape_text.find(',', cursor);
    const auto end = comma == std::string::npos ? shape_text.size() : comma;
    std::string_view token(shape_text.data() + cursor, end - cursor);
    while (!token.empty() &&
           std::isspace(static_cast<unsigned char>(token.front())) != 0) {
      token.remove_prefix(1U);
    }
    while (!token.empty() &&
           std::isspace(static_cast<unsigned char>(token.back())) != 0) {
      token.remove_suffix(1U);
    }
    if (token.empty()) {
      throw std::runtime_error("npy shape contains an empty dimension");
    }
    int dimension = 0;
    const auto parsed = std::from_chars(
        token.data(), token.data() + token.size(), dimension);
    if (parsed.ec != std::errc{} || parsed.ptr != token.data() + token.size() ||
        dimension <= 0) {
      throw std::runtime_error("npy shape contains an invalid dimension");
    }
    shape.push_back(dimension);
    if (shape.size() > 8U) {
      throw std::runtime_error("npy shape rank exceeds parser limits");
    }
    if (comma == std::string::npos) {
      break;
    }
    cursor = comma + 1U;
    if (cursor == shape_text.size()) {
      if (shape.size() != 1U) {
        throw std::runtime_error("npy shape has an invalid trailing comma");
      }
      break;
    }
    const auto remaining = std::string_view(shape_text).substr(cursor);
    if (std::all_of(remaining.begin(), remaining.end(), [](unsigned char ch) {
          return std::isspace(ch) != 0;
        })) {
      if (shape.size() != 1U) {
        throw std::runtime_error("npy shape has an invalid trailing comma");
      }
      break;
    }
  }
  NpyArray array;
  array.descr = descr;
  array.shape = std::move(shape);
  const size_t payload_offset = header_offset + header_len;
  array.payload.assign(bytes.begin() + static_cast<std::ptrdiff_t>(payload_offset), bytes.end());
  return array;
}

std::size_t CheckedGridCells(int rows, int cols, const std::string& context) {
  constexpr std::size_t kMaxGridCells = 25'000'000U;
  if (rows <= 0 || cols <= 0) {
    throw std::runtime_error(context + " grid is empty");
  }
  const auto rows_size = static_cast<std::size_t>(rows);
  const auto cols_size = static_cast<std::size_t>(cols);
  if (rows_size > kMaxGridCells / cols_size) {
    throw std::runtime_error(context + " grid exceeds parser limits");
  }
  return rows_size * cols_size;
}

const std::vector<std::uint8_t>& Entry(
    const std::map<std::string, std::vector<std::uint8_t>>& entries,
    const std::string& name) {
  const auto it = entries.find(name);
  if (it == entries.end()) {
    throw std::runtime_error("missing npz entry: " + name);
  }
  return it->second;
}

template <typename T>
std::vector<T> PayloadAs(const NpyArray& array, const std::string& descr, size_t count) {
  if (array.descr != descr) {
    throw std::runtime_error("unexpected npy dtype: " + array.descr + ", expected " + descr);
  }
  if (array.payload.size() != sizeof(T) * count) {
    throw std::runtime_error("npy payload size does not match shape");
  }
  std::vector<T> out(count);
  std::memcpy(out.data(), array.payload.data(), array.payload.size());
  return out;
}

double ScalarF64(const NpyArray& array) {
  if (array.descr != "<f8" || !array.shape.empty() || array.payload.size() != sizeof(double)) {
    throw std::runtime_error("expected scalar float64 npy");
  }
  double value = 0.0;
  std::memcpy(&value, array.payload.data(), sizeof(double));
  return value;
}

std::array<double, 2> OriginF64(const NpyArray& array) {
  if (array.descr != "<f8" || array.shape.size() != 1U || array.shape[0] != 2 ||
      array.payload.size() != sizeof(double) * 2U) {
    throw std::runtime_error("expected origin float64[2] npy");
  }
  std::array<double, 2> origin{};
  std::memcpy(origin.data(), array.payload.data(), sizeof(double) * origin.size());
  return origin;
}

OccupancyArtifactData LoadOccupancyImpl(const std::filesystem::path& path) {
  const auto entries = ReadStoredNpz(path);
  const auto grid = ParseNpy(Entry(entries, "grid.npy"));
  if (grid.descr != "|i1" || grid.shape.size() != 2U) {
    throw std::runtime_error("occupancy grid must be int8 HxW");
  }
  OccupancyArtifactData out;
  out.rows = grid.shape[0];
  out.cols = grid.shape[1];
  const auto cells = CheckedGridCells(out.rows, out.cols, "occupancy");
  out.grid = PayloadAs<std::int8_t>(grid, "|i1", cells);
  out.resolution = ScalarF64(ParseNpy(Entry(entries, "resolution.npy")));
  const auto origin = OriginF64(ParseNpy(Entry(entries, "origin.npy")));
  out.origin_x = origin[0];
  out.origin_y = origin[1];
  if (!std::isfinite(out.resolution) || out.resolution <= 0.0 ||
      !std::isfinite(out.origin_x) || !std::isfinite(out.origin_y)) {
    throw std::runtime_error("occupancy grid geometry is invalid");
  }
  for (const auto value : out.grid) {
    if (value < -1 || value > 100) {
      throw std::runtime_error("occupancy grid contains a value outside [-1, 100]");
    }
  }
  return out;
}

EsdfArray LoadEsdf(const std::filesystem::path& path) {
  const auto entries = ReadStoredNpz(path);
  const auto distance = ParseNpy(Entry(entries, "distance.npy"));
  if (distance.descr != "<f4" || distance.shape.size() != 2U) {
    throw std::runtime_error("esdf distance must be float32 HxW");
  }
  EsdfArray out;
  out.rows = distance.shape[0];
  out.cols = distance.shape[1];
  const auto cells = CheckedGridCells(out.rows, out.cols, "esdf");
  out.distance = PayloadAs<float>(
      distance,
      "<f4",
      cells);
  const auto grad_x_entry = entries.find("grad_x.npy");
  const auto grad_y_entry = entries.find("grad_y.npy");
  if (grad_x_entry != entries.end()) {
    const auto grad_x = ParseNpy(grad_x_entry->second);
    if (grad_x.shape != distance.shape) {
      throw std::runtime_error("esdf grad_x shape does not match distance");
    }
    out.grad_x = PayloadAs<float>(grad_x, "<f4", cells);
  }
  if (grad_y_entry != entries.end()) {
    const auto grad_y = ParseNpy(grad_y_entry->second);
    if (grad_y.shape != distance.shape) {
      throw std::runtime_error("esdf grad_y shape does not match distance");
    }
    out.grad_y = PayloadAs<float>(grad_y, "<f4", cells);
  }
  out.resolution = ScalarF64(ParseNpy(Entry(entries, "resolution.npy")));
  const auto origin = OriginF64(ParseNpy(Entry(entries, "origin.npy")));
  out.origin_x = origin[0];
  out.origin_y = origin[1];
  if (!std::isfinite(out.resolution) || out.resolution <= 0.0 ||
      !std::isfinite(out.origin_x) || !std::isfinite(out.origin_y)) {
    throw std::runtime_error("esdf grid geometry is invalid");
  }
  return out;
}

layers::Grid2D OccupancyToCostGrid(const OccupancyArtifactData& occupancy) {
  auto grid = layers::makeGrid2D(
      occupancy.rows,
      occupancy.cols,
      occupancy.resolution,
      occupancy.origin_x,
      occupancy.origin_y,
      0.0F);
  for (size_t i = 0; i < occupancy.grid.size(); ++i) {
    const int value = static_cast<int>(occupancy.grid[i]);
    grid.data[i] = value < 0 ? 100.0F : static_cast<float>(std::clamp(value, 0, 100));
  }
  return grid;
}

layers::Grid2D EsdfDistanceGrid(const EsdfArray& esdf) {
  layers::Grid2D grid;
  grid.rows = esdf.rows;
  grid.cols = esdf.cols;
  grid.resolution = esdf.resolution;
  grid.originX = esdf.origin_x;
  grid.originY = esdf.origin_y;
  grid.data = esdf.distance;
  return grid;
}

std::filesystem::path MakeStagingDir(
    const std::filesystem::path& map_dir,
    const std::string& prefix) {
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch();
  const auto suffix =
      std::chrono::duration_cast<std::chrono::nanoseconds>(stamp).count();
  auto staging = map_dir / ".builds" / (prefix + "_tmp_" + std::to_string(suffix));
  std::filesystem::create_directories(staging);
  return staging;
}

bool CommitFile(
    const std::filesystem::path& staged,
    const std::filesystem::path& final_path,
    std::string* error) {
  std::error_code ec;
  std::filesystem::remove(final_path, ec);
  ec.clear();
  std::filesystem::rename(staged, final_path, ec);
  if (ec) {
    if (error != nullptr) {
      *error = "failed to commit " + final_path.filename().string() + ": " + ec.message();
    }
    return false;
  }
  return true;
}

}  // namespace

OccupancyArtifactData LoadOccupancyArtifact(
    const std::filesystem::path& path) {
  return LoadOccupancyImpl(path);
}

GridArtifactResult BuildEsdfArtifact(
    const std::filesystem::path& map_dir,
    bool output_is_staged) {
  try {
    const auto occupancy_path = map_dir / "occupancy.npz";
    if (!std::filesystem::is_regular_file(occupancy_path)) {
      return Error("occupancy.npz is required before building esdf.npz");
    }
    const auto occupancy = LoadOccupancyImpl(occupancy_path);
    const auto occupancy_grid = OccupancyToCostGrid(occupancy);
    const auto esdf = layers::computeEsdf(occupancy_grid, 50.0F);

    const auto final_path = map_dir / "esdf.npz";
    const auto staging = output_is_staged ? map_dir : MakeStagingDir(map_dir, "esdf");
    const auto staged_path = staging / "esdf.npz";
    const bool wrote = WriteStoredZip(
        staged_path,
        {
            {"distance.npy", MakeGridF32Npy(esdf.distance.data, esdf.distance.rows, esdf.distance.cols)},
            {"grad_x.npy", MakeGridF32Npy(esdf.gradX.data, esdf.gradX.rows, esdf.gradX.cols)},
            {"grad_y.npy", MakeGridF32Npy(esdf.gradY.data, esdf.gradY.rows, esdf.gradY.cols)},
            {"resolution.npy", MakeScalarF64Npy(esdf.distance.resolution)},
            {"origin.npy", MakeOriginNpy(esdf.distance.originX, esdf.distance.originY)},
        });
    if (!wrote) {
      if (!output_is_staged) std::filesystem::remove_all(staging);
      return Error("failed to write staged esdf.npz");
    }
    if (output_is_staged) {
      GridArtifactResult result;
      result.ok = true;
      result.message = "built staged esdf.npz";
      result.path = final_path;
      result.rows = esdf.distance.rows;
      result.cols = esdf.distance.cols;
      result.resolution = esdf.distance.resolution;
      result.origin_x = esdf.distance.originX;
      result.origin_y = esdf.distance.originY;
      return result;
    }
    std::string commit_error;
    if (!CommitFile(staged_path, final_path, &commit_error)) {
      std::filesystem::remove_all(staging);
      return Error(commit_error);
    }
    std::filesystem::remove_all(staging);
    GridArtifactResult result;
    result.ok = true;
    result.message = "built esdf.npz";
    result.path = final_path;
    result.rows = esdf.distance.rows;
    result.cols = esdf.distance.cols;
    result.resolution = esdf.distance.resolution;
    result.origin_x = esdf.distance.originX;
    result.origin_y = esdf.distance.originY;
    return result;
  } catch (const std::exception& exc) {
    return Error(exc.what());
  }
}

GridArtifactResult BuildTraversabilityArtifact(
    const std::filesystem::path& map_dir,
    bool output_is_staged) {
  try {
    const auto occupancy_path = map_dir / "occupancy.npz";
    if (!std::filesystem::is_regular_file(occupancy_path)) {
      return Error("occupancy.npz is required before building traversability.npz");
    }
    const auto esdf_path = map_dir / "esdf.npz";
    if (!std::filesystem::is_regular_file(esdf_path)) {
      const auto esdf_result = BuildEsdfArtifact(map_dir, output_is_staged);
      if (!esdf_result.ok) {
        return Error("failed to build prerequisite esdf.npz: " + esdf_result.message);
      }
    }
    const auto occupancy = LoadOccupancyImpl(occupancy_path);
    const auto esdf = LoadEsdf(esdf_path);
    if (occupancy.rows != esdf.rows || occupancy.cols != esdf.cols ||
        std::fabs(occupancy.resolution - esdf.resolution) > 1e-9 ||
        std::fabs(occupancy.origin_x - esdf.origin_x) > 1e-9 ||
        std::fabs(occupancy.origin_y - esdf.origin_y) > 1e-9) {
      return Error("occupancy.npz and esdf.npz geometry mismatch");
    }
    const auto costmap = OccupancyToCostGrid(occupancy);
    const auto esdf_grid = EsdfDistanceGrid(esdf);
    const auto fused = layers::fuseTraversabilityCost(
        costmap,
        layers::Grid2D{},
        esdf_grid,
        layers::Grid2D{});

    const auto final_path = map_dir / "traversability.npz";
    const auto staging =
        output_is_staged ? map_dir : MakeStagingDir(map_dir, "traversability");
    const auto staged_path = staging / "traversability.npz";
    const bool wrote = WriteStoredZip(
        staged_path,
        {
            {"cost.npy", MakeGridF32Npy(fused.data, fused.rows, fused.cols)},
            {"resolution.npy", MakeScalarF64Npy(fused.resolution)},
            {"origin.npy", MakeOriginNpy(fused.originX, fused.originY)},
        });
    if (!wrote) {
      if (!output_is_staged) std::filesystem::remove_all(staging);
      return Error("failed to write staged traversability.npz");
    }
    if (output_is_staged) {
      GridArtifactResult result;
      result.ok = true;
      result.message = "built staged traversability.npz";
      result.path = final_path;
      result.rows = fused.rows;
      result.cols = fused.cols;
      result.resolution = fused.resolution;
      result.origin_x = fused.originX;
      result.origin_y = fused.originY;
      return result;
    }
    std::string commit_error;
    if (!CommitFile(staged_path, final_path, &commit_error)) {
      std::filesystem::remove_all(staging);
      return Error(commit_error);
    }
    std::filesystem::remove_all(staging);
    GridArtifactResult result;
    result.ok = true;
    result.message = "built traversability.npz";
    result.path = final_path;
    result.rows = fused.rows;
    result.cols = fused.cols;
    result.resolution = fused.resolution;
    result.origin_x = fused.originX;
    result.origin_y = fused.originY;
    return result;
  } catch (const std::exception& exc) {
    return Error(exc.what());
  }
}

}  // namespace lingtu::maps

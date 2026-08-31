#include "lingtu/maps/build/pcd.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_set>

namespace lingtu::maps {
namespace {

struct PcdHeader {
  std::vector<std::string> fields;
  std::vector<int> sizes;
  std::vector<std::string> types;
  std::vector<int> counts;
  std::string data;
  int64_t points{0};
  int64_t width{0};
  int64_t height{0};
  bool fields_declared{false};
  bool sizes_declared{false};
  bool types_declared{false};
  bool counts_declared{false};
  bool points_declared{false};
  bool width_declared{false};
  bool height_declared{false};
};

struct ScalarLayout {
  int scalar_count{0};
  int byte_stride{0};
  int x_scalar{-1};
  int y_scalar{-1};
  int z_scalar{-1};
  int x_byte{-1};
  int y_byte{-1};
  int z_byte{-1};
};

struct VoxelKey {
  int64_t x{0};
  int64_t y{0};
  int64_t z{0};

  bool operator==(const VoxelKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  size_t operator()(const VoxelKey& key) const {
    const auto hx = std::hash<int64_t>{}(key.x);
    const auto hy = std::hash<int64_t>{}(key.y);
    const auto hz = std::hash<int64_t>{}(key.z);
    return hx ^ (hy + 0x9e3779b97f4a7c15ULL + (hx << 6U) + (hx >> 2U)) ^
        (hz + 0x9e3779b97f4a7c15ULL + (hy << 6U) + (hy >> 2U));
  }
};

std::vector<std::string> SplitWords(const std::string& line) {
  std::istringstream stream(line);
  std::vector<std::string> out;
  std::string item;
  while (stream >> item) {
    out.push_back(item);
  }
  return out;
}

std::string Upper(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
    return static_cast<char>(std::toupper(ch));
  });
  return value;
}

std::string Lower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return value;
}

bool ParsePositiveInt(const std::string& value, int* output) {
  if (output == nullptr || value.empty()) {
    return false;
  }
  int parsed = 0;
  const auto result = std::from_chars(value.data(), value.data() + value.size(), parsed);
  if (result.ec != std::errc{} || result.ptr != value.data() + value.size() || parsed <= 0) {
    return false;
  }
  *output = parsed;
  return true;
}

bool ParseNonNegativeInt64(const std::string& value, int64_t* output) {
  if (output == nullptr || value.empty()) {
    return false;
  }
  int64_t parsed = 0;
  const auto result = std::from_chars(value.data(), value.data() + value.size(), parsed);
  if (result.ec != std::errc{} || result.ptr != value.data() + value.size() || parsed < 0) {
    return false;
  }
  *output = parsed;
  return true;
}

bool IsFinitePoint(const PointXyz& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
      std::abs(point.x) < 500.0F && std::abs(point.y) < 500.0F && std::abs(point.z) < 500.0F;
}

bool InBounds(const PointXyz& point, const PcdBounds& bounds) {
  return point.x >= bounds.min_x && point.x <= bounds.max_x &&
      point.y >= bounds.min_y && point.y <= bounds.max_y &&
      point.z >= bounds.min_z && point.z <= bounds.max_z;
}

bool BuildLayout(const PcdHeader& header, ScalarLayout* layout, std::string* error) {
  constexpr int kMaxFields = 256;
  constexpr int kMaxScalars = 65'536;
  constexpr int kMaxStrideBytes = 1'048'576;
  if (header.fields.empty()) {
    *error = "PCD missing FIELDS";
    return false;
  }
  if (header.fields.size() > static_cast<std::size_t>(kMaxFields)) {
    *error = "PCD has too many fields";
    return false;
  }
  std::vector<int> sizes = header.sizes;
  std::vector<std::string> types = header.types;
  std::vector<int> counts = header.counts;
  if (sizes.empty()) {
    sizes.assign(header.fields.size(), 4);
  }
  if (types.empty()) {
    types.assign(header.fields.size(), "F");
  }
  if (counts.empty()) {
    counts.assign(header.fields.size(), 1);
  }
  if (
      sizes.size() != header.fields.size() ||
      types.size() != header.fields.size() ||
      counts.size() != header.fields.size()) {
    *error = "PCD FIELDS/SIZE/TYPE/COUNT length mismatch";
    return false;
  }

  int scalar = 0;
  int byte_offset = 0;
  for (size_t i = 0; i < header.fields.size(); ++i) {
    const std::string field = Lower(header.fields[i]);
    const int count = counts[i];
    const int size = sizes[i];
    const std::string type = Upper(types[i]);
    const bool valid_type_size =
        (type == "F" && (size == 4 || size == 8)) ||
        ((type == "I" || type == "U") &&
         (size == 1 || size == 2 || size == 4 || size == 8));
    if (count <= 0 || size <= 0 || !valid_type_size) {
      *error = "PCD field layout is invalid";
      return false;
    }
    if (count > kMaxScalars || scalar > kMaxScalars - count ||
        size > kMaxStrideBytes / count ||
        byte_offset > kMaxStrideBytes - size * count) {
      *error = "PCD field layout exceeds parser limits";
      return false;
    }
    if (count == 1) {
      if (field == "x") {
        if (layout->x_scalar >= 0) {
          *error = "PCD contains duplicate x field";
          return false;
        }
        layout->x_scalar = scalar;
        layout->x_byte = byte_offset;
      } else if (field == "y") {
        if (layout->y_scalar >= 0) {
          *error = "PCD contains duplicate y field";
          return false;
        }
        layout->y_scalar = scalar;
        layout->y_byte = byte_offset;
      } else if (field == "z") {
        if (layout->z_scalar >= 0) {
          *error = "PCD contains duplicate z field";
          return false;
        }
        layout->z_scalar = scalar;
        layout->z_byte = byte_offset;
      }
    }
    if ((field == "x" || field == "y" || field == "z") &&
        (count != 1 || type != "F" || size != static_cast<int>(sizeof(float)))) {
      *error = "PCD x/y/z fields must be scalar float32";
      return false;
    }
    scalar += count;
    byte_offset += size * count;
  }
  layout->scalar_count = scalar;
  layout->byte_stride = byte_offset;
  if (layout->x_scalar < 0 || layout->y_scalar < 0 || layout->z_scalar < 0) {
    *error = "PCD missing x/y/z fields";
    return false;
  }
  return true;
}

PcdIoResult LoadAsciiPoints(std::istream& stream, const PcdHeader& header) {
  PcdIoResult result;
  ScalarLayout layout;
  if (!BuildLayout(header, &layout, &result.message)) {
    return result;
  }
  std::string line;
  while (std::getline(stream, line)) {
    if (line.empty()) {
      continue;
    }
    std::istringstream row(line);
    std::vector<float> values;
    values.reserve(static_cast<size_t>(layout.scalar_count));
    float value = 0.0F;
    while (row >> value) {
      values.push_back(value);
    }
    if (static_cast<int>(values.size()) <= std::max({layout.x_scalar, layout.y_scalar, layout.z_scalar})) {
      continue;
    }
    PointXyz point{values[layout.x_scalar], values[layout.y_scalar], values[layout.z_scalar]};
    if (IsFinitePoint(point)) {
      result.points.push_back(point);
    }
  }
  result.ok = true;
  return result;
}

PcdIoResult LoadBinaryPoints(std::istream& stream, const PcdHeader& header) {
  PcdIoResult result;
  ScalarLayout layout;
  if (!BuildLayout(header, &layout, &result.message)) {
    return result;
  }
  if (layout.byte_stride <= 0) {
    result.message = "PCD binary stride is invalid";
    return result;
  }
  if (layout.x_byte < 0 || layout.y_byte < 0 || layout.z_byte < 0) {
    result.message = "PCD binary missing x/y/z byte offsets";
    return result;
  }
  std::ostringstream payload_stream;
  payload_stream << stream.rdbuf();
  if (stream.bad()) {
    result.message = "PCD binary payload is unreadable";
    return result;
  }
  const std::string payload = payload_stream.str();
  if (payload.size() % static_cast<size_t>(layout.byte_stride) != 0U) {
    result.message = "PCD binary payload does not match its stride";
    return result;
  }
  int64_t available = static_cast<int64_t>(payload.size() / static_cast<size_t>(layout.byte_stride));
  if (header.points_declared && header.points != available) {
    result.message = "PCD binary payload does not match declared point count";
    return result;
  }
  const int64_t point_count = header.points_declared ? header.points : available;
  for (int64_t i = 0; i < point_count; ++i) {
    const char* base = payload.data() + i * layout.byte_stride;
    PointXyz point;
    std::memcpy(&point.x, base + layout.x_byte, sizeof(float));
    std::memcpy(&point.y, base + layout.y_byte, sizeof(float));
    std::memcpy(&point.z, base + layout.z_byte, sizeof(float));
    if (IsFinitePoint(point)) {
      result.points.push_back(point);
    }
  }
  result.ok = true;
  return result;
}

PcdIoResult MakeError(const std::string& message) {
  PcdIoResult result;
  result.ok = false;
  result.message = message;
  return result;
}

}  // namespace

PcdIoResult LoadPcdXyz(const std::filesystem::path& path) {
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    return MakeError("source PCD not found: " + path.string());
  }

  PcdHeader header;
  std::string line;
  while (std::getline(file, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.empty() || line[0] == '#') {
      continue;
    }
    const auto parts = SplitWords(line);
    if (parts.empty()) {
      continue;
    }
    const std::string key = Upper(parts[0]);
    if (key == "FIELDS") {
      if (header.fields_declared || parts.size() <= 1U) {
        return MakeError("PCD FIELDS declaration is invalid or duplicated");
      }
      header.fields_declared = true;
      header.fields.assign(parts.begin() + 1, parts.end());
    } else if (key == "SIZE") {
      if (header.sizes_declared || parts.size() <= 1U) {
        return MakeError("PCD SIZE declaration is invalid or duplicated");
      }
      header.sizes_declared = true;
      header.sizes.clear();
      for (size_t i = 1; i < parts.size(); ++i) {
        int parsed = 0;
        if (!ParsePositiveInt(parts[i], &parsed)) {
          return MakeError("PCD SIZE contains an invalid integer");
        }
        header.sizes.push_back(parsed);
      }
    } else if (key == "TYPE") {
      if (header.types_declared || parts.size() <= 1U) {
        return MakeError("PCD TYPE declaration is invalid or duplicated");
      }
      header.types_declared = true;
      header.types.clear();
      for (size_t i = 1; i < parts.size(); ++i) {
        header.types.push_back(Upper(parts[i]));
      }
    } else if (key == "COUNT") {
      if (header.counts_declared || parts.size() <= 1U) {
        return MakeError("PCD COUNT declaration is invalid or duplicated");
      }
      header.counts_declared = true;
      header.counts.clear();
      for (size_t i = 1; i < parts.size(); ++i) {
        int parsed = 0;
        if (!ParsePositiveInt(parts[i], &parsed)) {
          return MakeError("PCD COUNT contains an invalid integer");
        }
        header.counts.push_back(parsed);
      }
    } else if (key == "POINTS") {
      if (header.points_declared || parts.size() != 2U ||
          !ParseNonNegativeInt64(parts[1], &header.points)) {
        return MakeError("PCD POINTS declaration is invalid or duplicated");
      }
      header.points_declared = true;
    } else if (key == "WIDTH") {
      if (header.width_declared || parts.size() != 2U ||
          !ParseNonNegativeInt64(parts[1], &header.width)) {
        return MakeError("PCD WIDTH declaration is invalid or duplicated");
      }
      header.width_declared = true;
    } else if (key == "HEIGHT") {
      if (header.height_declared || parts.size() != 2U ||
          !ParseNonNegativeInt64(parts[1], &header.height)) {
        return MakeError("PCD HEIGHT declaration is invalid or duplicated");
      }
      header.height_declared = true;
    } else if (key == "DATA") {
      if (!header.data.empty() || parts.size() != 2U) {
        return MakeError("PCD DATA declaration is invalid or duplicated");
      }
      header.data = Lower(parts[1]);
      break;
    }
  }

  if (header.width_declared != header.height_declared) {
    return MakeError("PCD WIDTH and HEIGHT must be declared together");
  }
  if (header.width_declared) {
    if (header.height != 0 &&
        header.width > std::numeric_limits<int64_t>::max() / header.height) {
      return MakeError("PCD WIDTH x HEIGHT overflows point count");
    }
    const int64_t dimensions = header.width * header.height;
    if (header.points_declared && header.points != dimensions) {
      return MakeError("PCD POINTS does not match WIDTH x HEIGHT");
    }
    if (!header.points_declared) {
      header.points = dimensions;
      header.points_declared = true;
    }
  }

  if (header.data == "ascii") {
    return LoadAsciiPoints(file, header);
  }
  if (header.data == "binary") {
    return LoadBinaryPoints(file, header);
  }
  return MakeError("unsupported PCD DATA format: " + header.data);
}

bool WriteBinaryXyzPcd(
    const std::filesystem::path& path,
    const std::vector<PointXyz>& points,
    std::string* error) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  if (!file) {
    if (error != nullptr) {
      *error = "failed to write PCD: " + path.string();
    }
    return false;
  }
  file
      << "# .PCD v0.7 - Point Cloud Data file format\n"
      << "VERSION 0.7\n"
      << "FIELDS x y z\n"
      << "SIZE 4 4 4\n"
      << "TYPE F F F\n"
      << "COUNT 1 1 1\n"
      << "WIDTH " << points.size() << "\n"
      << "HEIGHT 1\n"
      << "VIEWPOINT 0 0 0 1 0 0 0\n"
      << "POINTS " << points.size() << "\n"
      << "DATA binary\n";
  for (const auto& point : points) {
    file.write(reinterpret_cast<const char*>(&point.x), sizeof(float));
    file.write(reinterpret_cast<const char*>(&point.y), sizeof(float));
    file.write(reinterpret_cast<const char*>(&point.z), sizeof(float));
  }
  file.flush();
  if (!file) {
    if (error != nullptr) {
      *error = "failed to finish writing PCD: " + path.string();
    }
    return false;
  }
  return true;
}

std::vector<PointXyz> FilterPcdPoints(
    const std::vector<PointXyz>& points,
    const PcdFilterOptions& options) {
  std::vector<PointXyz> bounded;
  bounded.reserve(points.size());
  for (const auto& point : points) {
    if (!IsFinitePoint(point)) {
      continue;
    }
    if (options.bounds.enabled) {
      const bool inside = InBounds(point, options.bounds);
      if (options.invert_bounds ? inside : !inside) {
        continue;
      }
    }
    bounded.push_back(point);
  }

  if (bounded.empty() || options.voxel_size <= 0.0) {
    return bounded;
  }
  std::unordered_set<VoxelKey, VoxelKeyHash> seen;
  std::vector<PointXyz> out;
  out.reserve(bounded.size());
  const double inv = 1.0 / options.voxel_size;
  for (const auto& point : bounded) {
    const VoxelKey key{
        static_cast<int64_t>(std::floor(static_cast<double>(point.x) * inv)),
        static_cast<int64_t>(std::floor(static_cast<double>(point.y) * inv)),
        static_cast<int64_t>(std::floor(static_cast<double>(point.z) * inv)),
    };
    if (seen.insert(key).second) {
      out.push_back(point);
    }
  }
  return out;
}

}  // namespace lingtu::maps

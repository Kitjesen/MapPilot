#include "core/io.hpp"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <limits>
#include <numeric>
#include <stdexcept>

#include "core/text.hpp"

namespace lingtu::map_cleaning {
namespace {

struct PcdHeader {
  std::vector<std::string> lines;
  std::vector<std::string> fields;
  std::vector<int> sizes;
  std::vector<char> types;
  std::vector<int> counts;
  std::uint64_t points{0};
  std::string data;
};

int fieldIndex(const PcdHeader &header, const std::string &name) {
  for (std::size_t i = 0; i < header.fields.size(); ++i) {
    if (header.fields[i] == name) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

std::vector<std::size_t> fieldOffsets(const PcdHeader &header, bool binary) {
  std::vector<std::size_t> offsets(header.fields.size(), 0);
  std::size_t offset = 0;
  for (std::size_t i = 0; i < header.fields.size(); ++i) {
    offsets[i] = offset;
    offset += static_cast<std::size_t>(header.counts[i]) * (binary ? header.sizes[i] : 1);
  }
  return offsets;
}

std::size_t pointStride(const PcdHeader &header) {
  std::size_t stride = 0;
  for (std::size_t i = 0; i < header.fields.size(); ++i) {
    stride += static_cast<std::size_t>(header.counts[i]) * header.sizes[i];
  }
  return stride;
}

template <typename T>
T readPod(const char *ptr) {
  T value{};
  std::memcpy(&value, ptr, sizeof(T));
  return value;
}

float readScalar(const char *row, const PcdHeader &header, const std::vector<std::size_t> &offsets,
                 int idx) {
  if (idx < 0 || idx >= static_cast<int>(header.fields.size())) {
    return 0.0F;
  }
  const char *ptr = row + offsets[static_cast<std::size_t>(idx)];
  const int size = header.sizes[static_cast<std::size_t>(idx)];
  const char type = header.types[static_cast<std::size_t>(idx)];
  if (type == 'F' && size == 4) {
    return readPod<float>(ptr);
  }
  if (type == 'F' && size == 8) {
    return static_cast<float>(readPod<double>(ptr));
  }
  if (type == 'U' && size == 1) {
    return static_cast<float>(readPod<std::uint8_t>(ptr));
  }
  if (type == 'U' && size == 2) {
    return static_cast<float>(readPod<std::uint16_t>(ptr));
  }
  if (type == 'U' && size == 4) {
    return static_cast<float>(readPod<std::uint32_t>(ptr));
  }
  if (type == 'U' && size == 8) {
    return static_cast<float>(readPod<std::uint64_t>(ptr));
  }
  if (type == 'I' && size == 1) {
    return static_cast<float>(readPod<std::int8_t>(ptr));
  }
  if (type == 'I' && size == 2) {
    return static_cast<float>(readPod<std::int16_t>(ptr));
  }
  if (type == 'I' && size == 4) {
    return static_cast<float>(readPod<std::int32_t>(ptr));
  }
  if (type == 'I' && size == 8) {
    return static_cast<float>(readPod<std::int64_t>(ptr));
  }
  throw std::runtime_error("unsupported PCD scalar field: " + header.fields[idx]);
}

PcdHeader readPcdHeader(std::ifstream &in, const std::filesystem::path &path) {
  PcdHeader header;
  std::string line;
  bool has_points = false;
  while (std::getline(in, line)) {
    header.lines.push_back(line);
    const std::string cleaned = trim(line);
    if (cleaned.empty() || cleaned[0] == '#') {
      continue;
    }
    const std::vector<std::string> parts = splitWords(cleaned);
    if (parts.empty()) {
      continue;
    }
    const std::string &key = parts[0];
    if (key == "FIELDS") {
      header.fields.assign(parts.begin() + 1, parts.end());
    } else if (key == "SIZE") {
      for (std::size_t i = 1; i < parts.size(); ++i) {
        header.sizes.push_back(std::stoi(parts[i]));
      }
    } else if (key == "TYPE") {
      for (std::size_t i = 1; i < parts.size(); ++i) {
        header.types.push_back(parts[i].empty() ? 'F' : parts[i][0]);
      }
    } else if (key == "COUNT") {
      for (std::size_t i = 1; i < parts.size(); ++i) {
        header.counts.push_back(std::stoi(parts[i]));
      }
    } else if (key == "POINTS" && parts.size() >= 2) {
      header.points = static_cast<std::uint64_t>(std::stoull(parts[1]));
      has_points = true;
    } else if (key == "DATA" && parts.size() >= 2) {
      header.data = parts[1];
      break;
    }
  }

  if (header.fields.empty() || header.data.empty() || !has_points) {
    throw std::runtime_error("invalid PCD header: " + path.string());
  }
  if (header.sizes.empty()) {
    header.sizes.assign(header.fields.size(), 4);
  }
  if (header.types.empty()) {
    header.types.assign(header.fields.size(), 'F');
  }
  if (header.counts.empty()) {
    header.counts.assign(header.fields.size(), 1);
  }
  if (header.sizes.size() != header.fields.size() || header.types.size() != header.fields.size() ||
      header.counts.size() != header.fields.size()) {
    throw std::runtime_error("unsupported PCD field metadata: " + path.string());
  }
  if (fieldIndex(header, "x") < 0 || fieldIndex(header, "y") < 0 || fieldIndex(header, "z") < 0) {
    throw std::runtime_error("PCD is missing x/y/z fields: " + path.string());
  }
  for (std::size_t i = 0; i < header.fields.size(); ++i) {
    const auto size = header.sizes[i];
    const auto type = header.types[i];
    if (header.counts[i] <= 0 ||
        !((type == 'F' && (size == 4 || size == 8)) ||
          ((type == 'I' || type == 'U') && (size == 1 || size == 2 || size == 4 || size == 8)))) {
      throw std::runtime_error("unsupported PCD field metadata: " + path.string());
    }
  }
  for (const auto *name : {"x", "y", "z", "intensity"}) {
    const auto index = fieldIndex(header, name);
    if (index >= 0 && header.counts[index] != 1)
      throw std::runtime_error("PCD geometry and intensity fields require COUNT 1: " + path.string());
  }
  if (header.data != "ascii" && header.data != "binary")
    throw std::runtime_error("unsupported PCD DATA mode: " + header.data);
  if (header.data == "binary") {
    const auto payload = in.tellg();
    in.seekg(0, std::ios::end);
    const auto bytes = in.tellg() - payload;
    in.seekg(payload);
    const auto stride = pointStride(header);
    if (header.points > std::numeric_limits<std::uint64_t>::max() / stride || bytes < 0 ||
        static_cast<std::uint64_t>(bytes) != header.points * stride)
      throw std::runtime_error("truncated or inconsistent binary PCD point data: " + path.string());
  }
  return header;
}

void writePartitionHeader(std::ofstream &out, const PcdHeader &header, std::uint64_t points) {
  bool has_width = false, has_height = false;
  for (const auto &line : header.lines) {
    const auto parts = splitWords(line);
    const auto key = parts.empty() ? std::string() : parts.front();
    if (key == "WIDTH") {
      out << "WIDTH " << points << '\n';
      has_width = true;
    } else if (key == "HEIGHT") {
      out << "HEIGHT 1\n";
      has_height = true;
    } else if (key == "POINTS") {
      out << "POINTS " << points << '\n';
    } else if (key == "DATA") {
      if (!has_width) out << "WIDTH " << points << '\n';
      if (!has_height) out << "HEIGHT 1\n";
      out << line << '\n';
    } else {
      out << line << '\n';
    }
  }
}

bool readAsciiRow(std::ifstream &in, const PcdHeader &header, const std::filesystem::path &path,
                  std::string &line, std::vector<std::string> &values) {
  while (std::getline(in, line)) {
    const auto cleaned = trim(line);
    if (cleaned.empty() || cleaned.front() == '#') continue;
    values = splitWords(line);
    const auto expected = std::accumulate(header.counts.begin(), header.counts.end(), std::size_t{0});
    if (values.size() != expected)
      throw std::runtime_error("inconsistent ASCII PCD point data: " + path.string());
    return true;
  }
  return false;
}

}  // namespace

std::vector<PointXYZI> readPcd(const std::filesystem::path &path) {
  std::ifstream in(path, std::ios::binary);
  if (!in.is_open()) {
    throw std::runtime_error("failed to open PCD: " + path.string());
  }
  const PcdHeader header = readPcdHeader(in, path);
  const int x_idx = fieldIndex(header, "x");
  const int y_idx = fieldIndex(header, "y");
  const int z_idx = fieldIndex(header, "z");
  const int intensity_idx = fieldIndex(header, "intensity");
  std::vector<PointXYZI> points;
  points.reserve(static_cast<std::size_t>(header.points));

  if (header.data == "ascii") {
    const auto offsets = fieldOffsets(header, false);
    std::string line;
    std::vector<std::string> values;
    while (readAsciiRow(in, header, path, line, values)) {
      PointXYZI pt;
      pt.x = std::stof(values[offsets[x_idx]]);
      pt.y = std::stof(values[offsets[y_idx]]);
      pt.z = std::stof(values[offsets[z_idx]]);
      pt.intensity =
          intensity_idx >= 0 ? std::stof(values[offsets[intensity_idx]]) : 1.0F;
      points.push_back(pt);
    }
    if (points.size() != header.points)
      throw std::runtime_error("truncated or inconsistent ASCII PCD point data: " + path.string());
    return points;
  }

  if (header.data != "binary") {
    throw std::runtime_error("unsupported PCD DATA mode: " + header.data);
  }

  const auto stride = pointStride(header);
  const auto offsets = fieldOffsets(header, true);
  std::vector<char> row(stride);
  for (std::uint64_t i = 0; i < header.points; ++i) {
    in.read(row.data(), static_cast<std::streamsize>(stride));
    if (in.gcount() != static_cast<std::streamsize>(stride)) {
      throw std::runtime_error("truncated PCD point data: " + path.string());
    }
    PointXYZI pt;
    pt.x = readScalar(row.data(), header, offsets, x_idx);
    pt.y = readScalar(row.data(), header, offsets, y_idx);
    pt.z = readScalar(row.data(), header, offsets, z_idx);
    pt.intensity =
        intensity_idx >= 0 ? readScalar(row.data(), header, offsets, intensity_idx) : 1.0F;
    points.push_back(pt);
  }
  return points;
}

void partitionPcd(const std::filesystem::path &source, const std::filesystem::path &clean,
    const std::filesystem::path &removed, const std::vector<std::uint8_t> &remove_mask) {
  std::ifstream in(source, std::ios::binary);
  if (!in) throw std::runtime_error("failed to open PCD: " + source.string());
  const auto header = readPcdHeader(in, source);
  if (remove_mask.size() != header.points ||
      !std::all_of(remove_mask.begin(), remove_mask.end(), [](auto value) { return value <= 1; }))
    throw std::runtime_error("PCD partition mask must contain one 0/1 entry per source point");
  const auto removed_count = static_cast<std::uint64_t>(std::count(remove_mask.begin(), remove_mask.end(), 1));
  std::ofstream clean_out(clean, std::ios::binary), removed_out(removed, std::ios::binary);
  if (!clean_out || !removed_out)
    throw std::runtime_error("failed to open PCD partition outputs");
  writePartitionHeader(clean_out, header, header.points - removed_count);
  writePartitionHeader(removed_out, header, removed_count);
  if (header.data == "binary") {
    std::vector<char> row(pointStride(header));
    for (const auto remove : remove_mask) {
      in.read(row.data(), static_cast<std::streamsize>(row.size()));
      if (in.gcount() != static_cast<std::streamsize>(row.size()))
        throw std::runtime_error("truncated PCD point data: " + source.string());
      auto &out = remove ? removed_out : clean_out;
      out.write(row.data(), static_cast<std::streamsize>(row.size()));
    }
  } else {
    std::string line;
    std::vector<std::string> values;
    for (const auto remove : remove_mask) {
      if (!readAsciiRow(in, header, source, line, values))
        throw std::runtime_error("truncated ASCII PCD point data: " + source.string());
      (remove ? removed_out : clean_out) << line << '\n';
    }
    if (readAsciiRow(in, header, source, line, values))
      throw std::runtime_error("inconsistent ASCII PCD point count: " + source.string());
  }
  clean_out.flush();
  removed_out.flush();
  if (!clean_out || !removed_out)
    throw std::runtime_error("failed to finish PCD partition outputs");
}

void writePcd(const std::filesystem::path &path, const std::vector<PointXYZI> &points) {
  std::ofstream out(path, std::ios::binary);
  if (!out.is_open()) {
    throw std::runtime_error("failed to write PCD: " + path.string());
  }
  out << "# .PCD v0.7 - Point Cloud Data file format\n";
  out << "VERSION 0.7\n";
  out << "FIELDS x y z intensity\n";
  out << "SIZE 4 4 4 4\n";
  out << "TYPE F F F F\n";
  out << "COUNT 1 1 1 1\n";
  out << "WIDTH " << points.size() << "\n";
  out << "HEIGHT 1\n";
  out << "VIEWPOINT 0 0 0 1 0 0 0\n";
  out << "POINTS " << points.size() << "\n";
  out << "DATA binary\n";
  for (const PointXYZI &pt : points) {
    const float row[4] = {pt.x, pt.y, pt.z, pt.intensity};
    out.write(reinterpret_cast<const char *>(row), sizeof(row));
  }
  out.flush();
  if (!out) {
    throw std::runtime_error("failed to finish PCD output: " + path.string());
  }
}

std::unordered_map<std::string, Pose> readLingtuPoses(const std::filesystem::path &path,
    std::vector<std::string>* patch_order) {
  std::unordered_map<std::string, Pose> poses;
  std::ifstream in(path);
  if (!in.is_open()) {
    throw std::runtime_error("failed to open poses.txt: " + path.string());
  }
  std::string line;
  while (std::getline(in, line)) {
    const std::vector<std::string> parts = splitWords(line);
    if (parts.size() < 8) {
      continue;
    }
    Pose pose;
    pose.tx = std::stod(parts[1]);
    pose.ty = std::stod(parts[2]);
    pose.tz = std::stod(parts[3]);
    pose.qw = std::stod(parts[4]);
    pose.qx = std::stod(parts[5]);
    pose.qy = std::stod(parts[6]);
    pose.qz = std::stod(parts[7]);
    if (!poses.emplace(parts[0], pose).second)
      throw std::runtime_error("duplicate scan in poses.txt: " + parts[0]);
    if (patch_order) patch_order->push_back(parts[0]);
  }
  return poses;
}

std::optional<std::array<float, 3>> readSensorOrigin(const std::filesystem::path& path) {
  std::ifstream file(path);
  std::string line;
  while (std::getline(file, line)) {
    const auto parts = splitWords(line);
    if (parts.empty() || parts[0] != "lidar_origin_in_patch") continue;
    if (parts.size() != 4) throw std::runtime_error("invalid lidar_origin_in_patch in " + path.string());
    return std::array<float, 3>{std::stof(parts[1]), std::stof(parts[2]), std::stof(parts[3])};
  }
  return std::nullopt;
}

}  // namespace lingtu::map_cleaning

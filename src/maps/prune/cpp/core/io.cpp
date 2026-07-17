#include "core/io.hpp"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <stdexcept>

#include "core/text.hpp"

namespace lingtu::map_cleaning {
namespace {

struct PcdHeader {
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

std::vector<int> fieldOffsets(const PcdHeader &header) {
  std::vector<int> offsets(header.fields.size(), 0);
  int offset = 0;
  for (std::size_t i = 0; i < header.fields.size(); ++i) {
    offsets[i] = offset;
    const int count = i < header.counts.size() ? std::max(1, header.counts[i]) : 1;
    const int size = i < header.sizes.size() ? header.sizes[i] : 4;
    offset += count * size;
  }
  return offsets;
}

int pointStride(const PcdHeader &header) {
  int stride = 0;
  for (std::size_t i = 0; i < header.fields.size(); ++i) {
    const int count = i < header.counts.size() ? std::max(1, header.counts[i]) : 1;
    const int size = i < header.sizes.size() ? header.sizes[i] : 4;
    stride += count * size;
  }
  return stride;
}

template <typename T>
T readPod(const char *ptr) {
  T value{};
  std::memcpy(&value, ptr, sizeof(T));
  return value;
}

float readScalar(const char *row, const PcdHeader &header, const std::vector<int> &offsets,
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
  if (type == 'I' && size == 1) {
    return static_cast<float>(readPod<std::int8_t>(ptr));
  }
  if (type == 'I' && size == 2) {
    return static_cast<float>(readPod<std::int16_t>(ptr));
  }
  if (type == 'I' && size == 4) {
    return static_cast<float>(readPod<std::int32_t>(ptr));
  }
  return 0.0F;
}

PcdHeader readPcdHeader(std::ifstream &in, const std::filesystem::path &path) {
  PcdHeader header;
  std::string line;
  while (std::getline(in, line)) {
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
    } else if (key == "DATA" && parts.size() >= 2) {
      header.data = parts[1];
      break;
    }
  }

  if (header.fields.empty() || header.data.empty()) {
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
  return header;
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
    std::string line;
    while (std::getline(in, line)) {
      const std::vector<std::string> values = splitWords(line);
      if (values.size() < header.fields.size()) {
        continue;
      }
      PointXYZI pt;
      pt.x = std::stof(values[static_cast<std::size_t>(x_idx)]);
      pt.y = std::stof(values[static_cast<std::size_t>(y_idx)]);
      pt.z = std::stof(values[static_cast<std::size_t>(z_idx)]);
      pt.intensity =
          intensity_idx >= 0 ? std::stof(values[static_cast<std::size_t>(intensity_idx)]) : 1.0F;
      points.push_back(pt);
    }
    return points;
  }

  if (header.data != "binary") {
    throw std::runtime_error("unsupported PCD DATA mode: " + header.data);
  }

  const int stride = pointStride(header);
  const std::vector<int> offsets = fieldOffsets(header);
  std::vector<char> row(static_cast<std::size_t>(stride));
  for (std::uint64_t i = 0; i < header.points; ++i) {
    in.read(row.data(), stride);
    if (in.gcount() != stride) {
      break;
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
}

std::unordered_map<std::string, Pose> readLingtuPoses(const std::filesystem::path &path) {
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
    poses[parts[0]] = pose;
  }
  return poses;
}

}  // namespace lingtu::map_cleaning

#include "localization/opt/poses.hpp"
#include "localization/opt/cloud.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <unordered_set>

namespace lingtu::localization::opt {
namespace {
struct Field {
  std::string name;
  int size = 0;
  char type = 'F';
  int count = 1;
  std::size_t offset = 0;
};

struct PcdHeader {
  std::vector<Field> fields;
  std::size_t points = 0;
  std::size_t point_step = 0;
  std::string data;
};

bool is_finite(double value) {
  return std::isfinite(value);
}

double sqr(double value) {
  return value * value;
}

Pose normalized(Pose pose) {
  const double norm = std::sqrt(
      sqr(pose.qw) + sqr(pose.qx) + sqr(pose.qy) + sqr(pose.qz));
  if (!is_finite(norm) || norm <= std::numeric_limits<double>::epsilon()) {
    pose.qw = 1.0;
    pose.qx = 0.0;
    pose.qy = 0.0;
    pose.qz = 0.0;
    return pose;
  }
  pose.qw /= norm;
  pose.qx /= norm;
  pose.qy /= norm;
  pose.qz /= norm;
  return pose;
}

bool parse_double(const std::string& token, double& value) {
  char* end = nullptr;
  value = std::strtod(token.c_str(), &end);
  return end != token.c_str() && end != nullptr && *end == '\0' && is_finite(value);
}

std::vector<std::string> split_ws(const std::string& line) {
  std::stringstream ss(line);
  std::vector<std::string> tokens;
  std::string token;
  while (ss >> token) {
    tokens.push_back(token);
  }
  return tokens;
}

bool parse_pose_tokens(
    const std::vector<std::string>& tokens,
    Keyframe& out,
    std::string* error) {
  auto fail = [&](const std::string& code) {
    if (error != nullptr) {
      *error = code;
    }
    return false;
  };
  if (tokens.size() != 8) {
    return fail("invalid_pose_row");
  }
  out.patch_name = tokens[0];
  const std::filesystem::path patch_name(out.patch_name);
  if (patch_name.filename() != patch_name || patch_name.extension() != ".pcd" ||
      out.patch_name == "." || out.patch_name == "..") {
    return fail("invalid_patch_name");
  }
  constexpr std::size_t offset = 1;
  double values[7]{};
  for (std::size_t i = 0; i < 7; ++i) {
    if (!parse_double(tokens[offset + i], values[i])) {
      return fail("invalid_pose_row");
    }
  }
  out.pose.x = values[0];
  out.pose.y = values[1];
  out.pose.z = values[2];
  // Canonical LingTu saved-map format: patch_name x y z qw qx qy qz.
  out.pose.qw = values[3];
  out.pose.qx = values[4];
  out.pose.qy = values[5];
  out.pose.qz = values[6];
  const double quaternion_norm = std::sqrt(
      sqr(out.pose.qw) + sqr(out.pose.qx) +
      sqr(out.pose.qy) + sqr(out.pose.qz));
  if (!is_finite(quaternion_norm) || quaternion_norm < 0.9 || quaternion_norm > 1.1) {
    return fail("invalid_pose_quaternion");
  }
  out.pose = normalized(out.pose);
  return true;
}

std::vector<std::filesystem::path> list_patches(const std::filesystem::path& dir) {
  std::vector<std::filesystem::path> patches;
  std::error_code ec;
  for (const auto& entry : std::filesystem::directory_iterator(dir, ec)) {
    if (!ec && entry.is_regular_file(ec) && entry.path().extension() == ".pcd") {
      patches.push_back(entry.path());
    }
  }
  std::sort(patches.begin(), patches.end());
  return patches;
}

std::string lower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

std::vector<std::string> parse_names_line(const std::vector<std::string>& tokens) {
  std::vector<std::string> values;
  values.reserve(tokens.size() > 1 ? tokens.size() - 1 : 0);
  for (std::size_t i = 1; i < tokens.size(); ++i) {
    values.push_back(tokens[i]);
  }
  return values;
}

std::vector<int> parse_ints_line(const std::vector<std::string>& tokens, int fallback) {
  std::vector<int> values;
  values.reserve(tokens.size() > 1 ? tokens.size() - 1 : 0);
  for (std::size_t i = 1; i < tokens.size(); ++i) {
    try {
      values.push_back(std::stoi(tokens[i]));
    } catch (...) {
      values.push_back(fallback);
    }
  }
  return values;
}

PcdHeader read_pcd_header(std::istream& in) {
  PcdHeader header;
  std::vector<std::string> names;
  std::vector<int> sizes;
  std::vector<int> counts;
  std::vector<std::string> types;

  std::string line;
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    const auto tokens = split_ws(line);
    if (tokens.empty()) {
      continue;
    }
    const std::string key = lower(tokens[0]);
    if (key == "fields") {
      names = parse_names_line(tokens);
    } else if (key == "size") {
      sizes = parse_ints_line(tokens, 4);
    } else if (key == "type") {
      types = parse_names_line(tokens);
    } else if (key == "count") {
      counts = parse_ints_line(tokens, 1);
    } else if (key == "points" && tokens.size() >= 2) {
      header.points = static_cast<std::size_t>(std::stoull(tokens[1]));
    } else if (key == "width" && header.points == 0 && tokens.size() >= 2) {
      header.points = static_cast<std::size_t>(std::stoull(tokens[1]));
    } else if (key == "data" && tokens.size() >= 2) {
      header.data = lower(tokens[1]);
      break;
    }
  }

  if (names.empty()) {
    names = {"x", "y", "z"};
  }
  if (sizes.size() < names.size()) {
    sizes.resize(names.size(), 4);
  }
  if (types.size() < names.size()) {
    types.resize(names.size(), "F");
  }
  if (counts.size() < names.size()) {
    counts.resize(names.size(), 1);
  }

  std::size_t offset = 0;
  for (std::size_t i = 0; i < names.size(); ++i) {
    Field field;
    field.name = lower(names[i]);
    field.size = sizes[i];
    field.type = types[i].empty() ? 'F' : types[i][0];
    field.count = counts[i] <= 0 ? 1 : counts[i];
    field.offset = offset;
    header.fields.push_back(field);
    offset += static_cast<std::size_t>(field.size * field.count);
  }
  header.point_step = offset;
  if (header.data.empty()) {
    throw std::runtime_error("pcd DATA line missing");
  }
  return header;
}

const Field* find_field(const PcdHeader& header, const std::string& name) {
  for (const auto& field : header.fields) {
    if (field.name == name) {
      return &field;
    }
  }
  return nullptr;
}

double read_binary_scalar(const std::vector<char>& row, const Field* field) {
  if (field == nullptr) {
    return 0.0;
  }
  const char* data = row.data() + field->offset;
  if (field->type == 'F' && field->size == 4) {
    float value = 0.0F;
    std::memcpy(&value, data, sizeof(float));
    return value;
  }
  if (field->type == 'F' && field->size == 8) {
    double value = 0.0;
    std::memcpy(&value, data, sizeof(double));
    return value;
  }
  if (field->type == 'I' && field->size == 4) {
    int32_t value = 0;
    std::memcpy(&value, data, sizeof(int32_t));
    return value;
  }
  if (field->type == 'U' && field->size == 4) {
    uint32_t value = 0;
    std::memcpy(&value, data, sizeof(uint32_t));
    return value;
  }
  return 0.0;
}

std::vector<Point> read_pcd(const std::filesystem::path& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in.is_open()) {
    throw std::runtime_error("failed to open pcd: " + path.string());
  }
  PcdHeader header = read_pcd_header(in);
  const Field* fx = find_field(header, "x");
  const Field* fy = find_field(header, "y");
  const Field* fz = find_field(header, "z");
  const Field* fi = find_field(header, "intensity");
  if (fx == nullptr || fy == nullptr || fz == nullptr) {
    throw std::runtime_error("pcd missing x/y/z fields: " + path.string());
  }

  std::vector<Point> points;
  points.reserve(header.points);
  if (header.data == "ascii") {
    std::string line;
    while (std::getline(in, line)) {
      const auto tokens = split_ws(line);
      if (tokens.size() < header.fields.size()) {
        continue;
      }
      auto value_at = [&](const Field* field) -> float {
        if (field == nullptr) {
          return 0.0F;
        }
        for (std::size_t i = 0; i < header.fields.size(); ++i) {
          if (&header.fields[i] == field && i < tokens.size()) {
            double value = 0.0;
            return parse_double(tokens[i], value) ? static_cast<float>(value) : 0.0F;
          }
        }
        return 0.0F;
      };
      points.push_back(Point{
          value_at(fx),
          value_at(fy),
          value_at(fz),
          value_at(fi),
      });
    }
    return points;
  }
  if (header.data != "binary") {
    throw std::runtime_error("unsupported pcd DATA mode: " + header.data);
  }
  std::vector<char> row(header.point_step);
  for (std::size_t i = 0; i < header.points; ++i) {
    in.read(row.data(), static_cast<std::streamsize>(row.size()));
    if (in.gcount() != static_cast<std::streamsize>(row.size())) {
      break;
    }
    points.push_back(Point{
        static_cast<float>(read_binary_scalar(row, fx)),
        static_cast<float>(read_binary_scalar(row, fy)),
        static_cast<float>(read_binary_scalar(row, fz)),
        static_cast<float>(read_binary_scalar(row, fi)),
    });
  }
  return points;
}

}
std::vector<Point> read_point_cloud(const std::filesystem::path& path) {
  return read_pcd(path);
}

std::vector<std::filesystem::path> sorted_point_cloud_files(
    const std::filesystem::path& directory) {
  return list_patches(directory);
}

std::vector<Keyframe> read_poses(const std::filesystem::path& path) {
  std::ifstream in(path);
  if (!in.is_open()) {
    throw std::runtime_error("failed to open poses.txt");
  }
  std::vector<Keyframe> poses;
  std::string line;
  while (std::getline(in, line)) {
    const auto comment = line.find('#');
    if (comment != std::string::npos) {
      line = line.substr(0, comment);
    }
    const auto tokens = split_ws(line);
    if (tokens.empty()) {
      continue;
    }
    Keyframe keyframe;
    std::string parse_error;
    if (!parse_pose_tokens(tokens, keyframe, &parse_error)) {
      throw std::runtime_error(parse_error + ": " + line);
    }
    poses.push_back(keyframe);
  }
  if (poses.empty()) {
    throw std::runtime_error("poses.txt has no poses");
  }
  return poses;
}

}

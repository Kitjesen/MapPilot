#include "localization/opt/graph.hpp"
#include "localization/opt/cloud.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <system_error>

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

Pose inverse(const Pose& pose) {
  Pose out;
  out.qw = pose.qw;
  out.qx = -pose.qx;
  out.qy = -pose.qy;
  out.qz = -pose.qz;
  const std::array<double, 3> t{-pose.x, -pose.y, -pose.z};
  const std::array<double, 3> rotated = {
      (1.0 - 2.0 * (out.qy * out.qy + out.qz * out.qz)) * t[0] +
          2.0 * (out.qx * out.qy - out.qz * out.qw) * t[1] +
          2.0 * (out.qx * out.qz + out.qy * out.qw) * t[2],
      2.0 * (out.qx * out.qy + out.qz * out.qw) * t[0] +
          (1.0 - 2.0 * (out.qx * out.qx + out.qz * out.qz)) * t[1] +
          2.0 * (out.qy * out.qz - out.qx * out.qw) * t[2],
      2.0 * (out.qx * out.qz - out.qy * out.qw) * t[0] +
          2.0 * (out.qy * out.qz + out.qx * out.qw) * t[1] +
          (1.0 - 2.0 * (out.qx * out.qx + out.qy * out.qy)) * t[2]};
  out.x = rotated[0];
  out.y = rotated[1];
  out.z = rotated[2];
  return normalized(out);
}

std::array<double, 3> rotate_point(const Pose& pose, double x, double y, double z) {
  return {
      (1.0 - 2.0 * (pose.qy * pose.qy + pose.qz * pose.qz)) * x +
          2.0 * (pose.qx * pose.qy - pose.qz * pose.qw) * y +
          2.0 * (pose.qx * pose.qz + pose.qy * pose.qw) * z,
      2.0 * (pose.qx * pose.qy + pose.qz * pose.qw) * x +
          (1.0 - 2.0 * (pose.qx * pose.qx + pose.qz * pose.qz)) * y +
          2.0 * (pose.qy * pose.qz - pose.qx * pose.qw) * z,
      2.0 * (pose.qx * pose.qz - pose.qy * pose.qw) * x +
          2.0 * (pose.qy * pose.qz + pose.qx * pose.qw) * y +
          (1.0 - 2.0 * (pose.qx * pose.qx + pose.qy * pose.qy)) * z};
}

Pose compose(const Pose& a, const Pose& b) {
  Pose out;
  out.qw = a.qw * b.qw - a.qx * b.qx - a.qy * b.qy - a.qz * b.qz;
  out.qx = a.qw * b.qx + a.qx * b.qw + a.qy * b.qz - a.qz * b.qy;
  out.qy = a.qw * b.qy - a.qx * b.qz + a.qy * b.qw + a.qz * b.qx;
  out.qz = a.qw * b.qz + a.qx * b.qy - a.qy * b.qx + a.qz * b.qw;
  const auto t = rotate_point(a, b.x, b.y, b.z);
  out.x = a.x + t[0];
  out.y = a.y + t[1];
  out.z = a.z + t[2];
  return normalized(out);
}

Pose between(const Pose& from, const Pose& to) {
  return compose(inverse(from), to);
}

lt_pose_graph_opt_pose3 to_kernel_pose(const Pose& pose) {
  lt_pose_graph_opt_pose3 out{};
  out.t_xyz[0] = pose.x;
  out.t_xyz[1] = pose.y;
  out.t_xyz[2] = pose.z;
  out.q_wxyz[0] = pose.qw;
  out.q_wxyz[1] = pose.qx;
  out.q_wxyz[2] = pose.qy;
  out.q_wxyz[3] = pose.qz;
  return out;
}

Pose from_kernel_pose(const lt_pose_graph_opt_pose3& pose) {
  return normalized(Pose{
      pose.t_xyz[0],
      pose.t_xyz[1],
      pose.t_xyz[2],
      pose.q_wxyz[0],
      pose.q_wxyz[1],
      pose.q_wxyz[2],
      pose.q_wxyz[3],
  });
}

void fill_diag(double (&upper)[21], double rot_weight, double trans_weight) {
  const std::array<double, 6> diagonal{
      rot_weight,
      rot_weight,
      rot_weight,
      trans_weight,
      trans_weight,
      trans_weight,
  };
  std::fill(std::begin(upper), std::end(upper), 0.0);
  std::size_t idx = 0;
  for (int row = 0; row < 6; ++row) {
    for (int col = row; col < 6; ++col) {
      if (row == col) {
        upper[idx] = diagonal[static_cast<std::size_t>(row)];
      }
      ++idx;
    }
  }
}

bool valid_information_diagonal(const std::array<double, 6>& diagonal) {
  bool has_information = false;
  for (double value : diagonal) {
    if (!std::isfinite(value) || value < 0.0 || value > 1.0e12) {
      return false;
    }
    has_information = has_information || value > 0.0;
  }
  return has_information;
}

void fill_diag(double (&upper)[21], const std::array<double, 6>& diagonal) {
  std::fill(std::begin(upper), std::end(upper), 0.0);
  std::size_t idx = 0;
  for (int row = 0; row < 6; ++row) {
    for (int col = row; col < 6; ++col) {
      if (row == col) {
        upper[idx] = diagonal[static_cast<std::size_t>(row)];
      }
      ++idx;
    }
  }
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
  if (tokens.size() != 7 && tokens.size() != 8) {
    return fail("invalid_pose_row");
  }
  std::size_t offset = 0;
  if (tokens.size() == 8) {
    out.patch_name = tokens[0];
    offset = 1;
  }
  double values[7]{};
  for (std::size_t i = 0; i < 7; ++i) {
    if (!parse_double(tokens[offset + i], values[i])) {
      return fail("invalid_pose_row");
    }
  }
  out.pose.x = values[0];
  out.pose.y = values[1];
  out.pose.z = values[2];
  if (tokens.size() == 8) {
    // Canonical LingTu saved-map format:
    // patch_name x y z qw qx qy qz
    out.pose.qw = values[3];
    out.pose.qx = values[4];
    out.pose.qy = values[5];
    out.pose.qz = values[6];
  } else {
    // Legacy unnamed/TUM format: x y z qx qy qz qw. Quaternion order
    // cannot be inferred from component magnitudes (for example yaw=180).
    out.pose.qx = values[3];
    out.pose.qy = values[4];
    out.pose.qz = values[5];
    out.pose.qw = values[6];
  }
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

void write_pcd(const std::filesystem::path& path, const std::vector<Point>& points) {
  std::ofstream out(path, std::ios::binary);
  if (!out.is_open()) {
    throw std::runtime_error("failed to write pcd: " + path.string());
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
  for (const Point& point : points) {
    const float row[4] = {point.x, point.y, point.z, point.intensity};
    out.write(reinterpret_cast<const char*>(row), sizeof(row));
  }
}

std::filesystem::path backup_path(const std::filesystem::path& path, const std::string& suffix) {
  std::filesystem::path candidate = path;
  candidate += suffix;
  if (!std::filesystem::exists(candidate)) {
    return candidate;
  }
  for (int i = 1; i < 1000; ++i) {
    std::filesystem::path numbered = path;
    numbered += suffix + "." + std::to_string(i);
    if (!std::filesystem::exists(numbered)) {
      return numbered;
    }
  }
  const auto now = std::chrono::system_clock::now().time_since_epoch().count();
  std::filesystem::path fallback = path;
  fallback += suffix + "." + std::to_string(now);
  return fallback;
}

std::vector<Point> transform_points(const std::vector<Point>& points, const Pose& pose) {
  std::vector<Point> out;
  out.reserve(points.size());
  for (const Point& point : points) {
    const auto rotated = rotate_point(pose, point.x, point.y, point.z);
    out.push_back(Point{
        static_cast<float>(rotated[0] + pose.x),
        static_cast<float>(rotated[1] + pose.y),
        static_cast<float>(rotated[2] + pose.z),
        point.intensity,
    });
  }
  return out;
}

void append_points(std::vector<Point>& dst, std::vector<Point>&& src) {
  dst.reserve(dst.size() + src.size());
  std::move(src.begin(), src.end(), std::back_inserter(dst));
}

void write_poses(const std::filesystem::path& path, const std::vector<Keyframe>& poses) {
  std::ofstream out(path);
  if (!out.is_open()) {
    throw std::runtime_error("failed to write poses: " + path.string());
  }
  out << std::setprecision(17);
  for (const Keyframe& keyframe : poses) {
    if (!keyframe.patch_name.empty()) {
      out << keyframe.patch_name << " ";
    }
    out << keyframe.pose.x << " " << keyframe.pose.y << " " << keyframe.pose.z << " "
        << keyframe.pose.qw << " " << keyframe.pose.qx << " " << keyframe.pose.qy << " "
        << keyframe.pose.qz << "\n";
  }
}

std::string json_escape(const std::string& value) {
  std::string out;
  out.reserve(value.size());
  for (char c : value) {
    if (c == '\\' || c == '"') {
      out.push_back('\\');
    }
    out.push_back(c);
  }
  return out;
}

void write_report(
    const std::filesystem::path& path,
    const OptimizeOptions& options,
    const Result& result,
    const lt_pose_graph_opt_report& report,
    const std::filesystem::path& map_backup,
    const std::filesystem::path& poses_backup) {
  std::ofstream out(path);
  if (!out.is_open()) {
    return;
  }
  out << std::setprecision(17);
  out << "{\n";
  out << "  \"schema\": \"lingtu.map_optimization.v1\",\n";
  out << "  \"strategy\": \"" << json_escape(options.strategy) << "\",\n";
  out << "  \"backend\": \"pose_graph_opt\",\n";
  out << "  \"success\": " << (result.ok ? "true" : "false") << ",\n";
  out << "  \"code\": \"" << json_escape(result.code) << "\",\n";
  out << "  \"message\": \"" << json_escape(result.message) << "\",\n";
  out << "  \"pose_count\": " << result.pose_count << ",\n";
  out << "  \"patch_count\": " << result.patch_count << ",\n";
  out << "  \"factor_count\": " << result.factor_count << ",\n";
  out << "  \"iterations\": " << report.iterations << ",\n";
  out << "  \"accepted_steps\": " << report.accepted_steps << ",\n";
  out << "  \"rejected_steps\": " << report.rejected_steps << ",\n";
  out << "  \"converged\": " << (report.converged ? "true" : "false") << ",\n";
  out << "  \"initial_cost\": " << report.initial_cost << ",\n";
  out << "  \"final_cost\": " << report.final_cost << ",\n";
  out << "  \"map_backup\": \"" << json_escape(map_backup.string()) << "\",\n";
  out << "  \"poses_backup\": \"" << json_escape(poses_backup.string()) << "\"\n";
  out << "}\n";
}

Result fail_from_exception(const Result& base, const std::exception& exc, std::string code) {
  Result result = base;
  result.ok = false;
  result.code = std::move(code);
  result.message = exc.what();
  return result;
}

}  // namespace

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

Result optimize_map(const Map& map, const OptimizeOptions& options) {
  Result result = check(map);
  if (!result.ok) {
    return result;
  }

  try {
    std::filesystem::create_directories(map.output_dir);
    std::vector<Keyframe> keyframes = read_poses(map.poses_txt);
    const auto patches = list_patches(map.patches_dir);
    if (patches.size() < keyframes.size()) {
      result.ok = false;
      result.code = "patch_pose_mismatch";
      result.message = "patch count is smaller than poses.txt pose count";
      result.pose_count = keyframes.size();
      return result;
    }
    for (std::size_t i = 0; i < keyframes.size(); ++i) {
      if (keyframes[i].patch_name.empty()) {
        keyframes[i].patch_name = patches[i].filename().string();
      }
    }

    if (options.geometric_constraints.empty()) {
      // Chain and skip factors below are derived from the same initial poses
      // they are meant to optimize.  Such a graph starts at zero residual and
      // contains no independent information.  Rebuilding from it can only
      // discard an upstream SLAM correction, so preserve the source artifacts.
      result.ok = true;
      result.code = "skipped_no_independent_constraints";
      result.message = options.strategy +
          " skipped: graph contains only constraints derived from the input odometry poses";
      result.patch_count = patches.size();
      result.pose_count = keyframes.size();
      result.factor_count = 0;
      result.iterations = 0;
      result.changed = false;
      const auto existing_report = map.map_dir / "map_optimization.json";
      if (std::filesystem::is_regular_file(existing_report)) {
        result.report_path = existing_report;
      }
      return result;
    }

    std::vector<lt_pose_graph_opt_pose3> poses;
    poses.reserve(keyframes.size());
    for (const Keyframe& keyframe : keyframes) {
      poses.push_back(to_kernel_pose(keyframe.pose));
    }

    std::vector<lt_pose_graph_opt_prior3> priors;
    lt_pose_graph_opt_prior3 prior{};
    prior.index = 0;
    prior.pose = poses.front();
    fill_diag(prior.information_upper, options.prior_weight, options.prior_weight);
    priors.push_back(prior);

    std::vector<lt_pose_graph_opt_between3> betweens;
    for (std::size_t i = 1; i < keyframes.size(); ++i) {
      lt_pose_graph_opt_between3 edge{};
      edge.from_index = static_cast<uint32_t>(i - 1);
      edge.to_index = static_cast<uint32_t>(i);
      edge.pose_from_to = to_kernel_pose(between(keyframes[i - 1].pose, keyframes[i].pose));
      fill_diag(edge.information_upper, options.chain_rot_weight, options.chain_trans_weight);
      betweens.push_back(edge);
    }
    if (options.skip_stride > 1) {
      for (std::size_t i = 0; i + options.skip_stride < keyframes.size(); ++i) {
        lt_pose_graph_opt_between3 edge{};
        edge.from_index = static_cast<uint32_t>(i);
        edge.to_index = static_cast<uint32_t>(i + options.skip_stride);
        edge.pose_from_to = to_kernel_pose(
            between(keyframes[i].pose, keyframes[i + options.skip_stride].pose));
        fill_diag(edge.information_upper, options.skip_rot_weight, options.skip_trans_weight);
        betweens.push_back(edge);
      }
    }
    for (const GeometricConstraint& constraint : options.geometric_constraints) {
      if (constraint.from_index >= keyframes.size() ||
          constraint.to_index >= keyframes.size() ||
          constraint.from_index == constraint.to_index ||
          !valid_information_diagonal(constraint.information_diagonal)) {
        result.ok = false;
        result.code = "geometric_constraint_invalid";
        result.message =
            "independent geometric constraint has invalid indices or information diagonal";
        result.pose_count = keyframes.size();
        return result;
      }
      lt_pose_graph_opt_between3 edge{};
      edge.from_index = static_cast<uint32_t>(constraint.from_index);
      edge.to_index = static_cast<uint32_t>(constraint.to_index);
      edge.pose_from_to = to_kernel_pose(constraint.pose_from_to);
      fill_diag(edge.information_upper, constraint.information_diagonal);
      betweens.push_back(edge);
    }

    lt_pose_graph_opt_config config{};
    config.struct_size = sizeof(lt_pose_graph_opt_config);
    config.version = LT_POSE_GRAPH_OPT_CONFIG_VERSION;
    config.max_iterations = static_cast<uint32_t>(options.max_iterations);
    config.method = 1;
    config.fixed_pose_index = 0;
    config.auto_anchor = 1;
    config.initial_lambda = 1e-3;
    config.tolerance = 1e-9;
    config.numeric_epsilon = 1e-6;

    lt_pose_graph_opt_handle* handle = lt_pose_graph_opt_create(&config);
    if (handle == nullptr) {
      result.ok = false;
      result.code = "optimizer_create_failed";
      result.message = "pose graph optimizer could not be created";
      return result;
    }

    lt_pose_graph_opt_report report{};
    const lt_pose_graph_opt_result status = lt_pose_graph_opt_process_se3(
        handle,
        poses.data(),
        static_cast<uint64_t>(poses.size()),
        priors.data(),
        static_cast<uint64_t>(priors.size()),
        betweens.empty() ? nullptr : betweens.data(),
        static_cast<uint64_t>(betweens.size()),
        &report);
    if (status != LT_POSE_GRAPH_OPT_OK) {
      lt_pose_graph_opt_destroy(handle);
      result.ok = false;
      result.code = "optimizer_failed";
      result.message = "pose graph optimizer failed: " + std::to_string(status);
      result.pose_count = keyframes.size();
      result.factor_count = priors.size() + betweens.size();
      return result;
    }

    uint64_t written = 0;
    const lt_pose_graph_opt_result copy_status = lt_pose_graph_opt_copy_result_poses(
        handle,
        poses.data(),
        static_cast<uint64_t>(poses.size()),
        &written);
    lt_pose_graph_opt_destroy(handle);
    if (copy_status != LT_POSE_GRAPH_OPT_OK || written != poses.size()) {
      result.ok = false;
      result.code = "optimizer_copy_failed";
      result.message = "pose graph optimizer did not return all poses";
      return result;
    }

    for (std::size_t i = 0; i < keyframes.size(); ++i) {
      keyframes[i].pose = from_kernel_pose(poses[i]);
    }

    std::vector<Point> map_points;
    for (const Keyframe& keyframe : keyframes) {
      const auto patch_path = map.patches_dir / keyframe.patch_name;
      if (!std::filesystem::is_regular_file(patch_path)) {
        result.ok = false;
        result.code = "patch_missing";
        result.message = "patch file missing: " + patch_path.string();
        return result;
      }
      append_points(map_points, transform_points(read_pcd(patch_path), keyframe.pose));
    }

    const std::filesystem::path out_map = map.output_dir / "map.pcd";
    const std::filesystem::path out_poses = map.output_dir / "poses.txt";
    std::filesystem::path map_backup;
    std::filesystem::path poses_backup;
    if (std::filesystem::equivalent(map.output_dir, map.map_dir)) {
      map_backup = backup_path(map.map_pcd, ".preopt");
      poses_backup = backup_path(map.poses_txt, ".preopt");
      std::filesystem::rename(map.map_pcd, map_backup);
      std::filesystem::rename(map.poses_txt, poses_backup);
    }
    write_pcd(out_map, map_points);
    write_poses(out_poses, keyframes);
    write_poses(map.output_dir / "poses_optimized.txt", keyframes);

    result.ok = true;
    result.code = "optimized";
    result.message = options.strategy + " optimization completed";
    result.patch_count = keyframes.size();
    result.pose_count = keyframes.size();
    result.factor_count = priors.size() + betweens.size();
    result.iterations = report.iterations;
    result.changed = true;
    result.report_path = map.output_dir / "map_optimization.json";
    write_report(result.report_path, options, result, report, map_backup, poses_backup);
    return result;
  } catch (const std::exception& exc) {
    return fail_from_exception(result, exc, "optimizer_io_failed");
  }
}

}  // namespace lingtu::localization::opt

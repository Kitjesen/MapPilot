#include "stager.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace fs = std::filesystem;

namespace lingtu::map_cleaning {
namespace {

struct PointXYZI {
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float intensity{0.0F};
};

struct Pose {
  double tx{0.0};
  double ty{0.0};
  double tz{0.0};
  double qw{1.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
};

struct PcdHeader {
  std::vector<std::string> fields;
  std::vector<int> sizes;
  std::vector<char> types;
  std::vector<int> counts;
  std::uint64_t points{0};
  std::string data;
  std::streampos data_pos{};
};

std::string trim(std::string value) {
  auto not_space = [](unsigned char c) { return !std::isspace(c); };
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
  return value;
}

std::vector<std::string> splitWords(const std::string &line) {
  std::istringstream in(line);
  std::vector<std::string> out;
  std::string item;
  while (in >> item) {
    out.push_back(item);
  }
  return out;
}

std::string jsonEscape(const std::string &value) {
  std::ostringstream out;
  for (char c : value) {
    switch (c) {
      case '\\':
        out << "\\\\";
        break;
      case '"':
        out << "\\\"";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        out << c;
        break;
    }
  }
  return out.str();
}

std::string genericString(const fs::path &path) {
  return path.empty() ? std::string() : path.generic_string();
}

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

PcdHeader readPcdHeader(std::ifstream &in, const fs::path &path) {
  PcdHeader header;
  std::string line;
  while (std::getline(in, line)) {
    std::string cleaned = trim(line);
    if (cleaned.empty() || cleaned[0] == '#') {
      continue;
    }
    std::vector<std::string> parts = splitWords(cleaned);
    if (parts.empty()) {
      continue;
    }
    const std::string key = parts[0];
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
      header.data_pos = in.tellg();
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

std::vector<PointXYZI> readPcd(const fs::path &path) {
  std::ifstream in(path, std::ios::binary);
  if (!in.is_open()) {
    throw std::runtime_error("failed to open PCD: " + path.string());
  }
  PcdHeader header = readPcdHeader(in, path);
  const int x_idx = fieldIndex(header, "x");
  const int y_idx = fieldIndex(header, "y");
  const int z_idx = fieldIndex(header, "z");
  const int intensity_idx = fieldIndex(header, "intensity");
  std::vector<PointXYZI> points;
  points.reserve(static_cast<std::size_t>(header.points));

  if (header.data == "ascii") {
    std::string line;
    while (std::getline(in, line)) {
      std::vector<std::string> values = splitWords(line);
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

std::unordered_map<std::string, Pose> readLingtuPoses(const fs::path &path) {
  std::unordered_map<std::string, Pose> poses;
  std::ifstream in(path);
  if (!in.is_open()) {
    throw std::runtime_error("failed to open poses.txt: " + path.string());
  }
  std::string line;
  while (std::getline(in, line)) {
    std::vector<std::string> parts = splitWords(line);
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

void writeBinCloud(const fs::path &path, const std::vector<PointXYZI> &points) {
  std::ofstream out(path, std::ios::binary);
  if (!out.is_open()) {
    throw std::runtime_error("failed to write scan bin: " + path.string());
  }
  for (const PointXYZI &pt : points) {
    const float row[4] = {pt.x, pt.y, pt.z, pt.intensity};
    out.write(reinterpret_cast<const char *>(row), sizeof(row));
  }
}

void writeLabels(const fs::path &path, const std::vector<std::uint32_t> &labels) {
  std::ofstream out(path, std::ios::binary);
  if (!out.is_open()) {
    throw std::runtime_error("failed to write label file: " + path.string());
  }
  out.write(reinterpret_cast<const char *>(labels.data()),
            static_cast<std::streamsize>(labels.size() * sizeof(std::uint32_t)));
}

std::string frameName(std::size_t index, const std::string &extension) {
  std::ostringstream out;
  out << std::setw(6) << std::setfill('0') << index << extension;
  return out.str();
}

void writeConfig(const fs::path &path, const Erasor2StageOptions &options,
                 const fs::path &dataset_root, const fs::path &output_root,
                 std::size_t frame_count) {
  std::ofstream out(path);
  if (!out.is_open()) {
    throw std::runtime_error("failed to write ERASOR2 config: " + path.string());
  }
  const std::size_t end_frame = frame_count == 0 ? 0 : frame_count - 1;
  out << "start_frame: 0\n";
  out << "end_frame: " << end_frame << "\n";
  out << "viz_interval: 10\n";
  out << "is_large_scale: true\n";
  out << "stop_for_each_frame: false\n";
  out << "\n";
  out << "dataloader:\n";
  out << "  dataset_name: \"HeLiPR\"\n";
  out << "  abs_data_dir: \"" << dataset_root.generic_string() << "\"\n";
  out << "  sequence: \"LingTu\"\n";
  out << "  abs_save_dir: \"" << output_root.generic_string() << "\"\n";
  out << "  instance_seg_method: \"hdbscan\"\n";
  out << "  accum_interval: 1\n";
  out << "  voxel_size: 0.2\n";
  out << "  map_voxel_size: 0.2\n";
  out << "\n";
  out << "erasor2:\n";
  out << "  grid_resolution: " << options.grid_resolution_m << "\n";
  out << "  egocentric_grid_resolution: 0.6\n";
  out << "  range_of_interest: " << options.range_of_interest_m << "\n";
  out << "  min_z_voi: -3.0\n";
  out << "  max_z_voi: 1.5\n";
  out << "  min_z_diff_thr: 0.4\n";
  out << "  scan_ratio_threshold: 0.2\n";
  out << "  log_odds:\n";
  out << "    increment_gain: 2.0\n";
  out << "    increment: 0.15\n";
  out << "  region_proposal_thr: 0.8\n";
  out << "  kernel_size: 1\n";
  out << "  ratio_num_pts: 0.95\n";
  out << "  minimum_num_pts: 5\n";
  out << "  moving_object_detection:\n";
  out << "    negative_log_odds: -2.0\n";
  out << "    obj_score_soft_thr: 0.8\n";
  out << "    obj_score_hard_thr: 14.0\n";
  out << "    hard_thr_radius: 10.0\n";
  out << "  volumetric_outlier_removal:\n";
  out << "    window_size: 1\n";
  out << "    use_adaptive_voxel_size: true\n";
  out << "    vor_cand_score_thr: 3.0\n";
  out << "    dist_thr_gain: 2.0\n";
  out << "  save_map: true\n";
  out << "  viz_flag:\n";
  out << "    set_scan_and_pose: false\n";
  out << "    set_submap: false\n";
  out << "    update: false\n";
  out << "    detect: false\n";
  out << "    over_seg: false\n";
  out << "\n";
  out << "extrinsic:\n";
  out << "  robot_body_size: 1.0\n";
  out << "  sensor_height: " << options.sensor_height_m << "\n";
  out << "  rotation: [1, 0, 0, 0, 1, 0, 0, 0, 1]\n";
  out << "  translation: [0.0, 0.0, 0.0]\n";
  out << "\n";
  out << "rerun:\n";
  out << "  enabled: false\n";
  out << "  spawn: false\n";
  out << "  save_path: \"\"\n";
}

Erasor2StageResult fail(std::string reason, std::string message) {
  Erasor2StageResult result;
  result.success = false;
  result.reason_code = std::move(reason);
  result.message = std::move(message);
  return result;
}

}  // namespace

Erasor2StageResult stageErasor2Dataset(const Erasor2StageOptions &options) {
  try {
    if (options.map_dir.empty()) {
      return fail("missing_map_dir", "missing --map-dir");
    }
    if (options.output_dir.empty()) {
      return fail("missing_output_dir", "missing --out");
    }
    const fs::path map_dir = fs::absolute(options.map_dir);
    const fs::path patches_dir = map_dir / "patches";
    const fs::path poses_path = map_dir / "poses.txt";
    const fs::path map_pcd = map_dir / "map.pcd";
    if (!fs::is_regular_file(map_pcd)) {
      return fail("missing_map_pcd", "map.pcd not found: " + map_pcd.string());
    }
    if (!fs::is_directory(patches_dir)) {
      return fail("missing_patches", "patches directory not found: " + patches_dir.string());
    }
    if (!fs::is_regular_file(poses_path)) {
      return fail("missing_poses", "poses.txt not found: " + poses_path.string());
    }
    if (fs::exists(options.output_dir) && !options.overwrite) {
      return fail("output_exists",
                  "output exists; pass --overwrite to replace: " + options.output_dir.string());
    }
    if (options.instance_grid_m <= 0.0F) {
      return fail("bad_instance_grid", "instance grid size must be positive");
    }

    const fs::path stage_root = fs::absolute(options.output_dir);
    const fs::path dataset_root = stage_root / "dataset";
    const fs::path sequence_root = dataset_root / "LingTu";
    const fs::path scans_dir = sequence_root / "velodyne";
    const fs::path ground_dir = sequence_root / "patchwork";
    const fs::path inst_dir = sequence_root / "hdbscan";
    const fs::path output_root = stage_root / "output";
    const fs::path config_path = stage_root / "erasor2.yaml";

    if (fs::exists(stage_root)) {
      fs::remove_all(stage_root);
    }
    fs::create_directories(scans_dir);
    fs::create_directories(ground_dir);
    fs::create_directories(inst_dir);
    fs::create_directories(output_root);

    std::vector<fs::path> patches;
    for (const fs::directory_entry &entry : fs::directory_iterator(patches_dir)) {
      if (entry.is_regular_file() && entry.path().extension() == ".pcd") {
        patches.push_back(entry.path());
      }
    }
    std::sort(patches.begin(), patches.end());
    if (patches.empty()) {
      return fail("no_patches", "no patch PCD files found in " + patches_dir.string());
    }

    const auto poses = readLingtuPoses(poses_path);
    if (poses.empty()) {
      return fail("no_poses", "no usable LingTu poses found in " + poses_path.string());
    }

    std::ofstream pose_out(sequence_root / "poses.txt");
    if (!pose_out.is_open()) {
      throw std::runtime_error("failed to write staged poses.txt");
    }

    Erasor2StageResult result;
    result.success = true;
    result.reason_code = "staged";
    result.message =
        "ERASOR2 staging dataset created; upstream ERASOR2 execution is a separate GPLv3 tool step";
    result.dataset_dir = dataset_root;
    result.config_path = config_path;
    result.output_dir = output_root;

    for (const fs::path &patch : patches) {
      const std::string patch_name = patch.filename().string();
      auto pose_it = poses.find(patch_name);
      if (pose_it == poses.end()) {
        continue;
      }
      std::vector<PointXYZI> points = readPcd(patch);
      if (points.empty()) {
        continue;
      }

      const std::size_t frame_idx = result.frames.size();
      std::vector<std::uint32_t> ground_labels;
      std::vector<std::uint32_t> instance_labels;
      ground_labels.reserve(points.size());
      instance_labels.reserve(points.size());
      std::map<std::pair<int, int>, std::uint32_t> cell_ids;
      std::uint64_t ground_count = 0;
      std::uint32_t next_id = 1;

      for (const PointXYZI &pt : points) {
        const bool is_ground = pt.z <= options.ground_z_threshold;
        ground_labels.push_back(is_ground ? 1U : 0U);
        if (is_ground) {
          instance_labels.push_back(0U);
          ++ground_count;
          continue;
        }
        const int gx = static_cast<int>(std::floor(pt.x / options.instance_grid_m));
        const int gy = static_cast<int>(std::floor(pt.y / options.instance_grid_m));
        const auto key = std::make_pair(gx, gy);
        auto inserted = cell_ids.emplace(key, next_id);
        if (inserted.second) {
          ++next_id;
        }
        instance_labels.push_back(inserted.first->second << 16U);
      }

      const fs::path scan_path = scans_dir / frameName(frame_idx, ".bin");
      const fs::path ground_path = ground_dir / frameName(frame_idx, ".label");
      const fs::path inst_path = inst_dir / frameName(frame_idx, ".label");
      writeBinCloud(scan_path, points);
      writeLabels(ground_path, ground_labels);
      writeLabels(inst_path, instance_labels);

      const Pose &pose = pose_it->second;
      pose_out << "0 " << std::setprecision(16) << pose.tx << " " << pose.ty << " " << pose.tz
               << " " << pose.qx << " " << pose.qy << " " << pose.qz << " " << pose.qw << "\n";

      Erasor2StageFrame frame;
      frame.source_patch = patch_name;
      frame.scan_bin = scan_path;
      frame.ground_label = ground_path;
      frame.instance_label = inst_path;
      frame.point_count = points.size();
      frame.ground_points = ground_count;
      frame.instance_count = cell_ids.size();
      result.total_points += frame.point_count;
      result.total_ground_points += frame.ground_points;
      result.frames.push_back(std::move(frame));
    }

    if (result.frames.empty()) {
      return fail("no_matched_frames", "no patch PCD files matched poses.txt");
    }

    writeConfig(config_path, options, dataset_root, output_root, result.frames.size());
    return result;
  } catch (const std::exception &exc) {
    return fail("exception", exc.what());
  }
}

std::string toJson(const Erasor2StageResult &result) {
  std::ostringstream out;
  out << "{\n";
  out << "  \"success\": " << (result.success ? "true" : "false") << ",\n";
  out << "  \"reason_code\": \"" << jsonEscape(result.reason_code) << "\",\n";
  out << "  \"message\": \"" << jsonEscape(result.message) << "\",\n";
  out << "  \"dataset_dir\": \"" << jsonEscape(genericString(result.dataset_dir)) << "\",\n";
  out << "  \"config_path\": \"" << jsonEscape(genericString(result.config_path)) << "\",\n";
  out << "  \"output_dir\": \"" << jsonEscape(genericString(result.output_dir)) << "\",\n";
  out << "  \"frame_count\": " << result.frames.size() << ",\n";
  out << "  \"total_points\": " << result.total_points << ",\n";
  out << "  \"total_ground_points\": " << result.total_ground_points << ",\n";
  out << "  \"license_boundary\": \"ERASOR2 upstream is GPLv3; this result only stages data for an "
         "external optional tool\",\n";
  out << "  \"frames\": [";
  for (std::size_t i = 0; i < result.frames.size(); ++i) {
    const Erasor2StageFrame &frame = result.frames[i];
    out << (i == 0 ? "\n" : ",\n");
    out << "    {";
    out << "\"source_patch\": \"" << jsonEscape(frame.source_patch) << "\", ";
    out << "\"points\": " << frame.point_count << ", ";
    out << "\"ground_points\": " << frame.ground_points << ", ";
    out << "\"instance_count\": " << frame.instance_count << "}";
  }
  if (!result.frames.empty()) {
    out << "\n  ";
  }
  out << "]\n";
  out << "}\n";
  return out.str();
}

}  // namespace lingtu::map_cleaning

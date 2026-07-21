#include "lingtu/maps/sources/unity_scene.hpp"

#include "lingtu/maps/semantic_map_persistence.hpp"
#include "lingtu/maps/semantic_taxonomy.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <locale>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <unordered_map>
#include <utility>

namespace lingtu::maps::sources {
namespace {

struct CategoryRow {
  std::string name;
  std::string cleaned;
  std::string nyu_class;
  std::string nyu40_class;
};

struct ObjectRow {
  std::uint64_t instance_id{0U};
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double size_x{0.0};
  double size_y{0.0};
  double size_z{0.0};
  double yaw{0.0};
  std::string label;
};

struct VoxelKey {
  std::int32_t x{0};
  std::int32_t y{0};
  std::int32_t z{0};

  bool operator==(const VoxelKey& other) const noexcept {
    return x == other.x && y == other.y && z == other.z;
  }

  bool operator<(const VoxelKey& other) const noexcept {
    if (x != other.x) return x < other.x;
    if (y != other.y) return y < other.y;
    return z < other.z;
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey& key) const noexcept {
    std::uint64_t hash = 14695981039346656037ULL;
    const auto mix = [&hash](std::uint32_t value) {
      hash ^= value;
      hash *= 1099511628211ULL;
    };
    mix(static_cast<std::uint32_t>(key.x));
    mix(static_cast<std::uint32_t>(key.y));
    mix(static_cast<std::uint32_t>(key.z));
    return static_cast<std::size_t>(hash);
  }
};

struct CellAccumulator {
  std::uint16_t label{0U};
  double priority_volume{std::numeric_limits<double>::infinity()};
  std::uint64_t priority_instance{std::numeric_limits<std::uint64_t>::max()};
  std::uint32_t source_count{0U};
  std::uint32_t chosen_label_count{0U};
};

using CsvTable = std::vector<std::vector<std::string>>;

std::string ReadText(const std::filesystem::path& path) {
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    throw std::runtime_error("failed to open Unity semantic source: " + path.string());
  }
  std::string text((std::istreambuf_iterator<char>(file)), {});
  if (text.size() >= 3U && static_cast<unsigned char>(text[0]) == 0xEFU &&
      static_cast<unsigned char>(text[1]) == 0xBBU &&
      static_cast<unsigned char>(text[2]) == 0xBFU) {
    text.erase(0U, 3U);
  }
  return text;
}

CsvTable ParseCsv(const std::filesystem::path& path) {
  const std::string input = ReadText(path);
  CsvTable rows;
  std::vector<std::string> row;
  std::string field;
  bool quoted = false;
  bool after_quote = false;
  for (std::size_t cursor = 0U; cursor <= input.size(); ++cursor) {
    const char ch = cursor < input.size() ? input[cursor] : '\n';
    if (quoted) {
      if (ch == '"') {
        if (cursor + 1U < input.size() && input[cursor + 1U] == '"') {
          field.push_back('"');
          ++cursor;
        } else {
          quoted = false;
          after_quote = true;
        }
      } else {
        field.push_back(ch);
      }
      continue;
    }
    if (after_quote && ch != ',' && ch != '\r' && ch != '\n' &&
        std::isspace(static_cast<unsigned char>(ch)) == 0) {
      throw std::invalid_argument(
          "unexpected data after quoted CSV field in " + path.string());
    }
    if (!after_quote && ch == '"') {
      if (!field.empty()) {
        throw std::invalid_argument("quote inside unquoted CSV field in " + path.string());
      }
      quoted = true;
      continue;
    }
    if (ch == ',') {
      row.push_back(std::move(field));
      field.clear();
      after_quote = false;
      continue;
    }
    if (ch == '\r' || ch == '\n') {
      if (ch == '\r' && cursor + 1U < input.size() && input[cursor + 1U] == '\n') {
        ++cursor;
      }
      row.push_back(std::move(field));
      field.clear();
      after_quote = false;
      const bool nonempty = std::any_of(
          row.begin(), row.end(), [](const std::string& value) { return !value.empty(); });
      if (nonempty) {
        rows.push_back(std::move(row));
      }
      row.clear();
      continue;
    }
    if (!after_quote) {
      field.push_back(ch);
    }
  }
  if (quoted) {
    throw std::invalid_argument("unterminated quoted CSV field in " + path.string());
  }
  return rows;
}

std::string HeaderKey(std::string_view value) {
  std::string key;
  for (const char ch : value) {
    if (std::isalnum(static_cast<unsigned char>(ch)) != 0) {
      key.push_back(static_cast<char>(
          std::tolower(static_cast<unsigned char>(ch))));
    }
  }
  return key;
}

std::size_t RequiredColumn(
    const std::unordered_map<std::string, std::size_t>& columns,
    const std::string& name,
    const std::filesystem::path& path) {
  const auto found = columns.find(HeaderKey(name));
  if (found == columns.end()) {
    throw std::invalid_argument(
        "Unity categories CSV is missing column '" + name + "': " + path.string());
  }
  return found->second;
}

std::vector<CategoryRow> LoadCategories(const std::filesystem::path& path) {
  const CsvTable rows = ParseCsv(path);
  if (rows.size() < 2U) {
    throw std::invalid_argument("Unity categories CSV has no data rows: " + path.string());
  }
  std::unordered_map<std::string, std::size_t> columns;
  for (std::size_t index = 0U; index < rows.front().size(); ++index) {
    const std::string key = HeaderKey(rows.front()[index]);
    if (key.empty() || !columns.emplace(key, index).second) {
      throw std::invalid_argument("Unity categories CSV has invalid/duplicate headers");
    }
  }
  const std::size_t name = RequiredColumn(columns, "name", path);
  const std::size_t cleaned = RequiredColumn(columns, "cleaned", path);
  const std::size_t nyu_class = RequiredColumn(columns, "nyuClass", path);
  const std::size_t nyu40_class = RequiredColumn(columns, "nyu40class", path);
  const std::size_t required_width =
      std::max({name, cleaned, nyu_class, nyu40_class}) + 1U;
  std::vector<CategoryRow> categories;
  categories.reserve(rows.size() - 1U);
  for (std::size_t row_index = 1U; row_index < rows.size(); ++row_index) {
    if (rows[row_index].size() < required_width) {
      throw std::invalid_argument(
          "Unity categories CSV row " + std::to_string(row_index + 1U) + " is truncated");
    }
    CategoryRow row;
    row.name = rows[row_index][name];
    row.cleaned = rows[row_index][cleaned];
    row.nyu_class = rows[row_index][nyu_class];
    row.nyu40_class = rows[row_index][nyu40_class];
    categories.push_back(std::move(row));
  }
  return categories;
}

std::vector<ObjectRow> LoadObjects(
    const std::filesystem::path& path,
    std::size_t max_objects) {
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("failed to open Unity object list: " + path.string());
  }
  file.imbue(std::locale::classic());
  std::vector<ObjectRow> objects;
  std::string line;
  std::size_t line_number = 0U;
  while (std::getline(file, line)) {
    ++line_number;
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    const auto first = line.find_first_not_of(" \t");
    if (first == std::string::npos || line[first] == '#') {
      continue;
    }
    std::istringstream stream(line);
    stream.imbue(std::locale::classic());
    ObjectRow object;
    if (!(stream >> object.instance_id >> object.x >> object.y >> object.z >> object.size_x >>
          object.size_y >> object.size_z >> object.yaw >> std::quoted(object.label))) {
      throw std::invalid_argument(
          "invalid Unity object_list row " + std::to_string(line_number));
    }
    std::string trailing;
    if (stream >> trailing) {
      throw std::invalid_argument(
          "unexpected trailing data in Unity object_list row " + std::to_string(line_number));
    }
    if (!std::isfinite(object.x) || !std::isfinite(object.y) || !std::isfinite(object.z) ||
        !std::isfinite(object.size_x) || !std::isfinite(object.size_y) ||
        !std::isfinite(object.size_z) || !std::isfinite(object.yaw) ||
        object.size_x <= 0.0 || object.size_y <= 0.0 || object.size_z <= 0.0 ||
        SemanticTaxonomy::NormalizeLabel(object.label).empty()) {
      throw std::invalid_argument(
          "non-finite/empty Unity object in row " + std::to_string(line_number));
    }
    if (objects.size() >= max_objects) {
      throw std::length_error("Unity object count exceeds configured limit");
    }
    objects.push_back(std::move(object));
  }
  if (objects.empty()) {
    throw std::invalid_argument("Unity object list is empty: " + path.string());
  }
  return objects;
}

std::optional<std::uint16_t> ResolveLabel(
    const std::string& label,
    const std::unordered_multimap<std::string, const CategoryRow*>& categories,
    const SemanticTaxonomy& taxonomy) {
  if (const auto direct = taxonomy.Resolve(label)) {
    return direct;
  }
  const std::string normalized = SemanticTaxonomy::NormalizeLabel(label);
  const auto range = categories.equal_range(normalized);
  std::set<std::uint16_t> candidates;
  for (auto item = range.first; item != range.second; ++item) {
    const CategoryRow& category = *item->second;
    for (const auto* candidate : {
             &category.cleaned, &category.nyu_class, &category.nyu40_class, &category.name}) {
      if (const auto resolved = taxonomy.Resolve(*candidate)) {
        candidates.insert(*resolved);
      }
    }
  }
  if (candidates.size() == 1U) {
    return *candidates.begin();
  }
  return std::nullopt;
}

std::int32_t ToIndex(double coordinate, double voxel_size) {
  const double index = std::floor(coordinate / voxel_size);
  if (!std::isfinite(index) ||
      index < static_cast<double>(std::numeric_limits<std::int32_t>::min()) ||
      index > static_cast<double>(std::numeric_limits<std::int32_t>::max())) {
    throw std::overflow_error("Unity semantic voxel index exceeds int32");
  }
  return static_cast<std::int32_t>(index);
}

double Center(std::int32_t index, double voxel_size) {
  return (static_cast<double>(index) + 0.5) * voxel_size;
}

std::size_t CheckedCellCount(
    std::int64_t x_count,
    std::int64_t y_count,
    std::int64_t z_count) {
  if (x_count <= 0 || y_count <= 0 || z_count <= 0) {
    return 0U;
  }
  const std::uint64_t x = static_cast<std::uint64_t>(x_count);
  const std::uint64_t y = static_cast<std::uint64_t>(y_count);
  const std::uint64_t z = static_cast<std::uint64_t>(z_count);
  if (x > std::numeric_limits<std::size_t>::max() / y ||
      x * y > std::numeric_limits<std::size_t>::max() / z) {
    throw std::overflow_error("Unity object voxel candidate count overflow");
  }
  return static_cast<std::size_t>(x * y * z);
}

void AddCell(
    std::unordered_map<VoxelKey, CellAccumulator, VoxelKeyHash>* cells,
    const VoxelKey& key,
    std::uint16_t label,
    double volume,
    std::uint64_t instance_id,
    UnitySemanticImportStats* stats,
    std::size_t max_voxels) {
  auto [found, inserted] = cells->try_emplace(key);
  if (inserted && cells->size() > max_voxels) {
    cells->erase(found);
    throw std::length_error("Unity semantic output exceeds configured voxel limit");
  }
  CellAccumulator& cell = found->second;
  if (cell.source_count != std::numeric_limits<std::uint32_t>::max()) {
    ++cell.source_count;
  }
  if (inserted) {
    cell.label = label;
    cell.priority_volume = volume;
    cell.priority_instance = instance_id;
    cell.chosen_label_count = 1U;
    return;
  }
  if (cell.label == label) {
    if (cell.chosen_label_count != std::numeric_limits<std::uint32_t>::max()) {
      ++cell.chosen_label_count;
    }
    if (volume < cell.priority_volume ||
        (volume == cell.priority_volume && instance_id < cell.priority_instance)) {
      cell.priority_volume = volume;
      cell.priority_instance = instance_id;
    }
    return;
  }
  ++stats->semantic_conflicts;
  if (volume < cell.priority_volume ||
      (volume == cell.priority_volume && instance_id < cell.priority_instance)) {
    cell.label = label;
    cell.priority_volume = volume;
    cell.priority_instance = instance_id;
    cell.chosen_label_count = 1U;
  }
}

void VoxelizeObject(
    const ObjectRow& object,
    std::uint16_t label,
    const UnitySemanticImportConfig& config,
    std::unordered_map<VoxelKey, CellAccumulator, VoxelKeyHash>* cells,
    UnitySemanticImportStats* stats) {
  const double voxel = config.voxel_size_m;
  const double half_cell = voxel * 0.5;
  const double hx = object.size_x * 0.5;
  const double hy = object.size_y * 0.5;
  const double hz = object.size_z * 0.5;
  const double cosine = std::cos(object.yaw);
  const double sine = std::sin(object.yaw);
  const double extent_x = std::fabs(cosine) * hx + std::fabs(sine) * hy + half_cell;
  const double extent_y = std::fabs(sine) * hx + std::fabs(cosine) * hy + half_cell;
  const double extent_z = hz + half_cell;
  const std::int32_t min_x = ToIndex(object.x - extent_x, voxel);
  const std::int32_t max_x = ToIndex(object.x + extent_x, voxel);
  const std::int32_t min_y = ToIndex(object.y - extent_y, voxel);
  const std::int32_t max_y = ToIndex(object.y + extent_y, voxel);
  const std::int32_t min_z = ToIndex(object.z - extent_z, voxel);
  const std::int32_t max_z = ToIndex(object.z + extent_z, voxel);
  const std::size_t checks = CheckedCellCount(
      static_cast<std::int64_t>(max_x) - min_x + 1,
      static_cast<std::int64_t>(max_y) - min_y + 1,
      static_cast<std::int64_t>(max_z) - min_z + 1);
  if (checks > config.max_voxel_checks -
                   std::min(config.max_voxel_checks, stats->candidate_voxel_checks)) {
    throw std::length_error("Unity semantic voxel checks exceed configured limit");
  }
  stats->candidate_voxel_checks += checks;

  const double shell = voxel * config.shell_thickness_voxels;
  const double volume = object.size_x * object.size_y * object.size_z;
  bool emitted = false;
  for (std::int64_t raw_x = min_x; raw_x <= static_cast<std::int64_t>(max_x); ++raw_x) {
    for (std::int64_t raw_y = min_y; raw_y <= static_cast<std::int64_t>(max_y); ++raw_y) {
      for (std::int64_t raw_z = min_z; raw_z <= static_cast<std::int64_t>(max_z); ++raw_z) {
        const auto ix = static_cast<std::int32_t>(raw_x);
        const auto iy = static_cast<std::int32_t>(raw_y);
        const auto iz = static_cast<std::int32_t>(raw_z);
        const double dx = Center(ix, voxel) - object.x;
        const double dy = Center(iy, voxel) - object.y;
        const double dz = Center(iz, voxel) - object.z;
        const double local_x = cosine * dx + sine * dy;
        const double local_y = -sine * dx + cosine * dy;
        const double ax = std::fabs(local_x);
        const double ay = std::fabs(local_y);
        const double az = std::fabs(dz);
        if (ax > hx + half_cell || ay > hy + half_cell || az > hz + half_cell) {
          continue;
        }
        const bool on_shell = ax >= std::max(0.0, hx - shell) ||
                              ay >= std::max(0.0, hy - shell) ||
                              az >= std::max(0.0, hz - shell);
        if (!on_shell) {
          continue;
        }
        AddCell(
            cells, {ix, iy, iz}, label, volume, object.instance_id, stats,
            config.max_voxels);
        emitted = true;
      }
    }
  }
  if (!emitted) {
    AddCell(
        cells,
        {ToIndex(object.x, voxel), ToIndex(object.y, voxel), ToIndex(object.z, voxel)},
        label, volume, object.instance_id, stats, config.max_voxels);
  }
}

layers::SemanticMapChunk BuildChunk(
    const std::unordered_map<VoxelKey, CellAccumulator, VoxelKeyHash>& cells,
    const SemanticTaxonomy& taxonomy,
    const UnitySemanticImportConfig& config) {
  std::vector<std::pair<VoxelKey, CellAccumulator>> ordered;
  ordered.reserve(cells.size());
  for (const auto& entry : cells) {
    ordered.push_back(entry);
  }
  std::sort(ordered.begin(), ordered.end(), [](const auto& lhs, const auto& rhs) {
    return lhs.first < rhs.first;
  });

  auto data = std::make_shared<layers::SemanticMapChunkSoA>();
#define LINGTU_RESERVE_FIELD(name) data->name.reserve(ordered.size())
  LINGTU_RESERVE_FIELD(index_x);
  LINGTU_RESERVE_FIELD(index_y);
  LINGTU_RESERVE_FIELD(index_z);
  LINGTU_RESERVE_FIELD(center_x_m);
  LINGTU_RESERVE_FIELD(center_y_m);
  LINGTU_RESERVE_FIELD(center_z_m);
  LINGTU_RESERVE_FIELD(occupancy_probability);
  LINGTU_RESERVE_FIELD(hit_count);
  LINGTU_RESERVE_FIELD(miss_count);
  LINGTU_RESERVE_FIELD(point_count);
  LINGTU_RESERVE_FIELD(mean_x_m);
  LINGTU_RESERVE_FIELD(mean_y_m);
  LINGTU_RESERVE_FIELD(mean_z_m);
  LINGTU_RESERVE_FIELD(covariance_xx);
  LINGTU_RESERVE_FIELD(covariance_xy);
  LINGTU_RESERVE_FIELD(covariance_xz);
  LINGTU_RESERVE_FIELD(covariance_yy);
  LINGTU_RESERVE_FIELD(covariance_yz);
  LINGTU_RESERVE_FIELD(covariance_zz);
  LINGTU_RESERVE_FIELD(dominant_label);
  LINGTU_RESERVE_FIELD(semantic_confidence);
#undef LINGTU_RESERVE_FIELD

  for (const auto& [key, cell] : ordered) {
    const float center_x = static_cast<float>(Center(key.x, config.voxel_size_m));
    const float center_y = static_cast<float>(Center(key.y, config.voxel_size_m));
    const float center_z = static_cast<float>(Center(key.z, config.voxel_size_m));
    data->index_x.push_back(key.x);
    data->index_y.push_back(key.y);
    data->index_z.push_back(key.z);
    data->center_x_m.push_back(center_x);
    data->center_y_m.push_back(center_y);
    data->center_z_m.push_back(center_z);
    data->occupancy_probability.push_back(config.occupied_probability);
    data->hit_count.push_back(std::max<std::uint32_t>(1U, cell.source_count));
    data->miss_count.push_back(0U);
    data->point_count.push_back(std::max<std::uint32_t>(1U, cell.source_count));
    data->mean_x_m.push_back(center_x);
    data->mean_y_m.push_back(center_y);
    data->mean_z_m.push_back(center_z);
    data->covariance_xx.push_back(0.0F);
    data->covariance_xy.push_back(0.0F);
    data->covariance_xz.push_back(0.0F);
    data->covariance_yy.push_back(0.0F);
    data->covariance_yz.push_back(0.0F);
    data->covariance_zz.push_back(0.0F);
    data->dominant_label.push_back(cell.label);
    data->semantic_confidence.push_back(
        cell.source_count == 0U
            ? 0.0F
            : static_cast<float>(cell.chosen_label_count) /
                  static_cast<float>(cell.source_count));
  }

  layers::SemanticMapChunk chunk;
  chunk.generation = config.generation;
  chunk.offset = 0U;
  chunk.total_voxels = data->index_x.size();
  chunk.complete = true;
  chunk.voxel_size_m = config.voxel_size_m;
  chunk.frame_id = config.frame_id;
  chunk.taxonomy = taxonomy.name();
  chunk.taxonomy_version = taxonomy.version();
  chunk.data = std::move(data);
  return chunk;
}

void ValidateConfig(const UnitySemanticImportConfig& config) {
  if (config.taxonomy_path.empty()) {
    throw std::invalid_argument("Unity semantic import requires taxonomy_path");
  }
  if (config.frame_id.empty() || !std::isfinite(config.voxel_size_m) ||
      config.voxel_size_m <= 0.0F || !std::isfinite(config.occupied_probability) ||
      config.occupied_probability <= 0.5F || config.occupied_probability >= 1.0F ||
      !std::isfinite(config.shell_thickness_voxels) ||
      config.shell_thickness_voxels <= 0.0F || config.generation == 0U ||
      config.max_objects == 0U || config.max_voxels == 0U ||
      config.max_voxel_checks == 0U) {
    throw std::invalid_argument("Unity semantic import config is invalid");
  }
}

}  // namespace

UnitySemanticImportResult BuildUnitySemanticMap(
    const std::filesystem::path& scene_dir,
    const UnitySemanticImportConfig& config) {
  ValidateConfig(config);
  if (!std::filesystem::is_directory(scene_dir)) {
    throw std::invalid_argument("Unity scene directory does not exist: " + scene_dir.string());
  }
  const SemanticTaxonomy taxonomy = SemanticTaxonomy::LoadJson(config.taxonomy_path);
  const auto categories = LoadCategories(scene_dir / config.categories_path);
  const auto objects = LoadObjects(scene_dir / config.objects_path, config.max_objects);

  std::unordered_multimap<std::string, const CategoryRow*> category_index;
  category_index.reserve(categories.size() * 2U);
  for (const auto& category : categories) {
    for (const auto* key : {&category.cleaned, &category.name}) {
      const std::string normalized = SemanticTaxonomy::NormalizeLabel(*key);
      if (!normalized.empty()) {
        category_index.emplace(normalized, &category);
      }
    }
  }

  std::set<std::uint16_t> dynamic_ids;
  if (config.exclude_dynamic_classes) {
    for (const auto* label : {"person", "animal", "vehicle"}) {
      if (const auto id = taxonomy.Resolve(label)) {
        dynamic_ids.insert(*id);
      }
    }
  }

  UnitySemanticImportResult result;
  result.stats.category_rows = categories.size();
  result.stats.object_rows = objects.size();
  std::set<std::string> unmapped;
  std::unordered_map<VoxelKey, CellAccumulator, VoxelKeyHash> cells;
  cells.reserve(std::min(config.max_voxels, objects.size() * 64U));
  for (const auto& object : objects) {
    const auto label = ResolveLabel(object.label, category_index, taxonomy);
    if (!label || (*label == 0U && !config.include_unknown_geometry)) {
      ++result.stats.skipped_unmapped_objects;
      unmapped.insert(SemanticTaxonomy::NormalizeLabel(object.label));
      continue;
    }
    if (dynamic_ids.find(*label) != dynamic_ids.end()) {
      ++result.stats.skipped_dynamic_objects;
      continue;
    }
    VoxelizeObject(object, *label, config, &cells, &result.stats);
    ++result.stats.accepted_objects;
  }
  if (cells.empty()) {
    throw std::runtime_error("Unity semantic import produced no mapped voxels");
  }
  result.stats.unmapped_labels.assign(unmapped.begin(), unmapped.end());
  result.stats.output_voxels = cells.size();
  result.chunk = BuildChunk(cells, taxonomy, config);
  return result;
}

UnitySemanticImportStats ImportUnitySemanticMap(
    const std::filesystem::path& scene_dir,
    const std::filesystem::path& output_path,
    const UnitySemanticImportConfig& config) {
  auto result = BuildUnitySemanticMap(scene_dir, config);
  WriteSemanticMapBinaryAtomic(output_path, result.chunk);
  std::string validation_error;
  if (!ValidateSemanticMapBinary(output_path, &validation_error)) {
    throw std::runtime_error(
        "Unity semantic artifact failed post-write validation: " + validation_error);
  }
  return result.stats;
}

}  // namespace lingtu::maps::sources

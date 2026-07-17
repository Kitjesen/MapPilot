#include "lingtu/maps/semantic_map_persistence.hpp"

#include <array>
#include <chrono>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#if defined(_WIN32)
#  define NOMINMAX
#  include <Windows.h>
#endif

namespace lingtu::maps {
namespace {

constexpr std::array<std::uint8_t, 8U> kMagic{'L', 'T', 'S', 'E', 'M', 'A', 'P', '\0'};
constexpr std::uint32_t kEndianMarker = 0x01020304U;
constexpr std::uint32_t kHeaderSize = 68U;
constexpr std::uint64_t kFnvOffset = 14695981039346656037ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

struct Header {
  std::array<std::uint8_t, 8U> magic{};
  std::uint32_t schema_version{0U};
  std::uint32_t endian_marker{0U};
  std::uint32_t header_size{0U};
  std::uint32_t frame_id_size{0U};
  std::uint32_t taxonomy_size{0U};
  std::uint32_t taxonomy_version{0U};
  float voxel_size_m{0.0F};
  std::uint64_t generation{0U};
  std::uint64_t voxel_count{0U};
  std::uint64_t body_size{0U};
  std::uint64_t checksum{0U};
};

template <typename T>
void AppendPod(std::vector<std::uint8_t>& out, const T& value) {
  const auto* bytes = reinterpret_cast<const std::uint8_t*>(&value);
  out.insert(out.end(), bytes, bytes + sizeof(T));
}

template <typename T>
void AppendVector(std::vector<std::uint8_t>& out, const std::vector<T>& values) {
  if (values.empty()) {
    return;
  }
  const auto* bytes = reinterpret_cast<const std::uint8_t*>(values.data());
  out.insert(out.end(), bytes, bytes + values.size() * sizeof(T));
}

void AppendString(std::vector<std::uint8_t>& out, const std::string& value) {
  out.insert(out.end(), value.begin(), value.end());
}

std::uint64_t Fnv1a(const std::vector<std::uint8_t>& bytes) {
  std::uint64_t hash = kFnvOffset;
  for (const auto byte : bytes) {
    hash ^= byte;
    hash *= kFnvPrime;
  }
  return hash;
}

std::uint64_t CheckedAdd(std::uint64_t a, std::uint64_t b) {
  if (a > std::numeric_limits<std::uint64_t>::max() - b) {
    throw std::overflow_error("semantic map binary size overflow");
  }
  return a + b;
}

std::uint64_t PayloadBytesFor(std::uint64_t count) {
  constexpr std::uint64_t per_voxel =
      3ULL * sizeof(std::int32_t) +
      14ULL * sizeof(float) +
      3ULL * sizeof(std::uint32_t) +
      sizeof(std::uint16_t);
  if (count > std::numeric_limits<std::uint64_t>::max() / per_voxel) {
    throw std::overflow_error("semantic map binary voxel payload overflow");
  }
  return count * per_voxel;
}

void ValidateChunk(const layers::SemanticMapChunk& chunk) {
  if (chunk.data == nullptr) {
    throw std::invalid_argument("semantic map chunk data is required");
  }
  if (chunk.offset != 0U || !chunk.complete || chunk.total_voxels != chunk.Size()) {
    throw std::invalid_argument("semantic map binary requires a complete chunk at offset 0");
  }
  if (!(chunk.voxel_size_m > 0.0F)) {
    throw std::invalid_argument("semantic map voxel_size_m must be positive");
  }
  if (chunk.generation == 0U || chunk.frame_id.empty()) {
    throw std::invalid_argument("semantic map requires a positive generation and frame_id");
  }
  if (chunk.taxonomy.empty() != (chunk.taxonomy_version == 0U)) {
    throw std::invalid_argument("semantic map taxonomy name/version must be present together");
  }
  if (chunk.frame_id.size() > std::numeric_limits<std::uint32_t>::max() ||
      chunk.taxonomy.size() > std::numeric_limits<std::uint32_t>::max()) {
    throw std::length_error("semantic map metadata strings are too large");
  }
#define LINGTU_CHECK_SEMANTIC_FIELD(name) \
  if (chunk.data->name.size() != chunk.Size()) { \
    throw std::invalid_argument("semantic map SoA field has inconsistent length: " #name); \
  }
  LINGTU_CHECK_SEMANTIC_FIELD(index_x)
  LINGTU_CHECK_SEMANTIC_FIELD(index_y)
  LINGTU_CHECK_SEMANTIC_FIELD(index_z)
  LINGTU_CHECK_SEMANTIC_FIELD(center_x_m)
  LINGTU_CHECK_SEMANTIC_FIELD(center_y_m)
  LINGTU_CHECK_SEMANTIC_FIELD(center_z_m)
  LINGTU_CHECK_SEMANTIC_FIELD(occupancy_probability)
  LINGTU_CHECK_SEMANTIC_FIELD(hit_count)
  LINGTU_CHECK_SEMANTIC_FIELD(miss_count)
  LINGTU_CHECK_SEMANTIC_FIELD(point_count)
  LINGTU_CHECK_SEMANTIC_FIELD(mean_x_m)
  LINGTU_CHECK_SEMANTIC_FIELD(mean_y_m)
  LINGTU_CHECK_SEMANTIC_FIELD(mean_z_m)
  LINGTU_CHECK_SEMANTIC_FIELD(covariance_xx)
  LINGTU_CHECK_SEMANTIC_FIELD(covariance_xy)
  LINGTU_CHECK_SEMANTIC_FIELD(covariance_xz)
  LINGTU_CHECK_SEMANTIC_FIELD(covariance_yy)
  LINGTU_CHECK_SEMANTIC_FIELD(covariance_yz)
  LINGTU_CHECK_SEMANTIC_FIELD(covariance_zz)
  LINGTU_CHECK_SEMANTIC_FIELD(dominant_label)
  LINGTU_CHECK_SEMANTIC_FIELD(semantic_confidence)
#undef LINGTU_CHECK_SEMANTIC_FIELD
}

std::vector<std::uint8_t> BuildBody(const layers::SemanticMapChunk& chunk) {
  ValidateChunk(chunk);
  std::vector<std::uint8_t> body;
  body.reserve(static_cast<size_t>(
      CheckedAdd(CheckedAdd(chunk.frame_id.size(), chunk.taxonomy.size()),
                 PayloadBytesFor(chunk.Size()))));
  AppendString(body, chunk.frame_id);
  AppendString(body, chunk.taxonomy);
#define LINGTU_APPEND_SEMANTIC_FIELD(name) AppendVector(body, chunk.data->name);
  LINGTU_APPEND_SEMANTIC_FIELD(index_x)
  LINGTU_APPEND_SEMANTIC_FIELD(index_y)
  LINGTU_APPEND_SEMANTIC_FIELD(index_z)
  LINGTU_APPEND_SEMANTIC_FIELD(center_x_m)
  LINGTU_APPEND_SEMANTIC_FIELD(center_y_m)
  LINGTU_APPEND_SEMANTIC_FIELD(center_z_m)
  LINGTU_APPEND_SEMANTIC_FIELD(occupancy_probability)
  LINGTU_APPEND_SEMANTIC_FIELD(hit_count)
  LINGTU_APPEND_SEMANTIC_FIELD(miss_count)
  LINGTU_APPEND_SEMANTIC_FIELD(point_count)
  LINGTU_APPEND_SEMANTIC_FIELD(mean_x_m)
  LINGTU_APPEND_SEMANTIC_FIELD(mean_y_m)
  LINGTU_APPEND_SEMANTIC_FIELD(mean_z_m)
  LINGTU_APPEND_SEMANTIC_FIELD(covariance_xx)
  LINGTU_APPEND_SEMANTIC_FIELD(covariance_xy)
  LINGTU_APPEND_SEMANTIC_FIELD(covariance_xz)
  LINGTU_APPEND_SEMANTIC_FIELD(covariance_yy)
  LINGTU_APPEND_SEMANTIC_FIELD(covariance_yz)
  LINGTU_APPEND_SEMANTIC_FIELD(covariance_zz)
  LINGTU_APPEND_SEMANTIC_FIELD(dominant_label)
  LINGTU_APPEND_SEMANTIC_FIELD(semantic_confidence)
#undef LINGTU_APPEND_SEMANTIC_FIELD
  return body;
}

void WriteHeader(std::ofstream& file, const Header& header) {
  file.write(reinterpret_cast<const char*>(header.magic.data()), header.magic.size());
  file.write(reinterpret_cast<const char*>(&header.schema_version), sizeof(header.schema_version));
  file.write(reinterpret_cast<const char*>(&header.endian_marker), sizeof(header.endian_marker));
  file.write(reinterpret_cast<const char*>(&header.header_size), sizeof(header.header_size));
  file.write(reinterpret_cast<const char*>(&header.frame_id_size), sizeof(header.frame_id_size));
  file.write(reinterpret_cast<const char*>(&header.taxonomy_size), sizeof(header.taxonomy_size));
  file.write(reinterpret_cast<const char*>(&header.taxonomy_version),
             sizeof(header.taxonomy_version));
  file.write(reinterpret_cast<const char*>(&header.voxel_size_m), sizeof(header.voxel_size_m));
  file.write(reinterpret_cast<const char*>(&header.generation), sizeof(header.generation));
  file.write(reinterpret_cast<const char*>(&header.voxel_count), sizeof(header.voxel_count));
  file.write(reinterpret_cast<const char*>(&header.body_size), sizeof(header.body_size));
  file.write(reinterpret_cast<const char*>(&header.checksum), sizeof(header.checksum));
}

Header ReadHeader(std::ifstream& file) {
  Header header;
  file.read(reinterpret_cast<char*>(header.magic.data()), header.magic.size());
  file.read(reinterpret_cast<char*>(&header.schema_version), sizeof(header.schema_version));
  file.read(reinterpret_cast<char*>(&header.endian_marker), sizeof(header.endian_marker));
  file.read(reinterpret_cast<char*>(&header.header_size), sizeof(header.header_size));
  file.read(reinterpret_cast<char*>(&header.frame_id_size), sizeof(header.frame_id_size));
  file.read(reinterpret_cast<char*>(&header.taxonomy_size), sizeof(header.taxonomy_size));
  file.read(reinterpret_cast<char*>(&header.taxonomy_version), sizeof(header.taxonomy_version));
  file.read(reinterpret_cast<char*>(&header.voxel_size_m), sizeof(header.voxel_size_m));
  file.read(reinterpret_cast<char*>(&header.generation), sizeof(header.generation));
  file.read(reinterpret_cast<char*>(&header.voxel_count), sizeof(header.voxel_count));
  file.read(reinterpret_cast<char*>(&header.body_size), sizeof(header.body_size));
  file.read(reinterpret_cast<char*>(&header.checksum), sizeof(header.checksum));
  if (!file) {
    throw std::runtime_error("semantic map binary header is truncated");
  }
  return header;
}

template <typename T>
void ReadVector(const std::vector<std::uint8_t>& body, size_t* cursor, size_t count,
                std::vector<T>* out) {
  const size_t bytes = count * sizeof(T);
  if (count != 0U && bytes / sizeof(T) != count) {
    throw std::runtime_error("semantic map vector size overflow");
  }
  if (*cursor > body.size() || bytes > body.size() - *cursor) {
    throw std::runtime_error("semantic map binary payload is truncated");
  }
  out->resize(count);
  if (bytes > 0U) {
    std::memcpy(out->data(), body.data() + *cursor, bytes);
  }
  *cursor += bytes;
}

std::filesystem::path TempPathFor(const std::filesystem::path& path) {
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  return path.parent_path() /
      (path.filename().string() + ".tmp." + std::to_string(stamp));
}

void AtomicReplace(const std::filesystem::path& temp, const std::filesystem::path& target) {
#if defined(_WIN32)
  if (!MoveFileExW(temp.c_str(), target.c_str(),
                   MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) {
    throw std::filesystem::filesystem_error(
        "failed to atomically replace semantic map", temp, target,
        std::error_code(static_cast<int>(GetLastError()), std::system_category()));
  }
#else
  std::filesystem::rename(temp, target);
#endif
}

}  // namespace

layers::SemanticMapChunk ReadSemanticMapBinary(
    const std::filesystem::path& path,
    const SemanticMapReadOptions& options) {
  const auto file_size = std::filesystem::file_size(path);
  if (file_size < kHeaderSize) {
    throw std::runtime_error("semantic map binary is smaller than its header");
  }
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    throw std::runtime_error("failed to open semantic map binary: " + path.string());
  }
  const Header header = ReadHeader(file);
  if (header.magic != kMagic) {
    throw std::runtime_error("semantic map binary magic mismatch");
  }
  if (header.schema_version != kSemanticMapBinarySchemaVersion) {
    throw std::runtime_error("unsupported semantic map binary schema version");
  }
  if (header.endian_marker != kEndianMarker) {
    throw std::runtime_error("unsupported semantic map binary endian marker");
  }
  if (header.header_size != kHeaderSize) {
    throw std::runtime_error("unsupported semantic map binary header size");
  }
  if (!(header.voxel_size_m > 0.0F)) {
    throw std::runtime_error("semantic map binary voxel_size_m is invalid");
  }
  if (header.voxel_count > options.max_voxels) {
    throw std::runtime_error("semantic map binary voxel count exceeds limit");
  }
  if (header.body_size > options.max_body_bytes) {
    throw std::runtime_error("semantic map binary body exceeds limit");
  }
  const std::uint64_t expected_body = CheckedAdd(
      CheckedAdd(header.frame_id_size, header.taxonomy_size),
      PayloadBytesFor(header.voxel_count));
  if (header.body_size != expected_body) {
    throw std::runtime_error("semantic map binary body size does not match header fields");
  }
  if (file_size != CheckedAdd(header.header_size, header.body_size)) {
    throw std::runtime_error("semantic map binary file size does not match header");
  }

  std::vector<std::uint8_t> body(static_cast<size_t>(header.body_size));
  if (!body.empty()) {
    file.read(reinterpret_cast<char*>(body.data()), static_cast<std::streamsize>(body.size()));
  }
  if (!file && !body.empty()) {
    throw std::runtime_error("semantic map binary payload is truncated");
  }
  if (Fnv1a(body) != header.checksum) {
    throw std::runtime_error("semantic map binary checksum mismatch");
  }

  size_t cursor = 0U;
  layers::SemanticMapChunk chunk;
  chunk.generation = header.generation;
  chunk.offset = 0U;
  chunk.total_voxels = static_cast<size_t>(header.voxel_count);
  chunk.complete = true;
  chunk.voxel_size_m = header.voxel_size_m;
  chunk.frame_id = std::string(
      reinterpret_cast<const char*>(body.data() + cursor), header.frame_id_size);
  cursor += header.frame_id_size;
  chunk.taxonomy = std::string(
      reinterpret_cast<const char*>(body.data() + cursor), header.taxonomy_size);
  cursor += header.taxonomy_size;
  chunk.taxonomy_version = header.taxonomy_version;
  auto data = std::make_shared<layers::SemanticMapChunkSoA>();
  const size_t count = static_cast<size_t>(header.voxel_count);
#define LINGTU_READ_SEMANTIC_FIELD(name) ReadVector(body, &cursor, count, &data->name);
  LINGTU_READ_SEMANTIC_FIELD(index_x)
  LINGTU_READ_SEMANTIC_FIELD(index_y)
  LINGTU_READ_SEMANTIC_FIELD(index_z)
  LINGTU_READ_SEMANTIC_FIELD(center_x_m)
  LINGTU_READ_SEMANTIC_FIELD(center_y_m)
  LINGTU_READ_SEMANTIC_FIELD(center_z_m)
  LINGTU_READ_SEMANTIC_FIELD(occupancy_probability)
  LINGTU_READ_SEMANTIC_FIELD(hit_count)
  LINGTU_READ_SEMANTIC_FIELD(miss_count)
  LINGTU_READ_SEMANTIC_FIELD(point_count)
  LINGTU_READ_SEMANTIC_FIELD(mean_x_m)
  LINGTU_READ_SEMANTIC_FIELD(mean_y_m)
  LINGTU_READ_SEMANTIC_FIELD(mean_z_m)
  LINGTU_READ_SEMANTIC_FIELD(covariance_xx)
  LINGTU_READ_SEMANTIC_FIELD(covariance_xy)
  LINGTU_READ_SEMANTIC_FIELD(covariance_xz)
  LINGTU_READ_SEMANTIC_FIELD(covariance_yy)
  LINGTU_READ_SEMANTIC_FIELD(covariance_yz)
  LINGTU_READ_SEMANTIC_FIELD(covariance_zz)
  LINGTU_READ_SEMANTIC_FIELD(dominant_label)
  LINGTU_READ_SEMANTIC_FIELD(semantic_confidence)
#undef LINGTU_READ_SEMANTIC_FIELD
  if (cursor != body.size()) {
    throw std::runtime_error("semantic map binary has trailing payload bytes");
  }
  chunk.data = data;
  ValidateChunk(chunk);
  return chunk;
}

void WriteSemanticMapBinaryAtomic(
    const std::filesystem::path& path,
    const layers::SemanticMapChunk& chunk) {
  const auto body = BuildBody(chunk);
  Header header;
  header.magic = kMagic;
  header.schema_version = kSemanticMapBinarySchemaVersion;
  header.endian_marker = kEndianMarker;
  header.header_size = kHeaderSize;
  header.frame_id_size = static_cast<std::uint32_t>(chunk.frame_id.size());
  header.taxonomy_size = static_cast<std::uint32_t>(chunk.taxonomy.size());
  header.taxonomy_version = chunk.taxonomy_version;
  header.voxel_size_m = chunk.voxel_size_m;
  header.generation = chunk.generation;
  header.voxel_count = chunk.Size();
  header.body_size = body.size();
  header.checksum = Fnv1a(body);

  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }
  const auto temp = TempPathFor(path);
  try {
    {
      std::ofstream file(temp, std::ios::binary | std::ios::trunc);
      if (!file) {
        throw std::runtime_error("failed to create semantic map temp file: " + temp.string());
      }
      WriteHeader(file, header);
      if (!body.empty()) {
        file.write(reinterpret_cast<const char*>(body.data()),
                   static_cast<std::streamsize>(body.size()));
      }
      file.flush();
      if (!file) {
        throw std::runtime_error("failed to write semantic map temp file: " + temp.string());
      }
    }
    AtomicReplace(temp, path);
  } catch (...) {
    std::error_code cleanup_error;
    std::filesystem::remove(temp, cleanup_error);
    throw;
  }
}

bool ValidateSemanticMapBinary(
    const std::filesystem::path& path,
    std::string* error,
    const SemanticMapReadOptions& options) {
  try {
    static_cast<void>(ReadSemanticMapBinary(path, options));
    if (error != nullptr) {
      error->clear();
    }
    return true;
  } catch (const std::exception& exc) {
    if (error != nullptr) {
      *error = exc.what();
    }
    return false;
  }
}

}  // namespace lingtu::maps

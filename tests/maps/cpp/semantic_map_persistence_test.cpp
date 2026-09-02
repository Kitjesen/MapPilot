#include "lingtu/maps/semantic_map_persistence.hpp"

#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>

namespace {

using lingtu::maps::ReadSemanticMapBinary;
using lingtu::maps::ValidateSemanticMapBinary;
using lingtu::maps::WriteSemanticMapBinaryAtomic;
using lingtu::maps::layers::SemanticMapChunk;
using lingtu::maps::layers::SemanticMapChunkSoA;

std::filesystem::path TempRoot() {
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  auto root = std::filesystem::temp_directory_path() /
      ("lingtu_semantic_map_persistence_test_" + std::to_string(stamp));
  std::filesystem::remove_all(root);
  std::filesystem::create_directories(root);
  return root;
}

SemanticMapChunk MakeChunk() {
  auto data = std::make_shared<SemanticMapChunkSoA>();
  data->index_x = {0, 1};
  data->index_y = {2, 3};
  data->index_z = {4, 5};
  data->center_x_m = {0.5F, 1.5F};
  data->center_y_m = {2.5F, 3.5F};
  data->center_z_m = {4.5F, 5.5F};
  data->occupancy_probability = {0.7F, 0.9F};
  data->hit_count = {7U, 9U};
  data->miss_count = {1U, 2U};
  data->point_count = {3U, 4U};
  data->mean_x_m = {0.1F, 1.1F};
  data->mean_y_m = {0.2F, 1.2F};
  data->mean_z_m = {0.3F, 1.3F};
  data->covariance_xx = {0.01F, 0.02F};
  data->covariance_xy = {0.03F, 0.04F};
  data->covariance_xz = {0.05F, 0.06F};
  data->covariance_yy = {0.07F, 0.08F};
  data->covariance_yz = {0.09F, 0.10F};
  data->covariance_zz = {0.11F, 0.12F};
  data->dominant_label = {11U, 12U};
  data->semantic_confidence = {0.8F, 1.0F};

  SemanticMapChunk chunk;
  chunk.generation = 42U;
  chunk.offset = 0U;
  chunk.total_voxels = 2U;
  chunk.complete = true;
  chunk.voxel_size_m = 0.25F;
  chunk.frame_id = "map";
  chunk.taxonomy = "lingtu.semantic";
  chunk.taxonomy_version = 3U;
  chunk.data = data;
  return chunk;
}

void TestRoundtripIsDeterministic(const std::filesystem::path& root) {
  const auto first = root / "semantic_map.bin";
  const auto second = root / "semantic_map_copy.bin";
  const auto chunk = MakeChunk();
  WriteSemanticMapBinaryAtomic(first, chunk);
  WriteSemanticMapBinaryAtomic(second, chunk);
  assert(std::filesystem::file_size(first) == std::filesystem::file_size(second));

  std::ifstream a(first, std::ios::binary);
  std::ifstream b(second, std::ios::binary);
  const std::string bytes_a((std::istreambuf_iterator<char>(a)), {});
  const std::string bytes_b((std::istreambuf_iterator<char>(b)), {});
  assert(bytes_a == bytes_b);

  const auto loaded = ReadSemanticMapBinary(first);
  assert(loaded.generation == 42U);
  assert(loaded.frame_id == "map");
  assert(loaded.taxonomy == "lingtu.semantic");
  assert(loaded.taxonomy_version == 3U);
  assert(loaded.voxel_size_m == 0.25F);
  assert(loaded.Size() == 2U);
  assert(loaded.data->index_x[1] == 1);
  assert(loaded.data->dominant_label[0] == 11U);
  assert(loaded.data->semantic_confidence[1] == 1.0F);
}

void TestCorruptAndTruncatedReject(const std::filesystem::path& root) {
  const auto source = root / "semantic_map.bin";
  const auto corrupt = root / "semantic_map_corrupt.bin";
  const auto truncated = root / "semantic_map_truncated.bin";
  std::filesystem::copy_file(source, corrupt, std::filesystem::copy_options::overwrite_existing);
  {
    std::fstream file(corrupt, std::ios::in | std::ios::out | std::ios::binary);
    file.seekp(80);
    char byte = '\x7f';
    file.write(&byte, 1);
  }
  std::string error;
  assert(!ValidateSemanticMapBinary(corrupt, &error));
  assert(error.find("checksum") != std::string::npos);

  {
    std::ifstream in(source, std::ios::binary);
    std::ofstream out(truncated, std::ios::binary);
    std::string bytes((std::istreambuf_iterator<char>(in)), {});
    out.write(bytes.data(), static_cast<std::streamsize>(bytes.size() - 1U));
  }
  assert(!ValidateSemanticMapBinary(truncated, &error));
  assert(error.find("file size") != std::string::npos);
}

void TestTransactionalVisibility(const std::filesystem::path& root) {
  const auto path = root / "nested" / "semantic_map.bin";
  assert(!std::filesystem::exists(path));
  WriteSemanticMapBinaryAtomic(path, MakeChunk());
  assert(std::filesystem::is_regular_file(path));
  assert(ValidateSemanticMapBinary(path));
  for (const auto& entry : std::filesystem::directory_iterator(path.parent_path())) {
    assert(entry.path().filename().string().find(".tmp.") == std::string::npos);
  }

  std::ifstream before_file(path, std::ios::binary);
  const std::string before((std::istreambuf_iterator<char>(before_file)), {});
  auto invalid = MakeChunk();
  invalid.generation = 0U;
  bool rejected = false;
  try {
    WriteSemanticMapBinaryAtomic(path, invalid);
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  assert(rejected);
  std::ifstream after_file(path, std::ios::binary);
  const std::string after((std::istreambuf_iterator<char>(after_file)), {});
  assert(after == before);
}

}  // namespace

int main() {
  const auto root = TempRoot();
  TestRoundtripIsDeterministic(root);
  TestCorruptAndTruncatedReject(root);
  TestTransactionalVisibility(root);
  std::filesystem::remove_all(root);
  return 0;
}

#include "semantic_map_client.hpp"

#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "lingtu/maps/c_api/semantic_occupancy.h"

#if defined(_WIN32)
#define NOMINMAX
#include <Windows.h>
#else
#include <dlfcn.h>
#endif

namespace lingtu::slam {
namespace {

template <typename Fn>
Fn LoadSymbol(void *library, const char *name) {
#if defined(_WIN32)
  return reinterpret_cast<Fn>(GetProcAddress(static_cast<HMODULE>(library), name));
#else
  return reinterpret_cast<Fn>(dlsym(library, name));
#endif
}

void *OpenLibrary() {
  std::vector<std::string> candidates;
  if (const char *configured = std::getenv("LINGTU_MAPS_LIB");
      configured != nullptr && configured[0] != '\0') {
    candidates.emplace_back(configured);
  }
#if defined(_WIN32)
  candidates.emplace_back("lingtu_maps.dll");
#else
  candidates.emplace_back("liblingtu_maps.so");
  candidates.emplace_back("lingtu_maps.so");
#endif
  for (const auto &candidate : candidates) {
#if defined(_WIN32)
    if (HMODULE library = LoadLibraryA(candidate.c_str()); library != nullptr) {
      return library;
    }
#else
    if (void *library = dlopen(candidate.c_str(), RTLD_NOW | RTLD_LOCAL); library != nullptr) {
      return library;
    }
#endif
  }
  return nullptr;
}

void CloseLibrary(void *library) {
  if (library == nullptr) {
    return;
  }
#if defined(_WIN32)
  FreeLibrary(static_cast<HMODULE>(library));
#else
  dlclose(library);
#endif
}

}  // namespace

struct SemanticMapClient::Impl {
  using AbiVersionFn = decltype(&lingtu_maps_semantic_abi_version);
  using OpenFileFn = decltype(&lingtu_maps_semantic_open_file);
  using DestroyFn = decltype(&lingtu_maps_semantic_destroy);
  using MetadataFn = decltype(&lingtu_maps_semantic_metadata);
  using MetadataStringsFn = decltype(&lingtu_maps_semantic_metadata_strings);
  using SnapshotCountFn = decltype(&lingtu_maps_semantic_snapshot_count);
  using SnapshotFillFn = decltype(&lingtu_maps_semantic_snapshot_fill);

  void *library{nullptr};
  AbiVersionFn abi_version{nullptr};
  OpenFileFn open_file{nullptr};
  DestroyFn destroy{nullptr};
  MetadataFn metadata{nullptr};
  MetadataStringsFn metadata_strings{nullptr};
  SnapshotCountFn snapshot_count{nullptr};
  SnapshotFillFn snapshot_fill{nullptr};

  bool ready() const {
    return library != nullptr && abi_version != nullptr && open_file != nullptr &&
           destroy != nullptr && metadata != nullptr && metadata_strings != nullptr &&
           snapshot_count != nullptr && snapshot_fill != nullptr && abi_version() == 1U;
  }
};

SemanticMapClient::SemanticMapClient() : impl_(new Impl()) {
  impl_->library = OpenLibrary();
  if (impl_->library == nullptr) {
    return;
  }
  impl_->abi_version =
      LoadSymbol<Impl::AbiVersionFn>(impl_->library, "lingtu_maps_semantic_abi_version");
  impl_->open_file = LoadSymbol<Impl::OpenFileFn>(impl_->library, "lingtu_maps_semantic_open_file");
  impl_->destroy = LoadSymbol<Impl::DestroyFn>(impl_->library, "lingtu_maps_semantic_destroy");
  impl_->metadata = LoadSymbol<Impl::MetadataFn>(impl_->library, "lingtu_maps_semantic_metadata");
  impl_->metadata_strings =
      LoadSymbol<Impl::MetadataStringsFn>(impl_->library, "lingtu_maps_semantic_metadata_strings");
  impl_->snapshot_count =
      LoadSymbol<Impl::SnapshotCountFn>(impl_->library, "lingtu_maps_semantic_snapshot_count");
  impl_->snapshot_fill =
      LoadSymbol<Impl::SnapshotFillFn>(impl_->library, "lingtu_maps_semantic_snapshot_fill");
  if (!impl_->ready()) {
    CloseLibrary(impl_->library);
    impl_->library = nullptr;
  }
}

SemanticMapClient::~SemanticMapClient() {
  if (impl_ != nullptr) {
    CloseLibrary(impl_->library);
    delete impl_;
  }
}

bool SemanticMapClient::available() const {
  return impl_ != nullptr && impl_->ready();
}

bool SemanticMapClient::load(const std::string &path, SemanticMapSnapshot *snapshot,
                             std::string *error) const {
  if (snapshot == nullptr) {
    if (error != nullptr) {
      *error = "semantic_map_snapshot_output_required";
    }
    return false;
  }
  if (!available()) {
    if (error != nullptr) {
      *error = "lingtu_maps_semantic_abi_unavailable";
    }
    return false;
  }
  LingtuMapsSemanticHandle *raw = impl_->open_file(path.c_str());
  if (raw == nullptr) {
    if (error != nullptr) {
      *error = "semantic_map_open_failed";
    }
    return false;
  }
  const auto destroy = [this](LingtuMapsSemanticHandle *handle) { impl_->destroy(handle); };
  std::unique_ptr<LingtuMapsSemanticHandle, decltype(destroy)> handle(raw, destroy);

  LingtuMapsSemanticMetadata metadata{};
  if (impl_->metadata(handle.get(), &metadata) != LINGTU_MAPS_SEMANTIC_OK) {
    if (error != nullptr) {
      *error = "semantic_map_metadata_failed";
    }
    return false;
  }
  std::vector<char> frame_id(static_cast<std::size_t>(metadata.frame_id_bytes), '\0');
  std::vector<char> taxonomy(static_cast<std::size_t>(metadata.taxonomy_bytes), '\0');
  if (frame_id.empty() || taxonomy.empty() ||
      impl_->metadata_strings(handle.get(), metadata.generation, frame_id.data(), frame_id.size(),
                              taxonomy.data(), taxonomy.size()) != LINGTU_MAPS_SEMANTIC_OK) {
    if (error != nullptr) {
      *error = "semantic_map_metadata_generation_changed";
    }
    return false;
  }

  std::uint64_t query_generation = 0U;
  std::uint64_t count = 0U;
  if (impl_->snapshot_count(handle.get(), 0.5F, &query_generation, &count) !=
          LINGTU_MAPS_SEMANTIC_OK ||
      query_generation != metadata.generation || count == 0U) {
    if (error != nullptr) {
      *error = count == 0U ? "semantic_map_has_no_occupied_voxels"
                           : "semantic_map_snapshot_count_failed";
    }
    return false;
  }
  if (count > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max())) {
    if (error != nullptr) {
      *error = "semantic_map_voxel_count_overflow";
    }
    return false;
  }
  const auto point_count_size = static_cast<std::size_t>(count);
  std::vector<float> center_x(point_count_size);
  std::vector<float> center_y(point_count_size);
  std::vector<float> center_z(point_count_size);
  std::vector<float> mean_x(point_count_size);
  std::vector<float> mean_y(point_count_size);
  std::vector<float> mean_z(point_count_size);
  std::vector<std::uint32_t> point_count(point_count_size);
  std::vector<std::uint16_t> labels(point_count_size);
  std::vector<float> confidence(point_count_size);
  LingtuMapsSemanticChunkBuffers buffers{};
  buffers.center_x_m = center_x.data();
  buffers.center_y_m = center_y.data();
  buffers.center_z_m = center_z.data();
  buffers.mean_x_m = mean_x.data();
  buffers.mean_y_m = mean_y.data();
  buffers.mean_z_m = mean_z.data();
  buffers.point_count = point_count.data();
  buffers.dominant_label = labels.data();
  buffers.semantic_confidence = confidence.data();
  std::uint64_t written = 0U;
  std::uint8_t complete = 0U;
  if (impl_->snapshot_fill(handle.get(), 0.5F, metadata.generation, 0U, &buffers, count, &written,
                           &complete) != LINGTU_MAPS_SEMANTIC_OK ||
      written != count || complete == 0U) {
    if (error != nullptr) {
      *error = "semantic_map_snapshot_fill_failed";
    }
    return false;
  }

  SemanticMapSnapshot loaded;
  loaded.generation = metadata.generation;
  loaded.frame_id = frame_id.data();
  loaded.taxonomy = taxonomy.data();
  loaded.taxonomy_version = metadata.taxonomy_version;
  loaded.points.reserve(point_count_size);
  for (std::size_t i = 0U; i < point_count_size; ++i) {
    const bool has_mean = point_count[i] > 0U;
    const float x = has_mean ? mean_x[i] : center_x[i];
    const float y = has_mean ? mean_y[i] : center_y[i];
    const float z = has_mean ? mean_z[i] : center_z[i];
    if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
      loaded.points.push_back({x, y, z, labels[i], confidence[i]});
    }
  }
  if (loaded.points.empty()) {
    if (error != nullptr) {
      *error = "semantic_map_has_no_finite_points";
    }
    return false;
  }
  *snapshot = std::move(loaded);
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

}  // namespace lingtu::slam

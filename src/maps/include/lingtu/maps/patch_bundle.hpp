#pragma once

#include <cstddef>
#include <filesystem>

namespace lingtu::maps {

// Reads a complete LINGTU_PATCH_BUNDLE_V1 manifest (no dropped patches) and
// returns its patch count.
bool ReadPatchManifest(const std::filesystem::path& path, std::size_t* patch_count);

// map.pcd, poses.txt, patches/ and the manifest are present, and poses.txt,
// the patch files and the manifest name the same, complete set of scans.
bool HasCompletePatchBundle(const std::filesystem::path& dir);

// The map keeps the scans its OctoMap is replayed from: poses.txt and
// scan_origin.txt, with no patch dropped when SLAM wrote a bundle manifest.
// A replay missing dropped scans leaves their surfaces without ray evidence.
bool HasSavedRays(const std::filesystem::path& dir);

}  // namespace lingtu::maps

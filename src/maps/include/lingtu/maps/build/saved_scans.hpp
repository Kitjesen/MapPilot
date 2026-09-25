#pragma once

#include <array>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

namespace lingtu::maps {

// A saved keyframe in map coordinates, with its measured physical ray origin.
// No support dilation or inferred free space is present in this representation.
struct SavedScan {
  std::array<double, 3> origin{};
  std::vector<double> xyz;
};

void VisitSavedScans(const std::filesystem::path& directory,
                     const std::function<void(const SavedScan&)>& visit);

}  // namespace lingtu::maps

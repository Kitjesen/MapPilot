#include "frame_sequence.hpp"

#include <array>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <vector>

int main() try {
  using lingtu::drivers::realsense::FrameSequence;
  // wait_for_frames may reuse the slower stream's exposure in the next set.
  const std::array<std::array<std::uint64_t, 2>, 6> framesets{{
      {{0, 0}}, {{1, 0}}, {{2, 1}}, {{3, 1}}, {{4, 2}}, {{4, 3}}}};
  FrameSequence color, depth;
  std::vector<std::uint64_t> published_color, published_depth;
  for (const auto& pair : framesets) {
    if (color.isNew(pair[0])) published_color.push_back(pair[0]);
    if (depth.isNew(pair[1])) published_depth.push_back(pair[1]);
  }
  if (published_color != std::vector<std::uint64_t>{0, 1, 2, 3, 4})
    throw std::runtime_error("repeated color exposure published as a new frame");
  if (published_depth != std::vector<std::uint64_t>{0, 1, 2, 3})
    throw std::runtime_error("repeated depth exposure published as a new frame");

  FrameSequence restarted;
  if (!restarted.isNew(0))
    throw std::runtime_error("new capture must accept its first exposure");
} catch (const std::exception& error) {
  std::cerr << error.what() << '\n';
  return 1;
}

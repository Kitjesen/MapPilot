#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>

namespace lingtu::nav::endpoint {

constexpr std::uint8_t kPointFieldFloat32 = 7;

inline bool pointFieldIsScalarFloat32(
    std::uint8_t datatype,
    std::uint32_t count,
    std::uint32_t offset,
    std::uint32_t point_step) {
  return datatype == kPointFieldFloat32 && count == 1 && point_step >= sizeof(float) &&
      offset <= point_step - sizeof(float);
}

inline bool pointCloudStorageIsValid(
    std::uint32_t width,
    std::uint32_t height,
    std::uint32_t point_step,
    std::uint32_t row_step,
    std::size_t data_length) {
  const std::size_t rows = height == 0 ? 1U : static_cast<std::size_t>(height);
  const std::size_t cols = static_cast<std::size_t>(width);
  const std::size_t step = static_cast<std::size_t>(point_step);
  if (cols == 0 || step == 0 || cols > std::numeric_limits<std::size_t>::max() / step) {
    return false;
  }
  const std::size_t minimum_row_step = cols * step;
  const std::size_t actual_row_step = row_step == 0
      ? minimum_row_step
      : static_cast<std::size_t>(row_step);
  if (actual_row_step < minimum_row_step ||
      rows > std::numeric_limits<std::size_t>::max() / actual_row_step) {
    return false;
  }
  return data_length >= rows * actual_row_step;
}

}  // namespace lingtu::nav::endpoint

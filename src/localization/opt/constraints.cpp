#include "localization/opt/constraints.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

namespace lingtu::localization::opt {
namespace {

constexpr const char* kHeader[] = {
    "LINGTU_PGO_CONSTRAINTS_V1",
    "T_from_to tx ty tz qw qx qy qz",
    "RIGHT_TANGENT omega_x omega_y omega_z upsilon_x upsilon_y upsilon_z",
    "UPPER_TRIANGLE row_major 21",
};

std::vector<std::string> tokens(const std::string& line) {
  std::istringstream input(line);
  std::vector<std::string> out;
  for (std::string token; input >> token;) {
    out.push_back(token);
  }
  return out;
}

double parse_double(const std::string& token, std::size_t line) {
  errno = 0;
  char* end = nullptr;
  const double value = std::strtod(token.c_str(), &end);
  if (token.empty() || end == token.c_str() || end == nullptr || *end != '\0' ||
      errno == ERANGE || !std::isfinite(value)) {
    throw std::runtime_error("invalid finite number on constraints line " + std::to_string(line));
  }
  return value;
}

std::size_t parse_index(const std::string& token, std::size_t line) {
  if (token.empty() || token.front() == '-') {
    throw std::runtime_error("invalid index on constraints line " + std::to_string(line));
  }
  errno = 0;
  char* end = nullptr;
  const unsigned long long value = std::strtoull(token.c_str(), &end, 10);
  if (end == token.c_str() || end == nullptr || *end != '\0' || errno == ERANGE ||
      value > std::numeric_limits<std::size_t>::max()) {
    throw std::runtime_error("invalid index on constraints line " + std::to_string(line));
  }
  return static_cast<std::size_t>(value);
}

}  // namespace

bool valid_information_upper(const std::array<double, 21>& upper) {
  double matrix[6][6]{};
  std::size_t packed = 0;
  double scale = 0.0;
  bool nonzero = false;
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t col = row; col < 6; ++col) {
      const double value = upper[packed++];
      if (!std::isfinite(value)) {
        return false;
      }
      matrix[row][col] = value;
      matrix[col][row] = value;
      if (row == col) {
        if (value < 0.0) {
          return false;
        }
        scale = std::max(scale, value);
        nonzero = nonzero || value > 0.0;
      }
    }
  }
  if (!nonzero) {
    return false;
  }

  // Unpivoted LDL^T is sufficient here: a zero PSD diagonal implies its whole
  // row/column is zero. The tolerance only absorbs floating-point roundoff.
  const double tolerance = std::max(1.0, scale) * 1.0e-12;
  double lower[6][6]{};
  double diagonal[6]{};
  for (std::size_t row = 0; row < 6; ++row) {
    lower[row][row] = 1.0;
    double pivot = matrix[row][row];
    for (std::size_t k = 0; k < row; ++k) {
      pivot -= lower[row][k] * lower[row][k] * diagonal[k];
    }
    if (pivot < -tolerance) {
      return false;
    }
    diagonal[row] = pivot > tolerance ? pivot : 0.0;
    for (std::size_t next = row + 1; next < 6; ++next) {
      double residual = matrix[next][row];
      for (std::size_t k = 0; k < row; ++k) {
        residual -= lower[next][k] * lower[row][k] * diagonal[k];
      }
      if (diagonal[row] == 0.0) {
        if (std::abs(residual) > tolerance) {
          return false;
        }
      } else {
        lower[next][row] = residual / diagonal[row];
      }
    }
  }
  return true;
}

std::vector<GeometricConstraint> read_constraints(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input.is_open()) {
    throw std::runtime_error("failed to open constraints file: " + path.string());
  }
  std::string line;
  for (std::size_t i = 0; i < std::size(kHeader); ++i) {
    if (!std::getline(input, line)) {
      throw std::runtime_error("constraints header is incomplete");
    }
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line != kHeader[i]) {
      throw std::runtime_error("constraints header mismatch on line " + std::to_string(i + 1));
    }
  }

  std::vector<GeometricConstraint> constraints;
  std::size_t line_number = std::size(kHeader);
  while (std::getline(input, line)) {
    ++line_number;
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.empty()) {
      continue;
    }
    const auto row = tokens(line);
    if (row.size() != 30) {
      throw std::runtime_error("constraints row must contain 30 fields on line " +
                               std::to_string(line_number));
    }
    GeometricConstraint constraint;
    constraint.from_index = parse_index(row[0], line_number);
    constraint.to_index = parse_index(row[1], line_number);
    constraint.pose_from_to.x = parse_double(row[2], line_number);
    constraint.pose_from_to.y = parse_double(row[3], line_number);
    constraint.pose_from_to.z = parse_double(row[4], line_number);
    constraint.pose_from_to.qw = parse_double(row[5], line_number);
    constraint.pose_from_to.qx = parse_double(row[6], line_number);
    constraint.pose_from_to.qy = parse_double(row[7], line_number);
    constraint.pose_from_to.qz = parse_double(row[8], line_number);
    const double qnorm = std::sqrt(
        constraint.pose_from_to.qw * constraint.pose_from_to.qw +
        constraint.pose_from_to.qx * constraint.pose_from_to.qx +
        constraint.pose_from_to.qy * constraint.pose_from_to.qy +
        constraint.pose_from_to.qz * constraint.pose_from_to.qz);
    if (qnorm < 0.9 || qnorm > 1.1) {
      throw std::runtime_error("constraint quaternion is not unit length on line " +
                               std::to_string(line_number));
    }
    constraint.pose_from_to.qw /= qnorm;
    constraint.pose_from_to.qx /= qnorm;
    constraint.pose_from_to.qy /= qnorm;
    constraint.pose_from_to.qz /= qnorm;
    for (std::size_t i = 0; i < constraint.information_upper.size(); ++i) {
      constraint.information_upper[i] = parse_double(row[9 + i], line_number);
    }
    if (!valid_information_upper(constraint.information_upper)) {
      throw std::runtime_error("constraint information must be finite, nonzero, and PSD on line " +
                               std::to_string(line_number));
    }
    constraints.push_back(constraint);
  }
  return constraints;
}

}  // namespace lingtu::localization::opt

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "localization/opt/constraints.hpp"
#include "localization/opt/loop_constraints.hpp"
#include "localization/opt/pgo.hpp"
#include "localization/opt/pose_math.hpp"

namespace opt = lingtu::localization::opt;
namespace detail = lingtu::localization::opt::detail;

namespace {

void require(bool condition, const std::string &message) {
  if (!condition)
    throw std::runtime_error(message);
}

double dot4(const std::array<double, 4> &a, const std::array<double, 4> &b) {
  double out = 0.0;
  for (std::size_t i = 0; i < 4; ++i)
    out += a[i] * b[i];
  return out;
}

std::array<std::array<double, 6>, 6> unpack(const std::array<double, 21> &upper) {
  std::array<std::array<double, 6>, 6> matrix{};
  std::size_t k = 0;
  for (std::size_t r = 0; r < 6; ++r) {
    for (std::size_t c = r; c < 6; ++c)
      matrix[r][c] = matrix[c][r] = upper[k++];
  }
  return matrix;
}

double quadratic6(const std::array<double, 6> &x, const std::array<std::array<double, 6>, 6> &m) {
  double out = 0.0;
  for (std::size_t r = 0; r < 6; ++r)
    for (std::size_t c = 0; c < 6; ++c)
      out += x[r] * m[r][c] * x[c];
  return out;
}

opt::Pose small_pose(const std::array<double, 6> &x) {
  opt::Pose pose;
  const double angle = std::sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
  if (angle > 0.0) {
    const double scale = std::sin(0.5 * angle) / angle;
    pose.qw = std::cos(0.5 * angle);
    pose.qx = scale * x[0];
    pose.qy = scale * x[1];
    pose.qz = scale * x[2];
  }
  pose.x = x[3];
  pose.y = x[4];
  pose.z = x[5];
  return pose;
}

void test_raw_jacobian() {
  const std::array<double, 3> point{1.2, -0.7, 0.4};
  const std::array<double, 3> normal{0.3, -0.4, 0.866025403784};
  const auto analytic = detail::point_to_plane_jacobian(point, normal);
  constexpr double h = 1.0e-7;
  for (std::size_t axis = 0; axis < 4; ++axis) {
    auto residual = [&](double step) {
      const double yaw = axis == 0 ? step : 0.0;
      const double x =
          std::cos(yaw) * point[0] - std::sin(yaw) * point[1] + (axis == 1 ? step : 0.0);
      const double y =
          std::sin(yaw) * point[0] + std::cos(yaw) * point[1] + (axis == 2 ? step : 0.0);
      const double z = point[2] + (axis == 3 ? step : 0.0);
      return normal[0] * x + normal[1] * y + normal[2] * z;
    };
    const double numeric = (residual(h) - residual(-h)) / (2.0 * h);
    require(std::abs(numeric - analytic[axis]) < 1.0e-8, "raw J4 finite difference mismatch");
  }
}

void test_body_conversion_and_information() {
  opt::Pose q;
  q.x = 1.0;
  q.y = -0.4;
  q.z = 0.2;
  q = opt::pose_with_rpy(q, 0.0, 0.0, 0.35);
  opt::Pose a;
  a.x = 0.2;
  a.y = 0.1;
  a.z = -0.3;
  a = opt::pose_with_rpy(a, 0.3, -0.25, 0.0);
  const auto c = detail::gravity_left_to_body_right_jacobian(q, a);

  detail::Matrix4 h4{};
  for (std::size_t i = 0; i < 4; ++i)
    h4[i][i] = 2.0 + i;
  const auto upper = detail::body_right_information_upper(h4, 2.0, q, a);
  require(opt::valid_information_upper(upper), "converted information is not PSD");
  const auto omega = unpack(upper);
  const std::array<double, 6> xi{0.2, -0.1, 0.3, 0.4, -0.2, 0.1};
  std::array<double, 4> delta{};
  for (std::size_t r = 0; r < 4; ++r)
    for (std::size_t col = 0; col < 6; ++col)
      delta[r] += c[r][col] * xi[col];
  std::array<double, 4> hdelta{};
  for (std::size_t r = 0; r < 4; ++r)
    for (std::size_t col = 0; col < 4; ++col)
      hdelta[r] += h4[r][col] * delta[col];
  require(std::abs(quadratic6(xi, omega) - dot4(delta, hdelta) / 4.0) < 1.0e-10,
          "information cost changed across tangent frames");

  // The two gravity roll/pitch directions must remain exact null modes after
  // transport into the tilted body-right tangent.
  const opt::Pose t_inv = opt::inverse_pose(opt::compose_pose(q, a));
  for (std::size_t gravity_axis = 0; gravity_axis < 2; ++gravity_axis) {
    std::array<double, 6> eta{};
    eta[gravity_axis] = 1.0;
    const auto r = opt::rotate_vector(t_inv, eta[0], eta[1], eta[2]);
    std::array<double, 6> null{r[0], r[1], r[2], 0.0, 0.0, 0.0};
    const std::array<double, 3> t{t_inv.x, t_inv.y, t_inv.z};
    null[3] = t[1] * r[2] - t[2] * r[1];
    null[4] = t[2] * r[0] - t[0] * r[2];
    null[5] = t[0] * r[1] - t[1] * r[0];
    require(std::abs(quadratic6(null, omega)) < 1.0e-9, "gravity null direction was constrained");
  }

  // End-to-end finite difference of Q'=Q*A*Exp(xi)*A^-1 gives P*Ad_(Q*A).
  const opt::Pose measurement = opt::compose_pose(q, a);
  constexpr double eps = 1.0e-7;
  for (std::size_t axis = 0; axis < 6; ++axis) {
    std::array<double, 6> step{};
    step[axis] = eps;
    const opt::Pose q_plus =
        opt::compose_pose(opt::compose_pose(measurement, small_pose(step)), opt::inverse_pose(a));
    step[axis] = -eps;
    const opt::Pose q_minus =
        opt::compose_pose(opt::compose_pose(measurement, small_pose(step)), opt::inverse_pose(a));
    const opt::Pose d_plus = opt::compose_pose(q_plus, opt::inverse_pose(q));
    const opt::Pose d_minus = opt::compose_pose(q_minus, opt::inverse_pose(q));
    const auto rp = opt::pose_rpy(d_plus);
    const auto rm = opt::pose_rpy(d_minus);
    const std::array<double, 4> numeric{
        (rp[2] - rm[2]) / (2.0 * eps), (d_plus.x - d_minus.x) / (2.0 * eps),
        (d_plus.y - d_minus.y) / (2.0 * eps), (d_plus.z - d_minus.z) / (2.0 * eps)};
    for (std::size_t row = 0; row < 4; ++row)
      require(std::abs(numeric[row] - c[row][axis]) < 2.0e-6,
              "body-right conversion finite difference mismatch");
  }
}

void test_noise_fail_closed() {
  std::vector<detail::PlaneSample> exact(8);
  for (std::size_t i = 0; i < exact.size(); ++i) {
    exact[i].source = {static_cast<double>(i), 0.0, 0.0};
    exact[i].target = exact[i].source;
    exact[i].normal = {0.0, 0.0, 1.0};
  }
  const auto information = detail::refine_plane_information(exact, opt::Pose{}, 3);
  require(!information.valid && information.sigma == 0.0,
          "zero-noise degenerate samples fabricated information");
  detail::Matrix4 identity{};
  for (std::size_t i = 0; i < 4; ++i)
    identity[i][i] = 1.0;
  require(!opt::valid_information_upper(
              detail::body_right_information_upper(identity, 0.0, opt::Pose{}, opt::Pose{})),
          "zero sigma produced graph information");
}

void write_cloud(const std::filesystem::path &path) {
  std::ofstream out(path, std::ios::binary);
  out << "VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n"
         "COUNT 1 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA binary\n";
  const float point[4]{};
  out.write(reinterpret_cast<const char *>(point), sizeof(point));
}

std::array<double, 21> full_information(double weight) {
  std::array<double, 21> out{};
  for (std::size_t i : {0U, 6U, 11U, 15U, 18U, 20U})
    out[i] = weight;
  return out;
}

opt::GeometricConstraint factor(std::size_t from, std::size_t to, double x,
                                const std::array<double, 21> &information) {
  opt::GeometricConstraint out;
  out.from_index = from;
  out.to_index = to;
  out.pose_from_to.x = x;
  out.information_upper = information;
  return out;
}

void test_rank4_loop_reduces_drift() {
  const auto root = std::filesystem::temp_directory_path() / "lingtu-loop-information-test";
  std::error_code error;
  std::filesystem::remove_all(root, error);
  const auto source = root / "source";
  std::filesystem::create_directories(source / "patches");
  std::ofstream(source / "poses.txt") << "0.pcd 0 0 0 1 0 0 0\n"
                                         "1.pcd 1 0 0 1 0 0 0\n"
                                         "2.pcd 2.4 0 0 1 0 0 0\n";
  std::ofstream(source / "patch_bundle.manifest")
      << "LINGTU_PATCH_BUNDLE_V1\ncomplete 1\ndropped_count 0\nfirst_sequence 0\n"
         "last_sequence 2\npatch_count 3\n";
  for (const char *name : {"0.pcd", "1.pcd", "2.pcd"})
    write_cloud(source / "patches" / name);
  write_cloud(source / "map.pcd");
  detail::Matrix4 h4{};
  for (std::size_t i = 0; i < 4; ++i)
    h4[i][i] = 1000.0;
  const auto loop_information =
      detail::body_right_information_upper(h4, 1.0, opt::Pose{}, opt::Pose{});
  require(opt::valid_information_upper(loop_information), "rank-4 loop information rejected");
  const std::vector<opt::GeometricConstraint> odometry{factor(0, 1, 1.0, full_information(1000.0)),
                                                       factor(1, 2, 1.4, full_information(1000.0))};
  const auto odometry_result = opt::pgo(opt::files(source, root / "odom"), odometry);
  require(odometry_result.ok, "odometry-only PGO failed");
  const auto odometry_poses = opt::read_poses(root / "odom" / "poses.txt");
  require(std::abs(odometry_poses.back().pose.x - 2.4) < 1.0e-3,
          "odometry-only baseline did not preserve its consistent drift");
  auto factors = odometry;
  factors.push_back(factor(0, 2, 2.0, loop_information));
  const auto result = opt::pgo(opt::files(source, root / "out"), factors);
  require(result.ok && result.changed,
          "rank-4 loop PGO failed: " + result.code + " / " + result.message);
  const auto poses = opt::read_poses(root / "out" / "poses.txt");
  require(std::abs(poses.back().pose.x - 2.0) < std::abs(odometry_poses.back().pose.x - 2.0),
          "rank-4 loop did not improve the odometry-only drift");
  std::filesystem::remove_all(root, error);
}

}  // namespace

int main() {
  try {
    test_raw_jacobian();
    test_body_conversion_and_information();
    test_noise_fail_closed();
    test_rank4_loop_reduces_drift();
    std::cout << "loop_information_test: passed\n";
    return 0;
  } catch (const std::exception &exception) {
    std::cerr << "loop_information_test: " << exception.what() << '\n';
    return 1;
  }
}

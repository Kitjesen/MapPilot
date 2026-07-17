#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

#include "components.hpp"
#include "localization/opt/graph.hpp"
#include "localization/opt/hba.hpp"
#include "localization/opt/map.hpp"
#include "localization/opt/pgo.hpp"

namespace {

void write_patch(const std::filesystem::path &path, const std::string &point) {
  std::ofstream out(path);
  out << "VERSION 0.7\n";
  out << "FIELDS x y z intensity\n";
  out << "SIZE 4 4 4 4\n";
  out << "TYPE F F F F\n";
  out << "COUNT 1 1 1 1\n";
  out << "WIDTH 1\n";
  out << "HEIGHT 1\n";
  out << "POINTS 1\n";
  out << "DATA ascii\n";
  out << point << "\n";
}

void require(bool condition, const std::string &message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

std::vector<char> read_bytes(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  return std::vector<char>(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

}  // namespace

int main() {
  using lingtu::native_runtime::find;
  using lingtu::native_runtime::ready;
  using lingtu::native_runtime::State;

  std::size_t count = 0;
  const auto *components = lingtu::native_runtime::components(count);
  require(components != nullptr, "component registry is null");
  require(count >= 9, "component registry is incomplete");

  require(ready("lidar"), "lidar component is not ready");
  require(ready("slam"), "slam component is not ready");
  require(ready("nav"), "nav component is not ready");

  const auto *pgo = find("pgo");
  require(pgo != nullptr, "pgo component is missing");
  require(pgo->state == State::Ready, "pgo component is not ready");
  require(!pgo->requires_ros2, "pgo unexpectedly requires ROS 2");
  require(pgo->product_default, "pgo component is not the registered default");

  const auto *hba = find("hba");
  require(hba != nullptr, "hba component is missing");
  require(hba->state == State::Ready, "hba component is not ready");
  require(!hba->requires_ros2, "hba unexpectedly requires ROS 2");
  require(!hba->product_default, "hba unexpectedly became the product default");

  const auto stamp = std::chrono::high_resolution_clock::now().time_since_epoch().count();
  const auto temp_dir = std::filesystem::temp_directory_path() /
                        ("lingtu_native_runner_contract_" + std::to_string(stamp));
  std::filesystem::remove_all(temp_dir);
  std::filesystem::create_directories(temp_dir / "patches");
  {
    write_patch(temp_dir / "map.pcd", "9 9 9 0");
    std::ofstream(temp_dir / "poses.txt") << "0.pcd 0 0 0 1 0 0 0\n"
                                          << "1.pcd 1 0 0 1 0 0 0\n";
    write_patch(temp_dir / "patches" / "0.pcd", "0 0 0 1");
    write_patch(temp_dir / "patches" / "1.pcd", "0 0 0 2");
  }

  const auto artifacts = lingtu::localization::opt::files(temp_dir);
  const auto validation = lingtu::localization::opt::check(artifacts);
  require(validation.ok, "map fixture validation failed");
  require(validation.patch_count == 2, "map fixture patch count is wrong");

  const auto map_before = read_bytes(temp_dir / "map.pcd");
  const auto poses_before = read_bytes(temp_dir / "poses.txt");

  const auto pgo_result = lingtu::localization::opt::pgo(artifacts);
  require(pgo_result.ok, "pgo did not return a safe result");
  require(pgo_result.code == "skipped_no_independent_constraints",
          "pgo must not optimize a graph derived only from its input poses");
  require(!pgo_result.changed, "pgo changed a map without independent constraints");
  require(pgo_result.pose_count == 2, "pgo reported the wrong pose count");
  require(read_bytes(temp_dir / "map.pcd") == map_before, "pgo changed map.pcd");
  require(read_bytes(temp_dir / "poses.txt") == poses_before, "pgo changed poses.txt");
  require(!std::filesystem::exists(temp_dir / "map.pcd.preopt"),
          "pgo created a backup despite skipping optimization");

  const auto hba_result = lingtu::localization::opt::hba(artifacts);
  require(hba_result.ok, "hba did not return a safe result");
  require(hba_result.code == "skipped_no_independent_constraints",
          "hba must not optimize a graph derived only from its input poses");
  require(!hba_result.changed, "hba changed a map without independent constraints");
  require(hba_result.pose_count == 2, "hba reported the wrong pose count");
  require(read_bytes(temp_dir / "map.pcd") == map_before, "hba changed map.pcd");
  require(read_bytes(temp_dir / "poses.txt") == poses_before, "hba changed poses.txt");

  lingtu::localization::opt::OptimizeOptions invalid_options;
  invalid_options.strategy = "invalid-default-constraint";
  lingtu::localization::opt::GeometricConstraint missing_information;
  missing_information.from_index = 0;
  missing_information.to_index = 1;
  invalid_options.geometric_constraints.push_back(missing_information);
  const auto invalid_constraint_result =
      lingtu::localization::opt::optimize_map(artifacts, invalid_options);
  require(!invalid_constraint_result.ok, "default geometric information failed open");
  require(invalid_constraint_result.code == "geometric_constraint_invalid",
          "default geometric information returned the wrong failure code");
  require(read_bytes(temp_dir / "map.pcd") == map_before,
          "invalid geometric constraint changed map.pcd");
  require(read_bytes(temp_dir / "poses.txt") == poses_before,
          "invalid geometric constraint changed poses.txt");

  const auto legacy_poses = temp_dir / "legacy_tum_poses.txt";
  { std::ofstream(legacy_poses) << "0 0 0 0 0 0.9999619230641713 0.0087265354983739\n"; }
  const auto parsed_legacy = lingtu::localization::opt::read_poses(legacy_poses);
  require(parsed_legacy.size() == 1, "legacy TUM pose was not parsed");
  const auto &legacy_pose = parsed_legacy.front().pose;
  const double legacy_yaw =
      std::atan2(2.0 * (legacy_pose.qw * legacy_pose.qz + legacy_pose.qx * legacy_pose.qy),
                 1.0 - 2.0 * (legacy_pose.qy * legacy_pose.qy + legacy_pose.qz * legacy_pose.qz));
  require(std::abs(legacy_yaw - 3.12413936106985) < 1.0e-9,
          "legacy TUM yaw near 179 degrees was misclassified as wxyz");

  const auto invalid_poses = temp_dir / "invalid_quaternion_poses.txt";
  { std::ofstream(invalid_poses) << "bad.pcd 0 0 0 0 0 0 0\n"; }
  bool invalid_quaternion_rejected = false;
  try {
    (void)lingtu::localization::opt::read_poses(invalid_poses);
  } catch (const std::exception &exception) {
    invalid_quaternion_rejected =
        std::string(exception.what()).find("invalid_pose_quaternion") != std::string::npos;
  }
  require(invalid_quaternion_rejected, "zero quaternion was not rejected explicitly");

  std::filesystem::remove_all(temp_dir);

  return 0;
}

#include "localization/map_optimization/cloud.hpp"
#include "localization/map_optimization/poses.hpp"

#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace m = lingtu::localization::map_optimization;

void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}

void reads_saved_binary_cloud(const std::filesystem::path& directory) {
  const auto path = directory / "scan.pcd";
  const std::array<float, 8> values{1.25F, -2.5F, .75F, 42.F,
                                    -3.F, 4.F, 5.F, 7.F};
  {
    std::ofstream out(path, std::ios::binary);
    out << "VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n"
           "TYPE F F F F\nCOUNT 1 1 1 1\nWIDTH 2\nHEIGHT 1\n"
           "VIEWPOINT 0 0 0 1 0 0 0\nPOINTS 2\nDATA binary\n";
    out.write(reinterpret_cast<const char*>(values.data()), sizeof(values));
    require(out.good(), "could not write binary cloud fixture");
  }
  const auto points = m::read_point_cloud(path);
  require(points.size() == 2, "saved cloud point count changed");
  require(points[0].x == 1.25F && points[0].y == -2.5F &&
              points[0].z == .75F && points[0].intensity == 42.F &&
              points[1].x == -3.F && points[1].intensity == 7.F,
          "saved cloud coordinates or intensity changed");
}

void reads_saved_pose_order_and_orientation(const std::filesystem::path& directory) {
  const auto path = directory / "poses.raw.txt";
  {
    std::ofstream out(path);
    out << "# patch_name x y z qw qx qy qz\n"
           "scan_2.pcd 1 2 3 0.8 0.6 0 0\n"
           "scan_1.pcd -4 5 6 1 0 0 0\n";
    require(out.good(), "could not write pose fixture");
  }
  const auto poses = m::read_poses(path);
  require(poses.size() == 2 && poses[0].patch_name == "scan_2.pcd" &&
              poses[1].patch_name == "scan_1.pcd",
          "saved pose order or patch identity changed");
  const auto& first = poses[0].pose;
  require(first.x == 1 && first.y == 2 && first.z == 3 &&
              first.qw == .8 && first.qx == .6 && first.qy == 0 && first.qz == 0,
          "saved pose coordinate or quaternion order changed");
}

int main() {
  const auto directory = std::filesystem::temp_directory_path() /
      ("lingtu-mapping-io-" + std::to_string(
          std::chrono::steady_clock::now().time_since_epoch().count()));
  try {
    std::filesystem::create_directories(directory);
    reads_saved_binary_cloud(directory);
    reads_saved_pose_order_and_orientation(directory);
    std::filesystem::remove_all(directory);
    std::cout << "mapping IO: binary cloud and saved pose contracts passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::filesystem::remove_all(directory);
    std::cerr << error.what() << '\n';
    return 1;
  }
}

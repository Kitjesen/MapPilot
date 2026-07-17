#include "bbs3d_global_localizer.h"
#include "icp_localizer.h"

#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>

namespace {

void usage() {
    std::cerr
        << "Usage: localizer_cli --map map.pcd --scan scan.pcd "
        << "[--x 0 --y 0 --z 0 --yaw 0] [--global]\n";
}

bool read_double(int argc, char** argv, int& i, double& value) {
    if (i + 1 >= argc) return false;
    try {
        value = std::stod(argv[++i]);
        return true;
    } catch (...) {
        return false;
    }
}

M4F pose_from_xyzyaw(double x, double y, double z, double yaw) {
    M4F pose = M4F::Identity();
    Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
    pose.block<3, 3>(0, 0) = yaw_angle.toRotationMatrix().cast<float>();
    pose.block<3, 1>(0, 3) = V3F(x, y, z);
    return pose;
}

void print_pose(const M4F& pose, double score) {
    std::cout << "success=true\n";
    std::cout << "score=" << score << "\n";
    std::cout << "pose=[";
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            if (r || c) std::cout << ", ";
            std::cout << pose(r, c);
        }
    }
    std::cout << "]\n";
}

}  // namespace

int main(int argc, char** argv) {
    std::string map_path;
    std::string scan_path;
    bool use_global = false;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double yaw = 0.0;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--map" && i + 1 < argc) {
            map_path = argv[++i];
        } else if (arg == "--scan" && i + 1 < argc) {
            scan_path = argv[++i];
        } else if (arg == "--global") {
            use_global = true;
        } else if (arg == "--x") {
            if (!read_double(argc, argv, i, x)) {
                usage();
                return 2;
            }
        } else if (arg == "--y") {
            if (!read_double(argc, argv, i, y)) {
                usage();
                return 2;
            }
        } else if (arg == "--z") {
            if (!read_double(argc, argv, i, z)) {
                usage();
                return 2;
            }
        } else if (arg == "--yaw") {
            if (!read_double(argc, argv, i, yaw)) {
                usage();
                return 2;
            }
        } else {
            usage();
            return 2;
        }
    }

    if (map_path.empty() || scan_path.empty()) {
        usage();
        return 2;
    }

    CloudType::Ptr scan(new CloudType);
    if (pcl::io::loadPCDFile<PointType>(scan_path, *scan) != 0 || scan->empty()) {
        std::cerr << "failed to load scan pcd: " << scan_path << "\n";
        return 1;
    }

    ICPConfig icp_config;
    ICPLocalizer icp(icp_config);
    if (!icp.loadMap(map_path)) {
        return 1;
    }

    M4F guess = pose_from_xyzyaw(x, y, z, yaw);
    if (use_global) {
        CloudType::Ptr map(new CloudType);
        if (pcl::io::loadPCDFile<PointType>(map_path, *map) != 0 || map->empty()) {
            std::cerr << "failed to load map pcd for BBS3D: " << map_path << "\n";
            return 1;
        }

        BBS3DGlobalLocalizer bbs3d;
        if (!bbs3d.available()) {
            std::cerr << "bbs3d unavailable; build with CPU_BBS3D_ROOT or omit --global\n";
            return 3;
        }
        if (!bbs3d.set_map(map)) {
            std::cerr << "failed to set BBS3D map\n";
            return 1;
        }
        auto global = bbs3d.localize(scan);
        if (!global.success) {
            std::cerr << "global relocalize failed: " << global.message << "\n";
            return 4;
        }
        guess = global.pose;
    }

    icp.setInput(scan);
    if (!icp.align(guess)) {
        std::cerr << "icp align failed; fitness=" << icp.getLastFitnessScore() << "\n";
        return 4;
    }

    print_pose(guess, icp.getLastFitnessScore());
    return 0;
}

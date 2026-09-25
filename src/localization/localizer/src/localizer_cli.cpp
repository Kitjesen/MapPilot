#include "bbs3d_global_localizer.h"
#include "icp_localizer.h"

#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#include <pcl/io/pcd_io.h>

namespace {

void usage() {
    std::cerr
        << "Usage: localizer_cli --map map.pcd --scan scan.pcd "
        << "[--x 0 --y 0 --z 0 --yaw 0] [--global] "
           "[--bbs-max-level N] [--bbs-min-res METERS] "
           "[--scan-roll RAD --scan-pitch RAD]\n"
           "--global expects a body-frame scan; --scan-roll/--scan-pitch are "
           "body ZYX angles in gravity-aligned odom (default: already level).\n";
}

bool read_double(int argc, char** argv, int& i, double& value) {
    if (i + 1 >= argc) return false;
    try {
        value = std::stod(argv[++i]);
        return std::isfinite(value);
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
    BBS3DGlobalLocalizer::Config bbs_config;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double yaw = 0.0;
    double scan_roll = 0.0;
    double scan_pitch = 0.0;
    double bbs_score_percentage = -1.0;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--map" && i + 1 < argc) {
            map_path = argv[++i];
        } else if (arg == "--scan" && i + 1 < argc) {
            scan_path = argv[++i];
        } else if (arg == "--global") {
            use_global = true;
        } else if (arg == "--bbs-max-level" && i + 1 < argc) {
            try {
                bbs_config.max_level = std::stoi(argv[++i]);
                if (bbs_config.max_level < 0 || bbs_config.max_level > 5) throw std::out_of_range("bbs level");
            } catch (...) {
                usage();
                return 2;
            }
        } else if (arg == "--bbs-min-res") {
            if (!read_double(argc, argv, i, bbs_config.min_level_res)
                || bbs_config.min_level_res < 0.1 || bbs_config.min_level_res > 2.0) {
                usage();
                return 2;
            }
        } else if (arg == "--scan-roll") {
            if (!read_double(argc, argv, i, scan_roll)) {
                usage();
                return 2;
            }
        } else if (arg == "--scan-pitch") {
            if (!read_double(argc, argv, i, scan_pitch)) {
                usage();
                return 2;
            }
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

        BBS3DGlobalLocalizer bbs3d(bbs_config);
        if (!bbs3d.available()) {
            std::cerr << "bbs3d unavailable; build with CPU_BBS3D_ROOT or omit --global\n";
            return 3;
        }
        if (!bbs3d.set_map(map)) {
            std::cerr << "failed to set BBS3D map\n";
            return 1;
        }
        const Eigen::Matrix3d body_to_level = (
            Eigen::AngleAxisd(scan_pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(scan_roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
        auto global = bbs3d.localize(scan, body_to_level);
        if (!global.success) {
            std::cerr << "global relocalize failed: " << global.message << "\n";
            return 4;
        }
        bbs_score_percentage = global.score_percentage;
        guess = global.pose;
    }

    icp.setInput(scan);
    if (!icp.align(guess)) {
        std::cerr << "icp align failed; fitness=" << icp.getLastFitnessScore() << "\n";
        return 4;
    }
    if (use_global) {
        const int evaluated = icp.getLastEvaluatedPoints();
        const double overlap = evaluated > 0
            ? static_cast<double>(icp.getLastInliers()) / evaluated : -1.0;
        if (icp.getLastFitnessScore() < 0.0 ||
            icp.getLastFitnessScore() > 0.0144 || overlap < 0.80) {
            std::cerr << "global refinement quality rejected; fitness="
                      << icp.getLastFitnessScore() << " overlap=" << overlap << "\n";
            return 4;
        }
    }

    print_pose(guess, icp.getLastFitnessScore());
    if (use_global) {
        std::cout << "bbs_score_percentage=" << bbs_score_percentage << "\n";
        std::cout << "icp_mse=" << icp.getLastFitnessScore() << "\n";
    }
    return 0;
}

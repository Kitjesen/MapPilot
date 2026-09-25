#include "map_icp.hpp"
#include "native_relocalizer.hpp"
#include "relocalization_gate.hpp"

#include <Eigen/Geometry>
#include <pcl/io/pcd_io.h>
#include <cmath>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <stdexcept>

namespace {
using namespace lingtu::slam;

void require(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}

Pose3d pose(const M4F& matrix) {
  Pose3d out;
  const Eigen::Quaternionf q(matrix.block<3, 3>(0, 0));
  out.x = matrix(0, 3); out.y = matrix(1, 3); out.z = matrix(2, 3);
  out.qx = q.x(); out.qy = q.y(); out.qz = q.z(); out.qw = q.w();
  return out;
}

Cloud scanFrom(const CloudType& cloud, const M4F& map_body) {
  Cloud out;
  out.frame_id = "body";
  const M4F inverse = map_body.inverse();
  for (const auto& point : cloud) {
    const Eigen::Vector4f p = inverse * Eigen::Vector4f(point.x, point.y, point.z, 1.F);
    PointXYZIT item;
    item.x = p.x(); item.y = p.y(); item.z = p.z();
    out.points.push_back(item);
  }
  return out;
}

void print(const NativeRelocalizationResult& r) {
  std::cout << "success=" << r.success << " engine=" << r.engine
            << " message=" << r.message << " backend=" << r.refine_backend
            << " overlap=" << r.overlap_inlier_ratio << " mse=" << r.quality
            << " xyz=" << r.map_body.x << ',' << r.map_body.y << ',' << r.map_body.z
            << " quaternion=" << r.map_body.qx << ',' << r.map_body.qy << ','
            << r.map_body.qz << ',' << r.map_body.qw << '\n';
}
}  // namespace

int run(int argc, char** argv) {
  if (argc == 8 && std::string(argv[1]) == "--status-field") {
    NativeRelocalizer localizer;
    require(localizer.loadMap(argv[2]), "field map load failed");
    CloudType cloud;
    require(pcl::io::loadPCDFile(argv[3], cloud) == 0, "field scan load failed");
    Pose3d odom;
    odom.qx = std::stod(argv[4]); odom.qy = std::stod(argv[5]);
    odom.qz = std::stod(argv[6]); odom.qw = std::stod(argv[7]);
    const auto scan = scanFrom(cloud, M4F::Identity());
    auto work = std::async(std::launch::async, [&] {
      return localizer.globalRelocalize(scan, odom);
    });
    double max_status_ms = 0.0;
    unsigned probes = 0;
    while (work.wait_for(std::chrono::milliseconds(1)) != std::future_status::ready) {
      const auto started = std::chrono::steady_clock::now();
      require(localizer.hasMap() && localizer.supportsGlobalRelocalization(),
              "localization capability disappeared during computation");
      const double elapsed = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started).count();
      max_status_ms = std::max(max_status_ms, elapsed);
      ++probes;
    }
    const auto result = work.get();
    print(result);
    std::cout << "status_probes=" << probes << " max_status_ms=" << max_status_ms << '\n';
    require(result.success && probes > 0, "concurrent field localization failed");
    require(max_status_ms < 200.0,
            "status queries blocked on localization computation and starved sensor processing");
    return 0;
  }
  // Real copied scans expose nearby false minima that ideal walls can miss.
  // Rejecting a bad hint is valid; accepting a different location is not.
  if ((argc == 10 || argc == 11) && std::string(argv[1]) == "--seed-field") {
    NativeRelocalizer localizer;
    require(localizer.loadMap(argv[2]), "field map load failed");
    CloudType cloud;
    require(pcl::io::loadPCDFile(argv[3], cloud) == 0, "field scan load failed");
    Pose3d odom;
    odom.qx = std::stod(argv[4]); odom.qy = std::stod(argv[5]);
    odom.qz = std::stod(argv[6]); odom.qw = std::stod(argv[7]);
    const Cloud scan = scanFrom(cloud, M4F::Identity());
    for (const double x : {0.0, 0.5, 0.75, 1.0}) {
      Pose3d hint;
      hint.x = x;
      const double yaw = argc == 11 ? std::stod(argv[10]) : 0.0;
      hint.qz = std::sin(0.5 * yaw);
      hint.qw = std::cos(0.5 * yaw);
      const auto result = localizer.relocalize(scan, hint, odom);
      std::cout << "hint_x=" << x << ' ';
      print(result);
      if (x <= 0.75) require(result.success, "nearby valid hint must still recover");
      if (x == 1.0) {
        require(!result.success && result.message == "native_relocalizer_seed_outside_search",
                "an out-of-range better match must reject the hint, not promote a false minimum");
      }
      if (!result.success) continue;
      require(std::hypot(result.map_body.x - std::stod(argv[8]),
                         result.map_body.y - std::stod(argv[9])) < 0.1,
              "seed recovery must not accept the observed false minimum");
      RelocalizationGateConfig config;
      config.min_evaluated_points = 100;
      RelocalizationGateInput input;
      input.converged = result.refine_converged;
      input.fitness = result.quality;
      input.inliers = result.refine_inliers;
      input.evaluated_points = result.evaluated_points;
      input.pos_cov_trace = result.refine_pos_cov_trace;
      input.candidate_map_odom = result.map_odom;
      require(EvaluateRelocalizationGate(config, input).accepted,
              "correct seed recovery must also pass the initialization gate");
    }
    return 0;
  }
  // Offline fixture for an interior pose, where saved endpoint seeds should
  // fail and the gravity-aligned BBS search must reach strict ICP refinement.
  if (argc == 10 && std::string(argv[1]) == "--global-field") {
    NativeRelocalizer localizer;
    require(localizer.loadMap(argv[2]), "field map load failed");
    CloudType cloud;
    require(pcl::io::loadPCDFile(argv[3], cloud) == 0, "field scan load failed");
    Pose3d odom;
    odom.qx = std::stod(argv[4]); odom.qy = std::stod(argv[5]);
    odom.qz = std::stod(argv[6]); odom.qw = std::stod(argv[7]);
    const auto result = localizer.globalRelocalize(scanFrom(cloud, M4F::Identity()), odom);
    print(result);
    require(result.success && result.message == "native_global_relocalized",
            "interior field scan must pass BBS and ICP");
    require(std::hypot(result.map_body.x - std::stod(argv[8]),
                       result.map_body.y - std::stod(argv[9])) < 0.5,
            "global field pose must recover the expected location");
    RelocalizationGateConfig config;
    config.min_evaluated_points = 100;
    RelocalizationGateInput gate_input;
    gate_input.converged = result.refine_converged;
    gate_input.fitness = result.quality;
    gate_input.inliers = result.refine_inliers;
    gate_input.evaluated_points = result.evaluated_points;
    gate_input.pos_cov_trace = result.refine_pos_cov_trace;
    gate_input.candidate_map_odom = result.map_odom;
    const auto gate = EvaluateRelocalizationGate(config, gate_input);
    std::cout << "initialization_gate=" << gate.reason
              << " alignment_tilt_rad=" << gate.alignment_tilt_rad << '\n';
    require(gate.accepted, "global field pose must also pass the runtime initialization gate");
    return 0;
  }
  // Optional field fixture: map.pcd scan-body.pcd odom-qx qy qz qw.
  // This evaluates copied data only; no runtime or motion transport is opened.
  if (argc == 7) {
    NativeRelocalizer localizer;
    require(localizer.loadMap(argv[1]), "field map load failed");
    CloudType cloud;
    require(pcl::io::loadPCDFile(argv[2], cloud) == 0, "field scan load failed");
    Pose3d odom;
    odom.qx = std::stod(argv[3]); odom.qy = std::stod(argv[4]);
    odom.qz = std::stod(argv[5]); odom.qw = std::stod(argv[6]);
    const auto scan = scanFrom(cloud, M4F::Identity());
    const auto initial = localizer.relocalize(scan, Pose3d{}, odom);
    print(initial);
    if (std::filesystem::is_regular_file(
            std::filesystem::path(argv[1]).parent_path() / "poses.txt")) {
      const auto recovered = localizer.globalRelocalize(scan, odom);
      print(recovered);
      require(recovered.success &&
                  recovered.message == "native_relocalized_saved_pose_verified",
              "field saved-pose recovery failed");
    }
    if (initial.success) {
      const auto prediction = localizer.relocalize(scan, initial.map_body, odom, true);
      print(prediction);
      std::cout << "same_scan_correction_m=" << std::sqrt(
          std::pow(prediction.map_body.x - initial.map_body.x, 2) +
          std::pow(prediction.map_body.y - initial.map_body.y, 2) +
          std::pow(prediction.map_body.z - initial.map_body.z, 2)) << '\n';
    }
    return 0;
  }
  require(argc == 1, "expected map, scan and odometry quaternion, or no arguments");

  CloudType map;
  // Unequal walls, a floor and a pillar constrain all four planar degrees.
  for (int i = 0; i < 90; ++i) {
    for (int j = 0; j < 35; ++j) {
      PointType p;
      p.x = -2.F + i * .065F; p.y = -1.6F; p.z = -.4F + j * .075F;
      map.push_back(p);
      p.x = -2.F; p.y = -1.6F + i * .049F;
      map.push_back(p);
      p.x = -2.F + i * .065F; p.y = -1.6F + j * .12F; p.z = -.4F;
      map.push_back(p);
      if (i < 30) {
        p.x = .8F + .23F * std::cos(i * .21F);
        p.y = .4F + .23F * std::sin(i * .21F);
        p.z = -.4F + j * .075F;
        map.push_back(p);
      }
    }
  }
  const auto directory = std::filesystem::temp_directory_path() /
      ("lingtu-map-icp-seed-" + std::to_string(
          std::filesystem::file_time_type::clock::now().time_since_epoch().count()));
  std::filesystem::create_directories(directory);
  const auto path = directory / "map.pcd";
  require(pcl::io::savePCDFileBinary(path.string(), map) == 0, "save fixture failed");

  NativeRelocalizer localizer;
  require(localizer.loadMap(path.string()), "load fixture failed");
  M4F actual = M4F::Identity();
  actual.block<3, 3>(0, 0) = (
      Eigen::AngleAxisf(.04F, Eigen::Vector3f::UnitZ()) *
      Eigen::AngleAxisf(.055F, Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(-.03F, Eigen::Vector3f::UnitX())).toRotationMatrix();
  actual(0, 3) = -.22F; actual(1, 3) = -.29F; actual(2, 3) = .04F;
  const Cloud scan = scanFrom(map, actual);
  M4F odom_matrix = actual;
  odom_matrix.block<3, 1>(0, 3).setZero();
  const Pose3d odom = pose(odom_matrix);
  const auto result = localizer.relocalize(scan, Pose3d{}, odom);
  print(result);
  require(result.success, "nearby yaw-only initial pose must be aligned before quality rejection");
  require(result.engine == "seeded_gicp", "seeded recovery must report its actual engine");
  require(std::hypot(result.map_body.x + .22, result.map_body.y + .29) < .04,
          "nearby initial pose must recover XY");
  require(std::abs(result.map_body.z - .04) < .03, "initial pose must recover height");
  require(std::abs(result.map_odom.qx) < 1e-5 && std::abs(result.map_odom.qy) < 1e-5,
          "gravity-aligned map and odometry must not acquire a tilt correction");
  require(result.quality <= .0144 && result.overlap_inlier_ratio >= .8,
          "initial alignment must retain the strict final quality thresholds");

  // A distant hint must not turn local initialization into an unbounded search.
  Pose3d distant_seed;
  distant_seed.x = 8.; distant_seed.y = 8.;
  require(!localizer.relocalize(scan, distant_seed, odom).success, "distant seed must fail");
  Pose3d outside_hint = pose(actual);
  outside_hint.x += 0.9;
  const auto outside_result = localizer.relocalize(scan, outside_hint, odom);
  require(!outside_result.success &&
              outside_result.message == "native_relocalizer_seed_outside_search",
          "a better solution outside the capture range must invalidate the hint");
  const auto exact = localizer.relocalize(scan, pose(actual), Pose3d{});
  require(exact.success, "explicit full attitude must remain usable");
  require(std::abs(exact.map_body.qx - pose(actual).qx) < .002 &&
              std::abs(exact.map_body.qy - pose(actual).qy) < .002,
          "explicit full attitude must not be replaced by odometry tilt");
#ifdef LINGTU_ENABLE_BBS3D
  // No poses.txt exists yet: this must exercise the gravity-aligned BBS path,
  // then refine the original body-frame scan through the production ICP gate.
  const auto global_recovery = localizer.globalRelocalize(scan, odom);
  print(global_recovery);
  require(global_recovery.success &&
              global_recovery.message == "native_global_relocalized",
          "global recovery must reach BBS and ICP without saved poses");
  require(global_recovery.engine == "bbs3d_gicp",
          "global recovery must report the BBS engine actually used");
  require(std::hypot(global_recovery.map_body.x - actual(0, 3),
                     global_recovery.map_body.y - actual(1, 3)) < .08,
          "global BBS recovery must restore the tilted body scan position");
#endif
  {
    std::ofstream index(directory / "poses.txt");
    const Pose3d saved = pose(actual);
    index << "scan_000000.pcd 8 8 0 1 0 0 0\n"
          << "scan_000001.pcd " << saved.x << ' ' << saved.y << ' ' << saved.z
          << ' ' << saved.qw << ' ' << saved.qx << ' ' << saved.qy << ' '
          << saved.qz << '\n';
  }
  NativeRelocalizer saved_pose_localizer;
  require(saved_pose_localizer.loadMap(path.string()), "saved-pose map load failed");
  const auto saved_recovery = saved_pose_localizer.globalRelocalize(scan, odom);
  require(saved_recovery.success &&
              saved_recovery.message == "native_relocalized_saved_pose_verified",
          "verified saved pose must recover before whole-map BBS3D");
  require(saved_recovery.engine == "saved_pose_icp",
          "saved-pose recovery must not claim that BBS ran");
  require(std::hypot(saved_recovery.map_body.x - actual(0, 3),
                      saved_recovery.map_body.y - actual(1, 3)) < .04,
          "saved-pose recovery must return the observed map position");
  // The map position can be unchanged while the quadruped's stance changes.
  // Saved endpoint attitude is historical; current gravity comes from odometry.
  M4F changed_stance = actual;
  changed_stance.block<3, 3>(0, 0) = (
      Eigen::AngleAxisf(.04F, Eigen::Vector3f::UnitZ()) *
      Eigen::AngleAxisf(.22F, Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(-.15F, Eigen::Vector3f::UnitX())).toRotationMatrix();
  M4F changed_odom = changed_stance;
  changed_odom.block<3, 1>(0, 3).setZero();
  const auto stance_recovery = saved_pose_localizer.globalRelocalize(
      scanFrom(map, changed_stance), pose(changed_odom));
  print(stance_recovery);
  require(stance_recovery.success && stance_recovery.engine == "saved_pose_icp",
          "a changed stance at a saved endpoint must not force a BBS search");
  require(std::abs(stance_recovery.map_odom.qx) < 1e-5 &&
              std::abs(stance_recovery.map_odom.qy) < 1e-5,
          "saved endpoint recovery must use current gravity instead of historical tilt");
#ifdef LINGTU_ENABLE_BBS3D
  // The LiDAR can map an area without the robot walking through it. A global
  // request must still find a pose outside the saved trajectory's 2 m margin.
  {
    std::ofstream index(directory / "poses.txt");
    index << "scan_000000.pcd 3 2 0 1 0 0 0\n";
  }
  NativeRelocalizer unvisited_localizer;
  require(unvisited_localizer.loadMap(path.string()), "unvisited map load failed");
  const auto unvisited_recovery = unvisited_localizer.globalRelocalize(scan, odom);
  print(unvisited_recovery);
  require(unvisited_recovery.success && unvisited_recovery.engine == "bbs3d_gicp",
          "global localization must search mapped space outside the saved trajectory");
  require(std::hypot(unvisited_recovery.map_body.x - actual(0, 3),
                     unvisited_recovery.map_body.y - actual(1, 3)) < .08,
          "unvisited mapped space must recover the correct global position");
#endif
  std::filesystem::remove(directory / "poses.txt");
  std::filesystem::remove(path);
  std::filesystem::remove(directory);
  return 0;
}

int main(int argc, char** argv) {
  try {
    return run(argc, argv);
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}

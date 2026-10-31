#include "ltlio/detail/Localizability.hpp"
#include "ltlio/frontend.hpp"
#include "ltlio_geometry_json.hpp"

#include <Eigen/Geometry>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
namespace lio = lingtu::ltlio;
using lio::detail::GeometryJacobian;
using lio::detail::analyzeLocalizability;

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

GeometryJacobian planes(const std::vector<int>& axes, double size = 1.0,
                        const Eigen::Matrix3d& body_rotation = Eigen::Matrix3d::Identity(),
                        const Eigen::Matrix3d& world_from_imu = Eigen::Matrix3d::Identity()) {
  GeometryJacobian rows(axes.size() * 49, 6);
  int row = 0;
  for (int axis : axes) {
    for (int a = -3; a <= 3; ++a) {
      for (int b = -3; b <= 3; ++b) {
        Eigen::Vector3d point = Eigen::Vector3d::Zero();
        point(axis) = 2.0;
        point((axis + 1) % 3) = a * 0.5;
        point((axis + 2) % 3) = b * 0.5;
        point = body_rotation * point * size;
        const Eigen::Vector3d normal = body_rotation.col(axis);
        rows.row(row).head<3>() = point.cross(normal).transpose();
        rows.row(row++).tail<3>() = (world_from_imu * normal).transpose();
      }
    }
  }
  return rows;
}

lio::GeometryDiagnostics analyze(const GeometryJacobian& rows,
    const Eigen::Matrix3d& rotation = Eigen::Matrix3d::Identity()) {
  return analyzeLocalizability(rows, std::vector<double>(rows.rows(), 1.0), rotation);
}

void checkGeometry() {
  const auto wall = analyze(planes({0}));
  require(wall.available && wall.degenerate_dof_count == 3,
          "one wall must leave two translations and its normal rotation unobservable");
  const auto corridor_rows = planes({1, 1, 2});
  const auto corridor = analyze(corridor_rows);
  require(corridor.available && corridor.degenerate_dof_count == 1,
          "corridor must have exactly one weak longitudinal direction");
  require(std::abs(corridor.principal_directions[0][3]) > 0.999999 &&
              corridor.categories[0] == "UNOBSERVABLE",
          "corridor weak mode must be the IMU x translation");
  require(!std::isfinite(corridor.condition_number) && !corridor.update_applied,
          "rank deficiency or diagnostic-only update flag was lost");
  const auto room = analyze(planes({0, 1, 2}));
  require(room.available && !room.detected && room.degenerate_dof_count == 0,
          "three orthogonal extended planes must constrain all six modes");

  const Eigen::Matrix3d tilted = (Eigen::AngleAxisd(0.61, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(-0.38, Eigen::Vector3d::UnitY())).toRotationMatrix();
  const auto rotated = analyze(planes({1, 1, 2}, 1.0, tilted));
  const Eigen::Vector3d weak(rotated.principal_directions[0][3],
      rotated.principal_directions[0][4], rotated.principal_directions[0][5]);
  require(rotated.degenerate_dof_count == 1 && std::abs(weak.dot(tilted.col(0))) > 0.999999,
          "rotating the corridor must rotate the weak translation in IMU axes");

  const auto world_rotated = analyze(planes({1, 1, 2}, 1.0,
      Eigen::Matrix3d::Identity(), tilted), tilted);
  for (int mode = 0; mode < 6; ++mode) {
    require(std::abs(world_rotated.eigenvalues[mode] - corridor.eigenvalues[mode]) < 1e-12 &&
                world_rotated.categories[mode] == corridor.categories[mode],
            "world coordinate rotation changed LiDAR localizability");
  }
  require(std::abs(world_rotated.principal_directions[0][3]) > 0.999999,
          "world-to-IMU Jacobian parameter conversion has the wrong direction");

  GeometryJacobian repeated(corridor_rows.rows() * 4, 6);
  for (int copy = 0; copy < 4; ++copy)
    repeated.middleRows(copy * corridor_rows.rows(), corridor_rows.rows()) = corridor_rows;
  const auto dense = analyze(repeated);
  const auto large = analyze(planes({1, 1, 2}, 12.0));
  for (int mode = 0; mode < 6; ++mode) {
    require(dense.categories[mode] == corridor.categories[mode] &&
                large.categories[mode] == corridor.categories[mode] &&
                std::abs(dense.eigenvalues[mode] - corridor.eigenvalues[mode]) < 1e-12 &&
                std::abs(large.eigenvalues[mode] - corridor.eigenvalues[mode]) < 1e-12,
            "repeated sampling or scene scale manufactured observability");
  }
  GeometryJacobian partial_rows = GeometryJacobian::Zero(520, 6);
  int row = 0;
  for (int axis = 0; axis < 6; ++axis)
    for (int i = 0; i < (axis == 5 ? 20 : 100); ++i) partial_rows(row++, axis) = 1.0;
  const auto partial = analyze(partial_rows);
  require(partial.available && partial.categories[0] == "PARTIAL" &&
              partial.degenerate_dof_count == 1,
          "small but nonzero directional support must be distinguished from no support");
  require(!analyze(GeometryJacobian(0, 6)).available &&
              !analyze(corridor_rows.topRows(9)).available,
          "empty or sparse geometry was reported as available");
  auto invalid = corridor_rows;
  invalid(0, 0) = std::numeric_limits<double>::quiet_NaN();
  require(!analyze(invalid).available &&
              !analyzeLocalizability(corridor_rows, {}, Eigen::Matrix3d::Identity()).available,
          "invalid geometry or missing weights manufactured diagnostics");
  std::cout << "wall/corridor/room, rotated frames, density/scale, partial and unavailable passed\n";
}

lio::TimedCloud roomCloud(double end) {
  lio::TimedCloud cloud;
  cloud.stamp_s = end - 0.02;
  cloud.end_stamp_s = end;
  for (int plane = 0; plane < 3; ++plane)
    for (int a = -6; a <= 6; ++a)
      for (int b = -6; b <= 6; ++b) {
        lio::TimedPoint point;
        if (plane == 0) { point.x = 1.1F; point.y = a * 0.1F; point.z = b * 0.1F; }
        else if (plane == 1) { point.x = a * 0.1F; point.y = 1.4F; point.z = b * 0.1F; }
        else { point.x = a * 0.1F; point.y = b * 0.1F; point.z = -0.8F; }
        cloud.points.push_back(point);
      }
  for (std::size_t i = 0; i < cloud.points.size(); ++i)
    cloud.points[i].offset_ns = i * 20'000'000ULL / (cloud.points.size() - 1);
  return cloud;
}

void checkFrontend() {
  lio::Config settings;
  settings.elevator_enabled = false;
  settings.automatic_door = false;
  settings.imu_init_samples = 20;
  settings.point_filter_num = 1;
  settings.blind_m = 0.1;
  settings.scan_resolution = 0.1;
  lio::Frontend frontend(settings);
  int next_imu = 0;
  auto observe = [&](lio::TimedCloud cloud) {
    while (next_imu * 0.005 <= cloud.end_stamp_s + 0.005) {
      lio::ImuSample sample;
      sample.stamp_s = next_imu++ * 0.005;
      sample.acceleration = {0.0, 0.0, 9.80665};
      frontend.feedImu(sample);
    }
    frontend.feedCloud(cloud);
    const auto result = frontend.processNext();
    require(result.has_value(), "covered frontend scan did not produce a result");
    return *result;
  };
  lio::Result last;
  for (int frame = 0; frame < 8; ++frame) {
    last = observe(roomCloud(0.22 + frame * 0.05));
    if (!last.attempted)
      require(!last.geometry.available, "map startup retained an older geometry diagnostic");
  }
  require(last.accepted && last.geometry.available && !last.geometry.detected,
          "real frontend room matching did not produce healthy geometry");
  auto sparse = roomCloud(0.62);
  sparse.points.resize(3);
  last = observe(sparse);
  require(!last.geometry.available && last.geometry.guidance == "awaiting_lidar_geometry",
          "sparse frontend scan retained healthy previous diagnostics");
  last = observe(roomCloud(0.67));
  require(last.accepted && last.geometry.available && !last.geometry.detected,
          "valid geometry did not recover after sparse input");
  std::cout << "actual Frontend startup/healthy/sparse/healthy passed\n";
}

void checkCollinearFrontend() {
  lio::Config settings;
  settings.elevator_enabled = false;
  settings.automatic_door = false;
  settings.imu_init_samples = 20;
  settings.point_filter_num = 1;
  settings.blind_m = 0.1;
  settings.scan_resolution = 0.01;
  lio::Frontend frontend(settings);
  int next_imu = 0;
  lio::Result last;
  for (int frame = 0; frame < 8; ++frame) {
    lio::TimedCloud cloud;
    cloud.end_stamp_s = 0.22 + frame * 0.05;
    cloud.stamp_s = cloud.end_stamp_s - 0.02;
    // Many valid returns on a single line cannot define any plane normal.
    for (int i = 0; i <= 200; ++i) {
      lio::TimedPoint point;
      point.x = (i - 100) * 0.02F;
      point.y = 0.5F;
      point.z = -0.8F;
      point.offset_ns = i * 100'000ULL;
      cloud.points.push_back(point);
    }
    while (next_imu * 0.005 <= cloud.end_stamp_s + 0.005) {
      lio::ImuSample sample;
      sample.stamp_s = next_imu++ * 0.005;
      sample.acceleration = {0.0, 0.0, 9.80665};
      frontend.feedImu(sample);
    }
    frontend.feedCloud(cloud);
    const auto result = frontend.processNext();
    require(result.has_value(), "covered collinear scan did not produce a result");
    last = *result;
  }
  std::cout << "collinear frontend: attempted=" << last.attempted
            << " effective=" << last.effective_points << '\n';
  require(last.attempted && last.effective_points == 0 && !last.geometry.available,
          "collinear map points manufactured a plane measurement");
}

void checkConfiguredPlaneNeighborhood() {
  auto match = [](int neighbors) {
    lio::Config settings;
    settings.elevator_enabled = false;
    settings.automatic_door = false;
    settings.imu_init_samples = 20;
    settings.point_filter_num = 1;
    settings.blind_m = 0.1;
    settings.scan_resolution = 0.005;
    settings.map_resolution = 0.005;
    settings.near_search_num = neighbors;
    lio::Frontend frontend(settings);
    int next_imu = 0;
    lio::Result last;
    for (int frame = 0; frame < 8; ++frame) {
      lio::TimedCloud cloud;
      cloud.end_stamp_s = 0.22 + frame * 0.05;
      cloud.stamp_s = cloud.end_stamp_s - 0.02;
      // The nearest five returns form one line, but nominate persistent cells
      // containing adjacent observed rows that resolve the floor plane.
      for (int row = -10; row <= 10; ++row)
        for (int column = -4; column <= 4; ++column) {
          lio::TimedPoint point;
          point.x = column * 0.01F;
          point.y = row * 0.15F;
          point.z = -0.8F;
          cloud.points.push_back(point);
        }
      for (std::size_t i = 0; i < cloud.points.size(); ++i)
        cloud.points[i].offset_ns = i * 20'000'000ULL / (cloud.points.size() - 1);
      while (next_imu * 0.005 <= cloud.end_stamp_s + 0.005) {
        lio::ImuSample sample;
        sample.stamp_s = next_imu++ * 0.005;
        sample.acceleration = {0.0, 0.0, 9.80665};
        frontend.feedImu(sample);
      }
      frontend.feedCloud(cloud);
      const auto result = frontend.processNext();
      require(result.has_value(), "covered configured-neighborhood scan produced no result");
      last = *result;
    }
    return last;
  };
  const auto narrow = match(5);
  const auto broad = match(20);
  std::cout << "configured plane neighborhood: five=" << narrow.effective_points
            << " twenty=" << broad.effective_points << '\n';
  require(narrow.accepted && narrow.effective_points > 100 && narrow.geometry.available,
          "five-neighbor nomination lost the cell's extended observed plane support");
  require(broad.accepted && broad.effective_points > 100 && broad.geometry.available,
          "twenty-neighbor nomination lost the cell's extended observed plane support");
}

void emitJson() {
  lingtu::slam::SlamOutputs out;
  std::cout << lingtu::slam::ltlioGeometryJson(out) << '\n';
  const auto geometry = analyze(planes({1, 1, 2}));
  out.ltlio_geometry_available = geometry.available;
  out.ltlio_degeneracy_detected = geometry.detected;
  out.ltlio_degenerate_dof_count = geometry.degenerate_dof_count;
  out.ltlio_condition_number = geometry.condition_number;
  out.ltlio_min_eigenvalue = geometry.min_eigenvalue;
  out.ltlio_max_eigenvalue = geometry.max_eigenvalue;
  out.ltlio_rotation_scale_m = geometry.rotation_scale_m;
  out.ltlio_geometry_eigenvalues = geometry.eigenvalues;
  out.ltlio_principal_directions = geometry.principal_directions;
  out.ltlio_geometry_categories = geometry.categories;
  out.ltlio_geometry_support_ratios = geometry.support_ratios;
  out.ltlio_geometry_guidance = geometry.guidance;
  const auto json = lingtu::slam::ltlioGeometryJson(out);
  require(json.find("\"condition_number\":null") != std::string::npos &&
              json.find("inf") == std::string::npos && json.find("nan") == std::string::npos,
          "rank deficient native status is not valid JSON");
  std::cout << json << '\n';
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc > 1 && std::string(argv[1]) == "--json") emitJson();
    else { checkGeometry(); checkFrontend(); checkCollinearFrontend();
      checkConfiguredPlaneNeighborhood(); emitJson(); }
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}

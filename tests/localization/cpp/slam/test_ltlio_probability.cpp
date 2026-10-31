#include "ltlio/detail/ProbabilityPlane.hpp"
#include "ltlio/detail/IkdMap.hpp"
#include "ltlio/detail/ESEKF.h"

#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace lingtu::ltlio::detail {
template <typename PointType>
struct IKD_MAP_TEST_ACCESS {
    using Map = IkdMap<PointType>;
    static bool pointCovariance(const Map& map, const PointType& point, Eigen::Matrix3d& covariance) {
        return map.pointCovariance(point, covariance);
    }
    static std::size_t covarianceCount(const Map& map) { return map.covarianceCount(); }
    static bool fitPlane(const Map& map, const typename Map::KDPointVector& points,
                         double max_rmse, ProbabilityPlane& plane) {
        return map.fitPlane(points, max_rmse, plane);
    }
    static std::size_t planeCacheRootCount(const Map& map) { return map.planeCacheRootCount(); }
    static void planeCandidates(Map& map, const PointType& query,
                                const typename Map::KDPointVector& nearest, double max_rmse,
                                std::vector<ProbabilityPlane>& planes) {
        map.planeCandidates(query, nearest, max_rmse, planes);
    }
};
}  // namespace lingtu::ltlio::detail

namespace {
namespace detail = lingtu::ltlio::detail;
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using Points = std::vector<Vec3>;
using Covariances = std::vector<Mat3>;
using Point = pcl::PointXYZINormal;
using Map = detail::IkdMap<Point>;
using MapTestAccess = detail::IKD_MAP_TEST_ACCESS<Point>;
using Cloud = pcl::PointCloud<Point>;
using Plane = detail::ProbabilityPlane;

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void requireNear(double actual, double expected, double tolerance, const char* message) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance) {
        std::cerr << message << ": actual=" << actual << " expected=" << expected << '\n';
        throw std::runtime_error(message);
    }
}

Eigen::Matrix3d rotation() {
    return (Eigen::AngleAxisd(0.41, Vec3::UnitZ()) *
            Eigen::AngleAxisd(-0.28, Vec3::UnitY()) *
            Eigen::AngleAxisd(0.19, Vec3::UnitX())).toRotationMatrix();
}

Points planePoints() {
    Points points;
    for (int x = -2; x <= 2; ++x)
        for (int y = -1; y <= 1; ++y)
            points.emplace_back(x * 0.8 + y * 0.07, y * 0.3, 1.7);
    return points;
}

Covariances pointCovariances(std::size_t count) {
    Covariances result;
    for (std::size_t i = 0; i < count; ++i) {
        const Mat3 axes = Eigen::AngleAxisd(0.13 * i, Vec3(1, 2, 3).normalized()).toRotationMatrix();
        const Vec3 variances(0.0004 + i * 0.00001, 0.0007, 0.0016);
        result.push_back(axes * variances.asDiagonal() * axes.transpose());
    }
    return result;
}

Plane fit(const Points& points, const Covariances& covariance) {
    Plane result;
    require(detail::fitProbabilityPlane(points, covariance, 0.05, result),
            "supported extended plane was rejected");
    return result;
}

void extendedPlaneHasFiniteDirectionalCovariance() {
    const Points points = planePoints();
    const Plane plane = fit(points, Covariances(points.size(), Mat3::Identity() * 0.0016));
    const Eigen::SelfAdjointEigenSolver<Mat6> spectrum(plane.covariance);
    require(plane.normal.allFinite() && plane.center.allFinite() && plane.covariance.allFinite() &&
                spectrum.info() == Eigen::Success && spectrum.eigenvalues().minCoeff() >= -1e-14,
            "extended plane uncertainty is not finite positive semidefinite");
    requireNear(std::abs(plane.normal.z()), 1.0, 1e-12, "plane normal has the wrong direction");
    require(plane.covariance(1, 1) > 4.0 * plane.covariance(0, 0),
            "shorter plane support failed to create greater normal tilt uncertainty");
    requireNear(plane.covariance(5, 5), 0.0016 / points.size(), 1e-14,
                "centroid covariance did not decrease with independent observations");
}

void planeCovarianceAgreesWithNumericallyPerturbedGeometry() {
    Points points = planePoints();
    const Mat3 r = rotation();
    for (auto& point : points) point = r * point + Vec3(0.3, -0.4, 0.8);
    const Covariances covariance = pointCovariances(points.size());
    const Plane base = fit(points, covariance);
    Mat6 numerical_covariance = Mat6::Zero();
    constexpr double epsilon = 1e-6;
    for (std::size_t i = 0; i < points.size(); ++i) {
        Eigen::Matrix<double, 6, 3> derivative;
        for (int axis = 0; axis < 3; ++axis) {
            Points plus = points;
            Points minus = points;
            plus[i](axis) += epsilon;
            minus[i](axis) -= epsilon;
            Plane positive = fit(plus, covariance);
            Plane negative = fit(minus, covariance);
            if (positive.normal.dot(base.normal) < 0.0) positive.normal *= -1.0;
            if (negative.normal.dot(base.normal) < 0.0) negative.normal *= -1.0;
            derivative.col(axis).head<3>() = (positive.normal - negative.normal) / (2 * epsilon);
            derivative.col(axis).tail<3>() = (positive.center - negative.center) / (2 * epsilon);
        }
        numerical_covariance.noalias() += derivative * covariance[i] * derivative.transpose();
    }
    const double relative_error = (base.covariance - numerical_covariance).norm() /
                                 numerical_covariance.norm();
    require(relative_error < 2e-6,
            "analytic normal/center covariance disagrees with finite differences");
    require(numerical_covariance.topRightCorner<3, 3>().norm() > 1e-7,
            "derivative fixture failed to exercise normal-center cross covariance");
}

Eigen::MatrixXd stateCovariance() {
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(21, 21);
    Mat6 factor = Mat6::Identity();
    factor(0, 4) = 0.4;
    factor(1, 3) = -0.3;
    factor(2, 5) = 0.2;
    factor(3, 2) = 0.1;
    covariance.topLeftCorner<6, 6>() = 0.001 * factor * factor.transpose();
    covariance.bottomRightCorner<15, 15>().diagonal().setConstant(0.02);
    return covariance;
}

void poseCovarianceAgreesWithRightRotationPointDerivative() {
    const Vec3 point(1.2, -0.7, 2.1);
    const Mat3 r = rotation();
    const Eigen::MatrixXd covariance = stateCovariance();
    Eigen::Matrix<double, 3, 6> derivative;
    constexpr double epsilon = 1e-6;
    for (int axis = 0; axis < 6; ++axis) {
        Vec3 positive;
        Vec3 negative;
        if (axis < 3) {
            positive = r * Eigen::AngleAxisd(epsilon, Vec3::Unit(axis)) * point;
            negative = r * Eigen::AngleAxisd(-epsilon, Vec3::Unit(axis)) * point;
        } else {
            positive = r * point + epsilon * Vec3::Unit(axis - 3);
            negative = r * point - epsilon * Vec3::Unit(axis - 3);
        }
        derivative.col(axis) = (positive - negative) / (2.0 * epsilon);
    }
    const Mat3 expected = derivative * covariance.topLeftCorner<6, 6>() * derivative.transpose();
    const Mat3 actual = detail::matchingPointPoseCovariance(point, r, covariance);
    require((actual - expected).norm() < 1e-11,
            "matching-point covariance lost pose cross terms or uses the wrong rotation perturbation");
}

void elevatorTransportUncertaintyDoesNotEnterMatchingCoordinates() {
    Eigen::MatrixXd covariance = stateCovariance();
    const Vec3 point(0.4, 1.1, -0.8);
    const Mat3 baseline = detail::matchingPointPoseCovariance(point, rotation(), covariance);
    // Index 18 is active elevator transport Z, deliberately distinct from P.z.
    covariance(18, 18) = 100.0;
    covariance(5, 18) = covariance(18, 5) = 0.1;
    const Mat3 transported = detail::matchingPointPoseCovariance(point, rotation(), covariance);
    require((baseline - transported).norm() < 1e-14,
            "active elevator height leaked into local matching-point covariance");
}

void rigidFrameChangePreservesResidualVariance() {
    const Points points = planePoints();
    const Covariances covariance = pointCovariances(points.size());
    const Plane original = fit(points, covariance);
    const Vec3 query(0.6, -0.13, 1.72);
    const Mat3 measurement = Mat3::Identity() * 0.001;
    const double expected = detail::planeResidualVariance(original, query, measurement);
    const Mat3 r = rotation();
    const Vec3 translation(2.0, -3.0, 8.0);
    Points transformed_points;
    Covariances transformed_covariance;
    for (std::size_t i = 0; i < points.size(); ++i) {
        transformed_points.push_back(r * points[i] + translation);
        transformed_covariance.push_back(r * covariance[i] * r.transpose());
    }
    const Plane transformed = fit(transformed_points, transformed_covariance);
    requireNear(detail::planeResidualVariance(transformed, r * query + translation,
                r * measurement * r.transpose()), expected, expected * 1e-10,
                "rigid coordinate change altered point-to-plane residual uncertainty");
}

void residualVarianceAgreesWithPerturbedSignedDistance() {
    const Points points = planePoints();
    const Plane plane = fit(points, pointCovariances(points.size()));
    const Vec3 query(0.4, -0.2, 1.72);
    const Mat3 measurement = Mat3::Identity() * 0.001;
    Eigen::Matrix<double, 9, 9> covariance = Eigen::Matrix<double, 9, 9>::Zero();
    covariance.topLeftCorner<6, 6>() = plane.covariance;
    covariance.bottomRightCorner<3, 3>() = measurement;
    Eigen::Matrix<double, 9, 1> variables;
    variables << plane.normal, plane.center, query;
    const auto distance = [](const Eigen::Matrix<double, 9, 1>& value) {
        return value.head<3>().dot(value.tail<3>() - value.segment<3>(3));
    };
    Eigen::Matrix<double, 1, 9> derivative;
    constexpr double epsilon = 1e-6;
    for (int axis = 0; axis < 9; ++axis) {
        auto positive = variables;
        auto negative = variables;
        positive(axis) += epsilon;
        negative(axis) -= epsilon;
        derivative(axis) = (distance(positive) - distance(negative)) / (2.0 * epsilon);
    }
    const double expected = (derivative * covariance * derivative.transpose()).value();
    requireNear(detail::planeResidualVariance(plane, query, measurement), expected,
                expected * 1e-8,
                "residual variance disagrees with finite-difference signed-distance propagation");
}

void noisierMeasurementHasLessInformationForSamePlane() {
    const Points points = planePoints();
    const Plane plane = fit(points, pointCovariances(points.size()));
    const Vec3 query(0.4, 0.1, 1.71);
    const Mat3 measurement = Mat3::Identity() * 0.001;
    const double precise = detail::scaledPlaneInformation(0.001,
        detail::planeResidualVariance(plane, query, measurement));
    const double noisy = detail::scaledPlaneInformation(0.001,
        detail::planeResidualVariance(plane, query, 9.0 * measurement));
    require(std::isfinite(noisy) && noisy > 0.0 && noisy < precise,
            "noisier return did not receive less information for identical point/plane geometry");
}

void noisierMapHasLessInformationForSamePlaneGeometry() {
    const Points points = planePoints();
    const Covariances covariance = pointCovariances(points.size());
    Covariances noisy_covariance = covariance;
    for (auto& value : noisy_covariance) value *= 9.0;
    const Plane precise_plane = fit(points, covariance);
    const Plane noisy_plane = fit(points, noisy_covariance);
    const Vec3 query(0.4, 0.1, 1.71);
    const Mat3 measurement = Mat3::Identity() * 0.001;
    const double precise = detail::scaledPlaneInformation(0.001,
        detail::planeResidualVariance(precise_plane, query, measurement));
    const double noisy = detail::scaledPlaneInformation(0.001,
        detail::planeResidualVariance(noisy_plane, query, measurement));
    require(noisy > 0.0 && noisy < precise,
            "noisier map points did not reduce plane information");
}

void scaledInformationReproducesPhysicalBayesianUpdate() {
    constexpr double prior_variance_m2 = 0.04;
    constexpr double residual_variance_m2 = 0.0025;
    constexpr double observation_m = 0.08;
    const double bayesian_gain = prior_variance_m2 / (prior_variance_m2 + residual_variance_m2);
    const double bayesian_variance = prior_variance_m2 * (1.0 - bayesian_gain);
    for (double scale : {0.001, 0.02, 1.0}) {
        const double information = detail::scaledPlaneInformation(scale, residual_variance_m2);
        const double scaled_posterior = 1.0 / (scale / prior_variance_m2 + information);
        const double correction = scaled_posterior * information * observation_m;
        requireNear(correction, bayesian_gain * observation_m, 1e-14,
                    "scaled native solve changed the physical Bayesian pose correction");
        requireNear(scale * scaled_posterior, bayesian_variance, 1e-14,
                    "scaled native solve changed physical posterior variance");
    }
}

void collinearReturnsCannotCreateAProbabilityPlane() {
    Points line;
    for (int i = -4; i <= 4; ++i) line.emplace_back(0.2 * i, 0.5, -0.8);
    Plane plane;
    require(!detail::fitProbabilityPlane(line, Covariances(line.size(), Mat3::Identity() * 0.001),
                                      0.05, plane),
            "collinear returns manufactured a probabilistic plane normal");
}

Cloud::Ptr mapCloud(double height = 1.0) {
    Cloud::Ptr cloud(new Cloud);
    for (int x = -3; x <= 3; ++x)
        for (int y = -2; y <= 2; ++y) {
            Point point;
            point.x = x * 0.3F;
            point.y = y * 0.3F;
            point.z = static_cast<float>(height);
            cloud->push_back(point);
        }
    return cloud;
}

Mat3 mapPointCovariance(const Point& point) {
    return Vec3(0.0004 + std::abs(point.x) * 0.0001,
                0.0007 + std::abs(point.y) * 0.0002,
                0.0016).asDiagonal();
}

Covariances mapCovariances(const Cloud& cloud) {
    Covariances covariance;
    for (const auto& point : cloud) covariance.push_back(mapPointCovariance(point));
    return covariance;
}

void requireSnapshotCovariances(Map& map, const Map::KDPointVector& snapshot) {
    for (const auto& point : snapshot) {
        Mat3 actual = Mat3::Zero();
        require(MapTestAccess::pointCovariance(map, point, actual), "live snapshot point lost its covariance association");
        require((actual - mapPointCovariance(point)).norm() < 1e-14,
                "snapshot covariance belongs to a different insertion point");
    }
}

void snapshotRetainsEachLivePointCovariance() {
    auto tree_owner = std::make_unique<Map::KDTreeType>();
    auto& tree = *tree_owner;
    Map map(Map::Config{}, &tree);
    map.init(0.01);
    const auto cloud = mapCloud();
    map.add(cloud, mapCovariances(*cloud));
    Map::KDPointVector snapshot;
    tree.Snapshot(snapshot);
    require(snapshot.size() == cloud->size(), "initial map snapshot lost fixture geometry");
    requireSnapshotCovariances(map, snapshot);
    Plane plane;
    require(MapTestAccess::fitPlane(map, snapshot, 0.05, plane) && plane.covariance.allFinite(),
            "live map metadata could not construct a finite probability plane");
}

void skippedDuplicateMeasurementPreservesTargetUncertainty() {
    auto tree_owner = std::make_unique<Map::KDTreeType>();
    auto& tree = *tree_owner;
    Map map(Map::Config{}, &tree);
    map.init(0.01);
    const auto cloud = mapCloud();
    const Covariances original = mapCovariances(*cloud);
    map.add(cloud, original);
    Covariances noisy = original;
    for (auto& value : noisy) value *= 100.0;
    map.add(cloud, noisy);
    Map::KDPointVector snapshot;
    tree.Snapshot(snapshot);
    require(snapshot.size() == cloud->size(), "duplicate returns changed active map geometry");
    requireSnapshotCovariances(map, snapshot);
}

void requireEstimatorReproducesBayesianPlaneUpdate(const char* mode) {
    for (double measurement_variance : {0.001, 0.02}) {
        lingtu::ltlio::Config config;
        config.elevator_enabled = false;
        config.automatic_door = false;
        config.map_resolution = 0.01;
        config.lidar_cov_inv = 1.0 / measurement_variance;
        auto context_owner = std::make_unique<detail::Context>(config);
        auto& context = *context_owner;
        context.lidar_update_mode = mode;
        context.lidar_ieskf_max_iterations = 1;
        detail::EskfEstimator estimator(context);
        auto state = estimator.get_cur_state();
        state.P = Eigen::MatrixXd::Identity(21, 21) * 1e-6;
        state.P(5, 5) = 0.04;
        estimator.set_cur_state(state);
        const auto target = mapCloud(1.0);
        context.ikd_map->add(target);
        Cloud query(*mapCloud(1.02));
        const double precision_sum = static_cast<double>(query.size()) / measurement_variance;
        const double residual = static_cast<double>(query.front().z) - 1.0;
        const double expected_posterior = 1.0 / (1.0 / 0.04 + precision_sum);
        const double expected_correction = -residual * precision_sum * expected_posterior;
        estimator.UpdateByLidarIESKF(query);
        require(estimator.lastUpdateAccepted() && estimator.validPointCount() == static_cast<int>(query.size()),
                "symmetric flat-plane fixture failed to produce a complete accepted estimator update");
        requireNear(estimator.get_cur_state().p.z(), expected_correction, 1e-10,
                    "actual estimator correction differs from physical scalar Bayesian update");
        requireNear(estimator.get_cur_state().P(5, 5), expected_posterior, 1e-10,
                    "actual estimator posterior variance differs from physical scalar Bayesian update");
    }
}

void pose6EstimatorUsesPhysicalPlaneInformation() {
    requireEstimatorReproducesBayesianPlaneUpdate("pose6");
}

void full21EstimatorUsesPhysicalPlaneInformation() {
    requireEstimatorReproducesBayesianPlaneUpdate("full21");
}

void resetAndRebuildReplaceGeometryAndCovarianceTogether() {
    auto tree_owner = std::make_unique<Map::KDTreeType>();
    auto& tree = *tree_owner;
    Map map(Map::Config{}, &tree);
    map.init(0.01);
    const auto old_cloud = mapCloud(1.0);
    map.add(old_cloud, mapCovariances(*old_cloud));
    map.reset();
    Mat3 covariance;
    require(MapTestAccess::covarianceCount(map) == 0 && !MapTestAccess::pointCovariance(map, old_cloud->front(), covariance),
            "map reset left retired covariance behind");
    const auto rebuilt_cloud = mapCloud(9.0);
    map.add(rebuilt_cloud, mapCovariances(*rebuilt_cloud));
    Map::KDPointVector snapshot;
    tree.Snapshot(snapshot);
    require(snapshot.size() == rebuilt_cloud->size(), "rebuilt map retained old geometry");
    for (const auto& point : snapshot)
        requireNear(point.z, 9.0, 1e-8, "rebuild snapshot contains pre-exit geometry");
    requireSnapshotCovariances(map, snapshot);
}

void budgetPruningPreservesUncertaintyOfRetainedGeometry() {
    auto tree_owner = std::make_unique<Map::KDTreeType>();
    auto& tree = *tree_owner;
    Map::Config config;
    config.max_map_points = 12;
    Map map(config, &tree);
    map.init(0.01);
    const auto cloud = mapCloud();
    map.add(cloud, mapCovariances(*cloud));
    Map::KDPointVector snapshot;
    tree.Snapshot(snapshot);
    require(snapshot.size() == config.max_map_points, "point budget failed to bound the active map");
    require(MapTestAccess::covarianceCount(map) == snapshot.size(), "point-budget pruning left retired uncertainty behind");
    requireSnapshotCovariances(map, snapshot);
}

void localCubePruningPreservesUncertaintyOfRetainedGeometry() {
    auto tree_owner = std::make_unique<Map::KDTreeType>();
    auto& tree = *tree_owner;
    Map::Config config;
    config.local_map_cube_len = 10.0;
    config.local_map_detect_range = 1.0;
    config.local_map_move_threshold = 1.5;
    Map map(config, &tree);
    map.init(0.01);
    const auto cloud = mapCloud();
    for (auto& point : *cloud) point.x -= 4.0F;
    const auto retained = mapCloud();
    cloud->points.insert(cloud->points.end(), retained->points.begin(), retained->points.end());
    cloud->width = static_cast<std::uint32_t>(cloud->points.size());
    map.add(cloud, mapCovariances(*cloud));
    map.removeFar(Vec3::Zero());
    map.removeFar(Vec3(4.5, 0.0, 0.0));
    Map::KDPointVector snapshot;
    tree.Snapshot(snapshot);
    require(!snapshot.empty() && snapshot.size() < cloud->size(),
            "local-cube fixture did not exercise spatial map deletion");
    require(MapTestAccess::covarianceCount(map) == snapshot.size(), "local-cube deletion left retired uncertainty behind");
    requireSnapshotCovariances(map, snapshot);
}

void elevatorExitMapTranslationPreservesResidualUncertainty() {
    auto tree_owner = std::make_unique<Map::KDTreeType>();
    auto& tree = *tree_owner;
    Map map(Map::Config{}, &tree);
    map.init(0.01);
    const auto local_cloud = mapCloud(1.0);
    const Covariances covariance = mapCovariances(*local_cloud);
    map.add(local_cloud, covariance);
    Map::KDPointVector snapshot;
    tree.Snapshot(snapshot);
    Plane before;
    require(MapTestAccess::fitPlane(map, snapshot, 0.05, before), "local elevator map plane was unavailable");
    const Vec3 local_query(0.12, -0.15, 1.02);
    const Mat3 measurement = Mat3::Identity() * 0.001;
    const double local_variance = detail::planeResidualVariance(before, local_query, measurement);

    // Transport is committed as a rigid world-Z translation at elevator exit.
    const Vec3 transport(0.0, 0.0, 8.0);
    Cloud::Ptr world_cloud(new Cloud(*local_cloud));
    for (auto& point : *world_cloud) point.z += static_cast<float>(transport.z());
    map.reset();
    map.add(world_cloud, covariance);
    snapshot.clear();
    tree.Snapshot(snapshot);
    Plane after;
    require(MapTestAccess::fitPlane(map, snapshot, 0.05, after), "world elevator map plane was unavailable");
    requireNear(detail::planeResidualVariance(after, local_query + transport, measurement),
                local_variance, local_variance * 1e-10,
                "committing elevator transport altered residual covariance");
}

Cloud::Ptr cellPatch(float origin_x = 0.2F, float height = 0.3F) {
    Cloud::Ptr cloud(new Cloud);
    for (int x = 0; x < 4; ++x)
        for (int y = 0; y < 4; ++y) {
            Point point;
            point.x = origin_x + 0.15F * x;
            point.y = 0.2F + 0.15F * y;
            point.z = height;
            cloud->push_back(point);
        }
    return cloud;
}

Point queryPoint(const Vec3& query) {
    Point point;
    point.x = static_cast<float>(query.x());
    point.y = static_cast<float>(query.y());
    point.z = static_cast<float>(query.z());
    return point;
}

std::vector<Plane> persistentCandidates(Map& map, Map::KDTreeType& tree, const Vec3& query) {
    Map::KDPointVector nearest;
    std::vector<float> distance;
    const Point point = queryPoint(query);
    tree.Nearest_Search(point, 5, nearest, distance);
    std::vector<Plane> candidates;
    MapTestAccess::planeCandidates(map, point, nearest, 0.05, candidates);
    return candidates;
}

Plane mostSupportedPlane(const std::vector<Plane>& candidates) {
    require(!candidates.empty(), "supported persistent cell did not return a plane candidate");
    return *std::max_element(candidates.begin(), candidates.end(),
        [](const Plane& left, const Plane& right) { return left.support_points < right.support_points; });
}

void persistentPlaneUsesMoreThanFiveActualMapPoints() {
    auto tree_owner = std::make_unique<Map::KDTreeType>();
    auto& tree = *tree_owner;
    Map map(Map::Config{}, &tree);
    map.init(0.01);
    const auto cloud = cellPatch();
    constexpr double variance = 0.0016;
    map.add(cloud, Covariances(cloud->size(), Mat3::Identity() * variance));
    const Plane plane = mostSupportedPlane(persistentCandidates(map, tree, Vec3(0.4, 0.4, 0.31)));
    require(plane.support_points == cloud->size() && plane.support_points > 5,
            "persistent plane fitted only nearest nominations instead of live cell support");
    requireNear(plane.covariance(5, 5), variance / cloud->size(), 1e-14,
                "claimed persistent support did not contribute to centroid uncertainty");
}

void cleanCellQueriesKeepCacheAndPlaneStable() {
    auto tree_owner = std::make_unique<Map::KDTreeType>();
    auto& tree = *tree_owner;
    Map map(Map::Config{}, &tree);
    map.init(0.01);
    const auto cloud = cellPatch();
    map.add(cloud, mapCovariances(*cloud));
    const Plane first = mostSupportedPlane(persistentCandidates(map, tree, Vec3(0.4, 0.4, 0.31)));
    require(MapTestAccess::planeCacheRootCount(map) == 1, "clean-cell fixture did not create one actual cached root");
    for (const Vec3& query : {Vec3(0.41, 0.42, 0.31), Vec3(0.43, 0.44, 0.32), Vec3(0.4, 0.4, 0.31)}) {
        const Plane again = mostSupportedPlane(persistentCandidates(map, tree, query));
        require(MapTestAccess::planeCacheRootCount(map) == 1 && again.support_points == first.support_points &&
                    (again.center - first.center).norm() < 1e-14 &&
                    (again.normal - first.normal).norm() < 1e-14 &&
                    (again.covariance - first.covariance).norm() < 1e-14,
                "clean-cell queries changed persistent geometry, uncertainty or cache size");
    }
}

void downsampleReplacementRefreshesCachedGeometryAndCovariance() {
    auto tree_owner = std::make_unique<Map::KDTreeType>();
    auto& tree = *tree_owner;
    Map map(Map::Config{}, &tree);
    map.init(0.2);
    const auto representatives = [](float start, float height) {
        Cloud::Ptr cloud(new Cloud);
        for (int x = 0; x < 5; ++x)
            for (int y = 0; y < 5; ++y) {
                Point point;
                point.x = start + 0.2F * x;
                point.y = start + 0.2F * y;
                point.z = height;
                cloud->push_back(point);
            }
        return cloud;
    };
    const auto old_cloud = representatives(0.06F, 0.06F);
    map.add(old_cloud, Covariances(old_cloud->size(), Mat3::Identity() * 0.0004));
    const Plane before = mostSupportedPlane(persistentCandidates(map, tree, Vec3(0.5, 0.5, 0.08)));
    const auto new_cloud = representatives(0.1F, 0.1F);
    map.add(new_cloud, Covariances(new_cloud->size(), Mat3::Identity() * 0.0036));
    Map::KDPointVector snapshot;
    tree.Snapshot(snapshot);
    require(snapshot.size() == new_cloud->size(), "replacement fixture did not replace each KD voxel representative");
    for (const auto& point : snapshot)
        requireNear(point.z, static_cast<double>(new_cloud->front().z), 1e-8,
                    "closer KD representative did not retire old plane geometry");
    const Plane after = mostSupportedPlane(persistentCandidates(map, tree, Vec3(0.5, 0.5, 0.11)));
    requireNear(after.center.z(), static_cast<double>(new_cloud->front().z), 1e-8,
                "cached plane retained replaced KD geometry");
    requireNear(after.covariance(5, 5) / before.covariance(5, 5), 9.0, 1e-10,
                "cached plane retained replaced target-point covariance");
}

void crossRootKdReplacementRetiresGhostPlane() {
    auto tree_owner = std::make_unique<Map::KDTreeType>();
    auto& tree = *tree_owner;
    Map map(Map::Config{}, &tree);
    map.init(0.6);
    Cloud::Ptr cloud(new Cloud);
    for (float x : {0.65F, 0.7F, 0.75F})
        for (float y : {0.1F, 0.3F, 0.5F}) cloud->push_back(queryPoint(Vec3(x, y, 0.65)));
    map.add(cloud, Covariances(cloud->size(), Mat3::Identity() * 0.0016));
    require(!persistentCandidates(map, tree, Vec3(0.7, 0.3, 0.66)).empty(),
            "cross-root fixture did not cache its original supported plane");
    Cloud::Ptr replacement(new Cloud);
    replacement->push_back(queryPoint(Vec3(1.01, 0.3, 0.9)));
    map.add(replacement, Covariances(1, Mat3::Identity() * 0.0036));
    Map::KDPointVector snapshot;
    tree.Snapshot(snapshot);
    require(snapshot.size() == 1 && snapshot.front().x > 1.0F,
            "cross-root fixture failed to exercise full KD voxel replacement");
    require(persistentCandidates(map, tree, Vec3(0.7, 0.3, 0.66)).empty(),
            "cross-root replacement left a cached plane supported only by deleted points");
}

void neighborNominationAssociatesAcrossPositiveAndNegativeRootBoundaries() {
    for (double direction : {1.0, -1.0}) {
        auto tree_owner = std::make_unique<Map::KDTreeType>();
        auto& tree = *tree_owner;
        Map map(Map::Config{}, &tree);
        map.init(0.01);
        Cloud::Ptr cloud(new Cloud);
        for (double x : {0.82, 0.87, 0.92, 0.97})
            for (double y : {0.25, 0.35, 0.45}) cloud->push_back(queryPoint(Vec3(direction * x, y, 0.5)));
        map.add(cloud, Covariances(cloud->size(), Mat3::Identity() * 0.0016));
        const Plane plane = mostSupportedPlane(persistentCandidates(map, tree, Vec3(direction * 1.01, 0.35, 0.51)));
        require(plane.support_points == cloud->size(),
                "root-boundary query failed to receive the actual neighbor-nominated support");
        requireNear(plane.center.z(), 0.5, 1e-12, "root boundary changed the observed plane height");
    }
}

void adaptiveCellsKeepParallelSurfacesDistinct() {
    auto tree_owner = std::make_unique<Map::KDTreeType>();
    auto& tree = *tree_owner;
    Map map(Map::Config{}, &tree);
    map.init(0.01);
    Cloud::Ptr cloud(new Cloud);
    for (double z : {0.2, 0.8})
        for (double x : {0.2, 0.3, 0.4})
            for (double y : {0.2, 0.3, 0.4}) cloud->push_back(queryPoint(Vec3(x, y, z)));
    map.add(cloud, Covariances(cloud->size(), Mat3::Identity() * 0.0016));
    for (double height : {0.2, 0.8}) {
        const auto candidates = persistentCandidates(map, tree, Vec3(0.3, 0.3, height + 0.01));
        bool observed_surface = false;
        for (const auto& plane : candidates)
            if (std::abs(plane.center.z() - height) < 1e-6 && std::abs(plane.normal.z()) > 0.999999 &&
                    plane.support_points == 9) observed_surface = true;
        require(observed_surface, "adaptive cell merged or discarded distinct observed parallel surfaces");
    }
}

void mapResetClearsCachedPlanesBeforeRebuild() {
    auto tree_owner = std::make_unique<Map::KDTreeType>();
    auto& tree = *tree_owner;
    Map map(Map::Config{}, &tree);
    map.init(0.01);
    const auto initial = cellPatch(0.2F, 0.2F);
    map.add(initial, mapCovariances(*initial));
    mostSupportedPlane(persistentCandidates(map, tree, Vec3(0.4, 0.4, 0.21)));
    require(MapTestAccess::planeCacheRootCount(map) == 1, "reset fixture did not populate an actual cached root");
    map.reset();
    require(MapTestAccess::planeCacheRootCount(map) == 0, "map reset retained cached probability-plane roots");
    const auto rebuilt = cellPatch(0.2F, 0.8F);
    map.add(rebuilt, mapCovariances(*rebuilt));
    const Plane plane = mostSupportedPlane(persistentCandidates(map, tree, Vec3(0.4, 0.4, 0.81)));
    requireNear(plane.center.z(), static_cast<double>(rebuilt->front().z), 1e-8,
                "cache rebuild reused the retired plane height");
}

void pointBudgetPruningBoundsCachedRootsToLiveGeometry() {
    auto tree_owner = std::make_unique<Map::KDTreeType>();
    auto& tree = *tree_owner;
    Map::Config config;
    config.max_map_points = 16;
    Map map(config, &tree);
    map.init(0.01);
    const auto original = cellPatch();
    map.add(original, mapCovariances(*original));
    mostSupportedPlane(persistentCandidates(map, tree, Vec3(0.4, 0.4, 0.31)));
    // First removeFar establishes the cube and updates the point-budget origin.
    map.removeFar(Vec3(10.0, 0.0, 0.0));
    const auto current = cellPatch(10.2F);
    map.add(current, mapCovariances(*current));
    Map::KDPointVector snapshot;
    tree.Snapshot(snapshot);
    require(snapshot.size() == config.max_map_points, "cache-budget fixture lost its active point budget");
    for (const auto& point : snapshot) require(point.x > 10.0F, "point budget failed to retire the original cached root");
    const Plane plane = mostSupportedPlane(persistentCandidates(map, tree, Vec3(10.4, 0.4, 0.31)));
    require(MapTestAccess::planeCacheRootCount(map) == 1 && plane.center.x() > 10.0,
            "point-budget pruning retained retired root cache state");
}

void cubePruningBoundsCachedRootsToLiveGeometry() {
    auto tree_owner = std::make_unique<Map::KDTreeType>();
    auto& tree = *tree_owner;
    Map::Config config;
    config.local_map_cube_len = 10.0;
    config.local_map_detect_range = 1.0;
    config.local_map_move_threshold = 1.5;
    Map map(config, &tree);
    map.init(0.01);
    const auto removed = cellPatch(-4.8F);
    const auto retained = cellPatch();
    Cloud::Ptr cloud(new Cloud(*removed));
    for (const auto& point : *retained) cloud->push_back(point);
    map.add(cloud, mapCovariances(*cloud));
    mostSupportedPlane(persistentCandidates(map, tree, Vec3(-4.6, 0.4, 0.31)));
    mostSupportedPlane(persistentCandidates(map, tree, Vec3(0.4, 0.4, 0.31)));
    require(MapTestAccess::planeCacheRootCount(map) == 2, "cube-pruning fixture did not cache both observed roots");
    map.removeFar(Vec3::Zero());
    map.removeFar(Vec3(4.5, 0.0, 0.0));
    Map::KDPointVector snapshot;
    tree.Snapshot(snapshot);
    require(snapshot.size() == retained->size(), "cube-pruning fixture did not retire its distant root");
    const Plane plane = mostSupportedPlane(persistentCandidates(map, tree, Vec3(0.4, 0.4, 0.31)));
    require(MapTestAccess::planeCacheRootCount(map) == 1 && plane.center.x() > 0.0,
            "local-cube pruning retained retired root cache state");
}

void requireSamePlane(const Plane& actual, const Plane& expected) {
    require((actual.normal.array() == expected.normal.array()).all() &&
                (actual.center.array() == expected.center.array()).all() &&
                (actual.covariance.array() == expected.covariance.array()).all() &&
                actual.radius == expected.radius && actual.support_points == expected.support_points,
            "batch changed an ordered plane's geometry, covariance or support");
}

Cloud::Ptr batchMapCloud() {
    Cloud::Ptr cloud(new Cloud);
    for (double z : {0.2, 0.8})
        for (double x : {0.2, 0.3, 0.4})
            for (double y : {0.2, 0.3, 0.4}) cloud->push_back(queryPoint(Vec3(x, y, z)));
    for (float origin : {2.2F, -1.8F}) {
        const auto patch = cellPatch(origin);
        for (const auto& point : *patch) cloud->push_back(point);
    }
    return cloud;
}

struct BatchQueries {
    Map::KDPointVector points;
    std::vector<Map::KDPointVector> nearest;
    std::vector<std::uint8_t> eligible;
};

BatchQueries batchQueries(Map::KDTreeType& tree) {
    BatchQueries result;
    for (const Vec3& position : {Vec3(0.3, 0.3, 0.21), Vec3(0.31, 0.32, 0.79),
                                Vec3(2.4, 0.4, 0.31), Vec3(-1.6, 0.4, 0.31),
                                Vec3(7.2, 7.2, 7.2), Vec3(1.3, 0.3, 0.3),
                                Vec3(0.3, 0.3, 0.21)}) {
        result.points.push_back(queryPoint(position));
        result.nearest.emplace_back();
        std::vector<float> distances;
        tree.Nearest_Search(result.points.back(), 5, result.nearest.back(), distances);
        result.eligible.push_back(1);
    }
    // Exercise an eligible empty-root query without neighbor nominations and
    // a caller-rejected query whose valid nearest points must remain unused.
    result.nearest[4].clear();
    result.eligible[6] = 0;
    return result;
}

void batchPreservesSequentialCandidateOrderAndValues() {
    auto tree_owner = std::make_unique<Map::KDTreeType>();
    auto& tree = *tree_owner;
    Map map(Map::Config{}, &tree);
    map.init(0.01);
    const auto cloud = batchMapCloud();
    map.add(cloud, mapCovariances(*cloud));
    const auto queries = batchQueries(tree);
    const auto batch = map.planeCandidatesBatch(queries.points, queries.nearest, queries.eligible, 0.05);
    require(batch.candidate_indices.size() == queries.points.size(),
            "batch did not retain one candidate sequence per input query");
    require(batch.candidate_indices[0].size() == 2 && batch.candidate_indices[1].size() == 2,
            "batch fixture did not exercise repeated adaptive multi-child roots");
    require(batch.candidate_indices[4].empty() && batch.candidate_indices[6].empty(),
            "empty-root or ineligible query manufactured batch candidates");
    require(batch.candidate_indices[5].size() == 3,
            "batch fixture did not exercise ordered nominations of different populated roots");
    require(batch.planes.size() == 4,
            "batch did not share one owned plane sequence for repeated root nominations");
    for (std::size_t i = 0; i < queries.points.size(); ++i) {
        std::vector<Plane> sequential;
        if (queries.eligible[i]) MapTestAccess::planeCandidates(map, queries.points[i], queries.nearest[i], 0.05, sequential);
        require(batch.candidate_indices[i].size() == sequential.size(),
                "batch and sequential query returned different candidate counts");
        for (std::size_t j = 0; j < sequential.size(); ++j) {
            const auto index = batch.candidate_indices[i][j];
            require(index < batch.planes.size(), "batch candidate index does not address its owned plane storage");
            requireSamePlane(batch.planes[index], sequential[j]);
        }
    }
}

void ownedBatchRemainsValidAfterMapMutationAndReset() {
    auto tree_owner = std::make_unique<Map::KDTreeType>();
    auto& tree = *tree_owner;
    Map map(Map::Config{}, &tree);
    map.init(0.01);
    const auto cloud = batchMapCloud();
    map.add(cloud, mapCovariances(*cloud));
    const auto queries = batchQueries(tree);
    const auto batch = map.planeCandidatesBatch(queries.points, queries.nearest, queries.eligible, 0.05);
    const auto expected_planes = batch.planes;
    const auto expected_indices = batch.candidate_indices;
    require(!batch.planes.empty(), "batch lifetime fixture did not acquire owned plane values");

    const auto added = cellPatch(2.2F, 0.6F);
    map.add(added, mapCovariances(*added));
    persistentCandidates(map, tree, Vec3(2.4, 0.4, 0.61));
    map.reset();
    const auto rebuilt = cellPatch(6.2F, 0.9F);
    map.add(rebuilt, mapCovariances(*rebuilt));
    persistentCandidates(map, tree, Vec3(6.4, 0.4, 0.91));

    require(batch.candidate_indices == expected_indices && batch.planes.size() == expected_planes.size(),
            "map mutation or reset invalidated a previously returned batch");
    for (std::size_t i = 0; i < expected_planes.size(); ++i)
        requireSamePlane(batch.planes[i], expected_planes[i]);
}

}  // namespace

int main() {
    std::cout << std::unitbuf;
    struct Test { const char* name; void (*run)(); };
    const Test tests[] = {
        {"extended plane has finite directional covariance", extendedPlaneHasFiniteDirectionalCovariance},
        {"plane covariance agrees with perturbed geometry", planeCovarianceAgreesWithNumericallyPerturbedGeometry},
        {"pose covariance agrees with right-rotation derivative", poseCovarianceAgreesWithRightRotationPointDerivative},
        {"active elevator transport is absent from matching covariance", elevatorTransportUncertaintyDoesNotEnterMatchingCoordinates},
        {"rigid frame change preserves residual variance", rigidFrameChangePreservesResidualVariance},
        {"residual variance agrees with perturbed signed distance", residualVarianceAgreesWithPerturbedSignedDistance},
        {"noisier measurement has less information", noisierMeasurementHasLessInformationForSamePlane},
        {"noisier map has less information", noisierMapHasLessInformationForSamePlaneGeometry},
        {"scaled information reproduces physical Bayesian update", scaledInformationReproducesPhysicalBayesianUpdate},
        {"collinear returns cannot create a probability plane", collinearReturnsCannotCreateAProbabilityPlane},
        {"snapshot retains live point covariance", snapshotRetainsEachLivePointCovariance},
        {"skipped duplicate preserves target uncertainty", skippedDuplicateMeasurementPreservesTargetUncertainty},
        {"pose6 estimator uses physical plane information", pose6EstimatorUsesPhysicalPlaneInformation},
        {"full21 estimator uses physical plane information", full21EstimatorUsesPhysicalPlaneInformation},
        {"reset and rebuild replace geometry and covariance", resetAndRebuildReplaceGeometryAndCovarianceTogether},
        {"budget pruning retains geometry uncertainty", budgetPruningPreservesUncertaintyOfRetainedGeometry},
        {"local-cube pruning retains geometry uncertainty", localCubePruningPreservesUncertaintyOfRetainedGeometry},
        {"elevator-exit translation preserves residual uncertainty", elevatorExitMapTranslationPreservesResidualUncertainty},
        {"persistent plane uses more than five actual map points", persistentPlaneUsesMoreThanFiveActualMapPoints},
        {"clean-cell queries keep real cache and plane stable", cleanCellQueriesKeepCacheAndPlaneStable},
        {"KD replacement refreshes cached geometry and covariance", downsampleReplacementRefreshesCachedGeometryAndCovariance},
        {"cross-root KD replacement retires ghost plane", crossRootKdReplacementRetiresGhostPlane},
        {"neighbor nominations cross positive and negative root boundaries", neighborNominationAssociatesAcrossPositiveAndNegativeRootBoundaries},
        {"adaptive cells keep parallel surfaces distinct", adaptiveCellsKeepParallelSurfacesDistinct},
        {"reset clears cached planes before rebuild", mapResetClearsCachedPlanesBeforeRebuild},
        {"point-budget pruning bounds cached roots", pointBudgetPruningBoundsCachedRootsToLiveGeometry},
        {"local-cube pruning bounds cached roots", cubePruningBoundsCachedRootsToLiveGeometry},
        {"batch preserves sequential candidate order and values", batchPreservesSequentialCandidateOrderAndValues},
        {"owned batch survives map mutation and reset", ownedBatchRemainsValidAfterMapMutationAndReset},
    };
    int failed = 0;
    for (const auto& test : tests) {
        try {
            test.run();
            std::cout << "PASS " << test.name << '\n';
        } catch (const std::exception& error) {
            ++failed;
            std::cerr << "FAIL " << test.name << ": " << error.what() << '\n';
        }
    }
    const std::size_t count = sizeof(tests) / sizeof(tests[0]);
    std::cout << count - failed << '/' << count << " LTLIO probability tests passed\n";
    return failed == 0 ? 0 : 1;
}

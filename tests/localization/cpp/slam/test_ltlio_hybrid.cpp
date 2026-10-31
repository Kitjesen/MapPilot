#include "ltlio/detail/ESEKF.h"
#include "ltlio/detail/HybridCorrespondence.hpp"
#include "ltlio/frontend.hpp"

#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <stdexcept>

namespace lingtu::ltlio::detail {
template <typename PointType>
struct IKD_MAP_TEST_ACCESS {
    static std::size_t pointBucketCount(const IkdMap<PointType>& map) {
        std::lock_guard<std::mutex> lock(map.mutex_);
        const auto count = [](auto&& self, const auto& node) -> std::size_t {
            std::size_t result = node.point_bucket ? 1 : 0;
            for (const auto& child : node.children) if (child) result += self(self, *child);
            return result;
        };
        std::size_t result = 0;
        for (const auto& root : map.plane_roots_) result += count(count, root.second.node);
        return result;
    }
};
}  // namespace lingtu::ltlio::detail

namespace {
namespace d = lingtu::ltlio::detail;
using Vec = Eigen::Vector3d;
using Mat = Eigen::Matrix3d;
using Map = d::IkdMap<d::PointType>;

void require(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}
void near(double actual, double expected, double tolerance, const char* message) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance) {
        std::cerr << actual << " != " << expected << '\n';
        throw std::runtime_error(message);
    }
}
d::PointType point(double x, double y, double z) {
    d::PointType p{};
    p.x = static_cast<float>(x); p.y = static_cast<float>(y); p.z = static_cast<float>(z);
    return p;
}
d::PointCloudXYZIPtr line(double y = 0.2, double z = 0.3, int count = 12) {
    auto cloud = d::PointCloudXYZIPtr(new d::PointCloudXYZI);
    for (int i = 0; i < count; ++i) cloud->push_back(point(0.1 + 0.06 * i, y, z));
    return cloud;
}
Map::PlaneBatch snapshot(Map& map, const d::PointType& query) {
    Map::KDPointVector queries{query};
    return map.planeCandidatesBatch(queries, std::vector<Map::KDPointVector>(1),
                                   std::vector<std::uint8_t>{1}, 0.05, true);
}
d::HybridPointBucket bucket(const std::vector<Vec>& points) {
    d::HybridPointBucket result;
    result.minimum = result.maximum = points.front();
    for (const auto& p : points) {
        result.minimum = result.minimum.cwiseMin(p);
        result.maximum = result.maximum.cwiseMax(p);
        result.points.push_back({p, Mat::Identity() * 0.004});
    }
    return result;
}

void scalarJacobianMatchesFiniteDifferences() {
    const Mat r = (Eigen::AngleAxisd(0.4, Vec::UnitZ()) *
                   Eigen::AngleAxisd(-0.2, Vec::UnitY())).toRotationMatrix();
    const Vec body(1.3, -0.7, 0.8), translation(0.1, 0.4, -0.2);
    const d::HybridMapPoint target{Vec(1.0, 0.3, 0.5), Mat::Identity() * 0.002};
    Vec direction;
    double residual, variance;
    require(d::linearizeHybridPoint(r * body + translation, target, Mat::Identity() * 0.001,
                                    0.1, 0.05, direction, residual, variance), "valid point term rejected");
    Eigen::Matrix<double, 1, 6> jacobian;
    jacobian.leftCols<3>() = -direction.transpose() * r * d::probabilitySkew(body);
    jacobian.rightCols<3>() = direction.transpose();
    constexpr double step = 1e-6;
    for (int axis = 0; axis < 6; ++axis) {
        Vec delta = Vec::Unit(axis % 3) * step;
        Vec plus = r * body + translation + delta;
        Vec minus = r * body + translation - delta;
        if (axis < 3) {
            plus = r * Eigen::AngleAxisd(step, Vec::Unit(axis)).toRotationMatrix() * body + translation;
            minus = r * Eigen::AngleAxisd(-step, Vec::Unit(axis)).toRotationMatrix() * body + translation;
        }
        const double numerical = ((plus - target.position).norm() - (minus - target.position).norm()) / (2.0 * step);
        near(jacobian[axis], numerical, 2e-9, "scalar point residual Jacobian differs from numerical derivative");
    }
}

void relinearizationUsesNewDirectionAndVariance() {
    const d::HybridMapPoint target{Vec::Zero(), Vec(0.01, 0.04, 0.09).asDiagonal()};
    const Mat sensor = Vec(0.02, 0.03, 0.07).asDiagonal();
    Vec direction;
    double residual, variance;
    require(d::linearizeHybridPoint(Vec(0.1, 0, 0), target, sensor, 0.2, 0.05,
                                    direction, residual, variance), "first point linearization failed");
    near(variance, 0.05 * (0.01 + 0.02 + 0.2), 1e-15, "point measurement lost target or sensor covariance");
    require(d::linearizeHybridPoint(Vec(0, 0.2, 0), target, sensor, 0.2, 0.05,
                                    direction, residual, variance), "second point linearization failed");
    require((direction - Vec::UnitY()).norm() < 1e-15, "cached point direction did not follow new state");
    near(residual, 0.2, 1e-15, "cached point residual did not follow new state");
    near(variance, 0.05 * (0.04 + 0.03 + 0.2), 1e-15, "cached point variance did not follow new direction");
}

void zeroResidualIsRejectedWithoutNaN() {
    const d::HybridMapPoint target{Vec(1, 2, 3), Mat::Identity()};
    Vec direction = Vec::Zero();
    double residual = -1, variance = -1;
    require(!d::linearizeHybridPoint(target.position, target, Mat::Identity(), 0.1, 0.05,
                                     direction, residual, variance), "coincident match supplied a spurious direction");
    require(direction.allFinite() && std::isfinite(residual), "coincident match generated NaN");
}

void rangeMotionGatePreservesStartupFloor() {
    const auto identity = Eigen::Quaterniond::Identity();
    near(d::hybridPointGateSquared(Vec(10, 0, 0), identity, Vec::Zero(), 0.5),
         1.5625, 0.0, "startup gate collapsed for stationary state");
    const Eigen::Quaterniond correction(Eigen::AngleAxisd(0.1, Vec::UnitZ()));
    const double close = d::hybridPointGateSquared(Vec(2, 0, 0), correction, Vec(0.3, 0, 0), 0.5);
    const double far = d::hybridPointGateSquared(Vec(10, 0, 0), correction, Vec(0.3, 0, 0), 0.5);
    require(far > close, "rotation correction gate does not reflect query range");
    near(d::hybridPointGateSquared(Vec(1000, 0, 0), correction, Vec::Zero(), 0.5),
         d::hybridPointGateSquared(Vec(100, 0, 0), correction, Vec::Zero(), 0.5),
         1e-12, "range-motion gate is unbounded at long range");
}

void pruningAgreesWithExhaustiveAndVisitsFewerPoints() {
    const std::vector<d::HybridPointBucket> buckets{
        bucket({Vec(0.02, 0.01, 0), Vec(0.03, 0, 0)}),
        bucket({Vec(0.7, 0.8, 0.9), Vec(0.9, 0.8, 0.7)}),
        bucket({Vec(-0.3, 0.2, 0), Vec(-0.2, 0.3, 0)})};
    for (const Vec& query : {Vec::Zero().eval(), Vec(0.8, 0.8, 0.8), Vec(-0.25, 0.25, 0)}) {
        const auto exact = d::findHybridPointMatch(query, buckets, {0, 1, 2}, 2.0, false);
        const auto pruned = d::findHybridPointMatch(query, buckets, {0, 1, 2}, 2.0, true);
        require(exact.found && pruned.found && exact.target.position == pruned.target.position,
                "AABB pruning changed nearest eligible map point");
        require(exact.candidate_points == pruned.candidate_points,
                "pruning changed discretization support and measurement model");
        require(pruned.evaluated_points < exact.evaluated_points, "AABB pruning did not skip any impossible bucket");
    }
}

void pruningRetainsDeterministicTieAndRadiusGate() {
    const std::vector<d::HybridPointBucket> buckets{
        bucket({Vec(-1, 0, 0)}), bucket({Vec(1, 0, 0)}), bucket({Vec(2, 0, 0)})};
    for (const bool prune : {false, true}) {
        const auto tied = d::findHybridPointMatch(Vec::Zero(), buckets, {0, 1, 2}, 2.0, prune);
        require(tied.found && tied.target.position == Vec(-1, 0, 0), "nearest tie changed with search traversal");
        const auto rejected = d::findHybridPointMatch(Vec::Zero(), buckets, {0, 1, 2}, 1.0, prune);
        require(!rejected.found, "point at radius boundary bypassed strict correspondence gate");
    }
}

void immatureCellsRetainPointsAndCovariance() {
    auto map = std::make_unique<Map>();
    map->init(0.005);
    auto cloud = line(0.2, 0.3, 3);
    map->add(cloud, std::vector<Mat>(3, Mat::Identity() * 0.007));
    const auto batch = snapshot(*map, cloud->front());
    require(batch.planes.empty() && batch.point_buckets.size() == 1 &&
            batch.point_buckets.front().points.size() == 3, "immature cell discarded its usable point candidates");
    near(batch.point_buckets.front().points.front().covariance(0, 0), 0.007, 0.0,
         "point bucket lost propagated map covariance");
}

void planeOnlyCacheDoesNotAllocatePointPayloads() {
    auto map = std::make_unique<Map>(); map->init(0.005);
    map->add(line());
    Map::KDPointVector queries{point(0.2, 0.2, 0.3)};
    const auto plane_only = map->planeCandidatesBatch(queries,
        std::vector<Map::KDPointVector>(1), std::vector<std::uint8_t>{1}, 0.05);
    require(plane_only.point_buckets.empty() &&
            d::IKD_MAP_TEST_ACCESS<d::PointType>::pointBucketCount(*map) == 0,
            "disabled hybrid allocated point covariance payloads");
    const auto hybrid = snapshot(*map, queries.front());
    require(!hybrid.point_buckets.empty() &&
            d::IKD_MAP_TEST_ACCESS<d::PointType>::pointBucketCount(*map) > 0,
            "first hybrid request did not populate a previously plane-only cache");
    map->add(line(0.5));
    map->planeCandidatesBatch(queries, std::vector<Map::KDPointVector>(1),
                             std::vector<std::uint8_t>{1}, 0.05);
    require(d::IKD_MAP_TEST_ACCESS<d::PointType>::pointBucketCount(*map) == 0,
            "dirty planar refresh retained stale point payloads");
}

void dirtyCellCanMatureIntoPlaneWithoutDuplicatePointEvidence() {
    auto map = std::make_unique<Map>(); map->init(0.005);
    map->add(line());
    const auto before = snapshot(*map, point(0.2, 0.2, 0.3));
    require(before.planes.empty() && !before.point_buckets.empty(), "collinear cell incorrectly supplied a plane");
    map->add(line(0.5));
    const auto after = snapshot(*map, point(0.2, 0.2, 0.3));
    require(!after.planes.empty() && after.point_buckets.empty(), "mature planar cell retained duplicate point evidence");
    require(!before.point_buckets.empty(), "map update mutated a prior batch snapshot");
}

void resetDropsStalePointBuckets() {
    auto map = std::make_unique<Map>(); map->init(0.005);
    map->add(line());
    const auto before = snapshot(*map, point(0.2, 0.2, 0.3));
    require(!before.point_buckets.empty(), "nonplanar cache fixture is empty");
    map->reset();
    auto moved = line(); for (auto& p : *moved) p.x += 3;
    map->add(moved);
    const auto after = snapshot(*map, point(0.2, 0.2, 0.3));
    require(after.point_buckets.empty() && after.candidate_roots[0] == 0,
            "reset retained stale nonplanar candidates or their uncertainty");
}

struct UpdateResult { Vec position; Eigen::MatrixXd covariance; int planes; int points; bool accepted; };
UpdateResult updateLine(bool hybrid, bool prune, bool exact = false) {
    lingtu::ltlio::Config cfg;
    cfg.elevator_enabled = false; cfg.automatic_door = false;
    cfg.map_resolution = 0.005; cfg.ieskf_max_iter = 4;
    cfg.hybrid_enabled = hybrid; cfg.correspondence_pruning_enabled = prune;
    auto ctx = std::make_unique<d::Context>(cfg);
    // Keep identity until the final iteration so cached point terms must be
    // relinearized as the posterior moves, rather than retaining the first r/u/R.
    ctx->lidar_ieskf_rematch_pos_m = ctx->lidar_ieskf_rematch_rot_rad = 1e6;
    d::EskfEstimator estimator(*ctx);
    auto state = estimator.get_cur_state();
    state.P = Eigen::MatrixXd::Identity(21, 21) * 0.001;
    state.P(5, 5) = 0.04;
    estimator.set_cur_state(state);
    ctx->ikd_map->add(line());
    auto query = line(0.2, exact ? 0.3 : 0.32);
    estimator.UpdateByLidarIESKF(*query);
    const auto result = estimator.get_cur_state();
    return {result.p, result.P, estimator.lastPlaneMatchCount(), estimator.lastPointMatchCount(),
            estimator.lastUpdateAccepted()};
}

void actualEstimatorUsesNonplanarPointsAndEquivalentPruning() {
    const auto baseline = updateLine(false, false);
    const auto exact = updateLine(true, false);
    const auto pruned = updateLine(true, true);
    require(!baseline.accepted && baseline.points == 0, "plane-only baseline fabricated a plane for a line");
    require(exact.accepted && exact.planes == 0 && exact.points > 0 &&
            exact.position.z() < -0.001 && exact.position.z() > -0.03,
            "hybrid measurement did not correct the observed line offset");
    require(pruned.accepted && pruned.points == exact.points && pruned.planes == exact.planes &&
            (pruned.position - exact.position).norm() < 1e-14 &&
            (pruned.covariance - exact.covariance).norm() < 1e-14,
            "pruned actual estimator changed its posterior measurement model");
}

void actualEstimatorRejectsCoincidentPointDirection() {
    const auto result = updateLine(true, true, true);
    require(!result.accepted && result.points == 0 && result.position.allFinite() && result.covariance.allFinite(),
            "coincident nonplanar points poisoned update or supplied spurious constraints");
}

void convergedPointIterationRetainsLastValidPosterior() {
    for (const char* mode : {"pose6", "full"}) {
        lingtu::ltlio::Config cfg;
        cfg.elevator_enabled = false; cfg.automatic_door = false;
        cfg.hybrid_enabled = true; cfg.map_resolution = 0.005; cfg.ieskf_max_iter = 4;
        auto ctx = std::make_unique<d::Context>(cfg);
        ctx->lidar_update_mode = mode;
        ctx->lidar_ieskf_converge_pos_m = ctx->lidar_ieskf_converge_rot_rad = 1e-12;
        ctx->lidar_ieskf_rematch_pos_m = ctx->lidar_ieskf_rematch_rot_rad = 1e6;
        d::EskfEstimator estimator(*ctx);
        auto state = estimator.get_cur_state();
        state.P = Eigen::MatrixXd::Identity(21, 21) * 1e-8;
        state.P(5, 5) = 1e6;
        estimator.set_cur_state(state);
        ctx->ikd_map->add(line(0.2, 0.0));
        auto query = line(0.2, 1e-7);
        estimator.UpdateByLidarIESKF(*query);
        const auto posterior = estimator.get_cur_state();
        require(estimator.lastUpdateAccepted() && estimator.lastConverged() &&
                estimator.lastIterationCount() >= 2 && estimator.lastPointMatchCount() > 0,
                "all-coincident later point iteration rolled back an already valid update");
        near(posterior.p.z(), -static_cast<double>(query->front().z), 1e-12,
             "converged point estimate did not retain the last valid correction");
        require(posterior.P.allFinite() && posterior.P(5, 5) < 0.01 && posterior.P(5, 5) >= 0.0,
                "converged point update lost the last valid H/K covariance");
    }
}

void invalidHybridParametersAreRejected() {
    for (const double value : {0.0, std::numeric_limits<double>::quiet_NaN()}) {
        for (const bool change_weight : {false, true}) {
            lingtu::ltlio::Config cfg;
            if (change_weight) cfg.hybrid_point_weight = value;
            else cfg.hybrid_point_initial_threshold_m = value;
            bool rejected = false;
            try { lingtu::ltlio::Frontend frontend(cfg); }
            catch (const std::invalid_argument&) { rejected = true; }
            require(rejected, "invalid hybrid covariance/gate parameter was accepted");
        }
    }
}
}  // namespace

int main() {
    struct Test { const char* name; void (*run)(); };
    const Test tests[]{
        {"scalar point residual Jacobian", scalarJacobianMatchesFiniteDifferences},
        {"point residual relinearization and uncertainty", relinearizationUsesNewDirectionAndVariance},
        {"coincident residual rejection", zeroResidualIsRejectedWithoutNaN},
        {"per-query accepted-correction gate", rangeMotionGatePreservesStartupFloor},
        {"exact/pruned correspondence and support equivalence", pruningAgreesWithExhaustiveAndVisitsFewerPoints},
        {"deterministic ties and radius rejection", pruningRetainsDeterministicTieAndRadiusGate},
        {"immature map covariance support", immatureCellsRetainPointsAndCovariance},
        {"point covariance payloads are lazy", planeOnlyCacheDoesNotAllocatePointPayloads},
        {"dirty cell maturation and single evidence", dirtyCellCanMatureIntoPlaneWithoutDuplicatePointEvidence},
        {"reset invalidates point snapshots", resetDropsStalePointBuckets},
        {"native hybrid posterior and pruning equivalence", actualEstimatorUsesNonplanarPointsAndEquivalentPruning},
        {"native coincident point rejection", actualEstimatorRejectsCoincidentPointDirection},
        {"converged later point iterate retains posterior in both solvers", convergedPointIterationRetainsLastValidPosterior},
        {"invalid hybrid parameters", invalidHybridParametersAreRejected},
    };
    int failed = 0;
    for (const auto& test : tests) try { test.run(); std::cout << "PASS " << test.name << '\n'; }
        catch (const std::exception& error) { ++failed; std::cerr << "FAIL " << test.name << ": " << error.what() << '\n'; }
    std::cout << std::size(tests) - failed << '/' << std::size(tests) << " hybrid tests passed\n";
    return failed ? 1 : 0;
}

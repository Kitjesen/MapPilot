#include "localization/opt/gravity_graph.hpp"
#include "localization/opt/pose_math.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>

namespace lingtu::localization::opt {
namespace {
using State = gtsam::Vector4;  // World yaw correction, followed by XYZ.

gtsam::Pose3 pose(const State& state, const gtsam::Rot3& reference) {
  return {gtsam::Rot3::Rz(state[0]) * reference,
          gtsam::Point3(state[1], state[2], state[3])};
}
gtsam::Rot3 rotation(const Pose& value) {
  Eigen::Quaterniond q(value.qw, value.qx, value.qy, value.qz);
  return gtsam::Rot3(q.normalized());
}

struct Measurement {
  gtsam::Pose3 relative;
  gtsam::Rot3 from_reference, to_reference;
  gtsam::Vector6 error(const State& from, const State& to) const {
    return gtsam::Pose3::Logmap(relative.inverse().compose(
        pose(from, from_reference).between(pose(to, to_reference))));
  }
  gtsam::Matrix derivative(const State& from, const State& to, bool first) const {
    gtsam::Matrix result(6, 4);
    constexpr double epsilon = 1e-6;
    for (int k = 0; k < 4; ++k) {
      State plus = first ? from : to, minus = plus;
      plus[k] += epsilon;
      minus[k] -= epsilon;
      result.col(k) = (first ? error(plus, to) - error(minus, to)
                            : error(from, plus) - error(from, minus)) / (2 * epsilon);
    }
    return result;
  }
};

class RelativeFactor final : public gtsam::NoiseModelFactor2<State, State> {
  Measurement measurement_;
 public:
  RelativeFactor(std::size_t from, std::size_t to, Measurement measurement,
                 const gtsam::SharedNoiseModel& noise)
      : NoiseModelFactor2(noise, from, to), measurement_(std::move(measurement)) {}
  gtsam::Vector evaluateError(const State& from, const State& to,
      boost::optional<gtsam::Matrix&> h1 = boost::none,
      boost::optional<gtsam::Matrix&> h2 = boost::none) const override {
    if (h1) *h1 = measurement_.derivative(from, to, true);
    if (h2) *h2 = measurement_.derivative(from, to, false);
    return measurement_.error(from, to);
  }
};

// Eliminate the fixed node instead of approximating it with a large prior weight.
class AnchorFactor final : public gtsam::NoiseModelFactor1<State> {
  Measurement measurement_;
  State anchor_;
  bool from_fixed_;
 public:
  AnchorFactor(std::size_t key, Measurement measurement, State anchor, bool from_fixed,
               const gtsam::SharedNoiseModel& noise)
      : NoiseModelFactor1(noise, key), measurement_(std::move(measurement)),
        anchor_(std::move(anchor)), from_fixed_(from_fixed) {}
  gtsam::Vector evaluateError(const State& value,
      boost::optional<gtsam::Matrix&> h = boost::none) const override {
    const State& from = from_fixed_ ? anchor_ : value;
    const State& to = from_fixed_ ? value : anchor_;
    if (h) *h = measurement_.derivative(from, to, !from_fixed_);
    return measurement_.error(from, to);
  }
};
}

GraphSolution optimize_gravity_graph(const std::vector<Keyframe>& keyframes,
                                     const OptimizeOptions& options) {
  GraphSolution result;
  try {
    std::vector<State> states;
    std::vector<gtsam::Rot3> references;
    for (std::size_t i = 0; i < keyframes.size(); ++i) {
      const auto& estimate = keyframes[i].pose;
      const auto& reference = options.gravity_reference[i].pose;
      State state;
      state << pose_rpy(estimate)[2] - pose_rpy(reference)[2], estimate.x, estimate.y, estimate.z;
      states.push_back(state);
      references.push_back(rotation(reference));
    }
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    for (std::size_t i = 1; i < states.size(); ++i) initial.insert(i, states[i]);
    for (const auto& edge : options.geometric_constraints) {
      const auto& p = edge.pose_from_to;
      Measurement measurement{{rotation(p), gtsam::Point3(p.x, p.y, p.z)},
                              references[edge.from_index], references[edge.to_index]};
      // A fixed-gravity solve cannot accept a measurement requiring different tilt.
      const auto g_from = references[edge.from_index].unrotate(gtsam::Point3(0, 0, 1));
      const auto g_to = references[edge.to_index].unrotate(gtsam::Point3(0, 0, 1));
      const auto predicted = measurement.relative.rotation().rotate(g_to);
      if (std::acos(std::clamp(g_from.dot(predicted), -1.0, 1.0)) > options.max_gravity_error_rad) {
        result.code = "optimizer_gravity_inconsistent";
        result.message = "measured rotation conflicts with the original gravity observations";
        return result;
      }
      gtsam::Matrix6 information;
      std::size_t packed = 0;
      for (int row = 0; row < 6; ++row)
        for (int col = row; col < 6; ++col)
          information(row, col) = information(col, row) = edge.information_upper[packed++];
      Eigen::SelfAdjointEigenSolver<gtsam::Matrix6> eig(information);
      if (eig.info() != Eigen::Success) throw std::runtime_error("information decomposition failed");
      // Rank-four information is intentional; do not invent precision in its nullspace.
      const gtsam::Matrix6 whitening = eig.eigenvalues().cwiseMax(0).cwiseSqrt().asDiagonal() *
                                       eig.eigenvectors().transpose();
      const auto noise = gtsam::noiseModel::Gaussian::SqrtInformation(whitening, false);
      if (edge.from_index == 0 || edge.to_index == 0)
        graph.emplace_shared<AnchorFactor>(edge.from_index == 0 ? edge.to_index : edge.from_index,
            measurement, states[0], edge.from_index == 0, noise);
      else
        graph.emplace_shared<RelativeFactor>(edge.from_index, edge.to_index, measurement, noise);
    }
    gtsam::LevenbergMarquardtParams parameters;
    parameters.maxIterations = options.max_iterations;
    parameters.relativeErrorTol = 1e-9;
    parameters.absoluteErrorTol = 1e-9;
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, parameters);
    const auto solved = optimizer.optimize();
    auto& report = result.report;
    report.struct_size = sizeof(report);
    report.version = LT_POSE_GRAPH_OPT_REPORT_VERSION;
    report.iterations = optimizer.iterations();
    report.initial_cost = graph.error(initial);
    report.final_cost = graph.error(solved);
    // Reaching the iteration ceiling is not evidence of convergence, even if
    // this problem's weighted cost happens to be numerically small.
    report.converged = report.iterations < options.max_iterations;
    if (!report.converged || !std::isfinite(report.initial_cost) || !std::isfinite(report.final_cost) ||
        report.final_cost > report.initial_cost + 1e-9) {
      result.code = "optimizer_quality_failed";
      result.message = "GTSAM fixed-gravity solve did not meet the convergence gate";
      return result;
    }
    result.keyframes = keyframes;
    for (std::size_t i = 0; i < keyframes.size(); ++i) {
      const auto state = i == 0 ? states[0] : solved.at<State>(i);
      if (!state.allFinite()) throw std::runtime_error("GTSAM returned a nonfinite state");
      const auto output = pose(state, references[i]);
      const auto q = output.rotation().toQuaternion();
      result.keyframes[i].pose = {output.x(), output.y(), output.z(), q.w(), q.x(), q.y(), q.z()};
    }
    result.ok = true;
    result.code = "optimized";
    result.message = "GTSAM optimized XYZ and yaw while preserving observed gravity";
  } catch (const std::exception& error) {
    result.code = "optimizer_failed";
    result.message = error.what();
  }
  return result;
}
}

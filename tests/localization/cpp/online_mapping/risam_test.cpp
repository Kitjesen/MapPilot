// Offline solver comparison against the GTSAM 4.3.0 riSAM API.
#include "backend.hpp"

#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/sam/RISAM.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>

namespace online_mapping = lingtu::localization::online_mapping;
namespace {
constexpr std::size_t kPoseCount = 12;
using Between = gtsam::BetweenFactor<gtsam::Pose3>;
using GMLoss = gtsam::noiseModel::mEstimator::GemanMcClure;

enum class Solver { ProductionHuber, Risam };
struct Scenario {
  const char* name;
  bool correct_loop;
  bool wrong_loop;
};
struct Result {
  gtsam::Values estimate;
  std::set<std::size_t> outliers;
  std::optional<std::size_t> correct_index;
  std::optional<std::size_t> wrong_index;
  std::size_t external_updates = 0;
  double rms_m = 0;
  double endpoint_error_m = 0;
  double correct_residual = 0;
  double wrong_residual = 0;
  double elapsed_ms = 0;
};

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

gtsam::Pose3 truth(std::size_t index) {
  const double i = static_cast<double>(index);
  return {gtsam::Rot3::RzRyRx(0.002 * i, -0.001 * i, 0.01 * i),
          {0.5 * i, 0.05 * std::sin(0.3 * i), 0.025 * i}};
}

gtsam::Pose3 raw(std::size_t index) {
  const double i = static_cast<double>(index);
  return truth(index).compose(gtsam::Pose3(gtsam::Rot3(), {0.01 * i, -0.005 * i, 0.002 * i}));
}

const char* name(Solver solver) {
  switch (solver) {
    case Solver::ProductionHuber: return "isam2_huber_gauss_newton";
    case Solver::Risam: return "risam_gm_gauss_newton";
  }
  throw std::logic_error("unknown comparison solver");
}

Result run(const Scenario& scenario, Solver solver) {
  const auto started = std::chrono::steady_clock::now();
  online_mapping::Config config;
  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.1;
  parameters.relinearizeSkip = 1;
  parameters.factorization = gtsam::ISAM2Params::QR;
  std::unique_ptr<gtsam::ISAM2> huber;
  std::unique_ptr<gtsam::RISAM> risam;
  if (solver == Solver::Risam)
    risam = std::make_unique<gtsam::RISAM>(gtsam::RISAM::Parameters(parameters));
  else
    huber = std::make_unique<gtsam::ISAM2>(parameters);

  Result result;
  auto update = [&](const gtsam::NonlinearFactorGraph& factors = {},
                    const gtsam::Values& initial = {}) {
    if (risam) risam->update(factors, initial);
    else huber->update(factors, initial);
    ++result.external_updates;
  };
  gtsam::Vector6 odom_variance;
  for (int i = 0; i < 6; ++i) odom_variance[i] = config.odom_variance[i];
  const auto odom_noise = gtsam::noiseModel::Diagonal::Variances(odom_variance);
  for (std::size_t index = 0; index < kPoseCount; ++index) {
    gtsam::NonlinearFactorGraph factors;
    gtsam::Values initial;
    initial.insert(index, raw(index));
    if (index == 0) {
      // A fixed synthetic anchor makes absolute error measurable in this fixture.
      factors.add(gtsam::PriorFactor<gtsam::Pose3>(
          index, truth(index), gtsam::noiseModel::Isotropic::Sigma(6, 1e-4)));
    } else {
      factors.add(Between(index - 1, index, raw(index - 1).between(raw(index)), odom_noise));
    }
    update(factors, initial);
    update();
  }

  std::array<double, 6> loop_variance;
  for (int i = 0; i < 6; ++i)
    loop_variance[i] = i < 3 ? config.rotation_sigma_rad * config.rotation_sigma_rad
                            : config.translation_sigma_m * config.translation_sigma_m;
  gtsam::Vector6 loop_values;
  for (int i = 0; i < 6; ++i) loop_values[i] = loop_variance[i];
  const auto loop_gaussian = gtsam::noiseModel::Diagonal::Variances(loop_values);
  const auto loss = GMLoss::Create(
      GMLoss::ShapeParameterFromInfluenceThreshold(0.1, 6, 0.95),
      GMLoss::GradScheme::SCALE_INVARIANT);
  const auto scheduler = std::make_shared<gtsam::GraduationScheduler>();
  const auto graduated = gtsam::RISAM::MakeGraduated<Between>(loss, scheduler);
  const auto correct = truth(kPoseCount - 1).between(truth(0));
  const auto wrong = correct.compose(gtsam::Pose3(gtsam::Rot3(), {-3.0, 1.0, 1.0}));

  auto add_loop = [&](const gtsam::Pose3& measurement) {
    const auto index = risam ? risam->getFactorsUnsafe().size() : huber->getFactorsUnsafe().size();
    gtsam::NonlinearFactorGraph factors;
    if (risam) factors.push_back(graduated(kPoseCount - 1, 0, measurement, loop_gaussian));
    else factors.add(Between(kPoseCount - 1, 0, measurement, online_mapping::loopNoise(config, loop_variance)));
    update(factors);
    // Keep the production external update sequence identical in all arms.
    for (int i = 0; i < 6; ++i) update();
    return index;
  };
  if (scenario.correct_loop) result.correct_index = add_loop(correct);
  if (scenario.wrong_loop) result.wrong_index = add_loop(wrong);

  result.estimate = risam ? risam->calculateEstimate() : huber->calculateEstimate();
  require(result.estimate.size() == kPoseCount, "solver lost a Pose3 variable");
  double sum_squared_error = 0;
  for (std::size_t index = 0; index < kPoseCount; ++index) {
    const auto& pose = result.estimate.at<gtsam::Pose3>(index);
    require(pose.matrix().allFinite(), "solver returned a nonfinite Pose3");
    sum_squared_error += (pose.translation() - truth(index).translation()).squaredNorm();
  }
  result.rms_m = std::sqrt(sum_squared_error / kPoseCount);
  const auto& endpoint = result.estimate.at<gtsam::Pose3>(kPoseCount - 1);
  result.endpoint_error_m = (endpoint.translation() - truth(kPoseCount - 1).translation()).norm();
  auto residual = [&](const gtsam::Pose3& measurement) {
    const Between factor(kPoseCount - 1, 0, measurement, loop_gaussian);
    return loop_gaussian->whiten(factor.evaluateError(
        endpoint, result.estimate.at<gtsam::Pose3>(0))).norm();
  };
  if (result.correct_index) result.correct_residual = residual(correct);
  if (result.wrong_index) result.wrong_residual = residual(wrong);
  if (risam) result.outliers = risam->getOutliers(0.95);
  result.elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
  std::cout << "scenario=" << scenario.name << " solver=" << name(solver)
            << " rms_m=" << result.rms_m << " endpoint_error_m=" << result.endpoint_error_m
            << " correct_whitened_residual=" << result.correct_residual
            << " wrong_whitened_residual=" << result.wrong_residual
            << " outliers=" << result.outliers.size()
            << " external_updates=" << result.external_updates
            << " elapsed_ms=" << result.elapsed_ms << '\n';
  return result;
}
}  // namespace

int main() {
  try {
    std::cout << std::setprecision(10);
    const std::array<Solver, 2> solvers{Solver::ProductionHuber, Solver::Risam};
    const std::array<Scenario, 4> scenarios{{
        {"odometry_only", false, false}, {"correct_loop", true, false},
        {"wrong_loop", false, true}, {"correct_and_wrong_loops", true, true}}};
    std::array<Result, 2> baseline;
    for (const auto& scenario : scenarios) {
      std::array<Result, 2> result;
      for (std::size_t i = 0; i < solvers.size(); ++i) result[i] = run(scenario, solvers[i]);
      if (!scenario.correct_loop && !scenario.wrong_loop) {
        baseline = result;
        require(result[0].estimate.equals(result[1].estimate, 1e-8),
                "riSAM changed the all-inlier odometry solution");
      }
      if (scenario.correct_loop && !scenario.wrong_loop) {
        for (std::size_t i = 0; i < solvers.size(); ++i)
          require(result[i].rms_m < baseline[i].rms_m, "correct loop did not improve trajectory error");
      }
      const auto& robust = result[1];
      if (robust.correct_index)
        require(robust.outliers.count(*robust.correct_index) == 0, "riSAM rejected the correct loop");
      if (robust.wrong_index)
        require(robust.outliers.count(*robust.wrong_index) == 1, "riSAM did not identify the wrong loop");
      if (scenario.wrong_loop && !scenario.correct_loop) {
        auto damage = [&](std::size_t i) {
          return (result[i].estimate.at<gtsam::Pose3>(kPoseCount - 1).translation() -
                  baseline[i].estimate.at<gtsam::Pose3>(kPoseCount - 1).translation()).norm();
        };
        for (std::size_t i = 0; i < solvers.size(); ++i)
          std::cout << "scenario=" << scenario.name << " solver=" << name(solvers[i])
                    << " false_loop_displacement_m=" << damage(i) << '\n';
        require(damage(1) < damage(0), "riSAM did not reduce false-loop displacement against matched Huber");
      }
    }
    std::cout << "riSAM offline comparison passed; no production backend changed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "riSAM offline comparison failed: " << error.what() << '\n';
    return 1;
  }
}

#include "ieskf.h"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

using M6 = Eigen::Matrix<double, 6, 6>;
using V6 = Eigen::Matrix<double, 6, 1>;

void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

void configureUpdate(
    IESKF& filter,
    const V12D& residual,
    bool valid = true,
    std::size_t effective_points = 37U) {
  filter.P().setIdentity();
  filter.setLossFunction(
      [residual, valid, effective_points](State&, SharedState& shared) {
        shared.H.setIdentity();
        shared.b = residual;
        shared.valid = valid;
        shared.effective_points = effective_points;
      });
  filter.setStopFunction([](const V21D&) { return true; });
}

void testPartiallyDegenerateUpdatePreservesValidCovariance() {
  IESKF filter;
  using M6 = Eigen::Matrix<double, 6, 6>;
  M6 basis = M6::Identity();
  const double c = std::sqrt(0.5);
  basis(0, 0) = c;
  basis(0, 1) = -c;
  basis(1, 0) = c;
  basis(1, 1) = c;
  M6 prior_pose = 0.01 * M6::Identity();
  prior_pose(0, 1) = prior_pose(1, 0) = 0.0099;
  filter.P() = 0.01 * M21D::Identity();
  filter.P().block<6, 6>(0, 0) = basis * prior_pose * basis.transpose();
  const M21D prior = filter.P();
  M6 information = 10000.0 * M6::Identity();
  information(1, 1) = 10.0;
  const M6 measured_information = basis * information * basis.transpose();
  filter.setLossFunction([measured_information](State&, SharedState& shared) {
    shared.H = 10000.0 * M12D::Identity();
    shared.H.block<6, 6>(0, 0) = measured_information;
    shared.b.setZero();
    shared.valid = true;
    shared.effective_points = 126U;
  });
  filter.setStopFunction([](const V21D&) { return true; });
  require(prior.llt().info() == Eigen::Success, "test prior must be positive definite");
  const bool accepted = filter.update();
  if (!accepted) {
    std::cerr << lidarUpdateRejectionReasonName(
        filter.lastLidarUpdateDiagnostics().rejection_reason) << '\n';
  }
  require(accepted, "a bounded partial-degeneracy update must not corrupt covariance");
  require(filter.P().llt().info() == Eigen::Success,
          "partial-degeneracy posterior must remain positive definite");
  const M6 after = basis.transpose() * filter.P().block<6, 6>(0, 0) * basis;
  require(std::abs(after(1, 1) - prior_pose(1, 1)) < 1e-10,
          "unobservable uncertainty must retain the prediction variance");
  require(after(0, 0) < 0.1 * prior_pose(0, 0),
          "observable directions must still receive LiDAR information");
}

void configurePoseMeasurement(
    IESKF& filter, const M6& information, const V6& residual) {
  filter.setLossFunction([information, residual](State&, SharedState& shared) {
    shared.H.setZero();
    shared.H.block<6, 6>(0, 0) = information;
    shared.b.setZero();
    shared.b.head<6>() = residual;
    shared.valid = true;
    shared.effective_points = 359U;
  });
  filter.setStopFunction([](const V21D&) { return true; });
}

M6 planeGeometry(int plane_count, double scale) {
  M6 information = M6::Zero();
  for (int i = -4; i <= 4; ++i) {
    for (int j = -4; j <= 4; ++j) {
      const double a = i * 0.25;
      const double b = j * 0.25;
      const V3D points[] = {
          scale * V3D(a, b, -0.5),
          scale * V3D(a, 0.5, b),
          scale * V3D(0.5, a, b)};
      const V3D normals[] = {V3D::UnitZ(), V3D::UnitY(), V3D::UnitX()};
      for (int plane = 0; plane < plane_count; ++plane) {
        Eigen::Matrix<double, 1, 6> jacobian;
        jacobian.head<3>() = -normals[plane].transpose() * Sophus::SO3d::hat(points[plane]);
        jacobian.tail<3>() = normals[plane].transpose();
        information += 1000.0 * jacobian.transpose() * jacobian;
      }
    }
  }
  return information;
}

void testGeometryScalePreservesObservability() {
  for (const double scale : {1.0, 12.0}) {
    for (int planes = 1; planes <= 3; ++planes) {
      IESKF filter;
      filter.P().setIdentity();
      configurePoseMeasurement(filter, planeGeometry(planes, scale), V6::Zero());
      const bool accepted = filter.update();
      if (planes == 3) {
        require(accepted, "a full-rank corner became pathological at longer range");
        require(filter.degeneracy().degenerate_dof_count == 0,
                "full-rank corner has false weak translation directions");
      } else {
        require(!accepted, "singular plane/corridor geometry was accepted");
        require(filter.lastLidarUpdateDiagnostics().rejection_reason ==
                    LidarUpdateRejectionReason::PathologicalDegeneracy,
                "singular geometry lost its health rejection");
        require(filter.degeneracy().degenerate_dof_count == (planes == 1 ? 3 : 1),
                "range scaling changed the plane/corridor weak-direction count");
      }
    }
  }
}

void testRecordedFullRankScanIsNotPathological() {
  // Native first-iteration Hessian from static MuJoCo raw frame 25, run 06.
  M6 information;
  information <<
      3004828.90769318, 1085729.28818410, 30005.12531006, 2760.83465344, 37921.26813746, -84258.34005661,
      1085729.28818410, 24580155.6005880, -47449.65681186, 171739.76028389, 14217.96378908, -2000121.34246599,
      30005.12531006, -47449.65681186, 1402037.32245030, 19203.71966495, 151766.49454648, 23207.66474604,
      2760.83465344, 171739.76028389, 19203.71966495, 45330.37759609, 3091.73381050, -22627.99966326,
      37921.26813746, 14217.96378908, 151766.49454648, 3091.73381050, 38524.47582241, 1408.47523906,
      -84258.34005661, -2000121.34246599, 23207.66474604, -22627.99966326, 1408.47523906, 275145.14680112;
  IESKF filter;
  filter.P() = 0.01 * M21D::Identity();
  configurePoseMeasurement(filter, information, V6::Zero());
  require(filter.update(), "recorded full-rank scan was rejected by mixed pose units");
  require(filter.degeneracy().degenerate_dof_count == 0,
          "recorded scan retained three false weak translation directions");
  require(std::abs(filter.degeneracy().condition_number - 65.2106298) < 1e-5,
          "pose-H diagnostics do not describe the balanced block");
}

void testRecordedPartialProjectionUsesPhysicalCoordinates() {
  // Run 06 frame 23 retains one real weak direction after group balancing.
  // Its physical projector is non-symmetric (max |A-A^T| = 0.1023539).
  M6 information;
  information <<
      3179099.87511301, -48280.23928912, 487294.07831521, -46799.24246966, 7819.32604944, 41584.51642509,
      -48280.23928912, 4273607.98472959, -1454095.49312772, 570125.69906367, -32921.05720970, -369691.94260139,
      487294.07831521, -1454095.49312772, 10241737.46432554, -360170.05741601, 33187.67818012, 26873.43230846,
      -46799.24246966, 570125.69906367, -360170.05741601, 160828.51264864, -11546.19075410, -22125.96119341,
      7819.32604944, -32921.05720970, 33187.67818012, -11546.19075410, 2480.86592412, -1342.74047544,
      41584.51642509, -369691.94260139, 26873.43230846, -22125.96119341, -1342.74047544, 178690.62146799;
  M21D prior = 1e-4 * M21D::Identity();
  prior.block<6, 6>(0, 0) = 0.01 * M6::Identity();
  prior.block<3, 3>(12, 12) = 0.05 * M3D::Identity();
  prior(1, 4) = prior(4, 1) = 0.003;
  prior(4, 12) = prior(12, 4) = 0.002;
  prior(0, 5) = prior(5, 0) = -0.001;
  V6 nominal;
  nominal << 0.001, -0.002, 0.0015, 0.004, -0.006, 0.003;
  const V6 residual = information * nominal;

  IESKF metres;
  metres.P() = prior;
  configurePoseMeasurement(metres, information, residual);
  require(metres.update(), "recorded partial update was rejected");
  require(metres.degeneracy().degenerate_dof_count == 1,
          "block balancing erased the recorded weak direction");
  const V21D physical_delta = metres.x() - State{};
  V6 expected_delta;
  expected_delta << -0.00099204, 0.00201058, -0.00149602,
                    -0.00443448, 0.00047943, -0.00307472;
  require((physical_delta.head<6>() - expected_delta).cwiseAbs().maxCoeff() < 1e-8,
          "OC correction used normalized eigenvectors as physical orthogonal vectors");
  require(metres.P().llt().info() == Eigen::Success,
          "non-symmetric physical OC projector corrupted covariance PSD");
  require(std::abs(metres.P()(0, 4) - (-1.35197125505711e-5)) < 1e-10 &&
              std::abs(metres.P()(4, 4) - 0.00948403456823649) < 1e-9 &&
              std::abs(metres.P()(4, 12) - 1.18825170320242e-8) < 1e-10,
          "OC covariance differs from independent scaled-eigenbasis block retention");

  // Re-express pose translation coordinates in cm; other estimator coordinates
  // retain their own units. Convert the corresponding physical guard as well.
  M21D units = M21D::Identity();
  units.block<3, 3>(3, 3) *= 100.0;
  M21D inverse_units = M21D::Identity();
  inverse_units.block<3, 3>(3, 3) *= 0.01;
  const M6 pose_inverse_units = inverse_units.block<6, 6>(0, 0);
  IESKF centimetres;
  centimetres.P() = units * prior * units.transpose();
  centimetres.setDegeneracyGuard(2, 50000.0, 50.0, 0.35, 3.0, 1.0, false, true);
  configurePoseMeasurement(centimetres,
      pose_inverse_units.transpose() * information * pose_inverse_units,
      pose_inverse_units.transpose() * residual);
  require(centimetres.update(), "a change of pose length units changed acceptance");
  require(centimetres.degeneracy().degenerate_dof_count == 1,
          "a change of pose length units changed the weak subspace");
  require((inverse_units * (centimetres.x() - State{}) - physical_delta).norm() < 1e-9,
          "physical state correction depends on pose length units");
  require((inverse_units * centimetres.P() * inverse_units.transpose() - metres.P())
              .cwiseAbs().maxCoeff() < 1e-9,
          "physical posterior covariance depends on pose length units");
}

}  // namespace

int main() {
  testGeometryScalePreservesObservability();
  testRecordedFullRankScanIsNotPathological();
  testRecordedPartialProjectionUsesPhysicalCoordinates();
  testPartiallyDegenerateUpdatePreservesValidCovariance();
  require(
      std::string(lidarUpdateRejectionReasonName(
          LidarUpdateRejectionReason::CandidateTranslationLimitExceeded)) ==
          "candidate_translation_limit_exceeded",
      "stable rejection reason name changed");

  IESKF invalid_measurement;
  configureUpdate(invalid_measurement, V12D::Zero(), false, 0U);
  require(!invalid_measurement.update(), "invalid measurement was accepted");
  const auto invalid_diagnostics = invalid_measurement.lastLidarUpdateDiagnostics();
  require(invalid_diagnostics.attempted, "invalid attempt was not recorded");
  require(!invalid_diagnostics.accepted, "invalid attempt was marked accepted");
  require(
      invalid_diagnostics.rejection_reason ==
          LidarUpdateRejectionReason::NoValidMeasurement,
      "invalid measurement rejection reason mismatch");
  require(invalid_diagnostics.attempt_sequence == 1U, "attempt sequence did not start at one");
  require(invalid_diagnostics.consecutive_rejections == 1U, "rejection streak did not start at one");
  require(invalid_diagnostics.effective_points == 0U, "invalid effective-point count mismatch");

  IESKF translation_guard;
  V12D translation_residual = V12D::Zero();
  translation_residual(3) = -4.0;
  configureUpdate(translation_guard, translation_residual);
  translation_guard.setDegeneracyGuard(
      2, 50000.0, 0.5, 0.35, 3.0, 1.0, false, true);
  require(!translation_guard.update(), "oversized translation update was accepted");
  const auto rejected = translation_guard.lastLidarUpdateDiagnostics();
  require(
      rejected.rejection_reason ==
          LidarUpdateRejectionReason::CandidateTranslationLimitExceeded,
      "translation rejection reason mismatch");
  require(rejected.candidate_translation_m > rejected.max_update_translation_m,
          "translation candidate/threshold evidence mismatch");
  require(rejected.candidate_rotation_rad == 0.0, "unexpected candidate rotation");
  require(rejected.candidate_velocity_mps == 0.0, "unexpected candidate velocity");
  require(rejected.candidate_velocity_delta_mps == 0.0, "unexpected velocity delta");
  require(rejected.effective_points == 37U, "effective-point evidence was lost");
  require(!rejected.information_ldlt_evaluated, "LDLT was evaluated after early rejection");

  configureUpdate(translation_guard, V12D::Zero());
  require(translation_guard.update(), "bounded recovery update was rejected");
  const auto recovered = translation_guard.lastLidarUpdateDiagnostics();
  require(recovered.accepted, "recovery update was not marked accepted");
  require(
      recovered.rejection_reason == LidarUpdateRejectionReason::None,
      "accepted update retained a current rejection reason");
  require(
      recovered.previous_rejection_reason ==
          LidarUpdateRejectionReason::CandidateTranslationLimitExceeded,
      "previous rejection reason was not retained");
  require(recovered.attempt_sequence == 2U, "attempt sequence did not advance");
  require(recovered.consecutive_rejections == 0U, "accepted update did not clear streak");
  require(recovered.information_ldlt_evaluated, "accepted update omitted LDLT evidence");
  require(recovered.information_ldlt_decomposition_success,
          "accepted update reported failed LDLT decomposition");
  require(recovered.information_ldlt_positive, "accepted update reported non-positive LDLT");
  require(recovered.candidate_covariance_evaluated, "candidate covariance was not recorded");
  require(recovered.candidate_covariance_finite, "candidate covariance was non-finite");
  require(recovered.candidate_covariance_positive_diagonal,
          "candidate covariance diagonal was non-positive");
  require(recovered.posterior_covariance_evaluated, "posterior covariance was not recorded");
  require(recovered.posterior_covariance_finite, "posterior covariance was non-finite");
  require(recovered.posterior_covariance_positive_diagonal,
          "posterior covariance diagonal was non-positive");

  IESKF nonconverged;
  configureUpdate(nonconverged, V12D::Zero());
  nonconverged.setMaxIter(1U);
  nonconverged.setStopFunction([](const V21D&) { return false; });
  nonconverged.setDegeneracyGuard(
      2, 50000.0, 0.5, 0.35, 3.0, 1.0, true, false);
  require(!nonconverged.update(), "strict non-converged update was accepted");
  require(
      nonconverged.lastLidarUpdateDiagnostics().rejection_reason ==
          LidarUpdateRejectionReason::NonconvergedUpdate,
      "non-converged rejection reason mismatch");

  IESKF pathological;
  pathological.P().setIdentity();
  pathological.setLossFunction([](State&, SharedState& shared) {
    shared.H.setZero();
    shared.b.setZero();
    shared.valid = true;
    shared.effective_points = 11U;
  });
  pathological.setStopFunction([](const V21D&) { return true; });
  require(!pathological.update(), "pathologically degenerate update was accepted");
  require(
      pathological.lastLidarUpdateDiagnostics().rejection_reason ==
          LidarUpdateRejectionReason::PathologicalDegeneracy,
      "pathological-degeneracy rejection reason mismatch");

  return 0;
}

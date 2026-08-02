#include "ieskf.h"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

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

}  // namespace

int main() {
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

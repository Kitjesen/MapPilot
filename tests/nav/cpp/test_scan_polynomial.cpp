#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

#include "planning/local/scan/upstream/traj_utils/polynomial_traj.h"

namespace {
using namespace nav_kernel::local::scan::upstream;

// Frozen pre-optimization formula from 2026-10-23. Keep this test-only dense
// implementation independent of the production segment-block assembly.
PolynomialTraj legacyMinSnap(const Eigen::MatrixXd &Pos, const Eigen::Vector3d &start_vel,
                                           const Eigen::Vector3d &end_vel, const Eigen::Vector3d &start_acc,
                                           const Eigen::Vector3d &end_acc, const Eigen::VectorXd &Time)
{
  int seg_num = Time.size();
  Eigen::MatrixXd poly_coeff(seg_num, 3 * 6);
  Eigen::VectorXd Px(6 * seg_num), Py(6 * seg_num), Pz(6 * seg_num);

  int num_f, num_p; // number of fixed and free variables
  int num_d;        // number of all segments' derivatives

  const static auto Factorial = [](int x) {
    int fac = 1;
    for (int i = x; i > 0; i--)
      fac = fac * i;
    return fac;
  };

  /* ---------- end point derivative ---------- */
  Eigen::VectorXd Dx = Eigen::VectorXd::Zero(seg_num * 6);
  Eigen::VectorXd Dy = Eigen::VectorXd::Zero(seg_num * 6);
  Eigen::VectorXd Dz = Eigen::VectorXd::Zero(seg_num * 6);

  for (int k = 0; k < seg_num; k++)
  {
    /* position to derivative */
    Dx(k * 6) = Pos(0, k);
    Dx(k * 6 + 1) = Pos(0, k + 1);
    Dy(k * 6) = Pos(1, k);
    Dy(k * 6 + 1) = Pos(1, k + 1);
    Dz(k * 6) = Pos(2, k);
    Dz(k * 6 + 1) = Pos(2, k + 1);

    if (k == 0)
    {
      Dx(k * 6 + 2) = start_vel(0);
      Dy(k * 6 + 2) = start_vel(1);
      Dz(k * 6 + 2) = start_vel(2);

      Dx(k * 6 + 4) = start_acc(0);
      Dy(k * 6 + 4) = start_acc(1);
      Dz(k * 6 + 4) = start_acc(2);
    }
    else if (k == seg_num - 1)
    {
      Dx(k * 6 + 3) = end_vel(0);
      Dy(k * 6 + 3) = end_vel(1);
      Dz(k * 6 + 3) = end_vel(2);

      Dx(k * 6 + 5) = end_acc(0);
      Dy(k * 6 + 5) = end_acc(1);
      Dz(k * 6 + 5) = end_acc(2);
    }
  }

  /* ---------- Mapping Matrix A ---------- */
  Eigen::MatrixXd Ab;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);

  for (int k = 0; k < seg_num; k++)
  {
    Ab = Eigen::MatrixXd::Zero(6, 6);
    for (int i = 0; i < 3; i++)
    {
      Ab(2 * i, i) = Factorial(i);
      for (int j = i; j < 6; j++)
        Ab(2 * i + 1, j) = Factorial(j) / Factorial(j - i) * pow(Time(k), j - i);
    }
    A.block(k * 6, k * 6, 6, 6) = Ab;
  }

  /* ---------- Produce Selection Matrix C' ---------- */
  Eigen::MatrixXd Ct, C;

  num_f = 2 * seg_num + 4; // 3 + 3 + (seg_num - 1) * 2 = 2m + 4
  num_p = 2 * seg_num - 2; //(seg_num - 1) * 2 = 2m - 2
  num_d = 6 * seg_num;
  Ct = Eigen::MatrixXd::Zero(num_d, num_f + num_p);
  Ct(0, 0) = 1;
  Ct(2, 1) = 1;
  Ct(4, 2) = 1; // stack the start point
  Ct(1, 3) = 1;
  Ct(3, 2 * seg_num + 4) = 1;
  Ct(5, 2 * seg_num + 5) = 1;

  Ct(6 * (seg_num - 1) + 0, 2 * seg_num + 0) = 1;
  Ct(6 * (seg_num - 1) + 1, 2 * seg_num + 1) = 1; // Stack the end point
  Ct(6 * (seg_num - 1) + 2, 4 * seg_num + 0) = 1;
  Ct(6 * (seg_num - 1) + 3, 2 * seg_num + 2) = 1; // Stack the end point
  Ct(6 * (seg_num - 1) + 4, 4 * seg_num + 1) = 1;
  Ct(6 * (seg_num - 1) + 5, 2 * seg_num + 3) = 1; // Stack the end point

  for (int j = 2; j < seg_num; j++)
  {
    Ct(6 * (j - 1) + 0, 2 + 2 * (j - 1) + 0) = 1;
    Ct(6 * (j - 1) + 1, 2 + 2 * (j - 1) + 1) = 1;
    Ct(6 * (j - 1) + 2, 2 * seg_num + 4 + 2 * (j - 2) + 0) = 1;
    Ct(6 * (j - 1) + 3, 2 * seg_num + 4 + 2 * (j - 1) + 0) = 1;
    Ct(6 * (j - 1) + 4, 2 * seg_num + 4 + 2 * (j - 2) + 1) = 1;
    Ct(6 * (j - 1) + 5, 2 * seg_num + 4 + 2 * (j - 1) + 1) = 1;
  }

  C = Ct.transpose();

  Eigen::VectorXd Dx1 = C * Dx;
  Eigen::VectorXd Dy1 = C * Dy;
  Eigen::VectorXd Dz1 = C * Dz;

  /* ---------- minimum snap matrix ---------- */
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);

  for (int k = 0; k < seg_num; k++)
  {
    for (int i = 3; i < 6; i++)
    {
      for (int j = 3; j < 6; j++)
      {
        Q(k * 6 + i, k * 6 + j) =
            i * (i - 1) * (i - 2) * j * (j - 1) * (j - 2) / (i + j - 5) * pow(Time(k), (i + j - 5));
      }
    }
  }

  /* ---------- R matrix ---------- */
  Eigen::MatrixXd R = C * A.transpose().inverse() * Q * A.inverse() * Ct;

  Eigen::VectorXd Dxf(2 * seg_num + 4), Dyf(2 * seg_num + 4), Dzf(2 * seg_num + 4);

  Dxf = Dx1.segment(0, 2 * seg_num + 4);
  Dyf = Dy1.segment(0, 2 * seg_num + 4);
  Dzf = Dz1.segment(0, 2 * seg_num + 4);

  Eigen::MatrixXd Rff(2 * seg_num + 4, 2 * seg_num + 4);
  Eigen::MatrixXd Rfp(2 * seg_num + 4, 2 * seg_num - 2);
  Eigen::MatrixXd Rpf(2 * seg_num - 2, 2 * seg_num + 4);
  Eigen::MatrixXd Rpp(2 * seg_num - 2, 2 * seg_num - 2);

  Rff = R.block(0, 0, 2 * seg_num + 4, 2 * seg_num + 4);
  Rfp = R.block(0, 2 * seg_num + 4, 2 * seg_num + 4, 2 * seg_num - 2);
  Rpf = R.block(2 * seg_num + 4, 0, 2 * seg_num - 2, 2 * seg_num + 4);
  Rpp = R.block(2 * seg_num + 4, 2 * seg_num + 4, 2 * seg_num - 2, 2 * seg_num - 2);

  /* ---------- close form solution ---------- */

  Eigen::VectorXd Dxp(2 * seg_num - 2), Dyp(2 * seg_num - 2), Dzp(2 * seg_num - 2);
  Dxp = -(Rpp.inverse() * Rfp.transpose()) * Dxf;
  Dyp = -(Rpp.inverse() * Rfp.transpose()) * Dyf;
  Dzp = -(Rpp.inverse() * Rfp.transpose()) * Dzf;

  Dx1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dxp;
  Dy1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dyp;
  Dz1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dzp;

  Px = (A.inverse() * Ct) * Dx1;
  Py = (A.inverse() * Ct) * Dy1;
  Pz = (A.inverse() * Ct) * Dz1;

  for (int i = 0; i < seg_num; i++)
  {
    poly_coeff.block(i, 0, 1, 6) = Px.segment(i * 6, 6).transpose();
    poly_coeff.block(i, 6, 1, 6) = Py.segment(i * 6, 6).transpose();
    poly_coeff.block(i, 12, 1, 6) = Pz.segment(i * 6, 6).transpose();
  }

  /* ---------- use polynomials ---------- */
  PolynomialTraj poly_traj;
  for (int i = 0; i < poly_coeff.rows(); ++i)
  {
    vector<double> cx(6), cy(6), cz(6);
    for (int j = 0; j < 6; ++j)
    {
      cx[j] = poly_coeff(i, j), cy[j] = poly_coeff(i, j + 6), cz[j] = poly_coeff(i, j + 12);
    }
    reverse(cx.begin(), cx.end());
    reverse(cy.begin(), cy.end());
    reverse(cz.begin(), cz.end());
    double ts = Time(i);
    poly_traj.addSegment(cx, cy, cz, ts);
  }

  return poly_traj;
}


struct ComparisonInput {
  Eigen::MatrixXd points;
  Eigen::VectorXd times;
  Eigen::Vector3d startVelocity{.08,.02,.01};
  Eigen::Vector3d endVelocity{.04,-.01,.02};
  Eigen::Vector3d startAcceleration{.03,-.02,.01};
  Eigen::Vector3d endAcceleration{-.02,.01,-.01};
};

void comparePolynomials(const ComparisonInput &input) {
  const auto beforeStart = std::chrono::steady_clock::now();
  auto before = legacyMinSnap(input.points,input.startVelocity,input.endVelocity,
                             input.startAcceleration,input.endAcceleration,input.times);
  const auto afterStart = std::chrono::steady_clock::now();
  auto after = PolynomialTraj::minSnapTraj(input.points,input.startVelocity,input.endVelocity,
                                        input.startAcceleration,input.endAcceleration,input.times);
  const auto afterEnd = std::chrono::steady_clock::now();
  before.init();
  after.init();
  ASSERT_EQ(before.getTimes(),after.getTimes());
  double maxPositionError = 0;
  double maxVelocityError = 0;
  double maxAccelerationError = 0;
  double beginning = 0;
  for (int segment = 0; segment < input.times.size(); ++segment) {
    for (int index = 0; index <= 20; ++index) {
      const double time = beginning + input.times(segment) * index / 20;
      const Eigen::Vector3d p = before.evaluate(time);
      const Eigen::Vector3d v = before.evaluateVel(time);
      const Eigen::Vector3d a = before.evaluateAcc(time);
      const double pError = (after.evaluate(time)-p).norm();
      const double vError = (after.evaluateVel(time)-v).norm();
      const double aError = (after.evaluateAcc(time)-a).norm();
      maxPositionError = std::max(maxPositionError,pError);
      maxVelocityError = std::max(maxVelocityError,vError);
      maxAccelerationError = std::max(maxAccelerationError,aError);
      EXPECT_LE(pError,1e-8*(1+p.norm())) << "time=" << time;
      EXPECT_LE(vError,1e-7*(1+v.norm())) << "time=" << time;
      EXPECT_LE(aError,1e-6*(1+a.norm())) << "time=" << time;
    }
    EXPECT_LE((after.evaluate(beginning)-input.points.col(segment)).norm(),1e-7);
    beginning += input.times(segment);
  }
  EXPECT_LE((after.evaluate(beginning)-input.points.col(input.points.cols()-1)).norm(),1e-7);
  EXPECT_LE((after.evaluateVel(0)-input.startVelocity).norm(),1e-7);
  EXPECT_LE((after.evaluateVel(beginning)-input.endVelocity).norm(),1e-7);
  EXPECT_LE((after.evaluateAcc(0)-input.startAcceleration).norm(),1e-6);
  EXPECT_LE((after.evaluateAcc(beginning)-input.endAcceleration).norm(),1e-6);
  const double beforeMs = std::chrono::duration<double,std::milli>(afterStart-beforeStart).count();
  const double afterMs = std::chrono::duration<double,std::milli>(afterEnd-afterStart).count();
  std::cout << "segments=" << input.times.size() << " legacy_ms=" << beforeMs
            << " block_ms=" << afterMs << " max_position_error_m=" << maxPositionError
            << " max_velocity_error_mps=" << maxVelocityError
            << " max_acceleration_error_mps2=" << maxAccelerationError << '\n';
}

TEST(ScanPolynomial, BlockSolvePreservesThreeDimensionalCurvesAndNonzeroBoundaries) {
  ComparisonInput input;
  input.points.resize(3,5);
  input.points << 0,.4,.7,1.4,2,
                  0,.1,-.2,.3,.4,
                  .35,.45,.4,.7,.8;
  input.times.resize(4);
  input.times << .8,1.1,.6,1.5;
  comparePolynomials(input);
}

TEST(ScanPolynomial, BlockSolvePreservesUnequalShortAndLongIntervals) {
  ComparisonInput input;
  input.points.resize(3,7);
  input.points << 0,.015,.3,.5,.9,.92,1.4,
                  0,.002,.1,-.1,.1,.12,.2,
                  .35,.36,.4,.5,.7,.71,.8;
  input.times.resize(6);
  input.times << .035,.4,1.2,.2,.075,.7;
  comparePolynomials(input);
}

TEST(ScanPolynomial, DenseReferenceRetainsTheLegacyPositionVelocityAndAcceleration) {
  constexpr int segments = 110;
  ComparisonInput input;
  input.points.resize(3,segments+1);
  input.times.resize(segments);
  for (int index = 0; index <= segments; ++index) {
    input.points.col(index) = Eigen::Vector3d(.3*index,.8*std::sin(.07*index),
                                            .35+.05*std::sin(.11*index));
    if (index > 0)
      input.times(index-1) = (input.points.col(index)-input.points.col(index-1)).norm()/.6;
  }
  input.times(0) *= 2;
  input.times(segments-1) *= 2;
  input.startVelocity.setZero();
  input.endVelocity.setZero();
  input.startAcceleration.setZero();
  input.endAcceleration.setZero();
  comparePolynomials(input);
}
}  // namespace

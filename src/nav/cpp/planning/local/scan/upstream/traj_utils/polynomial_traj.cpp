#include <iostream>
#include "planning/local/scan/upstream/traj_utils/polynomial_traj.h"

namespace nav_kernel::local::scan::upstream {

PolynomialTraj PolynomialTraj::minSnapTraj(const Eigen::MatrixXd &Pos, const Eigen::Vector3d &start_vel,
                                           const Eigen::Vector3d &end_vel, const Eigen::Vector3d &start_acc,
                                           const Eigen::Vector3d &end_acc, const Eigen::VectorXd &Time)
{
  int seg_num = Time.size();
  Eigen::MatrixXd poly_coeff(seg_num, 3 * 6);

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
  using SegmentMatrix = Eigen::Matrix<double, 6, 6>;
  std::vector<SegmentMatrix> inverse_mapping;
  inverse_mapping.reserve(seg_num);

  for (int k = 0; k < seg_num; k++)
  {
    SegmentMatrix Ab = SegmentMatrix::Zero();
    for (int i = 0; i < 3; i++)
    {
      Ab(2 * i, i) = Factorial(i);
      for (int j = i; j < 6; j++)
        Ab(2 * i + 1, j) = Factorial(j) / Factorial(j - i) * pow(Time(k), j - i);
    }
    // A is block diagonal. Invert each endpoint mapping once rather than
    // repeatedly factoring the complete 6N by 6N matrix for each axis.
    inverse_mapping.push_back(Ab.inverse());
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

  Eigen::MatrixXd derivatives(num_f + num_p, 3);
  derivatives.col(0) = C * Dx;
  derivatives.col(1) = C * Dy;
  derivatives.col(2) = C * Dz;

  /* ---------- minimum snap matrix ---------- */
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(num_f + num_p, num_f + num_p);

  for (int k = 0; k < seg_num; k++)
  {
    SegmentMatrix Q = SegmentMatrix::Zero();
    for (int i = 3; i < 6; i++)
    {
      for (int j = 3; j < 6; j++)
      {
        Q(i, j) =
            i * (i - 1) * (i - 2) * j * (j - 1) * (j - 2) / (i + j - 5) * pow(Time(k), (i + j - 5));
      }
    }
    const SegmentMatrix cost = inverse_mapping[k].transpose() * Q * inverse_mapping[k];
    Eigen::Index selected[6];
    for (int i = 0; i < 6; ++i)
      Ct.row(k * 6 + i).maxCoeff(&selected[i]);
    // Each row of Ct selects one endpoint derivative. Scatter the segment's
    // contribution to the same reduced objective without dense zero blocks.
    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 6; ++j)
        R(selected[i], selected[j]) += cost(i, j);
  }

  /* ---------- close form solution, shared by XYZ ---------- */
  const Eigen::MatrixXd Rfp = R.block(0, num_f, num_f, num_p);
  const Eigen::MatrixXd Rpp = R.block(num_f, num_f, num_p, num_p);
  const Eigen::PartialPivLU<Eigen::MatrixXd> solver(Rpp);
  derivatives.bottomRows(num_p) =
      solver.solve((-Rfp.transpose() * derivatives.topRows(num_f)).eval());

  for (int i = 0; i < seg_num; i++)
  {
    const Eigen::Matrix<double, 6, 3> coefficients =
        inverse_mapping[i] * Ct.middleRows(i * 6, 6) * derivatives;
    for (int axis = 0; axis < 3; ++axis)
      poly_coeff.block(i, axis * 6, 1, 6) = coefficients.col(axis).transpose();
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

PolynomialTraj PolynomialTraj::one_segment_traj_gen(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                    const Eigen::Vector3d &end_pt, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc,
                                                    double t)
{
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(6, 6), Crow(1, 6);
  Eigen::VectorXd Bx(6), By(6), Bz(6);

  C(0, 5) = 1;
  C(1, 4) = 1;
  C(2, 3) = 2;
  Crow << pow(t, 5), pow(t, 4), pow(t, 3), pow(t, 2), t, 1;
  C.row(3) = Crow;
  Crow << 5 * pow(t, 4), 4 * pow(t, 3), 3 * pow(t, 2), 2 * t, 1, 0;
  C.row(4) = Crow;
  Crow << 20 * pow(t, 3), 12 * pow(t, 2), 6 * t, 2, 0, 0;
  C.row(5) = Crow;

  Bx << start_pt(0), start_vel(0), start_acc(0), end_pt(0), end_vel(0), end_acc(0);
  By << start_pt(1), start_vel(1), start_acc(1), end_pt(1), end_vel(1), end_acc(1);
  Bz << start_pt(2), start_vel(2), start_acc(2), end_pt(2), end_vel(2), end_acc(2);

  Eigen::VectorXd Cofx = C.colPivHouseholderQr().solve(Bx);
  Eigen::VectorXd Cofy = C.colPivHouseholderQr().solve(By);
  Eigen::VectorXd Cofz = C.colPivHouseholderQr().solve(Bz);

  vector<double> cx(6), cy(6), cz(6);
  for (int i = 0; i < 6; i++)
  {
    cx[i] = Cofx(i);
    cy[i] = Cofy(i);
    cz[i] = Cofz(i);
  }

  PolynomialTraj poly_traj;
  poly_traj.addSegment(cx, cy, cz, t);

  return poly_traj;
}


}  // namespace nav_kernel::local::scan::upstream

#include "../../fastlio2/src/map_builder/ieskf.h"

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

}  // namespace

int main() {
  IESKF filter;
  const M21D& covariance = filter.P();

  require(covariance.allFinite(), "initial IEKF covariance is not finite");
  require(
      covariance.isApprox(M21D::Identity(), 1e-12),
      "initial IEKF covariance does not match the upstream identity prior");
  require(
      std::abs(covariance.diagonal().segment<3>(3).sum() - 3.0) < 1e-12,
      "initial IEKF position covariance trace is not deterministic");
  return 0;
}

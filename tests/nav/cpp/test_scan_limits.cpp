#include <cassert>
#include <cmath>
#include <iostream>
#include "collision_bitmap.hpp"
#include "planning/local/scan/backend.hpp"
#include "trajectory/spline.hpp"

int main() {
  using namespace nav_kernel;
  lingtu::nav::tests::CollisionBitmap bitmap;
  std::vector<Vec3> route{{0,0,.5},{1,.7,.5},{2,.7,.5},{3,0,.5}};
  LocalPlannerParams params;
  params.scan.voxelResolution = .1;
  params.scan.cylinderOffset = 0;
  params.scan.maxAcceleration = 1;
  LocalPlanRequest request;
  request.robot.pose = {route.front(),0};
  request.robot.kinematics.valid = true;
  request.robot.kinematics.linearVelocity = {.15,0,0};
  request.objective = RouteTarget{{route.data(),static_cast<int>(route.size()),1}};
  request.identity = {1,1,0};
  request.environment.collision = bitmap.view();
  request.maxLinearSpeedMps = .6;
  local::scan::Backend planner(params);
  LocalPlan plan;
  for (int tick=0; tick<100 && !plan.ready(); ++tick) {
    request.clock.timestampS = 1 + .01*tick;
    plan = planner.tick(request);
  }
  assert(plan.ready());
  const SplineView spline(std::get<SplineTarget>(plan.target()));
  double maximum = 0;
  for (double t=0; t<=spline.duration(); t+=.01) {
    const auto velocity = spline.velocity(t);
    maximum = std::max(maximum,std::hypot(velocity.x,velocity.y));
  }
  std::cout << "maximum speed " << maximum << '\n';
  assert(maximum <= request.maxLinearSpeedMps + 1e-6);
}

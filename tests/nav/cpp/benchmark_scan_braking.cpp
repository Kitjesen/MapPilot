#include "collision/body.hpp"
#include "collision/body_io.hpp"
#include "collision_bitmap.hpp"
#include "planning/local/scan/grid.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
namespace c = lingtu::nav::collision;
using Clock = std::chrono::steady_clock;

std::shared_ptr<c::Body> singleOBB() {
  auto body = std::make_shared<c::Body>();
  c::Part base;
  base.link = "base";
  // Fixed Go2 navigation-envelope fixture: .76 x .31 m plus .10 m per side,
  // with body-relative vertical extent [-.10, .24]. This is not a field test.
  base.center.z = .07;
  base.half_size = {.48,.255,.17};
  body->parts.push_back(base);
  body->margin_m = 0;
  return body;
}
}  // namespace

int main(int argc, char** argv) {
  try {
    int iterations = 100;
    std::string bodyPath;
    for (int i=1; i<argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--body" && i+1<argc) bodyPath = argv[++i];
      else if (arg == "--iterations" && i+1<argc)
        iterations = std::clamp(std::atoi(argv[++i]),1,1000);
      else throw std::runtime_error("usage: benchmark_scan_braking [--body BODY2] [--iterations 1..1000]");
    }
    auto body = singleOBB();
    if (!bodyPath.empty()) {
      std::ifstream input(bodyPath);
      if (!input) throw std::runtime_error("cannot open body file");
      body = c::readBody(input);
    }
    // The loaded model uses the recorded straight6-profile route height.
    // The single OBB uses the Go2 configured standing reference height.
    const double height = bodyPath.empty() ? .35 : .45076444581607977;
    const std::string model = bodyPath.empty() ? "single_obb" : "loaded_body";
    nav_kernel::LocalPlannerParams params;
    params.scan.voxelResolution = .05;
    params.scan.cylinderOffset = 0;
    struct Scenario {
      const char* name;
      nav_kernel::Twist command;
      bool obstacle;
    };
    const std::vector<Scenario> scenarios{
        {"straight_free",{.6,0,0},false},
        {"curve_free",{.6,.1,.8},false},
        {"straight_obstacle",{.6,0,0},true}};
    std::cout << "model,parts,scenario,queries,vx,vy,wz,reaction_s,linear_deceleration,yaw_deceleration,initial_occupancy,result,elapsed_ms,us_per_query\n"
              << std::setprecision(12);
    for (const auto& scenario : scenarios) {
      lingtu::nav::tests::CollisionBitmap map({-2,-2,-1},{2,2,2},.05);
      for (int x=-40; x<40; ++x)
        for (int y=-40; y<40; ++y)
          map.occupy({(x+.5)*.05,(y+.5)*.05,-.025});
      if (scenario.obstacle)
        for (int z=0; z<20; ++z)
          for (int y=-10; y<10; ++y)
            map.occupy({.725,(y+.5)*.05,(z+.5)*.05});
      nav_kernel::LocalPlanRequest request;
      request.body = body;
      request.robot.pose = {{0,0,height},0};
      request.environment.collision = map.view();
      auto& view = request.environment.collision;
      view.measuredOccupiedStorage =
          std::make_shared<const std::vector<std::uint8_t>>(
              view.inflatedBits,view.inflatedBits+view.inflatedBytes);
      view.knownFreeStorage = std::make_shared<const std::vector<std::uint8_t>>(
          view.inflatedBytes,255U);
      nav_kernel::local::scan::Grid grid(params,request);
      if (!grid.valid()) throw std::runtime_error(grid.reason());
      const int initial = grid.inflatedOccupancy(request.robot.pose.position,0);
      const int expected = grid.brakingOccupancy(request.robot.pose,scenario.command,.1,.5,1);
      int queries = 0;
      const auto start = Clock::now();
      // Three six-second budgets keep even a slow debug run bounded.
      do {
        const int result = grid.brakingOccupancy(request.robot.pose,scenario.command,.1,.5,1);
        if (result != expected) throw std::runtime_error("inconsistent repeated occupancy");
        ++queries;
      } while (queries<iterations && Clock::now()-start<std::chrono::seconds(6));
      const double elapsedMs = std::chrono::duration<double,std::milli>(Clock::now()-start).count();
      std::cout << model << ',' << body->parts.size() << ',' << scenario.name << ','
                << queries << ',' << scenario.command.vx << ',' << scenario.command.vy << ','
                << scenario.command.wz << ",0.1,0.5,1," << initial << ',' << expected << ','
                << elapsedMs << ',' << elapsedMs*1000/queries << '\n';
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}

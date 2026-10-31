#include "backend.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace m = lingtu::localization::online_mapping;
void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}

m::Cloud::Ptr bay(double phase, bool exit = false, bool distorted = false) {
  m::Cloud::Ptr cloud(new m::Cloud);
  auto add = [&](double x, double y, double z) {
    pcl::PointXYZI p;
    p.x = x-phase; p.y = y;
    p.z = z+(distorted ? .025*std::sin(1.7*x+.3*y) : 0);
    p.intensity = 1; cloud->push_back(p);
  };
  for (int x=-40;x<=40;++x) for (int y=-30;y<=30;++y) {
    add(x*.1,y*.1,-.5); add(x*.1,y*.1,2.5);
  }
  for (double x:{-2.,2.}) for (double y:{-1.5,1.5})
    for (int a=-3;a<=3;++a) for (int h=-5;h<=25;++h) {
      add(x-.3,y+a*.1,h*.1); add(x+.3,y+a*.1,h*.1);
      add(x+a*.1,y-.3,h*.1); add(x+a*.1,y+.3,h*.1);
    }
  if (exit) for (int a=-25;a<=25;++a) for (int h=-5;h<=25;++h)
    add(.7,a*.1,h*.1);
  return cloud;
}

gtsam::Pose3 pose(double x, double y=0, double z=0) {
  return {gtsam::Rot3(), {x,y,z}};
}

void repeatedBays(bool exit, bool third=false, bool distorted=false) {
  m::Config config; config.voxel_m=.15; config.submap_half_window=0;
  m::Backend backend(config);
  backend.append({1,pose(0),bay(0)});
  backend.append({2,pose(1.5),bay(1.5)});
  backend.append({3,pose(4),bay(0,exit,distorted)});
  backend.append({4,pose(5.5),bay(1.5,exit,distorted)});
  if (third) {
    backend.append({5,pose(8),bay(0)});
    backend.append({6,pose(9.5),bay(1.5)});
  }
  // Physical current x is 4/5.5 m, while drifted odometry says 0/1.5 m.
  const auto first=backend.append({40,pose(0),bay(0,exit,distorted)});
  const auto second=backend.append({41,pose(1.5),bay(1.5,exit,distorted)});
  require(!first.accepted,"single revisit bypassed independent confirmation");
  if (exit) {
    require(second.accepted && second.previous==3,
        "distinctive exit did not select the correct historical bay");
    require(second.measurement.translation().norm()<.02,
        "accepted exit edge differs from its known physical relative pose");
  } else {
    require(!second.accepted && backend.loops()==0,
        "coherent moving periodic alias entered the graph");
    require(second.reason==(third ? "candidate_budget_pending" : "ambiguous_place"),
        "unresolved competing places were silently discarded");
  }
  require(second.verified_candidates<=2,"full registration budget exceeded");
}

m::Cloud::Ptr aisle(double period, double phase_y) {
  m::Cloud::Ptr cloud(new m::Cloud);
  auto add=[&](double x,double y,double z) {
    pcl::PointXYZI p;p.x=x;p.y=y-phase_y;p.z=z;p.intensity=1;
    cloud->push_back(p);
  };
  for(int x=-20;x<=20;++x)for(int y=-30;y<=30;++y) {
    add(x*.1,y*.1,-.5);add(x*.1,y*.1,2.5);
  }
  for(int x=-20;x<=20;++x)for(int h=-5;h<=25;++h)add(x*.1,-3,h*.1);
  for(int k=-3;k<=3;++k)for(double y:{-1.5,1.5})
    for(int a=-2;a<=2;++a)for(int h=-5;h<=25;++h) {
      const double x=(k+.5)*period;
      add(x-.08,y+a*.05,h*.1);add(x+.08,y+a*.05,h*.1);
      add(x+a*.04,y-.1,h*.1);add(x+a*.04,y+.1,h*.1);
    }
  return cloud;
}

void neighboringAisles(double period) {
  m::Config config;config.voxel_m=.15;
  m::Backend backend(config); // Keep the production +/-25 target window.
  backend.append({1,pose(0,0),aisle(period,0)});
  backend.append({2,pose(0,1.5),aisle(period,1.5)});
  backend.append({3,pose(period,0),aisle(period,0)});
  backend.append({4,pose(period,1.5),aisle(period,1.5)});
  backend.append({40,pose(0,0),aisle(period,0)});
  const auto result=backend.append({41,pose(0,1.5),aisle(period,1.5)});
  require(!result.accepted && result.reason=="ambiguous_place",
      "overlapping target windows hid a distinct submeter registration mode");
  require(result.candidate_count>=2,"different aisle modes were merged by anchor distance");
}

m::Cloud::Ptr room(double x) {
  m::Cloud::Ptr cloud(new m::Cloud);
  auto add=[&](double a,double b,double z) {
    pcl::PointXYZI p;p.x=a-x;p.y=b;p.z=z;p.intensity=1;
    cloud->push_back(p);
  };
  for(int a=-10;a<=90;++a)for(int b=-10;b<=10;++b) {
    add(a*.2,b*.2,0);add(a*.2,b*.2,3);
  }
  for(int a=-10;a<=90;++a)for(int h=0;h<=15;++h) {
    add(a*.2,-2,h*.2);add(a*.2,2,h*.2);
  }
  for(int b=-10;b<=10;++b)for(int h=0;h<=15;++h) {
    add(-2,b*.2,h*.2);add(18,b*.2,h*.2);
  }
  for(int a=0;a<=10;++a)for(int h=0;h<=12;++h) {
    add(6.7+a*.1,.8,h*.1);add(6.7,.8+a*.1,h*.1);
  }
  return cloud;
}

void denseRevisit() {
  m::Config config;m::Backend backend(config);
  // Match production keyframe spacing: 0.20 m and 1 second.
  for(int i=0;i<80;++i)
    backend.append({1.+i,pose(i*.2,0,i*.0005),room(i*.2)});
  const auto before=backend.loops();
  for(int i=0;i<20;++i) {
    const auto result=backend.append({200.+i,pose(i*.2,0,.08+i*.0005),room(i*.2)});
    if(result.accepted) {
      const double expected_x=(result.previous-i)*.2;
      require(std::abs(result.measurement.x()-expected_x)<config.voxel_m,
          "dense revisit accepted a physically wrong longitudinal edge");
    }
    require(result.verified_candidates<=2,"dense history exceeded fine ICP budget");
  }
  require(backend.loops()>before,"dense normal history permanently exhausted candidate budget");
  std::cout<<"dense_revisit_loops="<<backend.loops()-before<<'\n';
}

void sparseAnchor() {
  m::Config config;config.voxel_m=.15;
  m::Backend backend(config);
  m::Cloud::Ptr sparse(new m::Cloud);sparse->push_back(pcl::PointXYZI());
  backend.append({1,pose(0),sparse});
  backend.append({2,pose(20),bay(20)});
  const auto result=backend.append({40,pose(0),bay(0)});
  require(result.reason=="confirmation_pending" && result.target_points>=1000,
      "sparse coarse anchor rejected a geometrically valid historical window");
  require(result.target_frames==std::vector<std::size_t>{0,1},
      "historical window lost its older frame outside the anchor radius");
}

int main() {
  try {
    repeatedBays(false); repeatedBays(false,false,true);
    repeatedBays(true); repeatedBays(false,true); repeatedBays(true,true);
    neighboringAisles(.6); neighboringAisles(.8); sparseAnchor(); denseRevisit();
    std::cout<<"online mapping competing-place regressions passed\n";
    return 0;
  } catch(const std::exception& error) {
    std::cerr<<error.what()<<'\n';return 1;
  }
}

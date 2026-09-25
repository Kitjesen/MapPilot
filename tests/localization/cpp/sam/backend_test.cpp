#include "backend.hpp"
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <cmath>
namespace sam = lingtu::localization::sam;
void require(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
sam::Cloud::Ptr room(const gtsam::Pose3& pose) {
  sam::Cloud::Ptr cloud(new sam::Cloud);
  for (int a=-18;a<=18;++a) for(int b=-18;b<=18;++b) {
    for(const auto& world : {gtsam::Point3(a*.15,b*.15,0),
                            gtsam::Point3(a*.15,-2.7,b*.15),
                            gtsam::Point3(2.7,a*.15,b*.15)}) {
      const auto local=pose.transformTo(world);
      pcl::PointXYZI p;p.x=local.x();p.y=local.y();p.z=local.z();p.intensity=1;
      cloud->push_back(p);
    }
  }
  return cloud;
}
int main() {
 try {
  sam::Config c;c.min_time_s=1000;c.voxel_m=.15;
  sam::Backend backend(c);
  gtsam::ISAM2Params p;p.relinearizeThreshold=.1;p.relinearizeSkip=1;
  gtsam::ISAM2 reference(p);
  gtsam::Pose3 previous;
  for(int i=0;i<8;++i) {
    gtsam::Pose3 pose(gtsam::Rot3::RzRyRx(.04*i,-.02*i,.06*i),{.3*i,.1*i,.2*i});
    sam::Cloud::Ptr cloud(new sam::Cloud);cloud->push_back(pcl::PointXYZI());
    backend.append({i+1.,pose,cloud});
    gtsam::NonlinearFactorGraph graph;gtsam::Values initial;initial.insert(i,pose);
    if(!i)graph.add(gtsam::PriorFactor<gtsam::Pose3>(0,pose,
      gtsam::noiseModel::Diagonal::Variances((gtsam::Vector6()<<1e-2,1e-2,M_PI*M_PI,1e8,1e8,1e8).finished())));
    else graph.add(gtsam::BetweenFactor<gtsam::Pose3>(i-1,i,previous.between(pose),
      gtsam::noiseModel::Diagonal::Variances((gtsam::Vector6()<<1e-6,1e-6,1e-6,1e-4,1e-4,1e-4).finished())));
    reference.update(graph,initial);reference.update();previous=pose;
    require(backend.poses().back().equals(reference.calculateEstimate<gtsam::Pose3>(i),1e-8),
      "native odometry graph differs from upstream factor/update sequence");
    require(backend.poses().back().equals(pose,1e-8),"height or attitude lost without loops");
  }
  require(backend.poses().size()==8 && backend.loops()==0,"sparse cloud disconnected odometry");
  bool rejected=false;try{backend.append({1.,previous,room(previous)});}catch(const std::invalid_argument&){rejected=true;}
  require(rejected && backend.poses().size()==8,"timestamp rollback mutated graph");
  // A sparse scan across a long pause must not lend its own points to a
  // historical target. The old +/-25 window accepted this self-match.
  sam::Config isolation; isolation.voxel_m=.15;
  sam::Backend isolated(isolation);
  sam::Cloud::Ptr sparse(new sam::Cloud); sparse->push_back(pcl::PointXYZI());
  isolated.append({1, gtsam::Pose3(), sparse});
  isolated.append({40, gtsam::Pose3(), room(gtsam::Pose3())});
  const auto self=isolated.append({41, gtsam::Pose3(), room(gtsam::Pose3())});
  require(!self.accepted && self.reason=="insufficient_geometry",
          "source/recent geometry contaminated historical loop target");
  require(self.target_frames==std::vector<std::size_t>{0},"temporal exclusion did not cover every target frame");
  sam::Backend revisit(isolation);
  revisit.append({1, gtsam::Pose3(), room(gtsam::Pose3())});
  const auto real=revisit.append({40, gtsam::Pose3(), room(gtsam::Pose3())});
  require(real.accepted && real.overlap>.99,"independent genuine revisit rejected");
  require(real.variance[0]!=real.variance[3],"angular and metric noise share ICP score");
  sam::Config overlap_config=isolation; overlap_config.max_fitness=1e6;
  sam::Backend partial(overlap_config);
  partial.append({1,gtsam::Pose3(),room(gtsam::Pose3())});
  auto unrelated=room(gtsam::Pose3());
  for(int x=0;x<60;++x) for(int y=0;y<60;++y) for(int z=0;z<2;++z) {
    pcl::PointXYZI p;p.x=20+x*.2;p.y=20+y*.2;p.z=10+z;
    unrelated->push_back(p);
  }
  const auto low_overlap=partial.append({40,gtsam::Pose3(),unrelated});
  require(!low_overlap.accepted && low_overlap.reason=="overlap_rejected",
          "low-overlap alignment accepted despite unbounded fitness allowance");
  std::ostringstream evidence; sam::writeLoopEvidence(evidence,isolation,revisit.records());
  require(evidence.str().find("target_frames")!=std::string::npos &&
          evidence.str().find("odom_variance")!=std::string::npos,"replay evidence/config missing");
  // Hold odometry fixed statistically, then inject a wrong closing edge.
  // Use the exact production robust model, compared with a Gaussian factor.
  auto false_loop_error=[&](bool robust) {
    gtsam::NonlinearFactorGraph graph; gtsam::Values initial;
    const auto prior=gtsam::noiseModel::Isotropic::Sigma(6,1e-4);
    const auto odom_noise=gtsam::noiseModel::Isotropic::Sigma(6,.1);
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(0,gtsam::Pose3(),prior));
    for(int i=0;i<6;++i) {
      initial.insert(i,gtsam::Pose3(gtsam::Rot3(),{double(i),0,0}));
      if(i)graph.add(gtsam::BetweenFactor<gtsam::Pose3>(i-1,i,
          gtsam::Pose3(gtsam::Rot3(),{1,0,0}),odom_noise));
    }
    std::array<double,6> variance;variance.fill(.01);
    gtsam::SharedNoiseModel noise=robust?sam::loopNoise(isolation,variance):odom_noise;
    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(0,5,gtsam::Pose3(),noise));
    auto estimate=gtsam::LevenbergMarquardtOptimizer(graph,initial).optimize();
    return std::abs(estimate.at<gtsam::Pose3>(5).x()-5);
  };
  const double gaussian_error=false_loop_error(false), robust_error=false_loop_error(true);
  std::cout<<"false_loop_gaussian_error="<<gaussian_error<<" robust_error="<<robust_error<<'\n';
  require(robust_error<gaussian_error*.5,"robust factor did not reduce false-loop damage");
  sam::Config loops;loops.voxel_m=.15;loops.min_time_s=30;loops.submap_half_window=2;
  sam::Backend closed(loops);
  for(int i=0;i<16;++i){
    const double x=(i<=7?i:15-i)*.3;
    gtsam::Pose3 truth(gtsam::Rot3(),{x,0,0});
    gtsam::Pose3 odom(gtsam::Rot3(),{x+i*.005,0,i*.003});
    const auto result=closed.append({i*3.+1,odom,room(truth)});
    if(result.accepted)std::cout<<"loop="<<i<<"->"<<result.previous<<" fitness="<<result.fitness<<'\n';
  }
  const auto& last=closed.poses().back();
  std::cout<<"loops="<<closed.loops()<<" final_xyz="<<last.translation().transpose()<<" error="<<closed.error()<<'\n';
  require(closed.loops()>0,"upstream ICP loop did not enter iSAM2");
  require(std::abs(last.z())<.045,"loop failed to reduce injected height drift");
  require(std::isfinite(closed.error()),"iSAM2 graph error nonfinite");
  std::cout<<"LIO-SAM native backend passed\n";
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
}

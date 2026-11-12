// Native global -> Executor -> SCAN -> Follower component probe.
// Internal integration is kinematic. --external consumes measured physics state.
#include "navigation/executor.hpp"
#include "planning/local/scan/grid.hpp"
#include "octoplanner3d_core.hpp"
#include "global_planner.h"
#include "collision/body_io.hpp"
#include "app/config/config.hpp"
#include "app/goal/task.hpp"

#include <chrono>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace c = lingtu::nav::collision;
namespace n = lingtu::nav::navigation;
namespace p = lingtu::nav::plan;
namespace r = octoplanner3d::runtime;
using Clock = std::chrono::steady_clock;

std::shared_ptr<c::Body> loadBody(const std::string& path) {
  std::ifstream in(path);
  return c::readBody(in);
}

std::vector<p::GlobalPlanPoint> loadRoute(const std::string& path) {
  std::ifstream in(path); std::size_t count;
  if(!(in>>count) || count<2) throw std::runtime_error("route_missing");
  std::vector<p::GlobalPlanPoint> points;
  for(std::size_t i=0;i<count;++i) {
    p::GlobalPlanPoint point; double yaw;
    in>>point.x>>point.y>>point.z>>yaw; point.yaw=yaw;
    if(!in) throw std::runtime_error("route_parse");
    points.push_back(point);
  }
  return points;
}

void generateMap(const std::string& scene, const std::string& path) {
  std::ifstream in(scene); double resolution; c::Vec3 low,high; std::size_t count;
  in>>resolution>>low.x>>low.y>>low.z>>high.x>>high.y>>high.z>>count;
  struct Box {c::Vec3 minimum,maximum;};
  std::vector<Box> obstacles(count);
  for(auto& box:obstacles) in>>box.minimum.x>>box.minimum.y>>box.minimum.z
      >>box.maximum.x>>box.maximum.y>>box.maximum.z;
  if(!in || resolution<=0) throw std::runtime_error("scene_parse");
  octomap::OcTree tree(resolution);
  for(int x=int(std::floor(low.x/resolution));x<int(std::round(high.x/resolution));++x)
    for(int y=int(std::floor(low.y/resolution));y<int(std::round(high.y/resolution));++y)
      for(int z=int(std::floor(low.z/resolution));z<int(std::round(high.z/resolution));++z) {
        const c::Vec3 q{(x+.5)*resolution,(y+.5)*resolution,(z+.5)*resolution};
        bool occupied=false;
        for(const auto& b:obstacles) occupied=occupied || (q.x>=b.minimum.x && q.x<b.maximum.x &&
            q.y>=b.minimum.y && q.y<b.maximum.y && q.z>=b.minimum.z && q.z<b.maximum.z);
        auto* node=tree.updateNode(q.x,q.y,q.z,occupied,true);
        node->setLogOdds(occupied?3.5f:-2.f);
      }
  tree.updateInnerOccupancy(); tree.prune();
  if(!tree.write(path)) throw std::runtime_error("map_write");
}

nav_kernel::LocalCollisionMapView bitmap(const octomap::OcTree& tree, bool legacy) {
  nav_kernel::LocalCollisionMapView view;
  tree.getMetricMin(view.aabbMin.x,view.aabbMin.y,view.aabbMin.z);
  tree.getMetricMax(view.aabbMax.x,view.aabbMax.y,view.aabbMax.z);
  view.resolution=tree.getResolution();
  view.sizeX=int(std::llround((view.aabbMax.x-view.aabbMin.x)/view.resolution));
  view.sizeY=int(std::llround((view.aabbMax.y-view.aabbMin.y)/view.resolution));
  view.sizeZ=int(std::llround((view.aabbMax.z-view.aabbMin.z)/view.resolution));
  const auto bytes=(view.cellCount()+7)/8;
  auto occupied=std::make_shared<std::vector<std::uint8_t>>(bytes,0);
  auto free=std::make_shared<std::vector<std::uint8_t>>(bytes,0);
  const auto set=[](auto& bits,std::size_t i){bits[i/8]|=std::uint8_t(1U<<(i%8));};
  for(int z=0;z<view.sizeZ;++z) for(int y=0;y<view.sizeY;++y) for(int x=0;x<view.sizeX;++x) {
    const std::size_t i=(std::size_t(z)*view.sizeY+y)*view.sizeX+x;
    const double qx=view.aabbMin.x+(x+.5)*view.resolution;
    const double qy=view.aabbMin.y+(y+.5)*view.resolution;
    const double qz=view.aabbMin.z+(z+.5)*view.resolution;
    const auto* node=tree.search(qx,qy,qz);
    if(node) set(tree.isNodeOccupied(node)?*occupied:*free,i);
  }
  auto inflated=std::make_shared<std::vector<std::uint8_t>>(*occupied);
  if(legacy) {
    const int cells=int(std::ceil(.25/view.resolution));
    for(int z=0;z<view.sizeZ;++z) for(int y=0;y<view.sizeY;++y) for(int x=0;x<view.sizeX;++x) {
      const std::size_t i=(std::size_t(z)*view.sizeY+y)*view.sizeX+x;
      if(!((*occupied)[i/8] & (1U<<(i%8)))) continue;
      for(int dz=-cells;dz<=cells;++dz) for(int dy=-cells;dy<=cells;++dy) for(int dx=-cells;dx<=cells;++dx) {
        if((dx*dx+dy*dy)*view.resolution*view.resolution>.25*.25+1e-9) continue;
        const int xx=x+dx,yy=y+dy,zz=z+dz;
        if(xx>=0 && yy>=0 && zz>=0 && xx<view.sizeX && yy<view.sizeY && zz<view.sizeZ)
          set(*inflated,(std::size_t(zz)*view.sizeY+yy)*view.sizeX+xx);
      }
    }
  }
  view.inflatedStorage=inflated; view.inflatedBits=inflated->data(); view.inflatedBytes=bytes;
  view.measuredOccupiedStorage=occupied; view.knownFreeStorage=free;
  view.complete=view.live=true; view.resetEpoch=view.generation=view.observationSequence=1;
  return view;
}

struct ObservedBox {
  nav_kernel::Vec3 center, half, velocity;
};

nav_kernel::LocalCollisionMapView observedMap(
    const nav_kernel::LocalCollisionMapView& base, const std::vector<ObservedBox>& boxes) {
  if (boxes.empty()) return base;
  auto view=base;
  auto occupied=std::make_shared<std::vector<std::uint8_t>>(*base.measuredOccupiedStorage);
  auto known=std::make_shared<std::vector<std::uint8_t>>(*base.knownFreeStorage);
  for(const auto& box:boxes) {
    const auto first=[&](double c,double h,double origin) {
      return int(std::floor((c-h-origin)/base.resolution));
    };
    const auto last=[&](double c,double h,double origin) {
      return int(std::ceil((c+h-origin)/base.resolution));
    };
    for(int z=std::max(0,first(box.center.z,box.half.z,base.aabbMin.z));
        z<std::min(base.sizeZ,last(box.center.z,box.half.z,base.aabbMin.z));++z)
      for(int y=std::max(0,first(box.center.y,box.half.y,base.aabbMin.y));
          y<std::min(base.sizeY,last(box.center.y,box.half.y,base.aabbMin.y));++y)
        for(int x=std::max(0,first(box.center.x,box.half.x,base.aabbMin.x));
            x<std::min(base.sizeX,last(box.center.x,box.half.x,base.aabbMin.x));++x) {
          const auto index=(std::size_t(z)*base.sizeY+y)*base.sizeX+x;
          const auto mask=std::uint8_t(1U<<(index%8));
          (*occupied)[index/8]|=mask;
          (*known)[index/8]&=std::uint8_t(~mask);
        }
  }
  view.measuredOccupiedStorage=occupied;view.knownFreeStorage=known;
  view.inflatedStorage=occupied;view.inflatedBits=occupied->data();
  return view;
}

int main(int argc,char** argv) try {
  std::map<std::string,std::string> args;
  for(int i=1;i<argc;++i) {
    const std::string key=argv[i];
    if(key=="--keep-turn-stages")
      throw std::runtime_error("--keep-turn-stages is retired: missing support metadata no longer forces intermediate stops; historical stop-stage results require their original executable");
    if(key=="--legacy" || key=="--external" || key=="--use-route" || key=="--obb") args[key]="1";
    else if(i+1<argc) args[key]=argv[++i];
    else throw std::runtime_error("argument_missing_value");
  }
  const bool legacy=args.count("--legacy"),external=args.count("--external");
  const std::filesystem::path out=args.at("--out"); std::filesystem::create_directories(out);
  std::shared_ptr<const c::Body> body=loadBody(args.at("--body"));
  const bool obb=args.count("--obb");
  if(obb && legacy) throw std::runtime_error("obb_and_legacy_are_exclusive");
  char config_name[]="navd",planner_option[]="--local-planner",planner_name[]="scan";
  char* config_argv[]={config_name,planner_option,planner_name};
  const auto native_config=legacy?lingtu::nav::endpoint::CliConfig{}
      :lingtu::nav::endpoint::parseArgs(3,config_argv);
  if(!legacy && !obb) {
    auto selected=std::make_shared<c::Body>(*body);
    selected->margin_m=0.0;
    selected->horizontal_margin_m=native_config.teleop_obstacle_margin_m;
    body=std::move(selected);
  }
  if(obb) {
    body=lingtu::nav::endpoint::buildNavigationBody(native_config);
    if(body->parts.size()!=1 || body->parts.front().shape!=c::Shape::Box)
      throw std::runtime_error("obb_requires_single_box_navigation_geometry");
  }
  if(!legacy) {
    std::ofstream geometry(out/"navigation-body.json");
    geometry<<std::setprecision(12)<<"{\"source\":"
      <<std::quoted(obb?"native buildNavigationBody from parsed native config":"--body BODY2 snapshot")
      <<",\"model_file\":"<<std::quoted(obb?native_config.navigation_body_file:args.at("--body"))
      <<",\"collision_model\":"<<std::quoted(obb?"single_obb":"articulated_parts")
      <<",\"parts\":"<<body->parts.size()<<",\"margin_m\":"<<body->margin_m
      <<",\"horizontal_margin_m\":"<<body->horizontal_margin_m;
    if(obb) {
      const auto& box=body->parts.front();
      geometry<<",\"center\":["<<box.center.x<<','<<box.center.y<<','<<box.center.z
        <<"],\"half_size\":["<<box.half_size.x<<','<<box.half_size.y<<','<<box.half_size.z<<']';
    }
    geometry<<"}\n";
  }
  auto points=loadRoute(args.at("--route"));
  if(args.count("--scene")) generateMap(args.at("--scene"),args.at("--map"));
  auto abstract=std::unique_ptr<octomap::AbstractOcTree>(octomap::AbstractOcTree::read(args.at("--map")));
  auto* raw=dynamic_cast<octomap::OcTree*>(abstract.get());
  if(!raw) throw std::runtime_error("map_not_octree");
  abstract.release(); auto tree=std::shared_ptr<octomap::OcTree>(raw);
  const auto start=points.front(),goal=points.back();
  p::GlobalPlanRequest request; request.start=start;request.goal=goal;
  request.map_identity={"body_execution",1,"map"};
  auto& options=request.options;
  options.robot_radius=.25;options.body_clearance_below_m=.1;options.body_clearance_above_m=.25;
  options.support_height_m=start.z;options.support_height_tolerance_m=.05;
  options.max_iterations=50000;options.max_slope=std::tan(3.141592653589793/6.);
  options.enable_preblocked_costmap=false;options.obstacle_clearance_weight=0;
  options.floor_change_penalty=0;options.same_floor_preference=false;
  options.terminal_goal_tolerance_m=options.terminal_goal_xy_tolerance_m=options.terminal_goal_z_tolerance_m=.03;
  if(!legacy) {
    options=native_config.octoplanner_options;
  }
  p::GlobalPlanResult global;
  bool global_timed_out=false;
  if(args.count("--use-route")) {global.ok=true;global.path=points;}
  else {
    r::PlannerSession session;
    const auto deadline=Clock::now()+lingtu::nav::endpoint::kDefaultGlobalPlanTimeout;
    const auto cancelled=[&] {
      global_timed_out=global_timed_out || Clock::now()>=deadline;
      return global_timed_out;
    };
    global=session.run(args.at("--map"),request.map_identity,request,cancelled);
    if(cancelled()) {
      global.ok=global.reached_goal=false;global.cancelled=true;global.path.clear();
      global.failure_reason=lingtu::nav::endpoint::kGlobalPlanTimeoutReason;
    }
  }
  std::ofstream summary(out/"summary.json"); summary<<std::setprecision(12)<<std::boolalpha;
  if(!global.ok) {
    summary<<"{\"global_ok\":false,\"reason\":"<<std::quoted(global.failure_reason)
      <<",\"global_ms\":"<<global.elapsed_ms<<",\"global_timed_out\":"<<global_timed_out
      <<",\"planning_timeout_ms\":"<<lingtu::nav::endpoint::kDefaultGlobalPlanTimeout.count()
      <<",\"execution_started\":false}\n";
    std::cout<<"@command {\"vx\":0,\"vy\":0,\"wz\":0,\"done\":true,\"status\":\"global_failed\",\"reason\":"
      <<std::quoted(global.failure_reason)<<"}"<<std::endl; return 0;
  }
  std::ofstream route_file(out/"global.csv");route_file<<"x,y,z,yaw\n"<<std::setprecision(12);
  const double yaw_tolerance=args.count("--yaw-tolerance-deg")
      ? std::stod(args.at("--yaw-tolerance-deg"))*std::acos(-1.)/180. : .025;
  if(!std::isfinite(yaw_tolerance) || yaw_tolerance<=0 || yaw_tolerance>std::acos(-1.))
    throw std::runtime_error("invalid_yaw_tolerance");
  n::Route route;route.goalToleranceM=.025;route.yawToleranceRad=yaw_tolerance;route.maxSpeedMps=.5;
  // The ablation retains the original checked stages with the same binary,
  // map and control limits; it does not disable any final support check.
  if(options.ground_support) {
    route.supportHeightM=options.support_height_m;
    route.supportHeightToleranceM=options.support_height_tolerance_m;
  }
  if(!legacy) route.body=body;
  for(const auto& point:global.path) {
    route.points.push_back({point.x,point.y,point.z});
    route_file<<point.x<<','<<point.y<<','<<point.z<<','<<point.yaw.value_or(0)<<'\n';
  }
  // The live video branch reads this complete route after the first response.
  route_file.flush();
  nav_kernel::LocalPlannerParams params;params.backend=nav_kernel::LocalPlannerBackend::Scan;
  params.scan.voxelResolution=tree->getResolution();params.scan.cylinderRadius=.25;params.scan.cylinderOffset=.19;
  params.scan.bodyClearanceBelow=.25;params.scan.bodyClearanceAbove=.25;params.scan.maxVelocity=.5;
  params.maxSpeed=params.autonomySpeed=.5;params.useTraversabilityCost=false;
  n::ExecutorConfig config;config.planning_frame=n::PlanningFrame::Map;
  config.max_speed=.5;config.follower.maxSpeed=.5;config.follower.nominalDt=.02;
  config.goal_height_tolerance_m=.03;
  if(!legacy) {
    params=lingtu::nav::endpoint::buildLocalPlannerParams(native_config);
    config=lingtu::nav::endpoint::buildExecutorConfig(native_config);
    config.body=body;
    route.maxSpeedMps=config.max_speed;
  }
  const double configured_resolution=params.scan.voxelResolution;
  // The input bitmap's voxel size is the only nonlegacy fixture override.
  params.scan.voxelResolution=tree->getResolution();
  const auto safety_config=lingtu::nav::endpoint::commandSafetyConfig(native_config);
  const double final_reaction=!legacy ? 1. / native_config.tick_hz : .1;
  const double final_linear_deceleration=!legacy?std::min(native_config.velocity_smoother.x.deceleration,
      native_config.velocity_smoother.y.deceleration):.5;
  const double final_yaw_deceleration=!legacy?native_config.velocity_smoother.yaw.deceleration:1.;
  {
    std::ofstream resolved(out/"resolved-control.json");
    resolved<<std::setprecision(12)<<"{\"tuning_source\":"
      <<std::quoted(legacy?"legacy_fixture":"native_config")
      <<",\"max_speed_mps\":"<<config.max_speed
      <<",\"max_accel_mps2\":"<<config.follower.maxAccel
      <<",\"nominal_dt_s\":"<<config.follower.nominalDt
      <<",\"planner_max_velocity_mps\":"<<params.scan.maxVelocity
      <<",\"planner_max_accel_mps2\":"<<params.scan.maxAcceleration
      <<",\"configured_voxel_resolution_m\":"<<configured_resolution
      <<",\"input_map_resolution_m\":"<<tree->getResolution()
      <<",\"voxel_resolution_m\":"<<params.scan.voxelResolution
      <<",\"reaction_s\":"<<final_reaction<<",\"linear_deceleration_mps2\":"<<final_linear_deceleration
      <<",\"yaw_deceleration_radps2\":"<<final_yaw_deceleration
      <<",\"support_height_m\":"<<options.support_height_m
      <<",\"support_height_tolerance_m\":"<<options.support_height_tolerance_m
      <<",\"global_max_iterations\":"<<options.max_iterations
      <<",\"final_geometry_braking_included\":true,\"full_final_control_dds_driver_included\":false}\n";
  }
  nav_kernel::LocalPlanner configured_planner(params);
  if(!configured_planner.configure("")) throw std::runtime_error("scan_configure");
  n::Executor executor(config,std::move(configured_planner));executor.setRoute(std::move(route));
  const auto base_view=bitmap(*tree,legacy);
  auto view=base_view;
  global_planner::OctoPlanner3D support; global_planner::PlannerConfig support_config;
  support_config.support_height_m=options.support_height_m;
  support_config.support_height_tolerance_m=options.support_height_tolerance_m;
  support_config.max_slope=options.max_slope;support_config.max_step_height=options.max_step_height;
  support_config.ground_support=options.ground_support;
  support_config.direct_support_only=options.direct_support_only;
  support_config.support_depth_cells=options.support_depth_cells;
  support.setConfig(support_config);support.setOctomap(tree);support.preparePoseQueries({start.x,start.y,start.z});
  std::ofstream states(out/"states.csv");
  std::ofstream events(out/"attempts.csv");
  // This is a sampled latest-attempt view, not a lossless worker event stream.
  events<<"observed_t,observed_route_generation,attempt_id,attempt_t,success,stage,reason,optimizer_code,collision_x,collision_y,collision_z\n"<<std::setprecision(12);
  std::uint64_t last_attempt=0;
  double last_attempt_time=-1;
  states<<"t,x,y,z,yaw,vx,vy,wz,cmd_vx,cmd_vy,cmd_wz,stage,stages,trajectory,tracking_error,tick_ms,final_blocked,reason,scan_reason,body_reason,body_link,contact_parts,spline_time_s,spline_duration_s,plan_input_t,plan_ms,local_start_x,local_start_y,local_start_z,executor_ms,final_grid_ms,final_body_ms,final_measured_braking_ms,final_command_braking_ms,final_support_ms,raw_cmd_vx,raw_cmd_vy,raw_cmd_wz,final_braking_scale\n"<<std::setprecision(12);
  nav_kernel::Pose pose;pose.position={start.x,start.y,start.z};pose.yaw=*start.yaw;
  nav_kernel::Vec3 velocity{};double yaw_rate=0,t=1,first_t=-1,travel=0,max_tick=0;
  const double dt=!legacy?1./native_config.tick_hz:.02;
  const double max_s=args.count("--seconds")?std::stod(args["--seconds"]):45.;
  int ticks=0,blocked=0,tracking_ticks=0;bool reached=false;std::string reason="timeout";
  std::map<std::string,int> reasons;
  auto previous=pose;
  for(;external || t<1+max_s;t+=dt) {
    std::shared_ptr<const c::Body> observed_body;
    std::vector<ObservedBox> dynamic_boxes;
    if(external) {
      std::string line;if(!std::getline(std::cin,line)) {reason="input_closed";break;}
      std::istringstream in(line);in>>t>>pose.position.x>>pose.position.y>>pose.position.z>>pose.yaw
        >>velocity.x>>velocity.y>>velocity.z>>yaw_rate;
      if(!in) throw std::runtime_error("state_parse");
      std::string marker;
      if(in>>marker) {
        if(marker=="R" && obb) {
          double roll,pitch;in>>roll>>pitch;
          if(!in || !std::isfinite(roll) || !std::isfinite(pitch))
            throw std::runtime_error("observed_body_attitude");
          auto measured=std::make_shared<c::Body>(*body);
          const double cr=std::cos(roll),sr=std::sin(roll),cp=std::cos(pitch),sp=std::sin(pitch);
          auto& part=measured->parts.front();
          part.rotation={cp,sp*sr,sp*cr, 0,cr,-sr, -sp,cp*sr,cp*cr};
          const auto center=part.center;
          part.center={cp*center.x+sp*sr*center.y+sp*cr*center.z,
                       cr*center.y-sr*center.z,
                       -sp*center.x+cp*sr*center.y+cp*cr*center.z};
          observed_body=std::move(measured);
        } else {
        std::size_t count=0;
        if(marker!="P" || !(in>>count) || count!=body->parts.size())
          throw std::runtime_error("observed_body_count");
        auto measured=std::make_shared<c::Body>(*body);
        for(auto& part:measured->parts) {
          in>>part.center.x>>part.center.y>>part.center.z;
          for(auto& value:part.rotation) in>>value;
          bool reserved = false;
          in>>reserved;
          if(!in)
            throw std::runtime_error("observed_body_pose");
        }
        observed_body=std::move(measured);
        }
        if(in>>marker) {
          std::size_t count=0;
          if(marker!="D" || !(in>>count) || legacy)
            throw std::runtime_error("dynamic_obstacle_packet");
          dynamic_boxes.resize(count);
          for(auto& box:dynamic_boxes) {
            in>>box.center.x>>box.center.y>>box.center.z
              >>box.half.x>>box.half.y>>box.half.z
              >>box.velocity.x>>box.velocity.y>>box.velocity.z;
            if(!in || !std::isfinite(box.center.x) || !std::isfinite(box.center.y) ||
                !std::isfinite(box.center.z) || !std::isfinite(box.half.x) ||
                !std::isfinite(box.half.y) || !std::isfinite(box.half.z) ||
                !std::isfinite(box.velocity.x) || !std::isfinite(box.velocity.y) ||
                !std::isfinite(box.velocity.z) || box.half.x<=0 || box.half.y<=0 || box.half.z<=0)
              throw std::runtime_error("dynamic_obstacle_geometry");
          }
        }
      }
    }
    const auto tick_start=Clock::now();
    if(first_t<0) first_t=t;
    travel+=nav_kernel::distance3D(previous.position,pose.position);previous=pose;
    n::ExecutionInput input;input.mapBody=input.odomBody=pose;input.timestampS=t;
    auto& obs=input.observation;obs.frame_epoch=1;
    // Repeat the unchanged synthetic scene observation at 5 Hz. A static
    // content generation must not freeze the recovery observation barrier.
    obs.cloud_generation=1+static_cast<std::uint64_t>(std::floor((t-first_t)*5.+1e-8));
    obs.odom_stamp_s=t;
    obs.cloud_stamp_s=first_t+static_cast<double>(obs.cloud_generation-1)*.2;
    if(!dynamic_boxes.empty()) {
      obs.cloud_generation=1+static_cast<std::uint64_t>(ticks);
      obs.cloud_stamp_s=t;
    }
    obs.body_velocity_valid=true;
    const double cs=std::cos(pose.yaw),sn=std::sin(pose.yaw);
    obs.body_linear_velocity={cs*velocity.x+sn*velocity.y,-sn*velocity.x+cs*velocity.y,velocity.z};
    obs.body_yaw_rate=yaw_rate;obs.clock_mode=nav_kernel::PlanClockMode::External;
    if(!legacy) obs.body_geometry=observed_body;
    // Ground-truth fixture evidence: replace only the live overlay each frame,
    // preserving the static observed map and releasing an actor's old cells.
    view=observedMap(base_view,dynamic_boxes);
    std::vector<nav_kernel::PredictedObstacle> predictions;
    constexpr double prediction_horizon=2.;
    for(const auto& box:dynamic_boxes) {
      const nav_kernel::Vec3 end{box.center.x+prediction_horizon*box.velocity.x,
          box.center.y+prediction_horizon*box.velocity.y,box.center.z+prediction_horizon*box.velocity.z};
      predictions.push_back({box.center,end,std::hypot(box.half.x,box.half.y),
          std::min(box.center.z,end.z)-box.half.z,std::max(box.center.z,end.z)+box.half.z});
    }
    obs.predictions={predictions.data(),predictions.size(),t+.1,t,prediction_horizon};
    view.observationSequence=obs.cloud_generation;
    view.stampS=view.receiveStampS=obs.cloud_stamp_s;obs.collision=view;
    const auto executor_start=Clock::now();
    auto output=executor.tick(input);const auto raw_cmd=output.cmd_vel;auto cmd=raw_cmd;
    const double executor_ms=std::chrono::duration<double,std::milli>(Clock::now()-executor_start).count();
    const auto& attempt=output.local_planner_debug.scanAttempt;
    if(attempt.attempted && (attempt.attemptId!=last_attempt || attempt.timestampS!=last_attempt_time)) {
      events<<t<<','<<output.route_generation<<','<<attempt.attemptId<<','<<attempt.timestampS<<','
        <<attempt.success<<','<<attempt.stage<<','<<attempt.reason<<','<<attempt.optimizerReturnCode<<','
        <<attempt.collisionPosition.x<<','<<attempt.collisionPosition.y<<','<<attempt.collisionPosition.z<<'\n';
      last_attempt=attempt.attemptId;last_attempt_time=attempt.timestampS;
    }
    nav_kernel::LocalPlanRequest safety;safety.robot.pose=pose;safety.body=body;
    safety.observedBody=observed_body;
    safety.environment.collision=view;safety.environment.predictions=obs.predictions;
    safety.clock.timestampS=t;
    const auto grid_start=Clock::now();
    nav_kernel::local::scan::Grid grid(params,safety);
    const double final_grid_ms=std::chrono::duration<double,std::milli>(Clock::now()-grid_start).count();
    const auto& current_body=observed_body?*observed_body:*body;
    const auto body_start=Clock::now();
    const int actual_occupancy=grid.inflatedOccupancy(pose.position,pose.yaw);
    const double final_body_ms=std::chrono::duration<double,std::milli>(Clock::now()-body_start).count();
    const auto contact_parts=std::count_if(current_body.parts.begin(),current_body.parts.end(),
        [](const auto& part){return part.support_contact;});
    const nav_kernel::Twist measured{obs.body_linear_velocity.x,obs.body_linear_velocity.y,yaw_rate};
    double final_measured_braking_ms=0,final_command_braking_ms=0,final_support_ms=0;
    bool safe=actual_occupancy==0 && grid.valid();
    double final_scale=safe?1.:0.;
    if(safe) {
      const auto started=Clock::now();
      safe=grid.brakingOccupancy(pose,measured,final_reaction,final_linear_deceleration,final_yaw_deceleration)==0;
      final_measured_braking_ms=std::chrono::duration<double,std::milli>(Clock::now()-started).count();
    }
    if(safe) {
      const auto started=Clock::now();
      final_scale=grid.brakingScale(pose,cmd,final_reaction,final_linear_deceleration,final_yaw_deceleration);
      safe=final_scale>0.;
      cmd.vx*=final_scale;cmd.vy*=final_scale;cmd.wz*=final_scale;
      final_command_braking_ms=std::chrono::duration<double,std::milli>(Clock::now()-started).count();
    }
    if(safe) {
      const auto started=Clock::now();
      const nav_kernel::Vec3 next{pose.position.x+(cs*cmd.vx-sn*cmd.vy)*dt,
        pose.position.y+(sn*cmd.vx+cs*cmd.vy)*dt,pose.position.z};
      safe=support.supportConnectionFree({pose.position.x,pose.position.y,pose.position.z},{next.x,next.y,next.z});
      final_support_ms=std::chrono::duration<double,std::milli>(Clock::now()-started).count();
    }
    if(!safe) {cmd={};final_scale=0.;++blocked;}
    executor.reportFinalMotionBlocked(!safe,t);
    const double tick_ms=std::chrono::duration<double,std::milli>(Clock::now()-tick_start).count();
    max_tick=std::max(max_tick,tick_ms);++ticks;++reasons[output.reason];
    if(output.tracking.active && !output.trajectory_frozen) ++tracking_ticks;
    const auto local_start=output.local_path_map.empty()
        ? nav_kernel::Vec3{std::nan(""),std::nan(""),std::nan("")}
        : output.local_path_map.front();
    states<<t<<','<<pose.position.x<<','<<pose.position.y<<','<<pose.position.z<<','<<pose.yaw<<','
      <<velocity.x<<','<<velocity.y<<','<<yaw_rate<<','<<cmd.vx<<','<<cmd.vy<<','<<cmd.wz<<','
      <<output.pose_stage_index<<','<<output.pose_stage_count<<','<<output.tracking.trajectoryId<<','
      <<output.tracking.positionErrorM<<','<<tick_ms<<','<<!safe<<','<<output.reason<<','<<output.local_planner_debug.searchReason<<','
      <<(actual_occupancy==1 ? "occupied_body" : grid.reason())<<",,"<<contact_parts<<','
      <<output.tracking.executionTimeS<<','<<output.tracking.durationS<<','
      <<attempt.timestampS<<','<<output.local_planner_debug.planningTimeMs<<','
      <<local_start.x<<','<<local_start.y<<','<<local_start.z<<','<<executor_ms<<','<<final_grid_ms<<','
      <<final_body_ms<<','<<final_measured_braking_ms<<','<<final_command_braking_ms<<','<<final_support_ms<<','
      <<raw_cmd.vx<<','<<raw_cmd.vy<<','<<raw_cmd.wz<<','<<final_scale<<'\n';
    reached=output.goal_reached && safe;
    if(external) std::cout<<std::setprecision(12)<<"@command {\"vx\":"<<cmd.vx<<",\"vy\":"<<cmd.vy
      <<",\"wz\":"<<cmd.wz<<",\"done\":"<<(reached?"true":"false")
      <<",\"status\":"<<std::quoted(output.reason)<<",\"reason\":"<<std::quoted(safe?output.reason:"final_body_or_support_blocked")<<"}"<<std::endl;
    if(reached) {reason="goal_reached";break;}
    if(!external) {
      velocity={cs*cmd.vx-sn*cmd.vy,sn*cmd.vx+cs*cmd.vy,0};yaw_rate=cmd.wz;
      pose.position.x+=velocity.x*dt;pose.position.y+=velocity.y*dt;
      pose.yaw=nav_kernel::normalizeAngle(pose.yaw+cmd.wz*dt);
      std::this_thread::sleep_until(tick_start+std::chrono::milliseconds(20));
    }
  }
  summary<<"{\"global_ok\":true,\"global_ms\":"<<global.elapsed_ms<<",\"global_points\":"<<global.path.size()
    <<",\"execution_started\":true,\"mode\":"<<std::quoted(external?"external_physics_state":"kinematic_closed_loop")
    <<",\"legacy\":"<<legacy<<",\"reached\":"<<reached<<",\"reason\":"<<std::quoted(reason)
    <<",\"collision_model\":"<<std::quoted(obb?"single_obb":"articulated_parts")
    <<",\"body_parts\":"<<body->parts.size()<<",\"yaw_tolerance_rad\":"<<yaw_tolerance
    <<",\"max_speed_mps\":"<<config.max_speed<<",\"max_accel_mps2\":"<<config.follower.maxAccel
    <<",\"final_geometry_braking_included\":true,\"full_final_control_dds_driver_included\":false"
    <<",\"dynamic_evidence\":\"optional ground-truth fixture boxes; live occupancy and timed predictions; no sensor pipeline\""
    <<",\"elapsed_sim_s\":"<<t-first_t<<",\"travel_m\":"<<travel<<",\"ticks\":"<<ticks
    <<",\"final_blocked_ticks\":"<<blocked<<",\"tracking_ticks\":"<<tracking_ticks
    <<",\"max_tick_ms\":"<<max_tick<<",\"goal_error_m\":"<<std::hypot(pose.position.x-goal.x,pose.position.y-goal.y)
    <<",\"yaw_error_rad\":"<<std::abs(c::shortestYawDelta(pose.yaw,*goal.yaw))<<",\"states\":{";
  bool first=true;for(const auto& entry:reasons) {if(!first)summary<<',';first=false;summary<<std::quoted(entry.first)<<':'<<entry.second;}
  summary<<"}}\n";
  return 0;
} catch(const std::exception& error) {std::cerr<<"body_execution: "<<error.what()<<'\n';return 2;}

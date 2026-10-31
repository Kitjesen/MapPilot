#include "navigation/pose_route.hpp"

#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <limits>

namespace lingtu::nav::navigation {

namespace {
bool forwardEdge(const nav_kernel::Vec3& from, const nav_kernel::Vec3& to,
                 double yaw, double tolerance, double resolution = 0) {
  const double length=std::hypot(to.x-from.x,to.y-from.y);
  if(length<=1e-8) return false;
  const double error=std::abs(collision::shortestYawDelta(yaw,
      std::atan2(to.y-from.y,to.x-from.x)));
  return error<M_PI/2 && length*std::sin(error)<=
      resolution/std::sqrt(2.)+length*std::sin(std::min(tolerance,M_PI/2))+1e-8;
}

bool forwardStage(const PoseRouteStage& stage, double tolerance, double resolution=0) {
  if(stage.rotation || stage.points.size()<2) return false;
  for(std::size_t i=1;i<stage.points.size();++i)
    if(!forwardEdge(stage.points[i-1],stage.points[i],stage.yaw,tolerance,resolution)) return false;
  return true;
}
}  // namespace

std::vector<PoseRouteStage> compilePoseRoute(const std::vector<PoseRoutePoint>& points,
                                          double yawToleranceRad) {
  std::vector<PoseRouteStage> stages;
  if(!std::isfinite(yawToleranceRad) || yawToleranceRad<0)
    throw std::invalid_argument("pose route yaw tolerance must be finite and nonnegative");
  for (const auto& point : points) {
    if (!std::isfinite(point.position.x) || !std::isfinite(point.position.y) ||
        !std::isfinite(point.position.z) || !std::isfinite(point.yaw))
      throw std::invalid_argument("pose route must contain finite positions and yaw");
  }
  if (points.empty()) return stages;
  stages.push_back({{points.front().position}, points.front().yaw, true});
  for (std::size_t i = 1; i < points.size(); ++i) {
    const auto& from = points[i - 1];
    const auto& to = points[i];
    const bool translated = nav_kernel::distance3D(from.position, to.position) > 1e-8;
    const bool turned = std::abs(std::atan2(std::sin(to.yaw - from.yaw),
                                          std::cos(to.yaw - from.yaw))) > 1e-8;
    if (translated && turned) {
      if(!forwardEdge(from.position,to.position,from.yaw,yawToleranceRad) ||
         !forwardEdge(from.position,to.position,to.yaw,yawToleranceRad))
        throw std::invalid_argument("coupled pose route requires forward tangent headings");
      if(stages.back().rotation ||
         (!stages.back().tangentHeading && !forwardStage(stages.back(),yawToleranceRad)))
        stages.push_back({{from.position,to.position},to.yaw,false,true});
      else {
        stages.back().points.push_back(to.position);
        stages.back().yaw=to.yaw;
        stages.back().tangentHeading=true;
      }
    } else if (turned) {
      stages.push_back({{to.position}, to.yaw, true});
    } else if (translated) {
      if (stages.back().rotation || (stages.back().tangentHeading &&
          !forwardEdge(from.position,to.position,to.yaw,yawToleranceRad)))
        stages.push_back({{from.position, to.position}, to.yaw, false});
      else
        stages.back().points.push_back(to.position);
    }
  }
  return stages;
}

std::optional<PoseRouteBlend> poseRouteBlend(
    const std::vector<PoseRouteStage>& stages, std::size_t index, double yawToleranceRad,
    double gridResolutionM) {
  if(!std::isfinite(yawToleranceRad) || yawToleranceRad<0) return {};
  if(!std::isfinite(gridResolutionM) || gridResolutionM<0) return {};
  const double lateralResolution=gridResolutionM/std::sqrt(2.0);
  if (index+2 >= stages.size()) return {};
  const auto& first = stages[index];
  if (first.rotation || first.tangentHeading || first.points.size()<2 || !stages[index+1].rotation) return {};
  const auto length = [&](const PoseRouteStage& leg) {
    const auto& points=leg.points;
    if(leg.rotation || leg.tangentHeading || points.size()<2) return -1.0;
    const auto& a=points.front(); const auto& b=points.back();
    const double total=nav_kernel::distance3D(a,b);
    if (total<1e-8 || std::abs(a.z-b.z)>1e-6) return -1.0;
    for (const auto& p:points) {
      const double progress=((p.x-a.x)*(b.x-a.x)+(p.y-a.y)*(b.y-a.y))/total;
      const double lateral=std::abs((p.x-a.x)*(b.y-a.y)-(p.y-a.y)*(b.x-a.x))/total;
      if (std::abs(p.z-a.z)>1e-6 || lateral>lateralResolution+1e-6 ||
          progress< -1e-6 || progress>total+1e-6)
        return -1.0;
    }
    const double error=std::abs(collision::shortestYawDelta(leg.yaw,
        std::atan2(b.y-a.y,b.x-a.x)));
    if(error>=M_PI/2 || total*std::sin(error)>
        lateralResolution+total*std::sin(std::min(yawToleranceRad,M_PI/2))+1e-6) return -1.0;
    return total;
  };
  struct Corner { double turn, trim, radius; };
  std::vector<std::size_t> legs{index};
  std::vector<Corner> corners;
  double incoming=length(first);
  if(incoming<0) return {};
  while(true) {
    const auto& before=stages[legs.back()];
    std::size_t next=legs.back()+1;
    double totalTurn=0,previousYaw=before.yaw;
    bool valid=true;
    while(next<stages.size() && stages[next].rotation) {
      const auto& turn=stages[next];
      if(turn.points.size()!=1 ||
         nav_kernel::distance3D(turn.points.front(),before.points.back())>1e-8) {valid=false;break;}
      const double step=collision::shortestYawDelta(previousYaw,turn.yaw);
      if(totalTurn*step< -1e-10) {valid=false;break;}
      totalTurn+=step;previousYaw=turn.yaw;
      if(std::abs(totalTurn)>M_PI/2+1e-8) {valid=false;break;}
      ++next;
    }
    if(!valid || next>=stages.size() || next==legs.back()+1) break;
    const auto& after=stages[next];
    const double outgoing=length(after);
    if(outgoing<0 || nav_kernel::distance3D(before.points.back(),after.points.front())>1e-8) break;
    const auto& a=before.points.front(); const auto& b=before.points.back();
    const auto& c=after.points.back();
    const double ux=(b.x-a.x)/incoming,uy=(b.y-a.y)/incoming;
    const double vx=(c.x-b.x)/outgoing,vy=(c.y-b.y)/outgoing;
    const double turn=std::atan2(ux*vy-uy*vx,ux*vx+uy*vy);
    const double resolutionAngle=std::asin(std::min(1.0,lateralResolution/incoming))+
        std::asin(std::min(1.0,lateralResolution/outgoing));
    if(std::abs(turn)<1e-6 || std::abs(turn)>M_PI/2+1e-8 ||
       std::abs(collision::shortestYawDelta(totalTurn,turn))>2*yawToleranceRad+resolutionAngle) break;
    const double tangent=std::tan(std::abs(turn)*.5);
    const double radius=std::min(1.0,.5*std::min(incoming,outgoing)/tangent);
    corners.push_back({turn,radius*tangent,radius});
    legs.push_back(next);incoming=outgoing;
  }
  if(legs.size()<2) return {};
  PoseRouteBlend blend;
  blend.lastStageIndex=legs.back();
  blend.stage=first;
  blend.stage.tangentHeading=true;
  blend.stage.yaw=stages[legs.back()].yaw;
  blend.stage.points.clear();
  blend.radiusM=1.0;
  const auto append=[&](nav_kernel::Vec3 point) {
    if(blend.stage.points.empty() ||
       nav_kernel::distance3D(blend.stage.points.back(),point)>1e-10)
      blend.stage.points.push_back(point);
  };
  for(std::size_t leg=0;leg<legs.size();++leg) {
    const auto& points=stages[legs[leg]].points;
    const auto& a=points.front(); const auto& b=points.back();
    const double total=nav_kernel::distance3D(a,b);
    const double ux=(b.x-a.x)/total,uy=(b.y-a.y)/total;
    if(leg==0) append(a);
    if(leg==corners.size()) {append(b);break;}
    const auto& corner=corners[leg];
    const nav_kernel::Vec3 entry{b.x-corner.trim*ux,b.y-corner.trim*uy,b.z};
    append(entry);
    const double direction=std::copysign(1.,corner.turn);
    const nav_kernel::Vec3 center{entry.x-direction*uy*corner.radius,
                                  entry.y+direction*ux*corner.radius,b.z};
    const double initial=std::atan2(entry.y-center.y,entry.x-center.x);
    const int samples=std::max(2,static_cast<int>(std::ceil(std::abs(corner.turn)/.1)));
    for(int sample=1;sample<=samples;++sample) {
      const double angle=corner.turn*sample/samples;
      append({center.x+corner.radius*std::cos(initial+angle),
              center.y+corner.radius*std::sin(initial+angle),b.z});
    }
    blend.radiusM=std::min(blend.radiusM,corner.radius);
  }
  return blend;
}

std::optional<PoseRouteBlend> poseRouteContinuous(
    const std::vector<PoseRouteStage>& stages, std::size_t index, double tolerance,
    double resolution) {
  if(index>=stages.size() || !std::isfinite(tolerance) || tolerance<0 ||
     !std::isfinite(resolution) || resolution<0 ||
     !forwardStage(stages[index],tolerance,resolution)) return {};
  PoseRouteBlend result;
  result.stage=stages[index];
  result.stage.tangentHeading=true;
  result.lastStageIndex=index;
  result.radiusM=std::numeric_limits<double>::infinity();
  while(result.lastStageIndex+2<stages.size()) {
    std::size_t next=result.lastStageIndex+1;
    const auto& previous=stages[result.lastStageIndex];
    double yaw=previous.yaw,total=0;
    bool valid=true;
    while(next<stages.size() && stages[next].rotation) {
      const auto& turn=stages[next];
      if(turn.points.size()!=1 ||
         nav_kernel::distance3D(turn.points.front(),previous.points.back())>1e-8) {valid=false;break;}
      const double delta=collision::shortestYawDelta(yaw,turn.yaw);
      if(total*delta< -1e-10) {valid=false;break;}
      total+=delta;yaw=turn.yaw;++next;
    }
    if(!valid || next==result.lastStageIndex+1 || next>=stages.size() || stages[next].rotation ||
       std::abs(total)>M_PI/2+1e-8 ||
       std::abs(collision::shortestYawDelta(yaw,stages[next].yaw))>1e-8 ||
       nav_kernel::distance3D(previous.points.back(),stages[next].points.front())>1e-8 ||
       !forwardStage(stages[next],tolerance,resolution)) break;
    result.stage.points.insert(result.stage.points.end(),stages[next].points.begin()+1,
                               stages[next].points.end());
    result.stage.yaw=stages[next].yaw;
    result.lastStageIndex=next;
  }
  if(result.lastStageIndex==index) return {};
  return result;
}

}  // namespace lingtu::nav::navigation

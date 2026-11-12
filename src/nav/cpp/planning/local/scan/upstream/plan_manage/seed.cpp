// Seed construction extracted from LingTu's SCAN manager.
// Upstream base: SCAN-Planner 348e8a590a50a5a6bbab8d8c6dcfd171f009be26.
// Velocity ramps and 3-D reference seeding are local adaptations.
// SPDX-License-Identifier: Apache-2.0

#include "planning/local/scan/upstream/plan_manage/seed.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace nav_kernel::local::scan::upstream::seed {

Eigen::VectorXd referenceDurations(const std::vector<Eigen::Vector3d> &points,
    const Eigen::Vector3d &startVelocity, const Eigen::Vector3d &endVelocity,
    double maxVelocity, double maxAcceleration) {
  const std::size_t count=points.size();
  std::vector<double> distance(count-1),speed(count,maxVelocity);
  for (std::size_t i=0;i+1<count;++i) distance[i]=(points[i+1]-points[i]).norm();
  speed.front()=std::min(maxVelocity,startVelocity.norm());
  speed.back()=std::min(maxVelocity,endVelocity.norm());
  for (std::size_t i=1;i<count;++i)
    speed[i]=std::min(speed[i],std::sqrt(speed[i-1]*speed[i-1]+2*maxAcceleration*distance[i-1]));
  for (std::size_t i=count-1;i>0;--i)
    speed[i-1]=std::min(speed[i-1],std::sqrt(speed[i]*speed[i]+2*maxAcceleration*distance[i-1]));
  Eigen::VectorXd times(count-1);
  for (std::size_t i=0;i+1<count;++i) {
    const double peak=std::min(maxVelocity,std::sqrt(maxAcceleration*distance[i]+
        .5*(speed[i]*speed[i]+speed[i+1]*speed[i+1])));
    const double ramps=(2*peak*peak-speed[i]*speed[i]-speed[i+1]*speed[i+1])/(2*maxAcceleration);
    times(i)=(2*peak-speed[i]-speed[i+1])/maxAcceleration+
        std::max(0.0,distance[i]-ramps)/maxVelocity;
    const double nominal=distance[i]/maxVelocity*(i==0 ? 2.0 : 1.0)*
        (i+2==count ? 2.0 : 1.0);
    times(i)=std::max(times(i),nominal);
  }
  return times;
}

bool boundedVelocitySeed(const PlanParameters &params,
                      const Eigen::Vector3d &start,
                      const Eigen::Vector3d &start_velocity,
                      const Eigen::Vector3d &start_acceleration,
                      const Eigen::Vector3d &target,
                      const Eigen::Vector3d &target_velocity,
                      PolynomialTraj &trajectory, double &duration,
                      double &interval) {
  // This seed joins constant-acceleration-bound velocity ramps to a cruise.
  // Nonzero initial acceleration and short maneuvers retain the existing
  // polynomial initializer, including all measured boundary derivatives.
  const double speed = params.max_vel_;
  if (start_acceleration.squaredNorm() != 0.0 ||
      start_velocity.norm() > speed + 1e-9 ||
      target_velocity.norm() > speed + 1e-9)
    return false;

  const auto ramp_time = [&](const Eigen::Vector3d &velocity,
                             const Eigen::Vector3d &cruise_velocity) {
    const double delta = (cruise_velocity - velocity).norm();
    if (delta <= 1e-9)
      return 0.0;
    return std::max(1.5 * delta / params.max_acc_,
                    std::sqrt(6.0 * delta / params.max_jerk_));
  };
  Eigen::Vector3d cruise_velocity = (target - start).normalized() * speed;
  double accelerate_time = 0.0;
  double brake_time = 0.0;
  Eigen::Vector3d cruise_displacement;
  bool converged = false;
  // Ramp displacement is T*(v_boundary + v_cruise)/2. Solve its
  // direction together with the actual vector velocity changes; equal
  // boundary and cruise velocities require no acceleration ramp.
  for (int iteration = 0; iteration < 64; ++iteration) {
    accelerate_time = ramp_time(start_velocity, cruise_velocity);
    brake_time = ramp_time(target_velocity, cruise_velocity);
    cruise_displacement =
        target - start - 0.5 * accelerate_time * start_velocity -
        0.5 * brake_time * target_velocity;
    if (cruise_displacement.norm() <=
        0.5 * speed * (accelerate_time + brake_time))
      return false;
    const Eigen::Vector3d next_velocity =
        cruise_displacement.normalized() * speed;
    if ((next_velocity - cruise_velocity).norm() <= 1e-9) {
      cruise_velocity = next_velocity;
      converged = true;
      break;
    }
    cruise_velocity = (cruise_velocity + next_velocity).normalized() * speed;
  }
  if (!converged)
    return false;
  const double cruise_equivalent_time = cruise_displacement.norm() / speed;
  const double cruise_time =
      cruise_equivalent_time - 0.5 * (accelerate_time + brake_time);
  if (cruise_time <= 0.0)
    return false;

  const Eigen::Vector3d cruise_start =
      start + 0.5 * accelerate_time * (start_velocity + cruise_velocity);
  const Eigen::Vector3d cruise_end =
      target - 0.5 * brake_time * (cruise_velocity + target_velocity);
  const auto append = [&](const Eigen::Vector3d &p0,
                          const Eigen::Vector3d &v0,
                          const Eigen::Vector3d &p1,
                          const Eigen::Vector3d &v1, const double time) {
    if (time == 0.0)
      return;
    auto segment = PolynomialTraj::one_segment_traj_gen(
        p0, v0, Eigen::Vector3d::Zero(), p1, v1,
        Eigen::Vector3d::Zero(), time);
    trajectory.addSegment(segment.getCoef(0).front(),
                          segment.getCoef(1).front(),
                          segment.getCoef(2).front(), time);
  };
  append(start, start_velocity, cruise_start, cruise_velocity, accelerate_time);
  append(cruise_start, cruise_velocity, cruise_end, cruise_velocity, cruise_time);
  append(cruise_end, cruise_velocity, target, target_velocity, brake_time);
  trajectory.init();
  duration = accelerate_time + cruise_time + brake_time;
  // Resolve a full acceleration ramp even at a low speed cap. A nearly
  // unchanged boundary velocity must not force vanishing sample intervals.
  const double full_ramp_time =
      std::max(1.5 * speed / params.max_acc_,
               std::sqrt(6.0 * speed / params.max_jerk_));
  interval = std::min(interval, full_ramp_time / 3.0);
  return true;
}

void applyLinearZReference(std::vector<Eigen::Vector3d> &points,
                           const double start_z, const double target_z) {
  if (points.empty()) {
    return;
  }
  if (points.size() == 1) {
    points.front()(2) = start_z;
    return;
  }

  std::vector<double> accumulated_xy_length(points.size(), 0.0);
  for (std::size_t i = 1; i < points.size(); ++i) {
    accumulated_xy_length[i] =
        accumulated_xy_length[i - 1] +
        (points[i].head<2>() - points[i - 1].head<2>()).norm();
  }

  const double total_xy_length = accumulated_xy_length.back();
  for (std::size_t i = 0; i < points.size(); ++i) {
    const double ratio =
        total_xy_length > 1e-6
            ? accumulated_xy_length[i] / total_xy_length
            : static_cast<double>(i) / static_cast<double>(points.size() - 1);
    points[i](2) = start_z + ratio * (target_z - start_z);
  }

  points.front()(2) = start_z;
  points.back()(2) = target_z;
}

bool applyReferenceSeed(std::vector<Eigen::Vector3d> &points, double &interval,
                        GlobalTrajData &reference,
                        const std::pair<double,double> &window,
                        const Eigen::Vector3d &start, const Eigen::Vector3d &target,
                        const double spacing,
                        const std::vector<Eigen::Vector3d> *planarReference,
                        double &referenceProgress) {
  if (window.second <= window.first || points.size() < 2) return false;
  const int samples=std::max(6,static_cast<int>(std::ceil(
      (window.second-window.first)/.05)));
  std::vector<Eigen::Vector3d> guide;
  std::vector<double> guide_length;
  guide.reserve(samples+1);
  guide_length.reserve(samples+1);
  if(planarReference && planarReference->size()>1) {
    std::vector<double> progress(planarReference->size(),0.0);
    for(std::size_t i=1;i<progress.size();++i)
      progress[i]=progress[i-1]+((*planarReference)[i]-(*planarReference)[i-1]).norm();
    const auto project=[&](const Eigen::Vector3d& point,double minimum) {
      std::pair<double,Eigen::Vector3d> result{minimum,planarReference->back()};
      double best=std::numeric_limits<double>::infinity();
      for(std::size_t i=1;i<progress.size();++i) {
        const double span=progress[i]-progress[i-1];
        if(span<=1e-12 || progress[i]<minimum) continue;
        const Eigen::Vector3d direction=(*planarReference)[i]-(*planarReference)[i-1];
        const double lower=std::clamp((minimum-progress[i-1])/span,0.0,1.0);
        const double fraction=std::clamp((point-(*planarReference)[i-1]).dot(direction)/(span*span),lower,1.0);
        const Eigen::Vector3d candidate=(*planarReference)[i-1]+fraction*direction;
        const double distance=(candidate-point).squaredNorm();
        if(distance<best-1e-12) {best=distance;result={progress[i-1]+fraction*span,candidate};}
      }
      return result;
    };
    const auto first=project(start,referenceProgress),last=project(target,first.first);
    if(last.first-first.first<=1e-9) return false;
    referenceProgress=first.first;
    guide.push_back(first.second);
    for(std::size_t i=1;i<progress.size();++i)
      if(progress[i]>first.first+1e-9 && progress[i]<last.first-1e-9)
        guide.push_back((*planarReference)[i]);
    guide.push_back(last.second);
  } else {
    for(int sample=0;sample<=samples;++sample)
      guide.push_back(reference.getPosition(window.first+
          (window.second-window.first)*sample/samples));
  }
  // Dense samples are not turns to rejoin after a lateral departure.
  std::size_t count = 0;
  for (std::size_t i = 0; i < guide.size(); ++i) {
    const Eigen::Vector3d point = guide[i];
    if (count >= 2) {
      const Eigen::Vector3d incoming = guide[count-1] - guide[count-2];
      const Eigen::Vector3d outgoing = point - guide[count-1];
      if (incoming.dot(outgoing) > 0 && incoming.cross(outgoing).norm() <=
          1e-6 * incoming.norm() * outgoing.norm()) {
        guide[count-1] = point;
        continue;
      }
    }
    guide[count++] = point;
  }
  guide.resize(count);
  guide_length.push_back(0.0);
  for(std::size_t i=1;i<guide.size();++i)
    guide_length.push_back(guide_length.back()+(guide[i]-guide[i-1]).norm());
  double length=guide_length.back();
  if(length<=1e-9) return false;
  bool curved_reference=false;
  for(std::size_t i=1;i+1<guide.size();++i) {
    const double ratio=guide_length[i]/length;
    const Eigen::Vector3d deviation=guide[i]-
        ((1-ratio)*guide.front()+ratio*guide.back());
    curved_reference=curved_reference || std::abs(deviation(2))>1e-5 ||
        (planarReference && deviation.head<2>().norm()>1e-5);
  }
  if(!curved_reference) return false;

  // Test the reference's curvature before an offset start introduces a false bend.
  guide.front()=start;
  guide.back()=target;
  for(std::size_t i=1;i<guide.size();++i)
    guide_length[i]=guide_length[i-1]+(guide[i]-guide[i-1]).norm();
  length=guide_length.back();

  // Preserve the reference's 3-D progress, including distinct surfaces at the
  // same XY. The old seed contributes its acceleration/cruise timing only.
  std::vector<double> seed_length(points.size(),0.0);
  for(std::size_t i=1;i<points.size();++i)
    seed_length[i]=seed_length[i-1]+(points[i]-points[i-1]).norm();
  const int old_intervals=static_cast<int>(points.size())-1;
  const int intervals=std::max(old_intervals,static_cast<int>(std::ceil(length/spacing)));
  std::vector<Eigen::Vector3d> result;
  result.reserve(intervals+1);
  std::size_t segment=0;
  for(int sample=0;sample<=intervals;++sample) {
    const double phase=static_cast<double>(sample)*old_intervals/intervals;
    const int seed=std::min(old_intervals-1,static_cast<int>(phase));
    const double fraction=phase-seed;
    const double progress=seed_length.back()>1e-9 ?
        ((1-fraction)*seed_length[seed]+fraction*seed_length[seed+1])/seed_length.back() :
        static_cast<double>(sample)/intervals;
    const double distance=progress*length;
    while(segment+2<guide.size() && guide_length[segment+1]<=distance) ++segment;
    const double span=guide_length[segment+1]-guide_length[segment];
    const double along=span>1e-12 ? (distance-guide_length[segment])/span : 0.0;
    const Eigen::Vector3d point=(1-along)*guide[segment]+along*guide[segment+1];
    result.push_back(point);
  }
  interval*=static_cast<double>(old_intervals)/intervals;
  points=std::move(result);
  return true;
}

}  // namespace nav_kernel::local::scan::upstream::seed

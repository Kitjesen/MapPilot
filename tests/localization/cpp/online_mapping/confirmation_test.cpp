#include "backend.hpp"
#include <iostream>
#include <stdexcept>
namespace m = lingtu::localization::online_mapping;
void require(bool v, const char* s) { if (!v) throw std::runtime_error(s); }
m::Cloud::Ptr room(const gtsam::Pose3& pose) {
  m::Cloud::Ptr cloud(new m::Cloud);
  for (int a=-18;a<=18;++a) for (int b=-18;b<=18;++b)
    for (const auto& w : {gtsam::Point3(a*.15,b*.15,0),
        gtsam::Point3(a*.15,-2.7,b*.15),gtsam::Point3(2.7,a*.15,b*.15)}) {
      const auto p=pose.transformTo(w);
      pcl::PointXYZI q; q.x=p.x();q.y=p.y();q.z=p.z();q.intensity=1;cloud->push_back(q);
    }
  return cloud;
}
int main() { try {
  m::Config c; c.voxel_m=.15;c.submap_half_window=0;
  m::Backend b(c);
  const gtsam::Pose3 origin;
  const gtsam::Pose3 slope(gtsam::Rot3::RzRyRx(.08,-.12,.2),{1.5,.2,.3});
  b.append({1,origin,room(origin)});
  b.append({2,slope,room(slope)});
  auto first=b.append({40,origin,room(origin)});
  require(!first.accepted && first.reason=="confirmation_pending", "single revisit immediately entered graph");
  for(int i=0;i<40;++i) {
    const auto dense=b.append({41+i*.01,origin,room(origin)});
    require(!dense.accepted,"stationary observations confirmed a loop");
  }
  const auto confirmed=b.append({42,slope,room(slope)});
  std::cout << "confirmed=" << confirmed.accepted << " previous=" << confirmed.previous
      << " support=" << confirmed.support_current << "->" << confirmed.support_previous
      << " cycle=" << confirmed.consistency_translation_m << "," << confirmed.consistency_rotation_rad << '\n';
  require(confirmed.accepted && confirmed.support_current==2,"independent sloped revisit not confirmed");
  require(confirmed.measurement.equals(gtsam::Pose3(),.02),"confirmed full-6D measurement is wrong");
  m::Backend same_target(c);
  same_target.append({1,origin,room(origin)});
  same_target.append({40,origin,room(origin)});
  const auto one_anchor=same_target.append({41,slope,room(slope)});
  require(!one_anchor.accepted && one_anchor.reason=="confirmation_pending",
      "source motion alone confirmed against the same historical anchor");
  m::Backend recycled_source(c);
  recycled_source.append({1,origin,room(origin)});
  recycled_source.append({40,slope,room(slope)});
  const gtsam::Pose3 further(gtsam::Rot3::RzRyRx(.08,-.12,.2),{3.,.4,.6});
  const auto recycled=recycled_source.append({81,further,room(further)});
  require(!recycled.accepted && recycled.reason=="confirmation_pending",
      "earlier source was recycled as independent historical support");
  m::Backend inconsistent(c);
  inconsistent.append({1,origin,room(origin)});
  inconsistent.append({2,slope,room(slope)});
  inconsistent.append({40,origin,room(origin)});
  const gtsam::Pose3 shifted(gtsam::Rot3::RzRyRx(.08,-.12,.2),{2.2,.2,.3});
  const auto wrong=inconsistent.append({41,shifted,room(slope)});
  std::cout<<"contradictory="<<wrong.accepted<<" reason="<<wrong.reason<<" cycle="<<wrong.consistency_translation_m<<'\n';
  require(!wrong.accepted && wrong.reason=="confirmation_inconsistent","contradictory loop passed confirmation");
  m::Backend wrong_rotation(c);
  wrong_rotation.append({1,origin,room(origin)});
  wrong_rotation.append({2,slope,room(slope)});
  wrong_rotation.append({40,origin,room(origin)});
  const gtsam::Pose3 tilted(slope.rotation().compose(gtsam::Rot3::Rx(.22)),slope.translation());
  const auto angular=wrong_rotation.append({41,tilted,room(slope)});
  std::cout<<"roll_cycle="<<angular.consistency_rotation_rad<<" reason="<<angular.reason<<'\n';
  require(!angular.accepted && angular.reason=="confirmation_inconsistent" &&
      angular.consistency_rotation_rad>c.consistency_rad,
      "roll contradiction was lost in a planar consistency check");
  std::cout<<"independent SE3 loop confirmation passed\n";
  return 0;
} catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}}

#include "lingtu/recording/recorded_scene.hpp"
#include "lingtu/recording/history_series.hpp"
#include "lingtu/recording/mcap_session.hpp"
#include "typed_cdr_fixture.hpp"
#include "messages.h"
#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>
namespace rec=lingtu::recording;
namespace {
void require(bool value,const char *message){if(!value)throw std::runtime_error(message);}
constexpr std::uint64_t origin=1700000000000000000ULL;
void set_header(lingtu_dds_Header&h,std::uint64_t ns){h.frame_id=const_cast<char*>("odom");h.stamp.sec=ns/1000000000ULL;h.stamp.nanosec=ns%1000000000ULL;}
void fixture(const std::filesystem::path&file,bool dense=false){
 const auto &cloud=*rec::find_recording_topic("/slam/registered_cloud"),&odom=*rec::find_recording_topic("/slam/odometry"),&path=*rec::find_recording_topic("/nav/global_path");
 rec::McapSessionWriter writer(file,rec::read_text_file(LINGTU_RECORDING_DEFAULT_IDL),{rec::channel_definition(cloud),rec::channel_definition(odom),rec::channel_definition(path)},1024);
 for(int i=0;i<(dense?40:20);++i){
  if(i>=3&&i<13)continue;const auto source=origin+i*100000000ULL;
  lingtu_dds_PointCloud2 frame{};set_header(frame.header,source);frame.width=dense?6000:3;frame.height=dense?1:2;frame.point_step=16;frame.row_step=dense?96000:64;frame.is_bigendian=i%2;
  lingtu_dds_PointField fields[3]{};for(int k=0;k<3;++k){fields[k].name=const_cast<char*>(k==0?"x":k==1?"y":"z");fields[k].offset=k*4;fields[k].datatype=7;fields[k].count=1;}
  frame.fields={3,3,fields,false};std::vector<std::uint8_t> data(frame.row_step*frame.height);
  for(int j=0;j<(dense?6000:6);++j)for(int k=0;k<3;++k){const float number=static_cast<float>(j+k*10+i);std::uint8_t bytes[4];std::memcpy(bytes,&number,4);if(frame.is_bigendian)std::reverse(bytes,bytes+4);std::memcpy(data.data()+(j/frame.width)*frame.row_step+(j%frame.width)*16+k*4,bytes,4);}
  frame.data={static_cast<std::uint32_t>(data.size()),static_cast<std::uint32_t>(data.size()),data.data(),false};writer.write({std::string(cloud.contract->dds_topic),source+2000000,source,static_cast<std::uint32_t>(i),rec::test::serialize(cloud,&frame)});
  lingtu_dds_Odometry pose{};set_header(pose.header,source);pose.child_frame_id=const_cast<char*>("body");pose.pose.pose.position.x=i;pose.pose.pose.orientation.w=1;
  writer.write({std::string(odom.contract->dds_topic),source,source,static_cast<std::uint32_t>(i),rec::test::serialize(odom,&pose)});
 }
 lingtu_dds_Path plan{};set_header(plan.header,origin);std::vector<lingtu_dds_PoseStamped> points(dense?9000:3);
 for(std::size_t i=0;i<points.size();++i){set_header(points[i].header,origin);points[i].pose.position.x=i*3;points[i].pose.orientation.w=1;}
 plan.poses={static_cast<std::uint32_t>(points.size()),static_cast<std::uint32_t>(points.size()),points.data(),false};writer.write({std::string(path.contract->dds_topic),origin,origin,0,rec::test::serialize(path,&plan)});writer.commit();
}
void transition_fixture(const std::filesystem::path &file) {
 const auto &binding=*rec::find_recording_topic("/nav/global_path");
 rec::McapSessionWriter writer(file,rec::read_text_file(LINGTU_RECORDING_DEFAULT_IDL),{rec::channel_definition(binding)},1024);
 lingtu_dds_PoseStamped points[2]{};for(auto &p:points){set_header(p.header,origin);p.pose.orientation.w=1;}
 points[1].pose.position.x=3;
 for(int i=0;i<32;++i) {
  lingtu_dds_Path plan{};const auto time=origin+i*200000000ULL;set_header(plan.header,time);
  if(i!=15)plan.poses={2,2,points,false};
  writer.write({std::string(binding.contract->dds_topic),time,time,static_cast<std::uint32_t>(i),rec::test::serialize(binding,&plan)});
 }
 writer.commit();
}

}
int main(){
 try{
  const auto file=std::filesystem::temp_directory_path()/("lingtu-scene-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".mcap");fixture(file);
  rec::SceneQuery query;query.end_s=2;query.max_frames=32;query.max_points=128;
  auto result=rec::read_recorded_scene(file,query);
  require(result.start_time_ns==origin&&result.cloud&&result.trajectory&&result.paths.size()==1,"scene discovery/clock missing");
  require(result.cloud->frames.size()==10&&result.trajectory->frames.size()==10,"window lost recorded frames");
  require(result.cloud->frames[0].source_time_ns==origin&&result.cloud->frames[0].t==0.002,"source/log clock collapsed");
  require(result.cloud->frames[1].points[0]==1&&result.cloud->frames[1].points[12]==5,"bigendian or organized-row padding decoded incorrectly");
  require(result.cloud->frames[3].gap_before&&result.trajectory->frames[3].gap_before,"scene timeline bridged missing frames");
  require(result.paths[0].frames[0].points.back()==0&&result.paths[0].frames[0].points[6]==6,"plan snapshot mixed with pose history");
  query.start_s=1.3;query.end_s=1.5;query.max_frames=2;query.max_points=2;query.max_poses=2;
  auto bounded=rec::read_recorded_scene(file,query);
  require(bounded.cloud->frames.size()<=2&&bounded.cloud->frames[0].points.size()<=6&&bounded.trajectory->frames.size()<=2,"scene response is not bounded");
  require(bounded.paths[0].frames.size()==1&&bounded.paths[0].frames[0].t==0&&bounded.paths[0].frames[0].carried_forward,"latest actual plan was lost outside visible window");
  query.info=true;auto info=rec::read_recorded_scene(file,query);require(info.topics.size()==3&&!info.cloud,"metadata discovery decoded scene payload");
  query.info=false;query.start_s=3;query.end_s=4;require(rec::read_recorded_scene(file,query).cloud->frames.empty(),"nonoverlap fabricated frames");
  query.start_s=0;query.end_s=31;bool rejected=false;try{rec::read_recorded_scene(file,query);}catch(const rec::SeriesError&e){rejected=e.code=="recording_scene_invalid_query";}require(rejected,"unbounded window accepted");
  std::filesystem::remove(file);fixture(file,true);
  query={};query.end_s=4;query.max_frames=32;query.max_points=4096;query.max_poses=4000;
  auto dense=rec::read_recorded_scene(file,query);
  std::size_t vertices=dense.trajectory->frames.size();for(const auto&f:dense.cloud->frames)vertices+=f.points.size()/3;
  for(const auto&path:dense.paths)for(const auto&f:path.frames)vertices+=f.points.size()/3;
  require(vertices<=65536&&dense.cloud_points_per_frame<4096,"combined scene vertex budget exceeded");
  require(dense.paths[0].frames[0].points[dense.paths[0].frames[0].points.size()-3]==26997,"path sampling discarded endpoint");
  require(dense.json().size()<5*1024*1024,"bounded dense scene exceeded HTTP response budget");
  std::filesystem::remove(file);transition_fixture(file);
  query={};query.end_s=7;query.max_frames=16;
  const auto transitions=rec::read_recorded_scene(file,query);
  const auto &snapshots=transitions.paths[0].frames;
  require(std::any_of(snapshots.begin(),snapshots.end(),[](const auto&f){return f.t==3&&f.original_count==0&&f.points.empty();}),"uniform sampling swallowed an actual plan clear");
  require(std::any_of(snapshots.begin(),snapshots.end(),[](const auto&f){return f.t==3.2&&f.original_count==2;}),"first resumed plan was lost after a clear");
  std::filesystem::remove(file);std::cout<<"native recorded scene: indexed bounds, source/log time, row padding, big endian, gap, pose/path separation and plan predecessor passed\n";return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
}

#include "lingtu/recording/recorded_scene.hpp"
#include "lingtu/recording/history_series.hpp"
#include "lingtu/recording/message_preview.hpp"
#include "lingtu/recording/mcap_session.hpp"
#include "mcap/reader.hpp"
#include "messages.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <map>
#include <set>
#include <sstream>
#include <locale>

namespace lingtu::recording {
namespace {
[[noreturn]] void fail(const char *message = "Recorded scene data could not be read.") {
  throw SeriesError("recording_scene_unreadable", message);
}
std::string kind(std::string_view type) {
  if (type == "lingtu.dds.PointCloud2" || type == "lingtu.dds.LivoxFrame") return "cloud";
  if (type == "lingtu.dds.Odometry") return "pose";
  if (type == "lingtu.dds.Path") return "path";
  return "";
}
std::uint64_t stamp(const lingtu_dds_Header &header) {
  return header.stamp.sec >= 0 ? static_cast<std::uint64_t>(header.stamp.sec) * 1000000000ULL + header.stamp.nanosec : 0;
}
bool finite(double x, double y, double z) { return std::isfinite(x) && std::isfinite(y) && std::isfinite(z); }
void point(std::vector<double> &output, double x, double y, double z) {
  if (finite(x,y,z)) { output.push_back(x); output.push_back(y); output.push_back(z); }
}
template <typename T> T scalar(const std::uint8_t *data, bool bigendian) {
  std::array<std::uint8_t, sizeof(T)> bytes{};
  std::memcpy(bytes.data(), data, sizeof(T));
  if (bigendian) std::reverse(bytes.begin(), bytes.end());
  T value{}; std::memcpy(&value, bytes.data(), sizeof(T)); return value;
}
SceneFrame cloud_frame(const TopicBinding &binding, const void *sample, std::size_t max_points) {
  SceneFrame frame;
  if (binding.contract->idl_type == "lingtu.dds.LivoxFrame") {
    const auto &cloud = *static_cast<const lingtu_dds_LivoxFrame *>(sample);
    frame.frame_id = cloud.header.frame_id ? cloud.header.frame_id : "";
    frame.source_time_ns = stamp(cloud.header);
    frame.original_count = cloud.points._length;
    const auto stride = std::max<std::size_t>(1, (cloud.points._length + max_points - 1) / max_points);
    for (std::size_t i=0; i<cloud.points._length; i+=stride) {
      const auto &p = cloud.points._buffer[i]; point(frame.points,p.x,p.y,p.z);
    }
    return frame;
  }
  const auto &cloud = *static_cast<const lingtu_dds_PointCloud2 *>(sample);
  frame.frame_id = cloud.header.frame_id ? cloud.header.frame_id : "";
  frame.source_time_ns = stamp(cloud.header);
  frame.original_count = static_cast<std::uint64_t>(cloud.width) * cloud.height;
  std::array<const lingtu_dds_PointField *,3> fields{};
  for (std::uint32_t i=0; i<cloud.fields._length; ++i) {
    const auto &f=cloud.fields._buffer[i];
    if (!f.name) continue;
    for (int axis=0;axis<3;++axis) if (std::string_view(f.name) == std::string_view("xyz"+axis,1)) fields[axis]=&f;
  }
  for (const auto *f : fields) if (!f || f->count != 1 || (f->datatype != 7 && f->datatype != 8) ||
      f->offset + (f->datatype == 7 ? 4ULL : 8ULL) > cloud.point_step)
    fail("Point cloud needs valid float32/float64 x, y and z fields.");
  if (cloud.point_step == 0 || static_cast<std::uint64_t>(cloud.width)*cloud.point_step > cloud.row_step ||
      static_cast<std::uint64_t>(cloud.row_step)*cloud.height > cloud.data._length)
    fail("Point cloud row layout is incomplete.");
  const auto stride = std::max<std::uint64_t>(1, (frame.original_count + max_points - 1)/max_points);
  for (std::uint64_t i=0;i<frame.original_count;i+=stride) {
    const auto offset=(i/cloud.width)*cloud.row_step+(i%cloud.width)*cloud.point_step;
    std::array<double,3> xyz{};
    for (int axis=0;axis<3;++axis) {
      const auto *f=fields[axis]; const auto *bytes=cloud.data._buffer+offset+f->offset;
      xyz[axis]=f->datatype==7 ? scalar<float>(bytes,cloud.is_bigendian) : scalar<double>(bytes,cloud.is_bigendian);
    }
    point(frame.points,xyz[0],xyz[1],xyz[2]);
  }
  return frame;
}
SceneFrame pose_frame(const lingtu_dds_Odometry &pose) {
  SceneFrame frame;
  frame.frame_id=pose.header.frame_id ? pose.header.frame_id : "";
  frame.source_time_ns=stamp(pose.header);
  const auto &p=pose.pose.pose;
  point(frame.points,p.position.x,p.position.y,p.position.z);
  frame.orientation={p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w};
  if (!std::all_of(frame.orientation.begin(),frame.orientation.end(),[](double v){return std::isfinite(v);})) frame.points.clear();
  return frame;
}
SceneFrame path_frame(const lingtu_dds_Path &path, std::size_t max_points = 512) {
  SceneFrame frame;
  frame.frame_id=path.header.frame_id ? path.header.frame_id : "";
  frame.source_time_ns=stamp(path.header); frame.original_count=path.poses._length;
  const auto count=std::min<std::size_t>(path.poses._length,max_points);
  for (std::size_t index=0;index<count;++index) {
    const auto i=count==1?path.poses._length-1:index*(path.poses._length-1)/(count-1);
    const auto &pose=path.poses._buffer[i];
    const std::string pose_frame_id=pose.header.frame_id ? pose.header.frame_id : "";
    if (frame.frame_id.empty()) frame.frame_id=pose_frame_id;
    if (!pose_frame_id.empty() && pose_frame_id!=frame.frame_id) fail("Path contains poses in different coordinate frames.");
    const auto &p=pose.pose.position; point(frame.points,p.x,p.y,p.z);
  }
  return frame;
}
struct Selection {
  const TopicBinding *binding{};
  SceneStream *output{};
  std::size_t limit{};
  std::vector<std::uint64_t> times;
  std::set<std::uint64_t> selected;
  std::vector<std::uint64_t> segments;
  std::vector<bool> path_nonempty;
};
std::string array(const std::vector<double> &values) {
  std::string out="["; for (const auto v:values) {if(out.size()>1)out+=',';out+=json_number(v);} return out+']';
}
}
RecordedScene read_recorded_scene(const std::filesystem::path &file,const SceneQuery &query) {
  if (!std::isfinite(query.start_s)||!std::isfinite(query.end_s)||query.start_s<0||query.end_s<query.start_s||
      (!query.info && query.end_s-query.start_s>30)||query.max_frames<1||query.max_frames>32||
      query.max_points<1||query.max_points>4096||
      query.max_poses<2||query.max_poses>4000||query.path_topics.size()>2)
    throw SeriesError("recording_scene_invalid_query","Use a nonnegative window up to 30 seconds and bounded scene detail.");
  mcap::McapReader reader;
  if(!reader.open(file.string()).ok()||!reader.header()||reader.header()->profile!=kMcapProfile) fail();
  bool failed=false;const auto problem=[&](const mcap::Status&){failed=true;};
  if(!reader.readSummary(mcap::ReadSummaryMethod::AllowFallbackScan,problem).ok()||failed||!reader.statistics())fail();
  const auto &stats=*reader.statistics();
  RecordedScene result;result.info=query.info;result.start_time_ns=stats.messageStartTime;
  result.duration_s=stats.messageCount ? static_cast<double>(stats.messageEndTime-stats.messageStartTime)/1e9:0;
  result.start_s=std::min(query.start_s,result.duration_s);result.end_s=std::min(query.end_s,result.duration_s);
  std::map<std::string,const TopicBinding*> available;
  for(const auto &[id,channel]:reader.channels()) {
    const auto *binding=find_recording_topic(channel->topic); if(!binding)continue;
    const auto role=kind(binding->contract->idl_type);if(role.empty())continue;
    const auto schema=reader.schema(channel->schemaId);const auto type=channel->metadata.find("lingtu.idl_type");
    if(channel->messageEncoding!="cdr"||!schema||schema->encoding!="omgidl"||schema->name!=binding->contract->idl_type||
       type==channel->metadata.end()||type->second!=binding->contract->idl_type)fail();
    const auto count=stats.channelMessageCounts.find(id);
    const auto count_value=count==stats.channelMessageCounts.end()?0:count->second;
    result.topics.push_back({std::string(binding->contract->topic),std::string(binding->contract->idl_type),role,count_value});
    available.emplace(std::string(binding->contract->topic),binding);
  }
  if(query.info)return result;
  const auto choose=[&](const std::string &requested,const std::string &role,const std::vector<std::string>& preferred)->const TopicBinding* {
    if(!requested.empty()) {
      const auto *binding=find_recording_topic(requested);
      if(!binding||kind(binding->contract->idl_type)!=role)throw SeriesError("recording_scene_unsupported_topic","Choose a recorded topic of the requested scene type.");
      const auto found=available.find(std::string(binding->contract->topic));return found==available.end()?nullptr:found->second;
    }
    for(const auto &name:preferred){const auto found=available.find(name);if(found!=available.end())return found->second;}
    for(const auto &[name,binding]:available)if(kind(binding->contract->idl_type)==role)return binding;
    return nullptr;
  };
  std::vector<Selection> streams;
  const auto add=[&](const TopicBinding *binding,SceneStream &stream,std::size_t limit){
    stream.topic=std::string(binding->contract->topic);
    for(const auto &topic:result.topics)if(topic.topic==stream.topic)stream.original_count+=topic.message_count;
    streams.push_back({binding,&stream,limit});
  };
  if(const auto *binding=choose(query.cloud_topic,"cloud",{"/slam/registered_cloud","/slam/map_cloud","/lidar/raw_frame"})) {
    result.cloud.emplace();add(binding,*result.cloud,query.max_frames);
  }
  if(const auto *binding=choose(query.pose_topic,"pose",{"/slam/odometry","/slam/state_at_scan","/driver/odometry"})) {
    result.trajectory.emplace();add(binding,*result.trajectory,query.max_poses);
  }
  std::vector<const TopicBinding*> paths;
  if(query.path_topics.empty()) {
    for(const auto &topic:result.topics)if(topic.kind=="path"&&paths.size()<2)paths.push_back(available.at(topic.topic));
  } else {
    for(const auto &name:query.path_topics)if(const auto *binding=choose(name,"path",{}))
      if(std::find(paths.begin(),paths.end(),binding)==paths.end())paths.push_back(binding);
  }
  result.paths.resize(paths.size());
  for(std::size_t i=0;i<paths.size();++i)add(paths[i],result.paths[i],std::min<std::size_t>(query.max_frames,16));
  if(query.start_s>result.duration_s||streams.empty())return result;
  mcap::ReadMessageOptions options;
  options.startTime=result.start_time_ns+static_cast<std::uint64_t>(std::llround(result.start_s*1e9));
  options.endTime=result.start_time_ns+static_cast<std::uint64_t>(std::llround(result.end_s*1e9))+1;
  options.readOrder=mcap::ReadMessageOptions::ReadOrder::LogTimeOrder;
  options.topicFilter=[&](std::string_view topic){return std::any_of(streams.begin(),streams.end(),[&](const Selection&s){return topic==s.binding->contract->dds_topic;});};
  try {
  for(const auto &view:reader.readMessages(problem,options))for(auto &stream:streams)if(view.channel->topic==stream.binding->contract->dds_topic) {
    stream.times.push_back(view.message.logTime);
    if(kind(stream.binding->contract->idl_type)=="path")
      stream.path_nonempty.push_back(recorded_path_pose_count(view.message.data,view.message.dataSize)>0);
  }
  } catch(const std::exception &) { fail(); }
  if(failed)fail();
  // A plan is a state snapshot, not a sensor frame. Retrieve its latest actual
  // predecessor through the reverse log-time index so it remains visible until
  // a later recorded plan replaces or clears it.
  for(auto &stream:streams) if(kind(stream.binding->contract->idl_type)=="path" &&
      options.startTime>result.start_time_ns && (stream.times.empty()||stream.limit>1)) {
    auto predecessor=options;predecessor.startTime=0;predecessor.endTime=options.startTime;
    predecessor.readOrder=mcap::ReadMessageOptions::ReadOrder::ReverseLogTimeOrder;
    predecessor.topicFilter=[&](std::string_view topic){return topic==stream.binding->contract->dds_topic;};
    for(const auto &view:reader.readMessages(problem,predecessor)) {
      auto sample=decode_recorded_sample(*stream.binding,view.message.data,view.message.dataSize);
      auto frame=path_frame(*static_cast<lingtu_dds_Path*>(sample.get()),result.path_points_per_frame);
      frame.t=(view.message.logTime-result.start_time_ns)/1e9;frame.carried_forward=true;
      stream.output->frames.push_back(std::move(frame));break;
    }
  }
  if(failed)fail();
  for(auto &stream:streams){
    const auto size=stream.times.size();stream.output->read_count=size;if(!size)continue;
    std::vector<double> periods;
    for(std::size_t i=1;i<size;++i)if(stream.times[i]>stream.times[i-1])periods.push_back((stream.times[i]-stream.times[i-1])/1e9);
    if(!periods.empty()){const auto mid=periods.begin()+periods.size()/2;std::nth_element(periods.begin(),mid,periods.end());stream.output->gap_threshold_s=*mid*5;}
    std::uint64_t segment=0;stream.segments.push_back(0);
    for(std::size_t i=1;i<size;++i){if(stream.output->gap_threshold_s>0&&(stream.times[i]-stream.times[i-1])/1e9>stream.output->gap_threshold_s)++segment;stream.segments.push_back(segment);}
    const auto count=std::min(size,stream.limit-stream.output->frames.size());
    for(std::size_t i=0;i<count;++i)stream.selected.insert(count==1?size-1:i*(size-1)/(count-1));
    // A sampled plan must not remain visible through an actual recorded clear.
    // Keep the clear and the first resumed nonempty snapshot, including runs
    // that occur entirely between the ordinary display sample times.
    for(std::size_t i=0;i<stream.path_nonempty.size();++i)
      if((!stream.path_nonempty[i]&&(i==0||stream.path_nonempty[i-1])) ||
         (stream.path_nonempty[i]&&i>0&&!stream.path_nonempty[i-1]))stream.selected.insert(i);
  }
  // One display budget spans every overlay. A high requested cloud detail
  // is sampled down to the remaining budget rather than rejecting the view.
  std::size_t reserved=0,cloud_frames=0,path_frames=0;
  for(const auto &stream:streams) {
    const auto role=kind(stream.binding->contract->idl_type);
    const auto count=stream.selected.size()+stream.output->frames.size();
    if(role=="path") {
      for(const auto index:stream.selected)if(stream.path_nonempty[index])++path_frames;
      for(const auto &frame:stream.output->frames)if(!frame.points.empty())++path_frames;
    } else if(role=="pose")reserved+=count;
    else cloud_frames=count;
  }
  result.path_points_per_frame=std::max<std::size_t>(2,std::min<std::size_t>(512,32768/std::max<std::size_t>(path_frames,1)));
  for(auto &stream:streams)if(kind(stream.binding->contract->idl_type)=="path")
    for(auto &frame:stream.output->frames)if(frame.points.size()/3>result.path_points_per_frame) {
      const auto points=std::move(frame.points);const auto count=points.size()/3;
      for(std::size_t i=0;i<result.path_points_per_frame;++i) {
        const auto index=i*(count-1)/(result.path_points_per_frame-1);
        frame.points.insert(frame.points.end(),points.begin()+index*3,points.begin()+index*3+3);
      }
    }
  reserved+=path_frames*result.path_points_per_frame;
  result.cloud_points_per_frame=std::min<std::size_t>(query.max_points,
      (65536-std::min<std::size_t>(reserved,65535))/std::max<std::size_t>(cloud_frames,1));
  result.cloud_points_per_frame=std::max<std::size_t>(1,result.cloud_points_per_frame);
  std::map<const TopicBinding*,std::uint64_t> indices,previous_segments;
  try {
    for(const auto &view:reader.readMessages(problem,options))for(auto &stream:streams)if(view.channel->topic==stream.binding->contract->dds_topic){
      const auto index=indices[stream.binding]++;if(!stream.selected.count(index))continue;
      auto sample=decode_recorded_sample(*stream.binding,view.message.data,view.message.dataSize);
      const auto role=kind(stream.binding->contract->idl_type);
      SceneFrame frame=role=="cloud"?cloud_frame(*stream.binding,sample.get(),result.cloud_points_per_frame):
                       role=="pose"?pose_frame(*static_cast<lingtu_dds_Odometry*>(sample.get())):
                                    path_frame(*static_cast<lingtu_dds_Path*>(sample.get()),result.path_points_per_frame);
      frame.t=(view.message.logTime-result.start_time_ns)/1e9;
      const auto previous=previous_segments.find(stream.binding);
      frame.gap_before=previous!=previous_segments.end()&&(previous->second!=stream.segments[index]||
          (!stream.output->frames.empty()&&stream.output->frames.back().frame_id!=frame.frame_id));
      previous_segments[stream.binding]=stream.segments[index];
      if(role=="pose"&&frame.points.empty()){++previous_segments[stream.binding];continue;}
      stream.output->frames.push_back(std::move(frame));
    }
  }catch(const SeriesError&){throw;}catch(const std::exception&){fail();}
  if(failed)fail();return result;
}
std::string RecordedScene::json() const {
  std::ostringstream out;out.imbue(std::locale::classic());
  out<<"{\"version\":1,\"ok\":true,\"mode\":"<<json_quote(info?"scene-info":"scene")<<",\"start_time_ns\":"<<json_quote(std::to_string(start_time_ns))
     <<",\"duration_s\":"<<json_number(duration_s)<<",\"window\":{\"start_s\":"<<json_number(start_s)<<",\"end_s\":"<<json_number(end_s)<<"},\"topics\":[";
  bool first=true;for(const auto &topic:topics){if(!first)out<<',';first=false;out<<"{\"topic\":"<<json_quote(topic.topic)<<",\"type_name\":"<<json_quote(topic.type_name)<<",\"kind\":"<<json_quote(topic.kind)<<",\"message_count\":"<<topic.message_count<<'}';}
  out<<']';
  const auto stream_json=[&](const SceneStream &stream,const std::string &role){
    out<<"{\"topic\":"<<json_quote(stream.topic)<<",\"original_count\":"<<stream.original_count<<",\"read_count\":"<<stream.read_count<<",\"returned_count\":"<<stream.frames.size()<<",\"gap_threshold_s\":"<<json_number(stream.gap_threshold_s)<<","<<json_quote(role=="pose"?"samples":"frames")<<":[";
    bool first_frame=true;for(const auto &frame:stream.frames){if(!first_frame)out<<',';first_frame=false;
      out<<"{\"t\":"<<json_number(frame.t)<<",\"source_time_ns\":"<<json_quote(std::to_string(frame.source_time_ns))<<",\"frame_id\":"<<json_quote(frame.frame_id);
      if(role=="pose")out<<",\"position\":"<<array(frame.points)<<",\"orientation\":"<<array(std::vector<double>(frame.orientation.begin(),frame.orientation.end()));
      else out<<","<<json_quote(role=="cloud"?"original_points":"original_poses")<<':'<<frame.original_count<<",\"points\":"<<array(frame.points);
      if(frame.gap_before)out<<",\"gap_before\":true";
      if(frame.carried_forward)out<<",\"carried_forward\":true";out<<'}';
    }out<<"]}";
  };
  out<<",\"display_budget\":{\"max_vertices\":65536,\"cloud_points_per_frame\":"<<cloud_points_per_frame<<",\"path_points_per_frame\":"<<path_points_per_frame<<"},\"cloud\":";if(cloud)stream_json(*cloud,"cloud");else out<<"null";
  out<<",\"trajectory\":";if(trajectory)stream_json(*trajectory,"pose");else out<<"null";
  out<<",\"paths\":[";first=true;for(const auto &path:paths){if(!first)out<<',';first=false;stream_json(path,"path");}
  out<<"]}";return out.str();
}
}  // namespace lingtu::recording

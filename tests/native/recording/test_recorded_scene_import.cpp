#include "lingtu/recording/camera_linux.hpp"
#include "lingtu/recording/mcap_session.hpp"
#include "lingtu/recording/message_preview.hpp"
#include "typed_cdr_fixture.hpp"
#include "messages.h"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <cmath>
#include <memory>

namespace rec=lingtu::recording;
namespace {
std::vector<std::byte> bytes(const std::filesystem::path &file) {
  std::ifstream input(file,std::ios::binary|std::ios::ate);
  if(!input)throw std::runtime_error("input binary absent: "+file.string());
  const auto length=input.tellg();input.seekg(0);
  std::vector<std::byte> data(static_cast<std::size_t>(length));
  if(!input.read(reinterpret_cast<char*>(data.data()),length))throw std::runtime_error("input binary incomplete");
  return data;
}
void header(lingtu_dds_Header &header,std::string &frame,std::uint64_t source) {
  header.frame_id=frame.data();header.stamp.sec=static_cast<std::int32_t>(source/1000000000ULL);header.stamp.nanosec=source%1000000000ULL;
}
class CameraSource:public rec::CameraFrameSource {
 public:
  CameraSource(std::filesystem::path root):root_(std::move(root)),input_(root_/"camera.tsv"){}
  bool next(rec::CameraFrame &frame) override {
    std::string line;if(!std::getline(input_,line))return false;
    std::istringstream row(line);std::string path;frame=rec::CameraFrame{};
    if(!(row>>frame.timestamp_ns>>frame.frame_id>>frame.width>>frame.height>>path))throw std::runtime_error("bad camera.tsv row");
    frame.sequence=++sequence_;frame.stride=frame.width*3;frame.encoding="rgb8";frame.payload=bytes(root_/path);return true;
  }
 private:std::filesystem::path root_;std::ifstream input_;std::uint64_t sequence_{};
};
}
int main(int argc,char **argv) {
  try {
    if(argc<4||argc>6)throw std::runtime_error("usage: test_recorded_scene_import INPUT OUTPUT IDL [FFMPEG] [CATALOG_DIRECTORY]");
    const std::filesystem::path root=argv[1],output=std::filesystem::absolute(argv[2]);
    const auto &cloud=*rec::find_recording_topic("/slam/registered_cloud"), &odom=*rec::find_recording_topic("/slam/odometry");
    const auto nonempty=[&](const char *name){const auto file=root/name;return std::filesystem::exists(file)&&std::filesystem::file_size(file)>0;};
    std::unique_ptr<rec::McapSessionWriter> writer;
    if(nonempty("clouds.tsv")||nonempty("poses.tsv")) {
      std::filesystem::create_directories(output/"dds");
      writer=std::make_unique<rec::McapSessionWriter>(output/"dds/sensors.mcap",rec::read_text_file(argv[3]),
        std::vector<rec::ChannelDefinition>{rec::channel_definition(cloud),rec::channel_definition(odom)});
    } else std::filesystem::create_directories(output);
    std::uint32_t sequence=0;std::uint64_t first=0,last=0;
    const auto note_time=[&](std::uint64_t t){if(first==0||t<first)first=t;if(t>last)last=t;};
    std::ifstream clouds(root/"clouds.tsv");std::string line;
    while(std::getline(clouds,line)) {
      std::istringstream row(line);std::uint64_t log,source;std::string frame,path;
      if(!(row>>log>>source>>frame>>path))throw std::runtime_error("bad clouds.tsv row");
      auto data=bytes(root/path);if(data.size()%12)throw std::runtime_error("cloud binary needs packed float32 XYZ");
      lingtu_dds_PointCloud2 sample{};header(sample.header,frame,source);
      sample.width=data.size()/12;sample.height=1;sample.point_step=12;sample.row_step=data.size();sample.is_dense=false;
      lingtu_dds_PointField fields[3]{};
      for(int axis=0;axis<3;++axis){fields[axis].name=const_cast<char*>(axis==0?"x":axis==1?"y":"z");fields[axis].offset=axis*4;fields[axis].datatype=7;fields[axis].count=1;}
      sample.fields={3,3,fields,false};sample.data={static_cast<std::uint32_t>(data.size()),static_cast<std::uint32_t>(data.size()),reinterpret_cast<std::uint8_t*>(data.data()),false};
      writer->write({std::string(cloud.contract->dds_topic),log,source,sequence++,rec::test::serialize(cloud,&sample)});note_time(log);
    }
    std::ifstream poses(root/"poses.tsv");sequence=0;
    while(std::getline(poses,line)) {
      std::istringstream row(line);std::uint64_t log,source;std::string frame;lingtu_dds_Odometry sample{};
      auto &p=sample.pose.pose.position;auto&q=sample.pose.pose.orientation;
      if(!(row>>log>>source>>frame>>p.x>>p.y>>p.z>>q.x>>q.y>>q.z>>q.w))throw std::runtime_error("bad poses.tsv row");
      header(sample.header,frame,source);sample.child_frame_id=const_cast<char*>("imu");
      writer->write({std::string(odom.contract->dds_topic),log,source,sequence++,rec::test::serialize(odom,&sample)});note_time(log);
    }
    if(writer)writer->commit();
    std::vector<std::uint64_t> camera_times;
    std::ifstream cameras(root/"camera.tsv");while(std::getline(cameras,line)){std::istringstream row(line);std::uint64_t timestamp;if(row>>timestamp){camera_times.push_back(timestamp);note_time(timestamp);}}
    std::vector<std::string> camera_artifacts;
    if(!camera_times.empty()) {
      std::vector<double> intervals;for(std::size_t i=1;i<camera_times.size();++i)if(camera_times[i]>camera_times[i-1])intervals.push_back((camera_times[i]-camera_times[i-1])/1e9);
      double fps=10;if(!intervals.empty()){auto mid=intervals.begin()+intervals.size()/2;std::nth_element(intervals.begin(),mid,intervals.end());fps=1/(*mid);}
      CameraSource source(root);rec::FfmpegSegmentEncoder encoder({argc>=5?argv[4]:"ffmpeg","libx264"});
      rec::CameraRecordingOptions options;options.session_directory=output;options.nominal_fps=fps;
      const auto segments=rec::CameraSegmentRecorder(options,source,encoder).record();
      camera_artifacts.push_back("camera_color.mcap");
      for(const auto &segment:segments)camera_artifacts.push_back(segment.relative_path);
    }
    const auto provenance=root/"provenance.json";
    if(std::filesystem::is_regular_file(provenance))std::filesystem::copy_file(provenance,output/"provenance.json");
    const std::string catalog=argc>=6?argv[5]:output.generic_string();
    std::ofstream manifest(output/"session.json");
    manifest<<"{\"version\":1,\"session_id\":"<<rec::json_quote(output.filename().string())<<",\"state\":\"completed\",\"session_directory\":"<<rec::json_quote(catalog)
      <<",\"context\":{\"product\":\"dataset_import\",\"robot_id\":\"GEODE_PUBLIC_RECORDING\"},\"manager_pid\":12345,\"created_at_unix_ns\":"<<first
      <<",\"started_at_unix_ns\":"<<first<<",\"ended_at_unix_ns\":"<<last
      <<",\"children\":[";
    if(writer) manifest<<"{\"name\":\"dds\",\"required\":true,\"pid\":0,\"state\":\"exited\",\"exit_code\":0,\"argv\":[],\"artifacts\":[\"dds/sensors.mcap\"],\"selected_topics\":[\"/slam/registered_cloud\",\"/slam/odometry\"],\"required_topics\":[]}";
    if(!camera_times.empty()) {
      if(writer)manifest<<',';
      manifest<<"{\"name\":\"camera\",\"required\":true,\"pid\":0,\"state\":\"exited\",\"exit_code\":0,\"argv\":[],\"artifacts\":[";
      bool first_artifact=true;for(const auto &artifact:camera_artifacts){if(!first_artifact)manifest<<',';first_artifact=false;manifest<<rec::json_quote(artifact);}
      manifest<<"],\"selected_topics\":[],\"required_topics\":[]}";
    }
    manifest<<"]";
    if(std::filesystem::is_regular_file(provenance))manifest<<",\"provenance\":\"provenance.json\"";
    manifest<<"}\n";
    std::cout<<"Imported real source records: "<<output<<" camera frames="<<camera_times.size()<<"\n";
    return 0;
  }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
}

#include "prepared.hpp"
#include "surface_graph.hpp"
#include "lingtu/maps/io/binary.hpp"

#include <chrono>
#include <cstdio>
#include <fstream>

#if defined(_WIN32)
#define NOMINMAX
#include <Windows.h>
#endif

namespace octoplanner3d::runtime {
namespace {
using lingtu::maps::BinaryReader;
using lingtu::maps::BinaryWriter;
using lingtu::nav::plan::MapIdentity;
using global_planner::PlannerConfig;
// Bump when prepared geometry semantics or the binary layout changes.
constexpr const char* kFormat="lingtu.navigation.prepared.2";
constexpr std::uint32_t kEndian=0x01020304U;
struct Header { MapIdentity map; std::uint64_t size; std::int64_t modified; std::array<double,23> preparation; };

Header readHeader(BinaryReader& in) {
  if(in.text()!=kFormat || in.value<std::uint32_t>()!=kEndian)
    throw std::runtime_error("unsupported prepared map format");
  Header header;
  header.map.map_id=in.text();header.map.content_epoch=in.value<std::uint64_t>();
  header.map.frame_id=in.text();
  header.size=in.value<std::uint64_t>();header.modified=in.value<std::int64_t>();
  for(auto& value:header.preparation)value=in.value<double>();
  return header;
}

std::filesystem::path profilePath(const std::filesystem::path& directory,
    const MapIdentity& identity,const PlannerConfig& config,bool exact) {
  if(!std::filesystem::is_directory(directory))return {};
  for(const auto& entry:std::filesystem::directory_iterator(directory)) {
    if(!entry.is_regular_file() || entry.path().extension()!=".bin" ||
        entry.path().filename().string().rfind("profile-",0)!=0)continue;
    try {
      std::ifstream stream(entry.path(),std::ios::binary);BinaryReader in(stream);
      const auto header=readHeader(in);
      if(header.map.map_id==identity.map_id && header.map.frame_id==identity.frame_id &&
          (!exact || header.map.content_epoch==identity.content_epoch) &&
          header.preparation==config.preparation())return entry.path();
    } catch(const std::exception&) { /* An incomplete optional cache is rebuilt. */ }
  }
  return {};
}
}

bool restorePrepared(const std::filesystem::path& directory,const std::filesystem::path& source,const MapIdentity& identity,
    const PlannerConfig& config,const std::shared_ptr<octomap::OcTree>& map,
    global_planner::OctoPlanner3D& planner) {
  try {
    const auto path=profilePath(directory,identity,config,true);
    if(path.empty())return false;
    std::ifstream stream(path,std::ios::binary);BinaryReader in(stream);
    const auto header=readHeader(in);
    if(!lingtu::nav::plan::sameMapIdentity(header.map,identity) ||
        header.preparation!=config.preparation() || header.size!=std::filesystem::file_size(source) ||
        header.modified!=std::filesystem::last_write_time(source).time_since_epoch().count())return false;
    auto surface=global_planner::SurfaceIndex::read(in);
    std::shared_ptr<global_planner::SurfaceGraph> graph;
    const auto has_graph=in.value<std::uint8_t>();
    const bool needs_graph=config.ground_support && config.support_height_m>0 && !config.lowest_traversable_only;
    if(has_graph>1 || bool(has_graph)!=needs_graph || surface->resolution()!=map->getResolution())
      throw std::runtime_error("prepared map does not match geometry");
    if(has_graph)graph=global_planner::SurfaceGraph::read(in,config);
    if(!in.complete())throw std::runtime_error("unexpected prepared map tail");
    planner.setOctomap(map,std::move(surface),std::move(graph));
    std::fprintf(stderr,"OctoPlanner3D restored prepared map: %s\n",path.string().c_str());
    return true;
  } catch(const std::exception& error) {
    std::fprintf(stderr,"OctoPlanner3D prepared map unavailable: %s\n",error.what());
    return false;
  }
}

void savePrepared(const std::filesystem::path& directory,const std::filesystem::path& source,const MapIdentity& identity,
    const PlannerConfig& config,const global_planner::OctoPlanner3D& planner) {
  std::filesystem::path temporary;
  try {
    std::filesystem::create_directories(directory);
    auto path=profilePath(directory,identity,config,false);
    if(path.empty()) {
      std::size_t index=0;
      do {path=directory/("profile-"+std::to_string(index++)+".bin");}
      while(std::filesystem::exists(path));
    }
    temporary=path;temporary+=".tmp-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    {
      std::ofstream stream(temporary,std::ios::binary|std::ios::trunc);BinaryWriter out(stream);
      out.text(kFormat);out.value(kEndian);out.text(identity.map_id);
      out.value<std::uint64_t>(identity.content_epoch);out.text(identity.frame_id);
      out.value<std::uint64_t>(std::filesystem::file_size(source));
      out.value<std::int64_t>(std::filesystem::last_write_time(source).time_since_epoch().count());
      for(double value:config.preparation())out.value(value);
      planner.writePrepared(out);stream.flush();
      if(!stream)throw std::runtime_error("cannot flush prepared map");
    }
#if defined(_WIN32)
    if(!MoveFileExW(temporary.wstring().c_str(),path.wstring().c_str(),MOVEFILE_REPLACE_EXISTING|MOVEFILE_WRITE_THROUGH))
      throw std::runtime_error("cannot publish prepared map");
#else
    std::filesystem::rename(temporary,path);
#endif
    std::fprintf(stderr,"OctoPlanner3D saved prepared map: %s\n",path.string().c_str());
  } catch(const std::exception& error) {
    std::error_code ignored;if(!temporary.empty())std::filesystem::remove(temporary,ignored);
    std::fprintf(stderr,"OctoPlanner3D prepared map not saved: %s\n",error.what());
  }
}

} // namespace octoplanner3d::runtime

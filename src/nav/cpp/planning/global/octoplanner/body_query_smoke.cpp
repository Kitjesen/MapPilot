#include "body_query.hpp"
#include "octoplanner3d_core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>

namespace c = lingtu::nav::collision;
namespace r = octoplanner3d::runtime;

namespace {

void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

}  // namespace

int main() try {
  octomap::OcTree tree(.10);
  for(int x=-4;x<4;++x)for(int y=-4;y<4;++y)for(int z=-4;z<4;++z)
    tree.updateNode((x+.5)*.1,(y+.5)*.1,(z+.5)*.1,false,true);
  tree.search(.05,.05,.05)->setLogOdds(3.5f);
  tree.updateInnerOccupancy();
  const lingtu::nav::plan::GlobalPlanTemporaryOverlay overlay;
  {
    c::Body body;
    body.margin_m = 0;
    c::Part part;
    part.link = "horizontal_margin_probe";
    part.center = {.25,.05,.05};
    part.half_size = {.05,.05,.05};
    body.parts.push_back(part);
    body.horizontal_margin_m = .05;
    require(r::BodyQuery(tree,body,overlay).pose({{},0}).free,
            "five centimetres horizontal margin must preserve a ten centimetre wall gap");
    body.horizontal_margin_m = .12;
    require(!r::BodyQuery(tree,body,overlay).pose({{},0}).free,
            "global body collision must consume configured horizontal margin");
    body.parts.front().center = {.05,.05,.25};
    require(r::BodyQuery(tree,body,overlay).pose({{},0}).free,
            "global horizontal margin must not enlarge the vertical extent");
  }
  for(int axis=0;axis<3;++axis)for(int sign:{-1,1}) {
    c::Body body; body.margin_m=0;
    c::Part part; part.link="contact_probe";part.half_size={.05,.05,.05};
    part.center={.05,.05,.05};
    if(axis==0)part.center.x+=sign*.10;
    if(axis==1)part.center.y+=sign*.10;
    if(axis==2)part.center.z+=sign*.10;
    body.parts.push_back(part);
    r::BodyQuery query(tree,body,overlay);
    const auto contact=query.pose({{},0});
    if(contact.free || contact.reason!="occupied_body")throw std::runtime_error("touching face was omitted");
    c::Pose separated;
    if(axis==0)separated.position.x=sign*.001;
    if(axis==1)separated.position.y=sign*.001;
    if(axis==2)separated.position.z=sign*.001;
    if(!query.pose(separated).free)throw std::runtime_error("separated shape was falsely blocked");
  }
  {
    c::Body body;
    body.margin_m = 0;
    c::Part part;
    part.link = "departure_probe";
    part.center = {.05, .05, .05};
    part.half_size = {.05, .05, .05};
    body.parts.push_back(part);
    r::BodyQuery query(tree, body, overlay);
    const c::Pose near_wall{{-.1005, 0, 0}, 0};
    const c::Pose away{{-.2005, 0, 0}, 0};
    require(query.pose(near_wall).free && query.pose(away).free,
            "departure endpoints must retain their measured clearance");
    require(query.edge(near_wall, away),
            "half-sample padding outside the start blocked a collision-free departure");
    require(query.edge(away, near_wall),
            "half-sample padding outside the goal blocked a collision-free approach");
    const c::Pose colliding{{-.0995, 0, 0}, 0};
    require(!query.pose(colliding).free && !query.edge(colliding, away),
            "departure must still reject a measured start collision");
    require(!query.edge(away, colliding),
            "approach must still reject a measured goal collision");
    const c::Pose other_side{{.1005, 0, 0}, 0};
    require(query.pose(other_side).free && !query.edge(near_wall, other_side),
            "free endpoints must not bypass the occupied volume between them");
  }
  {
    c::Body body;
    body.margin_m = 0;
    c::Part part;
    part.link = "cache_probe";
    part.half_size = {.05, .05, .05};
    body.parts.push_back(part);
    r::BodyQuery query(tree, body, overlay);
    const c::Pose free_pose{{-.25, -.25, -.25}, 0};
    const auto first = query.pose(free_pose);
    const auto second = query.pose(free_pose);
    require(first.free && second.free, "cached free pose changed its result");
    require(query.cellLookups()==0,
            "an unoccupied region should not enumerate every finest-resolution cell");
  }
  {
    octomap::OcTree blocks(.05);
    const int coordinates[]{-17,-16,-1,0,15,16};
    for(int i=0;i<6;++i)for(int j=0;j<6;++j)for(int k=0;k<6;++k) {
      const int state=(i+j+k)%3;
      if(state)blocks.updateNode((coordinates[i]+.5)*.05,(coordinates[j]+.5)*.05,
                                 (coordinates[k]+.5)*.05,state==1,true);
    }
    blocks.updateInnerOccupancy();
    c::Body body;body.margin_m=0;
    c::Part part;part.link="block_boundary";part.half_size={.005,.005,.005};
    body.parts.push_back(part);
    r::BodyQuery query(blocks,body,overlay);
    for(int repeat=0;repeat<2;++repeat)
      for(int i=0;i<6;++i)for(int j=0;j<6;++j)for(int k=0;k<6;++k) {
        const c::Pose pose{{(coordinates[i]+.5)*.05,(coordinates[j]+.5)*.05,
                            (coordinates[k]+.5)*.05},0};
        require(query.pose(pose).free==((i+j+k)%3!=1),
                "negative and positive block boundaries must retain occupied/free/unknown evidence");
      }
    require(query.cellLookups()==144 && query.cellCacheHits()==72,
            "occupied cells must retain cache reuse while empty regions are skipped");
  }
  {
    octomap::OcTree pruned(.05);
    for(int x=0;x<8;++x)for(int y=0;y<8;++y)for(int z=0;z<8;++z)
      pruned.updateNode((x+.5)*.05,(y+.5)*.05,(z+.5)*.05,true,true);
    pruned.updateInnerOccupancy();pruned.prune();
    require(pruned.getNumLeafNodes()==1,"coarse occupied-leaf fixture was not pruned");
    c::Body body;body.margin_m=0;
    c::Part part;part.half_size={.005,.005,.005};body.parts.push_back(part);
    r::BodyQuery query(pruned,body,overlay);
    require(!query.pose({{.395,.025,.025},0}).free &&
            !query.pose({{.405,.025,.025},0}).free &&
             query.pose({{.406,.025,.025},0}).free,
            "occupied leaf outside the query centre must still block its extent and touching face");
  }
  std::cout<<"offline body query passed: contact faces, endpoint-clearance sweeps, unknown air, occupied evidence, cache and coarse leaves\n";
  return 0;
}catch(const std::exception &error){std::cerr<<error.what()<<'\n';return 1;}

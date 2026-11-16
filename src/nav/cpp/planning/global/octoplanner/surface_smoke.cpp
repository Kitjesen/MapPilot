#include "global_planner.h"
#include "mls_backend.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace global_planner {
struct OctoPlannerGridQueryTest {
  static void run() {
    const auto require = [](bool ok, const char *why) {
      if (!ok) throw std::runtime_error(why);
    };
    auto tree = std::make_shared<octomap::OcTree>(.05);
    for (int z : {-1,39}) for (int x=-20; x<=20; ++x) for (int y=-20; y<=20; ++y)
      if (std::abs(x)>1 || std::abs(y)>1)
        tree->updateNode((x+.5)*.05,(y+.5)*.05,(z+.5)*.05,true,true);
    tree->updateNode(-2,-2,-1,false); tree->updateNode(2,2,3,false);
    tree->updateInnerOccupancy();
    const auto nodes = tree->size();

    PlannerConfig config;
    config.robot_radius=.1; config.body_clearance_below_m=.1; config.body_clearance_above_m=.25;
    config.support_height_m=.35; config.support_height_tolerance_m=.05;
    config.direct_support_only=false;
    config.surface_closing_radius_m=0;
    OctoPlanner3D planner;
    planner.setConfig(config); planner.setOctomap(tree);
    const auto raw=planner.artifact_->rust_graph;
    require(bool(raw), "raw Rust surface artifact is missing");
    require(!raw->inferred(0,0,-1,-1), "zero radius filled a hole");
    const octomap::point3d body(.025,.025,.35);
    require(planner.queryWorld(body,.1,true)==OctoPlanner3D::TraversabilityFailure::GroundSupport,
            "fixture was already supported without repair");
    config.surface_closing_radius_m=.2; planner.setConfig(config);
    const auto repaired=planner.artifact_->rust_graph;
    require(bool(repaired), "repaired Rust surface artifact is missing");
    require(repaired->inferred(0,0,-1,-1) && repaired->inferred(0,0,39,39),
            "closing lost one of two surfaces at the same XY");
    require(!planner.isOccupiedCell({0,0,-1}) && tree->size()==nodes,
            "support repair changed observed obstacles");
    require(!repaired->inferred(22,0,-1,-1), "closing expanded the outside floor edge");
    require(repaired->height(.025,.025,0,.05,.05)==0.0 &&
            repaired->height(.025,.025,2,.05,.05)==2.0,
            "shared support query lost inferred or stacked terrain");
    require(!raw->height(.025,.025,0,.05,.05) && !repaired->height(.025,.025,1,.05,.05),
            "shared support query fabricated an unsupported height");
    require(planner.queryWorld(body,.1,true)==OctoPlanner3D::TraversabilityFailure::None,
            "updated closing radius did not repair planner support");
    const auto oldSurface=planner.artifact_->rust_graph;
    auto visibleHole=std::make_shared<octomap::OcTree>(*tree);
    visibleHole->updateNode(.025,.025,-.025,false);
    planner.setOctomap(visibleHole);
    require(!planner.artifact_->rust_graph->inferred(0,0,-1,-1) &&
            !planner.artifact_->rust_graph->height(.025,.025,0,.01,.05),
            "replacement free evidence retained old inferred height");
    require(planner.queryWorld(body,.1,true)==OctoPlanner3D::TraversabilityFailure::GroundSupport,
            "replacement free evidence retained cached planner support");
    require(planner.artifact_->rust_graph && oldSurface!=planner.artifact_->rust_graph,
            "map replacement reused the old Rust surface artifact");
    require(oldSurface->inferred(0,0,-1,-1), "map replacement mutated a retained surface snapshot");
    planner.setOctomap(tree);

    auto oneCellHole=std::make_shared<octomap::OcTree>(.05);
    for (int x=-12;x<=12;++x) for (int y=-12;y<=12;++y)
      if (x || y) oneCellHole->updateNode((x+.5)*.05,(y+.5)*.05,-.025,true,true);
    oneCellHole->updateNode(-2,-2,-1,false);
    oneCellHole->updateNode(2,2,3,false);
    oneCellHole->updateInnerOccupancy();
    PlannerConfig completionConfig=config;

    OctoPlanner3D completion;
    completion.setConfig(completionConfig); completion.setOctomap(oneCellHole);
    require(completion.queryWorld(body,.1,true)==OctoPlanner3D::TraversabilityFailure::None,
            "prepared support rejected a closed one-cell hole");
    auto aboveFloor=std::make_shared<octomap::OcTree>(*oneCellHole);
    aboveFloor->updateNode(.025,.025,.025,false);
    completion.setOctomap(aboveFloor);
    require(completion.queryWorld(body,.1,true)==OctoPlanner3D::TraversabilityFailure::None,
            "prepared support confused air above floor with missing ground");
    auto freeFloor=std::make_shared<octomap::OcTree>(*aboveFloor);
    freeFloor->updateNode(.025,.025,-.025,false);
    completion.setOctomap(freeFloor);
    require(completion.queryWorld(body,.1,true)==OctoPlanner3D::TraversabilityFailure::GroundSupport,
            "prepared support bypassed contradictory free evidence");
    freeFloor->updateNode(.025,.025,-.025,true);
    freeFloor->updateNode(.025,.025,-.025,true);
    completion.setOctomap(freeFloor);
    require(completion.queryWorld(body,.1,true)==OctoPlanner3D::TraversabilityFailure::None,
            "new observed ground did not restore default planner support");
    for (bool through : {false,true}) {
      auto slot=std::make_shared<octomap::OcTree>(.05);
      for (int x=-12;x<=12;++x) for (int y=-12;y<=12;++y)
        if (x || (!through && y>0)) slot->updateNode((x+.5)*.05,(y+.5)*.05,-.025,true,true);
      slot->updateNode(-2,-2,-1,false);
      slot->updateNode(2,2,3,false);
      slot->updateInnerOccupancy();
      const auto slot_nodes=slot->size();
      completion.setOctomap(slot);
      // DimOS closing fills unknown narrow slots, and observed free cells veto inferred support.
      require(completion.artifact_->rust_graph->inferred(0,0,-1,-1) &&
              completion.queryWorld(body,.1,true)==OctoPlanner3D::TraversabilityFailure::None,
              "morphology did not fill an unknown narrow slot");
      require(!slot->search(.025,.025,-.025) && slot->size()==slot_nodes,
              "slot completion changed raw OctoMap evidence");
      auto freeSlot=std::make_shared<octomap::OcTree>(*slot);
      for (int y=-12;y<=12;++y)
        if (through || y<=0) freeSlot->updateNode(.025,(y+.5)*.05,-.025,false,true);
      freeSlot->updateInnerOccupancy();
      completion.setOctomap(freeSlot);
      require(!completion.artifact_->rust_graph->inferred(0,0,-1,-1) &&
              !completion.artifact_->rust_graph->height(.025,.025,0,.05,.05) &&
              completion.queryWorld(body,.1,true)==OctoPlanner3D::TraversabilityFailure::GroundSupport,
              "observed-free slot bypassed prepared support");
      require(!completion.supportConnectionFree({-.075,.025,.35},{.125,.025,.35}),
              "route support crossed an observed-free slot");
      require(completion.queryWorld({.125,.025,.35},.1,true)==OctoPlanner3D::TraversabilityFailure::None,
              "slot rejection removed real support beside it");
    }
    config.surface_closing_radius_m=0; planner.setConfig(config);
    require(!planner.poseSupportFree({.025,.025,.35}), "disabled repair reused stale support");

    auto obstructed=std::make_shared<octomap::OcTree>(*tree);
    obstructed->updateNode(.025,.025,.325,true);
    config.surface_closing_radius_m=.2; planner.setConfig(config); planner.setOctomap(obstructed);
    require(!planner.artifact_->rust_graph->inferred(0,0,-1,-1),
            "repair ignored observed headroom obstacle");
    obstructed->updateNode(.025,.025,-.025,true);
    planner.setOctomap(obstructed);
    require(planner.queryWorld(body,.1,true,false)==OctoPlanner3D::TraversabilityFailure::None,
            "raw observed ground below a headroom obstacle was discarded");
    require(planner.queryWorld(body,.1,false)==OctoPlanner3D::TraversabilityFailure::OccupiedBody,
            "obstacle column index lost a real collision");

    auto lowCeiling=std::make_shared<octomap::OcTree>(*oneCellHole);
    lowCeiling->updateNode(.025,.025,-.025,true);
    lowCeiling->updateNode(.025,.025,.575,true);
    lowCeiling->updateInnerOccupancy();
    completion.setOctomap(lowCeiling);
    require(completion.queryWorld(body,.1,true)==OctoPlanner3D::TraversabilityFailure::OccupiedBody,
            "low-ceiling fixture did not block the nominal stance");
    require(completion.queryWorld({.025,.025,.30},.1,true)==OctoPlanner3D::TraversabilityFailure::None,
            "low ceiling discarded a collision-free lower stance inside the support band");

    auto thinWall=std::make_shared<octomap::OcTree>(*tree);
    thinWall->updateNode(.075,.025,.325,true);
    config.search_resolution_m=.1;
    config.robot_radius=.001;
    planner.setConfig(config); planner.setOctomap(thinWall);
    require(planner.preparePoseQueries({.025,.025,.35}).xy_resolution==.1,
            "search spacing did not change independently of map resolution");
    require(planner.queryWorld({.025,.025,.35},.001,false)==OctoPlanner3D::TraversabilityFailure::None &&
            planner.queryWorld({.125,.025,.35},.001,false)==OctoPlanner3D::TraversabilityFailure::None &&
            !planner.sweptRouteFree({.025,.025,.35},{.125,.025,.35}),
            "coarse search jumped over a fine-map obstacle between its vertices");

    auto gap=std::make_shared<octomap::OcTree>(.05);
    for(int x=-20;x<=20;++x) for(int y=-20;y<=20;++y)
      if(std::abs(x)>8) gap->updateNode((x+.5)*.05,(y+.5)*.05,-.025,true,true);
    gap->updateInnerOccupancy();
    planner.setOctomap(gap);
    require(!planner.artifact_->rust_graph->inferred(0,0,-1,-1),
            "20 cm closing bridged an 85 cm gap");

    tree->prune();
    planner.setOctomap(tree);
    for(int x=-21;x<=21;++x) for(int y=-21;y<=21;++y) for(int z : {-1,0,39,40}) {
      const auto *node=tree->search((x+.5)*.05,(y+.5)*.05,(z+.5)*.05);
      require(planner.isOccupiedCell({x,y,z})==(node && tree->isNodeOccupied(node)),
              "planner occupancy differs from original coarse OctoMap occupancy");
    }
    const std::array<GridIndex,5> hits{{{-9,-2,1},{-8,0,3},{-1,7,2},{0,-1,1},{8,8,2}}};
    auto sparse=std::make_shared<octomap::OcTree>(.05);
    for (const auto &p : hits) sparse->updateNode((p.x+.5)*.05,(p.y+.5)*.05,(p.z+.5)*.05,true);
    config.ground_support=false; planner.setConfig(config); planner.setOctomap(sparse);
    for (int x=-10;x<=8;++x) for (int y=-3;y<=8;++y) for (int z=0;z<4;++z) {
      const bool expected=std::any_of(hits.begin(),hits.end(),[&](const GridIndex &p) {
        return p.x==x && p.y==y && p.z==z;
      });
      require(planner.isOccupiedCell({x,y,z})==expected,
              "planner occupancy lost a hit or included an obstacle outside its voxel");
    }
  }
};
}

int main() try {
  global_planner::OctoPlannerGridQueryTest::run();
  std::cout << "Surface repair passed: layered holes, bounds, obstacle, gap, cache and occupancy\n";
  return 0;
} catch(const std::exception &error) {
  std::cerr << error.what() << '\n';
  return 1;
}

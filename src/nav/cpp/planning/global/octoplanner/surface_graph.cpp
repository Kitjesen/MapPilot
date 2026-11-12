// Multi-source Voronoi corridor construction follows DimOS (Dimensional Inc.),
// Apache-2.0, commit 26144f7fd8424bdc33ab97dcd58e9497f2d413ee, edges.rs/nodes.rs.
// This C++ adaptation uses LingTu's three-dimensional pose and segment queries.
#include "surface_graph.hpp"
#include "route_smoothing.hpp"
#include "lingtu/maps/io/binary.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <queue>

namespace global_planner {
namespace {
double distance(const PointPose &a, const PointPose &b) {
  return std::hypot(a.x-b.x,a.y-b.y,a.z-b.z);
}
using Queue = std::priority_queue<std::pair<double,int>,
    std::vector<std::pair<double,int>>,std::greater<std::pair<double,int>>>;
constexpr double infinity = std::numeric_limits<double>::infinity();
}

GridIndex SurfaceGraph::key(const PointPose &point) const {
  return {static_cast<int>(std::floor(point.x/resolution_+1e-7)),
          static_cast<int>(std::floor(point.y/resolution_+1e-7)),0};
}

void SurfaceGraph::write(lingtu::maps::BinaryWriter& out) const {
  out.value(resolution_);out.value<std::uint64_t>(cells_.size());
  for(const auto& point:cells_){out.value(point.x);out.value(point.y);out.value(point.z);}
  out.value<std::uint64_t>(neighbors_.size());
  for(const auto& links:neighbors_) {
    out.value<std::uint64_t>(links.size());
    for(const auto& link:links){out.value<std::int32_t>(link.to);out.value(link.cost);}
  }
  out.vector(nodes_);out.vector(owner_);out.vector(predecessor_);out.vector(distances_);
  out.value<std::uint64_t>(edges_.size());
  for(const auto& edge:edges_) {
    out.value<std::int32_t>(edge.a);out.value<std::int32_t>(edge.b);out.value(edge.cost);out.vector(edge.cells);
  }
  out.value<std::uint64_t>(node_edges_.size());
  for(const auto& edges:node_edges_)out.vector(edges);
}

std::shared_ptr<SurfaceGraph> SurfaceGraph::read(lingtu::maps::BinaryReader& in,const PlannerConfig& config) {
  auto graph=std::shared_ptr<SurfaceGraph>(new SurfaceGraph);
  graph->config_=config;graph->resolution_=in.value<double>();
  if(!std::isfinite(graph->resolution_) || graph->resolution_<=0)
    throw std::runtime_error("invalid prepared graph resolution");
  const auto cells=in.count(24);graph->cells_.reserve(cells);
  for(std::size_t i=0;i<cells;++i) {
    PointPose point{in.value<double>(),in.value<double>(),in.value<double>()};
    if(!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
      throw std::runtime_error("invalid prepared graph position");
    graph->columns_[graph->key(point)].push_back(static_cast<int>(i));graph->cells_.push_back(point);
  }
  const auto neighbors=in.count(8);
  if(neighbors!=cells)throw std::runtime_error("invalid prepared adjacency size");
  graph->neighbors_.resize(neighbors);
  const auto index=[](int id,std::size_t size) {
    if(id<0 || static_cast<std::size_t>(id)>=size)throw std::runtime_error("invalid prepared graph index");
  };
  for(auto& links:graph->neighbors_) {
    const auto count=in.count(12);links.reserve(count);
    for(std::size_t i=0;i<count;++i) {
      const auto to=in.value<std::int32_t>();const auto cost=in.value<double>();index(to,cells);
      if(!std::isfinite(cost) || cost<=0)throw std::runtime_error("invalid prepared edge cost");
      links.push_back({to,cost});
    }
  }
  graph->nodes_=in.vector<int>();graph->owner_=in.vector<int>();
  graph->predecessor_=in.vector<int>();graph->distances_=in.vector<double>();
  const auto nodes=graph->nodes_.size();
  if(graph->owner_.size()!=cells || graph->predecessor_.size()!=cells || graph->distances_.size()!=cells)
    throw std::runtime_error("invalid prepared graph ownership size");
  for(int id:graph->nodes_)index(id,cells);
  for(std::size_t i=0;i<cells;++i) {
    index(graph->owner_[i],nodes);
    if(!std::isfinite(graph->distances_[i]) || graph->distances_[i]<0)
      throw std::runtime_error("invalid prepared graph distance");
    const int parent=graph->predecessor_[i];
    if(parent!=-1) {
      index(parent,cells);
      if(graph->distances_[parent]>=graph->distances_[i])
        throw std::runtime_error("cyclic prepared graph predecessor");
    }
  }
  const auto edges=in.count(24);graph->edges_.reserve(edges);
  for(std::size_t i=0;i<edges;++i) {
    const auto a=in.value<std::int32_t>(),b=in.value<std::int32_t>();
    const auto cost=in.value<double>();auto chain=in.vector<int>();index(a,nodes);index(b,nodes);
    if(!std::isfinite(cost) || cost<0 || chain.empty())throw std::runtime_error("invalid prepared corridor");
    for(int id:chain)index(id,cells);
    graph->edges_.push_back({a,b,cost,std::move(chain)});
  }
  const auto adjacency=in.count(8);
  if(adjacency!=nodes)throw std::runtime_error("invalid prepared node adjacency");
  graph->node_edges_.reserve(adjacency);
  for(std::size_t i=0;i<adjacency;++i) {
    auto links=in.vector<int>();for(int id:links)index(id,edges);
    graph->node_edges_.push_back(std::move(links));
  }
  return graph;
}

SurfaceGraph::SurfaceGraph(std::vector<PointPose> surfaces, double resolution,
    PlannerConfig config, const PoseCheck &pose_check, const EdgeCheck &edge_check,
    const CancelCheck &cancelled)
    : resolution_(resolution), config_(config) {
  const auto check_cancelled=[&] {if(cancelled&&cancelled())throw Cancelled{};};
  check_cancelled();
  std::sort(surfaces.begin(),surfaces.end(),[](const auto &a,const auto &b) {
    return std::array<double,3>{a.x,a.y,a.z}<std::array<double,3>{b.x,b.y,b.z};
  });
  for(const auto &point:surfaces) {
    check_cancelled();
    if(!pose_check(point))continue;
    auto &column=columns_[key(point)];
    if(!column.empty() && distance(cells_[column.back()],point)<1e-7)continue;
    column.push_back(static_cast<int>(cells_.size()));cells_.push_back(point);
  }
  neighbors_.resize(cells_.size());
  const int stride=std::max(1,static_cast<int>(std::ceil(config.search_resolution_m/resolution_)));
  // One quantized height step may need a longer horizontal connection than the search grid.
  const int height_stride=config.max_slope>0 ?
      std::max(stride,static_cast<int>(std::ceil(1./config.max_slope))) : stride;
  for(int i=0;i<static_cast<int>(cells_.size());++i) {
    check_cancelled();
    const auto &a=cells_[i];const auto xy=key(a);
    for(int hop=1;hop<=height_stride;++hop) for(int dx=-1;dx<=1;++dx)for(int dy=-1;dy<=1;++dy) {
      if(!dx&&!dy)continue;
      const auto column=columns_.find({xy.x+dx*hop,xy.y+dy*hop,0});
      if(column==columns_.end())continue;
      for(int j:column->second) {
        if(j<=i)continue;
        const auto &b=cells_[j];const double rise=std::abs(b.z-a.z);
        if(hop>stride && rise<1e-6)continue;
        const double run=std::hypot(b.x-a.x,b.y-a.y);
        if((config.max_step_height>0 && rise>config.max_step_height+1e-6) ||
           (config.support_height_m<=0 && config.max_slope>0 && rise>config.max_slope*run+1e-6))continue;
        if(!edge_check(a,b))continue;
        const double cost=distance(a,b)+std::max(0.,config.floor_change_penalty)*rise;
        neighbors_[i].push_back({j,cost});neighbors_[j].push_back({i,cost});
      }
    }
  }
  // Graph-boundary distance is a soft preference; actual clearance was checked above.
  std::vector<double> wall(cells_.size(),infinity);Queue queue;
  for(int i=0;i<static_cast<int>(cells_.size());++i) {
    check_cancelled();
    if(neighbors_[i].size()<static_cast<std::size_t>(8*stride)) {
      wall[i]=0;queue.push({0,i});
    }
  }
  while(!queue.empty()) {
    check_cancelled();
    const auto [d,u]=queue.top();queue.pop();if(d!=wall[u])continue;
    for(const auto &edge:neighbors_[u]) {
      const double nd=d+distance(cells_[u],cells_[edge.to]);
      if(nd<wall[edge.to]){wall[edge.to]=nd;queue.push({nd,edge.to});}
    }
  }
  const double buffer=std::max(resolution_,config.obstacle_clearance_radius_cells*resolution_);
  for(int i=0;i<static_cast<int>(cells_.size());++i) {
    check_cancelled();
    for(auto &edge:neighbors_[i]) {
      const double gap=std::min(wall[i],wall[edge.to]);
      const double penalty=std::max(0.,1.-gap/buffer);
      edge.cost+=distance(cells_[i],cells_[edge.to])*std::max(0.,config.obstacle_clearance_weight)*penalty*penalty;
    }
  }
  std::vector<int> order(cells_.size());std::iota(order.begin(),order.end(),0);
  std::stable_sort(order.begin(),order.end(),[&](int a,int b){return wall[a]>wall[b];});
  std::unordered_map<GridIndex,std::vector<int>,GridIndexHash> buckets;
  for(int id:order) {
    check_cancelled();
    const auto &p=cells_[id];const GridIndex bucket{int(std::floor(p.x)),int(std::floor(p.y)),int(std::floor(p.z))};
    bool near=false;
    for(int dx=-1;dx<=1&&!near;++dx)for(int dy=-1;dy<=1&&!near;++dy)for(int dz=-1;dz<=1&&!near;++dz) {
      const auto found=buckets.find({bucket.x+dx,bucket.y+dy,bucket.z+dz});
      if(found!=buckets.end()) for(int other:found->second)
        if(distance(p,cells_[other])<1. && std::abs(p.z-cells_[other].z)<=config.max_step_height) {near=true;break;}
    }
    if(!near){nodes_.push_back(id);buckets[bucket].push_back(id);}
  }
  owner_.assign(cells_.size(),-1);predecessor_.assign(cells_.size(),-1);
  distances_.assign(cells_.size(),infinity);
  const auto seed=[&](int id,int owner){owner_[id]=owner;distances_[id]=0;queue.push({0,id});};
  for(int i=0;i<static_cast<int>(nodes_.size());++i)seed(nodes_[i],i);
  const auto flood=[&] {
    while(!queue.empty()) {
      check_cancelled();
      const auto [d,u]=queue.top();queue.pop();if(d!=distances_[u])continue;
      for(const auto &edge:neighbors_[u])if(d+edge.cost<distances_[edge.to]) {
        distances_[edge.to]=d+edge.cost;owner_[edge.to]=owner_[u];predecessor_[edge.to]=u;
        queue.push({distances_[edge.to],edge.to});
      }
    }
  };
  flood();
  for(int i=0;i<static_cast<int>(cells_.size());++i) {
    check_cancelled();
    if(owner_[i]<0) {seed(i,static_cast<int>(nodes_.size()));nodes_.push_back(i);flood();}
  }
  struct Crossing { int u,v; double cost; };
  std::unordered_map<std::uint64_t,Crossing> boundaries;
  for(int u=0;u<static_cast<int>(cells_.size());++u) {
    check_cancelled();
    for(const auto &edge:neighbors_[u]) {
      const int v=edge.to,a=owner_[u],b=owner_[v];if(a>=b)continue;
      const auto k=(std::uint64_t(a)<<32)|std::uint32_t(b);
      const double cost=distances_[u]+edge.cost+distances_[v];
      const auto found=boundaries.find(k);
      if(found==boundaries.end() || cost<found->second.cost)boundaries[k]={u,v,cost};
    }
  }
  node_edges_.resize(nodes_.size());
  std::vector<std::uint64_t> keys;for(const auto &entry:boundaries)keys.push_back(entry.first);
  std::sort(keys.begin(),keys.end());
  for(auto k:keys) {
    check_cancelled();
    const auto &cross=boundaries.at(k);auto chain=toNode(cross.u);std::reverse(chain.begin(),chain.end());
    auto tail=toNode(cross.v);chain.insert(chain.end(),tail.begin(),tail.end());
    const int a=owner_[cross.u],b=owner_[cross.v],id=static_cast<int>(edges_.size());
    edges_.push_back({a,b,cross.cost,std::move(chain)});
    node_edges_[a].push_back(id);node_edges_[b].push_back(id);
  }
}

std::vector<int> SurfaceGraph::toNode(int cell) const {
  std::vector<int> path;
  for(int p=cell;p>=0;p=predecessor_[p])path.push_back(p);
  return path;
}

std::vector<SurfaceGraph::Attachment> SurfaceGraph::attachments(const PointPose &point,
    bool goal,const EdgeCheck &check,const CancelCheck &cancelled) const {
  const double radius=config_.snap_search_radius_cells*resolution_;
  const int range=static_cast<int>(std::ceil(radius/resolution_));const auto xy=key(point);
  std::vector<std::pair<double,int>> candidates;
  for(int dx=-range;dx<=range;++dx)for(int dy=-range;dy<=range;++dy) {
    const auto col=columns_.find({xy.x+dx,xy.y+dy,0});if(col==columns_.end())continue;
    for(int id:col->second) {
      const auto &p=cells_[id];const double d=distance(point,p);
      if(d>radius+1e-6)continue;
      candidates.push_back({d,id});
    }
  }
  std::sort(candidates.begin(),candidates.end());
  std::vector<Attachment> out;
  for(std::size_t i=0;i<std::min<std::size_t>(64,candidates.size());++i) {
    if(cancelled&&cancelled())return {};
    const auto [d,id]=candidates[i];
    auto chain=toNode(id);
    std::vector<PointPose> path{point};
    for(int cell:chain)path.push_back(cells_[cell]);
    auto entry=std::vector<PointPose>{point,cells_[id]};
    // The terminal is already resolved; this connector must reach it exactly.
    if(goal)std::reverse(entry.begin(),entry.end());
    if(checkRoute(entry,check,cancelled))
      out.push_back({owner_[id],d+distances_[id],std::move(path),id,1});
  }
  if(out.empty() && !candidates.empty())
    if(auto local=localAttachment(point,candidates,check,cancelled))out.push_back(std::move(*local));
  return out;
}

std::optional<SurfaceGraph::Attachment> SurfaceGraph::localAttachment(
    const PointPose &point,const std::vector<std::pair<double,int>> &candidates,
    const EdgeCheck &check,const CancelCheck &cancelled) const {
  const double radius=config_.snap_search_radius_cells*resolution_;
  const double xy=std::max(resolution_,config_.search_resolution_m);
  const double z=config_.max_slope>0 ? xy/std::max(1.,std::ceil(1./config_.max_slope)) : xy;
  const auto position=[&](const GridIndex &cell) {
    return PointPose{point.x+cell.x*xy,point.y+cell.y*xy,point.z+cell.z*z};
  };
  const auto estimate=[&](const PointPose &p) {
    double best=infinity;
    for(const auto &candidate:candidates)best=std::min(best,distance(p,cells_[candidate.second]));
    return best;
  };
  std::unordered_set<int> candidate_ids;
  for(const auto &candidate:candidates)candidate_ids.insert(candidate.second);
  struct State { double cost; GridIndex parent; };
  const GridIndex origin{0,0,0};
  std::unordered_map<GridIndex,State,GridIndexHash> states;
  states.emplace(origin,State{0,origin});
  std::priority_queue<QueueNode,std::vector<QueueNode>,QueueNodeCompare> queue;
  queue.push({origin,estimate(point),0});
  int iterations=0;
  while(!queue.empty()) {
    if(cancelled&&cancelled())return {};
    const auto current=queue.top();queue.pop();
    const auto state=states.find(current.idx);
    if(current.g>state->second.cost)continue;
    if(++iterations>config_.max_iterations)return {};
    const auto p=position(current.idx);
    const auto column_key=key(p);
    const int nearby=static_cast<int>(std::ceil(xy/resolution_));
    for(int dx=-nearby;dx<=nearby;++dx)for(int dy=-nearby;dy<=nearby;++dy) {
      const auto column=columns_.find({column_key.x+dx,column_key.y+dy,0});
      if(column==columns_.end())continue;
      for(int id:column->second) {
        const auto &target=cells_[id];
        if(!candidate_ids.count(id) || distance(p,target)>std::sqrt(3.)*xy || !check(p,target))continue;
        auto chain=toNode(id);
        std::vector<PointPose> path;
        for(auto cell=current.idx;;cell=states.at(cell).parent) {
          path.push_back(position(cell));if(cell==origin)break;
        }
        std::reverse(path.begin(),path.end());
        const auto entry=path.size();
        for(int cell:chain)path.push_back(cells_[cell]);
        std::printf("OctoPlanner3D surface local attachment: iterations=%d waypoints=%zu\n",iterations,path.size());
        return Attachment{owner_[id],current.g+distance(p,cells_[id])+distances_[id],std::move(path),id,entry};
      }
    }
    for(int dx=-1;dx<=1;++dx)for(int dy=-1;dy<=1;++dy)for(int dz=-1;dz<=1;++dz) {
      if(!dx&&!dy&&!dz)continue;
      const double rise=std::abs(dz*z),run=std::hypot(dx*xy,dy*xy);
      if((rise>0 && run==0) || (config_.max_step_height>0 && rise>config_.max_step_height+1e-6) ||
         (config_.max_slope>0 && rise>config_.max_slope*run+1e-6))continue;
      const GridIndex next{current.idx.x+dx,current.idx.y+dy,current.idx.z+dz};
      auto q=position(next);
      if(distance(point,q)>radius+1e-6)continue;
      const double cost=current.g+distance(p,q)+std::max(0.,config_.floor_change_penalty)*rise;
      const auto found=states.find(next);
      if(found!=states.end() && found->second.cost<=cost)continue;
      if(!check(p,q))continue;
      states.insert_or_assign(next,State{cost,current.idx});
      queue.push({next,cost+estimate(q),cost});
    }
  }
  return {};
}

std::vector<PointPose> SurfaceGraph::plan(const PointPose &start,const PointPose &goal,
    const EdgeCheck &check,const CancelCheck &cancelled,const EdgeCheck &prepared_edge_check) const {
  const auto &edge_check=prepared_edge_check ? prepared_edge_check : check;
  if(nodes_.empty() || (cancelled&&cancelled()))return {};
  const auto starts=attachments(start,false,check,cancelled);
  const auto goals=attachments(goal,true,check,cancelled);
  if(starts.empty()||goals.empty())return {};
  struct State {
    double cost{infinity};
    int parent{-1};
    std::vector<PointPose> segment;
  };
  std::vector<State> states(nodes_.size());
  const auto estimate=[&](int node) { return distance(cells_[nodes_[node]],goal); };
  Queue queue;
  for(int i=0;i<static_cast<int>(starts.size());++i) {
    auto path=starts[i].path;
    if(!checkRoute(path,check,cancelled))continue;
    const int node=starts[i].node;
    auto &state=states[node];
    if (starts[i].cost>=state.cost) continue;
    state.cost=starts[i].cost;state.segment=std::move(path);
    queue.push({state.cost+estimate(node),node});
  }
  int iterations=0,finish=-1;
  double best=infinity;
  std::vector<PointPose> terminal;
  while(!queue.empty()) {
    if(cancelled&&cancelled())return {};
    const auto [f,label]=queue.top();queue.pop();
    const double cost=states[label].cost;
    if(f!=cost+estimate(label))continue;
    if(f>=best || ++iterations>config_.max_iterations)break;
    for(const auto &attachment:goals) {
      if(attachment.node!=label || cost+attachment.cost>=best)continue;
      auto tail=attachment.path;std::reverse(tail.begin(),tail.end());
      if(!checkRoute(tail,check,cancelled))continue;
      finish=label;best=cost+attachment.cost;terminal=std::move(tail);
    }
    for(int id:node_edges_[label]) {
      const auto &edge=edges_[id];const bool forward=edge.a==label;
      const int next_label=forward?edge.b:edge.a;
      if(cost+edge.cost>=states[next_label].cost)continue;

      auto chain=edge.cells;if(!forward)std::reverse(chain.begin(),chain.end());
      std::vector<PointPose> segment;
      for(int cell:chain)segment.push_back(cells_[cell]);
      if (!checkRoute(segment,edge_check,cancelled)) continue;
      auto &next=states[next_label];
      next.cost=cost+edge.cost;
      next.parent=label;
      next.segment=std::move(segment);
      queue.push({next.cost+estimate(next_label),next_label});
    }
  }
  if(cancelled&&cancelled())return {};
  std::vector<PointPose> path;
  std::vector<bool> allowed(nodes_.size(),false);
  if(finish>=0) {
    std::vector<int> labels;
    for(int label=finish;label>=0;label=states[label].parent)labels.push_back(label);
    std::reverse(labels.begin(),labels.end());
    for(int label:labels) {
      const auto &state=states[label];allowed[label]=true;
      for(const auto &point:state.segment)
        if(path.empty() || distance(path.back(),point)>1e-7)path.push_back(point);
    }
    for(const auto &point:terminal)
      if(path.empty() || distance(path.back(),point)>1e-7)path.push_back(point);
    if(!path.empty())path.back().yaw=goal.yaw ? goal.yaw : terminal.back().yaw;
    auto refined=searchCells(starts,goals,allowed,check,cancelled,iterations,best,edge_check);
    const auto length=[](const auto &points) {
      double total=0;
      for(std::size_t i=1;i<points.size();++i)total+=distance(points[i-1],points[i]);
      return total;
    };
    if(!refined.empty() && length(refined)<=length(path)+1e-7)return refined;
    return cancelled&&cancelled() ? std::vector<PointPose>{} : path;
  }
  // A blocked corridor can still have a detour through its support cells.
  iterations=0;
  return searchCells(starts,goals,{},check,cancelled,iterations,infinity,edge_check);
}

std::vector<PointPose> SurfaceGraph::searchCells(const std::vector<Attachment> &starts,
    const std::vector<Attachment> &goals,const std::vector<bool> &allowed,
    const EdgeCheck &check,const CancelCheck &cancelled,int &iterations,double upper,
    const EdgeCheck &prepared_edge_check) const {
  struct State {
    double cost;
    int parent;
    int origin;
  };
  std::unordered_map<int,State> states;
  std::priority_queue<QueueNode,std::vector<QueueNode>,QueueNodeCompare> queue;
  const auto estimate=[&](int cell) {
    double result=infinity;
    for(const auto &goal:goals)result=std::min(result,distance(cells_[cell],cells_[goal.cell]));
    return result;
  };
  for(int i=0;i<static_cast<int>(starts.size());++i) {
    const auto &attachment=starts[i];
    auto prefix=std::vector<PointPose>(attachment.path.begin(),attachment.path.begin()+attachment.entry+1);
    if(!checkRoute(prefix,check,cancelled))continue;
    double cost=0;
    for(std::size_t j=1;j<prefix.size();++j)cost+=distance(prefix[j-1],prefix[j]);
    const int cell=attachment.cell;
    const auto found=states.find(cell);
    if (found!=states.end() && found->second.cost<=cost) continue;
    states.insert_or_assign(cell,State{cost,cell,i});
    queue.push({{cell,0,0},cost+estimate(cell),cost});
  }
  while(!queue.empty()) {
    if(cancelled&&cancelled())return {};
    const auto current=queue.top();queue.pop();
    const int cell=current.idx.x;
    const auto state=states.at(cell);
    if(current.g!=state.cost)continue;
    if(current.f>=upper)break;
    if(++iterations>config_.max_iterations)break;
    const auto &from=cells_[cell];
    for(const auto &goal:goals) {
      if(goal.cell!=cell)continue;
      std::vector<PointPose> tail(goal.path.begin(),goal.path.begin()+goal.entry+1);
      std::reverse(tail.begin(),tail.end());
      if(!checkRoute(tail,check,cancelled))continue;
      std::vector<PointPose> result;
      for(auto previous=cell;;previous=states.at(previous).parent) {
        result.push_back(cells_[previous]);
        if(previous==states.at(previous).parent)break;
      }
      std::reverse(result.begin(),result.end());
      const auto &start=starts[state.origin];
      result.insert(result.begin(),start.path.begin(),start.path.begin()+start.entry);
      result.insert(result.end(),tail.begin()+1,tail.end());
      if(!checkRoute(result,check,cancelled))continue;
      return result;
    }
    for(const auto &edge:neighbors_[cell]) {
      if(!allowed.empty() && !allowed[owner_[edge.to]])continue;
      const auto &to=cells_[edge.to];
      const int next=edge.to;
      const double cost=state.cost+edge.cost;
      const auto found=states.find(next);
      if(found!=states.end() && found->second.cost<=cost)continue;
      const double estimate_cost=cost+estimate(edge.to);
      if(estimate_cost>=upper)continue;
      if(!prepared_edge_check(from,to))continue;
      states.insert_or_assign(next,State{cost,cell,state.origin});
      queue.push({{next,0,0},estimate_cost,cost});
    }
  }
  return {};
}

}  // namespace global_planner

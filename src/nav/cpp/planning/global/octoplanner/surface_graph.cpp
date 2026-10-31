// Multi-source Voronoi corridor construction follows DimOS (Dimensional Inc.),
// Apache-2.0, commit 26144f7fd8424bdc33ab97dcd58e9497f2d413ee, edges.rs/nodes.rs.
// This C++ adaptation uses LingTu's three-dimensional pose and segment queries.
#include "surface_graph.hpp"

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
  for(int i=0;i<static_cast<int>(cells_.size());++i) {
    check_cancelled();
    const auto &a=cells_[i];const auto xy=key(a);
    for(int hop=1;hop<=stride;++hop) for(int dx=-1;dx<=1;++dx)for(int dy=-1;dy<=1;++dy) {
      if(!dx&&!dy)continue;
      const auto column=columns_.find({xy.x+dx*hop,xy.y+dy*hop,0});
      if(column==columns_.end())continue;
      for(int j:column->second) {
        if(j<=i)continue;
        const auto &b=cells_[j];const double rise=std::abs(b.z-a.z);
        const double run=std::hypot(b.x-a.x,b.y-a.y);
        if((config.max_step_height>0 && rise>config.max_step_height+1e-6) ||
           (config.max_slope>0 && rise>config.max_slope*run+1e-6))continue;
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
      if(goal && (d>config_.terminal_goal_tolerance_m+1e-6 ||
          std::hypot(p.x-point.x,p.y-point.y)>config_.terminal_goal_xy_tolerance_m+1e-6 ||
          std::abs(p.z-point.z)>config_.terminal_goal_z_tolerance_m+1e-6))continue;
      candidates.push_back({d,id});
    }
  }
  std::sort(candidates.begin(),candidates.end());
  std::unordered_map<int,Attachment> best;
  // DimOS also bounds endpoint attempts; failure here falls back to full XYZ search.
  for(std::size_t i=0;i<std::min<std::size_t>(64,candidates.size());++i) {
    if(cancelled&&cancelled())return {};
    const auto [d,id]=candidates[i];const int owner=owner_[id];const double cost=d+distances_[id];
    const auto found=best.find(owner);if(found!=best.end()&&found->second.cost<=cost)continue;
    auto chain=toNode(id);std::vector<PointPose> path{point};
    for(int cell:chain)path.push_back(cells_[cell]);
    bool valid=true;
    for(std::size_t j=1;j<path.size();++j) if(!check(path[j-1],path[j])){valid=false;break;}
    if(valid)best[owner]={owner,cost,std::move(path),id,1};
  }
  std::vector<Attachment> out;for(auto &entry:best)out.push_back(std::move(entry.second));
  if(out.empty() && !candidates.empty())
    if(auto local=localAttachment(point,candidates,check,cancelled))out.push_back(std::move(*local));
  std::sort(out.begin(),out.end(),[](const auto &a,const auto &b){return a.node<b.node;});
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
    const auto p=position(current.idx);const auto column_key=key(p);
    const int nearby=static_cast<int>(std::ceil(xy/resolution_));
    for(int dx=-nearby;dx<=nearby;++dx)for(int dy=-nearby;dy<=nearby;++dy) {
      const auto column=columns_.find({column_key.x+dx,column_key.y+dy,0});
      if(column==columns_.end())continue;
      for(int id:column->second) {
        if(!candidate_ids.count(id) || distance(p,cells_[id])>std::sqrt(3.)*xy || !check(p,cells_[id]))continue;
        auto chain=toNode(id);bool valid=true;
        for(std::size_t j=1;j<chain.size();++j)
          if(!check(cells_[chain[j-1]],cells_[chain[j]])){valid=false;break;}
        if(!valid)continue;
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
      const auto q=position(next);
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
    const EdgeCheck &check,const CancelCheck &cancelled) const {
  if(nodes_.empty() || (cancelled&&cancelled()) || !check(start,start))return {};
  const auto starts=attachments(start,false,check,cancelled),goals=attachments(goal,true,check,cancelled);
  if(starts.empty()||goals.empty())return {};
  std::vector<signed char> validity(edges_.size(),0);
  int iterations=0;
  for(;;) {
    if(cancelled&&cancelled())return {};
    std::vector<double> costs(nodes_.size(),infinity);std::vector<int> previous(nodes_.size(),-1),via(nodes_.size(),-1),origin(nodes_.size(),-1);
    Queue queue;
    for(int i=0;i<static_cast<int>(starts.size());++i) {
      const auto &a=starts[i];if(a.cost<costs[a.node]){costs[a.node]=a.cost;origin[a.node]=i;queue.push({a.cost,a.node});}
    }
    while(!queue.empty()) {
      if(cancelled&&cancelled())return {};
      const auto [d,u]=queue.top();queue.pop();if(d!=costs[u])continue;
      if(++iterations>config_.max_iterations)return {};
      for(int id:node_edges_[u]) {
        if(validity[id]<0)continue;
        const auto &edge=edges_[id];const int v=edge.a==u?edge.b:edge.a;
        if(d+edge.cost<costs[v]){costs[v]=d+edge.cost;previous[v]=u;via[v]=id;origin[v]=origin[u];queue.push({costs[v],v});}
      }
    }
    int finish=-1;double best=infinity;
    for(int i=0;i<static_cast<int>(goals.size());++i)if(costs[goals[i].node]+goals[i].cost<best) {
      best=costs[goals[i].node]+goals[i].cost;finish=i;
    }
    if(finish<0)return {};
    std::vector<std::pair<int,bool>> route;
    int node=goals[finish].node;
    for(int u=node;via[u]>=0;u=previous[u])route.push_back({via[u],edges_[via[u]].a==previous[u]});
    std::reverse(route.begin(),route.end());bool blocked=false;
    for(const auto &[id,forward]:route) {
      (void)forward;if(validity[id])continue;
      const auto &chain=edges_[id].cells;bool valid=true;
      for(std::size_t j=1;j<chain.size();++j)if(!check(cells_[chain[j-1]],cells_[chain[j]])){valid=false;break;}
      validity[id]=valid?1:-1;if(!valid){blocked=true;break;}
    }
    if(blocked)continue;
    auto path=starts[origin[node]].path;
    for(const auto &[id,forward]:route) {
      const auto &chain=edges_[id].cells;
      if(forward)for(int cell:chain)path.push_back(cells_[cell]);
      else for(auto it=chain.rbegin();it!=chain.rend();++it)path.push_back(cells_[*it]);
    }
    const auto &tail=goals[finish].path;path.insert(path.end(),tail.rbegin(),tail.rend());
    path.erase(std::unique(path.begin(),path.end(),[](const auto &a,const auto &b){return distance(a,b)<1e-7;}),path.end());
    return refineCorridor(std::move(path),starts[origin[node]],goals[finish],route,check,cancelled,iterations);
  }
}

std::vector<PointPose> SurfaceGraph::refineCorridor(std::vector<PointPose> path,
    const Attachment &start,const Attachment &goal,
    const std::vector<std::pair<int,bool>> &route,const EdgeCheck &check,
    const CancelCheck &cancelled,int &iterations) const {
  // Cached fine edges may bypass owner centres, but never leave the chosen owners.
  std::vector<bool> allowed(nodes_.size(),false);
  allowed[start.node]=true;
  allowed[goal.node]=true;
  double upper=distances_[start.cell]+distances_[goal.cell];
  for(const auto &part:route) {
    const auto &edge=edges_[part.first];
    allowed[edge.a]=true;
    allowed[edge.b]=true;
    upper+=edge.cost;
  }
  const auto length=[](const std::vector<PointPose> &points) {
    double total=0;
    for(std::size_t i=1;i<points.size();++i)total+=distance(points[i-1],points[i]);
    return total;
  };
  struct State { double cost; int parent; };
  std::unordered_map<int,State> states;
  states.emplace(start.cell,State{0,-1});
  Queue fine;
  fine.push({distance(cells_[start.cell],cells_[goal.cell]),start.cell});
  const int coarse_iterations=iterations;
  while(!fine.empty()) {
    if(cancelled&&cancelled())return {};
    const auto [f,u]=fine.top();
    fine.pop();
    const double g=states.at(u).cost;
    if(f>g+distance(cells_[u],cells_[goal.cell]))continue;
    if(f>=upper)break;
    if(u==goal.cell) {
      std::vector<PointPose> candidate;
      for(int cell=goal.cell;cell>=0;cell=states.at(cell).parent)
        candidate.push_back(cells_[cell]);
      std::reverse(candidate.begin(),candidate.end());
      candidate.insert(candidate.begin(),start.path.begin(),start.path.begin()+start.entry);
      for(std::size_t i=goal.entry;i>0;--i)candidate.push_back(goal.path[i-1]);
      candidate.erase(std::unique(candidate.begin(),candidate.end(),[](const auto &a,const auto &b) {
        return distance(a,b)<1e-7;
      }),candidate.end());
      const double original_length=length(path),refined_length=length(candidate);
      if(refined_length>original_length+1e-7)break;
      for(std::size_t i=1;i<candidate.size();++i) {
        if(cancelled&&cancelled())return {};
        if(!check(candidate[i-1],candidate[i])) {
          if(cancelled&&cancelled())return {};
          return path;
        }
      }
      std::printf("OctoPlanner3D surface corridor refinement: expanded=%d cost=%.3f->%.3f length=%.3f->%.3f\n",
                  iterations-coarse_iterations,upper,g,original_length,refined_length);
      return candidate;
    }
    if(++iterations>config_.max_iterations)break;
    for(const auto &edge:neighbors_[u]) {
      const int v=edge.to;
      if(!allowed[owner_[v]])continue;
      const double next=g+edge.cost,h=distance(cells_[v],cells_[goal.cell]);
      if(next+h>=upper)continue;
      const auto state=states.find(v);
      if(state!=states.end()&&state->second.cost<=next)continue;
      states.insert_or_assign(v,State{next,u});
      fine.push({next+h,v});
    }
  }
  return path;
}

}  // namespace global_planner

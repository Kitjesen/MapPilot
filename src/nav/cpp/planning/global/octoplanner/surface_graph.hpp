#pragma once

#include "global_planner.h"
#include <optional>

namespace global_planner {

class SurfaceGraph {
 public:
  using PoseCheck = std::function<bool(const PointPose &)>;
  using EdgeCheck = std::function<bool(const PointPose &, const PointPose &)>;
  using CancelCheck = std::function<bool()>;
  struct Cancelled {};

  SurfaceGraph(std::vector<PointPose> surfaces, double resolution,
               PlannerConfig config, const PoseCheck &pose_check,
               const EdgeCheck &edge_check, const CancelCheck &cancelled = {});
  std::vector<PointPose> plan(const PointPose &start, const PointPose &goal,
                              const EdgeCheck &current_edge_check,
                              const CancelCheck &cancelled = {},
                              const EdgeCheck &prepared_edge_check = {}) const;
  std::size_t nodeCount() const { return nodes_.size(); }
  std::size_t edgeCount() const { return edges_.size(); }
  std::size_t componentCount() const;
  void setQuery(int iterations, int snap_radius, bool same_floor_preference,
                double same_floor_z_tolerance) {
    config_.max_iterations=iterations;
    config_.snap_search_radius_cells=snap_radius;
    config_.same_floor_preference=same_floor_preference;
    config_.same_floor_z_tolerance=same_floor_z_tolerance;
  }
  void write(lingtu::maps::BinaryWriter& out) const;
  static std::shared_ptr<SurfaceGraph> read(lingtu::maps::BinaryReader& in,const PlannerConfig& config);

 private:
  SurfaceGraph() = default;
  friend struct OctoPlannerGridQueryTest;
  struct Link { int to; double cost; };
  struct Corridor { int a, b; double cost; std::vector<int> cells; };
  struct Attachment {
    int node;
    double cost;
    std::vector<PointPose> path;
    int cell;
    std::size_t entry;
  };
  std::vector<Attachment> attachments(const PointPose &point, bool goal,
                                     const EdgeCheck &check,
                                     const CancelCheck &cancelled) const;
  std::optional<Attachment> localAttachment(const PointPose &point,
      const std::vector<std::pair<double,int>> &candidates,
      const EdgeCheck &check, const CancelCheck &cancelled) const;
  double sameFloorPenalty(double z, double start_z, bool same_floor_request) const;
  double sameFloorPenalty(const std::vector<PointPose> &path, double start_z,
                          bool same_floor_request) const;
  double sameFloorPenalty(const std::vector<int> &cells, double start_z,
                          bool same_floor_request) const;
  std::vector<PointPose> searchCells(const std::vector<Attachment> &starts,
      const std::vector<Attachment> &goals, const std::vector<bool> &allowed,
      const EdgeCheck &check, const CancelCheck &cancelled, int &iterations,
      double upper, const EdgeCheck &prepared_edge_check, double start_z,
      bool same_floor_request) const;
  std::vector<int> toNode(int cell) const;
  GridIndex key(const PointPose &point) const;
  double resolution_;
  PlannerConfig config_;
  std::vector<PointPose> cells_;
  std::unordered_map<GridIndex, std::vector<int>, GridIndexHash> columns_;
  std::vector<std::vector<Link>> neighbors_;
  std::vector<int> nodes_, owner_, predecessor_;
  std::vector<double> distances_;
  std::vector<Corridor> edges_;
  std::vector<std::vector<int>> node_edges_;
};

}  // namespace global_planner

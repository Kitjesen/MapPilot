#pragma once

#include <cstdint>
#include <filesystem>
#include <functional>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace lingtu::maps {

enum class MapGraphNodeType {
  kPoi,
  kPortal,
};

enum class MapGraphEdgeDirection {
  kForward,
  kBidirectional,
};

struct MapGraphPose {
  std::string frame_id{"map"};
  double x_m{0.0};
  double y_m{0.0};
  double z_m{0.0};
  std::optional<double> yaw_rad;
};

struct MapGraphNode {
  std::string node_id;
  std::string map_id;
  MapGraphNodeType type{MapGraphNodeType::kPoi};
  std::string label;
  MapGraphPose pose;
  std::unordered_map<std::string, std::string> metadata;
};

struct MapGraphEdge {
  std::string edge_id;
  std::string from_node_id;
  std::string to_node_id;
  double cost{1.0};
  MapGraphEdgeDirection direction{MapGraphEdgeDirection::kForward};
  bool enabled{true};
  double health{1.0};
  std::unordered_map<std::string, std::string> transition_metadata;
};

struct MapGraphTransition {
  std::string edge_id;
  std::string from_map_id;
  std::string to_map_id;
  std::unordered_map<std::string, std::string> metadata;
};

struct MapGraphRoute {
  bool found{false};
  double total_cost{0.0};
  std::vector<std::string> node_ids;
  std::vector<std::string> edge_ids;
  std::vector<MapGraphTransition> transitions;
};

struct MapGraphResult {
  bool ok{false};
  bool corrupt{false};
  std::string message;
};

class MapGraph {
 public:
  using MapIdValidator = std::function<bool(const std::string&)>;

  explicit MapGraph(MapIdValidator validator = {});

  std::int64_t Version() const { return version_; }
  std::size_t NodeCount() const { return nodes_.size(); }
  std::size_t EdgeCount() const { return edges_.size(); }

  MapGraphResult AddNode(MapGraphNode node);
  MapGraphResult AddEdge(MapGraphEdge edge);
  MapGraphResult RemoveEdge(const std::string& edge_id);
  MapGraphResult RemoveNodeIfUnreferenced(const std::string& node_id);
  MapGraphResult SetEdgeEnabled(const std::string& edge_id, bool enabled);
  MapGraphResult SetEdgeHealth(const std::string& edge_id, double health);

  const MapGraphNode* FindNode(const std::string& node_id) const;
  const MapGraphEdge* FindEdge(const std::string& edge_id) const;
  bool ReferencesMap(const std::string& map_id) const;
  const std::unordered_map<std::string, MapGraphNode>& Nodes() const noexcept { return nodes_; }
  const std::unordered_map<std::string, MapGraphEdge>& Edges() const noexcept { return edges_; }

  MapGraphRoute ShortestRoute(
      const std::string& start_node_id,
      const std::string& goal_node_id) const;

  MapGraphResult Save(const std::filesystem::path& path) const;
  MapGraphResult Load(const std::filesystem::path& path);

 private:
  MapGraphResult ValidateNode(const MapGraphNode& node) const;
  MapGraphResult ValidateEdge(const MapGraphEdge& edge) const;
  void Touch();

  MapIdValidator map_id_validator_;
  std::int64_t version_{0};
  std::unordered_map<std::string, MapGraphNode> nodes_;
  std::unordered_map<std::string, MapGraphEdge> edges_;
};

}  // namespace lingtu::maps

#include "lingtu/maps/map_graph.hpp"

#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <set>
#include <string>

using lingtu::maps::MapGraph;
using lingtu::maps::MapGraphEdge;
using lingtu::maps::MapGraphEdgeDirection;
using lingtu::maps::MapGraphNode;
using lingtu::maps::MapGraphNodeType;

namespace {

std::filesystem::path TempRoot() {
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  auto root = std::filesystem::temp_directory_path() /
      ("lingtu_maps_graph_test_" + std::to_string(stamp));
  std::filesystem::remove_all(root);
  std::filesystem::create_directories(root);
  return root;
}

MapGraphNode Node(
    std::string node_id,
    std::string map_id,
    MapGraphNodeType type,
    double x_m) {
  MapGraphNode node;
  node.node_id = std::move(node_id);
  node.map_id = std::move(map_id);
  node.type = type;
  node.label = node.node_id + " label";
  node.pose.frame_id = node.map_id + "/map";
  node.pose.x_m = x_m;
  node.pose.y_m = x_m + 1.0;
  node.pose.z_m = 0.5;
  node.pose.yaw_rad = 0.25;
  node.metadata["role"] = type == MapGraphNodeType::kPortal ? "door,stair" : "poi=value";
  return node;
}

MapGraphEdge Edge(
    std::string edge_id,
    std::string from,
    std::string to,
    double cost,
    MapGraphEdgeDirection direction = MapGraphEdgeDirection::kForward) {
  MapGraphEdge edge;
  edge.edge_id = std::move(edge_id);
  edge.from_node_id = std::move(from);
  edge.to_node_id = std::move(to);
  edge.cost = cost;
  edge.direction = direction;
  edge.enabled = true;
  edge.health = 1.0;
  return edge;
}

void WriteFile(const std::filesystem::path& path, const std::string& text) {
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  file << text;
}

std::string ReadFile(const std::filesystem::path& path) {
  std::ifstream file(path, std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
}

}  // namespace

int main() {
  const std::set<std::string> valid_maps{"building_1f", "building_2f", "garage"};
  MapGraph graph([&valid_maps](const std::string& map_id) {
    return valid_maps.count(map_id) != 0;
  });

  assert(graph.AddNode(Node("start", "building_1f", MapGraphNodeType::kPoi, 0.0)).ok);
  assert(graph.AddNode(Node("door_1f", "building_1f", MapGraphNodeType::kPortal, 1.0)).ok);
  assert(graph.AddNode(Node("door_2f", "building_2f", MapGraphNodeType::kPortal, 2.0)).ok);
  assert(graph.AddNode(Node("goal", "building_2f", MapGraphNodeType::kPoi, 3.0)).ok);
  assert(graph.AddNode(Node("garage", "garage", MapGraphNodeType::kPoi, 4.0)).ok);
  assert(!graph.AddNode(Node("bad", "../bad", MapGraphNodeType::kPoi, 0.0)).ok);

  auto hallway = Edge("hallway", "start", "door_1f", 1.0);
  assert(graph.AddEdge(hallway).ok);

  auto stairs = Edge(
      "stairs_up",
      "door_1f",
      "door_2f",
      2.0,
      MapGraphEdgeDirection::kBidirectional);
  stairs.transition_metadata["kind"] = "stairs";
  stairs.transition_metadata["hint"] = "slow,align";
  assert(graph.AddEdge(stairs).ok);

  auto upstairs = Edge("upstairs", "door_2f", "goal", 1.0);
  assert(graph.AddEdge(upstairs).ok);

  auto shortcut = Edge("disabled_shortcut", "start", "goal", 0.1);
  shortcut.enabled = false;
  assert(graph.AddEdge(shortcut).ok);

  auto risky = Edge("risky_garage", "start", "garage", 0.1);
  risky.health = 0.0;
  assert(graph.AddEdge(risky).ok);

  auto route = graph.ShortestRoute("start", "goal");
  assert(route.found);
  assert(route.total_cost == 4.0);
  assert((route.node_ids == std::vector<std::string>{"start", "door_1f", "door_2f", "goal"}));
  assert((route.edge_ids == std::vector<std::string>{"hallway", "stairs_up", "upstairs"}));
  assert(route.transitions.size() == 1U);
  assert(route.transitions[0].edge_id == "stairs_up");
  assert(route.transitions[0].from_map_id == "building_1f");
  assert(route.transitions[0].to_map_id == "building_2f");
  assert(route.transitions[0].metadata.at("kind") == "stairs");
  assert(route.transitions[0].metadata.at("hint") == "slow,align");

  auto reverse = graph.ShortestRoute("goal", "start");
  assert(!reverse.found);
  assert(graph.SetEdgeEnabled("upstairs", false).ok);
  assert(!graph.ShortestRoute("start", "goal").found);
  assert(graph.SetEdgeEnabled("upstairs", true).ok);
  assert(graph.SetEdgeHealth("stairs_up", 0.5).ok);
  auto degraded = graph.ShortestRoute("start", "goal");
  assert(degraded.found);
  assert(degraded.total_cost == 6.0);
  assert(graph.ReferencesMap("building_1f"));
  assert(graph.ReferencesMap("building_2f"));
  assert(!graph.ReferencesMap("missing"));
  assert(!graph.RemoveNodeIfUnreferenced("door_1f").ok);
  assert(graph.RemoveEdge("risky_garage").ok);
  assert(graph.FindEdge("risky_garage") == nullptr);
  assert(graph.RemoveNodeIfUnreferenced("garage").ok);
  assert(graph.FindNode("garage") == nullptr);
  assert(!graph.ReferencesMap("garage"));
  assert(!graph.RemoveEdge("risky_garage").ok);

  const auto root = TempRoot();
  const auto path = root / "graph.ltg";
  const auto version_before_save = graph.Version();
  assert(graph.Save(path).ok);
  const std::string saved = ReadFile(path);
  assert(saved.rfind("LINGTU_MAP_GRAPH 1\n", 0) == 0);
  assert(saved.find('\t') == std::string::npos);
  assert(saved.find("node ") != std::string::npos);
  assert(saved.find("edge ") != std::string::npos);

  MapGraph loaded([&valid_maps](const std::string& map_id) {
    return valid_maps.count(map_id) != 0;
  });
  assert(loaded.Load(path).ok);
  assert(loaded.Version() == version_before_save);
  assert(loaded.NodeCount() == graph.NodeCount());
  assert(loaded.EdgeCount() == graph.EdgeCount());
  const auto* loaded_portal = loaded.FindNode("door_1f");
  assert(loaded_portal != nullptr);
  assert(loaded_portal->type == MapGraphNodeType::kPortal);
  assert(loaded_portal->pose.frame_id == "building_1f/map");
  assert(loaded_portal->metadata.at("role") == "door,stair");
  auto loaded_route = loaded.ShortestRoute("start", "goal");
  assert(loaded_route.found);
  assert(loaded_route.transitions[0].metadata.at("hint") == "slow,align");

  WriteFile(path, "not a graph\nnode id=half\n");
  const auto old_version = loaded.Version();
  const auto corrupt = loaded.Load(path);
  assert(!corrupt.ok);
  assert(corrupt.corrupt);
  assert(loaded.Version() == old_version);
  assert(loaded.FindNode("door_1f") != nullptr);

  std::filesystem::remove_all(root);
  return 0;
}

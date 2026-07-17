#include "lingtu/maps/map_graph.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <utility>

#if defined(_WIN32)
#  define NOMINMAX
#  include <windows.h>
#else
#  include <fcntl.h>
#  include <unistd.h>
#endif

namespace lingtu::maps {
namespace {

constexpr const char* kMagic = "LINGTU_MAP_GRAPH";
constexpr int kFormatVersion = 1;

bool IsFiniteNonNegative(double value) {
  return std::isfinite(value) && value >= 0.0;
}

bool IsFinitePositive(double value) {
  return std::isfinite(value) && value > 0.0;
}

std::string NodeTypeName(MapGraphNodeType type) {
  return type == MapGraphNodeType::kPortal ? "portal" : "poi";
}

std::optional<MapGraphNodeType> ParseNodeType(const std::string& value) {
  if (value == "poi") return MapGraphNodeType::kPoi;
  if (value == "portal") return MapGraphNodeType::kPortal;
  return std::nullopt;
}

std::string DirectionName(MapGraphEdgeDirection direction) {
  return direction == MapGraphEdgeDirection::kBidirectional ? "bidirectional" : "forward";
}

std::optional<MapGraphEdgeDirection> ParseDirection(const std::string& value) {
  if (value == "forward") return MapGraphEdgeDirection::kForward;
  if (value == "bidirectional") return MapGraphEdgeDirection::kBidirectional;
  return std::nullopt;
}

std::string Encode(const std::string& value) {
  std::ostringstream out;
  out << std::uppercase << std::hex;
  for (const unsigned char ch : value) {
    const bool plain =
        (ch >= 'A' && ch <= 'Z') ||
        (ch >= 'a' && ch <= 'z') ||
        (ch >= '0' && ch <= '9') ||
        ch == '_' || ch == '-' || ch == '.' || ch == '/';
    if (plain) {
      out << static_cast<char>(ch);
    } else {
      out << '%' << std::setw(2) << std::setfill('0') << static_cast<int>(ch);
    }
  }
  return out.str();
}

int HexDigit(char ch) {
  if (ch >= '0' && ch <= '9') return ch - '0';
  if (ch >= 'a' && ch <= 'f') return 10 + ch - 'a';
  if (ch >= 'A' && ch <= 'F') return 10 + ch - 'A';
  return -1;
}

std::optional<std::string> Decode(const std::string& value) {
  std::string out;
  for (std::size_t i = 0; i < value.size(); ++i) {
    if (value[i] != '%') {
      out.push_back(value[i]);
      continue;
    }
    if (i + 2 >= value.size()) return std::nullopt;
    const int hi = HexDigit(value[i + 1]);
    const int lo = HexDigit(value[i + 2]);
    if (hi < 0 || lo < 0) return std::nullopt;
    out.push_back(static_cast<char>((hi << 4) | lo));
    i += 2;
  }
  return out;
}

std::string EncodeMetadata(const std::unordered_map<std::string, std::string>& metadata) {
  std::vector<std::pair<std::string, std::string>> items(metadata.begin(), metadata.end());
  std::sort(items.begin(), items.end());
  std::ostringstream out;
  bool first = true;
  for (const auto& [key, value] : items) {
    if (!first) out << ',';
    first = false;
    out << Encode(key) << '=' << Encode(value);
  }
  return out.str();
}

std::optional<std::unordered_map<std::string, std::string>> DecodeMetadata(
    const std::string& value) {
  std::unordered_map<std::string, std::string> out;
  if (value.empty()) return out;
  std::size_t start = 0;
  while (start <= value.size()) {
    const std::size_t end = value.find(',', start);
    const std::string item = value.substr(start, end == std::string::npos
        ? std::string::npos
        : end - start);
    const std::size_t sep = item.find('=');
    if (sep == std::string::npos) return std::nullopt;
    auto key = Decode(item.substr(0, sep));
    auto decoded_value = Decode(item.substr(sep + 1));
    if (!key.has_value() || !decoded_value.has_value() || key->empty()) return std::nullopt;
    out[*key] = *decoded_value;
    if (end == std::string::npos) break;
    start = end + 1;
  }
  return out;
}

std::string FormatDouble(double value) {
  std::ostringstream out;
  out << std::setprecision(17) << value;
  return out.str();
}

std::optional<double> ParseDouble(const std::string& value) {
  try {
    std::size_t used = 0;
    const double parsed = std::stod(value, &used);
    if (used != value.size() || !std::isfinite(parsed)) return std::nullopt;
    return parsed;
  } catch (...) {
    return std::nullopt;
  }
}

std::optional<std::int64_t> ParseInt64(const std::string& value) {
  try {
    std::size_t used = 0;
    const auto parsed = std::stoll(value, &used);
    if (used != value.size()) return std::nullopt;
    return parsed;
  } catch (...) {
    return std::nullopt;
  }
}

using Fields = std::unordered_map<std::string, std::string>;

std::optional<Fields> ParseFields(const std::string& line) {
  Fields fields;
  std::size_t start = 0;
  while (start < line.size()) {
    const std::size_t end = line.find(' ', start);
    const std::string token = line.substr(start, end == std::string::npos
        ? std::string::npos
        : end - start);
    if (!token.empty()) {
      const std::size_t sep = token.find('=');
      if (sep == std::string::npos) return std::nullopt;
      auto key = Decode(token.substr(0, sep));
      if (!key.has_value() || key->empty()) return std::nullopt;
      fields[*key] = token.substr(sep + 1);
    }
    if (end == std::string::npos) break;
    start = end + 1;
  }
  return fields;
}

bool Required(const Fields& fields, const std::string& key, std::string* out) {
  const auto it = fields.find(key);
  if (it == fields.end() || it->second.empty()) return false;
  auto decoded = Decode(it->second);
  if (!decoded.has_value()) return false;
  *out = *decoded;
  return true;
}

void SyncPath(const std::filesystem::path& path, bool directory) {
#if defined(_WIN32)
  const DWORD flags = directory ? FILE_FLAG_BACKUP_SEMANTICS : FILE_ATTRIBUTE_NORMAL;
  HANDLE handle = CreateFileW(
      path.wstring().c_str(),
      GENERIC_READ,
      FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
      nullptr,
      OPEN_EXISTING,
      flags,
      nullptr);
  if (handle != INVALID_HANDLE_VALUE) {
    FlushFileBuffers(handle);
    CloseHandle(handle);
  }
#else
  const int flags = directory ? (O_RDONLY | O_DIRECTORY) : O_RDONLY;
  const int fd = open(path.c_str(), flags);
  if (fd >= 0) {
    fsync(fd);
    close(fd);
  }
#endif
}

void AtomicReplace(const std::filesystem::path& source, const std::filesystem::path& target) {
#if defined(_WIN32)
  if (MoveFileExW(
          source.wstring().c_str(),
          target.wstring().c_str(),
          MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) == 0) {
    throw std::runtime_error("failed to atomically replace map graph");
  }
#else
  if (::rename(source.c_str(), target.c_str()) != 0) {
    throw std::runtime_error("failed to atomically replace map graph");
  }
#endif
  SyncPath(target.parent_path(), true);
}

}  // namespace

MapGraph::MapGraph(MapIdValidator validator)
    : map_id_validator_(std::move(validator)) {}

MapGraphResult MapGraph::ValidateNode(const MapGraphNode& node) const {
  if (node.node_id.empty()) return {false, false, "node_id is required"};
  if (node.map_id.empty()) return {false, false, "map_id is required"};
  if (node.pose.frame_id.empty()) return {false, false, "frame_id is required"};
  if (!std::isfinite(node.pose.x_m) || !std::isfinite(node.pose.y_m) ||
      !std::isfinite(node.pose.z_m)) {
    return {false, false, "node pose must be finite"};
  }
  if (node.pose.yaw_rad.has_value() && !std::isfinite(*node.pose.yaw_rad)) {
    return {false, false, "node yaw must be finite"};
  }
  if (map_id_validator_ && !map_id_validator_(node.map_id)) {
    return {false, false, "map_id rejected: " + node.map_id};
  }
  return {true, false, "ok"};
}

MapGraphResult MapGraph::ValidateEdge(const MapGraphEdge& edge) const {
  if (edge.edge_id.empty()) return {false, false, "edge_id is required"};
  if (edge.from_node_id.empty() || edge.to_node_id.empty()) {
    return {false, false, "edge endpoints are required"};
  }
  if (edge.from_node_id == edge.to_node_id) {
    return {false, false, "edge endpoints must differ"};
  }
  if (!IsFinitePositive(edge.cost)) return {false, false, "edge cost must be positive"};
  if (!IsFiniteNonNegative(edge.health) || edge.health > 1.0) {
    return {false, false, "edge health must be in [0,1]"};
  }
  if (nodes_.find(edge.from_node_id) == nodes_.end()) {
    return {false, false, "from_node_id not found: " + edge.from_node_id};
  }
  if (nodes_.find(edge.to_node_id) == nodes_.end()) {
    return {false, false, "to_node_id not found: " + edge.to_node_id};
  }
  return {true, false, "ok"};
}

void MapGraph::Touch() {
  ++version_;
}

MapGraphResult MapGraph::AddNode(MapGraphNode node) {
  auto valid = ValidateNode(node);
  if (!valid.ok) return valid;
  if (nodes_.find(node.node_id) != nodes_.end()) {
    return {false, false, "node exists: " + node.node_id};
  }
  nodes_.emplace(node.node_id, std::move(node));
  Touch();
  return {true, false, "node added"};
}

MapGraphResult MapGraph::AddEdge(MapGraphEdge edge) {
  auto valid = ValidateEdge(edge);
  if (!valid.ok) return valid;
  if (edges_.find(edge.edge_id) != edges_.end()) {
    return {false, false, "edge exists: " + edge.edge_id};
  }
  edges_.emplace(edge.edge_id, std::move(edge));
  Touch();
  return {true, false, "edge added"};
}

MapGraphResult MapGraph::SetEdgeEnabled(const std::string& edge_id, bool enabled) {
  auto it = edges_.find(edge_id);
  if (it == edges_.end()) return {false, false, "edge not found: " + edge_id};
  if (it->second.enabled != enabled) {
    it->second.enabled = enabled;
    Touch();
  }
  return {true, false, "edge updated"};
}

MapGraphResult MapGraph::SetEdgeHealth(const std::string& edge_id, double health) {
  auto it = edges_.find(edge_id);
  if (it == edges_.end()) return {false, false, "edge not found: " + edge_id};
  if (!IsFiniteNonNegative(health) || health > 1.0) {
    return {false, false, "edge health must be in [0,1]"};
  }
  if (it->second.health != health) {
    it->second.health = health;
    Touch();
  }
  return {true, false, "edge updated"};
}

const MapGraphNode* MapGraph::FindNode(const std::string& node_id) const {
  const auto it = nodes_.find(node_id);
  return it == nodes_.end() ? nullptr : &it->second;
}

const MapGraphEdge* MapGraph::FindEdge(const std::string& edge_id) const {
  const auto it = edges_.find(edge_id);
  return it == edges_.end() ? nullptr : &it->second;
}

MapGraphRoute MapGraph::ShortestRoute(
    const std::string& start_node_id,
    const std::string& goal_node_id) const {
  MapGraphRoute route;
  if (nodes_.find(start_node_id) == nodes_.end() ||
      nodes_.find(goal_node_id) == nodes_.end()) {
    return route;
  }
  if (start_node_id == goal_node_id) {
    route.found = true;
    route.node_ids.push_back(start_node_id);
    return route;
  }

  struct Arc {
    std::string next_node_id;
    std::string edge_id;
    double cost;
  };
  std::unordered_map<std::string, std::vector<Arc>> adjacency;
  for (const auto& [edge_id, edge] : edges_) {
    if (!edge.enabled || edge.health <= 0.0) continue;
    const double weighted_cost = edge.cost / edge.health;
    adjacency[edge.from_node_id].push_back({edge.to_node_id, edge_id, weighted_cost});
    if (edge.direction == MapGraphEdgeDirection::kBidirectional) {
      adjacency[edge.to_node_id].push_back({edge.from_node_id, edge_id, weighted_cost});
    }
  }

  struct QueueItem {
    double cost;
    std::string node_id;
    bool operator>(const QueueItem& other) const { return cost > other.cost; }
  };
  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> queue;
  std::unordered_map<std::string, double> distance;
  std::unordered_map<std::string, std::string> previous_node;
  std::unordered_map<std::string, std::string> previous_edge;

  distance[start_node_id] = 0.0;
  queue.push({0.0, start_node_id});
  while (!queue.empty()) {
    const QueueItem current = queue.top();
    queue.pop();
    if (current.cost > distance[current.node_id]) continue;
    if (current.node_id == goal_node_id) break;
    const auto arcs = adjacency.find(current.node_id);
    if (arcs == adjacency.end()) continue;
    for (const auto& arc : arcs->second) {
      const double next_cost = current.cost + arc.cost;
      const auto known = distance.find(arc.next_node_id);
      if (known != distance.end() && known->second <= next_cost) continue;
      distance[arc.next_node_id] = next_cost;
      previous_node[arc.next_node_id] = current.node_id;
      previous_edge[arc.next_node_id] = arc.edge_id;
      queue.push({next_cost, arc.next_node_id});
    }
  }

  const auto final_distance = distance.find(goal_node_id);
  if (final_distance == distance.end()) return route;

  route.found = true;
  route.total_cost = final_distance->second;
  for (std::string node = goal_node_id; !node.empty();) {
    route.node_ids.push_back(node);
    const auto prev = previous_node.find(node);
    if (prev == previous_node.end()) break;
    route.edge_ids.push_back(previous_edge[node]);
    node = prev->second;
  }
  std::reverse(route.node_ids.begin(), route.node_ids.end());
  std::reverse(route.edge_ids.begin(), route.edge_ids.end());

  for (std::size_t i = 0; i < route.edge_ids.size(); ++i) {
    const auto* from = FindNode(route.node_ids[i]);
    const auto* to = FindNode(route.node_ids[i + 1]);
    const auto* edge = FindEdge(route.edge_ids[i]);
    if (from == nullptr || to == nullptr || edge == nullptr || from->map_id == to->map_id) {
      continue;
    }
    MapGraphTransition transition;
    transition.edge_id = edge->edge_id;
    transition.from_map_id = from->map_id;
    transition.to_map_id = to->map_id;
    transition.metadata = edge->transition_metadata;
    route.transitions.push_back(std::move(transition));
  }
  return route;
}

MapGraphResult MapGraph::Save(const std::filesystem::path& path) const {
  try {
    std::filesystem::create_directories(path.parent_path());
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    const auto temp = path.parent_path() /
        (path.filename().string() + ".tmp-" + std::to_string(stamp));
    {
      std::ofstream file(temp, std::ios::binary | std::ios::trunc);
      if (!file) return {false, false, "failed to open temp map graph"};
      file << kMagic << ' ' << kFormatVersion << '\n';
      file << "graph version=" << version_ << '\n';

      std::vector<std::string> node_ids;
      node_ids.reserve(nodes_.size());
      for (const auto& [node_id, _] : nodes_) node_ids.push_back(node_id);
      std::sort(node_ids.begin(), node_ids.end());
      for (const auto& node_id : node_ids) {
        const auto& node = nodes_.at(node_id);
        file << "node"
             << " id=" << Encode(node.node_id)
             << " map=" << Encode(node.map_id)
             << " type=" << NodeTypeName(node.type)
             << " label=" << Encode(node.label)
             << " frame=" << Encode(node.pose.frame_id)
             << " x=" << FormatDouble(node.pose.x_m)
             << " y=" << FormatDouble(node.pose.y_m)
             << " z=" << FormatDouble(node.pose.z_m)
             << " yaw=" << (node.pose.yaw_rad.has_value()
                    ? FormatDouble(*node.pose.yaw_rad)
                    : std::string("none"))
             << " meta=" << EncodeMetadata(node.metadata)
             << '\n';
      }

      std::vector<std::string> edge_ids;
      edge_ids.reserve(edges_.size());
      for (const auto& [edge_id, _] : edges_) edge_ids.push_back(edge_id);
      std::sort(edge_ids.begin(), edge_ids.end());
      for (const auto& edge_id : edge_ids) {
        const auto& edge = edges_.at(edge_id);
        file << "edge"
             << " id=" << Encode(edge.edge_id)
             << " from=" << Encode(edge.from_node_id)
             << " to=" << Encode(edge.to_node_id)
             << " cost=" << FormatDouble(edge.cost)
             << " direction=" << DirectionName(edge.direction)
             << " enabled=" << (edge.enabled ? "1" : "0")
             << " health=" << FormatDouble(edge.health)
             << " transition=" << EncodeMetadata(edge.transition_metadata)
             << '\n';
      }
      file.flush();
      if (!file) return {false, false, "failed to flush map graph"};
    }
    SyncPath(temp, false);
    AtomicReplace(temp, path);
    return {true, false, "saved"};
  } catch (const std::exception& error) {
    return {false, false, error.what()};
  }
}

MapGraphResult MapGraph::Load(const std::filesystem::path& path) {
  try {
    std::ifstream file(path, std::ios::binary);
    if (!file) return {false, false, "map graph not found"};

    std::string line;
    if (!std::getline(file, line) || line != std::string(kMagic) + " 1") {
      return {false, true, "invalid map graph header"};
    }
    if (!std::getline(file, line) || line.rfind("graph ", 0) != 0) {
      return {false, true, "missing graph record"};
    }
    auto graph_fields = ParseFields(line.substr(6));
    if (!graph_fields.has_value()) return {false, true, "corrupt graph record"};
    const auto version_it = graph_fields->find("version");
    if (version_it == graph_fields->end()) return {false, true, "missing graph version"};
    auto parsed_version = ParseInt64(version_it->second);
    if (!parsed_version.has_value() || *parsed_version < 0) {
      return {false, true, "invalid graph version"};
    }

    MapGraph parsed(map_id_validator_);
    parsed.version_ = *parsed_version;
    std::vector<MapGraphEdge> parsed_edges;
    while (std::getline(file, line)) {
      if (line.empty()) continue;
      const std::size_t sep = line.find(' ');
      if (sep == std::string::npos) return {false, true, "corrupt record"};
      const std::string kind = line.substr(0, sep);
      auto fields = ParseFields(line.substr(sep + 1));
      if (!fields.has_value()) return {false, true, "corrupt fields"};
      if (kind == "node") {
        MapGraphNode node;
        std::string type_value;
        if (!Required(*fields, "id", &node.node_id) ||
            !Required(*fields, "map", &node.map_id) ||
            !Required(*fields, "type", &type_value) ||
            !Required(*fields, "frame", &node.pose.frame_id)) {
          return {false, true, "node missing required field"};
        }
        const auto type = ParseNodeType(type_value);
        if (!type.has_value()) return {false, true, "invalid node type"};
        node.type = *type;
        if (fields->count("label") != 0) {
          auto decoded_label = Decode(fields->at("label"));
          if (!decoded_label.has_value()) return {false, true, "invalid node label"};
          node.label = *decoded_label;
        }
        const auto x = ParseDouble(fields->count("x") != 0 ? fields->at("x") : "");
        const auto y = ParseDouble(fields->count("y") != 0 ? fields->at("y") : "");
        const auto z = ParseDouble(fields->count("z") != 0 ? fields->at("z") : "");
        if (!x.has_value() || !y.has_value() || !z.has_value()) {
          return {false, true, "invalid node pose"};
        }
        node.pose.x_m = *x;
        node.pose.y_m = *y;
        node.pose.z_m = *z;
        const std::string yaw = fields->count("yaw") != 0 ? fields->at("yaw") : "none";
        if (yaw != "none") {
          auto parsed_yaw = ParseDouble(yaw);
          if (!parsed_yaw.has_value()) return {false, true, "invalid node yaw"};
          node.pose.yaw_rad = *parsed_yaw;
        }
        auto metadata = DecodeMetadata(fields->count("meta") != 0 ? fields->at("meta") : "");
        if (!metadata.has_value()) return {false, true, "invalid node metadata"};
        node.metadata = std::move(*metadata);
        auto added = parsed.AddNode(std::move(node));
        if (!added.ok) return {false, true, added.message};
        parsed.version_ = *parsed_version;
      } else if (kind == "edge") {
        MapGraphEdge edge;
        std::string direction_value;
        if (!Required(*fields, "id", &edge.edge_id) ||
            !Required(*fields, "from", &edge.from_node_id) ||
            !Required(*fields, "to", &edge.to_node_id) ||
            !Required(*fields, "direction", &direction_value)) {
          return {false, true, "edge missing required field"};
        }
        const auto direction = ParseDirection(direction_value);
        if (!direction.has_value()) return {false, true, "invalid edge direction"};
        edge.direction = *direction;
        const auto cost = ParseDouble(fields->count("cost") != 0 ? fields->at("cost") : "");
        const auto health = ParseDouble(fields->count("health") != 0 ? fields->at("health") : "");
        if (!cost.has_value() || !health.has_value()) {
          return {false, true, "invalid edge cost or health"};
        }
        edge.cost = *cost;
        edge.health = *health;
        const std::string enabled = fields->count("enabled") != 0 ? fields->at("enabled") : "";
        if (enabled != "0" && enabled != "1") return {false, true, "invalid edge enabled"};
        edge.enabled = enabled == "1";
        auto metadata = DecodeMetadata(
            fields->count("transition") != 0 ? fields->at("transition") : "");
        if (!metadata.has_value()) return {false, true, "invalid edge metadata"};
        edge.transition_metadata = std::move(*metadata);
        parsed_edges.push_back(std::move(edge));
      } else {
        return {false, true, "unknown map graph record"};
      }
    }
    for (auto& edge : parsed_edges) {
      auto added = parsed.AddEdge(std::move(edge));
      if (!added.ok) return {false, true, added.message};
      parsed.version_ = *parsed_version;
    }
    nodes_ = std::move(parsed.nodes_);
    edges_ = std::move(parsed.edges_);
    version_ = *parsed_version;
    return {true, false, "loaded"};
  } catch (const std::exception& error) {
    return {false, true, error.what()};
  }
}

}  // namespace lingtu::maps

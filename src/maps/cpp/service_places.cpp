#include "lingtu/maps/service.hpp"

#include "lingtu/maps/json.hpp"
#include "lingtu/maps/lock.hpp"
#include "lingtu/maps/map_graph.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#if defined(_WIN32)
#  define NOMINMAX
#  include <windows.h>
#else
#  include <fcntl.h>
#  include <unistd.h>
#endif

namespace lingtu::maps {
namespace {

void SyncPath(const std::filesystem::path& path, bool directory) {
#if defined(_WIN32)
  const DWORD flags = directory ? FILE_FLAG_BACKUP_SEMANTICS : FILE_ATTRIBUTE_NORMAL;
  const DWORD access = directory ? GENERIC_READ : (GENERIC_READ | GENERIC_WRITE);
  HANDLE handle = CreateFileW(
      path.wstring().c_str(),
      access,
      FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
      nullptr,
      OPEN_EXISTING,
      flags,
      nullptr);
  if (handle == INVALID_HANDLE_VALUE) {
    if (directory) return;
    throw std::runtime_error("failed to open path for durable map-service write");
  }
  const bool flushed = FlushFileBuffers(handle) != 0;
  CloseHandle(handle);
  if (!flushed && !directory) {
    throw std::runtime_error("failed to flush map-service file");
  }
#else
  const int flags = directory ? (O_RDONLY | O_DIRECTORY) : O_RDONLY;
  const int fd = open(path.c_str(), flags);
  if (fd < 0) {
    if (directory) return;
    throw std::runtime_error("failed to open path for durable map-service write");
  }
  const int result = fsync(fd);
  close(fd);
  if (result != 0 && !directory) {
    throw std::runtime_error("failed to flush map-service file");
  }
#endif
}

void AtomicReplace(const std::filesystem::path& source, const std::filesystem::path& target) {
#if defined(_WIN32)
  if (MoveFileExW(
          source.wstring().c_str(),
          target.wstring().c_str(),
          MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) == 0) {
    throw std::runtime_error("failed to atomically replace map-service file");
  }
#else
  if (::rename(source.c_str(), target.c_str()) != 0) {
    throw std::runtime_error("failed to atomically replace map-service file");
  }
#endif
  SyncPath(target.parent_path(), true);
}

void WriteTextAtomic(const std::filesystem::path& path, const std::string& value) {
  std::filesystem::create_directories(path.parent_path());
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto temp = path.parent_path() /
      (path.filename().string() + ".tmp." + std::to_string(stamp));
  try {
    {
      std::ofstream file(temp, std::ios::binary | std::ios::trunc);
      if (!file) {
        throw std::runtime_error("failed to create map-service temp file");
      }
      file.write(value.data(), static_cast<std::streamsize>(value.size()));
      file.flush();
      if (!file) {
        throw std::runtime_error("failed to flush map-service temp file");
      }
    }
    SyncPath(temp, false);
    AtomicReplace(temp, path);
  } catch (...) {
    std::error_code ignored;
    std::filesystem::remove(temp, ignored);
    throw;
  }
}

std::string JsonEscape(const std::string& value) {
  std::ostringstream stream;
  for (const unsigned char ch : value) {
    switch (ch) {
      case '"': stream << "\\\""; break;
      case '\\': stream << "\\\\"; break;
      case '\b': stream << "\\b"; break;
      case '\f': stream << "\\f"; break;
      case '\n': stream << "\\n"; break;
      case '\r': stream << "\\r"; break;
      case '\t': stream << "\\t"; break;
      default:
        if (ch < 0x20U) {
          constexpr char kHex[] = "0123456789abcdef";
          stream << "\\u00" << kHex[(ch >> 4U) & 0x0FU] << kHex[ch & 0x0FU];
        } else {
          stream << static_cast<char>(ch);
        }
    }
  }
  return stream.str();
}

std::string JsonString(const std::string& value) {
  return "\"" + JsonEscape(value) + "\"";
}

std::string BoolJson(bool value) {
  return value ? "true" : "false";
}

bool SafeTsvField(const std::string& value) {
  return value.find('\t') == std::string::npos &&
      value.find('\n') == std::string::npos &&
      value.find('\r') == std::string::npos;
}

std::vector<std::string> SplitTab(const std::string& line) {
  std::vector<std::string> out;
  std::size_t begin = 0U;
  while (true) {
    const auto tab = line.find('\t', begin);
    if (tab == std::string::npos) {
      out.push_back(line.substr(begin));
      break;
    }
    out.push_back(line.substr(begin, tab - begin));
    begin = tab + 1U;
  }
  return out;
}

bool ParseFiniteNumber(const std::string& value, double* result) {
  if (result == nullptr || value.empty()) return false;
  std::istringstream stream(value);
  stream >> std::noskipws >> *result;
  return stream && stream.eof() && std::isfinite(*result);
}

std::optional<std::string> NormalizeJsonObject(const std::string& value) {
  const auto begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) return std::string("{}");
  const auto end = value.find_last_not_of(" \t\r\n");
  const std::string trimmed = value.substr(begin, end - begin + 1U);
  return IsValidJsonObject(trimmed) ? std::optional<std::string>(trimmed) : std::nullopt;
}

bool ValidPoiRecord(const std::vector<std::string>& fields) {
  if (fields.size() != 8U || fields[0].empty() || fields[1].empty() ||
      !SafeTsvField(fields[0]) || !SafeTsvField(fields[1]) ||
      !SafeTsvField(fields[7]) || (fields[5] != "0" && fields[5] != "1")) {
    return false;
  }
  double value = 0.0;
  for (const std::size_t index : {2U, 3U, 4U, 6U}) {
    if (!ParseFiniteNumber(fields[index], &value)) return false;
  }
  return NormalizeJsonObject(fields[7]).has_value();
}

std::string GraphNodeId(const std::string& map_id) {
  return "map:" + map_id;
}

std::string GraphEdgeId(const std::string& from_map_id, const std::string& to_map_id) {
  return "map:" + from_map_id + "->" + to_map_id;
}

std::string MapIdFromGraphNodeId(const std::string& node_id) {
  constexpr const char* kPrefix = "map:";
  return node_id.rfind(kPrefix, 0) == 0 ? node_id.substr(4U) : node_id;
}

}  // namespace

std::filesystem::path MapsServiceCore::PoiPath(const std::string& map_id) const {
  return store_.MapPath(map_id) / "pois.tsv";
}

std::filesystem::path MapsServiceCore::GraphPath() const {
  return store_.RootDir() / "map_graph.ltg";
}

std::string MapsServiceCore::ListPoiJson(const std::string& map_id) const {
  const std::string resolved = map_id.empty() ? store_.ActiveMapId() : map_id;
  if (resolved.empty()) {
    return FailureJson("poi_list", "missing map name", "missing_map_name");
  }
  try {
    const std::string id = MapStore::NormalizeMapId(resolved);
    auto map_lock = MapLock::TryAcquire(store_.RootDir(), id, "poi-list");
    if (!map_lock.has_value()) {
      return FailureJson(
          "poi_list", "map write in progress: " + id, "map_write_in_progress");
    }
    if (!store_.GetMapRecord(id).has_value()) {
      return FailureJson("poi_list", "map not found: " + id, "map_not_found");
    }
    const auto poi_path = PoiPath(id);
    std::ifstream file(poi_path);
    if (!file && std::filesystem::exists(poi_path)) {
      return FailureJson("poi_list", "failed to read POI store", "poi_store_read_failed");
    }
    std::ostringstream pois;
    pois << "{";
    bool first = true;
    std::string line;
    while (std::getline(file, line)) {
      const auto fields = SplitTab(line);
      if (!ValidPoiRecord(fields)) {
        return FailureJson("poi_list", "POI store is corrupt", "poi_store_corrupt");
      }
      if (!first) {
        pois << ",";
      }
      first = false;
      const bool has_yaw = fields[5] == "1";
      const auto tags = NormalizeJsonObject(fields[7]);
      pois << JsonString(fields[0]) << ":{"
           << "\"frame_id\":" << JsonString(fields[1]) << ","
           << "\"x\":" << fields[2] << ","
           << "\"y\":" << fields[3] << ","
           << "\"z\":" << fields[4] << ","
           << "\"yaw\":" << (has_yaw ? fields[6] : "null") << ","
           << "\"tags\":" << *tags
           << "}";
    }
    if (file.bad()) {
      return FailureJson("poi_list", "failed to read POI store", "poi_store_read_failed");
    }
    pois << "}";
    return "{"
        "\"action\":\"poi_list\","
        "\"success\":true,"
        "\"schema_version\":\"map.poi.v1\","
        "\"map_id\":" + JsonString(id) + ","
        "\"pois\":" + pois.str() +
        "}";
  } catch (const std::invalid_argument& exc) {
    return FailureJson("poi_list", exc.what(), "invalid_map_name");
  } catch (const std::exception& exc) {
    return FailureJson("poi_list", exc.what(), "poi_store_read_failed");
  }
}

std::string MapsServiceCore::SetPoiJson(
    const std::string& map_id,
    const std::string& name,
    double x_m,
    double y_m,
    double z_m,
    double yaw_rad,
    bool has_yaw,
    const std::string& frame_id,
    const std::string& tags_json) {
  const std::string resolved = map_id.empty() ? store_.ActiveMapId() : map_id;
  if (resolved.empty()) {
    return FailureJson("poi_set", "missing map name", "missing_map_name");
  }
  if (name.empty() || !SafeTsvField(name) || !SafeTsvField(frame_id) ||
      !SafeTsvField(tags_json) || !std::isfinite(x_m) || !std::isfinite(y_m) ||
      !std::isfinite(z_m) || (has_yaw && !std::isfinite(yaw_rad))) {
    return FailureJson("poi_set", "invalid POI field", "invalid_poi");
  }
  const auto normalized_tags = NormalizeJsonObject(tags_json);
  if (!normalized_tags.has_value()) {
    return FailureJson("poi_set", "POI tags must be a valid JSON object", "invalid_poi");
  }
  try {
    const std::string id = MapStore::NormalizeMapId(resolved);
    auto map_lock = MapLock::TryAcquire(store_.RootDir(), id, "poi-set");
    if (!map_lock.has_value()) {
      return FailureJson(
          "poi_set", "map write in progress: " + id, "map_write_in_progress");
    }
    if (!store_.GetMapRecord(id).has_value()) {
      return FailureJson("poi_set", "map not found: " + id, "map_not_found");
    }
    std::vector<std::string> kept;
    {
      const auto poi_path = PoiPath(id);
      std::ifstream in(poi_path);
      if (!in && std::filesystem::exists(poi_path)) {
        return FailureJson("poi_set", "failed to read POI store", "poi_store_read_failed");
      }
      std::string line;
      while (std::getline(in, line)) {
        const auto fields = SplitTab(line);
        if (!ValidPoiRecord(fields)) {
          return FailureJson("poi_set", "POI store is corrupt", "poi_store_corrupt");
        }
        if (fields.empty() || fields[0] != name) {
          kept.push_back(line);
        }
      }
      if (in.bad()) {
        return FailureJson("poi_set", "failed to read POI store", "poi_store_read_failed");
      }
    }
    std::ostringstream out;
    for (const auto& line : kept) {
      out << line << "\n";
    }
    out << name << "\t" << (frame_id.empty() ? "map" : frame_id) << "\t"
        << std::setprecision(12) << x_m << "\t" << y_m << "\t" << z_m << "\t"
        << (has_yaw ? "1" : "0") << "\t" << yaw_rad << "\t"
        << *normalized_tags << "\n";
    WriteTextAtomic(PoiPath(id), out.str());
    return "{"
        "\"action\":\"poi_set\","
        "\"success\":true,"
        "\"schema_version\":\"map.poi.v1\","
        "\"map_id\":" + JsonString(id) + ","
        "\"name\":" + JsonString(name) + ","
        "\"poi\":{"
        "\"frame_id\":" + JsonString(frame_id.empty() ? "map" : frame_id) + ","
        "\"x\":" + std::to_string(x_m) + ","
        "\"y\":" + std::to_string(y_m) + ","
        "\"z\":" + std::to_string(z_m) + ","
        "\"yaw\":" + (has_yaw ? std::to_string(yaw_rad) : "null") + ","
        "\"tags\":" + *normalized_tags +
        "}}";
  } catch (const std::invalid_argument& exc) {
    return FailureJson("poi_set", exc.what(), "invalid_map_name");
  } catch (const std::exception& exc) {
    return FailureJson("poi_set", exc.what(), "poi_store_write_failed");
  }
}

std::string MapsServiceCore::DeletePoiJson(
    const std::string& map_id,
    const std::string& name) {
  const std::string resolved = map_id.empty() ? store_.ActiveMapId() : map_id;
  if (resolved.empty()) {
    return FailureJson("poi_delete", "missing map name", "missing_map_name");
  }
  if (name.empty() || !SafeTsvField(name)) {
    return FailureJson("poi_delete", "invalid POI field", "invalid_poi");
  }
  try {
    const std::string id = MapStore::NormalizeMapId(resolved);
    auto map_lock = MapLock::TryAcquire(store_.RootDir(), id, "poi-delete");
    if (!map_lock.has_value()) {
      return FailureJson(
          "poi_delete", "map write in progress: " + id, "map_write_in_progress");
    }
    if (!store_.GetMapRecord(id).has_value()) {
      return FailureJson("poi_delete", "map not found: " + id, "map_not_found");
    }
    std::vector<std::string> kept;
    bool removed = false;
    const auto poi_path = PoiPath(id);
    {
      std::ifstream in(poi_path);
      if (!in && std::filesystem::exists(poi_path)) {
        return FailureJson("poi_delete", "failed to read POI store", "poi_store_read_failed");
      }
      std::string line;
      while (std::getline(in, line)) {
        const auto fields = SplitTab(line);
        if (!ValidPoiRecord(fields)) {
          return FailureJson("poi_delete", "POI store is corrupt", "poi_store_corrupt");
        }
        if (!fields.empty() && fields[0] == name) {
          removed = true;
        } else {
          kept.push_back(line);
        }
      }
      if (in.bad()) {
        return FailureJson("poi_delete", "failed to read POI store", "poi_store_read_failed");
      }
    }
    if (!removed) {
      return FailureJson("poi_delete", "POI not found: " + name, "poi_not_found");
    }
    std::ostringstream out;
    for (const auto& item : kept) {
      out << item << "\n";
    }
    WriteTextAtomic(PoiPath(id), out.str());
    return "{"
        "\"action\":\"poi_delete\","
        "\"success\":true,"
        "\"map_id\":" + JsonString(id) + ","
        "\"name\":" + JsonString(name) +
        "}";
  } catch (const std::invalid_argument& exc) {
    return FailureJson("poi_delete", exc.what(), "invalid_map_name");
  } catch (const std::exception& exc) {
    return FailureJson("poi_delete", exc.what(), "poi_store_write_failed");
  }
}

std::string MapsServiceCore::ListMapGraphJson() const {
  auto graph_lock = MapLock::TryAcquire(store_.RootDir(), "__map_graph__", "map-graph-list");
  if (!graph_lock.has_value()) {
    return FailureJson("map_graph", "map graph write in progress", "map_graph_busy");
  }
  MapGraph graph([this](const std::string& id) {
    return MapStore::IsValidMapId(id) && store_.GetMapRecord(id).has_value();
  });
  if (std::filesystem::is_regular_file(GraphPath())) {
    const auto loaded = graph.Load(GraphPath());
    if (!loaded.ok) {
      return FailureJson(
          "map_graph",
          loaded.message,
          loaded.corrupt ? "graph_corrupt" : "graph_store_read_failed");
    }
  }
  std::vector<std::string> edge_ids;
  edge_ids.reserve(graph.Edges().size());
  for (const auto& item : graph.Edges()) {
    if (item.second.enabled) edge_ids.push_back(item.first);
  }
  std::sort(edge_ids.begin(), edge_ids.end());
  std::ostringstream edges;
  edges << "[";
  for (std::size_t i = 0; i < edge_ids.size(); ++i) {
    if (i != 0U) {
      edges << ",";
    }
    const auto& edge = graph.Edges().at(edge_ids[i]);
    const auto* from_node = graph.FindNode(edge.from_node_id);
    const auto* to_node = graph.FindNode(edge.to_node_id);
    const std::string from = from_node == nullptr
        ? MapIdFromGraphNodeId(edge.from_node_id)
        : from_node->map_id;
    const std::string to = to_node == nullptr
        ? MapIdFromGraphNodeId(edge.to_node_id)
        : to_node->map_id;
    const auto type = edge.transition_metadata.find("type");
    edges << "{"
          << "\"from\":" << JsonString(from) << ","
          << "\"to\":" << JsonString(to) << ","
          << "\"type\":" << JsonString(
              type == edge.transition_metadata.end() ? "link" : type->second) << ","
          << "\"bidirectional\":" << BoolJson(
              edge.direction == MapGraphEdgeDirection::kBidirectional)
          << "}";
  }
  edges << "]";
  return "{"
      "\"action\":\"map_graph\","
      "\"success\":true,"
      "\"schema_version\":\"map.graph.v1\","
      "\"storage\":\"map_graph.ltg\","
      "\"edges\":" + edges.str() +
      "}";
}

std::string MapsServiceCore::SetMapEdgeJson(
    const std::string& from_map_id,
    const std::string& to_map_id,
    const std::string& edge_type,
    bool bidirectional) {
  try {
    const std::string from = MapStore::NormalizeMapId(from_map_id);
    const std::string to = MapStore::NormalizeMapId(to_map_id);
    auto graph_lock = MapLock::TryAcquire(store_.RootDir(), "__map_graph__", "map-edge-set");
    if (!graph_lock.has_value()) {
      return FailureJson("map_edge_set", "map graph write in progress", "map_graph_busy");
    }
    if (!store_.GetMapRecord(from).has_value() || !store_.GetMapRecord(to).has_value()) {
      return FailureJson("map_edge_set", "map edge endpoint not found", "map_not_found");
    }
    if (edge_type.find('\n') != std::string::npos || edge_type.find('\r') != std::string::npos) {
      return FailureJson("map_edge_set", "invalid edge type", "invalid_edge");
    }
    MapGraph graph([this](const std::string& id) {
      return MapStore::IsValidMapId(id) && store_.GetMapRecord(id).has_value();
    });
    if (std::filesystem::is_regular_file(GraphPath())) {
      const auto loaded = graph.Load(GraphPath());
      if (!loaded.ok && loaded.corrupt) {
        return FailureJson("map_edge_set", loaded.message, "graph_corrupt");
      }
    }
    MapGraphNode from_node;
    from_node.node_id = GraphNodeId(from);
    from_node.map_id = from;
    from_node.type = MapGraphNodeType::kPortal;
    from_node.label = from;
    from_node.metadata["map_id"] = from;
    if (graph.FindNode(from_node.node_id) == nullptr) {
      const auto added = graph.AddNode(std::move(from_node));
      if (!added.ok) {
        return FailureJson("map_edge_set", added.message, "invalid_edge");
      }
    }
    MapGraphNode to_node;
    to_node.node_id = GraphNodeId(to);
    to_node.map_id = to;
    to_node.type = MapGraphNodeType::kPortal;
    to_node.label = to;
    to_node.metadata["map_id"] = to;
    if (graph.FindNode(to_node.node_id) == nullptr) {
      const auto added = graph.AddNode(std::move(to_node));
      if (!added.ok) {
        return FailureJson("map_edge_set", added.message, "invalid_edge");
      }
    }
    MapGraphEdge edge;
    edge.edge_id = GraphEdgeId(from, to);
    edge.from_node_id = GraphNodeId(from);
    edge.to_node_id = GraphNodeId(to);
    edge.direction = bidirectional
        ? MapGraphEdgeDirection::kBidirectional
        : MapGraphEdgeDirection::kForward;
    edge.transition_metadata["type"] = edge_type.empty() ? "link" : edge_type;
    const std::string edge_id = edge.edge_id;
    if (graph.FindEdge(edge_id) != nullptr) {
      const auto removed = graph.RemoveEdge(edge_id);
      if (!removed.ok) {
        return FailureJson("map_edge_set", removed.message, "invalid_edge");
      }
    }
    const auto added = graph.AddEdge(std::move(edge));
    if (!added.ok) {
      return FailureJson("map_edge_set", added.message, "invalid_edge");
    }
    const auto saved = graph.Save(GraphPath());
    if (!saved.ok) {
      return FailureJson("map_edge_set", saved.message, "graph_store_write_failed");
    }
    return "{"
        "\"action\":\"map_edge_set\","
        "\"success\":true,"
        "\"schema_version\":\"map.graph.v1\","
        "\"storage\":\"map_graph.ltg\","
        "\"edge\":{\"from\":" + JsonString(from) + ",\"to\":" + JsonString(to) +
        ",\"type\":" + JsonString(edge_type.empty() ? "link" : edge_type) +
        ",\"bidirectional\":" + BoolJson(bidirectional) + "}"
        "}";
  } catch (const std::exception& exc) {
    return FailureJson("map_edge_set", exc.what(), "invalid_map_name");
  }
}

std::string MapsServiceCore::DeleteMapEdgeJson(
    const std::string& from_map_id,
    const std::string& to_map_id) {
  try {
    const std::string from = MapStore::NormalizeMapId(from_map_id);
    const std::string to = MapStore::NormalizeMapId(to_map_id);
    auto graph_lock = MapLock::TryAcquire(store_.RootDir(), "__map_graph__", "map-edge-delete");
    if (!graph_lock.has_value()) {
      return FailureJson("map_edge_delete", "map graph write in progress", "map_graph_busy");
    }
    MapGraph graph([](const std::string& id) { return MapStore::IsValidMapId(id); });
    const auto loaded = graph.Load(GraphPath());
    if (!loaded.ok) {
      return FailureJson(
          "map_edge_delete",
          loaded.message,
          loaded.corrupt ? "graph_corrupt" : "edge_not_found");
    }
    const std::string edge_id = GraphEdgeId(from, to);
    const auto* existing = graph.FindEdge(edge_id);
    if (existing == nullptr) {
      return FailureJson("map_edge_delete", "map edge not found", "edge_not_found");
    }
    const std::string from_node_id = existing->from_node_id;
    const std::string to_node_id = existing->to_node_id;
    const auto removed = graph.RemoveEdge(edge_id);
    if (!removed.ok) {
      return FailureJson("map_edge_delete", removed.message, "edge_not_found");
    }
    for (const auto& node_id : {from_node_id, to_node_id}) {
      const auto* node = graph.FindNode(node_id);
      if (node != nullptr && node->type == MapGraphNodeType::kPortal &&
          node->node_id == GraphNodeId(node->map_id)) {
        (void)graph.RemoveNodeIfUnreferenced(node_id);
      }
    }
    const auto saved = graph.Save(GraphPath());
    if (!saved.ok) {
      return FailureJson("map_edge_delete", saved.message, "graph_store_write_failed");
    }
    return "{"
        "\"action\":\"map_edge_delete\","
        "\"success\":true,"
        "\"from\":" + JsonString(from) + ","
        "\"to\":" + JsonString(to) +
        "}";
  } catch (const std::exception& exc) {
    return FailureJson("map_edge_delete", exc.what(), "invalid_map_name");
  }
}

std::string MapsServiceCore::ShortestRouteJson(
    const std::string& start_map_id,
    const std::string& goal_map_id) const {
  try {
    const std::string start = MapStore::NormalizeMapId(start_map_id);
    const std::string goal = MapStore::NormalizeMapId(goal_map_id);
    auto graph_lock = MapLock::TryAcquire(store_.RootDir(), "__map_graph__", "shortest-route");
    if (!graph_lock.has_value()) {
      return FailureJson("shortest_route", "map graph write in progress", "map_graph_busy");
    }
    MapGraph graph([this](const std::string& id) {
      return MapStore::IsValidMapId(id) && store_.GetMapRecord(id).has_value();
    });
    const auto loaded = graph.Load(GraphPath());
    if (!loaded.ok) {
      return FailureJson(
          "shortest_route",
          loaded.message,
          loaded.corrupt ? "graph_corrupt" : "route_not_found");
    }
    const auto route = graph.ShortestRoute(GraphNodeId(start), GraphNodeId(goal));
    std::ostringstream nodes;
    nodes << "[";
    for (std::size_t i = 0; i < route.node_ids.size(); ++i) {
      if (i != 0U) nodes << ",";
      nodes << JsonString(MapIdFromGraphNodeId(route.node_ids[i]));
    }
    nodes << "]";
    std::ostringstream edges;
    edges << "[";
    for (std::size_t i = 0; i < route.edge_ids.size(); ++i) {
      if (i != 0U) edges << ",";
      edges << JsonString(route.edge_ids[i]);
    }
    edges << "]";
    std::ostringstream transitions;
    transitions << "[";
    for (std::size_t i = 0; i < route.transitions.size(); ++i) {
      if (i != 0U) transitions << ",";
      const auto& transition = route.transitions[i];
      transitions << "{"
                  << "\"edge_id\":" << JsonString(transition.edge_id) << ","
                  << "\"from_map_id\":" << JsonString(transition.from_map_id) << ","
                  << "\"to_map_id\":" << JsonString(transition.to_map_id)
                  << "}";
    }
    transitions << "]";
    return "{"
        "\"action\":\"shortest_route\","
        "\"success\":" + BoolJson(route.found) + ","
        "\"schema_version\":\"map.graph.route.v1\","
        "\"from\":" + JsonString(start) + ","
        "\"to\":" + JsonString(goal) + ","
        "\"found\":" + BoolJson(route.found) + ","
        "\"total_cost\":" + std::to_string(route.total_cost) + ","
        "\"nodes\":" + nodes.str() + ","
        "\"edges\":" + edges.str() + ","
        "\"transitions\":" + transitions.str() +
        "}";
  } catch (const std::exception& exc) {
    return FailureJson("shortest_route", exc.what(), "invalid_map_name");
  }
}

}  // namespace lingtu::maps

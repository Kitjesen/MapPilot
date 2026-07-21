#include "keypose_graph.hpp"

#include "frontier.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace lingtu::explore::detail {
namespace {

double Distance(const Pose2D& lhs, const Pose2D& rhs) {
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

bool FinitePose(const Pose2D& pose) {
  return std::isfinite(pose.x) && std::isfinite(pose.y) &&
      std::isfinite(pose.yaw);
}

}  // namespace

KeyposeGraph::KeyposeGraph(KeyposeGraphConfig config)
    : config_(std::move(config)) {
  if (!(config_.min_node_distance_m > 0.0) ||
      !(config_.connect_distance_m >= config_.min_node_distance_m) ||
      config_.max_nodes < 2U || config_.max_edges < 1U ||
      config_.max_neighbor_links == 0U) {
    throw std::invalid_argument("invalid keypose graph configuration");
  }
}

void KeyposeGraph::Reset() {
  nodes_.clear();
  edges_.clear();
  next_node_id_ = 1U;
}

bool KeyposeGraph::HasEdge(std::size_t from, std::size_t to) const {
  return std::any_of(
      edges_.begin(),
      edges_.end(),
      [from, to](const Edge& edge) {
        return (edge.from == from && edge.to == to) ||
            (edge.from == to && edge.to == from);
      });
}

bool KeyposeGraph::AddEdge(
    std::size_t from,
    std::size_t to,
    double cost_m,
    std::string* reason) {
  if (from == to || from >= nodes_.size() || to >= nodes_.size() ||
      !std::isfinite(cost_m) || cost_m <= 0.0) {
    if (reason != nullptr) {
      *reason = "invalid_keypose_edge";
    }
    return false;
  }
  if (HasEdge(from, to)) {
    return true;
  }
  if (edges_.size() >= config_.max_edges) {
    if (reason != nullptr) {
      *reason = "resource_limit_keypose_edges";
    }
    return false;
  }
  edges_.push_back({from, to, cost_m});
  return true;
}

bool KeyposeGraph::Update(
    const Pose2D& robot,
    const Grid2D& grid,
    std::uint64_t generation,
    const ExploreCancelCheck& cancel,
    std::string* reason) {
  if (!FinitePose(robot) || generation == 0U) {
    if (reason != nullptr) {
      *reason = "invalid_keypose_input";
    }
    return false;
  }
  if (cancel && cancel()) {
    if (reason != nullptr) {
      *reason = "cancelled";
    }
    return false;
  }

  if (nodes_.empty()) {
    nodes_.push_back({next_node_id_++, robot, generation});
    return true;
  }
  if (Distance(nodes_.back().pose, robot) < config_.min_node_distance_m) {
    return true;
  }
  if (nodes_.size() >= config_.max_nodes) {
    if (reason != nullptr) {
      *reason = "resource_limit_keypose_nodes";
    }
    return false;
  }
  if (edges_.size() >= config_.max_edges) {
    if (reason != nullptr) {
      *reason = "resource_limit_keypose_edges";
    }
    return false;
  }

  const std::size_t previous = nodes_.size() - 1U;
  std::vector<std::pair<double, std::size_t>> nearby;
  nearby.reserve(std::min(nodes_.size(), config_.max_neighbor_links * 4U));
  for (std::size_t index = 0; index < nodes_.size(); ++index) {
    if (index == previous) {
      continue;
    }
    const double distance = Distance(nodes_[index].pose, robot);
    if (distance > config_.connect_distance_m) {
      continue;
    }
    if (HasFreeLineOfSight(grid, nodes_[index].pose, robot)) {
      nearby.emplace_back(distance, index);
    }
    if ((index & 255U) == 0U && cancel && cancel()) {
      if (reason != nullptr) {
        *reason = "cancelled";
      }
      return false;
    }
  }
  std::sort(nearby.begin(), nearby.end());

  const std::size_t available_optional =
      config_.max_edges - edges_.size() - 1U;
  const std::size_t optional_count = std::min(
      {nearby.size(), config_.max_neighbor_links, available_optional});

  nodes_.push_back({next_node_id_++, robot, generation});
  const std::size_t current = nodes_.size() - 1U;
  if (!AddEdge(
          previous,
          current,
          Distance(nodes_[previous].pose, robot),
          reason)) {
    return false;
  }
  for (std::size_t index = 0; index < optional_count; ++index) {
    if (!AddEdge(nearby[index].second, current, nearby[index].first, reason)) {
      return false;
    }
  }
  return true;
}

std::vector<Pose2D> KeyposeGraph::RouteHome() const {
  if (nodes_.empty()) {
    return {};
  }
  if (nodes_.size() == 1U) {
    return {nodes_.front().pose};
  }

  const std::size_t start = nodes_.size() - 1U;
  const std::size_t goal = 0U;
  std::vector<double> distance(
      nodes_.size(),
      std::numeric_limits<double>::infinity());
  std::vector<std::size_t> predecessor(
      nodes_.size(),
      std::numeric_limits<std::size_t>::max());
  using Entry = std::pair<double, std::size_t>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
  distance[start] = 0.0;
  queue.emplace(0.0, start);

  while (!queue.empty()) {
    const auto [cost, node] = queue.top();
    queue.pop();
    if (cost > distance[node] + 1e-9) {
      continue;
    }
    if (node == goal) {
      break;
    }
    for (const Edge& edge : edges_) {
      std::size_t next = std::numeric_limits<std::size_t>::max();
      if (edge.from == node) {
        next = edge.to;
      } else if (edge.to == node) {
        next = edge.from;
      } else {
        continue;
      }
      const double next_cost = cost + edge.cost_m;
      if (next_cost + 1e-9 < distance[next]) {
        distance[next] = next_cost;
        predecessor[next] = node;
        queue.emplace(next_cost, next);
      }
    }
  }
  if (!std::isfinite(distance[goal])) {
    return {};
  }

  std::vector<std::size_t> indices;
  for (std::size_t node = goal;
       node != std::numeric_limits<std::size_t>::max();
       node = predecessor[node]) {
    indices.push_back(node);
    if (node == start) {
      break;
    }
  }
  if (indices.empty() || indices.back() != start) {
    return {};
  }
  std::reverse(indices.begin(), indices.end());

  std::vector<Pose2D> route;
  route.reserve(indices.size());
  for (const std::size_t index : indices) {
    route.push_back(nodes_[index].pose);
  }
  return route;
}

const Pose2D* KeyposeGraph::Home() const {
  return nodes_.empty() ? nullptr : &nodes_.front().pose;
}

KeyposeGraphStats KeyposeGraph::Stats() const {
  return {nodes_.size(), edges_.size()};
}

}  // namespace lingtu::explore::detail

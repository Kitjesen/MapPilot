#pragma once

#include "explore_contract.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace lingtu::explore::detail {

struct KeyposeGraphConfig {
  double min_node_distance_m{0.75};
  double connect_distance_m{6.0};
  std::size_t max_nodes{4096U};
  std::size_t max_edges{16384U};
  std::size_t max_neighbor_links{6U};
};

struct KeyposeGraphStats {
  std::size_t nodes{0U};
  std::size_t edges{0U};
};

class KeyposeGraph {
 public:
  explicit KeyposeGraph(KeyposeGraphConfig config = {});

  void Reset();
  [[nodiscard]] bool Update(
      const Pose2D& robot,
      const Grid2D& grid,
      std::uint64_t generation,
      const ExploreCancelCheck& cancel,
      std::string* reason);

  [[nodiscard]] std::vector<Pose2D> RouteHome() const;
  [[nodiscard]] const Pose2D* Home() const;
  [[nodiscard]] KeyposeGraphStats Stats() const;

 private:
  struct Node {
    std::uint64_t id{0U};
    Pose2D pose;
    std::uint64_t generation{0U};
  };

  struct Edge {
    std::size_t from{0U};
    std::size_t to{0U};
    double cost_m{0.0};
  };

  [[nodiscard]] bool HasEdge(std::size_t from, std::size_t to) const;
  [[nodiscard]] bool AddEdge(
      std::size_t from,
      std::size_t to,
      double cost_m,
      std::string* reason);

  KeyposeGraphConfig config_;
  std::vector<Node> nodes_;
  std::vector<Edge> edges_;
  std::uint64_t next_node_id_{1U};
};

}  // namespace lingtu::explore::detail

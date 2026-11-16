#include "mls_backend.hpp"

#include <cstdio>
#include <sstream>
#include <stdexcept>

using global_planner::PointPose;
using global_planner::mls::Artifact;
using global_planner::mls::Edge;
using global_planner::mls::Node;
using global_planner::mls::SurfaceCell;

namespace {
void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
}

int main() try {
  const double resolution = 0.05;
  std::vector<SurfaceCell> ring;
  for (int y = 0; y < 3; ++y) {
    for (int x = 0; x < 3; ++x) {
      if (x != 1 || y != 1) ring.push_back({x, y, 0});
    }
  }
  const LtMlsConfig surface_config{1, 1, 0.0F};
  const auto rust_surface = global_planner::mls::extractSurfacesRust(ring, {}, surface_config);
  require(rust_surface.observed.size() == ring.size(),
          "Rust MLS observed surface count changed");
  require(rust_surface.inferred == std::vector<SurfaceCell>{{1, 1, 0}},
          "Rust MLS bounded closing did not fill the center hole");

  std::vector<SurfaceCell> line;
  for (int x = 0; x < 12; ++x) line.push_back({x, 0, 0});
  const auto rust_graph = global_planner::mls::buildGraphRust(line, 3, 1);
  require(rust_graph.nodes.size() >= 3, "Rust MLS graph did not place sparse nodes");
  require(!rust_graph.edges.empty(), "Rust MLS graph did not build Voronoi edges");
  require(rust_graph.edges.front().corridor.size() > 2,
          "Rust MLS graph did not export a corridor chain");

  const LtMlsGraphConfig graph_config{3, 1, 0, 0, 0.0F, 0.0F};
  auto handle = global_planner::mls::PreparedArtifactHandle::create(
      line, {}, surface_config, graph_config, 7);
  const auto artifact_summary = handle.summary();
  require(artifact_summary.revision == 7 && artifact_summary.observed_surfaces == line.size(),
          "Rust MLS artifact summary changed");
  require(artifact_summary.inferred_surfaces == 0 && artifact_summary.nodes >= 3 &&
              artifact_summary.edges > 0,
          "Rust MLS artifact did not retain the prepared graph");
  const auto artifact_surface = handle.surfaces();
  require(artifact_surface.observed == line && artifact_surface.inferred.empty(),
          "Rust MLS artifact surface export changed");
  require(handle.below(3, 0, 0) == std::optional<std::int32_t>(0) &&
              !handle.inferred(3, 0, 0, 0),
          "Rust MLS surface queries changed");
  require(handle.occupied(3, 0, 0, 0), "Rust MLS occupied query changed");
  require(handle.occupiedBelow(3, 0, 1) == std::optional<std::int32_t>(0),
          "Rust MLS occupied-below query changed");
  require(handle.regionOccupied(LtMlsRegion{2, 3, 0, 0, 0, 0}),
          "Rust MLS region query changed");
  require(handle.height(0.35, 0.025, 0.0, 0.05, resolution) ==
              std::optional<double>(0.0),
          "Rust MLS height query changed");
  const auto artifact_graph = handle.graph();
  require(artifact_graph.nodes.size() == artifact_summary.nodes &&
              artifact_graph.edges.size() == artifact_summary.edges,
          "Rust MLS artifact graph export changed");
  require(!artifact_graph.edges.empty() && artifact_graph.edges.front().id != 0,
          "Rust MLS graph did not export stable edge IDs");
  const auto artifact_route = handle.query({0, 0, 0}, {11, 0, 0});
  require(!artifact_route.empty() && artifact_route.front() == SurfaceCell{0, 0, 0} &&
              artifact_route.back() == SurfaceCell{11, 0, 0},
          "Rust MLS artifact graph query did not return a route");
  const auto route_check = handle.checkRoute(
      artifact_route, LtMlsRouteCheckConfig{0, 0, 1, 1.0F, 0, {}});
  require(route_check.valid && route_check.failed_segment < 0,
          "Rust MLS route check rejected a valid route");
  const auto limited_route = handle.query({0, 0, 0}, {11, 0, 0}, 12);
  require(limited_route == artifact_route,
          "Rust MLS limited graph query changed an attached route");
  bool budget_rejected = false;
  try {
    (void)handle.query({0, 0, 0}, {11, 0, 0}, 12, 1);
  } catch (const std::exception&) {
    budget_rejected = true;
  }
  require(budget_rejected, "Rust MLS graph query ignored its iteration budget");
  const auto update = handle.update({1, 1, 0, 1, 0, 0}, {{1, 0, 0}}, {});
  require(update.revision == 8 && !update.graph_rebuilt_globally,
          "Rust MLS artifact update did not use local graph repair");
  auto surface_handle = global_planner::mls::PreparedArtifactHandle::createFromSurface(
      line, {}, graph_config, 11);
  require(surface_handle.summary().revision == 11 &&
              surface_handle.summary().observed_surfaces == line.size(),
          "Rust MLS surface artifact did not retain observed cells");
  require(!surface_handle.query({0, 0, 0}, {11, 0, 0}, 12).empty(),
          "Rust MLS surface artifact did not query a route");

  const auto cell = global_planner::mls::cellFromWorld({0.0, 0.0, 0.0}, resolution);
  require(cell == SurfaceCell{0, 0, 0}, "world-to-cell conversion changed");
  const auto point = global_planner::mls::worldFromCell(cell, resolution);
  require(point.x == 0.025 && point.y == 0.025 && point.z == 0.025,
          "cell-to-world conversion changed");

  Artifact source;
  source.resolution_m = resolution;
  source.revision = 7;
  source.observed = {{0, 0, 0}, {2, 0, 0}};
  source.inferred = {{1, 0, 0}};
  source.nodes = {{10, {0, 0, 0}}, {20, {2, 0, 0}}};
  source.edges = {Edge{10, 20, 0.10, {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}}}};
  require(global_planner::mls::validate(source), "valid MLS artifact rejected");

  std::stringstream stream(std::ios::in | std::ios::out | std::ios::binary);
  { lingtu::maps::BinaryWriter out(stream); global_planner::mls::write(out, source); }
  stream.seekg(0);
  Artifact decoded;
  { lingtu::maps::BinaryReader in(stream); decoded = global_planner::mls::read(in); }
  require(decoded == source, "MLS artifact round trip changed data");
  require(decoded.observed == source.observed && decoded.inferred == source.inferred,
          "MLS artifact round trip changed surface provenance");
  require(decoded.edges.front().id != 0, "MLS artifact did not restore stable edge ID");

  std::stringstream legacy_stream(std::ios::in | std::ios::out | std::ios::binary);
  {
    lingtu::maps::BinaryWriter out(legacy_stream);
    out.text("lingtu.mls.artifact");
    out.value<std::uint32_t>(2);
  }
  legacy_stream.seekg(0);
  bool legacy_rejected = false;
  try {
    lingtu::maps::BinaryReader in(legacy_stream);
    (void)global_planner::mls::read(in);
  } catch (const std::exception&) {
    legacy_rejected = true;
  }
  require(legacy_rejected, "legacy MLS artifact format accepted");

  Artifact invalid = source;
  invalid.edges[0].to = 99;
  require(!global_planner::mls::validate(invalid), "invalid node reference accepted");
  invalid = source;
  invalid.edges[0].corridor.clear();
  require(!global_planner::mls::validate(invalid), "empty MLS corridor accepted");
  invalid = source;
  invalid.observed.push_back({-2000000000, 2000000000, -2000000000});
  require(global_planner::mls::validate(invalid), "large signed MLS cell rejected");
  invalid.inferred.push_back({-2000000000, 2000000000, -2000000000});
  require(!global_planner::mls::validate(invalid), "duplicate large signed MLS cell accepted");
  return 0;
} catch (const std::exception& error) {
  return (std::fprintf(stderr, "%s\n", error.what()), 1);
}

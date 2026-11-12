#include "lingtu/maps/edit.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>
#include <stdexcept>
#include <vector>

namespace {

using lingtu::maps::PointXyz;
using lingtu::maps::SelectSourceEdits;
using lingtu::maps::SourceEditOperation;
using lingtu::maps::ValidateSourceEdits;

void Require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

template <typename Exception, typename Action>
void Throws(Action action, const char* message) {
  try {
    action();
  } catch (const Exception&) {
    return;
  }
  throw std::runtime_error(message);
}

void NoiseFilters() {
  std::vector<PointXyz> points;
  for (int x = 0; x < 3; ++x)
    for (int y = 0; y < 3; ++y)
      for (int z = 0; z < 3; ++z)
        points.push_back({x * .01F, y * .01F, z * .01F});
  points.push_back({5, 5, 5});
  SourceEditOperation statistical;
  statistical.operation = "statistical";
  statistical.neighbors = 5;
  statistical.std_ratio = 1;
  ValidateSourceEdits({statistical});
  const auto denoised = SelectSourceEdits(points, {statistical});
  Require(denoised.size() == 27 && denoised.back() == 26,
          "statistical filter did not separate isolated noise");

  SourceEditOperation radius;
  radius.operation = "radius";
  radius.radius = .05;
  radius.min_points = 5;
  Require(SelectSourceEdits(points, {radius}) == denoised,
          "radius filter did not separate isolated noise");
  radius.min_points = 1;
  Require(SelectSourceEdits({{0, 0, 0}}, {radius}).size() == 1,
          "radius neighborhood did not include the query point");
  Require(SelectSourceEdits({{0, 0, 0}}, {statistical}).size() == 1,
          "statistical filter discarded a singleton");
}

void StatisticalMatchesReference() {
  std::mt19937 random(7);
  std::uniform_real_distribution<float> coordinate(-1, 1);
  for (bool plane : {false, true}) {
    std::vector<PointXyz> points;
    for (int i = 0; i < 100; ++i)
      points.push_back({coordinate(random), coordinate(random), plane ? 7 : coordinate(random)});
    for (int i = 0; i < 12; ++i) points.push_back(points.front());
    points.push_back({8, 8, plane ? 7.0F : 8.0F});
    for (int neighbors : {1, 7, 20, 200}) {
      SourceEditOperation edit;
      edit.operation = "statistical";
      edit.neighbors = neighbors;
      edit.std_ratio = 1.2;
      const auto count = std::min<std::size_t>(neighbors, points.size() - 1);
      std::vector<double> averages(points.size());
      for (std::size_t i = 0; i < points.size(); ++i) {
        std::vector<double> distances;
        for (std::size_t j = 0; j < points.size(); ++j) {
          if (i == j) continue;
          const double x = static_cast<double>(points[i].x) - points[j].x;
          const double y = static_cast<double>(points[i].y) - points[j].y;
          const double z = static_cast<double>(points[i].z) - points[j].z;
          distances.push_back(std::sqrt(x*x + y*y + z*z));
        }
        std::sort(distances.begin(), distances.end());
        for (std::size_t n = count; n > 0; --n) averages[i] += distances[n - 1];
        averages[i] /= static_cast<double>(count);
      }
      const double mean = std::accumulate(averages.begin(), averages.end(), 0.0) /
                          static_cast<double>(averages.size());
      double variance = 0;
      for (double average : averages) variance += (average - mean) * (average - mean);
      const double threshold = mean + edit.std_ratio *
          std::sqrt(variance / static_cast<double>(averages.size() - 1));
      std::vector<std::size_t> expected;
      for (std::size_t i = 0; i < averages.size(); ++i)
        if (averages[i] <= threshold) expected.push_back(i);
      Require(SelectSourceEdits(points, {edit}) == expected,
              "indexed statistical filter differs from exhaustive kNN");
    }
  }
}

void DenseClusters() {
  std::vector<PointXyz> points;
  for (int x = 0; x < 5; ++x)
    for (int y = 0; y < 5; ++y)
      points.push_back({x * .01F, y * .01F, 0});
  for (int x = 0; x < 8; ++x) points.push_back({2 + x * .005F, 0, 0});
  points.push_back({5, 5, 5});
  SourceEditOperation clusters;
  clusters.operation = "clusters";
  clusters.radius = .1;
  clusters.neighbors = 5;
  clusters.min_cluster_points = 20;
  const auto kept = SelectSourceEdits(points, {clusters});
  Require(kept.size() == 25 && kept.front() == 0 && kept.back() == 24,
          "DBSCAN failed to remove small clusters and isolated noise");
  clusters.neighbors = 1;
  clusters.min_cluster_points = 1;
  Require(SelectSourceEdits({{0, 0, 0}}, {clusters}).size() == 1,
          "DBSCAN neighborhood did not include the query point");
}

void OrderedEdits() {
  const std::vector<PointXyz> points{{0, 0, 0}, {0, 0, 0}, {1, 0, 0}, {1, 0, 0}};
  SourceEditOperation voxel;
  voxel.operation = "voxel";
  voxel.voxel_size = .1;
  SourceEditOperation radius;
  radius.operation = "radius";
  radius.radius = .1;
  radius.min_points = 2;
  Require(SelectSourceEdits(points, {radius, voxel}) == std::vector<std::size_t>{0, 2},
          "voxel filter failed to keep the first original record");
  Require(SelectSourceEdits(points, {voxel, radius}).empty(),
          "edits were not applied in request order");
  Require(SelectSourceEdits(points, {}) == std::vector<std::size_t>{0, 1, 2, 3},
          "empty edits lost duplicate source identities");

  SourceEditOperation crop;
  crop.operation = "keep_inside";
  crop.bounds = {true, -.1, -.1, -.1, .1, .1, .1};
  Require(SelectSourceEdits(points, {crop}) == std::vector<std::size_t>{0, 1},
          "keep_inside did not preserve duplicate records");
  crop.operation = "delete_inside";
  Require(SelectSourceEdits(points, {crop}) == std::vector<std::size_t>{2, 3},
          "delete_inside selected the wrong source records");
  const std::vector<PointXyz> signedPoints{{-.01F, 0, 0}, {.01F, 0, 0}, {.02F, 0, 0}};
  Require(SelectSourceEdits(signedPoints, {voxel}) == std::vector<std::size_t>{0, 1},
          "voxel keys did not use floor across zero");
}

void PlaneSelection() {
  std::vector<PointXyz> points;
  for (int x = 0; x < 10; ++x)
    for (int y = 0; y < 10; ++y)
      points.push_back({x * .1F, y * .1F, 0});
  points.push_back({.4F, .4F, .05F});
  points.push_back({10, 10, 10});
  const auto original = points;
  SourceEditOperation plane;
  plane.operation = "plane_crop";
  plane.distance_threshold = .001;
  plane.margin = .1;
  const auto kept = SelectSourceEdits(points, {plane});
  Require(kept.size() == 101 && kept.front() == 0 && kept.back() == 100,
          "plane crop did not retain the expanded plane box");
  Require(kept == SelectSourceEdits(points, {plane}), "plane crop was not deterministic");
  for (std::size_t i = 0; i < points.size(); ++i)
    Require(points[i].x == original[i].x && points[i].y == original[i].y &&
            points[i].z == original[i].z, "plane crop changed source coordinates");
  Throws<std::runtime_error>([&] {
    SelectSourceEdits(std::vector<PointXyz>(points.begin(), points.begin() + 79), {plane});
  }, "plane crop silently accepted fewer than 80 plane points");
  std::vector<PointXyz> line;
  for (int i = 0; i < 100; ++i) line.push_back({static_cast<float>(i), 0, 0});
  Throws<std::runtime_error>([&] { SelectSourceEdits(line, {plane}); },
                            "plane crop silently accepted collinear points");
}

void InvalidParameters() {
  SourceEditOperation edit;
  Throws<std::invalid_argument>([&] { ValidateSourceEdits({edit}); },
                               "unknown edit operation was accepted");
  edit.operation = "keep_inside";
  Throws<std::invalid_argument>([&] { ValidateSourceEdits({edit}); },
                               "crop without bounds was accepted");
  edit.bounds = {true, 1, 0, 0, 0, 1, 1};
  Throws<std::invalid_argument>([&] { ValidateSourceEdits({edit}); },
                               "reversed crop bounds were accepted");
  edit.bounds = {true, 0, 0, 0, 1, 1, std::numeric_limits<double>::infinity()};
  Throws<std::invalid_argument>([&] { ValidateSourceEdits({edit}); },
                               "nonfinite crop bounds were accepted");
  edit.operation = "voxel";
  edit.voxel_size = 0;
  Throws<std::invalid_argument>([&] { ValidateSourceEdits({edit}); },
                               "zero voxel size was accepted");
  edit.operation = "statistical";
  edit.neighbors = 0;
  Throws<std::invalid_argument>([&] { ValidateSourceEdits({edit}); },
                               "zero statistical neighbors were accepted");
  edit.neighbors = 20;
  edit.std_ratio = std::numeric_limits<double>::quiet_NaN();
  Throws<std::invalid_argument>([&] { ValidateSourceEdits({edit}); },
                               "nonfinite statistical ratio was accepted");
  edit.operation = "radius";
  edit.radius = -1;
  Throws<std::invalid_argument>([&] { ValidateSourceEdits({edit}); },
                               "negative radius was accepted");
  edit.radius = .2;
  edit.min_points = 0;
  Throws<std::invalid_argument>([&] { ValidateSourceEdits({edit}); },
                               "zero radius neighbor count was accepted");
  edit.operation = "clusters";
  edit.min_cluster_points = 0;
  Throws<std::invalid_argument>([&] { ValidateSourceEdits({edit}); },
                               "zero cluster size was accepted");
  edit.operation = "plane_crop";
  edit.margin = -1;
  Throws<std::invalid_argument>([&] { ValidateSourceEdits({edit}); },
                               "negative plane margin was accepted");
  edit.margin = 0;
  edit.distance_threshold = 0;
  Throws<std::invalid_argument>([&] { ValidateSourceEdits({edit}); },
                               "zero plane threshold was accepted");
}

void ManualPlanes() {
  for (const auto* axis : {"x", "y", "z"}) {
    SourceEditOperation plane;
    plane.operation = "add_plane";
    plane.bounds = {true, 0, 0, 0, 1, 1, 1};
    plane.axis = axis;
    plane.offset = .5;
    plane.spacing = .5;
    ValidateSourceEdits({plane});
    std::vector<PointXyz> points{{3, 4, 5}};
    const auto selected = lingtu::maps::ApplySourceEdits(points, {plane});
    Require(selected.size() == 10 && points.size() == 10, "plane did not append its complete grid");
    Require(points.front().x == 3 && points.front().y == 4 && points.front().z == 5,
            "manual fill changed the original point");
    for (std::size_t i = 1; i < points.size(); ++i) {
      const float coordinate = plane.axis == "x" ? points[i].x : plane.axis == "y" ? points[i].y : points[i].z;
      Require(coordinate == .5F, "plane uses the wrong map axis");
    }
    SourceEditOperation crop;
    crop.operation = "delete_inside";
    crop.bounds = plane.bounds;
    points = {{3, 4, 5}};
    Require(lingtu::maps::ApplySourceEdits(points, {plane, crop}) == std::vector<std::size_t>{0},
            "later crops did not remove artificial points");
    plane.spacing = .00001;
    Throws<std::invalid_argument>([&] { ValidateSourceEdits({plane}); }, "unbounded manual grid accepted");
    plane.spacing = .5;
    plane.offset = 2;
    Throws<std::invalid_argument>([&] { ValidateSourceEdits({plane}); }, "plane outside selection accepted");
  }
}

}  // namespace

int main() {
  NoiseFilters();
  StatisticalMatchesReference();
  DenseClusters();
  OrderedEdits();
  PlaneSelection();
  InvalidParameters();
  ManualPlanes();
  return 0;
}

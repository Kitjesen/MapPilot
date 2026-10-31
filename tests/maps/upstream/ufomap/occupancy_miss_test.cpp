#include <ufo/map/ufomap.hpp>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
void Require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

template <class Map>
void MissAt(Map& map, ufo::Point point, const ufo::IntegrationParams& params) {
  const auto code = map.toCode(point);
  ufo::BitSet<8> siblings(0);
  siblings.set(code.offset());
  ufo::impl::Misses misses;
  misses.emplace_back(code, siblings);
  // Exercise the real update method with an explicit measured-free sibling.
  // Ray generation is intentionally outside this focused regression.
  ufo::impl::integrateMisses(map, std::move(misses), params);
}

template <ufo::mt_t Type>
void OccupancyMiss(bool parallel, float miss_probability) {
  ufo::Map<Type> map(.1, 8);
  const ufo::Point point(1.05F, .05F, .05F);
  const ufo::Point neighbor(1.15F, .05F, .05F);
  ufo::IntegrationParams params;
  params.parallel = parallel;
  params.occupancy_hit = .7F;
  params.occupancy_miss = miss_probability;
  ufo::IntegrationCloud<ufo::Point> hits;
  hits.emplace_back(point, map.toCode(point));
  const int initial = map.occupancyLogit(point);
  for (int frame = 0; frame < 5; ++frame) ufo::impl::integrateHits(map, hits, params);
  map.setOccupancy(neighbor, .6F, false);
  const int neighbor_before = map.occupancyLogit(neighbor);
  const int before = map.occupancyLogit(point);
  const float probability_before = map.occupancy(point);
  // This pinned version returns the magnitude for probabilities below .5.
  const int change = -static_cast<int>(map.toOccupancyChangeLogit(params.occupancy_miss));
  Require(before > initial && probability_before > .5F && change < 0,
      "fixture did not establish occupied and negative miss evidence");
  MissAt(map, point, params);
  const int after = map.occupancyLogit(point);
  std::cout << "type=" << Type << " parallel=" << parallel << " miss=" << miss_probability
            << " before=" << before << " change=" << change << " after=" << after << '\n';
  Require(after == before + change && map.occupancy(point) < probability_before,
      "occupancy miss did not apply the configured signed logit change");
  Require(map.occupancyLogit(neighbor) == neighbor_before,
      "miss updated an unselected sibling");
  if constexpr ((Type & ufo::SEEN_FREE) != 0) {
    Require(map.seenFree(point) && !map.seenFree(neighbor),
        "mixed occupancy map lost or spread SeenFree evidence");
  }
  for (int frame = 0; frame < 8; ++frame) MissAt(map, point, params);
  Require(map.occupancy(point) < .5F, "repeated misses did not clear occupancy");
  for (int frame = 0; frame < 32; ++frame) ufo::impl::integrateHits(map, hits, params);
  Require(map.occupancy(point) > .5F, "later hits could not restore occupancy");
}

void DufomapSeenFree(bool parallel) {
  ufo::Map<ufo::SEEN_FREE | ufo::REFLECTION | ufo::LABEL> map(.1, 8);
  const ufo::Point point(1.05F, .05F, .05F);
  ufo::IntegrationParams params;
  params.parallel = parallel;
  ufo::IntegrationCloud<ufo::Point> hits;
  hits.emplace_back(point, map.toCode(point));
  ufo::impl::integrateHits(map, hits, params);
  Require(!map.seenFree(point), "hit was marked as seen-free");
  MissAt(map, point, params);
  Require(map.seenFree(point), "DUFOMap SeenFree path regressed");
}
}

int main(int argc, char** argv) {
  try {
    const bool parallel = argc > 1 && std::string(argv[1]) == "parallel";
    for (const float miss : {.4F, .25F}) {
      OccupancyMiss<ufo::OCCUPANCY>(parallel, miss);
      OccupancyMiss<ufo::OCCUPANCY | ufo::SEEN_FREE>(parallel, miss);
    }
    DufomapSeenFree(parallel);
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}

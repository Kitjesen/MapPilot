#include <cmath>
#include <iostream>

#include <ufo/map/ufomap.hpp>

int main() {
  using Map = ufo::Map<ufo::MapType::SEEN_FREE | ufo::MapType::REFLECTION | ufo::MapType::LABEL>;
  Map map(0.05, 20);
  const auto origin = map.toKey(ufo::Point{0, 0, 0});
  for (int axis = 0; axis != 3; ++axis) {
    for (int cell : {-81, -21, -2, -1, 0, 1, 20, 80}) {
      ufo::Point point{0.025F, 0.025F, 0.025F};
      point[axis] = (cell + 0.5F) * 0.05F;
      const auto key = map.toKey(point);
      const auto expected = static_cast<ufo::key_t>(
          static_cast<long long>(origin[axis]) + cell);
      if (key[axis] != expected || map.toKey(map.toCoord(key)) != key ||
          std::abs(map.toCoord(key)[axis] - point[axis]) > 1e-5F) {
        std::cerr << "voxel coordinate mismatch: axis=" << axis << " cell=" << cell
                  << " expected=" << expected << " actual=" << key[axis] << '\n';
        return 1;
      }
    }
  }
}

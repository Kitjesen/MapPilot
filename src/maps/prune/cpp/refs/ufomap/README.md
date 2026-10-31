# UFOMap occupancy miss reference fix

This directory retains a small patch for the UFOMap submodule checked out by
DUFOMap. UFOMap is a research reference and is not a LingTu production map backend.
The standalone regression is not part of the normal maps/prune build.

- Upstream: <https://github.com/UnknownFreeOccupied/ufomap>
- Pinned commit: `fcb20f445d257280c039c07ab756c9d5b37ebd78`
- Checkout: `third_party/research_nav/dufomap/ufomap`
- License: upstream BSD-3-Clause; the original source license is retained.

`integration_impl.hpp::integrateMisses` declares an inner `auto prob` that
shadows the initialized outer zero. Both miss branches also call the unavailable
`decreaseOccupancyLogit` API. The patch assigns the outer variable and calls the
existing `updateOccupancyLogit(node, -prob)` in serial and parallel branches.
At this pinned commit, `toOccupancyChangeLogit` returns a positive magnitude for
miss probabilities below 0.5, and `updateOccupancyLogit` adds its signed argument.
Passing the magnitude without negation would increase occupancy.

Apply to a clean checkout of the pinned commit, from the LingTu repository root:

```bash
git -C third_party/research_nav/dufomap/ufomap apply --check \
  ../../../../src/maps/prune/cpp/refs/ufomap/occupancy-miss.patch
git -C third_party/research_nav/dufomap/ufomap apply \
  ../../../../src/maps/prune/cpp/refs/ufomap/occupancy-miss.patch
```

The local research checkout already has this patch applied. The retained patch
is necessary because `third_party/` is ignored by the main repository.

## Standalone regression

Requires a C++20 compiler, CMake >= 3.20, LZ4, the `liblzf/lzf.h` header, TBB and
OpenMP. These are test-host dependencies; this change adds no production dependency.
From the repository root:

```bash
cmake -S tests/maps/upstream/ufomap -B build/ufomap-miss-regression \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build/ufomap-miss-regression -j4
ctest --test-dir build/ufomap-miss-regression -V
```

For a locally extracted LZF development package, pass
`-DUFOMAP_LZF_INCLUDE_DIR=/absolute/path/to/usr/include` at configure time.
`UFOMAP_SOURCE_DIR` can select another checkout of the same pinned version.

The three CTest cases cover a build without `UFO_PARALLEL`, a parallel-enabled
build running the serial branch, and a parallel-enabled build running the
parallel branch. They use real maps with `OCCUPANCY`, `OCCUPANCY | SEEN_FREE`,
and DUFOMap's `SEEN_FREE | REFLECTION | LABEL` layers. Checks include configured
miss probabilities 0.4 and 0.25, exact logit decrements, an unaffected sibling,
repeated clearing, subsequent hit recovery, and retained `SeenFree` behavior.

The fixture starts from repeated actual hits and verifies an occupied probability
before applying misses. It does not validate the pinned library's default prior
or unfinished signed-logit conversion helpers.

The tests directly construct `Misses` to isolate the real miss application.
They do not cover ray generation or bit-grid-to-miss extraction. The pinned
upstream `integration/misses.hpp` still has a TODO in the serial extraction
branch: without `UFO_PARALLEL`, or with extraction thread count <= 1, that branch
returns an empty list. The serial ray-generation caller leaves this count at
zero. That separate issue is outside this patch; these passing tests do not
qualify the full ordinary occupancy pipeline for production.

# Offline historical point cleanup experiment

`IncrementalCleaner` is retained for recorded-scan experiments. It is not linked
into mapd or the native SLAM whole-map preview. Those paths keep their original
map aggregation. Save-time `prune` now uses the pinned DUFOMap/UFOMap backend.
The old header-only planar helper is retained here as `surface.hpp` solely for
this offline experiment. It is no longer part of the production classifier.

The integration adapts DUFOMap's conservative measured free-space principle and
FreeDOM's incremental historical relabeling. It is not a port of either complete
system: no UFOMap tree, ROS node, synthetic no-return rays, FreeDOM stair ray
enhancement, clustering/tracking, or semantic dynamic-object detector is linked.
References are pinned in `third_party/MANIFEST.md`.

For each chronological scan, confirmed endpoints reset later-free counts and
restore reoccupied cells. Missing/occluded scans add no evidence. A historical
sample and six neighboring probes must all lie within measured ray tubes in
three separate subsequent scans. Current endpoints conservatively shorten
nearby rays. Angular bins accelerate exact tube queries; they never themselves
prove a point free. Stable repeated plane support preserves floors, walls,
ramps and separate stair treads. Patchwork++ labels need repeated hits and
local support; a lone low object cannot become permanently protected.

One representative per 10 cm geometry cell retains XYZ, intensity, last-hit
scan, free count and removed state. Labels persist across unobserved frames;
a new hit restores geometry. Storage is capped at 500,000 cells including
removed history; capacity rejections are exposed rather than silently growing
memory. Reset clears both ground adaptation and free evidence.

`Snapshot()` returns only retained representatives, not a complete map. Other
points in the same cell have not necessarily received the representative's
free-space evidence. The former `Keep(point)` cell-wide filter was removed for
this reason. This experiment must not filter another map or replace its geometry
storage; its evidence capacity cannot be used as a production map capacity.

The current design favors static preservation and can retain ghosts when no
later measured ray covers the neighborhood or stable neighboring geometry
supports them. Before any live integration, point identity, capacity behavior,
pose correction and latency isolation need a separate design and validation.

Build this experiment separately (C++20, Eigen3):

```bash
cmake -S research/maps/cleaning -B build/cleaning-experiment -DBUILD_TESTING=ON
cmake --build build/cleaning-experiment -j4
ctest --test-dir build/cleaning-experiment --output-on-failure
```

The `maps_cleaning_replay` executable is an explicit, non-default target in
this standalone experiment. Build it with
`cmake --build build/cleaning-experiment --target maps_cleaning_replay`. It writes both
saved and incremental outputs from the same recorded scans without modifying
the source. Its `live.clean.pcd` output is an offline incremental experiment,
not validation of live mapd or native whole-map preview wiring.

# Map-cleaning experiments

These are offline comparisons, not product backends. The production save path
uses only [`src/maps/prune`](../../src/maps/prune/README.md), followed by native
map construction. Product build scripts do not configure the projects below.

| Directory | Purpose | Current limit |
| --- | --- | --- |
| [`erasor2/`](erasor2/README.md) | Pinned upstream ERASOR2 and LingTu staging/launch tools | The legacy stager substitutes height and XY bins for real ground/instance labels. It is an interface smoke test, not a valid upstream quality benchmark. |
| [`cleaning/`](cleaning/README.md) | Bounded incremental visibility-cleaning experiment and saved-map replay | No SLAM/mapd runtime caller; representative output is not a full-map deletion mask. |
| [`ground/`](ground/README.md) | Patchwork++ segmentation used by the incremental experiment | Not used by production prune; needs Eigen3. |
| [`ufomap/`](ufomap/README.md) | Pinned UFOMap miss-update patch and regression | Research library fix, not a product map backend or full ray-generation qualification. |
| `tools/dufomap_offline.py` | Legacy DUFOMap comparison runner | Requires an external local executable/config; assumes float binary PCD and does not read `scan_origin.txt`. Point-count differences are not quality metrics. |
| `tests/` | Ground, incremental and ERASOR2 adapter checks | Component evidence only; does not qualify NX operation or map quality. |

From the repository root, using already-installed dependencies:

```bash
cmake -S research/maps/cleaning -B build/cleaning-experiment -DBUILD_TESTING=ON
cmake --build build/cleaning-experiment --config Release
ctest --test-dir build/cleaning-experiment -C Release --output-on-failure
cmake --build build/cleaning-experiment --config Release --target maps_cleaning_replay
```

Run `maps_cleaning_replay MAP_DIRECTORY NEW_OUTPUT_DIRECTORY` to compare the
saved-map cleaner with the experimental incremental result. The original map is
not replaced. The two outputs use different representations; retained-point
counts alone do not establish which one is correct.

For ERASOR2, use its [build/check instructions](erasor2/README.md). No research
entrypoint installs dependencies automatically. To make a quality comparison,
first supply the official ground/instance labels, align sensor coordinates and
evaluate both outputs against the same static/dynamic reference.

See [the 2026-11-05 decision and cleanup](../../docs/research/map-cleaning-boundaries-20261105.md)
for the comparison limits, deleted report-only code and production verification.

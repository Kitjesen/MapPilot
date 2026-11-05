# Legacy ERASOR2 staging experiment

This isolated GPL-3.0-only wrapper stages LingTu scan records and can invoke
upstream ERASOR2. Production `prune`, mapd and release builds have no ERASOR2
build switch or dependency. The wrapper and upstream license obligations remain
in [NOTICE.md](NOTICE.md).

The current stager generates ground labels from a fixed scan-frame Z threshold
and instance labels from XY grid cells. These are synthetic labels for file-format
and execution smoke checks. They do not reproduce Patchwork++ or HDBSCAN
segmentation, and output from this pipeline cannot establish ERASOR2 cleaning
quality or a fair algorithm comparison. Quality evaluation requires valid labels,
matched scans, poses and independent checks of retained static structures.
Current SLAM patches are in the body frame; the stager also does not apply the
calibrated origin from `scan_origin.txt`. Do not treat its default sensor
height/extrinsics as a calibrated LingTu comparison.

Upstream source is fetched at pinned revision
`d43d94f7e06a900456042979e29c3c933a39fd48` into
`third_party/research_nav/ERASOR2`. [Upstream repository](https://github.com/url-kaist/ERASOR2)

## Build and smoke check

Run from the repository root after installing the upstream build dependencies
Eigen3, PCL, Boost, OpenCV, OpenMP and yaml-cpp.

```bash
bash research/maps/erasor2/fetch.sh
bash research/maps/erasor2/build.sh
bash research/maps/erasor2/check.sh /path/to/map /path/to/new-stage
```

`check.sh` stages an existing map and prints the upstream command using
`--skip-run`. It does not run upstream cleaning. To explicitly exercise the
whole upstream process on a disposable map copy, run the built wrapper directly.

```bash
build/research-erasor2/erasor2_clean --map-dir /path/to/map-copy --stage-dir /path/to/another-new-stage
```

The no-op `rerun_stub` avoids a visualization SDK download in the isolated build.
Configure with `-DLINGTU_ERASOR2_USE_RERUN_STUB=OFF` when intentionally building
interactive upstream visualization. No research executable is installed by this
CMake project.

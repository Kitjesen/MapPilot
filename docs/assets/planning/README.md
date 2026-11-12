# Global planning figures

**Status:** Published figure gallery with historical offline results
**Audience:** Documentation readers and planning researchers
**Runs on:** Offline native planner evidence recorded on 2026-10-16

Recorded on 2026-10-16. These six figures show LingTu's native OctoPlanner3D
global routes over the upstream sample point clouds. The root README uses
200 dpi previews; each linked PDF retains the publication export with
embedded Arial 10 pt text, vector routes and a 600 dpi raster point-cloud layer.
Legends sit 6 pt above the plotted bounds. Scaling a whole figure also scales
its text.

The README processing diagram is `pipeline.svg`; its PNG preview and
Matplotlib source are `pipeline.png` and `pipeline.py`. It describes this
offline PCD benchmark, including modeled free layers, native A* edge checks
and the separate path diagnostics.

## Figures and results

| Case | 3D | X–Z elevation | Route length | Route Z range | Segments passing body sweep |
| --- | --- | --- | ---: | --- | ---: |
| Building cross-building | [PNG](building-cross-3d.png) · [PDF](building-cross-3d.pdf) | [PNG](building-cross-xz.png) · [PDF](building-cross-xz.pdf) | 89.78 m | 0.5–13.9 m | 328 / 328 |
| Building ascent | [PNG](building-stairs-3d.png) · [PDF](building-stairs-3d.pdf) | [PNG](building-stairs-xz.png) · [PDF](building-stairs-xz.pdf) | 38.37 m | 0.5–13.9 m | 140 / 140 |
| Plaza crossing | [PNG](plaza-3d.png) · [PDF](plaza-3d.pdf) | [PNG](plaza-xz.png) · [PDF](plaza-xz.pdf) | 20.36 m | 0.4–0.6 m | 99 / 99 |

Gray points show the source cloud, green the returned route, blue the start,
and magenta the goal. All axes use equal metric scaling. The X–Z views project
all Y coordinates onto one plane, so separate surfaces may overlap.

## Inputs and method

The source maps are the OctoPlanner3D sample files at pinned upstream commit
`9a9cc431ea905a5878975cc6fbbce6c9618b31a4`:

- `octomap/pcd_files/building2_9.pcd`: 853,227 source points;
  94,803 displayed points after fixed-step sampling.
- `octomap/pcd_files/plaza3_10.pcd`: 1,675,994 source points;
  98,588 displayed points after fixed-step sampling.

The planning input is a 0.2 m OctoMap converted from the PCD. For these sample
cases, `modeled_surface_free` infers five free voxel layers above identified
support surfaces. This is modeled free space, not sensor-ray evidence.
The figures overlay the source PCD and the frozen returned path; drawing does
not smooth, replan or modify the path.

The two Building requests correspond to upstream test2 and test1. Plaza uses
a LingTu regression request on the upstream Plaza map. Coordinates are metres:

| Case | Requested start (X, Y, Z) | Requested goal (X, Y, Z) |
| --- | --- | --- |
| Building cross-building | (9.3, 0.5, 0.3) | (-6.1, -1.5, 0.3) |
| Building ascent | (7.9, -2.7, 0.3) | (2.5, -0.3, 13.7) |
| Plaza crossing | (-10, 0, 0.4) | (10, 0, 0.4) |

The plotted markers are the actual snapped route endpoints. The cross-building
route ascends to the upper connection and returns to the lower level; equal
endpoint heights do not imply a known ground-level connection in this map.

## Validation scope

These are local offline geometry results from the native headless planner.
After adding sweep checks to A* edges, the replanned Building routes and the
unchanged Plaza route pass all returned-segment body-sweep checks.

Independent voxel-edge slope diagnostics still report 134, 67 and 2 failures,
respectively: `motion_diagnostic_valid=false`, with aggregate `valid=false`.
Their discretized slope estimates still need terrain and local-trajectory
validation. Connections between the original requested endpoints and the
snapped route endpoints are outside these segment counts. SCAN execution,
robot simulation and field motion have not been demonstrated by this run.
These are upstream sample maps, not the 903 field map.

## Local source records

The full run records remain local, so the paths below are provenance references,
not downloadable links from this document:

- `artifacts/segment-motion-20261016/`: original failures, source snapshot,
  fixed requests, before/after paths, diagnostics and logs.
- `artifacts/segment-motion-20261016/after/`: frozen `path.xyz` files under
  `building_cross_building_modeled`, `building_stairs_modeled` and
  `plaza_traverse_modeled`.
- `artifacts/route-audit-20261016/`: source-cloud and route measurements.
- `artifacts/paper-figures-20261016/`: six 600 dpi PNGs, PDFs, editable SVGs,
  export script, manifest and font/layout checks.

This folder contains the six reviewed figure previews and their PDFs for the
README. The run evidence and source data are not bundled into this gallery.

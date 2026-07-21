# Perception

`src/perception/` owns scene perception: detection, embedding/encoding,
cross-frame tracking, scene-graph construction, and 3D reconstruction. It is
separate from decision-making (`src/decision/`) and from map/planning state
(`src/nav/`).

This package was reorganized by function in 2026-07 (see "History" below).
The reorganization was a structural move only -- no algorithm behavior
changed.

## Layout

| Subpackage | Role | Status |
| --- | --- | --- |
| `detection/` | 2D object detectors + detector registry entries | production |
| `encoding/` | CLIP-family embedding/encoder backends | production |
| `tracking/` | Cross-frame instance tracking, 2D->3D projection, BPU re-id/association | production |
| `scene_understanding/` | Spatial Connectivity Graph (SCG) scene representation (USS-Nav design) | experimental, not wired into the blueprint |
| `research/` | Offline evaluation/benchmark harness (USS-Nav vs. PCT-A* on HM3D/Gibson) | paper tooling, not part of the robot pipeline |
| `reconstruction/` | 3D volumetric (TSDF) reconstruction pipeline | production, unchanged by this reorg |
| `api/` + `impl/` | Alternate factory-pattern construction path for `yolo_world` + `clip` + `instance` tracker | production (narrow), unchanged by this reorg |
| `inspection/` | Native parking/inspection evidence bridge and runtime Module adapter | production interface |
| `adapters/ros2/` | ROS2 bag-reader + publisher protocol adapters | compatibility/offline only; not the product default |
| `examples/` | Usage demos and offline analysis scripts | dev tooling |
| `tests/` | Unit tests (flat, mirrors repo convention) | -- |

Root-level files stay at `src/perception/` because they are aggregate/composition
entry points that span more than one subpackage:

| File | Role |
| --- | --- |
| `perception_module.py` | The production `PerceptionModule` (`@register("perception", "scene")`): detector + encoder + tracker + scene graph, wired into the runtime blueprint. |
| `service.py` | `PerceptionService` -- a framework-free (no ROS/GPU setup) detect->project->encode->track coordinator used by tests and standalone tooling. |
| `__init__.py` | Package docstring only. |

Inspection-specific evidence analysis is intentionally separate from the main
scene-graph tracker. `inspection/bridge_module.py` is the Module boundary for
native inspection evidence, and `inspection/native_bridge.*` is the C++ bridge
used by the current parking/inspection contract.

### `detection/`

Detector backends registered under the `detector` registry category, plus
their shared base type:

`detector_base.py` (`DetectorBase` ABC + `Detection2D`), `yoloe_detector.py`,
`yolo_world_detector.py`, `grounding_dino_detector.py`, `bpu_detector.py`,
`sim_scene_observer.py` (sim-only detector+projector shortcut),
`detector_module.py` (standalone `DetectorModule` tool wrapper), and
`laplacian_filter.py` (blur-gate before running the detector). The unused
legacy `keyframe_selector.py` gate has been removed.

### `encoding/`

`clip_encoder.py`, `mobileclip_encoder.py` (the default, text-only encoder),
`encoder_module.py` (standalone `EncoderModule` tool wrapper).

### `tracking/`

`instance_tracker.py` (the production scene-graph tracker: cross-frame
matching, EMA position smoothing, belief updates, room inference),
`tracked_objects.py` (`TrackedObject` dataclass; re-exports shared scene types
from `runtime.msgs.scene`), `projection.py` (2D detection + depth -> 3D
`Detection3D`, the currency between detection and tracking), `bpu_tracker.py`
(BoT-SORT 2D tracker for BPU detections), `native_byte_tracker.py`, and
`person_counting.py`. The former all-in-one `bpu_qp_bridge.py` has been
replaced by these explicit tracking services.

### `scene_understanding/`

Spatial Connectivity Graph (SCG) scene representation from the "USS-Nav"
design: `polyhedron_expansion.py` (free-space decomposition into convex
polyhedra), `scg_builder.py` (topology graph over polyhedra), `scg_path_planner.py`
(A* over the SCG), `global_coverage_mask.py` (sparse coverage/frontier
tracking), `local_rolling_grid.py` (fixed-memory local occupancy grid),
`leiden_segmentation.py` (community detection for region segmentation),
`uncertainty_model.py` (entropy/information-gain for exploration),
`geometry_extractor.py` (room geometry from a Tomogram), `hybrid_planner.py`
(topology+A* hierarchical planner).

**None of these are wired into the production blueprint today.** They are
only exercised from `tests/` and `examples/uss_nav_integration_demo.py`.
`hybrid_planner.py` and `polyhedron_expansion.py` say so explicitly in their
module docstrings ("EXPERIMENTAL -- not used in production navigation
stack"); the rest of the cluster is the same generation of work and shares
the same status even though it isn't labeled. The one production-side
extension point that overlaps with this cluster is
`memory.spatial.topology_graph.TopologySemGraph.set_geometry_extractor()`,
which accepts a `GeometryExtractor`-shaped object via duck typing (no
import) -- it is currently never called outside tests/examples either.

### `research/`

`dataset_loader.py` (HM3D/Gibson/Replica loaders), `baseline_wrappers.py`
(PCT-A* vs. USS-Nav planner wrappers for comparison -- this is what actually
imports `scene_understanding/`), `evaluation_framework.py` (memory/rate/path/
exploration metrics), `end_to_end_evaluation.py` (full comparison driver),
`visualization_tools.py` (matplotlib/plotly plots). This is the benchmark
harness behind the `docs/archive/09-paper/` evaluation notes -- it is not
imported by any Module, blueprint, or registry entry.

## Boundary

- Perception produces detections, embeddings, scene graphs, and reconstruction
  artifacts.
- It does not choose missions, global paths, local paths, or velocity commands.
  (`scene_understanding/hybrid_planner.py` and `scg_path_planner.py` are
  planning-shaped code that technically lives here for historical reasons --
  see "Open questions.")
- Decision (`src/decision/`) and memory (`src/memory/`) consume perception
  outputs through runtime messages/ports. As of this reorg, neither imports
  perception concrete classes directly except `memory.modules.vector_memory_module`
  and `memory.spatial.topology_graph`, both of which go through
  `runtime.registry` / duck typing rather than a direct `import perception...`.
- ROS 2 code stays under `adapters/ros2/` as compatibility/offline tooling. The
  top-level perception package is not a ROS package and should not own
  `package.xml`, `setup.py`, or `srv/`.

## History: why this layout, and what was here before

Before this reorg, `src/perception/` had ~40 loose top-level `.py` files plus
`api/`, `impl/`, `adapters/`, `reconstruction/`, `examples/`, `tests/`. Two
older docs describe two different, non-current shapes:

- `CLAUDE.md` (older) describes perception as living under
  `src/semantic/perception/` alongside a `semantic/planner/` and
  `semantic/common/`. **`src/semantic/` does not exist in this tree** -- it
  was already fully retired to `src/perception/` (perception) and
  `src/decision/` (planner/common) before this reorg started. A repo-wide
  contract test (`tests/contracts/test_source_domain_migration.py`) actively
  asserts `src/semantic/` stays deleted and that no code imports it, so this
  is enforced, not just historical.
- `AGENTS.md` (newer) describes a flat top-level `src/perception/` ("perception,
  tracking, reconstruction, scene understanding"), which matches what was
  actually on disk, but the ~40 files under it were organized by history
  (whatever landed there over time), not by the function names AGENTS.md
  uses to describe it.

So the "docs describe an old structure, the tree already has a different
in-progress structure" pattern flagged for other subsystems (SLAM paths,
transport adapters) also applied here, but one level down: the *package*
name (`perception`) was already correct in both the tree and the newer doc;
what was missing was function-based organization *inside* the package.

Concretely, the pre-reorg tree also contained real duplication, not just
naming drift:

- `impl/clip_encoder.py`, `impl/instance_tracker.py`, `impl/yolo_world_detector.py`
  vs. the (now `encoding/`, `tracking/`, `detection/`) top-level files of the
  same name were **two independent implementations**, not a thin
  wrapper/impl split. `perception_module.py._setup_via_factory()` uses the
  `api/`+`impl/` path only when `detector_type == "yolo_world"`, and the
  `_setup_direct()` path (direct lazy imports of the top-level files) for
  every other backend (`yoloe`, `bpu`, `sim_scene`, `mobileclip`). This means
  the `yolo_world` + `clip` + `instance`-tracker combination has always run
  through a different tracker implementation than every other detector
  backend. This is pre-existing behavior, unchanged by this reorg -- see
  "Open questions."
- `reconstruction/README.md` referenced `bag_reader.py` as if it lived in
  `reconstruction/`; it has actually lived in `adapters/ros2/` for a while.
  Fixed as part of this pass (doc-only correction, no code moved).

## Reorganization mapping (2026-07)

All moves were `git mv` (history-preserving); no algorithm code changed.

| Old path (`src/perception/...`) | New path (`src/perception/...`) |
| --- | --- |
| `detector_base.py` | `detection/detector_base.py` |
| `yoloe_detector.py` | `detection/yoloe_detector.py` |
| `yolo_world_detector.py` | `detection/yolo_world_detector.py` |
| `grounding_dino_detector.py` | `detection/grounding_dino_detector.py` |
| `bpu_detector.py` | `detection/bpu_detector.py` |
| `sim_scene_observer.py` | `detection/sim_scene_observer.py` |
| `detector_module.py` | `detection/detector_module.py` |
| `laplacian_filter.py` | `detection/laplacian_filter.py` |
| `keyframe_selector.py` | removed (unused legacy detection gate) |
| `clip_encoder.py` | `encoding/clip_encoder.py` |
| `mobileclip_encoder.py` | `encoding/mobileclip_encoder.py` |
| `encoder_module.py` | `encoding/encoder_module.py` |
| `instance_tracker.py` | `tracking/instance_tracker.py` |
| `tracked_objects.py` | `tracking/tracked_objects.py` |
| `projection.py` | `tracking/projection.py` |
| `bpu_tracker.py` | `tracking/bpu_tracker.py` |
| `bpu_qp_bridge.py` | replaced by `tracking/bpu_tracker.py`, `tracking/native_byte_tracker.py`, and tracking services |
| `scg_builder.py` | `scene_understanding/scg_builder.py` |
| `scg_path_planner.py` | `scene_understanding/scg_path_planner.py` |
| `global_coverage_mask.py` | `scene_understanding/global_coverage_mask.py` |
| `leiden_segmentation.py` | `scene_understanding/leiden_segmentation.py` |
| `polyhedron_expansion.py` | `scene_understanding/polyhedron_expansion.py` |
| `uncertainty_model.py` | `scene_understanding/uncertainty_model.py` |
| `local_rolling_grid.py` | `scene_understanding/local_rolling_grid.py` |
| `geometry_extractor.py` | `scene_understanding/geometry_extractor.py` |
| `hybrid_planner.py` | `scene_understanding/hybrid_planner.py` |
| `dataset_loader.py` | `research/dataset_loader.py` |
| `baseline_wrappers.py` | `research/baseline_wrappers.py` |
| `evaluation_framework.py` | `research/evaluation_framework.py` |
| `end_to_end_evaluation.py` | `research/end_to_end_evaluation.py` |
| `visualization_tools.py` | `research/visualization_tools.py` |

Unchanged: `perception_module.py`, `service.py`, `__init__.py`, `api/`,
`impl/`, `adapters/`, `reconstruction/`, `examples/`, `tests/` (files inside
`examples/` and `tests/` had their imports updated to the new paths above,
but the directories themselves did not move).

## Open questions (not resolved by this pass)

- **`api/`+`impl/` vs. `detection/`/`encoding/`/`tracking/` duplication**: the
  factory path re-implements `YOLOWorldDetector`, `CLIPEncoder`, and
  `InstanceTracker` from scratch instead of wrapping the "direct path"
  classes. Whether to (a) delete the factory path and always use the direct
  path, (b) make the factory path a thin wrapper around the direct classes,
  or (c) keep both, is a product decision, not a structural one -- left
  untouched here.
- **`scene_understanding/` and `research/` disposition**: this is a
  substantial, self-contained body of work (9 algorithm modules + a 5-module
  benchmark harness + 3 example scripts + ~20 test files) that is not on the
  robot's execution path. Worth an explicit decision: keep as an active
  research track, archive it (e.g. under `docs/archive/` or a top-level
  `research/` outside `src/`), or delete if superseded. Left in place and
  clearly labeled, since deleting/archiving code is a product decision, not
  a reorg detail.
- **`hybrid_planner.py` / `scg_path_planner.py` are planning code living in
  `perception/`**: functionally these are path planners (A* variants), which
  arguably belong under `src/nav/` per the module boundary rules. They were
  kept inside `perception/scene_understanding/` in this pass because (1) they
  are experimental/unwired, (2) they only make sense bound to the SCG
  representation that lives here, and (3) moving code across top-level
  domain packages (`perception/` -> `nav/`) is a bigger, separately-reviewable
  change than an intra-package reorg. Flagged for a follow-up decision.
- **Legacy cleanup completed**: the unused `keyframe_selector.py` gate was
  removed. The former all-in-one `bpu_qp_bridge.py` was replaced by the
  explicit tracker modules and services under `tracking/`; live demos should
  use those current boundaries instead of reviving the retired bridge.

## C++/native rewrite candidates (recommended, not yet implemented)

This repo already has a working pattern for moving hot Python paths to native
code: `src/nav/cpp/` (C++ hot paths exposed through the `src/nav/kernel/` nanobind loader,
additive `@register(...)` backends so the pure-Python path stays as a
fallback -- see `terrain`, `local_planner`, and `path_follower` backends, and
the SoA layout / CSR sparse format / `scorePathFast` LUT / OpenMP scoring
optimizations documented in the root `CLAUDE.md`). The candidates below
follow that same shape. **No profiling or benchmark data for perception
specifically exists in this repo today** (`research/evaluation_framework.py`
and `examples/performance_analysis.py` define the *harness* for measuring
these things but no committed results were found) -- the reasoning below is
structural/algorithmic, not measured, and should be validated with the
existing `examples/performance_analysis.py` (cProfile-based) before any
native work starts.

| Candidate | Where | Why it's a plausible hot path | Native pattern to follow |
| --- | --- | --- | --- |
| Mask/box -> point-cloud projection | `tracking/projection.py`: `mask_to_pointcloud`, `pointcloud_centroid`, voxel downsample | Runs once per detected object per frame; iterates per-pixel over the mask region in NumPy with no SIMD-friendly batching across objects. Direct per-frame cost driver for the main `PerceptionModule` pipeline (this is the one candidate on the live robot path, not just research code). | `nav_kernel`-style nanobind extension; additive backend behind the existing duck-typed detector/tracker interface so pure-Python stays the fallback when the native module isn't built (matches how `nav.kernel.loader` picks native vs. Python). |
| Cross-frame association (IoU + IDs) | `tracking/instance_tracker.py: InstanceTracker.update()`, `tracking/bpu_tracker.py` | Classic O(n*m) association loop (new detections x existing tracks); this shape is exactly what `local_planner.hpp`'s CSR/SoA rewrite already targeted for planning, and is a well-known hot path in every tracking-by-detection system (this is why BoT-SORT/DeepSORT implementations in C/Cython exist upstream). | Same nanobind pattern; keep `TrackedObject`/`Detection3D` as the Python-facing dataclasses, move only the matching/cost-matrix inner loop native. |
| NMS / box decoding | Inside `detection/*_detector.py` wrappers around `ultralytics`/BPU SDK output | Currently delegated to `ultralytics` (already C/CUDA) or the BPU SDK -- **not actually a Python hot path today**, since neither `yolo_world_detector.py` nor `bpu_detector.py` hand-roll NMS. Listed because it's the textbook hot-path candidate the task asked to check for, but the evidence here says it's already native via the upstream SDKs; deprioritize versus the two rows above. | N/A unless a hand-rolled NMS path is added later. |
| Polyhedron expansion / SCG build / path search | `scene_understanding/polyhedron_expansion.py`, `scg_builder.py`, `scg_path_planner.py`, `hybrid_planner.py` | Heaviest pure-Python geometry in the package (convex hull expansion, KD-tree queries, Dijkstra/A* over graphs, all in `scipy`/`numpy`/`heapq`). Would be the biggest theoretical win by line-count and algorithmic weight. **Deprioritized below the two production rows above** because none of this is on the robot's execution path today (see "Open questions") -- optimizing unused code is premature until the product decision on `scene_understanding/`'s disposition is made. | Same nanobind pattern; only worth doing if/when this cluster is promoted to production use. |
| Embedding post-processing (cache hashing, normalization, batch stacking) | `encoding/clip_encoder.py`, `encoding/mobileclip_encoder.py` | Some batch tensor prep in Python, but the actual encode is already a `torch`/CoreML forward pass (GPU/NPU-bound, not Python-bound). Low priority -- the pre/post-processing overhead is small relative to model inference time. | N/A; not a good candidate. |

Priority if this work is picked up: **`tracking/projection.py`'s point-cloud
building** and **`tracking/instance_tracker.py`'s association loop** first,
since they are the only two candidates that run on every frame of the live
robot pipeline. The `scene_understanding/` geometry code is a larger native
rewrite opportunity in principle but should wait on the product decision
about whether that cluster ships at all.

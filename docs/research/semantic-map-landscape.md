# Semantic map methods and LingTu's next layer

**Status:** Engineering decision record, 2026-10-03

The current repository contains a real geometry fixture,
`sim/evaluation/data/octoplanner3d/building2_9.pcd`, and a runtime `SceneGraph`.
They are useful inputs to the same semantic-map product, but they are not yet
one fused object-level map. A point cloud alone supports geometric semantics
such as dominant surface levels and observed structure. Names such as `desk`,
`box`, and `meeting_room` require appearance or a separately supervised model.

## Closest open work

| Work | Input and representation | What LingTu should take from it |
| --- | --- | --- |
| [Point2Graph](https://point2graph.github.io/) | Point cloud only; room and object layers with open-vocabulary classification | Best direct reference when posed RGB-D is unavailable. It explicitly separates room and object processing and evaluates on Matterport3D and ScanNetv2. |
| [Open-Vocabulary Octree-Graph](https://openaccess.thecvf.com/content/ICCV2025/html/Wang_Open-Vocabulary_Octree-Graph_for_3D_Scene_Understanding_ICCV2025_paper.html) | RGB-D/3D instances, adaptive octrees, semantic graph edges | Keep occupancy geometry and semantic relations in one queryable representation without attaching a feature vector to every raw point. |
| [Terra](https://github.com/BYU-FROST-Lab/Terra) | LiDAR/metric map, camera observations, terrain-aware hierarchical graph | Closest to the persistent, terrain-aware, task-oriented outdoor/robot map boundary. Its public pipeline records global point clouds, camera images, LiDAR frames, and LiDAR-to-map/camera transforms. |
| [HOV-SG](https://github.com/hovsg/HOV-SG) | Hierarchical open-vocabulary scene graph | Use floor → room → object hierarchy and compact semantic retrieval as the reference for indoor language queries. |
| [ConceptGraphs](https://concept-graphs.github.io/) | Posed RGB-D sequence with multi-view association | Reference for object-centric open-vocabulary 3D instances and semantic relations when RGB-D and poses are available. |
| [GraphEQA](https://proceedings.mlr.press/v305/saxena25a.html) | Online 3D metric-semantic graph plus task-relevant images | Reference for online updates, multimodal memory, and semantic-guided exploration rather than only static post-processing. |

## What is implemented now

`tools/bench/semantic_pcd_scene.py` creates a **geometry-semantic scene** from
the official PCD. It detects dominant height bands, groups vertical geometry
into structure candidates, splits each height band into connected support
regions, links the resulting `surface_level` nodes by height order, and keeps
all observed points as collision geometry. It deliberately does not create
object names from shape alone.

```powershell
.venv/Scripts/python.exe tools/bench/semantic_pcd_scene.py `
  sim/evaluation/data/octoplanner3d/building2_9.pcd `
  --output-dir build/semantic-building2-9
```

The output contains `scene.json` and `scene.png`. In the current fixture the
result is a `site` with two structure groups, eight support-region candidates,
four dominant surface levels, 21 evidence-backed relations, and three
navigation layers (`support_candidate`, `collision_geometry`, and
`unknown_space`). This is a planning-support semantic layer, not an
activation-ready saved map and not a replacement for the native
occupancy/collision artifact.

The structure grouping is a deterministic geometric heuristic: a 0.25 m XY
grid, observed height span above 0.8 m, and connected components. A support
region is a connected component inside a dominant height band. Those labels
are useful evidence for inspection and candidate generation; they are not
claims that a group is a room, wall, desk, or box. The current `building_id`
on a support region records the nearest structure-group center so the
heuristic is explicit and replaceable when RGB-D association is added.

## Recommended LingTu pipeline

```text
LiDAR + IMU + native SLAM
  -> map-frame point cloud and poses
RGB/RGB-D + camera poses + LiDAR-camera extrinsics
  -> 2D masks / open-vocabulary detections
  -> project masks into map-frame point clusters
  -> associate clusters across frames
  -> object records with confidence and observation history
  -> room/group/object/relation SceneGraph
  -> semantic query and approach-pose resolver
  -> native 3D preview and navigation
```

The geometric navigation map remains authoritative for support, collision,
unknown space, and path execution. The semantic graph supplies names, context,
relations, and task-relevant candidates. This split follows the practical
lesson of the newer octree-graph and terrain-aware systems: semantics must be
queryable and persistent, while motion still uses explicit executable geometry.

## Decisions

1. Use the PCD-only tool as a reproducible geometry-semantic baseline.
2. Do not claim `desk`/`box` labels for a geometry-only PCD.
3. Add the RGB-LiDAR projection and object association contract before adding
   an LLM or Gaussian-splat representation to the motion path.
4. Compare against Point2Graph for point-cloud-only input and against
   ConceptGraphs/HOV-SG/Terra for multimodal or hierarchical semantic mapping.
5. Report object/room/relation accuracy, map-size and update latency, query
   accuracy, and navigation success separately; a semantic query pass does not
   prove global planning or field motion.

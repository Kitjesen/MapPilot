# Semantic dataset progress log

This log records executed dataset work. A line is considered complete only
when the input, command, output, and evidence level are all present.

## 2026-10-08 — object-image CLIP features in moving Habitat search

The continuous Habitat vision bridge now encodes accepted RGB object crops with
the existing registry-provided OpenCLIP ViT-B/32 encoder. It sends 512-dimensional
unit image features into the existing geometry/semantic tracker and preserves
the existing quality-weighted EMA. These are image features, separate from the
YOLO-World text prompt encoding. No new project dependency or lock change was needed.

Real reruns on official ReplicaCAD `apt_0` produced 44 encoded observations for
chair seed 7 and 31 for seed 8. Both moved, approached/re-observed candidates,
saved and reloaded features with matching dimensions/values, and resumed existing
IDs in later real observations. Paths and object counts were unchanged from the
earlier geometry-based episodes; this is wiring evidence, not an accuracy gain.
The water-bottle negative case produced no detections or features in 35 observations.

Outputs: `build/vlnav/episode-chair-clip7/`, `episode-chair-clip8/` and
`episode-bottle-clip7/`. Each has summary, full per-detection feature vectors,
tracker state, geometry evidence and H.264 video. Seed 7 also has a real crop
comparison: candidate ID 2, frames 10/27, image-feature cosine **0.869899**.
Local related tests: **75 passed**; touched Python files passed Ruff.
The [VL-Nav log](vlnav.md#2026-10-08--补齐对象-clip-图像特征) records weight-format
preparation, commands, failures and result links. Habitat NavMesh and simulator
poses remain experiment inputs; Qwen3-VL, native planning, SLAM, quadruped dynamics,
instance ground-truth accuracy and finding a water bottle remain unproved.

## 2026-10-08 — model-driven Habitat search with movement

Habitat-Sim 0.3.3 loaded official ReplicaCAD `apt_0`. Each action was followed
by a new RGB-D observation, YOLO-World detection, FastSAM segmentation,
world-frame projection and the existing LingTu instance tracker. The search
used LingTu's VL-Nav-inspired candidate scorer; Habitat's official NavMesh
provided reachable paths, and simulator poses provided the transforms.

Two `chair` episodes reached and re-observed chair candidates: seed 7 produced
28 observations over 3.60 m, and seed 8 produced 15 observations over 1.80 m.
Both saved and reloaded the tracker during movement, then matched checkpoint
IDs in actual later observations. A `water bottle` negative case produced no
detections over 35 observations and 2.40 m, then stopped at its budget.

Inputs, exact commands, projection/association fixes and evidence links are
recorded in [the VL-Nav reproduction log](vlnav.md#2026-10-08--habitat-连续移动找物实验).
Local outputs are `build/vlnav/episode-chair-live7/`,
`build/vlnav/episode-chair-live8/` and `build/vlnav/episode-bottle-live7/`.
Each contains `summary.json`, `steps.json`, tracker state, point clouds and
H.264 videos. Related local tests: **57 passed**; touched Python files passed Ruff.

This proves a moving Habitat camera-agent search with model observations and
object persistence. It does not prove native 3D navigation, quadruped dynamics,
SLAM, Qwen3-VL verification, moving-object tracking or paper benchmark success
rates. Dataset semantic labels were not supplied to the search strategy.

## 2026-10-04 — 3DSSG scene-graph adapter

**Dataset and modalities:** Official 3DSSG annotation archive; indoor object
labels, attributes, affordances, and relation triplets. The archive does not
include the 3RScan reconstructed geometry.

**Input:**

```text
build/datasets/3dssg/3DSSG/objects.json
build/datasets/3dssg/3DSSG/relationships.json
```

**Command:**

```powershell
.venv\Scripts\python.exe tools\datasets\3dssg_probe.py `
  build\datasets\3dssg\3DSSG `
  --output build\datasets\3dssg\lingtu_scene.json
```

**Output:**

```text
build/datasets/3dssg/lingtu_scene.json
scan = 0cac7676-8d6f-2d13-8f3a-d7bf7f03e721
nodes = 43
relations = 421
relation_example = table(27) close by box(39)
skipped_relations = 0
```

**Evidence level:** D3 structure parsing. The result proves that external
semantic nodes and relations can enter the LingTu graph schema. It does not
prove 3D point binding, localization, or navigation.

The follow-up query smoke test exposed and then fixed a real resolver gap. The
exact graph constraint returns box `39` for “找到桌子旁边的箱子”. The fast
resolver previously returned same-label box `32` because its `near` score was
generic; it now requires the target/reference relation edge and returns box
`39` with `spatial=1.0`. The graph still has no geometry, so the report keeps
`motion_authorization: false` and proves semantic selection only.

**Verification:**

```text
.venv\Scripts\python.exe -m pytest -q \
  tests\tools\test_3dssg_probe.py \
  tests\tools\test_semantic_dataset_inventory.py \
  tests\tools\test_tum_rgbd_probe.py
4 passed
```

The relation query was re-run with:

```text
.venv\Scripts\python.exe tools\datasets\3dssg_goal_query.py \
  build\datasets\3dssg\lingtu_scene.json \
  --output build\datasets\3dssg\query_result.json
```

It reports `relation_grounding.candidate_ids = [object:...:39]` and keeps the
fast-resolver candidate separately for comparison.

## 2026-10-04 — text navigation simulation evidence

The task text `先去会议室，然后找到桌子旁边的箱子` was parsed by
`TaskDecomposer.parse_decomposition_response` into two LingTu subgoals. The
plan is rendered over a MuJoCo recording produced by the existing ThunderV4
ONNX locomotion policy and MID-360 raycast backend: 134 frames, 15 FPS, 8.93 s,
and 0.695 m simulated XY motion. The policy clip is driven by a fixed velocity
command; the LingTu text plan is parsed and recorded alongside it. This is D2
simulation evidence for text decomposition plus physical policy motion. It is not yet D4 semantic-map
navigation: the plan has `semantic_binding = demo_plan_only` and
`motion_authorization = false` until the relation candidate has geometry and a
native planner preview accepts it.

Artifacts:

```text
artifacts/text-navigation-demo-20261004/text_navigation_browser.mp4
artifacts/text-navigation-demo-20261004/text_navigation_frame.png
artifacts/text-navigation-demo-20261004/text_task_plan.json
artifacts/text-navigation-demo-20261004/report.json
```

The native text-to-motion gate is still open. The available native acceptance
attempt did not produce valid motion evidence: the existing `navd` binary
rejected `--octo-ground-support`, and the sensor/driver bridge never reached
transport readiness, so no LiDAR/odom/IMU stream reached native navigation.
That run is not presented as a successful video. The next native run must use
the rebuilt `navd` and sensor/driver bridge from the same build, then prove
symbolic target → map pose → native goal admission → reached goal.

## 2026-10-04 — server geometry and visual layers

**Dataset and modalities:** `building2_9.pcd` geometry on the remote RTX 5090;
the earlier uploaded RGB scene and CLIP image-level hypotheses are kept as a
separate visual layer.

**Remote outputs:**

```text
/root/autodl-tmp/lingtu-semantic-vlm/geometry-semantic/scene.json
/root/autodl-tmp/lingtu-semantic-vlm/geometry-semantic/scene.png
/root/autodl-tmp/lingtu-semantic-vlm/semantic_map_building2_9.json
```

The geometry run processed 853,227 points, two structure groups, eight support
regions, and four dominant surface levels. The visual layer was produced by
CLIP ViT-B/32 on CUDA. The two layers remain unassociated because no shared
camera pose, timestamp alignment, or LiDAR-camera calibration was supplied.

**Evidence level:** D1 geometry plus a visual hypothesis layer. Visual labels
are not navigation-authorized.

## 2026-10-04 — official ReplicaCAD geometry and semantic binding

**Dataset and modalities:** Official AI Habitat ReplicaCAD `apt_0` scene:
scene-instance transforms, semantic lexicon, 80 unique object configs/render
GLBs, stage GLB, and navmesh. The scene declares 113 instances and all 113 are
geometry-bound in the local cache (about 65 MB); missing assets = 0.

**Official source and license:**
[ai-habitat/ReplicaCAD_dataset](https://huggingface.co/datasets/ai-habitat/ReplicaCAD_dataset).
The HF README metadata says CC BY 4.0, while the downloaded repository
`LICENSE.txt` says CC BY-NC 4.0; the discrepancy must be resolved with the
upstream terms before product redistribution.

**Input:**

```text
build/datasets/replicacad/configs/scenes/apt_0.scene_instance.json
build/datasets/replicacad/configs/ssd/replicaCAD_semantic_lexicon.json
build/datasets/replicacad/configs/objects/*.object_config.json
build/datasets/replicacad/objects/*.glb
build/datasets/replicacad/navmeshes/apt_0.navmesh
```

**Command:**

```powershell
.venv\Scripts\python.exe tools\datasets\replicacad_probe.py `
  build\datasets\replicacad `
  --scene apt_0 `
  --output build\datasets\replicacad\lingtu_replicacad.json

.venv\Scripts\python.exe tools\datasets\3dssg_goal_query.py `
  build\datasets\replicacad\lingtu_replicacad.json `
  --output build\datasets\replicacad\query_result.json
```

**Output:** `113` geometry-bound nodes, including the scene's `table` and
`box` instances, one geometry-derived `near` edge, and a map-frame candidate for the text query
“找到桌子旁边的箱子”. The resolver returns
`object:apt_0:108`, which is the relation-grounded box, with `confidence=1.0`,
`frame_id=map`, and `geometry_status=bound`. The edge connects it to table
`object:apt_0:19` at `1.5284377412 m`. ReplicaCAD is Y-up; the adapter writes
LingTu's Z-up map frame using `[x,y,z] -> [x,z,y]`. The official navmesh is
present, but its coordinate frame still needs to be checked before native
planner use. A top-down evidence image is saved at
`build/datasets/replicacad/lingtu_replicacad_topdown.png`.

The query adapter now emits `approach_goal_candidates`: eight standoff poses
around the bound object center, each facing the object. These are observation
hypotheses for native read-only preview (`requires_native_preview=true`); the
object center itself is never dispatched, and `motion_authorization=false`
remains explicit. On the 3DSSG annotation-only graph the same field is empty
with `status=geometry_unavailable`, so the adapter does not invent coordinates.

**Evidence level:** D3 geometry-bound semantic query. This is the first
official public indoor asset in the project that proves semantic instance IDs
and geometry can reach the same map-frame candidate. The license text
discrepancy is recorded above; this artifact is for research
validation and does not grant commercial redistribution. It still sets
`motion_authorization=false`; no native planner or robot motion is claimed.

The adapter now also emits a `graph_level=hierarchical` scene record with one
known scene/floor and semantic-label groups. It deliberately leaves
`room_segmentation=unavailable`: ReplicaCAD's `apt_0` scene file does not
provide room labels, so this is a structural floor → group → object graph, not
the completed floor → room → object graph used by HOV-SG.

The query adapter now follows the runtime's observation-goal boundary: a bound
object produces eight standoff candidates around its center, each marked
`requires_native_preview=true`; an unbound object produces none. With the
ReplicaCAD target and the recorded robot pose, the query produced eight
0.8-m candidates. This is the OVI-MAP/HOV-SG-style separation between semantic
candidate selection and navigation admission; it does not authorize any of the
poses.

The dataset adapter also emits `instance_records` with `instance_id`,
`geometry_ref`, `map_identity`, observation references, and semantic-evidence
references. `replicacad_persistence_check.py` saved and reloaded all 113
records with stable IDs and geometry references. This is a dataset-artifact
reload check only; it is not yet native `mapd` save/reload evidence.

The complete run ledger is `build/datasets/replicacad/run.json`; it records the
paper references, evidence level for each stage, native planner diagnostic, and
the fact that motion was not authorized.

The new `tools/bench/semantic_goal_preview.py` sends all 8 candidates through
the native headless planner and, when supplied, the independent path verifier.
The first run exposed two conversion problems rather than a semantic-query
problem. The stage GLB root transform was being dropped, and the large floor
faces were represented only by sparse vertices. The adapter now composes the
complete scene-node TRS/matrix chain and supports `--floor-spacing-m 0.2` to
rasterize floor triangles before PCD conversion. The corrected native-frame
PCD bounds are approximately `x=[-2.68,4.58]`, `y=[-4.83,8.16]`,
`z=[-0.09,3.04]`.

On the vertex-only map, the final preview remains `target_unreachable` with
`0/8` candidates. On the floor-triangle map, the discrete planner admits
`1/8` candidates and returns a 47-point path, but the same path fails the
independent continuous swept-route verifier. The final navigation admission is
therefore still `0/8`, with status `verifier_blocked` and
`motion_authorization=false`. The preview now
records `planner_admitted_count`, `verification_passed_count`, and each
verification failure instead of turning a discrete `reached_goal` into a motion
claim.

The verifier also accepts `--planning-anchor x y z`. This is required when a
planner snaps the requested start to a different first waypoint; reconstructing
the lattice offset from the snapped waypoint can move a boundary query into a
different voxel. With the original request anchor, the floor-sampled path
passes point support checks but still fails `segment_sweep_failed`, which is a
real planner/verifier geometry-contract gap. The independent gate remains
closed until that continuous sweep is handled by the native planning contract.

**Native map conversion probe:** The corrected adapter exports both the original
vertex-only PCD and a floor-triangle PCD at
`build/datasets/replicacad/apt_0_lingtu_floor.pcd` (347,887 points, Y-up to
Z-up, floor spacing 0.2 m). The latter is converted to
`build/datasets/replicacad/apt_0_floor.ot`. Passing the semantic object's center
directly to the planner correctly returns `goal_snap_exhausted` because the
object center is occupied. The floor-sampled map gives one discrete planner
admission out of eight candidates and a 47-waypoint path, but the independent
verifier rejects its first continuous sweep segment. The verifier now receives
the original request start through `--planning-anchor`; without that anchor it
can query a different voxel after the planner snaps the first waypoint. These
results are retained in `build/datasets/replicacad/semantic_goal_preview_floor.json`
and `replicacad_navigation_report.json`, not as accepted navigation evidence.

## 2026-10-05 — OVI-MAP text query slice

The first concrete open-vocabulary request is now wired through the offline
semantic query boundary:

```text
帮我找一瓶水
  -> ObjectQuery(label=water_bottle, quantity=1)
  -> persisted semantic instance/evidence match
  -> map-frame geometry binding
  -> eight standoff hypotheses
  -> native planner preview (read-only)
```

The implementation is shared by `src/decision/semantic_navigation/query.py`,
`tools/datasets/3dssg_goal_query.py`,
`tools/bench/water_bottle_slice.py`, and the live
`SemanticPlannerModule` resolver normalization. A label alias never creates a
pose: the instance must carry matching semantic evidence and bound geometry.

The official ReplicaCAD `apt_0` graph was run with:

```powershell
$env:PYTHONPATH = "src"
python tools/bench/water_bottle_slice.py `
  build/datasets/replicacad/lingtu_replicacad.json `
  --output build/datasets/replicacad/water_bottle_slice.json
```

It returned `target_unavailable` because the official scene has no water
bottle label or water-bottle evidence. That is the honest dataset result; no
planner call was made and `motion_authorization=false` remained in the report.

A separate contract fixture with explicit `semantic_evidence=["water bottle"]`
produced eight map-frame approach candidates. On the floor-sampled ReplicaCAD
planning map, the native planner admitted one discrete route, while the
continuous swept-route verifier rejected it, so the combined status is
`verifier_blocked` with `motion_authorization=false`. The fixture is marked
synthetic and proves only the query-to-planner wiring, not visual recognition
or robot motion.

The same run now has a visual trace at
`artifacts/water-bottle-search-demo-20261005/water_bottle_search_diagnostic.mp4`.
The video shows the command, the bound synthetic instance, eight approach
hypotheses, the native preview cursor, and the `verifier_blocked` gate. Its
report explicitly records `robot_motion_executed=false`; it is a diagnostic
simulation visualization rather than field evidence.

## Next gate

The geometry-binding gate is now satisfied on the ReplicaCAD subset. The next
accepted run must report:

1. instance IDs mapped to mesh vertices or point clusters;
2. a map-frame geometry reference for each object;
3. at least one relation such as `box near table` recomputed from geometry;
4. a reload check that returns the same object candidate;
5. a native `preview_plan` result with a concrete acceptance or rejection reason;
6. a route that also passes the independent path verifier under the same map
   profile.

Until these six items exist, the external dataset work remains a semantic
index test and cannot be described as end-to-end semantic navigation.

## 2026-10-09 — General observed objects, image features, and identity evidence

Two new server Habitat-Sim 0.3.3 runs use official ReplicaCAD `apt_0` RGB-D
with YOLO-World, FastSAM-s, and CLIP ViT-B/32. The task target is now observed
alongside a configurable context vocabulary, including table, cabinet, bottle,
cup, and chair. All valid observations receive actual 512-D crop image features;
context objects cannot satisfy the task target. Neither official classes nor
instance identities are used to select, associate, or navigate to objects.

| Run | Steps / actions | Feature observations | Candidate objects / predicted classes | Multiple stored views | Sparse near edges |
| --- | ---: | ---: | ---: | ---: | ---: |
| general-chair7 | 28 / 27 | 69 | 9 / 6 | 5 | 9 |
| general-cup8 | 31 / 30 | 303 | 35 / 8 | 21 | 46 |

Both runs preserve object geometry and features across save/reload. Raw camera
poses are explicitly Habitat Y-up; projected objects are Habitat Z-up.
Independent renderer reference errors are below 7.5e-7 m. These are exact
simulator-pose checks, not real SLAM, synchronization, or calibration evidence.
The second run records four Habitat collision microsteps. Motion remains a
NavMesh camera-agent experiment, without LingTu's native planner or quadruped
dynamics.

`habitat_scene.py` exports full fused features, real view references, and small
geometry-backed near relations. At most three distinct views are stored per
object. Descriptions summarize labels and view evidence; `vlm_caption=false`.
No image-description VLM ran. Relations select each object's two nearest
existing near edges and retain the union. The second run's original 219-edge
graph is retained alongside the 46-edge export. Exporting from saved state did
not rerun perception or motion.

Independent identity masks are read only after tracker updates. A predicted
mask is attributable when one assigned rigid-object ID covers at least 60% of
its pixels. Stage/background pixels do not have equivalent instance coverage.
The resulting attribution coverage is not an association success rate:

| Run | Attributed observations | Within-track oracle changes | Mixed tracks | Split oracle objects |
| --- | ---: | ---: | ---: | ---: |
| general-chair7 | 67 / 69 | 0 | 0 | 1 |
| general-cup8 | 103 / 303 | 4 | 3 | 5 |

The monitor splits into screen/lower-support IDs. The second run also mixes a
chair and beanbag in one track and switches between adjacent kitchen utensils.
Thus feature wiring is complete, but persistent instance identity is not
reliable yet.

A concrete semantic false positive is preserved: cup8 track 22 was labelled
`bottle` once at detector score 0.256675. Its mask maps to official handle
`frl_apartment_handbag_:0000`, and the crop shows the handbag handle. The
original dense graph nevertheless bound the water-bottle/table query with a
fast-resolver ranking score of 0.86285. That is a failed semantic result, not
successful water search. Track 17 consistently maps to one kitchen utensil;
its fine-grained cup subtype remains unverified. The installed dataset has no
bottle-class asset. Sparse export removing the false relation does not repair
the false label.

All query reports now declare `grounding_scope=detector_candidate_graph`,
`semantic_verification=not_performed`, and `motion_authorization=false`.
Missing retained edges mean insufficient graph evidence, not physical absence.

The detailed run commands, current code, actual map/crops, videos, and retained
failure evidence are indexed in [VL-Nav reproduction](vlnav.md).
Local focused tests: 57 passed; touched Python files pass Ruff. Four downloaded
H.264 videos decode fully with expected frames. Project dependencies and locks
are unchanged. The next gate is improved instance consistency and false-target
verification before richer relations or VLM descriptions.

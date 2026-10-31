# Semantic navigation implementation alignment

This note records how three recent open-vocabulary mapping papers map onto
LingTu's existing boundaries. It is an implementation guide and an evidence
ledger; it is not a claim that LingTu currently reproduces any of the papers.

The date context is 2026-10-04. The current reference set is:

| Paper | Publication | Why it is relevant | Official source |
| --- | --- | --- | --- |
| **OVI-MAP: Open-Vocabulary Instance-Semantic Mapping** | CVPR 2026 | Separates geometric instance reconstruction from sparse VLM semantic labeling | [CVF paper](https://openaccess.thecvf.com/content/CVPR2026/html/Deng_OVI-MAP_Open-Vocabulary_Instance-Semantic_Mapping_CVPR_2026_paper.html), [project page](https://ovi-map.github.io/), [code](https://github.com/OVI-MAP/OVI-MAP) |
| **One Map to Find Them All: Real-time Open-Vocabulary Mapping for Zero-shot Multi-Object Navigation** | ICRA 2025 | Reuses a probabilistic semantic map across sequential object-search tasks and reports Spot hardware results | [project page and videos](https://www.finnbusch.com/OneMap/), [ICRA record](https://ieeexplore.ieee.org/document/11128393/) |
| **Hierarchical Open-Vocabulary 3D Scene Graphs for Language-Grounded Robot Navigation** | RSS 2024 | Demonstrates floor → room → object decomposition and language-conditioned navigation on Spot and Habitat | [project page and videos](https://hovsg.github.io/), [RSS paper](https://www.roboticsproceedings.org/rss20/p077.html) |

## Mechanism-to-module mapping

| Paper mechanism | LingTu location today | Current status | Required LingTu evidence |
| --- | --- | --- | --- |
| RGB-D/pose or LiDAR observations become map-frame geometry | Native map and SLAM owners under `src/maps/`, `mapd`, and the native navigation stack | **Implemented for geometry**, with point-cloud conversion and native 3D planner checks. The semantic PCD tool is a planning-support view, not a replacement for `mapd`. | A recorded sensor sequence produces a map, and the saved map can be reloaded with the same map identity and 3D collision/planning result. |
| Class-agnostic 3D instance map | `src/maps/cpp/semantic/` contains semantic occupancy, taxonomy, persistence, and Unity import; Python semantic state is exposed by `src/memory/modules/semantic.py` | **Partial.** The current layers can carry semantic occupancy/taxonomy and room/object knowledge, but a persistent RGB-D/LiDAR instance map with stable `instance_id` and `geometry_ref` is not demonstrated end to end. | Two or more views of the same object produce one stable instance, with a map-frame point set or volume, observations, timestamps, and reload-stable identity. |
| Sparse VLM evidence and open-vocabulary query | `src/memory/storage/temporal_store.py` supports embedding lookup; `src/memory/modules/vector.py` provides candidate retrieval; the server-side CLIP probe is an external evidence run | **Partial.** CLIP retrieval has been exercised on an image, but there is no proven map-bound VLM worker that associates selected views with persistent 3D instances. | A query such as `box` returns a scored instance whose evidence references views and whose geometry is in the active map frame. |
| Object/room/floor semantic hierarchy | `src/memory/knowledge/room_object_kg.py`, `semantic_prior.py`, `src/memory/spatial/`, and `src/memory/modules/semantic.py` | **Partial.** Room/object relations and semantic topology exist as host-side knowledge. They are not yet proven to be generated from a geometry-bound, reloadable scene graph. | A dataset or recorded run builds room and object nodes from geometry, preserves relations, and returns the same hierarchy after save/reload. |
| Natural-language task decomposition | `src/decision/modules/semantic_planner.py`; `src/decision/modules/llm.py`; `tools/datasets/3dssg_goal_query.py` | **Implemented as a symbolic/API path**, with candidate goal resolution. It is not evidence of visual grounding by itself. | `go to the box near the table` must resolve to a geometry-backed candidate, report ambiguity when needed, and never authorize motion from a semantic-only node. |
| Goal dispatch and native 3D planning | `src/decision/modules/semantic_planner.py` dispatches navigation goals; native OctoPlanner3D and `tools/bench/official_global_planner.py` own 3D path validation | **Implemented as separate layers.** A semantic candidate is only a suggestion until the native planner accepts its map-frame goal. | The evidence bundle contains the semantic query, selected `geometry_ref`, planner input, planner result, and final termination state. |
| Reusable map for sequential object search | `src/memory/` stores temporal/vector/episodic context; native map persistence is owned by `mapd` | **Missing as a demonstrated product flow.** Existing memory results are candidates and do not authorize motion. | Search for object A, then B and C without rebuilding the map; report map identity, query latency, candidate confidence, and whether the target was physically reached. |

## What to borrow from each paper

### OVI-MAP (CVPR 2026)

The paper and project page describe a streaming RGB-D plus camera-pose input,
an incrementally reconstructed volumetric scene, a class-agnostic instance map,
and semantic features aggregated only from automatically selected views. The
important design choice for LingTu is the ownership split:

```text
sensor/pose -> geometry instance map -> selected-view VLM evidence -> query index
```

The first LingTu contract should therefore be a map-bound record, not a free
floating label:

```text
InstanceRecord {
  instance_id,
  geometry_ref,
  map_identity,
  observation_refs,
  semantic_evidence_refs,
  confidence,
  lifecycle
}
```

The native map owner should produce and persist `geometry_ref`. The Python
semantic layer may add `semantic_evidence_refs` and answer text queries, but it
must not replace native collision or traversability queries. The OVI-MAP paper
does not justify storing one VLM feature at every point; selected-view evidence
is the more appropriate first target for LingTu's compute and memory budget.

The first implemented text slice follows this boundary. `parse_object_query`
maps `帮我找一瓶水`, `找水瓶`, and `find a bottle of water` to the symbolic
label `water_bottle` and its aliases. `3dssg_goal_query.py` then requires a
matching persisted label or semantic-evidence alias, requires bound geometry,
and emits eight standoff hypotheses for native read-only preview. The
`SemanticPlannerModule` uses the same canonical label before calling its live
scene resolver, so the offline and runtime paths do not normalize the phrase
differently. No pose is created when the map has no matching instance.

The official ReplicaCAD `apt_0` run currently returns
`target_unavailable`: it contains cups and other objects but no water-bottle
instance. A labelled contract fixture reaches the next boundary and produces
the eight geometry-backed candidates; native preview on the floor-sampled map
currently ends `verifier_blocked` (one discrete planner admission, zero
continuous verifier admissions). This is the correct evidence boundary before
adding an RGB/VLM observation worker or field motion.

### OneMap (ICRA 2025)

OneMap keeps a reusable open-vocabulary feature map across multiple object
searches and updates semantic belief probabilistically. The useful LingTu
adaptation is to make uncertainty explicit in the query result:

```text
SemanticCandidate {
  instance_id,
  text_score,
  observation_count,
  spatial_consistency,
  map_identity,
  geometry_ref,
  navigation_admission: pending | accepted | rejected
}
```

The `navigation_admission` field is a product boundary, not a VLM score. A
candidate becomes executable only after map identity, localization state,
collision geometry, and native 3D planning all pass. OneMap's Spot videos are
useful evidence for the desired demonstration format, but they are not LingTu
evidence.

### HOV-SG (RSS 2024)

HOV-SG uses a hierarchy of floor, room, and object concepts, decomposes a long
instruction into queries at those levels, and then uses a navigation graph to
reach the selected node. This matches the intended LingTu instruction:

```text
先去会议室，然后找到桌子旁边的箱子
```

The first resolvable form should be:

```text
place(room=会议室)
relation(near, subject=箱子, reference=桌子)
target(instance_id, geometry_ref)
native_preview_plan(start, target.geometry_ref)
```

Do not treat an LLM-generated relation as a measured relation. `near`, `inside`,
and `standing_on` must come from geometry or a clearly labelled dataset
annotation. HOV-SG's project page contains public videos of text-conditioned
Spot navigation and Habitat cross-floor simulation; those videos are a
reference for presentation and acceptance design only.

## Current gaps that block a paper-level claim

1. **No complete sensor-to-instance association.** We need synchronized RGB,
   depth or LiDAR, camera/LiDAR calibration, timestamps, and poses in one
   replayable recording. A single RGB image plus an unrelated PCD can provide
   visual and geometric evidence separately, but cannot establish an object-to-
   point association.
2. **No proven persistent instance artifact.** `object_memory` remains a
   design target until the same instance survives multi-frame fusion, save,
   reload, and map identity changes.
3. **No end-to-end geometry-backed text navigation evidence.** The current
   symbolic 3DSSG query path explicitly produces semantic candidates when
   geometry is absent. It must not be presented as robot navigation.
4. **No LingTu video proving physical execution from text.** Existing local
   videos are simulation/evidence artifacts, and the public HOV-SG/OneMap videos
   belong to their authors. A LingTu demonstration must include the command,
   resolved candidate, native planner result, robot trajectory, and safe
   termination in one recorded run.

## Acceptance sequence

Use this order so failures remain attributable:

1. **Dataset geometry:** Replica/3RScan or an equivalent RGB-D sequence yields
   point or mesh geometry plus poses and object/room labels.
2. **Instance persistence:** build `InstanceRecord` values, save them with the
   native map identity, reload them, and compare query results.
3. **Text grounding:** decompose a long instruction into room/object/relation
   predicates, then resolve only geometry-backed candidates.
4. **Planner admission:** pass the candidate's map-frame goal to the native
   3D planner and record accepted/rejected reasons.
5. **Motion evidence:** run in simulation first, then supervised field motion;
   preserve the distinction between local contract tests, simulation evidence,
   and real-robot evidence.

The current repository has evidence for parts of steps 1 and 3, and separate
geometry/planner evidence. It does not yet have steps 2–5 as one semantic
navigation product flow. Until those artifacts exist, LingTu must not be
described as reproducing OVI-MAP, OneMap, or HOV-SG, nor as having reached their
reported accuracy or real-robot success rates.

## Evidence record format

Every future semantic-navigation run should save one directory containing:

```text
run.json              # dataset, model, map identity, command, outcome
observations.json     # frame/pose/calibration references
instances.json        # geometry-backed InstanceRecord values
query.json            # original text and decomposed predicates
planner_request.json  # native map-frame goal and planner options
planner_result.json   # accepted/rejected reason and path summary
trajectory.mp4        # simulation or field video, labelled accordingly
```

The report must state whether the semantic evidence is geometry-bound, whether
the native planner ran, and whether motion occurred. A VLM label alone is not a
navigation result.

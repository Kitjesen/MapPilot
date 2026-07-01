# Scene Graph: What Goes In, How Deep, How We Compare to FSR-VLN

## 1. What Lives in the Scene Graph (the "Memory")

**One-line rule:** anything detected by the open-vocab detector and either
matched into an existing tracked object or registered as a new one ends up in
the scene graph until it is pruned.

### 1.1 Entry Conditions

Source: per-frame 3D detection list (`label`, position, score, optional CLIP
feature).

For each detection, the tracker (`src/perception/.../instance_tracker.py`)
looks for an existing object with:

- the same label, **and**
- 3D distance below `merge_distance` (default 0.5 m), **and**
- if both have CLIP features, similarity above `clip_threshold` (default 0.75).

If a match is found the existing object is updated (position fusion, belief
update, `detection_count++`). Otherwise a new object is registered.

There is **no semantic priority filter** at registration time — every
non-merged detection enters the table; capacity is enforced afterwards.

### 1.2 What Gets Pruned

- **Capacity bound.** When `len(objects) > max_objects` (default 200), entries
  are sorted by `(credibility, detection_count)` descending and only the top
  200 survive.
- **Stale removal.** `prune_stale()` drops anything that has not been observed
  for `stale_timeout` seconds (default 300 s, i.e. 5 minutes).

### 1.3 Hierarchy

The serialised scene-graph JSON contains:

| Layer | Source | Contents |
|-------|--------|----------|
| `objects` | `InstanceTracker` | id, label, position, score, detection_count, region_id, belief |
| `relations` | rule-based geometry | `near`, `on`, `above`, `left_of`, … between object IDs |
| `regions` | DBSCAN clustering (ε=3 m) | `name` (e.g. `area_with_door_desk`), center, `object_ids` |
| `rooms` | `room_manager.py` | room id/name (corridor, office, …) inferred from region object types; rule-based or LLM-assisted |
| `groups` | `room_manager.py` | semantic groups inside a room (e.g. `office_workstation`) |
| `views` | `keyframe_selector.py` | keyframe poses + visible object_ids + key_labels |

There is **no `floor` node**. Effective hierarchy is **object → group → room
+ region + view**. Single-floor sites work fine; multi-floor would need a
separate Floor abstraction layered on top.

## 2. FSR-VLN vs LingTu

### 2.1 What FSR-VLN Does (arXiv 2509.13733, 2025)

- HMSG with **four** layers: Floor → Room → View → Object. Each View stores a
  CLIP embedding and a VLM-generated image caption; each Object is tied to its
  best view.
- Map building is **offline** — SLAM + open-vocab instance mapping first, then
  HMSG construction.
- **Fast stage.** LLM parses the instruction into a structured query
  (floor / region / object). CLIP is used to coarse-rank Rooms → Views →
  Objects, producing candidate goal view + goal object.
- **Slow stage.** A VLM (GPT-4o) inspects the best-view image of the candidate
  object to verify the target is actually visible. If not, the LLM picks a
  better view from unmatched view captions, the VLM compares it against the
  fast candidate, and the goal is updated.

### 2.2 Side by Side

| Aspect | FSR-VLN | LingTu |
|--------|---------|--------|
| Building | offline HMSG | online incremental, while exploring |
| Hierarchy | Floor → Room → View → Object | Object → Group → Room (+ regions, views), no Floor |
| View layer | CLIP embedding + VLM caption | Keyframes captured but no per-view caption |
| Fast path | layered CLIP coarse rank | multi-source fusion (label + CLIP + detector + spatial) |
| Slow path | VLM verifies + refines on real images | LLM reasoning over text scene-subgraph (vision verification optional) |
| VLM in the loop | yes, mandatory | optional; primary path is text-only |

The LingTu fast path scores all four signals together and only escalates to
the LLM when confidence drops; FSR-VLN escalates to the VLM whenever the
fast candidate could be wrong, which is more cautious but more expensive.

## 3. Where to Look Next

| Path | Topic |
|------|-------|
| `src/perception/.../instance_tracker.py` | Match / merge / prune |
| `src/perception/.../scg_builder.py` | Scene-graph assembly |
| `src/perception/.../room_manager.py` | Region → room inference |
| `src/perception/.../keyframe_selector.py` | View selection |
| `src/decision/.../fast_path.py` | Fast multi-source matcher |
| `src/decision/.../slow_path.py` | LLM reasoning + selective grounding |
| `src/memory/modules/semantic_mapper_module.py` | Scene-graph → Room-Object KG + topology |
| `FAST_SLOW_IMPLEMENTATION.md` | Goal-resolver internals |

## 4. References

- FSR-VLN: *Fast and Slow Reasoning for Vision-Language Navigation with
  Hierarchical Multi-modal Scene Graph*, arXiv:2509.13733 (2025).
- VLingNav: arXiv:2601.08665 (2026).
- ESCA / SGCLIP: NeurIPS 2025.
- HOV-SG: RSS 2024.

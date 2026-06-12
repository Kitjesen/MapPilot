# Semantic - perception, planning, reconstruction

`src/semantic/` owns LingTu's semantic navigation layer: visual perception,
language/scene reasoning, visual servo, and online reconstruction. The three
main subtrees are intentionally kept under `semantic/`; do not recreate
top-level `src/perception`, `src/planner`, or `src/reconstruction` packages.

## Subtree map

| Subtree | Role | Key modules |
| --- | --- | --- |
| `perception/` | L3 semantic perception. | Detector/encoder backends, scene-graph builder, instance tracking, perception service APIs. |
| `planner/` | L4 semantic decision layer. | `SemanticPlannerModule`, `GoalResolver`, `AgentLoop`, LLM clients, visual servo, semantic frontier scoring. |
| `reconstruction/` | 3D reconstruction and dataset capture. | `reconstruction_module.py`, keyframe export, dataset recorder, semantic labeling, reconstruction server. |
| `tests/` | Cross-semantic tests. | Use package-local tests when behavior is scoped to one subtree. |

## Perception

`perception/` turns camera/depth inputs into detections, embeddings, tracked
instances, and scene-graph evidence.

Backends register through `@register("detector", name)` and
`@register("encoder", name)`. Current detector families include YOLOE,
YOLO-World, Grounding DINO, and BPU-backed inference; encoder families include
CLIP and MobileCLIP.

## Planner

`planner/` turns instructions and semantic evidence into goals or near-field
commands. The goal-resolution chain is:

```text
instruction
  -> tagged location lookup
  -> fast scene-graph / CLIP matching
  -> vector memory search
  -> semantic frontier candidate scoring
  -> visual servo
  -> optional slow LLM reasoning
```

Important boundary: `semantic/planner/frontier_scorer.py` ranks semantic
frontier candidates for `SemanticPlannerModule`; it is not the same runtime
module as `nav/frontier_explorer_module.py`, which publishes exploration goals
for the navigation stack.

## Reconstruction

`reconstruction/` contains online 3D reconstruction and data capture helpers.
It belongs under `semantic/` because reconstruction consumes perception outputs
and semantic labels; it is not a first-level `src/` package.

## Dependency rules

Semantic code may depend on `core/`, `memory/` contracts, and semantic-local
helpers. It must not directly import `nav/`, `drivers/`, or `gateway/` business
logic. Cross-layer flow belongs in Blueprint wires, ports, registry lookups, or
gateway/MCP contracts.

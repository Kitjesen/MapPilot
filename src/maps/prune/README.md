# src/maps/prune — dynamic-object ghost pruning

This module owns LingTu's ghost-pruning product path for point-cloud maps.
Two boundaries must stay separate:

- **Live navigation keeps current obstacles.** Anything observed right now is
  an obstacle the local planner must respect, even if it was not in the saved
  map. Live obstacles are never "cleaned" out of the runtime view.
- **Saved-map** cleanup runs offline (or on save/rebuild) to remove
  dynamic-object ghosts baked into a recorded map: trails left by pedestrians,
  vehicles, or other movers while the map was being built.

Mixing the two directions corrupts behavior in both directions: a live run
must not erase real obstacles, and a saved map must not keep ghost trails.

## Product path

Ghost pruning is a **LingTu-owned product path** implemented under
`cpp/prune`, built from LingTu's own cleaner/core implementation. It is
licensed clean: the product path reimplements the ERASOR2 workflow
**without bringing ERASOR2 GPLv3 code** into the product tree.

- Product implementation: `cpp/prune` (this directory's C++ sources).
- Reference material: `third_party/research_nav/ERASOR2` is an optional external
  checkout and does not enter the default product build.
- Do not include upstream ERASOR2 headers in `cpp/prune` sources; the product
  path must stay license-clean.

## Extracted ERASOR2 Workflow

The reference workflow is decomposed into LingTu stages:

`load -> label -> submap -> evidence -> protect -> score -> split -> save`

| Stage | Status |
| --- | --- |
| `load` | done |
| `label` | done |
| `submap` | done |
| `evidence` | done |
| `protect` | done |
| `score` | partial |
| `split` | partial |
| `save` | done |

Stages marked partial are completed for the `lingtu_field_v1` product
contract and are extended only behind that contract's schema version.

## Runtime Local Planning vs Saved Map / Rebuild

- Runtime Local Planning: Current obstacles stay in `rt/nav/traversability`
  and the local planner consumes them at control rate. Runtime ghost
  suppression is a runtime concern and stays out of the batch cleaner.
- Saved Map / Rebuild: batch cleaning produces a ghost-free map artifact for
  later localization and global planning.

Do not feed a saved-map batch cleaner directly into the 10 Hz local planner:
batch latency and artifact semantics are incompatible with the control loop.

## Product Framework

The pruning pipeline plugs into the maps artifact framework as the
`lingtu_field_v1` cleaner stage: it consumes labeled scans, applies the
extracted workflow above, and saves a cleaned artifact with schema version
metadata so downstream consumers can pin compatibility.

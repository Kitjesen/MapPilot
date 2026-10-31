# Memory

`memory/` stores semantic context around a robot run. It answers questions
such as “what did we see here?”, “what is this place called?”, and “what
objects were observed recently?”. It helps semantic tasks, operator queries,
inspection review, and exploration heuristics.

It is not the robot's geometric map. LiDAR accumulation, map save/reload,
localization, traversability, global planning, and final motion remain owned by
the native map, SLAM, and navigation services. A memory result can suggest or
resolve a semantic target; it cannot authorize motion or replace OctoPlanner3D
and the local planner.

The product design for a future map-bound `object_memory`, including stable
object identity, relations, map epochs, and the LLM boundary, is documented in
[`docs/research/semantic-map.md`](../../docs/research/semantic-map.md).
That design does not mean the persistent object artifact is implemented yet.

## What is active today

The memory stack is created by
[`src/lingtu/assembly/host/stacks/memory.py`](../lingtu/assembly/host/stacks/memory.py)
only when the resolved Host enables semantic planning. The standard `map` and
`nav` Products therefore do not depend on this directory for the core
建图 → 保存/重载 → 定位 → 自主导航 path.

| Runtime module | What it remembers | Where it is used |
| --- | --- | --- |
| `modules/semantic.py` | Room/object knowledge and a semantic topology graph derived from `SceneGraph` | Semantic planner room summaries, object queries, and exploration hints |
| `modules/episodic.py` | Recent scene observations with pose/time context | MCP history queries and post-run inspection |
| `modules/locations.py` | Operator-named poses and aliases in `tagged_locations.json` | Named-goal resolution and location APIs when the module is present |
| `modules/vector.py` | Embedding-indexed scene observations | Fuzzy semantic lookup; a result is only a candidate |
| `modules/temporal.py` | Time-indexed object sightings in SQLite | Temporal gateway queries |

These modules are optional Host components. Their unit tests prove their local
contracts; they do not prove that a `map`/`nav` field RunPlan loads them or that
their output is safe to execute as a motion command.

Vector retrieval has an additional admission rule: a stored observation is a
semantic candidate only when its scene, pose, and navigation-state snapshot
are fresh and bound to the active map/version. Older or unbound observations
remain useful for history queries, but they are not sent directly to the motion
stack. Changing maps re-evaluates candidate eligibility without deleting the
history.

## The four code areas

The folders describe implementation responsibility rather than four different
“memory products”:

| Folder | Owns | Important entry points |
| --- | --- | --- |
| `modules/` | Host adapters, subscriptions, skills, and outputs | The five modules listed above |
| `spatial/` | Places, named locations, episodes, room inference, and semantic topology data structures | `places.py`, `tagged_locations.py`, `episodic.py`, `topology_graph.py` |
| `knowledge/` | Industrial knowledge graph, room/object relations, semantic priors, and optional belief inference | `knowledge_graph.py`, `room_object_kg.py`, `semantic_prior.py`, `belief/` |
| `storage/` | Persistence used by active consumers | `temporal_store.py` and the SQLite backend used by episodic memory |

`scheduling/voi_scheduler.py` is an offline/research exploration helper. It is
tested independently but is not added by the `memory()` Host stack. It should
not be treated as part of the core navigation loop until a Product explicitly
wires it.

## Two location stores that still need one owner

There are two intentionally visible location entry points:

- `spatial/tagged_locations.py` stores operator tags in a Host JSON file.
- `spatial/places.py` reads canonical map-bound POIs from mapd `pois.tsv` and
  serves the `/api/v1/places` APIs.

They do not synchronize automatically. The map-bound POI path is the durable
direction for productization; the JSON tag path remains a compatibility layer
until name/alias migration and map-change behavior are complete. See
[`docs/roadmap.md`](../../docs/roadmap.md#map-bound-places).

## What is deliberately not here

- Point-cloud or occupancy-map ownership
- SLAM or `map ← odom` correction
- Relocalization health and degraded-state transitions
- OctoPlanner3D global paths, local collision checks, or `cmd_vel`

Keeping these boundaries explicit prevents semantic history from being used as
navigation truth. The next useful memory work is to finish the map-bound place
ownership migration, then decide which optional semantic modules deserve a
field RunPlan; adding more storage abstractions or an unconnected mission
logger does not advance the core navigation product.

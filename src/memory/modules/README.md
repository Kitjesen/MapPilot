# Runtime memory modules

`modules/` contains the Host-facing adapters. They subscribe to runtime
messages, keep a bounded in-process view, and expose skills or outputs to the
Gateway and semantic planner. The data structures and persistence code live in
`spatial/`, `knowledge/`, and `storage/`; this directory does not own the
geometric map, localization, or motion command.

The standard `map` and `nav` Products do not load this stack. The modules are
loaded when the selected RunPlan enables semantic planning (currently the
inspection-oriented path). Read [`../README.md`](../README.md) before changing
which module belongs in the stack.

| Module | Runtime purpose | Current status |
| --- | --- | --- |
| `semantic.py` | Turns scene-graph observations into room/object knowledge and a semantic topology summary for semantic planning. | In the optional semantic-planning stack |
| `episodic.py` | Keeps recent observed events and poses for history/MCP queries. | In the optional semantic-planning stack |
| `locations.py` | Stores named poses and resolves operator location names. | Optional; JSON tags still coexist with mapd POIs |
| `vector.py` | Retrieves similar scene observations from embeddings. | Optional semantic query path |
| `temporal.py` | Persists time-indexed object sightings for temporal queries. | Optional semantic/query path |
| `topology.py` | Standalone explored-area graph module. | Registered for experiments, but not added by `memory()` |

`_odometry.py` is shared input plumbing for modules that need pose updates;
it is not a second memory backend.

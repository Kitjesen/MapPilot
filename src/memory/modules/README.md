# Memory Modules — Persistent and Ephemeral Robot Memory

This package provides the memory layer for the robot: short-term episodic buffers, persistent tagged locations, semantic scene maps, and vector-similarity retrieval. All memory modules are independent and can be composed via the memory stack factory.

## Files

- **`semantic_mapper_module.py`** — SceneGraph→RoomObjectKG converter; builds hierarchical room-object-topology graph from perception detections.
- **`episodic_module.py`** — Episodic memory: records time-ordered event sequences (observations, actions, outcomes) for retrospective analysis.
- **`tagged_locations_module.py`** — Tagged locations: persistent named waypoints with fuzzy-match lookup for semantic goal resolution.
- **`vector_memory_module.py`** — CLIP + ChromaDB vector search: stores embedding-indexed observations for semantic similarity retrieval.
- **`mission_logger_module.py`** — Mission logger: serializes complete mission state (goals, path, events) to disk for post-hoc replay.
- **`temporal_memory_module.py`** — Temporal memory: time-windowed state buffer with decay; short-term working memory for recent events.
- **`topological_module.py`** — Topological graph: connectivity graph of explored areas; supports frontier-based exploration targets. Registered via `@register` but **not** added by the `memory()` stack factory, so it is not on the production path today. `semantic_mapper_module.py`'s `TopologySemGraph` is the room-level topology that is actually wired in.
- **`_odom_mixin.py`** — Shared odometry mixin: provides odometry integration helpers for memory modules that need pose interpolation.

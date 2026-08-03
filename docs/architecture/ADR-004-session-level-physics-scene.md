# ADR-004: MuJoCo physics is composed at session scope

Status: Accepted

One contact-coupled simulation session owns one `mjModel`/`mjData` pair. A
`PhysicsSceneComposer` loads the WorldPackage, attaches each robot with a
unique namespace, resolves global-attribute conflicts, compiles the model, and
emits a generation-scoped ModelDescriptor.

Stable IDs are derived from `instance_id + local_name`; MuJoCo dense indices
are valid only inside the current `model_generation`. The first implementation
supports static composition at session start. Runtime add/delete and
`mj_recompile` are deferred until whole-session generation swaps are reliable.

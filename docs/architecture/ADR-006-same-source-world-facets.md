# ADR-006: Worlds compile into runtime facets from one source

Status: Accepted

## Context

RobotSimUE needs high-fidelity geometry and materials while MuJoCo must remain
the single clock, contact, dynamics, and physics-raycast authority. An Unreal
level cannot be consumed directly by MuJoCo, and running Chaos as a peer physics
engine would introduce divergent contacts and poses. A shared render mesh is
also insufficient: MuJoCo's normal mesh collision operates on the mesh convex
hull, so arbitrary concave UE geometry is not an equivalent collider.

## Decision

One canonical SceneDraft/entity graph is published through `WorldImporter` into
one immutable `WorldPackage`. Publication generates and integrity-binds:

- a MuJoCo facet containing heightfields, primitives, and qualified convex
  proxies;
- an Unreal visual projection containing render assets and canonical entity
  transforms;
- a sensor-surface policy declaring whether each geometry-bearing modality uses
  the MuJoCo proxy, a qualified Unreal surface, or excludes visual-only detail;
- stable entity IDs, provenance, license, and content hashes shared by all
  facets.

The Session Compiler emits `physics.plan.json` and `visual.plan.json` from that
same package. `PhysicsSceneComposer` compiles the world and all robots into one
session `mjModel`; RobotSimUE follows package transforms and immutable MuJoCo
snapshots. Runtime UE Actors do not participate in Chaos physics.

Tripo, scan, Fab, and DCC meshes are visual inputs by default. They require a
separate collision recipe and qualification before the package can declare
collision support.

The detailed contract is
[`SIM_WORLD_COLLISION_CONTRACT.md`](./SIM_WORLD_COLLISION_CONTRACT.md).

## Consequences

- Visual assets can be replaced or upgraded without silently changing physics.
- Contacts, LiDAR ray casts, replay, and qualification have one authority.
- Navigation depth cannot silently observe an obstacle that is absent from the
  MuJoCo collision/raycast world.
- A package is larger because it can carry separate render and collision assets.
- Concave environments require primitive composition or offline convex
  decomposition; one arbitrary mesh geom cannot claim shape fidelity.
- Runtime structural edits require a new session/model generation in the first
  release.

## Rejected Alternatives

- **UE Chaos as the world authority:** conflicts with MuJoCo robot dynamics and
  produces two contact truths.
- **Bidirectional Chaos/MuJoCo synchronization:** feedback loops and ordering
  ambiguity make deterministic replay and qualification unreliable.
- **Treat `.umap` as runtime physics configuration:** couples content authoring
  to physics compilation and bypasses package/digest validation.
- **Reuse every render mesh as a MuJoCo collider:** incorrect for concave meshes,
  expensive, and unsuitable for generated assets.

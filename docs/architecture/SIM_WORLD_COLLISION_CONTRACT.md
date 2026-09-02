# Simulation World Authoring and Collision Contract

Status: current boundary; rollout is incremental
Audience: SimStudio, asset/import, catalog, MuJoCo runtime, RobotSimUE, sensor, and qualification maintainers
Replaced by: not replaced

## Purpose

LingTu must render a high-fidelity Unreal world while preserving truthful robot
contact, LiDAR ray casting, and navigation physics in MuJoCo. The solution is not
to make an Unreal level authoritative and it is not to run Chaos and MuJoCo as
peer physics engines. One canonical scene declaration is published into two
integrity-bound runtime facets:

- a MuJoCo physics facet used for contacts, dynamics, and physics ray casts;
- an Unreal visual facet used for rendering, RGB/depth capture, and presentation.

Both facets retain the same stable entity IDs, canonical transforms, package
identity, and session ID. MuJoCo remains the sole simulation clock
and physics authority.

## Goals

- A wall, terrain feature, or prop visible in RobotSimUE has a corresponding
  MuJoCo collider when and only when the package declares it collidable.
- Static and dynamic entities use one stable identity across authoring,
  `WorldPackage`, `physics.plan.json`, `visual.plan.json`, runtime evidence, and
  replay.
- Visual quality can improve without changing robot dynamics or invalidating the
  collision contract.
- Generated or third-party visual assets cannot silently become physics truth.
- Collision fidelity is qualified with contacts and ray casts, not inferred from
  a screenshot or from the existence of a mesh file.

## Non-goals

- Loading a `.umap` directly into MuJoCo at runtime.
- Mirroring UE Chaos contacts into MuJoCo.
- Bidirectional per-frame transform synchronization between physics engines.
- Treating a Tripo, Fab, scan, or Blender render mesh as a valid collider without
  an explicit physics representation and qualification.
- Runtime structural editing of the active `mjModel` in the first release.

## Fixed Authority Decision

| Concern | Authority | Consumer |
| --- | --- | --- |
| Simulation clock, dynamics, contacts, forces | MuJoCo Physics Runtime | UE, sensors, controller, recorder |
| Robot and MuJoCo-dynamic object pose | MuJoCo snapshot | RobotSimUE visual projection |
| Scenario-kinematic object intent | Scenario Runtime | MuJoCo mocap/proxy update |
| Scenario-kinematic applied pose | MuJoCo readback | RobotSimUE and evidence |
| RGB/depth appearance | RobotSimUE renderer | Sensor Runtime / SHM adapter |
| LiDAR and physics ray casts | MuJoCo geometry | Sensor Runtime / evidence |
| Editable scene declaration | SimStudio `SceneDraft` | Scene publication only |
| Immutable world identity and assets | `WorldPackage` | Session Compiler |

UE Actors are presentation objects. A transform applied to a UE Actor is never
accepted as contact, odometry, or motion truth.

## Canonical Pipeline

```text
Source Inbox / CAD / scan / Tripo / Blender
                    |
                    v
            SimStudio SceneDraft
       mutable intent + stable entity IDs
                    |
                    v
          ScenePublicationService
        validation + canonical source handoff
                    |
                    v
               WorldImporter
       +-------------------------------+
       | MuJoCo MJCF / hfield / proxies|
       | UE visual projection / assets |
       | provenance / license / hashes |
       | qualification evidence        |
       +-------------------------------+
                    |
                    v
         immutable versioned WorldPackage
                    |
                    v
        Catalog Resolver + Session Compiler
             /                       \
            v                         v
    physics.plan.json          visual.plan.json
            |                         |
            v                         v
  PhysicsSceneComposer       RobotSimUE Visual Runtime
    one mjModel/mjData        NoCollision presentation
            |                         ^
            +---- truth snapshot -----+
```

The `SceneDraft` is authoring state, not runtime configuration. Publication is
the only path from an editable draft to a catalog-visible package. The Session
Compiler resolves the immutable package once and emits backend-specific plans;
neither runtime parses the draft or source YAML again.

## World Entity Contract

The existing `WorldPackage` entity fields are the cross-facet identity seam:

```yaml
entity_id: cabinet_01
entity_type: equipment_cabinet
authority: mujoco
initial_transform:
  position_m: [12.0, -4.0, 0.6]
  quaternion_wxyz: [1.0, 0.0, 0.0, 0.0]
physics_proxy: mujoco
semantic_class: equipment_cabinet
collision: true
geometry:
  shape: box
  size_m: [0.8, 0.4, 1.2]
visual:
  mode: level
```

`visual.mode=level` means that the package publisher emits a world visual
projection for UE materialization; it does not authorize a
hand-authored `.umap` to become the source of entity identity or physics.
The current importer maps qualified primitive geometry to an Unreal asset in
`visual/world.visual-projection.json`. A future high-fidelity asset binding
must extend that projection contract explicitly rather than adding an
unrecognized field to the world manifest.

Canonical authoring coordinates are right-handed, Z-up, metres, with
quaternions ordered `w, x, y, z`. The Unreal projection performs the single
approved conversion:

```text
(x, y, z) metres -> (100*x, -100*y, 100*z) centimetres
```

Conversion is implemented at the visual boundary and must be covered by
round-trip and known-transform tests. Source assets may declare their own DCC
frames, but the importer must normalize them before package publication.

## Entity Modes

| Package declaration | Runtime meaning | UE collision policy |
| --- | --- | --- |
| `authority=mujoco`, `physics_proxy=mujoco`, `collision=true` | Static or MuJoCo-owned physics entity | `NoCollision`; follow package/snapshot transform |
| Scenario entity with `authority=scenario`, kinematic proxy | Scenario proposes poses; MuJoCo applies a mocap/proxy batch and owns contact/raycast readback | `NoCollision`; follow MuJoCo-applied readback |
| `physics_proxy=none`, `collision=false` | Visual-only dressing | `NoCollision` |
| Editor preview/query helper | Authoring selection only; excluded from package runtime truth | Editor-only query channel |

UE query helpers must not feed navigation, LiDAR, safety, qualification, or
episode evidence. Packaged simulation Actors corresponding to MuJoCo entities
must not participate in Chaos simulation.

## Collision Representation Policy

The render mesh and the collision representation are separate facets of the
same entity. They may share an authoring source, but they do not have to share
topology.

| World feature | Required MuJoCo representation | Notes |
| --- | --- | --- |
| Large terrain | Heightfield generated from the same elevation samples as the UE Landscape/mesh | Preserve grid, extents, min/max elevation, orientation, and digest |
| Roads, floors, walls, curbs | Plane or box primitives | Fast and deterministic |
| Poles, tanks, bollards | Cylinder/capsule/box primitives | Prefer analytic geometry |
| Convex prop | Certified convex mesh or primitive approximation | Record source and proxy digests |
| Concave prop/building/interior | Multiple primitives or an offline convex decomposition | Never rely on one arbitrary mesh geom |
| Scenario pedestrian/vehicle proxy | Kinematic primitive or convex-piece set | Pose batch is generation- and sequence-stamped |
| Visual dressing | No MuJoCo geom | Must declare `collision=false` |

MuJoCo renders arbitrary triangle meshes, but normal mesh collision uses the
mesh convex hull. Therefore the current compatibility input
`mesh: {collision: true}` is not sufficient evidence for a concave environment.
Before a mesh-backed world can be qualified as collision-faithful, publication
must either prove the mesh is convex or consume an explicit decomposition
manifest. Until that gate exists, high-fidelity environment collision is
qualified only for heightfields, primitives, and declared convex proxies.

SDF and rigid-flex non-convex collision are deferred. Revisit them only when a
real target asset cannot meet accuracy and performance gates with primitives or
convex decomposition.

## Sensor Surface Consistency

World publication also assigns every geometry-bearing sensor modality an
explicit surface policy:

| Modality | Authoritative surface |
| --- | --- |
| RGB | Unreal render facet |
| MuJoCo LiDAR / physics ray cast | MuJoCo physics facet |
| Navigation depth / segmentation | Qualified sensor surface tied to the same stable entity and pose |

A high-fidelity render mesh may differ visually from its conservative collision
proxy, but it must not silently create contradictory navigation truth. An entity
visible to navigation depth must therefore either:

1. have a corresponding MuJoCo proxy and pass declared geometric tolerance
   checks; or
2. be excluded from the authoritative depth/segmentation pass and remain RGB
   dressing only.

An obstacle that appears in navigation depth but is absent from MuJoCo contact
and LiDAR, or the inverse, fails package qualification. The first vertical slice
may use the existing primitive geometry as both the MuJoCo proxy and the sensor
surface; high-fidelity replacement comes only after this policy is represented
in the compiled SensorPlan.

## Generated Asset Policy

Tripo and similar generators enter through Source Inbox as untrusted visual
sources. Their default status is `visual-only candidate`.

Before promotion, the offline Blender/import pipeline must establish:

- source URI, owner, license file, input-image digest, prompt/task ID, generator
  model version, and generated-file digest;
- canonical scale, origin, axes, normals, UVs, PBR texture roles, and material
  identity;
- render mesh role and a separately declared collision recipe;
- stable entity/asset ID and deterministic export settings;
- triangle/texture budgets and any Nanite/LOD decision.

Replacing a primitive UE preview with a Tripo or scanned high-fidelity render
mesh must not change the entity ID, canonical transform, or MuJoCo proxy unless
the physics change is explicitly versioned and re-qualified.

## Module Boundaries And Persisted Data

| Owner | Public input/output | Persisted data |
| --- | --- | --- |
| `tools/simstudio/` | Scene editing commands and validated `SceneDraft` | Draft revision, draft digest, authoring metadata |
| `ScenePublicationService` | Draft + publication request -> import/promotion result | Publication event and resulting package identity |
| `sim/catalog/importers/world.py` | Canonical world source -> qualified `ImportDraft` | MJCF, hfield/proxies, visual projection, provenance, qualification report |
| `CatalogPromoter` | Qualified draft -> immutable catalog package | Versioned `WorldPackage` and content digests |
| `SessionCompiler` | `SessionSpec` + exact package versions -> plans | `session.yaml`, plans, shared `session_id` |
| `PhysicsSceneComposer` | Typed `PhysicsScenePlan` -> `mjModel` + `ModelDescriptor` | No package mutation; generation-scoped runtime state only |
| `ULingTuSimVisualWorldSubsystem` | `visual.plan.json` + truth snapshots | UE assets/maps are cooked content; runtime state is ephemeral |
| Recorder/qualification | Contacts, ray casts, snapshots, visual binding evidence | Run-local immutable evidence |

## Failure Modes And Mitigations

| Failure | Required response |
| --- | --- |
| UE and MuJoCo use different entity transforms | Fail publication or session preparation; never apply a corrective runtime offset silently |
| Concave render mesh is supplied as one MuJoCo mesh geom | Fail high-fidelity collision qualification; require convex proof/decomposition |
| Missing collider for `collision=true` entity | Fail import/qualification |
| Collider exists for a visual-only entity | Fail import/qualification |
| UE physics becomes enabled on a physics-shared Actor | Fail UE automation/qualification |
| Dynamic pose batch is stale, partial, duplicated, or from another generation | Reject the entire batch before mutating `mjData` |
| Generated asset lacks license/provenance | Quarantine; do not promote to Catalog |
| Visual binding and package identity or referenced path differ | Fail session preparation |

## Qualification Contract

A claim that a UE scene has truthful MuJoCo collision requires one run-bound
evidence set containing:

1. exact `WorldPackage`, session ID, and `model_generation`;
2. entity identity parity across package, physics plan, visual plan, and UE
   binding;
3. coordinate/extent parity for terrain and each tested proxy;
4. MuJoCo compile success and expected geom/body registry;
5. MuJoCo contact or ray-cast evidence naming the expected stable entity;
6. UE visual binding evidence for the same entity and generation;
7. sensor-surface policy and depth/LiDAR visibility parity for tested obstacles;
8. proof that the UE Actor is not a competing physics authority;
9. reset/replay evidence showing the same initial state.

A screenshot alone proves appearance only. A UE hit event proves only a UE
query. Neither proves MuJoCo collision.

## First Vertical Slice

Use `FactoryPark_HF + ThunderV4` and publish three SceneDraft elements:

- `equipment_cabinet` — colliding box proxy, high-fidelity visual candidate;
- `jersey_barrier` — colliding box proxy;
- `traffic_cone` — colliding cylinder proxy, as declared by the current element
  catalog.

The slice is complete when:

1. one publication produces the immutable world manifest, generated MJCF, and
   visual projection from the same compiled element batch;
2. the three stable IDs resolve into the expected physics and visual plans;
3. MuJoCo ray casts hit every declared collider and miss visual-only entities;
4. Thunder cannot pass through the cabinet/barrier proxy and the contact is
   recorded with the expected stable ID;
5. RobotSimUE materializes the corresponding visuals at the converted poses
   with runtime Chaos collision disabled;
6. replacing one visual with a Blender/Tripo asset changes only the visual
   artifact and package version, not the collision semantics;
7. reset reproduces the same poses and evidence identities.

Do not start with a whole AI-generated factory. This slice establishes the
contract cheaply before asset volume and visual complexity increase.

## Rollout

1. **Primitive parity:** retain the existing heightfield/box/cylinder importer,
   publication, MuJoCo compile, and UE projection gates.
2. **Visual replacement:** import one conditioned Blender/Tripo cabinet asset
   while retaining its qualified box proxy.
3. **Convex proxy contract:** add explicit render/collision asset roles,
   convex certification or decomposition metadata, and fail-closed validation.
4. **Dynamic props:** add MuJoCo-owned movable bodies and scenario-kinematic
   proxies with UE readback following.
5. **Large-world qualification:** add bounded spatial parity probes, contact
   sampling, sensor/raycast checks, and performance budgets per world package.

Whole-session generation swap remains the structural update mechanism. Runtime
`mj_recompile` editing stays deferred until session swap, state migration, and
rollback evidence are reliable.

## Official Engine Evidence

- MuJoCo mesh assets and collision behavior:
  https://mujoco.readthedocs.io/en/stable/XMLreference.html#asset-mesh
- MuJoCo geoms, heightfields, and contact geometry:
  https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-geom
- MuJoCo `mjSpec` subtree attachment used by session composition:
  https://mujoco.readthedocs.io/en/latest/programming/modeledit.html#attachment
- Unreal Engine 5.8 Interchange import and custom collision options:
  https://dev.epicgames.com/documentation/en-us/unreal-engine/interchange-import-reference-in-unreal-engine

# Robot-mounted weapon gameplay vertical slice

Status: proposed implementation plan

## Outcome

Add one fictional, robot-mounted remote weapon station to the RobotSimUE
simulation product. In Tactical UI mode, a focused left mouse press produces a
correlated fire intent. The MuJoCo-owned runtime accepts or rejects that intent,
advances a deterministic projectile, detects the first authoritative hit, and
applies recoil to the robot. Unreal presents the station, muzzle flash, tracer,
impact, audio, and HUD feedback; it never becomes a second collision or
ballistics authority.

This is a game/simulation feature only. It does not define a real firearm,
manufacturing process, real ammunition, or field-robot command path.

## Frozen decisions

1. MuJoCo remains the sole simulation clock, rigid-body, collision-geometry,
   and robot-state authority.
2. A mounted station is a versioned `PayloadPackage`, not a Thunder-specific
   branch and not a Scenario entity.
3. The Thunder package exposes a generic `payload_top` mount. It does not name
   or own a weapon.
4. Tactical mouse input emits intent only. UE does not spawn authoritative
   projectile Actors or apply UE/Chaos impulses.
5. Projectile flight uses a deterministic point-mass solver plus swept MuJoCo
   geometry queries. It does not create one tiny MuJoCo rigid body per bullet.
6. Recoil and dynamic-target impact are applied in the MuJoCo owner thread at
   resolved bodies and world-space points.
7. Fire input uses a distinct `lingtu.sim.weapon-intent.v1` envelope on the
   existing allocation-owned loopback control transport. It does not overload
   the drive-axis schema or lifecycle requests.
8. Existing v1 schemas stay immutable. Payload-aware sessions and plans use
   explicit v2 schemas rather than silently expanding `additionalProperties:
   false` v1 contracts.
9. The first slice is semi-automatic single fire. Trigger cadence is owned by
   the runtime, never by frame rate or mouse-repeat behavior.

## Ownership

| Concern | Owner | Output |
| --- | --- | --- |
| Left mouse press/release and Tactical focus | `LingTuSimUI` | `FWeaponIntentSample` |
| Identity, serialization, successful-send evidence, ACK | `LingTuSimSession` | `lingtu.sim.weapon-intent.v1` |
| Payload package resolution and mount validation | Catalog/Session Compiler | Physics and Visual v2 plans |
| Gimbal state, fire admission, projectile integration, hit query, recoil | MuJoCo Physics process | truth snapshot plus ballistic events |
| Tracer, muzzle flash, impact, audio, camera shake, reticle | `LingTuSimVisual` / `LingTuSimUI` | presentation only |
| Shot, hit, command, snapshot, and rejection evidence | Recorder | deterministic replay material |
| Damage, scoring, target response rules | Scenario evaluator, later slice | gameplay verdicts |

No Adapter may advance projectile time, choose a hit, or apply a body impulse.

## Package and session model

### Generic robot mount

The frozen qualification package `thunderv4@1.0.1` remains byte-identical.
`thunderv4@1.0.2` receives one package-owned frame and matching MJCF site:

```yaml
frames:
  - name: payload_top
    role: payload_mount
    parent_frame: base_link
    extrinsic:
      position_m: [PACKAGE_CALIBRATED]
      quaternion_wxyz: [PACKAGE_CALIBRATED]
```

The resolver must prove that the declared frame resolves to exactly one current
MJCF body/site and that its transform is finite. The value is measured from the
robot asset; it is not guessed in runtime code.

### PayloadPackage

Initial package identity: `fictional_rws@1.0.0`.

```yaml
schema: lingtu.sim.payload-package.v1
id: fictional_rws
version: 1.0.0
kind: payload
compatibility:
  runtime_abi: lingtu.sim.payload.v1
  mujoco: 3.10.x
mount:
  attach_root: station_base
  required_parent_role: payload_mount
physics:
  mjcf: mjcf/fictional_rws.xml
  global_options: inherit_session
visual:
  binding: PayloadVisual:FictionalRWS
  projection: visual/payload.visual-projection.json
frames:
  yaw_axis: yaw_link
  pitch_axis: pitch_link
  muzzle: muzzle_site
interfaces:
  command:
    - lingtu.sim.payload-aim.v1
    - lingtu.sim.weapon-intent.v1
  events:
    - lingtu.sim.shot-event.v1
    - lingtu.sim.hit-event.v1
ballistics:
  model: point_mass_quadratic_drag_v1
  profile: ballistics/fictional_profile.yaml
recoil:
  profile: recoil/fictional_profile.yaml
```

The package owns mass, inertia, joint limits, actuator limits, muzzle frame,
fictional ballistic constants, visual projection, and content digests. It owns
no port, DDS domain, process ID, run ID, or robot instance name.

### Session v2 loadout

```yaml
schema: lingtu.sim.session.v2
robots:
  - instance_id: thunder_01
    package: thunderv4@1.0.2
    controller: thunderv4_locomotion@1.0.0
    sensor_rig: thunderv4_navigation@1.0.0
    payloads:
      - instance_id: rws_01
        package: fictional_rws@1.0.0
        parent_frame: payload_top
```

The compiler resolves this into:

- `physics.plan.v2`: robot, payload attachment, body/site/actuator identities,
  ballistic profile, recoil profile, and collision groups.
- `visual.plan.v2`: payload visual projection, parent stable frame, gimbal-link
  bindings, and effect asset IDs.
- Existing `transport.intent`: the same allocated loopback endpoint; no new
  port is required.
- `session.yaml`: authored `session_id`, exact package references, MuJoCo
  version, and seed. The generated plans carry normalized runtime parameters.

## Physics composition

`PhysicsSceneComposer` composes in this order before one `mjModel` is compiled:

```text
World
  -> Robot instance at spawn
      -> Payload instance at robot payload_mount
```

The payload is a real MuJoCo subtree. Its station base, yaw link, pitch link,
barrel link, mass, inertia, joints, limits, and actuators therefore participate
in the same solver as the quadruped. The compiled namespace is unique, for
example:

```text
thunder_01__base_link
rws_01__station_base
rws_01__yaw_joint
rws_01__pitch_joint
rws_01__muzzle_site
```

Stable IDs remain slash-based and generation-scoped:

```text
thunder_01/base_link
rws_01/muzzle_site
```

`ModelDescriptor` gains payload-instance descriptors and the resolved parent
mount. Dense MuJoCo indexes are valid only for the current `model_generation`.

## Input and command flow

The existing `IInputProcessor` remains the input seam. Add mouse down/up
overrides and a dedicated publisher; do not add fire fields to
`FRobotDriveInputState`.

Acceptance conditions for left mouse:

- UI mode is exactly `Tactical`.
- The RobotSimUE viewport is focused and input capture is valid.
- The click is `EKeys::LeftMouseButton`.
- The current session, model generation, and reset generation are available.
- The payload instance is bound and reports ready.
- UI widgets that consume the click prevent weapon intent publication.
- Press and release each produce at most one source sequence transition.
- Focus loss, mode exit, pause, reset, or teardown emits/reconciles a release.

Envelope shape:

```json
{
  "schema": "lingtu.sim.weapon-intent.v1",
  "run_id": "...",
  "session_id": "weapon-demo",
  "boot_id": "...",
  "model_generation": 0,
  "reset_generation": 0,
  "source_id": "robotsimue.operator",
  "source_epoch": 1,
  "source_sequence": 42,
  "event_id": "<boot_id>:1:42",
  "robot_instance_id": "thunder_01",
  "payload_instance_id": "rws_01",
  "action": "trigger_press",
  "source_monotonic_ns": 0
}
```

The Session side writes successful-send evidence before the runtime ACK can be
trusted. The receiver rejects duplicate, stale, future-generation, wrong-run,
wrong-payload, paused, unready, and out-of-order intents. A shot is identified
by an authoritative `shot_id`, not by the UI event ID alone.

Mouse motion may later publish requested gimbal yaw/pitch. The actual muzzle
pose always comes from the current MuJoCo gimbal state after limits and actuator
dynamics; the client-provided camera ray is never accepted as hit truth.

## Deterministic ballistic model

For each accepted shot:

1. Read the current authoritative muzzle site position, orientation, and body
   velocity from MuJoCo.
2. Initialize projectile position at the muzzle and velocity as muzzle-axis
   launch velocity plus the muzzle body's world velocity.
3. Advance at a fixed simulation-time step, with bounded substeps independent
   of UE frame rate.
4. Use SI units and the resolved world gravity.
5. Apply quadratic aerodynamic drag relative to the configured wind:

```text
v_relative = v_projectile - v_wind
a = gravity - drag_k * |v_relative| * v_relative
```

6. Sweep the segment from the previous to next position against current MuJoCo
   geoms. Take only the nearest eligible hit and stop that projectile.
7. Emit `ShotEvent`, zero or more trajectory samples for presentation, and one
   terminal `HitEvent` or `ExpiredEvent`.

The first implementation uses a point mass. It includes gravity, quadratic
drag, wind, moving-platform launch velocity, deterministic dispersion from the
session seed, and swept collision. It deliberately excludes spin drift,
Coriolis, penetration, ricochet, fragmentation, deformable surfaces, barrel
heating, and real ammunition tables.

The current sensor raycast API cannot be reused as the public projectile API:
it starts every ray at a named sensor frame and deliberately excludes the
owning robot subtree. Ballistics needs an internal arbitrary-origin segment
query and a precomputed `geom_id -> stable body/entity/material` table. This
query stays private to the MuJoCo owner thread.

## Recoil and impact response

When a shot is accepted, compute a package-defined reaction impulse. Schedule
it as a finite force curve over one or more fixed MuJoCo steps:

```text
force_at_step = recoil_impulse * normalized_profile_weight / timestep
```

Apply the world-space force at the resolved mount/muzzle point so MuJoCo
produces both translation and torque. The runtime uses `mj_applyFT` to add the
Cartesian force/torque to `qfrc_applied` before `mj_step`, then clears the
one-step application. The station's real modeled mass and inertia remain in
the same `mjModel`, so recoil can disturb balance and the locomotion controller
must recover naturally.

For a dynamic hit body, apply a separately clamped impact impulse at the hit
point. Static world geometry receives only a hit event. Scenario- or
animation-authoritative kinematic entities remain under their declared
authority; the hit event may affect later game rules, but it cannot silently
take over their transform.

Rejected shots apply no recoil, spawn no projectile, and emit a correlated
rejection reason.

## Unreal presentation and asset pipeline

The station is a rigid articulated asset, not a skinned character:

```text
Tripo or authored concept mesh
  -> Blender cleanup, scale, UV/PBR, separate rigid links and set pivots
  -> UE payload visual projection
  -> MuJoCo body/joint snapshots drive UE components
```

Required rigid pieces are `station_base`, `yaw_link`, `pitch_link`, and
`barrel_link`. Blender bones are optional; separate rigid meshes with exact
pivots are simpler for this payload. UE components use `NoCollision` and
`SimulatePhysics=false` because MuJoCo owns collision and dynamics.

Tripo output is visual source material only. Authoritative MuJoCo collision is
built from validated simplified primitives/convex meshes with measured scale,
mass, inertia, and mount transform. A photoreal mesh must never become an
unreviewed physics mesh.

UE consumes ballistic events to render:

- muzzle flash and short-lived light;
- pooled tracer segments interpolated in simulation time;
- impact decal/particle selected by semantic material class;
- audio and camera shake;
- Tactical reticle, payload-ready state, and rejection feedback.

These effects may be dropped or reduced for performance without changing shot,
hit, recoil, recording, or qualification truth.

## State and recording

Every weapon event carries:

```text
run_id
session_id
boot_id
model_generation
reset_generation
sequence
sim_time_ns
robot_instance_id
payload_instance_id
shot_id
```

The recorder stores accepted/rejected intents, shot origin and velocity,
trajectory terminal state, hit stable IDs, recoil application, and matching
truth snapshots. Replay drives UE effects from recorded events and never
recomputes a different hit.

Reset increments `reset_generation`, clears active projectiles and trigger
state, invalidates cached dense indexes, and requires payload rebind before fire
can become ready again.

## TDD delivery gates

### Gate W0 — contract freeze

- Add ADR for PayloadPackage and physics authority.
- Add immutable schemas for PayloadPackage and payload visual projection.
- Add session/physics/visual v2 schemas; v1 fixtures remain byte-compatible.
- Add fail-closed resolver tests for missing mounts, duplicate payload IDs,
  wrong parent role, unknown frames, asset digest drift, and global MuJoCo
  option ownership.

### Gate W1 — mounted inert payload

- Add `payload_top` to Thunder MJCF/package with measured transform.
- Add `fictional_rws@1.0.0` with simplified MuJoCo mass/inertia and placeholder
  UE rigid-link visuals.
- Compose World + Thunder + payload into one `mjModel`.
- Prove payload mass changes robot dynamics and all payload elements have stable
  descriptors.
- Prove UE binds all payload links from `visual.plan.v2` with zero UE collision.

### Gate W2 — input and one deterministic shot

- Add Tactical left-button down/up tests around focus, UI consumption, mode
  exit, pause, reset, and teardown.
- Add distinct weapon-intent serialization/parser/ACK tests with current
  identity and generation contracts.
- Implement one semi-automatic shot, gravity/drag/wind integration, nearest
  swept hit, expiration, and deterministic seeded dispersion.
- Prove identical bundle, seed, command, and snapshots produce identical event
  bytes.

### Gate W3 — recoil and visual effects

- Add one-step/multi-step force-curve tests and no-recoil-on-rejection tests.
- Prove the recoil impulse changes authoritative robot body velocity/orientation
  and the quadruped remains under its locomotion controller.
- Add pooled UE muzzle/tracer/impact effects and event-generation filtering.
- Keep the current RGB/depth performance gate unchanged.

### Gate W4 — same-run playable evidence

One run must prove:

```text
Tactical focus
-> left mouse press/release
-> successful-send evidence
-> accepted weapon intent
-> one shot_id
-> deterministic trajectory
-> one authoritative hit or expiry
-> recoil in same-generation truth snapshots
-> UE visual event
-> recorder/replay equality
-> natural CLOSED cleanup
```

No cross-run assembly of evidence is accepted.

## Explicitly deferred

- Real firearm names, calibers, ammunition tables, or manufacturing details.
- Automatic fire, magazines, reloads, inventory, upgrades, and weapon economy.
- Damage/health/destruction, penetration, ricochet, fragmentation, and gore.
- Multiplayer prediction, lag compensation, anti-cheat, or remote field use.
- AI aiming or autonomous target engagement.
- A new UE gameplay module, Enhanced Input migration, or Chaos projectile
  authority.

Those features require separate product decisions after the first vertical
slice passes.

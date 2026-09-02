# FactoryPark_HF 1.0

FactoryPark_HF is a deterministic 220 × 180 m modern industrial site for
ThunderV4 navigation work. Its expanded layout is the static-site authority;
MuJoCo geometry, Blender/Unreal authoring inputs, semantic entities, and the
SVG site plan are derived from the same document.

## Site plan

- South entrance: an 18 m clear gate, open leaves, gate canopy, posts, and a
  guardhouse. ThunderV4 spawns on the level entry boulevard at
  `(0, -76, 0)`.
- Roads: a 10 m two-way ring road, 12 m central cross street and boulevard,
  an 8 m loading apron, lane markings, curbs, and three physical speed bumps.
- Production: a 70 × 40 × 12 m main factory, roof monitor, administration
  office, and a 60 × 32 × 10 m warehouse.
- Logistics: three loading docks with physical ramps, a concrete container
  yard with seven containers (including one stacked unit), and a 3.5 m
  forklift loop.
- Utilities: three storage tanks inside a 35 × 35 m concrete containment
  bund, an elevated pipe rack, and a terrain-baked eastern drainage channel
  with a physical road grate.
- Robot tests: a northern 28 × 14 m rough gravel pad, a 4 m-wide acceptance
  loop, six semantic checkpoints, parking spaces and stops, streetlights,
  perimeter fencing, and a 1.8 m pedestrian route.

## Coordinate and geometry contract

`expanded-layout.json` uses MuJoCo right-handed, Z-up metres. Every
`position_m` is the geometric centre. A box uses full dimensions in `size_m`;
a cylinder uses `radius_m` and `half_height_m`. `yaw_deg` rotates around +Z,
and loading-ramp boxes additionally carry `pitch_deg`.

The terrain OBJ deliberately follows the existing OpenField/Unreal import
contract: Unreal left-handed, Z-up centimetres, with source-to-Unreal axis
mapping `(x, y, z) -> (x, -y, z)`. To recover Blender/MuJoCo source space from
an OBJ vertex, use `(x_cm / 100, -y_cm / 100, z_cm / 100)`.

## PhysicsShared and VisualOnly

- `collision: true` is PhysicsShared: the object is emitted as a static
  MuJoCo geom and must retain matching dimensions in Blender/Unreal.
- `visual_only: true` is non-colliding decoration or marking. The generator
  never combines it with `collision: true`.
- `collision: false, visual_only: false` denotes a physical terrain feature
  already baked into the shared heightfield, such as the drainage channel.
- MuJoCo remains physics truth. Dynamic pedestrian and forklift routes are
  intent only until a scenario-authority executor materializes them.

## Regeneration

From the repository root on this workstation:

```powershell
.\.venv\Scripts\python.exe -m sim.tools.worlds.factory_park_hf.generate `
  --repo-root . --seed 20260808
```

The generator uses only the Python standard library and emits canonical JSON,
SHA-256 records, provenance, same-source heightfields, MJCF, OBJ, SVG, the
WorldPackage, Unreal recipe, and ThunderV4 SessionSpec.

## Direct element production

Static factory elements can be authored as strict JSON batches and passed to
the same world generator. The generator resolves the support surface, computes
the geometric-centre Z value, assigns a stable ID, enforces the catalog's
PhysicsShared/VisualOnly policy, checks footprint containment and robot spawn
clearance, and then regenerates every MuJoCo, Blender, Unreal, semantic, and
digest-bound source artifact.

The canonical map includes
`element_batches/operations_safety.v1.json`, which adds 19 safety and logistics
fixtures. Additional batches are appended with a repeatable flag:

```powershell
.\.venv\Scripts\python.exe -m sim.tools.worlds.factory_park_hf.generate `
  --repo-root . --seed 20260808 `
  --element-batch sim/packages/worlds/factory_park_hf/my_factory_elements.json `
  --force-overwrite
```

The generator compares every existing output before writing. It accepts
byte-identical reruns, but refuses changed files unless `--force-overwrite` is
explicitly supplied after review. A custom batch changes the canonical
FactoryPark_HF layout digest while retaining the package identity; use a new
world package/version when the result must coexist as a separately supported
map. Use `--no-default-elements` to build without the canonical element batch.
The supported v1 types are `equipment_cabinet`, `fire_cabinet`,
`industrial_drum`, `jersey_barrier`, `lane_marker`, `pallet_stack`,
`safety_bollard`, `safety_sign`, `traffic_cone`, and `wheel_stop`. The input
schema is `sim/contracts/schemas/factory-park.v1.json`.

This is a structured-text interface. Free-form natural-language prompts should
compile to this batch schema rather than writing Blender or Unreal actors
directly.

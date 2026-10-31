# Interactive industrial asset slice — 2026-10-17

## Current revision: repaired wheel surfaces (V2)

The user's rotation screenshots revealed torn surfaces under the front and rear
wheels. A texture-free clay render reproduced the same spikes: the original AI
mesh is incomplete where the wheel arch occludes the tire and has fused adjacent
geometry. Keeping only large connected components did not repair this defect.

The current files in `build/tripo/truck-articulation-20261017/articulated/` now
replace those four damaged wheel groups with closed local tire/rim geometry:
two front tires and four rear tires. The original chassis mesh and its three 4K
maps are unchanged. Wheels use separate rubber, steel and dark-metal PBR
materials, with geometric tread; they do not retain the damaged baked wheel atlas.

- Previous wheel boundary-edge counts: 72 / 74 / 95 / 116.
- Rebuilt wheels: zero boundary and zero non-manifold edges for all four groups.
- GLB: 5 meshes, 4 materials, 139,265 triangles; UVs and tangents on every primitive.
- Four-wheel roll and two-front-pivot steer channels remain functional.
- Eight rotation samples over 0–315 degrees and +/-20-degree steering renders are
  in `wheel-repair/v2/`; no protruding fragments were visible in the inspected views.
- `articulated/repair-report.json` preserves measured before/after results.
- Original articulated V1 files are recoverable in `wheel-repair/v1/`.
- Original Tripo source GLB remains unchanged. No new Tripo task or credit spend.

Reproduction: run `repair_wheels.py` in headless Blender, then
`node finalize.cjs <lane>/wheel-repair/v2` from the lane directory. The repair
reads the preserved V1 `.blend`; do not regenerate V1 over the active V2 files.
The source chassis still emits its existing texture-sampler selection warning;
the wheel tangent warnings were resolved by triangulation and non-collapsed UVs.
This repair does not imply watertight chassis geometry, validated collision,
realistic vehicle dynamics, or completed AAA material quality.

## Original V1 delivery (superseded wheel geometry)

The generated truck is now a **visual articulation draft**: one chassis mesh,
four wheel-group meshes, and two front steering pivots. Rear dual tires are
grouped per side. Original UVs and three embedded PBR maps are retained.

The partition retains all 93,251 source triangles exactly once. A playback
inspection caught small rear-wheel cut fragments; 89 faces were reassigned
to the chassis. The four wheel rotation channels and two steering channels
were verified from the exported GLB data, then merged into one clip per mode.
The source single-mesh GLB was not modified.

Artifacts are under `build/tripo/truck-articulation-20261017/`:

- `articulated/truck-parts.glb`: independent rigid visual parts, no animation.
- `articulated/truck-articulated.blend`: editable parts, pivots and roll animation.
- `articulated/truck-roll-preview.glb`: four wheels, 0.5 rev/s demonstration.
- `articulated/truck-steer-preview.glb`: front pivot yaw, +/-20 degrees.
- `articulated/articulation.json`: dimensions, pivots, partition counts and limitations.
- `articulated/animation-checks.json`: exported-channel verification.
- `index.html` / `serve.cjs`: local preview with mode, play/pause, speed and scrub controls.

Observed browser checks: roll and steer modes loaded one combined animation;
forklift mode loaded with animation controls disabled; pause/play changed the
visible state; at 1.5x playback, scrubbing to 1.00s stayed at 1.00s after fixing
the viewer's scaled-time setter interaction. Actual screenshots are saved as
`truck-interactive-preview.png` and `forklift-preview.png` beside `index.html`.
The headless Blender processes exited normally. No UE build or simulation was run.

To reopen the workbench from the repository root:

```powershell
node build/tripo/truck-articulation-20261017/serve.cjs
```

Open the localhost URL printed by this command. The model-viewer module is
loaded from Google's public CDN, so this preview currently needs network access.

The 7m overall length is a **working visual scale assumption**, not a measured
vehicle specification. Internal cut seams still need close-up repair. Do not
promote this draft as watertight, collision-ready, or AAA-quality.

## Ownership when integrating with the simulation

| Responsibility | Owner | Current slice |
| --- | --- | --- |
| Meshes, UVs, material channels, named pivots | Authored asset | Truck draft delivered |
| Chassis pose, wheel spin, steering and physical contacts | MuJoCo session physics | Not implemented for this truck |
| Drive, brake, steering and emergency-stop intent | Existing session/controller route | Not wired for this truck |
| Mesh transforms, lighting and presentation | UE visual runtime | Not imported/wired this turn |
| Spawn and scripted traffic objectives | Scenario owner | Not implemented for this truck |

Rigid wheels do not require character-style skeletal rigging. This hierarchy is
enough for independent visual transforms:

```text
Vehicle
  Chassis
  Steer_FL -> Wheel_FL
  Steer_FR -> Wheel_FR
  Wheel_RL
  Wheel_RR
```

Blender authoring axes are +X forward, +Y left, +Z up; GLB export converts to
glTF Y-up. An integration must convert axes explicitly instead of treating
raw GLB node coordinates as MuJoCo or UE coordinates.

## Smallest next integration

1. Finish wheel cut seams, confirm scale and axle measurements, and inspect
   near-field appearance. Keep the visual draft out of the production map.
2. Author a separate low-complexity collision model, mass/inertia and wheel
   joints. For a background traffic vehicle choose scripted kinematics;
   for a genuinely drivable/robot-contact vehicle use dynamic MuJoCo bodies
   with wheel contact and drive/brake actuation. Do not silently equate them.
3. Bind stable part names to the session model descriptor. UE follows body
   poses and wheel/steering joint angles from MuJoCo; UE Chaos must not also
   simulate the same vehicle.
4. Validate spawn, no-input rest, forward/backward motion, braking, turning,
   robot/obstacle contact and reset. These are future tests, not passed results.

For a purely visual route-following prop, accumulated distance divided by
wheel radius gives a rolling angle. For physical driving, use the actual wheel
joint angle: the distance/radius approximation ignores slip. Front wheel
steering must eventually use the intended vehicle geometry; this preview's
equal +/-20-degree angles are not calibrated Ackermann steering.

## Additional asset produced

One textured forklift was generated with Tripo v3.1, detailed geometry and
texture, PBR enabled, face limit 100,000, seeds 20261017.

- Task: `229a6e7d-0c09-4ea5-8ae9-b5441ba21554` — success.
- File: `build/tripo/factory-forklift-20261017/tripo-out/factory-forklift-v1-229a6e7d/model.glb`.
- 88,323 triangles; one mesh/material; color, metallic/roughness and normal
  maps are bound in the GLB.
- Cost: 60 credits; afterward the account reported 2445 available, 0 frozen.
- Status: source candidate only, not articulated, not imported into UE.

Forklift follow-up should separate chassis, steering/wheel groups, mast,
carriage and forks. Mast lift is a prismatic joint, not a rotating bone;
tilt and load-contact behavior require their own declared joints/physics.

No robot package, production map, collision authority, UE C++ code or runtime
control contract was changed by this asset-preparation slice.

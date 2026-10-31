# Factory equipment candidates — 2026-10-17

This batch adds two textured **source candidates** to the existing local asset
preview. It does not replace production map assets, create physics, or enable
opening doors or spinning fan blades.

## Delivered assets

| Asset | Tripo task | Triangles | GLB |
| --- | --- | ---: | --- |
| Outdoor dual-fan HVAC unit | `9c87fcde-824f-45dc-8113-74b832d08e52` | 87,424 | `build/tripo/factory-hvac-20261017/tripo-out/factory-hvac-v1-9c87fcde/model.glb` |
| Floorstanding electrical cabinet | `b08679ad-7ead-409a-afdc-d1e8bfcc32f6` | 76,934 | `build/tripo/factory-cabinet-20261017/tripo-out/factory-cabinet-v1-b08679ad/model.glb` |

Each GLB contains one mesh and one material, with UVs, normals and three embedded
4096x4096 maps: base color, metallic/roughness and OpenGL normal. No animation or
tangent attribute is present. Map dimensions were read from the extracted images;
material bindings and geometry counts are in each output folder's `inspection.json`.
These checks do not certify topology, collision, physical scale or UE appearance.

The HVAC service panels, coil grilles and two fan guards survived generation;
the cabinet keeps two door seams, handles, vents and a warning pictogram. Small
manufacturing edges and grille details are softened by generation and still
need authored cleanup for close-up use. Single-view reconstruction does not
guarantee unseen surfaces match the reference design.

## Generation and cost

Original references were generated with the built-in image generation tool:

- `docs/design/assets/factory-replacements-20261017/hvac-unit-v1.png`
- `docs/design/assets/factory-replacements-20261017/electrical-cabinet-v1.png`
- Full prompts: `docs/design/assets/factory-replacements-20261017/equipment-prompts.md`.

Tripo CLI 0.7.0, domestic `default` profile, model `v3.1-20260211`, one candidate
per asset, texture and PBR enabled, detailed texture and geometry, face limit
100,000, model and texture seeds 20261017. Both dry-runs passed before upload.
Both tasks succeeded on the first submission; there were no paid retries.

- HVAC: 60 credits.
- Cabinet: 60 credits.
- Total: 120 credits; available balance 2445 -> 2325, frozen 0.

The generated GLBs are normalized, not metre-calibrated. Dimensions described
in the prompts are design intent only. No license or distribution qualification
was performed this turn; keep these candidates outside the qualified catalogue.

## Preview and next practical work

The existing `build/tripo/truck-articulation-20261017/serve.cjs` serves both new
models alongside the repaired truck and forklift. Start it with Node, optionally
passing port `63426`. The preview needs access to its existing model-viewer CDN.
Static assets disable animation controls and reset the displayed time to zero.

Observed browser checks: both new GLBs loaded with their materials and zero
animation clips; switching to truck roll enabled its one animation and advanced
the displayed time, then switching back to HVAC disabled controls and restored
0.00s. The preview viewport was inset so the tall cabinet is not hidden behind
the bottom controls. Actual browser screenshots are saved under
`build/tripo/factory-equipment-20261017/` as `hvac-browser-preview.png` and
`cabinet-browser-preview.png`. The preview remains served on localhost:63426.
Node syntax checks passed for the server, inspection script and inline preview JS.

Next priority is **make selected assets usable, not generate an unlimited pile**:

1. Cabinet: straighten key hard edges, separate both doors and author hinge
   pivots; build an interior only if opening the doors is part of the task.
2. HVAC: keep guards stationary, create separate closed fan rotors below them;
   do not rotate the entire fan grille as a shortcut.
3. Confirm metre scale, add a simple separate collision proxy, validate UE import
   tangents and normal-map convention, then place a small equipment yard in the
   factory map. MuJoCo remains the robot-contact physics authority.
4. Bind inspection targets and fault indicators through the existing scenario
   and sensor contracts; a visual warning decal is not a simulated fault.

No UE/UBT/UAT/MuJoCo process or production simulation contract was changed for
this generation batch. Original truck and forklift model files were preserved.

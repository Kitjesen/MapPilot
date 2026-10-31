# FactoryPark visual pass — 2026-10-17

## Delivered, and quality boundary

This is the first photographic-material/daylight pass, **not an AAA-quality
completion claim**. Geometry remains the current blockout: trucks, tree crowns,
containers and several facade details are visibly simplified. Repeated road
cracks, flat roof silhouettes and the surrounding empty landscape still need art.

The actual runtime level was updated in place:
`/Game/RobotSim/Generated/FactoryParkHF/Maps/FactoryPark_HF_Instanced_9d314c86ef83ea06`.
The selector image `Plugins/LingTuSim/Resources/Selection/world-factory_park_hf.png`
now uses an actual Unreal overview of that saved level, not the old Blender image.

Four Poly Haven CC0 surface sets supply 2K base color, roughness and DirectX normals.
Raw source files and exact URLs are in
`sim/packages/worlds/factory_park_hf/visual/surfaces/`.
Unreal assets live under
`/Game/RobotSim/Worlds/FactoryParkHF/Materials/Photographic`.
Existing FactoryPark material instances reference the new material; source images
are unmodified. Metal-panel color is desaturated/tinted in the material graph.

Final lighting is 85,000 lux directional daylight, 6,000 K sun temperature,
3.0 skylight intensity and manual EV100 12.5 exposure with 6,500 K white balance.
The authoring camera captures use high-quality render settings. No playable FPS,
sensor-rate or packaged-build claim follows from these editor images.

## Reproduce and verify

```powershell
.venv/Scripts/python.exe -m sim.tools.worlds.factory_park_hf.lookdev
.venv/Scripts/python.exe -m sim.tools.worlds.factory_park_hf.run_lookdev --apply
# Subsequent daylight iteration without reimporting surfaces:
.venv/Scripts/python.exe -m sim.tools.worlds.factory_park_hf.run_lookdev --lighting-only
# Fresh-process inspection/capture, no map or material writes:
.venv/Scripts/python.exe -m sim.tools.worlds.factory_park_hf.run_lookdev
```

- Initial actual-map baseline: `build/factory-park-lookdev/20261017-161550/`.
- First complete material/map save-reload pass: `20261017-164112`.
- Final daylight was saved by `20261017-164643`; its captures completed, but the
  editor returned Windows access-violation exit status during shutdown. This run
  is **not** a clean-process success and its original evidence remains intact.
- Fresh-process final reload: `build/factory-park-lookdev/20261017-165256/`.
  Exit 0, three screenshots, 19 unique photographic material bindings read back
  with all three texture channels. The geometry check retained 263 static-mesh
  components with unchanged root transforms, mesh references, collision settings
  and instance counts. This is not a new physics-contact qualification.
- No MuJoCo or build process was started. Final UE/build/MuJoCo process scan was
  empty. Unrelated Blender sessions were not operated or terminated.
- Two surface-routing regression tests pass; Ruff passes. The pytest invocation
  with cacheprovider disabled emitted the existing `cache_dir` config warning.

Binary backups exist in each write run's `backup/` directory. To recover the
pre-pass material set, use each asset's **earliest** backup: early attempts could
save some instances before encountering an unsupported property or non-rendered
semantic record. Do not treat the last backup as necessarily the original.
The original selector image is `20261017-165256/selector-preview-before.png`.
The updated two-step selector was rendered and visually checked afterward:
`build/unreal-tests/robot-map-20261017-170115/result.json`, passed, exit 0, no
selection intent and no physics process. Its map image matches the final Unreal
overview rather than the superseded Blender thumbnail.

## Next asset-quality work

1. Replace the blockout trucks/trees with license-cleared detailed visual assets;
   preserve canonical placement and existing physics authority. Audit visual vs
   sensor/physics extents instead of silently treating render detail as collision.
2. Add authored facade modules, loading-bay doors, signage, gutters and equipment
   detail inside a coherent scale/material palette.
3. Break obvious road tiling with restrained wear masks, curb dirt and local
   repairs; improve the perimeter/vista instead of leaving an empty backdrop.
4. Re-test a real driving route and the render-sensor workload before calling the
   upgraded map performance-qualified.

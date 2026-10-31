# Local asset library

The current per-asset mechanics and acceptance ledger is [ASSETS.md](ASSETS.md).
Twelve entries are now archived, including one eight-module industrial pipe set.
The latest mountain geometry review is in
`build/scene-previews/mountain-continuous-20261018/`; the older
`build/mountain-outpost-hf-20261018/` preview remains preserved. The new review
fixes grounded trail/asset geometry but is **not visually qualified**: terrain
detail, understory, lighting and real-time tree LODs remain unfinished. See
[the scene expansion plan](../../docs/design/scene-experience-expansion.md)
for actual results and [SOURCES.md](SOURCES.md) for source records.
The barrier, service door and new CC0 barrel include standalone MuJoCo proxies;
all entries still need live UE visual/physics integration acceptance.
No asset in this intake is claimed to have passed live UE physical interaction.

See [WORKBENCH.md](WORKBENCH.md) for the installed authoring tool, Windows fixes,
and actual MCP motion/export workflow; [SOURCES.md](SOURCES.md) lists licensed
sources. The Fab Factory Environment Collection is now acquired, downloaded on D,
and selected by the user as a product background after a real UE 5.8.1 preview.
See the current decision at the top of [SOURCES.md](SOURCES.md). The approved
reference still contains editor overlays: it is not a production UI resource,
and the running product background has not yet been replaced.

`industrial/` contains versioned visual source assets for review. It is not a
RobotPackage, WorldPackage or a second runtime catalog. Models here do not grant
simulation, collision, driving, or distribution qualification.

Each asset version includes its actual GLB, preview, generation reference and
an `asset.json` source/status record. Textures remain embedded in the GLB.
The repaired truck additionally retains its editable Blender file and two
visual animation variants. Original generation/build outputs are not deleted.

`industrial/catalog.json` uses the existing
`lingtu.sim.game-asset-review-catalog.v1` contract, compiled by
`sim.tools.game_asset_review_catalog`. Its pinned file hashes are required by
that existing contract, not a new integrity framework. The corresponding
`review-spec.json` can be recompiled through the existing compiler. Change an
asset into a new version rather than silently replacing its pinned files.

## Open the library

From the repository root:

```powershell
node sim/tools/assets/library_preview/server.cjs 63426
```

Open http://127.0.0.1:63426/. This is a read-only local browser for the archived
catalog, with category/search, thumbnails, a real GLB viewport, source info and
downloads. It uses the existing model-viewer CDN and needs network access for
that module. No UE, MuJoCo or remote service is launched by this viewer.

The UE selection catalog compiler accepts this catalog through:

```text
--asset-review-catalog sim/library/industrial/catalog.json
```

This has been accepted by the production selection-catalog loader. It does not
mean the running UE game's old embedded catalog or hard-coded thumbnail list
has been updated. UE mesh import, thumbnail wiring and in-game verification
remain separate work; this batch did not compile or launch Unreal.

## 2026-10-17 intake

Seven archived assets: box truck (repaired wheels), forklift, HVAC, electrical
cabinet, fire hydrant, waste container and pipe valve. Categories are vehicle,
equipment, fire safety and environment. All are `quarantined` review candidates;
UE `NoCollision` and MuJoCo physics authority remain unchanged.

The three newest source tasks succeeded once each, at 60 credits per task:

- Fire hydrant: `8b1cb696-fb9d-44b7-9a61-813dcb01b351`, 58,246 triangles.
- Waste container: `a59ef010-f7ef-47ea-b34f-c9f2b906ef2b`, 51,258 triangles.
- Pipe valve: `9d3d3a6b-d871-43ce-ace0-f4bab49a33c8`, 57,600 triangles.

All three have a UV-mapped mesh and embedded color, metallic/roughness and normal
maps. They are not mechanically articulated. Tripo v3.1, PBR, detailed texture
and geometry, face limit 60,000, model/texture seeds 20261017. Total new spend:
180 credits; balance 2325 -> 2145, frozen 0. Reference prompts are in
`docs/design/assets/factory-replacements-20261017/yard-props-prompts.md`.

The one-time intake recipe is preserved in
`build/tripo/library-intake-20261017/`. It copied instead of moving and refused
to overwrite existing versions. Current library operation needs none of those
build paths; `source_directory` fields are provenance, not runtime dependencies.

Checks: two Node library/server tests, JS syntax and production review-loader
acceptance passed. Browser verification is separate from UE import qualification.

Observed browser verification: all seven cards present; Equipment filter shows
three; searching `配电` narrows to the single electrical cabinet; the new hydrant
and pipe-valve GLBs render with textures. The repaired truck retains its static,
roll and steering variants. Actual library screenshots are in
`build/tripo/library-intake-20261017/`.

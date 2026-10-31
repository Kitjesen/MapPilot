# RobotSimUE project context

Verified 2026-09-24 for the 3DGS rendering slice; refactor alignment updated 2026-10-12.

Current refactor contract: `config/runtime/` -> `lingtu.assembly` -> one RunPlan ->
ProductControl. Packages live only under `sim/packages/`; SessionSpecs under
`sim/sessions/`; Python simulation tests under `tests/sim/`. UE consumes compiled
plans sharing `session_id`; Product execution uses `product_session_id`. Do not
revive old digest-only Bundles or the retired `config/runtime_graph/` paths.
The repository-wide `.agents/ue-project-context.md` records the current boundary.

- Project: `RobotSimUE.uproject`, a Win64 Unreal visual host for LingTu simulation. Its `EngineAssociation` is empty; the installed engine used for local work is UE 5.8.1 (`D:\Program Files\Epic Games\UE_5.8`).
- Targets: `RobotSimUE` and `RobotSimUEEditor`. The host module depends on Core, CoreUObject, and Engine.
- Enabled project plugins: `LingTuSim`, `LCC4Unreal` (Win64), `PythonScriptPlugin`, `EditorScriptingUtilities`, `ModelContextProtocol`, `AllToolsets`, and `Terminal`. `Tripo3DUEBridge` is currently disabled because its source build requires an absent IXWebSocket library; its assets remain intact.
- `LingTuSim` contains Runtime, Session, Visual, Sensors, and UI modules. MuJoCo owns physics and the simulation clock; UE applies immutable snapshots for presentation and render sensors. UE visuals must not become physics truth.
- Existing maps are under `/Game/RobotSim/Maps/`, including `ThunderV4_RuntimePreview`, `OpenField_HF`, `Forest_HF_2km`, and `FactoryPark_HF`. The project has no configured default map; launchers and scripts choose a map explicitly.
- The existing Tripo bridge imports ordinary PBR meshes. It is not a Gaussian Splat renderer and records no collision authority.
- Before this slice, the project had no 3DGS plugin or GS scene asset. The local PoC now enables XGRIDS LCC4Unreal 3.4.0 in `RobotSimUE.uproject`. `Scripts/GSPOC/stage_xgrids_poc.ps1` mounts the vendor plugin and official sample data from outside the repository; both junctions are Git-ignored.
- The generated `/Game/RobotSim/Maps/GSPOC_XGrids_LCC2` map renders the official LCC2 splats and contains a separate, invisible collision proxy exported from that LCC2's `data/mesh/*.ply` through Blender. The proxy uses `BlockAll` and complex-as-simple collision.
- UE 5.8.1 locally reloaded the saved map with valid splat data. `Saved/GSPOC/persisted_evidence.json` records a ray hit that disappears when the proxy is ignored. This is a UE visual/interaction check; Thunder training physics remains with MuJoCo.

Confirmed: the plugin's 5.8 runtime loads and renders on the local 8 GB RTX 5060 Laptop GPU. The official v3.4 documentation describes LCC2 `data/mesh/*.ply` collision fallback, but the tested Win64 binary repeatedly searched for `collision.lci` and returned no valid internal collision on two official LCC2 samples. The explicit UE proxy is the working path. Packaging, sustained frame rate, our captured site data, and dynamic relighting remain untested.

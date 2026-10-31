# Light robot showroom

The selected visual reference is the light architectural hall: off-white
surfaces, daylight, black typography and a blue selection accent. This is a
presentation change, not a change to robot physics or qualification.

## Implementation

- `SLingTuSimRobotMapPicker` keeps the existing category/search/model and the
  Robot -> World -> Review selection model. It now uses a light three-column
  composition: model list, concise information, robot display.
- `SLingTuSimShowroom` owns an isolated game-preview `FPreviewScene` with no physics scene,
  navigation or gameplay controller. A real StaticMesh, architectural geometry
  and lighting are captured together. The model is floor-aligned from its bounds;
  its framing scale never writes back to the source or physics package.
- The display's silver-material slots use a light coated-alloy finish. Rubber,
  sensors and custom-color slots retain their imported materials. Overrides are
  cleared on every model switch; the underlying assets remain unchanged.
- The scene captures 40 warmup frames, or 24 after a robot change, then stops
  rendering into its retained render target. This is a static pose preview, not
  locomotion and not an animated simulation.
- Editor Game loads shaders lazily: completed shader maps are finalized before
  convergence is counted. QA waits for this readiness, and captures off-screen
  so desktop input cannot change the chosen model during the capture sequence.
- Clicking the model still opens the existing isolated-body image viewer.
  Category, search, next/back and modal focus remain native Slate controls.
- Only exact compiled session options authorize entry into a runnable scene.
  Imported models without controllers remain explicitly labeled as such.

## Content and rebuilding

Runtime meshes and materials live in
`sim/runtime/visual/RobotSimUE/Content/RobotSim/FrontEnd/Showroom/`.
That project-content directory is included in the packaging cook list. A packaged
Windows build has not been validated by this change.

Offline source geometry is converted by
`sim/tools/assets/export_robot_showroom.py --force`. It consumes the existing
normalized source render plans, preserves face material assignments and real
dimensions, joins each visual body, and limits each display mesh to approximately
600,000 faces. Original robot geometry, kinematics and collision shapes are not
overwritten. Intermediate FBX and the provenance manifest are under
`build/robot-showroom/`.

The editor-only script
`sim/runtime/visual/RobotSimUE/Scripts/import_robot_showroom.py` imports those FBXs,
assigns materials and checks imported height against meters-to-centimeters
conversion. It uses an explicit FBX factory and the installed engine's material
pin names. It produces `build/robot-showroom/import-result.json`.

The local library contains 29 classified choices; six DAE-based models currently
lack an executable converter, and are not replaced with another robot. The other
23 have geometry export inputs. Import results and the InstalledAssets automation
test, not the availability of PNG previews, establish actual mesh availability.

## Verification scope

- Python regression covers Blender clearing polygon material assignments when
  material slots are rebuilt.
- UE Automation covers identifier mapping, installed meshes/materials and the
  existing picker, search, focus, controls and player settings.
- Front-end screenshot automation captures the real selector, humanoid category,
  map and review steps without starting MuJoCo or writing a selection intent.
- Architecture, daylight balance, robot grounding and text overlap require
  reviewing those actual rendered screenshots; a build alone is insufficient.

Asset import and display do not qualify these models for driving. The six missing
conversions, richer surface detail, model articulation/orbit controls and packaged
runtime verification remain separate work.

## Verified local result — 2026-10-16

- 23/23 StaticMesh assets imported; actual heights and material sections passed
  the importer checks (`build/robot-showroom/import-result.json`). Early failed
  import logs remain in the timestamped import directories; they are not success
  evidence. Source-mesh near-zero binormal warnings remain an asset-quality issue.
- Focused UI/player-settings/control Automation: 92 Success, 0 failures, 0
  warnings in `build/unreal-tests/light-showroom-final-20261016/report/index.json`.
- Final lighting/display-finish build: `ubt-light-showroom-20261016-i.log`,
  Result: Succeeded.
- Final actual off-screen Editor Game capture:
  `build/unreal-tests/robot-map-20261016-234644/result.json`, passed. Thunder and G1
  capture targets are both nonblank; robot, humanoid, map and review screens were
  visually inspected. No MuJoCo was started and no launch intent was written.
- Images:
  `sim/runtime/visual/RobotSimUE/Saved/Automation/LingTuSimUI/FrontEnd/robot-map-20261016-234644/`.

This is a usable first presentation pass, not a pixel-identical match to the
reference. The outdoor mountain vista is not built, the hall surfaces are still
sparse, and several source robots retain visibly faceted or incomplete geometry.
No claim of photorealistic or locomotion qualification is made.

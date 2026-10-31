# Robot library import — source-confirmed intake

The confirmed source is `https://github.com/fan-ziqi/robot_lab` at revision
`500399ed75f510aeaff28705a8ce736c514dbec3`, VERSION `2.3.2`. The source-only
intake is complete for every robot model directory present at that revision.

The local `D:/inovxio/.remote_robot_lab` is an incomplete Thunder training tree:
it has task configurations but no model meshes, URDF/MJCF/USD assets, upstream
Git metadata or repository license. It is not a source for a complete import.

A shallow partial clone is retained at
`build/robot-library-import-20261016/upstream/robot_lab`. Its sparse checkout now
includes `source/robot_lab/data/Robots`, the RobotLab asset configurations, task
configurations used to identify external families, and the repository license
notices. The robot subtree contains 573 files and 469,903,522 bytes across 22
model directories. The inactive LingTu copy is at `sim/.imports/robot_lab`; it
contains the same robot subtree plus `LICENSE`, `.github/LICENSE_HEADER.txt`, and
the two generated inventory documents.

## Inventory at that revision

The README lists 24 robot families:

- Quadruped: ANYmal D, Unitree Go2/B2/A1, DeepRobotics Lite3, Zsibot ZSL1,
  MagicLab MagicDog, Agibot D1.
- Wheeled: Unitree Go2W/B2W, DeepRobotics M20, DDTRobot Tita, Zsibot ZSL1W,
  MagicLab MagicDog-W.
- Humanoid: Unitree G1/H1, FFTAI GR1T1/GR1T2, Booster T1, RobotEra Xbot,
  OpenLoong Loong, RoboParty ATOM01, MagicLab MagicBot-Gen1/MagicBot-Z1.

The Git tree contains 22 corresponding local model directories under
`source/robot_lab/data/Robots/`, with URDF and mesh assets (STL/OBJ and associated
materials, plus DAE for several Unitree models). They contain 31 URDF variants;
24 are referenced by RobotLab asset configuration. The presentation entry for
each model uses an explicit primary URDF rather than selecting by file ordering.
ANYmal D is referenced through `isaaclab_assets.robots.anymal.ANYMAL_D_CFG`.
H1 likewise has no model directory in this RobotLab tree; its task imports
`isaaclab_assets.robots.unitree.H1_MINIMAL_CFG` with an explicit cloud-assets
comment. Their separate public model sources are now recorded in
[`robot-library-external.md`](robot-library-external.md): official ANYmal D
simple description and Menagerie H1. These are not claimed to be the exact
Isaac cloud USD assets.
The root repository declares Apache-2.0 and the copied notices preserve that
declaration. No model-specific license file is present beside any of the 22
model directories, so downstream redistribution still needs model-origin
license review; the inventory does not overstate the root declaration as an
independent license proof for every third-party mesh.

## Machine inventories

`python -m sim.tools.assets.import_robot_lab_sources` performs the sparse source
acquisition, checks the confirmed revision, copies through `SourceIntake`, and
writes byte-stable JSON without creating an active RobotPackage. Re-running the
same revision validates source paths and sizes and reuses the existing intake;
`--replace` is required when the owned destination differs.

- `sim/.imports/robot_lab/source-library.json` and
  `build/robot-library-import-20261016/robot_lab-source-inventory.json` use schema
  `lingtu.sim.robot-source-library.v1`. They record repository revision and
  license, all source files, all 22 model directories, all 31 URDF entries,
  configured-primary evidence, and every unique URDF mesh/texture reference.
- `sim/.imports/robot_lab/robot-library.presentation.json` is the intentionally
  small UI adapter document. It contains exactly
  `{ "robots": [{ "id", "version", "label" }] }` for 22 source-only models.
  Rich entries in `source-library.json.presentation_entries` additionally carry
  `source_path`, `urdf`, and `status: source_only`.
- Stable source versions are `robot_lab_2.3.2`; they are upstream source
  identities, not LingTu package versions or controller qualification claims.

The reference inventory contains 907 unique per-URDF references: 789 resolve
within the copied source tree and 118 are genuinely missing. All 118 belong to
four non-primary Unitree G1 hand variants:

- `g1_29dof_lock_waist_with_hand_rev_1_0.urdf`: 16 missing hand meshes.
- `g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf`: 26 missing hand meshes.
- `g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf`: 60 missing hand meshes.
- `g1_29dof_with_hand_rev_1_0.urdf`: 16 missing hand meshes.

The selected Unitree G1 presentation entry,
`g1_29dof_rev_1_0.urdf`, has no missing reference. All other selected primary
URDFs also have fully resolved visual references. Relative `meshes/...`,
`package://<description>/...`, and RobotEra's `file://$(find ...)/...` forms are
resolved against their actual package layout with exact-case checks.

## Offline visual plans

`python -m sim.tools.assets.build_robot_library_preview_plans` reads the source
inventory and emits
`build/robot-library-import-20261016/robot-library-render-plan.json`. It parses
the selected URDF link/joint trees at declared zero joint pose, composes joint
and visual origins, includes visual geometry only, preserves mesh scale, and
fails instead of synthesizing geometry when a visual mesh is unresolved.

The current plan contains 22 robots and 536 visual geometries: 520 meshes, four
boxes, and 12 spheres. Mesh use is 418 STL, 29 OBJ, and 73 DAE instances. The
existing preview-plan loader successfully normalized all 22 into
`build/robot-library-previews-20261016/robot-selection-render-plan.json`, with
planned outputs `build/robot-library-previews-20261016/robot-<id>.png` at
1400x1200. Actual Blender 5.2 rendering subsequently completed for 17 RobotLab
models and four additional Menagerie models (H1, Panda, UR5e, Spot). All 21 PNGs
were checked for transparency and framing, then installed in the UE plugin's
`Resources/Selection/` directory. Together with the existing Thunder and
OmniCart isolated renders, there are **23 installed model previews**.

Blender 5.2 exposes a Collada RNA name but rejects the actual `collada_import`
call as unavailable. Thus A1, B2, B2W, Go2, Go2W and the external ANYmal D
currently have source geometry but no installed preview. Their files are
preserved; no substitute robot or fabricated image is used. See
`build/robot-library-previews-20261016/pending-previews.json`.

The screenshot of the real 29-choice UE selector is
`sim/runtime/visual/RobotSimUE/Saved/Automation/LingTuSimUI/FrontEnd/robot-map-20261016-203812/asset-library.png`.
This is a static geometry library, not 29 qualified locomotion controllers.

## Remaining promotion sequence

1. Review model-origin license terms before redistributing third-party assets
   where the repository has no per-model notice; finish the six DAE previews.
2. Use the existing importer/catalog boundaries to normalize link and joint
   names, units, mesh references and model entrypoints; enumerate supported
   variants explicitly.
3. Build MuJoCo-loadable robot packages and validate geometry, joints and
   actuators without claiming locomotion. URDF conversion alone does not
   provide a floating base, actuator control contract or a locomotion policy.
4. Produce visual projections and actual preview images; add catalog entries
   to the stepped robot/map selector with their actual readiness state.
5. Bind compatible ControllerPackages and qualify each intended robot/world
   session before enabling its Enter Scene action.
6. Supplement missing robot classes from manufacturer model repositories or
   MuJoCo-native sources, with provenance and format compatibility recorded.

No imported RobotLab source is currently marked runnable. There are no new
directories under `sim/packages/robots`, no ControllerPackage claim, and no
session option. New model promotion must not replace the existing DDS contract,
MuJoCo physics ownership, or lifecycle owner.

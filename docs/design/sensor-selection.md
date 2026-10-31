# Sensor selection and illustrations

Updated: 2026-10-23. Incremental implementation, not full Factory Product acceptance.

## Nine-sensor selection increment

Thunder navigation and headless-navigation now have explicit `1.1.0` rigs with
all nine installed choices: RGB, depth, MID-360 LiDAR, IMU, truth odometry,
thermal, semantic/instance ground truth, microphone and GNSS. The FactoryPark
and open-field selector sources and the formal headless default select these
rigs. Version `1.0.0` remains unchanged. The new four optional streams default
off: compiling the default still produces exactly the original five streams,
with their original owner, rate, geometry and transport. This is not a camera
quality or source downgrade.

The drawer adds:

- Thermal and semantic camera projection presets: pinhole, 180-degree
  equidistant fisheye, and 360-by-180-degree equirectangular panorama. These
  are explicit software calibrations at the installed measurement resolution,
  not extra RGB/depth render cameras or measured hardware lens calibration.
- Software pan/tilt controls with declared angle limits. The authored angles
  compile into the actual measurement rays. No physical gimbal motor is claimed.
- An explicit GNSS simulation-fix toggle. Off means NO_FIX; on requests a
  SIMULATION fix using the rig's displayed WGS84 origin, not RTK or a surveyed
  factory. Current rig origin is synthetic zero; altitude is ellipsoidal.
- Acoustic cadence locked to the declared 16,000 Hz / 160-sample blocks =
  100 blocks/s. It is not a freely editable audio sampling-rate control.
- World-property availability reasons. A World without `measurements` cannot
  enable thermal/semantic/acoustic through the selector. The compiler still
  rejects a direct request. No empty scene, temperature, label or waveform is
  silently manufactured to bypass that contract.

**Current limitation:** the existing FactoryPark/open-field/formal factory
WorldPackages have no authored `measurements`. Their three measurement choices
are visible but disabled with a reason; GNSS can be selected independently.
This increment is not "all nine sensors are usable in the factory". Real
factory thermal surfaces, physical-geometry labels, sound sources, a matching
localization map, and same-Product runtime acceptance remain necessary.

Only compatible drafts share state across maps; the comparison now includes
availability, calibration preset, angle limits, cadence and GNSS defaults.
Startup selection does not add remote runtime switches to legacy LiDAR/IMU/GNSS.

The inventory is not a universal sensor catalog. There are no standalone
radar, ultrasonic rangefinder, magnetometer, force/torque/tactile or gas-sensor
packages wired through this selector. Physics joint state is not an independent
calibrated encoder model. MID-360 is the current LiDAR implementation; this does
not establish every LiDAR model. Fisheye/panorama here are measurement-camera
projections, not yet panoramic RGB render-sensor support.

Verification for this increment is under `build/inspection-tests/20261023/`:

- `sensor-suite-focused-01.xml`: 99 passed, no failures/skips. Covers four
  selection/package/launcher test files, including strict schemas with the
  existing Miniconda runtime; no dependency installed. A pytest `cache_dir`
  configuration warning remains, not a sensor-test failure.
- Focused Ruff: seven Python files passed; Mypy: three implementation files
  passed. Independent read-only implementation review: APPROVE, zero findings.
- `sensor-suite-ui-build-01/build.log`: failed with MSVC C3859/C1076 and Windows
  error 1455 (commit/pagefile exhaustion); retained unchanged. Single-action
  `sensor-suite-ui-build-02/build.log`: Succeeded, exit 0, 85.29 s.
- `sensor-suite-ui-render-01/report/index.json`: 16 succeeded, zero
  failed/notRun/warnings, process exit 0 and natural cleanup. Its real production
  writer's intent independently compiled to RGB 15 Hz, depth absent, GNSS 10 Hz
  with explicit simulation fix, unchanged IMU/LiDAR/truth rates and source file.
  These are verification edits, not new defaults or live-rate measurements.
- Visual inspection of render-01 found a dark default combo style on the light
  drawer. The style and the render test's final GNSS scroll position were fixed;
  `sensor-suite-ui-build-03/build.log`: Succeeded, exit 0, 68.37 s. Final rendered
  acceptance is recorded below after verifying that delta.

No Product, native navigation, MuJoCo runtime or robot-motion task was started
by these Editor-only UI checks. UE-owned RGB/depth still requires the separate
formal Product camera SHM relay; merely adding GNSS in a UE-source plan cannot
bypass that owner contract.

Final rendered acceptance: `sensor-suite-ui-render-02/report/index.json`
records **16 succeeded, zero failed/notRun/warnings**. Its owned Editor process
exited naturally with code 0 in 68.484 s. The screenshot was inspected: the
projection label is readable on the light drawer and the bottom GNSS simulation
fix checkbox is visible. `intent-compiled-validation.json` independently
confirms the actual widget-written settings reached the native GNSS declaration
without changing the source Session or RGB producer. The 16 tests are repeated
verification of one suite, not 32 distinct cases. Delivery summary and scoped
process scan: `sensor-suite-delivery-increment.json` and
`sensor-suite-postflight.json` in the evidence root.

## User flow

`01 Robot → optional sensor drawer → 02 Map/task → ProductControl`

The drawer edits the next session, not the running robot. It lists only sensors
declared by the selected SensorRig. Every row shows a type illustration, package,
mount frame, enabled state, required/optional status and sample rate in Hz.
Required navigation dependencies cannot be disabled. Inspection additionally
requires the declared RGB camera. Configuration drafts survive switching between
choices with identical robot/rig/defaults/timebase; incompatible setups do not
share edits. Back closes the drawer before leaving the robot step.

This is not arbitrary hardware installation: adding a package, mount, extrinsic
calibration or noise configuration still requires an authored compatible rig.
Projection choices are limited to the explicit software presets above. Nine
illustrations do not prove nine valid measurements. Truth odometry is not SLAM.

## Ownership and startup

The selector emits optional `sensor_selection` data in the existing selection
intent. Python validates the selection against the catalog and checks that the
current source SessionSpec still semantically matches the selected bundle's
saved source. Stale catalogs are rejected, including non-sensor world changes.

Only actual edits produce a new SessionSpec under
`sim/sessions/user_selection/<selector-run>/session.yaml`, with a new session ID
and `robots[].sensor_overrides`. Source packages, immutable rigs and the old
compiled bundle are not modified. The existing CatalogResolver applies enabled
and frequency overrides when constructing actual streams. Even unchanged source
choices are checked for the requested Product's dependencies. Physics frequencies
must satisfy the existing exact physics-timebase contract before writing a file.

Launcher opt-in: `--execute-selector --execute-product --product <product>
--robot <native-robot-config>`; navigation/inspection also require an explicit
saved `--map`. A WorldPackage is not automatically a saved localization map.
The launcher hands the source to `python -m lingtu.control switch ... --sim-session
<repo-relative-path>`; ProductControl owns the only compile/start lifecycle.
The old playable-coordinator path rejects custom sensor choices rather than
silently ignoring them. Building a command does not prove a Product was started.

The formal MuJoCo camera producer now follows independently enabled RGB/depth
declarations and their individual rates, using one existing bounded render queue.
An unselected peer has no writer, frame or scheduler counter. Evidence and
Product acceptance use `camera_rgb` and `camera_depth` for declared routes;
the sparse legacy test contract retains `camera_rgbd`. Measurement-only runs
reuse the native camera endpoint without creating RGB/depth frames. This does
not add live remote RGB/depth controls. A declared UE-owned camera is rejected
before engine creation: a MuJoCo image cannot stand in for that source.

## Image provenance

Nine original, reproducible 480×240 SVG/PNG type diagrams live in
`sim/runtime/visual/RobotSimUE/Plugins/LingTuSim/Resources/Sensors/`:
RGB, depth, LiDAR, IMU, thermal, acoustic, semantic/instance, GNSS, truth odometry.
`manifest.json` and `SOURCES.md` identify them as non-measurement schematics.
They are generated by `sim/tools/assets/render_sensor_type_previews.py`.
No commercial-game assets, downloaded hardware photos or generated sensor data
are used as evidence. The UI explicitly labels them “类型示意，非实测数据”.

Actual measured previews belong to the runtime viewer, including units, validity,
source identity and freshness. Existing controlled UE examples are under
`build/inspection-tests/20261022/ue-measurements-live-02/`; these are not the Fab
factory or a completed patrol. Raw measurement archives and display images remain
distinct. The startup diagrams are not selectable measurements.

## Delivery checklist

- [x] Original type diagrams and manifest/provenance tests.
- [x] Sensor drawer connected to the actual robot/map selector and intent writer.
- [x] SessionSpec overrides compiled into actual enabled/rate stream declarations.
- [x] ProductControl CLI handoff; legacy launch cannot discard custom choices.
- [x] Stale source, exact timebase and requested-Product dependency checks.
- [x] Actual rendered selector, focus/edit events and saved intent verification.
- [x] Formal MuJoCo independent RGB/depth scheduling and matching acceptance
      counters; disabled peers remain absent.
- [x] Authored mixed-owner Thunder rig and actual robot-geometry mount checks
      for thermal, semantic and microphone measurements; see its
      [README](../../sim/packages/sensor_rigs/doso/thunder_v4/multimodal_unreal/README.md).
- [ ] Same-Product startup with the user's selected rig and live source ACKs.
- [ ] Actual Product-owned UE visual/camera bridge acceptance, then make the
      tested multimodal preset selectable with actual World properties. Relay
      code/contracts are implemented in the 2026-10-23 increment; the authored
      rig is not an accepted runnable catalog choice yet. Original RGB/depth
      requirements are retained; see [delivery status](sensor-delivery-status.md).
- [ ] Actual Factory WorldPackage + localization map + three inspection points.
- [ ] Same-run navigation/avoidance/manual-takeover, measurement capture and report
      acceptance. No claim of these gates from a UI screenshot or unit tests.

The larger capability plan and measurement-model limits are tracked in
[multimodal-sensors.md](multimodal-sensors.md), and the actual Fab intake in the
[factory README](../../sim/tools/worlds/factory_environment_collection/README.md).

## Verification ledger

Artifacts are under `build/inspection-tests/20261022/`.

| Check | Recorded result / scope |
| --- | --- |
| `sensor-selection-focused.xml` | 127 passed; two schema cases skipped in project venv, separately passed below |
| `sensor-selection-override-schemas.xml` | Both SessionSpec v1/v2 override schema cases passed using existing Miniconda/jsonschema |
| `sensor-selection-catalog-schemas.xml` | 244 Catalog/measurement/GNSS cases passed, no skips; not a runtime trial |
| Focused Ruff / Mypy | Seven changed/new Python files Ruff; helper and launcher Mypy passed |
| Independent review | APPROVE/CLEAR after stale-source, no-edit dependencies, timebase and cross-world sharing fixes |
| `ui-sensor-selection-build-05/result.json` | UE 5.8 build succeeded, exit 0 |

Failures remain visible: first rendered run passed 23/24 tests, with one widget
test failing three assertions. Slate's bound enabled state needed a real prepass
before the immediate test assertion, and its default error widget was lazily
created; the panel now provides the error widget up front. Build-03 failed on a
test's nonexistent `InvalidatePrepass` API; build-04 uses the installed
`MarkPrepassAsDirty` API. Neither failure was rewritten as success.

The first actual rendered intent independently compiled offline to RGB 15 Hz,
no depth stream, IMU 200 Hz, LiDAR 10 Hz and truth odometry 100 Hz. Its original
source is unchanged. `ui-sensor-selection-render-01/intent-compiled-validation.json`
records this narrower success; it does not override that run's failing widget
test or prove any live sensor rate. The next rendered run passed all 24 tests,
but manual image inspection found the supplied error widget's default icon was
visible on valid untouched rows. It is now explicitly collapsed at construction,
with an initial-visibility regression assertion. Final rendered acceptance is
recorded below only after this repair is verified.

Final acceptance: `ui-sensor-selection-render-03/report/index.json` records
**24 succeeded, zero failed/notRun/warnings, exit 0**. The actual
`sensor-selection.png` was visually inspected: all five declared rows are visible,
the modified RGB value is readable, disabled depth is dimmed, and untouched valid
rates do not show error icons. `selection.intent.json` came from the production
UE writer after actual checkbox/text-commit events. Its
`intent-compiled-validation.json` confirms RGB 15 Hz, depth absent, original
IMU/LiDAR/truth-odometry rates retained and original source unchanged. This is
an offline compile of the real UI intent, not a live-rate measurement. The
15 Hz choice is an explicit verification edit; default camera rates stay 30 Hz.

At `2026-10-22T18:43:21+08:00`, exact UE/build-process names and relevant Python
measurement/MuJoCo/manual/Product commands matched no live processes. No Product
or robot motion was started in this increment. Summary:
`sensor-selection-delivery-increment.json` in the evidence root.

### Runtime selection / robot-mount follow-up

The `2026-10-22` follow-up is recorded in
`multimodal-mount-runtime-increment.json` under the same evidence root.

| Check | Actual result / boundary |
| --- | --- |
| `selected-camera-runtime-final.xml` | 314 cases: 311 passed, 3 tracking failures; zero errors/skips |
| Changed producer/consumer/evidence subset in that XML | 179 passed; these are included in the 314, not additional cases |
| `multimodal-rig-contracts-final.xml` | 54 passed; four real-MuJoCo cases deliberately selected out for the runs below |
| `thunder-mount-proof-02.xml` | Three actual Thunder optical projection/mount cases passed |
| `thunder-acoustic-mount-proof-01.xml` | One actual Thunder microphone mount/pressure/NPZ case passed |
| Focused Ruff | Six changed/new Python files passed |
| Focused Mypy | Feeder, evidence and two new producer/rig test files passed; acceptance consumer retains its unrelated `Path \| None` baseline diagnostic at line 366 |
| Independent read-only review | Producer/rig/schema APPROVE; subsequent acceptance consumer/microphone-test delta APPROVE |

The three wider-suite failures are not waived: the shared tree declares a
`tracking: [standard]` variant, but the catalog assertion still expects only
map/nav variants, and tracking's acceptance manifest resolves no variant. The
attach-only preflight and tracking dispatcher therefore fail with
`plan='standard', manifest=None`. This follow-up does not modify tracking
configuration, manifests or its existing tests. No whole-suite green is claimed.

At `2026-10-22T22:48:36+08:00` no UE/UBT/UAT, Blender or active simulation entry
process matched the scoped scan. Four MSBuild processes remained on the machine
and were not touched; therefore the broad build-process scan is **not clean**.
No UE, ProductControl run, native runtime or robot motion was launched by this
follow-up. The actual MuJoCo checks are local test-model sampling, not a live
Factory Product or navigation qualification.

The next implementation step is tracked in
[one Product visual session](product-visual-session.md): explicit Unreal mode,
feeder-owned presentation, same-engine snapshots and bounded visual readiness.
That document separately records the camera-source and actual-runtime gates;
code wiring alone does not make the multimodal Unreal rig a runnable choice.

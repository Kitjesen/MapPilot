# Sensor and Factory inspection delivery status

Updated 2026-10-23. A model, selectable preset, and accepted Product capability
are different milestones. This ledger prevents one from being reported as another.

## What was missing and why

Previous increments delivered sensor models, raw formats, controls and UI before
the actual downloaded Factory world and single-Product render bridge were ready.
That is incomplete delivery, not evidence that Factory inspection already works.
Reading a joint/contact value from MuJoCo also does not implement a calibrated,
sampled encoder or force sensor. More UI switches would not close these gaps.

The immediate priority is the existing measurement-to-inspection chain. New
modalities are explicitly unimplemented; they do not silently return simulation
truth under a hardware-sensor name.

## Capability inventory

| Capability | Implemented layer | Still not accepted / missing |
| --- | --- | --- |
| RGB / depth | Selection, independent rates, MuJoCo sampling; new exact UE-source SHM/native relay with latched physical truth | Rendered same-Product Factory run, original-rate acceptance; uniform legacy runtime controls |
| LiDAR | Existing physical ray sampling / native navigation feed | Same-run Factory navigation and avoidance acceptance; uniform remote control coverage |
| IMU | Existing physical sampling / native feed | Full selected-sensor Factory joint acceptance and uniform remote controls |
| GNSS | Mounted simulation fix/no-fix, covariance/noise and source metadata | Joint acceptance; no real positioning, indoor GNSS certainty or surveyed origin is implied |
| Thermal | Per-pixel K, emissivity, unknown/occluded masks, raw NPZ, controls, evidence rules | Actual Factory Product run. Greybody simulation is not calibrated LWIR hardware or a heat solver |
| Acoustic | Pa waveforms, sample clock, distance/delay, noise/calibration metadata, raw NPZ and evidence | Factory Product run; reflections, diffraction and full room acoustics remain unsupported |
| Semantic / instance | Physical class/instance truth, mounting, raw labels and viewer | Factory Product acceptance; truth is not an algorithm's recognition output |
| Gimbal / fisheye / panorama | Mounted sampling and calibrated projection models; selectable thermal/semantic projection | Physical motor limits/backlash are not implemented by a software pan/tilt control; actual Product projection/rate acceptance remains |
| Millimetre-wave radar | **Not implemented** | Range / radial velocity / RCS, Doppler and beam/noise model, raw format and capture adapter |
| Ultrasonic ranging | **Not implemented** | Beam / time-of-flight / blind zone / invalid echo, units, calibration and raw capture |
| Magnetometer | **Not implemented** | Declared magnetic field and mounting, hard/soft iron, calibration/noise and units |
| Force / torque | **Not implemented as an independent sensor** | A mounted six-axis wrench measurement and calibration; contact truth alone is insufficient |
| Tactile | **Not implemented** | Taxel/contact area, pressure/force distribution, sampling, noise and validity |
| Gas | **Not implemented** | Species/concentration units, sources/transport assumptions, response/calibration/noise and evidence |
| Joint / wheel encoders | Physical joint and wheel states available, **not independent encoder sensors** | Tick quantization, wrap/direction, sampling/latency, calibration/noise and raw records |

## This increment: what actually changed

### Factory bay follow-through, 2026-10-23

The real first bay now has 27 original static components / 122 original convex
geoms, 99 migrated asset files (460121362 bytes), and a saved/reloaded RobotSimUE
level with original transforms/material slots and visual NoCollision. The source
map/default product entry were not changed. NullRHI authoring is not rendered RGB
acceptance. Selected MuJoCo mesh vertices are unchanged; no substitute floor was
inserted. The runtime loader now detects explicit static World geometry
structurally rather than guessing from floor names.

Mounted static review on the actual Thunder at two non-colliding proposed views
produced 463 / 301 valid target thermal/semantic pixels at 64x48 and 160 valid
16-kHz Pa samples each. Against the original 1065-geom static collision candidate,
no target pixel was falsely exposed by omitted geometry. Outside-scope background
rays do differ. A previous proposed console pose collided with the conveyor and
was rejected. No arrival, navigation, RGB capture or Product report is implied.

One physical zero-command observation used the public builder and declared
locomotion controller on the full static Factory: 4.9 simulation seconds including
warmup, 961 recorded samples, no nonfinite/solver warning/non-wheel loaded contact.
It includes startup contact peaks and brief wheel unloading, so continuous
four-wheel support and spawn/motion qualification are not claimed. See the
[Factory source README](../../sim/tools/worlds/factory_environment_collection/README.md)
for exact paths, failures retained, positions, limitations and next gates.

Source collision responses were subsequently read from all 27 original components;
the Custom component blocks the exposed Pawn/physics/visibility channels. The wet-floor
sign is physically simulated in the source and explicitly frozen in this static candidate,
not delivered as a pushable prop. Windows mapd's existing query endpoint is enabled
again; 125 existing graph and 3 focused wiring tests passed, with Linux unchanged.

The same-Product gate is still open: final world/spawn/render acceptance; current
World/session binding; genuine factory save/reload map identity; then native navigation,
settling, paired captures and task completion. No new sensor modality was added
to hide these integration gaps.

The Windows PGO link blocker was resolved later on 2026-10-23 using a separate
static GTSAM 4.3.0 SDK built from existing local sources. The shared SLAM SDK was
not changed, no old optimizer was substituted, and no link errors were ignored.
The current optimizer passes four native tests and has no GTSAM DLL dependency.
Windows simulation now explicitly declares its executable in the map process.
Configuration/transport regression: 129 + 30 Python tests; native SaveMap regression
also passed. A new real-PGO integration test optimized a synthetic 25-patch closed
trajectory, saved a 3D map, reopened the store, strictly loaded it, and proved a
failed replacement preserves the previous map. It ran with only System32 in PATH.
Static-fixture cleanup was explicitly disabled; no live SLAM capture, Product,
factory map identity, or navigation acceptance is implied. Evidence:
`build/inspection-tests/20261023/map-save-native-01/real-save-reload-02/`.

- [x] Existing inspection measurement capture rejects repeated, out-of-order or
  simulation-time-regressing data from the same exact source generation. A new
  invalid measurement cannot be replaced with an older normal result.
- [x] Controlled real-MuJoCo thermal/acoustic sampling passes the existing wire,
  capture action, NPZ/JPEG/pose persistence, evidence reload and report routes for
  normal, anomaly and inconclusive outcomes. Task arrival is a controlled input,
  not proof of autonomous motion or a real DDS task run.
- [x] Actual Fab Control22 and Engine source components have explicit authored
  K/emissivity/semantic/acoustic assumptions bound to existing geometry, with no
  substitute boxes or fabricated hardware calibration.
- [x] Candidate intake is separated from qualification/promotion. The public
  importer retains source/provenance; no approved spawn, visual alignment or
  localization map is fabricated for it.
- [x] UE capture truth extension compiled and passed 6 CameraShm plus 18
  CameraCapture Automation cases. Both runs used NullRHI; this is not a
  rendered throughput or Factory Product acceptance claim.
- [x] Product-owned camera relay code and focused contracts are implemented.
- [x] Independent code review: Factory candidate intake/authoring APPROVE after
  four concrete fixes; camera/owner/compiler/capture review APPROVE after startup,
  cadence, low-rate watchdog and failed-run evidence fixes. Frozen-tree focused
  checks are recorded below; these verdicts do not approve a live Factory run.
- [ ] Migrate and verify the actual Fab visual/collision subset and device views.
- [ ] Build/save/reload its genuine localization map and bind route identity.
- [ ] In the same Product: native arrival/settling, sensor ACK, live viewing,
  capture, normal/anomaly/inconclusive result and durable task report.
- [ ] Verify obstacle stop, manual takeover/release and natural owned cleanup.
- [ ] Extend the unified runtime control contract to legacy sensors, then add
  new modality packages/models/adapters/tests without a parallel switch system.

Temperatures, sound clips and thresholds authored for the Factory candidate
are simulation test assumptions. All unbound surfaces remain unknown; a colored
image, playable sound or a saved file does not establish a valid measurement.

## Evidence and next acceptance

See [same Product bridge](product-visual-session.md),
[multi-modal plan](multimodal-sensors.md),
[selection status](sensor-selection.md), and the
[Factory source README](../../sim/tools/worlds/factory_environment_collection/README.md).
Build/test artifacts are under `build/inspection-tests/20261023/`.

| Final evidence | Actual result and scope |
| --- | --- |
| `product-camera-relay-final-reviewed.xml` | 216 passed, zero failed/errors/skipped: feeder, exact selected cameras, relay, optional visual owner, measurements and Windows SHM |
| `inspection-measurement-final.xml` | 264 passed, zero failed/errors/skipped: physical measurement capture, raw artifacts, evidence and report routes |
| `ue-camera-frame-green.xml` | 88 passed: compiler optical frame and existing measurement/visual assembly contracts |
| Factory authoring/importer focused run | 70 passed, zero skipped: 25 candidate, 9 authoring and 36 existing heightmap/promotion; independent 8-case rerun is a subset, not 8 additional cases |
| `factory-measurement-authoring-05/strict-schema-validation.json` | WorldPackage, MeasurementScene and SensorPlan strictly validated with existing jsonschema; two invalid controls rejected |
| `lane-c-shm-truth-01/validation-result.json` | One successful UE build; 6 CameraShm and 18 CameraCapture Automation cases succeeded, NullRHI |
| Focused static checks | Changed relay/visual/feeder: Ruff, Mypy and py_compile passed; Factory importer/authoring: Ruff and Mypy passed |

These suites cover different, partly overlapping contracts and are not summed
into an end-to-end acceptance number. The compiler's wider Mypy invocation still
reports its pre-existing map-path assignment diagnostic at line 610; the new
optical-frame helper is not marked as an all-file typecheck pass. Existing
`cache_dir` pytest configuration warnings are retained. The Editor startup logs
also retain 13 existing condition diagnostics per run and a connectivity warning;
the selected Automation cases themselves have zero warnings/errors.

Retained RED artifacts include `ue-route-red.xml`, `visual-alloc-red.xml`,
`relay-boundary-red.xml`, `relay-review-red.xml` and
`relay-failed-evidence-red.xml`. A failed run is not rewritten as success.

At `2026-10-23T03:48:07+08:00`, the scoped process scan found no UE, Automation,
dotnet, Blender, native simulation publisher/driver, or relevant Product/MuJoCo
Python entry. It did find `cl` PID 27656 and MSBuild PIDs 77508, 125224, 128072;
ownership was not established and they were not touched. The owned build and
Automation runs exited, but **a machine-wide build-process all-clear is not
claimed**. No Product or robot motion was launched by this increment.

The next accepted result must use the real Factory source and existing native
inspection authority. It must not use a legacy map, invented map ID, mock arrival
or a second manually owned simulation to claim that this Product gate is done.

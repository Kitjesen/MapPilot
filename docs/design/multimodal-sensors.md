# Multi-modal sensor delivery

Goal started 2026-10-21 at the user's request. This is the active delivery plan,
not a declaration that these capabilities already work in the factory product.
Status as of 2026-10-22: acknowledged native remote controls and raw archives
are implemented for thermal, acoustic and semantic measurements. A real
HTTP-route → native DDS → MuJoCo controlled thermal fixture passed. Factory
Product inspection, the selected Fab physical world and UE data/control UI
acceptance remain unfinished; the full feature is not accepted.
Bounded Gateway previews and a real Web `MeasurementInspector` with controls
in `InspectionWorkbench` have passed a no-motion MuJoCo/browser fixture. Cached
ACK and in-transit expiry defects are fixed and independently APPROVE/CLEAR.
This does not qualify the complete Factory Product, UE UI or sustained rates.

UE increment (2026-10-22): native Slate viewing and controls now pass a real
HTTP/native-control/MuJoCo diagnostic through the actual runtime HUD. Latest
affected native contracts: 30 passed; Python launch/device binding: 97 passed;
final rendered interaction fixture: 1 passed, nine matching ACKs, 23 raw NPZ
samples closed without errors. Independent code review is APPROVE/CLEAR after
concrete fixes. This is still not Factory Product/inspection acceptance.

## Scope and authority

Deliver per-pixel thermal measurements, calibrated acoustic waveforms, semantic
and instance ground truth, mounted PTZ/fisheye/panoramic cameras, one sensor
control/recording interface, and inspection capture with normal/anomaly/
inconclusive outcomes. Thermal and acoustics are first; GNSS receives only
integration maintenance. A scalar thermometer is a test oracle, not the thermal
camera deliverable. Pseudocolor and sound playback are presentation only.

ProductControl remains the process owner; MuJoCo owns time, body poses and
occlusion geometry. Measurements consume these without integrating physics.
Native inspection remains the authority for settling and point advancement.
Capture results cannot infer task success. UI displays actual sensor state and
raw evidence. No copied MATRiX machine code/assets enter LingTu runtime.

## Parallel work ownership

| Lane | Owned implementation | Current result |
| --- | --- | --- |
| Thermal | `sim/runtime/sensors/thermal.py`, `measurement_scene.py`, `measurement_extractors.py` | Actual MuJoCo raster → K matrix → recording/replay and evidence passed |
| Acoustics | `acoustics.py`, `acoustic_scene.py`, extractor | Actual scene body poses/occlusion → Pa block → recording/evidence passed |
| Projection / semantic | `camera_geometry.py`, `semantic_camera.py`, extractor | Real front/rear geom labels and gimbal/fisheye/panorama sampling passed |
| REA camera chain | `research/matrix/CAMERA_CHAIN_20261021.md` | 48 functions decoded, 29 decompiled; camera output chain recovered; production Blueprint bridge and infrared shader remain unverified |
| Controls / recording | SensorRuntime/SessionSensorRuntime, `commands.py`, native control bridge/Host module and raw archive | Three new modalities share acknowledged remote controls and raw recording; real HTTP→DDS→MuJoCo thermal fixture passed, not Factory Product acceptance |
| Inspection | Evidence module + `measurement_capture.py` / `measurement_checks.py` | Typed input, sensor selection, identity/time checks and raw measurements with RGB/pose transaction implemented; controlled producer-to-capture tests passed |
| Main / product integration | Catalog, formal feeder/native wire, Host, Gateway, UE | Opt-in packages/rig, compiler/feeder/Host, measurement GETs and acknowledged control POST wired; selected Fab bindings, actual Factory Product inspection and UE UI acceptance remain open |

Workers own disjoint files. Shared schema/registry/lifecycle edits stay with main.
Independent review follows real integration, not just isolated model tests.
Completed code-level gates below do not imply the unchecked Product/UI gates.

## Delivery gates

- [x] Source-bound physical raster: nearest geom, calibrated world rays, miss /
  invalid pixels, mounted/gimbal transforms, same simulation time/generation.
- [x] Thermal image: Kelvin raw matrix, validity, apparent versus corrected
  temperature, declared surface properties and radiometric assumptions; record
  raw data, not just a palette image. Known temperature, occlusion and missing
  property cases must pass.
- [x] Acoustic blocks: pressure Pa, exact sample clock, source distance/delay,
  channel positions, calibration, noise and invalid path semantics. Explicitly
  disclose direct-path/no-reflection limits.
- [x] Semantic/instance label images from physical objects, stable IDs and
  unknowns; never detector confidence or evidence that a perception model works.
- [x] PTZ/fisheye/panorama: calibrated projections and mount pose affect actual
  sampling; no visual-only camera switch presented as a sensor implementation.
- [x] Unified controls for the three new modalities: query, enabled, rate,
  noise/noise_enabled, recording and camera gimbal where supported, through the
  existing session owner and acknowledged native DDS command path. Unsupported
  operations fail explicitly; semantic label noise stays disabled. Preserve
  existing Product defaults.
- [x] Recording/replay: raw arrays/waveforms plus units, calibration, stamp,
  source/model assumptions and validity survive a round trip; bounded raw
  archive is wired to the optional formal measurement path.
- [x] Opt-in sensor packages and full eight-stream rig pass strict schemas;
  real HTTP-route/native DDS/physical thermal fixture closes its raw archive.
- [ ] Extend unified remote-control coverage to all legacy RGB/Depth/LiDAR/IMU.
- [ ] Factory Product integration acceptance: selected Fab device/source and
  physical World declarations, actual activation, owned sampling and cleanup.
- [ ] Actual Product inspection acceptance: native arrival/settling, matching
  measurement/RGB/pose capture, configured threshold or inconclusive result,
  durable evidence and native result receipt in the same run.
- [x] Web measurement inspector: real browser thermal/acoustic/semantic display,
  native settings ACK, recording, disable/clear and re-enable/new sample in an
  isolated MuJoCo fixture; complete preview/API/UI review APPROVE/CLEAR.
- [ ] UE viewing/control acceptance in the actual Factory Product. The Web
  fixture and its screenshots do not close this gate.
- [ ] Same-run factory acceptance and review; no claim based solely on unit tests.
- [x] Narrower UE diagnostic gate: native measurement page displays thermal,
  acoustic and semantic data and exercises actual query/record/enable widgets
  against real native source ACKs; original pause/resume and focus are preserved.

## Measurement assumptions

Thermal first implementation is a broadband greybody simulation, not a calibrated
LWIR detector or a heat-equation solver. World surfaces must explicitly supply
temperature/emissivity; absent values stay invalid. Acoustic first implementation
uses direct paths and block-frozen geometry; reflections/diffraction are not
silently approximated. These limitations remain attached to recorded data.

The selected Factory Environment Collection still needs authoritative collision
and device/source identity binding. Controlled MuJoCo fixtures can verify model
contracts but cannot qualify that factory. Existing map/content identity remains
required before an inspection route can run.

## Evidence policy

Track each gate separately as code/contract, real engine data, and product/UI/
inspection acceptance. Preserve failed runs and measured limitations. Do not
sum overlapping test suites or mark the whole goal complete after partial APIs.

## Actual data and review (2026-10-21)

`build/inspection-tests/20261021/multimodal-physical-01/result.json` is a bounded
MuJoCo 3.10 controlled scene, NOT the Fab factory or a native inspection run.
It uses the shared SessionSensorRuntime, real nearest-geom occlusion, MuJoCo RGB,
raw NPZ timeline and existing inspection store. Six captures were persisted:

| Controlled case | Thermal ROI | Acoustic unweighted RMS | Both verdicts |
| --- | --- | --- | --- |
| normal | 320 K | 0.07040445 Pa | normal |
| over threshold | 390 K | 0.23468149 Pa | anomaly |
| unknown foreground cover | all invalid | blocked/invalid, not silence | inconclusive |

The configured 350 K / 0.15 Pa thresholds are demonstration rules, not industrial
safety limits. Raw measurements, same-source pose, calibration, scene XML and
RGB PNGs are retained. The in-process renderer/sinks closed normally.

Combined focused regression: **585 passed**, report `multimodal-regression.xml`.
Two later physical wide-angle cases separately passed in the 8-test extractor
suite; counts overlap and are not added. Existing `cache_dir` warning persists.

Independent review found and closed: floating-point corrected-temperature
endpoint rejection; fixed prepare-thread ownership breaking background dispatch;
initial acoustic rate/block overlap; new payload/acoustic type errors. Narrow
final review APPROVE/CLEAR, 237 tests passed. Four-file targeted mypy passed;
normal import expansion still reports 15 unrelated errors in six existing files.

## Implemented wiring and remaining production acceptance

1. Implemented: SensorPackage/Rig/scene measurements → Catalog → immutable
   RunPlan. Thermal properties belong to named World surfaces, not PBR colors.
2. Implemented: existing formal feeder/engine lease → owned LTU1/native
   publisher, and recording-enabled samples → bounded raw archive. No second
   process owner; actual Factory Product activation remains to be accepted.
3. Implemented: typed DDS measurements → Host adapter → typed inspection input,
   with explicit Product/session and simulation/host clock mapping.
4. Capture/persistence code and controlled tests exist. Real Product native
   arrival/settling/capture/result receipt in one qualified run remain open;
   saving an artifact cannot independently authorize point advancement.
5. Implemented: Gateway measurement download/live projection and acknowledged
   control POST. UE thermal/waveform/label viewing and control UI acceptance
   remain open; legacy sensor remote-control coverage is not yet uniform.
6. Required: selected Fab-world collision/map/device/source binding and bounded
   same-Product factory inspection acceptance, not the legacy workshop.

Do not put thermal Kelvin through the existing depth `32FC1` decoder; do not
encode acoustic channel×sample data as an Image. Both would change meaning.

## 2026-10-22 integration increment

Completed code paths, not a factory run:

- Catalog compiles named world thermal surfaces, semantic labels and acoustic
  sources. The transport schema includes all three modalities. Reusable
  `thermal_camera@1.0.0`, `semantic_camera@1.0.0`, `microphone@1.0.0` and
  `thunderv4_headless_multimodal@1.0.0` are opt-in, not a default-session switch.
  Complete rig and SensorPlan schemas now pass without the two inherited
  xfails: the Mid360 overlay is boolean-only and Mid360-only, while only
  `mujoco_camera` may omit UE depth conversion fields. UE still requires valid
  requested encoding and `depth_scale=0.001`; wire `16UC1` is unchanged. No
  defaults were inserted into existing plans; generation stays byte-identical.
- Compiler binds the existing camera publisher to the source session. Host
  reconstruction supplies active Product identity/domain, adds `measurements`
  to readiness and wires its typed ports to inspection and Gateway. No second
  process owner or fabricated Product identity is introduced.
- Feeder samples under its existing engine lease. Shared camera-endpoint writes
  are whole-record locked; partial-send failure invalidates the stream before
  another producer can append. Epoch mapping uses the same observed physics
  tick as RGB, not a late scheduled deadline. Status distinguishes samples from
  queued/written batches; neither is a subscriber acknowledgement.
- Inspection saves raw arrays/masks/model metadata with RGB/pose. Missing,
  unmapped, stale or wrong-generation inputs cannot pass. Unknown calibration,
  model version or insufficient valid coverage is inconclusive. Semantic labels
  remain ground truth and never establish recognition capability.
- Gateway measurement reads expose `GET /api/v1/measurements`, `.../{sensor_id}` and
  `.../{sensor_id}/raw` plus `.../{sensor_id}/preview`. Only declared sensors are cached, one latest sample per
  sensor. Unknown=404, missing raw=503, stale/invalid receipt clock raw=409.
  NPZ preserves NaNs/masks; JSON uses null for unknown values. Receipt freshness
  is separate from source-clock mapping. Controls use the separate acknowledged
  POST below; UE views and controls are not yet accepted.

### Bounded previews and implemented Web inspector

[`GET /api/v1/measurements/{sensor_id}/preview`](../../src/gateway/routes/measurements.py)
and its [display projection](../../src/gateway/services/measurement_preview.py)
are implemented. Thermal K values and semantic class/instance IDs use nearest
source-pixel sampling capped at 192x144, retain original row/column indices and
valid masks, and represent invalid/nonfinite values as JSON null. Acoustic
display keeps every channel as a min/max Pa envelope capped at 256 bins per
channel, with valid fractions, sample rate, source sample count and bin sample
intervals for relative sample time. It is not a full-resolution waveform.
The response retains measurement identity/time/status alongside
`display_only=true`; full raw NPZ download and recording remain unchanged.
Missing/stale data is not replaced with a synthetic display.

The actual [Web MeasurementInspector](../../web/src/components/MeasurementInspector.tsx)
is embedded in [InspectionWorkbench](../../web/src/components/InspectionWorkbench.tsx)
with preview, raw download and source-ACK controls. Current checks: 25 preview
API tests, 391 combined control/data/inspection tests, targeted mypy on 12 source
files; Web 47 tests, ESLint and production build pass. The existing large
Three.js chunk warning remains. Do not add these overlapping scopes into a
single acceptance count. The ACK/list revision reducer prevents old GETs or
permanently cached ACKs from hiding a newer setting; request transit time is
conservatively added to preview receipt age before freshness/expiry decisions.

Real browser checks of a bounded no-motion MuJoCo fixture are retained in
`build/inspection-tests/20261022/measurement-browser-02/` and `-03/`: 300–400 K,
8000 Hz/80 samples with approximately ±0.04998 Pa, semantic class 7/instance 42,
source-ACK recording and disable/re-enable behavior. A zero-length SVG sample
line was invisible; round caps fixed the display without changing any Pa value.
The fixture uses native DDS controls but in-process LTU1 measurement decoding;
it is not a full Product/Host deployment. The first attempt timed out before
browser interaction, and the second expired before an acoustic recording ACK;
their original results remain. All fixture services exited. UE UI, the Fab
physical world, real Product inspection and sustained delivery remain open.

The complete preview/API/Web scope is independently APPROVE/CLEAR after fixes
for cached ACKs, invisible single-sample acoustic bins, and in-transit expiry.
The long-clip acoustic hot path is also optimized without changing its model:
scene sources reuse an explicitly validated immutable snapshot, and each block
only gathers its interpolation neighbours. An agent-reported bounded benchmark
of 20 scene samples (420-second/8 kHz clip, two channels, 80 samples/block,
initialization excluded) changed mean time from 50.71288 ms to 0.369655 ms.
333 related integration tests pass. This is not a Product RTF qualification.

### Acknowledged native controls and raw archive

The implemented control path is HTTP → Host `sensor.controls` → native DDS →
the existing MuJoCo sampling owner → matching source ACK. Its sources are
[`SensorControlRequest/Result`](../../src/runtime/contracts/sensor_control.py),
[`SensorCommandProcessor`](../../sim/runtime/sensors/commands.py),
[`NativeSensorControlBridge`](../../src/drivers/sim/measurement/control_bridge.py)
and [`SensorControlModule`](../../src/drivers/sim/measurement/control_module.py).
The [HTTP route](../../src/gateway/routes/sensor_control.py) is
`POST /api/v1/sensors/{sensor_id}/control`, with `request_id`,
`expected_product_session_id`, `operation` and operation-specific `params`.
Operations are `query`, `enabled`, `rate`, `noise`, `noise_enabled`, `recording`
and `gimbal`. See the [runtime API and status](../../sim/runtime/sensors/README.md)
for parameter shapes and unsupported-operation limits.

Effective settings come only from the ACK matching request, Product/source
session, sensor and model/reset generation, with `applied_sim_time_ns`,
`applied`, `reason` and `settings`. Bridge readiness, a sent request or queued
measurement publication is not that ACK. The wait is bounded to two seconds;
HTTP 504 means the result is unknown, not that the source certainly did not
apply the command. A control ACK is not a measurement-subscriber receipt,
persisted capture receipt or native inspection advancement authority.

[`MeasurementArchive`](../../sim/runtime/recording/measurement_archive.py) is
implemented and connected to optional formal measurement recording. Samples
are recorded only when enabled; the archive retains raw NPZ arrays/masks and
metadata under the existing run, with a complete-file index, bounded capacity
and explicit close/error summary. It creates no worker. A recording-enable
ACK alone does not establish that a sample reached disk; inspect `index.jsonl`
and `recording.json`. This shared path currently covers the three new
thermal/acoustic/semantic modalities, not all legacy RGB/Depth/LiDAR/IMU remote
controls. Camera gimbal is not a microphone operation, and semantic label noise
cannot be enabled.

### Controlled native control evidence

[`sensor-control-s07ezbq8/result.json`](../../build/inspection-tests/20261022/sensor-control-s07ezbq8/result.json)
records `passed=true`, scope `controlled_http_native_dds_mujoco_sensor` and
`factory_product=false`. The corresponding
[integration test](../../tests/sim/test_sensor_control_native_integration.py)
uses the real FastAPI route via TestClient, real native DDS client/owner
endpoints and an actual MuJoCo `mj_step` source loop. This is not a mock ACK,
but also not an external HTTP deployment or ProductControl activation test.

Seven actual ACKs for `robot.thermal` cover all seven operations: query, rate,
noise, noise_enabled, recording, gimbal and enabled (disable). The returned
settings show 20 Hz, configured K noise with seed 7 then disabled, recording
enabled, pan 10 degrees, and the final disabled state. Applied source times
range from 66 to 122 ms; source errors are empty and the source thread closed.
The [recording](../../build/inspection-tests/20261022/sensor-control-s07ezbq8/recording/)
is closed with one 2530-byte NPZ containing a 2x2 float32 Kelvin matrix, bool
validity mask and metadata, plus its index and summary. The fixture uses a
controlled thermal declaration, not the 64x48 package/eight-stream rig, an
acoustic/semantic remote same-run proof or an inspection route. It establishes
neither sustained throughput nor Factory Product/UE UI acceptance.

Evidence under `build/inspection-tests/20261022/`:

| Check | Result | Evidence |
| --- | --- | --- |
| Catalog with strict schemas | 110 passed, no skips | `measurement-catalog-schemas.xml`; existing Miniconda/jsonschema |
| Feeder, inspection, Gateway, assembly | 215 passed | `measurement-production-integration.xml`; project .venv |
| Compiled inspection Host reconstruction and API contracts | 23 passed | `measurement-compiled-host-gateway-green.xml` |
| Final generation/default/rebuild regression | 21 passed, 59 deselected | `measurement-compilation-increment.xml` |
| Compiler identity/generation cases | 79 passed | New compiler tests before the final graph case; overlaps the above |
| Targeted helper Mypy | 5 files passed | FormalMeasurements, measurement stack, inspection capture and Gateway service/routes; `--follow-imports=silent` |

The first graph-test XML is retained: a test-only baseline assertion was placed
in the wrong function and raised NameError after all wire assertions. It was
moved back; production checks were not relaxed. Existing cacheprovider warnings
and bridge type diagnostics remain disclosed. Feeder and capture independent
incremental reviews are APPROVE/CLEAR after their concrete findings were fixed.
The assembly/Gateway reviewer also returned APPROVE/CLEAR after six new compiler
type diagnostics were removed without changing runtime behavior. One existing
non-measurement compiler type diagnostic remains; whole-file Mypy is not green.

Remaining: selected Fab factory physical collision/map/device/source binding;
actual Factory Product activation and bounded same-Product native inspection;
UE thermal/waveform/label viewing and control UI acceptance; full legacy sensor
remote-control coverage. Reusable packages, acknowledged native controls and raw
archives are implemented, but these remaining gates are not accepted. Do not
relabel the legacy workshop as the downloaded Fab production line or infer
factory success from controlled tests.

Formal acoustic sampling currently requires integer samples per physics tick
(8/16/48 kHz at 200 Hz work). Incompatible 44.1 kHz/200 Hz now fails before engine
creation, without implicit resampling/restamping. Sources are finite waveform
clips, not endless machine sound models. RGB/Odometry DTOs have observation times
but not full measurement-generation identity; current tests prove temporal
pairing, not new identity fields on those legacy messages. Full goal is active.

## Native UE viewer / actual factory-geometry increment

Production sources: `LingTuSimUI/Private/LingTuSimMeasurementClient.*`,
`LingTuSimMeasurementProjection.*`, `SLingTuSimMeasurements.*`, integrated into
the existing HUD/WorldSubsystem. Optional `MeasurementPresentationBinding`
supplies the existing Product identity and Gateway base URL. It accepts HTTP(S)
and optional path prefixes, but rejects query/fragment/userinfo. Default `None`
adds no launch arguments. Source session and model/reset generation come from
the actual runtime snapshot, never a RunId or guessed HTTP response binding.

The viewer polls only while visible. It uses non-driving live-view mode rather
than pausing sampling; returning restores the original pause/resume target and
button focus. Query works for declared sensors before their first sample; an
exact typed ACK can establish control state without inventing measurement
identity. Slow list responses cannot starve previews. Hidden, disabled, expired
or rebound data clears. Display textures use nearest-neighbour filtering and
deferred GC/render cleanup. Raw download means the latest NPZ, not necessarily
the displayed frame, and browser launch is not proof of saving the file.

Final evidence under `build/inspection-tests/20261022/`:

| Evidence | Actual result / limit |
| --- | --- |
| `ue-measurements-build-05` | UE 5.8 build succeeded, exit 0 |
| `ue-measurements-contracts-03/report/index.json` | 30 succeeded; zero failed, skipped or warnings |
| `ue-binding-factory-final.xml` | 97 focused Python tests; existing disabled-cacheprovider warning |
| `ue-measurements-live-02/report/index.json` | One opt-in real-HUD rendered interaction test passed; actual Slate query/recording/enabled controls |
| `ue-measurements-live-02/{thermal,acoustic,semantic,disabled}.png` | Actual engine screenshots; controlled hot-body geometry, not Fab factory |
| `ue-measurements-live-02-source/result.json` | Nine native DDS ACKs; no source errors; naturally closed; Product not started |
| `ue-measurements-live-02-source/recording/recording.json` | 23 NPZ samples, 56,902 payload bytes, closed/error=null |

The fixture deliberately delays list responses 1.1 seconds and suppresses the
first semantic subscriber receipts until query ACK. It verifies safe first-frame
query, continued previews, disable/clear, re-enable/new data and page closure.
Measurement transport is in-process LTU1 decode; controls use native DDS. It does
not prove complete native measurement transport, Product activation, movement,
sustained rates, numeric settings UI coverage or factory inspection acceptance.

Earlier `live-01` remains preserved (seven ACKs, 21 NPZ/51,904 bytes). The first
contract filter omitted two display-math tests because their names include
`Runtime`; corrected final count is 30, not an additive total across runs. The
first build's deprecated screenshot API warning was fixed using the installed
UE API. No failure evidence was deleted or rewritten as timely success.

The separate `factory-device-measurements-01` contains real 64x48 thermal/semantic
sampling and dual-channel 8 kHz Pa blocks on the downloaded factory's 1065-geom
candidate. All original geometry remains; 9216 nearest-hit comparisons pass.
Device temperatures/emissivity, labels, finite sounds and static rig calibration
are explicitly synthetic declarations. Unknown surfaces/blocked paths remain
invalid. See the factory tool README: this is not a qualified WorldPackage,
robot-mounted calibration, RGB alignment or a completed inspection route.

## Startup selection increment

The robot-selection step now includes illustrated sensor enable/rate editing
and immutable SessionSpec authoring, with a ProductControl-only handoff for
custom choices. See [sensor-selection.md](sensor-selection.md) for the exact
checks, real-image provenance and acceptance ledger. This exposes the declared
rig rather than pretending every supported measurement model is already mounted.
Next are compatible multimodal mounting presets, same-Product live controls and
the actual Fab navigation/inspection acceptance; this increment does not close
those larger gates.

## Robot-mounted mixed-owner and selected-camera increment

The experimental
[`thunderv4_multimodal_unreal@1.0.0` rig](../../sim/packages/sensor_rigs/doso/thunder_v4/multimodal_unreal/README.md)
preserves the five original UE-navigation declarations byte-for-structure:
IMU 200 Hz, MID-360 10 Hz, UE RGB/depth 640x480 at 30 Hz and truth odometry
100 Hz. Thermal/semantic 64x48 at 10 Hz and a 16 kHz microphone are appended,
not substituted for the existing camera backend. Strict rig measurement
overrides reuse the sensor-plan schema definitions and reject wrong units,
unknown fields and non-physical mounting frames through schema/Catalog checks.

Actual Thunder MuJoCo geometry now verifies optical mounting on `camera1_link`,
world pose updates after base rotation, calibrated 15-degree software gimbal
changes and chassis occlusion under pinhole/fisheye/panorama projections.
`thunder-mount-proof-02` contains raw NPZ arrays and their display projections.
The declared 310 K/emissivity 0.9 calibration target yields approximately
308.44 K apparent temperature with the explicit 293.15 K background. This is
not a Fab device temperature, spectral LWIR model or an actuated mechanical
gimbal. Invalid and occluded pixels remain NaN/false, not synthetic ambient.

The separate `thunder-acoustic-mount-proof-01` uses actual Thunder base/mount
transforms at two fixed positions. Four raw blocks verify 16 kHz/160 samples,
Pa, the explicit software sensitivity and greater direct-path propagation delay
with lower pressure at the farther position. It is not robot motion, room
reflections, hardware calibration or a complete Product transmission test.

Formal headless RGB/depth scheduling now respects each selected stream's
enabled state and rate. Only selected writers and counters are created, busy
queue replacement/late slots are counted per stream, and partial transport
failure does not count an already completed peer as dropped. The acceptance
consumer follows the same names/rates instead of forcing paired equal-rate
frames. Its existing 1% drop, 5% rate-error and 3-second minimum observation
gates are unchanged; no test here establishes sustained live throughput.

The next integration boundary remains explicit: ProductControl must own a UE
visual/camera process following the **same** formal MuJoCo snapshot source and
immutable RunPlan. Starting the older playable coordinator inside the Product
would create a second physics/lifecycle owner and is not the repair. Until the
bridge is accepted and the selected Fab World/map/device sources are bound,
this mixed-owner rig is not offered as an accepted Product launch. The
2026-10-23 increment implements the exact explicit-Unreal route and rejects
unsupported/mixed routes before engine creation; actual rendered Product
activation and Factory inspection remain pending. The
[current delivery ledger](sensor-delivery-status.md) records the new relay,
actual Factory measurement candidate and still-unimplemented sensor types.

The REA follow-up is recorded in
[`CAMERA_BINDING_FOLLOWUP_20261022.md`](../../research/matrix/CAMERA_BINDING_FOLLOWUP_20261022.md).
REA 6.6.0 rejected the loose `BP_PTZCamera.uexp` format; cached payload anchors
were checked, but production Blueprint assignment/body-follow and infrared
material behavior remain unproven. No unsupported-temperature or acoustic
implementation is attributed to MATRiX.

Final counts, retained failures and process-scan scope are in the
[runtime/mount verification ledger](sensor-selection.md#runtime-selection--robot-mount-follow-up).
The directly affected producer/consumer/evidence subset passed 179 cases;
the wider 314-case run retained three tracking variant/manifest failures.
Separate strict-schema checks passed 54 cases, and actual Thunder optical
and microphone checks passed three plus one cases. These scopes overlap with
earlier development runs and must not be added as independent acceptance runs.

# One Product session: formal physics and UE presentation

Implementation started 2026-10-22/23. This record distinguishes wiring and
component checks from an accepted same-Product Factory inspection run.

## Ownership and flow

| Concern | Existing owner / connection |
| --- | --- |
| Compile, session, start/stop, rollback | ProductControl; exactly one immutable RunPlan |
| Physics, physical time, sampling, driver heartbeat | Existing `formal_feeder.py` and its single MuJoCo engine |
| Optional UE child | `FormalVisual`, a feeder-owned presentation component like the existing passive viewer; no second Coordinator or MuJoCo process |
| Robot pose shown by UE | Complete stable-ID body snapshots copied from that same engine, sent over the existing localhost latest-value UDP transport |
| Thermal/acoustic/semantic controls and archives | Existing native sensor endpoint, Host control bridge, matching source ACK, and run-owned raw archive |
| Inspection execution and final motion | Existing native inspection/navigation/safety owners; UE does not advance a point or manufacture evidence success |
| Sensor controls and measurement viewer in UE | Existing Gateway URL and exact `product_session_id` supplied by `MeasurementPresentationBinding` |

## Explicit activation, no silent mode substitution

An authored SessionSpec with `runtime.mode: unreal` is preserved by assembly.
The normal headless/preview defaults remain unchanged. Combining Unreal mode
and the passive MuJoCo viewer is rejected, including the direct feeder CLI
override. The editor installation path is explicit in the Env's existing
feeder command environment as `LINGTU_UNREAL_EDITOR`; the component does not
search for another installation or start a build.

ProductControl is still the public entry. Existing source selection uses
`python -m lingtu.control switch ... --env sim --backend mujoco --sim-session
<repo-relative-session>`. An inspection switch also requires the existing saved
localization map and its valid identity. No new manual launcher, alternate
Product resolver or map identity is introduced here.

## Startup and shutdown

1. The feeder builds its existing engine and native sensor connections.
2. For explicit Unreal mode only, it serializes the **already compiled** plans
   to `<Product session root>/visual/<product_session_id>/bundle`. This is a
   language-boundary projection, not Catalog resolution or package mutation.
3. `FormalVisual` creates a fresh canonical allocation and logs directory,
   preserves the selected DDS domain, and starts the existing `UnrealProcess`
   in SDK-quiet game mode. Its boot/run identity is bound to the Product run.
4. Before driver activation, the feeder publishes current physical snapshots
   and waits at most 45 seconds for UE's own `visual-readiness.json` to say
   ACTIVE with the same compiled session and model/reset generations. A camera
   ready file or the feeder's own ready file cannot substitute for this proof.
5. Normal feeder READY follows the existing physical activation checks. During
   execution, presentation publishes at most approximately 30 Hz without
   stepping physics or changing qpos/qvel. UE exit raises into the existing
   feeder emergency-stop path. Finally closes UE/UDP before the engine closes.

The snapshot uses compiled `session_id`, not Product ID, because the existing
UE wire contract distinguishes those identities. The allocation/log directory
provides Product ownership. UE visual readiness itself has no Product ID,
timestamp or truth sequence; do not claim those fields were verified in it.

## Camera boundary and same-owner relay increment (2026-10-23)

The feeder now accepts the exact `visual / unreal_camera / camera_shm` route
only in explicit Unreal mode. It creates no MuJoCo renderer for this route and
does not substitute another image source. One feeder-owned `FormalCameraRelay`
reads only the current allocation's named mappings and forwards the selected
streams through the existing native camera endpoint and Host file rings.
The compiler supplies the actual optical frame through the existing native
`--camera-frame` option, rather than leaving DDS images labelled `camera_link`.
Disabled peers remain absent. Mixed source owners and different RGB/depth
optical frames are rejected; no protocol or second session owner is introduced.

LTCSHM01 v1 keeps its 64-byte superblock and 256-byte slot headers. Slot flag 1
declares a little-endian `<QQq>` extension in the existing 24 reserved bytes:
latched truth sequence, reset generation, simulation nanoseconds. Header
reserved1 carries model generation. Old unflagged frames remain readable as
legacy frames, but cannot satisfy this simulation relay's identity checks.
The UE transport timestamp remains Unix time; it is **not** relabelled as
simulation time. The relay joins the captured truth to the already-published
same-engine snapshot and its original physics-to-Host epoch mapping.

During cold startup, physics is frozen. Retransmission repeats the same truth
sequence/time/epoch, not a new sequence with unchanged simulation time. A
matched warm-up image may validate source geometry and readiness, but retains
its original timestamp, is not counted as a runtime sample, and cannot become
fresh inspection evidence by restamping. After physical activation, source-age
checks apply independently of transport freshness. The stop-stream watchdog
uses the selected period, including low-rate cameras. Presentation uses an
absolute deadline with its fractional remainder; 200 Hz physics must not
quantize a 30 Hz request down to 28.57 Hz. Independent declared-slot counts
remain visible even when the source produces no frames.

These are implemented contracts, not an accepted real same-Product rendered
Factory run. The mixed-owner rig remains an authored candidate until actual
runtime activation, sustained rates, factory alignment and inspection gates
pass. The original 640x480@30 requirements are not downgraded.

The UE capture parser skips only the exact external camera route
`sensor_runtime / mujoco_camera / camera_shm`. Its own render streams remain
strictly validated. A valid plan with no UE-owned captures is an explicit no-op,
logged as `LINGTU_CAMERA_NO_UE_CAPTURES`: no camera SHM, no camera ACTIVE and no
fabricated camera-readiness file. Non-camera payload validation stays with its
own modules/Catalog, not this camera parser.

## Evidence and remaining work

Initial Python RED: `product-visual-owner-red.xml` (five expected new failures,
one default-headless pass). After wiring, the initial combined regression
`product-visual-owner-regression-01.xml` passed 181 tests. The subsequent
`product-visual-measurement-owner-01.xml` passed eight owner tests, including
the real MuJoCo thermal/control-ACK/raw-archive path with a substituted UE
process boundary. An earlier green attempt retained a test-only failure from
expecting a nonexistent stop-payload field; the corrected assertion checks
the actual terminal ACK and zero-command event.

`tests/sim/test_formal_visual.py` separately checks actual MuJoCo transforms,
velocities, integration-state immutability, detached snapshots, real loopback
UDP, source identity and partial-start cleanup. Its UE process boundary is
substituted; these tests are not a live Product, Editor frame-rate or motion
qualification. Counts overlap development runs and must not be added together.

Remaining before full delivery:

- Actual UE-authored visual ACTIVE against this feeder component. The controlled
  build and source-ownership Automation are complete, as recorded below.
- Same-Product runtime activation, Host/native source ACKs and UE live viewing.
- Bind the actual Fab WorldPackage, saved map, device properties and three
  inspection points, then verify navigation, takeover, capture and report.
- Actual rendered Product-owned UE RGB/depth relay and sustained-rate acceptance;
  relay code/contracts now exist; legacy sensor remote-control coverage remains
  distinct.

## Final implementation checks, 2026-10-23

| Evidence under `build/inspection-tests/20261023/` | Result / scope |
| --- | --- |
| `product-visual-python-final.xml` | 202 passed, zero failures/errors/skips; assembly, optional owner, helper, cameras and formal measurements |
| Focused Ruff / Mypy | Six Python files passed both; final diff whitespace check passed |
| Independent review | Parent/helper Python APPROVE; separate C++ ownership trim APPROVE |
| `product-visual-build-01/build.log` | UE 5.8 Editor incremental build succeeded, exit 0, 199.68 seconds; two compile actions plus linking/metadata |
| `product-visual-camera-automation-01/report/index.json` | 18 succeeded, zero failed/notRun/succeededWithWarnings, exit 0; NullRHI, not rendered sensor throughput |

The Automation editor process naturally exited after 168.937 seconds including
startup/shutdown; the test report's execution time alone is about 0.268 seconds.
Neither number is a measurement of this Product's game-mode startup. The
45-second presentation wait and existing Product startup deadline remain
unproven against a cold real game launch; a missed deadline must fail and clean
up, not silently extend or claim readiness. No Product was activated, no UE
rendered same-engine first frame was accepted, and no robot motion or inspection
task was started by these checks. In particular, compiled code and this NullRHI
report do not satisfy the remaining actual same-Product runtime gate.

### Camera relay follow-up, 2026-10-23

Final focused regression `product-camera-relay-final-reviewed.xml` passed 216
cases, with zero failures/errors/skips. Compiler optical-frame/measurement
contracts in `ue-camera-frame-green.xml` passed 88 cases. Independent review
closed frozen startup identity, absolute cadence, low-rate watchdog and failed
cleanup evidence defects; its final scoped verdict is APPROVE. Original RED
reports remain alongside the GREEN artifacts. These counts overlap earlier
checks and are not additional runtime runs.

`lane-c-shm-truth-01/validation-result.json` records a single successful UE
build and NullRHI Automation: CameraShm 6/6 and CameraCapture 18/18. This verifies
the new ABI extension/logic, not actual render readback throughput. Actual
same-Product first frame and full Factory inspection are still pending. The
[delivery ledger](sensor-delivery-status.md) includes process-scan scope,
remaining World/map gates and explicitly unimplemented modalities.

Postflight at `2026-10-23T00:24:20+08:00` found no processes matching the
scoped UE/UBT/UAT/dotnet/native nav/SLAM/map/simulation-publisher names. Both
owned build and Automation commands exited. This is a scoped cleanup check,
not a claim that unrelated applications or all IDE build workers were closed.

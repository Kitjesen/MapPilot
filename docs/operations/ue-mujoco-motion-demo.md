# Thunder UE / MuJoCo motion demonstration — 2026-10-13

This is a real runtime demonstration, not an offline pose preview or a complete
playable/performance qualification. MuJoCo owns motion and contact; UE follows
the native body snapshots. Existing ThunderV4 1.0.3 and FactoryPark_HF 1.2.1 are
unchanged. A separately resolved diagnostic session spawns Thunder at
`[0,-69,0]`, yaw +90 degrees, facing the existing south speed bump.

## Demonstrated result

Run: `build/ue-motion-runs-20261013/manual-20261013-154920-c6b93957`.

- `ue-motion.mp4`: DirectX Desktop Duplication of the owned client region,
  H.264 1920x1080, 10 fps, 17 seconds, wall-clock playback (not sped up).
  Frames extracted at 2s and 14s visibly differ in robot/world position and HUD
  truth sequence (3523 versus 5367); the movie is not a frozen-window capture.
- W only, no Shift: 0.329899 m displacement during the accepted movement interval.
- Release: 1.79 simulation seconds observed; maximum drift 0.076282 m,
  final planar speed 0.018696 m/s. Release request, accepted zero command and
  released authority were observed, but the drift **exceeds the 0.05 m gate**.
- Body remained upright through movement/release: minimum height 0.457690 m,
  absolute roll <=0.011230 rad and pitch <=0.006210 rad in that interval.
- Real `mjData::contact` recorded both front wheels against `speed_bump_south`.
  Sampled peak normal forces: FR 30.404108 N, FL 20.102850 N. This proves contact,
  not successful traversal of the obstacle. Ground/wheel contact is also recorded.
- 542 truth snapshots preserve contact data. The whole-run final displacement
  (0.225700 m) differs from the hold-interval displacement; do not interchange them.
- Runtime episode completed successfully and the launcher closed. The overall
  locomotion gate remains **FAIL**: press/hold control-origin log correlation and
  post-release drift did not pass. Displacement, upright and explicit release
  transport subchecks passed.
  Do not rewrite that artifact or report overall qualification as passed.

## Repairs included

- Native C++ snapshots expose geom names/IDs, body stable IDs, contact distance,
  and normal solver force (null if unavailable), including realtime sampled IPC.
- The visual UDP publisher omits contact details without mutating recorded truth.
- Windows receiver validates actual received datagram bytes, not FIONREAD queue
  bytes. Local reproduction: five 12,000-byte datagrams report 60,000 queued
  bytes but a single recv returns 12,000. New DLL produced no oversized warnings
  in the final run.
- UE 5.8 cached-lighting pre-exposure is 8, covering physical daylight EV14.
  The demo additionally uses a documented +2-stop exposure offset. Art direction,
  ambient fill, materials and performance still need work.
- The diagnostic input probe follows current direct-key controls and checks real
  release/stop, not the obsolete Shift-held sequence.

## Verification and limitations

- Native C++ contact test: 1/1 passed.
- Combined Python focused tests: 23 passed; publisher/follow/fanout regression:
  13 passed (overlapping suites, not additive totals).
- Controlled UE build: `build/unreal-tests/ubt-udp-queue-20261013.log`, Succeeded.
- Lint passed for touched Python sources/tests.
- An earlier attempt used continuous UE PNG screenshots, lost foreground, and
  failed a 1-second camera pipeline deadline. Its FAILED artifacts are retained.
  The final run instead recorded the owned window without continuous UE PNG export.
- The intermediate `manual-20261013-153531-0a21277d` GDI window recording is
  **invalid as motion video**: sampled frames retain HUD truth 4512. Preserve it
  as failed capture evidence; do not use it for the demo. That run's independent
  truth/screenshot evidence remains valid (0.247893 m hold displacement and
  0.008581 m release drift). Those more favorable stop metrics must not replace
  the latest run's failing stop result.
- This demo uses a decimated 64-ray Mid360 preview. It does not qualify full-rate
  LiDAR, 30 Hz camera throughput, stable real-time factor, or packaged distribution.
- The existing manual harness and generated bundles are local `build/` artifacts;
  do not treat their presence as a portable installed product or CI dependency.

## Local operation

Double-click `build/manual-playable/PlayThunder.cmd`. This starts the tested
diagnostic bundle and native contact host, without automated key injection.
Click the game window; W/S move forward/back, A/D strafe, Q/E turn. Release movement
keys to stop. Close the game window normally to let the launcher clean up.

Next focused work: fix post-contact release drift and press/hold evidence correlation; establish a performance
budget without synchronous screenshots; improve scene/robot lighting and materials;
then test longer routes and obstacle traversal. Do not rework the message protocol
or replace the robot model to address these issues.

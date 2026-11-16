# MLS staircase delivery, 2026-11-17

This report separates static planning, actual MuJoCo motion and field readiness.
The changes were developed on 2026-11-16 and verified through 2026-11-17.

## Delivered behavior

- Normal MLS queries use Rust for endpoint attachment, route search, body,
  support, step/slope, no-go, translation sweep and vertex turns. The query
  returns the exact metric poses it checked. C++ smoothing calls that checker.
- SCAN and global planning derive radius and front/rear offsets from the same
  configured cylinders. Vertical clearance retains the existing map-cell
  rounding. A separate global radius no longer overrides that body.
- Thunder's sim package supplies the existing 30-degree slope parameter, as
  the real configuration does. Sim does not load real RobotConfig.
- Cross-height candidates cover the required diagonal and longer lead-in
  connections. The minimum run from a slope limit is not a maximum search radius.
- Body candidates use the occupied voxel top plus the existing support height.
  Surface selection preserves the original voxel-interval tolerance, while its
  returned height is the physical top. Measured endpoint heights stay intact.
- Initial pose tolerance does not exempt the rest of an entry segment from
  ground support. The recorded 0.8 m unsupported gap is rejected.
- Same-map committed revisions update the Rust artifact regionally, including
  cache-restored artifacts. Complete columns and surrounding extraction context
  prevent artificial surfaces inside a clipped high wall.
- Smoothing preserves the existing soft wall cost. Formal defaults are weight
  2 and radius 8 map cells. These are route preferences, not additional hard
  clearance or a change to the body dimensions.
- Occupancy columns avoid scanning empty Z cells in body/sweep queries. Same
  input comparisons preserve first-hit and cancellation behavior.

The existing legacy/free-space C++ planner and offline C++ verifier remain.
They are not an additional admission stage for a successful normal Rust route.
The final ownership cleanup returns Rust exhaustion/errors directly instead
of launching a second C++ search for that same MLS request.
The default local navd was rebuilt after this cleanup at 2026-11-17 00:21:31
(3,180,032 bytes). Eight blocked matrix requests were rerun and returned Rust
exhaustion without a C++ retry. A navigation-only integration delta against
`d02b427c` is saved at `build/mls-unified-20261116/final-nav.patch`.

## Static scene results

The 16-scene staircase matrix contains 32 directional requests: 24 Rust routes
reach the requested target and pass geometry checks; eight physically blocked
requests return no route. The matrix includes L/U landings, four flights across
three levels, same-XY stacked floors, an alternate staircase and a long approach.
The largest repeated stair rise in this matrix is 0.30 m; maximum total ascent
is 4.8 m. Source and results are in `build/mls-unified-20261116/accepted/`.

Four additional saved-map cases, including 0.20 m voxel stair/multifloor maps,
now return Rust routes rather than successful C++ fallback routes. Repeated
query/cache results are in `build/mls-unified-20261116/coarse-accepted/`.

The 0.05 m boundary maps use radius 0.32 m, front/rear offset 0.23 m, support
height 0.435 m, tolerance 0.10 m, maximum step 0.45 m and maximum route slope
tan(30 degrees). Results are in `build/mls-limits-20261116/accepted/`.

| Scene | Forward / reverse result | Scope |
|---|---|---|
| Single 0.35, 0.40 and 0.45 m risers | All Rust routes and geometry pass | Static body-reference routes |
| Single 0.50 m riser | Both return no route | Above the configured 0.45 m step |
| 20, 25 and 30 degree ramps | Both directions pass | Actual voxel end-to-end angles 20.136, 25.017, 29.899 degrees |
| 35 degree ramp | Both return Rust routes | Zig-zag routes, not straight ascent |

The 35-degree route has maximum segment slope 0.573462, below tan(30 degrees).
Planning took 77.144 s forward and 23.336 s reverse. This case has **not passed
performance acceptance**. A bounded diagnostic located the time in Rust query
search/geometry; disabling soft cost did not remove it, and C++ smoothing was
only about 16.6 ms in the separated diagnostic. Other successful boundary cases
took 0.475-1.622 s in this local run. These are standalone native request times,
not isolated warm-query benchmarks or field latency guarantees.

## Actual MuJoCo motion

One formal ProductControl run used `stair_showcase`, its matching
`showcase_rays_v4` map and the current CTS MoE v39 policy. The scene has twelve
0.06 m rises, total 0.72 m. The earlier proposed six-by-0.20 m map was discarded
before running because it did not match this world.

The run exercised native global planning, SCAN local planning, the native
driver bridge, ONNX policy and MuJoCo physics. Localization was MuJoCo truth,
so this does not validate LTLIO. The executed default navd was built at
2026-11-16 23:43:10. Both global requests used Rust, with no C++ fallback.

| Leg | Native outcome | Time to arrival | XY error | Height error |
|---|---|---:|---:|---:|
| Up | REACHED / SUCCESS | 24.96 s | 0.2019 m | 0.0169 m |
| Down | REACHED / SUCCESS | 26.12 s | 0.1257 m | 0.0372 m |

Measured path length was 13.845 m. The round trip ended about 0.1014 m from its
initial position. Final Product stop and process cleanup were verified.

**Functional arrival passed; complete acceptance did not.**

- Combined readiness was 182/217, or 83.87%. Main recorded causes included
  deadline misses, high loop utilization and stale collision input. Maximum
  loop interval was 73.96 ms; maximum work time was 42.87 ms.
- About 62.11 s of simulated physics took 75.77 s of wall time: factor 0.820.
  IMU/LiDAR were approximately 199.95/9.98 Hz per simulated second, but
  163.91/8.18 Hz per wall second. Scheduled and published counts matched and
  dropped counts were zero; that is not proof of complete DDS delivery.
- 1,953 physics steps recorded non-support contact. The first was the front
  right wheel/foot against the first riser's side, about 0.955 mm penetration
  and 433.87 N normal force. The force is the first event's value, not a measured
  peak. The available log does not classify every event, so it does not prove
  zero body contacts or qualified stair locomotion.

The original report is retained under
`build/mls-unified-20261116/mujoco-stairs/report.json`. `recomputed.json` evaluates
the same recording after fixing two harness errors: a round trip must not fail
for returning near its start, and a stop request that was never issued does
not require an acknowledgement. Per-leg progress, height, requested-stop
acknowledgements, final Product stop, contact and readiness requirements remain.
The formal manifest also removes obsolete model switching: all three old
profile names already resolved to the same MoE policy. No video was generated.

## Evidence and limits

Rust, native geometry, cache/region updates, configuration and goal-request
tests accompany the changes. The regression logs and builds are under
`build/mls-unified-20261116/`. Acceptance runner regressions cover single-policy
height checking, round-trip progress and requested versus unrequested stops.

The hardware-safe claim is limited: the planner has been checked on the stated
maps, and one 6 cm-step physical simulation reached both goals. A 45 cm physical
step, 30-degree physical climb, LTLIO performance and NX/real robot deployment
have not been demonstrated by this work. Slow 35-degree queries and simulated
real-time/contact quality remain explicit unresolved items.

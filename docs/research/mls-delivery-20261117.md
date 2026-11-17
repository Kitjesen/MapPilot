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

## Startup audit and terminal repair (2026-11-17)

Startup decisions have distinct owners; this repair adds no startup state or
motion threshold:

| Decision | Current owner and behavior |
| --- | --- |
| Who may command motion? | `ControlAuthority` holds estop, operator takeover and fault holds. Goal admission requires autonomy mode and a request newer than the last resume. |
| Can the current pose and map be used? | Goal admission checks map identity/frame; `InputGate` checks required odometry, transforms, localization, clouds and local collision data. Missing pose can queue an external goal. Driver status is reported here, not an additional rejection branch in `InputGate`. |
| Is there a usable departure and route? | Rust checks the actual-start connector, support, body, steps/slope, no-go and route sweep. A missing observation at the actual start does not exempt an unsupported departure segment. Local SCAN still checks its trajectory against current observations. |
| Can this command be executed now? | `FinalControl` checks measured braking and the proposed command (`brake_blocked`, `cmd_blocked`, `cmd_limited`). Driver/platform own command freshness, control lease and hardware readiness. |

The removed execution-clock rule that paused/cleared the trajectory after a
0.2 s gap is absent. The remaining 0.2 s bookkeeping in manual yaw control is
not that rule. FinalControl still derives its reaction horizon from
`max(teleop_cmd_max_age_s, 1/tick_hz)`; this audit did not establish a measured
replacement latency or change that behavior.

The integrated `start_evidence` log already admitted a 2 cm offset beyond the
observed floor edge. The failing assertion instead expected a 22 cm unsupported
entry to succeed. The corrected test rejects that entry and succeeds again
after restoring observed ground. It retains initial saved-occupancy departure,
actual-start preservation, request isolation, outside walls and no-go tests.
This is not a universal 2 cm permission threshold; it is a fixed regression.

The real migration omission was terminal-region handling: the metric Rust query
received the exact target but not the existing total/XY/Z goal tolerances.
Those three values now cross the ABI. Rust retains a valid raw goal, otherwise
resolves a nearby terminal within all three limits using the existing anchored
terminal lattice. Its vertical spacing preserves the C++ slope subdivision.
The 0.410 m test now resolves to 0.400 m, matching the original planner, rather
than the incomplete migration's 0.375 m choice. Connectors and the final route
remain checked in Rust; no production C++ fallback was reintroduced. Terminal
resolution shares query cancellation and iteration budget. ABI is now 10;
prepared artifacts with a different ABI are rebuilt through the existing cache
contract.

Validation after the final correction:

- Primary workspace Rust: 79 passed, one explicitly ignored release benchmark
  (`build/mls-unified-20261116/startup-rust.log`).
- Primary Windows native route checker and prepared surface: 2/2 passed,
  including terminal-region bounds and geometry
  (`startup-native-tests.log` in the same directory).
- Freshly built control authority, operator authority, input gate, goal plan
  controller and teleop safety test programs: 5/5 passed
  (`startup-gates-tests.log`). These do not replace a physical autonomous run.
- Integration worktree Linux prepared surface and start evidence: 2/2 passed,
  retaining strict C++ terminal equality and the unsupported-entry negative
  (`../lingtu-pr-repair/build/start-goal-ctest.log`).
- Default local Windows `build/nav-cpp/windows-x64-nav-endpoint/Release/navd.exe`
  rebuilt on 2026-11-17 at 01:45:57, 3,194,880 bytes. The build still reports
  upstream `lbfgs.hpp` C4702/C4701 warnings; this repair does not change that file.

No new MuJoCo round trip or NX deployment was performed for this startup repair.
The earlier readiness, slow-cycle, contact-quality and 35-degree performance
limits above remain open; passing the startup regressions does not close them.

## Wider departure and global matrix (2026-11-17, subsequent user request)

The user requested more permissive startup and a wider global-planning test.
The earlier 2 cm / 22 cm examples used a small, single-cylinder regression
fixture. This round separately uses the declared Thunder twin-cylinder envelope
(radius 0.32 m, offsets +/-0.23 m, support height 0.435 m) on 5 cm maps.
These distances measure the actual body reference point to the observed floor
edge, not foot placement, step height, or a measured physical void.

The previous first-pose-only exception accepted 2 cm but rejected 4 cm in the
same map. It also rejected a small unobserved patch under the initial body.
Departure now tolerates missing support samples inside the **initial** body XY
envelope, on the first connection only. The target of that connection still
needs support. Occupancy and no-go are still checked; an occupied column below
the sample preserves a measured drop/height mismatch rather than treating it
as missing observation. The initial envelope uses measured heading and does not
move along the route. No new distance threshold, map edits or state bits were
added. This is a local startup policy, not a claim about measured foot support.

DimOS reference remains commit 26144f7fd8424bdc33ab97dcd58e9497f2d413ee:
`planner.rs::snap_candidates` searches within 1.5 m and tries up to 64 starting
candidates. Its snap radius is a candidate-search range, not a guarantee that
the physical robot can traverse an unsupported connector. LingTu retains its
physical connector and collision checks, with the bounded departure exception.

Cleanup on the touched path removes the always-true `support` argument from
Rust `sweep`. When missing support and a preblocked body overlap, the secondary
clearance query now returns its actual failure instead of reporting the earlier
support failure. The C++ comparison implementation follows the same behavior.
Tests cover initial versus later segments, initial yaw, no-go, measured lower
ground and a gap extending outside the initial envelope.

Evidence: `build/startup-global-20261117/` retains scripts, maps, requests,
before/after routes, geometry results, images and the standalone viewer.

| Batch | Requests | Result |
| --- | ---: | --- |
| Departure/heading/door/obstacle matrix | 69 | 18 to 61 checked routes; 43 prior rejections become valid routes. Eight still reject: two 45 cm sideways starts, three 0.5 m mid-route gaps, three 0.6 m doors. |
| Existing stairs/overlap/clearance matrix | 32 | 24 Rust routes, eight blocked cases. All 32 route arrays match before/after exactly. |
| Step and slope probes using resolved sim configuration | 16 | 14 Rust routes, two 50 cm-step rejections. |

Total: 31 scenes, 117 requests, 99 checked Rust routes, 18 no-route results.
An unobserved 40 cm-wide patch under the body now connects in all three tested
headings. The 0.8 m door and box detour pass; the 0.6 m door and mid-route gap
remain blocked. Single steps 35/40/45 cm and nominal 20/25/30 degree ramps pass
in both directions. The nominal 35 degree map also returns geometrically
checked winding routes, but took 101.503 s forward and 22.194 s reverse in this
run. These elapsed times are observations under concurrent task load, not an
isolated performance comparison or proof of physical 35 degree climbing.

Rust: 80 passed, one ignored release benchmark. Native prepared-surface and
route-check smoke pass; the latter now includes departure, departure_no_go and
measured_drop A/B cases. The default local Windows navd was rebuilt with this
behavior at 02:32:01 (3,196,416 bytes). No field deployment or physical MuJoCo
run was performed in this batch. The 35-degree performance issue remains open.

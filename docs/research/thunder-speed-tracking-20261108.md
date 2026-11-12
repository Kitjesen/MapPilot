# Thunder navigation speed and tracking, 2026-11-08

The nav Product now requests forward speed 1.2 m/s, acceleration 0.75 m/s²,
and SCAN position gain 1.5. Lateral speed retains its 1 m/s upper limit;
there is no minimum lateral command. Real RobotConfig still clamps Thunder
forward speed to 1 m/s. This is local MuJoCo evidence, not a hardware release.

## Why low lateral commands barely moved

The CTS v39 observation command slots retained the requested values. Independent
four-second commands after three seconds of standing produced these means:

| Command m/s | Actual body-axis mean m/s |
| --- | ---: |
| vx +1.0 | +0.913 |
| vx +1.2 | +1.105 |
| vy +0.25 | +0.044 |
| vy -0.25 | -0.026 |
| vy +0.35 | +0.253 |
| vy -0.35 | -0.269 |

The previous -0.175 m/s test produced only -0.007 m/s lateral motion. The saved
training configuration's nonzero flat-terrain lateral sampling starts at 0.2
m/s, but even 0.25 was weak here. These experiments locate the response problem
after command delivery; they do not isolate policy training from contact and
friction effects. A forced 0.35 m/s minimum would change stopping behavior and
is not introduced. The policy weights and observation mapping are unchanged.

SCAN uses spline velocity plus position error times a gain. Thus raising only
the maximum speed cannot increase a small unsaturated correction. The existing
position gain is adjusted instead, with measured navigation outcomes below.

## Changes and ownership

- `config/system/products/nav.yaml`: the three nav-specific values above.
- `endpoint/nav/app/config/build.cpp`: autonomous SCAN final limits use its X/Y
  follower budgets; CMU retains its existing budget.
- `endpoint/nav/safety/command.*`: represent X/Y limits separately, preserving
  total-speed, yaw, freshness and low-budget deadband behavior.
- `endpoint/nav/control/teleop.cpp`: operator takeover retains operator limits.
- `endpoint/nav/main.cpp`: refresh the resolved limits when switching planner.
- Existing RobotConfig clamping in `compile_plan.py` continues to bound real
  motion after Product and session overrides. Driver limits are unchanged.

No geometry, goal tolerance, heading gate, policy, or collision exception was
changed for these trials. No new controller or speed-floor logic was added.

## Matched comparison

Pinned SCAN is `348e8a590a50a5a6bbab8d8c6dcfd171f009be26`. Its example launch
uses max velocity 0.75, acceleration 0.5 and position gain 0.8. LingTu's previous
forward limit was 1.0. Heading preview 0.8 s and alignment gate 0.8 rad remain.

All four variants use the same frozen current binary, CTS v39, continuous
warehouse world, simulated-LiDAR OctoMap, endpoints, and acceptance thresholds.

| Variant (vx / acceleration / gain) | Active tracking s | Local curve distance RMSE cm |
| --- | ---: | ---: |
| 1.0 / 0.5 / 0.8 | 40.74 | 8.70 |
| 1.2 / 0.5 / 0.8 | 37.02 | 7.21 |
| 1.2 / 0.75 / 0.8 | 29.80 | 7.88 |
| 1.2 / 0.75 / 1.5 | 31.26 | 6.31 |

All four arrived without non-support contacts. Speed alone made the same bend
slower and introduced near-zero translation windows. Raising the shared
acceleration removed those windows in this run. Higher gain improved curve
tracking but was slower than the acceleration-only candidate and increased
the bend's roll range. Lower tracking error does not prove best-looking motion.

The final Product and rebuilt navd were separately verified. Evidence, including
the first door run that arrived but failed input-readiness acceptance, is kept
in `artifacts/nav/speed-tracking-20261108/README.md` and `formal-summary.json`.
The 35 selected configuration tests and four native test targets passed.
The first test link failure was fixed by keeping configuration assertions in
the configuration target and command behavior in the existing safety target.

Videos replay recorded physical qpos at the original driving timeline. Only
replay lighting is reduced to avoid clipping; world physics is unchanged.
Each tuning variant has one run. Truth localization is used; ProductControl,
Web dispatch, LTLIO accuracy, NX and hardware motion are outside this evidence.

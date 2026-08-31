# MuJoCo Native Teleop Evidence

Status: PASS within diagnostic component scope
Date: 2026-08-10
Environment: Windows x64, MuJoCo, native CycloneDDS processes

## Claim Boundary

This run proves the native typed operator-motion, navd, simulated physical
driver bridge, and stop/cleanup chain. It is not a formal Product pass because
the runner was invoked without a published RunPlan. It is not Brainstem or
S100P field evidence.

```text
evidence_scope: diagnostic_component
product_acceptance_passed: false
blockers: []
```

## Evidence

- The robot moved 0.864 m in XY and travelled 0.948 m along its simulated path.
- All 63 operator events were accepted: claim, 60 samples, hold, and release.
- The physical bridge applied 119 non-zero commands and observed Driver-ready.
- The last accepted physical output correlated producer identity, output
  sequence 197, and bridge command sequence 130.
- The bridge completed `deactivate_zero` at command sequence 132 and physics
  step 3437, then exited without a protocol parse error.
- `navd` reported `stop_confirmation_evidence=driver_ack` for pure `teleop`.
- The typed stop command returned success and four post-stop snapshots remained
  at zero command with no active operator authority.
- An autonomous goal submitted in `teleop` was rejected with
  `goal_not_allowed_in_teleop`.
- The sensor fixture published 2,045 IMU messages and 153 LiDAR frames. These
  streams are fixture evidence and are not requirements of the map-free
  `teleop` Product.

## Artifacts

- Report:
  `C:\Users\99563\.codex\tmp\lingtu-teleop-native-run-20260810-final\report.json`
- Report SHA-256:
  `382d9e1694c4fb9b5cf30531c1f06fd8074bf866abfc4100549312065322b315`
- Strict preflight:
  `C:\Users\99563\.codex\tmp\lingtu-teleop-native-preflight-20260810-final\report.json`

## Open Field Gate

The real motion gate remains BLOCKED because the S100P and Brainstem endpoints
were unreachable during this run. Promotion requires a current release on the
robot, `lingtu teleop-preflight --stage motion --strict`, checked initial-zero
Brainstem evidence, and a separately supervised bounded-motion run. No non-zero
robot command was sent while collecting this record.

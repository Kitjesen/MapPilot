# Navigation Compute Contract

Status: current native Product contract

This document defines where navigation computation runs. Both `real` and `sim`
Products use the native C++ endpoint; the Host is the command and capability
surface, not a second autonomy implementation.

## Ownership

| Layer | Owner | Responsibility |
| --- | --- | --- |
| Command and skill entry | `src/nav/services/`, `src/nav/commands/`, `src/nav/skills/` | Admit typed user/task intent and forward it through the native client. |
| Global planning | `src/nav/cpp/planning/global/`, `src/nav/cpp/endpoint/nav/runtime/goal/` | Build a verified route from the active saved map. |
| Route execution | `src/nav/cpp/navigation/` | Slice the active route, track progress, and coordinate recovery. |
| Local planning | `src/nav/cpp/planning/local/` | Produce a collision- and terrain-aware local path or trajectory. |
| Tracking | `src/nav/cpp/tracking/` | Convert the selected path or trajectory into pre-safety body velocity. |
| Final safety and authority | `src/nav/cpp/endpoint/nav/safety/`, `src/nav/cpp/endpoint/nav/control/` | Gate, shape, stop, and publish the final command. |
| Hardware command sink | native driver | Forward the checked command to the robot. |

## Data flow

```text
Host goal/cancel/stop/teleop intent
  -> native command adapter
  -> navd DDS ingress
  -> global planning when the command requires a saved-map route
  -> Executor
  -> CMU or SCAN local planner
  -> Follower
  -> final safety + control authority
  -> /nav/cmd_vel
  -> driver
```

The internal route, local-path, and tracking handoffs are direct C++ calls.
`/nav/global_path` and `/nav/local_path` are telemetry topics, not internal
control handoffs.

## Planner policy

- OctoPlanner3D is the default saved-map global planner.
- FAR is an explicit 2-D occupancy planner.
- CMU and SCAN are explicit local-planner selections behind one C++ interface.
- The CMU path bank lives at `src/nav/cpp/planning/local/cmu/paths/`.
- A backend failure stops or reports failure according to the native lifecycle;
  it does not start another Host planner.

## Map and frame policy

The maps service owns active map state and artifacts. `navd` validates and
snapshots the artifact required by the selected global planner. Live collision
and traversability inputs remain local runtime evidence rather than a second
authoritative saved map.

Saved-map goals and global routes use the configured planning frame, normally
`map`. Local planner inputs use one coherent frame per planning tick. Frame or
map-identity mismatches are rejected rather than guessed.

## Motion policy

- A goal or local path is never a robot command.
- Recovery output is pre-safety intent and passes through the same final gate.
- Teleop and autonomy share final safety and authority.
- Only `navd` publishes the final navigation command; only the driver forwards
  it to hardware.
- A running process or published path does not prove motion readiness.

## Verification

Use the native C++ planner, navigator, follower, endpoint safety, and command
lifecycle tests. Product acceptance must additionally prove typed DDS delivery,
readiness, correlated goal status, and confirmed zero output on stop. Field
claims require S100P evidence.

# Local Planner I/O Contract

Status: current native Product contract

The local planner is an in-process C++ boundary inside `navd`. Its public code
contract is `src/nav/cpp/planning/local/planner.hpp`; endpoint input assembly is
under `src/nav/cpp/endpoint/nav/input/`.

## Input

One planning call receives one coherent frame containing:

- current body pose and measured motion;
- the active global-route slice or assisted-motion intent;
- obstacle and collision evidence;
- optional traversability evidence;
- frame and source generations;
- a monotonic receiver timestamp.

An omitted or stale input is not replaced with retained data from an older
planning frame.

## Output

CMU returns a body-relative geometric path. SCAN returns an exact body-relative
B-spline for execution plus sampled path/trajectory telemetry. Both use the same
local planner entry point and are selected explicitly.

The result passes directly to the native `Follower`. `/nav/local_path` is
telemetry and is not re-subscribed as the internal control handoff.

## Assets and safety

The CMU path bank lives at `src/nav/cpp/planning/local/cmu/paths/`. Local planner
output is pre-safety intent: final obstacle, traversability, freshness, speed,
authority, and stop gates remain under `src/nav/cpp/endpoint/nav/`.

See `LOCAL_PLANNING_AND_TRACKING_CONTRACT.md` for algorithm and tracking details.

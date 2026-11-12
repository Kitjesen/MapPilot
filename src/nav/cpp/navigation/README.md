# Navigation execution

This directory owns execution of an already admitted route or assisted-motion
intent. It does not own global search, DDS, backend selection, or final robot
velocity.

## 中文导览

这个目录负责把“已经接受的路线或遥控意图”变成**候选速度**。它不负责生成全局路线，也不直接驱动机器人。普通导航的主线是：

```text
全局规划产生路线 → Executor 请求局部规划 → Follower 跟踪局部结果
                 → 返回候选速度 → 端点 FinalControl 做最终检查和发布
```

| 文件 | 作用 |
| --- | --- |
| [`executor.hpp / executor.cpp`](executor.cpp) | 统一入口和普通路线每拍执行：处理阶段、终点、局部规划结果，调用跟踪，返回候选速度。 |
| [`intent.cpp`](intent.cpp) | 辅助运动的完整流程：滚动参考、操作方向变化、局部规划、边界脱离及跟踪。 |
| [`route.hpp / route.cpp`](route.cpp) | 路线参考、进度与位置到达判断；map/odom/机身坐标转换、规划请求构造及诊断转换。 |
| [`pose_route.hpp / pose_route.cpp`](pose_route.cpp) | 把带方向的路线整理成行走或转身阶段，并对适合的阶段生成连续参考。 |
| [`state.cpp`](state.cpp) | 路线与姿态阶段的激活、限速、清除、暂停和恢复；实际命令反馈、运动状态及停滞记录。 |
| [`recovery.hpp / recovery.cpp`](recovery.cpp) | 路线受阻或运动停滞时，管理恢复动作及重试。 |
| [`recovery_search.hpp / recovery_search.cpp`](recovery_search.cpp) | 为 CMU 平移恢复搜索路径，并检查旋转恢复，供 Recovery 内部使用。 |
| [`stairs.hpp / stairs.cpp`](stairs.cpp) | 实验性的楼梯段匹配和进出梯状态；正式导航端点目前未接通这套流程，不等于 SCAN 的三维上下楼算法。 |

局部规划算法位于 `../planning/local/`，轨迹跟踪位于 `../tracking/`；最终运动检查由 `../endpoint/nav/control/` 所有。要读正常导航，先看 `route.hpp` 的 `Route`，再看 `executor.hpp` 的 `ExecutionInput` / `ExecutionOutput` 和 `Executor::tick()`；需要排查某种具体行为时，再进入对应实现。

`executor.cpp`、`intent.cpp`、`route.cpp` 和 `state.cpp` 实现同一个 `Executor`，没有新增管理器或重复状态。看正常路线从 `tick()` / `tickInPlanningFrame()` 进入；看辅助运动直接读 `intent.cpp`。辅助遥控的平移意图也可进入 `Executor::tick()`；纯原地转向则由遥控端点分流后进行最终检查。

最终控制通过 `commitApplied()` 反馈实际采用的命令，与 Follower 的同名接口一致；该调用不负责向驱动发送命令。2026-11-08 从较长的 `commitAppliedCommand()` 改名，源码调用、测试和基准程序同步更新，未保留第二个别名。

`Executor::resetMotion()` 重置局部规划、跟踪和恢复状态，保留已接受的路线与目标；
`clear()` 会同时清除路线。`pauseLinearMotion()` 则保留轨迹并冻结执行进度。
自主 SCAN 的最终碰撞拦截通过 `reportFinalMotionBlocked()` 撤掉被拒绝的局部轨迹，
随后从当前实测状态重新规划；全局路线和目标由 Executor 继续持有。
缓存的局部引导路径以 `guide_map_`、`guide_generation_`、`guide_stamp_s_` 表示：
分别为地图坐标下的路径、所属路线版本和更新时间（秒）。
空路径表示引导缓存无效；重新填充时同步更新版本和时间。临时 `segment` 由
`buildSegment()` 在每次构造时统一清空，路线生命周期不再重复清理。

## Data flow

Explicit `Route.posePoints` plus `Route.body` supports fixed-heading movement,
forward tangent motion and genuine in-place turns. `pose_route.*` preserves
same-position turns. Sampled edges may translate and change yaw when both
headings match forward travel within the route's yaw tolerance. Arbitrary
independent yaw/crabbing on one moving edge is not represented by this tangent
interface and is rejected explicitly. Mixed explicit stair/pose routes retain
their existing boundary. Ordinary goals use XYZ position; no requested final yaw
is sent to Follower. Trailing yaw-only pose stages are discarded, while turns
needed to join another movement segment retain their 3D sweep checks.

Compatible intermediate forward corners share one SCAN reference. Existing
fillet conditioning remains available; when it does not apply, the original
ordered XYZ polyline can be passed directly to SCAN. Its corner is no longer a
local stopping endpoint. SCAN validates its resulting spline, and the follower
uses that spline's tangent.

Separate turn stages validate the actual centre, measured yaw, turn sweep and
reconnection to the next segment. Intermediate arrival uses the existing goal,
spline-finish and support-height tolerances after that geometric check; a
centimetre-perfect base position or a separate observed-body snapshot is not an
additional prerequisite. Movement into a separate turn still settles first;
an aligned turn hands measured motion to the next translation without waiting
for another zero-velocity sample. The reference generation changes on stage
transition, not each control tick. Path headings guide movement and collision
checks; arrival does not add a final heading adjustment. Cancellation, replacement and frame changes discard pose
state and prior model candidates. Endpoint FinalControl remains required. Its veto stops commands and pauses execution;
it does not reset the local planner. SCAN retains ownership of trajectory
collision detection, replacement and emergency replanning.

The component probe in `tests/nav/cpp/benchmark_body_execution.cpp` runs
the real global planner, SCAN and Follower on fixed map fixtures. Its internal
feedback is kinematic command integration; the separate Thunder MuJoCo bridge
uses policy physics, contacts and ray-cast sensors. Historical sole-contact
experiments are not current per-foot admission requirements. Select the body
through the platform configuration and record the actual part set used by a
trial. See `docs/design/navigation-body-model-and-motion-posture.md` and
`docs/operations/error_record.md` for current failures and validation boundaries.

Follower tracks the supplied local plan with the SCAN controller. The reference
clock and position error determine spline completion. Executor checks the task's
position/height region; reaching an intermediate local endpoint does not finish
the task. A goal already inside that region with no executing spline needs no
new trajectory. The endpoint's existing stop acknowledgement still precedes
formal task success.

`FollowerGoal`, terminal-heading takeover, the minimum terminal turn rate,
terminal yaw hysteresis and the `goal_follower` settings were removed. Ordinary
heading alignment still freezes the reference clock, but does not skip the
existing linear acceleration limiter. Collision and emergency holds still stop
commands immediately. Applied-command feedback remains for platform rate limits.
Evidence: `build/scan-upstream-goal-20261108/`.

```text
GlobalPlanner -> Route -> Executor -> LocalPlanner -> LocalPlan -> Follower
                              |                         |
                              +------ Recovery ---------+ (CMU / teleop)
                                                        |
                                                 MotionCandidate
                                                        |
                                           endpoint FinalControl
```

- `route.*` defines the activated global `Route`, prepares the complete
  reference, and computes a bounded segment where the execution mode needs it.
- `executor.*` calls one already configured `LocalPlanner`, then one
  `Follower`, and returns execution state plus a pre-safety command.
- `recovery.*` coordinates CMU / assisted-teleop recovery. Autonomous SCAN
  owns its local retries without this additional loop.
- `state.cpp` owns odometry-derived kinematics and measured progress.
- `stairs.*` resolves measured stair surfaces against the actual 3D route and
  owns Executor's entry/profile/exit phases. It does not recognize point clouds,
  run global search, choose concrete platform models, or publish commands.

Autonomous SCAN retries inside its FSM at the upstream 100 Hz target period,
with a separate 20 Hz future-collision callback. After 1000 consecutive failed
planning rounds it stops, waits for measured 3-D speed below 0.1 m/s, clears
the local target and reports `NoPath`. Executor forwards `planning_failed` to
the existing task termination transaction, which confirms stopping and fails
the goal without an automatic global retry. It does not insert temporary
recovery goals, direct recovery rotations or an independent stall latch.
Solver duration and native handoff/command holds make this a failure-count
limit, not a fixed elapsed-time limit. FinalControl still checks final commands.

CMU recovery exhaustion retains its typed `local_recovery_exhausted` event and
the existing retained-goal retry path. Explicit cancellation, inspection action
failures and assisted teleop retain their existing business policy.

The endpoint's goal controller also retains an accepted initial goal when an
ordinary search finds no path or times out. It uses the existing pending request
and retry delay, takes a fresh start pose, and keeps the original task identity.
Until a route is activated the state remains Planning. Invalid requests,
algorithm exceptions, map replacement and explicit cancellation retain their
terminal handling; preview requests never start this motion-task retry loop.

CMU and teleop Recovery use current geometry through RecoverySearch. A supplied
Grid remains authoritative for those geometric checks when point sampling is
disabled. This separate policy does not participate in autonomous SCAN.

The five responsibilities are:

| Owner | Produces | Does not do |
| --- | --- | --- |
| Global planner | A route through the saved 3D map | Local control or recovery |
| LocalPlanner | An executable local path or spline using live collision data | Global search, task progress, recovery state, or final command publication |
| Recovery | Temporary goals, measured completion and retries; rotation/CMU candidate search | Replace the global route or publish velocity |
| Executor | Route/recovery coordination and a candidate command | Goal admission or final authority arbitration |
| FinalControl | The checked final command | Search routes or choose recovery candidates |

The endpoint's `GoalPlanController` owns goal admission, cancellation and
replanning. `Executor` coordinates an admitted route and assisted motion; it is
not the task manager or the hardware driver.

`Recovery` is Executor's CMU/teleop recovery state, not a separate global planner.
It tracks the selected candidate's progress and retries. Its rotation and CMU
search remain internal details.

`LocalPlanner` is the module name. SCAN is the default algorithm behind that
interface; CMU remains an explicitly selected alternative. Executor calls the
same module interface for either algorithm.

The local boundary uses `local_no_path`, `local_blocked`, and
`local_near_field_stop` for its stable status vocabulary. `execution_stalled`
belongs to Executor: it compares requested motion with measured progress.
SCAN's `searchReason` (also surfaced as the execution reason when relevant)
retains the specific search diagnostic. A diagnostic is not another owner of
the motion state machine.

Planner capabilities are derived at the local-planner boundary. Executor and
recovery consume these capabilities; they do not branch on Scan/Cmu. SCAN
does not consume the optional 2D traversability grid, and its request identity
ignores that grid's generation. The grid remains available to the CMU backend
used by other Products.

## Local planning boundary

`Executor` submits one `LocalPlanRequest`:

```cpp
struct LocalPlanRequest {
  RobotState robot;
  LocalObjective objective;      // RouteTarget or MotionIntentTarget
  LocalRouteView reference;      // Complete planning-frame reference
  EnvironmentView environment;  // obstacles, collision, traversability
  PlanIdentity identity;
  PlanClock clock;
};
```

`LocalPlan` carries one executable `FollowTarget`:

- CMU returns `PathTarget`.
- SCAN returns `SplineTarget`.

The Follower dispatches that variant internally. Executor converts preview
geometry and robot state between the target's frames, but does not select
CMU/SCAN, inspect backend strings, or manage SCAN task reuse and continuity.
The endpoint composition root chooses and configures the Planner before
constructing Executor.

For terminal alignment, the follower computes the yaw command and
LocalPlanner checks the turn sweep. These remain separate
in-place stages, not ordinary spline following. LocalPlanner also owns the
grid queries for pose-stage reconnect and assisted boundary departure.
These geometry queries retain the existing SCAN Grid implementation even
when CMU is selected; this change does not add a CMU-specific checker.
FinalControl checks braking with the configured runtime values when enabled.

Composition selects `PlanningFrame::Map` for SCAN and `PlanningFrame::Odom`
for CMU. SCAN's complete reference, robot kinematics, collision queries and
spline tracking share map coordinates; the Follower still returns body-frame
velocity. Small map-to-odom corrections update the measured robot pose, not
the reference generation or spline execution clock.

The complete map reference is prepared once after route activation using the
route's original heights. The map-frame SCAN path reuses that reference; the odom-frame path
continues to update its bounded local segment. Suspension or a frame-epoch
reset discards the reference and pending local work; the next valid input
rebuilds it. A new goal and cancellation use `setRoute()` and `clear()`
respectively, which reset the planner and follower.

The previous Executor shifted a route without a requested goal by the initial
body-to-route height difference (for example, 0.30 m became 0.48 m for a body
at 0.48 m). It now keeps 0.30 m: route points are already in the planning
frame, while rolling exploration assigns the observed body height at admission.
Only the initial projection point keeps the measured body height, preventing
an in-place vertical connection before approaching the original forward route.
The fixed [SCAN upstream](https://github.com/wuyi2121/SCAN-Planner/blob/348e8a590a50a5a6bbab8d8c6dcfd171f009be26/src/planner/plan_manage/src/scan_replan_fsm.cpp)
uses its configured body height to convert ground-reference input; that frame
conversion remains in the SCAN adapter and is distinct from shifting the route.
`Executor.RouteHeightIsNotShiftedToRobot` covers the before/after example.
`Executor.ScanReconnectsSmallHeightDifference` covers a requested-goal route
with the measured start 3 cm above/below it. Both connector variants planned in
this empty-map test; retaining the measured projection avoids a synthetic
vertical first edge without claiming a reproduced field planning failure.

The global route is one ordered polyline, not a stream of new goals for its
individual points. LocalPlanner owns local target selection and replanning along
that polyline. Execution feedback goes to the endpoint's goal lifecycle; Executor
and LocalPlanner never invoke global search themselves.

## Executor ownership

Executor may:

- activate and clear a Route;
- project progress and choose the local route segment;
- request a local plan;
- invoke recovery;
- follow the returned target;
- report target, preview path, control hints, and a pre-safety velocity.

Executor may not:

- run global search;
- select/configure a local backend;
- decode or publish DDS;
- perform final authority or safety arbitration;
- duplicate CMU/SCAN continuity state;
- branch on free-form planner failure strings.

The endpoint `FinalControl` remains the sole owner of shaping, authority,
hard-stop policy, and the final velocity publication.

## Stair segments

The following describes an experimental native prototype. The navigation
endpoint does not currently attach stair segments, request stair profiles, or
run the stair entry/exit handoff. Stair recognition, model switching, and
physical stair traversal have not been validated for deployment.

In the prototype, map-bound stair geometry can be attached as matching
`StairSegment` values to `Route`. Matching samples the actual 3D route against
the measured tread elevations and flight width; an XY/AABB overlap does not
attach a stair requirement. Both directions, partial flight routes and multiple
flights use the same resolver.

Before entry, Executor shortens the local reference to a landing position that
keeps the full robot extent before the first riser. After alignment and measured
stop it requests `stairs_up` or `stairs_down`. The driver maps that requirement
to the installed model/profile, owns any required Grounded transition, and
reports readiness only after actual profile readback and motion readiness.
Missing, stale, unsupported, or mismatching feedback holds the same route.

Local collision planning remains active while traversing. Executor limits speed,
rejects a local lateral detour outside the observed flight, and suppresses flat
ground recovery rotations/translations there. Existing measured-stall stopping
still holds zero; it does not complete or terminate the navigation task.

The driver receives `restore` only after the complete footprint clears onto an
observed exit landing and measured motion is quiet. Until then Executor holds
zero output and retains the stair profile. Once restoration has been requested,
temporary odometry loss during Grounded/Stand does not interrupt its renewal.
Connected flights needing the same profile can keep it
while approaching the next entrance. A goal outside the flight cannot complete
until clearance and restoration are confirmed. If that goal lies close to the
last riser, the local reference may extend onto the observed landing within the
goal's existing acceptance radius; the requested goal is unchanged. When no
whole-body exit fits that radius, execution waits with `stair_exit_target`.
A deliberate goal on a tread can complete while retaining the stair profile.
`clear()` and replanning retain a profile
already in use on a flight; a changed crossing direction holds on the existing
model until a landing permits switching. The commandless
`tickStairTransition()` allows the endpoint to finish a requested handoff or
landing restoration while ordinary motion is gated. These native contracts do
not establish that an installed robot model can climb the recognized stairs.

## MuJoCo single-floor goal check

Run from the repository root with its Python development environment and freshly
built native binaries. Prepare a saved `map.pcd` / `octomap.ot` bundle from the
same industrial-park geometry as the MuJoCo scene, then run:

```sh
python sim/scripts/mujoco/native_navigation_acceptance.py \
  --manifest config/acceptance/mujoco/scan_goal.json \
  --product-map <same-source-map-directory> \
  --out-dir artifacts/scan-goal
```

This entry starts and stops the formal `nav + scan` Product through
ProductControl. Robot, policy, spawn and native parameters come from RunPlan.
MuJoCo navigation uses truth localization by default; explicit
`env_config={"backend": "mujoco", "localization": "ltlio"}` selects the
separate SLAM path. Mapping Products retain their estimator-backed defaults.
Truth replaces localization only: registered clouds contain actual simulated
LiDAR returns, without synthetic ground patches.
Truth poses are published independently of raycasting; delayed scans retain
their capture pose without rewinding live odometry. Viewer refreshes use owned
snapshots off the physics loop. Endpoint input holds freeze both SCAN planning
time and Follower execution time without replacing the active reference.

SCAN's final command check sweeps the actual body's braking path through the
same LiDAR collision bitmap used by local planning. If the requested speed
cannot stop in the available clearance, it scales the complete twist down
along the same arc and retains the active trajectory. Occupied current space,
unsafe measured motion, or an unavailable safe prefix still stops execution.
The sweep keeps measured body height: an ascending global route cannot be
used as evidence that the physical robot has already climbed the stairs.

Disabling automatic recovery does not disable stall stopping. If a commanded
robot makes no measured progress for the existing recovery observation interval,
the executor holds zero output with `execution_stalled`. A new route or
operator intent clears this hold; periodic local replanning does not. This keeps
an unqualified locomotion policy from repeatedly pushing against the same step.

Stall progress follows the previous requested motion in the planning frame.
Translation accumulates signed horizontal displacement along the commanded
path-following direction; heading alignment accumulates rotation in the
requested direction. Reverse drift, cross-track slip, body heave and incidental
yaw do not refresh a translation timer. Instantaneous velocity alone does not
prove progress. The existing distance/angle threshold is capped at half the
requested travel accumulated in that observation window, so a slow command is
not required to cover an impossible fixed distance. This is a stall detector,
not a locomotion tracking-accuracy qualification.

When local planning returns `Pending`, no executable trajectory is available:
the executor sends zero and pauses this observation window without discarding
earlier motion evidence. A safe active SCAN trajectory is returned as `Ready`
even while its replacement is being prepared, so it keeps normal progress
monitoring. `Blocked` and near-field holds still count toward failed motion.

A translating recovery also rejects excursions beyond its existing cross-track
limit. Completion requires both along-path progress and actual XY proximity to
the exit; merely projecting onto the endpoint cannot finish recovery. A rejected
action advances the existing checked candidate sequence. These checks do not
raise recovery speed, change braking limits or establish that policy 4998 can
execute low-speed lateral commands.

The goal is `(56, 32, 0.3)`. A successful global plan or accepted command is not
arrival: the report requires physical travel, native `REACHED`, final position,
collision evidence and terminal zero acknowledgement. The Viewer shows the
global Route, local spline preview and actual traveled path. Keep failures in
the report; this entry is not a claim that long-range navigation already passes.
Use a fresh output directory per run. Contact results are finalized separately
from best-effort live display updates and must match the Product session.

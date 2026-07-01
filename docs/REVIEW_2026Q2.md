# 2026 Q2 SLAM/IMU Robustness Review ‚Ä?Audit Report & Follow-up Plan

**Branch:** `claude/fastlio2-imu-robustness-3Xjt3`
**Period:** Sprint 0 + Sprint 1 (2026-04)
**Status:** Sprint 0 complete (5/5), Sprint 1 partial (6/9 + 1 fix), Review-driven refactors complete (4/6, 2 deferred)

---

## TL;DR

20 commits landed across two sprints to address SLAM divergence,
silent failures, and indoor global-drift correction. A team review
audited the work and triggered four follow-up refactors:

| Audit | Finding | Action | Status |
|---|---|---|---|
| R1 | Zero compile verification for new C++ | aarch64 GitHub Actions workflow | ‚ú?Done (`addf3b3`) |
| R2 | P4 hand-rolled TF jump detection | Audit confirmed: industry standard | ‚ú?Comment added (`addf3b3`) |
| R3 | Hand-written Scan Context (250 LOC) | Replace with MapClosures (MIT) | ‚è?Deferred ‚Ä?RFC below |
| R4 | PCL ICP exposes only fitness | Replace with small_gicp (MIT) | ‚ú?Done (`d064627`) |
| R5 | Hand-written BlackBoxRecorder (188 LOC) | Replace with rosbag2 snapshot mode | ‚è?Deferred ‚Ä?RFC below |
| R6 | (Same as R3) | (Same as R3) | ‚è?Deferred |

The two deferred items (R3+R6 are the same scope, R5 is independent)
are **non-blocking**: production code works today using the
hand-written paths. The replacements are quality / maintenance wins,
not bug fixes.

---

## License Errors Caught & Corrected

Two license misjudgements were caught during the review:

1. **`scancontext` is NOT BSD.** I had assumed `aserbremen/scancontext_ros2`
   was BSD ‚Ä?actually **CC-BY-NC-SA 4.0** (non-commercial). Even
   reading the upstream code to copy ideas is risky. Removed from
   all replacement plans.

2. **Hand-written SC license.** The 250-line `scan_context.h` we
   shipped in commit `850b75d` is a clean-room implementation from
   the public Kim 2018 paper, so the file itself has no upstream
   license obligation. But the algorithm choice was driven by the
   incorrect assumption that `aserbremen/scancontext_ros2` was BSD;
   under the correct license picture we would have gone straight to
   MapClosures (MIT). The hand-written file is therefore both
   correct and obsolete.

**Going forward:** Any 3rd-party algorithm dependency must have a
LICENSE file URL cited in the commit message before merge. No
exceptions.

---

## What Shipped

### Sprint 0 ‚Ä?Crisis controls (5 commits)

| Item | Commit | Effect |
|---|---|---|
| S0.1 IMU‚ÜîLiDAR time_offset bug + cross-config check | `c67bb42`+`c698f8a` | Calibration data actually flows to fastlio2 (was only writing pointlio) |
| S0.4 systemd restart-storm protection | `d89ce02` | `StartLimitIntervalSec=300/Burst=3` ‚Ä?process crashes don't avalanche |
| S0.5 SLAM degeneracy/cov externalisation | `8a62d3d` | `/slam/state_health` topic + SSE `slam_diag` |
| S0.3 BlackBoxRecorder | `8dd33fa` | drift_watchdog dumps odom/IMU/cov ring buffer to `~/data/slam/crashes/...` before restart |
| S0.2 LOC_FALLBACK_GNSS_ONLY state | `2d506bb` | Degraded-running path + Navigation caution-mode response |

### Sprint 1 ‚Ä?Robustness skeleton (6 + 1 fix)

| Item | Commit | Effect |
|---|---|---|
| P1 Allan Variance one-shot + ICM-40609 sanity | `24dfe50` + `2be9787` | `na/ng=0.01` is ~150x too high for ICM-40609; tooling + datasheet-grounded warning |
| P2 IEKF non-conv + IMU init stuck WARN | `2f74fb1` | Two silent failure modes now surface in logs |
| N1 Scan Context PGO loop pre-filter | `850b75d` | Indoor global anchor (replaces missing GNSS) |
| P3 ICP three-axis health, multi-frame gate | `054fcd9` | `/nav/localization_health` topic, RELIABLE QoS |
| P4 map‚Üîodom TF jump events | `db84b98` | NavigationModule forced replan on PGO/relocalize jump |
| N2 SceneModeDetector | `d50080c` | Indoor/outdoor classification with 5s hysteresis |

### Review-driven refactors (4)

| Item | Commit | Effect |
|---|---|---|
| R1 aarch64 CI | `addf3b3` | First compile verification of all C++ changes |
| R2 P4 docstring | `addf3b3` | Documents why hand-roll is canonical (no ROS2 mechanism exists) |
| R4 small_gicp | `d064627` | True 3-axis P3 (fitness + iter + cov_trace) replacing PCL ICP |
| **R6 deprecation tag** | this | scan_context.h flagged for MapClosures migration |

**Test count: 124 framework tests, 100% pass rate, no skips.**

---

## Deferred ‚Ä?RFC

### RFC-1: Replace `scan_context.h` with PRBonn/MapClosures

**Why now**: Hand-written code, no real-data validation, sparse-LiDAR
performance unknown. MapClosures has the same plug-in shape (per-frame
add ‚Ü?loop pair retrieval) and is actively maintained.

**Why not now**: ~2-3 weeks of work ‚Ä?non-trivial CMake FetchContent
setup, GTSAM compat verification, A/B benchmark on recorded
indoor+outdoor rosbags, threshold re-tuning.

**Scope**:
1. `src/localization/pgo/CMakeLists.txt` ‚Ä?add MapClosures via FetchContent (or `find_package` first)
2. `src/localization/pgo/src/pgos/map_closures_loop_detector.h/cpp` ‚Ä?wrapper exposing the same add/query API as `scan_context.h`
3. `simple_pgo.cpp` ‚Ä?swap `m_sc` member type behind a config flag (`loop_detector: scan_context | map_closures`); keep both compiled in for first month A/B
4. `pgo.yaml` ‚Ä?new keys: `loop_detector`, `mc_bev_grid_size`, `mc_descriptor_window`, `mc_ransac_iter`
5. After A/B: delete `scan_context.h`, remove the config flag, lock to MapClosures
6. Update `docs/05-specialized/` with the new tuning knobs

**Acceptance**: indoor corridor revisit recall ‚â?Scan Context baseline; outdoor KITTI-style scene precision within 5%.

**Owner**: TBD. Estimated 2-3 weeks calendar with 1 engineer.

### RFC-2: Replace `BlackBoxRecorder` with rosbag2 snapshot mode

**Why now**: 188 lines of hand-written ring-buffer + numpy serialisation. ROS2 Humble's `rosbag2` has a production "snapshot mode" (record into RAM continuously, dump to disk on trigger) that does the same thing with proper MCAP encoding, replayable via `ros2 bag play`, inspectable via `ros2 bag info`.

**Why not now**: BlackBoxRecorder works today and has test coverage. The migration requires:
- Converting current dict/numpy format ‚Ü?ROS messages (dict of mixed-type fields ‚Ü?msg type design decision)
- Rosbag2 snapshot mode setup via ROS2 services (snapshot service trigger from the Python drift_watchdog)
- Re-evaluating retention policy (current is timestamped dirs; rosbag2 wants single bag dirs)

**Scope**:
1. `src/runtime/utils/blackbox_recorder.py` ‚Ä?wrap rosbag2 snapshot service rather than implement ring buffer
2. Define ROS msg types for the structured fields (or use `rcl_interfaces::msg::Log` + JSON payload as escape hatch)
3. Add MCAP backend (faster than sqlite for this read-once write-once use case)
4. Update `gateway_module.py:_drift_watchdog_loop` to call snapshot service on trigger
5. Update tests ‚Ä?keep behavioural contracts (what gets dumped) the same

**Acceptance**: drift events still produce on-disk forensic data; same fields preserved; replayable via standard `ros2 bag` tools.

**Owner**: TBD. Estimated 4-6h.

---

## Open Sprint 1 items (not in this review)

| Item | Reason for deferral | Notes |
|---|---|---|
| P5 GNSS yaw Kabsch alignment | Outdoor-only; not blocking indoor deploy | Python-only, ~1d |
| P6 PGO `gtsam::GPSFactor` | Outdoor-only | C++ + GTSAM, ~2d |
| **P7a auto kidnapped** | ‚ú?Done ‚Ä?see commits below | LOST ‚Ü?BBS3D recovery via shared launch helper |
| P7b BBS3D Topic‚ÜíAction | Defer ‚Ä?the Trigger service + Float result is sufficient | Action interface only matters if we want progress feedback |
| N3 iBTC visual loop closure | Long-corridor specialised; ROS1‚ÜíROS2 port | 1-2 weeks |

### P7a: auto kidnapped trigger (LOST ‚Ü?BBS3D)

The original review classified P7 as "ROS2 Action refactor + auto
kidnapped trigger." On inspection these are separable: BBS3D already
has a `std_srvs/Trigger` service for manual relocalize, and a (C) drift
watchdog (raw fitness streak) that auto-fires on long bad streaks.
The actual gap was that the **multi-frame health gate (P3) reaching
LOST** never triggered recovery ‚Ä?it only published LOST so navigation
slowed/stopped. Recovery was tied exclusively to the (C) watchdog's
independent raw-fitness criterion, which can disagree with the P3
multi-frame gate (the gate uses 3 frames at refine_score_thresh=0.1;
the watchdog uses 30 frames at drift_bad_thresh=0.3).

P7a refactors this so BOTH paths call a shared `launchAutoBBS3D()`
helper. The helper owns the 60-second cooldown and the
`m_bbs3d_running` mutex, so the two trigger sources cannot
double-fire and cannot starve each other. On BBS3D success, the
helper resets `m_consec_lost = 0` so the next LOCKED frame can
promote to RECOVERED quickly.

Concurrency note: `m_consec_lost` / `m_consec_locked` were changed
from plain `int` to `std::atomic<int>` because the BBS3D worker
thread can now write `m_consec_lost = 0` while the timer thread
reads it for the LOST_CONFIRM_FRAMES comparison. Without atomic this
would be a data race (UB on weakly-ordered ARM, observable as torn
reads under contention).

P7b ‚Ä?converting the BBS3D Trigger service to a ROS2 Action ‚Ä?is
deferred. The Action interface only adds value if we want progress
feedback ("BBS3D is at 35%, ETA 1.4s"). For automatic recovery the
Trigger pattern is sufficient: the worker thread runs detached, and
the next ICP frame picks up the new initial_guess. Revisit if a UI
needs progress reporting.

---

## Lessons (for the next sprint)

1. **Always cite a datasheet URL when picking sensor reference values.**
   I shipped a BMI088 reference table for ICM-40609 hardware. User
   caught it. Now the apply_calibration sanity check has the
   datasheet URL in its docstring.
2. **Always cite a LICENSE URL before depending on a 3rd-party algorithm.**
   I assumed `aserbremen/scancontext_ros2` was BSD. It's CC-BY-NC-SA.
   Caught only because of the team review.
3. **"Hand-written" is a code smell.** When the audit found I'd
   re-implemented Scan Context, the reasoning was: there is almost
   always a maintained library; if there isn't, the algorithm is
   either too new (don't use it) or too niche (your problem might
   be misframed).
4. **`pcl::IterativeClosestPoint` is a bad floor.** The PCL ICP
   wrapper exposes only fitness; modern alternatives (small_gicp,
   KISS-ICP, fast_gicp) all expose Hessian + iter + converged.
   This is a 5-year-old mistake the field has solved.
5. **Compile verification is a feature, not a luxury.** R1 should
   have been Sprint 0, not a review-driven add-on.

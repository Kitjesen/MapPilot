# LingTu Backlog — Single Source of Truth

> The only registry for features, regressions, and tech debt. Every PR / on-robot test must reference an item here as `[BACKLOG-##]`.
>
> **Status values**: `todo`, `doing`, `verified`, `blocked`.
>
> **Last on robot**: `YYYY-MM-DD` or `—` (never).
>
> Discipline:
> 1. Every new PR adds or modifies an entry here, then opens the PR.
> 2. After an on-robot run, update status and date.
> 3. `doing` requires an owner; `verified` requires a `docs/07-testing/field-runs/` link.

---

## P0 — Must Be Correct (customer-blocking)

| # | Item | Acceptance script | Unit tests | Status | Last on robot | Owner |
|---|------|-------------------|-----------|--------|---------------|-------|
| P0-01 | S100P cold boot | `docs/07-testing/p0_cold_boot.sh` | 1522 tests | `todo` | — | — |
| P0-02 | Map -> save -> activate -> localize | `docs/07-testing/p0_mapping.sh` | `test_map_occupancy.py` | `todo` | — | — |
| P0-03 | Click goal -> arrive | `docs/07-testing/p0_goto.sh` | `test_new_modules.py::Navigation` | `todo` | — | — |
| P0-04 | E-stop reflex | `docs/07-testing/p0_estop.sh` | `test_new_modules.py::SafetyRing` | `todo` | — | — |
| P0-05 | Wave 1 hard-guardrail production path | embedded in P0-01 | 17 tests | `verified` (unit) / `todo` (robot) | — | — |
| P0-06 | scipy / `_nav_runtime.so` ready on S100P | first launch under P0-01 | — | `todo` | — | — |

---

## P1 — Should Be Correct (differentiator)

| # | Item | Acceptance script | Unit tests | Status | Last on robot | Owner |
|---|------|-------------------|-----------|--------|---------------|-------|
| P1-01 | TARE autonomous exploration | `docs/07-testing/p1_tare_explore.sh` | `test_exploration_supervisor.py` | `todo` | — | — |
| P1-02 | GNSS RTK fusion (WTRTK-980) | `docs/07-testing/p1_gnss_fusion.sh` | GNSS integration suite present | `todo` | — | — |
| P1-03 | NTRIP RTCM injection | embedded in P1-02 | `ntrip_client_module` | `todo` | — | — |
| P1-04 | OSNet person follow on BPU | `docs/07-testing/p1_follow_person.sh` | 14 tests | `todo` | — | — |
| P1-05 | Safe degradation on sensor disconnect | `docs/07-testing/p1_degraded.sh` | `test_degeneracy_fusion.py` | `todo` | — | — |
| P1-06 | SLAM drift watchdog + auto-relocalize | embedded in P0-02 | `test_drift_guard.py` | `todo` | — | — |
| P1-07 | Worker subprocess isolation for heavy ML | embedded in P0-01 (Gateway must not block) | `test_worker_isolation.py` | `todo` | — | — |
| P1-08 | Wave 2 — twelve precision upgrades | per-suite tests | 39 tests | `verified` (unit) / `todo` (robot) | — | — |
| P1-09 | Wave 3 — OSNet / PD / TSDF / Bayesian / config | per-suite tests | 28 tests | `verified` (unit) / `todo` (robot) | — | — |

---

## P2 — Acceptable (long-tail)

| # | Item | Acceptance | Status | Notes |
|---|------|-----------|--------|-------|
| P2-01 | LLM multi-turn AgentLoop (Kimi) | manual chat run | `doing` | Needs `MOONSHOT_API_KEY` |
| P2-02 | Vector memory query | `query_memory` MCP returns a tag | `todo` | Wave 3 task: needs `sentence-transformers` or hard-fail |
| P2-03 | Relocalization service | REPL `relocalize` command | `todo` | — |
| P2-04 | LLM backend swap (Kimi / OpenAI / Claude / Qwen) | edit `robot_config.yaml` `llm.backend` | `todo` | Backend swap not yet validated on robot |
| P2-05 | Local-path web rendering (amber) | dashboard scene tab | `verified` | 2026-04-15 commit `14b14b2` |
| P2-06 | Agent dialogue animation (`thinking` / `agent_message`) | manual chat | `verified` | 2026-04-16 commit `17718a4` |

---

## P3 — Tech Debt (non-blocking)

| # | Item | Estimate | Status | Notes |
|---|------|---------|--------|-------|
| P3-01 | ROS 2 Phase 1 — drop rclpy from CameraBridge | 1 day | `todo` | Windows dev currently can't run camera |
| P3-02 | ROS 2 Phase 1 — drop rclpy from ROS2SimDriver | 1 day | `todo` | Same as P3-01 |
| P3-03 | ROS 2 Phase 2 — Fast-LIO2 nanobind library | 1-2 weeks | `todo` | Highest-value refactor, deferred |
| P3-04 | ROS 2 Phase 2 — Point-LIO library | 1 week | `todo` | — |
| P3-05 | OTA backend agent under `/api/v1/ota/*` | 2 days | `todo` | Frontend modal already prepared |
| P3-06 | Reconstruction TSDF live (Open3D GPU) | 2 days | `todo` | open3d S100P install state unknown |
| P3-07 | Semantic-scoring weight learning (sgnav / fast_path / frontier) | 2 weeks | `todo` | Needs labelled data |
| P3-08 | Askme voice integration | 1 week | `todo` | Independent product, integrated as voice agent |
| P3-09 | Remaining 14 simplifications across Wave 1/2/3 | — | `todo` | Algorithm research; see plan history |
| P3-10 | Dashboard diagnostic-bundle export backend | 0.5 day | `doing` | This week, day 3 |

---

## Recently Delivered (commit -> backlog item)

| Commit | Content | Backlog |
|--------|---------|---------|
| `9f7171c` | Dashboard UI unification + settings menu + scene bubbles | P2-05 / new UI |
| `13cbd35` | TARE Supervisor + Gateway SSE | P1-01 |
| `b903ee9` | Wave recovery — MapManager tomogram + tests | P0-02 / P0-05 |
| `c440b8c` | Wave 3 — OSNet / PD / TSDF / Bayesian / config | P1-04 / P1-09 |
| `d1dbc78` | Wave 2 C/D — perception + motion | P1-08 |
| `14d3bee` | Wave 2 A/B — config + bbox / vlm / agent | P1-08 |
| `a26bf24` | Wave 1 hard guardrails | P0-05 |
| `b3c93e8` | TARE C++ code import | P1-01 |
| `439fac2` | TARE integration | P1-01 |
| `0a44163` | GNSS dashboard + NTRIP | P1-02 / P1-03 |
| `66a9696` | GNSS runtime control + MCP skill | P1-02 |
| `612ac25` | GNSS antenna lever-arm compensation | P1-02 |
| `c0d47d0` | GNSS `fusion_health` diagnostics | P1-02 |
| `643ee96` / `b4919a8` | SLAM drift watchdog | P1-06 |
| `dbc7507` | SLAM ENU long-term anchor | P1-06 |
| `c09fd11` | Worker subprocess isolation | P1-07 |
| `336413a` | GIL backpressure (`latest` policy) | P1-07 |
| `0d816b5` | DeviceRegistry framework | P0-01 |
| `525d1ee` | UI palette consolidation | P2-05 |
| `17718a4` | `agent_message` SSE | P2-06 |

---

## Update Discipline

1. **Before opening a PR**: add or update an entry here; commit message references `[BACKLOG-##]`.
2. **Unit tests pass**: state moves `doing -> verified (unit)`.
3. **On-robot run passes**: state moves to `verified (robot)`, last-on-robot date updated, `docs/07-testing/field-runs/YYYY-MM-DD.md` link added.
4. **Blocker hit**: state `blocked` plus reason ("waiting for NTRIP credentials", "S100P off network", ...).

**Review cadence**: refresh status every Friday after the L3 pass.

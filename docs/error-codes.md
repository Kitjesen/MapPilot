# LingTu numeric error codes

**Status:** Current navigation health code registry
**Audience:** Operators, Gateway clients, and runtime maintainers
**Runs on:** Real and simulation Products

This is the lookup table for numeric navigation health codes returned by
`GET /api/v1/health`, `GET /api/v1/metrics`, and the Gateway Faults RPC. The
Faults RPC stores the numeric code in its string field for protobuf compatibility.
The human-readable machine reason remains in `reason`.

Motion-affecting entries appear in `issues`. Diagnostic-only entries such as
`21001` appear in `advisories`.

## Impact meanings

| Impact | Meaning |
| --- | --- |
| `diagnostic_only` | Recorded and shown to the operator. It does not change motion permission. |
| `motion_degrading` | Timing or processing quality is degraded. Native motion ownership stays unchanged, while acceptance or recovery checks may reject the window. |
| `motion_hold` | Final command output is held at zero until the named condition recovers. |
| `motion_blocking` | New motion is blocked by a hard control or emergency condition. |

## Navigation codes

| Numeric code | Name | Source | Impact | Trigger | Operator action |
| ---: | --- | --- | --- | --- | --- |
| `21001` | `motion_update_over_period` | `navigation.motion_worker` | `diagnostic_only` | The last completed asynchronous obstacle-worker update exceeds the current control period. | Inspect control-loop overrun and input freshness. |
| `21002` | `motion_processing_failed` | `navigation.motion_worker` | `motion_degrading` | An obstacle-worker frame failed and did not refresh the current snapshot. | Read the worker detail and wait for a fresh collision snapshot. |
| `21101` | `consecutive_deadline_misses` | `navigation.control_loop` | `motion_degrading` | Consecutive native control-loop deadlines were missed. | Inspect retained timing history before supervised motion. |
| `21102` | `deadline_miss_ratio_high` | `navigation.control_loop` | `motion_degrading` | The rolling deadline-miss ratio exceeds its configured limit. | Review P99 overrun and workload before motion acceptance. |
| `21103` | `p95_utilization_high` | `navigation.control_loop` | `motion_degrading` | Rolling P95 control-loop work utilization exceeds its configured limit. | Reduce control-thread work or inspect CPU contention. |
| `21104` | `control_loop_unhealthy` | `navigation.control_loop` | `motion_degrading` | The native loop is unhealthy without a known reason. | Read native timing history and the original reason. |
| `21201` | `input_gate_not_ready` | `navigation.input_gate` | `motion_hold` | Required odometry, TF, cloud, collision, localization, or terrain input is missing, stale, future-dated, or invalid. | Restore the named fresh input. |
| `21301` | `resume_required` | `navigation.control_authority` | `motion_hold` | Operator takeover or resume confirmation remains active. | Complete the existing resume transaction. |
| `21401` | `estop_latched` | `navigation.control_authority` | `motion_blocking` | The software emergency stop is latched. | Complete the estop release transaction. |

## Contract

The numeric code is stable and suitable for logs, dashboards, tests, and
support queries. `reason` can contain a native detail such as `cloud_stale` or
`motion_processing_failed: bad cloud`. A detail may evolve while the numeric
code remains stable.

`21001` is deliberately returned in `advisories`. The duration belongs to the
last completed worker result. It is not the current control-thread work duration
and does not directly set `overrun_ms`.

`21002` is derived from the source-specific
`navigation.frame_gate.motion_worker_error` field. The native input projector
clears that field after the next successful obstacle snapshot, so a recovered
worker does not leave a permanent active issue in the Gateway.

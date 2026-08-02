# Inspection Product Definition

Status: current product definition

## Register

product

## Users

The primary user is a trained field inspection operator supervising one quadruped robot in substations, distribution rooms, cable tunnels, utility corridors, and similar power or energy facilities. They may work in strong sunlight, low light, dust, rain, cold, gloves, intermittent connectivity, and GPS-denied spaces, sometimes while the robot is out of sight.

Secondary users are shift supervisors who review completion and anomalies, equipment specialists who judge evidence, and integrators or administrators who configure devices, maps, and diagnostics.

The Web application is for mission supervision, preflight, evidence review, and safe takeover or recovery. Continuous manual driving should prefer a physical controller; engineering diagnostics remain available as an advanced surface rather than the default operator workflow.

## Product Purpose

LingTu is an inspection-first quadruped system for complex power and energy spaces. It repeats fixed routes and inspection points, binds trustworthy evidence to the robot pose and task context, and supports human review and reinspection today. Report and work-order handoff remain product targets until their runtime integrations are available.

The operational loop is:

1. Plan a route and its inspection points.
2. Reach each point at a repeatable pose.
3. Capture evidence and expose its quality and freshness.
4. Detect or flag anomalies with an explainable evidence trail.
5. Let a person review uncertain or consequential results.
6. Archive the result and trigger repair or reinspection when required.

The current verified baseline is route, point, action, timestamp, RGB capture, pose, and detections. Thermal, gas, acoustic or partial-discharge sensing, trend analysis, automatic report generation, and work-order integration are product targets and must not be presented as delivered until their runtime contracts and field evidence exist.

The visible action catalog is backend-driven. Pilot-specific actions such as parking, bin-state, or identifier capture must be labeled honestly, shown only when advertised as supported, and never generalized into a power-equipment diagnosis capability.

Success is measured by safe mission completion, inspection-point coverage, repeatable capture, evidence integrity and freshness, understandable failure and control ownership, review quality, system availability, and time to an actionable maintenance decision.

## Brand Personality

Calm, decisive, explainable.

## Anti-references

- A ROS or DDS topic browser presented as the product.
- A science-fiction telemetry wall that makes state harder to understand.
- A chat interface that bypasses explicit motion intent, safety gates, or control ownership.
- An alarm wall without prioritization, evidence, or a recovery path.
- A generic card dashboard with no task hierarchy.
- Claims of complete autonomy or sensor capability without runtime and field evidence.

## Design Principles

1. **Complete the inspection loop.** Organize the product around task, route, point, evidence, anomaly, review, and reinspection—not around internal processes.
2. **Answer four operator questions immediately.** What is the robot doing? Is it safe? Who has control? What must I do next?
3. **Make motion an authorized intent.** Distinguish preflight, start, pause, cancel, takeover, release, resume, and emergency stop; show acceptance separately from physical outcome.
4. **Keep evidence reproducible and honest.** Bind evidence to route, point, pose, action, sensor, and time; expose missing, stale, unsupported, or invalid evidence instead of implying certainty.
5. **Separate field work from engineering detail.** Default to task language and actionable recovery. Keep transports, topics, processes, and tuning in advanced diagnostics.
6. **Degrade visibly and safely.** Weak connectivity, sensor loss, localization degradation, unsupported actions, and stale data must have explicit state, impact, and recovery guidance.

## Accessibility & Inclusion

- Meet WCAG 2.2 AA for contrast, focus visibility, keyboard operation, and semantic structure.
- Use at least 44 by 44 CSS pixels for touch targets where field interaction is expected; prefer 48 by 48 for critical controls.
- Never communicate safety, severity, connectivity, or ownership by color alone; pair color with text and shape.
- Support reduced motion and avoid decorative animation in operational surfaces.
- Use Chinese-first operational language with complete English equivalents; keep units, timestamps, and status terms unambiguous.
- Design for strong sunlight, low light, gloves, intermittent or offline operation, and stale telemetry. Always show freshness and whether an action is pending, accepted, rejected, or confirmed.
- Keep emergency stop, cancel, pause, takeover, release, and resume visually and semantically distinct.

## Inspection Result PRD

### Problem Statement

An operator can currently start and supervise an inspection task and can browse
recent evidence, but the product does not answer the final business question:
did this specific task deliver every result that the route required? Navigation
terminal state and evidence completeness are shown on separate technical
surfaces, so a successful route can hide missing evidence and a recording
failure can be mistaken for a navigation failure.

### Solution

Expose one task-addressed inspection report that combines only authoritative
native task facts, the immutable route revision, and integrity-verified
inspection evidence. The report keeps execution outcome, inspection result,
and optional diagnostic recording separate. It identifies each missing or
invalid required capture and gives the operator a clear acceptance decision
without inventing point completion from command acknowledgements.

### User Stories

1. As a field operator, I want to open a report by task ID, so that I know I am reviewing the task I actually ran.
2. As a field operator, I want to see whether execution is still active or terminal, so that I do not treat an in-progress task as complete.
3. As a field operator, I want navigation success separated from evidence completeness, so that missing captures are never hidden.
4. As a field operator, I want every required inspection point listed, so that I can see route coverage rather than only the latest point.
5. As a field operator, I want missing evidence named by point and action, so that I know exactly what must be reinspected.
6. As a field operator, I want corrupt evidence labeled invalid, so that a damaged artifact is never presented as a valid capture.
7. As a field operator, I want unavailable evidence storage labeled unavailable, so that infrastructure failure is distinguishable from a missing capture.
8. As a field operator, I want an active task reported as pending, so that partial evidence collected so far is not treated as a final result.
9. As a shift supervisor, I want one acceptance outcome, so that I can quickly distinguish acceptable, review-required, and not-acceptable work.
10. As a shift supervisor, I want execution reason and evidence issues together, so that I can decide between recovery, reinspection, and maintenance review.
11. As an equipment specialist, I want evidence identity, pose, action, and integrity metadata preserved, so that a conclusion can be traced to its source.
12. As an integrator, I want report schema and outcome vocabulary versioned, so that clients do not infer meaning from display text.
13. As an integrator, I want task-not-found and journal-unavailable responses to remain distinct, so that clients retry safely.
14. As an operator on an intermittent network, I want a terminal report to remain queryable after Gateway restart, so that a browser refresh does not lose the result.
15. As a safety reviewer, I want report generation to remain read-only, so that opening a report cannot command motion or change task state.
16. As a product owner, I want unsupported sensors and analyses omitted or labeled unsupported, so that pilot capability is not marketed as delivered product capability.

### Implementation Decisions

- The public seam is a read-only task report resource addressed by task ID.
- Native task events remain the only source of execution state and terminal truth.
- The minimal route-requirements snapshot persisted at admission defines the
  required enabled points and actions; the mutable route store is never used to
  reconstruct a historical task.
- Only evidence that passes the existing manifest and artifact integrity checks counts as verified.
- Execution outcome, inspection result, acceptance decision, and diagnostic recording are separate concepts. P0 exposes the first three in the task report; recording remains a separate optional diagnostic surface until task binding exists.
- The minimal route requirements used at start are persisted with the task journal, so later route edits cannot rewrite a historical task result.
- Every request ID is durably bound before native contact to one task, action,
  reason, and (for start) route-requirements hash. A retry cannot redefine any
  of them.
- A native event whose map or route identity conflicts with the frozen task is
  rejected as terminal truth and makes the report review-required.
- A terminal execution success with missing required evidence produces a partial result and review-required acceptance, not a navigation failure.
- A native failed or cancelled terminal produces a not-acceptable report even if some evidence was captured.
- An active or unreconciled task produces a pending or unknown report and never an acceptable result.
- Report generation is a Gateway read model; it does not become task authority and does not control RecordingManager.
- Full DDS or camera recording remains optional diagnostic evidence. Automatic recording lifecycle binding is outside P0.
- The first product slice covers the currently verified RGB, pose, detections, and declared action evidence contract.

### Testing Decisions

- Test through the public task report resource rather than private aggregation helpers.
- The first tracer test proves that a native-success task with a verified required capture is acceptable.
- The second tracer test proves that native success with missing required evidence is partial and review-required.
- Further tests cover active, failed, cancelled, corrupt evidence, unavailable storage, unknown task, and Gateway restart.
- Tests use the existing in-process inspection adapter and filesystem-backed evidence store; DDS and storage are mocked only at their existing process seams.
- MuJoCo acceptance must later prove that the same task ID, point progress, evidence identity, terminal fact, and report outcome remain connected end to end.

### Out of Scope

- Automatic MCAP start and stop for every task.
- ProductControl ownership of per-task recording or task state.
- Thermal, gas, acoustic, partial-discharge, or other sensors without current runtime contracts.
- Automatic maintenance diagnosis, work-order creation, trend analysis, and regulatory report formatting.
- Inferring historical point outcomes when the native fact stream is incomplete.

### Further Notes

The report is a product read model, not an execution state machine. When source
facts are incomplete, it must return unknown or review-required rather than
guessing. The repository has no configured issue-tracker publishing workflow,
so this PRD remains the local source of truth until one is added.

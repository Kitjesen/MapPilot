# Confirmed-person continuity check — 2026-11-17

## Conclusion

The current local default `tracking / real / unitree/go2` configuration cannot
be accepted as consistently following only the operator-selected person.
Two controlled post-confirmation sequences changed physical target and each
published two new navigation-goal requests for the wrong person. No driver,
native planner, robot or external model endpoint was connected.

## Reproduction

From the repository root, PowerShell:

```powershell
$env:PYTHONPATH='src'
.venv/Scripts/python.exe build/person-follow/stability/replay.py
```

Evidence: `build/person-follow/stability/report.json` (per-frame trace) and
`run.log`. This invokes production `confirm_follow_target`, feeds typed
`SemanticObservation` messages, executes `PersonTracker`, and captures
`VisualGoalRequest` output. Initial person-review verdicts are controlled inputs.
DINO comparison cases use explicit synthetic 384-dimensional appearance vectors;
they validate decision logic, not a real encoder's accuracy.

The source clock advances at 10 Hz without real-time pacing. Each 300-frame case
represents 30 seconds of source timestamps, not a measured real-time run.

## Results

| Scenario | Frames | Accepted selected person | Wrong person accepted | Held | Wrong-person goal requests |
|---|---:|---:|---:|---:|---:|
| Default, continuous observations | 300 | 300 | 0 | 0 | 0 |
| Default, crossing positions with correct stable IDs | 300 | 300 | 0 | 0 | 0 |
| Default, target absent | 20 | 0 | 0 | 20 | 0 |
| Default, target replaced by nearby different-ID bystander | 12 | 0 | 10 | 2 | 2 |
| Default, original ID assigned to a different person | 12 | 0 | 12 | 0 | 2 |
| Controlled DINO features, original ID assigned to another person | 12 | 0 | 0 | 12 | 0 |
| Controlled DINO features, nearby different-ID bystander | 12 | 0 | 0 | 12 | 0 |
| Controlled DINO features, target returns, identity UNKNOWN | 32 | 0 | 0 | 32 | 0 |
| Controlled DINO features, target returns, identity MATCH | 32 | 9 | 0 | 23 | 0 |

The different-ID bystander is accepted on frame 3. The same-ID replacement is
accepted immediately on frame 1. In both cases the person moves at 0.8 m/s from
near the original location; this is sufficient to produce changed goal requests.
The MATCH return case contains 20 missing frames, three candidate-evidence frames,
and nine accepted return frames. Its zero goal count is not proof of navigation
resumption: cancellation acknowledgements from native navigation are not simulated.

## Cause

Fresh local RunPlan/Host configuration resolution returned empty values for
`person_text_model_path`, `person_appearance_model` and `vision_service_url`.
The default VisualServo instance attached no CLIP, OSNet or DINO encoder.
Only native executable file availability was bypassed when inspecting config;
no robot process or Product was started.

- `src/decision/vision/person.py`, `_match_person`: without active DINO identity
  matching, the original tracker ID is accepted directly.
- `_motion_reacquire`: a nearby candidate with a different ID can be accepted
  after three consistent observations, without appearance proof.
- `src/decision/modules/visual_servo.py`, `_tick_follow`: the accepted tracker ID
  becomes the current target and can generate navigation requests. The task's
  initial `confirmed_target_id` does not prevent this transition.

An unchanged internal identity UUID or detector ID is not independent proof of
physical identity. The initial person/non-person review also does not provide
continuous identity verification.

## Existing tests

Ran `tests/decision/test_person_tracker.py`, `test_dino_person.py`,
`tests/runtime/test_follow_confirmation.py`, and `test_follow_person_selection.py`.
First pass: 155 passed, one watchdog timing test failed at its 1-second event wait.
That test passed when rerun individually (8.75 seconds total). Logs:
`build/person-follow/stability-tests.log`. This timing result is retained rather
than reported as an entirely clean first run.

## Next work

1. Load and verify the actual appearance encoder on the target deployment.
2. For an operator-confirmed target, require identity evidence for changed IDs;
   do not allow the proximity-only fallback to authorize a replacement person.
3. Validate same-ID contamination with real encoder features, since ID continuity
   alone cannot detect it.
4. Run recorded and then supervised live multi-person crossing, occlusion,
   similar-clothing and return tests, reporting wrong-person acceptance separately
   from tracking coverage and reacquisition delay.

This check does not change production behavior or deploy to NX. It does not
measure current NX model loading, real detector/encoder accuracy, real-time
performance, native planning, or physical following stability.

Follow-up: a fresh real-video experiment is recorded in
[`follow-video-20261117.md`](follow-video-20261117.md). It separates fresh model
inference, cached-detection diagnostics, and Host timeout behavior.

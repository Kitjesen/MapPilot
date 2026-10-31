# Inspection observations

This package consumes native inspection evidence requests. It captures the
current camera frame and pose, persists them through the shared evidence store,
and acknowledges persistence. Native C++ still owns arrival, point advancement,
retry policy and task state; this worker neither plans nor commands motion.

Camera and odometry rates may differ. The newest odometry sample establishes
that the pose stream is online; the photo is paired with the nearest real pose
at its own source timestamp. Short bounded history preserves that pairing until
the photo expires. Readiness and persistence use the same match. A later current
pose cannot silently replace the pose associated with the captured image.

## Equipment findings

The operator selects the expected state as the point action:

| Action | Expected observation |
| --- | --- |
| `capture:indicator_on` | An identifiable lit indicator, of any supported color |
| `capture:indicator_off` | An identifiable unlit indicator |
| `capture:indicator_green` | A lit green indicator |
| `capture:indicator_red` | A lit red indicator |
| `capture:indicator_yellow` | A lit yellow indicator |
| `capture:door_open` | A visibly open cabinet door |
| `capture:door_closed` | A visibly closed cabinet door |

`visual_checks.py` reuses the registered remote vision client and the existing
`vision_service_url` configuration. The shared service owns model startup and
offers `/health` and `/chat`; inspection does not download or launch a model.
The default text-only Qwen client is not an image reader.

The model reports visible state, visibility and target count. The shared
`InspectionFinding` compares the observation with the configured expectation;
the model does not decide its own pass/fail rule. A lit color satisfies `on`.
An unspecified lit color cannot prove a specific expected color. Multiple
possible targets, an invisible target, blur, a missing service or an unreadable
response yield `unknown` and an inconclusive finding. Use an observation pose
where the intended device is individually identifiable, and qualify the model
on labelled photographs of the actual devices before relying on its accuracy.

For the first route, save an observation pose and heading facing the intended
device. The point's yaw and post-arrival settling use the existing native route
executor. This version does not automatically seek a better camera viewpoint
when the target is occluded; the report retains that unreadable observation for
review rather than treating it as a normal device.

Reader failure preserves the photograph and returns a successful persistence
acknowledgement. Replaying the same request reuses its committed observation;
it does not overwrite an earlier image or ask the model again.

## Reports and review

Task completion, evidence integrity and equipment condition are independent.
Plain capture can complete a point with complete evidence while the equipment
still needs review. Missing, skipped or unvisited points cannot become normal.

The Gateway review API appends human findings to `reviews.json` beside the
committed evidence. It preserves the original image and model analysis. The
latest review is shown with its note and UTC timestamp and takes precedence in
the equipment summary. Review does not resume a task or publish a motion command.

Contract tests cover observation interpretation, persistence, retries, model
unavailability, HTTP review and reports. Local JPEG/HTTP fixtures establish
client wiring, not recognition accuracy or physical inspection acceptance.

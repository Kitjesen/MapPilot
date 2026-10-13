# Natural-language person-follow experiment

This record follows the staged person-follow experiments beginning on
2026-10-09, including recorded Go2 RGB-D replay and subsequent Host integration.
As of 2026-10-13, no new online following release or physical following motion
has been qualified. Recorded-camera replay, HTTP contract tests and CPU model
export are separate evidence levels.

Local consumer regressions now cover the remote vision interfaces and expose
visual-follow waiting/rejection reasons. Real-model acceptance remains open.
The [Follow task lifecycle](tasks.md#已落地的-follow-边界) now has a stable execution
ID, session-bound query/cancel, scoped navigation children and typed native
terminal feedback. Local ownership tests cover old-task events and cancellations;
this does not establish new camera, model, native-simulation or field evidence.
The next hardware work is corrected NX model inference and
camera-to-body calibration. The fixed-person YOLO export agrees with CPU
references, but the current NX DINO engines fail real-crop feature comparison.
The torso-anchor change reduces recorded position variation; foreground-chair
depth contamination remains unresolved. See the dated sections below and the
[current delivery order](../roadmap.md#spot-reference-and-next-local-work-2026-10-13).

## What ran

The input was the first 180 consecutive frames of the official
`MOT17-09-FRCNN` training sequence: 30 FPS and 6 seconds of real pedestrian
video. The experiment used this pipeline:

```text
Chinese instruction
  -> local Qwen3-VL-2B-Instruct task decomposition
  -> YOLO-World person boxes
  -> Qwen3-VL selection from numbered person crops
  -> project NativeByteTracker updates in source-video pixel coordinates
  -> annotated replay and per-frame evidence
  -> MOT17 annotations used only for evaluation after the replay
```

`NativeByteTracker` is LingTu's dependency-light tracker inspired by ByteTrack.
It is not the official ByteTrack implementation or a full reproduction of that
algorithm. The model, detector, and tracker ran on the server; the evidence was
then copied to the local workspace.

The positive instruction was:

```text
请持续跟踪穿灰色上衣、携带粉色包的那个人。
```

The real model returned one `follow` subgoal with the target description
`a person wearing a gray shirt and carrying a pink bag`. On frame 0 it selected
numbered crop 6. Post-run attribution matched that box to MOT ground-truth
identity 22 with IoU 0.867. This establishes the initial selection for this
clip; it does not establish general clothing-recognition accuracy.

## Retained results

| Run | Correct reference | Wrong identity | Present but unattributed | Missing | Interpretation |
| --- | ---: | ---: | ---: | ---: | --- |
| Fixed initial track ID | 13 | 0 | 2 | 165 | The selected NativeByteTracker ID fragmented after the opening frames. |
| First VLM reacquisition attempt | 94 | 41 | 0 | 45 | Reacquisition recovered more target frames but also switched to another person. This result is a failure, not an accuracy improvement. |
| Attribute and pair-verified reacquisition | 88 | 0 | 0 | 92 | Pair verification removed wrong-identity output in this clip, but emitted the correct target on only 48.9% of frames. This is still incomplete tracking. |

All counts are over 180 frames. The evaluator assigns a selected box to the
highest-IoU MOT pedestrian at IoU >= 0.5. The first attributable selected box
defines the reference identity. Ground truth is never passed to detection,
selection, tracking, or reacquisition. This is a narrow continuity diagnostic,
not a standard MOT benchmark, clothing ground truth, or proof that the initial
language target was semantically correct.

The first reacquisition policy called the VLM after the selected tracker ID had
been absent for 0.5 seconds. It made three successful ID changes according to
its own verdict, but 41 selected frames belonged to a different annotated
person. Attribute-only crop selection is therefore not a reliable re-ID rule.
The verified policy uses two real VLM judgments rather than a hand-tuned score:
it first checks whether a numbered candidate matches the requested attributes,
then compares that crop with the original target crop and returns `MATCH`,
`OTHER`, or `UNKNOWN`. It made six reacquisition attempts. Frame 28 matched the
new tracker ID 12 to the original target and post-run evaluation still assigned
it to ground-truth identity 22. Candidates at frames 118, 133, and 148 were
rejected as `OTHER`; candidates at frames 163 and 178 were left `UNKNOWN`.
Only the frame-28 decision bound a new ID. This removed the earlier 41
wrong-identity frames, but the remaining 92 missing frames show that a cautious
verifier alone does not provide continuous tracking.

The verified run retained 15 real model calls. Their recorded inference wait
totalled 3.0308 seconds: instruction parsing took 1.191 seconds, initial target
selection and verification took 0.266 and 0.081 seconds, reacquisition selection
took 0.101-0.183 seconds per call, and pair verification took 0.078-0.131 seconds
per call. These measurements exclude model loading, YOLO-World detection, video
decoding/encoding, and file I/O. A 30 FPS source video therefore does not show
that this pipeline runs at 30 FPS or in real time.

Two instruction-control probes also exposed separate boundaries:

- `停止跟踪，取消跟随任务。` produced a real `stop` plan and processed zero
  replay frames. This proves instruction parsing and replay termination only;
  it is not evidence of native robot cancellation.
- The original absent-target probe still selected crop 6 and exposed an
  unreliable no-match judgment. After adding attribute verification, the same
  bright-purple-down-jacket/yellow-hat probe returned `no_match`: no selected ID
  in all 30 replay frames. This is one negative clip, not a calibrated
  open-set-rejection benchmark.

## Paper-guided corrections (2026-10-11)

This iteration addresses association and target identity, rather than adding
another detector or changing native motion. It uses two specific references:

- [ByteTrack, ECCV 2022](https://arxiv.org/abs/2110.06864): preserve high/low
  confidence association, and allow unmatched tracks and detections during
  assignment. The existing two-stage policy was already present in LingTu;
  this iteration corrects the assignment objective.
- [TrackVLA++, 2025 preprint](https://arxiv.org/html/2510.07134v1): section IV-A
  preserves reliable target identity memory when the target is absent or the
  observation is uncertain. LingTu applies that principle with a retained
  reference crop and explicit VLM identity verification. It does not implement
  the paper's trained TIM features, Polar-CoT, or VLA action head.

### Cost-limited association

Previously, `NativeByteTracker._associate` solved a complete assignment and
discarded over-threshold pairs afterward. Those rejected pairs could occupy
columns needed by a valid match. A concrete two-track/two-detection regression
has this IoU-derived cost matrix:

```text
1 - IoU = [[0.823227, 0.936177],
           [0.451095, 0.544180]]
low-stage limit = 0.5
```

The old solver chose the diagonal and then rejected both pairs. The corrected
solver retains track 1 -> detection 0 at cost 0.451095, leaving the other
track and detection unmatched. Low-confidence detections still never start
tracks, and class isolation and the existing motion predictor remain intact.

The correction follows the partial-assignment objective used by ByteTrack's
[official matching function](https://github.com/FoundationVision/ByteTrack/blob/d1bf0191adff59bc8fcfeaa0b33d3d1642552a99/yolox/tracker/matching.py#L39-L50).
The underlying [LAP implementation](https://github.com/gatagat/lap/blob/e29341d9e0ec8c3a1f11f239eb65a31973f4a0ab/_lapjv_cpp/_lapjv.pyx#L84-L90)
extends the cost matrix with unmatched entries at `limit / 2` for each side.
LingTu uses the same matrix construction with its existing SciPy solver.
This minimizes `sum(matched costs) + limit / 2 * unmatched endpoints`;
it does not force the maximum number of weak matches. Solver tie choices may
differ from LAP, and this correction is not a full ByteTrack reproduction.

The geometry case lives in `tests/perception/test_native_byte_tracker.py` and
`build/person-follow/tracker-association/gate-counterexample.json`. The former
fails on the old implementation and passes after the correction. A second
case retains an exact overlap rather than forcing two weaker matches.

The 180-frame cached-observation comparison is retained in
`build/person-follow/tracker-association/replay-final.json`, together with
before/after tracks and `replay_tracker_association.py`. It reuses saved active
track boxes as inputs; discarded or unmatched original detector candidates are
unavailable. Both versions return 13 correct, 0 wrong, 2 unattributed, and 165
missing target frames under the existing `person_eval.evaluate` rules. This
correction fixes the geometry regression but has not improved target coverage
on these cached observations. There was no new detector or VLM inference,
full raw-detection replay, or standard MOT benchmark. Ground truth was used only
after tracking, and selected-ID changes are not whole-scene identity switches.

New `person_follow.py` runs now record the complete `TrackerInput` as each
frame's `detections`, alongside the existing `tracks` and `target`. Each row
contains `bbox_xyxy`, `score`, `class_id`, and `detection_index`, including
low-score and unmatched candidates. `summary.json` declares
`tracker_input_detections_recorded: true`. This enables a future comparison on
the exact tracker inputs; it cannot reconstruct candidates missing from older
artifacts.

### Reference identity in the Host

The VLM selection path now shares `decision.vision.person.verify_candidate`
with the dataset replay. The initial candidate must visibly match the
description before it is locked. The Host saves that confirmed crop for the
current follow intent, retains it across a lost-target tracker reset, and uses
it in both numbered reselection and candidate verification. Only the exact
verdict `MATCH` can bind a candidate; `OTHER`, `UNKNOWN`, malformed responses,
and failed calls leave the target unbound. An accepted new ID does not replace
the original reference. A new task or STOP clears it, and a late result cannot
bind a previous intent. Once a reference exists, an unavailable vision client
cannot fall back to single-person or CLIP attribute selection and overwrite
that identity reference.

The VLM runs at selection/reselection boundaries. Frame-to-frame tracking and
native navigation still own ongoing target updates and motion. This identity
check does not detect every possible tracker ID switch, provide trained ReID
features, or prove that the configured AIXForge route can understand images.
The existing short-gap local motion/appearance reacquisition path does not
perform this VLM check, and this iteration does not change reselection timing.
The old 88/180-frame Qwen result remains the most recent real verified replay;
this Host integration has local contract evidence rather than a new model run.

## Reproduce the RGB replay

The positive run used the following command from
`/root/autodl-tmp/lingtu-vlnav` on the configured server. The paths are kept
absolute so that the retained command matches the recorded run.

```bash
export PYTHONPATH=/root/autodl-tmp/lingtu-vlnav/source:/root/autodl-tmp/lingtu-vlnav/source/src:/root/autodl-tmp/models/clip-vit-base-patch32
export YOLO_AUTOINSTALL=false
/root/autodl-tmp/lingtu-vlnav/vision/bin/python \
  /root/autodl-tmp/lingtu-vlnav/source/tools/datasets/person_follow.py \
  --video /root/autodl-tmp/lingtu-vlnav/data/mot17-09/input.mp4 \
  --yolo-weights /root/autodl-tmp/lingtu-vlnav/yolov8s-worldv2.pt \
  --model-path /root/autodl-tmp/models/qwen3-vl-2b \
  --instruction '请持续跟踪穿灰色上衣、携带粉色包的那个人。' \
  --output /root/autodl-tmp/lingtu-vlnav/person-gray-pink-verified \
  --max-frames 180 \
  --device cuda \
  --source-url https://motchallenge.net/data/MOT17.zip
```

Evaluate a copied run from the repository root with:

```powershell
.\.venv\Scripts\python.exe tools\datasets\person_eval.py `
  --tracks build\person-follow\verified\tracks.json `
  --gt build\person-follow\gt.txt `
  --output build\person-follow\verified\evaluation.json
```

The CLI shapes above were checked against the current scripts. Reproduction
still requires the already configured model environment, model weights, and
dataset files; this documentation run did not download them again.

## Kimi API option (2026-10-10)

The same replay now accepts `--backend kimi` instead of `--model-path`. Without
endpoint or key overrides, it selects `kimi-k3` for this visual replay while using the existing registered
Moonshot client, `https://api.moonshot.cn/v1`, and the `MOONSHOT_API_KEY`
environment variable. K3 keeps thinking enabled and requests low reasoning
effort; its thinking mode cannot be disabled. An explicit
`--model kimi-k2.6` remains available for a controlled comparison. The shared
Moonshot client default remains K2.6 for other call sites, so this replay
choice does not silently upgrade existing multi-turn Agent sessions.
The API receives both the instruction-decomposition prompt and the numbered
person crops; attribute and reference-crop verification use that same client.
The local Qwen mode remains available for comparing the same recorded input.
No new dependency was added; the existing `llm` extra supplies the OpenAI SDK.

With the key configured in the replay process environment, run from the server
checkout using the existing data and detector weights:

```bash
export PYTHONPATH=/root/autodl-tmp/lingtu-vlnav/source:/root/autodl-tmp/lingtu-vlnav/source/src
export YOLO_AUTOINSTALL=false
/root/autodl-tmp/lingtu-vlnav/vision/bin/python \
  /root/autodl-tmp/lingtu-vlnav/source/tools/datasets/person_follow.py \
  --video /root/autodl-tmp/lingtu-vlnav/data/mot17-09/input.mp4 \
  --yolo-weights /root/autodl-tmp/lingtu-vlnav/yolov8s-worldv2.pt \
  --backend kimi \
  --model kimi-k3 \
  --base-url https://api.aixforge.com/v1 \
  --api-key-env AIXFORGE_API_KEY \
  --instruction '请持续跟踪穿灰色上衣、携带粉色包的那个人。' \
  --output /root/autodl-tmp/lingtu-vlnav/person-gray-pink-kimi \
  --max-frames 180 --device cuda --timeout-sec 30 \
  --source-url https://motchallenge.net/data/MOT17.zip
```

`--model`, `--base-url`, and `--api-key-env` override the registered defaults.
API and local model modes are mutually exclusive. Missing credentials fail
explicitly. API resources close on completion, STOP, and failure. Each retained
call records its messages, final response, and elapsed time; `summary.json`
records the backend, model, endpoint, and `real_api_requested`. That last field
identifies the requested execution mode, not independent proof of provider
availability or recognition quality. Keys are not written into these artifacts.

The `tracking` Product selects `kimi-k3` through the OpenAI-compatible AIXForge
endpoint `https://api.aixforge.com/v1` and reads its credential from
`AIXFORGE_API_KEY`. Assembly retains the model, endpoint, key-variable name, and
request timeout in the RunPlan and creates `LLMModule` alongside
`VisualServoModule`. It continues to exclude `SemanticPlannerModule` and
`AgentPlannerModule`. The Product's command entry is still an explicit
visual-follow command, for example:

```json
{"mode": "follow", "target": "穿灰色上衣、携带粉色包的那个人"}
```

This body goes to `POST /api/v1/visual_servo`; `{"mode":"stop"}` cancels the
visual task through its existing native navigation owner. Loading the model
does not add arbitrary task decomposition to the tracking Product's instruction
endpoint. Full instruction decomposition is exercised in the replay above.

The credential is stored in the Windows User environment. An already-open
PowerShell does not automatically inherit a later User-environment change, so
import it into the current process before starting the Product or replay:

```powershell
$env:AIXFORGE_API_KEY = [Environment]::GetEnvironmentVariable(
    "AIXFORGE_API_KEY",
    "User"
)
```

For a systemd-managed field Host, the key must be in the service environment.
The declared `lt-host.service` already reads `/etc/lingtu/gateway.env`; setting
the key only in the operator's shell does not give it to that service. Add an
`AIXFORGE_API_KEY=...` entry to that environment file on the Host and keep the
actual value out of Product YAML, RunPlan artifacts, logs, and experiment
records. The simulation supervisor intentionally excludes ambient credentials
from child environments, so the
standalone replay command is the API test entry until that deployment is
explicitly configured. Product lifecycle operations remain owned by
`python -m lingtu.control`.

AIXForge is a forwarding provider rather than the official Moonshot endpoint.
The configured request model is `kimi-k3`; a live response that declares the
same `model` value is evidence only of the provider's returned identity. It is
not independent proof of the upstream model implementation or an official
Moonshot guarantee. Any live result must be recorded as an AIXForge-routed
request with its returned fields and measured behavior.

Visual person selection now submits to the shared LLMModule event loop, avoiding
reuse of one HTTP client across temporary loops. With a vision API available,
even a sole confirmed candidate is checked against the description. A rejected
or failed selection leaves no target bound; a reply after STOP cannot bind a
target. Selection itself does not emit motion: current map-frame detections
must still drive `VisualServo -> nav.goals -> nav.commands -> native DDS ->
navigation -> driver`. The API is used for semantic selection, while the
existing local tracking and bounded goal updates handle subsequent observations.
The Host now uses the same reference-crop selection and identity-verification
helpers as the pair-verified reacquisition experiment. The replay's recorded
accuracy and latency still belong to that earlier Qwen run; they are not Host
or AIXForge deployment measurements.

The original Kimi integration evidence used the installed OpenAI SDK with
in-memory responses and therefore did not measure provider latency. The live
AIXForge probes below now provide individual request timings, but they do not
establish Kimi-to-field motion. The Qwen and MuJoCo results below remain
separate evidence for their respective runs.

The API format was checked against the official [Kimi parameter reference](https://platform.kimi.com/docs/api/models-overview)
and [vision input guide](https://platform.kimi.com/docs/guide/use-kimi-vision-model).
For K3, the adapter keeps thinking enabled, requests low reasoning effort, and
sends JPEG crops as base64 image parts in the messages array. It uses the
existing 4096-token generation budget and fails explicitly if the response is
truncated at that budget; whether 4096 is sufficient remains unmeasured until
a successful full replay. K2.6 comparison requests continue to disable thinking and omit
temperature. These are request contracts; K3 recognition accuracy remains
unmeasured, and the individual timings below are not latency percentiles.

### AIXForge live API evidence (2026-10-10)

The retained evidence is [results.json](../../artifacts/aixforge-api-20261010/results.json),
[wire.json](../../artifacts/aixforge-api-20261010/wire.json), and
[retest.json](../../artifacts/aixforge-api-20261010/retest.json), plus the final
single-purpose [follow-final.json](../../artifacts/aixforge-api-20261010/follow-final.json)
and the later [image-first.json](../../artifacts/aixforge-api-20261010/image-first.json)
format comparison. Additional calling-mode reports are retained under
[modes/](../../artifacts/aixforge-api-20261010/modes/).
These files do not contain the API key. They cover API/task parsing and bounded
image and text transport diagnostics only;
they do not run the robot or the navigation stack.

The first text `follow` request returned HTTP 200 in 12.5944 seconds, but its
plan was `find -> follow`. This passed the general TaskPlan structure check,
but is incompatible with the specialized replay's exact-one-`follow` contract.
The `stop` request returned HTTP 200 in 6.2532 seconds and produced one `stop`
subgoal, which passed the replay contract. The decomposition prompt was then
changed so that the `follow` skill owns person selection, identity confirmation,
continuous tracking, and rediscovery; the focused prompt/replay regressions pass
38 focused task/replay tests. The final single-purpose live `follow` retest after that
change received HTTP 429 in 2.0329 seconds. Repeated follow retries were stopped
at that point, so the corrected prompt has
not yet passed live single-step acceptance.

The first image request returned HTTP 200 in 6.6419 seconds but answered that no
image had been uploaded. A native SDK non-streaming retest removed the optional
`detail` field and requested low reasoning effort; it completed in 6.9798
seconds with the same no-image answer. The request hook proves that this retest
contained one `data:image/jpeg;base64` part with 64,716 encoded characters,
representing the 48,535-byte JPEG. This proves client serialization, not that
the forwarding service delivered the image to the upstream model. Other native
plain and streaming attempts received HTTP 429 `Upstream rate limit exceeded`.
The evidence cannot distinguish whether the forwarding layer dropped the image,
rejected this image format, or routed the request to a model path without vision
support. Visual recognition through this route is therefore not accepted.

The [official Kimi K3 quickstart](https://platform.kimi.com/docs/guide/kimi-k3-quickstart)
uses a Base64 `image_url` in an object-array `content` and places the image before
the text. A later isolated AIXForge comparison followed this order, omitted
`detail`, and used the raw SDK with non-streaming output and low reasoning
effort. It returned HTTP 200 in 6.5755 seconds but answered:
`当前对话中没有收到任何图片，因此无法描述画面内容。请上传需要分析的图片，我再据实回答。`
The serialized body again contained the complete JPEG. Neither content order
nor switching to non-streaming output resolved the observed no-image response.

Further diagnostics changed the image encoding, size, endpoint, and upload
method independently:

- A 640 x 480 PNG of 194,639 bytes timed out after 33.5758 seconds.
- A reduced 256 x 192 JPEG of 7,083 bytes, sent with only `model` and
  `messages`, timed out after 30.5281 seconds.
- `GET /v1/models` completed in 1.9853 seconds and listed only `kimi-k3`.
- A `/v1/files` upload with `purpose=image` returned HTTP 403 in 3.1703
  seconds with `This token has no access to model` and an empty model name. No
  file was created, so an `ms://` reference could not be tested.
- A Responses API `input_image` request timed out after 30.8578 seconds.
- A text-only control requesting the reply `OK`, with low reasoning effort and
  the 4096-token limit, also timed out after 30.5301 seconds.
- A streaming request sent as direct HTTP without the SDK received HTTP 200 and
  SSE headers after 11.5957 seconds, but produced no final answer before the
  80.0073-second total deadline and ended with `TimeoutError`.
- A text-only request with `trust_env=False` bypassed the configured proxy
  environment and still ended in `ReadTimeout` after 20.678 seconds. No further
  direct image request was sent.

Because the text control timed out in the same window, these timeouts do not
show that PNG, a smaller JPEG, Base64 input, or the Responses route is
unsupported. A timeout also does not prove that an endpoint is absent. The 403
records this token and gateway request being refused; the empty model name and
absence of a created file do not establish that the account itself lacks image
permission. Official Kimi documents JPEG/PNG image input through Base64 or an
uploaded `ms://` file with `purpose=image`; whether AIXForge forwards each of
those official paths remains unknown.

The direct HTTP result shows that removing the SDK alone did not produce a
usable vision response. HTTP 200 and `text/event-stream` headers establish
transport admission only, not image understanding or a completed answer. The
development process had `HTTP_PROXY` and `HTTPS_PROXY` configured when this was
noticed. The no-proxy text control also timed out, so there is no evidence that
the proxy caused the failures. The collected calls show an unstable model API
path that did not consistently return complete results; they do not isolate the
failing network or forwarding component. API calls were stopped after this
control rather than adding another image request.

The provider response and model listing declared `kimi-k3`. That is retained as
the AIXForge-returned identity only; it is not independent authentication of the
upstream implementation or an official Moonshot guarantee. The observed times
are single request samples under changing rate-limit conditions and must not be
reported as p50, p95, or deployment latency. No visual accuracy or real robot
behavior was established.

## Local evidence

| Location | Contents |
| --- | --- |
| `D:\inovxio\brain\lingtu\build\person-follow\` | Baseline plan, raw LLM/VLM exchanges, initial selection images, tracks, source metadata, summary, ground truth, and independent evaluation. |
| `D:\inovxio\brain\lingtu\build\person-follow\reacquire\` | First VLM-reacquisition artifacts and its 94/41/45 evaluation. Preserve this failed run as a regression case. |
| `D:\inovxio\brain\lingtu\build\person-follow\verified\` | Positive attribute- and pair-verified run: 88 correct, 0 wrong, 92 missing. |
| `D:\inovxio\brain\lingtu\build\person-follow\tracker-association\` | Concrete assignment regression, before/after cached-observation tracks, consistent post-run evaluation, and rerun script. |
| `D:\inovxio\brain\lingtu\build\person-follow\person-no-match-verified\` | Thirty-frame absent-target run with no selected ID. |
| `D:\inovxio\brain\lingtu\build\person-follow-native-20261009-envfix\` | Failed MuJoCo/native-navigation run retained for the terminal-tolerance diagnosis. |
| `D:\inovxio\brain\lingtu\build\person-follow-native-20261009-tolerance-fix\` | Passing controlled-shirt RGB-D/native-navigation smoke. |

The most useful files are `plan.json`, `llm-calls.json`,
`selection-source.jpg`, `selection-crops.jpg`, `tracks.json`, `summary.json`,
and the corresponding evaluation JSON. The annotated video is evidence for the
specific run whose directory contains it; an older successful video must not be
used to represent this run.

- [Verified RGB inspection video](D:/inovxio/brain/lingtu/build/person-follow/verified/inspection.mp4): 6 seconds and 180 frames, with the full frame, original target crop, and current crop. The inspection layout does not change model decisions.
- [Passing native MuJoCo video](D:/inovxio/brain/lingtu/build/person-follow-native-20261009-tolerance-fix/native/motion/native_navigation.mp4): 34.75 seconds and 139 frames, encoded as H.264 at 960 x 540.

## Separate MuJoCo/native diagnostic

The MuJoCo runs are not the RGB replay and did not use learned person
recognition. Their detector was a controlled shirt-color fixture. Passing the
resolved native map environment to the sensor child fixed the fixture startup.
The first resulting run produced 329/329 visible RGB-D samples and all 12/12
follow goals were accepted. The simulated person moved 3 m.

The robot did not follow. Its path length was about 0.017 m, it had zero
non-zero velocity samples, and OctoPlanner3D repeatedly returned
`goal_snap_exhausted` while `map_check=ok`. This is useful evidence that target
observation and goal admission reached the native boundary, while executable
path generation did not. It must be reported as a navigation failure rather
than a person-follow success.

The follow-goal fixture used a 0.15 m terminal tolerance inherited from the
resolved RunPlan. On the fixture's 0.2 m map grid, body-height offset plus grid
quantization produced about 0.19 m of goal-height error, so the goal could not
snap within that tolerance. The diagnostic rerun set only the fixture's
explicit scalar tolerance to 0.25 m. It did not change the native field
Product's 0.15 m setting, or relax collision, ground-support, unknown-space,
XY, or Z planning constraints.

That controlled rerun passed: `overall_ok=true` with no native blockers. It
accepted 12/12 follow goals, produced 12 paths with zero planning failures, and
recorded 568 non-zero forward-motion samples. The robot travelled 3.5164 m
along its path and 3.3813 m net while the person moved 3 m. Person distance was
1.909 m minimum, 2.250 m median, and 3.631 m maximum. All 327 RGB-D observations
were visible, map-frame projection XY error had 0.1124 m P95, and there were no
contacts over 7,714 simulation steps. Terminal stop evidence showed physical
zero motion and cleared command authority.

This proves the controlled-shirt RGB-D observation, production
`VisualServo`/`PersonTracker` target update, native 3-D planning, and simulated
locomotion components can run together. Qwen3-VL and YOLO-World from the real
video experiment were not inputs to this MuJoCo run. It is therefore not an
end-to-end natural-language robot-follow demonstration.

Run this smoke from the repository root with a fresh output directory:

```powershell
$env:PYTHONPATH = (Get-Location).Path
.\.venv\Scripts\python.exe sim\scripts\mujoco\rgbd_follow_acceptance.py `
  --out-dir build\person-follow-native-<new-run-name> `
  --smoke
```

Do not reuse an existing output directory. An earlier attempt without
`PYTHONPATH` failed during import and did not start motion; it is not a motion
result.

## Acceptance boundary

The next defensible milestone is one retained run in which:

1. the real model parses the instruction and selects the intended person;
2. disappearance and tracker-ID fragmentation trigger bounded re-ID;
3. independent post-run attribution shows no switch to another identity;
4. an absent description returns no match;
5. the correct identity remains selected through most of the sequence;
6. RGB-D projects the selected person into the map frame; and
7. native planning produces non-zero supervised simulation motion while
   maintaining a bounded following distance.

Items 1-4 are demonstrated for this one positive clip and one thirty-frame
negative probe. Item 5 is not met: the verified run outputs the correct target
on 88 of 180 frames. Items 6-7 have component-level evidence in the separate
controlled-shirt MuJoCo smoke, including successful native motion. The missing
acceptance result is one joined run that feeds real language parsing and learned
person recognition into RGB-D map projection and native following without an
identity switch.

## Verification

For the 2026-10-11 paper-guided correction, 7 NativeByteTracker tests passed;
the new geometry regression failed on the old code before the fix. The 4
directly affected Host/replay test files passed 106 tests, covering shared
verification, reference retention, strict rejection, late replies, and complete
tracker-input records. After adding the unavailable-vision fallback regression,
the 2 directly affected selection/SDK files passed 50 tests. The latter run
rechecks existing cases and must not be added to 106 as a unique-test total.
Ruff passed on the touched Python files. API transport checks used in-memory
responses; no live API, new model inference, or field motion was tested.

Fifty focused tests covering the person-follow replay, Qwen message conversion,
post-run MOT attribution, task decomposition, and the touched tracking behavior
passed. Ruff also passed on the touched Python files. These checks establish
local contracts and artifact evaluation; they do not add field-motion or
real-time performance evidence.

For the 2026-10-10 Kimi integration, 132 focused checks passed: 45 Product/stack
checks, 25 replay checks, and 62 SDK/VisualServo/Gateway checks. They cover the
resolved real/sim module graphs, provider parameters, API/local mode selection,
missing credentials, rejection of non-visual mock backends, resource closure,
numbered multi-image messages, reuse of the Host event loop, no-match and API
error results, selection deadlines, STOP with a late reply, and Host shutdown.
Ruff passed on the touched Python files. SDK tests used in-memory HTTP replies
and made no external API calls. Model accuracy and deployment latency remain
unmeasured for Kimi.

For the K3 selection update, 58 client and goal-resolver checks passed, including
K3 request parameters, separation of reasoning output from the final person
number, and rejection of a response truncated at the generation budget. The 28
person-follow replay tests and 7 Product Host blueprint tests also passed
together, and Ruff passed on the three touched Python files. The SDK checks used
in-memory mock HTTP responses: they made no real K3 request and provide no K3
accuracy, latency, token-budget, or field-motion evidence.

For the AIXForge endpoint and follow-contract correction, 40 focused checks
passed: 38 task/replay checks and 2 real/sim tracking Host blueprint checks.
Ruff passed on `decomposition.py` and `test_product_host_blueprint.py`. These
checks prove the local request and module contracts; the final live follow
request still ended at the provider's HTTP 429 response.

## DINOv3 continuous-video comparison (2026-10-11)

The optional DINO path is implemented and evaluated with real model inference.
It is not enabled by default in the `tracking` Product. CLIP remains the
text/image selector; DINO supplies a separate, normalized 384-dimensional
person-crop descriptor. Neither DINO features nor its scores enter saved CLIP
object features, depth projection, or native motion authorization.

The implementation is in `src/perception/encoding/dino.py`,
`src/decision/vision/person.py`, and `VisualServoModule.setup()`. A current
tracker ID that passes appearance validation remains selected, even when a
nearby person scores higher. Only a missing/rejected current ID starts the
candidate search; a new ID still needs three consecutive confirmations.
Accepted observations alone update the adaptive appearance descriptor; the
original target descriptor stays fixed across absence and reappearance. This is LingTu's
association logic, not an official DINOv3 person-tracking algorithm.

Host consumer fields are explicit:

```yaml
host:
  person_text_model_path: /models/clip/ViT-B-32-state.pt
  person_text_device: auto
  person_appearance_model: /models/dinov3/dinov3-vits16-336.onnx
  person_appearance_device: auto
  person_appearance_cpu_threads: 4
  person_appearance_min_similarity: 0.65
  person_appearance_margin: 0.03
```

The CLIP Host path expects an OpenCLIP-compatible checkpoint. The ONNX graph
and its `.onnx.data` weights must remain together. The two DINO thresholds are
recorded engineering settings, not calibrated identity probabilities.

Experiments ran in `/root/autodl-tmp/lingtu-dino-20261011`, preserving the prior
server source. YOLO-World and CLIP used the server's RTX 5090; DINO used ONNX
Runtime **CPUExecutionProvider**, because the installed runtime has no CUDA
provider. Three crop probes took 72-91 ms. Video playback follows the source
frame cadence and is not evidence of real-time inference.

Both A/B paths consume the same video, real YOLO-World detections, and one
fresh NativeByteTracker output. MOT labels are read only after execution.
Offline YOLO-World text encoding also needs the existing OpenAI
`ViT-B-32.pt` under the working directory's `weights/clip/`; the experiment
copied the server's complete cached model there instead of downloading it.
The DINO model is the public [Kornia ONNX conversion](https://huggingface.co/kornia/dinov3)
of Meta's `facebook/dinov3-vits16-pretrain-lvd1689m`, with no new training.
Model/license files and datasets are retained under `build/dino-follow/assets/`.

| MOT17-09, first 180 frames | CLIP baseline | CLIP + DINOv3 |
| --- | ---: | ---: |
| Correct reference-person attribution | 101 | 110 |
| Other-person attribution | 55 | 4 |
| Unattributed output | 2 | 0 |
| Missing target | 22 | 66 |
| Upstream tracker ID changes | 2 | 3 |
| Mean tracker-update time, excluding detection | 82 ms | 335 ms |

The initial box comes from the earlier VLM-selected gray-shirt/pink-bag
reference, not from MOT labels. Post-run IoU attribution uses threshold 0.5
and reference GT22. This is a selected-target diagnostic, not standard HOTA or
IDF1. Overlapping people can change which annotation owns a predicted box;
the four other-person frames must not be presented as zero identity error.
The actual selected ID sequences were `6 -> 12 -> 2` and
`6 -> 12 -> 4 -> 28`. Correct-reference coverage was 56.1% and 61.1%; adding
DINO reduced wrong-person output but did not establish continuous identity.

| TUM Freiburg3 walking_xyz, first 300 RGB-D pairs | CLIP baseline | CLIP + DINOv3 |
| --- | ---: | ---: |
| Target output / missing frames | 226 / 74 | 111 / 189 |
| Upstream tracker ID changes | 3 | 0 |
| Raw adjacent-position step, p95 | 8.42 cm | 3.79 cm |
| Raw adjacent-position step, maximum | 1.99 m | 11.83 cm |
| Raw adjacent steps over 0.5 m | 2 | 0 |
| EMA adjacent-position step, p95 | 10.55 cm | 2.60 cm |
| Mean tracker-update time, excluding detection | 15.9 ms | 55.9 ms |

CLIP selects the plaid-shirt person from the text `a person wearing a plaid
shirt`. DINO retains ID1 for the first 111 frames, then rejects a candidate
with similarity 0.478. The baseline subsequently follows the black-shirt
person. Inspection of the complete sequence also shows later plaid-shirt
observations that this original run did not recover. Therefore its missing
outputs include both appropriate refusals and failures to reacquire; they
must not all be described as the target being outside the image.

Projection uses the existing `bbox_center_depth()` and `project_to_3d()`:
Freiburg3 intrinsics `(535.4, 539.2, 320.1, 247.6)`, raw depth divided by
5000, and the dataset camera-to-world pose. It tracks a box-centre depth
representative, not a body centroid. Camera ground truth excludes SLAM drift
from this experiment. There is no person 3D or identity ground truth in TUM;
position metrics establish continuity only. Steps exclude missing-frame gaps,
which are reported separately; raw and EMA traces preserve those gaps.
The baseline's 1.83 m / 1.99 m jumps at frames 165 and 167 occur within ID2.
DINO's rejection of that substituted person does not fix the underlying depth
sampling. RGB-only MOT records explicitly keep XYZ null.

Reusable case: `tools/datasets/dino_follow.py`. Exact invocations:

```bash
export PYTHONPATH="$PWD/src:$PWD"
python tools/datasets/dino_follow.py \
  --video /root/autodl-tmp/lingtu-vlnav/data/mot17-09/input.mp4 \
  --yolo-weights /root/autodl-tmp/lingtu-vlnav/yolov8s-worldv2.pt \
  --clip-model-path /root/autodl-tmp/lingtu-vlnav/weights/clip/ViT-B-32-state.pt \
  --dino-model /root/autodl-tmp/models/dinov3-vits16-onnx/dinov3-vits16-336.onnx \
  --target-bbox 1286.97 462.72 1358.973 658.427 --max-frames 180 \
  --gt /root/autodl-tmp/lingtu-vlnav/data/mot17-09/gt/gt.txt \
  --output results/mot17

python tools/datasets/dino_follow.py \
  --rgbd-sequence data/rgbd_dataset_freiburg3_walking_xyz \
  --yolo-weights /root/autodl-tmp/lingtu-vlnav/yolov8s-worldv2.pt \
  --clip-model-path /root/autodl-tmp/lingtu-vlnav/weights/clip/ViT-B-32-state.pt \
  --dino-model /root/autodl-tmp/models/dinov3-vits16-onnx/dinov3-vits16-336.onnx \
  --target-description 'a person wearing a plaid shirt' --max-frames 300 \
  --output results/tum
```

Results are retained in `build/dino-follow/results/{mot17,tum}/`:
`metrics.json`, `tracks_baseline.json`, `tracks_dino.json`, readable H.264
`follow-view.mp4`, and previews. TUM also has `position.png` with raw/EMA XYZ
and a visibility trace. Display overlays were refreshed from saved records;
the inference decisions were not changed. The final tool records the same
parameters and relative time directly for future runs.

The final six focused tracking, encoder, Host and replay test files passed
80 tests; Ruff passed on the touched Python surfaces. Model inference and
video codec checks were separate from these local contract tests.

The decision is to retain DINO as an explicit comparison option, not make it
the default. Appearance verification is useful; this test does not establish
that DINO is the best person-specific encoder. A returning-target and depth
sampling comparison is still needed before deployment. No Habitat, MuJoCo,
MiniCPM policy inference, or field motion was run for this DINO experiment.

MiniCPM-RobotTrack's [official deployment guide](https://github.com/OpenBMB/MiniCPM-Robot/blob/f666593066641d2c0cef06bc17bfb1e954547674/MiniCPM-RobotTrack/docs/GO2_DEPLOYMENT.md)
places DINO + SigLIP TensorRT encoders and the local MiniCPM policy on a
Go2 EDU's Jetson Orin NX 16GB. LingTu has not integrated that trained policy;
the crop-association experiment above is not a reproduction of it.

## Initial returning-target and depth correction (2026-10-11)

The follow Host previously reset the selected DINO identity after its lost
timeout or a perception watchdog timeout. It now cancels the native navigation
goal but preserves the initial identity and selection while reporting
`reacquiring`. Subsequent observations continue through `PersonTracker`.
Explicit stop or a new target clears the reference. The non-DINO selector
retains its existing behavior.

The original target crop and DINO descriptor are immutable. Similarity is
computed against the original and at most eight VLM-verified recent views;
adaptive observations alone cannot create identity anchors. The score is a
cosine-similarity gate, not an identity probability. The reachable area
expands with elapsed source time so a returning person can have moved beyond
the old fixed two-metre search radius. A changed ID or a return after more
than one second requires three distinct, increasing-time observations with
continuous positions; repeated frames do not complete confirmation.

The first full-sequence correction still accepted the black-shirt person's
ID22 and rejected many correct view changes. It is retained as
`full-person-depth`, a failed intermediate result, and is not a successful
return test. Tighter frozen-reference gating reduced the selected MOT17
case's wrong-person attribution from four frames to zero, but reduced correct
output from 110 to 67 frames; that trade-off alone was insufficient.

The final association uses the existing `verify_candidate()` helper after a
stable proposal. During a long absence, retained CLIP text retrieval can rank
which candidate to propose using the original instruction. This addresses a
real failure in which the seated plaid-shirt target had lower DINO similarity
than the black-shirt person. Text ranking does not accept an identity or a
position: it only chooses the candidate sent through the same confirmation
and image-verification path. A changed ID or a long absence cannot authorize
itself from DINO similarity. The Host starts background image verification against the
original crop and keeps the native goal cancelled. `MATCH` only queues a
candidate-specific authorization; the next observation must have a newer
source time, the same ID, continuous motion and a compatible verified view
before position or identity changes. In this initial run, `OTHER` blocked that
ID while it remained visible; `UNKNOWN` or a model failure cleared the proposal
and imposed a global one-second source-time retry delay. These two policies
were replaced by the candidate-specific retry correction below. Stop, a changed instruction, an intervening
absence and stale callbacks invalidate authorization. A Host without an
image-capable verifier stays unconfirmed rather than switching people.

Person depth now uses `person_depth_position()` in
`src/perception/tracking/projection.py`, shared by the RGB-D observation source
and the video replay. With an instance mask it samples that mask; otherwise
it samples the inner torso portion of the detection box. It chooses a
supported foreground depth layer instead of the box centre, which can show
a wall through the person's arms. Missing support produces no position.
The output is a visible-surface representative, not a ground-truth body
centroid or robot navigation pose.

Before updating position, velocity or adaptive features, timestamped
measurements must be within `3 m/s * dt + 0.2 m` of the previous raw
measurement. These are bounded walking-follow engineering settings, not a
universal model of human motion. After a confirmed long absence, position is
reseeded from the current observation and velocity starts at zero; no EMA
path is fabricated across the missing interval. The original raw observations
and rejection states remain available for inspection.

The reusable replay saves `detector_cache.json` with original detections,
NativeByteTracker rows, source frame indices and timestamps. Replaying that
cache with `--rgbd-sequence ... --tracks detector_cache.json` holds the input
constant. `--depth-method center` preserves the previous depth sampler for
comparison; the default is `person`. The complete official Freiburg3
`walking_xyz` sequence contains 859 RGB frames and 833 depth frames over
approximately 28.8 seconds, with multiple natural exits, returns and crossings.
The 20 ms RGB/depth association yields 827 valid pairs spanning the full clip;
32 RGB images lack matching depth and are not used for three-dimensional
positions. `source_pair_index` identifies the synchronized pair, while source
timestamps identify the original image. The data are not reordered or reversed
to manufacture a return.

The first verified full run is `build/dino-return/results/verified-return-text/`.
All 827 cached detections, NativeByteTracker rows, pair indices and source
timestamps are equal to the retained `before` run. Initial selection uses
CLIP with `a person wearing a plaid shirt`; no person labels or identities
from the dataset are supplied to tracking or verification.

| Evidence | Final result |
| --- | --- |
| Source coverage | 827 RGB-D pairs, 28.824 s, original chronological order |
| Accepted / unconfirmed observations | 164 / 663 |
| First changed-ID return | Pair 225, 7.81 s, ID 1 -> 4 |
| Second changed-ID return | Pair 682, 23.86 s, ID 4 -> 28 |
| Seated-view recovery | Pair 780, 27.21 s, ID 28; accepted through pair 826 |
| Identity calls | 15: 4 MATCH, 3 OTHER, 8 UNKNOWN |
| Accepted-crop manual inspection | All 164 depict the initial plaid-shirt person |
| Adjacent raw XYZ displacement | P95 0.1298 m, max 0.2469 m; none above 0.5 m |
| Adjacent filtered XYZ displacement | P95 0.0624 m, max 0.1064 m |

The last recovery is a genuine improvement over `verified-return`, which
accepted 117 frames and never recovered the seated view. CLIP proposal
ranking selected the plaid-shirt crop at pair 779, Qwen returned MATCH, and
pair 780 passed fresh-frame confirmation. The earlier long-gap recoveries
occurred after 116 and 445 unconfirmed pairs. These counts describe algorithm
output gaps; they are not independently labelled physical out-of-view
intervals. Native tracker ID changes are temporary detection bookkeeping,
not evidence of changing the followed person's identity.

This remains an incomplete tracking result. The target is visibly present
during some unconfirmed intervals, and pair 404's partial plaid-shirt crop
was incorrectly classified OTHER. UNKNOWN on blurred or partial views also
delays recovery. The 663 unconfirmed pairs include actual absence as well as
these missed observations, so they are not a false-negative rate. TUM has no
person-identity or body-position ground truth for this evaluation; manual
crop inspection is not a standard IDF1/HOTA score. The raw-step statistics
only cover accepted, adjacent observations and cannot establish position
accuracy for missing frames.

Readable outputs are retained in `build/dino-return/results/comparison/`:
`return-view.mp4` is an H.264 before/final comparison with 827 readable
frames, original source-time playback and a 28.86 s encoded duration.
`position.png` shows raw XYZ with missing intervals left empty. The final
run also retains `metrics.json`, `tracks_dino.json`, `tracks_baseline.json`,
`detector_cache.json`, all candidate/reference images in `identity-attempts/`,
`identity-audit.jpg`, and `all-accepted-audit.jpg`.

The independent selected-target depth comparison is saved as
`comparison/depth-sample-selected-target.json` and `.png`. For the same
plaid-shirt native ID 28, detection boxes, source depth and camera transform,
pair 767 -> 768 changes from a 0.513 m box-centre raw XYZ step to a 0.027 m
person-region step; pair 772 -> 773 changes from 0.501 m to 0.029 m. These
measurements are before EMA or motion rejection. Following was still
unconfirmed at those pairs, so this is a depth-component diagnostic, not
additional accepted-follow observations. The separate black-shirt ID 2
diagnostic at pairs 164-168 shows a 1.988 m centre-depth step versus 0.066 m
with person sampling; it is not the selected follow identity. Foreground
sampling does not improve every observation: at ID 4 pairs 225-236, old and
new maximum steps are 0.081 m and 0.116 m respectively.

Re-run the final case on the authorized research server with the existing
models and cached detector output:

```bash
research_root=/root/autodl-tmp/lingtu-dino-return-20261011
export PYTHONPATH="$research_root/source/src:$research_root/source"
export HF_HUB_OFFLINE=1
cd "$research_root"
/root/autodl-tmp/lingtu-vlnav/vision/bin/python source/tools/datasets/dino_follow.py \
  --rgbd-sequence "$research_root/data/rgbd_dataset_freiburg3_walking_xyz" \
  --tracks "$research_root/data/tum-person-detector-cache.json" \
  --target-description 'a person wearing a plaid shirt' \
  --clip-model-path /root/autodl-tmp/lingtu-vlnav/weights/clip/ViT-B-32-state.pt \
  --dino-model /root/autodl-tmp/models/dinov3-vits16-onnx/dinov3-vits16-336.onnx \
  --identity-model /root/autodl-tmp/models/qwen3-vl-2b \
  --max-frames 1000 --depth-method person --device auto \
  --output "$research_root/results/verified-return-text"
```

Actual Qwen3-VL-2B-Instruct and CLIP ran on the server GPU. The installed ONNX
Runtime exposes CPUExecutionProvider for DINO, so the rendered playback is
not a real-time or embedded-board performance demonstration. Camera poses
come from official TUM ground truth: this tests depth sampling, projection
and association with a known camera transform, not live SLAM pose quality.
No simulator policy or field robot motion was run for this case. Host tests
separately confirm navigation-goal cancellation while reacquiring, fresh-frame
authorization, stale callbacks, stop, and perception timeout behavior.

At this stage, the eight directly affected test files passed 140 tests. Ruff passed on
the modified tracking, depth, Host, replay and test surfaces. Cached-input
equality, actual model inference, crop inspection and H.264 decoding are
separate evidence from those local contract tests.

## Candidate-specific retries after return (2026-10-11)

The initial full video still delayed or missed visible returns. Cached
observations establish three state-machine causes, separately from VLM errors:

- At pairs 723-744, the known ID 28 remained visible but DINO similarity was
  below 0.65. The current-ID low-similarity branch reset confirmation and
  waited for a gap over one second before permitting a VLM proposal.
- Black-shirt ID 29 received UNKNOWN at pair 747. The global delay then
  blocked the actual plaid-shirt ID 28 at all 29 pairs 748-776, although both
  IDs remained in the cached detections. The target recovered only at 780.
- Plaid-shirt ID 17 was incorrectly classified OTHER at pair 404. The
  permanent visible-ID rejection prevented improved views of the same person
  from being verified again through approximately pair 446.

The corrected tracker permits a low-similarity current ID to build the same
three-frame proposal immediately. It does not lower the DINO acceptance
threshold or commit a low-similarity observation. OTHER, UNKNOWN and model
failure set a candidate-specific 0.3-second source-time retry interval. That
candidate is excluded from proposal ranking during the interval; other
visible candidates can proceed. After the interval, three new continuous
observations and a new VLM MATCH are still required. A rejection does not
become trusted just because its DINO score rises again.

The rejected current ID remains untrusted across detector dropouts. Its
retry entry is cleared by a new explicit target, stop/reset, or MATCH, not by
one missing frame. Non-current IDs that are no longer visible can be removed
because they already require changed-ID verification if they return. Pending
proposals and queued MATCH authorizations still require actual visibility
and are discarded on absence. The retained retry state is bounded by the
current target plus visible candidates, rather than accumulating all historic
detector IDs. The replay records `identity_retry_until` for each source frame
so candidate waiting can be distinguished from detection absence.

Five new regression cases reproduced the old failures before the fixes:
short-gap current-ID view change, a false OTHER that stays visible, UNKNOWN
on one person blocking another, a rejected current ID regaining a high DINO
score, and a rejected current ID disappearing for one detection frame. The
directly affected tracker, Host and replay test files passed 125 tests; the
final identity-state expression also passed all 23 person-identity tests and
Ruff.

The final real-inference run is retained at
`build/dino-return/results/return-retry-dropout/`. It uses the same official
TUM `walking_xyz` sequence as `verified-return-text`: all 827 synchronized
RGB-D pairs, their original chronological order, and identical detections and
native tracker rows. Input comparison found no differences in source frame,
timestamp, detections or tracks. Model weights, thresholds and person-depth
projection also remain the same; the change is the identity retry logic.

| Measurement | Initial verified run | Candidate-specific retries |
| --- | ---: | ---: |
| Accepted target observations | 164 | 260 |
| Missing target observations | 663 | 567 |
| Changed-ID return, ID 4 | pair 225, 7.81 s | pair 215, 7.47 s |
| Changed-ID return, ID 28 | pair 682, 23.86 s | pair 650, 22.75 s |
| Seated-view recovery, ID 28 | pair 780, 27.21 s | pair 756, 26.41 s |
| Qwen image-verification calls | 15 | 55 |
| Total Qwen inference time | 3.48 s | 9.91 s |

The 96 additional accepted observations increase the accepted count by
58.5%; this is not a dataset recall score, because missing observations also
include real target absence. Manual inspection of every accepted crop found
the intended plaid-shirt person in 260/260 observations. All eight MATCH
decisions were also that person; no false MATCH or accepted wrong person was
observed in this sequence. Native detector IDs still change from 1 to 4 to
28, while the selected target reference stays the same.

The final raw adjacent accepted-position displacement has P95 0.132 m and
maximum 0.257 m; the filtered trace has P95 0.068 m and maximum 0.140 m. No
adjacent accepted step exceeds 0.5 m. These statistics omit missing intervals
and do not establish ground-truth person-position accuracy: camera poses are
the dataset reference poses, and sampled positions represent visible person
surfaces rather than a fixed anatomical point.

The tradeoff is increased image-verification work. Of 55 calls, eight returned
MATCH, 22 OTHER and 25 UNKNOWN. Median model time was 173 ms and P95 276 ms
on the research server's RTX 5090; 17 OTHER calls came from repeatedly
evaluating black-shirt ID 22. Total model time is not end-to-end frame latency.
The video is an offline replay at the source cadence, not evidence of a
real-time robot or embedded-board deployment. A later performance change
should address repeated stable OTHER candidates without weakening identity
acceptance or making their rejection permanent again.

The hard partial-body return is still unresolved: ID 17 receives UNKNOWN
at pairs 372, 382, 393, 405, 416, 427 and 436. Retry now occurs, so it is no
longer permanently suppressed by the earlier false OTHER; the available
candidate crops still lack sufficient identity evidence. Neither this run nor
its tests prove reliable following under all occlusions or live SLAM.

Retained evidence:

- `build/dino-return/results/retry-comparison/return-view.mp4`: Chinese
  side-by-side H.264 replay of all 827 pairs, about 28.8 seconds.
- `build/dino-return/results/retry-comparison/position.png`: raw XYZ and
  accepted-target traces, with missing intervals left empty.
- `build/dino-return/results/return-retry-dropout/all-accepted-audit.jpg`:
  all 260 accepted target crops.
- `build/dino-return/results/return-retry-dropout/identity-audit.jpg` and
  `identity-attempts/`: decisive judgments and all candidate/reference crops.

A separate actual Qwen A/B diagnostic is retained at
`build/dino-return/multiview-diagnostic/`. The second reference is the trusted
front view verified at pair 99 and accepted on fresh pair 100. Adding it to
the original reference changes two blurred true-target crops, pairs 192 and
650, from UNKNOWN to MATCH. Four partial-body crops, pairs 373, 404, 405 and
416, remain UNKNOWN; neither of the two tested black-shirt crops becomes
MATCH. This eight-crop diagnostic supports a limited multiview benefit, not
a completed full-video multiview implementation. At that diagnostic stage,
product code still sent only the original reference to Qwen. The following
implementation connects trusted views and temporal candidate evidence.

## Multiview and temporal identity evidence (2026-10-12)

The actual follow Host and the recorded-data runner now use the same
`verify_candidate` image protocol. The first target crop remains the identity
anchor. The tracker can retain up to two additional trusted RGB views, and a
reacquisition proposal contains up to three chronological crops of the same
candidate track. Candidate frames are evidence for comparison, not trusted
target references. The model must confirm the latest candidate and reject
identity-inconsistent sequences rather than finding a matching person in any
one image.

Additional references are admitted only after a VLM MATCH is followed by a
fresh observation that passes the DINO and 3D position checks. The MATCH
callback itself does not change identity, position or the RGB gallery. Stop
or a new target clears the gallery; target loss retains it. Candidate image
sequences are discarded on disappearance, candidate-ID change or explicit
perception interruption. The pending request snapshots its images so later
camera updates cannot change the evidence sent to the model.

Reference crops with DINO cosine similarity at least 0.98 to an existing
reference are skipped, so repeated views do not evict a different viewpoint
or consume another image slot. This threshold selects image evidence; it
does not change the 0.65 identity acceptance threshold or authorize motion.
When a third distinct view arrives, the two retained references maximize
their combined cosine distance from the original anchor and each other;
the oldest useful view is not unconditionally removed.

Repeated OTHER or UNKNOWN results for the same visible candidate back off
for 0.3, 0.6 and then 1.2 source seconds. The retry level is capped before
calculating the interval. A target-similarity improvement of at least 0.08,
or crossing from below to at least the 0.65 threshold, permits earlier
rechecking. Arbitrary pose change alone does not trigger another model call.
Rechecking still requires three consistent observations and a new MATCH.
Candidate-specific state and current-ID rejection across dropouts remain
intact. OTHER denotes a visible clothing or carried-item contradiction;
insufficient evidence or an identity-inconsistent sequence remains UNKNOWN.

The Host also applies its existing 0.1-second RGB/detection timestamp
tolerance during tracking and reacquisition, rather than only initial
selection. An unmatched image cannot be cropped with the new detection box.
The DINO missing-image branch discards partial candidate evidence and pending
or queued authorizations while keeping the target reference. Matching RGB
that arrives after detections can resume an unconfirmed observation without
reprocessing an already accepted one. Explicit-ID DINO selection waits for
the matching image before constructing its initial identity reference.

An asynchronous verification is owned by its intent generation, candidate
ID and source timestamp. A new candidate cancels a superseded owned future;
the obsolete reply cannot resolve the new proposal or clear its running
state. This avoids waiting for an absent candidate's API timeout before
checking a newly visible candidate.

Replay artifacts retain every input reference and candidate image, their
source timestamps, the exact model response and call duration. These records
allow a recovered observation to be traced to the images that authorized it.
Local tests establish the evidence and authorization contracts; full-video
inference and manual identity inspection provide separate outcome evidence.
The five directly affected tracker, Host and replay test files pass 145 tests;
the six existing initial-selection/pair-verification cases also pass. Ruff
passes on the three implementation files and their affected test files.

The first real full-video iteration is retained as `return-temporal`. It
accepts 296 target observations but makes 97 calls, versus the previous 260
observations and 55 calls. It returns 12 MATCH, 85 UNKNOWN and no OTHER; 49
UNKNOWN calls come from non-target ID 22. All 296 accepted crops were
manually inspected as the plaid-shirt target, including the new partial-body
ID 17 recovery. The increased count does not justify the call regression.
This concrete failure motivated the graduated UNKNOWN retry and the explicit
visible-contradiction OTHER wording described above. The first iteration is
preserved separately from the final source run.

The final source run is retained as
`build/dino-return/results/return-temporal-final/`, with the same full 827-pair
TUM sequence and previous `return-retry-dropout` inputs.

| Measurement | Previous candidate retries | Final multiview/temporal run |
| --- | ---: | ---: |
| Accepted target observations | 260 | 290 |
| Missing target observations | 567 | 537 |
| Image-verification calls | 55 | 36 |
| Sum of verification request time | 9.911 s | 10.985 s |
| Mean tracker update time | 76.01 ms | 105.51 ms |
| P95 tracker update time | 171.41 ms | 181.25 ms |
| Raw adjacent-step P95 | 0.132 m | 0.109 m |
| Raw adjacent-step maximum | 0.257 m | 0.257 m |
| Filtered adjacent-step P95 | 0.068 m | 0.064 m |
| Filtered adjacent-step maximum | 0.140 m | 0.102 m |

Calls decrease by 19 (34.5%), and accepted observations increase by 30
(11.5%). Multi-image requests cost more individually, so fewer calls do not
yet mean lower cumulative request time or faster tracking updates. The final
model-call envelope sums to 10.850 s; the 10.985 s request total additionally
includes the verifier's image encoding and call handling. Both timings are
from this actual research-server run, not an embedded-platform benchmark.
No accepted adjacent raw or filtered step exceeds 0.5 m.

| Recovery segment | Previous acceptance | Final acceptance |
| --- | --- | --- |
| First changed-ID return, ID 4 | pair 215, 7.476 s | pair 224, 7.776 s |
| Partial-body return, ID 17 | never accepted | pair 382, 13.367 s |
| Later changed-ID return, ID 28 | pair 650, 22.754 s | pair 645, 22.550 s |
| Seated-view recovery, ID 28 | pair 756, 26.406 s | pair 742, 25.905 s |

These are elapsed positions in the source video, not API response latencies.
Recovery is not uniformly faster: the first changed-ID return is 0.300 s
later, while the last two recoveries are 0.204 s and 0.501 s earlier. The main
new capability is correct partial-body reacquisition. ID 17 is accepted at
pairs 382-387, 389-390, 394-399 and 402-409 (22 observations); it is still
unconfirmed in other partial-view intervals. The remaining missing count
includes true absence and missed target observations, so it is not a recall
score and does not establish complete recovery under every occlusion.

The final model outputs are eight MATCH and 28 UNKNOWN, with no OTHER or
invalid response. Every request contains three chronological candidate
images. The first MATCH request contains the original reference only; the
second contains one additional trusted view; later MATCH requests contain
two additional views. Clear negative classification remains a limitation:
non-target candidates are often UNKNOWN instead of OTHER, but neither
verdict authorizes acceptance. Black-shirt ID 22 is evaluated nine times,
compared with 17 in the previous run and 49 in the first temporal iteration.

The final and previous detector-cache records were compared across all 827
pairs: source frame, timestamps, detections and native tracker output have
zero differences. This isolates the comparison to the identity-evidence and
reacquisition changes. Manual inspection of all 290 accepted target crops
found the same plaid-shirt person throughout, including native IDs
`1 -> 4 -> 17 -> 28`. All eight MATCH requests were also inspected with their
actual references and consecutive candidate images; no other person was
observed in accepted evidence. This is evidence for this sequence, not a
guarantee against identity errors in other videos.

The complete comparison video is
`build/dino-return/results/temporal-comparison/return-view.mp4`: H.264,
YUV420P, 1280 x 700, 827 frames and 28.859 seconds. A full decode succeeds.
The left side shows the previous result; the right side shows the final
multiview/temporal result. Both sides use the same source frames and elapsed
timestamps, with no interpolation across missing observations.

`build/dino-return/results/return-temporal-final/identity-proof-pair-381.jpg`
shows the six actual images used to recover ID 17: the original reference
from pair 0, trusted views from pairs 64 and 83, and consecutive candidate
images from pairs 379, 380 and 381. Qwen returns MATCH for this request; the
next fresh observation, pair 382, becomes accepted after the tracker checks.
The same result directory contains `metrics.json`, the complete
`identity-attempts/` request evidence, three `all-accepted-audit-page-*.jpg`
pages and `identity-audit-page-001.jpg` for independent inspection.

These are offline RGB-D replay results with dataset camera poses. XYZ denotes
the observed body-surface position; the adjacent-step statistics exclude
missing-observation gaps. They establish neither absolute person-position
accuracy nor live SLAM, Habitat motion or robot-side follow readiness.

## Removing repeated appearance work (2026-10-12)

Profiling the preceding multiview run shows that 10.850 of its 10.985 seconds
of verification-request time is spent inside the Qwen processor/generation
envelope. JPEG/Base64 conversion and the remaining wrapper account for about
0.135 seconds across 36 requests. Copying fewer Host image snapshots would
not resolve that model cost; those copies retain owned evidence for the
background request. The 512-token generation ceiling is also not the number
of tokens actually generated: the saved answers already stop at one verdict.

The previous tracker contained work that its active matching path did not use:

- Each DINO-accepted frame also updated a CLIP image EMA and, when attached,
  an OSNet EMA. `_match_person` uses DINO whenever its encoder and target
  feature are available, so those backup features do not enter the decision.
  Their updates now run only while the fallback appearance path owns
  association. CLIP remains available for target-description retrieval.
- Long-absence retrieval used to encode text and an image even when there
  was only one candidate. Removing that ranking leaves the same candidate
  subject to all DINO, temporal, VLM and fresh-geometry checks. Multiple
  candidates still use CLIP retrieval.
- Retrieval repeatedly calls `encode_text`, but the existing CLIP encoder
  already caches the normalized text embedding. The timed baseline records
  109 calls totalling only 4.13 ms. A trial PersonTracker cache duplicated
  that ownership and was removed; it is not counted as a speed improvement.
  Person image features are not cached across observations.

The existing `_frame_dino` cache already avoids duplicate DINO forwards
between matching and updating the same observation. No additional DINO
image cache, reduced image budget, fewer temporal frames or weaker identity
threshold was introduced. The original anchor, trusted reference admission,
negative-verdict state and next-observation geometry checks remain intact.

Three counting tests cover unused-backup removal, single-candidate retrieval
removal and retained multicandidate text selection. The six directly
affected tracker/Host/replay/model-adapter suites pass 151 tests; six existing
initial-selection and pair-verification cases also pass. Ruff passes on the
changed code and test surfaces.

The instrumented 4-thread baseline reproduces the same 290 accepted
observations, 36 calls and eight MATCH / 28 UNKNOWN results. Its DINO variant
makes 846 DINO forwards totalling 84.013 s and 404 CLIP image calls totalling
3.254 s. Mean tracker-update time is 105.87 ms, versus the preceding
uninstrumented run's 105.51 ms. These encoder counts and timings are scoped to
the DINO variant; the separate CLIP-only comparison tracker is excluded.

An intermediate run with the redundant-work removal still uses four DINO
threads. It reduces CLIP image calls from 404 to 12 and accepts exactly the
same observations, but its DINO total also changes from 84.013 to 69.450 s
with all 846 forwards retained. That variation is not caused by removing
DINO computation. Consequently, the full 105.87 -> 84.27 ms reduction cannot
be attributed to the CLIP change. The text-cache experiment is removed
before the final source run.

CPU threads were separately measured on the same 20 real crops, three
repetitions per setting after three warm-up calls. The research server
reports 128 logical CPUs. Model weights, crop preprocessing, input resolution
and descriptors are unchanged:

| ONNX CPU threads | Mean crop-encoding time |
| --- | ---: |
| 1 | 228.33 ms |
| 2 | 121.84 ms |
| 4 | 78.07 ms |
| 8 | 54.82 ms |

All normalized descriptor values are exactly equal across these settings
on the tested crops; maximum absolute difference is zero. Eight threads are
29.8% faster than four in this warmed-crop measurement. This selects an
explicit eight-thread setting for the research-server replay; it does not
change other deployments' default.

The thread budget is configurable end-to-end as the optional Host field
`person_appearance_cpu_threads`, through the existing Blueprint/provider
construction path. Its default remains four. Direct recorded-data replay
uses `--dino-cpu-threads 8` for this server. The encoder, Product Host field,
immutable RunPlan and provider tests cover explicit eight-thread propagation
and retained four-thread defaults. The final eight affected suites pass 169
tests, and Ruff passes on their changed source and test surfaces.

The final full replay is retained as
`build/dino-return/results/return-speed-final/`; the instrumented previous
source is `return-speed-baseline/`. The previous source hard-codes four
threads and predates the explicit metrics parameter; the final metrics record
`dino_cpu_threads: 8`.

| Measurement, DINO tracking variant | Previous source, 4 threads | Final source, 8 threads |
| --- | ---: | ---: |
| Accepted target observations | 290 | 290 |
| Image-verification calls | 36 | 36 |
| MATCH / UNKNOWN | 8 / 28 | 8 / 28 |
| Mean tracker update | 105.87 ms | 62.04 ms |
| P95 tracker update | 180.42 ms | 120.33 ms |
| Sum of tracker update time | 87.555 s | 51.309 s |
| Sum of identity-request time | 7.518 s | 9.961 s |
| Sum of tracker plus identity time | 95.074 s | 61.271 s |
| CLIP image calls | 404 | 12 |
| DINO crop forwards | 846 | 846 |
| DINO encoding time | 84.013 s | 50.749 s |

The observed mean tracker-update reduction is 41.4%, P95 reduction is 33.3%,
and tracker-plus-identity reduction is 35.6%. This total covers only the DINO
tracker's updates and its identity-verification requests; it excludes model
loading, cached detections, the separate CLIP-only tracker and rendering.
Qwen's final request time increases instead of decreasing. Identical model
inputs and generation settings, plus the intermediate run's variation,
prevent attributing every timing difference to source changes. The direct
causal evidence is removed CLIP work and the separate warmed-crop thread
measurement. The 28.859-second video remains source-time playback, not proof
of real-time processing.

All 827 detection/native-track inputs match. Each source frame's target,
ID, raw/filtered XYZ, event, candidate and verdict match the previous source.
All 36 requests retain the same candidate/reference timestamps, image counts
and input/output token counts. Thus the earlier audit of 290 accepted crops
continues to apply, with no additional person accepted or target interval
lost. Raw and filtered adjacent-step maxima remain 0.257 m and 0.102 m.
The partial-body and seated-view recoveries remain at pairs 382 and 742.

`build/dino-return/results/speed-comparison/return-view.mp4` is the final
H.264, YUV420P, 1280 x 700 comparison. All 827 frames decode successfully.
Each side displays the current tracker time, identity time and cumulative
tracker-plus-identity time. Key frames 404 and 742 were inspected again;
both sides retain the plaid-shirt target, with the black-shirt distractor
unaccepted. The exact CPU-thread measurements are retained in
`build/dino-return/results/dino-thread-benchmark.json`; each result directory
retains its full `metrics.json`, track records and actual request images.

Reproduce the final run with the command in the earlier RGB-D case, adding
`--dino-cpu-threads 8` and changing its output directory to
`"$research_root/results/return-speed-final"`. Choose the thread budget by
measurement on each deployment host. No Product is enabled, field thread
budget is changed or robot motion is authorized by this research replay.

## GPU appearance backend (2026-10-12)

The research server has CUDA-capable PyTorch 2.7.0+cu128 on an RTX 5090,
Transformers 4.57.6 and timm 1.0.30. Its ONNX Runtime 1.29.0 exposes only
AzureExecutionProvider and CPUExecutionProvider. Consequently the preceding
ONNX replay's `device=auto` selects CPU even though CLIP and Qwen use CUDA.
An explicit CUDA request for that ONNX file still fails when the CUDA
execution provider is absent; it does not silently fall back.

The GPU experiment uses the public
[`timm/vit_small_patch16_dinov3.lvd1689m`](https://huggingface.co/timm/vit_small_patch16_dinov3.lvd1689m)
conversion of Meta's ViT-S/16 weights. The checkpoint is retained at
`/root/autodl-tmp/models/timm-dinov3-vits16-lvd1689m` with its configuration,
model card and license. No dependency was installed. `DinoEncoder` loads it
through the existing Transformers `AutoModel` path on CUDA in FP32, reads
the wrapper's pooled 384-dimensional CLS descriptor and retains the existing
336-pixel PIL/ImageNet preprocessing and unit normalization. The public
model card specifies a BF16-to-FP32 round trip of the timm RoPE periods to
match the original Meta periods; the adapter applies that operation without
changing the rest of the model to BF16.

An initial native timm probe compares 20 real TUM RGB images using the same
336-pixel tensor for ONNX and Torch. It uses OpenCV resizing on both sides,
so it is a backend-equivalence diagnostic rather than the production crop
preprocessing test. Normalized descriptor cosine similarity has minimum
0.999999344 and mean 0.999999556; maximum absolute component difference is
0.000188779. Twenty warmed forwards average 3.90 ms. The broader transfer /
forward / return probe usually takes 6.3-10.2 ms, with three 81-89 ms spikes
while another GPU training workload is active. These isolated measurements
do not establish continuous-pipeline real-time performance.

The encoder tests cover CUDA selection, explicit ONNX CUDA failure, the
FP32 load, RoPE-period conversion, wrapper CLS output, unchanged input shape
and normalized descriptor validation. The original Host fields
`person_appearance_model` and `person_appearance_device` already carry the
checkpoint and CUDA choice through the Blueprint/provider boundary. The
research runs do not activate a robot Product or turn this local encoder
into a network model service.

The first complete CUDA run is retained as
`build/dino-return/results/return-gpu-sync/`. It uses the actual
`DinoEncoder` Transformers wrapper and PIL preprocessing, all 827 recorded
RGB-D pairs and the fixed detector/native-track cache. The comparison's
CLIP-only branch and video rendering are disabled; CLIP remains attached
for text retrieval. Verification is deliberately synchronous in this run,
preserving the earlier verdict/observation ordering for a backend comparison.

| DINO tracking measurement | Previous ONNX CPU, 8 threads | Transformers CUDA |
| --- | ---: | ---: |
| Accepted target observations | 290 | 290 |
| Image-verification calls | 36 | 36 |
| MATCH / UNKNOWN | 8 / 28 | 8 / 28 |
| Mean tracker update | 62.04 ms | 10.82 ms |
| P50 tracker update | 52.63 ms | 10.69 ms |
| P95 tracker update | 120.33 ms | 23.24 ms |
| Maximum tracker update | 360.70 ms | 46.20 ms |
| DINO crop forwards during updates | 846 | 846 |
| DINO encoding time during updates | 50.749 s | 8.784 s |

The observed mean tracker reduction is 82.6%. The CPU baseline's tracker
timer already excludes its separate CLIP-only branch, but the runs are
sequential under varying shared-server load; these are measured timings,
not an isolated causal estimate of every millisecond saved. Qwen's mean
request time also changes from 276.71 to 128.92 ms with the same evidence
images. The GPU experiments use an eight-token generation cap, while the
previous CPU experiment uses 512; every measured response emits only two
tokens including EOS. This is not counted as DINO acceleration. The final
source retains the existing 512-token default rather than changing the
general recorded-data model adapter's output contract.

All 827 source-frame/timestamp, accepted-target, native ID, raw/filtered XYZ
and tracker-event records exactly match the CPU run. The existing full
290-crop identity audit therefore continues to apply. The intended target's
detector IDs remain 1 -> 4 -> 17 -> 28. Raw and filtered adjacent-step maxima
remain 0.257 m and 0.102 m, with no adjacent step over 0.5 m. Missing intervals
remain missing. Input resolution, thresholds, the original anchor, two
additional trusted views, three candidate images and fresh-observation
geometry checks are unchanged. This deterministic replay establishes CUDA
backend correctness and tracking cost; it excludes real-time detector input
and retains blocking verification, so it is not an end-to-end real-time run.

The complete CPU/GPU source-order comparison is
`build/dino-return/results/gpu-sync-comparison/return-view.mp4`. Both sides
show all 827 pairs, native ID, observed XYZ, identity waiting state and actual
per-frame tracker timings. Its 28.859-second playback follows source time;
the timing records establish the processing-speed comparison.

### Source-paced asynchronous verification

The replay now has `--pace-source-time`, `--async-identity`, `--dino-only`
and `--no-render` switches. One worker executes a real Qwen request while
the foreground continues observing. The submitted candidate and reference
images are snapshots; the worker does not fetch mutable tracker state.
At each arrival, the foreground polls a completed request before loading
the current RGB-D observation. A request completing during that observation
is applied at the next arrival. Candidate ID/timestamp checks reject stale
results. MATCH queues identity evidence, and only a subsequent fresh
observation passing the geometry checks resumes target output. Historical
waiting frames are never filled in as accepted.

All measured paced runs process source pairs 0 through 826 exactly once,
in order, with no skipped or duplicated source frame. Their first-to-last
source span is 28.824169 seconds. Median source interval is 32.228589 ms
(about 31 Hz). The fixed-cache run isolates tracking and asynchronous
identity; the live runs execute YOLO-World on every actual RGB image and
estimate positions from the corresponding depth. Each live run's detector
and native-track records exactly match the fixed inputs on all 827 pairs.

| Measurement | Fixed detector cache | Live YOLO, warmed | Live YOLO, separate Qwen CUDA stream |
| --- | ---: | ---: | ---: |
| Accepted observations | 249 | 229 | 246 |
| Verification requests | 35 | 34 | 33 |
| MATCH / UNKNOWN | 8 / 27 | 8 / 26 | 12 / 21 |
| Mean foreground processing | 21.16 ms | 31.25 ms | 31.63 ms |
| P95 foreground processing | 32.40 ms | 48.70 ms | 45.83 ms |
| Maximum foreground processing | 137.25 ms | 160.20 ms | 100.36 ms |
| P95 arrival delay | 2.45 ms | 357.45 ms | 298.13 ms |
| Maximum arrival delay | 77.99 ms | 401.43 ms | 336.40 ms |
| Last frame completion delay | 23.00 ms | 191.88 ms | 310.58 ms |
| Wall time to finish the last frame | 28.847 s | 29.016 s | 29.135 s |

Foreground processing includes RGB/depth file decode, live detection where
enabled, native association, position extraction, tracking and request
submission/polling. It excludes model loading, pre-clock warm-up, rendering,
and the worker's separately recorded Qwen time. The observation-load stage
combines RGB/depth decode and YOLO detection; these measurements cannot
attribute that stage's cost separately to file decode or the detector.

The first live diagnostic records a 3.335-second first-frame cost and
roughly 3.3 seconds of arrival backlog because prompt/detector and CLIP
initialization happen after its clock starts. It is retained in
`return-gpu-paced-live/`. Subsequent live runs warm YOLO's `person` prompt,
CLIP and DINO on synthetic zero images before the source clock, without
reading future dataset observations. The warmed baseline records about
3.127 s, 125 ms and 57 ms respectively as separate setup costs. A live
deployment needs this preparation before advertising perception readiness.

The separate-stream experiment moves only recorded-data Qwen GPU work to
one owned CUDA stream, after waiting for model initialization on its
construction stream. Transfers and generation run there; phase timing
synchronizes that stream rather than the entire CUDA device. The single
physical identity worker, evidence images, Qwen model/pixel budget and
tracking thresholds remain unchanged. Both live comparison runs use the
same eight-token cap and produce two-token responses. The stream run
reduces observed spikes and P95 delay, while its final backlog is larger.
The server also hosts an unrelated training workload. These sequential
observations support retaining the smaller synchronization scope, but do
not prove a repeatable latency improvement under every GPU load.

The fixed-cache run keeps pace with the source with 23 ms final delay.
The complete live pipeline is close to source throughput but does not
establish stable 31 Hz service: 32 ms source intervals have very little
headroom, and 45.83 ms P95 processing still causes transient backlog. The
new code does not claim to eliminate that backlog. The next performance
measurement must split image decode from detection and test the selected
deployment's actual detector; it must not infer device performance from
this research-server YOLO measurement.

A final CPU I/O probe measures 20 warmed RGB-D PNG pairs spread across
the sequence. PIL depth decoding averages 3.351 ms; OpenCV unchanged-depth
decoding averages 3.363 ms. All 20 shapes and depth arrays match exactly,
with maximum pixel difference zero. RGB OpenCV decoding averages 6.168 ms.
`build/dino-return/results/decode-probe.json` retains the samples and timing
distributions. Changing depth readers would not reduce measured cost, so
the PIL path is retained and no extra full GPU run is justified by this
experiment. The roughly 9.5 ms warmed file-decode sum is a standalone CPU
measurement, not a per-frame subtraction proving YOLO's exact cost.

The synchronous and asynchronous accepted-frame totals are different
because real verification takes time and evidence can become obsolete.
Neither the 246 accepted observations nor the increase in MATCH count
means the model is more accurate: worker completion changes which fresh
three-frame request is subsequently submitted. Missing intervals remain
missing. The original anchor, up to two trusted RGB views, the last three
candidate images, CLIP text retrieval and fresh geometry checks are retained.

The full fixed-cache audit covers all 249 accepted crops, including frame
108, which was not accepted by the synchronous baseline. All 229 accepted
crops in the warmed live run are a subset of those accepted source frames,
with identical input boxes. The retained audit pages show the plaid-shirt
target and no accepted black-shirt distractor. These are observations on
this sequence, not a general guarantee against wrong-person tracking.

The final separate-stream audit checks all 246 accepted crops in three
`all-accepted-audit-page-*.jpg` pages. Forty accepted source frames were not
accepted by the fixed-cache paced run, so that older audit alone was
insufficient; the new frames were checked as well. All accepted crops show
the original plaid-shirt person, with no observed accepted distractor.
Detector IDs still change 1 -> 4 -> 17 -> 28: the system reacquires the same
person after a native tracker rebuild, rather than assigning one immutable
native ID across every absence. Reacquisitions are at source frames 199,
388 and 654; renewed confirmation during the final pose-change segment
starts at frame 739. The final raw adjacent-position maximum is 0.257 m,
filtered maximum 0.101 m, with no observed adjacent step over 0.5 m. These
statistics exclude missing intervals and measure visible-body estimates,
not a ground-truth person-center accuracy bound.

After restoring the 512-token source default, two real six-image requests
from the stream experiment are replayed with the same model. Attempt 8
retains MATCH and attempt 10 retains UNKNOWN; both generate two tokens.
`build/dino-return/results/qwen-512-check/result.json` records the actual
512 cap, image counts and responses. This checks those samples after the
source restoration; it does not relabel the earlier eight-token timing runs
as complete 512-token runs.

The live results are retained in `return-gpu-paced-live-final/` and
`return-gpu-paced-live-stream/`; each contains `metrics.json`,
`tracks_dino.json`, exact detector/native inputs and request evidence.
The complete fixed-cache/live video is
`gpu-paced-comparison/return-view.mp4`. A separate live baseline/stream
comparison is `gpu-stream-comparison/return-view.mp4`. Videos show green
only for currently accepted targets and display recorded processing and
completion delays, without interpolating missing XYZ.

Reproduce a live run on the existing research server environment:

```bash
research_root=/root/autodl-tmp/lingtu-dino-return-20261011
export PYTHONPATH="$research_root/source/src:$research_root/source"
export HF_HUB_OFFLINE=1
/root/autodl-tmp/lingtu-vlnav/vision/bin/python \
  "$research_root/source/tools/datasets/dino_follow.py" \
  --rgbd-sequence "$research_root/data/rgbd_dataset_freiburg3_walking_xyz" \
  --yolo-weights /root/autodl-tmp/lingtu-vlnav/yolov8s-worldv2.pt \
  --clip-model-path /root/autodl-tmp/lingtu-vlnav/weights/clip/ViT-B-32-state.pt \
  --dino-model /root/autodl-tmp/models/timm-dinov3-vits16-lvd1689m \
  --identity-model /root/autodl-tmp/models/qwen3-vl-2b \
  --target-description 'a person wearing a plaid shirt' \
  --max-frames 1000 --depth-method person --device cuda \
  --dino-only --pace-source-time --async-identity --no-render \
  --output "$research_root/results/return-gpu-paced-live"
```

For fixed detector input add
`--tracks "$research_root/data/tum-person-detector-cache.json"` and choose
a different output directory. The current source keeps the 512-token
default; the recorded GPU timing experiments above used eight and retain
their actual generation provenance. Every result is recorded-data evidence
using dataset camera poses and RGB-D visible-body positions. It neither
validates live robot SLAM alignment nor activates a Product, publishes
motion or deploys a GPU model API.

### Exact Host RGB evidence

Code inspection found a separate supported-path issue in the tracking Host.
Its raw-camera RGB input was throttled to 10 Hz while Perception could
publish detections from a 30-fps camera more frequently. A 0.1-second
timestamp tolerance permitted one RGB image to be reused for three distinct
detection timestamps. That established three detection observations, not
three separate image observations. The source-paced dataset replay uses the
actual RGB of each pair and is not affected by that Host mismatch.

The visual-servo wire now uses `PerceptionModule.observation_image`, which
is published from the same SynchronizedFrame immediately before its
detections. The RGB input is unthrottled and selection/tracking require the
same source timestamp. The regression sends one RGB image followed by
three increasing detection timestamps and verifies that it cannot produce
three-image identity evidence; only the corresponding new RGB observations
can complete that evidence. Blueprint wiring is checked as well.

This fixes evidence provenance within the existing Host ports; it adds no
transport, model service or runtime scheduler. The actual tracking Product's
detector is BPU, while these server runs use YOLO-World. Perception's worker
coalesces latest incoming frames, and its output rate depends on processing
throughput. RGB throttling never established a general 10 Hz Host contract.
Neither these local tests nor the server replay establishes a measured BPU
rate, motion readiness or hardware following accuracy.

Final affected checks pass without new or baseline failures on these surfaces:

- DINO backend suite: eight tests; encoder/source Ruff passes.
- Final recorded replay and Qwen adapter suites: 29 tests; their Ruff passes.
- `test_visual_servo_semantic_integration.py`,
  `test_product_host_blueprint.py` and
  `test_sim_semantic_pipeline_blueprint.py`: 46 tests.
- `test_follow_person_selection.py`,
  `test_perception_assembly_convergence.py` and
  `test_sim_tracking_person.py`: 45 tests.
- Ruff passes on the changed visual-servo, wire and Blueprint-test surfaces.
- Both complete comparison videos are H.264 / YUV420P, 1280 x 720,
  827 frames / 28.858569 seconds and pass full FFmpeg decoding.

An earlier broader check of the six directly affected encoder/replay/identity
suites passes 136 tests before the final warm-up/stream and Host-wire changes.
The final narrow checks above validate those later edits; overlapping suite
counts are not summed into one purported final full-repository result.

## Decode, detector and backlog attribution (2026-10-12)

The next pass splits the previously combined observation-load timer into
`rgb_decode_ms`, `depth_decode_ms`, `detector_ms` and `cache_restore_ms`.
YOLO time is absent on cached-input frames; restoring cached detections is
not reported as inference. Additional timers cover native association,
position extraction, person-tracker updates, interframe preparation and
source-clock idle sleep. Each frame retains arrival and phase-completion
wall times relative to the same source clock, so queue delay is distinct
from that phase's own work.

Both full comparison runs use the final Qwen adapter's 512-token cap,
the same local model, pixel budget, source images and evidence contract.
Responses actually emit two tokens. The instrumented baseline is
`build/dino-return/results/return-gpu-paced-live-stages-baseline/`:

| Baseline stage | Mean | P95 | Maximum |
| --- | ---: | ---: | ---: |
| RGB PNG decode | 6.42 ms | 7.23 ms | 63.73 ms |
| Depth PNG decode | 3.51 ms | 4.44 ms | 18.85 ms |
| Live YOLO detection and result extraction | 11.45 ms | 20.53 ms | 96.84 ms |
| Native tracker | 0.13 ms | 0.18 ms | 0.31 ms |
| RGB-D person position extraction | 1.01 ms | 1.80 ms | 18.15 ms |
| Person tracker, including DINO when used | 8.29 ms | 20.83 ms | 62.68 ms |
| Interframe preparation | 0.28 ms | 0.43 ms | 1.01 ms |

Mean foreground processing is 30.92 ms, P95 47.31 ms and maximum 165.80 ms.
The final frame completes 199.30 ms late. Across 827 frames, the main source
of backlog is work exceeding individual source intervals, especially
detector and tracking tails. Interframe preparation is measured and much
smaller; normal source-clock sleep is recorded separately, not counted as
work or blamed for backlog. These timings measure the shared research
server, not the selected robot's BPU detector.

The YOLO result parser previously fetched each box's coordinates,
confidence and class independently from CUDA. The retained change transfers
one complete Ultralytics `Boxes` result to CPU, then builds the same
`Detection2D` values in the same order. Single-frame and batch detection
share this parser. Tests and a direct installed-Ultralytics `Boxes` call
check values, order and one CPU transfer per nonempty result. Model,
resolution, precision, class prompt, NMS and thresholds are unchanged.

A separate same-frame depth/YOLO concurrency experiment is retained in
`return-gpu-paced-live-stages-final/` as a rejected experiment. It decodes
only the current pair's depth on one CPU worker while detecting its RGB,
without reading future frames. This reduces the final completion delay to
49.42 ms, but mean processing changes only from 30.92 to 30.72 ms. The
depth worker's measured wall duration grows from 3.51 to 9.51 ms; remaining
wait after YOLO is 2.11 ms, and maximum completion delay grows from 379.89
to 447.62 ms. Thus the final-tail improvement alone is insufficient to
retain the extra thread. The experiment is reverted; it does not become a
new configuration switch. Its 237 accepted crops were visually audited
without an observed accepted distractor, but are not used as the final
optimized run's evidence.

The final source retains serial depth decode, the split timers and the
single-transfer YOLO parser. All multiview references, three candidate
images, strict verdict checks and fresh-position acceptance remain intact.

The retained final run is `return-gpu-paced-live-stages-final2/`. It contains
all 827 source pairs in original order, and its detection/native-track
records exactly match the instrumented baseline on every pair. The rejected
parallel experiment keeps its original directory, so it is not overwritten
or relabeled as the final source.

| Measured quantity | Instrumented baseline | Final serial decode + one Boxes transfer |
| --- | ---: | ---: |
| Mean RGB decode | 6.42 ms | 6.36 ms |
| Mean depth decode | 3.51 ms | 3.48 ms |
| Mean YOLO detection and result extraction | 11.45 ms | 11.19 ms |
| Mean person-tracker update | 8.29 ms | 8.41 ms |
| Mean foreground processing | 30.92 ms | 30.66 ms |
| P95 foreground processing | 47.31 ms | 46.82 ms |
| Maximum foreground processing | 165.80 ms | 113.12 ms |
| Mean completion delay | 108.74 ms | 58.13 ms |
| P95 completion delay | 321.61 ms | 156.09 ms |
| Maximum completion delay | 379.89 ms | 333.13 ms |
| Last-frame completion delay | 199.30 ms | 34.31 ms |
| Wall time to finish source span 28.824169 s | 29.023 s | 28.858 s |
| Accepted observations | 245 | 233 |
| Verification requests | 33 | 34 |
| MATCH / UNKNOWN | 11 / 22 | 8 / 26 |

Average processing improves only 0.9%, not a large throughput gain. The
final run has much smaller measured queue delay, but both experiments run
sequentially on a shared GPU. The removed repeated result transfers are
direct structural evidence; the complete delay difference cannot be
assigned solely to the parser. YOLO's maximum duration still reaches
96.92 ms in the final run. Combined tails remain, including a 333 ms
completion delay, so finishing one frame late at the end does not establish
an every-frame 31 Hz deadline.

The largest final-run completion delay is at source frame 646, 22.591927 s.
It arrives at processing with 220.02 ms already accumulated, then spends
96.92 ms in YOLO; total foreground work is 113.12 ms and completion delay
reaches 333.13 ms. The five slowest YOLO frames overlap an in-flight Qwen
request. This is a useful contention hypothesis, not proof that Qwen alone
causes those delays: another queue increase at frame 642 includes a 44.34 ms
person-tracker update without a Qwen request in flight. The trace and largest
queue increases are retained in
`build/dino-return/results/stage-diagnosis.json`.

The reusable diagnostic `build/dino-return/phases.py` plots separate RGB-D,
YOLO, tracker and completion-delay traces from these measurements.
`gpu-stages-comparison/phases.png` and `phases.json` retain the plot and
backlogged-loop attribution. A full comparison video is retained in the
same directory. It shows actual stage work and accumulated delay rather
than using smooth source-time playback as evidence of processing speed.

Final raw and filtered adjacent-position maxima are 0.257 m and 0.098 m,
with no adjacent accepted-frame jump over 0.5 m. The detector IDs remain
1 -> 4 -> 17 -> 28. Asynchronous waiting and newly selected evidence
change the accepted-frame total; unchanged detection input does not imply
identical request timing or MATCH count. Missing frames are not filled in.

All 233 final accepted crops were visually reviewed on
`return-gpu-paced-live-stages-final2/all-accepted-audit-page-001.jpg` and
`all-accepted-audit-page-002.jpg`; every accepted crop belongs to the plaid-shirt
target, with no observed black-shirt distractor accepted. There are 32 accepted
source frames absent from the baseline's accepted set, so the final audit
reviews the complete final set rather than reusing the baseline audit.
Cross-ID recoveries occur at frames 229 (ID 4), 387 (ID 17) and 653 (ID 28),
after accepted-observation gaps of 120, 150 and 243 frames respectively.
Frame 748 recovers the same ID 28 and subsequent accepted observations include
the sitting segment. These gaps include waiting for evidence and are not
claimed to be exclusively time outside the camera view. Native detector IDs
change; the selected person's identity remains the same in accepted output.
Every verification package retains exactly three chronological candidate
images and one to three trusted reference images.

`gpu-stages-comparison/return-view.mp4` compares the instrumented baseline
and retained final run using all 827 original frames. Its overlays separate
RGB decode, depth decode, YOLO, tracker and accumulated delay, and leave
unconfirmed frames unconfirmed. The file is H.264 / YUV420P, 1280 x 720,
827 frames / 28.858569 s; full FFmpeg decoding passes.
Final runner/Qwen tests pass 30 tests, and detector/parser plus detector-failure
tests pass 26 tests. Ruff passes on their modified source and test surfaces.

### Actual-camera attempt and deferred RGB-D test

The user confirmed the machine dog is powered off and requested completing
the other tests first. Earlier read-only connections to NX
`192.168.66.65:5050`, SSH 22 and the historical `.95` SSH hop time out;
connection retries stop after that confirmation. No Product switch,
service restart or motion command is issued.

The existing D435i path supplies aligned RGB8 and uint16 millimeter-depth
arrays through `/dev/shm/lingtu_camera_color`, `depth` and `info`.
`build/dino-camera/capture_rgbd.py` prepares a bounded read-only recorder
using the existing SHM reader and image decoder. It pairs only equal
original `timestamp_ns` values and records raw depth, intrinsics and the
actual depth scale. It does not start a capture service. Camera-to-body
extrinsics and map pose are not asserted; a future projection from this
recording belongs to the camera optical frame until those values are
measured. The TUM scale of 1/5000 must not be substituted for D435i's 0.001.

The available Windows `USB2.0 FHD UVC WebCam` returns 444 frames over
29.838 seconds at 640 x 480, observed 14.847 Hz, with strictly increasing
reception timestamps. However, visual inspection shows an initial black
frame followed by a camera-disabled placeholder. No actual scene or person
is visible. `build/dino-camera/local-webcam-30s/manifest.json` explicitly
records `person_visible: false` and `suitable_for_person_reid: false`.
That capture verifies only the USB read/timestamp/recording path; it is not
sent to the model as a person-follow success case.

The actual D435i test remains deferred until the robot is powered on with
its camera Product already active. Camera SHM supplies raw arrays, so
the roughly 10 ms dataset PNG decoding measured above is not established
as a field-camera cost. Syntax, CLI loading and Ruff checks of the capture
scripts pass; these checks do not establish hardware RGB-D capture or
live GPU inference.

The supported raw-camera path has a separate structural duplication:
SHM `rgb8` previously became an RGB-tagged Host `Image`, after which
perception quality checking, RGB-D observation and visual follow each called
`to_bgr()` and could copy that same frame again. The camera decoder now
normalizes only `rgb8` to a correctly tagged BGR image during its existing
single copy. Its subsequent `to_bgr()` consumers return the same object;
RGB consumers still obtain the original channels through `to_rgb()`.
Device/SHM encoding, original timestamp, frame ID, depth, intrinsics and
other image formats are unchanged. This does not alter the TUM replay,
which already loads BGR through OpenCV, and no field latency improvement
is claimed without the deferred hardware measurement.

`tests/drivers/test_camera_dds_module.py` passes 12 tests, including actual
local SHM pairing, RGB/BGR roundtrip and a padded-row RGB sample. The sensor
format and camera JPEG consumer suites pass 29 tests. They can reveal
mislabelled channels, incorrect stride cropping or changed depth/timestamp
behavior; Ruff and diff checks pass for the two modified camera surfaces.

## YOLO / Qwen synchronization diagnosis (2026-10-12)

The next pass targets the 333.13 ms completion-delay spike while keeping
the robot test deferred, the same 827 source pairs, CLIP text retrieval,
multiview references and exactly three candidate images per identity request.

Installed Ultralytics 8.4.173 on the research server constructs three
`ops.Profile(device=self.device)` timers inside `BasePredictor.stream_inference`.
Each timer calls accelerator device synchronization on entry and exit:
six device-wide waits per detection batch. Local Ultralytics 8.4.21 has
the same mechanism. Qwen already uses its own CUDA stream and synchronizes
only that stream; nevertheless the detector's device-wide synchronization
can wait for Qwen work queued in the same process/context. Other training
processes are resource-competition background, not streams directly waited
on by this process's device synchronization.

This is an executable blocking mechanism, beyond the earlier correlation
between identity requests in flight and YOLO wall-time spikes. The existing
`detector_ms` remains useful end-to-end elapsed time, but includes these
waits and is not a measurement of YOLO kernels alone. The
[Ultralytics Profile implementation](https://docs.ultralytics.com/reference/utils/ops)
and [PyTorch device synchronization contract](https://docs.pytorch.org/docs/main/generated/torch.cuda.synchronize.html)
describe the corresponding behavior. The installed source, rather than
assuming the latest documentation matches every detail, determines this run.

The controlled probe is retained in
`build/dino-return/results/yolo-qwen-contention/result.json`. It fixes TUM
frame 646 and two real six-image identity packages, one MATCH and one UNKNOWN,
each containing three chronological candidate crops and three trusted target
references. Qwen keeps its 512-token cap, pixel budget, precision and model.
CPU-only processor pressure and real GPU generation are measured separately.

During four confirmed GPU-generation overlaps, the original detector path
executes 24 device-wide waits totaling 32.62 ms; the longest wait is 19.97 ms.
Probe-only current-stream timing reduces their sum to 1.50 ms. Detection boxes,
scores and classes are equal in all probe modes, and MATCH/UNKNOWN responses
are unchanged. These small samples establish the blocking mechanism; they
do not rank whole-video implementations reliably on the shared training GPU.
The probe's continuously repeated CPU preprocessing also produces large
spikes, but is a pressure test, not the production rate of single identity
requests. CUDA event spans can include scheduling and host-enqueue gaps and
are not described as sums of pure kernel execution times.

The scoped detector change reuses the already initialized Ultralytics
predictor. The first call follows normal `model.predict()` for setup and
warmup; subsequent numpy image/single-batch calls use the same
`setup_source`, `preprocess`, `inference` and `postprocess` under its lock
and `torch.inference_mode()`. Confidence and IoU values remain current,
and the existing whole-Boxes CPU transfer completes the returned result.
The detector interface consumes only boxes, not Ultralytics save/display
callbacks or `Results.speed`. No installed-library edits, global profiler
monkeypatch, replacement NMS, generalized scheduler or new dependencies are
introduced.

Two fresh full source-paced runs compare the frozen original detector
(`return-gpu-sync-baseline/`) and this change (`return-gpu-sync-final/`).
Both process all 827 source pairs in order. The complete detector/native
tracking JSON records match exactly. Model weights, precision, resolution,
thresholds, source pacing, Qwen pixel budget and 512-token cap are unchanged;
both make 34 identity requests with 8 MATCH and 26 UNKNOWN responses.
Every request retains three candidate images and up to three trusted target
references. No Qwen warmup or new CPU-thread/stream-priority setting is
combined with this modification.

| Full-run measurement | Fresh baseline | Scoped predictor stages |
| --- | ---: | ---: |
| YOLO call mean | 11.23 ms | 9.41 ms |
| YOLO call P95 | 19.93 ms | 15.94 ms |
| YOLO call maximum | 81.62 ms | 49.91 ms |
| Foreground mean | 30.60 ms | 29.17 ms |
| Foreground P95 | 46.77 ms | 45.20 ms |
| Foreground maximum | 98.33 ms | 123.05 ms |
| Completion delay mean | 66.81 ms | 39.39 ms |
| Completion delay P95 | 225.82 ms | 84.75 ms |
| Completion delay maximum | 317.25 ms | 169.44 ms |
| Final-frame completion delay | 47.98 ms | 28.59 ms |
| Wall time for source span 28.824169 s | 28.872 s | 28.853 s |
| Accepted observations | 230 | 231 |

This is the retained source change. Mean foreground cost decreases 4.7%;
YOLO mean decreases 16.2% and completion-delay P95 decreases 62.5% in
these measured runs. The historical final2 run's 333.13 ms delay is not
silently reused as this fresh baseline; the current baseline peaks at
317.25 ms. At frame 646 itself, fresh baseline completion delay is
297.40 ms and final delay is 153.18 ms. Shared training load still affects
both runs, so the observed reduction is not a guaranteed deadline or
a precise decomposition of all GPU resource competition. The unchanged
outputs and measured removal of cross-stream waits support keeping the
small patch. Foreground maximum actually rises to 123.05 ms; remaining
RGB-D decode and tracking spikes are not concealed by the better queue
statistics. The largest remaining completion delay is at frame 66,
2.274725 s, and still reaches 169.44 ms.

The 123.05 ms foreground maximum is source frame 0. Ordinary tracker-update
timing is near zero there because initial target selection and `lock_target`
run outside that timer. Their first-frame work is not independently timed,
so this maximum is not assigned wholly to decode, DINO or Qwen. Later frame
93 has 59.40 ms RGB decode and frame 536 has 42.78 ms RGB decode. These are
remaining recorded-data loading spikes, not demonstrated raw-camera costs.

The final detector, runner and Qwen adapter suites pass 53 tests; the
detector/failure-semantics subset also passes 29 tests. These overlapping
counts are not added. Ruff passes on the modified detector/tests and the
directly affected replay/adapter surfaces. No model or dependency upgrade
is part of this change.

`gpu-syncfix-comparison/phases.png`, `phases.json` and `queue-peaks.json`
retain the stage curves and largest-delay frame records.

Final accepted observations increase from 230 to 231, with 227 shared frames.
Final-only frames are 399, 684, 685 and 747; baseline-only frames are 98,
228 and 389. All 231 final accepted crops, including every final-only frame,
are visually audited in `return-gpu-sync-final/all-accepted-audit-page-001.jpg`
and `all-accepted-audit-page-002.jpg`. All belong to the plaid-shirt target;
the black-shirt distractor is not accepted. Accepted native IDs are still
1, 4, 17 and 28, so this is same-person reacquisition across rebuilt tracks,
not permanent numerical-ID continuity. Reconfirmations include frames 229
(ID 4), 390/399 (ID 17) and 652/684/747 (ID 28). An arriving MATCH at frame
77 still leaves the target unconfirmed until current-frame evidence suffices.
The 210 consecutive accepted-frame XYZ steps have P95 0.109 m and maximum
0.257 m (frames 777 to 778); no step exceeds 0.5 m. This remains position
continuity on dataset poses, not independent ground-truth person-centre error.

`gpu-syncfix-comparison/return-view.mp4` uses all 827 original source frames
with split-stage latency, identity status, ID and XYZ overlays on both sides.
It is H.264 / YUV420P, 1280 x 720, 28.858569 s and passes full FFmpeg
decoding. The plot/report and video retain their separate generation outputs.

## First-frame and CPU preprocessing attribution (2026-10-12)

This pass follows the retained 169.44 ms completion-delay maximum. Robot
testing remains deferred. The replay now records initial target selection
and target locking separately, with CLIP/DINO call-count and elapsed-time
deltas for each. These costs are included in complete foreground processing;
they are not attributed to ordinary tracker updates, which deliberately skip
the just-locked frame. The text-selection path previously called stateful
`PersonTracker.select_by_clip()` on a temporary selector: besides scoring
the candidate crops, it encoded and locked the winning crop before the
temporary selector was discarded. The final tracker subsequently locked
the same selected crop again. That discarded lock is a concrete redundant
operation; candidate scoring and the real target lock still have to run.

RGB and depth loading are also separated into file-byte reads and actual
image decoding. Reads remain serial and happen after the corresponding
source-frame arrival. No future frames are preloaded or dropped. Qwen's
existing image-decode and processor totals remain available; new nested
measurements expose base64 decode, PIL image decode, CPU processor work,
and the input transfer plus owned-stream wait. These nested times must not
be summed with their parent totals.

The installed image processor is `Qwen2VLImageProcessorFast` from
Transformers 4.57.6. Its torchvision path groups images by shape, resizes,
rescales/normalizes and builds pixel tensors on CPU before transfer to the
GPU. The research process defaults to 64 PyTorch intra-op and 64 inter-op
threads, while OpenCV already uses one thread. A CPU-only sweep reprocesses
all 34 actual multiview/three-frame requests from `return-gpu-sync-final`,
with unchanged pixel limits and image ordering. Every tensor is compared
directly against the one-thread result: `input_ids`, `attention_mask`,
`pixel_values` and `image_grid_thw` have identical shape, dtype and values
at every tested budget.

| CPU intra-op threads | Processor mean | P95 | Maximum |
| ---: | ---: | ---: | ---: |
| 1 | 7.38 ms | 11.24 ms | 12.01 ms |
| 2 | 5.82 ms | 7.97 ms | 8.01 ms |
| 4 | 5.05 ms | 6.41 ms | 9.58 ms |
| 8 | 4.56 ms | 5.70 ms | 9.75 ms |
| 16 | 4.67 ms | 6.58 ms | 10.05 ms |
| 32 | 9.10 ms | 51.23 ms | 62.36 ms |
| 64 | 19.02 ms | 86.28 ms | 87.34 ms |

`build/dino-return/results/qwen-processor-thread-sweep.json` retains this
controlled measurement. Eight threads are selected for a fresh full replay
through process-start environment variables `OMP_NUM_THREADS=8` and
`MKL_NUM_THREADS=8`, not a global thread mutation inside a Qwen worker.
This sets intra-op to 8 while inter-op remains 64. It is a measured research
server resource budget, not a hardware-default change for robot boards.
The standalone sweep proves a CPU preprocessing reduction with identical
inputs/outputs; complete replay delay and identity behavior require the
separate source-paced comparison.

The fresh instrumented runs are `return-gpu-cpu-baseline64/` and
`return-gpu-cpu-final8/`; the latter is the thread-only intermediate, not
the final source snapshot. Both use the same 827 source pairs and text
selection of the plaid-shirt person, with identical detection/native-track
records. Both make 34 real requests, yielding 8 MATCH and 26 UNKNOWN.

| Fresh full replay | Default 64 threads | Only thread budget changed to 8 |
| --- | ---: | ---: |
| Initial text selection | 26.57 ms | 21.00 ms |
| Initial actual target lock | 58.14 ms | 19.01 ms |
| First-frame processing | 116.02 ms | 70.80 ms |
| Foreground mean | 29.49 ms | 29.15 ms |
| Foreground P95 | 45.45 ms | 43.79 ms |
| Foreground maximum | 116.02 ms | 70.80 ms |
| Completion-delay P95 | 92.08 ms | 69.44 ms |
| Completion-delay maximum | 170.11 ms | 140.71 ms |
| Qwen CPU processor mean | 16.93 ms | 10.53 ms |
| Qwen CPU processor P95 | 35.28 ms | 14.36 ms |
| Accepted target observations | 229 | 227 |

The default-thread first selection uses three CLIP image calls: two
candidate scores plus the discarded selector lock, taking 26.05 ms.
The real lock uses one CLIP image call (6.86 ms) and one DINO call
(51.21 ms). At eight threads the real DINO call takes 12.45 ms. DINO's
CPU transform uses PIL/NumPy, so this first-call difference is not assigned
directly to PyTorch CPU normalization. Startup ordering and shared GPU
scheduling remain plausible contributors; no DINO transform or warmup
ordering is changed by the thread-only comparison.

Default-thread RGB file reads have mean/P95/maximum 0.145/0.184/2.414 ms;
RGB decode has 6.334/8.715/15.599 ms. Depth file reads have
0.070/0.093/2.217 ms and depth decode 3.526/4.621/14.632 ms. This
separates small file-access costs from actual PNG decoding. The thread-only
run's depth decode maximum increases to 24.02 ms, despite improved overall
delay; neither decoding nor CPU scheduling is claimed to be uniformly
faster. Qwen base64 decoding averages 0.321 ms and PIL decode 5.124 ms
in the default-thread run, while CPU preprocessing averages 16.93 ms and
input transfer plus owned-stream completion 3.53 ms.

The thread-only largest delay still occurs at source frame 66: 95.33 ms
late arrival followed by 45.38 ms foreground work. The first Qwen request
has 35.92 ms CPU preprocessing and 358.22 ms generation. This identifies
model cold-start work as the next startup-specific probe; it does not
justify weakening three-frame evidence or loading later source frames early.

A separate cold/hot probe uses only four generated gradient images: one
synthetic anchor plus three synthetic candidates. It exercises the same
identity prompt, Qwen processor, BF16/SDPA and 512-token cap without reading
dataset images or modifying a tracker. The first request takes 792.30 ms
(CPU processor 30.28 ms, generation 735.82 ms); the second identical request
takes 68.30 ms (processor 4.77 ms, generation 61.22 ms). Both return UNKNOWN
with two output tokens. `qwen-synthetic-warmup-probe.json` retains the probe.

The retained replay runs this synthetic request after Qwen loading and
before the source clock starts. Its response is never applied to a tracker.
`identity-warmup.json`, `identity_warmup` and
`warmup_timing_ms.identity_ms` record its synthetic-input marker, model,
response, token/image counts and separate stage timings. Only this startup
call is removed from the real evidence-call list. Startup inference has a
real cost; moving it before input acceptance improves model readiness and
does not establish a reduction in application cold-start time. Real frames
still incur all their decoding, inference and verification costs after
their recorded arrival. The final source change also removes the discarded
CLIP selection lock; the final tracker keeps its original CLIP/DINO lock.
Existing CLIP/DINO warmup order is retained, and no extra stream priority,
image transport API, JPEG roundtrip elimination or frame-dropping policy
is added.

The warmup/stateless intermediate is `return-gpu-cpu-final8-stateless/`.
Initial text selection becomes 15.82 ms with two CLIP image calls; actual
target locking is 17.57 ms (CLIP 6.95 ms, DINO 10.51 ms). Its recorded
synthetic startup request costs 402.14 ms. The first real identity request
now completes in 146.79 ms with generation 111.77 ms and resolves at source
frame 70 instead of the thread-only frame 76. Frame 66 completion delay
falls to 83.34 ms, so startup warmup addresses the earlier cold-start peak.

This intermediate does not improve the complete worst case: completion
delay P95 is 78.26 ms and maximum 184.96 ms at frame 644. Foreground mean
is 28.95 ms and maximum 70.05 ms. Before Qwen is in flight, frames 641,
642 and 643 already accumulate delay with tracker work of 45.15, 19.92
and 23.53 ms. Frame 644 starts 117.57 ms late and needs another 67.40 ms;
184.96 ms is accumulated completion delay, not that frame's own processing.
There is no repeated DINO encoding of the same crop: frames 641/644 have
three distinct native IDs 22, 27 and 28, while 642/643 have two. Three
DINO descriptors plus the required two-candidate CLIP text ranking cost
42.24 ms in frame 644. The current-frame descriptor cache already eliminates
repeat requests for the same `(id, bbox)` and is cleared for every new frame.

All 230 accepted intermediate crops were visually audited as the plaid-shirt
person in two audit pages; native IDs remain 1, 4, 17 and 28. All requests
retain three candidate images. There are 35 real requests (9 MATCH and
26 UNKNOWN); earlier completion changes proposal timing, so request count
and accepted-frame count are not assumed to stay identical. Raw consecutive
accepted-frame XYZ steps have P95 0.139 m and maximum 0.257 m. The intermediate
`gpu-cpu-final-comparison/return-view.mp4` retains all 827 frames and passes
complete H.264 decoding. Its larger queue maximum is retained in the ledger.

The next bounded optimization probes current-frame batching without changing
the encoder, precision, crop transform, candidate set or identity algorithm.
On the three actual crops in frame 641, sequential descriptor computation
has median 27.22 ms and a single batch median 21.70 ms; frame 644 changes
from 27.93 to 14.76 ms. Normalized feature maximum absolute differences
are 5.44e-7 and 2.58e-6, per-crop cosine similarity is at least 0.99999994,
and reference-score order remains `[2, 0, 1]` in both modes. The controlled
probe is `dino-batch-probe.json`. These small floating-point differences
are measured rather than described as bit-identical. A complete replay
and identity audit are still required for any batch integration.

## Retained first-frame / batch replay (2026-10-12)

The final source snapshot is `return-gpu-cpu-final8-batched/`, compared with
the fresh instrumented `return-gpu-cpu-baseline64/`. The 185 ms intermediate
remains retained above; it is not replaced by a selective rerun of that
source. The final adds current-frame descriptor batching to the measured
eight-thread budget, stateless selection and separately recorded synthetic
model readiness.

`DinoEncoder.encode_batch()` keeps the original BGR/PIL/NumPy transform,
336-pixel input, FP32 Transformers model and row-wise normalization. It
executes one forward pass and one descriptor transfer for the current-frame
crop list. Fixed-batch-one ONNX models still execute one crop at a time.
`PersonTracker` batches only when timestamp/geometry filtering leaves at
least two candidates, none has the current target ID, and no verified
identity is waiting to be applied. Current-ID and authorized-ID paths retain
their sequential early returns. The same per-frame feature cache feeds the
unchanged candidate order, text ranking, thresholds, retries, multiview
references and chronological three-frame confirmation. There is no
cross-frame descriptor reuse or dropped-candidate shortcut. The research
runner warms a three-crop synthetic batch separately, recording 27.66 ms;
its synthetic Qwen readiness request costs 422.97 ms. These startup costs
are excluded from source-paced input latency and explicitly retained in
the metrics, rather than claimed to disappear.

| Same 827 source pairs | Fresh baseline, 64 threads | Retained final, 8 threads + batch |
| --- | ---: | ---: |
| Initial CLIP text selection | 26.57 ms / 3 image calls | 13.85 ms / 2 image calls |
| Actual target lock | 58.14 ms | 20.14 ms |
| First-frame foreground processing | 116.02 ms | 56.92 ms |
| Foreground mean | 29.49 ms | 28.93 ms |
| Foreground P95 | 45.45 ms | 42.00 ms |
| Foreground maximum | 116.02 ms | 62.03 ms |
| Completion-delay P95 | 92.08 ms | 64.77 ms |
| Completion-delay maximum | 170.11 ms | 118.57 ms |
| Final completion delay | 40.78 ms | 38.35 ms |
| Qwen CPU preprocessing mean / P95 | 16.93 / 35.28 ms | 9.52 / 13.24 ms |
| Actual identity requests | 34: 8 MATCH, 26 UNKNOWN | 35: 9 MATCH, 26 UNKNOWN |
| Accepted observations | 229 | 231 |

Selection saves the discarded temporary lock's CLIP computation; the real
lock still encodes one CLIP crop (6.50 ms) and one DINO crop (13.50 ms).
Within tracker updates there are 21 DINO batch API calls encoding 45 crops
in 290.84 ms, plus 575 scalar calls/crops in 6261.89 ms; startup warmup
and the separately measured initial lock are excluded. Timing includes preprocessing,
inference and descriptor transfer. The bounded crop probe above establishes
the batching mechanism's benefit, while shared-server scheduling still
affects the complete replay comparison. The foreground mean changes by
only 1.9%; the queue improvement is not described as a 30% reduction in
all inference work.

The final's largest completion delay occurs at frame 63. Frames 0--826
are all processed once in source order; all 827 detection/native-track
records exactly match the baseline. The source span is 28.824169 seconds;
processing reaches the last frame in 28.862520 seconds. Final scheduling
delay is 0.056 ms and completion delay is 38.351 ms. This run does not
accumulate an increasing queue over the clip, but its 42.00 ms foreground
P95 still exceeds the source's 32.23 ms median interval. Stable per-frame
31 Hz deadlines remain unproved on the shared RTX 5090 and unmeasured on
robot hardware.

CPU decode attribution is retained, including unfavourable tails. Final
RGB file reads average 0.145 ms and actual PNG decoding 6.213 ms; depth
reads average 0.076 ms and decode 3.512 ms. RGB decode P95 changes from
8.715 to 7.888 ms, while depth decode P95 changes from 4.621 to 4.480 ms.
Depth decode maximum grows from 14.632 to 24.477 ms. File access is not
the main observed cost; no speculative disk cache or prefetch pipeline
is added. Final real Qwen requests average 0.298 ms for base64 decode,
4.483 ms for PIL decode, 9.516 ms for processor CPU work, 3.173 ms for
input transfer/wait and 131.735 ms for generation. Its first real
generation is 123.09 ms after startup readiness, compared with the fresh
baseline's 340.33 ms cold generation.

The three major changed-native-ID returns are frame 230 / ID 4, frame
389 / ID 17 and frame 651 / ID 28, following 95, 152 and 241 missing
output frames respectively. The fresh baseline's first accepted frames
for these IDs are 229, 389 and 652: one final return is a frame later,
one is unchanged and one is a frame earlier. Faster foreground processing
does not establish uniformly faster reacquisition, and asynchronous request
completion can change proposal timing. A missing output interval can include absence,
unusable depth, appearance rejection or verification wait; it is not an
annotation of the person's physical absence. The TUM sequence has no
official per-person identity/position ground truth. Native IDs are tracker
identifiers, not a measure of persistent human identity.

There are 231 valid accepted XYZ observations and 210 consecutive-frame
step samples. Raw XYZ steps have P95 0.132 m and maximum 0.257 m; EMA
steps have P95 0.068 m and maximum 0.110 m. Neither has a step over
0.5 m. Twenty transitions across observation gaps are excluded from these
step statistics. This establishes measured adjacent-frame stability, not
absolute 3D accuracy or zero displacement after an occlusion.

Reproduction uses `build/dino-follow/run-gpu-cpu-final8-batched.sh`, with
process-start `OMP_NUM_THREADS=8` and `MKL_NUM_THREADS=8`, the same local
model/data paths, live YOLO, source pacing and asynchronous identity
verification. The directly affected DINO/person/runner/Qwen suites pass
90 tests. The separately run generic
`tests/decision/test_person_tracker.py` suite passes another 27 tests;
these are distinct files, for 117 passing tests in total. Ruff passes on
the eight touched source/test files, and the affected documentation passes
`git diff --check`.

All 231 accepted target crops were manually reviewed in
`return-gpu-cpu-final8-batched/all-accepted-audit-page-001.jpg` and
`all-accepted-audit-page-002.jpg`. Every accepted crop is the plaid-shirt
target; no black-shirt distractor is accepted in this clip. Output still
contains 596 missing/unconfirmed frames. Uncertain identity, unusable
observations and pending verification remain visible as gaps, rather than
being filled with another person or interpolated target observations.

Final media and raw evidence:

- [Complete baseline/final video](../../build/dino-return/results/gpu-cpu-batched-final-comparison/return-view.mp4):
  H.264/yuv420p, 1280x720, 827 frames, 28.858569 seconds; complete FFmpeg
  decode succeeds. It displays source order, current native ID, raw XYZ,
  first selection/lock, per-frame inference and completion delay.
- [First-frame cost split](../../build/dino-return/results/gpu-cpu-batched-final-comparison/first-frame.png)
  and [complete phase/queue plot](../../build/dino-return/results/gpu-cpu-batched-final-comparison/phases.png).
- [Accepted-target audit](../../build/dino-return/results/return-gpu-cpu-final8-batched/all-accepted-audit.jpg),
  [final metrics](../../build/dino-return/results/return-gpu-cpu-final8-batched/metrics.json)
  and [raw tracking records](../../build/dino-return/results/return-gpu-cpu-final8-batched/tracks_dino.json).

For a fresh reproduction, choose a new output directory rather than
overwriting the retained baseline, intermediate or final evidence.
These results are research-server recorded RGB-D evidence and local code
contracts. They do not demonstrate Habitat motion, robot-board timing,
absolute person position accuracy or field following.

## Go2/NX no-motion camera validation (2026-10-12)

The actual target is the Go2 host at `192.168.66.65`, NVIDIA Orin NX 16GB,
running release `v2.3.0-go2.20261010.93`. ProductControl started the supported
`real / map / camera` variant. No following target or motion goal was submitted.
This is camera and loader evidence, not field following evidence.

Two recordings read the already-running camera SHM and pair RGB and depth by
exact source timestamp. `follow-001` contains 900 pairs over 29.990 seconds
(29.976 Hz); `follow-002` contains 1800 pairs over 60.009 seconds. Both have
zero timestamp skew and zero rejected reads. RGB is 640x480; raw depth is
uint16 with `depth_scale=0.001`. Recorded intrinsics are fx=604.1031,
fy=603.5495, cx=327.9823, cy=245.6598. Sixteen sampled frames from the first
recording have nonzero depth in approximately 89% of pixels. Nonzero depth
alone does not establish depth accuracy.

The first unsuccessful probe ran without permission to read the
`root:lingtu 0640` SHM files. The capture helper hid `ShmUnavailable` behind
`no_timestamp_matched_rgbd_frames`. Running the same helper with the existing
sudo permission immediately captured 91 pairs in three seconds. The helper
now retains per-reader rejection counts and original exception messages in
its zero-frame error. No publisher change or timestamp tolerance was needed.

A separate live SHM benchmark on the NX decoded 180 exact RGB-D pairs at
29.979 Hz. The measured work excludes waiting for the next frame, inference,
and disk codecs:

| Stage | Mean ms | P95 ms | Maximum ms |
| --- | ---: | ---: | ---: |
| RGB SHM read | 0.623 | 0.839 | 0.899 |
| Depth SHM read | 0.440 | 0.593 | 0.626 |
| RGB conversion | 2.585 | 2.759 | 3.239 |
| Raw depth decode | 0.187 | 0.528 | 0.634 |
| Paired read and conversion | 3.835 | 4.286 | 4.929 |

The replay runner's existing `--rgbd-sequence` now accepts the capture's
`frames.json`, `camera.json`, and `manifest.json` directly. It preserves the
recorded pairing/timestamps and uses actual camera intrinsics and depth scale.
With no map pose, output is explicitly camera-local forward/left/up
`[optical_z, -optical_x, -optical_y]`; it is not a map or robot-body pose.
The loader/projection/runner suite passes 30 tests, with Ruff and the affected
diff check passing. The TUM path and identity-verification thresholds remain
unchanged.

Sampled frames from both initial recordings show an empty office view, so no
person ID, reacquisition, or person-position result is claimed for those two
recordings. The user initially postponed the dynamic recording, then returned
for the `follow-003` test below. The NX Host environment does not yet
contain the DINO/CLIP inference deployment, and camera-to-body calibration is
unverified. These remain prerequisites for a supervised following test.

Evidence is retained under `build/dino-camera/field-20261012/`: the first
complete RGB-D capture, both capture manifests and contact sheets,
`capture-inspection.json`, `live-decode-benchmark.json`, and
`follow-001/loader-smoke.json`. The original recordings also remain on the NX
under `/home/unitree/lingtu-follow-20261012/`. The synchronized RGB/depth
preview is `rgbd-preview.mp4`; it has no inferred tracking labels.

### Actual person recording: follow-003

The user subsequently stood in front of the camera. A fresh SHM observation
confirmed the person before recording; the HTTP snapshot endpoint was stale
and was not used as evidence of the current scene. The 45-second recording
contains 1341 exact-timestamp RGB-D pairs, covering 44.964 source seconds at
29.801 pairs/s. RGB/depth timestamp skew and rejected reads are zero. Eight
source sequence increments were skipped during capture; the median/P95/maximum
source intervals are 33.357/33.583/66.978 ms. No following or motion command
was sent. This is a recording from the physical camera, followed by a research
GPU replay; it is not an on-robot inference or motion result.

The intended target is described as `a person wearing a black short-sleeved
shirt and blue jeans`. Frame 58 (1.968 seconds) is a clear full-body view chosen
as the earliest selection opportunity; the replay still processes all source
frames from frame 0. Source-image inspection shows a fully occluded interval
around 22.95--23.95 seconds, a head-only return around 24.45 seconds, and a
more complete body return around 25.45 seconds. After the final exit around
33 seconds, the person does not return. These are image observations, not
inferred tracker-gap labels. Only one person appears, so this recording cannot
establish identity discrimination between two similar people.

A same-entry-point YOLO probe detects the person in frame 58 with score 0.915,
but also incorrectly detects the cat at the lower left as `person` with score
0.452. Frame 1048 has no detections. The full replay must therefore expose
whether text selection and identity verification reject the cat; a detector
box alone is not an accepted target. Thresholds, multiple reference views, and
consecutive three-frame identity evidence are unchanged from the prior replay.

The complete capture, source timing, probe evidence and RGB-D preview are
retained under `build/dino-camera/field-20261012/`. The preview
`follow-003-source-rgbd.mp4` contains all 1341 recorded pairs, uses H.264/yuv420p
at 1280x540, and passes a complete FFmpeg decode. It contains no tracking labels.
The position frame remains camera-local forward/left/up: no map pose was
recorded, and camera-to-body calibration remains unverified.

The complete live-YOLO/DINO/CLIP/Qwen replay on the research RTX 5090 processes
all 1341 recorded pairs once, in source order. Text/image selection chooses the
person at frame 58, native ID 1: selection costs 14.30 ms and the actual lock
costs 16.56 ms. There are 617 accepted observations (461 under ID 1, 156 under
ID 9). All 617 accepted crops were reviewed across six retained audit pages:
every crop belongs to the intended person, and none belongs to the cat. This
is a manual review of one clip, not an official identity benchmark.

The missing-output interval from frames 635--772 includes partial visibility,
complete physical occlusion, and reacquisition verification; it is not all
physical absence. Frame 773 (26.051 seconds) is the first accepted observation
under ID 9, approximately 0.601 seconds after the more complete body view at
frame 755. The target identity is recovered, while the native track ID changes.
The last accepted observation is frame 939 (31.588 seconds); frames 940--1340
all contain no accepted target. Later cat proposals under ID 11 trigger two
`UNKNOWN` verdicts and are not accepted.

Fifteen actual Qwen requests produce eight `MATCH` and seven `UNKNOWN` verdicts.
Every request contains three chronological candidate crops, plus one to three
reference views; the first request has only the initial anchor because further
reference views have not yet been accumulated. Mean request time is 146.24 ms.
The foreground loop remains asynchronous; a model verdict is applied only when
the proposal is still current. The model's `MATCH` alone does not guarantee an
immediate accepted position: the geometry gate can still reject that frame.

| Research GPU replay stage | Mean ms | P95 ms | Maximum ms |
| --- | ---: | ---: | ---: |
| RGB PNG read/decode | 6.221 | 8.238 | 26.467 |
| Depth PNG read/decode | 3.787 | 4.970 | 6.312 |
| YOLO detection | 8.492 | 14.020 | 204.498 |
| Person tracker update | 5.641 | 12.321 | 27.089 |
| Full foreground processing | 24.651 | 35.979 | 227.220 |
| Source-to-completion delay | 27.307 | 37.690 | 227.279 |

The overall foreground average includes frames with no person. Accepted-target
frames average 29.13 ms, and their P95 is 36.39 ms. The recorded source span is
44.964 seconds and the loop reaches the last result at 44.984 seconds; final
completion delay is 19.98 ms. This run does not end with a growing queue, but
its P95 exceeds the approximately 33.36 ms frame interval and has one 227 ms
tail. The largest tail is frame 434, dominated by a 204.50 ms YOLO call while
the identity verifier is idle. This evidence does not establish Qwen contention
as the cause of that tail. It also excludes capture/upload latency, model startup,
and video rendering, and does not prove an online 30 Hz field following loop.

There are 54 `position_rejected` frames. Among accepted observations only,
adjacent raw XYZ steps have P95 0.242 m and maximum 0.300 m; EMA steps have
P95 0.100 m and maximum 0.381 m. Sixty transitions across observation gaps are
excluded from those adjacent-step samples. These are measurements after
acceptance filtering; they do not demonstrate that rejected depth observations
were stable or that absolute person position is accurate.

Recomputing all 54 rejected observations with the existing production projection
and gate finds two measured problems. Thirty-two have their largest displacement
on the camera-local up axis: at frame 64, depth changes only 0.073 m, but the
depth-layer median pixel moves 40 rows, creating a 0.322 m height change. The
other 22 are dominated by forward-depth changes; the largest measured depth
switch is 1.516 m. The current gate rejects these changes, so the accepted-step
maximum near 0.300 m is a filtering consequence rather than evidence that the
raw observations never jump.

There is also visible occluder contamination during reacquisition. At frame 755,
the selected depth-layer pixels and sample `(457,174,2.275 m)` lie on the white
chair back inside the person's box, while the person stands behind it. That
candidate surface is not a valid person location and is not accepted as the
target. Matching image identity and selecting the correct person depth surface
are separate requirements. The geometric formula, depth units, and axis
conversion agree exactly with the stored positions at frames 58, 830, and 932;
the remaining issue is the selected surface and representative point. A later
repair should stabilize the torso anchor and depth-layer selection, then replay
the same input, rather than simply widening the current motion gate.

Raw replay evidence lives in
`build/dino-camera/field-20261012/results/follow-003/`: `metrics.json`,
`tracks_dino.json`, `detector_cache.json`, `field-evidence-summary.json`,
`all-accepted-audit-page-001..006.jpg`, and `identity-audit-page-001..002.jpg`,
with the original model evidence retained in `identity-attempts/`.
The final field navigation status remains `IDLE`, authority `NONE`, motion
permission `HELD`, observation `QUIET`, and admission reason
`navigation_session_inactive`. No following or body-motion test is claimed.

The complete annotated RGB-D video is
[follow-003-tracking.mp4](../../build/dino-camera/field-20261012/follow-003-tracking/follow-003-tracking.mp4).
It contains 1341 frames in source order, H.264/yuv420p at 1280x720, and passes
complete FFmpeg decoding. The encoded duration is 44.998 seconds, including the
final frame interval. Green boxes indicate accepted targets only; gray boxes
are detector/tracker candidates. Missing, pending, rejected, and `UNKNOWN`
observations are not interpolated. Raw and EMA coordinates are displayed only
for accepted targets, explicitly in camera-local forward/left/up coordinates.
The geometry audit and rejection overlays are retained under
`build/dino-camera/field-20261012/geometry-audit/`.

### Local corrections found during field preparation

Two source corrections have passed narrow local tests but are not deployed to
the NX release used above:

- `src/gateway/routes/camera.py` now requests the relay's current one-shot JPEG
  before falling back to the cached gateway image. Previously the first
  nonempty cached snapshot short-circuited subsequent requests indefinitely
  when there was no WebSocket consumer. Two snapshot-route regression tests
  pass. The recorded SHM stream was current and was not affected by this bug.
- `config/runtime/products/tracking.yaml` now declares `tick_hz: 100.0`, so an
  explicit SCAN selection satisfies the native compiler's frequency contract.
  The default backend remains CMU. Both backend selections compile in the new
  regression test, which also preserves the required map and critical modules.
  The declaration also changes the default CMU loop to 100 Hz; its NX CPU cost
  has not been measured. It is not described as a behavior-preserving cadence
  change or as completed field deployment.

Ruff and the affected diff checks pass. ProductControl remains on the existing
`real / map / camera` run throughout the recording.

### Online integration and onboard preparation, 2026-10-12

The operator confirmed that this Go2 EDU's D435i is fixed to the factory
bracket. Its pose must not be taken from the project's printed MID360/camera
cradle or from Thunder calibration. No numerical factory-bracket extrinsics
have been established yet.

The online Host now accepts a `vision_service_url` Endpoint through stack
composition. The detector sends RGB and the source timestamp; depth, intrinsics,
and the frame's `map_from_camera` remain on the Host. DINO descriptors have a
separate `dino_feature` field through projection, runtime messages, and person
tracking. Descriptors returned with detections skip repeated crop encoding.
An explicit local `person_appearance_model` now takes precedence over the
shared vision Endpoint, so onboard DINO can coexist with remote CLIP and VLM.
An explicit local CLIP path likewise takes precedence for that encoder. Three
local setup tests cover these choices; they do not prove model availability.
`native_track_id` may change after occlusion; `target_identity_id` identifies
the locked person across accepted reacquisition. These identifiers distinguish
the evidence being tracked; a stable identifier alone does not prove identity.
Multiview references, consecutive three-frame evidence, VLM identity checks,
and the position gate remain in place.

The shared inference service is implemented in
`tools/inference/follow_service.py`. A real server start failed during model
loading because two existing Isaac jobs occupied about 27.7 GB of GPU memory.
The failed process exited and was not treated as a ready service. No training
job was stopped. Local HTTP contract tests are not online hardware evidence.

The NX was inspected directly: NVIDIA Orin NX, 16 GB unified memory, about
13 GB available at inspection, CUDA 11.4, TensorRT 8.5.2. System Python 3.8 has
TensorRT bindings; the Host uses Python 3.10 and has no TensorRT, Torch,
Ultralytics, Transformers, OpenCLIP, or ONNX Runtime installed. A board-only
continuous perception loop therefore requires model deployment and a compatible
runtime. Two fixed TensorRT C ABI adapters now use the existing CUDA/TensorRT
libraries from Host Python without installing Torch or Python TensorRT:
`src/perception/encoding/dino_trt.cpp` and
`src/perception/detection/yolo_trt.cpp`. Engine and sibling shared-library paths
are explicit model configuration. The detector's vocabulary is fixed to
`person`; only the tracking Product now requests that vocabulary. Other
Products keep their configured object vocabulary. These adapters alone do
not establish model accuracy.

DINO's original ONNX could not import `LayerNormalization` on TensorRT 8.5.2.
Decomposing 25 normalization nodes made the build succeed; a CPU ONNX Runtime
random-input comparison differed by at most 3.10e-6. The resulting FP16 engine
is 44,602,303 bytes. A random-input `trtexec` measurement averaged 13.377 ms,
P95 13.670 ms, excluding crop preprocessing and the remaining tracking loop.
However, its first real frame-58 person crop returned 384 NaN descriptor values.
The real-crop check therefore failed, and the random-input speed is not usable
tracking evidence. FP32 subsequently built and returned finite descriptors,
but the actual crop comparison failed too: corresponding CPU/TRT descriptor
cosines ranged from -0.033 to 0.114, and the reference-to-cat similarity rose
from about 0.183 on CPU to about 0.898 in TensorRT. The FP32 features therefore
cannot be used for identity association either. CPU comparison of the original
and decomposed ONNX on the same real crop agrees (cosine 1.0, maximum absolute
difference 1.79e-7). The direct real-input `trtexec` comparison now agrees with
the C ABI descriptor (cosine 1.0; maximum absolute difference about 5.01e-6),
so C ABI output selection does not explain the discrepancy with CPU inference.
The first divergent network operation still needs to be located; the retained
checkpoint ONNX and CPU intermediate outputs are ready for that comparison.
This is an identity-feature failure, not a reason to change the association
threshold.

The fixed-person YOLO-World ONNX returns plausible frame-58 boxes and scores on
CPU ONNX Runtime. The NX FP16 engine adds high-confidence false boxes, and the
FP32 engine collapses the maximum person score to approximately 0.000141 on
the exact same 640x640 input. Official `trtexec` and the C ABI reproduce the
FP32 collapse, excluding the adapter as the source of this discrepancy. Neither
engine is usable for following. The official YOLO-World `set_classes()` stores
the text embedding but does not fuse its contrastive head into a fixed
convolution. LingTu's fixed-person conversion preserves the actual
BNContrastiveHead normalization and replaces the fixed-text contraction with
a 1x1 convolution. On the same 640x640 frame-58 input, original/fixed PyTorch
score maximum difference is 5.07e-6 and fixed/ONNX Runtime score maximum
difference is 5.25e-6. The corresponding maximum box-coordinate differences
over raw proposals are 0.00223 and 0.0232 pixels. NMS preserves the three
candidates and their ordering, including the original false-positive
proposals; export agreement does not establish detector recognition accuracy.
The new fixed-person ONNX still needs a fresh NX TensorRT comparison. This is
a LingTu conversion, not an official YOLO-World fusion API. Detection and
identity thresholds have not been raised to conceal engine errors.

The TensorRT detector adapter now retains low-score proposals for ByteTrack's
second association stage. Previously its high-score prefilter made that stage
unreachable. Two-frame local tests show that a 0.2-score observation can retain
an existing 0.9-score track, while a standalone 0.2 proposal creates no new
track. The configured high-score/new-track threshold remains 0.3. This fixes
the adapter contract without establishing that the current engines are usable.

No complete onboard FPS or usable online model deployment is claimed. CLIP
image/text and VLM board execution remain separate unfinished work. Low-frequency
VLM calls must not be described as fully offline operation or silently removed
from reacquisition verification.

A fresh no-motion joint recording succeeded after selecting the Host's Python
3.10: 91 RGB-D frames over three seconds, exact RGB/depth timestamp pairing,
zero rejected SHM reads, and 33 registered-cloud / 33 odometry samples with
zero DDS recorder drops. Evidence is on the NX at
`/tmp/lingtu-d435i-mid360-static-pair2`. The native camera video recorder failed
because FFmpeg is absent; this joint recording reads the existing SHM stream
and saves PNGs instead. Neither result establishes camera-to-body extrinsics.

Root wiring tests pass: 47 tests covering Host composition and VisualServo,
including descriptor propagation and persistent-identity status. The remote
perception/message tests pass separately (48), and the person tracker/DINO
tests pass separately (76). These suites are local contract evidence. The NX
release has not been updated with this new inference wiring; online tracking
and supervised motion remain unproven.

### Local consumer repair, 2026-10-13

The operator confirmed that the robot is off; this work uses only local code
and an in-process HTTP test service.

The real `PersonTracker` consumer exposed a remote CLIP mismatch that the
previous isolated client test missed. `encode_text([description])` requires an
`(N, D)` matrix. The adapter instead serialized the list as a string and
returned a one-dimensional vector. The tracker caught the resulting similarity
error, set both candidate scores to zero and selected no target. The failing
regression demonstrated both the wrong request and the rejected selection.

`RemoteCLIPEncoder.encode_text` now preserves text batches and order, verifies
the returned count, and returns a matrix. An empty batch sends no request.
The consumer regression uses two distinct controlled image features and the
real `PersonTracker.select_by_clip` through HTTP; the intended candidate is
selected and locked. A separate `verify_candidate -> RemoteVisionClient ->
/chat` test receives all six JPEGs in order: the original anchor, two trusted
views and three chronological candidate frames. This proves image transport
and verdict parsing, not real model recognition or physical following.

`tests/perception/test_remote.py`: 6 passed after repair; the two failing
consumer/request cases were observed before repair. Ruff passes on the touched
adapter and test. No identity thresholds, motion gates, robot configuration,
deployed release or dependencies were changed by this repair.

The stateless torso-anchor production sampler was also checked on the retained
617 accepted RGB-D frames (556 consecutive pairs). Its raw 3-D step P95 is
0.12229 m and absolute camera-local vertical step P95 is 0.01987 m. These
metrics concern accepted observations from one recording, not absolute position
accuracy. Frame 755 still samples the foreground chair at about 2.40 m; this
occluder problem remains open.

### Operator-visible follow evidence, 2026-10-13

The visual-follow status now separates `reason` (perception, target selection,
identity evidence or rejected observations) from `navigation_reason` (the
existing native goal result). It exposes missing aligned images, consecutive
reacquisition frames, identity verification without an available service,
position jumps and repeated observation timestamps. The last accepted position
and persistent identity remain unchanged when an observation is rejected;
that retained position is not a new measurement. A valid subsequent observation
clears its rejection reason.

A new intent clears a previous navigation failure only when no owned native
tasks remain. Pending native cancellation is preserved. The follow/stop skill
responses now acknowledge requests rather than claiming motion or a completed
physical stop. Multi-view references, three-frame evidence and identity/motion
thresholds are unchanged.

The operator panel presents visual tracking, follow navigation and robot
motion separately. A locked target does not imply that the robot is moving.
When the target is not visible, retained coordinates are labelled as the last
observed map position. Native motion is read through the existing freshness
check; `QUIET`, a cancellation acknowledgement and task termination do not
mean confirmed physical stop. Only `motion.stop_confirmation = CONFIRMED`
does. Native task reasons are attributed to following only for a matching
task ID; robot-wide motion conditions remain a separate fact.

Five focused regressions failed before repair and pass after it. The related
VisualServo, semantic-module, PersonTracker and DINO tracker suites pass
(144 tests), with Ruff passing on the changed Python source and runtime tests.
The Gateway SSE regression also passes with the new visual reason and the
existing navigation reason preserved.
The three affected Web suites pass (35 tests); TypeScript and touched-file
ESLint checks pass. These tests distinguish normal lifecycle acknowledgements
from actual holds/rejections and require native confirmation for a stopped
claim. They do not constitute a rendered-browser or hardware acceptance test.
These are local contract tests using controlled features; they do not qualify
onboard recognition, physical stopping or a new field release.

## Sources

- [MOT17 official dataset](https://motchallenge.net/data/MOT17.zip)
- [Qwen3-VL-2B-Instruct official ModelScope repository](https://www.modelscope.cn/models/Qwen/Qwen3-VL-2B-Instruct)
- [Official ByteTrack repository](https://github.com/FoundationVision/ByteTrack)
- [DINOv3 official repository](https://github.com/facebookresearch/dinov3)
- [Kornia DINOv3 ONNX export and license](https://huggingface.co/kornia/dinov3)
- [TUM RGB-D download and dynamic sequences](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download)
- [TUM calibration, depth units, and pose format](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats)

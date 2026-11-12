# LingTu research notes

This directory contains research references used to shape product decisions.
The papers and notes here are evidence for design choices, not evidence that
LingTu already implements the techniques described in them.

Use [Chat UI reuse](chat-ui-reuse-20261110.md) for Flowith identification,
T3 Code and assistant-ui integration choices, current licenses and the existing
conversation API boundary. This is source research, not a completed UI migration.

Use [Astryx UI reference](astryx-ui.md) for the pinned Meta design-system source,
the local checkout, Web compatibility and useful component entry points.

Use [Mapping archive restoration](mapping-restore-20261109.md) for the native
graph restore, registration before continuation, independent-process and DDS
checks, and ProductControl cold-start wiring. Full Product hot switching is
still unfinished; the note keeps that boundary separate from SLAM mode changes.

Use [Navigation project comparison: Jie and DDDMR](navigation-project-comparison-20261107.md)
for the 2026-11-07 pinned official-source review added alongside LingTu and DimOS.
It distinguishes map editing, XYZ search, actual Go2 trajectory generators,
collision geometry and dynamic clearing from author demonstrations. The two
new projects have not been built or tested against LingTu in matched scenarios.

Use [Tora garage navigation](tora-garage-navigation-20261106.md) for the fixed-URDF
solid model, matched full-wheel/driving-envelope test, native global/SCAN ideal
feedback and 1080p replay. It does not qualify wheel physics, sensors or hardware.

Use [Narrow-passage physical trials](narrow-passages-physical-20261106.md) for
the actual door/L-corridor/double-door failures, paired ray-noise evidence,
quiet-heading repair and A* guide replay. The original narrow trials failed;
the matched 1.20m and 1.50m L-corridors reached their goals, with pauses and
diagnostic limitations still recorded rather than a complete acceptance claim.

Use [Global/local collision comparison](global-local-collision-comparison-20261106.md)
for the matched native XYZ, single-center, twin-cylinder and fixed-route heading
experiments. It records path changes, timing and remaining posture limitations;
the candidate is not connected to production navigation.

Use [Map-cleaning production and research boundaries](map-cleaning-boundaries-20261105.md)
for the ERASOR2/prune comparison, legacy adapter input limits, removed report-only
computation and isolated research builds. It does not claim a glass-persistence fix
or a matched algorithm-quality ranking.

Use [Map save latency and local reductions](map-save-latency-20261104.md) for
the NX save-latency investigation, current full-save call chain, bounded local
prune benchmark, repeated I/O and main-loop blocking findings. It separates
local performance evidence from the still-unmeasured NX save job.

Use [Glass visibility after map saving](glass-map-persistence-20261105.md) for
the native prune and OctoMap transmission counterexamples, performance-baseline
comparison, display-source boundaries, and the unimplemented glass persistence
requirements. These reproductions do not establish a field fix.

Use [Confirmed changes and production integration](production-integration-20261103.md)
for the source-to-RunPlan audit, completed Thunder overall model selection, state
corrections, and the explicit remaining implementation/deployment gaps. It links
the matched native and MuJoCo evidence rather than treating experiments as delivery.

Use [Map reconstruction and goal resolution](map-reconstruction-and-goal-resolution.md)
for the 2026-10-31 pinned DimOS/RViz/Nav2 comparison, display versus navigation
map semantics, actual-start anchoring, endpoint rejection causes, and the
planning-endpoint/arrival tolerance composition found in the current code.
It records implementation proposals and does not claim new motion validation.

Use [Terrain surface completion](terrain-surface-completion.md) for official
inpainting, smoothing and multi-level surface closing references, current native
support behavior, and the retained 2026-10-30 fixed-map closing comparison.
It distinguishes reconstructed terrain from sensor hits; no production surface
completion or new motion validation was performed by that review.

Use [GLIM independent validation and native integration assessment](glim-evaluation-and-integration.md)
for the 2026-10-29 author/independent CPU MID-360 evidence, mixed and failed
results, existing isolated comparison, exact native adapter seam, and the next
pose/cloud coordinate-and-time experiment, completed on 2026-10-30 with two
bounded real-recording replays. It also records observed cross-run variation and
the same-frame verification method. This is not an implemented alternate field
backend or a replacement of Product defaults. The latest 2026-10-30 decision
keeps LTLIO as the main implementation and narrows the next task to matching
mechanisms. A completed compatibility probe distinguishes GLIM shape weights
from native physical observation variance; it does not establish an accuracy gain.

Use [Online mapping ownership and build boundaries](online-mapping-boundaries-20261101.md)
for the dated investigation that preceded the online worker move and build
dependency cleanup. Its header points to the current code and behavior.

Use [Mapping and localization responsibilities](mapping-responsibilities-20261102.md)
for the current runtime/data owners, online geometry versus saved-map handoffs,
the meanings and limits of each queue and metric, and the remaining evidence gaps.

Use [Collision model and motion posture](../design/navigation-body-model-and-motion-posture.md#名称行业来源与实现边界)
for the 2026-10-26 naming decision **组合碰撞模型 / Compound Collision Model**,
primary Bullet/MoveIt/FCL/Drake and paper references, and the distinction between
the established representation, LingTu's implementation, and internal BODY2 serialization.

Use [Official SCAN and LingTu extension audit](scan-upstream-comparison-20261026.md)
for the pinned-source collision, unknown-space, support and trajectory comparison,
current URDF wiring, and the distinction between useful extensions and confirmed
over-restriction. Dated errors are maintained in the [error record](../operations/error_record.md).

Use [Navigation blockers and simplification audit](navigation-blockers-20261028.md)
for the current OctoPlanner3D + SCAN entry, input, planning, control and simulation
conditions, their waiting versus terminal effects, removed over-restrictions,
and the remaining height-input, retry and presentation inconsistencies.

Use [Recording and replay](recording.md) for the 2026-10-25 official-source
comparison of DDS Record & Replay, rosbag2, MCAP/Foxglove, Spot and DimOS. It
connects selectable native topics, joint telemetry, export and offline checks
to the remaining clock, configuration, retention and replay-workspace gaps.

Use [Paper materials and experiment entrypoints](paper-materials.md) for the
local collection of current source, figures, commands, logs, and historical
results. It records external raw-data locations and separates failed,
offline-component, simulation, and field evidence.

Use [Multi-robot remote video](video.md) for the 2026-10-17 official-source
comparison of go2rtc, MediaMTX and LiveKit, single-source hardware encoding,
fleet bandwidth budgets and the proposed staged verification. This is a
media-platform proposal, not evidence of an implemented WebRTC fleet service.

Use [Spot public architecture and LingTu design](spot.md) for the verified
protocol/service distinction, control and task-feedback semantics, and the
four-stage follow-task/API/transport proposal. It describes design work, not a
new deployed capability.

Use [LingTu task interface design](tasks.md) for the proposed responsibility
split between descriptions, running tasks, requests and execution evidence,
and for the small public interface over existing domain owners. Its Follow
lifecycle section now records the implemented task/session identity, query,
scoped cancellation and local-contract evidence; field qualification is open.
Its 2026-10-16 research section compares VL-Nav, SysNav, ViTL, VLAs-as-Tools,
Vesta and the alternative TANGO/G0.5 approach, and records the missing real-LLM
decomposition call in the single-instruction path.

The product boundary remains the native mapping -> save/reload -> localization
-> autonomous navigation workflow. Research code must not become a second map
owner or bypass native collision, traversability, localization, or motion
gates.

Use [SLAM corridor garage and glass implementation design](../design/slam-scene-adaptation.md)
for the 2026-10-19 SC-02/03/04 source audit, paper-to-function mapping, native
integration points, and ordered implementation evidence. Its work items are
proposals; the note does not establish completed scene acceptance.

See [Persistent semantic mapping](semantic-map.md) for the
current design, the map-layer vocabulary, the `object_memory` proposal, and
the implementation status. Downloaded papers are indexed in
[papers/README.md](papers/README.md).

For the localization map-optimization history, use the
[migration and acceptance record](localization-opt-migration.md) and the dated
[September handoff](../operations/localization-opt-handoff-20260917.md).
Current delivery work belongs to the [Roadmap](../roadmap.md#current-delivery-cycle).

Use [Semantic-map dataset plan](semantic-dataset-plan.md) for the evidence
ladder, current local dataset inventory, and the next multimodal recording
milestone.

Use [Semantic-dataset benchmarks](semantic-dataset-benchmarks.md) for the
downloaded 3DSSG adapter, the first reproducible scene-graph run, and the
planned Replica, SemanticKITTI, and KITTI-360 validation stages.

Use [Semantic-dataset progress](semantic-dataset-progress.md) for the
timestamped execution record, commands, outputs, evidence levels, and the
next acceptance gate.

Use [Semantic-navigation implementation](semantic-navigation-implementation.md)
for the OVI-MAP, OneMap, and HOV-SG comparison and the evidence required before
calling the text-to-motion loop a paper-level result.

Use [2026 semantic-navigation comparison](semantic-navigation-2026-comparison.md)
for the verified SysNav, VL-Nav, and language-to-goal platform matrix and the
first HabitatSim implementation slice.

Use [VL-Nav reproduction](vlnav.md) as the current implementation and evidence
ledger: the fixed 2026 paper version, selection kernel, server setup, and the
remaining perception-to-navigation loop.

Use [Room / floor / object validation](room-hierarchy-validation.md) for the
smallest HOV-SG-aligned hierarchy check, the official-dataset room-label gap,
and the next permitted room/region data gate.

Use [Natural-language person-follow experiment](person-follow.md) for the
real-video Qwen3-VL target-selection and tracking evidence, the retained
reacquisition failures, and the passing controlled RGB-D/native MuJoCo follow
test. It also records the ByteTrack cost-limited association correction and the
Host's retained-reference identity check. Their combined language-to-motion
loop remains unverified.

Use [Person tracking development reference](person-tracking-development-reference.md)
for the 2026-10-20 update on TrackVLA++, recent 2025–2026 referring, ReID,
association, memory, mask and search research. It maps stable paper IDs to
current code gaps and proposed work, separates training from inference costs,
and records hardware, timing and reuse limits. Its first deterministic fixes
now have local regression evidence; association, identity recovery and field
qualification remain open. Tasks are checked only with the stated evidence.

The same note records the 2026-10-11 DINOv3/CLIP continuous-video comparison:
MOT17 target attribution, TUM RGB-D position traces, explicit Host encoder
configuration, and why DINO remains optional after reducing wrong-person
output while increasing missing frames. Its full 827-pair returning-target
case retains three reacquisitions, candidate/reference VLM audit images,
raw XYZ traces and a Chinese H.264 comparison video. Candidate-specific
retries increase accepted observations from 164 to 260 and shorten three
recovery delays, at the cost of increasing Qwen calls from 15 to 55. The note
also records the remaining partial-body misses and a separate eight-crop
multiview diagnostic; those results are not a claim of reliable field following.
The 2026-10-12 follow-up connects trusted RGB references and three-frame
candidate evidence in both Host and replay. Its final full run accepts 290
observations with 36 verification calls, including partial-body recovery.
The note records uneven recovery timing and higher cumulative request/update
time despite fewer calls, alongside RGB timestamp and superseded-request
regression coverage.
The subsequent speed pass removes unused CLIP/OSNet appearance updates and
single-candidate reranking, while retaining the existing CLIP text cache.
It also exposes the ONNX CPU thread budget with a four-thread default.
The same 827-pair replay with eight threads on the research server retains
all 290 accepted observations and all 36 verdicts. Mean tracker update falls
from 105.87 to 62.04 ms; tracker-plus-identity time falls from 95.07 to
61.27 seconds. The note separates these observed timings from system-load
variation and the independent 4-versus-8-thread crop benchmark, and links the
comparison video with per-frame processing-time overlays.

The GPU follow-up moves DINO to the research server's RTX 5090 through the
existing Transformers backend. The same 827-pair synchronous replay retains
all 290 accepted observations, identities and XYZ while mean tracker time
falls from 62.04 to 10.82 ms. A source-paced replay with live YOLO, RGB-D and
asynchronous multiview/three-frame Qwen verification processes every pair in
29.135 seconds against a 28.824-second source span. Its foreground P95 is
45.83 ms and final delay is 311 ms; it is close to source throughput but does
not establish stable 31 Hz service. All 246 accepted target crops were
visually audited, with no observed distractor accepted. The note links complete
CPU/GPU and default-stream/independent-stream videos, separates setup costs,
and records actual model-generation settings. Host wiring also now uses the
exact processed RGB frame from Perception rather than treating one throttled
camera image as several independent candidate images. This remains server
recorded-data and local Host-contract evidence, not a measured robot-board
rate or an end-to-end cloud-GPU motion demonstration.

The next 2026-10-12 pass splits RGB PNG decode, depth PNG decode, live YOLO,
native association, person tracking and interframe preparation. It retains
serial depth loading and removes repeated per-box CUDA-to-CPU transfers.
On the same 827 pairs, mean foreground processing changes from 30.92 to
30.66 ms (0.9%); measured completion-delay P95 changes from 321.61 to
156.09 ms and final delay from 199.30 to 34.31 ms. Shared-server scheduling
also affects delay, so the complete queue improvement is not attributed to
the parser. The largest remaining completion delay is 333.13 ms; strict
every-frame 31 Hz is not demonstrated. All 233 final accepted crops were
visually audited as the plaid-shirt target. Multiview references, three-frame
candidate verification and CLIP text retrieval remain intact. The note links
the complete stage-comparison video, phase plots and rejected depth-thread
experiment. The user confirmed the robot is powered off, so actual D435i
RGB-D capture and robot-side timing are deferred; a read-only SHM recorder
is prepared. The local USB camera returned a camera-disabled placeholder
and is explicitly excluded from person-follow inference evidence.
The raw camera adapter also now converts `rgb8` to BGR in its existing
single copy, eliminating repeated BGR conversions in Host consumers.
Local SHM, pixel-format and JPEG contracts pass; robot-side speed remains
unmeasured while the robot is off.

The preceding same-video pass confirms a further blocking mechanism in the
installed Ultralytics predictor: its three profilers perform six device-wide
waits per batch, coupling YOLO to Qwen's independent CUDA stream. LingTu now
reuses the initialized predictor's original image/model/NMS stages without
those profiling barriers. A fresh 827-pair baseline/final comparison retains
identical detection/native tracking output and 34 multiview/three-frame
identity requests. YOLO mean changes from 11.23 to 9.41 ms; completion-delay
P95 changes from 225.82 to 84.75 ms and maximum from 317.25 to 169.44 ms.
This is a measured improvement on the shared research GPU, not an every-frame
deadline. The person-follow note records the controlled Qwen overlap probe,
full comparison evidence and remaining decode/tracker spikes. Robot testing
remains deferred at the user's request.

The latest pass separately measures first-frame CLIP selection and actual
target locking, file reads versus RGB/depth PNG decoding, and Qwen image
decode/CPU preprocessing/input transfer. It removes a discarded temporary
selection lock, uses a measured eight-thread CPU budget, records synthetic
model readiness before accepting input, and batches DINO only during full
current-frame reacquisition scans. The same 827-pair fresh baseline/final
comparison keeps detection/native-track output identical: first-frame
processing changes from 116.02 to 56.92 ms, completion-delay P95 from
92.08 to 64.77 ms and maximum from 170.11 to 118.57 ms. The larger
184.96 ms intermediate is also retained and explained. Multiview references,
three chronological candidate images and existing identity thresholds stay
intact. Final depth decoding still has a 24.48 ms outlier; stable per-frame
31 Hz and robot-side behaviour remain unverified. See the person-follow
ledger for the complete timing split, batch comparison, startup cost,
identity review and final video.

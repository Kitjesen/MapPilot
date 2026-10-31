# Experimental headless multimodal rig

Opt in with `thunderv4_headless_multimodal@1.0.0`. The first five declarations
are unchanged from `thunderv4_headless_navigation@1.0.0`: IMU 200 Hz, Mid360
10 Hz, RGB/depth 1 Hz each, truth odometry 100 Hz. Three streams are appended:
thermal/semantic 64x48 at 10 Hz and mono microphone 16000 Hz / 160-sample
blocks at 100 Hz. All three use `physics/mujoco_measurement/typed_dds`, physical
body ancestor `base_link`, disabled noise and seed 0.

Both new cameras use the actual `front_camera` position. Their explicit
extrinsic replaces (does not multiply) that frame's MJ camera convention:
optical right/down/forward map to base -Y/-Z/+X. The microphone uses the actual
`payload_top` frame at `[0, 0, 0.14]` m; no robot frames are invented.

The selected World must explicitly declare `measurements` with named
`thermal_surfaces` (World MJCF geom -> temperature/emissivity), `semantic_labels`
(World MJCF geom -> class/instance), and `acoustic_sources` (World MJCF body +
finite Pa waveform and timing). An empty declaration is schema-legal but does
not supply meaningful targets. Factory scene/source bindings belong to the
parent integration lane; this package supplies none and changes no default
session, World identity or existing package.

Calibration `simulated-reference-v1` is a software reference: thermal output
is Kelvin apparent temperature under the existing greybody assumptions;
microphone sensitivity is 0.02 V/Pa. Semantic output is simulation ground
truth. Package tests use a temporary World to validate all eight streams,
mounts and factory parameters. Complete rig and SensorPlan strict schemas now
pass without xfails: the existing Mid360 overlay is boolean-only and limited
to Mid360; headless `mujoco_camera` keeps its existing `16UC1` plan without UE
conversion fields, while UE still requires valid `requested_encoding` and
`depth_scale`. Existing rig/plan generation was not changed to add defaults.

Acknowledged native remote controls and raw archives are implemented for the
three new modalities, not yet uniformly for legacy RGB/Depth/LiDAR/IMU. The
[runtime status](../../../../../runtime/sensors/README.md) links the passed real
HTTP-route → native DDS → MuJoCo thermal control fixture and its recording.
That fixture is not a run of this eight-stream rig in the Factory Product.
Selected Fab-world collision/map/device/source binding, UE data/control UI and
same-Product real inspection acceptance remain unfinished; no full-feature,
performance or hardware/field qualification is claimed.

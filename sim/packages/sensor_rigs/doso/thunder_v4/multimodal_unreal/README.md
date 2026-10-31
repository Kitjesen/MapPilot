# Experimental mixed-owner Thunder rig

`thunderv4_multimodal_unreal@1.0.0` preserves the five declarations from
`thunderv4_navigation@1.0.0`: IMU 200 Hz, MID-360 10 Hz, UE RGB and depth
640×480 at 30 Hz, truth odometry 100 Hz. It appends thermal/semantic 64×48 at
10 Hz and a mono microphone (16 kHz, 160-sample blocks at 100 Hz).

This is an authored, tested package, **not a runnable default or a qualified
Factory Product**. No existing session or camera rate is silently changed.
World `measurements` must name actual thermal surfaces, semantic labels and
finite pressure sources. No temperatures, labels or sound sources are inferred
from the visual material. Source ownership remains UE for RGB/depth and MuJoCo
for thermal/semantic/acoustic; a headless camera must not impersonate UE.

## Physical installation

The new optical measurements reference the actual `camera1_link` MuJoCo body,
which already exists in Thunder's declared frame inventory. Its local position
is zero. The local optical quaternion is calculated as
`inverse(camera1_link MJCF quaternion) * [0.5, -0.5, 0.5, -0.5]` so optical
right/down/forward still align with base -Y/-Z/+X. The robot's original MJCF,
frames and RGB/depth declarations are unchanged. Body motion is read at capture
time; there is no UE spectator-pose substitution.

This choice also excludes only the optical carrier's own geometry from sensor
rays. Earlier base-body binding allowed a backward ray to hit the camera housing
about 15 micrometres from its origin. The chassis and limbs remain occluders.
Turning a nose camera back across the chassis does **not** guarantee that a
target becomes visible. Unknown surfaces and occluded targets remain invalid,
not room-temperature data. The microphone remains on `payload_top`, 0.14 m
above the base origin.

The existing software gimbal rotates calibrated rays about the optical origin;
it is not a new actuated MuJoCo gimbal, moving visual housing, motor or torque
model. Fisheye (equidistant) and panorama (equirectangular) use the existing
calibrated camera implementations; their controlled test overrides are not
newly exposed selector presets or hardware lens calibration claims.

## Evidence and current boundary

`tests/sim/test_unreal_multimodal_rig.py` checks unchanged UE declarations,
mixed-owner compilation, strict schemas, and real Thunder geometry under three
camera projections. Raw thermal and class/instance frames agree on visible
geometry; body rotation changes capture pose; 15-degree pan changes the image;
backward views preserve chassis occlusion. Canonical NPZ roundtrips preserve
arrays, units and source stamps. The target is an explicitly declared calibration
board, not an actual Fab machine, autonomous trajectory or measured hardware.

The final recorded optical check is under
`build/inspection-tests/20261022/thunder-mount-proof-02/`, with a separate XML.
It contains four poses for each projection, raw thermal/semantic NPZ files and
display-only previews. The temporary WorldPackage and actual test geometry
use the same explicitly positioned target. The earlier `proof-01` is retained.

`thunder-acoustic-mount-proof-01/` separately retains four actual microphone NPZ
blocks from the Thunder model at two fixed base positions. The receiver's world
position includes its 0.14 m mount offset; the farther source path arrives later
and has less pressure. Blocks retain 16,000 Hz, 160 samples, Pa, validity and the
explicit 0.02 V/Pa software calibration. Sampling leaves qpos unchanged. These
are static pose observations, not continuous driving, hardware microphone
calibration or a native-subscriber delivery test.

Thermal output retains the broadband apparent-Kelvin greybody assumptions and
the microphone's reference 0.02 V/Pa sensitivity; no LWIR or room-acoustics
qualification is implied.

The formal Product feeder still needs a Product-owned UE visual/camera bridge.
Until that bridge is implemented and verified, this rig must not be offered as
an accepted Product launch. The separate headless rig remains explicitly
headless; using it is not evidence that this mixed-owner rig ran.

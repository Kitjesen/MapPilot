# Experimental thermal camera

`thermal_camera@1.0.0`: 64x48 pinhole at 10 Hz, `fx=fy=32`,
`cx=32`, `cy=24` pixels (edge coordinates), optical +X right/+Y down/+Z forward.
`simulated-reference-v1` is explicit software calibration, not measured hardware
calibration. Output and noise metadata use Kelvin (`K`); noise is off, seed 0.
The existing broadband greybody model reports apparent temperature, not a
band-limited LWIR detector response or heat dynamics.

The consuming rig supplies the mount on a robot with physical ancestor
`base_link`. The selected World must explicitly declare `measurements` and
`thermal_surfaces` keyed by actual World MJCF geom names, with `temperature_k`
and `emissivity`. No temperatures are inferred from render materials.

The shared measurement path implements acknowledged native remote controls and
raw NPZ recording; see the [runtime status](../../../../runtime/sensors/README.md).
A real HTTP-route → native DDS → MuJoCo controlled thermal fixture passed,
including owner ACKs and a closed raw archive. That fixture used a 2x2 camera,
not this 64x48 package in a Factory Product. UE viewing/controls, the selected
Fab physical world and real Product inspection acceptance remain unfinished;
neither software calibration nor fixture evidence is hardware qualification.

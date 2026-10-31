# Thunder V4 explicit navigation sensor suite 1.1.0

Select `thunderv4_navigation@1.1.0`. The sibling legacy manifest remains
`thunderv4_navigation@1.0.0`, unchanged. This version declares nine rows for
the existing sensor selector; it does not introduce another runtime mode.

The first five declarations are identical to 1.0.0, including owner, source,
mount, configuration and rate: IMU 200 Hz, MID-360 10 Hz, Unreal RGB/depth
640 x 480 at 30 Hz, truth odometry 100 Hz. RGB/depth remain visual-owned
`unreal_camera` streams, not headless camera replacements.

The four appended, optional streams are:

| Sensor ID | Rate | Physical frame | Typed DDS message / source |
| --- | --- | --- | --- |
| `front_thermal` | 10 Hz | `camera1_link` | `ThermalImage` / `mujoco_measurement` |
| `front_semantic` | 10 Hz | `camera1_link` | `SemanticImage` / `mujoco_measurement` |
| `microphone` | 100 blocks/s | `payload_top` | `AcousticBlock` / `mujoco_measurement` |
| `gnss` | 10 Hz | `payload_top` | `GnssFix` / `mujoco_gnss` |

All four are physics-owned. The 64 x 48 physical cameras copy the verified
[`multimodal_unreal`](../../multimodal_unreal/README.md) declarations: zero local
translation on the actual `camera1_link` MJCF body, with local optical quaternion
`inverse(camera1_link MJCF quaternion) * [0.5, -0.5, 0.5, -0.5]`.
Optical right/down/forward therefore align with base -Y/-Z/+X. This does not
copy the older `headless_multimodal` base-body mount. The microphone remains
0.14 m above `base_link` on `payload_top`, mono 16 kHz with 160-sample blocks.

GNSS copies `thunderv4_gnss@1.0.0` explicitly: synthetic WGS84 latitude,
longitude and ellipsoidal altitude zero; zero world origin/yaw; noise off with
zero standard deviations/seed; `fix_available: false` (`NO_FIX`). This is not a
survey, site georeference, RTK fix or measured receiver calibration.

## Source Session defaults

`sim/sessions/examples/thunderv4_factory_park_hf/session.yaml` and
`sim/sessions/examples/thunderv4_open_field_hf/session.yaml` select this version
with only these four overrides:

```yaml
sensor_overrides:
  front_thermal: {enabled: false}
  front_semantic: {enabled: false}
  microphone: {enabled: false}
  gnss: {enabled: false}
```

The selector sees nine rows, four initially disabled. Default compilation still
produces exactly the original five streams and transport channels. Enabling an
extra changes Session data through the existing selector; it does not change
World, robot, physics policy, mode or RGB/depth cadence.

Physical measurements require explicitly declared World `measurements` under
the current catalog contract. An empty declaration contains no measured site
properties; absent/unknown surfaces, semantic labels and acoustic sources must
not be fabricated from visual materials or treated as valid measurements.
The package does not add any World properties. GNSS stays `NO_FIX` even when
selected, until explicitly configured otherwise.

The selector rejects enabling physical measurement rows when the source World
has no `measurements` declaration. Contract tests compile explicitly enabled
temporary Sessions against in-memory empty declarations; they do not bypass
this selector admission check or claim valid site measurements. GNSS selection
write-back is also tested against the unchanged production Worlds.

`tests/sim/test_sensor_suite_packages.py` checks discovery/schema, exact legacy
declarations, nine selector rows versus five default streams, each optional
typed route and all four together, real MJCF optical composition, microphone
mount and synthetic GNSS `NO_FIX`. These are Python contract/factory tests,
not UE, Product readiness, native DDS delivery or live physics evidence.

# Thunder V4 explicit headless navigation sensor suite 1.1.0

Select `thunderv4_headless_navigation@1.1.0`. The sibling 1.0.0 manifest is
unchanged. The nine declarations retain the first five exactly: IMU 200 Hz,
MID-360 10 Hz (including `navigation_fixture_raw_overlay: true`), RGB/depth
640 x 480 at 1 Hz, and truth odometry 100 Hz. RGB/depth remain
`sensor_runtime` / `mujoco_camera`; depth keeps `encoding: 16UC1`.

The optional `front_thermal`, `front_semantic`, `microphone` and `gnss` rows
use exactly the same physical declarations as the
[`navigation@1.1.0` suite](../../navigation/v1.1.0/README.md). Thermal/semantic
use the actual `camera1_link` body, zero local translation and the verified
body-relative optical quaternion from `multimodal_unreal`, not the older
`headless_multimodal` mount. Microphone and GNSS remain on `payload_top`.
Thermal/semantic are 10 Hz typed DDS measurements; the mono microphone emits
160-sample, 16 kHz blocks at 100 Hz. GNSS is 10 Hz typed DDS, synthetic WGS84
zero origin, ellipsoidal altitude, `NO_FIX`, noise off: never a site survey.

The formal headless default is
`sim/sessions/products/doso/thunder_v4/default.yaml`, selected by
`config/system/envs/sim.yaml`'s Thunder V4 default preset. It selects this new
version and disables only the four extras via `sensor_overrides`. The
`controlled_headless` example and the separate `nav.yaml` preset are not this
default and remain unchanged. World, robot, controller, spawn, physics, mode,
legacy camera rate and the original five compiled outputs are preserved.

The selector can expose all nine rows while default compilation publishes only
five. Optional measurements still need explicit World declarations under the
current catalog contract. No temperatures, labels, pressure sources or survey
coordinates are added to production Worlds. Empty/unannotated measurement
properties do not establish valid measurements. GNSS remains `NO_FIX` when
enabled with this configuration.

The shared `tests/sim/test_sensor_suite_packages.py` verifies schema/discovery,
legacy equality, defaults, individual and combined typed routes, real MJCF
mounts and GNSS no-fix semantics without starting any engine or Product.
This is not native delivery, simulation-run or field qualification evidence.

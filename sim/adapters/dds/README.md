# Native DDS simulation adapter

This directory is the transport boundary between MuJoCo and LingTu's canonical
CycloneDDS contracts. It does not own physics, SLAM, map state, planning, or
robot process lifecycle.

The Simulation Runtime truth-odometry path is deliberately separate from the
legacy navigation fixture:

```text
Sensor Runtime TruthOdometrySample
  -> LTOD v1 binary records on stdin
  -> lingtu_truth_odom_publisher
  -> /sim/truth/odom (rt/sim/truth/odom, lingtu.dds.Odometry)
```

`lingtu_truth_odom_publisher` requires `--dds-domain`, `--session-id`,
`--parent-frame`, `--child-frame`, and `--ready-file`. It publishes readiness
only after the dedicated DDS writer exists. LTOD v1 carries the session ID,
model/reset generations, sequence, simulation timestamp, pose, available
velocities, and optional covariance. The process rejects malformed records,
identity mismatches, backward generations, and non-increasing sequence or time
within one generation, then exits non-zero without publishing the rejected
record. A new model/reset generation must begin at sequence zero and may reset
simulation time.

Default truth odometry never writes `rt/slam/odom_prior`. The older sensor
publisher can still emit that topic only through its explicit diagnostic
navigation-fixture path described below.

The portable sensor path is:

```text
MuJoCo sensor generator
  -> LTU1 binary records on stdin
  -> lingtu_mujoco_sensor_publisher
  -> rt/lidar/raw_packet + rt/lidar/raw_frame + rt/imu/raw
```

When the navigation fixture is explicitly enabled, the same process may also
publish odometry prior, registered cloud, TF, and localization-health fixture
data. Those are simulation truth inputs for native navigation acceptance; they
are not physical sensor mocks and must not be presented as SLAM output.

The Product control path uses `lingtu_mujoco_driver_bridge`. It reads the same
typed `FinalVelocityCommand` as the real driver, applies the real native driver
normalization/freshness/writer-gate rules, and sends a versioned controller
command to MuJoCo. `DriverControlState` acknowledges an output only after the
MuJoCo controller reports the matching physical step as applied. The ready file
means only that the DDS transport has exactly one command writer; it is not a
driver-ready signal.

`lingtu_mujoco_cmd_vel_tap` remains a read-only protocol diagnostic. It does not
own safety state, physical application, or Product acceptance evidence. Camera
color/depth remains on the existing camera/SHM path and is intentionally not
packed into this high-rate LiDAR/IMU process.

| Simulation signal | Source | Boundary in this adapter |
| --- | --- | --- |
| LiDAR raw packet/frame | MuJoCo MID-360 raycast | Canonical typed DDS |
| IMU | MuJoCo angular velocity and conditioned specific force | Canonical typed DDS |
| Truth odometry | Sensor Runtime `TruthOdometrySample` | Dedicated `rt/sim/truth/odom` typed DDS |
| Odom prior | Optional MuJoCo truth fixture | Canonical typed DDS, off by default |
| Registered cloud / TF / localization health | Optional navigation fixture | Canonical typed DDS, never claimed as SLAM evidence |
| Final `cmd_vel` | Native navigation output | Physical driver bridge into MuJoCo; ACK after applied step |
| Camera color/depth | MuJoCo camera renderer | Separate camera/SHM adapter |
| GNSS | Not modeled by this bridge | Not covered |
| Battery and driver state | Simulation driver state | Separate driver/control boundary |

## Windows build

Install or build the official CycloneDDS SDK, then point LingTu at its install
prefix. The prefix must contain `lib/cmake/CycloneDDS/CycloneDDSConfig.cmake`
(or the distribution's `share/CycloneDDS` equivalent), `bin/idlc.exe`,
`bin/cycloneddsidlc.dll`, and
`bin/ddsc.dll`.

```powershell
$env:LINGTU_CYCLONEDDS_PREFIX = "C:\path\to\cyclonedds-install"
powershell -ExecutionPolicy Bypass -File scripts\build\build_mujoco_native_dds_windows.ps1
```

With no generator override, the helper preserves the existing Visual Studio
`x64` build. A different Visual Studio target can be selected explicitly:

```powershell
powershell -ExecutionPolicy Bypass -File scripts\build\build_mujoco_native_dds_windows.ps1 `
  -Generator "Visual Studio 17 2022" -Architecture ARM64
```

Ninja is also supported. It is a single-configuration generator, so the helper
sets `CMAKE_BUILD_TYPE` and keeps the executable under the same
configuration-named directory used by the Visual Studio build. Do not pass
`-Architecture` with Ninja because CMake's Ninja generator does not accept
`-A`:

```powershell
powershell -ExecutionPolicy Bypass -File scripts\build\build_mujoco_native_dds_windows.ps1 `
  -Generator Ninja -BuildDirectory build\windows-native-dds-adapter-ninja
```

Use a separate build directory when changing generators for an existing build
tree. The `CMAKE_GENERATOR` environment variable is also honored when
`-Generator` is omitted.

The helper configures, builds, runs CTest, and copies `ddsc.dll` beside the
adapter executables. The public bridge therefore does not need a persistent
SDK `PATH` modification:

```powershell
python sim\scripts\mujoco\native_dds_sensors.py --domain-id 42 --duration 10
```

The Python bridge selects the portable publisher and command tap from
`build/windows-native-dds-adapter/Release` automatically. An explicit
`--publisher-bin` remains available for compatibility experiments, but the
hardware-specific `livox_sdk2_stream` is not a default MuJoCo dependency.

Use an isolated DDS domain for simulation. The supported product range is
`0..232`, which remains valid with CycloneDDS's default UDP port mapping.

## Linux build

```bash
cmake -S sim/adapters/dds -B build/mujoco_native_dds \
  -DLINGTU_MUJOCO_NATIVE_DDS_BUILD_RUNTIME=ON \
  -DBUILD_TESTING=ON
cmake --build build/mujoco_native_dds -j
ctest --test-dir build/mujoco_native_dds --output-on-failure
```

The sensor publisher writes an atomic readiness marker only after DDS writers
have initialized. `native_dds_sensors.py` waits for that marker and fails fast
if the child exits or never becomes ready. Shutdown owns the local Windows
process directly; WSL launches retain their separate Linux PID handshake.

## Scope of the Windows proof

The portable build proves canonical IDL generation, CycloneDDS participant
creation, LiDAR/IMU record parsing and publishing, DDS loopback, and the final
`cmd_vel` tap on Windows. It does not by itself prove that production `slamd`,
`mapd`, traversability, or `navd` have been ported to Windows, and it is not a
real-robot acceptance result.

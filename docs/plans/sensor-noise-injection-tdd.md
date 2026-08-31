# Simulation Sensor Noise Injection TDD

Status: proposed; architecture gate before implementation
Audience: simulation catalog, Sensor Runtime, RobotSimUE sensor, recording, and qualification maintainers

## Decision Summary

LingTu will use one compiled noise decision per sensor stream. A session-level
switch decides whether nominal sensor noise is active, each `SensorPackage`
owns its calibrated model and parameters, and the Catalog Resolver compiles the
effective immutable decision into `sensor.plan.json`.

Noise is applied exactly once, after an ideal sample is produced and before it
is encoded or published. The runtime that owns the sensor owns the operation:
physics-owned sensors apply noise in the simulation Sensor Runtime; RGB and
depth apply it inside RobotSimUE. There is no process-global mutable noise flag.

The public operation is deliberately small:

```python
def apply_sensor_noise(sample, *, spec, stamp):
    if not spec.enabled:
        return sample
    return apply_model(sample, spec, stamp)
```

Each runtime has an equivalent typed entry point. `enabled=false` must be an
identity operation: the returned measurement payload is byte-for-byte equal to
the ideal path, and no random generator or model state is advanced.

## Goals

- Provide one explicit, testable switch for nominal noise injection.
- Keep sensor physics parameters in versioned sensor packages.
- Make identical session input and sample identity produce identical output.
- Preserve timestamps, sequence numbers, frames, and generation identity.
- Support Python physics sensors and C++ Unreal camera sensors without a second
  configuration source.
- Leave a narrow extension point for later calibrated stateful models.

## Non-goals

- Noise does not model complete signal loss, packet loss, latency, stale data,
  malicious corruption, or process suspension. Those are fault injection.
- Noise never modifies `truth_odom` or another channel explicitly marked as
  simulation truth.
- Version one does not support changing noise while a session generation is
  running.
- Version one does not introduce a plugin framework, dependency, or generic
  expression language for noise models.

## Noise And Faults Are Separate

| Concern | Owner | Control |
| --- | --- | --- |
| White measurement noise, calibrated range error, quantization | Sensor Runtime | `sensor_noise.enabled` |
| Dropout, loss of fix, latency, frozen stream, malformed packet | Scenario / qualification fault injector | separate fault plan |
| Ground-truth state | Physics Runtime | never noisy |

This separation prevents a normal-realism checkbox from unexpectedly causing
sensor outages. Existing Mid360 dropout controls remain stress/fault controls
and must not be folded into the nominal noise switch.

## Ownership And Data Flow

```text
SessionSpec.sensor_noise.enabled + SessionSpec.seed
                         │
SensorPackage.noise ─────┤
                         ▼
                 Catalog Resolver
          validates and compiles effective spec
                         │
                         ▼
                   sensor.plan.json
        per stream: enabled/model/parameters/seed/spec digest
                         │
          ┌──────────────┴──────────────┐
          ▼                             ▼
 Physics Sensor Runtime           RobotSimUE Sensors
 IMU / Mid360 / GNSS              RGB / Depth
          │                             │
          └──── ideal sample ───────────┘
                         ▼
                 apply sensor noise
                         ▼
                  encode / transport
                         ▼
             readiness / recording evidence
```

| Layer | Owns | Must not own |
| --- | --- | --- |
| `SessionSpec` | Whether nominal sensor noise is enabled for this deterministic session | Model parameters or mutable runtime state |
| `SensorPackage` | Noise model ID, units, calibrated defaults, valid ranges | Per-run enable state or random state |
| `SensorRigPackage` | Mount, rate, and sensor selection | A second copy of the noise model in version one |
| Catalog Resolver | Validation, seed derivation, effective per-stream plan, digest | Sampling or random state |
| Sensor owner Runtime | Applying the compiled model exactly once | Re-reading packages or SessionSpec |
| Recorder / evidence | Recording the effective spec digest and applied status | Reconstructing an undeclared noise configuration |

## Configuration Contract

### SessionSpec

The additive optional setting is:

```yaml
sensor_noise:
  enabled: true
```

If omitted, it means disabled and preserves existing session behavior. The
existing top-level `seed` is the only run-content seed; a second noise seed is
not introduced. When present, the resolved SensorPlan carries the setting.
ProductControl must publish a new RunPlan before the change can affect a
running Product.

The user-facing Simulation Studio checkbox edits this setting before session
resolution. A live checkbox must not mutate a running process. If live changes
are required later, they become generation-stamped Scenario events and are
recorded explicitly.

### SensorPackage

The existing optional `noise` field becomes a validated contract:

```yaml
noise:
  model: imu_white_gaussian.v1
  parameters:
    angular_velocity_std_rad_s: 0.005
    linear_acceleration_std_m_s2: 0.05
```

Presence means that the package supports nominal noise. Model-specific units
and bounds are validated by the Catalog Resolver. Unknown fields, unknown
models, non-finite values, and negative standard deviations fail resolution.

The package does not contain `enabled`. Enabling is a session decision, while
the package describes what the sensor is.

### Compiled SensorPlan

When `sensor_noise` is explicitly present, each non-truth stream receives:

```json
{
  "noise": {
    "enabled": true,
    "model": "imu_white_gaussian.v1",
    "parameters": {
      "angular_velocity_std_rad_s": 0.005,
      "linear_acceleration_std_m_s2": 0.05
    },
    "seed_hex": "0123456789abcdef"
  }
}
```

`seed_hex` is a fixed 16-character hexadecimal string rather than a JSON
number, avoiding 64-bit precision loss in C++/JSON consumers.

When the SessionSpec omits `sensor_noise`, the resolver emits no new field in
version one. Existing SensorPlans therefore retain their previous shape. An
explicitly enabled session fails resolution if a selected
non-truth sensor has no supported noise model.

## Determinism Contract

The compiler derives one stream seed from the existing session seed and stable
sensor ID:

```text
SHA256("lingtu.sensor-noise.v1\0" + decimal(session.seed)
       + "\0" + sensor_id_utf8)[0:8]
```

The eight bytes are interpreted as an unsigned big-endian value and emitted as
`seed_hex`. A sample derives its random sequence only from:

- stream seed;
- `model_generation`;
- `reset_generation`;
- sensor sample sequence;
- model-defined component or point index.

Wall time, thread ID, process ID, publication order, and transport allocation
must not affect noise. Dropping one frame must not shift the random values of
later frames.

Version-one models are stateless. Bias drift and random walk are deferred until
calibration evidence requires them. A future stateful model must own a
per-stream `NoiseState`, reset on `reset_generation`, and advance using
simulation time rather than wall time.

## Runtime Interface

### Physics-owned streams

The simulation Sensor Runtime adds one post-extraction operation before the
existing sink call:

```python
ideal = endpoint.extractor(scheduled, snapshot)
measured = apply_sensor_noise(ideal, spec=scheduled.stream.noise, stamp=ideal.stamp)
endpoint.sink.publish(measured)
```

The dispatcher stays explicit and small: a `match` on the versioned model ID
calls a typed function such as `apply_imu_white_noise` or
`apply_mid360_return_noise`. A registry/plugin system is not justified yet.

### Unreal-owned camera streams

RobotSimUE parses the same compiled `noise` object. RGB noise is applied after
scene capture and before image encoding/SHM publication. Depth noise is applied
to metric depth before `16UC1` quantization. The camera implementation should
use a GPU pass or vectorized native path; it must not add a Python or per-pixel
game-thread loop.

The C++ public boundary is equivalent to:

```cpp
bool ApplyCameraNoise(
    FMutableArrayView<float> Values,
    const FNoiseSpec& Spec,
    const FTruthSampleStamp& Stamp,
    FString& OutError);
```

Color and depth use separate typed functions behind this boundary. A stream
with `enabled=false` bypasses allocation and processing.

## First Model Set

| Stream | Version-one model | First parameters |
| --- | --- | --- |
| RGB | `rgb_gaussian.v1` | per-channel standard deviation and clamp range |
| Depth | `depth_axial_gaussian.v1` | base and range-scaled standard deviation, min/max range |
| IMU | `imu_white_gaussian.v1` | gyro and acceleration standard deviation |
| Mid360 | `mid360_return_gaussian.v1` | radial range, angle, and reflectivity standard deviation |
| GNSS | `gnss_fix_gaussian.v1` | horizontal/vertical standard deviation by fix class |
| Truth odometry | none | injection forbidden |

Thermal and acoustic models are added only when those SensorPackages and their
measurement contracts exist.

## MuJoCo Boundary

MuJoCo 3.1.4 and later no longer apply MJCF sensor `noise` automatically. The
field is metadata for user processing. LingTu may import those values as
package defaults, but the only product injection point remains the Sensor
Runtime operation described above. This avoids hidden or double noise.

Reference:
https://mujoco.readthedocs.io/en/stable/changelog.html#version-3-1-4-april-10th-2024

## Lifecycle And Reset

- `Prepare`: validate every compiled model and construct immutable specs.
- `Start`: no random state is created for disabled or stateless models.
- `Sample`: ideal measurement -> exactly one noise operation -> encode/publish.
- `Reset`: discard future state, bind the new reset generation, and never reuse
  values derived for the previous generation.
- `Rebind`: discard model-index caches and specs from the prior model generation.
- `Stop`: release state without modifying recorded evidence.

## Evidence

Per-stream readiness/evidence adds:

```json
{
  "noise_enabled": true,
  "noise_model": "imu_white_gaussian.v1"
}
```

Evidence does not claim that statistical calibration passed. Qualification is
a separate test result tied to the package version and model parameters.

## Failure Rules

| Failure | Result |
| --- | --- |
| Noise enabled but package has no model | Catalog resolution fails |
| Unknown model or invalid parameter/unit | Catalog resolution fails |
| Noise requested on truth stream | Catalog resolution fails |
| Runtime does not implement compiled model | Stream fails readiness; session fails if sensors are required |
| Prepared noise settings differ from the SensorPlan | Stream fails readiness |
| Generation changes during sample processing | Discard sample; do not publish across generations |

## Verification Gates

1. Disabled path returns the exact original sample and consumes no RNG state.
2. Same session seed, sensor ID, generation, sequence, and input produce the
   same measurement across repeated runs of the same runtime implementation.
3. Different seeds produce different measurements.
4. Processing order and a dropped prior sample do not change a later sample.
5. Reset/model generation boundaries cannot publish stale noisy samples.
6. Resolver rejects unsupported models, invalid units, truth-channel noise, and
   enabled streams without profiles.
7. Statistical smoke tests verify mean and standard deviation within bounded
   tolerance over a fixed deterministic sample set.
8. Camera and Mid360 performance tests prove the noise path still meets the
   stream-rate contract.
9. Readiness and recording include the exact effective noise spec digest.

## Rollout

1. Add optional SessionSpec and SensorPlan schema fields plus deterministic
   resolver compilation. Do not change samples yet.
2. Implement the shared Python control entry and IMU/Mid360 typed models.
3. Integrate the existing WTRTK-980 Gaussian model through a formal GNSS
   SensorPackage; keep loss windows in fault injection.
4. Implement RobotSimUE RGB/depth noise without game-thread pixel loops.
5. Add the pre-launch Simulation Studio checkbox.
6. Promote the proposal into an architecture contract only after the schema,
   resolver, runtime, and evidence tests are green.

## Deferred Decisions

| Decision | Revisit trigger |
| --- | --- |
| Per-sensor enable overrides | A real session requires mixed noisy/ideal non-truth streams |
| Bias instability and random walk | Calibration data requires time-correlated IMU/GNSS noise |
| Cross-language bit-identical RNG | One sensor model must migrate between Python and C++ while preserving exact replay |
| Runtime live toggling | Scenario qualification requires a recorded mid-run noise transition |
| Model plugin registry | Independent external packages need models without modifying the runtime |

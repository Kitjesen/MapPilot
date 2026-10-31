# Simulation measurement wire v1

This is the implementation contract for the next production slice. Declaring it
does not mean native publication, Host subscription or the factory UI is complete.
Existing ProductControl/feeder/native-camera process ownership remains unchanged.
Measurements share the camera-owned LTU1 endpoint, not the IMU low-latency queue.

## Native DDS

Source: `src/message/idl/sensors.idl`, topics in `topics/sensors.yaml`.
ThermalImage uses Kelvin float arrays; AcousticBlock uses Pa float arrays and
optional volts; SemanticImage uses two uint32 label arrays. All are keyed by
sensor_id so one sensor does not evict another's latest history instance.
See [CycloneDDS key annotations](https://cyclonedds.io/content/guides/supported-idl.html).
Header carries the original simulation time and physical frame. MeasurementIdentity
retains product_session_id separately from compiled session_id, instance_id,
model/reset generations, sequence and exact uint64 sim_time_ns.
The IDL field is `sample_sequence` (`sequence` is an IDL keyword); it maps to
the source stamp's `sequence` without changing its value. Host must map
source time to its observation clock explicitly; it must not overwrite identity.

## LTU1 record type 9, nested LTM1 payload

Outer LTU1 timestamp/sequence/count must equal inner time/sequence/rows*columns.
The current outer sequence is uint32; reject overflow rather than truncate.

All inner integers/floats little endian. Fixed prefix Python struct:
`<4sHH6I4Q5H2x` (76 bytes).

| Field order | Type / meaning |
| --- | --- |
| magic, version, modality | `4s='LTM1', uint16=1, uint16`: thermal=1, acoustic=2, semantic=3 |
| header_bytes | uint32; 76 + sum of five UTF-8 identifier lengths |
| metadata_bytes | uint32; 1..1 MiB finite JSON object, calibration/model/units/full source metadata |
| rows, columns | uint32; H,W for images; channels,samples for acoustic; both positive |
| sample_rate_hz | uint32; acoustic positive; images zero |
| flags | uint32; bit0 only = acoustic voltage present, all other bits rejected |
| model_generation, reset_generation, sequence, sim_time_ns | four uint64 |
| five identifier lengths | five uint16: product_session_id, session_id, instance_id, sensor_id, frame_id |
| reserved | two zero bytes |

After fixed prefix: five identifiers in that order, no NUL padding or terminator;
then UTF-8 metadata JSON; then primary array; then N one-byte 0/1 mask values;
then secondary array when present. Identifiers must be nonempty trimmed UTF-8,
max 255 bytes each, no embedded NUL. Product session must match the authenticated
endpoint identity; source session must match the compiled plan's bound identity.

N=rows*columns; cap total inner payload at 32 MiB, guard arithmetic before allocation.
Thermal primary=N float32 K, no secondary. Acoustic primary=N float32 Pa;
secondary=N float32 V iff bit0. Semantic primary=N uint32 class ID,
secondary=N uint32 instance ID always, flags=0. No padding between arrays.
Invalid thermal/acoustic values must be NaN; valid ones finite (K strictly >0).
Invalid semantic IDs zero; valid IDs positive and <=int32 max for Python parity.
All masks exactly 0 or 1. These checks occur at the native input boundary.

Metadata is retained verbatim after finite JSON validation in Python. Native
transport need not interpret the radiometric model, but must retain all bytes
and typed shape/time/identity. NPZ is for lossless recording, not this online wire.
Native DDS sends typed arrays. No temperature/depth encoding alias is permitted.

## Stream semantics

Validate each sensor independently: session and sensor/frame/instance identity,
generation monotonic, time/sequence monotonic within generation; reset permits
time/sequence restart. Do not compare sequence counters across different sensors.
Publishers report malformed/stale input; never silently restamp/reset it.
No claim of reliable delivery for best-effort streams; recording belongs to the
existing owner and stores source data before transport loss.

Acoustic blocks must obey block_samples/sample_rate duration. Geometry is frozen
at the block's source observation; direct-path propagation and no-Doppler/no-
reflection limitations remain in metadata. New physics snapshots cannot fill
unobserved historical geometry with fake timestamps.

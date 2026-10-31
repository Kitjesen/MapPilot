# Experimental microphone

`microphone@1.0.0`: one channel, 16000 Hz pressure samples, 160 samples per
block (100 Hz). Sensitivity is explicitly 0.02 V/Pa with calibration ID
`simulated-reference-v1`; this is software reference calibration, not measured
hardware calibration. Pressure/noise use Pa; noise is off, seed 0.

The rig supplies the mount; the channel offset is local to that mount and
Catalog/factory compose it into physical ancestor `base_link` exactly once.
The selected World must explicitly declare `measurements.acoustic_sources`:
unique `source_id`, actual World MJCF `body_name`, finite nonempty mono
`pressure_pa`, `sample_rate_hz`, `start_sim_time_ns`, `reference_distance_m`,
and optional body-local `local_offset_m`. No engine sounds or waveforms are
inferred. Sources are finite, not implicitly looped; the existing direct-path
model does not simulate reflections/diffraction.

This microphone shares the new thermal/semantic/acoustic acknowledged native
control and raw NPZ archive path; see the
[runtime status](../../../runtime/sensors/README.md). Rate changes must respect
the sample-rate/block cadence; camera-only gimbal control is not supported.
The real HTTP-route → native DDS → MuJoCo control fixture exercised thermal,
not a microphone Factory Product run. UE waveform/control UI, the selected Fab
physical world and real Product inspection acceptance remain unfinished. This
is not real-time hardware capture or physical microphone qualification.

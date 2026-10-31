from __future__ import annotations

import json
import tracemalloc
from dataclasses import replace

import numpy as np
import pytest
from sim.runtime.sensors.acoustics import AcousticClip, AcousticSource, MicrophoneConfig, MicrophoneSensor
from sim.runtime.sensors.measurement_control import MeasurementNoise
from sim.runtime.sensors.samples import SensorSampleStamp

pytestmark = [pytest.mark.sim]


def _stamp(time_ns=0, **changes):
    fields = dict(
        session_id="acoustic-test",
        instance_id="robot",
        sensor_id="robot.microphone",
        frame_id="robot/microphone",
        model_generation=1,
        reset_generation=0,
        sequence=0,
        sim_time_ns=time_ns,
    )
    fields.update(changes)
    return SensorSampleStamp(**fields)


def _source(**changes):
    fields = dict(
        stamp=_stamp(),
        source_id="speaker",
        pressure_pa=np.array([1.0, 0.0, 0.0]),
        reference_distance_m=1.0,
        sample_rate_hz=1000,
        start_sim_time_ns=0,
        position_world_m=(2.0, 0.0, 0.0),
    )
    fields.update(changes)
    return AcousticSource(**fields)


def _config(**changes):
    fields = dict(
        sample_rate_hz=1000,
        block_samples=6,
        channel_offsets_m=((0.0, 0.0, 0.0),),
        speed_of_sound_mps=1000.0,
    )
    fields.update(changes)
    return MicrophoneConfig(**fields)


def test_impulse_arrives_after_direct_path_delay_in_pressure_units():
    stamp = _stamp()
    frame = MicrophoneSensor(_config()).sample(stamp, (_source(),))
    assert frame.stamp is stamp
    assert frame.sample_rate_hz == 1000
    assert frame.pressure_pa.dtype == np.float32
    np.testing.assert_allclose(frame.pressure_pa, [[0, 0, 0.5, 0, 0, 0]])
    np.testing.assert_array_equal(frame.valid_mask, np.ones((1, 6), dtype=bool))
    assert frame.metadata["simulation"] is True
    assert frame.metadata["pressure_unit"] == "Pa"
    assert frame.metadata["provenance"] == {
        "model_id": "direct_path_pressure", "model_version": "1", "qualification": "EXPERIMENTAL"}


@pytest.mark.parametrize(
    "waveform,expected",
    [
        ([1.0, 0.0, 0.0], [[0, 0, 0, 0.2, 0, 0]]),
        ([0.0, 1.0, 0.0], [[0, 0, 0, 0.2, 0.2, 0]]),
    ],
)
def test_fractional_delay_interpolates_the_zero_extended_waveform(waveform, expected):
    frame = MicrophoneSensor(_config()).sample(
        _stamp(),
        (_source(position_world_m=(2.5, 0, 0), pressure_pa=waveform),),
    )
    np.testing.assert_allclose(frame.pressure_pa, expected, atol=1e-7)


def test_channels_use_supplied_world_positions_or_world_aligned_offsets():
    sensor = MicrophoneSensor(_config(channel_offsets_m=((0, 0, 0), (-2, 0, 0))))
    source = _source()
    offsets_frame = sensor.sample(_stamp(), (source,))
    world_frame = sensor.sample(
        _stamp(),
        (source,),
        channel_positions_world_m=((0, 0, 0), (-2, 0, 0)),
    )
    expected = [[0, 0, 0.5, 0, 0, 0], [0, 0, 0, 0, 0.25, 0]]
    np.testing.assert_allclose(offsets_frame.pressure_pa, expected, atol=1e-7)
    np.testing.assert_array_equal(world_frame.pressure_pa, offsets_frame.pressure_pa)


def test_sources_superpose_and_reference_radius_caps_near_field_gain():
    frame = MicrophoneSensor(_config()).sample(
        _stamp(),
        (
            _source(source_id="near", position_world_m=(0, 0, 0)),
            _source(source_id="far"),
        ),
    )
    np.testing.assert_allclose(frame.pressure_pa, [[1, 0, 0.5, 0, 0, 0]], atol=1e-7)


def test_blocked_direct_path_marks_only_affected_channel_unmodelled():
    config = _config(channel_offsets_m=((0, 0, 0), (-2, 0, 0)))
    frame = MicrophoneSensor(config).sample(_stamp(), (_source(),), blocked_paths=(("speaker", 0),))
    assert np.isnan(frame.pressure_pa[0]).all()
    assert not frame.valid_mask[0].any()
    assert frame.valid_mask[1].all()
    np.testing.assert_allclose(frame.pressure_pa[1], [0, 0, 0, 0, 0.25, 0], atol=1e-7)
    assert frame.metadata["blocked_paths"] == [{"source_id": "speaker", "channel": 0}]
    assert frame.metadata["near_field_policy"] == "gain_capped_at_reference_radius"


def test_voltage_requires_explicit_calibration_and_never_relabels_pressure():
    calibration = {"calibration_id": "simulation-reference", "sensitivity_unit": "V/Pa"}
    config = _config(sensitivity_volts_per_pa=0.02, calibration_metadata=calibration)
    frame = MicrophoneSensor(config).sample(_stamp(), (_source(),))
    np.testing.assert_allclose(frame.pressure_pa[0, 2], 0.5)
    np.testing.assert_allclose(frame.voltage_v, frame.pressure_pa * 0.02)
    assert frame.metadata["calibration"] == calibration
    assert frame.metadata["sensitivity_volts_per_pa"] == 0.02
    json.dumps(frame.metadata, allow_nan=False)
    assert MicrophoneSensor(_config()).sample(_stamp(), ()).voltage_v is None
    with pytest.raises(ValueError, match="calibration_metadata"):
        _config(sensitivity_volts_per_pa=0.02)


def test_source_rate_start_time_and_finite_support_are_explicit():
    source = _source(
        pressure_pa=np.array([0.0, 2.0, 4.0]),
        sample_rate_hz=500,
        start_sim_time_ns=1_000_000,
        position_world_m=(0, 0, 0),
    )
    frame = MicrophoneSensor(_config(block_samples=8)).sample(_stamp(), (source,))
    np.testing.assert_allclose(frame.pressure_pa, [[0, 0, 1, 2, 3, 4, 2, 0]], atol=1e-7)
    assert frame.valid_mask.all()


def test_fractional_nanosecond_grid_has_exact_chunk_continuity():
    source = _source(
        sample_rate_hz=44100,
        pressure_pa=np.linspace(0, 3, 40),
        position_world_m=(0, 0, 0),
    )
    whole = MicrophoneSensor(_config(sample_rate_hz=44100, block_samples=13)).sample(_stamp(), (source,))
    sensor = MicrophoneSensor(_config(sample_rate_hz=44100, block_samples=5))
    first = sensor.sample(_stamp(), (source,))
    second_stamp = _stamp(5 * 1_000_000_000 // 44100, sequence=1)
    second = sensor.sample(second_stamp, (replace(source, stamp=second_stamp),))
    third_stamp = _stamp(10 * 1_000_000_000 // 44100, sequence=2)
    third = MicrophoneSensor(_config(sample_rate_hz=44100, block_samples=3)).sample(
        third_stamp,
        (replace(source, stamp=third_stamp),),
    )
    np.testing.assert_array_equal(
        np.concatenate([first.pressure_pa, second.pressure_pa, third.pressure_pa], axis=1),
        whole.pressure_pa,
    )
    assert second.metadata["first_sample_index"] == 5
    with pytest.raises(ValueError, match="sample grid"):
        sensor.sample(_stamp(1), ())


@pytest.mark.parametrize(
    "changes",
    [
        {"reference_distance_m": 0},
        {"reference_distance_m": float("inf")},
        {"sample_rate_hz": 0},
        {"sample_rate_hz": True},
        {"sample_rate_hz": 2.5},
        {"start_sim_time_ns": -1},
        {"start_sim_time_ns": None},
        {"pressure_pa": []},
        {"pressure_pa": [[1, 2]]},
        {"pressure_pa": [float("nan")]},
        {"pressure_pa": [float("inf")]},
        {"position_world_m": None},
        {"position_world_m": (0, float("nan"), 1)},
        {"source_id": ""},
    ],
)
def test_missing_or_invalid_source_fields_raise_instead_of_becoming_silence(changes):
    with pytest.raises(ValueError):
        _source(**changes)


@pytest.mark.parametrize(
    "changes",
    [
        {"sample_rate_hz": 0},
        {"sample_rate_hz": 1_000_000_001},
        {"sample_rate_hz": True},
        {"block_samples": -1},
        {"block_samples": 1.5},
        {"channel_offsets_m": ()},
        {"channel_offsets_m": ((0, 0),)},
        {"channel_offsets_m": ((0, 0, float("inf")),)},
        {"speed_of_sound_mps": 0},
        {"speed_of_sound_mps": float("nan")},
        {"sensitivity_volts_per_pa": -1, "calibration_metadata": {"id": "sim"}},
        {"sensitivity_volts_per_pa": 1, "calibration_metadata": {"bad": float("nan")}},
        {"sensitivity_volts_per_pa": 1, "calibration_metadata": "not-an-object"},
        {"calibration_metadata": {"id": "sim"}},
    ],
)
def test_invalid_microphone_configuration_is_rejected(changes):
    with pytest.raises(ValueError):
        _config(**changes)


@pytest.mark.parametrize(
    "changes",
    [
        {"session_id": "other"},
        {"model_generation": 2},
        {"reset_generation": 1},
        {"sim_time_ns": 1},
    ],
)
def test_source_geometry_must_match_sampling_generation_and_source_time(changes):
    with pytest.raises(ValueError, match=r"source.*stamp"):
        MicrophoneSensor(_config()).sample(_stamp(), (_source(stamp=_stamp(**changes)),))


@pytest.mark.parametrize(
    "options",
    [
        {"channel_positions_world_m": ((0, 0, 0), (1, 0, 0))},
        {"position_world_m": (0, float("nan"), 0)},
        {"blocked_paths": (("unknown", 0),)},
        {"blocked_paths": (("speaker", -1),)},
        {"blocked_paths": (("speaker", 1),)},
        {"blocked_paths": (("speaker", True),)},
    ],
)
def test_invalid_geometry_or_blocked_path_references_are_rejected(options):
    with pytest.raises(ValueError):
        MicrophoneSensor(_config()).sample(_stamp(), (_source(),), **options)


def test_duplicate_source_ids_and_missing_source_metadata_are_errors():
    sensor = MicrophoneSensor(_config())
    with pytest.raises(ValueError, match="source_id"):
        sensor.sample(_stamp(), (_source(), _source()))
    with pytest.raises(ValueError, match="AcousticSource"):
        sensor.sample(_stamp(), ({"pressure_pa": [1]},))


def test_pa_noise_is_replayable_chunk_invariant_and_has_requested_rms():
    noise = MeasurementNoise(unit="Pa", enabled=True, stddev=0.2, seed=73)
    config = _config(sample_rate_hz=44100, block_samples=4097, noise=noise, channel_offsets_m=((0, 0, 0), (1, 0, 0)))
    whole = MicrophoneSensor(config).sample(_stamp(), ())
    np.testing.assert_array_equal(whole.pressure_pa, MicrophoneSensor(config).sample(_stamp(), ()).pressure_pa)
    first = MicrophoneSensor(replace(config, block_samples=1500)).sample(_stamp(), ())
    second = MicrophoneSensor(replace(config, block_samples=2597)).sample(
        _stamp(1500 * 1_000_000_000 // 44100, sequence=1),
        (),
    )
    np.testing.assert_array_equal(np.concatenate([first.pressure_pa, second.pressure_pa], axis=1), whole.pressure_pa)
    assert np.sqrt(np.mean(whole.pressure_pa**2)) == pytest.approx(0.2, rel=0.04)
    assert abs(np.corrcoef(whole.pressure_pa)[0, 1]) < 0.05
    assert whole.metadata["noise"] == noise.to_dict()
    changed = MicrophoneSensor(config).sample(_stamp(reset_generation=1), ())
    assert not np.array_equal(whole.pressure_pa, changed.pressure_pa)
    disabled = MicrophoneSensor(replace(config, noise=replace(noise, enabled=False))).sample(_stamp(), ())
    np.testing.assert_array_equal(disabled.pressure_pa, np.zeros_like(whole.pressure_pa))
    assert disabled.metadata["noise"]["stddev"] == 0.2
    with pytest.raises(ValueError, match="Pa"):
        _config(noise=MeasurementNoise(unit="m", enabled=False, stddev=0.2))


@pytest.mark.parametrize("calibrated", [False, True])
def test_finite_input_that_overflows_float32_is_not_reported_as_valid(calibrated):
    options = (
        {"sensitivity_volts_per_pa": 1e40, "calibration_metadata": {"calibration_id": "sim"}} if calibrated else {}
    )
    source = _source(pressure_pa=[1 if calibrated else 1e40], position_world_m=(0, 0, 0))
    with pytest.raises(ValueError, match="float32"):
        MicrophoneSensor(_config(**options)).sample(_stamp(), (source,))


@pytest.mark.parametrize("second_pressure,expected", [(1.0, 2.0), (-1.0, 0.0)])
def test_coincident_sources_add_pressure_with_sign(second_pressure, expected):
    sources = (
        _source(source_id="first", pressure_pa=[1.0], position_world_m=(0, 0, 0)),
        _source(source_id="second", pressure_pa=[second_pressure], position_world_m=(0, 0, 0)),
    )
    frame = MicrophoneSensor(_config()).sample(_stamp(), sources)
    np.testing.assert_array_equal(frame.pressure_pa, [[expected, 0, 0, 0, 0, 0]])


def test_source_owns_a_read_only_copy_even_when_input_is_read_only():
    original = np.array([1.0, -2.0, 3.0])
    original.setflags(write=False)
    source = _source(pressure_pa=original)
    original.setflags(write=True)
    original[:] = 9
    np.testing.assert_array_equal(source.pressure_pa, [1, -2, 3])
    assert source.pressure_pa.flags.owndata
    assert not source.pressure_pa.flags.writeable
    other = _source(pressure_pa=source.pressure_pa)
    source.pressure_pa.setflags(write=True)
    source.pressure_pa[:] = 4
    np.testing.assert_array_equal(other.pressure_pa, [1, -2, 3])
    np.testing.assert_array_equal(original, [9, 9, 9])
    original[0] = np.nan
    original.setflags(write=False)
    with pytest.raises(ValueError, match="finite"):
        _source(pressure_pa=original)


@pytest.mark.parametrize("waveform", [[2.0], [3.0, -7.0, 0.0, 9.0, -4.0]])
@pytest.mark.parametrize("delay", [0.0, 0.25, 0.999999, 1.0, 2.5])
@pytest.mark.parametrize("first_sample", [0, 3, 19, 100_000])
@pytest.mark.parametrize("source_rate", [500, 1500, 44100])
def test_block_interpolation_matches_full_clip_reference_at_boundaries(waveform, delay, first_sample, source_rate):
    stamp = _stamp(first_sample * 1_000_000)
    source = _source(
        stamp=stamp, pressure_pa=waveform, sample_rate_hz=source_rate,
        start_sim_time_ns=1_000_000, position_world_m=(delay, 0, 0),
    )
    indices = (first_sample + np.arange(13)) * (source_rate / 1000) - source_rate / 1000 - delay * source_rate / 1000
    expected = np.interp(indices, np.arange(len(waveform) + 1), np.pad(waveform, (0, 1)), left=0, right=0)
    expected /= max(delay, 1.0)
    frame = MicrophoneSensor(_config(block_samples=13)).sample(stamp, (source,))
    np.testing.assert_array_equal(frame.pressure_pa[0], expected.astype(np.float32))
    assert frame.valid_mask.all()


def test_validated_clip_can_be_shared_without_copying_or_trusting_plain_arrays():
    original = np.array([1.0, -2.0, 3.0])
    clip = AcousticClip(original)
    first = _source(pressure_pa=clip)
    second = _source(pressure_pa=clip, stamp=_stamp(1_000_000))
    assert first.pressure_pa is second.pressure_pa is clip.pressure_pa
    original[:] = np.nan
    np.testing.assert_array_equal(first.pressure_pa, [1, -2, 3])
    with pytest.raises(ValueError):
        clip.pressure_pa.setflags(write=True)
    with pytest.raises(ValueError, match="finite"):
        AcousticClip(original)


def test_sampling_working_memory_is_bounded_by_block_not_clip_length():
    source = _source(pressure_pa=np.ones(400_000), position_world_m=(0, 0, 0))
    sensor = MicrophoneSensor(_config(block_samples=80))
    tracemalloc.start()
    try:
        frame = sensor.sample(_stamp(), (source,))
        _, peak_bytes = tracemalloc.get_traced_memory()
    finally:
        tracemalloc.stop()
    np.testing.assert_array_equal(frame.pressure_pa, np.ones((1, 80)))
    assert peak_bytes < source.pressure_pa.nbytes

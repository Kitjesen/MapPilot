"""Canonical Python/native layout with typed units and explicit identities."""

import json
import struct
from dataclasses import replace

import numpy as np
import pytest
from sim.runtime.sensors.camera_geometry import PinholeCalibration

from tests.sim.test_measurement_payload import _acoustic, _semantic, _thermal


@pytest.mark.parametrize("kind", [1, 2, 3])
def test_measurement_ltu1_packet_preserves_shape_stamp_and_raw_values(kind):
    from sim.scripts.mujoco.measurement_wire import encode_measurement

    frame = {1: _thermal, 2: _acoustic, 3: lambda: _semantic(PinholeCalibration(2, 2, 80, 80, 1, 1))}[kind]()
    record = encode_measurement(frame, product_session_id="product-session")
    outer = struct.unpack("<4sB3xQIII", record.header)
    fields = struct.unpack_from("<4sHH6I4Q5H2x", record.payload)
    magic, version, actual_kind, header_size, meta_size, rows, cols, fs, flags = fields[:9]
    assert (outer[0], outer[1], outer[2], outer[3], outer[4]) == (
        b"LTU1",
        9,
        frame.stamp.sim_time_ns,
        frame.stamp.sequence,
        rows * cols,
    )
    assert (magic, version, actual_kind) == (b"LTM1", 1, kind)
    assert fields[9:13] == (2, 3, 7, 1_000_000_000)
    assert record.payload[74:76] == b"\0\0"
    identifiers, cursor = [], 76
    for length in fields[13:18]:
        identifiers.append(record.payload[cursor : cursor + length].decode())
        cursor += length
    assert cursor == header_size
    assert identifiers == [
        "product-session",
        frame.stamp.session_id,
        frame.stamp.instance_id,
        frame.stamp.sensor_id,
        frame.stamp.frame_id,
    ]
    metadata = json.loads(record.payload[header_size : header_size + meta_size])
    assert metadata["stamp"]["sensor_id"] == frame.stamp.sensor_id
    assert (fs, flags) == ((8000, 1) if kind == 2 else (0, 0))
    cursor += meta_size
    primary = np.frombuffer(
        record.payload, dtype="<u4" if kind == 3 else "<f4", count=rows * cols, offset=cursor
    ).reshape(rows, cols)
    expected = frame.temperature_k if kind == 1 else frame.pressure_pa if kind == 2 else frame.class_ids
    np.testing.assert_array_equal(primary, expected)
    mask = np.frombuffer(record.payload, dtype=np.uint8, count=rows * cols, offset=cursor + 4 * rows * cols)
    np.testing.assert_array_equal(mask.reshape(rows, cols), frame.valid_mask)
    assert metadata["units"]["temperature_k" if kind == 1 else "pressure_pa" if kind == 2 else "class_ids"] == (
        "K" if kind == 1 else "Pa" if kind == 2 else "label"
    )


@pytest.mark.parametrize("identity", ["", " padded", "bad\0name", "x" * 256])
def test_product_identity_must_fit_exact_wire_contract(identity):
    from sim.scripts.mujoco.measurement_wire import encode_measurement

    with pytest.raises(ValueError):
        encode_measurement(_thermal(), product_session_id=identity)


def test_unrepresentable_float32_is_rejected_not_published_as_valid_infinity():
    from sim.scripts.mujoco.measurement_wire import encode_measurement

    sample = _acoustic(calibrated=False)
    pressure = sample.pressure_pa.astype(np.float64)
    pressure[sample.valid_mask] = 1e100
    with pytest.raises(ValueError, match="float32"):
        encode_measurement(replace(sample, pressure_pa=pressure), product_session_id="product-session")


def test_wire_does_not_run_npz_compression(monkeypatch):
    from sim.scripts.mujoco.measurement_wire import encode_measurement

    def forbidden(*args, **kwargs):
        raise AssertionError("online wire must not allocate/compress an NPZ archive")

    monkeypatch.setattr(np, "savez_compressed", forbidden)
    assert encode_measurement(_thermal(), product_session_id="product-session").record_type == 9

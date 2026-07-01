"""Round-trip + size-budget tests for the gateway binary cloud codec."""

from __future__ import annotations

import json

import numpy as np
import pytest

from runtime.utils.binary_codec import (
    FLAG_HAS_COLOR,
    HEADER_SIZE,
    MAGIC,
    decode_pointcloud,
    encode_pointcloud,
)


def _rand_cloud(n: int, seed: int = 0) -> np.ndarray:
    rng = np.random.default_rng(seed)
    return rng.uniform(-50.0, 50.0, size=(n, 3)).astype(np.float32)


def test_header_layout_is_stable():
    buf = encode_pointcloud(_rand_cloud(128))
    assert buf[:4] == MAGIC
    assert buf[4] == 1                 # version
    assert buf[5] == 0                 # flags (no color)
    # 28 bytes header + 128 * 6 bytes payload
    assert len(buf) == HEADER_SIZE + 128 * 6


def test_round_trip_within_quantization_error():
    pts = _rand_cloud(2_000)
    buf = encode_pointcloud(pts, scale=0.005)
    decoded, colors = decode_pointcloud(buf)
    assert colors is None
    assert decoded.shape == pts.shape
    # 5 mm scale → max error half a step plus float epsilon.
    assert np.max(np.abs(decoded - pts)) < 0.005


def test_color_channel_round_trips():
    pts = _rand_cloud(64)
    rgb = np.tile(np.arange(64, dtype=np.uint8)[:, None], (1, 3))
    buf = encode_pointcloud(pts, colors=rgb)
    assert buf[5] & FLAG_HAS_COLOR
    decoded, colors = decode_pointcloud(buf)
    assert colors is not None
    np.testing.assert_array_equal(colors, rgb)
    assert np.max(np.abs(decoded - pts)) < 0.005


def test_empty_cloud_yields_header_only():
    buf = encode_pointcloud(np.empty((0, 3), dtype=np.float32))
    assert len(buf) == HEADER_SIZE
    decoded, _ = decode_pointcloud(buf)
    assert decoded.shape == (0, 3)


def test_binary_is_dramatically_smaller_than_json():
    pts = _rand_cloud(60_000)
    binary = encode_pointcloud(pts)
    json_bytes = json.dumps(
        {"type": "map_cloud", "points": pts.flatten().tolist()},
    ).encode()
    # Binary should be at least 3x smaller than JSON for a 60k cloud.
    assert len(binary) * 3 < len(json_bytes), (
        f"binary={len(binary)} json={len(json_bytes)}"
    )


def test_bad_magic_raises():
    with pytest.raises(ValueError):
        decode_pointcloud(b"XXXX" + b"\x00" * (HEADER_SIZE - 4))

import struct

import numpy as np
import pytest
from tools.datasets.elevator_bag_to_ltu1 import LTU_POINT
from tools.datasets.superloc_corridor_to_ltu1 import pointcloud_payload


def cloud_bytes(times):
    fields = [("x", 0, 7), ("y", 4, 7), ("z", 8, 7),
              ("intensity", 12, 7), ("ring", 16, 4), ("time", 18, 7)]
    points = np.zeros(3, dtype=np.dtype(dict(
        names=[v[0] for v in fields], formats=["<f4"] * 4 + ["<u2", "<f4"],
        offsets=[v[1] for v in fields], itemsize=22)))
    points["x"] = [1, 2, 3]
    points["intensity"] = [11, 22, 33]
    points["ring"] = [15, 0, 8]
    points["time"] = times
    frame = b"cmu_rc1_velodyne"
    raw = struct.pack("<IIII", 17, 1645999726, 278098106, len(frame)) + frame
    raw += struct.pack("<III", 1, 3, len(fields))
    for name, offset, kind in fields:
        raw += struct.pack("<I", len(name)) + name.encode() + struct.pack("<IBI", offset, kind, 1)
    raw += struct.pack("<BIII", 0, 22, 66, 66) + points.tobytes() + b"\x01"
    return raw, points


def test_velodyne_carrier_keeps_high_rings_and_actual_relative_point_times():
    raw, original = cloud_bytes([.1, 0, .05])
    header, payload = pointcloud_payload(raw)
    result = np.frombuffer(payload, dtype=LTU_POINT)

    assert header["header_timestamp_ns"] == 1645999726278098106
    assert header["ring_counts"] == {"0": 1, "8": 1, "15": 1}
    assert not header["original_time_ordered"]
    np.testing.assert_array_equal(result["x"], original["x"][[1, 2, 0]])
    np.testing.assert_array_equal(result["intensity"], original["intensity"][[1, 2, 0]])
    expected = np.rint(original["time"].astype(np.float64) * 1e9).astype(np.uint32)
    np.testing.assert_array_equal(result["offset_time_ns"], np.sort(expected))
    assert len(result) == 3
    assert np.all(result["line"] == 0) and np.all(result["tag"] == 0)


@pytest.mark.parametrize("bad_time", [-.01, float("nan"), 1.1])
def test_carrier_rejects_missing_or_different_point_time_meaning(bad_time):
    raw, _ = cloud_bytes([0, bad_time, .05])
    with pytest.raises(ValueError, match="relative seconds"):
        pointcloud_payload(raw)

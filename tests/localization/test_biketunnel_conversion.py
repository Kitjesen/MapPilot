import struct

import numpy as np
from tools.datasets.biketunnel_to_ltu1 import pointcloud_payload
from tools.datasets.elevator_bag_to_ltu1 import LTU_POINT


def test_absolute_nanosecond_points_preserve_geometry_time_and_lines():
    # A rounded double can place the first point just before the integer header.
    stamp = 1_770_376_960_500_283_957
    fields = [("x", 0, 7), ("y", 4, 7), ("z", 8, 7), ("intensity", 12, 7),
              ("tag", 16, 2), ("line", 17, 2), ("timestamp", 18, 8)]
    dtype = np.dtype({"names": [v[0] for v in fields],
                      "formats": ["<f4"] * 4 + ["u1", "u1", "<f8"],
                      "offsets": [v[1] for v in fields], "itemsize": 26})
    points = np.zeros(2, dtype=dtype)
    points["x"] = [1.5, -2.25]
    points["y"] = [2.5, 3.75]
    points["z"] = [.5, -.5]
    points["intensity"] = [9., 255.]
    points["line"] = [0, 3]
    points["tag"] = [0, 0x10]
    points["timestamp"] = [float(stamp), float(stamp + 99_000_000)]
    frame = b"livox_lidar"
    raw = struct.pack("<IIII", 12, stamp // 1_000_000_000, stamp % 1_000_000_000, len(frame)) + frame
    raw += struct.pack("<III", 1, 2, len(fields))
    for name, off, kind in fields:
        raw += struct.pack("<I", len(name)) + name.encode() + struct.pack("<IBI", off, kind, 1)
    raw += struct.pack("<BIII", 0, 26, 52, 52) + points.tobytes() + b"\x01"
    header, payload, _ = pointcloud_payload(raw)
    decoded = np.frombuffer(payload, dtype=LTU_POINT)
    for name in ("x", "y", "z", "intensity", "line", "tag"):
        np.testing.assert_array_equal(decoded[name], points[name])
    represented_absolute = np.rint(points["timestamp"]).astype(np.int64)
    np.testing.assert_array_equal(header["header_timestamp_ns"] + decoded["offset_time_ns"].astype(np.int64),
                                  represented_absolute)
    assert -128 <= header["header_shift_ns"] <= 0

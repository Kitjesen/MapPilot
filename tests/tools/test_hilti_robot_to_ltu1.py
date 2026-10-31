"""Robot BPearl conversion preserves measured timing and organized geometry."""
import struct

import numpy as np
import pytest
from tools.datasets.hilti_robot_to_ltu1 import FIELDS, SOURCE_POINT, normalized_cloud, seconds_to_ns


def cloud_message(points, *, height=1, padding=0):
    width = len(points) // height
    header = struct.pack("<IIII", 7, 1680000000, 900000000, 7) + b"rslidar"
    fields = b""
    for name, (offset, datatype, count) in FIELDS.items():
        encoded = name.encode()
        fields += struct.pack("<I", len(encoded)) + encoded + struct.pack("<IBI", offset, datatype, count)
    row_step = width * 26 + padding
    data = b"".join(points[row * width:(row + 1) * width].tobytes() + b"\xff" * padding
                    for row in range(height))
    return (header + struct.pack("<III", height, width, len(FIELDS)) + fields
            + struct.pack("<?III", False, 26, row_step, len(data)) + data + b"\x00")


def source_points():
    points = np.zeros(4, dtype=SOURCE_POINT)
    points["x"] = [1, 2, 3, 4]
    points["ring"] = [0, 7, 16, 31]
    points["timestamp"] = 1680000000 + np.array([.3, .1, .2, .4])
    return points


def test_organized_rows_padding_and_stable_timestamp_sort():
    points = source_points()
    info, stamp, output = normalized_cloud(cloud_message(points, height=2, padding=13))
    assert info["row_step"] == 65
    np.testing.assert_array_equal(output["x"], [2, 3, 1, 4])
    np.testing.assert_array_equal(stamp + output["offset_time_ns"].astype(np.int64),
                                  np.sort(seconds_to_ns(points["timestamp"])))
    assert output["line"].max() == output["tag"].max() == output["flags"].max() == 0
    assert info["source_ring_counts"][31] == 1


def test_nan_endpoint_preserves_real_scan_end_without_fabricating_return():
    points = source_points()
    points["x"][[0, 3]] = np.nan
    info, stamp, output = normalized_cloud(cloud_message(points))
    assert info["filtered_nonfinite_geometry"] == 2
    assert info["valid_geometry_points"] == 2
    assert info["timing_only_nan_endpoint"] == 1
    assert len(output) == 3
    assert np.isnan(output["x"][-1])
    assert stamp + int(output["offset_time_ns"][-1]) == info["source_max_point_ns"]
    np.testing.assert_array_equal(output["x"][:-1], [2, 3])


def test_non_endpoint_nan_is_removed_without_marker():
    points = source_points()
    points["z"][0] = np.nan
    info, _, output = normalized_cloud(cloud_message(points))
    assert info["timing_only_nan_endpoint"] == 0
    assert len(output) == 3
    assert np.isfinite(output["x"]).all() and np.isfinite(output["z"]).all()


def test_epoch_conversion_keeps_float64_fraction_precision():
    values = np.array([1680000000.000001, 1680000000.9999998])
    expected = [1680000000000000954, 1680000000999999762]
    np.testing.assert_array_equal(seconds_to_ns(values), expected)


@pytest.mark.parametrize("field,value", [("timestamp", np.nan), ("ring", 32)])
def test_invalid_measured_time_or_ring_rejected(field, value):
    points = source_points()
    points[field][0] = value
    with pytest.raises(ValueError):
        normalized_cloud(cloud_message(points))


def test_all_nan_cloud_rejected():
    points = source_points()
    points["x"] = np.nan
    with pytest.raises(ValueError, match="no finite geometry"):
        normalized_cloud(cloud_message(points))

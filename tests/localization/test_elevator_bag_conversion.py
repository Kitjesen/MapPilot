import bz2
import ctypes
import struct
from pathlib import Path

import numpy as np
import pytest
from tools.datasets.elevator_bag_to_ltu1 import (
    HEADER,
    IMU,
    IMU_TOPIC,
    LIDAR_TOPIC,
    LIVOX_POINT,
    LTU_POINT,
    Ros1Bag,
    _decompress_lz4,
    _lz4_library,
    convert_bag,
)


def fields(values):
    parts = []
    for key, value in values.items():
        entry = key.encode() + b"=" + value
        parts.append(struct.pack("<I", len(entry)) + entry)
    return b"".join(parts)


def record(values, data):
    header = fields(values)
    return struct.pack("<I", len(header)) + header + struct.pack("<I", len(data)) + data


def connection(number, topic, msgtype):
    return record(
        {"op": b"\x07", "conn": struct.pack("<I", number), "topic": topic.encode()},
        fields({"type": msgtype.encode(), "message_definition": b"fixture definition"}),
    )


def sensor_header(seconds=100, nanos=123):
    return struct.pack("<IIII", 9, seconds, nanos, 5) + b"livox"


def message(number, nanos, data):
    return record({"op": b"\x02", "conn": struct.pack("<I", number),
                   "time": struct.pack("<II", 100, nanos)}, data)


def fixture_bag(path: Path, compression: str):
    points = np.array([(900, 1.0, 2.0, 3.0, 255, 0x10, 3),
                       (100, 4.0, 5.0, 6.0, 12, 0x30, 7)], dtype=LIVOX_POINT)
    cloud = sensor_header() + struct.pack("<QIBBBBI", 100_000_000_120, 2, 4, 0, 1, 2, 2) + points.tobytes()
    imu = bytearray(296)
    struct.pack_into("<ddd", imu, 104, 0.1, 0.2, 0.3)
    struct.pack_into("<ddd", imu, 200, 0.0, 0.0, 1.0)
    contents = connection(0, LIDAR_TOPIC, "livox_ros_driver2/CustomMsg")
    contents += message(0, 200, cloud)
    contents += message(1, 300, sensor_header(nanos=456) + imu)
    contents += message(2, 350, b"\x01")
    payload = bz2.compress(contents) if compression == "bz2" else contents
    if compression == "lz4":
        payload = compress_lz4(contents)
    chunk = record({"op": b"\x05", "compression": compression.encode(),
                    "size": struct.pack("<I", len(contents))}, payload)
    # LiDAR's connection is inside the chunk; IMU/Bool connections follow it.
    path.write_bytes(b"#ROSBAG V2.0\n" + chunk
                     + connection(1, IMU_TOPIC, "sensor_msgs/Imu")
                     + connection(2, "/LIO/set_elevator_flag", "std_msgs/Bool"))
    return points


def compress_lz4(payload):
    try:
        library = _lz4_library()
    except RuntimeError as error:
        pytest.skip(str(error))
    library.LZ4F_compressFrameBound.argtypes = [ctypes.c_size_t, ctypes.c_void_p]
    library.LZ4F_compressFrameBound.restype = ctypes.c_size_t
    library.LZ4F_compressFrame.argtypes = [ctypes.c_void_p, ctypes.c_size_t,
                                        ctypes.c_void_p, ctypes.c_size_t, ctypes.c_void_p]
    library.LZ4F_compressFrame.restype = ctypes.c_size_t
    size = library.LZ4F_compressFrameBound(len(payload), None)
    output = ctypes.create_string_buffer(size)
    used = library.LZ4F_compressFrame(output, size, payload, len(payload), None)
    assert not library.LZ4F_isError(used)
    return output.raw[:used]


@pytest.mark.parametrize("compression", ["none", "bz2", "lz4"])
def test_chunk_connection_positions_and_exact_sensor_fields(tmp_path, compression):
    bag = tmp_path / "fixture.bag"
    points = fixture_bag(bag, compression)
    result = convert_bag(bag, tmp_path / "converted")

    with (tmp_path / "converted/fixture.ltu").open("rb") as stream:
        header = HEADER.unpack(stream.read(HEADER.size))
        assert header == (b"LTU1", 1, 100_000_000_123, 0, 2, 48)
        output = np.frombuffer(stream.read(header[-1]), dtype=LTU_POINT)
        for field in ("x", "y", "z", "tag", "line"):
            assert np.array_equal(output[field], points[field])
        assert output["offset_time_ns"].tolist() == [900, 100]
        assert output["intensity"].tolist() == [255.0, 12.0]
        assert output["flags"].tolist() == [0, 0]
        header = HEADER.unpack(stream.read(HEADER.size))
        assert header == (b"LTU1", 2, 100_000_000_456, 1, 1, 24)
        assert IMU.unpack(stream.read(24)) == pytest.approx((0.1, 0.2, 0.3, 0, 0, 1))
        assert stream.read() == b""
    assert result["counts"]["clouds_with_unsorted_offsets"] == 1
    assert result["timebase_minus_header_ns"] == {"min": -3, "max": -3}
    assert result["chunk_compression_counts"] == {compression: 1}
    assert result["recorded_manual_events"] == [{"timestamp_ns": 100_000_000_350, "active": True, "topic": "/LIO/set_elevator_flag"}]
    assert (tmp_path / "converted/fixture.events.tsv").read_text() == "100000000350\t1\n"


def test_truncated_real_layout_chunk_is_rejected(tmp_path):
    bag = tmp_path / "fixture.bag"
    fixture_bag(bag, "none")
    raw = bag.read_bytes()
    # Cut inside the chunk payload, not at a valid record boundary.
    bag.write_bytes(raw[:150])
    with pytest.raises(ValueError, match="truncated ROS1 bag record"):
        list(Ros1Bag(bag).messages())


def test_lz4_chunk_requires_complete_frame_and_expected_length():
    original = b"real bag messages" * 10000
    payload = compress_lz4(original)
    assert _decompress_lz4(payload, len(original)) == original
    with pytest.raises(ValueError, match=r"incomplete|mismatch"):
        _decompress_lz4(payload[:-1], len(original))
    with pytest.raises(ValueError, match=r"incomplete|mismatch"):
        _decompress_lz4(payload, len(original) + 1)

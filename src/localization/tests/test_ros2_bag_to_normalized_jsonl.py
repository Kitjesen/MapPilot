from __future__ import annotations

import importlib.util
import json
import struct
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "datasets" / "ros2_bag_to_normalized_jsonl.py"
ROS_MODULES_BEFORE_IMPORT = {
    name for name in sys.modules if name == "rclpy" or name.startswith("rclpy.") or name == "rosbag2_py"
}
SPEC = importlib.util.spec_from_file_location("ros2_bag_to_normalized_jsonl", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
CONVERTER = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = CONVERTER
SPEC.loader.exec_module(CONVERTER)


def _manifest(
    tmp_path: Path,
    *,
    point_time_field: str | None = "t",
    point_time_unit: str | None = "us",
    source_path: str | None = None,
) -> Path:
    path = tmp_path / "manifest.json"
    path.write_text(
        json.dumps(
            {
                "schema_version": "lingtu.slam.replay.v1",
                "source": {
                    "path": str(tmp_path / "bag") if source_path is None else source_path,
                    "format": "ros2_bag",
                    "conversion_adapter": "ros2_bag_to_normalized_jsonl",
                },
                "streams": {
                    "lidar": {
                        "source_topic": "/livox/lidar",
                        "source_message_type": "sensor_msgs/msg/PointCloud2",
                        "point_time": {"field": point_time_field, "unit": point_time_unit},
                    },
                    "imu": {
                        "source_topic": "/livox/imu",
                        "source_message_type": "sensor_msgs/msg/Imu",
                    },
                },
            }
        ),
        encoding="utf-8",
    )
    return path


def _field(name: str, offset: int, datatype: int, count: int = 1) -> SimpleNamespace:
    return SimpleNamespace(name=name, offset=offset, datatype=datatype, count=count)


def _stamp(timestamp_ns: int) -> SimpleNamespace:
    return SimpleNamespace(sec=timestamp_ns // 1_000_000_000, nanosec=timestamp_ns % 1_000_000_000)


def _imu(timestamp_ns: int) -> SimpleNamespace:
    return SimpleNamespace(
        header=SimpleNamespace(stamp=_stamp(timestamp_ns)),
        angular_velocity=SimpleNamespace(x=0.1, y=0.2, z=0.3),
        linear_acceleration=SimpleNamespace(x=1.0, y=2.0, z=9.81),
    )


def _pointcloud2(
    *,
    timestamp_ns: int,
    is_bigendian: bool = False,
    row_padding: bytes = b"",
    offsets: tuple[int, int] = (10, 20),
) -> SimpleNamespace:
    endian = ">" if is_bigendian else "<"
    point_struct = struct.Struct(endian + "ffffH")
    data = bytearray()
    data.extend(point_struct.pack(1.0, 2.0, 3.0, 42.0, offsets[0]))
    data.extend(point_struct.pack(4.0, 5.0, 6.0, 43.0, offsets[1]))
    data.extend(row_padding)
    return SimpleNamespace(
        header=SimpleNamespace(stamp=_stamp(timestamp_ns)),
        height=1,
        width=2,
        fields=[
            _field("x", 0, 7),
            _field("y", 4, 7),
            _field("z", 8, 7),
            _field("intensity", 12, 7),
            _field("t", 16, 4),
        ],
        is_bigendian=is_bigendian,
        point_step=18,
        row_step=36 + len(row_padding),
        data=bytes(data),
    )


def _records(imu_time: int = 1_000, cloud_time: int = 2_000) -> list:
    return [
        CONVERTER.BagRecord("/livox/imu", "sensor_msgs/msg/Imu", _imu(imu_time), imu_time),
        CONVERTER.BagRecord(
            "/livox/lidar",
            "sensor_msgs/msg/PointCloud2",
            _pointcloud2(timestamp_ns=cloud_time, row_padding=b"pad!"),
            cloud_time,
        ),
    ]


def _read_jsonl(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]


def test_module_has_no_import_time_ros_dependency() -> None:
    ros_modules_after_import = {
        name for name in sys.modules if name == "rclpy" or name.startswith("rclpy.") or name == "rosbag2_py"
    }
    assert ros_modules_after_import == ROS_MODULES_BEFORE_IMPORT


def test_lazily_reports_missing_ros_dependencies(monkeypatch: pytest.MonkeyPatch) -> None:
    real_import = __import__

    def fake_import(name, *args, **kwargs):
        if name == "rosbag2_py":
            raise ImportError(name)
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr("builtins.__import__", fake_import)

    with pytest.raises(CONVERTER.ConversionError, match="rosbag2_py"):
        list(CONVERTER.iter_ros2_bag_records("missing", {"/livox/lidar"}))


def test_bag_reader_accepts_storage_id_without_rclpy_context(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    calls: dict[str, object] = {}

    class FakeStorageOptions:
        def __init__(self, *, uri: str, storage_id: str) -> None:
            calls["storage"] = (uri, storage_id)

    class FakeConverterOptions:
        def __init__(self, *, input_serialization_format: str, output_serialization_format: str) -> None:
            calls["converter"] = (input_serialization_format, output_serialization_format)

    class FakeReader:
        def open(self, storage_options, converter_options) -> None:
            calls["open"] = (storage_options, converter_options)

        def get_all_topics_and_types(self) -> list:
            return []

        def has_next(self) -> bool:
            return False

    fake_rosbag2 = SimpleNamespace(
        SequentialReader=FakeReader,
        StorageOptions=FakeStorageOptions,
        ConverterOptions=FakeConverterOptions,
    )
    monkeypatch.setattr(
        CONVERTER,
        "_load_ros2_bag_dependencies",
        lambda: (fake_rosbag2, lambda *_: None, lambda *_: None),
    )

    assert list(CONVERTER.iter_ros2_bag_records(tmp_path / "bag", {"/livox/lidar"}, storage_id="mcap")) == []
    assert calls["storage"] == (str(tmp_path / "bag"), "mcap")


def test_converts_pointcloud2_and_imu_to_normalized_jsonl(tmp_path: Path) -> None:
    manifest = CONVERTER.load_manifest(_manifest(tmp_path))
    output = tmp_path / "normalized.jsonl"

    stats = CONVERTER._write_jsonl_atomically(_records(), output, manifest)

    assert stats.records == 2
    assert stats.imu_records == 1
    assert stats.cloud_records == 1
    assert stats.points == 2
    payloads = _read_jsonl(output)
    assert payloads[0] == {
        "type": "imu",
        "timestamp_ns": 1_000,
        "sequence": 0,
        "gyro": [0.1, 0.2, 0.3],
        "acc": [1.0, 2.0, 9.81],
    }
    assert payloads[1]["timestamp_ns"] == 2_000
    assert payloads[1]["transport_only"] is False
    assert payloads[1]["points"] == [
        {"x": 1.0, "y": 2.0, "z": 3.0, "intensity": 42.0, "offset_time_ns": 10_000},
        {"x": 4.0, "y": 5.0, "z": 6.0, "intensity": 43.0, "offset_time_ns": 20_000},
    ]


def test_pointcloud2_parser_honors_big_endian_and_row_step(tmp_path: Path) -> None:
    manifest = CONVERTER.load_manifest(_manifest(tmp_path, point_time_unit="ns"))
    cloud = _pointcloud2(timestamp_ns=2_000, is_bigendian=True, row_padding=b"ignored", offsets=(7, 9))

    payload, point_count, undeskewed = CONVERTER.normalize_pointcloud2(
        cloud,
        timestamp_ns=2_000,
        sequence=0,
        point_time_field="t",
        point_time_unit="ns",
        allow_undeskewed=False,
    )

    assert point_count == 2
    assert undeskewed is False
    assert payload["points"][0] == {"x": 1.0, "y": 2.0, "z": 3.0, "intensity": 42.0, "offset_time_ns": 7}
    assert payload["points"][1] == {"x": 4.0, "y": 5.0, "z": 6.0, "intensity": 43.0, "offset_time_ns": 9}
    assert manifest.lidar.topic == "/livox/lidar"


def test_pointcloud2_rejects_field_that_crosses_point_boundary() -> None:
    cloud = _pointcloud2(timestamp_ns=2_000)
    cloud.fields.append(_field("bad", 17, 4))

    with pytest.raises(CONVERTER.ConversionError, match="exceeds point_step"):
        CONVERTER.normalize_pointcloud2(
            cloud,
            timestamp_ns=2_000,
            sequence=0,
            point_time_field="t",
            point_time_unit="ns",
            allow_undeskewed=False,
        )


def test_pointcloud2_rejects_non_positive_field_count() -> None:
    cloud = _pointcloud2(timestamp_ns=2_000)
    cloud.fields[0] = _field("x", 0, 7, count=0)

    with pytest.raises(CONVERTER.ConversionError, match="count"):
        CONVERTER.normalize_pointcloud2(
            cloud,
            timestamp_ns=2_000,
            sequence=0,
            point_time_field="t",
            point_time_unit="ns",
            allow_undeskewed=False,
        )


def test_pointcloud2_struct_errors_are_reported_as_conversion_errors() -> None:
    with pytest.raises(CONVERTER.ConversionError, match="failed to unpack"):
        CONVERTER._field_value(b"\x00", 0, _field("x", 0, 7), endian="<", label="PointCloud2.x")


def test_missing_point_time_is_rejected_by_default(tmp_path: Path) -> None:
    manifest = CONVERTER.load_manifest(_manifest(tmp_path, point_time_field=None, point_time_unit=None))
    output = tmp_path / "normalized.jsonl"

    with pytest.raises(CONVERTER.ConversionError, match="point time field"):
        CONVERTER._write_jsonl_atomically(_records(), output, manifest)

    assert not output.exists()


def test_allow_undeskewed_marks_transport_only_and_zero_offsets(tmp_path: Path) -> None:
    manifest = CONVERTER.load_manifest(_manifest(tmp_path, point_time_field=None, point_time_unit=None))
    output = tmp_path / "normalized.jsonl"

    stats = CONVERTER._write_jsonl_atomically(_records(), output, manifest, allow_undeskewed=True)

    assert stats.undeskewed_cloud_records == 1
    assert stats.undeskewed_points == 2
    cloud = _read_jsonl(output)[1]
    assert cloud["transport_only"] is True
    assert [point["offset_time_ns"] for point in cloud["points"]] == [0, 0]


def test_rejects_non_monotonic_record_timestamps_without_replacing_output(tmp_path: Path) -> None:
    manifest = CONVERTER.load_manifest(_manifest(tmp_path))
    output = tmp_path / "normalized.jsonl"
    output.write_text("known-good\n", encoding="utf-8")

    with pytest.raises(CONVERTER.ConversionError, match="timestamps must be monotonic"):
        CONVERTER._write_jsonl_atomically(_records(imu_time=2_000, cloud_time=1_000), output, manifest)

    assert output.read_text(encoding="utf-8") == "known-good\n"


def test_requires_at_least_one_cloud_and_one_imu(tmp_path: Path) -> None:
    manifest = CONVERTER.load_manifest(_manifest(tmp_path))
    output = tmp_path / "normalized.jsonl"

    with pytest.raises(CONVERTER.ConversionError, match="at least one cloud"):
        CONVERTER._write_jsonl_atomically(
            [CONVERTER.BagRecord("/livox/imu", "sensor_msgs/msg/Imu", _imu(1_000), 1_000)],
            output,
            manifest,
        )


def test_rejects_cloud_records_without_any_points(tmp_path: Path) -> None:
    manifest = CONVERTER.load_manifest(_manifest(tmp_path))
    output = tmp_path / "normalized.jsonl"
    empty_cloud = _pointcloud2(timestamp_ns=2_000)
    empty_cloud.width = 0
    empty_cloud.row_step = 0
    empty_cloud.data = b""

    with pytest.raises(CONVERTER.ConversionError, match="at least one point"):
        CONVERTER._write_jsonl_atomically(
            [
                CONVERTER.BagRecord(
                    "/livox/imu",
                    "sensor_msgs/msg/Imu",
                    _imu(1_000),
                    1_000,
                ),
                CONVERTER.BagRecord(
                    "/livox/lidar",
                    "sensor_msgs/msg/PointCloud2",
                    empty_cloud,
                    2_000,
                ),
            ],
            output,
            manifest,
        )

    assert not output.exists()


def test_livox_custom_msg_is_supported(tmp_path: Path) -> None:
    manifest_path = _manifest(tmp_path)
    data = json.loads(manifest_path.read_text(encoding="utf-8"))
    data["streams"]["lidar"]["source_message_type"] = "livox_ros_driver2/msg/CustomMsg"
    data["streams"]["lidar"]["point_time"] = {"field": "offset_time", "unit": "us"}
    manifest_path.write_text(json.dumps(data), encoding="utf-8")
    manifest = CONVERTER.load_manifest(manifest_path)
    output = tmp_path / "normalized.jsonl"
    livox = SimpleNamespace(
        header=SimpleNamespace(stamp=_stamp(2_000)),
        timebase=123_000,
        points=[
            SimpleNamespace(x=1.0, y=2.0, z=3.0, reflectivity=11, offset_time=5, tag=1, line=2),
        ],
    )

    stats = CONVERTER._write_jsonl_atomically(
        [
            CONVERTER.BagRecord("/livox/imu", "sensor_msgs/msg/Imu", _imu(1_000), 1_000),
            CONVERTER.BagRecord("/livox/lidar", "livox_ros_driver2/msg/CustomMsg", livox, 2_000),
        ],
        output,
        manifest,
    )

    assert stats.cloud_records == 1
    cloud = _read_jsonl(output)[1]
    assert cloud["timestamp_ns"] == 123_000
    assert cloud["points"] == [
        {
            "x": 1.0,
            "y": 2.0,
            "z": 3.0,
            "intensity": 11.0,
            "offset_time_ns": 5_000,
            "tag": 1,
            "line": 2,
        }
    ]


def test_livox_manifest_requires_offset_time_semantics(tmp_path: Path) -> None:
    manifest_path = _manifest(tmp_path)
    data = json.loads(manifest_path.read_text(encoding="utf-8"))
    data["streams"]["lidar"]["source_message_type"] = "livox_ros_driver2/msg/CustomMsg"
    data["streams"]["lidar"]["point_time"] = {"field": None, "unit": None}
    manifest_path.write_text(json.dumps(data), encoding="utf-8")

    with pytest.raises(CONVERTER.ConversionError, match="offset_time"):
        CONVERTER.load_manifest(manifest_path)


def test_rejects_lidar_topic_type_family_mismatch(tmp_path: Path) -> None:
    manifest = CONVERTER.load_manifest(_manifest(tmp_path))
    livox = SimpleNamespace(timebase=2_000, points=[])

    with pytest.raises(CONVERTER.ConversionError, match="does not match manifest"):
        CONVERTER._write_jsonl_atomically(
            [
                CONVERTER.BagRecord("/livox/imu", "sensor_msgs/msg/Imu", _imu(1_000), 1_000),
                CONVERTER.BagRecord("/livox/lidar", "livox_ros_driver2/msg/CustomMsg", livox, 2_000),
            ],
            tmp_path / "normalized.jsonl",
            manifest,
        )


def test_manifest_validation_rejects_wrong_schema(tmp_path: Path) -> None:
    manifest = _manifest(tmp_path)
    data = json.loads(manifest.read_text(encoding="utf-8"))
    data["schema_version"] = "wrong"
    manifest.write_text(json.dumps(data), encoding="utf-8")

    with pytest.raises(CONVERTER.ConversionError, match="schema_version"):
        CONVERTER.load_manifest(manifest)


def test_manifest_validation_rejects_missing_source_path(tmp_path: Path) -> None:
    with pytest.raises(CONVERTER.ConversionError, match=r"source\.path"):
        CONVERTER.load_manifest(_manifest(tmp_path, source_path=""))


def test_manifest_validation_rejects_non_ros2_or_wrong_adapter(tmp_path: Path) -> None:
    manifest = _manifest(tmp_path)
    data = json.loads(manifest.read_text(encoding="utf-8"))
    data["source"]["format"] = "ros1_bag"
    manifest.write_text(json.dumps(data), encoding="utf-8")

    with pytest.raises(CONVERTER.ConversionError, match=r"source\.format"):
        CONVERTER.load_manifest(manifest)

    data["source"]["format"] = "ros2_bag"
    data["source"]["conversion_adapter"] = "some_other_adapter"
    manifest.write_text(json.dumps(data), encoding="utf-8")

    with pytest.raises(CONVERTER.ConversionError, match="conversion_adapter"):
        CONVERTER.load_manifest(manifest)


def test_convert_manifest_requires_existing_source_path(tmp_path: Path) -> None:
    manifest = _manifest(tmp_path, source_path=str(tmp_path / "missing_bag"))

    with pytest.raises(CONVERTER.ConversionError, match="does not exist"):
        CONVERTER.convert_manifest(manifest, tmp_path / "normalized.jsonl")

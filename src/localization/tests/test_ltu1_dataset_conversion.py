from __future__ import annotations

import importlib.util
import json
import struct
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "datasets" / "normalized_lidar_imu_to_ltu1.py"
SPEC = importlib.util.spec_from_file_location("normalized_lidar_imu_to_ltu1", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
CONVERTER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CONVERTER)

HEADER = struct.Struct("<4sB3xQIII")
POINT = struct.Struct("<ffffIBBH")
IMU = struct.Struct("<ffffff")


def _write_jsonl(path: Path, records: list[dict]) -> None:
    path.write_text(
        "".join(json.dumps(record) + "\n" for record in records),
        encoding="utf-8",
    )


def test_convert_normalized_jsonl_to_exact_ltu1_records(tmp_path: Path) -> None:
    source = tmp_path / "normalized.jsonl"
    output = tmp_path / "sensors.ltu"
    _write_jsonl(
        source,
        [
            {
                "type": "imu",
                "timestamp_ns": 1_000,
                "gyro": [0.1, -0.2, 0.3],
                "acc": [0.0, 0.0, 9.80665],
            },
            {
                "type": "cloud",
                "timestamp_ns": 2_000,
                "points": [
                    {
                        "x": 1.0,
                        "y": 2.0,
                        "z": 3.0,
                        "intensity": 42.0,
                        "offset_time_ns": 17,
                        "tag": 2,
                        "line": 3,
                        "flags": 4,
                    }
                ],
            },
        ],
    )

    stats = CONVERTER.convert_jsonl(source, output)

    assert stats.records == 2
    assert stats.imu_records == 1
    assert stats.cloud_records == 1
    assert stats.points == 1

    data = output.read_bytes()
    first_header = HEADER.unpack_from(data, 0)
    assert first_header == (b"LTU1", 2, 1_000, 0, 1, IMU.size)
    assert IMU.unpack_from(data, HEADER.size) == pytest.approx((0.1, -0.2, 0.3, 0.0, 0.0, 9.80665))

    second_offset = HEADER.size + IMU.size
    second_header = HEADER.unpack_from(data, second_offset)
    assert second_header == (b"LTU1", 1, 2_000, 1, 1, POINT.size)
    point = POINT.unpack_from(data, second_offset + HEADER.size)
    assert point[:4] == pytest.approx((1.0, 2.0, 3.0, 42.0))
    assert point[4:] == (17, 2, 3, 4)


def test_converter_requires_per_point_time_for_frontend_replay(tmp_path: Path) -> None:
    source = tmp_path / "missing_point_time.jsonl"
    output = tmp_path / "sensors.ltu"
    _write_jsonl(
        source,
        [
            {
                "type": "imu",
                "timestamp_ns": 1_000,
                "gyro": [0.0, 0.0, 0.0],
                "acc": [0.0, 0.0, 9.81],
            },
            {
                "type": "cloud",
                "timestamp_ns": 2_000,
                "points": [{"x": 1.0, "y": 2.0, "z": 3.0}],
            },
        ],
    )

    with pytest.raises(CONVERTER.ConversionError, match="offset_time_ns"):
        CONVERTER.convert_jsonl(source, output)

    assert not output.exists()


def test_explicit_undeskewed_mode_is_marked_in_conversion_stats(tmp_path: Path) -> None:
    source = tmp_path / "transport_only.jsonl"
    output = tmp_path / "transport_only.ltu"
    _write_jsonl(
        source,
        [
            {
                "type": "imu",
                "timestamp_ns": 1_000,
                "gyro": [0.0, 0.0, 0.0],
                "acc": [0.0, 0.0, 9.81],
            },
            {
                "type": "cloud",
                "timestamp_ns": 2_000,
                "points": [{"x": 1.0, "y": 2.0, "z": 3.0}],
            },
        ],
    )

    stats = CONVERTER.convert_jsonl(
        source,
        output,
        allow_undeskewed=True,
    )

    assert stats.undeskewed_points == 1
    point = POINT.unpack_from(
        output.read_bytes(),
        HEADER.size + IMU.size + HEADER.size,
    )
    assert point[4] == 0


def test_transport_only_marker_cannot_bypass_explicit_undeskewed_gate(tmp_path: Path) -> None:
    source = tmp_path / "transport_only_marked.jsonl"
    output = tmp_path / "transport_only_marked.ltu"
    _write_jsonl(
        source,
        [
            {
                "type": "imu",
                "timestamp_ns": 1_000,
                "gyro": [0.0, 0.0, 0.0],
                "acc": [0.0, 0.0, 9.81],
            },
            {
                "type": "cloud",
                "timestamp_ns": 2_000,
                "transport_only": True,
                "points": [
                    {
                        "x": 1.0,
                        "y": 2.0,
                        "z": 3.0,
                        "offset_time_ns": 0,
                    }
                ],
            },
        ],
    )

    with pytest.raises(CONVERTER.ConversionError, match="transport_only"):
        CONVERTER.convert_jsonl(source, output)

    stats = CONVERTER.convert_jsonl(source, output, allow_undeskewed=True)
    assert stats.undeskewed_points == 1


def test_converter_rejects_replay_without_both_sensor_streams(tmp_path: Path) -> None:
    source = tmp_path / "cloud_only.jsonl"
    output = tmp_path / "cloud_only.ltu"
    _write_jsonl(
        source,
        [
            {
                "type": "cloud",
                "timestamp_ns": 2_000,
                "points": [
                    {
                        "x": 1.0,
                        "y": 2.0,
                        "z": 3.0,
                        "offset_time_ns": 0,
                    }
                ],
            }
        ],
    )

    with pytest.raises(CONVERTER.ConversionError, match="at least one IMU"):
        CONVERTER.convert_jsonl(source, output)

    assert not output.exists()


def test_converter_rejects_cloud_stream_without_any_points(tmp_path: Path) -> None:
    source = tmp_path / "empty_cloud.jsonl"
    output = tmp_path / "empty_cloud.ltu"
    _write_jsonl(
        source,
        [
            {
                "type": "imu",
                "timestamp_ns": 1_000,
                "gyro": [0.0, 0.0, 0.0],
                "acc": [0.0, 0.0, 9.81],
            },
            {
                "type": "cloud",
                "timestamp_ns": 2_000,
                "points": [],
            },
        ],
    )

    with pytest.raises(CONVERTER.ConversionError, match="at least one point"):
        CONVERTER.convert_jsonl(source, output)

    assert not output.exists()


def test_failed_conversion_does_not_replace_existing_output(tmp_path: Path) -> None:
    source = tmp_path / "backwards.jsonl"
    output = tmp_path / "sensors.ltu"
    output.write_bytes(b"known-good")
    _write_jsonl(
        source,
        [
            {
                "type": "imu",
                "timestamp_ns": 2_000,
                "gyro": [0.0, 0.0, 0.0],
                "acc": [0.0, 0.0, 9.81],
            },
            {
                "type": "imu",
                "timestamp_ns": 1_000,
                "gyro": [0.0, 0.0, 0.0],
                "acc": [0.0, 0.0, 9.81],
            },
        ],
    )

    with pytest.raises(CONVERTER.ConversionError, match="monotonic"):
        CONVERTER.convert_jsonl(source, output)

    assert output.read_bytes() == b"known-good"


def test_converter_reports_values_outside_exact_float32_range(tmp_path: Path) -> None:
    source = tmp_path / "float32_overflow.jsonl"
    output = tmp_path / "sensors.ltu"
    _write_jsonl(
        source,
        [
            {
                "type": "imu",
                "timestamp_ns": 1_000,
                "gyro": [0.0, 0.0, 0.0],
                "acc": [0.0, 0.0, 9.81],
            },
            {
                "type": "cloud",
                "timestamp_ns": 2_000,
                "points": [
                    {
                        "x": 3.4028235e38,
                        "y": 0.0,
                        "z": 0.0,
                        "offset_time_ns": 0,
                    }
                ],
            },
        ],
    )

    with pytest.raises(CONVERTER.ConversionError, match="float32 range"):
        CONVERTER.convert_jsonl(source, output)

    assert not output.exists()

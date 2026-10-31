"""Host measurement seam uses local LTM1 fixtures, not simulator imports."""

import json
import struct
import subprocess
import sys
from dataclasses import FrozenInstanceError
from pathlib import Path

import numpy as np
import pytest

_HEADER = struct.Struct("<4sHH6I4Q5H2x")
_IDS = ("product-run", "source-session", "robot", "thermal", "optical")


def _fixture(kind=1, *, voltage=True):
    stamp = dict(zip(("session_id", "instance_id", "sensor_id", "frame_id"), _IDS[1:]))
    stamp.update(model_generation=2, reset_generation=3, sequence=7, sim_time_ns=1_000_000_000)
    arrays = {
        "temperature_k": np.array([[320, np.nan], [280, 300]], dtype="<f4"),
        "valid_mask": np.array([[True, False], [True, True]]),
    }
    metadata = {
        "schema": "lingtu.sim.measurement-payload.v1",
        "wire_encoding": "ltm1-le-v1",
        "modality": "thermal",
        "stamp": stamp,
        "units": {"temperature_k": "K", "valid_mask": "1"},
        "arrays": {name: {"dtype": a.dtype.str, "shape": list(a.shape)} for name, a in arrays.items()},
        "frame_metadata": {
            "camera_calibration": {"model": "pinhole", "width": 2, "height": 2},
            "temperature_unit": "K",
            "source_stamp": dict(stamp),
            "qualification": "simulation, not calibrated hardware",
        },
        "metadata_tuple_paths": [],
    }
    if kind == 2:
        arrays["pressure_pa"] = np.array([[0.2, np.nan], [-0.1, 0.3]], dtype="<f4")
        del arrays["temperature_k"]
        if voltage:
            arrays["voltage_v"] = arrays["pressure_pa"] * np.float32(0.01)
        metadata.update(modality="acoustic", sample_rate_hz=8000)
        metadata["units"] = {"pressure_pa": "Pa", "valid_mask": "1"}
        if voltage:
            metadata["units"]["voltage_v"] = "V"
        metadata["frame_metadata"] = {
            "pressure_unit": "Pa",
            "calibration": {"id": "sim-mic"} if voltage else None,
            "sensitivity_volts_per_pa": 0.01 if voltage else None,
        }
        metadata["arrays"] = {name: {"dtype": a.dtype.str, "shape": list(a.shape)} for name, a in arrays.items()}
    elif kind == 3:
        del arrays["temperature_k"]
        arrays.update(
            class_ids=np.array([[1, 0], [2, 2]], dtype="<u4"), instance_ids=np.array([[11, 0], [22, 23]], dtype="<u4")
        )
        metadata.update(
            modality="semantic",
            source_stamp=dict(stamp),
            simulation=True,
            ground_truth=True,
            configuration={"calibration": {"width": 2, "height": 2}, "calibration_model": "pinhole"},
            calibration_metadata={"projection": {"width": 2, "height": 2, "model": "pinhole"}},
        )
        del metadata["frame_metadata"]
        metadata["units"] = {"class_ids": "label", "instance_ids": "label", "valid_mask": "1"}
        metadata["arrays"] = {name: {"dtype": a.dtype.str, "shape": list(a.shape)} for name, a in arrays.items()}
    return metadata, arrays


def _payload(metadata, arrays, *, ids=_IDS):
    identifiers = [value.encode("utf-8") for value in ids]
    encoded = json.dumps(metadata, ensure_ascii=False).encode("utf-8")
    stamp = metadata["stamp"]
    kind = {"thermal": 1, "acoustic": 2, "semantic": 3}[metadata["modality"]]
    primary = {1: "temperature_k", 2: "pressure_pa", 3: "class_ids"}[kind]
    rows, columns = arrays[primary].shape
    secondary = "instance_ids" if kind == 3 else "voltage_v" if "voltage_v" in arrays else None
    header = _HEADER.pack(
        b"LTM1",
        1,
        kind,
        76 + sum(map(len, identifiers)),
        len(encoded),
        rows,
        columns,
        metadata.get("sample_rate_hz", 0),
        int(kind == 2 and secondary is not None),
        *(stamp[key] for key in ("model_generation", "reset_generation", "sequence", "sim_time_ns")),
        *map(len, identifiers),
    )
    raw = arrays[primary].tobytes() + arrays["valid_mask"].astype("u1").tobytes()
    if secondary:
        raw += arrays[secondary].tobytes()
    return header + b"".join(identifiers) + encoded + raw


def _decode(payload, **kwargs):
    from runtime.endpoints.dds.measurement import measurement_from_record

    return measurement_from_record(
        payload,
        received_at_s=kwargs.pop("received_at_s", 1_800_000_000.5),
        expected_product_session_id=kwargs.pop("expected_product_session_id", _IDS[0]),
        expected_session_id=kwargs.pop("expected_session_id", _IDS[1]),
        **kwargs,
    )


def test_thermal_record_preserves_native_values_identity_and_source_clock():
    from runtime.msgs.measurement import MeasurementIdentity, ThermalMeasurement

    metadata, arrays = _fixture()
    message = _decode(_payload(metadata, arrays))
    assert isinstance(message, ThermalMeasurement)
    assert message.identity == MeasurementIdentity(*_IDS, 2, 3, 7, 1_000_000_000)
    np.testing.assert_array_equal(message.temperature_k, arrays["temperature_k"])
    np.testing.assert_array_equal(message.valid_mask, arrays["valid_mask"])
    assert message.temperature_k.dtype == np.dtype("<f4")
    assert message.valid_mask.dtype == np.bool_
    assert message.metadata == metadata
    assert message.received_at_s == 1_800_000_000.5
    assert message.captured_at_s is None
    with pytest.raises(FrozenInstanceError):
        message.received_at_s = 1
    with pytest.raises(FrozenInstanceError):
        message.identity.sequence = 8


@pytest.mark.parametrize("binding", ["expected_product_session_id", "expected_session_id"])
@pytest.mark.parametrize("value", ["other-run", "", None])
def test_endpoint_bindings_must_match_exact_nonempty_identities(binding, value):
    metadata, arrays = _fixture()
    with pytest.raises(ValueError, match="session"):
        _decode(_payload(metadata, arrays), **{binding: value})


def _header_change(payload, index, value):
    fields = list(_HEADER.unpack_from(payload))
    fields[index] = value
    return _HEADER.pack(*fields) + payload[76:]


@pytest.mark.parametrize(
    "index,value",
    [
        (0, b"BAD!"),
        (1, 2),
        (2, 0),
        (2, 4),
        (3, 76),
        (4, 0),
        (4, 1_048_577),
        (5, 0),
        (6, 0),
        (5, 0xFFFFFFFF),
        (7, 1),
        (8, 1),
        (8, 2),
        (11, 2**32),
        (12, (2**31) * 1_000_000_000),
        (13, 0),
        (13, 256),
    ],
)
def test_invalid_header_is_rejected_before_array_decode(index, value):
    metadata, arrays = _fixture()
    with pytest.raises(ValueError):
        _decode(_header_change(_payload(metadata, arrays), index, value))


@pytest.mark.parametrize("damage", ["short", "truncated", "trailing", "reserved", "oversized"])
def test_exact_record_size_and_reserved_bytes_are_required(damage):
    metadata, arrays = _fixture()
    payload = _payload(metadata, arrays)
    if damage == "oversized":
        payload = b"x" * (32 * 1024 * 1024 + 1)
    else:
        payload = {
            "short": payload[:75],
            "truncated": payload[:-1],
            "trailing": payload + b"x",
            "reserved": payload[:74] + b"\1\0" + payload[76:],
        }[damage]
    with pytest.raises(ValueError):
        _decode(payload)


@pytest.mark.parametrize("voltage", [False, True])
def test_acoustic_preserves_channel_major_pressure_rate_and_optional_voltage(voltage):
    from runtime.msgs.measurement import AcousticMeasurement

    metadata, arrays = _fixture(2, voltage=voltage)
    message = _decode(_payload(metadata, arrays))
    assert isinstance(message, AcousticMeasurement)
    np.testing.assert_array_equal(message.pressure_pa, arrays["pressure_pa"])
    np.testing.assert_array_equal(message.valid_mask, arrays["valid_mask"])
    assert message.sample_rate_hz == 8000
    assert message.metadata == metadata
    assert message.captured_at_s is None
    if voltage:
        np.testing.assert_array_equal(message.voltage_v, arrays["voltage_v"])
        assert message.voltage_v.dtype == np.dtype("<f4")
    else:
        assert message.voltage_v is None


def test_semantic_preserves_two_native_uint32_label_arrays():
    from runtime.msgs.measurement import SemanticMeasurement

    metadata, arrays = _fixture(3)
    message = _decode(_payload(metadata, arrays))
    assert isinstance(message, SemanticMeasurement)
    for name in ("class_ids", "instance_ids", "valid_mask"):
        np.testing.assert_array_equal(getattr(message, name), arrays[name])
        assert getattr(message, name).dtype == arrays[name].dtype
    assert message.metadata == metadata
    assert message.captured_at_s is None


@pytest.mark.parametrize(
    "kind,name,index,value",
    [
        (1, "temperature_k", (0, 0), np.nan),
        (1, "temperature_k", (0, 0), np.inf),
        (1, "temperature_k", (0, 0), 0),
        (1, "temperature_k", (0, 0), -1),
        (1, "temperature_k", (0, 1), 300),
        (1, "temperature_k", (0, 1), -np.inf),
        (2, "pressure_pa", (0, 0), np.nan),
        (2, "pressure_pa", (0, 1), 0),
        (2, "voltage_v", (0, 0), np.inf),
        (2, "voltage_v", (0, 1), 0),
        (3, "class_ids", (0, 0), 0),
        (3, "instance_ids", (0, 0), 2**31),
        (3, "class_ids", (0, 1), 1),
        (3, "instance_ids", (0, 1), 1),
    ],
)
def test_values_must_obey_the_native_validity_mask(kind, name, index, value):
    metadata, arrays = _fixture(kind)
    arrays[name][index] = value
    with pytest.raises(ValueError, match=r"valid|mask|Kelvin|label"):
        _decode(_payload(metadata, arrays))


@pytest.mark.parametrize("kind", [1, 2, 3])
def test_mask_bytes_are_exactly_zero_or_one(kind):
    metadata, arrays = _fixture(kind)
    payload = _payload(metadata, arrays)
    fields = _HEADER.unpack_from(payload)
    offset = fields[3] + fields[4] + 16
    payload = payload[:offset] + b"\2" + payload[offset + 1 :]
    with pytest.raises(ValueError, match="mask"):
        _decode(payload)


@pytest.mark.parametrize("position", range(5))
@pytest.mark.parametrize("value", ["", " padded", "padded\u3000", "nul\0id", "x" * 256, "热" * 86])
def test_identifiers_are_trimmed_nonempty_utf8_with_byte_limits(position, value):
    metadata, arrays = _fixture()
    ids = list(_IDS)
    ids[position] = value
    with pytest.raises(ValueError):
        _decode(_payload(metadata, arrays, ids=ids))


@pytest.mark.parametrize("kind", [1, 2, 3])
def test_utf8_identifiers_and_metadata_are_losslessly_retained(kind):
    metadata, arrays = _fixture(kind)
    ids = ("产品运行", "源会话", "机器人", "传感器", "光学坐标")
    for stamp in (
        metadata["stamp"],
        metadata.get("source_stamp"),
        metadata.get("frame_metadata", {}).get("source_stamp"),
    ):
        if stamp is not None:
            stamp.update(zip(("session_id", "instance_id", "sensor_id", "frame_id"), ids[1:]))
    metadata["note"] = "温度、声压与物理标签 🚀"
    message = _decode(
        _payload(metadata, arrays, ids=ids), expected_product_session_id=ids[0], expected_session_id=ids[1]
    )
    assert message.identity.sensor_id == ids[3]
    assert message.metadata == metadata


def test_invalid_utf8_is_rejected_in_identifiers_and_json():
    metadata, arrays = _fixture()
    payload = _payload(metadata, arrays)
    for offset in (76, _HEADER.unpack_from(payload)[3]):
        with pytest.raises(ValueError):
            _decode(payload[:offset] + b"\xff" + payload[offset + 1 :])


@pytest.mark.parametrize("kind", [1, 2, 3])
@pytest.mark.parametrize(
    "damage",
    [
        "nan",
        "infinity",
        "nested_infinity",
        "units",
        "missing_units",
        "modality",
        "missing_stamp",
        "stamp_type",
        "missing_calibration",
        "empty_calibration",
        "manifest_dtype",
        "manifest_shape",
        "manifest_extra",
        "mask_manifest",
    ],
)
def test_metadata_must_be_finite_and_match_native_arrays_and_declarations(kind, damage):
    metadata, arrays = _fixture(kind)
    primary = {1: "temperature_k", 2: "pressure_pa", 3: "class_ids"}[kind]
    if damage in ("nan", "infinity", "nested_infinity"):
        metadata["extra"] = (
            {"nested": [float("inf")]} if damage == "nested_infinity" else float("nan" if damage == "nan" else "inf")
        )
    elif damage == "units":
        metadata["units"][primary] = "wrong-unit"
    elif damage == "missing_units":
        del metadata["units"]
    elif damage == "modality":
        payload = _payload(metadata, arrays)
        metadata["modality"] = "other"
        with pytest.raises(ValueError):
            _decode(_replace_metadata(payload, json.dumps(metadata).encode()))
        return
    elif damage == "missing_stamp":
        payload = _payload(metadata, arrays)
        del metadata["stamp"]
        with pytest.raises(ValueError):
            _decode(_replace_metadata(payload, json.dumps(metadata).encode()))
        return
    elif damage == "stamp_type":
        metadata["stamp"]["model_generation"] = True
    elif damage in ("missing_calibration", "empty_calibration"):
        container = metadata["configuration"] if kind == 3 else metadata["frame_metadata"]
        key = "camera_calibration" if kind == 1 else "calibration"
        if damage == "missing_calibration":
            del container[key]
        else:
            container[key] = {}
    elif damage == "manifest_dtype":
        metadata["arrays"][primary]["dtype"] = "<f8" if kind != 3 else "<i4"
    elif damage == "manifest_shape":
        metadata["arrays"][primary]["shape"] = [4, 1]
    elif damage == "manifest_extra":
        metadata["arrays"]["undeclared"] = {"dtype": "<f4", "shape": [2, 2]}
    elif damage == "mask_manifest":
        metadata["arrays"]["valid_mask"]["dtype"] = "|u1"
    with pytest.raises(ValueError):
        _decode(_payload(metadata, arrays))


def _replace_metadata(payload, encoded):
    fields = list(_HEADER.unpack_from(payload))
    old_end = fields[3] + fields[4]
    fields[4] = len(encoded)
    return _HEADER.pack(*fields) + payload[76 : fields[3]] + encoded + payload[old_end:]


@pytest.mark.parametrize("raw", [b"[]", b"null", b"{", b'{"overflow":1e999}', b'{"literal":NaN}'])
def test_json_must_be_a_finite_object(raw):
    metadata, arrays = _fixture()
    with pytest.raises(ValueError):
        _decode(_replace_metadata(_payload(metadata, arrays), raw))


@pytest.mark.parametrize(
    "field",
    [
        "session_id",
        "instance_id",
        "sensor_id",
        "frame_id",
        "model_generation",
        "reset_generation",
        "sequence",
        "sim_time_ns",
    ],
)
@pytest.mark.parametrize("location", ["stamp", "source_stamp", "frame_source_stamp"])
def test_every_declared_source_stamp_must_match_the_binary_identity(field, location):
    metadata, arrays = _fixture()
    payload = _payload(metadata, arrays)
    stamp = dict(metadata["stamp"])
    stamp[field] = "other" if isinstance(stamp[field], str) else stamp[field] + 1
    if location == "frame_source_stamp":
        metadata["frame_metadata"]["source_stamp"] = stamp
    else:
        metadata[location] = stamp
    with pytest.raises(ValueError, match="stamp"):
        _decode(_replace_metadata(payload, json.dumps(metadata).encode()))


@pytest.mark.parametrize("kind", [1, 2, 3])
def test_explicit_epoch_mapping_does_not_change_source_or_receipt_clocks(kind):
    metadata, arrays = _fixture(kind)
    for stamp in (
        metadata["stamp"],
        metadata.get("source_stamp"),
        metadata.get("frame_metadata", {}).get("source_stamp"),
    ):
        if stamp is not None:
            stamp.update(sim_time_ns=2**60 + 123, model_generation=2**64 - 1, sequence=2**32 - 1)
    metadata.setdefault("frame_metadata", {})["source_clock"] = {"mapped_epoch_s": 1_700_000_000.25}
    payload = _payload(metadata, arrays)
    first = _decode(payload, received_at_s=1_800_000_000.5)
    later = _decode(payload, received_at_s=1_900_000_000.5)
    assert first.captured_at_s == later.captured_at_s == 1_700_000_000.25
    assert first.identity.sim_time_ns == 2**60 + 123
    assert first.identity.model_generation == 2**64 - 1
    assert first.identity.sequence == 2**32 - 1
    assert first.identity == later.identity
    assert first.received_at_s != later.received_at_s


@pytest.mark.parametrize("value", [None, True, "1700000000", [], {}, float("nan"), float("inf"), 10**400])
def test_declared_epoch_mapping_must_be_a_finite_number(value):
    metadata, arrays = _fixture()
    metadata["frame_metadata"]["source_clock"] = {"mapped_epoch_s": value}
    with pytest.raises(ValueError):
        _decode(_payload(metadata, arrays))


@pytest.mark.parametrize("value", [None, True, "clock", []])
def test_source_clock_declaration_must_be_an_object(value):
    metadata, arrays = _fixture()
    metadata["frame_metadata"]["source_clock"] = value
    with pytest.raises(ValueError):
        _decode(_payload(metadata, arrays))


@pytest.mark.parametrize("value", [None, True, -1, "now", float("nan"), float("inf")])
def test_receipt_time_must_be_explicit_finite_nonnegative_seconds(value):
    metadata, arrays = _fixture()
    with pytest.raises(ValueError, match="received_at_s"):
        _decode(_payload(metadata, arrays), received_at_s=value)


@pytest.mark.parametrize(
    "kind,damage",
    [
        (1, "width"),
        (1, "height"),
        (1, "unit"),
        (1, "null_calibration"),
        (2, "rate"),
        (2, "unit"),
        (2, "missing_voltage"),
        (2, "extra_voltage"),
        (2, "bad_sensitivity"),
        (3, "width"),
        (3, "height"),
        (3, "projection"),
        (1, "float_shape"),
        (2, "float_shape"),
        (3, "float_shape"),
    ],
)
def test_calibration_shape_rate_and_voltage_declarations_match_the_record(kind, damage):
    metadata, arrays = _fixture(kind)
    payload = _payload(metadata, arrays)
    if damage in ("width", "height", "null_calibration"):
        container = metadata["configuration"] if kind == 3 else metadata["frame_metadata"]
        key = "camera_calibration" if kind == 1 else "calibration"
        if damage == "null_calibration":
            container[key] = None
        else:
            container[key][damage] = 3
    elif damage == "projection":
        metadata["calibration_metadata"]["projection"]["width"] = 3
    elif damage == "rate":
        metadata["sample_rate_hz"] += 1
    elif damage == "unit":
        metadata["frame_metadata"]["temperature_unit" if kind == 1 else "pressure_unit"] = "wrong"
    elif damage in ("missing_voltage", "extra_voltage", "bad_sensitivity"):
        frame = metadata["frame_metadata"]
        frame["sensitivity_volts_per_pa"] = (
            None if damage == "extra_voltage" else -0.1 if damage == "bad_sensitivity" else 0.01
        )
        if damage == "missing_voltage":
            del arrays["voltage_v"]
            del metadata["arrays"]["voltage_v"]
            del metadata["units"]["voltage_v"]
            payload = _payload(metadata, arrays)
    elif damage == "float_shape":
        metadata["arrays"]["valid_mask"]["shape"] = [2.0, 2.0]
    with pytest.raises(ValueError):
        _decode(_replace_metadata(payload, json.dumps(metadata).encode()))


@pytest.mark.parametrize("kind", [1, 2, 3])
def test_decoder_runs_without_sim_or_python_dds_packages(kind):
    metadata, arrays = _fixture(kind)
    script = """
import importlib.abc
import sys
sys.path.insert(0, 'src')
class NoSimulatorOrPythonDDS(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'sim', 'cyclonedds', 'pydds'}:
            raise AssertionError('Forbidden runtime dependency: ' + fullname)
sys.meta_path.insert(0, NoSimulatorOrPythonDDS())
from runtime.endpoints.dds.measurement import measurement_from_record
message = measurement_from_record(
    sys.stdin.buffer.read(), received_at_s=1800000000.5,
    expected_product_session_id='product-run', expected_session_id='source-session',
)
assert message.identity.sequence == 7
assert message.captured_at_s is None
assert not any(name.split('.')[0] in {'sim', 'cyclonedds', 'pydds'} for name in sys.modules)
"""
    result = subprocess.run(
        [sys.executable, "-c", script],
        input=_payload(metadata, arrays),
        cwd=Path(__file__).resolve().parents[2],
        capture_output=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stderr.decode(errors="replace")


@pytest.mark.parametrize("kind", [1, 2, 3])
@pytest.mark.parametrize("shape", [(1, 4), (3, 2)])
def test_non_square_shapes_keep_row_or_channel_major_layout(kind, shape):
    metadata, arrays = _fixture(kind)
    for name, array in arrays.items():
        arrays[name] = np.resize(array, shape)
    metadata["arrays"] = {name: {"dtype": a.dtype.str, "shape": list(a.shape)} for name, a in arrays.items()}
    if kind == 1:
        metadata["frame_metadata"]["camera_calibration"].update(height=shape[0], width=shape[1])
    elif kind == 3:
        metadata["configuration"]["calibration"].update(height=shape[0], width=shape[1])
        metadata["calibration_metadata"]["projection"].update(height=shape[0], width=shape[1])
    message = _decode(_payload(metadata, arrays))
    for name, expected in arrays.items():
        actual = getattr(message, name)
        np.testing.assert_array_equal(actual, expected)
        assert actual.shape == shape
        assert not actual.flags.writeable

"""Exercise the sensor-control C ABI without loading DDS or a native library."""

import ctypes
import json
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from threading import Event

import pytest

from message.generated.enums import SensorControlKind
from runtime.contracts.sensor_control import SensorControlRequest, SensorControlResult


class Function:
    def __init__(self, callback):
        self.callback = callback

    def __call__(self, *args):
        return self.callback(*args)


class Library:
    def __init__(self):
        self.created = []
        self.destroyed = []
        self.pending = {"request": None, "result": None}
        self.records = {}
        self.status = {}
        self.lt_sensor_control_create = Function(self.create)
        self.lt_sensor_control_destroy = Function(self.destroyed.append)
        self.lt_sensor_control_last_error = Function(lambda: b"native sensor control failure")
        for kind in ("request", "result"):
            setattr(self, f"lt_sensor_control_take_{kind}", Function(self.take(kind)))
            setattr(self, f"lt_sensor_control_write_{kind}", Function(self.write(kind)))

    def create(self, *args):
        self.created.append(args)
        return 42

    def take(self, kind):
        def call(handle, output):
            assert handle == 42
            status = self.status.get(f"take_{kind}")
            if status is not None:
                return status
            payload = self.pending[kind]
            if payload is None:
                return 0
            ctypes.memmove(output, payload, len(payload))
            self.pending[kind] = None
            return 1

        return call

    def write(self, kind):
        def call(handle, record):
            assert handle == 42
            self.records[kind] = type(record._obj).from_buffer_copy(record._obj)
            self.pending[kind] = bytes(record._obj)
            return self.status.get(f"write_{kind}", 0)

        return call


def request(**changes):
    return SensorControlRequest(
        **{
            "request_id": "request-1",
            "product_session_id": "product",
            "session_id": "source",
            "sensor_id": "thermal",
            "model_generation": 2**64 - 1,
            "reset_generation": 7,
            "deadline_s": 1234.5,
            "operation": SensorControlKind.CONFIGURE_NOISE,
            "enabled": True,
            "rate_hz": 30.0,
            "noise_stddev": 0.1,
            "noise_seed": 2**64 - 1,
            "noise_unit": "degC",
            "pan_deg": -30.0,
            "tilt_deg": 15.0,
            **changes,
        }
    )


def test_request_roundtrip_and_owned_handle_close_once():
    from drivers.sim.measurement.control_bridge import NativeSensorControlBridge

    library = Library()
    with NativeSensorControlBridge(6, "product", "source", False, library=library) as bridge:
        assert library.created == [(6, b"product", b"source", 0)]
        assert bridge.take_request() is None
        expected = request()
        assert bridge.write_request(expected) is None
        assert bridge.take_request() == expected
        assert bridge.take_request() is None
    bridge.close()
    assert library.destroyed == [42]


@pytest.mark.parametrize("operation", list(SensorControlKind))
def test_every_generated_operation_roundtrips_as_enum(operation):
    from drivers.sim.measurement.control_bridge import NativeSensorControlBridge

    with NativeSensorControlBridge(6, "product", "source", True, library=Library()) as bridge:
        expected = request(operation=operation, enabled=False)
        bridge.write_request(expected)
        sample = bridge.take_request()
        assert sample == expected
        assert sample.operation is operation
        assert sample.enabled is False


def result(**changes):
    return SensorControlResult(
        **{
            "request_id": "request-1",
            "product_session_id": "product",
            "session_id": "source",
            "sensor_id": "thermal",
            "model_generation": 2**64 - 1,
            "reset_generation": 7,
            "applied_sim_time_ns": 2**64 - 1,
            "applied": True,
            "reason": "applied",
            "settings": {"enabled": True, "label": "温度", "nested": {"values": [1, 2.5, None]}},
            **changes,
        }
    )


@pytest.mark.parametrize("changes", [{}, {"settings": {}, "applied": False, "reason": ""}])
def test_result_roundtrip_preserves_settings_and_owner_mode(changes):
    from drivers.sim.measurement.control_bridge import NativeSensorControlBridge

    library = Library()
    with NativeSensorControlBridge(6, "product", "source", True, library=library) as bridge:
        assert library.created == [(6, b"product", b"source", 1)]
        assert bridge.take_result() is None
        expected = result(**changes)
        assert bridge.write_result(expected) is None
        assert json.loads(library.records["result"].settings_json) == expected.settings
        assert bridge.take_result() == expected
        assert bridge.take_result() is None


def test_native_layout_and_all_function_signatures():
    from drivers.sim.measurement.control_bridge import NativeSensorControlBridge, _Request, _Result

    expected_request = {
        "request_id": 0,
        "product_session_id": 256,
        "session_id": 512,
        "sensor_id": 768,
        "model_generation": 1024,
        "reset_generation": 1032,
        "deadline_s": 1040,
        "operation": 1048,
        "enabled": 1052,
        "rate_hz": 1056,
        "noise_stddev": 1064,
        "noise_seed": 1072,
        "noise_unit": 1080,
        "pan_deg": 1096,
        "tilt_deg": 1104,
    }
    expected_result = {
        "request_id": 0,
        "product_session_id": 256,
        "session_id": 512,
        "sensor_id": 768,
        "model_generation": 1024,
        "reset_generation": 1032,
        "applied_sim_time_ns": 1040,
        "applied": 1048,
        "reason": 1052,
        "settings_json": 1308,
    }
    for record_type, size, offsets in ((_Request, 1112, expected_request), (_Result, 9504, expected_result)):
        assert ctypes.sizeof(record_type) == size
        assert ctypes.alignment(record_type) == 8
        assert {name: getattr(record_type, name).offset for name, _ in record_type._fields_} == offsets
    library = Library()
    with NativeSensorControlBridge(6, "product", "source", True, library=library):
        assert library.lt_sensor_control_create.argtypes == [
            ctypes.c_int,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_int,
        ]
        assert library.lt_sensor_control_create.restype is ctypes.c_void_p
        assert library.lt_sensor_control_destroy.argtypes == [ctypes.c_void_p]
        assert library.lt_sensor_control_destroy.restype is None
        assert library.lt_sensor_control_last_error.argtypes == []
        assert library.lt_sensor_control_last_error.restype is ctypes.c_char_p
        for kind, record_type in (("request", _Request), ("result", _Result)):
            for action in ("take", "write"):
                function = getattr(library, f"lt_sensor_control_{action}_{kind}")
                assert function.argtypes == [ctypes.c_void_p, ctypes.POINTER(record_type)]
                assert function.restype is ctypes.c_int


@pytest.mark.parametrize(
    "changes",
    [{"domain_id": value} for value in (-1, 233, True, 1.0, "6", None)]
    + [
        {name: value}
        for name in ("product_session_id", "session_id")
        for value in ("", " leading", "trailing ", "bad\0text", "é" * 128, None)
    ]
    + [{"owner_mode": value} for value in (0, 1, None, "owner")],
)
def test_constructor_validates_before_loading_dll(monkeypatch, changes):
    from drivers.sim.measurement import control_bridge

    def unexpected_load():
        pytest.fail("library resolution must not happen for invalid constructor arguments")

    monkeypatch.setattr(control_bridge, "_library_path", unexpected_load)
    with pytest.raises(ValueError):
        control_bridge.NativeSensorControlBridge(
            **{
                "domain_id": 6,
                "product_session_id": "product",
                "session_id": "source",
                "owner_mode": True,
                **changes,
            }
        )


@pytest.mark.parametrize("domain", [0, 232])
def test_domain_boundaries(domain):
    from drivers.sim.measurement.control_bridge import NativeSensorControlBridge

    library = Library()
    with NativeSensorControlBridge(domain, "product", "source", False, library=library):
        assert library.created[0][0] == domain


@pytest.mark.parametrize("kind", ["request", "result"])
def test_exact_utf8_bounds_are_preserved(kind):
    from drivers.sim.measurement.control_bridge import NativeSensorControlBridge

    library = Library()
    with NativeSensorControlBridge(6, "product", "source", True, library=library) as bridge:
        changes = {"request_id": "é" * 127 + "x", "sensor_id": "s" * 255}
        if kind == "request":
            sample = request(noise_unit="é" * 7 + "x", **changes)
        else:
            sample = result(reason="é" * 127 + "x", settings={"v": "x" * 8183}, **changes)
        getattr(bridge, f"write_{kind}")(sample)
        assert getattr(bridge, f"take_{kind}")() == sample


@pytest.mark.parametrize(
    "changes",
    [
        {"request_id": ""},
        {"sensor_id": "s" * 256},
        {"request_id": "é" * 128},
        {"reason": "r" * 256},
        {"reason": "bad\0text"},
        {"reason": "é" * 128},
        {"settings": {"v": "x" * 8184}},
        {"settings": {"v": "é" * 4092}},
        {"model_generation": -1},
        {"reset_generation": 2**64},
        {"applied_sim_time_ns": True},
        {"applied": 1},
        {"settings": []},
        {"settings": {"v": float("nan")}},
        {"settings": {"nested": [float("inf")]}},
        {"settings": {"v": float("-inf")}},
        {"settings": {1: "not a string key"}},
        {"settings": {"tuple": (1, 2)}},
        {"settings": {"v": object()}},
    ],
)
def test_result_rejects_lossy_or_invalid_data_before_native_write(changes):
    from drivers.sim.measurement.control_bridge import NativeSensorControlBridge

    library = Library()
    with NativeSensorControlBridge(6, "product", "source", True, library=library) as bridge:
        with pytest.raises(ValueError):
            bridge.write_result(result(**changes))
        assert not library.records


@pytest.mark.parametrize("kind", ["request", "result"])
@pytest.mark.parametrize("field", ["product_session_id", "session_id"])
def test_foreign_bindings_are_not_written_or_relabelled(kind, field):
    from drivers.sim.measurement.control_bridge import NativeSensorControlBridge

    library = Library()
    with NativeSensorControlBridge(6, "product", "source", True, library=library) as bridge:
        factory = request if kind == "request" else result
        with pytest.raises(ValueError, match="bound"):
            getattr(bridge, f"write_{kind}")(factory(**{field: "foreign"}))
        assert not library.records
        getattr(bridge, f"write_{kind}")(factory())
        record = library.records[kind]
        setattr(record, field, b"foreign")
        library.pending[kind] = bytes(record)
        with pytest.raises(ValueError, match="bound"):
            getattr(bridge, f"take_{kind}")()


@pytest.mark.parametrize(
    "payload",
    [
        "",
        "{",
        "[]",
        "null",
        "1",
        '{"v":NaN}',
        '{"v":Infinity}',
        '{"v":-Infinity}',
        '{"v":1e999}',
        '{"nested":[-1e999]}',
    ],
)
def test_native_settings_must_decode_to_finite_dictionary(payload):
    from drivers.sim.measurement.control_bridge import NativeSensorControlBridge

    library = Library()
    with NativeSensorControlBridge(6, "product", "source", False, library=library) as bridge:
        bridge.write_result(result())
        record = library.records["result"]
        record.settings_json = payload.encode()
        library.pending["result"] = bytes(record)
        with pytest.raises(ValueError):
            bridge.take_result()


@pytest.mark.parametrize(
    "kind,field,value",
    [
        ("request", "enabled", 2),
        ("request", "operation", 999),
        ("request", "rate_hz", float("nan")),
        ("request", "deadline_s", float("inf")),
        ("request", "noise_unit", b"\xff"),
        ("result", "applied", -1),
        ("result", "reason", b"\xff"),
    ],
)
def test_invalid_native_fields_are_rejected(kind, field, value):
    from drivers.sim.measurement.control_bridge import NativeSensorControlBridge

    library = Library()
    with NativeSensorControlBridge(6, "product", "source", True, library=library) as bridge:
        getattr(bridge, f"write_{kind}")((request if kind == "request" else result)())
        record = library.records[kind]
        setattr(record, field, value)
        library.pending[kind] = bytes(record)
        with pytest.raises(ValueError):
            getattr(bridge, f"take_{kind}")()


@pytest.mark.parametrize("kind,field,capacity", [("request", "noise_unit", 16), ("result", "reason", 256)])
def test_unterminated_native_strings_are_rejected(kind, field, capacity):
    from drivers.sim.measurement.control_bridge import NativeSensorControlBridge

    library = Library()
    with NativeSensorControlBridge(6, "product", "source", True, library=library) as bridge:
        getattr(bridge, f"write_{kind}")((request if kind == "request" else result)())
        record = library.records[kind]
        ctypes.memmove(ctypes.addressof(record) + getattr(type(record), field).offset, b"x" * capacity, capacity)
        library.pending[kind] = bytes(record)
        with pytest.raises(ValueError, match="NUL-terminated"):
            getattr(bridge, f"take_{kind}")()


@pytest.mark.parametrize("method", ["take_request", "take_result", "write_request", "write_result"])
@pytest.mark.parametrize("status", [-1, 2, -2])
def test_native_errors_and_unexpected_statuses_are_not_success(method, status):
    from drivers.sim.measurement.control_bridge import NativeSensorControlBridge, SensorControlBridgeError

    library = Library()
    library.status[method] = status
    with NativeSensorControlBridge(6, "product", "source", True, library=library) as bridge:
        args = () if method.startswith("take") else ((request if method.endswith("request") else result)(),)
        with pytest.raises(SensorControlBridgeError, match="failure" if status == -1 else "invalid status"):
            getattr(bridge, method)(*args)


@pytest.mark.parametrize("method", ["take_request", "take_result", "write_request", "write_result"])
def test_closed_handle_rejects_every_operation(method):
    from drivers.sim.measurement.control_bridge import NativeSensorControlBridge, SensorControlBridgeError

    library = Library()
    bridge = NativeSensorControlBridge(6, "product", "source", True, library=library)
    bridge.close()
    bridge.close()
    args = () if method.startswith("take") else ((request if method.endswith("request") else result)(),)
    with pytest.raises(SensorControlBridgeError, match="closed"):
        getattr(bridge, method)(*args)
    assert library.destroyed == [42]


def test_failed_create_does_not_destroy_null_handle():
    from drivers.sim.measurement.control_bridge import NativeSensorControlBridge, SensorControlBridgeError

    library = Library()
    library.lt_sensor_control_create = Function(lambda *args: None)
    with pytest.raises(SensorControlBridgeError, match="native sensor control failure"):
        NativeSensorControlBridge(6, "product", "source", True, library=library)
    assert library.destroyed == []


def test_context_exception_closes_handle_and_preserves_exception():
    from drivers.sim.measurement.control_bridge import NativeSensorControlBridge

    library = Library()
    with pytest.raises(ValueError, match="body failed"):
        with NativeSensorControlBridge(6, "product", "source", True, library=library):
            raise ValueError("body failed")
    assert library.destroyed == [42]


def test_close_waits_for_inflight_native_call():
    from drivers.sim.measurement.control_bridge import NativeSensorControlBridge

    library = Library()
    entered, release, closing, destroyed = Event(), Event(), Event(), Event()

    def take(handle, output):
        entered.set()
        assert release.wait(5)
        assert not destroyed.is_set()
        return 0

    def destroy(handle):
        library.destroyed.append(handle)
        destroyed.set()

    library.lt_sensor_control_take_request = Function(take)
    library.lt_sensor_control_destroy = Function(destroy)
    bridge = NativeSensorControlBridge(6, "product", "source", True, library=library)

    def close():
        closing.set()
        bridge.close()

    with ThreadPoolExecutor(max_workers=2) as pool:
        taking = pool.submit(bridge.take_request)
        try:
            assert entered.wait(5)
            closed = pool.submit(close)
            assert closing.wait(5)
            assert not destroyed.wait(0.05)
        finally:
            release.set()
        assert taking.result(timeout=5) is None
        closed.result(timeout=5)
    assert library.destroyed == [42]


@pytest.mark.skipif(os.name != "nt", reason="Windows DLL search directories")
@pytest.mark.parametrize("fail_at", [None, "load", "bind", "create"])
def test_dll_search_directory_is_retained_and_cleaned_on_failure(monkeypatch, fail_at):
    from drivers.sim.measurement import control_bridge

    library = Library()
    closed, loaded = [], []
    path = Path("D:/native/lingtu_sensor_control_bridge.dll")

    class Directory:
        def close(self):
            closed.append("directory")

    def add_directory(directory):
        assert directory == str(path.parent)
        return Directory()

    def load(filename):
        loaded.append(filename)
        if fail_at == "load":
            raise OSError("load failed")
        if fail_at == "bind":
            del library.lt_sensor_control_write_result
        if fail_at == "create":
            library.lt_sensor_control_create = Function(lambda *args: None)
        return library

    monkeypatch.setattr(control_bridge, "_library_path", lambda: path)
    monkeypatch.setattr(control_bridge.os, "add_dll_directory", add_directory)
    monkeypatch.setattr(control_bridge.ctypes, "CDLL", load)
    if fail_at is None:
        with control_bridge.NativeSensorControlBridge(6, "product", "source", True) as bridge:
            assert closed == []
            assert bridge.take_request() is None
        bridge.close()
        assert library.destroyed == [42]
    else:
        with pytest.raises((OSError, AttributeError, control_bridge.SensorControlBridgeError)):
            control_bridge.NativeSensorControlBridge(6, "product", "source", True)
        assert library.destroyed == []
    assert loaded == [str(path)]
    assert closed == ["directory"]


@pytest.mark.skipif(os.name != "nt", reason="Windows Release DLL candidates")
@pytest.mark.parametrize("candidate", ["sensor-control-bridge-win", "windows-native-dds-adapter"])
def test_default_library_candidates(monkeypatch, candidate):
    from drivers.sim.measurement import control_bridge

    monkeypatch.delenv("LINGTU_SENSOR_CONTROL_BRIDGE_LIBRARY", raising=False)
    expected = (
        Path(control_bridge.__file__).resolve().parents[4]
        / "build"
        / candidate
        / "Release"
        / "lingtu_sensor_control_bridge.dll"
    )
    monkeypatch.setattr(Path, "is_file", lambda path: path == expected)
    assert control_bridge._library_path() == expected.resolve()


@pytest.mark.parametrize("exists", [True, False])
def test_explicit_library_environment_is_authoritative(monkeypatch, exists):
    from drivers.sim.measurement import control_bridge

    expected = Path("explicit") / "lingtu_sensor_control_bridge.dll"
    monkeypatch.setenv("LINGTU_SENSOR_CONTROL_BRIDGE_LIBRARY", str(expected))
    observed = []

    def is_file(path):
        observed.append(path)
        return exists

    monkeypatch.setattr(Path, "is_file", is_file)
    if exists:
        assert control_bridge._library_path() == expected.resolve()
    else:
        with pytest.raises(control_bridge.SensorControlBridgeError, match="LINGTU_SENSOR_CONTROL_BRIDGE_LIBRARY"):
            control_bridge._library_path()
    assert observed == [expected]


def test_destroy_failure_still_marks_handle_closed_once():
    from drivers.sim.measurement.control_bridge import NativeSensorControlBridge

    library = Library()

    def destroy(handle):
        library.destroyed.append(handle)
        raise RuntimeError("destroy failed")

    library.lt_sensor_control_destroy = Function(destroy)
    bridge = NativeSensorControlBridge(6, "product", "source", True, library=library)
    with pytest.raises(RuntimeError, match="destroy failed"):
        bridge.close()
    bridge.close()
    assert library.destroyed == [42]


def test_import_does_not_require_python_dds():
    root = Path(__file__).resolve().parents[2]
    env = {**os.environ, "PYTHONPATH": str(root / "src")}
    script = """
import importlib.abc
import sys
class NoPythonDDS(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'cyclonedds', 'rti', 'fastdds'}:
            raise AssertionError('Python DDS import: ' + fullname)
sys.meta_path.insert(0, NoPythonDDS())
from drivers.sim.measurement.control_bridge import NativeSensorControlBridge
"""
    completed = subprocess.run([sys.executable, "-c", script], env=env, capture_output=True, text=True, timeout=20)
    assert completed.returncode == 0, completed.stdout + completed.stderr

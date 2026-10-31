"""Owned ctypes sensor-control endpoint; no Python DDS runtime."""

from __future__ import annotations

import ctypes
import json
import os
from pathlib import Path
from threading import RLock
from typing import Any

from message.generated.enums import SensorControlKind
from runtime.contracts.sensor_control import SensorControlRequest, SensorControlResult

_ID_CAP = 256
_UNIT_CAP = 16
_REASON_CAP = 256
_SETTINGS_CAP = 8192


class SensorControlBridgeError(RuntimeError):
    """Native endpoint unavailable or violated its C ABI contract."""


class _Request(ctypes.Structure):
    # control_bridge.h uses native alignment, not packed structures.
    _fields_ = [
        ("request_id", ctypes.c_char * _ID_CAP),
        ("product_session_id", ctypes.c_char * _ID_CAP),
        ("session_id", ctypes.c_char * _ID_CAP),
        ("sensor_id", ctypes.c_char * _ID_CAP),
        ("model_generation", ctypes.c_uint64),
        ("reset_generation", ctypes.c_uint64),
        ("deadline_s", ctypes.c_double),
        ("operation", ctypes.c_int32),
        ("enabled", ctypes.c_int32),
        ("rate_hz", ctypes.c_double),
        ("noise_stddev", ctypes.c_double),
        ("noise_seed", ctypes.c_uint64),
        ("noise_unit", ctypes.c_char * _UNIT_CAP),
        ("pan_deg", ctypes.c_double),
        ("tilt_deg", ctypes.c_double),
    ]


class _Result(ctypes.Structure):
    _fields_ = [
        ("request_id", ctypes.c_char * _ID_CAP),
        ("product_session_id", ctypes.c_char * _ID_CAP),
        ("session_id", ctypes.c_char * _ID_CAP),
        ("sensor_id", ctypes.c_char * _ID_CAP),
        ("model_generation", ctypes.c_uint64),
        ("reset_generation", ctypes.c_uint64),
        ("applied_sim_time_ns", ctypes.c_uint64),
        ("applied", ctypes.c_int32),
        ("reason", ctypes.c_char * _REASON_CAP),
        ("settings_json", ctypes.c_char * _SETTINGS_CAP),
    ]


def _text(value: str, name: str, capacity: int, *, identity: bool = False) -> bytes:
    if not isinstance(value, str) or "\0" in value:
        raise ValueError(f"{name} must be NUL-free UTF-8 text")
    if identity and (not value or value != value.strip()):
        raise ValueError(f"{name} must be nonempty trimmed UTF-8 text")
    encoded = value.encode("utf-8")
    if len(encoded) >= capacity:
        raise ValueError(f"{name} exceeds {capacity - 1} UTF-8 bytes")
    return encoded


def _read_text(record: ctypes.Structure, name: str, capacity: int) -> str:
    raw = ctypes.string_at(ctypes.addressof(record) + getattr(type(record), name).offset, capacity)
    if b"\0" not in raw:
        raise ValueError(f"{name} is not NUL-terminated")
    return raw.split(b"\0", 1)[0].decode("utf-8")


def _settings_json(settings: dict) -> bytes:
    if not isinstance(settings, dict):
        raise ValueError("settings must be a JSON dictionary")
    try:
        encoded = json.dumps(settings, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
    except (TypeError, OverflowError) as exc:
        raise ValueError("settings must contain finite JSON data") from exc
    if json.loads(encoded) != settings:
        raise ValueError("settings must roundtrip as a JSON dictionary without changes")
    return _text(encoded, "settings_json", _SETTINGS_CAP)


def _library_path() -> Path:
    explicit = os.environ.get("LINGTU_SENSOR_CONTROL_BRIDGE_LIBRARY", "")
    if explicit:
        candidates = [Path(explicit)]
    else:
        root = Path(__file__).resolve().parents[4]
        candidates = (
            [
                root / "build/sensor-control-bridge-win/Release/lingtu_sensor_control_bridge.dll",
                root / "build/windows-native-dds-adapter/Release/lingtu_sensor_control_bridge.dll",
            ]
            if os.name == "nt"
            else [
                root / "build/mujoco_native_dds/liblingtu_sensor_control_bridge.so",
                Path("/opt/lingtu/current/lib/liblingtu_sensor_control_bridge.so"),
            ]
        )
    for path in candidates:
        if path.is_file():
            return path.resolve()
    raise SensorControlBridgeError("native sensor control bridge missing; set LINGTU_SENSOR_CONTROL_BRIDGE_LIBRARY")


class NativeSensorControlBridge:
    """Serialize all calls and close on one explicitly bound native handle."""

    def __init__(self, domain_id: int, product_session_id: str, session_id: str, owner_mode: bool, library: Any = None):
        if type(domain_id) is not int or not 0 <= domain_id <= 232:
            raise ValueError("domain_id must be an integer in 0..232")
        product = _text(product_session_id, "product_session_id", _ID_CAP, identity=True)
        source = _text(session_id, "session_id", _ID_CAP, identity=True)
        if type(owner_mode) is not bool:
            raise ValueError("owner_mode must be boolean")
        self.product_session_id, self.session_id = product_session_id, session_id
        self._lock = RLock()
        self._handle = None
        self._dll_directory = None
        try:
            if library is None:
                path = _library_path()
                if os.name == "nt":
                    self._dll_directory = os.add_dll_directory(str(path.parent))
                library = ctypes.CDLL(str(path))
            self._lib = library
            library.lt_sensor_control_create.argtypes = [
                ctypes.c_int,
                ctypes.c_char_p,
                ctypes.c_char_p,
                ctypes.c_int,
            ]
            library.lt_sensor_control_create.restype = ctypes.c_void_p
            library.lt_sensor_control_destroy.argtypes = [ctypes.c_void_p]
            library.lt_sensor_control_destroy.restype = None
            library.lt_sensor_control_last_error.argtypes = []
            library.lt_sensor_control_last_error.restype = ctypes.c_char_p
            for kind, record_type in (("request", _Request), ("result", _Result)):
                for action in ("take", "write"):
                    function = getattr(library, f"lt_sensor_control_{action}_{kind}")
                    function.argtypes = [ctypes.c_void_p, ctypes.POINTER(record_type)]
                    function.restype = ctypes.c_int
            self._handle = library.lt_sensor_control_create(domain_id, product, source, int(owner_mode))
            if not self._handle:
                self._handle = None
                raise SensorControlBridgeError(self._error())
        except Exception:
            self.close()
            raise

    def _error(self) -> str:
        raw = self._lib.lt_sensor_control_last_error()
        return raw.decode("utf-8", errors="replace") if raw else "native sensor control bridge failed"

    def _require_open(self) -> None:
        if self._handle is None:
            raise SensorControlBridgeError("native sensor control bridge is closed")

    def _binding(self, product: str, source: str) -> None:
        if product != self.product_session_id or source != self.session_id:
            raise ValueError("sensor control sample does not match bound product_session_id/session_id")

    def _take(self, kind: str, record: ctypes.Structure) -> bool:
        self._require_open()
        status = getattr(self._lib, f"lt_sensor_control_take_{kind}")(self._handle, ctypes.byref(record))
        if status == -1:
            raise SensorControlBridgeError(self._error())
        if status not in (0, 1):
            raise SensorControlBridgeError(f"native sensor control take_{kind} returned invalid status {status}")
        return bool(status == 1)

    def _write(self, kind: str, record: ctypes.Structure) -> None:
        status = getattr(self._lib, f"lt_sensor_control_write_{kind}")(self._handle, ctypes.byref(record))
        if status == -1:
            raise SensorControlBridgeError(self._error())
        if status != 0:
            raise SensorControlBridgeError(f"native sensor control write_{kind} returned invalid status {status}")

    def take_request(self) -> SensorControlRequest | None:
        with self._lock:
            record = _Request()
            if not self._take("request", record):
                return None
            values: dict[str, Any] = {}
            for field in _Request._fields_:
                name, field_type = field[:2]
                values[name] = (
                    _read_text(record, name, ctypes.sizeof(field_type))
                    if issubclass(field_type, ctypes.Array)
                    else getattr(record, name)
                )
            if values["enabled"] not in (0, 1):
                raise ValueError("enabled must be 0 or 1")
            values["enabled"] = bool(values["enabled"])
            values["operation"] = SensorControlKind(values["operation"])
            sample = SensorControlRequest(**values)
            self._binding(sample.product_session_id, sample.session_id)
            return sample

    def write_request(self, request: SensorControlRequest) -> None:
        with self._lock:
            self._require_open()
            self._binding(request.product_session_id, request.session_id)
            record = _Request()
            for field in _Request._fields_:
                name, field_type = field[:2]
                value = getattr(request, name)
                if issubclass(field_type, ctypes.Array):
                    value = _text(value, name, ctypes.sizeof(field_type), identity=name != "noise_unit")
                setattr(record, name, value)
            self._write("request", record)

    def take_result(self) -> SensorControlResult | None:
        with self._lock:
            record = _Result()
            if not self._take("result", record):
                return None
            if record.applied not in (0, 1):
                raise ValueError("applied must be 0 or 1")
            values: dict[str, Any] = {}
            for field in _Result._fields_:
                name, field_type = field[:2]
                values[name] = (
                    _read_text(record, name, ctypes.sizeof(field_type))
                    if issubclass(field_type, ctypes.Array)
                    else getattr(record, name)
                )
            values["settings"] = json.loads(values.pop("settings_json"))
            values["applied"] = bool(values["applied"])
            sample = SensorControlResult(**values)
            self._result_record(sample)
            return sample

    def _result_record(self, result: SensorControlResult) -> _Result:
        self._binding(result.product_session_id, result.session_id)
        record = _Result()
        for name in ("request_id", "product_session_id", "session_id", "sensor_id"):
            setattr(record, name, _text(getattr(result, name), name, _ID_CAP, identity=True))
        for name in ("model_generation", "reset_generation", "applied_sim_time_ns"):
            value = getattr(result, name)
            if type(value) is not int or not 0 <= value < 2**64:
                raise ValueError(f"{name} must be uint64")
            setattr(record, name, value)
        if type(result.applied) is not bool:
            raise ValueError("applied must be boolean")
        record.applied = int(result.applied)
        record.reason = _text(result.reason, "reason", _REASON_CAP)
        record.settings_json = _settings_json(result.settings)
        return record

    def write_result(self, result: SensorControlResult) -> None:
        with self._lock:
            self._require_open()
            self._write("result", self._result_record(result))

    def close(self) -> None:
        with self._lock:
            try:
                if self._handle is not None:
                    handle, self._handle = self._handle, None
                    self._lib.lt_sensor_control_destroy(handle)
            finally:
                if self._dll_directory is not None:
                    directory, self._dll_directory = self._dll_directory, None
                    directory.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

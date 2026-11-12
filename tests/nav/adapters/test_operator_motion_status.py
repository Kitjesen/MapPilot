from __future__ import annotations

import ctypes
import time
from types import SimpleNamespace

import pytest

from nav.adapters.native import abi
from nav.commands import module


class Function:
    def __init__(self, callback):
        self.callback = callback

    def __call__(self, *args):
        return self.callback(*args)


class Library:
    def __init__(self):
        self.available = True
        self.result = 1
        self.age = 2.5
        self.fresh = 1
        self.owner = b"sdk:owner"
        self.flags = 1
        self.reads = []
        self.lingtu_nav_client_abi_version = Function(lambda: abi.NATIVE_COMMAND_ABI_VERSION)
        self.lingtu_nav_client_capabilities = Function(lambda: abi.NATIVE_COMMAND_CAP_OPERATOR_MOTION_STATUS)
        self.lingtu_nav_client_create = Function(lambda domain: 41)
        self.lingtu_nav_client_destroy = Function(lambda handle: None)
        self.lingtu_nav_client_last_error = Function(lambda handle: b"read failed")
        self.lingtu_nav_client_read_operator_motion_status_v1 = Function(self.read)

    def read(self, handle, age, output):
        self.reads.append((handle, age))
        if not self.available:
            return 0
        state = ctypes.cast(output, ctypes.POINTER(abi._NativeOperatorMotionStatusV1)).contents
        state.timestamp_s = time.time()
        state.active_source_id = self.owner
        state.active_source_epoch = 17
        state.has_active_authority = self.flags
        state.holding = 1
        state.has_active_sample = 0
        state.last_sample_sequence = 31
        state.admitted_sequence = 30
        state.final_output_sequence = 400
        state.authority_reason = b"claimed"
        state.input_gate_reason = b"driver_control_not_ready"
        state.received_age_ms = self.age
        state.fresh = self.fresh
        return self.result


def test_status_preserves_native_identity_and_stale_evidence():
    library = Library()
    session = abi.NativeCommandSession("unused", library=library)
    try:
        state = session.read_operator_motion_status()
        assert state["active_source_id"] == "sdk:owner"
        assert state["active_source_epoch"] == 17
        assert state["has_active_authority"] is True and state["holding"] is True
        assert state["last_sample_sequence"] == 31 and state["final_output_sequence"] == 400
        assert state["received_age_ms"] == 2.5 and state["fresh"] is True
        library.age = 300
        assert session.read_operator_motion_status()["fresh"] is False
        library.age = 2.5
        library.fresh = 0
        assert session.read_operator_motion_status()["fresh"] is False
        library.available = False
        assert session.read_operator_motion_status() is None
        assert library.reads == [(41, 250)] * 4
    finally:
        session.close()


@pytest.mark.parametrize("fault", ["flags", "owner", "age", "result"])
def test_invalid_native_status_is_not_authority(fault):
    library = Library()
    if fault == "flags":
        library.flags = 7
    elif fault == "owner":
        library.owner = b""
    elif fault == "age":
        library.age = float("nan")
    else:
        library.result = -1
    session = abi.NativeCommandSession("unused", library=library)
    try:
        with pytest.raises(abi.NativeCommandClientError):
            session.read_operator_motion_status()
    finally:
        session.close()


def test_unsupported_status_is_lazy_and_commands_forward_read_only(monkeypatch):
    library = Library()
    library.lingtu_nav_client_capabilities = Function(lambda: 0)
    session = abi.NativeCommandSession("unused", library=library)
    try:
        with pytest.raises(abi.NativeCommandClientError, match="operator motion status"):
            session.read_operator_motion_status()
        assert library.reads == []
    finally:
        session.close()
    observed = []
    state = {"fresh": False, "received_age_ms": 300.0}

    def status(age):
        observed.append(age)
        return state

    monkeypatch.setattr(module, "get_native_operator_motion_client", lambda **kwargs: SimpleNamespace(status=status))
    commands = module.Commands()
    assert commands.operator_motion_status(250) is state
    assert observed == [250]

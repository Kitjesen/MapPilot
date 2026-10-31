"""Host C ABI ownership and packet conversion without a second DDS runtime."""

import ctypes

import pytest
from sim.scripts.mujoco.measurement_wire import encode_measurement

from tests.sim.test_measurement_payload import _thermal


class Function:
    def __init__(self, callback):
        self.callback = callback

    def __call__(self, *args):
        return self.callback(*args)


class Library:
    def __init__(self, payload=None, needed=None):
        self.payload = payload
        self.needed = needed
        self.destroyed = []
        self.lt_measurement_create = Function(lambda *args: 42)
        self.lt_measurement_destroy = Function(self.destroyed.append)
        self.lt_measurement_last_error = Function(lambda: b"native bridge failure")
        self.lt_measurement_take = Function(self.take)

    def take(self, handle, buffer, capacity, needed):
        length = self.needed if self.needed is not None else len(self.payload or b"")
        ctypes.cast(needed, ctypes.POINTER(ctypes.c_size_t))[0] = length
        if not length:
            return 0
        if capacity < length:
            return -2
        ctypes.memmove(buffer, self.payload, length)
        self.payload = None
        return 1


def test_owned_reader_converts_exact_packet_and_closes_once():
    from drivers.sim.measurement.native_bridge import NativeMeasurementBridge

    payload = encode_measurement(_thermal(), product_session_id="product").payload
    library = Library(payload)
    reader = NativeMeasurementBridge(
        domain_id=6, product_session_id="product", session_id="recording-test", library=library
    )
    sample = reader.take(received_at_s=1234.0)
    assert sample.identity.sensor_id == "thermal"
    assert sample.identity.product_session_id == "product"
    assert sample.received_at_s == 1234 and sample.captured_at_s is None
    assert reader.take(received_at_s=1234.0) is None
    reader.close()
    reader.close()
    assert library.destroyed == [42]
    with pytest.raises(RuntimeError, match="closed"):
        reader.take(received_at_s=1234.0)


def test_native_capacity_is_checked_before_allocating_and_exception_cleanup_works():
    from drivers.sim.measurement.native_bridge import NativeMeasurementBridge

    library = Library(needed=33 * 1024 * 1024)
    with pytest.raises(RuntimeError, match="size"):
        with NativeMeasurementBridge(
            domain_id=6, product_session_id="product", session_id="source", library=library
        ) as reader:
            reader.take(received_at_s=1234.0)
    assert library.destroyed == [42]


def test_foreign_packet_is_not_relabelled_as_bound_product():
    from drivers.sim.measurement.native_bridge import NativeMeasurementBridge

    library = Library(encode_measurement(_thermal(), product_session_id="wrong").payload)
    with NativeMeasurementBridge(
        domain_id=6, product_session_id="product", session_id="recording-test", library=library
    ) as reader:
        with pytest.raises(ValueError):
            reader.take(received_at_s=1234.0)

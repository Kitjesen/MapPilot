"""Measurement endpoint behavior through real Module ports."""

import threading
from collections import deque
from dataclasses import replace

import numpy as np
import pytest

from runtime.contracts.sensor_control import SensorControlResult
from runtime.module import Module
from runtime.msgs.measurement import AcousticMeasurement, MeasurementIdentity, SemanticMeasurement, ThermalMeasurement
from runtime.registry import get
from runtime.stream import In, Out


class Bridge:
    def __init__(self, samples=()):
        self.samples = deque(samples)
        self.receipts = []
        self.closed = 0
        self.binding = None

    def factory(self, **binding):
        self.binding = binding
        return self

    def take(self, *, received_at_s):
        self.receipts.append(received_at_s)
        if not self.samples:
            return None
        sample = self.samples.popleft()
        if isinstance(sample, Exception):
            raise sample
        return replace(sample, received_at_s=received_at_s)

    def close(self):
        self.closed += 1


def thermal(sensor_id="thermal"):
    return ThermalMeasurement(
        MeasurementIdentity("product", "source", "robot", sensor_id, "optical", 2, 3, 7, 1_000_000_000),
        np.array([[300.0, np.nan]], dtype=np.float32),
        np.array([[True, False]]),
        {},
        0.0,
    )


def make_module(bridge, **kw):
    from drivers.sim.measurement.dds_module import DdsMeasurementModule

    required = kw.pop("required_sensor_ids", ("thermal",))
    start_thread = kw.pop("start_thread", False)
    return DdsMeasurementModule(
        product_session_id="product",
        session_id="source",
        domain_id=6,
        required_sensor_ids=required,
        bridge_factory=bridge.factory,
        start_thread=start_thread,
        **kw,
    )


def test_acknowledged_disable_is_not_a_missing_sensor_but_enable_requires_new_receipt():
    module = make_module(Bridge())
    module.setup()
    module.start()
    try:
        disabled = SensorControlResult("disable", "product", "source", "thermal", 2, 3, 1,
                                       True, "applied", {"enabled": False})
        module.control_state._deliver(disabled)
        assert module.startup_readiness() is None
        info = module.health()["sensors"]["thermal"]
        assert info["enabled"] is False and info["state"] == "DISABLED" and info["frames"] == 0
        module.control_state._deliver(replace(disabled, request_id="enable", settings={"enabled": True}))
        assert module.startup_readiness() == "measurement_sensors_missing:thermal"
    finally:
        module.stop()


def test_registered_endpoint_delivers_thermal_on_real_typed_port():
    bridge = Bridge([thermal()])
    module = make_module(bridge)
    received = []
    assert isinstance(module, Module)
    assert isinstance(module.thermal, Out)
    assert module.thermal.msg_type is ThermalMeasurement
    assert get("measurements", "native_dds") is type(module)
    assert bridge.binding is None
    module.setup()
    assert received == [] and bridge.receipts == []
    module.thermal.subscribe(received.append)
    module.start()
    try:
        assert module.poll_once() == 1
        assert len(received) == 1
        assert received[0].identity == thermal().identity
        assert received[0].received_at_s == bridge.receipts[0] > 0
        assert received[0].captured_at_s is None
        assert module.thermal.msg_count == 1
        assert bridge.binding == dict(domain_id=6, product_session_id="product", session_id="source")
    finally:
        module.stop()


def test_readiness_requires_every_configured_sensor_fresh_but_exposes_quality_and_clock_separately(monkeypatch):
    now = [10.0]
    monkeypatch.setattr("drivers.sim.measurement.dds_module.time.monotonic", lambda: now[0])
    bridge = Bridge()
    module = make_module(bridge, required_sensor_ids=("thermal", "thermal-rear"), stale_timeout_s=1.0)
    alive = []
    module.alive.subscribe(alive.append)
    assert module.startup_readiness() == "not_running"
    module.setup()
    module.start()
    try:
        assert module.poll_once() == 0
        health = module.health()
        assert health["ready"] is False
        assert health["status"] == "running"
        assert module.startup_readiness() == "measurement_sensors_missing:thermal,thermal-rear"
        assert health["sensors"]["thermal"]["frames"] == 0
        assert health["sensors"]["thermal"]["received_at_s"] is None
        assert alive and not any(alive)
        bridge.samples.append(thermal())
        assert module.poll_once() == 1
        assert module.startup_readiness() == "measurement_sensors_missing:thermal-rear"
        invalid = replace(
            thermal("thermal-rear"), temperature_k=np.full((1, 2), np.nan), valid_mask=np.zeros((1, 2), dtype=bool)
        )
        bridge.samples.append(invalid)
        assert module.poll_once() == 1
        assert module.startup_readiness() is None
        health = module.health()
        assert health["ready"] is True and alive[-1] is True
        front = health["sensors"]["thermal"]
        rear = health["sensors"]["thermal-rear"]
        assert front["frames"] == rear["frames"] == 1
        assert front["clock_mapped"] is False and front["captured_at_s"] is None
        assert front["quality"] == {"valid_elements": 1, "total_elements": 2, "valid_fraction": 0.5}
        assert rear["quality"]["valid_elements"] == 0
        assert rear["fresh"] is True
        now[0] += 1.1
        assert module.startup_readiness() == "measurement_sensors_stale:thermal,thermal-rear"
        assert module.health()["sensors"]["thermal"]["receipt_age_s"] == pytest.approx(1.1)
        assert module.poll_once() == 0
        assert alive[-1] is False and module.health()["ready"] is False
        mapped = replace(thermal(), metadata={"frame_metadata": {"source_clock": {"mapped_epoch_s": 12.0}}})
        bridge.samples.append(mapped)
        module.poll_once()
        assert module.startup_readiness() == "measurement_sensors_stale:thermal-rear"
        assert module.health()["sensors"]["thermal"]["clock_mapped"] is True
    finally:
        module.stop()
    assert bridge.closed == 1


@pytest.mark.parametrize("kind", ["thermal", "acoustic", "semantic"])
def test_dispatches_each_frame_type_without_replacing_source_clock(kind):
    base = thermal()
    metadata = {"frame_metadata": {"source_clock": {"mapped_epoch_s": 123.5}}}
    samples = {
        "thermal": replace(base, metadata=metadata),
        "acoustic": AcousticMeasurement(
            base.identity,
            np.array([[0.1, np.nan]], dtype=np.float32),
            base.valid_mask,
            8000,
            None,
            metadata,
            0.0,
        ),
        "semantic": SemanticMeasurement(
            base.identity,
            np.array([[4, 0]], dtype=np.uint32),
            np.array([[12, 0]], dtype=np.uint32),
            base.valid_mask,
            metadata,
            0.0,
        ),
    }
    bridge = Bridge([samples[kind]])
    module = make_module(bridge)
    received = {name: [] for name in samples}
    for name, values in received.items():
        port = module.ports_out[name]
        assert port.msg_type is type(samples[name])
        port.subscribe(values.append)
    module.setup()
    module.start()
    try:
        assert module.poll_once() == 1
        assert [name for name, values in received.items() if values] == [kind]
        sample = received[kind][0]
        assert sample.captured_at_s == 123.5
        assert sample.received_at_s > sample.captured_at_s
        assert sample.identity.sim_time_ns == 1_000_000_000
        np.testing.assert_array_equal(sample.valid_mask, base.valid_mask)
    finally:
        module.stop()


def test_poll_batch_bounds_native_takes_even_when_records_are_rejected():
    bridge = Bridge([ValueError("foreign product"), thermal(), thermal(), thermal()])
    module = make_module(bridge, max_frames_per_poll=2)
    received = []
    module.thermal.subscribe(received.append)
    assert module.poll_once() == 0
    module.setup()
    module.start()
    try:
        assert module.poll_once() == 1
        assert len(bridge.receipts) == 2
        assert len(received) == 1
        assert module.health()["rejected_frames"] == 1
        assert module.poll_once() == 2
        assert len(bridge.receipts) == 4 and len(received) == 3
        assert module.poll_once() == 0
        assert len(bridge.receipts) == 5
    finally:
        module.stop()
    assert module.poll_once() == 0 and len(bridge.receipts) == 5


def test_owned_thread_starts_only_after_setup_and_subscribers_then_joins_before_single_close():
    class ThreadBridge(Bridge):
        readers = set()

        def take(self, **kw):
            self.readers.add(threading.current_thread())
            return super().take(**kw)

        def close(self):
            assert all(not reader.is_alive() for reader in self.readers)
            super().close()

    bridge = ThreadBridge([thermal()])
    module = make_module(bridge, start_thread=True, poll_interval_s=0.02)
    delivered = threading.Event()
    received = []
    module.setup()
    module.setup()
    assert bridge.receipts == []
    module.thermal.subscribe(lambda sample: (received.append(sample), delivered.set()))
    module.start()
    module.start()
    try:
        assert delivered.wait(1.0)
        threading.Event().wait(0.06)
        assert len(received) == 1
        assert 1 <= len(bridge.receipts) <= 5
        assert len(bridge.readers) == 1
    finally:
        module.stop()
        module.stop()
    assert bridge.closed == 1
    assert module.running is False
    assert all(not reader.is_alive() for reader in bridge.readers)
    assert module.thermal.callback_count == 0


@pytest.mark.parametrize("stage", ["setup", "take", "close"])
def test_lifecycle_errors_are_visible_and_never_claim_alive(stage):
    class FailingBridge(Bridge):
        def factory(self, **binding):
            if stage == "setup":
                raise RuntimeError("setup failed")
            return super().factory(**binding)

        def close(self):
            super().close()
            if stage == "close":
                raise RuntimeError("close failed")

    bridge = FailingBridge([RuntimeError("take failed")] if stage == "take" else [thermal()])
    module = make_module(bridge)
    alive = []
    module.alive.subscribe(alive.append)
    try:
        if stage == "setup":
            with pytest.raises(RuntimeError, match="setup failed"):
                module.setup()
        else:
            module.setup()
            module.start()
            if stage == "take":
                with pytest.raises(RuntimeError, match="take failed"):
                    module.poll_once()
            else:
                module.poll_once()
                assert module.health()["ready"] is True
                with pytest.raises(RuntimeError, match="close failed"):
                    module.stop()
        health = module.health()
        assert health["ready"] is False
        assert health["status"] == "error"
        assert f"{stage} failed" in health["error"]
        assert alive[-1] is False
    finally:
        module.stop()
        module.stop()
    assert bridge.closed == (0 if stage == "setup" else 1)
    assert module.thermal.callback_count == 0


@pytest.mark.parametrize("foreign", ["product", "source"])
def test_foreign_session_rejection_is_delegated_to_real_native_bridge(foreign):
    from sim.scripts.mujoco.measurement_wire import encode_measurement

    from drivers.sim.measurement.dds_module import DdsMeasurementModule
    from drivers.sim.measurement.native_bridge import NativeMeasurementBridge
    from tests.drivers.test_measurement_native_bridge import Library
    from tests.sim.test_measurement_payload import _thermal

    packet = encode_measurement(_thermal(), product_session_id="wrong" if foreign == "product" else "product")
    library = Library(packet.payload)
    module = DdsMeasurementModule(
        product_session_id="product",
        session_id="wrong" if foreign == "source" else "recording-test",
        domain_id=6,
        required_sensor_ids=("thermal",),
        start_thread=False,
        bridge_factory=lambda **kw: NativeMeasurementBridge(**kw, library=library),
    )
    received, alive = [], []
    module.thermal.subscribe(received.append)
    module.alive.subscribe(alive.append)
    module.setup()
    module.start()
    try:
        assert module.poll_once() == 0
        assert not received and not any(alive)
        health = module.health()
        assert health["rejected_frames"] == 1
        assert "session mismatch" in health["error"]
        assert health["sensors"]["thermal"]["frames"] == 0
        assert module.startup_readiness() is not None
    finally:
        module.stop()
        module.stop()
    assert library.destroyed == [42]


@pytest.mark.parametrize(
    "invalid",
    [
        {"product_session_id": ""},
        {"session_id": None},
        {"domain_id": None},
        {"domain_id": True},
        {"domain_id": 233},
        {"required_sensor_ids": ()},
        {"required_sensor_ids": "thermal"},
        {"required_sensor_ids": ("",)},
        {"poll_interval_s": float("nan")},
        {"poll_interval_s": 0},
        {"stale_timeout_s": float("inf")},
        {"stale_timeout_s": -1},
        {"max_frames_per_poll": 0},
        {"max_frames_per_poll": 1.5},
        {"max_frames_per_poll": 1025},
    ],
)
def test_explicit_binding_and_bounded_configuration_cannot_fall_back_to_auto_binding(invalid, monkeypatch):
    from drivers.sim.measurement.dds_module import DdsMeasurementModule

    monkeypatch.setenv("LINGTU_PRODUCT_SESSION_ID", "environment-product")
    monkeypatch.setenv("LINGTU_SESSION_ID", "environment-source")
    config = dict(
        product_session_id="product",
        session_id="source",
        domain_id=6,
        required_sensor_ids=("thermal",),
        start_thread=False,
    )
    config.update(invalid)
    with pytest.raises(ValueError):
        DdsMeasurementModule(**config)


def test_receipt_freshness_is_not_renewed_by_slow_downstream_publication(monkeypatch):
    now = [10.0]
    monkeypatch.setattr("drivers.sim.measurement.dds_module.time.monotonic", lambda: now[0])
    bridge = Bridge([thermal()])
    module = make_module(bridge)
    module.thermal.subscribe(lambda sample: now.__setitem__(0, 20.0))
    module.setup()
    module.start()
    try:
        assert module.poll_once() == 1
        assert module.health()["sensors"]["thermal"]["receipt_age_s"] == 10.0
        assert module.startup_readiness() == "measurement_sensors_stale:thermal"
    finally:
        module.stop()


def test_port_delivery_failure_is_visible_even_when_out_swallows_callback_exception():
    bridge = Bridge([thermal()])
    module = make_module(bridge)
    alive = []

    def failing_consumer(sample):
        raise RuntimeError("consumer unavailable")

    module.thermal.subscribe(failing_consumer)
    module.alive.subscribe(alive.append)
    module.setup()
    module.start()
    try:
        with pytest.raises(RuntimeError, match="publication failed"):
            module.poll_once()
        assert module.health()["ready"] is False
        assert "publication failed" in module.health()["error"]
        assert module.thermal.publish_errors == 1
        assert alive[-1] is False
    finally:
        module.stop()


def test_endpoint_cannot_start_unconfigured_or_reopen_after_stop():
    bridge = Bridge()
    module = make_module(bridge)
    with pytest.raises(RuntimeError, match="setup"):
        module.start()
    module.setup()
    module.stop()
    with pytest.raises(RuntimeError, match="closed"):
        module.setup()
    with pytest.raises(RuntimeError, match="closed"):
        module.start()
    module.stop()
    assert bridge.closed == 1


def test_stop_timeout_reports_error_without_closing_bridge_under_live_poll_thread():
    bridge = Bridge([thermal()])
    module = make_module(bridge, start_thread=True)
    entered, release = threading.Event(), threading.Event()
    alive = []

    def slow_consumer(sample):
        entered.set()
        assert release.wait(3.0)

    module.thermal.subscribe(slow_consumer)
    module.alive.subscribe(alive.append)
    module.setup()
    module.start()
    try:
        assert entered.wait(1.0)
        with pytest.raises(RuntimeError, match="did not stop"):
            module.stop()
        assert bridge.closed == 0
        assert module.health()["status"] == "error"
        assert "did not stop" in module.health()["error"]
        assert alive[-1] is False
    finally:
        release.set()
        module.stop()
    assert bridge.closed == 1
    assert alive[-1] is False


@pytest.mark.parametrize("delivery", ["callback", "local"])
def test_host_builder_delivers_mixed_native_records_without_new_bus_or_encoding(delivery):
    from sim.runtime.sensors.camera_geometry import PinholeCalibration
    from sim.scripts.mujoco.measurement_wire import encode_measurement

    from drivers.sim.measurement.dds_module import DdsMeasurementModule
    from drivers.sim.measurement.native_bridge import NativeMeasurementBridge
    from runtime.host import HostBuilder
    from tests.drivers.test_measurement_native_bridge import Library
    from tests.sim.test_measurement_payload import _acoustic, _semantic, _thermal

    samples = [_thermal(), _acoustic(), _semantic(PinholeCalibration(2, 2, 80, 80, 1, 1))]
    packets = deque(encode_measurement(sample, product_session_id="product").payload for sample in samples)

    class QueuedLibrary(Library):
        def take(self, *args):
            if self.payload is None and packets:
                self.payload = packets.popleft()
            return super().take(*args)

    class Sink(Module):
        thermal: In[ThermalMeasurement]
        acoustic: In[AcousticMeasurement]
        semantic: In[SemanticMeasurement]

        def setup(self):
            for name, port in self.ports_in.items():
                port.subscribe(lambda sample, name=name: received[name].append(sample))

    library = QueuedLibrary()
    received = {name: [] for name in ("thermal", "acoustic", "semantic")}
    endpoint = DdsMeasurementModule(
        product_session_id="product",
        session_id="recording-test",
        domain_id=6,
        required_sensor_ids=("thermal", "microphone", "semantic"),
        stale_timeout_s=5.0,
        bridge_factory=lambda **kw: NativeMeasurementBridge(**kw, library=library),
    )
    sink = Sink()
    builder = HostBuilder().add(endpoint, alias="measurements").add(sink, alias="sink")
    for name in received:
        builder.wire("measurements", name, "sink", name, delivery=delivery)
    builder.require_modules("measurements")
    handle = builder.build()
    try:
        handle.start(startup_timeout_s=1.0)
        assert endpoint.startup_readiness() is None
        assert all(len(values) == 1 for values in received.values())
        for name, values in received.items():
            assert sink.ports_in[name].latest is values[0]
            assert values[0].identity.product_session_id == "product"
            assert values[0].captured_at_s is None
        np.testing.assert_array_equal(received["thermal"][0].temperature_k, samples[0].temperature_k)
        np.testing.assert_array_equal(received["acoustic"][0].voltage_v, samples[1].voltage_v)
        np.testing.assert_array_equal(received["semantic"][0].class_ids, samples[2].class_ids)
        assert endpoint.health()["errors"] == {}
    finally:
        handle.stop()
    assert library.destroyed == [42]


def test_background_reader_failure_publishes_false_and_remains_visible_until_cleanup():
    bridge = Bridge([RuntimeError("native reader lost")])
    module = make_module(bridge, start_thread=True)
    failed = threading.Event()
    alive = []

    def on_alive(value):
        alive.append(value)
        if not value and module.health()["error"]:
            failed.set()

    module.alive.subscribe(on_alive)
    module.setup()
    module.start()
    try:
        assert failed.wait(1.0)
        assert module.health()["status"] == "error"
        assert "native reader lost" in module.health()["error"]
        assert module.running is False
        assert not any(alive)
    finally:
        module.stop()
    assert bridge.closed == 1


def test_thread_start_failure_records_health_and_still_closes_bridge(monkeypatch):
    bridge = Bridge()
    module = make_module(bridge, start_thread=True)
    alive = []

    def fail_start(thread):
        raise RuntimeError("thread unavailable")

    monkeypatch.setattr(threading.Thread, "start", fail_start)
    module.alive.subscribe(alive.append)
    module.setup()
    try:
        with pytest.raises(RuntimeError, match="thread unavailable"):
            module.start()
        assert module.health()["status"] == "error"
        assert "thread unavailable" in module.health()["error"]
        assert module.running is False and not any(alive)
    finally:
        module.stop()
    assert bridge.closed == 1

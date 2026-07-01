"""Tests for transport backends — LocalTransport, SHM, and Dual.

TODO: Replace time.sleep with threading.Event for subscriber delivery
      synchronization. SHM subscriber startup is near-instant; poll
      sleeps can be replaced with a delivery-count event.
"""

import threading
import time

import pytest

pytestmark = [pytest.mark.ros2]

from runtime.transport.abc import TopicConfig, TransportStrategy
from runtime.transport.local import LocalTransport, Transport


class TestTransportDefaults:
    """Default transport behavior must stay ROS-free and in-process."""

    def test_topic_config_defaults_to_local(self):
        assert TopicConfig(name="/test/default").strategy is TransportStrategy.LOCAL

    def test_create_transport_defaults_to_shared_local(self):
        from runtime.transport.factory import create_transport

        first = create_transport()
        second = create_transport(TransportStrategy.LOCAL)

        assert isinstance(first, LocalTransport)
        assert first is second
        assert first.name == "local"

    def test_create_transport_accepts_string_strategy(self):
        from runtime.transport.factory import create_transport

        assert create_transport("local") is create_transport()

    def test_create_transport_rejects_unknown_string_strategy(self):
        from runtime.transport.factory import create_transport

        with pytest.raises(ValueError, match="Unknown strategy"):
            create_transport("mqtt")

    def test_shortcut_pub_sub_uses_shared_local_bus(self):
        from runtime.transport.factory import create_publisher, create_subscriber

        topic = "/test/default_local_shortcut"
        received = []

        sub = create_subscriber(topic, callback=received.append)
        pub = create_publisher(topic)
        pub.publish({"ok": True})

        sub.close()
        assert received == [{"ok": True}]


class TestLocalTransport:
    """LocalTransport in-process bus tests."""

    def test_pub_sub_basic(self):
        t = LocalTransport()
        received = []
        t.subscribe("topic_a", received.append)
        t.publish("topic_a", "hello")
        assert received == ["hello"]
        t.close()

    def test_multiple_subscribers(self):
        t = LocalTransport()
        a, b = [], []
        t.subscribe("t", a.append)
        t.subscribe("t", b.append)
        t.publish("t", 42)
        assert a == [42]
        assert b == [42]
        t.close()

    def test_no_cross_topic(self):
        t = LocalTransport()
        received = []
        t.subscribe("a", received.append)
        t.publish("b", "wrong")
        assert received == []
        t.close()

    def test_unsubscribe(self):
        t = LocalTransport()
        received = []
        cb = received.append
        t.subscribe("t", cb)
        t.publish("t", 1)
        t.unsubscribe("t", cb)
        t.publish("t", 2)
        assert received == [1]
        t.close()

    def test_topics_list(self):
        t = LocalTransport()
        t.subscribe("alpha", lambda m: None)
        t.subscribe("beta", lambda m: None)
        assert sorted(t.topics) == ["alpha", "beta"]
        t.close()

    def test_subscriber_count(self):
        t = LocalTransport()
        t.subscribe("t", lambda m: None)
        t.subscribe("t", lambda m: None)
        assert t.subscriber_count("t") == 2
        assert t.subscriber_count("empty") == 0
        t.close()

    def test_close_clears_all(self):
        t = LocalTransport()
        t.subscribe("t", lambda m: None)
        t.close()
        assert t.topics == []

    def test_callback_error_doesnt_crash(self):
        t = LocalTransport()
        good = []
        t.subscribe("t", lambda m: 1 / 0)  # will raise
        t.subscribe("t", good.append)
        t.publish("t", "ok")
        assert good == ["ok"]
        t.close()

    def test_thread_safety(self):
        t = LocalTransport()
        results = []
        t.subscribe("t", results.append)

        def publisher():
            for i in range(100):
                t.publish("t", i)

        threads = [threading.Thread(target=publisher) for _ in range(4)]
        for th in threads:
            th.start()
        for th in threads:
            th.join()

        assert len(results) == 400
        t.close()

    def test_implements_transport_protocol(self):
        t = LocalTransport()
        assert isinstance(t, Transport)
        t.close()


class TestSHMTransport:
    """SHM transport tests (requires multiprocessing.shared_memory)."""

    def test_pub_sub_bytes(self):
        from runtime.transport.shm import SHMTransport
        topic = TopicConfig(name="/test/shm_bytes", strategy=TransportStrategy.SHM, buffer_size=1024)

        transport = SHMTransport()
        received = []

        pub = transport.create_publisher(topic)
        sub_topic = TopicConfig(name="/test/shm_bytes", strategy=TransportStrategy.SHM)
        transport.create_subscriber(sub_topic, lambda data, ts: received.append(data))

        # Give subscriber time to start polling
        time.sleep(0.012)

        pub.publish(b"hello shm")
        time.sleep(0.012)  # wait for poll to pick it up

        transport.close()
        assert len(received) >= 1
        assert b"hello shm" in received

    def test_pub_sub_pickle(self):
        from runtime.transport.shm import SHMTransport
        topic = TopicConfig(name="/test/shm_pickle", strategy=TransportStrategy.SHM, buffer_size=4096)

        transport = SHMTransport()
        received = []

        pub = transport.create_publisher(topic)
        sub_topic = TopicConfig(name="/test/shm_pickle", strategy=TransportStrategy.SHM)
        transport.create_subscriber(sub_topic, lambda data, ts: received.append(data))

        time.sleep(0.012)
        pub.publish({"key": "value", "n": 42})
        time.sleep(0.012)

        transport.close()
        # SHM delivers raw bytes for pickle; verify we got something
        assert len(received) >= 1

    def test_shm_name_generation(self):
        from runtime.transport.shm import _shm_name
        assert _shm_name("/nav/odometry") == "lingtu_nav_odometry"
        assert _shm_name("simple") == "lingtu_simple"

    def test_buffer_overflow_raises(self):
        """Publishing data larger than buffer should raise ValueError."""
        from runtime.transport.shm import SHMTransport
        topic = TopicConfig(name="/test/shm_overflow", strategy=TransportStrategy.SHM, buffer_size=64)

        transport = SHMTransport()
        pub = transport.create_publisher(topic)
        big_data = b"x" * 200
        with pytest.raises(ValueError, match="exceeds buffer capacity"):
            pub.publish(big_data)
        transport.close()


class TestTransportAdapter:
    """TransportAdapter bridges TransportABC backends to Transport Protocol."""

    def test_adapter_wraps_shm_pub_sub(self):
        from runtime.transport.adapter import TransportAdapter
        from runtime.transport.shm import SHMTransport

        adapter = TransportAdapter(SHMTransport())
        received = []

        # Publish first to create the SHM region, then subscribe
        adapter.publish("/test/adapter_shm2", {"init": True})
        time.sleep(0.006)

        adapter.subscribe("/test/adapter_shm2", received.append)
        time.sleep(0.012)  # subscriber poll startup

        adapter.publish("/test/adapter_shm2", {"key": "value"})
        time.sleep(0.012)  # poll picks it up

        adapter.close()
        assert len(received) >= 1
        assert received[-1] == {"key": "value"}

    def test_adapter_implements_transport_protocol(self):
        from runtime.transport.adapter import TransportAdapter
        from runtime.transport.shm import SHMTransport

        adapter = TransportAdapter(SHMTransport())
        # Must have publish, subscribe, close
        assert hasattr(adapter, 'publish')
        assert hasattr(adapter, 'subscribe')
        assert hasattr(adapter, 'close')
        adapter.close()

    def test_adapter_lazy_publisher_creation(self):
        from runtime.transport.adapter import TransportAdapter
        from runtime.transport.shm import SHMTransport

        adapter = TransportAdapter(SHMTransport())
        assert len(adapter._publishers) == 0
        # First publish creates the publisher
        try:
            adapter.publish("/test/lazy", b"hello")
        except Exception:
            pass  # SHM might not have subscriber
        assert len(adapter._publishers) == 1
        adapter.close()

    def test_adapter_multiple_subscribers_same_topic(self):
        from runtime.transport.adapter import TransportAdapter
        from runtime.transport.shm import SHMTransport

        adapter = TransportAdapter(SHMTransport())
        a, b = [], []
        adapter.subscribe("/test/multi", a.append)
        adapter.subscribe("/test/multi", b.append)
        assert len(adapter._subscribers["/test/multi"]) == 2
        adapter.close()

    def test_adapter_backend_name(self):
        from runtime.transport.adapter import TransportAdapter
        from runtime.transport.shm import SHMTransport

        adapter = TransportAdapter(SHMTransport())
        assert adapter.backend_name == "shm"
        adapter.close()

    def test_adapter_close_cleans_up(self):
        from runtime.transport.adapter import TransportAdapter
        from runtime.transport.shm import SHMTransport

        adapter = TransportAdapter(SHMTransport())
        adapter.subscribe("/test/cleanup", lambda m: None)
        try:
            adapter.publish("/test/cleanup", b"data")
        except Exception:
            pass
        adapter.close()
        assert len(adapter._publishers) == 0
        assert len(adapter._subscribers) == 0

    def test_adapter_with_out_in_ports(self):
        """Adapter works as transport for Out[T] → In[T] data flow."""
        from runtime.stream import In, Out
        from runtime.transport.adapter import TransportAdapter
        from runtime.transport.shm import SHMTransport

        adapter = TransportAdapter(SHMTransport())

        out = Out(int, "test_val")
        In(int, "test_val")

        # Bind transport to out port
        out._bind_transport(adapter, "/test/port_flow")

        # The transport publish path works (won't deliver to In without
        # explicit subscribe wiring, but verifies no crash)
        out.publish(42)
        assert out.msg_count == 1

        adapter.close()


class TestLCMTransport:
    """LCM is optional and stays behind the transport boundary."""

    def test_lcm_transport_requires_optional_package(self, monkeypatch):
        import runtime.transport.lcm as lcm_mod

        monkeypatch.setattr(lcm_mod, "_LCM_AVAILABLE", False)
        with pytest.raises(ImportError, match="lcm is not installed"):
            lcm_mod.LCMTransport()

    def test_lcm_factory_requires_optional_package(self, monkeypatch):
        import runtime.transport.lcm as lcm_mod
        from runtime.transport.factory import create_transport

        monkeypatch.setattr(lcm_mod, "_LCM_AVAILABLE", False)
        with pytest.raises(ImportError, match="lcm is not installed"):
            create_transport(TransportStrategy.LCM)

    def test_lcm_publisher_accepts_only_bytes(self):
        from runtime.transport.lcm import LCMPublisher

        class FakeLCM:
            def __init__(self):
                self.published = []

            def publish(self, channel, data):
                self.published.append((channel, data))

        client = FakeLCM()
        topic = TopicConfig(name="/test/lcm_bytes", strategy=TransportStrategy.LCM)
        pub = LCMPublisher(topic, client)

        with pytest.raises(TypeError, match="expects bytes"):
            pub.publish({"not": "serialized"})

        pub.publish(bytearray(b"serialized"))
        assert client.published == [("/test/lcm_bytes", b"serialized")]

    def test_lcm_subscriber_delivers_bytes_with_timestamp(self):
        from runtime.transport.lcm import LCMSubscriber

        class FakeLCM:
            pass

        received = []
        topic = TopicConfig(name="/test/lcm_sub", strategy=TransportStrategy.LCM)
        sub = LCMSubscriber(topic, lambda data, ts: received.append((data, ts)), FakeLCM())
        sub._on_lcm_message("/test/lcm_sub", bytearray(b"payload"))

        assert received
        assert received[0][0] == b"payload"
        assert isinstance(received[0][1], float)

    def test_json_codec_roundtrips_core_message(self):
        import json
        from runtime.msgs.geometry import Twist, Vector3
        from runtime.transport.json_codec import dumps_message, loads_message

        msg = Twist(linear=Vector3(1.0, 2.0, 0.0), angular=Vector3(0.0, 0.0, 0.5))
        encoded = dumps_message(msg, topic="/nav/cmd_vel")
        decoded = loads_message(encoded)
        envelope = json.loads(encoded)

        assert encoded.startswith(b"{")
        assert b"lingtu.transport.json.v1" in encoded
        assert envelope["schema_version"] == 1
        assert envelope["schema"] == "lingtu.geometry.twist.v1"
        assert envelope["type"] == "runtime.msgs.geometry.Twist"
        assert envelope["topic"] == "/nav/cmd_vel"
        assert envelope["frame_id"] == "body"
        assert "ts" in envelope
        assert isinstance(decoded, Twist)
        assert decoded.linear.x == 1.0
        assert decoded.linear.y == 2.0
        assert decoded.angular.z == 0.5

    def test_lcm_transport_adapter_uses_json_codec(self, monkeypatch):
        from runtime.msgs.geometry import Twist, Vector3
        from runtime.transport.abc import TopicConfig
        import runtime.transport.factory as factory_mod

        class BytesOnlyPublisher:
            def __init__(self):
                self.payloads = []

            def publish(self, msg):
                if not isinstance(msg, (bytes, bytearray)):
                    raise TypeError("bytes required")
                self.payloads.append(bytes(msg))

            def close(self):
                pass

        class CapturingBackend:
            name = "lcm"

            def __init__(self):
                self.publisher = BytesOnlyPublisher()
                self.callback = None

            def create_publisher(self, topic: TopicConfig):
                return self.publisher

            def create_subscriber(self, topic: TopicConfig, callback):
                self.callback = callback
                return type("Sub", (), {"start": lambda self: None, "close": lambda self: None})()

            def close(self):
                pass

        backend = CapturingBackend()
        monkeypatch.setattr(factory_mod, "create_transport", lambda strategy, ros_node=None: backend)

        transport = factory_mod.create_transport_adapter("lcm")
        received = []
        transport.subscribe("/nav/cmd_vel", received.append)
        transport.publish(
            "/nav/cmd_vel",
            Twist(linear=Vector3(0.2, 0.0, 0.0), angular=Vector3(0.0, 0.0, -0.1)),
        )

        payload = backend.publisher.payloads[-1]
        assert payload.startswith(b"{")
        assert b"runtime.msgs.geometry.Twist" in payload
        assert b'"/nav/cmd_vel"' in payload
        assert backend.callback is not None
        backend.callback(payload, time.time())
        assert isinstance(received[-1], Twist)
        assert received[-1].linear.x == 0.2
        assert received[-1].angular.z == -0.1
        before_invalid = len(received)
        backend.callback(b"not-json", time.time())
        assert len(received) == before_invalid

    @pytest.mark.parametrize("strategy", ["dds", "dual"])
    def test_dds_backed_transport_adapter_uses_raw_json_envelope(self, monkeypatch, strategy):
        import json
        from runtime.msgs.geometry import Twist, Vector3
        from runtime.transport.abc import TopicConfig
        from runtime.transport.dds import RawMessage
        import runtime.transport.factory as factory_mod

        class CapturingPublisher:
            def __init__(self):
                self.payloads = []

            def publish(self, msg):
                self.payloads.append(msg)

            def close(self):
                pass

        class CapturingBackend:
            name = "dds"

            def __init__(self):
                self.publisher = CapturingPublisher()
                self.publisher_topic = None
                self.subscriber_topic = None
                self.callback = None

            def create_publisher(self, topic: TopicConfig):
                self.publisher_topic = topic
                return self.publisher

            def create_subscriber(self, topic: TopicConfig, callback):
                self.subscriber_topic = topic
                self.callback = callback
                return type("Sub", (), {"start": lambda self: None, "close": lambda self: None})()

            def close(self):
                pass

        backend = CapturingBackend()
        monkeypatch.setattr(factory_mod, "create_transport", lambda strategy, ros_node=None: backend)

        transport = factory_mod.create_transport_adapter(strategy)
        received = []
        topic = "/test/cmd_vel"
        transport.subscribe(topic, received.append)
        transport.publish(
            topic,
            Twist(linear=Vector3(0.2, 0.0, 0.0), angular=Vector3(0.0, 0.0, -0.1)),
        )

        payload = backend.publisher.payloads[-1]
        envelope = json.loads(payload)
        assert backend.publisher_topic.msg_type is RawMessage
        assert backend.subscriber_topic.msg_type is RawMessage
        assert envelope["schema"] == "runtime.msgs.geometry.Twist"
        assert envelope["topic"] == topic
        assert envelope["frame_id"] is None
        assert backend.callback is not None
        backend.callback(payload, time.time())
        assert isinstance(received[-1], Twist)
        assert received[-1].linear.x == 0.2
        assert received[-1].angular.z == -0.1
        before_invalid = len(received)
        backend.callback(b"not-json", time.time())
        assert len(received) == before_invalid

    @pytest.mark.parametrize("strategy", ["dds", "dual"])
    def test_dds_backed_transport_adapter_rejects_product_topics(self, monkeypatch, strategy):
        import runtime.transport.factory as factory_mod

        class CapturingBackend:
            name = "dds"

            def create_publisher(self, topic):
                raise AssertionError("publisher should not be created")

            def create_subscriber(self, topic, callback):
                raise AssertionError("subscriber should not be created")

            def close(self):
                pass

        monkeypatch.setattr(factory_mod, "create_transport", lambda strategy, ros_node=None: CapturingBackend())

        transport = factory_mod.create_transport_adapter(strategy)
        with pytest.raises(ValueError, match="registered product topic /nav/cmd_vel"):
            transport.subscribe("/nav/cmd_vel", lambda _msg: None)
        with pytest.raises(ValueError, match="registered product topic /nav/cmd_vel"):
            transport.publish("/nav/cmd_vel", object())


def test_dds_transport_product_topic_uses_typed_contract(monkeypatch):
    import runtime.transport.dds as dds_mod

    class FakeOdom:
        pass

    class FakeWriter:
        instances = []

        def __init__(self, _participant, _topic):
            self.messages = []
            FakeWriter.instances.append(self)

        def write(self, msg):
            self.messages.append(msg)

        def close(self):
            pass

    monkeypatch.setattr(dds_mod, "_CYCLONE_AVAILABLE", True)
    monkeypatch.setattr(dds_mod, "DomainParticipant", lambda _domain_id: object())

    created = {}

    def fake_topic(_participant, name, msg_type):
        created["name"] = name
        created["msg_type"] = msg_type
        return name

    monkeypatch.setattr(
        dds_mod,
        "Topic",
        fake_topic,
        raising=False,
    )
    monkeypatch.setattr(dds_mod, "DataWriter", FakeWriter)
    monkeypatch.setattr(dds_mod, "dds_type_for_topic", lambda topic: FakeOdom)

    transport = dds_mod.DDSTransport(domain_id=7)
    pub = transport.create_publisher(TopicConfig(name="/slam/odometry"))
    msg = FakeOdom()

    pub.publish(msg)

    assert created == {"name": "rt/slam/odometry", "msg_type": FakeOdom}
    assert FakeWriter.instances[0].messages == [msg]
    with pytest.raises(TypeError, match="typed DDS"):
        pub.publish({"schema_version": 1, "pose": [1, 2, 3]})


class TestDDSTransport:
    """DDSTransport tests — skipped when cyclonedds is not installed."""

    @pytest.fixture(autouse=True)
    def require_cyclonedds(self):
        pytest.importorskip("cyclonedds", reason="cyclonedds not installed")

    def test_create_transport(self):
        from runtime.transport.dds import DDSTransport
        t = DDSTransport(domain_id=0)
        assert t.name == "dds"
        t.close()

    def test_pub_sub_roundtrip(self):
        from runtime.transport.abc import TopicConfig
        from runtime.transport.dds import DDSTransport

        transport = DDSTransport(domain_id=0)
        topic = TopicConfig(name="lingtu_test_roundtrip")

        received = []
        transport.create_subscriber(topic, received.append)
        pub = transport.create_publisher(topic)

        # Allow DDS discovery to complete before publishing
        time.sleep(0.03)

        pub.publish({"hello": "dds", "value": 42})

        # Wait for listener delivery
        deadline = time.time() + 2.0
        while not received and time.time() < deadline:
            time.sleep(0.012)

        transport.close()
        assert len(received) == 1
        assert received[0] == {"hello": "dds", "value": 42}

    def test_close_cleanup(self):
        from runtime.transport.abc import TopicConfig
        from runtime.transport.dds import DDSTransport

        transport = DDSTransport(domain_id=0)
        topic = TopicConfig(name="lingtu_test_cleanup")
        transport.create_publisher(topic)
        transport.create_subscriber(topic, lambda m: None)

        assert len(transport._publishers) == 1
        assert len(transport._subscribers) == 1

        transport.close()

        assert len(transport._publishers) == 0
        assert len(transport._subscribers) == 0

    def test_import_error_without_cyclonedds(self, monkeypatch):
        """DDSTransport raises ImportError with clear message when cyclonedds missing."""
        import runtime.transport.dds as dds_mod
        monkeypatch.setattr(dds_mod, "_CYCLONE_AVAILABLE", False)
        with pytest.raises(ImportError, match="cyclonedds-python is not installed"):
            dds_mod.DDSTransport()


class TestTransportFactory:
    """Transport factory tests."""

    def test_create_shm(self):
        from runtime.transport.factory import create_transport
        from runtime.transport.shm import SHMTransport
        t = create_transport(TransportStrategy.SHM)
        assert isinstance(t, SHMTransport)
        t.close()

    def test_create_auto_returns_shm(self):
        from runtime.transport.factory import create_transport
        from runtime.transport.shm import SHMTransport
        # AUTO prefers SHM on same machine
        t = create_transport(TransportStrategy.AUTO)
        assert isinstance(t, SHMTransport)
        t.close()

    def test_create_dds(self):
        pytest.importorskip("cyclonedds", reason="cyclonedds not installed")
        from runtime.transport.dds import DDSTransport
        from runtime.transport.factory import create_transport
        t = create_transport(TransportStrategy.DDS)
        assert isinstance(t, DDSTransport)
        t.close()

    def test_create_dual(self):
        pytest.importorskip("cyclonedds", reason="cyclonedds not installed")
        from runtime.transport.dual import DualTransport
        from runtime.transport.factory import create_transport
        t = create_transport(TransportStrategy.DUAL)
        assert isinstance(t, DualTransport)
        t.close()
